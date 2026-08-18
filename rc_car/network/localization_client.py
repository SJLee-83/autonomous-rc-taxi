"""LocalizationClient — 위치 서버 WebSocket 클라이언트 (protocol_2 §3.1·§3.2).

- ControlClient와 같은 구조다: 전용 스레드의 asyncio 루프 (§9.3). 제어 연산이 이벤트
  루프를 붙잡아 수신이 밀리는 일을 막는다.
- 위치 서버는 자체 서명 인증서를 쓰므로 wss에서 검증을 끈다 (§3.1 — `tls_verify: false`).
- **재연결할 때마다 `subscribe`를 다시 보낸다** (§3.1). 접속과 구독은 한 묶음이다.
- ping/pong을 쓰지 않는다. 위치 서버의 pong 지원 여부에 기대는 대신 **무수신
  자체를 끊김으로 본다** — 규약의 미수신 판정과 같은 값(`pose_timeout_sec`)이라
  절반 열린 연결에 매달려 정지한 채 남는 경우가 없다.
- 서버가 규약 위반으로 끊는 close 코드(4400/4404/4408)는 **재접속으로 절대 낫지 않는
  설정 오류**다. 조용히 무한 재시도하지 않고 사유를 명시해 로그에 남긴다.
- 콜백은 네트워크 스레드에서 호출된다. 수신자는 스레드 안전 객체만 써야 한다.
"""
import asyncio
import json
import logging
import ssl
import threading

import websockets

log = logging.getLogger("network.localization")

_OPEN_TIMEOUT_SEC = 2.0   # 접속 시도 1회 상한 — 종료 시퀀스를 오래 붙잡지 않게 한다

#: 위치 서버가 구독을 거부하며 쓰는 close 코드. 전부 우리 쪽 설정 문제라 재접속으로
#: 낫지 않는다 — 사람이 config를 고쳐야 한다는 사실이 로그에서 드러나야 한다.
_FATAL_CLOSE_CODES = {
    4400: "subscribe 형식 오류 — marker_id/interval_ms 값 확인",
    4404: "설정에 등록되지 않은 마커 ID — network.yaml의 localization.marker_id 확인",
    4408: "subscribe 10초 내 미송신 — 접속 직후 구독이 나가지 않았다",
}


def _build_ssl(url: str, verify: bool):
    """wss가 아니면 None. 자체 서명 인증서라 기본값은 검증 비활성화다 (§3.1)."""
    if not url.startswith("wss://"):
        return None
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    if not verify:
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
    else:
        ctx.load_default_certs()
    return ctx


class LocalizationClient:
    def __init__(self, cfg: dict, on_message, on_disconnected, on_connected=None):
        self._url = cfg["websocket_url"]
        self._reconnect_sec = cfg["reconnect_interval_sec"]
        self._recv_timeout = cfg["pose_timeout_sec"]
        self._ssl = _build_ssl(self._url, cfg.get("tls_verify", False))
        # 큐브 모드(§0-31)면 4면 ID 전부 구독, 아니면 단일 marker_id (기존 동작)
        cube = cfg.get("cube") or {}
        if cube.get("enabled"):
            ids = sorted(int(k) for k in cube["face_yaw_offset_deg"])
        else:
            ids = [cfg["marker_id"]]
        self._subscribes = [json.dumps({
            "type": "subscribe",
            "marker_id": i,
            "interval_ms": cfg["interval_ms"],
        }) for i in ids]
        self._on_message = on_message
        self._on_disconnected = on_disconnected
        self._on_connected = on_connected

        self._loop: "asyncio.AbstractEventLoop | None" = None
        self._ws = None
        self._stop_async: "asyncio.Event | None" = None
        self._stopping = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._thread_main,
                                        name="LocalizationClient", daemon=True)

    # ---------- 생명주기 ----------

    def start(self) -> None:
        self._thread.start()
        self._ready.wait(timeout=3.0)

    def stop(self) -> None:
        self._stopping.set()
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(self._request_shutdown)
        self._thread.join(timeout=3.0)
        if self._thread.is_alive():
            log.warning("LocalizationClient join 시간 초과")

    @property
    def connected(self) -> bool:
        return self._ws is not None

    # ---------- 수신 루프 ----------

    def _thread_main(self) -> None:
        asyncio.run(self._main())

    async def _main(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._stop_async = asyncio.Event()
        self._ready.set()
        while not self._stopping.is_set():
            was_connected = False
            try:
                async with websockets.connect(
                        self._url,
                        ssl=self._ssl,
                        ping_interval=None,          # 무수신 감지로 대체 (모듈 docstring)
                        open_timeout=_OPEN_TIMEOUT_SEC) as ws:
                    self._ws = ws
                    was_connected = True
                    for sub in self._subscribes:     # 재연결마다 재구독 (§3.1) — 큐브면 4면 전부
                        await ws.send(sub)
                    log.info("위치 서버 접속·구독 %s", self._url)
                    self._safe_callback(self._on_connected)
                    while not self._stopping.is_set():
                        raw = await asyncio.wait_for(ws.recv(), timeout=self._recv_timeout)
                        self._safe_callback(self._on_message, raw)
            except asyncio.TimeoutError:
                if was_connected:
                    log.warning("위치 %.1f초 무수신 — 연결 끊김으로 보고 재접속",
                                self._recv_timeout)
            except websockets.ConnectionClosed as e:
                self._log_close(e)
            except (ConnectionError, OSError, ssl.SSLError):
                pass
            except Exception:  # noqa: BLE001 — 루프는 죽지 않는다. 재접속으로 이어간다
                log.exception("LocalizationClient 예기치 못한 예외 — 재접속 계속")
            finally:
                self._ws = None
            if self._stopping.is_set():
                break
            if was_connected:
                # 유지되던 연결이 끊긴 경우만 알린다. 기동 직후 접속 실패는 아직
                # 운행 전이므로 격상하지 않는다 (첫 pose 게이트는 서비스가 판정)
                self._safe_callback(self._on_disconnected)
            try:
                await asyncio.wait_for(self._stop_async.wait(), timeout=self._reconnect_sec)
            except asyncio.TimeoutError:
                pass
        log.info("LocalizationClient 종료")

    def _log_close(self, exc: "websockets.ConnectionClosed") -> None:
        """구독 거부(4400/4404/4408)는 재접속으로 낫지 않는다 — 사유를 드러낸다."""
        code = getattr(exc.rcvd, "code", None) if exc.rcvd is not None else None
        hint = _FATAL_CLOSE_CODES.get(code)
        if hint:
            reason = getattr(exc.rcvd, "reason", "") or ""
            log.error("위치 서버가 구독 거부 (close %d): %s%s — 재접속해도 해소되지 않는다",
                      code, hint, f" / 서버 사유: {reason}" if reason else "")

    def _request_shutdown(self) -> None:
        if self._stop_async is not None:
            self._stop_async.set()
        if self._ws is not None:
            asyncio.ensure_future(self._ws.close())

    def _safe_callback(self, cb, *args) -> None:
        if cb is None:
            return
        try:
            cb(*args)
        except Exception:  # noqa: BLE001 — 콜백 예외가 수신 루프를 죽이면 안 된다
            log.exception("콜백 예외 — 수신 루프 유지")
