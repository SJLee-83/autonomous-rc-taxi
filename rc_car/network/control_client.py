"""ControlClient — 관제 서버 WebSocket 클라이언트 (protocol_2 §2.8·§2.9·§9.3).

- 전용 스레드의 asyncio 루프에서 동작한다. 제어 연산과 스레드를 분리해
  이벤트 루프 블로킹으로 인한 ping_timeout 오검출을 막는다 (§9.3).
- ping_interval / ping_timeout 으로 관제 끊김을 최대 3초 내 감지한다 (§2.8).
- 끊기면 1초 간격으로 재접속을 반복한다. 재접속도 차량이 건다 (§2.1).
- 콜백(on_message / on_connected / on_disconnected)은 네트워크 스레드에서 호출된다.
  수신자는 StateStore 등 스레드 안전 객체만 사용해야 하며 오래 블로킹하면 안 된다.
"""
import asyncio
import logging
import threading

import websockets

log = logging.getLogger("network.control")

_OPEN_TIMEOUT_SEC = 2.0   # 접속 시도 1회 상한 — 종료 시퀀스를 오래 붙잡지 않게 한다


class ControlClient:
    def __init__(self, cfg: dict, on_message, on_connected, on_disconnected):
        self._url = cfg["websocket_url"]
        self._reconnect_sec = cfg["reconnect_interval_sec"]
        self._ping_interval = cfg["ping_interval_sec"]
        self._ping_timeout = cfg["ping_timeout_sec"]
        self._on_message = on_message
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected

        self._loop: asyncio.AbstractEventLoop | None = None
        self._ws = None
        self._stop_async: asyncio.Event | None = None
        self._stopping = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._thread_main,
                                        name="ControlClient", daemon=True)

    # ---------- 생명주기 ----------

    def start(self) -> None:
        self._thread.start()
        self._ready.wait(timeout=3.0)

    def stop(self) -> None:
        """종료 시퀀스 5단계 — WebSocket 종료 (명세서 §25.3)."""
        self._stopping.set()
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(self._request_shutdown)
        self._thread.join(timeout=3.0)
        if self._thread.is_alive():
            log.warning("ControlClient join 시간 초과")

    @property
    def connected(self) -> bool:
        return self._ws is not None

    # ---------- 송신 (임의 스레드에서 호출 가능) ----------

    def send_json(self, obj: dict) -> bool:
        """비블로킹 송신 예약. 반환값은 호출 시점의 연결 여부 —
        미연결이면 버린다 (재전송 규약 §5.2가 유실을 복구한다)."""
        loop, ws = self._loop, self._ws
        if loop is None or ws is None or not loop.is_running():
            return False
        if threading.get_ident() == self._thread.ident:
            # 네트워크 스레드(콜백 안)에서의 송신 — 결과 대기 시 데드락이라 예약만 한다
            loop.call_soon(lambda: asyncio.ensure_future(self._send(ws, obj)))
        else:
            asyncio.run_coroutine_threadsafe(self._send(ws, obj), loop)
        return True

    async def _send(self, ws, obj: dict) -> None:
        import json
        try:
            await ws.send(json.dumps(obj, ensure_ascii=False))
        except Exception as e:  # noqa: BLE001 — 송신 실패는 연결 종료로 이어져 §7 경로가 처리
            log.debug("송신 실패 (연결 종료 중): %r", e)

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
                        ping_interval=self._ping_interval,
                        ping_timeout=self._ping_timeout,
                        open_timeout=_OPEN_TIMEOUT_SEC) as ws:
                    self._ws = ws
                    was_connected = True
                    log.info("관제 서버 접속 %s", self._url)
                    self._safe_callback(self._on_connected)
                    async for raw in ws:
                        self._safe_callback(self._on_message, raw)
            except (websockets.ConnectionClosed, ConnectionError, OSError, asyncio.TimeoutError):
                pass
            except Exception:  # noqa: BLE001 — 루프는 죽지 않는다. 재접속으로 이어간다
                log.exception("ControlClient 예기치 못한 예외 — 재접속 계속")
            finally:
                self._ws = None
            if self._stopping.is_set():
                break
            if was_connected:
                # 유지되던 연결이 끊긴 경우만 오류로 본다 — 기동 직후 접속 실패는
                # 아직 운행 전이므로 §7.1 조건 3(관제 끊김)으로 격상하지 않는다
                log.warning("관제 연결 끊김 — §7.3 절차 진입, %.1f초 간격 재접속",
                            self._reconnect_sec)
                self._safe_callback(self._on_disconnected)
            try:
                await asyncio.wait_for(self._stop_async.wait(), timeout=self._reconnect_sec)
            except asyncio.TimeoutError:
                pass
        log.info("ControlClient 종료")

    def _request_shutdown(self) -> None:
        if self._stop_async is not None:
            self._stop_async.set()
        if self._ws is not None:
            asyncio.ensure_future(self._ws.close())

    def _safe_callback(self, cb, *args) -> None:
        try:
            cb(*args)
        except Exception:  # noqa: BLE001 — 콜백 예외가 수신 루프를 죽이면 안 된다
            log.exception("콜백 예외 — 수신 루프 유지 (§2.10 정신)")
