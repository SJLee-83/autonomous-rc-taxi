"""단계 2 A1 — 실제 WebSocket 위치 서버를 상대로 접속·구독·재구독·유실 검증 (protocol_2 §3).

시나리오
  1. 접속 → subscribe 1회 (marker_id·interval_ms)
  2. 10Hz push → StateStore pose 갱신
  3. 강제 절단 → 재접속 → **재구독**
  4. 무수신 → 끊김으로 판정하고 재접속 (절반 열린 연결 방어)
  5. found=false 지속 → 유실 → 정상 재개 → 복귀

재연결·timeout 주기는 줄여서 돌린다 — 검증 대상은 주기값이 아니라 동작이다.
"""
import asyncio
import json
import ssl
import threading
import time

import pytest
import websockets

from core.state_store import StateStore
from localization.localization_service import LocalizationService
from localization.pose_validator import PoseValidator
from network.localization_client import LocalizationClient, _build_ssl

MAP_X, MAP_Y = 5.0, 3.0
MARKER_ID = 4


def wait_until(pred, timeout=5.0, msg="조건"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return
        time.sleep(0.02)
    pytest.fail(f"시간 초과 ({timeout}s): {msg}")


class GpsServer:
    """테스트 전용 위치 서버 — subscribe 기록 + 임의 push + 강제 절단."""

    def __init__(self, reject_code: int | None = None):
        self.port: int | None = None
        self.reject_code = reject_code     # 설정하면 subscribe를 이 코드로 거부한다
        self.subscribes: list[dict] = []
        self._lock = threading.Lock()
        self._conn = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop: asyncio.Event | None = None
        self._ready = threading.Event()
        self._thread = threading.Thread(target=lambda: asyncio.run(self._main()),
                                        name="GpsServer", daemon=True)

    async def _main(self):
        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        async with websockets.serve(self._handler, "127.0.0.1", 0) as server:
            self.port = server.sockets[0].getsockname()[1]
            self._ready.set()
            await self._stop.wait()

    async def _handler(self, ws):
        self._conn = ws
        try:
            async for raw in ws:
                with self._lock:
                    self.subscribes.append(json.loads(raw))
                if self.reject_code:
                    await ws.close(code=self.reject_code,
                                   reason="설정에 등록되지 않은 마커 ID입니다.")
                    return
        except websockets.ConnectionClosed:
            pass

    # ---------- 테스트 스레드 API ----------

    def start(self):
        self._thread.start()
        assert self._ready.wait(3.0), "서버 기동 실패"

    def stop(self):
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._stop.set)
        self._thread.join(3.0)

    def push(self, x=1.0, y=1.0, heading=90.0, ts=None, found=True, marker_id=MARKER_ID):
        m = {"marker_id": marker_id, "found": found,
             "timestamp": time.time() if ts is None else ts}
        if found:
            m["position"] = {"x": x, "y": y, "z": 0.12}
            m["heading"] = heading
        conn = self._conn
        assert conn is not None, "연결 없음"
        asyncio.run_coroutine_threadsafe(conn.send(json.dumps(m)), self._loop).result(2.0)

    def kill_connection(self):
        conn = self._conn
        if conn is not None:
            try:
                asyncio.run_coroutine_threadsafe(conn.close(), self._loop).result(2.0)
            except Exception:  # noqa: BLE001 — 이미 끊겼으면 무방
                pass

    def subscribe_count(self) -> int:
        with self._lock:
            return len([m for m in self.subscribes if m.get("type") == "subscribe"])

    def connected(self) -> bool:
        conn = self._conn
        return conn is not None and conn.state is websockets.protocol.State.OPEN


class Stack:
    """차량 측 위치 처리 스택 — client → service → StateStore / 훅."""

    def __init__(self, server: GpsServer):
        self.stop_event = threading.Event()
        self.store = StateStore()
        self.lost = 0
        self.recovered = 0
        cfg = {
            "websocket_url": f"ws://127.0.0.1:{server.port}/ws/v1/localization",
            "tls_verify": False,
            "marker_id": MARKER_ID,
            "interval_ms": 100,
            "marker_yaw_offset_deg": 0,
            "reconnect_interval_sec": 0.15,
            "pose_timeout_sec": 0.4,
            "lost_hold_sec": 0.4,
            "max_jump_m": 0.08,
            "max_heading_jump_deg": 15.0,
        }
        self.service = LocalizationService(
            self.stop_event, self.store, PoseValidator(cfg, MAP_X, MAP_Y), cfg,
            on_lost=self._on_lost, on_recovered=self._on_recovered)
        self.client = LocalizationClient(
            cfg,
            on_message=self.service.on_raw,
            on_disconnected=self.service.on_disconnected)

    def _on_lost(self):
        self.lost += 1

    def _on_recovered(self):
        self.recovered += 1

    def start(self):
        self.client.start()
        self.service.start()

    def shutdown(self):
        self.stop_event.set()
        self.service.join()
        self.client.stop()

    def pose(self):
        return self.store.snapshot().pose

    def stale(self) -> bool:
        return self.store.snapshot().pose_stale


@pytest.fixture()
def env():
    server = GpsServer()
    server.start()
    stack = Stack(server)
    stack.start()
    wait_until(lambda: server.subscribe_count() >= 1, msg="최초 접속·구독")
    yield server, stack
    stack.shutdown()
    server.stop()


# ---------- 1·2. 접속·구독·수신 ----------

def test_접속하면_subscribe를_1회_보낸다(env):
    server, _ = env
    sub = server.subscribes[0]
    assert sub == {"type": "subscribe", "marker_id": MARKER_ID, "interval_ms": 100}


def test_push한_pose가_state_store에_반영된다(env):
    server, stack = env
    base = time.time()
    for i in range(5):
        server.push(x=1.0 + i * 0.02, y=2.0, heading=90.0, ts=base + i * 0.1)
        time.sleep(0.02)
    wait_until(lambda: stack.pose() is not None and stack.pose().x == pytest.approx(1.08),
               msg="pose 5건 반영")
    pose = stack.pose()
    assert (pose.y, pose.heading_deg) == (2.0, 90.0)
    assert stack.stale() is False
    assert stack.lost == 0


# ---------- 3. 재연결 시 재구독 ----------

def test_강제_절단_후_재접속하면_다시_구독한다(env):
    server, stack = env
    server.push()
    wait_until(lambda: stack.pose() is not None, msg="첫 pose")

    server.kill_connection()
    wait_until(lambda: server.subscribe_count() >= 2, msg="재접속·재구독")
    assert stack.lost == 1                        # 연결 끊김은 즉시 유실 (§7.4)
    assert stack.stale() is True

    server.push(x=1.2, y=1.0)
    wait_until(lambda: stack.recovered == 1, msg="재수신 후 복귀")
    assert stack.stale() is False


# ---------- 4. 무수신 ----------

def test_무수신이_지속되면_끊김으로_보고_재접속한다(env):
    server, stack = env
    server.push()
    wait_until(lambda: stack.pose() is not None, msg="첫 pose")

    # 서버가 살아 있는데도 아무것도 보내지 않는 상태 — 절반 열린 연결과 구분되지 않는다
    wait_until(lambda: stack.lost == 1, timeout=2.0, msg="무수신 유실 판정")
    wait_until(lambda: server.subscribe_count() >= 2, timeout=2.0, msg="무수신 후 재구독")


# ---------- 5. 미인식 지속 ----------

def test_found_false_지속은_유실_정상화되면_복귀(env):
    server, stack = env
    base = time.time()
    server.push(ts=base)
    wait_until(lambda: stack.pose() is not None, msg="첫 pose")

    # found=false를 계속 보낸다 — 수신은 되므로 연결은 살아 있고, 미인식만 지속된다
    def keep_pushing_not_found():
        for i in range(12):
            server.push(found=False, ts=base + 0.1 * (i + 1))
            time.sleep(0.05)

    t = threading.Thread(target=keep_pushing_not_found, daemon=True)
    t.start()
    wait_until(lambda: stack.stale(), timeout=1.0, msg="즉시 stale (§3.4)")
    wait_until(lambda: stack.lost == 1, timeout=2.0, msg="지속 후 유실 판정")
    t.join(3.0)
    assert server.connected(), "미인식은 연결 문제가 아니다 — 연결은 유지되어야 한다"
    assert server.subscribe_count() == 1, "재접속이 일어나면 안 된다"

    server.push(x=1.5, y=1.5, ts=time.time())
    wait_until(lambda: stack.recovered == 1, msg="마커 재인식 후 복귀")
    assert stack.stale() is False


# ---------- 구독 거부 (close 4404) ----------

def test_미등록_마커는_close_4404를_사유와_함께_남긴다(caplog):
    """재접속으로 낫지 않는 설정 오류다 — 조용히 무한 재시도하면 원인을 못 찾는다."""
    server = GpsServer(reject_code=4404)
    server.start()
    stack = Stack(server)
    with caplog.at_level("ERROR", logger="network.localization"):
        stack.start()
        wait_until(lambda: any("close 4404" in r.getMessage() for r in caplog.records),
                   msg="4404 오류 로그")
        # 거부당해도 재접속 자체는 계속한다 — 서버가 설정을 고쳐 살아날 수도 있다
        wait_until(lambda: server.subscribe_count() >= 2, msg="거부 후 재접속")
    stack.shutdown()
    server.stop()


# ---------- TLS 설정 (실서버는 wss + 자체 서명) ----------

def test_wss는_자체_서명_인증서를_검증하지_않는다():
    ctx = _build_ssl("wss://10.0.0.1:8000/ws/v1/localization", verify=False)
    assert ctx.verify_mode is ssl.CERT_NONE and ctx.check_hostname is False
    assert _build_ssl("ws://127.0.0.1:8000/ws", verify=False) is None
