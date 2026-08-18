"""C5 시나리오 테스트 — 실제 WebSocket 서버를 상대로 전체 스택 검증 (protocol_2 §8 · §7).

시나리오: 정상 사이클 + 오류 5종
  1. GPS 유실 → 자체 복귀 (resume 불필요)
  2. 관제 끊김 → 재연결 → error 반복 → resume 복귀
  3. 동시 유실 → 두 조건 모두 충족 후에만 복귀 (§7.5)
  4. 대기 중 오류 → 복귀만, 주행 없음
  5. complete 재전송 중 오류 → 복귀 후 같은 timestamp로 재전송 재개

테스트 가속을 위해 재연결·재전송·ping 주기를 줄여서 돌린다 (규약의 값 자체가 아니라
"응답이 올 때까지 반복한다"는 동작을 검증하는 것이므로 주기 단축은 무방).
"""
import asyncio
import json
import threading
import time

import pytest
import websockets

from core.enums import DrivingState, ServiceKind, to_service_state
from core.models import LocalizationPose
from core.state_store import StateStore
from network.command_policy import CommandPolicy
from network.control_client import ControlClient
from network.report_manager import ReportManager
from network.telemetry import TelemetryWorker

MAP_X, MAP_Y = 5.0, 3.0


def wait_until(pred, timeout=5.0, msg="조건"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return
        time.sleep(0.02)
    pytest.fail(f"시간 초과 ({timeout}s): {msg}")


class ScenarioServer:
    """테스트 전용 관제 서버 — 수신 기록 + 명령 송신 + 강제 절단."""

    def __init__(self):
        self.port: int | None = None
        self._received: list[dict] = []
        self._lock = threading.Lock()
        self._conn = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop: asyncio.Event | None = None
        self._ready = threading.Event()
        self._thread = threading.Thread(target=lambda: asyncio.run(self._main()),
                                        name="ScenarioServer", daemon=True)

    async def _main(self):
        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        async with websockets.serve(self._handler, "127.0.0.1", 0) as server:
            self.port = server.sockets[0].getsockname()[1]
            self._ready.set()
            await self._stop.wait()

    async def _handler(self, ws):
        old, self._conn = self._conn, ws   # 새 연결 우선 — 옛 연결 즉시 폐기 (§2.9)
        if old is not None:
            await old.close()
        try:
            async for raw in ws:
                with self._lock:
                    self._received.append(json.loads(raw))
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

    def send_command(self, command: str, loc: dict | None = None):
        msg = {"protocol_version": "2.3", "header": "command",
               "timestamp": time.time(), "command": command}
        if loc is not None:
            msg["loc"] = loc
        conn = self._conn
        assert conn is not None, "연결 없음"
        asyncio.run_coroutine_threadsafe(
            conn.send(json.dumps(msg, ensure_ascii=False)), self._loop).result(2.0)

    def kill_connection(self):
        """관제 서버 사망·네트워크 절단 재현."""
        conn = self._conn
        if conn is not None:
            try:
                asyncio.run_coroutine_threadsafe(conn.close(), self._loop).result(2.0)
            except Exception:  # noqa: BLE001 — 이미 끊겼으면 무방
                pass

    def reports(self, kind: str) -> list[dict]:
        with self._lock:
            return [m for m in self._received
                    if m.get("header") == "report" and m.get("report") == kind]

    def infos(self) -> list[dict]:
        with self._lock:
            return [m for m in self._received if m.get("header") == "info"]

    def connected(self) -> bool:
        conn = self._conn
        return conn is not None and conn.state is websockets.protocol.State.OPEN


class FakeSupervisor:
    def __init__(self):
        self.force_stop_count = 0

    def force_stop(self):
        self.force_stop_count += 1


class Stack:
    """차량 측 전체 스택 (드라이버 제외) — ControlClient·정책·보고·텔레메트리."""

    def __init__(self, server: ScenarioServer):
        self.stop_event = threading.Event()
        self.store = StateStore()
        self.store.set_driving_state(DrivingState.WAITING)
        self.supervisor = FakeSupervisor()
        cfg = {
            "websocket_url": f"ws://127.0.0.1:{server.port}/ws/vehicle",
            "reconnect_interval_sec": 0.15,
            "ping_interval_sec": 0.5,
            "ping_timeout_sec": 1.0,
            "complete_resend_interval_sec": 0.25,
            "error_resend_interval_sec": 0.25,
        }
        self.client = ControlClient(
            cfg,
            on_message=lambda raw: self.policy.on_raw_message(raw),
            on_connected=lambda: self.policy.on_control_connected(),
            on_disconnected=lambda: self.policy.on_control_disconnected(),
        )
        self.reports = ReportManager(self.stop_event, self.client, self.store, cfg)
        self.policy = CommandPolicy(self.store, self.supervisor, self.reports, MAP_X, MAP_Y)
        self.telemetry = TelemetryWorker(
            self.stop_event, self.client, self.store,
            {"center_deg": 108, "wheel_angle_ratio": 0.526}, rate_hz=5)

    def start(self):
        self.client.start()
        self.reports.start()
        self.telemetry.start()

    def shutdown(self):
        self.stop_event.set()
        self.reports.join()
        self.telemetry.join()
        self.client.stop()

    def give_pose(self, x=1.0, y=1.0, heading=0.0):
        self.store.update_pose(LocalizationPose(x, y, heading, time.time(), time.monotonic()))

    def service_state(self) -> str:
        snap = self.store.snapshot()
        return to_service_state(snap.driving_state, snap.service_kind)


@pytest.fixture()
def env():
    server = ScenarioServer()
    server.start()
    stack = Stack(server)
    stack.start()
    wait_until(server.connected, msg="차량 최초 접속")
    yield server, stack
    stack.shutdown()
    server.stop()


# ---------- 정상 사이클 (§8) ----------

def test_normal_full_cycle(env):
    server, stack = env

    # 첫 유효 pose 전 info 미송신 (§4.0)
    time.sleep(0.5)
    assert server.infos() == []

    stack.give_pose(x=0.5, y=0.3)
    wait_until(lambda: len(server.infos()) >= 3, msg="info 5Hz 송신")
    assert server.infos()[0]["state"] == "대기 중"

    # 호출 배차 → accept → 픽업 이동
    server.send_command("move", {"x": 3.4, "y": 2.2})
    wait_until(lambda: len(server.reports("accept")) == 1, msg="accept")
    assert stack.service_state() == "호출 응답 이동 중"

    # 도착 → complete 재전송 (timestamp 고정) → stop → 탑승 대기 중
    stack.policy.notify_arrived()
    wait_until(lambda: len(server.reports("complete")) >= 3, msg="complete 재전송")
    assert len({m["timestamp"] for m in server.reports("complete")}) == 1
    server.send_command("stop")
    wait_until(lambda: stack.service_state() == "탑승 대기 중", msg="stop 전이")

    # 목적지 이동 → 도착 → stop → 대기 중
    server.send_command("move", {"x": 0.8, "y": 0.6})
    wait_until(lambda: len(server.reports("accept")) == 2, msg="2차 accept")
    assert stack.service_state() == "고객 탑승 이동 중"
    assert stack.store.snapshot().mission.kind == ServiceKind.CARRY
    stack.policy.notify_arrived()
    wait_until(lambda: len(server.reports("complete")) >= 4, msg="2차 complete")
    server.send_command("stop")
    wait_until(lambda: stack.service_state() == "대기 중", msg="사이클 완료")


# ---------- 오류 1: GPS 유실 자체 복귀 (§7.2) ----------

def test_gps_loss_self_recovery(env):
    server, stack = env
    stack.give_pose()
    server.send_command("move", {"x": 3.0, "y": 2.0})
    wait_until(lambda: len(server.reports("accept")) == 1, msg="accept")

    stack.policy.on_gps_lost()
    stack.store.mark_pose_stale()
    wait_until(lambda: len(server.reports("error")) == 1, msg="error 1회")
    wait_until(lambda: any(i["state"] == "오류 정지" and i["loc"]["stale"]
                           for i in server.infos()), msg="info에 오류 정지+stale")
    time.sleep(0.6)
    assert len(server.reports("error")) == 1      # GPS 유실 error는 재전송하지 않는다 (§5.2)

    stack.give_pose()                              # 위치 정상화
    stack.policy.on_gps_recovered()
    wait_until(lambda: len(server.reports("recovered")) == 1, msg="recovered")
    assert stack.service_state() == "호출 응답 이동 중"   # resume 없이 자체 복귀
    assert stack.store.snapshot().mission is not None


# ---------- 오류 2: 관제 끊김 → 재연결 → resume (§7.3) ----------

def test_control_loss_resume_recovery(env):
    server, stack = env
    stack.give_pose()
    server.send_command("move", {"x": 3.0, "y": 2.0})
    wait_until(lambda: len(server.reports("accept")) == 1, msg="accept")

    server.kill_connection()
    wait_until(lambda: stack.store.snapshot().driving_state == DrivingState.ERROR,
               msg="관제 끊김 → 오류 정지")
    assert stack.supervisor.force_stop_count >= 1

    # 재연결 후 resume까지 error 반복 (timestamp 고정)
    wait_until(lambda: len(server.reports("error")) >= 3, msg="error 반복")
    assert len({m["timestamp"] for m in server.reports("error")}) == 1

    server.send_command("resume")
    wait_until(lambda: stack.service_state() == "호출 응답 이동 중", msg="직전 state 복귀")
    assert stack.store.snapshot().mission is not None   # 보관된 목적지로 재개 가능
    time.sleep(0.6)
    n = len(server.reports("error"))
    time.sleep(0.6)
    assert len(server.reports("error")) == n      # resume 후 error 재전송 중단


# ---------- 오류 3: 동시 유실 (§7.5) ----------

def test_simultaneous_loss(env):
    server, stack = env
    stack.give_pose()
    server.send_command("move", {"x": 3.0, "y": 2.0})
    wait_until(lambda: len(server.reports("accept")) == 1, msg="accept")

    server.kill_connection()
    stack.policy.on_gps_lost()
    stack.store.mark_pose_stale()
    wait_until(lambda: stack.store.snapshot().driving_state == DrivingState.ERROR,
               msg="오류 정지")

    wait_until(lambda: len(server.reports("error")) >= 2, msg="재연결 후 error 반복")
    server.send_command("resume")                  # 조건 ①만 충족
    time.sleep(0.5)
    assert stack.store.snapshot().driving_state == DrivingState.ERROR   # 복귀 보류

    stack.give_pose()
    stack.policy.on_gps_recovered()                # 조건 ② 충족 → 복귀
    wait_until(lambda: len(server.reports("recovered")) == 1, msg="recovered")
    wait_until(lambda: stack.service_state() == "호출 응답 이동 중", msg="복귀")


# ---------- 오류 4: 대기 중 오류 (§7.2 — 복귀만, 주행 없음) ----------

def test_error_while_waiting(env):
    server, stack = env
    stack.give_pose()
    server.kill_connection()
    wait_until(lambda: stack.store.snapshot().driving_state == DrivingState.ERROR,
               msg="대기 중 오류 정지")

    wait_until(lambda: len(server.reports("error")) >= 1, msg="error")
    server.send_command("resume")
    wait_until(lambda: stack.service_state() == "대기 중", msg="대기 복귀")
    assert stack.store.snapshot().mission is None  # 재개할 목적지가 없다


# ---------- 오류 5: complete 재전송 중 오류 (2차 검증 I 시나리오) ----------

def test_error_during_complete_resend(env):
    server, stack = env
    stack.give_pose()
    server.send_command("move", {"x": 3.0, "y": 2.0})
    wait_until(lambda: len(server.reports("accept")) == 1, msg="accept")
    stack.policy.notify_arrived()
    wait_until(lambda: len(server.reports("complete")) >= 2, msg="complete 재전송")
    first_ts = server.reports("complete")[0]["timestamp"]

    server.kill_connection()
    wait_until(lambda: stack.store.snapshot().driving_state == DrivingState.ERROR,
               msg="complete 재전송 중 오류 정지")
    wait_until(lambda: len(server.reports("error")) >= 2, msg="error 반복")
    n_complete = len(server.reports("complete"))
    time.sleep(0.6)
    assert len(server.reports("complete")) == n_complete   # 오류 중 complete 정지 (§7.2)

    server.send_command("resume")
    wait_until(lambda: stack.store.snapshot().driving_state == DrivingState.ARRIVED,
               msg="ARRIVED 복귀")
    wait_until(lambda: len(server.reports("complete")) > n_complete,
               msg="complete 재전송 재개")
    assert server.reports("complete")[-1]["timestamp"] == first_ts   # 사건 식별자 유지

    server.send_command("stop")
    wait_until(lambda: stack.service_state() == "탑승 대기 중", msg="정상 마무리")
