"""단계 1 임시 worker 2종.

- IdleControlWorker: 50Hz로 정지 명령을 SafetySupervisor에 계속 밀어 넣는다.
  단계 5에서 실제 ControlWorker(Pure Pursuit)로 교체된다 — 교체 지점을 여기 한 곳으로 고정.
- HeartbeatWorker: 1초마다 상태 요약 로그. 살아 있음을 눈으로 확인하는 용도.
"""
import logging
import threading

from core.enums import to_service_state
from core.models import STOP_COMMAND
from core.state_store import StateStore
from safety.safety_supervisor import SafetySupervisor
from workers.periodic_worker import PeriodicWorker

log = logging.getLogger("app")


class IdleControlWorker(PeriodicWorker):
    """단계 1: 항상 정지 출력. 유효 pose가 없으므로 SafetySupervisor도 어차피 차단한다(이중 안전)."""

    def __init__(self, stop_event: threading.Event, supervisor: SafetySupervisor,
                 loop_hz: float, on_error=None):
        super().__init__("ControlWorker", 1.0 / loop_hz, stop_event, on_error)
        self._supervisor = supervisor

    def tick(self) -> None:
        self._supervisor.apply(STOP_COMMAND)


class HeartbeatWorker(PeriodicWorker):
    def __init__(self, stop_event: threading.Event, store: StateStore,
                 interval_sec: float, on_error=None):
        super().__init__("Heartbeat", interval_sec, stop_event, on_error)
        self._store = store

    def tick(self) -> None:
        s = self._store.snapshot()
        log.info("state=%s(%s) pose=%s stale=%s throttle=%.2f",
                 s.driving_state.value,
                 to_service_state(s.driving_state, s.service_kind),
                 f"({s.pose.x:.2f},{s.pose.y:.2f})" if s.pose else "없음",
                 s.pose_stale, s.control_command.throttle)
