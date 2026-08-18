"""SafetySupervisor — 모든 모터 출력의 단일 통과점 (명세서 §33-6).

어떤 모듈도 드라이버를 직접 호출하지 않는다. 반드시 apply()를 거친다.

차단 조건 (하나라도 걸리면 무조건 정지 출력):
  1. 비상 래치(latch_stop) — 오류 정지 등. release 전까지 유지
  2. 유효 pose 미보유(stale) — 위치 없이는 어떤 주행도 금지 (명세서 §25.2)
  3. DrivingState가 주행 계열이 아님
"""
import logging
import threading

from core.enums import DRIVING_STATES
from core.models import ControlCommand, STOP_COMMAND
from core.state_store import StateStore
from hardware.motor_driver import MotorDriver
from hardware.steering_driver import SteeringDriver

log = logging.getLogger("safety")


class SafetySupervisor:
    def __init__(self, store: StateStore, motor: MotorDriver, steering: SteeringDriver,
                 max_wheel_deg: float, max_throttle: float = 1.0):
        self._store = store
        self._motor = motor
        self._steering = steering
        self._max_wheel = max_wheel_deg
        self._max_throttle = max_throttle
        self._latched = threading.Event()

    # ---------- 차단 래치 ----------

    def latch_stop(self, reason: str) -> None:
        if not self._latched.is_set():
            log.warning("비상 정지 래치: %s", reason)
        self._latched.set()
        self.force_stop()

    def release(self, reason: str) -> None:
        if self._latched.is_set():
            log.info("정지 래치 해제: %s", reason)
        self._latched.clear()

    @property
    def latched(self) -> bool:
        return self._latched.is_set()

    # ---------- 출력 ----------

    def apply(self, command: ControlCommand) -> ControlCommand:
        """제어 명령을 검증·제한 후 하드웨어에 출력. 실제 적용된 명령을 반환."""
        snap = self._store.snapshot()
        blocked = (self._latched.is_set()
                   or snap.pose_stale
                   or snap.driving_state not in DRIVING_STATES)
        if blocked:
            applied = STOP_COMMAND
        else:
            applied = ControlCommand(
                throttle=min(max(command.throttle, 0.0), self._max_throttle),
                steering_wheel_deg=min(max(command.steering_wheel_deg, -self._max_wheel),
                                       self._max_wheel),
            )
        self._motor.set_throttle(applied.throttle)
        self._steering.set_angle_deg(applied.steering_wheel_deg)
        self._store.set_control_command(applied)
        return applied

    def force_stop(self) -> None:
        """무조건 정지 출력 — 종료·오류 경로에서 사용. 예외를 삼키지 않는다."""
        self._motor.stop()
        self._steering.center_steering()
        self._store.set_control_command(STOP_COMMAND)
