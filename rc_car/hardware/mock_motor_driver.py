"""Mock 구동 드라이버 — 하드웨어 없이 전체 로직을 돌리기 위한 구현 (명세서 §33-2)."""
import logging

from .motor_driver import MotorDriver

log = logging.getLogger("hw.motor")


class MockMotorDriver(MotorDriver):
    def __init__(self, config: dict | None = None):
        self.throttle = 0.0
        self.stopped = True
        self.cleaned_up = False

    def set_throttle(self, value: float) -> None:
        if value < 0.0:
            raise ValueError("후진 없음 — throttle은 0 이상 (protocol_2 §4.3)")
        value = min(value, 1.0)
        if value != self.throttle:
            log.debug("throttle %.3f", value)
        self.throttle = value
        self.stopped = value == 0.0

    def stop(self) -> None:
        self.throttle = 0.0
        self.stopped = True
        log.info("모터 정지")

    def cleanup(self) -> None:
        self.cleaned_up = True
        log.info("모터 드라이버 cleanup")
