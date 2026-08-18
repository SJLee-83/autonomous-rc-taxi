"""Mock 조향 드라이버 — 바퀴각→서보각 변환 검증 포함."""
import logging

from .steering_driver import SteeringDriver

log = logging.getLogger("hw.steering")


class MockSteeringDriver(SteeringDriver):
    def __init__(self, steering_cfg: dict):
        super().__init__(steering_cfg)
        self._servo_deg = self.center
        self.cleaned_up = False

    def set_angle_deg(self, wheel_deg: float) -> None:
        servo = self.wheel_to_servo(wheel_deg)
        if servo != self._servo_deg:
            log.debug("바퀴각 %+.1f° → 서보 %.1f°", wheel_deg, servo)
        self._servo_deg = servo

    def center_steering(self) -> None:
        self._servo_deg = self.center
        log.info("조향 중앙 (서보 %.0f°)", self.center)

    def cleanup(self) -> None:
        self.cleaned_up = True
        log.info("조향 드라이버 cleanup")

    @property
    def current_servo_deg(self) -> float:
        return self._servo_deg
