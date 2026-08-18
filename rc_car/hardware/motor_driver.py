"""구동 모터 드라이버 인터페이스 (명세서 §6.4 하드웨어 추상화).

상위 로직은 set_throttle(0.0~1.0)만 안다. PWM·핀·듀티는 구현체의 일이다.
실물 드라이버는 이슈 D2(모델·핀 확정) 해소 후 구현한다.
"""
from abc import ABC, abstractmethod


class MotorDriver(ABC):
    @abstractmethod
    def set_throttle(self, value: float) -> None:
        """0.0(정지) ~ 1.0(최대). 후진 없음 — 음수 금지."""

    @abstractmethod
    def stop(self) -> None:
        """즉시 정지. 어떤 상태에서도 안전해야 한다."""

    @abstractmethod
    def cleanup(self) -> None:
        """종료 시 자원 해제 (GPIO 등). 반드시 stop 이후에 호출된다."""

# 실물 구현: real_motor_driver.RealMotorDriver (팀 구동 코드 어댑터, 2026-07-24 D2 해소)
