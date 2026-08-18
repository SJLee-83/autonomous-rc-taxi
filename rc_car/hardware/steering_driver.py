"""조향 드라이버 인터페이스 — 바퀴각↔서보각 변환은 이 계층만 안다 (protocol_2 §4.4).

상위 로직: set_angle_deg(바퀴각, -30~+30)
드라이버 내부: 서보각 = center + 바퀴각 / wheel_angle_ratio  (108 ± 57)
"""
from abc import ABC, abstractmethod


class SteeringDriver(ABC):
    def __init__(self, steering_cfg: dict):
        self.center = float(steering_cfg["center_deg"])          # 108
        self.left_max = float(steering_cfg["left_max_deg"])      # 51
        self.right_max = float(steering_cfg["right_max_deg"])    # 165
        self.ratio = float(steering_cfg["wheel_angle_ratio"])    # 0.526

    def wheel_to_servo(self, wheel_deg: float) -> float:
        """바퀴각(-30~+30, +우) → 서보각(51~165). 범위 밖은 잘라낸다."""
        servo = self.center + wheel_deg / self.ratio
        return min(max(servo, self.left_max), self.right_max)

    def servo_to_wheel(self, servo_deg: float) -> float:
        return (servo_deg - self.center) * self.ratio

    @abstractmethod
    def set_angle_deg(self, wheel_deg: float) -> None:
        """바퀴각 지시. 구현체가 서보각으로 변환해 출력한다."""

    @abstractmethod
    def center_steering(self) -> None:
        """조향 중앙(서보 108) — 종료 시퀀스에서 반드시 호출 (명세서 §25.3)."""

    @abstractmethod
    def cleanup(self) -> None: ...

    @property
    @abstractmethod
    def current_servo_deg(self) -> float:
        """현재 서보각 — info.steer 로 그대로 나간다 (protocol_2 §4.4)."""

# 실물 구현: real_steering_driver.RealSteeringDriver (팀 구동 코드 어댑터, 2026-07-24 D2 해소)
