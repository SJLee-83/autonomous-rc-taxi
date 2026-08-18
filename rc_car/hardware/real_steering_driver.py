"""실물 조향 어댑터 — 팀 SteeringController를 rc_car SteeringDriver 인터페이스에 연결.

상위는 바퀴각(-30~+30)을 주고, 여기서 서보각(51~165, 중립 108)으로 변환해
vendor set_angle(서보각)에 넘긴다 (protocol_2 §4.4 — 변환은 드라이버 계층만 안다).
"""
import logging

from .steering_driver import SteeringDriver
from .vendor_bridge import load_vendor

log = logging.getLogger("hw.steering")


class RealSteeringDriver(SteeringDriver):
    def __init__(self, steering_cfg: dict, hw_cfg: dict):
        super().__init__(steering_cfg)
        vendor = load_vendor(hw_cfg)
        self._impl = vendor.SteeringController()   # 생성 시 center() 호출됨 (vendor 코드)
        # 직진 트림 = 물리 중립(hardware.yaml) − 보고 중립(vehicle.yaml 108).
        # 물리 출력에만 더한다 — 보고(current_servo_deg→info.steer)는 108 프레임 불변 (§4.4).
        # 2026-08-04 이전엔 center()에만 반영되고 주행 명령엔 빠져 있었다 (실주행 좌 드리프트 원인)
        self._trim = float(hw_cfg["servo_center_angle"]) - self.center
        self._servo_deg = self.center
        log.info("실물 조향 드라이버 초기화 (PCA9685 addr=0x%02X ch=%d, 보고 중립 %.0f° / 물리 트림 %+.0f°)",
                 hw_cfg["servo_i2c_address"], hw_cfg["servo_channel"], self.center, self._trim)

    def set_angle_deg(self, wheel_deg: float) -> None:
        servo = self.wheel_to_servo(wheel_deg)     # 108 + 바퀴각/0.526, [51,165] 클램프
        self._impl.set_angle(servo + self._trim)   # 물리 = 보고 + 트림, vendor가 [48,168] 클램프
        self._servo_deg = servo

    def center_steering(self) -> None:
        self._impl.center()
        self._servo_deg = self.center

    def cleanup(self) -> None:
        # vendor에 close()가 없다 — 조향 중앙 복귀만 보장하고 종료 (마지막 각도 유지 특성)
        self.center_steering()

    @property
    def current_servo_deg(self) -> float:
        return self._servo_deg
