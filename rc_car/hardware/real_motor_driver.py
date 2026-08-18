"""실물 구동 어댑터 — 팀 MotorController를 rc_car MotorDriver 인터페이스에 연결."""
import logging

from .motor_driver import MotorDriver
from .vendor_bridge import load_vendor

log = logging.getLogger("hw.motor")


class RealMotorDriver(MotorDriver):
    def __init__(self, hw_cfg: dict):
        vendor = load_vendor(hw_cfg)
        self._impl = vendor.MotorController()   # 생성 시 stop() 호출됨 (vendor 코드)
        log.info("실물 모터 드라이버 초기화 (PCA9685 addr=0x%02X ch=%d)",
                 hw_cfg["motor_i2c_address"], hw_cfg["motor_channel"])

    def set_throttle(self, value: float) -> None:
        # vendor forward()는 abs()로 음수를 조용히 전진시킨다 → 여기서 명시적으로 거부
        if value < 0.0:
            raise ValueError("후진 없음 — throttle은 0 이상 (protocol_2 §4.3)")
        self._impl.forward(min(value, 1.0))

    def stop(self) -> None:
        # vendor stop()은 코스트(관성) 정지다. 정지 거리는 B3 캘리브레이션에서 실측한다.
        self._impl.stop()

    def cleanup(self) -> None:
        self._impl.close()
