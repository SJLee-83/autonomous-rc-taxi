"""vendor 코드(팀 구동 코드)가 기대하는 최상위 `config` 모듈을 생성·주입한다.

vendor 파일은 `import config`로 상수 8개를 읽는다. 팀 원본을 수정하지 않기 위해
config/hardware.yaml 값으로 가짜 `config` 모듈을 만들어 sys.modules에 넣은 뒤 import한다.

⚠️ adafruit 라이브러리는 Jetson에만 있으므로 이 모듈의 import 시점이 아니라
   load_vendor() 호출 시점에 vendor를 import한다 (PC/mock 모드 무영향).
"""
import sys
import types
from types import SimpleNamespace


def load_vendor(hw: dict) -> SimpleNamespace:
    """hardware.yaml의 orin_car 블록으로 config 모듈을 주입하고 vendor 모듈을 반환한다."""
    left = hw["servo_left_angle"]
    right = hw["servo_right_angle"]
    if left <= right:
        # servo_controller.py 클램프(max(RIGHT, min(LEFT, x)))는 LEFT > RIGHT 전제.
        # 뒤집혀 있으면 조향이 한쪽 극단으로 고정된다 — 기동 자체를 거부한다.
        raise ValueError(
            f"servo_left_angle({left}) > servo_right_angle({right}) 이어야 한다. "
            "config/hardware.yaml의 클램프 주석 참조")

    cfg = types.ModuleType("config")
    cfg.MOTOR_I2C_ADDRESS = hw["motor_i2c_address"]
    cfg.MOTOR_CHANNEL = hw["motor_channel"]
    cfg.MOTOR_SIGN = hw["motor_sign"]
    cfg.SERVO_I2C_ADDRESS = hw["servo_i2c_address"]
    cfg.SERVO_CHANNEL = hw["servo_channel"]
    cfg.SERVO_LEFT_ANGLE = left
    cfg.SERVO_RIGHT_ANGLE = right
    cfg.SERVO_CENTER_ANGLE = hw["servo_center_angle"]
    sys.modules["config"] = cfg          # vendor의 `import config`가 이것을 받는다

    from hardware.vendor import motor_controller, servo_controller
    return SimpleNamespace(
        MotorController=motor_controller.MotorController,
        SteeringController=servo_controller.SteeringController,
    )
