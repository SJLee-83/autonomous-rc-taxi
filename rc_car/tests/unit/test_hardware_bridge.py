"""하드웨어 브리지 단위 테스트 — vendor 사본 동일성 · 클램프 전제 · 변환 정합.

adafruit 라이브러리는 PC에 없으므로 vendor 모듈 자체는 import하지 않는다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

RC_CAR = Path(__file__).resolve().parents[2]
VENDOR_ORIGIN = RC_CAR.parent / "vendor_원본"   # 차량파트/vendor_원본 (2026-07-27 폴더 정리)


def test_vendor_copies_match_originals():
    """팀 원본과 rc_car 내 사본이 달라지면 실패한다 (사본 수정 금지 규칙)."""
    for name in ("motor_controller.py", "servo_controller.py"):
        original = VENDOR_ORIGIN / name
        copy = RC_CAR / "hardware" / "vendor" / name
        if not original.exists():
            continue    # 배포 환경(rc_car만 복사)에서는 원본이 없을 수 있다
        assert original.read_bytes() == copy.read_bytes(), \
            f"{name}: 원본과 사본이 다름 — 팀 원본 갱신 시 vendor/에 재복사할 것"


def test_hardware_yaml_clamp_precondition():
    """servo_controller.py 클램프(max(RIGHT, min(LEFT, x)))의 전제: LEFT > RIGHT (수치상)."""
    from core.config import load_hardware
    hw = load_hardware()
    assert hw["servo_left_angle"] > hw["servo_right_angle"], \
        "클램프 전제 위반 — 이름 그대로(좌51/우165) 넣으면 조향이 165로 고정된다"
    assert hw["servo_center_angle"] == 118                      # 2026-08-04 오후 GPS 실측 확정 (전축 수리 후 재트림)
    assert hw["servo_right_angle"] <= hw["servo_center_angle"] <= hw["servo_left_angle"]


def test_vendor_bridge_rejects_swapped_angles():
    """클램프 전제가 뒤집힌 설정으로는 기동 자체가 거부되어야 한다."""
    from hardware.vendor_bridge import load_vendor
    bad = {"motor_i2c_address": 0x60, "motor_channel": 0, "motor_sign": 1,
           "servo_i2c_address": 0x40, "servo_channel": 0,
           "servo_left_angle": 51, "servo_right_angle": 165,   # 이름 그대로 = 잘못
           "servo_center_angle": 108}
    try:
        load_vendor(bad)
        raise AssertionError("뒤집힌 각도 설정이 거부되지 않음")
    except ValueError:
        pass


def test_wheel_conversion_stays_inside_vendor_clamp():
    """우리 변환(108 ± 바퀴각/0.526)의 출력이 vendor 클램프 [51,165] 안에 있는지."""
    from core.config import load_hardware
    from hardware.mock_steering_driver import MockSteeringDriver
    hw = load_hardware()
    d = MockSteeringDriver({"center_deg": 108, "left_max_deg": 51,
                            "right_max_deg": 165, "wheel_angle_ratio": 0.526})
    for wheel in (-30, -15, 0, 15, 30):
        servo = d.wheel_to_servo(wheel)
        assert hw["servo_right_angle"] <= servo <= hw["servo_left_angle"]


def test_real_driver_applies_trim_to_physical_only(monkeypatch):
    """주행 명령의 물리 출력 = 보고 서보각 + 트림, 보고는 108 프레임 불변 (2026-08-04 버그 회귀 방지).

    vendor는 adafruit이 필요하므로 기록만 하는 가짜로 대체한다.
    """
    from hardware import real_steering_driver

    sent = []

    class _FakeImpl:
        def __init__(self):
            sent.append("center")          # vendor 생성자와 동일하게 center() 상당 호출 표시

        def set_angle(self, angle):
            sent.append(float(angle))

        def center(self):
            sent.append("center")

    class _FakeVendor:
        SteeringController = _FakeImpl

    monkeypatch.setattr(real_steering_driver, "load_vendor", lambda hw: _FakeVendor)
    d = real_steering_driver.RealSteeringDriver(
        {"center_deg": 108, "left_max_deg": 51, "right_max_deg": 165,
         "wheel_angle_ratio": 0.526},
        {"servo_i2c_address": 0x60, "servo_channel": 0, "servo_center_angle": 118})

    d.set_angle_deg(0.0)                                   # 직진
    assert sent[-1] == 118.0                               # 물리 = 108 + 트림 10
    assert d.current_servo_deg == 108.0                    # 보고는 108 그대로 (§4.4)

    d.set_angle_deg(10.0)                                  # 우로 10° (보고 127.01)
    assert abs(sent[-1] - (108 + 10 / 0.526 + 10)) < 1e-6  # 물리 = 보고 + 10
    assert abs(d.current_servo_deg - (108 + 10 / 0.526)) < 1e-6
