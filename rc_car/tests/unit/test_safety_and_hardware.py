"""SafetySupervisor 차단 조건 + 서보 변환 단위 테스트 (protocol_2 §4.4, 명세서 §33-6)."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from core.enums import DrivingState
from core.models import ControlCommand, LocalizationPose
from core.state_store import StateStore
from hardware.mock_motor_driver import MockMotorDriver
from hardware.mock_steering_driver import MockSteeringDriver

STEERING_CFG = {"center_deg": 108, "left_max_deg": 51, "right_max_deg": 165,
                "wheel_angle_ratio": 0.526}


def _supervisor():
    from safety.safety_supervisor import SafetySupervisor
    store = StateStore()
    motor = MockMotorDriver()
    steering = MockSteeringDriver(STEERING_CFG)
    return SafetySupervisor(store, motor, steering, max_wheel_deg=30.0), store, motor, steering


def _valid_pose(store):
    store.update_pose(LocalizationPose(1.0, 0.5, 90.0, time.time(), time.monotonic()))


def test_wheel_to_servo_conversion():
    d = MockSteeringDriver(STEERING_CFG)
    assert d.wheel_to_servo(0) == 108                       # 중앙 = 108 (0 아님)
    assert abs(d.wheel_to_servo(30) - 165) < 1.1            # 우측 최대
    assert abs(d.wheel_to_servo(-30) - 51) < 1.1            # 좌측 최대
    assert d.wheel_to_servo(999) <= 165                     # 범위 초과는 잘림
    assert abs(d.servo_to_wheel(d.wheel_to_servo(15)) - 15) < 0.5   # 왕복 일관성


def test_blocked_without_pose():
    sup, store, motor, _ = _supervisor()
    store.set_driving_state(DrivingState.FOLLOWING_ROUTE)
    applied = sup.apply(ControlCommand(throttle=0.5, steering_wheel_deg=10))
    assert applied.throttle == 0.0                          # 유효 pose 없이는 주행 금지
    assert motor.stopped


def test_blocked_when_not_driving_state():
    sup, store, motor, _ = _supervisor()
    _valid_pose(store)                                      # pose는 있지만 WAITING
    store.set_driving_state(DrivingState.WAITING)
    assert sup.apply(ControlCommand(throttle=0.5)).throttle == 0.0
    assert motor.stopped


def test_allowed_when_driving_with_pose():
    sup, store, motor, steering = _supervisor()
    _valid_pose(store)
    store.set_driving_state(DrivingState.FOLLOWING_ROUTE)
    applied = sup.apply(ControlCommand(throttle=0.5, steering_wheel_deg=10))
    assert applied.throttle == 0.5
    assert motor.throttle == 0.5
    assert abs(steering.current_servo_deg - (108 + 10 / 0.526)) < 0.1


def test_latch_blocks_until_release():
    sup, store, motor, _ = _supervisor()
    _valid_pose(store)
    store.set_driving_state(DrivingState.FOLLOWING_ROUTE)
    sup.latch_stop("테스트")
    assert sup.apply(ControlCommand(throttle=0.5)).throttle == 0.0
    sup.release("테스트")
    assert sup.apply(ControlCommand(throttle=0.5)).throttle == 0.5


def test_force_stop_centers_steering():
    sup, _, motor, steering = _supervisor()
    steering.set_angle_deg(20)
    sup.force_stop()
    assert motor.stopped
    assert steering.current_servo_deg == 108                # 조향 중앙 (§25.3)


def test_no_reverse():
    motor = MockMotorDriver()
    try:
        motor.set_throttle(-0.1)
        raise AssertionError("음수 throttle이 거부되지 않음")
    except ValueError:
        pass                                                # 후진 없음 (protocol_2 §4.3)
