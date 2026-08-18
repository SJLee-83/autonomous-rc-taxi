"""config 로더·상태 매핑 단위 테스트."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from core.config import load_config
from core.enums import DrivingState, ServiceKind, to_service_state


def test_config_loads_with_confirmed_values():
    cfg = load_config()
    assert cfg.vehicle["vehicle"]["wheelbase_m"] == 0.14          # 실측 (0.075 아님!)
    # max_speed_mps는 B3 실측 전까지 잠정값이라 고정하지 않는다.
    # 지켜야 할 조건은 아래 test_max_speed_is_trackable_by_pose_validator 가 본다
    assert 0.0 < cfg.vehicle["vehicle"]["max_speed_mps"] <= 1.0
    assert cfg.vehicle["steering"]["center_deg"] == 108
    assert cfg.network["control_server"]["ping_timeout_sec"] == 2.0
    assert cfg.network["telemetry"]["rate_hz"] == 5
    assert cfg.network["localization"]["interval_ms"] == 100
    # lookahead가 최소 회전반경 이상인지 (명세서 §18.2)
    assert cfg.control["control"]["lookahead_min_m"] >= cfg.vehicle["vehicle"]["min_turn_radius_m"]


def test_max_speed_is_trackable_by_pose_validator():
    """차량이 낼 수 있는 속도를 위치 검증기가 따라갈 수 있어야 한다.

    넘어서면 **정상 주행 pose가 점프로 거부**되고, 0.6초 뒤 위치 유실로 판정해
    차가 스스로 멈춘다 (protocol_2 §3.4·§7.2). 속도를 올릴 때마다 확인해야 하는
    조건이라 값 고정 대신 관계식으로 감시한다.

    현재 임계값 기준 상한: 직진 0.80 m/s / 최소반경 선회 0.64 m/s
    """
    cfg = load_config()
    v = cfg.vehicle["vehicle"]["max_speed_mps"]
    r = cfg.vehicle["vehicle"]["min_turn_radius_m"]
    loc = cfg.network["localization"]
    dt = loc["interval_ms"] / 1000.0

    travel_m = v * dt
    assert travel_m <= loc["max_jump_m"], (
        f"최고 속도 {v} m/s면 주기당 {travel_m*100:.1f}cm 이동 — "
        f"max_jump_m {loc['max_jump_m']*100:.0f}cm를 넘어 정상 pose가 거부된다")

    # 최소 회전반경으로 선회할 때가 heading 변화가 가장 크다 (yaw rate = v / r)
    yaw_deg = math.degrees(v / r) * dt
    assert yaw_deg <= loc["max_heading_jump_deg"], (
        f"최고 속도 {v} m/s로 반경 {r}m 선회 시 주기당 {yaw_deg:.1f}° — "
        f"max_heading_jump_deg {loc['max_heading_jump_deg']}를 넘어 거부된다")

    # 도착 판정(반경 0.10m)이 주기당 이동보다 충분히 커야 그 안에서 표본이 잡힌다
    assert travel_m * 2 <= cfg.control["control"]["arrival_radius_m"], (
        f"주기당 {travel_m*100:.1f}cm 이동은 도착 반경 "
        f"{cfg.control['control']['arrival_radius_m']*100:.0f}cm 대비 과도하다")


def test_service_state_mapping():
    # protocol_2 §4.5 — 관제로 나가는 한글 state 5종
    assert to_service_state(DrivingState.ERROR, ServiceKind.PICKUP) == "오류 정지"
    assert to_service_state(DrivingState.FOLLOWING_ROUTE, ServiceKind.PICKUP) == "호출 응답 이동 중"
    assert to_service_state(DrivingState.FOLLOWING_ROUTE, ServiceKind.CARRY) == "고객 탑승 이동 중"
    # ARRIVED = complete 재전송 중 → 이동 중 상태 유지 (전이는 stop 수신 시)
    assert to_service_state(DrivingState.ARRIVED, ServiceKind.CARRY) == "고객 탑승 이동 중"
    assert to_service_state(DrivingState.WAITING, None) == "대기 중"
    assert to_service_state(DrivingState.WAITING, ServiceKind.PICKUP) == "탑승 대기 중"
