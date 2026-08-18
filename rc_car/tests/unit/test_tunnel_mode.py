"""터널 모드 테스트 (2026-08-06) — 동편 측위 유실 구역 한정 추측 항법(DR).

덮는 범위
    ① DeadReckoner — 직진·회전 적분, 맹목 한도 회계
    ② LocalizationService 터널 배선 — 구역 내 블랙아웃 유예 / 구역 밖은 기존 정지 /
       found=false 순간 단절 브리징 / 왜곡 pose 의심 거부(주행 중에만) / 한도 초과 시 유실
    ③ 폐루프 — 시나리오 1 픽업(안 1)이 동편 블랙아웃(x>3.95, found=false)을 뚫고 완주.
       한도를 좁히면 구역 안에서 안전 정지(폭주 없음)까지.

시계: time.monotonic 을 가짜 시계로 패치해 실시간 대기 없이 돌린다 —
서비스·워커·검증기가 전부 같은 시계를 쓰므로 전역 패치가 곧 시뮬 시계다.
"""
import json
import math
import threading

import pytest

import time
from behavior.driving_worker import DrivingWorker
from control.lane_follower import LaneFollower
from core.enums import DrivingState, ServiceKind
from core.models import ControlCommand
from core.state_store import StateStore
from core.config import load_config
from hardware.mock_motor_driver import MockMotorDriver
from hardware.mock_steering_driver import MockSteeringDriver
from localization.dead_reckoner import DeadReckoner
from localization.localization_service import LocalizationService
from localization.pose_validator import PoseValidator
from mapping.lane_map import load_lane_map
from mapping.map_matcher import MapMatcher
from navigation.arrival_checker import ArrivalChecker
from navigation.destination_resolver import DestinationResolver
from navigation.route_planner import RoutePlanner
from navigation.turn_table import TurnTable
from safety.safety_supervisor import SafetySupervisor
from pathlib import Path

MAP_PATH = Path(__file__).resolve().parents[3] / "map" / "main_track_map.yaml"
TABLE_PATH = Path(__file__).resolve().parents[2] / "config" / "turn_table.yaml"

MAP_X, MAP_Y = 5.0, 3.0

LOC_CFG = {
    "marker_id": 4, "interval_ms": 100, "marker_yaw_offset_deg": 0,
    "max_jump_m": 0.08, "max_heading_jump_deg": 15.0,
    "pose_timeout_sec": 0.3, "lost_hold_sec": 0.3,
}

# 테스트용 터널 모델 — 시뮬 운동학(v = throttle × 0.25, 즉시 조향·가감속)과 동일하게 맞춘다.
# 실차 config 의 실측 곡선과 다른 것이 정상 — 모델은 config 로 갈아끼우는 것이 설계다.
def tunnel_cfg(**over):
    cfg = {
        "enabled": True,
        "zones": [[3.0, 5.10, -0.10, 3.10]],
        "max_blind_distance_m": 0.5,
        "max_blind_time_s": 30.0,
        "distrust_deviation_m": 0.30,
        "wheel_bias_deg": 0.0,
        "servo_rate_dps": 1e9,
        "accel_mps2": 1e9,
        "speed_points": [[0.0, 0.0], [1.0, 0.25]],
    }
    cfg.update(over)
    return cfg


def msg(x=1.0, y=1.0, heading=90.0, ts=1000.0, marker_id=4, found=True):
    m = {"marker_id": marker_id, "found": found, "timestamp": ts}
    if found:
        m["position"] = {"x": x, "y": y, "z": 0.12}
        m["heading"] = heading
    return json.dumps(m)


class Hooks:
    def __init__(self):
        self.lost = 0
        self.recovered = 0

    def on_lost(self):
        self.lost += 1

    def on_recovered(self):
        self.recovered += 1


@pytest.fixture
def clock(monkeypatch):
    """전역 가짜 단조 시계 — simt[0] 를 직접 전진시킨다."""
    simt = [1000.0]
    monkeypatch.setattr(time, "monotonic", lambda: simt[0])
    return simt


def make_service(tun=None, loc_over=None):
    cfg = dict(LOC_CFG)
    cfg.update(loc_over or {})
    store = StateStore()
    hooks = Hooks()
    svc = LocalizationService(threading.Event(), store, PoseValidator(cfg, MAP_X, MAP_Y),
                              cfg, hooks.on_lost, hooks.on_recovered,
                              tunnel_cfg=tun, wheelbase_m=0.14)
    return store, svc, hooks


def drive_setup(store, throttle=0.4, steer=0.0):
    store.set_driving_state(DrivingState.FOLLOWING_ROUTE)
    store.set_control_command(ControlCommand(throttle=throttle, steering_wheel_deg=steer))


# ---------- ① DeadReckoner ----------

def test_직진_적분과_한도_회계():
    dr = DeadReckoner(tunnel_cfg(), 0.14)
    dr.anchor(3.5, 1.0, 0.0, monotonic_s=100.0, source_timestamp_s=1000.0)
    dr.propagate(throttle=0.4, steering_wheel_deg=0.0, monotonic_s=102.0)   # 2초, 0.1m/s
    x, y, heading, ts = dr.predict()
    assert x == pytest.approx(3.7, abs=1e-6)
    assert y == pytest.approx(1.0, abs=1e-6)
    assert heading == pytest.approx(0.0, abs=1e-6)
    assert dr.blind_distance_m == pytest.approx(0.2, abs=1e-6)
    assert dr.blind_time_s == pytest.approx(2.0, abs=1e-6)
    assert ts == pytest.approx(1002.0 - 0.05, abs=1e-6)
    assert dr.within_budget


def test_우조향이면_heading_감소():
    dr = DeadReckoner(tunnel_cfg(), 0.14)
    dr.anchor(4.0, 1.0, 90.0, 100.0, 1000.0)
    dr.propagate(0.4, 20.0, 101.0)          # 우회전 명령 1초
    _, _, heading, _ = dr.predict()
    assert heading < 90.0                    # §2.7 부호 규약: +바퀴각(우) → 반시계각 감소


def test_한도_초과():
    dr = DeadReckoner(tunnel_cfg(max_blind_distance_m=0.1), 0.14)
    dr.anchor(3.5, 1.0, 0.0, 100.0, 1000.0)
    dr.propagate(0.4, 0.0, 102.0)            # 0.2m > 0.1m
    assert not dr.within_budget
    dr.anchor(3.7, 1.0, 0.0, 102.0, 1002.0)  # 재앵커 → 한도 리셋
    assert dr.within_budget


# ---------- ② LocalizationService 터널 배선 ----------

def test_구역_내_블랙아웃은_DR이_잇는다(clock):
    store, svc, hooks = make_service(tun=tunnel_cfg())
    drive_setup(store)
    svc.on_raw(msg(x=3.5, y=1.0, heading=0.0, ts=1000.0))
    clock[0] += 0.35                         # timeout(0.3) 초과 — 실측 없음
    svc.tick()
    snap = store.snapshot()
    assert not svc.lost and hooks.lost == 0
    assert not snap.pose_stale
    assert snap.pose.x == pytest.approx(3.5 + 0.35 * 0.1, abs=0.01)   # DR 전진분
    clock[0] += 0.1
    svc.tick()
    assert store.snapshot().pose.x > snap.pose.x                       # 계속 잇는다


def test_구역_밖_블랙아웃은_기존대로_유실(clock):
    store, svc, hooks = make_service(tun=tunnel_cfg())
    drive_setup(store)
    svc.on_raw(msg(x=1.0, y=1.0, heading=0.0, ts=1000.0))   # 구역(x>=3.0) 밖
    clock[0] += 0.35
    svc.tick()
    assert svc.lost and hooks.lost == 1

def test_주행_상태가_아니면_DR_공급_안함(clock):
    store, svc, hooks = make_service(tun=tunnel_cfg())
    store.set_driving_state(DrivingState.WAITING)
    svc.on_raw(msg(x=3.5, y=1.0, heading=0.0, ts=1000.0))
    clock[0] += 0.35
    svc.tick()
    assert svc.lost and hooks.lost == 1      # 대기 중 유실은 기존 규칙 그대로


def test_found_false_순간단절을_브리징한다(clock):
    store, svc, hooks = make_service(tun=tunnel_cfg())
    drive_setup(store)
    svc.on_raw(msg(x=3.5, y=1.0, heading=0.0, ts=1000.0))
    clock[0] += 0.1
    svc.on_raw(msg(found=False, ts=1000.1))  # 미인식 한 장 → 즉시 stale (§3.4)
    assert store.snapshot().pose_stale
    svc.tick()                               # timeout 전이라도 stale 이면 바로 잇는다
    assert not store.snapshot().pose_stale
    assert not svc.lost


def test_왜곡_pose는_의심_거부하고_DR을_유지한다(clock):
    store, svc, hooks = make_service(tun=tunnel_cfg(), loc_over={"max_jump_m": 5.0})
    drive_setup(store, throttle=0.0)         # 정지 명령 — DR 예측 = 제자리
    svc.on_raw(msg(x=3.5, y=1.0, heading=0.0, ts=1000.0))
    clock[0] += 0.1
    svc.on_raw(msg(x=3.9, y=1.0, heading=0.0, ts=1000.1))   # 0.4m 이탈 — 8/5 왜곡 패턴
    snap = store.snapshot()
    assert snap.pose.x == pytest.approx(3.5)                 # 의심 거부 — 반영 안 됨
    clock[0] += 0.1
    svc.on_raw(msg(x=3.52, y=1.0, heading=0.0, ts=1000.2))  # 정합 실측 복귀
    assert store.snapshot().pose.x == pytest.approx(3.52)    # 재앵커


def test_정차중에는_의심판정_없이_실측이_이긴다(clock):
    store, svc, hooks = make_service(tun=tunnel_cfg(), loc_over={"max_jump_m": 5.0})
    store.set_driving_state(DrivingState.WAITING)            # 수동 재배치 상황
    svc.on_raw(msg(x=3.5, y=1.0, heading=0.0, ts=1000.0))
    clock[0] += 0.1
    svc.on_raw(msg(x=3.9, y=1.0, heading=0.0, ts=1000.1))
    assert store.snapshot().pose.x == pytest.approx(3.9)


def test_한도_초과시_유실_사슬로_떨어진다(clock):
    store, svc, hooks = make_service(tun=tunnel_cfg(max_blind_time_s=0.5))
    drive_setup(store)
    svc.on_raw(msg(x=3.5, y=1.0, heading=0.0, ts=1000.0))
    clock[0] += 0.35
    svc.tick()
    assert not svc.lost                      # 한도 안 — 유예
    clock[0] += 0.4
    svc.tick()                               # 맹목 0.75s > 0.5s
    assert svc.lost and hooks.lost == 1
    assert store.snapshot().pose_stale       # 정지 차단 복원


def test_터널_비활성이면_기존동작(clock):
    store, svc, hooks = make_service(tun=tunnel_cfg(enabled=False))
    drive_setup(store)
    svc.on_raw(msg(x=3.5, y=1.0, heading=0.0, ts=1000.0))
    clock[0] += 0.35
    svc.tick()
    assert svc.lost and hooks.lost == 1


def test_배포_config에_터널_섹션이_있다():
    cfg = load_config()
    tun = cfg.control["tunnel"]
    assert isinstance(tun["enabled"], bool)
    assert tun["zones"], "터널 구역이 비어 있다"


# ---------- ③ 폐루프 — 시나리오 1 픽업이 동편 블랙아웃을 뚫는다 ----------

PICKUP_PLAN1 = (12, 30, 13, 14, 19, 20, 3)
MARKET = (3.70, 1.12, 0.0)
HOME = (1.20, 2.61)

MATCH_CFG = {"distance_weight": 1.0, "heading_weight_per_deg": 0.01,
             "max_center_distance_m": 0.20, "max_heading_error_deg": 60.0}
ARRIVAL_CFG = {"max_estimated_speed_mps": 0.03, "settle_time_sec": 0.30,
               "max_heading_error_deg": 25.0}
STEERING_CFG = {"center_deg": 108, "left_max_deg": 168, "right_max_deg": 48,
                "wheel_angle_ratio": 0.526}
CONTROL_CFG = {
    "lookahead_min_m": 0.25, "destination_slowdown_distance_m": 0.40,
    "stop_trigger_radius_m": 0.05, "steer_full_lock_error_deg": 30.0,
    "align_heading_error_deg": 10.0, "align_throttle_ratio": 1.0,
    "cruise_throttle": 0.3, "turn_throttle": 0.5, "near_target_throttle_ratio": 1.0,
    "departure_ramp_distance_m": 0.30, "corner_exit_hold_distance_m": 0.20,
}
MAX_WHEEL, MAX_SPEED, WHEELBASE = 30.0, 0.25, 0.14


def _map():
    if not hasattr(_map, "cached"):
        _map.cached = load_lane_map(MAP_PATH)
    return _map.cached


class _PolicyStub:
    def __init__(self, store, supervisor):
        self.notified = 0
        self._store = store
        self._supervisor = supervisor

    def notify_arrived(self):
        self.notified += 1
        self._store.set_driving_state(DrivingState.ARRIVED)
        self._supervisor.force_stop()


def closed_loop(clock, blackout_x, tun, max_sim_s=300.0):
    """실배선 폐루프 + GPS found=false 블랙아웃 구역. (완주여부, 최종좌표, hooks, 경로)"""
    lane_map = _map()
    store = StateStore()
    supervisor = SafetySupervisor(store, MockMotorDriver(),
                                  MockSteeringDriver(STEERING_CFG), MAX_WHEEL)
    policy = _PolicyStub(store, supervisor)
    # 설계 기하 표 — 이상 차량 모델 폐루프라 배포 표(하드웨어 보정 개루프 값)와 양립
    # 불가 (§0-57). 배포값 검증은 test_seg_heading.TestDeployedConfig 소관.
    follower = LaneFollower(CONTROL_CFG,
                            TurnTable({"turn_table": {"defaults": {}, "turns": {}}},
                                      WHEELBASE, MAX_WHEEL),
                            MAX_WHEEL, MAX_SPEED, WHEELBASE)
    worker = DrivingWorker(
        threading.Event(), store, supervisor, policy, lane_map,
        MapMatcher(lane_map, MATCH_CFG), DestinationResolver(lane_map, 0.30),
        RoutePlanner(lane_map), ArrivalChecker(0.15, ARRIVAL_CFG), follower,
        50, CONTROL_CFG["destination_slowdown_distance_m"],
        forced_routes=(PICKUP_PLAN1,))
    hooks = Hooks()
    svc = LocalizationService(threading.Event(), store,
                              PoseValidator(LOC_CFG, MAP_X, MAP_Y), LOC_CFG,
                              hooks.on_lost, hooks.on_recovered,
                              tunnel_cfg=tun, wheelbase_m=WHEELBASE)
    x, y, heading = MARKET
    dt, gps_step, ts = 0.02, 5, 1000.0
    svc.on_raw(msg(x=x, y=y, heading=heading % 360.0, ts=ts))
    store.set_driving_state(DrivingState.WAITING)
    store.set_mission(HOME[0], HOME[1], ServiceKind.PICKUP)
    store.set_driving_state(DrivingState.PLANNING)
    path = []
    for i in range(int(max_sim_s / dt)):
        clock[0] += dt
        if i % gps_step == 0:
            ts += dt * gps_step
            if x >= blackout_x:
                svc.on_raw(msg(found=False, ts=ts))          # 동편 서버 실명 재현
            else:
                svc.on_raw(msg(x=x, y=y, heading=heading % 360.0, ts=ts))
        svc.tick()
        worker.tick()
        if policy.notified:
            break
        cmd = store.snapshot().control_command
        v = cmd.throttle * MAX_SPEED
        heading -= math.degrees(v / WHEELBASE
                                * math.tan(math.radians(cmd.steering_wheel_deg)) * dt)
        x += v * math.cos(math.radians(heading)) * dt
        y += v * math.sin(math.radians(heading)) * dt
        path.append((x, y))
    return policy.notified, (x, y), hooks, path


def test_동편_블랙아웃을_터널모드로_뚫고_완주한다(clock):
    tun = tunnel_cfg(zones=[[3.78, 5.10, -0.10, 3.10]], max_blind_distance_m=3.5,
                     max_blind_time_s=120.0)
    done, final, hooks, path = closed_loop(clock, blackout_x=3.95, tun=tun)
    assert done == 1, "블랙아웃 구간에서 완주 실패"
    assert hooks.lost == 0, "터널 모드가 유실을 막지 못했다"
    assert math.hypot(final[0] - HOME[0], final[1] - HOME[1]) < 0.30
    assert max(p[0] for p in path) > 4.0, "동편 블랙아웃 구간을 실제로 지나지 않았다"
    for px, py in path:
        assert -0.2 <= px <= 5.2 and -0.2 <= py <= 3.2, f"맵 이탈: ({px:.2f},{py:.2f})"


def test_한도가_모자라면_구역_안에서_안전정지(clock):
    tun = tunnel_cfg(zones=[[3.78, 5.10, -0.10, 3.10]], max_blind_distance_m=0.3,
                     max_blind_time_s=120.0)
    done, final, hooks, path = closed_loop(clock, blackout_x=3.95, tun=tun,
                                           max_sim_s=120.0)
    assert done == 0                          # 완주 못 하는 게 정상
    assert hooks.lost >= 1                    # 한도 소진 → 기존 유실 사슬
    # 폭주 없음 — 블랙아웃 진입 후 이동이 한도+관성 여유 안
    blind = [p for p in path if p[0] >= 3.95]
    if blind:
        first = blind[0]
        drift = max(math.hypot(px - first[0], py - first[1]) for px, py in blind)
        assert drift < 0.3 + 0.15, f"한도 초과 이동: {drift:.2f}m"
