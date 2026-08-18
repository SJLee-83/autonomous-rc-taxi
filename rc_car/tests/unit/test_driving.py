"""⑤ 테스트 — WaypointFollower(P-제어 이식) + DrivingWorker(전 사슬 배선).

핵심은 폐루프 시뮬: 자전거 모델 운동학으로 pose를 적분하며 DrivingWorker.tick을
50Hz로 돌려, 실제 시연 맵 위에서 PLANNING → 주행 → 코너 통과 → 도착 → notify까지
전 사슬이 실코드로 완주하는지 본다 (네트워크 없는 ⑦-라이트).

운동학 부호: 바퀴각 +(우) → heading 감소 (§2.7 반시계+).
"""
import math
import threading
import unittest
from pathlib import Path

from behavior.driving_worker import DrivingWorker
from control.legacy.waypoint_follower import WaypointFollower
from core.enums import DrivingState, ServiceKind
from core.models import LocalizationPose
from core.state_store import StateStore
from hardware.mock_motor_driver import MockMotorDriver
from hardware.mock_steering_driver import MockSteeringDriver
from mapping.lane_map import load_lane_map
from mapping.map_matcher import MapMatcher
from navigation.arrival_checker import ArrivalChecker
from navigation.destination_resolver import DestinationResolver
from navigation.route_planner import RoutePlanner
from safety.safety_supervisor import SafetySupervisor

MAP_PATH = Path(__file__).resolve().parents[3] / "map" / "main_track_map.yaml"

CONTROL_CFG = {
    "loop_hz": 50,
    "lookahead_min_m": 0.25,
    "corner_lookahead_m": 0.12,
    "ff_entry_lead_m": 0.10,
    "destination_slowdown_distance_m": 0.40,
    "arrival_radius_m": 0.10,
    "stop_trigger_radius_m": 0.05,
    "waypoint_spacing_m": 0.02,
    "steer_full_lock_error_deg": 45.0,
    "align_heading_error_deg": 10.0,
    "align_throttle_ratio": 0.6,
    "cruise_throttle": 0.5,
    "turn_throttle": 0.4,
    "near_target_throttle_ratio": 0.45,
    "departure_ramp_distance_m": 0.30,
    "corner_slowdown_distance_m": 0.40,
    "corner_exit_hold_distance_m": 0.20,
}
MATCH_CFG = {"distance_weight": 1.0, "heading_weight_per_deg": 0.01,
             "max_center_distance_m": 0.20, "max_heading_error_deg": 60.0}
ARRIVAL_CFG = {"max_estimated_speed_mps": 0.03, "settle_time_sec": 0.30,
               "max_heading_error_deg": 20.0}

MAX_WHEEL = 30.0
MAX_SPEED = 0.25
WHEELBASE = 0.14
STEERING_CFG = {"center_deg": 108, "left_max_deg": 168, "right_max_deg": 48,
                "wheel_angle_ratio": 0.526}


def _map():
    if not hasattr(_map, "cached"):
        _map.cached = load_lane_map(MAP_PATH)
    return _map.cached


class _PolicyStub:
    """CommandPolicy.notify_arrived 대역 — ARRIVED 전이 + 정지만 재현."""

    def __init__(self, store, supervisor):
        self.notified = 0
        self._store = store
        self._supervisor = supervisor

    def notify_arrived(self):
        self.notified += 1
        self._store.set_driving_state(DrivingState.ARRIVED)
        self._supervisor.force_stop()


def _make_stack():
    lane_map = _map()
    store = StateStore()
    supervisor = SafetySupervisor(store, MockMotorDriver(),
                                  MockSteeringDriver(STEERING_CFG), MAX_WHEEL)
    policy = _PolicyStub(store, supervisor)
    follower = WaypointFollower(CONTROL_CFG, MAX_WHEEL, MAX_SPEED)
    worker = DrivingWorker(
        threading.Event(), store, supervisor, policy, lane_map,
        MapMatcher(lane_map, MATCH_CFG), DestinationResolver(lane_map, 0.30),
        RoutePlanner(lane_map), ArrivalChecker(0.10, ARRIVAL_CFG), follower,
        CONTROL_CFG["loop_hz"], CONTROL_CFG["destination_slowdown_distance_m"])
    return store, supervisor, policy, follower, worker


def _set_pose(store, x, y, heading, t):
    store.update_pose(LocalizationPose(x, y, heading % 360.0, t, t))


class TestWaypointFollower(unittest.TestCase):
    def _route(self, a, sa, b, sb):
        return RoutePlanner(_map()).plan(a, sa, b, sb)

    def setUp(self):
        self.f = WaypointFollower(CONTROL_CFG, MAX_WHEEL, MAX_SPEED)

    def test_straight_ahead_centered(self):
        self.f.set_route(self._route("top_inner_eb_w", 0.1, "top_inner_eb_w", 1.1))
        cmd, remaining = self.f.compute(0.75, 2.61, 0.0)  # 차선 위, 정방향
        self.assertAlmostEqual(cmd.steering_wheel_deg, 0.0, delta=0.5)
        self.assertGreater(cmd.throttle, 0.3)
        self.assertGreater(remaining, 0.5)

    def test_target_left_steers_left(self):
        self.f.set_route(self._route("top_inner_eb_w", 0.1, "top_inner_eb_w", 1.1))
        cmd, _ = self.f.compute(0.75, 2.50, 0.0)  # 차선보다 아래 → 목표가 왼쪽(위)
        self.assertLess(cmd.steering_wheel_deg, -3.0)  # 좌 = 음수

    def test_slowdown_and_stop_near_destination(self):
        self.f.set_route(self._route("top_inner_eb_w", 0.1, "top_inner_eb_w", 1.0))
        far_cmd, _ = self.f.compute(0.80, 2.61, 0.0)
        near_cmd, rem = self.f.compute(1.45, 2.61, 0.0)   # 목적지(1.65) 0.20m 앞
        self.assertLess(near_cmd.throttle, far_cmd.throttle)
        stop_cmd, rem = self.f.compute(1.62, 2.61, 0.0)   # 정지 트리거 0.05 안
        self.assertEqual(stop_cmd.throttle, 0.0)
        self.assertLessEqual(rem, 0.05)

    def test_creeps_between_stop_trigger_and_arrival_gate(self):
        # 트리거(0.05)·게이트(0.10) 분리 — 게이트 안이라도 트리거 밖이면 계속 접근한다
        self.f.set_route(self._route("top_inner_eb_w", 0.1, "top_inner_eb_w", 1.0))
        creep_cmd, rem = self.f.compute(1.57, 2.61, 0.0)  # 목적지(1.65) 0.08m 앞
        self.assertGreater(creep_cmd.throttle, 0.0)
        self.assertLessEqual(rem, CONTROL_CFG["arrival_radius_m"])
        self.assertGreater(rem, CONTROL_CFG["stop_trigger_radius_m"])

    def test_turn_segment_fixed_throttle(self):
        # 코너(커넥터) 목표 구간에서는 turn_throttle 고정 (2026-08-03 실차 결정 — 직진/회전 분리)
        route = self._route("top_inner_eb_e", 0.1, "right_inner_sb_n", 0.15)
        self.f.set_route(route)
        # 코너 진입 직전 지점 (corner_ne_inner 시작 부근 x 4.35, y 2.61)
        cmd, _ = self.f.compute(4.30, 2.61, 0.0)
        self.assertLess(cmd.throttle, CONTROL_CFG["cruise_throttle"])
        self.assertAlmostEqual(cmd.throttle, CONTROL_CFG["turn_throttle"], delta=0.01)

    def test_departure_ramp_then_cruise(self):
        # 재출발 램프 — 경로 시작 0.3m까지 turn_throttle, 벗어나면 순항 복귀
        # (2026-08-04 실차: 정지 heading 틀어진 채 0.8 출발 → 인도 침범·이탈 대책)
        self.f.set_route(self._route("top_inner_eb_w", 0.1, "top_inner_eb_w", 1.1))
        first, _ = self.f.compute(0.80, 2.61, 0.0)
        self.assertAlmostEqual(first.throttle, CONTROL_CFG["turn_throttle"], delta=0.01)
        later, _ = self.f.compute(1.20, 2.61, 0.0)     # 출발점에서 0.40m — 램프 밖
        self.assertAlmostEqual(later.throttle, CONTROL_CFG["cruise_throttle"], delta=0.01)

    def test_corner_preslow_before_connector_target(self):
        # 코너 앞 선감속 — 목표가 아직 차선이어도 커넥터가 경로상 0.40m 안이면 turn_throttle
        # (2026-08-04 실차: 0.8→0.5 계단 전환 관성 대책 — lookahead 전환보다 앞서 감속)
        route = self._route("top_inner_eb_e", 0.1, "right_inner_sb_n", 0.15)
        self.f.set_route(route)
        self.f.compute(3.60, 2.61, 0.0)                # 램프 소진용 출발점
        cmd, _ = self.f.compute(4.00, 2.61, 0.0)       # 코너(x≈4.35) 0.35m 앞
        self.assertTrue(self.f.target_allows_seg)      # 목표는 아직 차선
        self.assertAlmostEqual(cmd.throttle, CONTROL_CFG["turn_throttle"], delta=0.01)

    def test_corner_feedforward_by_car_position(self):
        # FF는 차량이 호 위에 있을 때만 — 목표(lookahead 앞) 기준이면 진입 전 선회·호 중간
        # 소멸로 안쪽(우회전=인도) 커팅 (2026-08-05 turn_param_sim 진단, 이탈 12.2cm)
        import dataclasses as dc
        route = self._route("top_inner_eb_e", 0.1, "right_inner_sb_n", 0.15)
        no_ff = dc.replace(route, segments=tuple(
            dc.replace(s, radius_m=None) for s in route.segments))
        # 차량이 코너(4.35)와 FF 선행 개시(lead 0.10)보다 충분히 앞 — FF 기여 0
        self.f.set_route(route)
        pre, _ = self.f.compute(4.15, 2.61, 0.0)
        g = WaypointFollower(CONTROL_CFG, MAX_WHEEL, MAX_SPEED)
        g.set_route(no_ff)
        pre_no, _ = g.compute(4.15, 2.61, 0.0)
        self.assertAlmostEqual(pre.steering_wheel_deg, pre_no.steering_wheel_deg, delta=0.5)
        # 차량이 호 중간(45° 지점) — FF가 기본 조향 (R0.26 필요각 28.3°)
        self.f.set_route(route)
        mid, _ = self.f.compute(4.53, 2.53, 315.0)
        self.assertGreater(mid.steering_wheel_deg, 20.0)

    def test_corner_exit_hold_keeps_turn_throttle(self):
        # 커넥터 통과 후 0.20m까지 turn_throttle 유지 — lookahead가 목표를 출구 차선으로
        # 먼저 넘겨 회전 중 스로틀이 차선 값으로 떨어지던 문제 (2026-08-04 동쪽 삼거리 실측)
        # ※ 이음매 keep(follow)은 차선 취급이라 대상 아님 — right/left/lane_change만 해당
        route = self._route("top_inner_eb_e", 0.1, "right_inner_sb_n", 0.15)
        self.f.set_route(route)
        self.f.compute(3.30, 2.61, 0.0)                # 램프 소진용 출발점
        far, _ = self.f.compute(3.90, 2.61, 0.0)       # 코너(4.35) 0.45m 전 — 창 밖, 순항
        self.assertAlmostEqual(far.throttle, CONTROL_CFG["cruise_throttle"], delta=0.01)
        held, _ = self.f.compute(4.61, 2.28, 270.0)    # 코너 끝(y2.35) 0.07m 지남 — 유지
        self.assertTrue(self.f.target_allows_seg)      # 목표는 이미 출구 차선
        self.assertAlmostEqual(held.throttle, CONTROL_CFG["turn_throttle"], delta=0.01)

    def test_align_slowdown_exempt_on_connector(self):
        # 커넥터 기동 중에는 heading 오차가 커도 정렬 감속(×0.6) 면제 (8/1 시뮬 검증)
        route = self._route("top_inner_eb_e", 0.1, "right_inner_sb_n", 0.15)
        self.f.set_route(route)
        cmd, _ = self.f.compute(4.30, 2.61, 30.0)     # 코너 목표 + 오차 30°
        self.assertFalse(self.f.target_allows_seg)     # 목표가 커넥터인지 확인
        self.assertAlmostEqual(cmd.throttle, CONTROL_CFG["turn_throttle"], delta=0.01)

    def test_seg_gate_by_car_position_near_corner(self):
        # seg 보정은 차량이 코너 창(선감속 0.40/출구 홀드 0.20) 밖일 때만 —
        # 목표(lookahead) 기준 게이트는 코너 중 보정 누수 (2026-08-05 저녁 사고)
        route = self._route("top_inner_eb_e", 0.1, "right_inner_sb_n", 0.15)
        self.f.set_route(route)
        self.f.compute(3.30, 2.61, 0.0)            # 램프 소진용 출발점
        self.f.compute(4.00, 2.61, 0.0)            # 코너(4.35) 0.35m 앞
        self.assertTrue(self.f.target_allows_seg)  # 목표는 아직 차선인데
        self.assertFalse(self.f.seg_allowed)       # 차량 기준 코너 창 안 — 보정 금지
        g = WaypointFollower(CONTROL_CFG, MAX_WHEEL, MAX_SPEED)
        g.set_route(self._route("top_inner_eb_w", 0.1, "top_inner_eb_w", 1.1))
        g.compute(0.60, 2.61, 0.0)
        g.compute(1.20, 2.61, 0.0)                 # 코너 없는 직선 — 허용
        self.assertTrue(g.seg_allowed)

    def test_passed_destination_stops_instead_of_orbiting(self):
        # 정지 트리거(0.05)를 못 밟고 스친 경우: 최근접 후 0.08m 다시 멀어지면 정지.
        # 되돌기 배회로 맵 밖 이탈(2026-08-05 저녁) 재발 방지 — 판정은 게이트(0.15) 몫
        self.f.set_route(self._route("top_inner_eb_w", 0.1, "top_inner_eb_w", 1.0))
        self.f.compute(1.55, 2.53, 0.0)            # 접근 (y 8cm 이탈 상태)
        near, _ = self.f.compute(1.65, 2.53, 0.0)  # 최근접 ~8cm — 아직 전진
        self.assertGreater(near.throttle, 0.0)
        passed, rem = self.f.compute(1.85, 2.53, 0.0)   # 20cm 지나침
        self.assertEqual(passed.throttle, 0.0)     # 배회 대신 그 자리 정지
        self.assertFalse(self.f.seg_allowed)

    def test_align_slowdown_applies_on_lane(self):
        # 차선 목표에서는 정렬 감속 유지
        self.f.set_route(self._route("top_inner_eb_w", 0.1, "top_inner_eb_w", 1.1))
        cmd, _ = self.f.compute(0.75, 2.61, 30.0)      # 차선 목표 + 오차 30°
        self.assertTrue(self.f.target_allows_seg)
        self.assertAlmostEqual(
            cmd.throttle,
            CONTROL_CFG["cruise_throttle"] * CONTROL_CFG["align_throttle_ratio"], delta=0.01)

    def test_clear_stops(self):
        self.f.set_route(self._route("top_inner_eb_w", 0.1, "top_inner_eb_w", 1.0))
        self.f.clear()
        cmd, rem = self.f.compute(0.75, 2.61, 0.0)
        self.assertEqual(cmd.throttle, 0.0)
        self.assertFalse(self.f.has_route)

    def test_zero_length_route_stops_in_place(self):
        # 재검증 발견 버그: 제자리 목적지(길이 0 경로)에서 assert로 worker가 죽던 문제
        self.f.set_route(self._route("top_inner_eb_w", 0.5, "top_inner_eb_w", 0.5))
        cmd, rem = self.f.compute(1.15, 2.61, 0.0)
        self.assertEqual(cmd.throttle, 0.0)
        self.assertLessEqual(rem, 0.10)

    def test_seg_allowed_only_on_lane_and_follow(self):
        # 회전 커넥터 목표 구간에서는 seg 보정 금지 (경로와 싸움 방지)
        self.f.set_route(self._route("top_inner_eb_e", 0.1, "right_inner_sb_n", 0.15))
        self.f.compute(3.40, 2.61, 0.0)          # 직선 구간 — 목표는 차선 위
        self.assertTrue(self.f.target_allows_seg)
        self.f.compute(4.30, 2.61, 0.0)          # 코너 진입 — 목표는 회전 호 위
        self.assertFalse(self.f.target_allows_seg)


class TestDrivingWorkerClosedLoop(unittest.TestCase):
    """폐루프 시뮬 — 시연 맵 위 실주행 (코너 1회 포함 경로)."""

    def _simulate(self, start, dest, max_sim_s=90.0):
        store, supervisor, policy, follower, worker = _make_stack()
        dt, t = 0.02, 1000.0
        x, y, heading = start
        _set_pose(store, x, y, heading, t)
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(dest[0], dest[1], ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)

        seen_states = set()
        steps = int(max_sim_s / dt)
        for _ in range(steps):
            worker.tick()
            seen_states.add(store.snapshot().driving_state)
            if policy.notified:
                break
            cmd = store.snapshot().control_command
            v = cmd.throttle * MAX_SPEED           # 스로틀-속도 선형 가정 (mock)
            heading -= math.degrees(v / WHEELBASE * math.tan(
                math.radians(cmd.steering_wheel_deg)) * dt)  # +바퀴각(우) → heading 감소
            x += v * math.cos(math.radians(heading)) * dt
            y += v * math.sin(math.radians(heading)) * dt
            t += dt
            _set_pose(store, x, y, heading, t)
            store.set_estimated_speed(v)
        return store, policy, (x, y), seen_states

    def test_full_mission_with_corner(self):
        # top_inner_eb_w 출발 → 우측 링 바깥 북행 차선 목적지
        # (far 회전 + 사거리 직진 + 좌회전 + 외곽 코너까지 밟는 5.9m 경로)
        store, policy, final, seen = self._simulate(
            start=(0.90, 2.61, 0.0), dest=(4.86, 0.80), max_sim_s=150.0)
        self.assertEqual(policy.notified, 1, "도착 통지가 없었다")
        self.assertLess(math.hypot(final[0] - 4.86, final[1] - 0.80), 0.15)
        self.assertIn(DrivingState.FOLLOWING_ROUTE, seen)
        self.assertIn(DrivingState.APPROACHING_DESTINATION, seen)
        self.assertEqual(store.snapshot().driving_state, DrivingState.ARRIVED)

    def test_same_lane_short_hop(self):
        store, policy, final, _ = self._simulate(
            start=(0.80, 2.61, 0.0), dest=(1.70, 2.61), max_sim_s=30.0)
        self.assertEqual(policy.notified, 1)
        self.assertLess(math.hypot(final[0] - 1.70, final[1] - 2.61), 0.15)


class TestDrivingWorkerGuards(unittest.TestCase):
    def test_planning_waits_for_pose(self):
        store, supervisor, policy, follower, worker = _make_stack()
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(1.0, 1.88, ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)
        worker.tick()   # pose 없음 → 계획 보류 + 정지 유지
        self.assertEqual(store.snapshot().driving_state, DrivingState.PLANNING)
        self.assertEqual(store.snapshot().control_command.throttle, 0.0)

    def test_planning_blocked_when_unmatched(self):
        store, supervisor, policy, follower, worker = _make_stack()
        _set_pose(store, 2.5, 1.5, 90.0, 1.0)   # 사거리 한복판, 커넥터 방향과 안 맞는 북향?
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(1.0, 1.88, ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)
        _set_pose(store, 1.2, 2.2, 0.0, 2.0)    # block_nw 내부 — 매칭 불가 (§12.3)
        worker.tick()
        self.assertEqual(store.snapshot().driving_state, DrivingState.PLANNING)

    def test_stale_pose_stops_driving(self):
        store, supervisor, policy, follower, worker = _make_stack()
        _set_pose(store, 0.9, 2.61, 0.0, 1.0)
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(4.61, 0.80, ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)
        worker.tick()   # 계획 완료 → FOLLOWING_ROUTE
        self.assertEqual(store.snapshot().driving_state, DrivingState.FOLLOWING_ROUTE)
        worker.tick()   # 주행 1 tick — throttle > 0
        self.assertGreater(store.snapshot().control_command.throttle, 0.0)
        store.mark_pose_stale()
        worker.tick()   # §3.4 — 즉시 정지 명령
        self.assertEqual(store.snapshot().control_command.throttle, 0.0)

    def test_call_at_current_position_arrives_without_moving(self):
        # 관제가 차가 서 있는 자리로 호출 — 이동 없이 도착 처리돼야 한다
        store, supervisor, policy, follower, worker = _make_stack()
        _set_pose(store, 1.15, 2.61, 0.0, 1.0)
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(1.15, 2.61, ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)
        for _ in range(5):
            worker.tick()
            if policy.notified:
                break
        self.assertEqual(policy.notified, 1)
        self.assertEqual(store.snapshot().control_command.throttle, 0.0)
        self.assertEqual(store.snapshot().driving_state, DrivingState.ARRIVED)

    def test_arrival_gate_accepts_connector_into_dest_lane(self):
        # S6 실측 버그: 목적지가 교차로 틈새 좌표 → 다음 차선 시작으로 스냅되고,
        # 차는 반경 안 커넥터 위에 정지 → 차선 매칭 불일치로 complete 데드락.
        # 커넥터의 진출 차선 환산으로 도착이 성립해야 한다.
        # (배치는 정지 트리거 0.05 이내 — 8/2 트리거·게이트 분리로 그 밖은 접근 계속 구간)
        store, supervisor, policy, follower, worker = _make_stack()
        _set_pose(store, 3.11, 2.62, 0.0, 1.0)   # 상단 T 직진 커넥터 위 (틈새 x 1.86~3.14)
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(3.0, 2.61, ServiceKind.PICKUP)   # 틈새 좌표 → (3.14, 2.61) 스냅
        store.set_driving_state(DrivingState.PLANNING)
        for _ in range(5):
            worker.tick()
            if policy.notified:
                break
        self.assertEqual(policy.notified, 1)

    def test_mission_cleared_resets(self):
        store, supervisor, policy, follower, worker = _make_stack()
        _set_pose(store, 0.9, 2.61, 0.0, 1.0)
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(4.61, 0.80, ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)
        worker.tick()
        self.assertTrue(follower.has_route)
        store.clear_mission(None)               # stop 수락 상황 재현
        store.set_driving_state(DrivingState.WAITING)
        worker.tick()
        self.assertFalse(follower.has_route)


if __name__ == "__main__":
    unittest.main()
