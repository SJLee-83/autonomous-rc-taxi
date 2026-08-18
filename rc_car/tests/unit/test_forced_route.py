"""번호 노선 강제 지정 배선 테스트 (2026-08-06 시나리오 1 안 1 확정).

플래너는 순수 길이 최단이라 시장→우리집에서 안 2(12→30→13→25→6→20→3)를 고른다.
사용자 확정 노선(안 1)은 config/routes.yaml → Runtime → DrivingWorker → plan_via_numbers
통로로만 탈 수 있다 — 이 파일이 그 통로 전체의 말뚝이다.

덮는 범위
    ① RoutePlanner.plan_via_numbers — 시퀀스 편성·트림·단절/오타 거부
    ② DrivingWorker._forced_route  — 종점·현 차선 매칭, 중간 재호출 이어타기, 미해당 폴백
    ③ config/routes.yaml           — 시나리오 1 노선(안 1 + 목적 원안)이 실려 있는가
       (배포 후 이 파일이 사라지면 조용히 안 2로 도는 사고 — 스테일 배포 감지)
"""
import threading
import unittest
from pathlib import Path

from behavior.driving_worker import DrivingWorker
from control.lane_follower import LaneFollower
from core.config import load_config
from core.enums import DrivingState, ServiceKind
from core.models import LocalizationPose
from core.state_store import StateStore
from hardware.mock_motor_driver import MockMotorDriver
from hardware.mock_steering_driver import MockSteeringDriver
from mapping.lane_map import load_lane_map
from mapping.map_matcher import MapMatcher
from navigation import lane_route
from navigation.arrival_checker import ArrivalChecker
from navigation.destination_resolver import DestinationResolver
from navigation.lane_route import NUM2ID
from navigation.route_planner import RouteError, RoutePlanner
from navigation.turn_table import TurnTable
from safety.safety_supervisor import SafetySupervisor

MAP_PATH = Path(__file__).resolve().parents[3] / "map" / "main_track_map.yaml"
TABLE_PATH = Path(__file__).resolve().parents[2] / "config" / "turn_table.yaml"

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

# 배선 검증용 노선 (시나리오 1 안 1 — 플래너 자연 선택(안 2)과 다른 유일한 확정 노선이라
# "강제 지정이 플래너를 덮는가"를 검증하는 재료로 유지한다. 시연 자체는 시나리오 2로 대체됨)
PICKUP_PLAN1 = (12, 30, 13, 14, 19, 20, 3)     # 안 1 — 강제 지정 대상
PICKUP_PLAN2 = (12, 30, 13, 25, 6, 20, 3)      # 안 2 — 플래너(길이 최단)가 고르는 것
DEST_LEG = (3, 4, 29, 5)                        # 목적 구간 원안
MARKET = (3.70, 1.12, 0.0)                      # 시장 stop_point @ 12번
HOME = (1.20, 2.61)                             # 우리집 stop_point @ 3번
CLINIC = (3.70, 1.88)                           # 보건소 stop_point @ 5번

# 시나리오 2 (2026-08-06 확정, 오후에 픽업 앞 3 추가) — config/routes.yaml 이 실어야 하는 것
# 목적 노선은 (14,19,20,3) → (14,19,20) 정정 (2026-08-17): 우리집 정차 차선이 20번으로
# 옮겨진 뒤(2026-08-09, places.yaml home) 종점 3이 남아 _forced_route 매칭이 깨져 있었다.
SC2_PICKUP = (3, 4, 29, 7, 22, 14)
SC2_DEST = (14, 19, 20)


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


# ---------- ① plan_via_numbers ----------

class TestPlanViaNumbers(unittest.TestCase):
    def setUp(self):
        self.map = _map()
        self.planner = RoutePlanner(self.map)

    def test_builds_plan1_with_trims(self):
        route = self.planner.plan_via_numbers(PICKUP_PLAN1, start_s=0.30, dest_s=0.50)
        lr = lane_route.from_route(route)
        self.assertEqual(lr.lane_numbers(), PICKUP_PLAN1)
        self.assertEqual(lr.describe(), "12 →우 30 →우 13 →직 14 →우 19 →직 20 →우 3")
        first = self.map.lanes[NUM2ID[12]]
        self.assertAlmostEqual(route.segments[0].length_m, first.length_m - 0.30, places=6)
        self.assertAlmostEqual(route.segments[-1].length_m, 0.50, places=6)
        # 편성 형태는 플래너 계약과 동일해야 한다 — 차선/커넥터 교대
        kinds = [s.kind for s in route.segments]
        self.assertEqual(kinds, ["lane", "connector"] * (len(PICKUP_PLAN1) - 1) + ["lane"])

    def test_total_length_is_consistent(self):
        route = self.planner.plan_via_numbers(DEST_LEG, start_s=0.0, dest_s=0.10)
        self.assertAlmostEqual(route.total_length_m,
                               sum(s.length_m for s in route.segments), places=6)

    def test_rejects_disconnected_sequence(self):
        with self.assertRaises(RouteError):
            self.planner.plan_via_numbers([12, 13], 0.0, 0.1)   # 12→13 직접 커넥터 없음

    def test_rejects_unknown_and_short(self):
        with self.assertRaises(RouteError):
            self.planner.plan_via_numbers([12, 99], 0.0, 0.1)
        with self.assertRaises(RouteError):
            self.planner.plan_via_numbers([12], 0.0, 0.1)


# ---------- ② DrivingWorker 배선 ----------

class TestForcedRouteWiring(unittest.TestCase):
    def _stack(self, forced):
        lane_map = _map()
        store = StateStore()
        supervisor = SafetySupervisor(store, MockMotorDriver(),
                                      MockSteeringDriver(STEERING_CFG), MAX_WHEEL)
        policy = _PolicyStub(store, supervisor)
        follower = LaneFollower(CONTROL_CFG, TurnTable.load(WHEELBASE, MAX_WHEEL, TABLE_PATH),
                                MAX_WHEEL, MAX_SPEED, WHEELBASE)
        worker = DrivingWorker(
            threading.Event(), store, supervisor, policy, lane_map,
            MapMatcher(lane_map, MATCH_CFG), DestinationResolver(lane_map, 0.30),
            RoutePlanner(lane_map), ArrivalChecker(0.15, ARRIVAL_CFG), follower,
            50, CONTROL_CFG["destination_slowdown_distance_m"],
            forced_routes=forced)
        return store, follower, worker

    def _plan(self, forced, start, dest):
        store, follower, worker = self._stack(forced)
        t = 1000.0
        store.update_pose(LocalizationPose(start[0], start[1], start[2] % 360.0, t, t))
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(dest[0], dest[1], ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)
        worker.tick()
        self.assertTrue(follower.has_route, "PLANNING 1 tick 에 경로가 잡혀야 한다")
        return follower._route.lane_numbers()

    def test_without_forced_planner_picks_plan2(self):
        """대조군 — 플래너 단독은 안 2를 고른다 (여기가 깨지면 강제 지정의 전제가 바뀐 것)."""
        self.assertEqual(self._plan((), MARKET, HOME), PICKUP_PLAN2)

    def test_forced_route_overrides_planner(self):
        """🔴 핵심 — 같은 호출인데 강제 노선(안 1)이 플래너(안 2)를 덮는다."""
        self.assertEqual(self._plan((PICKUP_PLAN1,), MARKET, HOME), PICKUP_PLAN1)

    def test_resumes_from_middle_of_route(self):
        """중간 차선(19번)에서 재호출 — 그 지점부터 이어 탄다."""
        lane = _map().lanes[NUM2ID[19]]
        (x0, y0), (x1, y1) = lane.centerline[0], lane.centerline[-1]
        mid = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        import math
        heading = math.degrees(math.atan2(y1 - y0, x1 - x0))
        nums = self._plan((PICKUP_PLAN1,), (mid[0], mid[1], heading), HOME)
        self.assertEqual(nums, (19, 20, 3))

    def test_unrelated_destination_falls_back_to_planner(self):
        """종점이 다른 호출(시장→보건소)은 강제 노선을 건드리지 않는다."""
        nums = self._plan((PICKUP_PLAN1, DEST_LEG), MARKET, CLINIC)
        self.assertEqual((nums[0], nums[-1]), (12, 5))
        self.assertNotEqual(nums, PICKUP_PLAN1)


# ---------- ③ config 말뚝 ----------

class TestRoutesConfigShipsScenario2(unittest.TestCase):
    def test_scenario2_routes_present(self):
        """routes.yaml 이 배포에서 빠지거나 구버전이면 여기서 잡는다 (스테일 배포 감지)."""
        cfg = load_config()
        forced = [tuple(r) for r in (cfg.routes or {}).get("forced_routes") or []]
        self.assertIn(SC2_PICKUP, forced, "시나리오 2 픽업 노선이 config 에 없다")
        self.assertIn(SC2_DEST, forced, "시나리오 2 목적 노선이 config 에 없다")


if __name__ == "__main__":
    unittest.main()
