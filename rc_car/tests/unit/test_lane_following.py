"""차선 번호 point-to-point + 2단 회전 테스트 (2026-08-06 재설계 §0-45).

덮는 범위
    ① lane_route  — Route(원호 포함) → LaneRoute 변환에서 **커넥터 원호가 실제로 버려지는가**
    ② lane_route  — 번호 노선 직접 지정 (`11 → 22 → 14`)
    ③ turn_table  — 조향각 기하 산출·부호·표 덮어쓰기·검증
    ④ vision_marks— 노면표시 트리거 판정 (근접행·conf·횡거리·신선도)
    ⑤ LaneFollower— 상태기계(FOLLOW→ARMED→TURNING)와 두 트리거 경로(비전 / 좌표 폴백)
    ⑥ 폐루프 시뮬 — 시연 맵 위에서 회전 포함 경로를 실제로 완주하는가

부호 규약: 바퀴각 +(우) → heading 감소 (§2.7 반시계+). test_driving.py 와 같다.
"""
import math
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from behavior.driving_worker import DrivingWorker
import control.lane_follower as lane_follower_mod
from control.lane_follower import ARMED, FOLLOW, TURNING, LaneFollower
from core.enums import DrivingState, ServiceKind
from core.exceptions import ConfigError
from core.models import LocalizationPose
from core.state_store import StateStore
from hardware.mock_motor_driver import MockMotorDriver
from hardware.mock_steering_driver import MockSteeringDriver
from mapping.lane_map import load_lane_map
from mapping.map_matcher import MapMatcher
from navigation import lane_route
from navigation.arrival_checker import ArrivalChecker
from navigation.destination_resolver import DestinationResolver
from navigation.lane_route import NUM2ID, LaneRouteError
from navigation.route_planner import RoutePlanner
from navigation.turn_table import TurnTable
from perception.vision_marks import MarkAdapter, StaticMarkSource
from safety.safety_supervisor import SafetySupervisor

MATCH_CFG = {"distance_weight": 1.0, "heading_weight_per_deg": 0.01,
             "max_center_distance_m": 0.20, "max_heading_error_deg": 60.0}
ARRIVAL_CFG = {"max_estimated_speed_mps": 0.03, "settle_time_sec": 0.30,
               "max_heading_error_deg": 25.0}
STEERING_CFG = {"center_deg": 108, "left_max_deg": 168, "right_max_deg": 48,
                "wheel_angle_ratio": 0.526}


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

MAP_PATH = Path(__file__).resolve().parents[3] / "map" / "main_track_map.yaml"
TABLE_PATH = Path(__file__).resolve().parents[2] / "config" / "turn_table.yaml"

CONTROL_CFG = {
    "lookahead_min_m": 0.25,
    "destination_slowdown_distance_m": 0.40,
    "stop_trigger_radius_m": 0.05,
    "steer_full_lock_error_deg": 30.0,
    "align_heading_error_deg": 10.0,
    "align_throttle_ratio": 1.0,
    "cruise_throttle": 0.3,
    "turn_throttle": 0.5,
    "near_target_throttle_ratio": 1.0,
    "departure_ramp_distance_m": 0.30,
    "corner_exit_hold_distance_m": 0.20,
}
MAX_WHEEL, MAX_SPEED, WHEELBASE = 30.0, 0.25, 0.14


def _map():
    if not hasattr(_map, "cached"):
        _map.cached = load_lane_map(MAP_PATH)
    return _map.cached


def _table(raw=None):
    if raw is None:
        return TurnTable.load(WHEELBASE, MAX_WHEEL, TABLE_PATH)
    return TurnTable(raw, WHEELBASE, MAX_WHEEL)


def _payload(dets, ts=None, height=241.0, width=954.0, ppm=250.0):
    return {"timestamp": time.time() if ts is None else ts,
            "birdseye_size": [width, height], "pixels_per_meter": ppm,
            "model": {"detections": dets}}


def _det(cls, conf, y_bottom, x_center=477.0, w=40.0):
    return {"cls": cls, "conf": conf,
            "xyxy_px": [x_center - w / 2, y_bottom - 20, x_center + w / 2, y_bottom]}


# ---------- ① 원호 폐기 ----------

class TestLaneRouteFromRoute(unittest.TestCase):
    def setUp(self):
        self.map = _map()
        self.planner = RoutePlanner(self.map)

    def _lane_route(self, a, b):
        la, lb = self.map.lanes[a], self.map.lanes[b]
        route = self.planner.plan(a, 0.05, b, lb.length_m - 0.05)
        return route, lane_route.from_route(route)

    def test_connector_arc_points_are_discarded(self):
        """🔴 핵심 — 회전은 '지시'만 남고 원호 좌표는 사라진다."""
        route, lr = self._lane_route("mid_eb2_w", "bot_inner_wb_w")
        arc_points = sum(len(s.centerline) for s in route.segments if s.kind == "connector")
        self.assertGreater(arc_points, 0, "원본 경로에 커넥터 원호가 있어야 대조가 된다")
        for turn in lr.turns:
            self.assertFalse(hasattr(turn, "centerline"),
                             "Turn 이 원호 좌표를 들고 있다 — 가상 라인 트레이싱 잔재")
        # 노선이 들고 있는 좌표는 전부 차선(직선) 위 점이다
        for leg in lr.legs:
            self.assertLessEqual(len(leg.points), 2,
                                 f"{leg.lane_id}: 차선은 2점 직선이어야 한다")

    def test_legs_and_turns_interleave(self):
        _, lr = self._lane_route("mid_eb2_w", "bot_inner_wb_w")
        self.assertEqual(len(lr.turns), len(lr.legs) - 1)
        for turn, nxt in zip(lr.turns, lr.legs[1:]):
            self.assertEqual(turn.to_lane_id, nxt.lane_id)
            self.assertIn(turn.maneuver, ("left", "right", "straight"))

    def test_turn_key_uses_lane_numbers(self):
        _, lr = self._lane_route("mid_eb2_w", "bot_inner_wb_w")
        for turn in lr.turns:
            self.assertRegex(turn.key, r"^\d+->\d+$")

    def test_describe_is_readable(self):
        _, lr = self._lane_route("mid_eb2_w", "bot_inner_wb_w")
        text = lr.describe()
        self.assertIn("→", text)
        self.assertTrue(text.split()[0].isdigit())

    def test_tail_length_shrinks(self):
        _, lr = self._lane_route("mid_eb2_w", "bot_inner_wb_w")
        tails = [lr.tail_length_m(i) for i in range(len(lr.legs))]
        self.assertEqual(tails[-1], 0.0)
        for a, b in zip(tails, tails[1:]):
            self.assertGreater(a, b)


# ---------- ② 번호 노선 직접 지정 ----------

class TestLaneRouteFromNumbers(unittest.TestCase):
    def test_numbering_matches_map(self):
        lanes = _map().lanes
        self.assertEqual(len(NUM2ID), 32)
        for num, lid in NUM2ID.items():
            self.assertIn(lid, lanes, f"{num}번({lid})이 맵에 없다")

    def test_user_route_11_22_14(self):
        lr = lane_route.from_lane_numbers(_map(), [11, 22, 14])
        self.assertEqual(lr.lane_numbers(), (11, 22, 14))
        self.assertEqual([t.maneuver for t in lr.turns], ["right", "right"])

    def test_disconnected_pair_raises(self):
        with self.assertRaises(LaneRouteError):
            lane_route.from_lane_numbers(_map(), [11, 16])

    def test_unknown_number_raises(self):
        with self.assertRaises(LaneRouteError):
            lane_route.from_lane_numbers(_map(), [11, 99])


# ---------- ③ 회전 표 ----------

class TestTurnTable(unittest.TestCase):
    def setUp(self):
        self.lr = lane_route.from_lane_numbers(_map(), [11, 22, 14])
        self.table = _table()

    def test_right_turn_wheel_is_positive(self):
        spec = self.table.resolve(self.lr.turns[0])
        self.assertGreater(spec.wheel_deg, 0, "우회전 바퀴각은 + (ControlCommand 부호)")

    def test_wheel_from_arc_geometry(self):
        turn = self.lr.turns[0]
        spec = self.table.resolve(turn)
        expected = math.degrees(math.atan(WHEELBASE / turn.radius_m))
        self.assertAlmostEqual(abs(spec.wheel_deg), min(expected, MAX_WHEEL), places=3)

    def test_left_turn_wheel_is_negative(self):
        lr = lane_route.from_lane_numbers(_map(), [7, 22])
        spec = self.table.resolve(lr.turns[0])
        self.assertEqual(lr.turns[0].maneuver, "left")
        self.assertLess(spec.wheel_deg, 0)

    def test_wheel_clamped_to_max(self):
        table = _table({"turn_table": {"defaults": {"wheel_deg": 90.0}, "turns": {}}})
        spec = table.resolve(self.lr.turns[0])
        self.assertEqual(spec.wheel_deg, MAX_WHEEL)

    def test_table_entry_overrides_defaults(self):
        spec = self.table.resolve(self.lr.turns[0])   # 11->22 는 yaml 에 항목이 있다
        self.assertTrue(spec.from_table)
        self.assertEqual(spec.trigger.cls, "crosswalk")
        self.assertAlmostEqual(spec.trigger.near_row_frac, 0.70)
        # 표에 없는 값은 defaults 에서 온다
        self.assertAlmostEqual(spec.arm_distance_m, 0.60)

    def test_unlisted_turn_falls_back_to_defaults(self):
        # 12->30: 표에 없고 대칭(6->20)도 없다 (0806에 7->22 가 표에 들어가 예시 교체)
        lr = lane_route.from_lane_numbers(_map(), [12, 30])
        spec = self.table.resolve(lr.turns[0])
        self.assertFalse(spec.from_table)
        self.assertFalse(spec.trigger.uses_vision, "표에 없으면 좌표 폴백이어야 한다")
        self.assertTrue(spec.fallback_at_lane_end)

    def test_straight_connector_is_not_a_turn(self):
        lanes = _map().lanes
        planner = RoutePlanner(lanes and _map())
        route = planner.plan("top_inner_eb_w", 0.05, "top_inner_eb_e", 0.5)
        lr = lane_route.from_route(route)
        straights = [t for t in lr.turns if t.maneuver == "straight"]
        self.assertTrue(straights, "이 경로에는 교차로 직진 커넥터가 있어야 한다")
        for turn in straights:
            spec = self.table.resolve(turn)
            self.assertFalse(spec.is_turn)
            self.assertEqual(spec.wheel_deg, 0.0)

    def test_bad_trigger_class_rejected(self):
        with self.assertRaises(ConfigError):
            _table({"turn_table": {"defaults": {"trigger": {"class": "zebra"}}, "turns": {}}})

    def test_bad_row_fraction_rejected(self):
        with self.assertRaises(ConfigError):
            _table({"turn_table": {"defaults": {"trigger": {"near_row_frac": 1.4}}, "turns": {}}})

    def test_bad_turn_key_rejected(self):
        with self.assertRaises(ConfigError):
            _table({"turn_table": {"defaults": {}, "turns": {"11-22": {}}}})

    def test_shipped_table_covers_recorded_turns(self):
        """비전 트레이스(녹화)가 있는 회전만 표에 있어야 한다 (근거 없는 값 금지).

        0805 drive3/4 = {11->22, 22->14, 7->22} + 0806 run1 = 시나리오 2 전 6회전.
        """
        recorded = ("11->22", "22->14", "7->22",
                    "4->29", "29->7", "14->19", "20->3")
        for key in self.table.tuned_keys:
            self.assertIn(key, recorded)


# ---------- ④ 비전 트리거 ----------

class TestMarkAdapter(unittest.TestCase):
    def setUp(self):
        self.source = StaticMarkSource()
        self.marks = MarkAdapter(self.source, freshness_max_s=0.5)
        self.spec = _table().resolve(
            lane_route.from_lane_numbers(_map(), [11, 22]).turns[0]).trigger

    def _observe(self, payload):
        self.source.payload = payload
        return self.marks.observe()

    def test_near_mark_fires(self):
        self._observe(_payload([_det("crosswalk", 0.8, y_bottom=200.0)]))  # 241*0.70=168.7
        self.assertTrue(self.marks.triggered(self.spec))

    def test_far_mark_does_not_fire(self):
        self._observe(_payload([_det("crosswalk", 0.8, y_bottom=120.0)]))
        self.assertFalse(self.marks.triggered(self.spec))

    def test_wrong_class_does_not_fire(self):
        self._observe(_payload([_det("stop_line", 0.9, y_bottom=230.0)]))
        self.assertFalse(self.marks.triggered(self.spec))

    def test_low_confidence_does_not_fire(self):
        self._observe(_payload([_det("crosswalk", 0.2, y_bottom=230.0)]))
        self.assertFalse(self.marks.triggered(self.spec))

    def test_other_lane_mark_rejected_by_lateral(self):
        # 축에서 1.0m 떨어진 검출 (250px/m) — max_lateral_m 0.30 밖
        self._observe(_payload([_det("crosswalk", 0.9, y_bottom=230.0, x_center=477 + 250)]))
        self.assertFalse(self.marks.triggered(self.spec))

    def test_stale_payload_is_invalid(self):
        obs = self._observe(_payload([_det("crosswalk", 0.9, 230.0)], ts=time.time() - 5.0))
        self.assertFalse(obs.valid)
        self.assertFalse(self.marks.triggered(self.spec))

    def test_missing_payload_is_invalid(self):
        self.assertFalse(self._observe(None).valid)

    def test_malformed_detection_is_skipped_not_fatal(self):
        obs = self._observe(_payload([{"cls": "crosswalk"},                    # 상자 없음
                                      {"xyxy_px": [1, 2, 3, 4]},               # 클래스 없음
                                      _det("crosswalk", 0.9, 230.0)]))
        self.assertTrue(obs.valid)
        self.assertEqual(len(obs.marks), 1)
        self.assertTrue(self.marks.triggered(self.spec))

    def test_axis_defaults_to_half_width(self):
        obs = self._observe(_payload([_det("crosswalk", 0.9, 230.0)]))
        self.assertAlmostEqual(obs.axis_px, 477.0)

    def test_no_vision_trigger_when_spec_has_none(self):
        spec = _table({"turn_table": {"defaults": {}, "turns": {}}}).resolve(
            lane_route.from_lane_numbers(_map(), [11, 22]).turns[0]).trigger
        self._observe(_payload([_det("crosswalk", 0.9, 230.0)]))
        self.assertFalse(self.marks.triggered(spec))


# ---------- ⑤ 상태기계 ----------

class TestLaneFollowerStates(unittest.TestCase):
    def setUp(self):
        self.route = lane_route.from_lane_numbers(_map(), [11, 22, 14])
        self.source = StaticMarkSource()
        self.marks = MarkAdapter(self.source, freshness_max_s=0.5)
        self.f = LaneFollower(CONTROL_CFG, _table(), MAX_WHEEL, MAX_SPEED,
                              WHEELBASE, marks=self.marks)
        self.f.set_route(self.route)
        self.leg0 = self.route.legs[0]

    def _at(self, along_m):
        """첫 다리 위 진행거리 along_m 지점의 좌표·방위."""
        (x0, y0), (x1, y1) = self.leg0.points[0], self.leg0.points[-1]
        n = math.hypot(x1 - x0, y1 - y0)
        ux, uy = (x1 - x0) / n, (y1 - y0) / n
        return x0 + ux * along_m, y0 + uy * along_m, math.degrees(math.atan2(uy, ux))

    def test_starts_in_follow_and_drives_straight(self):
        x, y, h = self._at(0.05)
        cmd, rem = self.f.compute(x, y, h)
        self.assertEqual(self.f.state, FOLLOW)
        self.assertAlmostEqual(cmd.steering_wheel_deg, 0.0, delta=1.0)
        self.assertGreater(cmd.throttle, 0.0)
        self.assertAlmostEqual(rem, self.route.total_length_m - 0.05, delta=0.02)

    def test_arms_near_lane_end(self):
        self.f.compute(*self._at(0.05))
        self.assertEqual(self.f.state, FOLLOW)
        self.f.compute(*self._at(self.leg0.length_m - 0.50))    # arm 0.60 안
        self.assertEqual(self.f.state, ARMED)

    def test_armed_blocks_seg_correction(self):
        self.f.compute(*self._at(0.05))
        self.assertTrue(self.f.seg_allowed)
        self.f.compute(*self._at(self.leg0.length_m - 0.50))
        self.assertFalse(self.f.seg_allowed, "코너 창에서는 seg 보정 금지")

    def test_armed_uses_turn_throttle(self):
        self.f.compute(*self._at(0.05))
        cmd, _ = self.f.compute(*self._at(self.leg0.length_m - 0.50))
        self.assertAlmostEqual(cmd.throttle, CONTROL_CFG["turn_throttle"])

    def test_vision_trigger_starts_turn_before_lane_end(self):
        self.f.compute(*self._at(0.05))
        x, y, h = self._at(self.leg0.length_m - 0.40)
        self.f.compute(x, y, h)
        self.assertEqual(self.f.state, ARMED, "아직 차선 끝이 아니다")
        self.source.payload = _payload([_det("crosswalk", 0.9, y_bottom=230.0)])
        self.marks.observe()
        cmd, _ = self.f.compute(x, y, h)
        self.assertEqual(self.f.state, TURNING, "비전 트리거로 차선 끝 전에 개시해야 한다")
        self.assertGreater(cmd.steering_wheel_deg, 5.0, "우회전 = + 바퀴각")

    def test_coordinate_fallback_when_no_vision(self):
        self.f.compute(*self._at(0.05))
        self.f.compute(*self._at(self.leg0.length_m - 0.40))
        self.assertEqual(self.f.state, ARMED)
        self.f.compute(*self._at(self.leg0.length_m + 0.01))    # 차선 끝 통과
        self.assertEqual(self.f.state, TURNING, "비전이 없으면 차선 끝에서 개시(좌표 폴백)")

    def test_stale_vision_falls_back(self):
        self.source.payload = _payload([_det("crosswalk", 0.9, 230.0)], ts=time.time() - 9.0)
        self.marks.observe()
        self.f.compute(*self._at(0.05))
        self.f.compute(*self._at(self.leg0.length_m - 0.40))
        self.assertEqual(self.f.state, ARMED, "낡은 관측으로 발화하면 안 된다")

    def test_turn_holds_table_angle_and_blocks_seg(self):
        self.f.compute(*self._at(0.05))
        self.f.compute(*self._at(self.leg0.length_m + 0.01))
        spec = _table().resolve(self.route.turns[0])
        cmd, _ = self.f.compute(*self._at(self.leg0.length_m + 0.02))
        self.assertEqual(self.f.state, TURNING)
        self.assertAlmostEqual(cmd.steering_wheel_deg, spec.wheel_deg, places=6)
        self.assertAlmostEqual(cmd.throttle, CONTROL_CFG["turn_throttle"])
        self.assertFalse(self.f.seg_allowed)

    def test_turn_ends_on_alignment_with_next_lane(self):
        self.f.compute(*self._at(0.05))
        self.f.compute(*self._at(self.leg0.length_m + 0.01))
        self.assertEqual(self.f.state, TURNING)
        self.assertEqual(self.f.leg_index, 0)
        nxt = self.route.legs[1]
        ex, ey = self.route.turns[0].exit_point
        # 다음 차선 방위로 정렬된 채 min_turn_m 이상 진행한 상태를 만든다.
        # 22번은 짧은 차선(0.35m)이라 회전이 끝나는 즉시 다음 회전으로 재무장한다 —
        # 상태가 아니라 **다리 인덱스**로 종료를 판정해야 하는 이유다.
        for _ in range(30):
            self.f.compute(ex, ey, nxt.heading_deg())
            if self.f.leg_index == 1:
                break
        self.assertEqual(self.f.leg_index, 1, "정렬되면 회전이 끝나고 다음 다리로 넘어가야 한다")
        self.assertNotEqual(self.f.state, TURNING)
        self.assertEqual(self.f.current_lane_num, 22)

    def test_no_route_stops(self):
        f = LaneFollower(CONTROL_CFG, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        cmd, rem = f.compute(1.0, 1.0, 0.0)
        self.assertEqual(cmd.throttle, 0.0)
        self.assertFalse(f.has_route)

    def test_clear_drops_route(self):
        self.f.clear()
        cmd, _ = self.f.compute(*self._at(0.1))
        self.assertEqual(cmd.throttle, 0.0)
        self.assertFalse(self.f.has_route)

    def test_departure_ramp_caps_throttle(self):
        x, y, h = self._at(0.02)
        cmd, _ = self.f.compute(x, y, h)
        self.assertLessEqual(cmd.throttle, CONTROL_CFG["turn_throttle"] + 1e-9)


class TestLaneFollowerArrival(unittest.TestCase):
    """마지막 다리 — 정지 트리거와 지나침 정지."""

    def setUp(self):
        self.route = lane_route.from_lane_numbers(_map(), [11])
        self.f = LaneFollower(CONTROL_CFG, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        self.f.set_route(self.route)
        self.leg = self.route.legs[0]

    def _at(self, along_m):
        (x0, y0), (x1, y1) = self.leg.points[0], self.leg.points[-1]
        n = math.hypot(x1 - x0, y1 - y0)
        return x0 + (x1 - x0) / n * along_m, y0 + (y1 - y0) / n * along_m

    def test_stops_inside_trigger_radius(self):
        x, y = self._at(self.leg.length_m - 0.02)
        cmd, rem = self.f.compute(x, y, self.leg.heading_deg())
        self.assertEqual(cmd.throttle, 0.0)
        self.assertLessEqual(rem, CONTROL_CFG["stop_trigger_radius_m"])

    def test_passed_target_stops_instead_of_wandering(self):
        h = self.leg.heading_deg()
        self.f.compute(*self._at(self.leg.length_m - 0.30), h)
        self.f.compute(*self._at(self.leg.length_m - 0.10), h)
        cmd, _ = self.f.compute(*self._at(self.leg.length_m + 0.20), h)
        self.assertEqual(cmd.throttle, 0.0, "지나쳤으면 되돌지 말고 정차 (배회 사고 대책)")


# ---------- ⑥ 폐루프 ----------

class TestClosedLoop(unittest.TestCase):
    """자전거 모델로 실제 완주하는지 — 회전이 열리고 닫히는지까지 본다."""

    def _run(self, numbers, vision=False, max_s=180.0, dt=0.02):
        route = lane_route.from_lane_numbers(_map(), numbers)
        source = StaticMarkSource()
        marks = MarkAdapter(source, freshness_max_s=1e9) if vision else None
        f = LaneFollower(CONTROL_CFG, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE, marks=marks)
        f.set_route(route)
        x, y = route.legs[0].points[0]
        heading = route.legs[0].heading_deg()
        if vision:
            source.payload = _payload([_det("crosswalk", 0.9, 235.0),
                                       _det("stop_line", 0.9, 235.0)], ts=time.time())
            marks.observe()
        states, path = set(), []
        for _ in range(int(max_s / dt)):
            cmd, rem = f.compute(x, y, heading)
            states.add(f.state)
            path.append((x, y))
            if cmd.throttle == 0.0 and rem <= CONTROL_CFG["stop_trigger_radius_m"]:
                return True, (x, y), states, rem, path
            v = cmd.throttle * MAX_SPEED
            heading -= math.degrees(v / WHEELBASE
                                    * math.tan(math.radians(cmd.steering_wheel_deg)) * dt)
            x += v * math.cos(math.radians(heading)) * dt
            y += v * math.sin(math.radians(heading)) * dt
        return False, (x, y), states, rem, path

    def test_straight_only_route_completes(self):
        done, final, states, rem, _ = self._run([11])
        self.assertTrue(done, f"직선 노선 미완주 (남은 {rem:.3f}m)")
        goal = _map().lanes[NUM2ID[11]].centerline[-1]
        self.assertLess(math.hypot(final[0] - goal[0], final[1] - goal[1]), 0.10)

    def test_turn_route_completes_with_coordinate_fallback(self):
        done, final, states, rem, _ = self._run([11, 22, 14])
        self.assertTrue(done, f"회전 노선 미완주 (남은 {rem:.3f}m, 상태 {states})")
        self.assertIn(ARMED, states)
        self.assertIn(TURNING, states)
        goal = _map().lanes[NUM2ID[14]].centerline[-1]
        self.assertLess(math.hypot(final[0] - goal[0], final[1] - goal[1]), 0.15)

    def test_vision_trigger_turns_earlier_than_fallback(self):
        """비전이 있으면 좌표 폴백보다 **먼저** 꺾어야 한다 (설계 목적 자체)."""
        _, _, _, _, path_fb = self._run([11, 22, 14], vision=False)
        _, _, _, _, path_v = self._run([11, 22, 14], vision=True)

        def first_turn_x(path):
            # 첫 다리는 동향(mid_eb2_w) — 진행 중 y 가 처음으로 크게 꺾인 지점의 x
            y0 = path[0][1]
            for px, py in path:
                if abs(py - y0) > 0.05:
                    return px
            return float("inf")

        self.assertLess(first_turn_x(path_v), first_turn_x(path_fb),
                        "비전 트리거가 좌표 폴백보다 먼저 꺾지 않았다")

    def test_stays_inside_map(self):
        _, _, _, _, path = self._run([11, 22, 14])
        for px, py in path:
            self.assertTrue(-0.2 <= px <= 5.2 and -0.2 <= py <= 3.2,
                            f"맵 밖 이탈: ({px:.2f}, {py:.2f})")


class TestTimeBasedStartOffset(unittest.TestCase):
    """start_offset 소모가 GPS 이동량이 아니라 시간x모델속도인지 (2026-08-06 4차 주행 교훈).

    동편 왜곡 GPS 에서는 좌표 이동량 적산이 실물과 어긋난다 — 차가 제자리(좌표 동결)여도
    시간이 차면 개시되고, 시간이 안 찼으면 좌표가 아무리 움직여도 개시되지 않아야 한다.
    """

    def test_offset_consumes_model_time_not_gps_distance(self):
        raw = {"turn_table": {"defaults": {}, "turns": {"11->22": {
            "trigger": {"class": "crosswalk"},
            # 정확히 1.0초분 — 상수를 박지 않는다 (turn_throttle 이 바뀌면 같이 움직인다)
            "start_offset_m": lane_follower_mod.OFFSET_SPEED_MPS}}}}
        source = StaticMarkSource()
        marks = MarkAdapter(source, freshness_max_s=1e9)
        f = LaneFollower(CONTROL_CFG, _table(raw), MAX_WHEEL, MAX_SPEED, WHEELBASE,
                         marks=marks)
        route = lane_route.from_lane_numbers(_map(), [11, 22])
        f.set_route(route)
        source.payload = _payload([_det("crosswalk", 0.9, 235.0)], ts=time.time())
        marks.observe()
        end = route.legs[0].points[-1]
        x, y = end[0] - 0.05, end[1]          # ARMED 창 안, 좌표는 이후 동결
        with mock.patch("control.lane_follower.time") as mt:
            clock = [100.0]
            mt.monotonic = lambda: clock[0]
            f.compute(x, y, 0.0)              # ARMED + 발화 (같은 tick)
            self.assertNotEqual(f.state, TURNING, "발화 즉시 개시되면 안 된다 (오프셋)")
            for _ in range(9):                # 0.9초분 — 아직 부족
                clock[0] += 0.1
                f.compute(x, y, 0.0)
            self.assertNotEqual(f.state, TURNING, "0.9초분에 조기 개시")
            for _ in range(2):                # 누적 1.1초분 > 오프셋
                clock[0] += 0.1               # (0.2 한 번에 가면 GAP 상한에 걸려 미적산 — 의도된 동작)
                f.compute(x, y, 0.0)
            self.assertEqual(f.state, TURNING, "시간이 찼는데 개시 안 됨")

    def test_stationary_gap_does_not_consume_offset(self):
        """유실 정지 등으로 compute 가 끊긴 시간은 적산 금지 (GAP 상한)."""
        raw = {"turn_table": {"defaults": {}, "turns": {"11->22": {
            "trigger": {"class": "crosswalk"}, "start_offset_m": 0.094}}}}
        source = StaticMarkSource()
        marks = MarkAdapter(source, freshness_max_s=1e9)
        f = LaneFollower(CONTROL_CFG, _table(raw), MAX_WHEEL, MAX_SPEED, WHEELBASE,
                         marks=marks)
        route = lane_route.from_lane_numbers(_map(), [11, 22])
        f.set_route(route)
        source.payload = _payload([_det("crosswalk", 0.9, 235.0)], ts=time.time())
        marks.observe()
        end = route.legs[0].points[-1]
        x, y = end[0] - 0.05, end[1]
        with mock.patch("control.lane_follower.time") as mt:
            clock = [100.0]
            mt.monotonic = lambda: clock[0]
            f.compute(x, y, 0.0)              # 발화
            clock[0] += 5.0                   # 5초 공백 (유실 정지 재현) — 적산 금지
            f.compute(x, y, 0.0)
            self.assertNotEqual(f.state, TURNING, "정지 공백이 오프셋을 소모했다")


class TestDrivingWorkerWithLaneFollower(unittest.TestCase):
    """실제 배선 그대로 — 매칭 → 스냅 → 플래너 → LaneFollower → 도착 통지.

    앞의 폐루프는 follower 단독이라 '플래너가 준 Route 를 LaneFollower 가 받는' 경로를
    타지 않는다. 보드에 올라가는 건 이쪽이므로 여기까지 봐야 검증이 끝난다.
    """

    def _stack(self):
        lane_map = _map()
        store = StateStore()
        supervisor = SafetySupervisor(store, MockMotorDriver(),
                                      MockSteeringDriver(STEERING_CFG), MAX_WHEEL)
        policy = _PolicyStub(store, supervisor)
        # 설계 기하 표 — 이 폐루프의 차량 모델은 명령을 100% 순종하는 이상 차량이라,
        # 시연 하드웨어(실효 조향 ≈46%)에 캘리브레이션된 배포 표(개루프 호장 0.856 등)를
        # 주면 과회전으로 완주가 안 된다 (§0-57 규명: 양립 불가 — 시뮬은 알고리즘 검증,
        # 배포값 검증은 test_seg_heading.TestDeployedConfig 말뚝 소관)
        follower = LaneFollower(CONTROL_CFG,
                                _table({"turn_table": {"defaults": {}, "turns": {}}}),
                                MAX_WHEEL, MAX_SPEED, WHEELBASE)
        worker = DrivingWorker(
            threading.Event(), store, supervisor, policy, lane_map,
            MapMatcher(lane_map, MATCH_CFG), DestinationResolver(lane_map, 0.30),
            RoutePlanner(lane_map), ArrivalChecker(0.15, ARRIVAL_CFG), follower,
            50, CONTROL_CFG["destination_slowdown_distance_m"])
        return store, policy, follower, worker

    def _simulate(self, start, dest, max_sim_s=200.0):
        store, policy, follower, worker = self._stack()
        dt, t = 0.02, 1000.0
        x, y, heading = start
        store.update_pose(LocalizationPose(x, y, heading % 360.0, t, t))
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(dest[0], dest[1], ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)
        seen, lanes_seen = set(), []
        for _ in range(int(max_sim_s / dt)):
            worker.tick()
            seen.add(store.snapshot().driving_state)
            if follower.has_route and follower.current_lane_num not in lanes_seen:
                lanes_seen.append(follower.current_lane_num)
            if policy.notified:
                break
            cmd = store.snapshot().control_command
            v = cmd.throttle * MAX_SPEED
            heading -= math.degrees(v / WHEELBASE
                                    * math.tan(math.radians(cmd.steering_wheel_deg)) * dt)
            x += v * math.cos(math.radians(heading)) * dt
            y += v * math.sin(math.radians(heading)) * dt
            t += dt
            store.update_pose(LocalizationPose(x, y, heading % 360.0, t, t))
            store.set_estimated_speed(v)
        return policy, (x, y), seen, lanes_seen

    def test_senior_center_to_township_office_completes(self):
        """경로당(11번) → 면사무소(14번) — 추천 시연 경로. 위험 우회전 2회."""
        policy, final, seen, lanes = self._simulate(start=(1.20, 1.12, 0.0),
                                                    dest=(1.20, 0.39))
        self.assertEqual(policy.notified, 1, f"도착 통지 없음 (거쳐간 차선 {lanes})")
        self.assertLess(math.hypot(final[0] - 1.20, final[1] - 0.39), 0.20)
        self.assertIn(DrivingState.FOLLOWING_ROUTE, seen)
        self.assertIn(DrivingState.APPROACHING_DESTINATION, seen)
        self.assertEqual(lanes[0], 11)
        self.assertEqual(lanes[-1], 14)

    def test_zero_length_mission_stops_in_place(self):
        """제자리 목적지 — 길이 0 경로에서 죽지 않고 정지해야 한다 (기존 회귀 항목)."""
        store, policy, follower, worker = self._stack()
        t = 1000.0
        store.update_pose(LocalizationPose(1.20, 1.12, 0.0, t, t))
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(1.20, 1.12, ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)
        for _ in range(50):
            worker.tick()
        self.assertEqual(store.snapshot().control_command.throttle, 0.0)


if __name__ == "__main__":
    unittest.main()
