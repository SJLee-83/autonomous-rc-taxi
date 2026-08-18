"""D2 테스트 — geometry·MapMatcher(§12)·DestinationResolver(§13)·RoutePlanner(§14).

실제 시연 맵(map/main_track_map.yaml (rc_car 형제))을 상대로 검증한다. 좌표 근거:
- top_inner_eb_w: y=2.61 동행, x [0.65, 1.86]
- mid_wb1_w: y=1.88 서행, x [1.86, 0.65] (block_nw 남측 curb — 2026-08-04 a/b 병합)
- vert_sb1_n: x=2.12 남행 스텁 (destination_allowed=false)
"""
import math
import unittest
from pathlib import Path

from mapping import geometry
from mapping.lane_map import load_lane_map
from mapping.map_matcher import MapMatcher
from navigation.destination_resolver import DestinationResolver
from navigation.route_planner import RouteError, RoutePlanner

MAP_PATH = Path(__file__).resolve().parents[3] / "map" / "main_track_map.yaml"

MATCH_CFG = {
    "distance_weight": 1.0,
    "heading_weight_per_deg": 0.01,
    "max_center_distance_m": 0.20,
    "max_heading_error_deg": 60.0,
}


def _map():
    if not hasattr(_map, "cached"):
        _map.cached = load_lane_map(MAP_PATH)
    return _map.cached


class TestGeometry(unittest.TestCase):
    PTS = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))  # 길이 2.0, ㄱ자

    def test_length_and_point_at(self):
        self.assertAlmostEqual(geometry.polyline_length(self.PTS), 2.0)
        self.assertEqual(geometry.point_at(self.PTS, 0.5), (0.5, 0.0))
        self.assertEqual(geometry.point_at(self.PTS, 1.5), (1.0, 0.5))
        self.assertEqual(geometry.point_at(self.PTS, 9.9), (1.0, 1.0))  # 클램프

    def test_project(self):
        s, dist, q = geometry.project_point(self.PTS, (0.5, 0.2))
        self.assertAlmostEqual(s, 0.5)
        self.assertAlmostEqual(dist, 0.2)
        self.assertEqual(q, (0.5, 0.0))
        s, dist, _ = geometry.project_point(self.PTS, (1.3, 0.8))  # 세로 구간 옆
        self.assertAlmostEqual(s, 1.8)
        self.assertAlmostEqual(dist, 0.3)

    def test_heading_at(self):
        self.assertAlmostEqual(geometry.heading_at(self.PTS, 0.5), 0.0)   # +x
        self.assertAlmostEqual(geometry.heading_at(self.PTS, 1.5), 90.0)  # +y

    def test_trim_preserves_interior_vertex(self):
        pts = geometry.trim(self.PTS, 0.5, 1.5)
        self.assertEqual(pts, ((0.5, 0.0), (1.0, 0.0), (1.0, 0.5)))
        self.assertAlmostEqual(geometry.polyline_length(pts), 1.0)

    def test_resample_spacing(self):
        pts = geometry.resample(self.PTS, 0.3)
        self.assertEqual(pts[0], self.PTS[0])
        self.assertEqual(pts[-1], self.PTS[-1])
        for a, b in zip(pts, pts[1:]):
            self.assertLessEqual(math.hypot(b[0] - a[0], b[1] - a[1]), 0.3 + 1e-9)

    def test_heading_diff_circular(self):
        self.assertAlmostEqual(geometry.heading_diff_deg(350.0, 10.0), -20.0)
        self.assertAlmostEqual(geometry.heading_diff_deg(10.0, 350.0), 20.0)


class TestMapMatcher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.matcher = MapMatcher(_map(), MATCH_CFG)

    def test_on_lane_exact(self):
        m = self.matcher.match(1.2, 2.61, 0.0)  # top_inner_eb_w 위, 동향
        self.assertIsNotNone(m)
        self.assertEqual(m.segment_id, "top_inner_eb_w")
        self.assertAlmostEqual(m.distance_m, 0.0, places=6)

    def test_heading_separates_opposite_ring_lanes(self):
        # 상단 링 두 차선(y 2.61 동행 / 2.86 서행) 사이 지점 — heading이 결정한다
        east = self.matcher.match(1.2, 2.735, 0.0)
        west = self.matcher.match(1.2, 2.735, 180.0)
        self.assertEqual(east.segment_id, "top_inner_eb_w")
        self.assertEqual(west.segment_id, "top_outer_wb_w")

    def test_reverse_heading_rejected(self):
        self.assertIsNone(self.matcher.match(1.2, 2.61, 180.0))  # 동행 차선 위 역방향

    def test_block_interior_rejected(self):
        self.assertIsNone(self.matcher.match(1.2, 2.2, 0.0))  # block_nw 내부

    def test_connector_matchable(self):
        # 사거리 한복판 (직진 커넥터 위) — §12.3 "명확한 connector" 허용
        m = self.matcher.match(2.12, 1.5, 270.0)  # ix_sb1_straight 경로상, 남향
        self.assertIsNotNone(m)
        self.assertEqual(m.kind, "connector")

    def test_off_road_rejected(self):
        self.assertIsNone(self.matcher.match(2.5, 2.99, 90.0))  # 외곽 테두리 위, 북향


class TestDestinationResolver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.resolver = DestinationResolver(_map(), warn_snap_distance_m=0.30)

    def test_snaps_block_curb_to_adjacent_lane(self):
        # block_nw 남측 curb 중앙 (1.2, 2.01) → mid_wb1_w 계열 (y 1.88)
        stop = self.resolver.resolve(1.2, 2.01)
        self.assertEqual(stop.lane_id, "mid_wb1_w")   # 2026-08-04 병합 — 이음매 포함 정차 가능
        self.assertAlmostEqual(stop.y, 1.88)
        self.assertAlmostEqual(stop.heading_deg, 180.0)  # 서행 차선

    def test_never_snaps_to_stub_or_connector(self):
        # 사거리 북측 스텁(vert_sb1_n, dest 금지) 옆 좌표 → 다른 허용 차선으로
        stop = self.resolver.resolve(2.12, 2.25)
        self.assertTrue(_map().lanes[stop.lane_id].destination_allowed)
        self.assertNotEqual(stop.lane_id, "vert_sb1_n")

    def test_exact_on_lane(self):
        stop = self.resolver.resolve(4.86, 0.7)  # right_outer_nb_s 위
        self.assertEqual(stop.lane_id, "right_outer_nb_s")
        self.assertAlmostEqual(stop.snap_distance_m, 0.0, places=6)
        self.assertAlmostEqual(stop.heading_deg, 90.0)


class TestRoutePlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = _map()
        cls.planner = RoutePlanner(cls.m)

    def test_same_lane_forward(self):
        r = self.planner.plan("top_inner_eb_w", 0.2, "top_inner_eb_w", 0.9)
        self.assertEqual(len(r.segments), 1)
        self.assertAlmostEqual(r.total_length_m, 0.7)
        self.assertEqual(r.segments[0].maneuver, "follow")

    def test_same_lane_behind_loops_around(self):
        # 목적지가 뒤 → 후진 없이 한 바퀴 (§3.2)
        r = self.planner.plan("top_inner_eb_w", 0.9, "top_inner_eb_w", 0.2)
        self.assertGreater(len(r.segments), 3)
        self.assertEqual(r.segments[0].segment_id, "top_inner_eb_w")
        self.assertEqual(r.segments[-1].segment_id, "top_inner_eb_w")
        self.assertGreater(r.total_length_m, 1.0)

    def test_route_continuity_and_length(self):
        r = self.planner.plan("top_inner_eb_w", 0.1, "bot_inner_wb_e", 0.5)
        for a, b in zip(r.segments, r.segments[1:]):
            pa, pb = a.centerline[-1], b.centerline[0]
            self.assertAlmostEqual(pa[0], pb[0], places=6)
            self.assertAlmostEqual(pa[1], pb[1], places=6)
        recomputed = sum(geometry.polyline_length(s.centerline) for s in r.segments)
        self.assertAlmostEqual(r.total_length_m, recomputed, places=3)

    def test_cross_circuit_route_uses_bridge(self):
        # outer 출발 → inner 도착: 다리(차선 변경 또는 _far 회전)를 반드시 지난다
        r = self.planner.plan("top_outer_wb_e", 0.1, "top_inner_eb_w", 0.5)
        bridges = [s for s in r.segments
                   if s.maneuver == "lane_change" or s.segment_id.endswith("_far")]
        self.assertTrue(bridges)

    def test_far_turn_essential_for_vertical_second_lane(self):
        # 세로 도로는 차선 변경 구간이 없다 → 상단 도로에서 세로 2차선(vert_sb2_n)으로
        # 가려면 _far 회전 진입이 압도적으로 짧다 (없으면 외곽을 크게 돌아야 함)
        r = self.planner.plan("top_inner_eb_w", 0.1, "vert_sb2_n", 0.1)
        self.assertIn("tee_n_eb_right_far", [s.segment_id for s in r.segments])
        self.assertLess(r.total_length_m, 3.0)

    def test_all_pairs_plannable(self):
        # 강연결 실증 — 모든 (시작, 목적) 차선 쌍에 경로가 존재한다
        lanes = list(self.m.lanes)
        for a in lanes:
            for b in lanes:
                if a == b:
                    continue
                r = self.planner.plan(a, 0.01, b, 0.01)
                self.assertGreater(r.total_length_m, 0.0, f"{a}->{b}")

    def test_waypoints_spacing(self):
        r = self.planner.plan("top_inner_eb_w", 0.1, "right_inner_sb_n", 0.1)
        wps = r.waypoints(0.02)
        self.assertGreater(len(wps), 20)
        for a, b in zip(wps, wps[1:]):
            self.assertLessEqual(math.hypot(b[0] - a[0], b[1] - a[1]), 0.02 + 1e-6)

    def test_unknown_lane_raises(self):
        with self.assertRaises(RouteError):
            self.planner.plan("nope", 0.0, "top_inner_eb_w", 0.0)


if __name__ == "__main__":
    unittest.main()
