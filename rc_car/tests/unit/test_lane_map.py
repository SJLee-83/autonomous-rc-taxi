"""차선 그래프 맵 테스트 (D1).

두 축:
1) 실제 산출물(map/main_track_map.yaml (rc_car 형제))이 규약·명세 제약을 전부 만족하는가
2) 로더가 깨진 맵을 확실히 거부하는가

구조적 사실을 테스트로 고정한다: 노면 화살표 규칙만으로는 두 순환계(inner/outer)가
완전 분리되고, 가로 도로 점선 구간의 차선 변경(lane_change) 커넥터 8개(2026-07-28
결정)만이 둘을 잇는다. 맵을 수정해 이 구조가 바뀌면 의도 확인 없이는 통과하지 못한다.
"""
import copy
import math
import unittest
from pathlib import Path

import yaml

from core.exceptions import MapError
from mapping.lane_map import LaneMap, load_lane_map

MAP_PATH = Path(__file__).resolve().parents[3] / "map" / "main_track_map.yaml"
MIN_TURN_RADIUS = 0.14 / math.tan(math.radians(30.0))  # wheelbase / tan(최대 조향각)


def _load_raw() -> dict:
    with open(MAP_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)


class TestCanonicalMap(unittest.TestCase):
    """실제 시연 맵 파일 검증 — 이 테스트가 곧 D1의 DoD."""

    @classmethod
    def setUpClass(cls):
        cls.m = load_lane_map(MAP_PATH)

    def test_inventory(self):
        self.assertEqual(len(self.m.lanes), 32)       # 2026-08-04 가로 도로 a/b 병합 (40→32)
        # 회전·직진 48 + 회전 진입 차선 선택(_far) 16 — 2026-08-04 변경 구간 16 폐지
        self.assertEqual(len(self.m.connectors), 64)
        self.assertEqual(len(self.m.blocks), 4)
        self.assertEqual(len(self.m.stop_line_zones), 24)

    def test_fully_connected_via_far_entries(self):
        # 전 차선 쌍 도달 가능 — 회전 진입 차선 선택(_far)이 두 순환계를 잇는다
        # (2026-08-04: 점선 구간 lane_change 폐지 후에도 강연결 유지 검증)
        all_lanes = set(self.m.lanes)
        for seed in ("top_inner_eb_w", "top_outer_wb_e", "vert_sb1_n", "mid_eb1_w"):
            self.assertEqual(self.m.reachable_lanes(seed), all_lanes, seed)

    def test_no_lane_change_connectors_remain(self):
        # 2026-08-04 사용자 결정: 짧은 이음매의 대각 기동 폐지 — 재유입 방지 말뚝
        changes = [cid for cid, c in self.m.connectors.items()
                   if c.maneuver == "lane_change"]
        self.assertEqual(changes, [])
        # follow(이음매 유지선)도 병합으로 함께 소멸했다
        follows = [c for c in self.m.connectors.values() if c.maneuver == "follow"]
        self.assertEqual(follows, [])

    def test_far_turns_offer_second_entry_lane(self):
        # 2차로 도로로 회전할 때 가까운/먼 차선 두 진입 경로가 모두 있다 (사용자 결정)
        fars = {cid: c for cid, c in self.m.connectors.items() if cid.endswith("_far")}
        self.assertEqual(len(fars), 16)  # 사거리 8 + T자 8

        def fixed_coord(lane):  # 축 정렬 차선의 고정축 좌표
            (x0, y0), (x1, y1) = lane.centerline[0], lane.centerline[-1]
            return x0 if x0 == x1 else y0

        from_lane = {cid: lid for lid, lane in self.m.lanes.items()
                     for cid in lane.successors}
        for cid, far in fars.items():
            near = self.m.connectors[cid[: -len("_far")]]  # 짝이 되는 기본 회전
            self.assertEqual(far.maneuver, near.maneuver, cid)
            self.assertEqual(from_lane[cid], from_lane[near.id], cid)  # 같은 차선에서 출발
            # 진입 차선은 서로 인접한 평행 차선 (간격 = 차로 폭 0.25m)
            near_lane, far_lane = self.m.lanes[near.successor], self.m.lanes[far.successor]
            self.assertEqual(near_lane.heading_hint_deg, far_lane.heading_hint_deg, cid)
            self.assertAlmostEqual(abs(fixed_coord(near_lane) - fixed_coord(far_lane)),
                                   0.25, places=6, msg=cid)
            # 먼 차선 진입은 서킷을 넘는 다리다
            self.assertNotEqual(self.m.lanes[from_lane[cid]].circuit,
                                far_lane.circuit, cid)

    def test_circuit_labels_are_16_16(self):
        # 라벨은 _far 제외 시의 분리 구조를 기록한다 (2026-08-04 병합 후 20/20 → 16/16)
        self.assertEqual(len(self.m.lanes_in_circuit("inner")), 16)
        self.assertEqual(len(self.m.lanes_in_circuit("outer")), 16)

    def test_block_curbs_reference_stoppable_lanes(self):
        for block in self.m.blocks.values():
            served = [lid for curb in block.curb_edges.values()
                      for lid in curb.adjacent_destination_lanes]
            self.assertTrue(served, f"{block.id}: 정차 가능 curb 없음")
            for lid in served:
                self.assertTrue(self.m.lanes[lid].destination_allowed, lid)

    def test_turn_radius_at_least_vehicle_minimum(self):
        self.m.validate_turn_radius(MIN_TURN_RADIUS)  # 미달이면 MapError

    def test_speed_limits_within_vehicle_max(self):
        for seg in list(self.m.lanes.values()) + list(self.m.connectors.values()):
            self.assertLessEqual(seg.speed_limit_mps, 0.25, seg.id)

    def test_known_geometry_spot_checks(self):
        # 픽셀 판독 대조점 — 생성기 상수가 바뀌면 여기서 잡힌다
        self.assertEqual(self.m.lanes["top_outer_wb_e"].centerline[0][1], 2.86)
        self.assertEqual(self.m.lanes["vert_sb1_n"].centerline[0][0], 2.12)
        self.assertEqual(self.m.lanes["mid_eb2_w"].centerline[0], (0.65, 1.12))
        # 교차로 사이 스텁은 정차 금지
        self.assertFalse(self.m.lanes["vert_sb1_n"].destination_allowed)
        self.assertLess(self.m.lanes["vert_sb1_n"].length_m, 0.3)

    def test_stop_zone_near_its_approach_lane(self):
        # 정지선은 접근 차선 연장선상(차선 폭 이내)에 있어야 한다
        for zone in self.m.stop_line_zones.values():
            lane = self.m.lanes[zone.approach_lane_id]
            (x0, y0), (x1, y1) = lane.centerline[0], lane.centerline[-1]
            px, py = zone.expected_center
            # 축 정렬 차선: 고정축 좌표가 일치하는지 확인
            if x0 == x1:
                self.assertAlmostEqual(px, x0, delta=0.13, msg=zone.id)
            else:
                self.assertAlmostEqual(py, y0, delta=0.13, msg=zone.id)


class TestLoaderRejectsBrokenMaps(unittest.TestCase):
    def setUp(self):
        self.raw = _load_raw()

    def test_missing_successor_connector(self):
        self.raw["lanes"]["top_inner_eb_w"]["successors"].append("no_such_conn")
        with self.assertRaises(MapError):
            LaneMap.from_dict(self.raw)

    def test_connector_to_unknown_lane(self):
        self.raw["connectors"]["ix_sb1_straight"]["successor"] = "ghost"
        with self.assertRaises(MapError):
            LaneMap.from_dict(self.raw)

    def test_dead_end_lane_rejected(self):
        self.raw["lanes"]["top_inner_eb_w"]["successors"] = []
        with self.assertRaises(MapError):
            LaneMap.from_dict(self.raw)

    def test_out_of_bounds_point(self):
        self.raw["lanes"]["top_inner_eb_w"]["centerline"][0] = [5.5, 2.61]
        with self.assertRaises(MapError):
            LaneMap.from_dict(self.raw)

    def test_discontinuity_detected(self):
        self.raw["connectors"]["corner_nw_inner"]["centerline"][0] = [0.39, 2.0]
        with self.assertRaises(MapError):
            LaneMap.from_dict(self.raw)

    def test_wrong_map_size_rejected(self):
        self.raw["map"]["width_m"] = 4.0
        with self.assertRaises(MapError):
            LaneMap.from_dict(self.raw)

    def test_removing_all_bridges_breaks_connectivity(self):
        # 다리(먼 차선 진입 _far — 2026-08-04부터 유일한 다리)를 전부 지우면 두 순환계로
        # 갈라진다 → 로더가 강연결 위반으로 거부.
        bridge_ids = [cid for cid, c in self.raw["connectors"].items()
                      if c["maneuver"] == "lane_change" or cid.endswith("_far")]
        self.assertEqual(len(bridge_ids), 16)  # _far 16 (lane_change는 폐지로 0)
        for cid in bridge_ids:
            del self.raw["connectors"][cid]
        for lane in self.raw["lanes"].values():
            lane["successors"] = [c for c in lane["successors"] if c not in bridge_ids]
        with self.assertRaises(MapError):
            LaneMap.from_dict(self.raw)

    def test_turn_radius_validation(self):
        m = LaneMap.from_dict(copy.deepcopy(self.raw))
        with self.assertRaises(MapError):
            m.validate_turn_radius(0.30)  # 최소 반경을 올리면 우회전(0.26)이 걸린다

    def test_heading_mismatch_rejected(self):
        lane = self.raw["lanes"]["top_inner_eb_w"]
        lane["heading_hint_deg"] = 180  # 실제 진행은 동쪽(0°)
        with self.assertRaises(MapError):
            LaneMap.from_dict(self.raw)


class TestGraphApi(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = load_lane_map(MAP_PATH)

    def test_bipartite_successors(self):
        # lane → connector, connector → lane
        for lid, lane in self.m.lanes.items():
            for cid in lane.successors:
                self.assertIn(cid, self.m.connectors, lid)
        for cid in self.m.connectors:
            (nxt,) = self.m.successors_of(cid)
            self.assertIn(nxt, self.m.lanes, cid)

    def test_known_route_exists_cross_circuit(self):
        # 대표 시나리오: 바깥 링(outer)에서 출발해도 블록 curb(inner 인접)에 도달
        reach = self.m.reachable_lanes("top_outer_wb_e")
        self.assertIn("mid_eb2_e", reach)     # block_se 북측 curb 차선
        self.assertIn("top_inner_eb_w", reach)

    def test_unknown_segment_raises(self):
        with self.assertRaises(MapError):
            self.m.successors_of("nope")


if __name__ == "__main__":
    unittest.main()
