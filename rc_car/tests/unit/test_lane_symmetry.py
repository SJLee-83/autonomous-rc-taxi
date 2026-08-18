"""맵 대칭 검증 — 튜닝 전이의 근거를 상시 지킨다 (2026-08-06 사용자 제시).

사용자 설계: *"각각 짝을 지어 놓은 차선들은 직진 및 회전하는 도로의 형태가 동일하기 때문에
하나의 차선을 튜닝하면 그에 대칭되는 차선도 이를 활용해서 주행할 수 있다"*

이 전제가 깨지면 **튜닝값이 조용히 엉뚱한 회전에 적용된다.** 실차에서 이건 사고다.
맵 생성기를 고치거나 맵을 재생성했을 때 대칭이 유지되는지 여기서 강제한다.

검증 대상
    ① 인접 리스트가 사용자 표와 정확히 일치 (32노드 / 커넥터 64개)
    ② 16쌍이 180° 점대칭 (x, y) → (5 − x, 3 − y)
    ③ 대응하는 회전이 합동 (maneuver · radius_m · length_m)
    ④ 점대칭은 좌우를 뒤집지 않는다 (우회전의 대칭은 우회전)
    ⑤ TurnTable 이 대칭 항목을 실제로 상속한다
"""
import unittest
from pathlib import Path

from mapping.lane_map import load_lane_map
from navigation import lane_route
from navigation.lane_route import (ID2NUM, NUM2ID, SYMMETRIC_NUM, SYMMETRIC_PAIRS,
                                   symmetric_key, symmetric_num, symmetric_point,
                                   tuning_class)
from navigation.turn_table import TurnTable

MAP_PATH = Path(__file__).resolve().parents[3] / "map" / "main_track_map.yaml"

# 사용자가 제시한 인접 리스트 (2026-08-06 input.md) — 맵의 독립 대조군이다.
USER_ADJACENCY = {
    1: [2, 21, 23], 2: [17], 3: [4, 23, 21], 4: [29], 5: [6, 26, 28], 6: [20],
    7: [8, 22, 24], 8: [18], 17: [18, 11, 9], 18: [15], 19: [20, 9, 11], 20: [3],
    21: [22, 8, 6], 23: [24, 12, 10], 26: [2], 28: [4],
    15: [16, 27, 25], 16: [31], 13: [14, 25, 27], 14: [19], 11: [12, 22, 24], 12: [30],
    9: [10, 28, 26], 10: [32], 31: [32, 5, 7], 32: [1], 29: [30, 7, 5], 30: [13],
    27: [28, 10, 12], 25: [26, 6, 8], 24: [16], 22: [14],
}


def _map():
    if not hasattr(_map, "cached"):
        _map.cached = load_lane_map(MAP_PATH)
    return _map.cached


def _successors(num):
    lane = _map().lanes[NUM2ID[num]]
    return sorted(ID2NUM[_map().connectors[c].successor] for c in lane.successors)


def _turn(a, b):
    """a번 → b번 커넥터 (maneuver, radius_m, length_m). 없으면 None."""
    for cid in _map().lanes[NUM2ID[a]].successors:
        c = _map().connectors[cid]
        if ID2NUM[c.successor] == b:
            return c.maneuver, c.radius_m, round(c.length_m, 6)
    return None


class TestAdjacencyMatchesUserModel(unittest.TestCase):
    """① 사용자 인접 리스트 = 맵 그래프."""

    def test_all_nodes_present(self):
        self.assertEqual(sorted(USER_ADJACENCY), sorted(NUM2ID))

    def test_successors_match(self):
        for num in sorted(USER_ADJACENCY):
            with self.subTest(lane=num):
                self.assertEqual(sorted(USER_ADJACENCY[num]), _successors(num))

    def test_connector_count(self):
        total = sum(len(v) for v in USER_ADJACENCY.values())
        self.assertEqual(total, len(_map().connectors))
        self.assertEqual(total, 64)


class TestPointSymmetry(unittest.TestCase):
    """② 16쌍이 180° 점대칭."""

    def test_pairs_cover_all_lanes(self):
        covered = {n for pair in SYMMETRIC_PAIRS for n in pair}
        self.assertEqual(covered, set(NUM2ID), "32차선이 전부 대칭 쌍에 들어가야 한다")
        self.assertEqual(len(SYMMETRIC_PAIRS), 16)

    def test_mapping_is_an_involution(self):
        for n in NUM2ID:
            self.assertEqual(symmetric_num(symmetric_num(n)), n)
            self.assertNotEqual(symmetric_num(n), n, "자기 자신과 짝인 차선은 없어야 한다")

    def test_centerlines_are_point_symmetric(self):
        lanes = _map().lanes
        for a, b in SYMMETRIC_PAIRS:
            with self.subTest(pair=(a, b)):
                ca = lanes[NUM2ID[a]].centerline
                cb = lanes[NUM2ID[b]].centerline
                self.assertEqual(len(ca), len(cb))
                for pa, pb in zip(ca, cb):
                    want = symmetric_point(pa)
                    self.assertAlmostEqual(want[0], pb[0], places=6)
                    self.assertAlmostEqual(want[1], pb[1], places=6)

    def test_lane_lengths_match(self):
        lanes = _map().lanes
        for a, b in SYMMETRIC_PAIRS:
            with self.subTest(pair=(a, b)):
                self.assertAlmostEqual(lanes[NUM2ID[a]].length_m,
                                       lanes[NUM2ID[b]].length_m, places=6)


class TestTurnCongruence(unittest.TestCase):
    """③④ 대응 회전이 합동이고 좌우가 보존된다."""

    def test_every_turn_has_a_congruent_mirror(self):
        checked = 0
        for a in sorted(NUM2ID):
            for b in _successors(a):
                spec = _turn(a, b)
                sa, sb = symmetric_num(a), symmetric_num(b)
                with self.subTest(turn=f"{a}->{b}"):
                    mirror = _turn(sa, sb)
                    self.assertIsNotNone(
                        mirror, f"{a}→{b} 의 대칭 {sa}→{sb} 커넥터가 맵에 없다")
                    self.assertEqual(spec, mirror,
                                     f"{a}→{b} {spec} 와 {sa}→{sb} {mirror} 가 합동이 아니다 "
                                     "— 튜닝 전이 전제가 깨졌다")
                checked += 1
        self.assertEqual(checked, 64)

    def test_handedness_is_preserved(self):
        """점대칭(180° 회전)은 좌우를 뒤집지 않는다 — 거울 대칭이면 전이가 불가능하다."""
        for a in sorted(NUM2ID):
            for b in _successors(a):
                man = _turn(a, b)[0]
                mirror_man = _turn(symmetric_num(a), symmetric_num(b))[0]
                with self.subTest(turn=f"{a}->{b}"):
                    self.assertEqual(man, mirror_man)

    def test_tuning_classes_are_half(self):
        turns = {f"{a}->{b}" for a in NUM2ID for b in _successors(a)
                 if _turn(a, b)[0] in ("left", "right")}
        classes = {tuning_class(k) for k in turns}
        self.assertEqual(len(turns), 48)
        self.assertEqual(len(classes), 24, "회전 48개 → 튜닝 클래스 24종이어야 한다")

    def test_symmetric_key_round_trip(self):
        self.assertEqual(symmetric_key("11->22"), "5->28")
        self.assertEqual(symmetric_key("5->28"), "11->22")
        self.assertEqual(symmetric_key("22->14"), "28->4")
        self.assertIsNone(symmetric_key("ix_some_connector"))


class TestTurnTableInheritsSymmetry(unittest.TestCase):
    """⑤ 한쪽을 튜닝하면 대칭 상대가 실제로 값을 물려받는다."""

    def setUp(self):
        self.raw = {"turn_table": {
            "defaults": {"arm_distance_m": 0.6, "exit_align_deg": 12.0,
                         "trigger": {"class": None, "near_row_frac": 0.8}},
            "turns": {"11->22": {"exit_align_deg": 17.0,
                                 "trigger": {"class": "crosswalk", "near_row_frac": 0.61}}}}}
        self.table = TurnTable(self.raw, 0.14, 30.0)
        self.lane_map = _map()

    def _resolve(self, a, b):
        lr = lane_route.from_lane_numbers(self.lane_map, [a, b])
        return self.table.resolve(lr.turns[0])

    def test_direct_entry_is_not_marked_inherited(self):
        spec = self._resolve(11, 22)
        self.assertTrue(spec.from_table)
        self.assertFalse(spec.inherited)
        self.assertEqual(spec.source_key, "11->22")

    def test_mirror_turn_inherits_the_tuning(self):
        spec = self._resolve(5, 28)                 # 11->22 의 대칭
        self.assertTrue(spec.from_table)
        self.assertTrue(spec.inherited, "대칭 상속이 안 됐다 — 튜닝 절반 재사용이 무효")
        self.assertEqual(spec.source_key, "11->22")
        self.assertEqual(spec.trigger.cls, "crosswalk")
        self.assertAlmostEqual(spec.trigger.near_row_frac, 0.61)
        self.assertAlmostEqual(spec.exit_align_deg, 17.0)

    def test_inherited_wheel_angle_keeps_own_geometry(self):
        """조향각은 자기 커넥터 반경에서 나온다 — 합동이라 결과도 같아야 한다."""
        a, b = self._resolve(11, 22), self._resolve(5, 28)
        self.assertAlmostEqual(a.wheel_deg, b.wheel_deg, places=6)

    def test_unrelated_turn_still_falls_back_to_defaults(self):
        spec = self._resolve(7, 22)
        self.assertFalse(spec.from_table)
        self.assertFalse(spec.inherited)
        self.assertFalse(spec.trigger.uses_vision)


if __name__ == "__main__":
    unittest.main()
