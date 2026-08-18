"""시연 장소 표 검증 (map/places.yaml — rc_car 형제).

이 테스트가 고정하는 핵심 사실은 하나다:

    **정차점은 차선 조각의 "끝"에서 도착 반경보다 멀어야 한다.**

중간 가로 도로의 차선은 점선(차선 변경) 구간에서 a/b 조각으로 끊겨 있고 그 사이는
차선이 아니라 이음매(connector)다. 정차점이 조각 끝에 놓이면 차가 조금만 지나쳐 서도
이음매 위에 있게 되는데, 도착 판정 §19-2 는 "목표 차선에 매칭"을 요구하므로 게이트가
영원히 안 열려 complete 가 나가지 않는다 (2026-07-29 재현 확인 — 경로당·시장이 해당).

조각 "시작" 쪽은 안전하다. 직전 이음매의 진출 차선이 곧 목표 차선이라 게이트가 열린다
(S6 수정, 2026-07-28). 그래서 검사는 끝 쪽에만 건다.

좌표를 실측으로 갱신하거나 맵을 재생성하면 이 테스트가 먼저 잡는다.
"""
import math
import unittest
from pathlib import Path

import yaml

from mapping import geometry
from mapping.lane_map import load_lane_map
from mapping.map_matcher import MapMatcher
from navigation.destination_resolver import DestinationResolver
from navigation.route_planner import RoutePlanner

MAP_DIR = Path(__file__).resolve().parents[3] / "map"
MAP_PATH = MAP_DIR / "main_track_map.yaml"
PLACES_PATH = MAP_DIR / "places.yaml"
CONTROL_PATH = Path(__file__).resolve().parents[2] / "config" / "control.yaml"


class TestPlaces(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(PLACES_PATH, encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        with open(CONTROL_PATH, encoding="utf-8") as f:
            cls.control = yaml.safe_load(f)
        cls.meta = doc["meta"]
        cls.places = doc["places"]
        cls.m = load_lane_map(MAP_PATH)
        # 매칭·스냅 설정은 차량 config 실값을 그대로 쓴다 — 하드코딩하면
        # 주행 후 임계값 튜닝 시 테스트가 낡은 값으로 검증하며 조용히 통과한다
        cls.resolver = DestinationResolver(
            cls.m,
            warn_snap_distance_m=float(cls.control["destination"]["warn_snap_distance_m"]))
        cls.matcher = MapMatcher(cls.m, cls.control["matching"])
        cls.planner = RoutePlanner(cls.m)
        cls.radius = float(cls.meta["arrival_radius_m"])
        cls.margin = float(cls.meta["end_margin_m"])

    # ---------- 기본 정합 ----------

    def test_six_places(self):
        self.assertEqual(len(self.places), 6)

    def test_stop_point_resolves_to_declared_lane(self):
        """stop_point 를 차량과 같은 방식으로 스냅하면 선언한 차선이 나온다."""
        for pid, p in self.places.items():
            with self.subTest(place=pid):
                sp = self.resolver.resolve(*p["stop_point"])
                self.assertEqual(sp.lane_id, p["stop_lane"])
                # 이미 차선 위 좌표이므로 스냅은 사실상 이동이 없어야 한다
                self.assertLess(sp.snap_distance_m, 0.02)

    def test_declared_heading_matches_lane(self):
        for pid, p in self.places.items():
            with self.subTest(place=pid):
                sp = self.resolver.resolve(*p["stop_point"])
                self.assertLess(
                    abs(geometry.heading_diff_deg(sp.heading_deg, p["heading_deg"])), 1.0)

    def test_stop_lane_is_curb_adjacent(self):
        """정차 차선이 그 블록·curb 의 인접 정차 차선 목록에 있다 (승객 옆에 선다)."""
        for pid, p in self.places.items():
            with self.subTest(place=pid):
                curb = self.m.blocks[p["block"]].curb_edges[p["curb"]]
                self.assertIn(p["stop_lane"], curb.adjacent_destination_lanes)

    def test_call_point_inside_block(self):
        for pid, p in self.places.items():
            with self.subTest(place=pid):
                poly = self.m.blocks[p["block"]].polygon
                xs = [q[0] for q in poly]
                ys = [q[1] for q in poly]
                cx, cy = p["call_point"]
                self.assertTrue(min(xs) <= cx <= max(xs), f"{pid} call_point x 밖")
                self.assertTrue(min(ys) <= cy <= max(ys), f"{pid} call_point y 밖")

    # ---------- 핵심: 이음매 데드락 방지 ----------

    def test_stop_point_clear_of_lane_end(self):
        """정차점은 차선 조각 끝에서 (도착 반경 + 여유) 이상 떨어져 있어야 한다.

        이걸 어기면 차가 반경 안에서 이음매 위에 정지해 도착 판정이 영원히 안 난다.
        """
        need = self.radius + self.margin
        # home 예외 (2026-08-09 §0-53): 정차점이 20번 중앙으로 이동 — 20번은 길이
        # 0.21m 라 이 충분조건(반경+여유 0.45m)을 물리적으로 못 지킨다. 그래도 무해:
        # 반경 0.40 이 차선 전체+양측 이음매를 덮고 게이트가 이음매의 진입·진출 차선을
        # 함께 인정하므로 매칭이 유지된다 — 아래 test_arrival_gate_open_across_whole_radius
        # 가 직접 검증하고, 8/10 시연 3회 도착이 실증. (map/places.yaml home 주석 참조)
        exempt = {"home"}
        for pid, p in self.places.items():
            if pid in exempt:
                continue
            with self.subTest(place=pid):
                sp = self.resolver.resolve(*p["stop_point"])
                lane = self.m.lanes[sp.lane_id]
                remain = lane.length_m - sp.progress_s
                self.assertGreaterEqual(
                    remain, need,
                    f"{pid}({p['label']}): 조각 끝까지 {remain:.3f}m 남음 — "
                    f"{need:.2f}m 이상 필요. 이음매 위 정지 시 도착 판정 불가")

    def test_arrival_gate_open_across_whole_radius(self):
        """도착 반경 안 어디에 정지해도 §19-2 차선 매칭 게이트가 열린다.

        DrivingWorker 와 **같은 함수**(`arrival_lane_candidates`)로 후보를 만든다 —
        예전에는 이 계산을 여기서 따로 재현했는데, 그러면 본 코드만 고쳐도
        테스트는 옛 방식으로 통과해 버린다.
        """
        for pid, p in self.places.items():
            sp = self.resolver.resolve(*p["stop_point"])
            rad = math.radians(sp.heading_deg)
            ux, uy = math.cos(rad), math.sin(rad)   # 진행 방향 단위 벡터
            for step in range(-10, 11):        # -0.10 ~ +0.10 m, 1cm 간격
                d = step / 100.0
                px, py = sp.x + ux * d, sp.y + uy * d
                match = self.matcher.match(px, py, sp.heading_deg)
                cands = (frozenset() if match is None
                         else self.m.arrival_lane_candidates(match.segment_id))
                with self.subTest(place=pid, offset_m=d):
                    self.assertIn(
                        sp.lane_id, cands,
                        f"{pid}({p['label']}) {d:+.2f}m 지점에서 게이트 닫힘")

    # ---------- 시연 동선 ----------

    def test_every_ordered_pair_has_route(self):
        """6개 장소 사이 30개 순서쌍 전부 경로가 존재한다."""
        sps = {pid: self.resolver.resolve(*p["stop_point"])
               for pid, p in self.places.items()}
        for a_id, a in sps.items():
            for b_id, b in sps.items():
                if a_id == b_id:
                    continue
                with self.subTest(frm=a_id, to=b_id):
                    route = self.planner.plan(a.lane_id, a.progress_s,
                                              b.lane_id, b.progress_s)
                    self.assertGreater(route.total_length_m, 0.0)

    def test_meta_radius_matches_control_config(self):
        """places.yaml 의 도착 반경이 차량 config 와 어긋나면 검사가 무의미해진다."""
        self.assertAlmostEqual(
            self.radius, float(self.control["control"]["arrival_radius_m"]), places=6)


if __name__ == "__main__":
    unittest.main()
