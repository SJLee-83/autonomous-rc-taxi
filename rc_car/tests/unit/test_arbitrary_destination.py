"""임의 목적지 좌표에서도 도착 판정이 성립하는가 (§19-2, 2026-07-30).

고정 6곳은 `test_places.py` 가 지킨다. 이 파일이 지키는 것은 더 일반적인 사실이다:

    **정차 가능 차선 위 어느 좌표를 목적지로 받아도, 도착 반경 안 어디에 정지하든
    차선 매칭 게이트가 열려야 한다.**

관제(웹팀 백엔드)는 승객이 선 자리에 맞춰 정차점 x 를 연속으로 계산한다. 우리 맵은
점선 구간에서 차선을 a/b 조각으로 끊어 두었으므로, 관제가 보내는 좌표 중 상당수가
조각 끝이나 이음매 위로 떨어진다. 이때 게이트가 진출 차선만 인정하면 차가 반경 안에
멀쩡히 서 있어도 complete 가 영영 안 나간다 — 2026-07-30 재현 확인(워크로그 §0-21,
가운데 가로도로 4개 curb 에서 각 0.34m 구간이 교착).

해소는 `LaneMap.arrival_lane_candidates` 가 이음매에서 진출·진입 차선을 모두
인정하는 것이다. 이 테스트는 그 성질을 좌표 전수로 고정한다.
"""
import math
import unittest
from pathlib import Path

import yaml

from mapping import geometry
from mapping.lane_map import load_lane_map
from mapping.map_matcher import MapMatcher
from navigation.destination_resolver import DestinationResolver

MAP_PATH = Path(__file__).resolve().parents[3] / "map" / "main_track_map.yaml"
CONTROL_PATH = Path(__file__).resolve().parents[2] / "config" / "control.yaml"

STEP_M = 0.02          # 목적지 좌표 표본 간격
RADIUS_STEP_M = 0.02   # 반경 안 정지 위치 표본 간격

# 도착 반경 0.40 의 알려진 한계 (2026-08-08 "0.40으로 통일" 사용자 결정의 수용 비용,
# §0-53 "알려진 테스트 실패" — 실차 결함 아님):
#   목적지가 차선 양끝 0~2cm 밴드에 있으면, 반경 스윕의 원단(|d| ≥ 0.38)이 인접
#   회전 커넥터 기하 너머(이웃 차선)까지 나가 게이트 후보 밖이 된다. 시연 6곳
#   정차점은 전부 이 밴드 밖(test_places.py 가 보장). 여기서는 그 예외만 알려진
#   것으로 허용하고, 예외가 **늘어나면** 아래 상한 말뚝이 잡는다.
KNOWN_EDGE_BAND_M = 0.02
KNOWN_EDGE_FAILURE_MAX = 9   # 2026-08-17 전수 실측: 경계 밴드 실패 좌표 9건


class TestArbitraryDestination(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(CONTROL_PATH, encoding="utf-8") as f:
            cls.control = yaml.safe_load(f)
        cls.m = load_lane_map(MAP_PATH)
        cls.resolver = DestinationResolver(
            cls.m,
            warn_snap_distance_m=float(cls.control["destination"]["warn_snap_distance_m"]))
        cls.matcher = MapMatcher(cls.m, cls.control["matching"])
        cls.radius = float(cls.control["control"]["arrival_radius_m"])

    # ---------- 역인덱스 ----------

    def test_every_connector_has_an_entry_lane(self):
        """진입 차선이 없는 커넥터가 있으면 그 위에서는 진출 쪽만 인정된다."""
        orphan = [cid for cid in self.m.connectors
                  if not self.m.entry_lanes_of(cid)]
        self.assertEqual(orphan, [], f"진입 차선 없는 커넥터: {orphan[:5]}")

    def test_entry_index_is_unambiguous(self):
        """커넥터마다 진입 차선이 정확히 1개 — 합류가 생기면 여기서 알게 된다.

        여러 개여도 `arrival_lane_candidates` 는 동작한다(전부 인정). 다만 그때는
        게이트가 그만큼 넓어지므로, 맵에 합류가 도입되면 반경 게이트로 충분한지
        다시 판단해야 한다. 그 시점을 놓치지 않으려고 고정한다.
        """
        multi = {cid: sorted(self.m.entry_lanes_of(cid))
                 for cid in self.m.connectors
                 if len(self.m.entry_lanes_of(cid)) > 1}
        self.assertEqual(multi, {}, f"진입 차선이 여럿인 커넥터: {list(multi)[:5]}")

    def test_lane_on_lane_candidate_is_itself(self):
        lid = next(iter(self.m.lanes))
        self.assertEqual(self.m.arrival_lane_candidates(lid), frozenset({lid}))
        self.assertEqual(self.m.arrival_lane_candidates("없는세그먼트"), frozenset())

    def test_connector_candidates_are_exit_and_entry(self):
        cid = next(iter(self.m.connectors))
        cands = self.m.arrival_lane_candidates(cid)
        self.assertIn(self.m.connectors[cid].successor, cands)
        self.assertTrue(self.m.entry_lanes_of(cid) <= cands)

    # ---------- 좌표 전수 ----------

    def _gate_failures(self, sp):
        """정차점 sp 기준, 반경 안에서 게이트가 닫히는 지점 목록."""
        rad = math.radians(sp.heading_deg)
        ux, uy = math.cos(rad), math.sin(rad)
        out = []
        n = int(round(self.radius / RADIUS_STEP_M))
        for step in range(-n, n + 1):
            d = step * RADIUS_STEP_M
            match = self.matcher.match(sp.x + ux * d, sp.y + uy * d, sp.heading_deg)
            cands = (frozenset() if match is None
                     else self.m.arrival_lane_candidates(match.segment_id))
            if sp.lane_id not in cands:
                out.append(round(d, 3))
        return out

    def test_gate_opens_for_any_point_on_any_destination_lane(self):
        """정차 가능 차선을 2cm 간격으로 전수 훑어도 게이트가 닫히지 않는다.

        예외: 차선 양끝 KNOWN_EDGE_BAND_M 밴드의 원단 스윕 실패(위 상수 주석) —
        알려진 한계로 분리 집계하고, 총량이 늘면 여기서 잡는다.
        """
        failures, known_edge = [], []
        for lid, lane in self.m.lanes.items():
            if not lane.destination_allowed:
                continue
            steps = max(1, int(lane.length_m / STEP_M))
            for i in range(steps + 1):
                s = min(lane.length_m, i * STEP_M)
                x, y = geometry.point_at(lane.centerline, s)
                sp = self.resolver.resolve(x, y)
                bad = self._gate_failures(sp)
                if not bad:
                    continue
                near_edge = (s <= KNOWN_EDGE_BAND_M + 1e-9
                             or lane.length_m - s <= KNOWN_EDGE_BAND_M + 1e-9)
                far_only = all(abs(d) >= self.radius - KNOWN_EDGE_BAND_M for d in bad)
                if near_edge and far_only:
                    known_edge.append((lid, round(s, 3), bad))
                else:
                    failures.append((lid, round(s, 3), sp.lane_id, bad))
        self.assertEqual(
            failures[:5], [],
            f"게이트가 닫히는 목적지 좌표 {len(failures)}건 (앞 5건만 표시)")
        self.assertLessEqual(
            len(known_edge), KNOWN_EDGE_FAILURE_MAX,
            f"알려진 경계 밴드 실패가 늘었다({len(known_edge)}건) — 맵/반경 변경 확인: "
            f"{known_edge[:5]}")

    def test_middle_road_seam_bands_are_clear(self):
        """2026-07-30 에 교착이 재현된 그 구간을 직접 짚는다.

        가운데 가로도로 조각의 **끝 부근**과 **이음매 위**가 표적이다. 좌표는
        관제가 실제로 만들어 보내는 값의 형태 그대로(차선 중심선 위 임의 x)다.
        """
        seam_lanes = [lid for lid in self.m.lanes
                      if lid.startswith(("mid_eb2", "mid_wb1"))]
        self.assertTrue(seam_lanes, "가운데 가로도로 조각을 못 찾음 — 맵 이름 규칙 변경?")
        for lid in seam_lanes:
            lane = self.m.lanes[lid]
            if not lane.destination_allowed:
                continue
            # 조각 끝 20cm 안쪽 + 그 앞 이음매 위 20cm 를 1cm 간격으로
            for d in [lane.length_m - k / 100.0 for k in range(0, 21)]:
                if d < 0:
                    continue
                x, y = geometry.point_at(lane.centerline, d)
                sp = self.resolver.resolve(x, y)
                bad = self._gate_failures(sp)
                # 끝 KNOWN_EDGE_BAND_M 밴드의 원단 스윕은 알려진 한계 (상단 상수 주석)
                if (lane.length_m - d <= KNOWN_EDGE_BAND_M + 1e-9
                        and bad
                        and all(abs(f) >= self.radius - KNOWN_EDGE_BAND_M for f in bad)):
                    continue
                with self.subTest(lane=lid, from_end_m=round(lane.length_m - d, 3)):
                    self.assertEqual(
                        bad, [],
                        f"{lid} 끝에서 {lane.length_m - d:.2f}m 지점 — 게이트 닫힘")


if __name__ == "__main__":
    unittest.main()
