"""MapMatcher — 현재 pose가 어느 차선(또는 커넥터) 위인가 (명세서 §12).

점수: score = distance_weight × 중심선 수직거리 + heading_weight_per_deg × |방위차|.
컷을 통과한 후보가 없으면 None — §12.3 매칭 실패 시 출발 금지(모터 정지 유지)의 근거.

녹색 블록 내부는 후보 컷 이전에 기각한다 (§12.3 "녹색 블록·도로 밖·역방향 아님").
블록 가장자리와 인접 차선 중심의 거리가 0.14m 수준이라 거리 컷만으로는 못 거른다.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from mapping import geometry
from mapping.lane_map import LaneMap

log = logging.getLogger("mapping.matcher")

# 회전 중에는 원호를 따라가지 않아 차량이 어떤 centerline에서도 0.20m 밖이다 —
# 매칭 실패가 tick의 ~10%라 건별 경고는 로그를 폭증시킨다 (0805 board_final 3,315줄).
# 주행·도착에는 영향 없음(§19-2 게이트는 반경 0.10m가 이중으로 막는다).
_WARN_EVERY_SEC = 2.0


@dataclass(frozen=True)
class LaneMatch:
    segment_id: str
    kind: str                 # lane | connector
    distance_m: float         # 중심선까지 수직 거리
    heading_error_deg: float  # 세그먼트 진행 방향 대비 (−180~+180]
    progress_s: float         # 세그먼트 시작 기준 진행 거리
    score: float


class MapMatcher:
    def __init__(self, lane_map: LaneMap, cfg: dict):
        """cfg = control.yaml `matching` 섹션 (초안값 — 주행 후 조정)."""
        self._map = lane_map
        self._w_dist = float(cfg["distance_weight"])
        self._w_head = float(cfg["heading_weight_per_deg"])
        self._max_dist = float(cfg["max_center_distance_m"])
        self._max_head = float(cfg["max_heading_error_deg"])
        self._last_warn = 0.0
        self._suppressed = 0

    def match(self, x: float, y: float, heading_deg: float) -> LaneMatch | None:
        """§12.2 절차. 후보가 없으면 None (역방향·도로 밖·블록 위)."""
        if self._inside_block(x, y):
            return None

        best: LaneMatch | None = None
        for seg_id, kind, centerline in self._candidates():
            s, dist, _ = geometry.project_point(centerline, (x, y))
            if dist > self._max_dist:
                continue
            herr = geometry.heading_diff_deg(
                heading_deg, geometry.heading_at(centerline, s))
            if abs(herr) > self._max_head:
                continue
            score = self._w_dist * dist + self._w_head * abs(herr)
            if best is None or score < best.score:
                best = LaneMatch(seg_id, kind, dist, herr, s, score)
        if best is None:
            now = time.monotonic()
            if now - self._last_warn >= _WARN_EVERY_SEC:
                log.warning("차선 매칭 실패: (%.3f, %.3f) heading=%.1f° (그간 %d건 생략)",
                            x, y, heading_deg, self._suppressed)
                self._last_warn = now
                self._suppressed = 0
            else:
                self._suppressed += 1
        return best

    # ---------- 내부 ----------

    def _candidates(self):
        for lid, lane in self._map.lanes.items():
            yield lid, "lane", lane.centerline
        for cid, conn in self._map.connectors.items():
            yield cid, "connector", conn.centerline

    def _inside_block(self, x: float, y: float) -> bool:
        for block in self._map.blocks.values():
            xs = [p[0] for p in block.polygon]
            ys = [p[1] for p in block.polygon]
            if min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys):
                return True
        return False
