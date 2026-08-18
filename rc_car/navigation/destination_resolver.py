"""DestinationResolver — 관제가 보낸 도로 위 좌표 → 차선 위 정차점 (명세서 §13 축소판).

관제 서버가 이미 도로 위로 보정한 좌표를 받으므로 여기서는 미세 조정만 한다 (§13.2-1).
- 후보는 `destination_allowed: true` 차선뿐 — 커넥터·교차로 사이 스텁 제외 (§13.2-5)
- 좌표가 멀어도 거부하지 않는다 (§13.3 명령 거부 없음) — 경고 로그만 남긴다.
  맵 범위 검사는 CommandPolicy(§10.1)가 이미 수행했다.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

from mapping import geometry
from mapping.lane_map import LaneMap

log = logging.getLogger("navigation.destination")


@dataclass(frozen=True)
class StopPose:
    lane_id: str
    x: float
    y: float
    heading_deg: float        # 도착 방향 = 차선 진행 방향 (§13.2-4)
    progress_s: float         # 차선 시작 기준 진행 거리
    snap_distance_m: float    # 원 좌표 → 정차점 거리


class DestinationResolver:
    def __init__(self, lane_map: LaneMap, warn_snap_distance_m: float):
        self._map = lane_map
        self._warn_dist = warn_snap_distance_m
        self._lanes = [
            (lid, lane) for lid, lane in lane_map.lanes.items() if lane.destination_allowed
        ]
        assert self._lanes, "정차 가능 차선이 없는 맵"

    def resolve(self, x: float, y: float) -> StopPose:
        best = None
        for lid, lane in self._lanes:
            s, dist, q = geometry.project_point(lane.centerline, (x, y))
            if best is None or dist < best[1]:
                best = (lid, dist, s, q, lane)
        lid, dist, s, (qx, qy), lane = best
        if dist > self._warn_dist:
            # 관제 보정이 전제라 발생하면 관제 쪽 문제 (§13.2 주석) — 그래도 진행한다
            log.warning("목적지 (%.3f, %.3f)가 차선에서 %.2fm 벗어남 → %s 로 스냅",
                        x, y, dist, lid)
        return StopPose(lane_id=lid, x=qx, y=qy,
                        heading_deg=geometry.heading_at(lane.centerline, s),
                        progress_s=s, snap_distance_m=dist)
