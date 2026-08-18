"""RoutePlanner — 일방통행 차선 그래프 최단 경로 (명세서 §14, WBS D2).

- 그래프: 차선 → 커넥터 → 차선 (successors 그대로 — 회전 진입 선택·차선 변경 포함)
- 비용: 순수 길이 (§14.2 초기 버전). 회전 페널티 등은 가중치 자리만 남긴다
- 알고리즘: Dijkstra (§14.3 — 맵이 작아 충분)
- 목적지가 현재 위치 뒤라면 후진 없이 한 바퀴 돌아 접근한다 (§3.2) —
  시작 차선 재진입을 허용하는 것으로 자연 처리된다

waypoint 추출: Route.waypoints(spacing)이 §15의 재샘플링 간이판이다. 곡률·목표 속도가
필요해지면 §15 TrajectoryGenerator 를 별도 모듈로 얹는다 (폴백 P-제어는 점 목록이면 된다).
"""
from __future__ import annotations

import heapq
from dataclasses import dataclass

from core.enums import ErrorCode
from core.exceptions import RcCarError
from mapping import geometry
from mapping.geometry import Point
from mapping.lane_map import LaneMap


class RouteError(RcCarError):
    """경로 없음 — 맵이 강연결이라 발생하면 내부 버그다 (§13.3: 도달 불가 경로는 시연 맵에 없음)."""

    def __init__(self, message: str):
        super().__init__(ErrorCode.INTERNAL_UNHANDLED_EXCEPTION, message)


@dataclass(frozen=True)
class RouteSegment:
    segment_id: str
    kind: str                       # lane | connector
    maneuver: str                   # follow | straight | left | right | lane_change
    centerline: tuple[Point, ...]
    speed_limit_mps: float
    length_m: float
    radius_m: float | None = None   # 회전 커넥터 호 반경 — 피드포워드 조향용 (2026-08-04)


@dataclass(frozen=True)
class Route:
    start_lane_id: str
    destination_lane_id: str
    segments: tuple[RouteSegment, ...]
    total_length_m: float

    def centerline(self) -> tuple[Point, ...]:
        """구간 중심선 연결 (접점 중복 제거)."""
        pts: list[Point] = []
        for seg in self.segments:
            for p in seg.centerline:
                if not pts or abs(p[0] - pts[-1][0]) > 1e-9 or abs(p[1] - pts[-1][1]) > 1e-9:
                    pts.append(p)
        return tuple(pts)

    def waypoints(self, spacing_m: float) -> tuple[Point, ...]:
        """일정 간격 waypoint (§15 간이 — 권장 0.02m)."""
        return geometry.resample(self.centerline(), spacing_m)


class RoutePlanner:
    def __init__(self, lane_map: LaneMap):
        self._map = lane_map

    def plan(self, start_lane_id: str, start_s: float,
             dest_lane_id: str, dest_s: float) -> Route:
        """차선 위 지점 → 차선 위 지점 최단 경로.

        start_s/dest_s 는 각 차선 시작 기준 진행 거리 (MapMatcher·DestinationResolver 출력).
        """
        lanes = self._map.lanes
        if start_lane_id not in lanes or dest_lane_id not in lanes:
            raise RouteError(f"알 수 없는 차선: {start_lane_id} → {dest_lane_id}")

        start = lanes[start_lane_id]
        # 같은 차선에서 목적지가 앞이면 그 자리에서 끝 (§14 자명 경로)
        if start_lane_id == dest_lane_id and dest_s >= start_s - 1e-9:
            pts = geometry.trim(start.centerline, start_s, dest_s)
            seg = RouteSegment(start_lane_id, "lane", "follow", pts,
                               start.speed_limit_mps, dest_s - start_s)
            return Route(start_lane_id, dest_lane_id, (seg,), max(0.0, dest_s - start_s))

        # Dijkstra — dist[lane] = 출발점에서 그 차선의 "시작점"까지의 길이.
        # 목적지가 같은 차선 뒤쪽이어도 재진입으로 돌아온다 (§3.2 한 바퀴).
        dist: dict[str, float] = {}
        prev: dict[str, tuple[str, str]] = {}  # lane → (직전 차선, 경유 커넥터)
        heap: list[tuple[float, str]] = []
        remain = start.length_m - start_s
        for cid in start.successors:
            conn = self._map.connectors[cid]
            cand = remain + conn.length_m
            nxt = conn.successor
            if cand < dist.get(nxt, float("inf")):
                dist[nxt] = cand
                prev[nxt] = (start_lane_id, cid)
                heapq.heappush(heap, (cand, nxt))

        settled: set[str] = set()
        while heap:
            d, lid = heapq.heappop(heap)
            if lid in settled or d > dist.get(lid, float("inf")):
                continue
            if lid == dest_lane_id:
                return self._reconstruct(start_lane_id, start_s, dest_lane_id, dest_s,
                                         prev, d + dest_s)
            settled.add(lid)
            lane = lanes[lid]
            for cid in lane.successors:
                conn = self._map.connectors[cid]
                cand = d + lane.length_m + conn.length_m
                nxt = conn.successor
                if nxt not in settled and cand < dist.get(nxt, float("inf")):
                    dist[nxt] = cand
                    prev[nxt] = (lid, cid)
                    heapq.heappush(heap, (cand, nxt))

        raise RouteError(f"경로 없음: {start_lane_id} → {dest_lane_id} (맵 강연결 위반?)")

    def plan_via_numbers(self, numbers, start_s: float, dest_s: float) -> Route:
        """번호 노선 강제 지정 — Dijkstra 를 우회하고 주어진 차선 번호 시퀀스를 그대로 편성.

        시나리오 노선 운용의 입구 (2026-08-06 안 1 확정 — 순수 길이 최단 플래너는
        안 2를 고르므로 사용자 확정 노선은 이 통로로만 탈 수 있다).
        출력은 plan() 과 같은 Route (시작 start_s / 목적 dest_s 트림 포함) —
        LaneFollower·lane_route.from_route() 등 하류는 강제 노선을 구분할 필요가 없다.
        """
        from navigation.lane_route import NUM2ID
        nums = [int(n) for n in numbers]
        if len(nums) < 2:
            raise RouteError(f"강제 노선은 차선 2개 이상이어야 함: {nums}")
        unknown = [n for n in nums if n not in NUM2ID]
        if unknown:
            raise RouteError(f"강제 노선에 알 수 없는 차선 번호: {unknown}")

        lanes, conns = self._map.lanes, self._map.connectors
        first = lanes[NUM2ID[nums[0]]]
        start_s = min(max(0.0, start_s), first.length_m)
        segs: list[RouteSegment] = [RouteSegment(
            NUM2ID[nums[0]], "lane", "follow",
            geometry.trim(first.centerline, start_s, first.length_m),
            first.speed_limit_mps, first.length_m - start_s)]
        total = first.length_m - start_s
        for i in range(1, len(nums)):
            prev_id, cur_id = NUM2ID[nums[i - 1]], NUM2ID[nums[i]]
            conn_id = next((cid for cid in lanes[prev_id].successors
                            if conns[cid].successor == cur_id), None)
            if conn_id is None:
                raise RouteError(f"강제 노선 단절: {nums[i-1]} → {nums[i]} 커넥터 없음")
            conn = conns[conn_id]
            segs.append(RouteSegment(conn_id, "connector", conn.maneuver, conn.centerline,
                                     conn.speed_limit_mps, conn.length_m,
                                     radius_m=conn.radius_m))
            total += conn.length_m
            lane = lanes[cur_id]
            if i == len(nums) - 1:
                dest_s = min(max(0.0, dest_s), lane.length_m)
                segs.append(RouteSegment(cur_id, "lane", "follow",
                                         geometry.trim(lane.centerline, 0.0, dest_s),
                                         lane.speed_limit_mps, dest_s))
                total += dest_s
            else:
                segs.append(RouteSegment(cur_id, "lane", "follow", lane.centerline,
                                         lane.speed_limit_mps, lane.length_m))
                total += lane.length_m
        return Route(NUM2ID[nums[0]], NUM2ID[nums[-1]], tuple(segs), total)

    # ---------- 내부 ----------

    def _reconstruct(self, start_id: str, start_s: float, dest_id: str, dest_s: float,
                     prev: dict, total: float) -> Route:
        """prev 사슬 역추적. 시작 차선을 재진입하는 한 바퀴 경로도 같은 방식으로 풀린다
        (시작 차선 경유 재완화는 초기 시드보다 항상 비싸므로, prev가 start를 가리키는
        항목은 곧 경로의 첫 진입이다)."""
        chain: list[tuple[str, str]] = []  # (차선, 그 차선으로 들어온 커넥터)
        lid = dest_id
        while True:
            p_lane, p_conn = prev[lid]
            chain.append((lid, p_conn))
            if p_lane == start_id:
                break
            lid = p_lane
        chain.reverse()

        lanes, conns = self._map.lanes, self._map.connectors
        start = lanes[start_id]
        segs: list[RouteSegment] = [RouteSegment(
            start_id, "lane", "follow",
            geometry.trim(start.centerline, start_s, start.length_m),
            start.speed_limit_mps, start.length_m - start_s)]
        for lid, cid in chain:
            conn = conns[cid]
            segs.append(RouteSegment(cid, "connector", conn.maneuver, conn.centerline,
                                     conn.speed_limit_mps, conn.length_m,
                                     radius_m=conn.radius_m))
            lane = lanes[lid]
            if (lid, cid) == chain[-1]:  # 목적지 차선 — dest_s 에서 자른다
                segs.append(RouteSegment(lid, "lane", "follow",
                                         geometry.trim(lane.centerline, 0.0, dest_s),
                                         lane.speed_limit_mps, dest_s))
            else:
                segs.append(RouteSegment(lid, "lane", "follow", lane.centerline,
                                         lane.speed_limit_mps, lane.length_m))
        return Route(start_id, dest_id, tuple(segs), total)
