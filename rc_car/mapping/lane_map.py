"""차선 그래프 맵 — 로더·검증·그래프 API (명세서 §11·§14.1, WBS D1).

맵 데이터의 단일 소스는 `map/main_track_map.yaml` (rc_car 형제) 이다 (관제 서버와 공유 — 사본 금지).
이 모듈은 읽기와 검증만 한다. 경로 탐색은 navigation(D2), 차선 매칭은 §12 MapMatcher.

노면 화살표 규칙만으로는 맵이 두 순환계(inner/outer)로 완전 분리되지만, 회전 진입
차선 선택(_far) 커넥터 16개가 두 계를 이어 전체 차선이 강연결이다 (2026-08-04:
점선 구간 차선 변경 커넥터는 폐지 — 짧은 이음매의 대각 기동이 인도 침입을 유발).
로더는 전 차선 쌍 도달 가능성을 검증한다 — 다리가 빠지면 로드가 거부된다.
lanes.*.circuit 은 링 계열 참고 라벨일 뿐, 도달성 제약이 아니다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from core.exceptions import MapError

Point = tuple[float, float]

_CONTINUITY_TOL = 0.005   # 차선-커넥터 접점 허용 오차 (생성기는 1e-6 보장)
_MANEUVERS = ("straight", "left", "right", "follow", "lane_change")
# follow = 차선 변경 구간에서 차선 유지 / lane_change = 인접 차선으로 대각 이동 (§3.2 예외)


@dataclass(frozen=True)
class LaneSegment:
    id: str
    width_m: float
    centerline: tuple[Point, ...]
    heading_hint_deg: float
    speed_limit_mps: float
    destination_allowed: bool
    circuit: str                    # inner | outer
    successors: tuple[str, ...]     # connector id 목록
    length_m: float


@dataclass(frozen=True)
class Connector:
    id: str
    maneuver: str                   # straight | left | right
    centerline: tuple[Point, ...]
    speed_limit_mps: float
    successor: str                  # lane id
    radius_m: float | None          # 회전 호 반경 (straight 은 None)
    length_m: float


@dataclass(frozen=True)
class CurbEdge:
    segment: tuple[Point, Point]
    adjacent_destination_lanes: tuple[str, ...]


@dataclass(frozen=True)
class Block:
    id: str
    polygon: tuple[Point, ...]
    curb_edges: dict[str, CurbEdge]


@dataclass(frozen=True)
class StopLineZone:
    id: str
    approach_lane_id: str
    expected_center: Point
    activation_distance_m: float


class LaneMap:
    """불변 차선 그래프. 생성 시 전체 검증을 수행한다 (실패 = MapError)."""

    def __init__(self, lanes, connectors, blocks, stop_line_zones,
                 width_m: float, height_m: float):
        self.lanes: dict[str, LaneSegment] = lanes
        self.connectors: dict[str, Connector] = connectors
        self.blocks: dict[str, Block] = blocks
        self.stop_line_zones: dict[str, StopLineZone] = stop_line_zones
        self.width_m = width_m
        self.height_m = height_m
        # 커넥터 → 진입 차선 역인덱스. 참조 검증(_validate) 전에 만들어지므로
        # 아직 미검증인 successor 는 건너뛴다 — 검증이 뒤에서 잡는다.
        entry: dict[str, set[str]] = {cid: set() for cid in connectors}
        for lid, lane in lanes.items():
            for cid in lane.successors:
                if cid in entry:
                    entry[cid].add(lid)
        self._entry_lanes: dict[str, frozenset[str]] = {
            cid: frozenset(v) for cid, v in entry.items()}

    # ---------- 그래프 API ----------

    def successors_of(self, segment_id: str) -> tuple[str, ...]:
        """lane → connector 목록, connector → (successor lane,) — 이분 그래프."""
        if segment_id in self.lanes:
            return self.lanes[segment_id].successors
        if segment_id in self.connectors:
            return (self.connectors[segment_id].successor,)
        raise MapError(f"알 수 없는 세그먼트: {segment_id}")

    def entry_lanes_of(self, connector_id: str) -> frozenset[str]:
        """이 커넥터로 진입하는 차선들 (successors_of 의 역방향). 미지의 id 는 빈 집합."""
        return self._entry_lanes.get(connector_id, frozenset())

    def arrival_lane_candidates(self, segment_id: str) -> frozenset[str]:
        """도착 판정 §19-2 용 — 이 세그먼트 위의 차가 "있다고 인정"되는 차선 집합.

        차선 위면 그 차선 하나. **이음매(커넥터) 위면 진출 차선과 진입 차선을 모두**
        인정한다 — 이음매는 "다음 조각으로 가는 길"이면서 동시에 "직전 조각에서
        나온 자리"이기 때문이다. 어느 쪽에서 왔는지는 매칭만으로 알 수 없으므로
        양쪽을 인정하는 것이 사실에 맞는 답이고, 반경 게이트(0.10m)가 오용을 막는다.

        한쪽만 인정하면 조각 끝에 선 차가 1cm만 지나쳐도 게이트가 영영 안 열려
        complete 가 나가지 않는다 (2026-07-28 S6 = 진입 쪽 누락,
        2026-07-30 = 진출 쪽 누락. 워크로그 §0-19·§0-21).
        """
        if segment_id in self.lanes:
            return frozenset({segment_id})
        if segment_id in self.connectors:
            return (frozenset({self.connectors[segment_id].successor})
                    | self.entry_lanes_of(segment_id))
        return frozenset()

    def lanes_in_circuit(self, circuit: str) -> tuple[str, ...]:
        return tuple(lid for lid, l in self.lanes.items() if l.circuit == circuit)

    def reachable_lanes(self, from_lane_id: str) -> set[str]:
        """from(포함)에서 도달 가능한 차선 집합."""
        if from_lane_id not in self.lanes:
            raise MapError(f"알 수 없는 차선: {from_lane_id}")
        seen, stack = {from_lane_id}, [from_lane_id]
        while stack:
            for cid in self.lanes[stack.pop()].successors:
                nxt = self.connectors[cid].successor
                if nxt not in seen:
                    seen.add(nxt)
                    stack.append(nxt)
        return seen

    # ---------- 구성 ----------

    @classmethod
    def from_dict(cls, data: dict) -> "LaneMap":
        if not isinstance(data, dict):
            raise MapError("맵 데이터 최상위가 매핑이 아님")
        meta = _require(data, "map", dict)
        for key, expect in (("width_m", 5.0), ("height_m", 3.0), ("origin", "bottom_left")):
            if meta.get(key) != expect:
                raise MapError(f"map.{key} 는 {expect!r} 이어야 함 (§2.6): {meta.get(key)!r}")

        lanes = {}
        for lid, raw in _require(data, "lanes", dict).items():
            pts = _points(raw, f"lanes.{lid}", meta)
            lanes[lid] = LaneSegment(
                id=lid,
                width_m=_num(raw, "width_m", f"lanes.{lid}", positive=True),
                centerline=pts,
                heading_hint_deg=_num(raw, "heading_hint_deg", f"lanes.{lid}"),
                speed_limit_mps=_num(raw, "speed_limit_mps", f"lanes.{lid}", positive=True),
                destination_allowed=bool(raw.get("destination_allowed", False)),
                circuit=str(raw.get("circuit", "")),
                successors=tuple(raw.get("successors", []) or []),
                length_m=_length(pts),
            )

        connectors = {}
        for cid, raw in _require(data, "connectors", dict).items():
            pts = _points(raw, f"connectors.{cid}", meta)
            man = raw.get("maneuver")
            if man not in _MANEUVERS:
                raise MapError(f"connectors.{cid}.maneuver 불명: {man!r}")
            r = raw.get("radius_m")
            connectors[cid] = Connector(
                id=cid,
                maneuver=man,
                centerline=pts,
                speed_limit_mps=_num(raw, "speed_limit_mps", f"connectors.{cid}", positive=True),
                successor=str(raw.get("successor", "")),
                radius_m=float(r) if r is not None else None,
                length_m=_length(pts),
            )

        blocks = {}
        for bid, raw in (data.get("blocks") or {}).items():
            poly = tuple((float(x), float(y)) for x, y in raw.get("polygon", []))
            curbs = {}
            for side, edge in (raw.get("curb_edges") or {}).items():
                seg = edge.get("segment", [])
                if len(seg) != 2:
                    raise MapError(f"blocks.{bid}.curb_edges.{side}.segment 는 2점")
                curbs[side] = CurbEdge(
                    segment=((float(seg[0][0]), float(seg[0][1])),
                             (float(seg[1][0]), float(seg[1][1]))),
                    adjacent_destination_lanes=tuple(edge.get("adjacent_destination_lanes") or []),
                )
            blocks[bid] = Block(id=bid, polygon=poly, curb_edges=curbs)

        zones = {}
        for zid, raw in (data.get("stop_line_zones") or {}).items():
            center = raw.get("expected_center", [])
            if len(center) != 2:
                raise MapError(f"stop_line_zones.{zid}.expected_center 는 [x, y]")
            zones[zid] = StopLineZone(
                id=zid,
                approach_lane_id=str(raw.get("approach_lane_id", "")),
                expected_center=(float(center[0]), float(center[1])),
                activation_distance_m=_num(raw, "activation_distance_m",
                                           f"stop_line_zones.{zid}", positive=True),
            )

        lane_map = cls(lanes, connectors, blocks, zones,
                       float(meta["width_m"]), float(meta["height_m"]))
        lane_map._validate()
        return lane_map

    # ---------- 검증 ----------

    def _validate(self) -> None:
        if not self.lanes:
            raise MapError("차선이 없음")
        self._validate_references()
        self._validate_continuity()
        self._validate_headings()
        self._validate_circuits()

    def _validate_references(self) -> None:
        for lid, lane in self.lanes.items():
            if not lane.successors:
                raise MapError(f"{lid}: successor 없음 — 막다른 차선 (§14.1 위반)")
            for cid in lane.successors:
                if cid not in self.connectors:
                    raise MapError(f"{lid}: 알 수 없는 successor 커넥터 {cid!r}")
        for cid, conn in self.connectors.items():
            if conn.successor not in self.lanes:
                raise MapError(f"{cid}: 알 수 없는 successor 차선 {conn.successor!r}")
        for bid, block in self.blocks.items():
            for side, curb in block.curb_edges.items():
                for lid in curb.adjacent_destination_lanes:
                    if lid not in self.lanes:
                        raise MapError(f"{bid}.{side}: 알 수 없는 차선 {lid!r}")
                    if not self.lanes[lid].destination_allowed:
                        raise MapError(f"{bid}.{side}: {lid}는 정차 금지 차선")
        for zid, zone in self.stop_line_zones.items():
            if zone.approach_lane_id not in self.lanes:
                raise MapError(f"{zid}: 알 수 없는 접근 차선 {zone.approach_lane_id!r}")

    def _validate_continuity(self) -> None:
        for lid, lane in self.lanes.items():
            for cid in lane.successors:
                conn = self.connectors[cid]
                if _dist(lane.centerline[-1], conn.centerline[0]) > _CONTINUITY_TOL:
                    raise MapError(f"{lid} 끝 ≠ {cid} 시작 (불연속)")
                nxt = self.lanes[conn.successor]
                if _dist(conn.centerline[-1], nxt.centerline[0]) > _CONTINUITY_TOL:
                    raise MapError(f"{cid} 끝 ≠ {conn.successor} 시작 (불연속)")

    def _validate_headings(self) -> None:
        for lid, lane in self.lanes.items():
            (x0, y0), (x1, y1) = lane.centerline[0], lane.centerline[-1]
            net = math.hypot(x1 - x0, y1 - y0)
            if net <= 0:
                raise MapError(f"{lid}: 길이 0 차선")
            h = math.radians(lane.heading_hint_deg)
            dot = ((x1 - x0) * math.cos(h) + (y1 - y0) * math.sin(h)) / net
            if dot < 0.99:
                raise MapError(f"{lid}: centerline 방향이 heading_hint와 불일치")

    def _validate_circuits(self) -> None:
        labels = {l.circuit for l in self.lanes.values()}
        if labels != {"inner", "outer"}:
            raise MapError(f"circuit 라벨은 inner/outer 이어야 함: {labels}")
        # 전체 강연결 — 임의 seed에서 전방·후방 도달이 모두 전 차선을 덮으면 강연결이다.
        # (lane_change 커넥터가 빠지면 두 순환계로 갈라져 여기서 걸린다)
        seed = next(iter(self.lanes))
        forward = self.reachable_lanes(seed)
        if forward != set(self.lanes):
            raise MapError(
                f"{seed}에서 도달 불가 차선 존재: {sorted(set(self.lanes) - forward)[:4]} "
                "(lane_change 커넥터 누락?)")
        backward, stack = {seed}, [seed]
        pred: dict[str, set[str]] = {lid: set() for lid in self.lanes}
        for lid, lane in self.lanes.items():
            for cid in lane.successors:
                pred[self.connectors[cid].successor].add(lid)
        while stack:
            for prev in pred[stack.pop()]:
                if prev not in backward:
                    backward.add(prev)
                    stack.append(prev)
        if backward != set(self.lanes):
            raise MapError(
                f"{seed}로 복귀 불가 차선 존재: {sorted(set(self.lanes) - backward)[:4]}")

    def validate_turn_radius(self, min_radius_m: float) -> None:
        """회전 커넥터 반경이 차량 최소 회전 반경 이상인지 (§15). 호출자가 차량값 주입."""
        for cid, conn in self.connectors.items():
            if conn.radius_m is not None and conn.radius_m < min_radius_m:
                raise MapError(
                    f"{cid}: 반경 {conn.radius_m}m < 최소 회전 반경 {min_radius_m}m")


def load_lane_map(path: str | Path) -> LaneMap:
    p = Path(path)
    if not p.exists():
        raise MapError(f"맵 파일 없음: {p}")
    with open(p, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return LaneMap.from_dict(data)


# ---------- 내부 도우미 ----------

def _require(data: dict, key: str, typ: type):
    if key not in data or not isinstance(data[key], typ):
        raise MapError(f"'{key}' 섹션 없음 또는 형식 오류")
    return data[key]


def _num(raw: dict, key: str, where: str, positive: bool = False) -> float:
    if key not in raw:
        raise MapError(f"{where}.{key} 없음")
    try:
        v = float(raw[key])
    except (TypeError, ValueError):
        raise MapError(f"{where}.{key} 숫자 아님: {raw[key]!r}")
    if positive and v <= 0:
        raise MapError(f"{where}.{key} 는 양수여야 함: {v}")
    return v


def _points(raw: dict, where: str, meta: dict) -> tuple[Point, ...]:
    pts = raw.get("centerline")
    if not isinstance(pts, list) or len(pts) < 2:
        raise MapError(f"{where}.centerline 은 2점 이상")
    out = []
    w, h = float(meta["width_m"]), float(meta["height_m"])
    for p in pts:
        if not isinstance(p, list) or len(p) != 2:
            raise MapError(f"{where}.centerline 점 형식 오류: {p!r}")
        x, y = float(p[0]), float(p[1])
        if not (0.0 <= x <= w and 0.0 <= y <= h):
            raise MapError(f"{where}: 맵 범위 밖 좌표 ({x}, {y}) (§2.6)")
        out.append((x, y))
    return tuple(out)


def _length(pts: tuple[Point, ...]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
