"""LaneRoute — 차선 번호 노선 (2026-08-06 재설계 §0-45).

주행을 "가상 라인 트레이싱"에서 **차선 번호 point-to-point + 2단 회전**으로 바꾼 핵심 자료구조.

무엇이 달라지나
    (구) 경로 = 차선 원호 + 커넥터 원호를 0.02m로 재샘플한 좌표열 → 그 선을 그대로 추종
    (신) 경로 = **차선 번호 시퀀스**. 차선 구간은 시작점→끝점 직선 주행(point-to-point),
         회전은 좌표가 아니라 **지시**(어느 방향으로, 몇 번 차선으로)로만 들고 간다.
         꺾는 시점·조향각은 TurnTable + 비전이 정한다 (control/lane_follower.py).

왜 성립하나 (2026-08-06 맵 실측)
    - 32개 차선 centerline이 **전부 정확히 2점 = 직선**이다. 차선 구간은 point-to-point가
      근사가 아니라 정확하다.
    - 커넥터는 이미 `maneuver`(left/right/straight)를 갖고 있다 → "다음은 우회전"을
      알려주는 데 맵 수정이 전혀 필요 없다. 원호 좌표(centerline)만 **안 쓰면 된다**.
      맵은 손대지 않는다(사용자 결정) — 원호는 폐기 폴백용으로 그대로 남는다.

경로 탐색은 기존 RoutePlanner(Dijkstra)를 그대로 재사용한다. 이 모듈은 그 결과에서
**원호 좌표를 버리고** 차선 다리(leg)와 회전 지시(turn)로 접는 변환기다.

번호 노선 직접 지정(`from_lane_numbers`)도 지원한다 — 사용자가 `11 → 22 → 14` 처럼
노선을 직접 설계하는 운용을 위한 입구다 (tools/route_analysis.py route 와 같은 번호 체계).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from core.enums import ErrorCode
from core.exceptions import RcCarError
from mapping.geometry import Point

# 차선 번호 ↔ ID — 자료/차선번호맵, tools/route_analysis.py 와 같은 순번.
# 코드에서 유일한 정의가 되도록 여기에 둔다 (도구 쪽이 이 표를 참조해도 된다).
NUM2ID: dict[int, str] = {
    1: "top_outer_wb_e", 2: "top_outer_wb_w", 3: "top_inner_eb_w", 4: "top_inner_eb_e",
    5: "mid_wb1_e", 6: "mid_wb1_w", 7: "mid_wb2_e", 8: "mid_wb2_w",
    9: "mid_eb1_w", 10: "mid_eb1_e", 11: "mid_eb2_w", 12: "mid_eb2_e",
    13: "bot_inner_wb_e", 14: "bot_inner_wb_w", 15: "bot_outer_eb_w", 16: "bot_outer_eb_e",
    17: "left_outer_sb_n", 18: "left_outer_sb_s", 19: "left_inner_nb_s", 20: "left_inner_nb_n",
    21: "vert_sb1_n", 22: "vert_sb1_s", 23: "vert_sb2_n", 24: "vert_sb2_s",
    25: "vert_nb1_s", 26: "vert_nb1_n", 27: "vert_nb2_s", 28: "vert_nb2_n",
    29: "right_inner_sb_n", 30: "right_inner_sb_s", 31: "right_outer_nb_s", 32: "right_outer_nb_n",
}
ID2NUM: dict[str, int] = {v: k for k, v in NUM2ID.items()}

# ---------- 대칭 (2026-08-06 사용자 제시 — 튜닝 전이의 근거) ----------
#
# 시연 맵은 **180° 점대칭**이다: (x, y) → (5 − x, 3 − y).
# 사용자 표현: "각각 짝을 지어 놓은 차선들은 직진 및 회전하는 도로의 형태가 동일하기 때문에
#              하나의 차선을 튜닝하면 그에 대칭되는 차선도 이를 활용해서 주행할 수 있다"
#
# 맵 실물 대조 결과 (tests/unit/test_lane_symmetry.py 가 상시 검증):
#   - 16쌍 전부 centerline 이 정확히 점대칭
#   - 대응하는 회전 32쌍 전부 maneuver·radius_m·length_m 합동
#   - 차선 길이 16쌍 전부 일치
#
# 왜 회전 방향이 보존되나: 점대칭(180° 회전)은 손잡이(handedness)를 바꾸지 않는다.
# 우회전은 대칭 후에도 우회전이다. (거울 대칭이었다면 좌↔우가 뒤집혀 전이가 불가능했다)
#
# 효과: 회전 커넥터 48개 → **튜닝 클래스 24종**. 시나리오 1 안 1은 회전 6회 → 4종.
SYMMETRIC_PAIRS: tuple[tuple[int, int], ...] = (
    (1, 15), (2, 16), (3, 13), (4, 14), (5, 11), (6, 12), (7, 9), (8, 10),
    (17, 31), (18, 32), (19, 29), (20, 30), (21, 27), (23, 25), (26, 24), (28, 22),
)
SYMMETRIC_NUM: dict[int, int] = {}
for _a, _b in SYMMETRIC_PAIRS:
    SYMMETRIC_NUM[_a] = _b
    SYMMETRIC_NUM[_b] = _a
del _a, _b


def symmetric_num(num: int | None) -> int | None:
    """대칭 차선 번호. 모르는 번호면 None."""
    return SYMMETRIC_NUM.get(num) if num is not None else None


def symmetric_key(key: str) -> str | None:
    """회전 키 "11->22" → 대칭 회전 키 "5->28". 번호 키가 아니면 None."""
    if "->" not in key:
        return None
    a, _, b = key.partition("->")
    try:
        sa, sb = SYMMETRIC_NUM.get(int(a)), SYMMETRIC_NUM.get(int(b))
    except ValueError:
        return None
    if sa is None or sb is None:
        return None
    return f"{sa}->{sb}"


def tuning_class(key: str) -> str:
    """대칭 쌍을 하나로 묶은 대표 이름 — 튜닝 단위. 두 키 중 사전순 앞선 쪽."""
    other = symmetric_key(key)
    if other is None:
        return key
    return min(key, other)


def symmetric_point(p: Point) -> Point:
    """맵 점대칭 변환 — 검증·시각화용."""
    return (5.0 - p[0], 3.0 - p[1])


class LaneRouteError(RcCarError):
    def __init__(self, message: str):
        super().__init__(ErrorCode.INTERNAL_UNHANDLED_EXCEPTION, message)


@dataclass(frozen=True)
class LaneLeg:
    """직선 주행 구간 — 한 차선 위의 point-to-point 다리."""
    lane_id: str
    lane_num: int | None
    points: tuple[Point, ...]        # 보통 2점(직선). trim된 부분 차선도 2점이다
    speed_limit_mps: float
    length_m: float

    @property
    def entry(self) -> Point:
        return self.points[0]

    @property
    def exit(self) -> Point:
        return self.points[-1]

    def heading_deg(self) -> float:
        """다리 진행 방위 — 회전 종료(다음 차선 정렬) 판정 기준."""
        (x0, y0), (x1, y1) = self.points[0], self.points[-1]
        return math.degrees(math.atan2(y1 - y0, x1 - x0))


@dataclass(frozen=True)
class Turn:
    """회전 지시 — **좌표가 아니라 지시다.** 원호 centerline은 의도적으로 들고 오지 않는다."""
    connector_id: str
    maneuver: str                    # left | right | straight
    from_num: int | None
    to_num: int | None
    to_lane_id: str
    radius_m: float | None           # 기본 조향각 산출용(atan(축거/R)) — 표에서 덮어쓸 수 있다
    length_m: float                  # 남은거리 회계용. 추종 경로로는 쓰지 않는다
    entry_point: Point               # 직전 차선의 끝점 = 좌표 게이트(1차) 기준점
    exit_point: Point                # 다음 차선의 시작점 = 회전 종료 목표

    @property
    def key(self) -> str:
        """TurnTable 조회 키 — "11->22". 번호를 모르면 커넥터 id로 떨어진다."""
        if self.from_num is None or self.to_num is None:
            return self.connector_id
        return f"{self.from_num}->{self.to_num}"


@dataclass(frozen=True)
class LaneRoute:
    """legs[i] 직선 주행 → turns[i] 회전 → legs[i+1] ... (turns 길이 = legs 길이 − 1)."""
    legs: tuple[LaneLeg, ...]
    turns: tuple[Turn, ...]
    total_length_m: float

    def __post_init__(self) -> None:
        if not self.legs:
            raise LaneRouteError("빈 노선")
        if len(self.turns) != len(self.legs) - 1:
            raise LaneRouteError(
                f"회전 수 불일치: legs {len(self.legs)} / turns {len(self.turns)}")

    def lane_numbers(self) -> tuple[int | None, ...]:
        return tuple(leg.lane_num for leg in self.legs)

    def describe(self) -> str:
        """로그용 — `11 →우 22 →우 14`. 기동 로그에서 노선을 눈으로 확인하기 위한 것."""
        mark = {"left": "좌", "right": "우", "straight": "직"}
        out = [str(self.legs[0].lane_num or self.legs[0].lane_id)]
        for turn, leg in zip(self.turns, self.legs[1:]):
            out.append(f"→{mark.get(turn.maneuver, '?')}")
            out.append(str(leg.lane_num or leg.lane_id))
        return " ".join(out)

    def tail_length_m(self, leg_index: int) -> float:
        """legs[leg_index] 이후(회전 포함)에 남은 총 길이 — 남은거리 회계용."""
        total = 0.0
        for i in range(leg_index, len(self.turns)):
            total += self.turns[i].length_m + self.legs[i + 1].length_m
        return total


# ---------- Route(원호 포함) → LaneRoute(원호 폐기) ----------

def from_route(route, lane_map=None) -> LaneRoute:
    """RoutePlanner.Route 를 차선 번호 노선으로 접는다.

    🔴 커넥터의 centerline(원호 좌표)은 **여기서 버려진다.** 이 함수가
    "가상 라인 트레이싱 폐기"가 실제로 일어나는 지점이다.
    """
    legs: list[LaneLeg] = []
    turns: list[Turn] = []
    pending: list = []          # 차선 사이에서 만난 커넥터들 (보통 1개)

    for seg in route.segments:
        if seg.kind == "lane":
            pts = tuple(seg.centerline)
            if not pts:
                continue                # 좌표 없는 구간은 다리가 될 수 없다
            if len(pts) == 1:
                # 제자리 목적지(관제가 현 위치로 호출) — 길이 0 경로.
                # 2점으로 복제해 두면 추종 계산이 정상 경로와 같은 길을 타고
                # 남은거리 ≈ 0 → 정지 명령이 나간다 (길이 0에서 죽던 회귀 대책)
                pts = pts + pts
            leg = LaneLeg(seg.segment_id, ID2NUM.get(seg.segment_id),
                          pts, seg.speed_limit_mps, seg.length_m)
            if legs and not pending:
                # 커넥터 없이 차선이 연달아 나오는 일은 없어야 한다 (플래너 계약)
                raise LaneRouteError(f"연속 차선 구간: {legs[-1].lane_id} → {leg.lane_id}")
            if legs:
                conn = pending[-1]
                turns.append(Turn(
                    connector_id=conn.segment_id, maneuver=conn.maneuver,
                    from_num=legs[-1].lane_num, to_num=leg.lane_num,
                    to_lane_id=leg.lane_id,
                    radius_m=getattr(conn, "radius_m", None),
                    length_m=conn.length_m,
                    entry_point=legs[-1].exit, exit_point=leg.entry))
                pending = []
            legs.append(leg)
        else:
            pending.append(seg)

    if not legs:
        raise LaneRouteError("차선 구간이 없는 경로")
    return LaneRoute(tuple(legs), tuple(turns), route.total_length_m)


# ---------- 번호 노선 직접 지정 ----------

def from_lane_numbers(lane_map, numbers) -> LaneRoute:
    """`[11, 22, 14]` → LaneRoute. 사용자가 노선을 직접 설계하는 입구.

    각 차선은 전 구간을 달린다 (부분 구간 지정 없음). 연속한 두 번호 사이에
    커넥터가 없으면 오류 — tools/route_analysis.py route 와 같은 검증이다.
    """
    nums = [int(n) for n in numbers]
    if len(nums) < 1:
        raise LaneRouteError("빈 번호 노선")
    unknown = [n for n in nums if n not in NUM2ID]
    if unknown:
        raise LaneRouteError(f"알 수 없는 차선 번호: {unknown}")

    legs: list[LaneLeg] = []
    turns: list[Turn] = []
    total = 0.0
    for i, num in enumerate(nums):
        lid = NUM2ID[num]
        lane = lane_map.lanes[lid]
        leg = LaneLeg(lid, num, tuple(lane.centerline), lane.speed_limit_mps, lane.length_m)
        total += lane.length_m
        if i > 0:
            prev = lane_map.lanes[NUM2ID[nums[i - 1]]]
            conn = None
            for cid in prev.successors:
                cand = lane_map.connectors[cid]
                if cand.successor == lid:
                    conn = cand
                    break
            if conn is None:
                raise LaneRouteError(
                    f"{nums[i-1]} → {num} 직접 연결 없음 (맵 그래프에 커넥터 부재)")
            turns.append(Turn(
                connector_id=conn.connector_id if hasattr(conn, "connector_id") else cid,
                maneuver=conn.maneuver, from_num=nums[i - 1], to_num=num,
                to_lane_id=lid, radius_m=getattr(conn, "radius_m", None),
                length_m=conn.length_m,
                entry_point=legs[-1].exit, exit_point=leg.entry))
            total += conn.length_m
        legs.append(leg)
    return LaneRoute(tuple(legs), tuple(turns), total)
