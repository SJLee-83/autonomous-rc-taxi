"""폴리라인 기하 연산 (명세서 §7 mapping/geometry).

차선·커넥터 중심선(점 목록)에 대한 공통 연산. MapMatcher(§12)·DestinationResolver(§13)·
RoutePlanner(§14)·waypoint 추출(§15 간이)이 전부 이 모듈 위에서 동작한다.

진행 좌표 s = 폴리라인 시작점부터 선을 따라 잰 거리(m).
"""
from __future__ import annotations

import math

Point = tuple[float, float]


def polyline_length(pts: tuple[Point, ...]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def project_point(pts: tuple[Point, ...], p: Point) -> tuple[float, float, Point]:
    """점 p를 폴리라인에 투영 → (진행 s, 수직 거리, 투영점).

    각 선분에 클램프 투영해 최근접을 취한다 (§12.2-2·3).
    """
    best = None
    acc = 0.0
    for a, b in zip(pts, pts[1:]):
        seg = (b[0] - a[0], b[1] - a[1])
        seg_len = math.hypot(*seg)
        if seg_len <= 0.0:
            continue
        t = ((p[0] - a[0]) * seg[0] + (p[1] - a[1]) * seg[1]) / (seg_len * seg_len)
        t = min(1.0, max(0.0, t))
        q = (a[0] + seg[0] * t, a[1] + seg[1] * t)
        d = math.hypot(p[0] - q[0], p[1] - q[1])
        if best is None or d < best[1]:
            best = (acc + seg_len * t, d, q)
        acc += seg_len
    if best is None:  # 점 1개짜리 폴리라인
        d = math.hypot(p[0] - pts[0][0], p[1] - pts[0][1])
        return 0.0, d, pts[0]
    return best


def point_at(pts: tuple[Point, ...], s: float) -> Point:
    """진행 s 지점의 좌표 (범위 밖은 양 끝으로 클램프)."""
    if s <= 0.0:
        return pts[0]
    acc = 0.0
    for a, b in zip(pts, pts[1:]):
        seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
        if acc + seg_len >= s and seg_len > 0.0:
            t = (s - acc) / seg_len
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        acc += seg_len
    return pts[-1]


def heading_at(pts: tuple[Point, ...], s: float) -> float:
    """진행 s 지점의 진행 방향 (도, 0~360 — §2.7 반시계·+x 기준)."""
    if len(pts) < 2:
        return 0.0
    acc = 0.0
    for a, b in zip(pts, pts[1:]):
        seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
        if (acc + seg_len >= s or (a, b) == (pts[-2], pts[-1])) and seg_len > 0.0:
            return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360.0
        acc += seg_len
    return 0.0


def trim(pts: tuple[Point, ...], s0: float, s1: float) -> tuple[Point, ...]:
    """[s0, s1] 부분 폴리라인 (끝점은 보간). s0 ≤ s1 필수."""
    assert s0 <= s1 + 1e-9, f"trim 순서 오류: {s0} > {s1}"
    p0, p1 = point_at(pts, s0), point_at(pts, s1)
    if s1 - s0 <= 1e-9:
        return (p0,)
    out = [p0]
    acc = 0.0
    for a, b in zip(pts, pts[1:]):
        seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
        if s0 < acc < s1 and math.hypot(a[0] - out[-1][0], a[1] - out[-1][1]) > 1e-9:
            out.append(a)
        acc += seg_len
        # 마지막 꼭짓점(b)은 다음 루프의 a로 처리된다
    if math.hypot(p1[0] - out[-1][0], p1[1] - out[-1][1]) > 1e-9:
        out.append(p1)
    else:
        out[-1] = p1
    return tuple(out) if len(out) >= 2 else (p0, p1)


def resample(pts: tuple[Point, ...], spacing_m: float) -> tuple[Point, ...]:
    """일정 간격 재샘플링 (§15). 양 끝점은 항상 포함한다."""
    total = polyline_length(pts)
    if total <= 0.0 or spacing_m <= 0.0:
        return (pts[0], pts[-1]) if len(pts) >= 2 else pts
    n = max(1, int(math.ceil(total / spacing_m)))
    return tuple(point_at(pts, total * i / n) for i in range(n + 1))


def heading_diff_deg(a: float, b: float) -> float:
    """두 방위의 순환 차 (−180~+180]."""
    return (a - b + 180.0) % 360.0 - 180.0
