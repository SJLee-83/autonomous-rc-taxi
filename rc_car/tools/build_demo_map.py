"""build_demo_map.py — 시연지도 차선 그래프 YAML 생성기 (D1).

`자료/자율주행무인택시_시연지도최종안.png`(2617x1581px)를 픽셀 판독해 미터로 환산한
기하 상수로부터 차선 그래프를 **계산**해 `map/main_track_map.yaml` (rc_car 형제)을 생성한다.
좌표를 손으로 치지 않는 이유: 차선-커넥터 접점(접선 트림)은 서로 물려 있어서
수작업으로는 연속성이 반드시 깨진다. 이 스크립트가 유일한 좌표 산출 경로다.

실행:  python tools/build_demo_map.py        (rc_car/ 에서)

── 픽셀 판독 근거 (원본 2617x1581, 변환: x[m]=px*5/2617, y[m]=3-px*3/1581) ──────────
- 외곽 노란 테두리 안쪽 도로 경계: ~8px → 0.015m ≈ 0.02m
- 링 중앙 황색선 inset: 135px → 0.258m ≈ 0.26m  (좌 x=0.26 / 우 x=4.74 / 상 y=2.74 / 하 y=0.26)
- 링 안쪽 경계(=녹색 블록 외곽) inset: 270px → 0.516m ≈ 0.52m
- 중간 가로 도로: 중앙선 y=1.50 (790px), 차로 중심 y = 1.50 ± 0.13, ± 0.38
- 세로 도로: 중앙선 x=2.50 (1310px), 차로 중심 x = 2.50 ± 0.13, ± 0.38
- 차로 방향(노면 화살표 판독): 링 바깥 차로 = 반시계(상단 서→좌 남→하단 동→우 북),
  링 안쪽 차로 = 시계. 중간·세로 도로는 안쪽차로 직진+좌회전 / 바깥차로 직진+우회전.
- 정지선 실측치는 STOP_ZONES 표 참조 (픽셀 판독값 그대로).
- 차선 변경 구간 4곳 (2026-07-28 사용자 결정, 2차 수정): 화살표 규칙만으로는 완전
  분리되는 두 순환계(inner/outer)를 잇는다. 처음엔 사거리 대각 통과로 넣었으나
  "차선 변경은 두 차선이 나란히 달리는 점선 구간에서"라는 지적으로 **중간 가로 도로의
  점선 구간 중앙**으로 이동. 각 반도로에서 차선을 a/b로 분할하고 follow(유지) +
  lane_change(대각, 횡 0.25m/종 0.5m ≈ 27°) 커넥터로 잇는다. 4곳 × 양방향 = 8개.
  세로 도로는 구간이 짧고(0.2~0.5m) 구분선이 실선이라 변경 구간을 두지 않는다.
  점선 실측(픽셀): 서쪽 WB [0.84,1.68] / 서쪽 EB [0.51,1.26] / 동쪽은 대칭 —
  변경 구간은 차선쌍 겹침의 중앙 [1.005,1.505]·[3.495,3.995] (EB 서쪽·WB 동쪽은
  끝 0.2m가 실선에 걸릴 수 있음 → 실측 후 상수 조정).
- 회전 진입 차선 선택 (2026-07-28 사용자 결정, 3차 보강): 진입 도로가 2차로(가로·세로)인
  회전은 가까운 차선(기본)과 먼 차선(_far) 두 경로를 모두 등록 — 16개. 세로 도로는
  차선 변경 구간이 없어 회전 진입 선택이 유일한 차선 선택 수단이다.

── ⚠️ 실측 대기 ─────────────────────────────────────────────────────────────────
- 실제 출력물에서 노란선 꼭짓점 기준 Initialize 후 주요 지점(중앙선·블록 모서리)
  좌표를 probe_gps.py 로 재확인해야 한다. 픽셀 판독 오차 ±2cm 수준 가정.
- 회전 반경: 우회전 0.26m / 좌회전·코너 0.35m (최소 회전 반경 0.243m 이상).
  안쪽 링 코너는 도색선을 수 cm 침범할 수 있는 기하(포켓 폭 < 차폭+여유) — B3 실측 후 조정.
"""
from __future__ import annotations

import math
from pathlib import Path

# ── 기하 상수 (m) ────────────────────────────────────────────────────────────
LANE_W = 0.25            # 차로 폭 (공칭)
SPEED_LANE = 0.25        # 직선 차로 제한 속도 (vehicle.yaml max_speed 잠정치와 동일)
SPEED_STRAIGHT = 0.20    # 교차로 직진 통과
SPEED_TURN = 0.20        # 회전·차선 변경 — 0.12(protocol 예시값) → 0.20:
                         # 2026-08-01 시뮬 검증 — 코너 이탈 불변·완주 시간 단축 (turn_param_sim.py)
                         # ⚠️ 변경 시 웹팀 main_track.yaml 동기화 필수
R_RIGHT = 0.26           # 우회전 반경 (최소 회전 반경 0.243 + 여유)
R_LEFT = 0.35            # 좌회전·코너 반경
MIN_TURN_RADIUS = 0.243  # wheelbase 0.14 / tan(30°)
DEST_MIN_LEN = 0.30      # 이보다 짧은 차선(교차로 사이 스텁)은 정차 금지 (전장 0.28m)
ARC_STEP_DEG = 10.0
# 차선 변경 구간 (가로 도로 반쪽마다 1곳, 점선 구간 중앙. 종거리 0.5m)
ZONE_W = (1.505, 1.005)  # 서쪽 반: x 1.505~1.005 (WB는 1.505 진입, EB는 1.005 진입)
ZONE_E = (3.995, 3.495)  # 동쪽 반: 대칭

# 방향 벡터
E, N, W, S = (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)
HEADING = {E: 0.0, N: 90.0, W: 180.0, S: 270.0}

# ── 차선 정의: id → (종류, 고정좌표, 진행방향, 공칭 시작, 공칭 끝, 서킷 라벨) ──
# 종류 'h': y 고정(가로), 'v': x 고정(세로). 공칭 구간은 교차로 상자 경계 기준 —
# 실제 끝점은 커넥터 접선이 자동으로 다듬는다(트림). 서킷 라벨은 검증으로 재확인된다.
LANES = {
    # 링 바깥 차로 (반시계)
    "top_outer_wb_e":   ("h", 2.86, W, 4.86, 3.01, "outer"),
    "top_outer_wb_w":   ("h", 2.86, W, 1.99, 0.14, "outer"),
    "left_outer_sb_n":  ("v", 0.14, S, 2.86, 2.01, "outer"),
    "left_outer_sb_s":  ("v", 0.14, S, 0.99, 0.14, "outer"),
    "bot_outer_eb_w":   ("h", 0.14, E, 0.14, 1.99, "outer"),
    "bot_outer_eb_e":   ("h", 0.14, E, 3.01, 4.86, "outer"),
    "right_outer_nb_s": ("v", 4.86, N, 0.14, 0.99, "outer"),
    "right_outer_nb_n": ("v", 4.86, N, 2.01, 2.86, "outer"),
    # 링 안쪽 차로 (시계)
    "top_inner_eb_w":   ("h", 2.61, E, 0.39, 1.99, "inner"),
    "top_inner_eb_e":   ("h", 2.61, E, 3.01, 4.61, "inner"),
    "right_inner_sb_n": ("v", 4.61, S, 2.61, 2.01, "inner"),
    "right_inner_sb_s": ("v", 4.61, S, 0.99, 0.39, "inner"),
    "bot_inner_wb_e":   ("h", 0.39, W, 4.61, 3.01, "inner"),
    "bot_inner_wb_w":   ("h", 0.39, W, 1.99, 0.39, "inner"),
    "left_inner_nb_s":  ("v", 0.39, N, 0.39, 0.99, "inner"),
    "left_inner_nb_n":  ("v", 0.39, N, 2.01, 2.61, "inner"),
    # 중간 가로 도로 (서행 2 / 동행 2) — 2026-08-04: a/b 분할·이음매 존 폐지, 반도로당
    # 단일 차선으로 병합. 차선 변경 기동이 사라져(사용자 결정 — 짧은 이음매에서 대각
    # 이동이 인도 침입 유발) 이음매 구간이 일반 차선이 됐고, 정차도 가능해졌다.
    # inner↔outer 다리는 회전 진입 차선 선택(_far) 16개가 전담한다.
    "mid_wb1_e":        ("h", 1.88, W, 4.48, 3.01, "inner"),
    "mid_wb1_w":        ("h", 1.88, W, 1.99, 0.52, "inner"),
    "mid_wb2_e":        ("h", 1.63, W, 4.48, 3.01, "outer"),
    "mid_wb2_w":        ("h", 1.63, W, 1.99, 0.52, "outer"),
    "mid_eb1_w":        ("h", 1.37, E, 0.52, 1.99, "outer"),
    "mid_eb1_e":        ("h", 1.37, E, 3.01, 4.48, "outer"),
    "mid_eb2_w":        ("h", 1.12, E, 0.52, 1.99, "inner"),
    "mid_eb2_e":        ("h", 1.12, E, 3.01, 4.48, "inner"),
    # 세로 도로 (남행 2 / 북행 2)
    "vert_sb1_n":       ("v", 2.12, S, 2.48, 2.01, "inner"),
    "vert_sb1_s":       ("v", 2.12, S, 0.99, 0.52, "inner"),
    "vert_sb2_n":       ("v", 2.37, S, 2.48, 2.01, "outer"),
    "vert_sb2_s":       ("v", 2.37, S, 0.99, 0.52, "outer"),
    "vert_nb1_s":       ("v", 2.63, N, 0.52, 0.99, "outer"),
    "vert_nb1_n":       ("v", 2.63, N, 2.01, 2.48, "outer"),
    "vert_nb2_s":       ("v", 2.88, N, 0.52, 0.99, "inner"),
    "vert_nb2_n":       ("v", 2.88, N, 2.01, 2.48, "inner"),
}

# ── 커넥터 정의: id → (진입 차선, 진출 차선, maneuver) ───────────────────────
# maneuver: straight | left | right. 반경은 left→R_LEFT, right→R_RIGHT.
CONNECTORS = {
    # 링 코너 (바깥=좌회전 곡선, 안쪽=우회전 곡선)
    "corner_nw_outer": ("top_outer_wb_w", "left_outer_sb_n", "left"),
    "corner_sw_outer": ("left_outer_sb_s", "bot_outer_eb_w", "left"),
    "corner_se_outer": ("bot_outer_eb_e", "right_outer_nb_s", "left"),
    "corner_ne_outer": ("right_outer_nb_n", "top_outer_wb_e", "left"),
    "corner_nw_inner": ("left_inner_nb_n", "top_inner_eb_w", "right"),
    "corner_ne_inner": ("top_inner_eb_e", "right_inner_sb_n", "right"),
    "corner_se_inner": ("right_inner_sb_s", "bot_inner_wb_e", "right"),
    "corner_sw_inner": ("bot_inner_wb_w", "left_inner_nb_s", "right"),
    # 중앙 사거리 (안쪽차로 직+좌 / 바깥차로 직+우 — 노면 화살표)
    "ix_sb1_straight": ("vert_sb1_n", "vert_sb1_s", "straight"),
    "ix_sb1_right":    ("vert_sb1_n", "mid_wb1_w", "right"),
    "ix_sb2_straight": ("vert_sb2_n", "vert_sb2_s", "straight"),
    "ix_sb2_left":     ("vert_sb2_n", "mid_eb1_e", "left"),
    "ix_nb1_straight": ("vert_nb1_s", "vert_nb1_n", "straight"),
    "ix_nb1_left":     ("vert_nb1_s", "mid_wb2_w", "left"),
    "ix_nb2_straight": ("vert_nb2_s", "vert_nb2_n", "straight"),
    "ix_nb2_right":    ("vert_nb2_s", "mid_eb2_e", "right"),
    "ix_wb1_straight": ("mid_wb1_e", "mid_wb1_w", "straight"),
    "ix_wb1_right":    ("mid_wb1_e", "vert_nb2_n", "right"),
    "ix_wb2_straight": ("mid_wb2_e", "mid_wb2_w", "straight"),
    "ix_wb2_left":     ("mid_wb2_e", "vert_sb2_s", "left"),
    "ix_eb1_straight": ("mid_eb1_w", "mid_eb1_e", "straight"),
    "ix_eb1_left":     ("mid_eb1_w", "vert_nb1_n", "left"),
    "ix_eb2_straight": ("mid_eb2_w", "mid_eb2_e", "straight"),
    "ix_eb2_right":    ("mid_eb2_w", "vert_sb1_s", "right"),
    # (2026-08-04) 차선 변경 존(zone_*) 16개 폐지 — 이음매 구간은 병합 차선에 흡수.
    # inner↔outer 다리는 아래 _far 회전 진입 선택이 전담한다.
    # 상단 T (세로 도로 북단 x 상단 도로)
    "tee_n_eb_straight": ("top_inner_eb_w", "top_inner_eb_e", "straight"),
    "tee_n_eb_right":    ("top_inner_eb_w", "vert_sb1_n", "right"),
    "tee_n_wb_straight": ("top_outer_wb_e", "top_outer_wb_w", "straight"),
    "tee_n_wb_left":     ("top_outer_wb_e", "vert_sb2_n", "left"),
    "tee_n_nb1_left":    ("vert_nb1_n", "top_outer_wb_w", "left"),
    "tee_n_nb2_right":   ("vert_nb2_n", "top_inner_eb_e", "right"),
    # 하단 T
    "tee_s_wb_straight": ("bot_inner_wb_e", "bot_inner_wb_w", "straight"),
    "tee_s_wb_right":    ("bot_inner_wb_e", "vert_nb2_s", "right"),
    "tee_s_eb_straight": ("bot_outer_eb_w", "bot_outer_eb_e", "straight"),
    "tee_s_eb_left":     ("bot_outer_eb_w", "vert_nb1_s", "left"),
    "tee_s_sb1_right":   ("vert_sb1_s", "bot_inner_wb_w", "right"),
    "tee_s_sb2_left":    ("vert_sb2_s", "bot_outer_eb_e", "left"),
    # 좌측 T (좌측 가장자리 x 중간 도로)
    "tee_w_sb_straight": ("left_outer_sb_n", "left_outer_sb_s", "straight"),
    "tee_w_sb_left":     ("left_outer_sb_n", "mid_eb1_w", "left"),
    "tee_w_nb_straight": ("left_inner_nb_s", "left_inner_nb_n", "straight"),
    "tee_w_nb_right":    ("left_inner_nb_s", "mid_eb2_w", "right"),
    "tee_w_wb1_right":   ("mid_wb1_w", "left_inner_nb_n", "right"),
    "tee_w_wb2_left":    ("mid_wb2_w", "left_outer_sb_s", "left"),
    # 우측 T
    "tee_e_sb_straight": ("right_inner_sb_n", "right_inner_sb_s", "straight"),
    "tee_e_sb_right":    ("right_inner_sb_n", "mid_wb1_e", "right"),
    "tee_e_nb_straight": ("right_outer_nb_s", "right_outer_nb_n", "straight"),
    "tee_e_nb_left":     ("right_outer_nb_s", "mid_wb2_e", "left"),
    "tee_e_eb1_left":    ("mid_eb1_e", "right_outer_nb_n", "left"),
    "tee_e_eb2_right":   ("mid_eb2_e", "right_inner_sb_s", "right"),
    # 회전 진입 차선 선택 (2026-07-28 사용자 결정): 진입 도로가 2차로면 회전 시
    # 두 차선 중 하나를 고를 수 있다. 기본(위 항목) = 가까운 차선, _far = 먼 차선.
    # 반경은 기본 회전과 동일 — 착지 깊이가 같아져 대상 차선 트림이 대칭이 된다.
    # 진출 차선 규칙(노면 화살표: 안쪽=좌회전, 바깥=우회전)은 그대로다.
    # 링 도로는 방향당 1차로라 해당 없음.
    "ix_sb1_right_far":   ("vert_sb1_n", "mid_wb2_w", "right"),
    "ix_sb2_left_far":    ("vert_sb2_n", "mid_eb2_e", "left"),
    "ix_nb1_left_far":    ("vert_nb1_s", "mid_wb1_w", "left"),
    "ix_nb2_right_far":   ("vert_nb2_s", "mid_eb1_e", "right"),
    "ix_wb1_right_far":   ("mid_wb1_e", "vert_nb1_n", "right"),
    "ix_wb2_left_far":    ("mid_wb2_e", "vert_sb1_s", "left"),
    "ix_eb1_left_far":    ("mid_eb1_w", "vert_nb2_n", "left"),
    "ix_eb2_right_far":   ("mid_eb2_w", "vert_sb2_s", "right"),
    "tee_n_eb_right_far": ("top_inner_eb_w", "vert_sb2_n", "right"),
    "tee_n_wb_left_far":  ("top_outer_wb_e", "vert_sb1_n", "left"),
    "tee_s_wb_right_far": ("bot_inner_wb_e", "vert_nb1_s", "right"),
    "tee_s_eb_left_far":  ("bot_outer_eb_w", "vert_nb2_s", "left"),
    "tee_w_sb_left_far":  ("left_outer_sb_n", "mid_eb2_w", "left"),
    "tee_w_nb_right_far": ("left_inner_nb_s", "mid_eb1_w", "right"),
    "tee_e_sb_right_far": ("right_inner_sb_n", "mid_wb2_e", "right"),
    "tee_e_nb_left_far":  ("right_outer_nb_s", "mid_wb1_e", "left"),
}

# ── 녹색 블록 (승객 호출 영역) ───────────────────────────────────────────────
# 동/서 curb의 인접 차선은 교차로 사이 스텁(<0.4m)이라 정차 불가 → 빈 목록.
BLOCKS = {
    "block_nw": ((0.52, 2.01, 1.99, 2.48),
                 {"north": ["top_inner_eb_w"], "south": ["mid_wb1_w"]}),
    "block_ne": ((3.01, 2.01, 4.48, 2.48),
                 {"north": ["top_inner_eb_e"], "south": ["mid_wb1_e"]}),
    "block_sw": ((0.52, 0.52, 1.99, 0.99),
                 {"north": ["mid_eb2_w"], "south": ["bot_inner_wb_w"]}),
    "block_se": ((3.01, 0.52, 4.48, 0.99),
                 {"north": ["mid_eb2_e"], "south": ["bot_inner_wb_e"]}),
}

# ── 정지선 (픽셀 판독 실측치) ────────────────────────────────────────────────
STOP_ZONES = {
    # 중앙 사거리 접근
    "stop_ix_sb1": ("vert_sb1_n", (2.12, 2.26)),
    "stop_ix_sb2": ("vert_sb2_n", (2.37, 2.26)),
    "stop_ix_nb1": ("vert_nb1_s", (2.63, 0.75)),
    "stop_ix_nb2": ("vert_nb2_s", (2.88, 0.75)),
    "stop_ix_wb1": ("mid_wb1_e", (3.34, 1.88)),
    "stop_ix_wb2": ("mid_wb2_e", (3.34, 1.63)),
    "stop_ix_eb1": ("mid_eb1_w", (1.66, 1.37)),
    "stop_ix_eb2": ("mid_eb2_w", (1.66, 1.12)),
    # 상단 T 접근
    "stop_tee_n_eb":  ("top_inner_eb_w", (1.99, 2.61)),
    "stop_tee_n_wb":  ("top_outer_wb_e", (3.01, 2.86)),
    "stop_tee_n_nb1": ("vert_nb1_n", (2.63, 2.49)),
    "stop_tee_n_nb2": ("vert_nb2_n", (2.88, 2.49)),
    # 하단 T 접근
    "stop_tee_s_wb":  ("bot_inner_wb_e", (3.01, 0.39)),
    "stop_tee_s_eb":  ("bot_outer_eb_w", (1.99, 0.14)),
    "stop_tee_s_sb1": ("vert_sb1_s", (2.12, 0.52)),
    "stop_tee_s_sb2": ("vert_sb2_s", (2.37, 0.52)),
    # 좌측 T 접근
    "stop_tee_w_sb": ("left_outer_sb_n", (0.14, 2.03)),
    "stop_tee_w_nb": ("left_inner_nb_s", (0.39, 1.00)),
    "stop_tee_w_wb1": ("mid_wb1_w", (0.52, 1.88)),
    "stop_tee_w_wb2": ("mid_wb2_w", (0.52, 1.63)),
    # 우측 T 접근
    "stop_tee_e_sb": ("right_inner_sb_n", (4.61, 2.02)),
    "stop_tee_e_nb": ("right_outer_nb_s", (4.86, 1.00)),
    "stop_tee_e_eb1": ("mid_eb1_e", (4.48, 1.37)),
    "stop_tee_e_eb2": ("mid_eb2_e", (4.48, 1.12)),
}


# ── 기하 계산 ────────────────────────────────────────────────────────────────
def _line_point(lane: tuple, s: float) -> tuple[float, float]:
    """진행 좌표 s(= dot(P, dir))를 실좌표로. h차선이면 x=±s, v차선이면 y=±s."""
    kind, fixed, d, *_ = lane
    if kind == "h":
        return (s * (1 if d == E else -1), fixed)
    return (fixed, s * (1 if d == N else -1))


def _s_of(lane: tuple, p: tuple[float, float]) -> float:
    _, _, d, *_ = lane
    return p[0] * d[0] + p[1] * d[1]


def _corner_point(a: tuple, b: tuple) -> tuple[float, float]:
    """수직 교차하는 두 차선 중심선의 교점."""
    (ka, fa, *_), (kb, fb, *_) = a, b
    assert ka != kb, "회전 커넥터는 가로/세로 차선을 잇는다"
    return (fa, fb) if ka == "v" else (fb, fa)


def _turn_geometry(a: tuple, b: tuple, r: float):
    """접선점 T0(진입선상)·T1(진출선상)과 호 중심. T0 = P - da*r, T1 = P + db*r."""
    da, db = a[2], b[2]
    p = _corner_point(a, b)
    t0 = (p[0] - da[0] * r, p[1] - da[1] * r)
    t1 = (p[0] + db[0] * r, p[1] + db[1] * r)
    center = (p[0] + (db[0] - da[0]) * r, p[1] + (db[1] - da[1]) * r)
    ccw = da[0] * db[1] - da[1] * db[0] > 0  # 좌회전이면 반시계 호
    return t0, t1, center, ccw


def _arc_points(t0, t1, center, r: float, ccw: bool) -> list[tuple[float, float]]:
    a0 = math.atan2(t0[1] - center[1], t0[0] - center[0])
    a1 = math.atan2(t1[1] - center[1], t1[0] - center[0])
    sweep = (a1 - a0) % (2 * math.pi) if ccw else -((a0 - a1) % (2 * math.pi))
    steps = max(2, int(abs(math.degrees(sweep)) / ARC_STEP_DEG))
    return [(center[0] + r * math.cos(a0 + sweep * i / steps),
             center[1] + r * math.sin(a0 + sweep * i / steps))
            for i in range(steps + 1)]


def build():
    radius = {"straight": None, "follow": None, "lane_change": None,
              "right": R_RIGHT, "left": R_LEFT}

    # 1) 트림: 각 차선의 실제 시작/끝 진행좌표
    lanes = _lanes()
    start_s = {lid: _s_of(l, _nominal(l, "start")) for lid, l in lanes.items()}
    end_s = {lid: _s_of(l, _nominal(l, "end")) for lid, l in lanes.items()}

    geo = {}
    for cid, (fr, to, man) in CONNECTORS.items():
        r = radius[man]
        if r is None:
            continue
        t0, t1, center, ccw = _turn_geometry(lanes[fr], lanes[to], r)
        geo[cid] = (t0, t1, center, ccw, r)
        end_s[fr] = min(end_s[fr], _s_of(lanes[fr], t0))       # 이른 접선이 차선 끝을 당긴다
        start_s[to] = max(start_s[to], _s_of(lanes[to], t1))   # 깊은 착지가 차선 시작을 민다

    lane_pts = {}
    for lid, l in lanes.items():
        p0, p1 = _line_point(l, start_s[lid]), _line_point(l, end_s[lid])
        length = end_s[lid] - start_s[lid]
        assert length > 0.05, f"{lid}: 트림 후 길이 {length:.3f}m — 과도한 트림"
        lane_pts[lid] = [p0, p1]

    # 2) 커넥터 폴리라인: 차선 끝 → (직선 리드) → 호 → (직선 리드) → 차선 시작
    conn_pts = {}
    for cid, (fr, to, man) in CONNECTORS.items():
        a_end = lane_pts[fr][-1]
        b_start = lane_pts[to][0]
        if radius[man] is None:  # straight·follow·lane_change — 직선 (변경은 대각선)
            pts = [a_end, b_start]
        else:
            t0, t1, center, ccw, r = geo[cid]
            pts = [a_end] + _arc_points(t0, t1, center, r, ccw) + [b_start]
        conn_pts[cid] = _dedup(pts)

    # 3) 검증: 연속성 / 경계 / 서킷 분할
    for cid, (fr, to, _) in CONNECTORS.items():
        assert _close(conn_pts[cid][0], lane_pts[fr][-1]), f"{cid}: 시작 불연속"
        assert _close(conn_pts[cid][-1], lane_pts[to][0]), f"{cid}: 끝 불연속"
    for pts in list(lane_pts.values()) + list(conn_pts.values()):
        for x, y in pts:
            assert -0.001 <= x <= 5.001 and -0.001 <= y <= 3.001, f"맵 범위 밖: {(x, y)}"

    successors = {lid: [] for lid in lanes}
    for cid, (fr, _, _) in CONNECTORS.items():
        successors[fr].append(cid)

    circuits = _verify_circuits(lanes, successors)

    for _, curbs in BLOCKS.values():
        for lane_ids in curbs.values():
            for lid in lane_ids:
                assert end_s[lid] - start_s[lid] >= DEST_MIN_LEN, f"{lid}: 정차 구간이 짧음"

    return lanes, lane_pts, conn_pts, successors, circuits, start_s, end_s


def _lanes():
    return LANES


def _nominal(l: tuple, which: str) -> tuple[float, float]:
    kind, fixed, d, n_start, n_end, _ = l
    v = n_start if which == "start" else n_end
    return (v, fixed) if kind == "h" else (fixed, v)


def _dedup(pts, eps=1e-9):
    out = [pts[0]]
    for p in pts[1:]:
        if abs(p[0] - out[-1][0]) > eps or abs(p[1] - out[-1][1]) > eps:
            out.append(p)
    return out


def _close(p, q, tol=1e-6):
    return abs(p[0] - q[0]) <= tol and abs(p[1] - q[1]) <= tol


def _verify_circuits(lanes, successors) -> dict[str, str]:
    """서킷 라벨과 연결성을 도달성으로 검증한다.

    두 순환계를 잇는 "다리"는 두 종류다: lane_change(점선 구간 차선 변경)와
    _far 회전(먼 차선 진입 — 인접 차선은 서킷이 엇갈리므로 필연적으로 서킷을 넘는다).

    1) 다리를 제외한 부분그래프는 inner/outer 로 완전 분리된다
       (D1 원 발견의 보존 — 화살표+가까운 차선 진입 규칙만으로는 이어지지 않는다)
    2) 다리 포함 전체 그래프는 강연결이다 — 어느 차선에서든 어느 차선으로든 도달
    """
    def reach(seed, adj):
        seen, stack = {seed}, [seed]
        while stack:
            for nxt in adj[stack.pop()]:
                if nxt not in seen:
                    seen.add(nxt)
                    stack.append(nxt)
        return seen

    def is_bridge(cid: str) -> bool:
        return CONNECTORS[cid][2] == "lane_change" or cid.endswith("_far")

    def make_adj(include_bridges: bool):
        return {lid: {CONNECTORS[c][1] for c in succ
                      if include_bridges or not is_bridge(c)}
                for lid, succ in successors.items()}

    # 1) 다리 제외 — 라벨의 근거가 되는 두 순환계 분리
    part = make_adj(include_bridges=False)
    inner_seed, outer_seed = "top_inner_eb_w", "top_outer_wb_e"
    inner, outer = reach(inner_seed, part), reach(outer_seed, part)
    assert inner.isdisjoint(outer), "다리 제외 시에도 서킷이 섞임 — 화살표 판독 재확인 필요"
    assert inner | outer == set(lanes), "어느 서킷에도 속하지 않는 차선 존재"
    computed = {lid: ("inner" if lid in inner else "outer") for lid in lanes}
    for lid, l in lanes.items():
        assert computed[lid] == l[5], f"{lid}: 서킷 라벨 불일치 (정의 {l[5]} / 계산 {computed[lid]})"

    # 다리가 실제로 서킷을 넘는지 (같은 서킷 내부 다리 = 정의 오류)
    for cid, (fr, to, _) in CONNECTORS.items():
        if is_bridge(cid):
            assert computed[fr] != computed[to], f"{cid}: 서킷 내부 다리 (정의 재확인)"

    # 2) 다리 포함 — 전체 강연결 (모든 차선 쌍 도달 가능해야 경로 계획이 성립)
    full = make_adj(include_bridges=True)
    all_lanes = set(lanes)
    for lid in lanes:
        assert reach(lid, full) == all_lanes, f"{lid}에서 도달 불가한 차선 존재"
    return computed


# ── YAML 출력 ────────────────────────────────────────────────────────────────
def _fmt_pt(p) -> str:
    return f"[{round(p[0], 4):g}, {round(p[1], 4):g}]"


def emit(out_path: Path) -> None:
    lanes, lane_pts, conn_pts, successors, circuits, start_s, end_s = build()
    n_inner = sum(1 for c in circuits.values() if c == "inner")

    L = []
    L.append("# main_track_map.yaml — 시연지도 차선 그래프 (단일 소스)")
    L.append("#")
    L.append("# 🔴 이 파일은 차량(rc_car)과 관제 서버가 공유하는 유일한 맵 데이터다. 사본 금지.")
    L.append("# 생성: rc_car/tools/build_demo_map.py (직접 수정하지 말고 생성기를 고쳐 재생성)")
    L.append("# 근거: 자료/자율주행무인택시_시연지도최종안.png 픽셀 판독 (±2cm 추정 오차, 실측 대기)")
    L.append("#")
    L.append("# 두 순환계와 다리 (2026-07-28 결정):")
    L.append(f"#   화살표+가까운 차선 진입 규칙만으로는 inner {n_inner} / outer {len(lanes) - n_inner} 차선이 완전 분리된다.")
    L.append("#   두 순환계를 잇는 다리는 두 종류다:")
    L.append("#   ① (2026-08-04 폐지) 점선 구간 차선 변경 — 짧은 이음매에서 대각 기동이")
    L.append("#      인도 침입을 유발해 제거. 이음매는 병합 차선에 흡수(정차 가능)")
    L.append("#   ② 회전 진입 차선 선택(_far) 16개 — 2차로 도로로 회전 시 먼 차선으로도 진입")
    L.append(f"#   → 전체 {len(lanes)}차선 강연결: 목적지는 destination_allowed 차선이면 어디든 가능.")
    L.append("#   → 다리를 전부 지우면 다시 분리된다 — 로더·테스트가 강연결을 검증한다.")
    L.append("#")
    L.append("# 확장(명세서 §11.2에 없는 것): lanes.*.circuit(링 계열 참고용), connectors.*.radius_m")
    L.append("")
    L.append("map:")
    L.append("  id: main_track")
    L.append("  width_m: 5.0")
    L.append("  height_m: 3.0")
    L.append("  origin: bottom_left")
    L.append("  image: 자료/자율주행무인택시_시연지도최종안.png")
    L.append("")
    L.append("vehicle_rules:")
    L.append("  lane_change_enabled: false")
    L.append("  reverse_enabled: false")
    L.append("")
    L.append("lanes:")
    for lid, l in lanes.items():
        d = l[2]
        L.append(f"  {lid}:")
        L.append("    kind: lane")
        L.append(f"    width_m: {LANE_W}")
        L.append(f"    heading_hint_deg: {HEADING[d]:g}")
        L.append(f"    speed_limit_mps: {SPEED_LANE}")
        dest = (end_s[lid] - start_s[lid]) >= DEST_MIN_LEN
        L.append(f"    destination_allowed: {'true' if dest else 'false'}")
        L.append(f"    circuit: {circuits[lid]}")
        L.append("    centerline:")
        for p in lane_pts[lid]:
            L.append(f"      - {_fmt_pt(p)}")
        L.append("    successors:")
        for cid in successors[lid]:
            L.append(f"      - {cid}")
    L.append("")
    L.append("connectors:")
    conn_radius = {"straight": None, "follow": None, "lane_change": None,
                   "right": R_RIGHT, "left": R_LEFT}
    conn_speed = {"straight": SPEED_STRAIGHT, "follow": SPEED_LANE,
                  "lane_change": SPEED_TURN, "right": SPEED_TURN, "left": SPEED_TURN}
    for cid, (fr, to, man) in CONNECTORS.items():
        r = conn_radius[man]
        L.append(f"  {cid}:")
        L.append("    kind: connector")
        L.append(f"    maneuver: {man}")
        L.append(f"    speed_limit_mps: {conn_speed[man]}")
        if r:
            L.append(f"    radius_m: {r}")
        L.append("    centerline:")
        for p in conn_pts[cid]:
            L.append(f"      - {_fmt_pt(p)}")
        L.append(f"    successor: {to}")
    L.append("")
    L.append("blocks:")
    for bid, ((x0, y0, x1, y1), curbs) in BLOCKS.items():
        L.append(f"  {bid}:")
        L.append("    polygon:")
        for p in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            L.append(f"      - {_fmt_pt(p)}")
        L.append("    curb_edges:")
        segs = {"north": ((x0, y1), (x1, y1)), "south": ((x0, y0), (x1, y0)),
                "west": ((x0, y0), (x0, y1)), "east": ((x1, y0), (x1, y1))}
        for side in ("north", "east", "south", "west"):
            lane_ids = curbs.get(side, [])
            L.append(f"      {side}:")
            a, b = segs[side]
            L.append(f"        segment: [{_fmt_pt(a)}, {_fmt_pt(b)}]")
            if lane_ids:
                L.append("        adjacent_destination_lanes:")
                for lid in lane_ids:
                    L.append(f"          - {lid}")
            else:
                L.append("        adjacent_destination_lanes: []  # 인접 차선이 교차로 스텁 — 정차 불가")
    L.append("")
    L.append("stop_line_zones:")
    for zid, (lane_id, center) in STOP_ZONES.items():
        L.append(f"  {zid}:")
        L.append(f"    approach_lane_id: {lane_id}")
        L.append(f"    expected_center: {_fmt_pt(center)}")
        L.append("    activation_distance_m: 0.60")
    L.append("")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(L), encoding="utf-8")
    print(f"생성 완료: {out_path}")
    print(f"  lanes={len(lanes)} (inner {n_inner} / outer {len(lanes) - n_inner}) "
          f"connectors={len(CONNECTORS)} blocks={len(BLOCKS)} stop_zones={len(STOP_ZONES)}")


def render_overlay(yaml_path: Path, image_path: Path, out_path: Path) -> None:
    """차선 그래프를 지도 이미지 위에 겹쳐 그린다 (검수용 — PIL 필요, 선택 기능).

    inner=하늘색 / outer=자홍 / 직진=주황 / 좌회전=파랑 / 우회전=빨강 /
    차선 유지(follow)=회색 / 차선 변경(lane_change)=노랑.
    """
    import yaml
    from PIL import Image, ImageDraw  # 선택 의존성 — 렌더 시에만

    img = Image.open(image_path).convert("RGB")
    w, h = img.size
    draw = ImageDraw.Draw(img)

    def px(p):
        return (p[0] / 5.0 * w, h - p[1] / 3.0 * h)

    data = yaml.safe_load(open(yaml_path, encoding="utf-8"))
    for lane in data["lanes"].values():
        color = (0, 220, 255) if lane["circuit"] == "inner" else (255, 0, 255)
        pts = [px(p) for p in lane["centerline"]]
        draw.line(pts, fill=color, width=7)
        (x0, y0), (x1, y1) = pts[-2], pts[-1]
        ang = math.atan2(y1 - y0, x1 - x0)
        for da in (2.6, -2.6):  # 진행 방향 화살촉
            draw.line([(x1, y1), (x1 + 22 * math.cos(ang + da),
                                  y1 + 22 * math.sin(ang + da))], fill=color, width=7)
    palette = {"straight": (255, 160, 0), "left": (60, 120, 255), "right": (255, 60, 60),
               "follow": (190, 190, 190), "lane_change": (255, 230, 0)}
    for conn in data["connectors"].values():
        draw.line([px(p) for p in conn["centerline"]],
                  fill=palette[conn["maneuver"]], width=5)
    img.resize((1600, int(1600 * h / w))).save(out_path)
    print(f"오버레이 저장: {out_path}")


if __name__ == "__main__":
    import sys

    # rc_car·map 형제 전제 (§0-38 폴더 재편) — 노트북은 차량파트/, 보드는 ~ 가 root가 된다.
    # (구버전은 재편 전 "코드/map"을 가리켜 보드에서 /home/코드 생성 시도로 죽었다 — 2026-08-04 수정)
    root = Path(__file__).resolve().parents[2]
    yaml_out = root / "map" / "main_track_map.yaml"
    emit(yaml_out)
    if "--overlay" in sys.argv:
        render_overlay(yaml_out,
                       root.parent / "자료" / "자율주행무인택시_시연지도최종안.png",
                       root / "map" / "main_track_map_overlay.png")
