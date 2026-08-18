"""LaneFollower — 차선 번호 point-to-point 주행 + 2단 회전 (2026-08-06 재설계 §0-45).

이 모듈이 **현재 기본 주행 로직**이다. 이전의 원호 추종(가상 라인 트레이싱)은
control/legacy/waypoint_follower.py 로 격리되었고 환경변수로만 되살릴 수 있다.

구조
    직선 = 차선의 시작점 → 끝점 **point-to-point**.
           맵의 32개 차선 centerline 이 전부 정확히 2점(직선)이라 근사가 아니다.
    회전 = 2단.
           1차(GPS·좌표): 차선 끝까지 arm_distance 안에 들면 ARMED — "곧 우회전이다"를 인지
           2차(비전)   : ARMED 중 지정 노면표시가 버드아이 하단에 들어오면 **그 순간 개시**
           회전 중     : 표(TurnTable)의 고정 조향각 유지 (원호를 추종하지 않는다)
           종료        : 다음 차선 방위와 exit_align_deg 안으로 정렬되면 종료 → 직선 복귀

상태기계
    FOLLOW ──(차선 끝 arm_distance)──▶ ARMED ──(비전 트리거 or 좌표 폴백)──▶ TURNING
       ▲                                                                      │
       └──────────────(다음 차선 정렬 or max_turn_m)─────────────────────────┘

무중단 폴백: 비전이 죽으면 트리거가 영원히 안 뜨고, fallback_at_lane_end 로
차선 끝(=좌표 단독)에서 회전이 개시된다. 비전 0으로도 주행은 끝까지 간다.

seg heading 대체 (2026-08-07 신설 — control.yaml seg_heading)
    지정 차선의 직선 구간에서 **추종에 쓰는 차량 방위를 GPS 대신 비전으로 갈아끼운다**:
        방위 = 차선 공칭 방위 − seg.heading_error_deg
    근거(0806 1차 주행 실측): 차선 4에서 GPS heading 이 +37.3° 로 튄 순간 seg 는 +6.8°
    였고 실물은 seg 가 맞았다. 그 상태로 _pursue 가 돌면 오차 −38.8° → **우 풀락 +30°**
    가 나가 차가 인도로 밀린다 (11회 중 6회 재현). 같은 tick 에 seg 방위를 쓰면
    오차 +5.3° → 좌 5.3° 로 정반대 부호의 온건한 지령이 된다.
    ⚠️ **횡방향만** 바꾼다. 차선 끝까지 남은 거리·회전 개시·도착 판정은 GPS 그대로다
       (좌표 폴백 개시 재현성 ±1.7cm 는 어제 지표 중 최고 — 건드리지 않는다).
    ⚠️ seg 의 **heading 부호만** 사용한다. lateral_offset 부호는 아직 미검증이라 안 쓴다.
    seg invalid·차선 밖·회전 중에는 자동으로 GPS 로 되돌아간다 (기존 동작).

살려서 가져온 실차 대책 (전부 2026-08-03~05 실주행 근거)
    - 재출발 램프(departure_ramp_distance_m): 정지 heading 스윙 → 인도 침범 방지
    - 회전 전후 turn_throttle 고정: 계단 관성·회전 중 토크 강하 대책
    - 코너 출구 홀드(corner_exit_hold_distance_m): 회전 마무리 토크 유지
    - 지나침 정지(PASSED_HYSTERESIS_M): 최종 목표를 스치면 되돌기 배회 대신 정차
    - seg 보정 게이트: 직선(FOLLOW) 구간에서만 허용 — 코너 중 보정 누수 차단

이 모듈은 순수 계산만 한다 — 하드웨어 출력은 DrivingWorker 가 SafetySupervisor.apply()로
보낸다 (모터 출력 단일 통과점 원칙).
"""
from __future__ import annotations

import logging
import math
import time

from core.models import ControlCommand, STOP_COMMAND
from mapping import geometry
from navigation import lane_route

log = logging.getLogger("control.lane")

PASSED_HYSTERESIS_M = 0.08   # 최종 목표 최근접 후 이만큼 멀어지면 '지나침' 확정
                             # (GPS sd 1cm 대비 여유 — 2026-08-05 저녁 배회 사고)

FOLLOW, ARMED, TURNING = "FOLLOW", "ARMED", "TURNING"

# start_offset 소모용 모델 속도 (2026-08-06 4차 주행 — GPS 좌표 이동량 적산은 동편 왜곡
# 구역에서 신뢰 불가: 발화 후 0.50m 명령이 실물에선 조기 개시 = 인도 침범 반복).
# 발화 시점부터는 코너 창이라 turn_throttle(0.5) ≈ 실측 0.094 m/s 로 균일하다.
OFFSET_SPEED_MPS = 0.094   # turn_throttle 0.5 실측 (0.8 시절 0.157 에서 원복, 2026-08-07 저녁)
                           # ⚠️ 현재 표에 start_offset_m 을 쓰는 회전이 없어 사실상 비활성이다.
                           #    다시 쓸 때는 turn_throttle 과 같이 맞춰야 한다.
# compute 호출 간격이 이보다 크면 그 사이를 적산하지 않는다 — 유실 정지 등으로
# 차가 서 있던 시간이 오프셋을 갉아먹는 것을 막는다 (제어 주기 0.02s 의 7배 여유)
OFFSET_GAP_MAX_S = 0.15

# 회전 종료용 코스각 산출 베이스라인 (2026-08-07 — _turn_course_deg 참조).
# GPS 잡음 1cm 기준 각도 오차 ≈ atan(0.01/0.15) = 3.8° 로, exit_align 12~15° 보다 충분히 작다.
# 더 짧게 잡으면 잡음이, 더 길게 잡으면 회전 초반 방향이 섞여 종료가 늦어진다.
COURSE_WINDOW_M = 0.15


def _path_len(pts) -> float:
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def _course_deg(path, window_m: float) -> float | None:
    """자취 끝점에서 window_m 만큼 물러난 점까지의 방위 — 위치만 쓴다 (GPS heading 무관).

    물러날 거리를 못 채우면 None (정지·저속 구간). 짧은 베이스라인은 GPS 잡음이
    각도로 증폭되므로, 확보되기 전에는 판단하지 않는 편이 안전하다.
    """
    if len(path) < 2:
        return None
    hx, hy = path[-1]
    for i in range(len(path) - 2, -1, -1):
        dx, dy = hx - path[i][0], hy - path[i][1]
        if math.hypot(dx, dy) >= window_m:
            return math.degrees(math.atan2(dy, dx)) % 360.0
    return None


class LaneFollower:
    def __init__(self, control_cfg: dict, turn_table, max_wheel_deg: float,
                 max_speed_mps: float, wheelbase_m: float = 0.14, marks=None, seg=None):
        self._table = turn_table
        self._marks = marks                  # MarkAdapter | None (None = 좌표 폴백 전용)
        self._seg = seg                      # SegAdapter | None — heading 대체 전용 소비
        # seg heading 대체 설정 (모듈 docstring 참조). 키가 없으면 전부 기존 동작.
        sh = dict(control_cfg.get("seg_heading") or {})
        self._seg_hdg_lanes = frozenset(int(n) for n in (sh.get("lane_nums") or ()))
        self._disagree_warn = float(sh.get("disagree_warn_deg", 15.0))
        rate = float(sh.get("steer_rate_deg_per_s", 0.0))
        hz = float(control_cfg.get("loop_hz", 50.0)) or 50.0
        self._steer_step = (rate / hz) if rate > 0 else 0.0   # 0 = 레이트 리밋 없음
        # heading 튐 방어 (_trusted_heading 참조). disagree_deg <= 0 = 완전 비활성.
        hg = dict(control_cfg.get("heading_guard") or {})
        self._hdg_guard_deg = float(hg.get("disagree_deg", 0.0))
        self._hdg_window = float(hg.get("window_m", COURSE_WINDOW_M)) or COURSE_WINDOW_M
        # 교차로 직진 게이트 (_straight_gate_target 참조). 기본 켬.
        self._gate_enabled = bool(control_cfg.get("straight_connector_gate", True))
        # 평행 정차 창 (_parallel_stop_target 참조). 0 이하 = 비활성.
        self._parallel_stop = float(control_cfg.get("parallel_stop_distance_m", 0.0))
        # 회전 직후 차선 복귀 거리 (_recenter_target 참조). 0 이하 = 비활성.
        self._recenter = float(control_cfg.get("lane_recenter_distance_m", 0.0))
        # 🔴 직선 추종 방식 (_recenter_target 참조). True = 차선 위 추종(기본), False = 끝점 겨냥.
        self._line_follow = bool(control_cfg.get("lane_line_following", True))
        # 횡오차 불감대 — 이 안이면 중심선으로 당기지 않는다 (_recenter_target 참조).
        self._deadband = float(control_cfg.get("lane_deadband_m", 0.0))
        # GPS 방위를 아예 안 쓰는 차선 (_trusted_heading 참조).
        self._course_lanes = frozenset(int(n) for n in (control_cfg.get("course_heading_lanes") or ()))
        _hold = control_cfg.get("hold_steer_lanes")
        if _hold is None:                       # 구 설정 호환 (bool + course_heading_lanes)
            _hold = self._course_lanes if control_cfg.get("hold_steer_when_stopped", True) else ()
        self._hold_lanes = frozenset(int(n) for n in _hold)
        # 🔴 직선 구간 조향 완전 금지 차선 (2026-08-08, 사용자 지시 "4번에서는 조향하지
        #   말고 직진만"). hold_steer_lanes 는 **정지 중**에만 막지만 이쪽은 주행 중에도
        #   막는다 — 그 차선에서는 횡오차가 얼마든 복귀 조향을 내지 않는다.
        #   회전(TURNING)에는 걸리지 않는다: 이 함수는 직선 추종 경로 전용이다.
        #   ⚠️ 대가: 차선 밖에서 출발하면 그 오차를 안고 끝까지 간다. 배치를 정확히
        #      맞추는 것이 전제다 (0808 3회차는 y=2.762 = 대향 차선에서 출발했고
        #      복귀 조향이 회전 개시 시점에 2.616 까지 되돌려 놨다).
        #   빈 목록 = 기능 꺼짐(기존 동작).
        self._straight_only = frozenset(
            int(n) for n in (control_cfg.get("straight_only_lanes") or ()))
        mb = dict(control_cfg.get("map_bounds") or {})
        self._map_x = float(mb.get("x_max", 5.0))
        self._map_y = float(mb.get("y_max", 3.0))
        self._map_margin = float(mb.get("turn_margin_m", 0.0))
        self._lookahead = float(control_cfg["lookahead_min_m"])
        self._full_lock_err = float(control_cfg["steer_full_lock_error_deg"])
        self._align_err = float(control_cfg["align_heading_error_deg"])
        self._align_ratio = float(control_cfg["align_throttle_ratio"])
        self._cruise = float(control_cfg["cruise_throttle"])
        self._turn = float(control_cfg["turn_throttle"])
        self._slowdown = float(control_cfg["destination_slowdown_distance_m"])
        self._near_ratio = float(control_cfg["near_target_throttle_ratio"])
        self._stop_radius = float(control_cfg["stop_trigger_radius_m"])
        self._ramp_dist = float(control_cfg["departure_ramp_distance_m"])
        self._exit_hold = float(control_cfg["corner_exit_hold_distance_m"])
        self._max_wheel = float(max_wheel_deg)
        self._max_speed = float(max_speed_mps)
        self._wheelbase = float(wheelbase_m)
        self._route: lane_route.LaneRoute | None = None
        self._reset_state()

    # ---------- 경로 수명 ----------

    def _reset_state(self) -> None:
        self._state = FOLLOW
        self._leg = 0
        self._pt = 1                     # 현재 다리에서 향하는 점의 인덱스
        self._spec = None                # 진행 중/예정 회전의 ResolvedTurn
        self._fired_at: tuple[float, float] | None = None   # 트리거 발화 위치 (로그용)
        self._fired_travel = 0.0         # 발화 후 모델 이동량 (시간 x OFFSET_SPEED_MPS)
        self._fired_last_t: float | None = None
        self._turn_from: tuple[float, float] | None = None  # 회전 개시 위치
        self._turn_travel = 0.0
        self._turn_path: list[tuple[float, float]] = []     # 회전 중 위치 자취 (코스각 산출용)
        self._last_pos: tuple[float, float] | None = None
        self._exit_hold_from: tuple[float, float] | None = None
        self._origin: tuple[float, float] | None = None
        self._best_remaining = float("inf")
        self._passed_warned = False
        self._seg_ok_now = False
        self._seg_hdg_on = False         # 직전 tick 에 seg 방위가 쓰였나 (전이 로그·텔레메트리)
        self._last_wheel: float | None = None    # 레이트 리밋 상태 (직선 구간 전용)
        self._disagree_at = 0.0          # 불일치 경고 마지막 시각 (로그 폭주 방지)
        self._follow_path: list[tuple[float, float]] = []   # 직선 구간 자취 (heading 튐 방어용)
        self._hdg_guard_on = False       # 직전 tick 에 코스각으로 대체됐나 (전이 로그용)
        self._hdg_guard_at = 0.0         # 튐 경고 마지막 시각
        self._straight_gate: tuple[float, float] | None = None  # 직진 커넥터 출구 (게이트)
        self._straight_from: tuple[float, float] | None = None  # 게이트 진입 위치 (지나침 판정용)
        self._fuse_span = 1              # 이번 회전이 다리를 몇 개 건너뛰나 (_fused_span 참조)

    def set_route(self, route) -> None:
        """RoutePlanner.Route 또는 LaneRoute 를 받는다.

        Route 를 받으면 여기서 lane_route.from_route() 로 접는다 —
        🔴 커넥터 원호 좌표가 버려지는 지점이다 (가상 라인 트레이싱 폐기).
        """
        lr = route if isinstance(route, lane_route.LaneRoute) else lane_route.from_route(route)
        self._route = lr
        self._reset_state()
        log.info("노선 확정: %s (다리 %d, 회전 %d, %.2fm)",
                 lr.describe(), len(lr.legs), len(lr.turns), lr.total_length_m)
        for turn in lr.turns:
            spec = self._table.resolve(turn)
            if not spec.is_turn:
                continue
            if spec.inherited:
                src = f"대칭 상속 ←{spec.source_key}"
            elif spec.from_table:
                src = "표"
            else:
                src = "기본값"
            trig = (f"{spec.trigger.cls}@{spec.trigger.near_row_frac:.2f}"
                    if spec.trigger.uses_vision else "좌표단독")
            log.info("  회전 %s(%s): 조향 %+.1f° · 트리거 %s · %s",
                     spec.key, spec.maneuver, spec.wheel_deg, trig, src)

    def clear(self) -> None:
        self._route = None
        self._reset_state()

    @property
    def has_route(self) -> bool:
        return self._route is not None

    @property
    def seg_allowed(self) -> bool:
        """seg 횡보정 허용 — 직선(FOLLOW) 구간이면서 코너 창 밖일 때만."""
        return self._seg_ok_now

    @property
    def target_allows_seg(self) -> bool:
        return self._seg_ok_now

    @property
    def state(self) -> str:
        return self._state

    @property
    def leg_index(self) -> int:
        """현재 몇 번째 다리를 달리는가 — 로그·텔레메트리·테스트용."""
        return self._leg

    @property
    def seg_heading_active(self) -> bool:
        """지금 tick 의 추종 방위가 seg 에서 왔는가 — 로그·텔레메트리·말뚝 테스트용."""
        return self._seg_hdg_on

    @property
    def heading_guard_active(self) -> bool:
        """지금 tick 의 추종 방위가 GPS 대신 **실제 진행 방향**으로 대체됐는가.

        True = GPS heading 이 튀는 중이라는 뜻이다 (_trusted_heading 참조).
        """
        return self._hdg_guard_on

    @property
    def current_lane_num(self) -> int | None:
        """지금 달리는 차선 번호 — "지금 11번" 을 그대로 관제·로그에 실을 수 있다."""
        if self._route is None:
            return None
        return self._route.legs[self._leg].lane_num

    # ---------- 제어 계산 ----------

    def compute(self, x: float, y: float, heading_deg: float) -> tuple[ControlCommand, float]:
        if self._route is None:
            return STOP_COMMAND, 0.0
        route = self._route
        if self._origin is None:
            self._origin = (x, y)
        self._accumulate_travel(x, y)

        leg = route.legs[self._leg]
        turn = route.turns[self._leg] if self._leg < len(route.turns) else None

        if self._state == TURNING:
            cmd, remaining = self._drive_turn(x, y, heading_deg, route, turn)
        else:
            cmd, remaining = self._drive_leg(x, y, heading_deg, route, leg, turn)

        if cmd is STOP_COMMAND:
            self._seg_ok_now = False
            return cmd, remaining
        return self._apply_throttle_rules(cmd, x, y), remaining

    # ---------- 직선 (FOLLOW / ARMED) ----------

    def _drive_leg(self, x, y, heading_deg, route, leg, turn):
        # 다리 안에서 목표점 전진 (전진만 — 등 뒤 목표로 역조향하지 않는다)
        last_pt = len(leg.points) - 1
        self._pt = min(self._pt, last_pt)      # 방어 — 퇴화 다리(길이 0)에서도 색인 유효
        while self._pt < last_pt:
            if math.hypot(leg.points[self._pt][0] - x, leg.points[self._pt][1] - y) < self._lookahead:
                self._pt += 1
                continue
            break
        tx, ty = leg.points[self._pt]

        # 🔴 교차로 직진 통과 중이면 **커넥터 출구(=다음 차선 시작점)** 를 먼저 겨눈다.
        #    (_straight_gate 주석 참조 — 왜 다음 차선 끝점을 바로 겨누면 안 되는가)
        # 목표점 결정 우선순위: 직진 게이트 > 차선 위 추종 > 다리 끝점(구 동작)
        gate = self._straight_gate_target(x, y)
        if gate is not None:
            tx, ty = gate
        rc = self._recenter_target(leg, x, y)          # 차선 중심선 위 추종
        if rc is not None:
            # 게이트가 살아 있어도, 이미 이 다리(20번) 위에 올라섰으면 차선 추종이 우선이다.
            # 사용자 지시: "19번에서 20번 **끝지점**까지 직진으로 가고 끝점에서 우회전"
            if gate is None or self._exit_ahead(leg, x, y) <= leg.length_m:
                tx, ty = rc
        if self._leg == len(route.legs) - 1:
            par = self._parallel_stop_target(leg, x, y,
                                             self._remaining_on_leg(leg, x, y))
            if par is not None:
                tx, ty = par[0], par[1]          # 정차점 너머 당근 (평행 정차)

        remaining = self._remaining_on_leg(leg, x, y) + route.tail_length_m(self._leg)
        is_final = self._leg == len(route.legs) - 1

        if is_final:
            # 🔴 평행 정차 (2026-08-07 저녁, 사용자 지시 — 면사무소 정차가 비스듬했다).
            #    _parallel_stop_target 주석 참조. 창 밖이면 None 이라 기존 동작 그대로다.
            par = self._parallel_stop_target(leg, x, y, remaining)
            if par is not None:
                tx, ty, along = par
                if along <= self._stop_radius:      # **차선 방향 남은 거리**로 정지 판정
                    return STOP_COMMAND, remaining
            if remaining <= self._stop_radius:
                return STOP_COMMAND, remaining
            # 지나침 감지 — 최종 목표 최근접 후 다시 멀어지면 그 자리 정지
            if remaining < self._best_remaining:
                self._best_remaining = remaining
            elif remaining > self._best_remaining + PASSED_HYSTERESIS_M:
                if not self._passed_warned:      # 50Hz 스팸 방지 — 최초 1회만 (2026-08-06 7차)
                    self._passed_warned = True
                    log.warning("최종 목표 지나침 — 정차 (남은 %.3fm, 최근접 %.3fm)",
                                remaining, self._best_remaining)
                return STOP_COMMAND, remaining

        # 회전 준비·개시 판정 (1차 = 좌표, 2차 = 비전)
        if turn is not None:
            if self._spec is None:
                self._spec = self._table.resolve(turn)
            spec = self._spec
            ahead = self._exit_ahead(leg, x, y)
            if spec.is_turn:
                arm = min(spec.arm_distance_m, leg.length_m)
                # 🔴 직진 게이트를 아직 못 지났으면 다음 회전을 준비하지 않는다.
                #    차선에 들어오기도 전에 ARMED 가 되면 그때부터 다음 차선을 향해
                #    비스듬히 겨누게 된다 (사용자 지적: 삼거리에서 사거리 쪽으로 파고듦).
                if self._straight_gate is not None:
                    arm = -1.0
                if self._state == FOLLOW and ahead <= arm:
                    self._state = ARMED
                    log.info("회전 준비 %s: 차선 끝 %.2fm 앞 (다음 %s번)",
                             spec.key, max(0.0, ahead), turn.to_num)
                if self._state == ARMED and self._should_start_turn(spec, ahead, x, y):
                    return self._begin_turn(x, y, heading_deg, route, turn, spec)
            elif ahead <= 0.0:
                # 교차로 직진 커넥터 — 회전 상태기계 없이 다음 다리로 이어 달린다.
                # 다만 **커넥터 구간을 겨눌 목표점** 은 남긴다 (_straight_gate 주석 참조).
                self._advance_leg(route)
                if self._gate_enabled and turn.maneuver == "straight":
                    self._straight_gate = tuple(turn.exit_point)
                    self._straight_from = (x, y)
                return self._drive_leg(x, y, heading_deg, route,
                                       route.legs[self._leg],
                                       route.turns[self._leg] if self._leg < len(route.turns) else None)
        elif not is_final:
            # 방어 — turn 이 없는데 마지막 다리도 아니면 노선 구성 오류
            log.error("노선 구성 오류: 다리 %d 뒤 회전 없음", self._leg)

        wheel = self._pursue(tx, ty, x, y, self._pursue_heading(leg, heading_deg))
        # 🔴 정지 중에는 조향하지 않는다 (2026-08-07 밤, 사용자 지시).
        #   "4번 주행 시작 직전에 heading 받아서 정지 상태의 직진 정렬을 수정하는데 막아라."
        #   측정 (21:22 대기 중): pose 가 (3.13~3.16, 2.71~2.72) 로 붙박이인데 y 오차가
        #   +0.105 로 불감대(0.10) 를 아슬하게 벗어나 남쪽 조향이 계속 나갔다.
        #   서 있는 동안 바퀴를 트는 것은 아무 이득이 없고(움직여야 효과가 난다),
        #   출발 순간 그 각도로 튀어나가 오히려 해가 된다.
        #   판정: 진행 방향 베이스라인(0.15m)을 못 채웠다 = 실질 정지.
        # 🔴 적용 차선을 **따로** 둔다 (2026-08-07 밤, 실주행으로 즉시 드러남).
        #   course_heading_lanes 에 묶어 뒀더니 19·20 을 거기 넣는 순간 같이 걸려,
        #   22:27 주행에서 차가 x=0.02(중심선 서쪽 0.37m)에 서서 **조향 0 으로 95초간
        #   직진만** 했다. 서편은 원래 기어가는 구간이라 진행 방향 베이스라인을 못 채우고,
        #   그게 곧 "정지"로 판정돼 복귀 조향이 영영 안 나갔다.
        #   정지 조향금지가 필요한 곳은 **출발 지점(4번)** 하나뿐이다.
        if leg.lane_num in self._hold_lanes                 and _course_deg(self._follow_path, self._hdg_window) is None:
            wheel = 0.0
        # 🔴 직선 구간 조향 완전 금지 (2026-08-08, 사용자 지시). 정지 여부와 무관하다.
        #   위 hold_steer_lanes 가 "서 있을 때만" 막는 것과 달리 주행 중에도 0 을 낸다.
        if leg.lane_num in self._straight_only:
            wheel = 0.0
        if self._state == ARMED:
            throttle = self._turn                # 코너 앞 선감속 = 회전 스로틀 그 자체
            self._seg_ok_now = False             # 코너 창 — seg 보정 차단
            if self._fired_at is not None:
                # 발화 후 오프셋 구간 = 조향 직진 고정 (2026-08-06 5차 실측).
                # 정지선 앵커 순간 차는 차선에 정렬돼 있다 — 이후 0.x m 를 왜곡된
                # 동편 GPS 로 P-추종하면 차가 블록 쪽으로 파고든다 (블록 중앙 진입 관찰).
                # 직진 고정이면 회전 진입 전체가 GPS 무관 결정론이 된다.
                wheel = 0.0
        else:
            throttle = self._cruise * min(1.0, leg.speed_limit_mps / self._max_speed)
            throttle *= self._slowdown_ratio(remaining) if is_final else 1.0
            err = abs(geometry.heading_diff_deg(
                math.degrees(math.atan2(ty - y, tx - x)), heading_deg))
            if err > self._align_err:
                throttle *= self._align_ratio
            self._seg_ok_now = self._exit_hold_from is None
        return (ControlCommand(throttle=throttle,
                               steering_wheel_deg=self._rate_limit(wheel)), remaining)

    # ---------- seg heading 대체 (모듈 docstring 참조) ----------

    def _trusted_heading(self, leg, heading_deg: float) -> float:
        """🔴 heading 튐 방어 — GPS 방위가 **실제 진행 방향**과 크게 어긋나면 후자를 쓴다.

        왜 (2026-08-07 저녁 18:04 주행, 사용자 지목):
          4번 차선 직선 구간에서 GPS heading 이 실제 진행 방향보다 +30~80° 왼쪽으로
          읽혔다. 차는 동쪽(코스 ≈350°)으로 가는데 "북동쪽 10~45°를 향한다"고 보고했다.
            18:04:30  hdg 13.3°  실제 293.6°  차이 +80°
            18:04:36  hdg 42.2°  실제 333.9°  차이 +68°
          _pursue 는 heading 을 그대로 믿으므로 "차가 차선보다 왼쪽으로 틀어졌다"고 읽고
          우측 풀락을 냈다. 그 결과 y 오차가 +0.11 → -0.29m 로 **단조 감소**하며 40cm를
          남쪽으로 끌려갔고, 그 자세로 회전에 진입해 인도를 밟았다.
          같은 기전이 0806 인도 침범(x≈3.6 부터 +17° 고원)에서도 확인됐다.

        왜 코스각을 믿나: 위치는 자기일관적이었다. 회전 종료 판정에서도 위치가 그린
        각도(_turn_course_deg)가 heading 보다 실제와 맞았다.

        한계 — 이 방어가 못 하는 것:
          ① 정지·저속에서는 베이스라인(window_m)을 못 채워 판단 자체를 못 한다.
             그때는 GPS 원값을 그대로 쓴다 (기존 동작 유지).
          ② 위치와 방위가 **같이** 틀리면 둘 다 어긋난 채로 일치하므로 못 잡는다.
          ③ 임계는 25° 기본이다. 직선 구간에서 조향 보정 중 코스각이 heading 보다
             뒤처지는 양은 창 0.15m·반경 0.47m 기준 약 9° 라 충분한 여유가 있다.
        """
        course = _course_deg(self._follow_path, self._hdg_window)

        # 🔴 GPS 방위 **전면 무시** 차선 (2026-08-07 밤, 사용자 지시).
        #   "4번 도로에서는 좌표만 받고 heading 값 무시하고 가라."
        #   근거 (21:00 주행, 4번): 차는 실제로 341~351°(남동)로 내려가는데 GPS 는
        #     0~11°(동·약간 북)라고 보고했다. 제어기가 그 값을 믿고 "0°에 맞춰야 한다"며
        #     **우측 조향**을 계속 냈고, 우측 = 남쪽이라 y 가 0.54m 동안 12cm 떨어졌다.
        #     차이가 19~21° 라 튐 방어 임계(25°) 바로 아래로 새고 있었다.
        #   → 이 차선에서는 임계 판정 없이 **위치가 그린 진행 방향만** 쓴다.
        #   ⚠️ 정지·저속에서는 베이스라인을 못 채워 course 가 None 이다. 그때만 GPS 로
        #      떨어진다 (달리 쓸 값이 없다 — 대신 정지 중에는 조향이 의미 없다).
        if leg.lane_num in self._course_lanes:
            if course is None:
                # 🔴 정지·저속이라 진행 방향을 못 구했다. 여기서 GPS 로 떨어지면
                #   "출발하자마자 방위 보정한다고 우회전" 이 그대로 재발한다 —
                #   21:09 실측: 정차 중 GPS 9~17°(북동)를 믿고 우측 조향이 나갔다.
                #   이 차선은 GPS 방위를 **아예 안 쓰기로** 했으므로, 대신 차선 공칭
                #   방위를 쓴다 (= 차가 차선과 나란하다고 본다 → 방위 항 0).
                #   남는 조향은 순수 횡오차분뿐이다.
                self._hdg_guard_on = True
                return leg.heading_deg()
            self._hdg_guard_on = True
            return course

        if self._hdg_guard_deg <= 0.0:
            self._hdg_guard_on = False
            return heading_deg
        if course is None:                       # 베이스라인 미확보 (정지·저속) — 원값 유지
            self._hdg_guard_on = False
            return heading_deg
        gap = geometry.heading_diff_deg(heading_deg, course)
        if abs(gap) < self._hdg_guard_deg:
            if self._hdg_guard_on:
                log.info("heading 튐 해제 (%s번): GPS %.1f° ≈ 코스 %.1f° — GPS 방위 복귀",
                         leg.lane_num, heading_deg, course)
            self._hdg_guard_on = False
            return heading_deg
        if not self._hdg_guard_on:
            log.warning("🔴 heading 튐 감지 (%s번): GPS %.1f° vs 실제 진행 %.1f° "
                        "(차이 %+.1f°) — 진행 방향으로 대체", leg.lane_num, heading_deg,
                        course, gap)
        else:
            now = time.monotonic()
            if now - self._hdg_guard_at >= 1.0:      # 50Hz 폭주 방지 — 1초 1회
                self._hdg_guard_at = now
                log.warning("🔴 heading 튐 지속 (%s번): GPS %.1f° vs 실제 %.1f° (%+.1f°)",
                            leg.lane_num, heading_deg, course, gap)
        self._hdg_guard_on = True
        return course

    def _pursue_heading(self, leg, heading_deg: float) -> float:
        """추종에 쓸 차량 방위 — 조건이 맞으면 GPS 대신 seg 로 갈아끼운다.

        조건: ①이 차선이 허용 목록 안 ②seg 어댑터 있음 ③seg valid.
        하나라도 어긋나면 heading 튐 방어를 거친 값 (기본은 GPS 원값).
        """
        if not self._seg_hdg_lanes or self._seg is None or leg.lane_num not in self._seg_hdg_lanes:
            self._seg_hdg_on = False
            return self._trusted_heading(leg, heading_deg)
        obs = self._seg.latest()
        if not obs.valid:
            if self._seg_hdg_on:
                log.info("seg 방위 대체 해제 (%s번, seg invalid) — GPS 방위 복귀", leg.lane_num)
            self._seg_hdg_on = False
            return self._trusted_heading(leg, heading_deg)

        # 계약 §5 부호: + = 좌측 보정 필요 = 차가 차선보다 우향 → 방위는 차선보다 작다
        seg_heading = leg.heading_deg() - obs.heading_error_deg
        gap = geometry.heading_diff_deg(heading_deg, seg_heading)
        if not self._seg_hdg_on:
            log.info("seg 방위 대체 개시 (%s번): 차선 %.1f° − seg %.1f° = %.1f° "
                     "(GPS %.1f°, 차이 %+.1f°)",
                     leg.lane_num, leg.heading_deg(), obs.heading_error_deg,
                     seg_heading, heading_deg, gap)
        self._seg_hdg_on = True
        self._hdg_guard_on = False       # seg 가 이미 방위를 쥐고 있다 — 튐 방어는 비활성
        # 🔴 GPS heading 이상 감지 — 값 자체는 seg 로 이미 대체됐고, 이 줄은 증거 수집용이다.
        #    (0806 차선 4: GPS +37.3° vs seg +6.8° = 차이 44° 인 순간이 인도 침범의 시작점)
        if abs(gap) >= self._disagree_warn:
            now = time.monotonic()
            if now - self._disagree_at >= 1.0:      # 50Hz 폭주 방지 — 1초 1회
                self._disagree_at = now
                log.warning("🔴 GPS heading 의심 (%s번): GPS %.1f° vs seg %.1f° (차이 %+.1f°) "
                            "— seg 방위로 주행 중", leg.lane_num, heading_deg, seg_heading, gap)
        return seg_heading

    def _recenter_target(self, leg, x: float, y: float):
        """🔴 직선 구간은 **차선 위를 따라간다** (2026-08-07 밤, 사용자 지시 3건을 한 번에).

        사용자 신고 3건이 전부 같은 구조였다:
            ① "14번에서 정렬되면 그냥 가라 — 계속 인도 쪽으로 조향해서 발차 때 인도로 든다"
            ② "14->19 가 끝점에서 도는 게 아닌 것 같다"
            ③ "20번 들어가기 직전에 잘못된 GPS 로 인도로 돌진한다"

        측정 (20:35 주행):
            ① 14번  y −0.083 → **+0.107** (중심선 0.39 를 지나 북쪽으로 넘어감).
                    14번 북쪽 가장자리 0.515, 인도(block_sw) 0.52 — 2.3cm 남았다.
            ② 14->19 개시 (0.696, 0.492). x 는 끝점(0.65) 0.046m 앞이라 맞는데 **y 가
                    10cm 북쪽**이라, 그 자세의 호는 (0.535, 0.520) = 인도를 통과한다.
            ③ 19/20  x 0.00~0.16 으로 서쪽을 달리다 마지막 3초에 0.169 → **0.489** 로
                    31cm 동진. heading 튐 방어는 정상 작동했고(GPS 170~256° 기각),
                    대체된 진행방향이 **45° 북동** 이었다 — 사선으로 붙다가 넘어간 것이다.

        공통 원인:
            직선 다리의 목표가 **차선 끝점**이다. 7번이면 1.26m, 20번이면 1.3m 앞이다.
            횡오차가 크면 그 먼 점은 **대각선**이 되어 차가 사선으로 돌진하고, 중심선에
            닿는 순간 멈추지 못해 반대쪽으로 넘어간다. 반대로 횡오차가 작으면(3cm)
            방위 오차가 1.4° 밖에 안 나와 복구가 아예 안 된다. 양쪽 다 나쁘다.

        고치는 방법 — 차선 위 추종:
            목표를 **차선 중심선에 투영한 점 + lookahead** 로 잡는다. 순수 추종의 정석이라
            횡오차가 크든 작든 **점근 수렴**한다: 사선 돌진도, 넘어감도 없다.
            정렬되어 중심선 위에 있으면 목표가 정면이라 조향이 0 이 된다 — 사용자 요구
            "정렬되면 그냥 가라" 가 그대로 성립한다.

        ⚠️ 이 방식은 **차선 위를 벗어난 채 코너에 들어가는 것을 막는 게 목적**이다.
           코너 진입 직전 의도적인 바깥쪽 물기가 필요해지면 lane_line_following: false 로
           끄고 회전 직후 recenter 창(lane_recenter_distance_m)만 쓰는 구 동작으로 돌아간다.
        """
        if not self._line_follow:
            # 구 동작: 회전 직후 recenter 거리 안에서만 중심선 복귀
            if self._recenter <= 0.0 or self._exit_hold_from is None:
                return None
            ex, ey = self._exit_hold_from
            if math.hypot(x - ex, y - ey) > self._recenter:
                return None
        (x0, y0), (x1, y1) = leg.points[0], leg.points[-1]
        dx, dy = x1 - x0, y1 - y0
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            return None
        ux, uy = dx / norm, dy / norm
        s = (x - x0) * ux + (y - y0) * uy          # 차선 위 투영 위치
        off = -(x - x0) * uy + (y - y0) * ux       # 좌(+)/우(−) 횡오차
        s = min(s + self._lookahead, norm)         # 차선 끝을 넘기지 않는다
        if abs(off) <= self._deadband:
            # 🔴 불감대 — 차선 안에 있으면 **중심선으로 당기지 않고 그냥 간다**.
            #   사용자 지시: "14번에서 정렬되면 좀 그냥 가라. 계속 인도 쪽으로 조향해서
            #   정차 후 발차 과정에 인도로 든다."
            #   14번은 북쪽 가장자리(0.515) 바로 위가 인도(block_sw 0.52)라 중심선 복귀가
            #   곧 인도 방향이다. 20:35 실측에서 y −0.083 → **+0.107** 로 넘어가 인도까지
            #   2.3cm 남았다. 게다가 그 자세로 14->19 를 돌면 호가 (0.535, 0.520) =
            #   인도를 통과한다 — 남쪽에 붙어 있는 편이 이 코너에서는 오히려 안전하다.
            #   차선 위 자기 위치에서 진행 방향으로만 겨눈다 → 정렬돼 있으면 조향 0.
            return x + ux * self._lookahead, y + uy * self._lookahead
        return x0 + ux * s, y0 + uy * s

    def _parallel_stop_target(self, leg, x: float, y: float, remaining: float):
        """🔴 최종 목표 부근에서 **차선과 평행하게** 정차시킨다 (2026-08-07 저녁, 사용자 지시).

        문제 (사용자 실차 관찰 — 면사무소):
            "면사무소 좌표 들어갈 때 조향을 주고 정지한다."

        왜 그렇게 되나:
            정차점은 차선 중심선 위의 한 점(면사무소는 14번 위 1.20, 0.39)인데, 차가
            횡으로 조금 벗어나 있으면 그 점은 **옆으로 비껴 있는 목표**가 된다. 추종은
            점을 맞히려 하므로 마지막까지 조향이 들어가고, 그 자세로 멈춘다.
            19:44 실측: 정차 직전 방위 190~235° (14번 방위는 180°) — 최대 55° 틀어졌다.

        고치는 방법 (당근):
            목표를 정차점 **너머 차선 방향으로 lookahead 만큼** 옮긴다. 그러면 차는 점을
            맞히는 대신 **차선 위로 올라타** 달리게 되고, 도착 시 자세가 차선과 나란해진다.
            정지 판정은 유클리드 거리가 아니라 **차선 방향 남은 거리(along)** 로 한다 —
            횡오차가 남아 있어도 세로 위치가 맞으면 서야 하기 때문이다.

        ⚠️ 22->14 회전 자체는 건드리지 않는다 (사용자 명시). 이 보정은 회전이 끝나고
           14번 직선을 달리는 **최종 접근 구간**에서만 걸린다.
        ⚠️ 창(parallel_stop_distance_m) 밖에서는 None 을 돌려 기존 동작을 그대로 둔다.

        반환: (목표x, 목표y, 차선방향 남은거리) 또는 None
        """
        if self._parallel_stop <= 0.0 or remaining > self._parallel_stop:
            return None
        (x0, y0), (gx, gy) = leg.points[0], leg.points[-1]
        dx, dy = gx - x0, gy - y0
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            return None
        ux, uy = dx / norm, dy / norm
        along = (gx - x) * ux + (gy - y) * uy      # + = 아직 정차점 앞
        return gx + ux * self._lookahead, gy + uy * self._lookahead, along

    def _straight_gate_target(self, x: float, y: float):
        """🔴 교차로 직진 커넥터를 지나는 동안 겨눌 목표점 (2026-08-07 저녁, 사용자 지시).

        문제 (사용자 실차 관찰 — 19->20):
            "삼거리에서 계속 사거리 방향으로 진입하려고 한다."

        왜 그렇게 되나:
            직진 커넥터는 회전 상태기계를 타지 않고 곧바로 다음 다리로 넘어간다.
            그런데 **커넥터 구간(19->20 은 1.28m)은 어느 다리에도 속하지 않는다.**
            그래서 19번 끝(0.39, 0.86)을 지나는 순간부터 차는 20번 차선의 **끝점**
            (0.39, 2.35)을 겨눈다 — 1.49m 앞이다.
            차가 차선에서 옆으로 벗어나 있으면(실측 x 오차 −0.15~−0.24m) 그 먼 목표는
            **비스듬한 대각선**이 되고, 그 방향이 마침 티 교차로에서 중앙 도로
            (mid_eb1_w 0.65,1.37 / mid_eb2_w 0.65,1.12)로 파고드는 방향과 같다.
            목표가 멀수록 같은 횡오차가 만드는 방위 오차는 작아져 복구도 느리다 —
            18:38 실측에서 x 오차 −0.20m 를 10초간 끌고 갔다.

        고치는 방법:
            커넥터를 지나는 동안은 **커넥터 출구 = 다음 차선의 시작점**을 겨눈다.
            19->20 이면 (0.39, 2.14) 다. 같은 직선 위의 가까운 점이라 차는 대각선이
            아니라 선 위로 복귀한다. 출구를 지나면 게이트를 풀고 평소대로 다리를 탄다.
            게이트가 살아 있는 동안은 다음 회전을 ARMED 시키지도 않는다 —
            차선에 들어오기 전에 다음 차선을 겨누기 시작하면 같은 대각선이 된다.

        결과적으로 사용자 요구 그대로가 된다:
            "19번에 제대로 진입하면 20번 끝나는 부근에서만 3번으로 조향한다."

        ⚠️ 시나리오 2 에서 직진 커넥터는 19->20 **하나뿐**이라 실제 영향 지점도 하나다.
           (픽업 다리 4->29->7->22->14 에는 직진 커넥터가 없다.)
        """
        if self._straight_gate is None:
            return None
        gx, gy = self._straight_gate
        if math.hypot(gx - x, gy - y) < self._lookahead:
            self._straight_gate = None                # 출구 도달 — 평소 추종으로 복귀
            return None
        if self._straight_from is not None:           # 출구를 지나쳤으면(투영 음수) 풀어 준다
            fx, fy = self._straight_from
            dx, dy = gx - fx, gy - fy
            norm = math.hypot(dx, dy)
            if norm > 1e-9 and ((x - gx) * dx + (y - gy) * dy) / norm > 0.0:
                self._straight_gate = None
                return None
        return gx, gy

    def _rate_limit(self, wheel: float) -> float:
        """직선 구간 조향 슬루 제한 — 방위 출처가 바뀌는 순간의 계단 지령을 막는다.

        회전(TURNING)은 거치지 않는다 (_drive_turn 이 표의 고정각을 그대로 낸다).
        회전 종료 직후에도 걸지 않는다 (_advance_leg 에서 상태를 비운다) — 코너 출구
        복귀에는 권한이 필요하다.
        """
        if self._steer_step <= 0.0 or self._last_wheel is None:
            self._last_wheel = wheel
            return wheel
        lo, hi = self._last_wheel - self._steer_step, self._last_wheel + self._steer_step
        self._last_wheel = max(lo, min(hi, wheel))
        return self._last_wheel

    def _should_start_turn(self, spec, ahead: float, x: float, y: float) -> bool:
        """2차 트리거. 비전 발화가 우선, 없으면 좌표 폴백(차선 끝).

        발화 후 오프셋 소모는 GPS 이동량이 아니라 **시간 x 모델 속도**로 잰다 —
        동편 왜곡 GPS 로 재면 명령 거리와 실물 거리가 어긋난다 (2026-08-06 4차 실측).
        정지(유실 등)로 compute 가 끊긴 시간은 적산하지 않는다.
        """
        if spec.trigger.uses_vision and self._marks is not None:
            now = time.monotonic()
            if self._fired_at is None and self._marks.triggered(spec.trigger):
                self._fired_at = (x, y)
                self._fired_travel = 0.0
                self._fired_last_t = now
                log.info("비전 트리거 발화 %s: %s (차선 끝 %.2fm 앞, 오프셋 %.2fm≈%.1fs)",
                         spec.key, spec.trigger.cls, max(0.0, ahead),
                         spec.start_offset_m, spec.start_offset_m / OFFSET_SPEED_MPS)
            if self._fired_at is not None:
                if self._fired_last_t is not None:
                    dt = now - self._fired_last_t
                    if 0.0 < dt <= OFFSET_GAP_MAX_S:
                        self._fired_travel += dt * OFFSET_SPEED_MPS
                self._fired_last_t = now
                return self._fired_travel >= spec.start_offset_m
        return spec.fallback_at_lane_end and ahead <= spec.fallback_lead_m

    def _begin_turn(self, x, y, heading_deg, route, turn, spec):
        self._state = TURNING
        self._turn_from = (x, y)
        self._turn_travel = 0.0
        self._turn_path = [(x, y)]
        self._follow_path = []           # 직선 자취 종료 — 회전 원호가 섞이면 튐으로 오인된다
        self._hdg_guard_on = False
        self._fuse_span = self._fused_span(route, spec)
        by = "비전" if self._fired_at is not None else "좌표 폴백"
        if self._fuse_span > 1:
            log.info("회전 개시 %s (%s, %s): 조향 %+.1f° — **%s 까지 이어 돈다** (중간 종료 없음)",
                     spec.key, spec.maneuver, by, spec.wheel_deg,
                     route.turns[self._leg + 1].key)
        else:
            log.info("회전 개시 %s (%s, %s): 조향 %+.1f°",
                     spec.key, spec.maneuver, by, spec.wheel_deg)
        return self._drive_turn(x, y, heading_deg, route, turn)

    def _fused_span(self, route, spec) -> int:
        """🔴 이 회전이 다음 회전까지 **한 번에** 도는가 (2026-08-07 저녁, 사용자 제안).

        4->29->7 이 대상이다. 지금은 90° 코너 두 개로 쪼개져 있고 각 코너의 설계 반경이
        0.26m 인데, 이 차의 **실효 반경은 0.47m** 라 애초에 낼 수 없는 값이다.
        그래서 매 회전 밖으로 밀리고, 중간(29번, 길이 0.21m)에서 종료 판정이 한 번 더
        끼어들어 조향을 놓는다 — 19:43 실측에서 4->29 가 **0.30m 만에** 끝났다
        (heading 8.8° 로 정렬됐다고 판정했는데 실제 진행 방향은 63.5° 어긋나 있었다).

        180° 한 번으로 보면 기하가 정반대로 맞는다:
            4번(y=2.61) → 7번(y=1.63) 횡간격 0.98m = 180° 회전의 지름
            필요 반경 R = 0.98 / 2 = 0.49m   vs   실측 실효 반경 0.47m  (2cm 차이)
        즉 "못 내는 반경"이 이 묶음에서는 정답이 된다.

        호의 최동단은 개시 x + R 이다:
            차선 끝(4.35)에서 개시  → 4.84  = 바깥 북행 차선(x=4.86) 침범
            0.21m 앞(4.14)에서 개시 → 4.63  = 29번 중심선(4.61)에 붙어 지나감  ← 채택
        (인도 block_ne 는 x ≤ 4.48 이라 안쪽이다 — 크게 돌면 대향 차선, 작게 자르면 인도.)

        종료 판정은 중간 차선(29번)이 아니라 **끝 차선(7번) 방위**로 한다.
        90° 지점에서 차는 남향이라 7번 방위(180°)와 90° 어긋나 있어 조기 종료하지 않는다.
        """
        if not spec.fuse_next:
            return 1
        nxt = self._leg + 1
        if nxt >= len(route.turns) or nxt + 1 >= len(route.legs):
            return 1                                  # 뒤에 이어 돌 회전이 없다
        if not self._table.resolve(route.turns[nxt]).is_turn:
            return 1                                  # 다음이 직진 커넥터면 묶지 않는다
        return 2

    # ---------- 회전 (TURNING) ----------

    def _drive_turn(self, x, y, heading_deg, route, turn):
        spec = self._spec
        span = self._fuse_span                    # 1 = 보통 / 2 = 다음 회전까지 이어 돈다
        next_leg = route.legs[self._leg + span]   # 정렬 판정 기준 = **묶음의 끝 차선**
        last_turn = route.turns[self._leg + span - 1]
        remaining = (math.hypot(last_turn.exit_point[0] - x, last_turn.exit_point[1] - y)
                     + next_leg.length_m + route.tail_length_m(self._leg + span))

        want = next_leg.heading_deg()
        aligned = abs(geometry.heading_diff_deg(want, heading_deg))
        course = self._turn_course_deg()          # 위치가 그린 진행 방향 (GPS heading 무관)
        aligned_c = None if course is None else abs(geometry.heading_diff_deg(want, course))

        # 🔴 맵 경계 방어 (2026-08-07 밤, 사용자 지시 "맵 바깥으로 조향하는 거 막아야 함").
        #   회전은 개루프라 한 번 열리면 정렬될 때까지 같은 각도로 돈다. 실효 반경이
        #   커진 날에는 그 호가 맵 밖으로 나간다 — 21:24 실측 최서단 **x = -0.034**.
        #   14->19 를 뒤로 미룰수록(사용자 요청) 종료 지점이 그만큼 서쪽이라 더 잦아진다.
        #   → 경계 여유 안으로 들어오면 정렬과 무관하게 **그 자리에서 회전을 끊는다.**
        #   끊긴 뒤에는 직선 추종이 차선 중심선으로 되돌린다 (개루프보다 훨씬 안전).
        out_of_map = (self._map_margin > 0.0 and (
            x < self._map_margin or y < self._map_margin
            or x > self._map_x - self._map_margin or y > self._map_y - self._map_margin))
        if out_of_map:
            log.warning("회전 %s 맵 경계 종료: (%.3f, %.3f) — %.2fm 주행, 정렬 오차 %.1f°",
                        spec.key, x, y, self._turn_travel,
                        abs(geometry.heading_diff_deg(next_leg.heading_deg(), heading_deg)))
            self._exit_hold_from = (x, y)
            for _ in range(span):
                self._advance_leg(route)
            return self._drive_leg(x, y, heading_deg, route, route.legs[self._leg],
                                   route.turns[self._leg] if self._leg < len(route.turns) else None)

        long_enough = self._turn_travel >= spec.min_turn_m
        by_heading = long_enough and aligned <= spec.exit_align_deg
        by_course = long_enough and aligned_c is not None and aligned_c <= spec.exit_align_deg
        by_arc = (spec.arc_complete_m is not None
                  and self._turn_travel >= spec.arc_complete_m)
        done = by_heading or by_course or by_arc
        over = self._turn_travel >= spec.max_turn_m
        if done or over:
            why = ("heading" if by_heading else
                   "코스각" if by_course else
                   "호장" if by_arc else "상한")
            if over and not done:
                log.warning("회전 %s 상한 종료: %.2fm 주행, 정렬 오차 %.1f° (코스각 %s, 임계 %.1f°)",
                            spec.key, self._turn_travel, aligned,
                            "—" if aligned_c is None else "%.1f°" % aligned_c,
                            spec.exit_align_deg)
            else:
                log.info("회전 %s 종료(%s): %.2fm, 정렬 오차 %.1f° / 코스각 %s",
                         spec.key, why, self._turn_travel, aligned,
                         "—" if aligned_c is None else "%.1f°" % aligned_c)
            self._exit_hold_from = (x, y)
            for _ in range(span):        # 묶음이면 중간 차선(29번)을 건너뛴다
                self._advance_leg(route)
            return self._drive_leg(x, y, heading_deg, route, route.legs[self._leg],
                                   route.turns[self._leg] if self._leg < len(route.turns) else None)

        self._seg_ok_now = False          # 회전 중 seg 보정 금지 (코너 시야 오인 대책)
        self._seg_hdg_on = False          # 회전 중에는 방위 대체도 없다 (표의 고정각만 나간다)
        return ControlCommand(throttle=self._turn,
                              steering_wheel_deg=spec.wheel_deg), remaining

    def _advance_leg(self, route) -> None:
        self._leg += 1
        self._pt = 1
        self._state = FOLLOW
        self._spec = None
        self._fired_at = None
        self._fired_travel = 0.0
        self._fired_last_t = None
        self._turn_from = None
        self._turn_travel = 0.0
        self._turn_path = []
        self._last_wheel = None          # 코너 출구 복귀에 슬루 제한을 걸지 않는다
        self._follow_path = []           # 회전 전 자취는 버린다 (원호 방향이 섞이면 오탐)
        self._hdg_guard_on = False
        self._straight_gate = None       # 직진 게이트는 다리마다 새로 세운다
        self._straight_from = None
        self._fuse_span = 1

    # ---------- 공통 ----------

    def _pursue(self, tx, ty, x, y, heading_deg) -> float:
        """P 추종 — 오차 +(목표 왼쪽) → 바퀴각 −(좌). 풀락 오차에서 최대 조향 (taxi 기하)."""
        target_heading = math.degrees(math.atan2(ty - y, tx - x))
        err = geometry.heading_diff_deg(target_heading, heading_deg)
        wheel = -self._max_wheel * max(-1.0, min(1.0, err / self._full_lock_err))
        return max(-self._max_wheel, min(self._max_wheel, wheel))

    def _apply_throttle_rules(self, cmd: ControlCommand, x: float, y: float) -> ControlCommand:
        throttle = cmd.throttle
        # 재출발 램프 — 정지 시 틀어진 heading 에서 급가속하면 인도 침범 (2026-08-04 실차)
        if math.hypot(x - self._origin[0], y - self._origin[1]) < self._ramp_dist:
            throttle = min(throttle, self._turn)
        # 코너 출구 홀드 — 회전 직후 토크 유지 (2026-08-04 동쪽 삼거리 실측)
        if self._exit_hold_from is not None:
            if math.hypot(x - self._exit_hold_from[0], y - self._exit_hold_from[1]) < self._exit_hold:
                throttle = self._turn
                self._seg_ok_now = False
            else:
                self._exit_hold_from = None
        if throttle == cmd.throttle:
            return cmd
        return ControlCommand(throttle=throttle, steering_wheel_deg=cmd.steering_wheel_deg)

    def _accumulate_travel(self, x: float, y: float) -> None:
        if self._last_pos is not None and self._state == TURNING:
            self._turn_travel += math.hypot(x - self._last_pos[0], y - self._last_pos[1])
            self._turn_path.append((x, y))       # 코스각 산출용 (오래된 점은 아래서 버린다)
            while len(self._turn_path) > 2 and _path_len(self._turn_path) > COURSE_WINDOW_M * 2.5:
                self._turn_path.pop(0)
        else:
            # 직선 구간 자취 — heading 튐 방어 · 4번 방위 대체 · 정지 판정이 함께 쓴다.
            # 🔴 **무조건 쌓는다.** 예전엔 heading_guard 설정에 묶여 있었는데, 방어를 끄면
            #   진행 방향을 못 구해 course_heading_lanes 차선에서 조향이 영영 0 이 됐다
            #   (2026-08-07 밤 발견 — 세 기능이 같은 자취를 쓰게 되면서 드러난 결합 버그).
            # 회전 중에는 쌓지 않는다: 회전은 원호라 코스각이 heading 보다 구조적으로
            # 뒤처지고(창 0.15m·반경 0.47m 기준 약 9°), 그걸 튐으로 오인하면 안 된다.
            if self._last_pos is None or math.hypot(x - self._last_pos[0],
                                                    y - self._last_pos[1]) > 1e-6:
                self._follow_path.append((x, y))
                while (len(self._follow_path) > 2
                       and _path_len(self._follow_path) > self._hdg_window * 2.5):
                    self._follow_path.pop(0)
        self._last_pos = (x, y)

    def _turn_course_deg(self) -> float | None:
        """회전 중 **위치가 그린 진행 방향**. GPS heading 을 전혀 쓰지 않는다.

        왜 필요한가 (2026-08-07 실주행): 같은 회전에서 위치는 75~84° 를 그렸는데
        heading 은 33~49° 만 보고했다. 종료 판정이 heading 뿐이라 "다 돌았는데 못 알아보고"
        풀락을 유지하다 max_turn 에 걸렸다 — 4회 중 3회 상한 종료.
        위치는 자기일관적이었으므로(4->29 는 코스각 −76° vs heading −80° 로 일치) 이쪽을 쓴다.
        """
        p = self._turn_path
        if len(p) < 3:
            return None
        end = p[-1]
        for i in range(len(p) - 2, -1, -1):      # 뒤에서부터 COURSE_WINDOW_M 만큼 물러난 점
            dx, dy = end[0] - p[i][0], end[1] - p[i][1]
            if math.hypot(dx, dy) >= COURSE_WINDOW_M:
                return math.degrees(math.atan2(dy, dx))
        return None

    @staticmethod
    def _exit_ahead(leg, x: float, y: float) -> float:
        """차선 진행 방향 기준 끝점까지의 부호 있는 거리 — 끝점을 지나면 음수."""
        (x0, y0), (x1, y1) = leg.points[0], leg.points[-1]
        dx, dy = x1 - x0, y1 - y0
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            return 0.0
        return ((x1 - x) * dx + (y1 - y) * dy) / norm

    def _remaining_on_leg(self, leg, x: float, y: float) -> float:
        pts = leg.points
        rem = math.hypot(pts[self._pt][0] - x, pts[self._pt][1] - y)
        for j in range(self._pt, len(pts) - 1):
            rem += math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1])
        return rem

    def _slowdown_ratio(self, remaining: float) -> float:
        """감속 구간에서 near_ratio → 1.0 선형 (taxi _throttle_for_distance)."""
        if remaining >= self._slowdown:
            return 1.0
        usable = max(self._slowdown - self._stop_radius, 1e-9)
        progress = max(0.0, min(1.0, (remaining - self._stop_radius) / usable))
        return self._near_ratio + (1.0 - self._near_ratio) * progress
