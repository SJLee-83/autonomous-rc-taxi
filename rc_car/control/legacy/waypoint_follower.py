"""🔴 [폐기 보류] WaypointFollower — 가상 라인 트레이싱 (커넥터 원호 centerline 추종).

⚠️ 이 모듈은 **기본 주행 경로에서 빠져 있다** (2026-08-06 §0-45 재설계).
   현 기본은 control/lane_follower.py — 차선 번호 point-to-point + 2단 회전.
   여기로 되돌아오려면 환경변수 RC_CAR_LEGACY_ARC=1 뿐이다 (control/legacy/__init__.py 참조).
   config·yaml로는 켜지지 않으며, 켜지면 기동 로그에 🔴 경고가 남는다.

   남겨둔 이유: 시연 당일 비전 트리거가 무너졌을 때의 최후 폴백.
   미해결로 안고 들어온 결함 — 커넥터 기하 자체가 도로 문법과 어긋나 좌회전이
   "반대편 횡단보도 가서야 회전"한다 (2026-08-05 사고 C, 곡률 FF로도 미해소).

--- 이하 원 문서 (2026-08-05 시점) ---

GPS 폴백 조향 (WBS ⑤, D3 결정: taxi/navigator.py P-제어 이식).

taxi 실증 로직을 경로(waypoint 열) 추종으로 확장한 것:
- 목표점 = 현재 위치에서 lookahead 이상 떨어진 첫 waypoint (인덱스는 전진만 한다)
- 조향 = heading 오차 비례 (풀락 오차 45°에서 최대 조향 — taxi STEER_GAIN과 동일 기하)
  부호: 오차 + = 목표가 왼쪽 (§2.7 반시계) → 바퀴각 − (ControlCommand는 +가 우측)
- 스로틀 = 순항 × (구간 제한속도/최고속도) × 목적지 감속 × 정렬 감속
  정렬 감속은 차선(lane·follow) 목표에서만 적용 — 커넥터 기동(회전·차선 변경·교차로
  직진) 중에는 큰 heading 오차가 정상 상태라 면제한다 (2026-08-01 시뮬 검증:
  이탈 영향 없이 완주 시간 −15~20%, tools/turn_param_sim.py)
- 목적지 감속도 차선 목표에서만 — 커넥터는 turn_throttle 고정이라 겹감속하면 데드존에
  들어간다. 저속 상황은 전부 turn_throttle(실차 검증된 최저 유효 스로틀)로 통일:
  ① 재출발 램프(경로 시작 departure_ramp_distance_m까지 — 정지 heading 스윙 방지)
  ② 코너 앞 선감속(커넥터 진입점 corner_slowdown_distance_m 안 — 계단 관성 방지)
  (2026-08-04 실차: 픽업 오버슈트 한 바퀴·재출발 인도 침범 이탈 대책)
- 남은 거리 ≤ stop_trigger_radius 면 정지 명령 (코스트 정지 — 정지 거리는 B3 실측 항목).
  트리거(0.05)는 도착 게이트(arrival_radius 0.10)와 분리 — 컷 반경 진입 즉시 코스트
  정지라 정지 오차 ≈ 트리거 반경. 게이트는 0.10 유지로 판정 신뢰성 보존 (2026-08-02)

이 모듈은 순수 계산만 한다 — 하드웨어 출력은 호출자(DrivingWorker)가
SafetySupervisor.apply()로 보낸다 (모터 출력 단일 통과점 원칙).
seg 조향 승격 시 이 모듈이 폴백으로 내려간다 (조향 입력원 교체 구조).
"""
from __future__ import annotations

import bisect
import math

from core.models import ControlCommand, STOP_COMMAND
from mapping import geometry
from navigation.route_planner import Route

PASSED_HYSTERESIS_M = 0.08   # 최종 목표 최근접 이후 이만큼 다시 멀어지면 '지나침' 확정
                             # (GPS sd 1cm 대비 충분한 여유 — 2026-08-05 저녁 배회 사고)


class WaypointFollower:
    def __init__(self, control_cfg: dict, max_wheel_deg: float, max_speed_mps: float,
                 wheelbase_m: float = 0.14):
        self._spacing = float(control_cfg["waypoint_spacing_m"])
        self._lookahead = float(control_cfg["lookahead_min_m"])
        self._corner_lookahead = float(control_cfg["corner_lookahead_m"])
        self._ff_lead = float(control_cfg["ff_entry_lead_m"])
        self._full_lock_err = float(control_cfg["steer_full_lock_error_deg"])
        self._align_err = float(control_cfg["align_heading_error_deg"])
        self._align_ratio = float(control_cfg["align_throttle_ratio"])
        self._cruise = float(control_cfg["cruise_throttle"])
        self._turn = float(control_cfg["turn_throttle"])
        self._slowdown = float(control_cfg["destination_slowdown_distance_m"])
        self._near_ratio = float(control_cfg["near_target_throttle_ratio"])
        self._stop_radius = float(control_cfg["stop_trigger_radius_m"])
        self._ramp_dist = float(control_cfg["departure_ramp_distance_m"])
        self._corner_slow = float(control_cfg["corner_slowdown_distance_m"])
        self._exit_hold = float(control_cfg["corner_exit_hold_distance_m"])
        self._max_wheel = float(max_wheel_deg)
        self._max_speed = float(max_speed_mps)
        self._wheelbase = float(wheelbase_m)
        self._wps: list[tuple[float, float, float, bool, float]] = []  # (x, y, 제한속도, seg허용, FF바퀴각)
        self._cum: list[float] = []                        # 시작부터 각 waypoint까지 거리
        self._idx = 0
        self._target_allows_seg = False
        self._seg_ok_now = False                           # 차량 위치 기준 (코너 창 밖)
        self._near_corner = False                          # 전 tick 판정 — 시선 단축용
        self._origin: tuple[float, float] | None = None    # 경로 시작 위치 (재출발 램프 기준)
        self._best_remaining = float("inf")                # 지나침 감지 (최종 목표 최근접)

    # ---------- 경로 수명 ----------

    def set_route(self, route: Route) -> None:
        """구간별 재샘플 + 구간 제한속도 부착. 접점 중복은 제거한다.

        seg 허용 플래그: 차선(lane)과 차선 유지(follow) 구간에서만 seg 보정을 허용한다.
        회전·차선 변경·교차로 직진 중에는 보정이 경로와 싸우므로 GPS 단독 (명세서 §18-3
        "신뢰도 낮으면 미사용"의 경로 기하 버전).
        """
        wps: list[tuple[float, float, float, bool, float]] = []
        for seg in route.segments:
            seg_ok = seg.kind == "lane" or seg.maneuver == "follow"
            # 회전 커넥터는 필요 바퀴각을 기하로 미리 안다 — 피드포워드 (2026-08-04 실차:
            # P-제어 단독은 오차가 커져야 꺾어서 진입 부풂(역주행 진입)·후반 커팅(인도)을 만든다.
            # 우회전 R 0.26 필요각 28.3° ≈ 풀락이라 특히 치명적). 부호: +가 우측 (§2.7 반대)
            #
            # 각도는 세그먼트 반경이 아니라 **waypoint별 국소 곡률**로 계산한다 —
            # _far 다리 커넥터(예: 1.51m, radius_m 0.35)는 원호가 아니라 직선+호 복합이라
            # 상수 FF가 직선부에서 과조향을 만든다 (2026-08-05 저녁 좌회전 S자의 뿌리)
            use_ff = (seg.kind == "connector" and seg.maneuver in ("right", "left")
                      and getattr(seg, "radius_m", None))
            pts = (geometry.resample(seg.centerline, self._spacing)
                   if len(seg.centerline) >= 2 else seg.centerline)
            ffs = self._local_ff(pts) if use_ff else [0.0] * len(pts)
            for (x, y), ff in zip(pts, ffs):
                if wps and math.hypot(x - wps[-1][0], y - wps[-1][1]) < 1e-9:
                    continue
                wps.append((x, y, seg.speed_limit_mps, seg_ok, ff))
        assert wps, "빈 경로"
        if len(wps) == 1:
            # 제자리 목적지 (관제가 현 위치로 호출) — 정지 명령이 나가도록 복제한다
            wps.append(wps[0])
        cum = [0.0]
        for a, b in zip(wps, wps[1:]):
            cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self._wps, self._cum, self._idx = wps, cum, 0
        self._target_allows_seg = False
        self._seg_ok_now = False
        self._near_corner = False
        self._origin = None
        self._best_remaining = float("inf")

    def _local_ff(self, pts) -> list[float]:
        """waypoint별 FF 바퀴각 — 국소 곡률 κ에서 atan(축거·κ). 부호 +우 (§2.7 반대).

        κ_i = 이웃 방향벡터의 부호 있는 회전각 / 호길이. ±2점(0.04m) 이동평균으로
        재샘플 잡음을 눌러 서보가 튀지 않게 한다. 직선부는 자연히 0."""
        n = len(pts)
        if n < 3:
            return [0.0] * n
        raw = [0.0] * n
        for i in range(1, n - 1):
            v1x, v1y = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
            v2x, v2y = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
            ds = (math.hypot(v1x, v1y) + math.hypot(v2x, v2y)) / 2.0
            if ds < 1e-9:
                continue
            dth = math.atan2(v1x * v2y - v1y * v2x, v1x * v2x + v1y * v2y)
            raw[i] = -math.degrees(math.atan(self._wheelbase * dth / ds))
        raw[0], raw[-1] = raw[1], raw[-2]
        out = []
        for i in range(n):
            lo, hi = max(0, i - 2), min(n, i + 3)
            out.append(sum(raw[lo:hi]) / (hi - lo))
        return out

    def clear(self) -> None:
        self._wps, self._cum, self._idx = [], [], 0
        self._target_allows_seg = False
        self._seg_ok_now = False
        self._near_corner = False
        self._origin = None
        self._best_remaining = float("inf")

    @property
    def target_allows_seg(self) -> bool:
        """현재 목표 waypoint가 seg 보정 허용 구간(lane·follow)에 있는가."""
        return self._target_allows_seg

    @property
    def seg_allowed(self) -> bool:
        """seg 보정 허용 — 목표뿐 아니라 **차량 위치**도 코너 창 밖일 때만.

        목표(lookahead 앞) 기준만 쓰면 코너 진입·한중간에 보정이 새어 들어온다
        (2026-08-05 저녁 주행: 코너 중 시야가 돌아가 heading +26° 오인 →
        ±8° 풀클램프 보정 → 출구 편향 → 도착 지나침). FF의 위상 결함과 동일 계열.
        """
        return self._seg_ok_now

    @property
    def has_route(self) -> bool:
        return bool(self._wps)

    # ---------- 제어 계산 ----------

    def compute(self, x: float, y: float, heading_deg: float) -> tuple[ControlCommand, float]:
        """(제어 명령, 목적지까지 남은 거리). 경로 없으면 정지."""
        if not self._wps:
            return STOP_COMMAND, 0.0

        # 목표점 전진 (전진만 한다 — 후퇴 없음):
        #   ① lookahead 이내로 들어온 waypoint는 통과한 것으로 본다
        #   ② 다음 waypoint가 지금 목표보다 가까우면 목표를 지나친 것이다 (경로 중간
        #      시작·오버슈트 복원 — 목표가 등 뒤에 남아 역조향하는 것을 막는다)
        # 코너 창 안에서는 시선 단축 — 긴 lookahead가 진입 전에 호 위 목표를 바라보며
        # 미리 꺾는 조기 커팅을 만든다 (2026-08-05 저녁 좌회전: 사전 13cm 파고듦 →
        # 비정렬 진입 → 출구 밀림 S자). 판정은 전 tick의 코너 창(20ms 지연 = 2mm) 사용
        lookahead = self._corner_lookahead if self._near_corner else self._lookahead
        last = len(self._wps) - 1
        while self._idx < last:
            d_cur = math.hypot(self._wps[self._idx][0] - x, self._wps[self._idx][1] - y)
            if d_cur < lookahead:
                self._idx += 1
                continue
            d_next = math.hypot(self._wps[self._idx + 1][0] - x,
                                self._wps[self._idx + 1][1] - y)
            if d_next < d_cur:
                self._idx += 1
                continue
            break
        tx, ty, speed_limit, seg_ok, _ = self._wps[self._idx]
        self._target_allows_seg = seg_ok

        remaining = (self._cum[last] - self._cum[self._idx]) + math.hypot(tx - x, ty - y)
        if remaining <= self._stop_radius:
            self._seg_ok_now = False
            return STOP_COMMAND, remaining   # 도착권 — throttle 0 (§19-4), 조향 중앙

        # 지나침 감지 — 최종 목표에 최근접한 뒤 다시 멀어지면 그 자리 정지.
        # 정지 트리거(0.05)를 못 밟고 스친 경우 되돌기 배회(2026-08-05 저녁: 맵 밖
        # 이탈)를 만들지 않는다. 정지 후 판정은 도착 게이트(0.15)의 몫.
        if self._idx == last:
            if remaining < self._best_remaining:
                self._best_remaining = remaining
            elif remaining > self._best_remaining + PASSED_HYSTERESIS_M:
                self._seg_ok_now = False
                return STOP_COMMAND, remaining

        # FF는 목표(lookahead 앞)가 아니라 차량의 경로상 위치에서 읽는다 — 목표 기준이면
        # 진입 전 선회 시작 + 호 중간에 FF 소멸로 안쪽(우회전=인도) 커팅
        # (2026-08-05 turn_param_sim: 목표 기준 이탈 12.2cm 안쪽 / 차량 기준 정상)
        s_car = max(0.0, self._cum[self._idx] - math.hypot(tx - x, ty - y))
        ff_deg = self._wps[max(0, bisect.bisect_right(self._cum, s_car) - 1)][4]
        if ff_deg == 0.0 and self._ff_lead > 0.0:
            # 선행 개시 — 서보 스윙(~0.2s)과 저속 관성 때문에 호 시작에서 꺾기 시작하면
            # 진입점을 지나친다 (2026-08-05 저녁 실차: "횡단보도부터 회전 시작해야").
            # 진입 lead 안이면 다가오는 호의 FF를 미리 건다. 출구는 무연장 (현 호 우선)
            ahead = bisect.bisect_right(self._cum, s_car + self._ff_lead) - 1
            ff_deg = self._wps[max(0, ahead)][4]

        # 조향 = 피드포워드(호 기하) + P 보정. 직선·직진 커넥터는 FF 0이라 기존과 동일.
        # 오차 +(목표 왼쪽) → 보정 −(좌). 풀락 오차에서 최대 보정 (taxi 기하)
        target_heading = math.degrees(math.atan2(ty - y, tx - x))
        err = geometry.heading_diff_deg(target_heading, heading_deg)
        wheel = ff_deg - self._max_wheel * max(-1.0, min(1.0, err / self._full_lock_err))
        wheel = max(-self._max_wheel, min(self._max_wheel, wheel))

        if seg_ok:
            throttle = self._cruise * min(1.0, speed_limit / self._max_speed)
            # 도착 감속은 차선 목표에서만 — 커넥터는 turn 고정이라 겹감속(데드존 진입) 금지
            throttle *= self._slowdown_ratio(remaining)
        else:
            # 회전(커넥터) 고정 스로틀 — 2026-08-03 실차: cruise와 분리해 회전 속도를 직접 지정
            throttle = self._turn
        if abs(err) > self._align_err and seg_ok:   # 커넥터 기동 중 면제 (모듈 docstring)
            throttle *= self._align_ratio

        # 특수 상황 스로틀 (2026-08-04 실차: 픽업 오버슈트·재출발 스윙 / 회전 중 토크 강하)
        if self._origin is None:
            self._origin = (x, y)                    # 경로 첫 compute 위치 = 출발점
        if math.hypot(x - self._origin[0], y - self._origin[1]) < self._ramp_dist:
            throttle = min(throttle, self._turn)     # 재출발 램프
        near_corner = (self._connector_within(x, y, tx, ty)
                       or self._connector_behind(x, y, tx, ty))
        if near_corner:
            # 커넥터 근접(앞 corner_slowdown / 뒤 corner_exit_hold)은 회전 스로틀 그 자체.
            # min이 아니라 할당 — 직진<회전 역전 구성(8/4)에서는 올려야 회전 토크가 유지된다
            throttle = self._turn
        # seg 보정 허용은 목표뿐 아니라 차량도 코너 창 밖일 때 (seg_allowed docstring)
        self._seg_ok_now = seg_ok and not near_corner
        self._near_corner = near_corner        # 다음 tick의 시선 단축 판정
        return ControlCommand(throttle=throttle, steering_wheel_deg=wheel), remaining

    def _connector_behind(self, x: float, y: float, tx: float, ty: float) -> bool:
        """직전 통과한 커넥터 끝에서 corner_exit_hold 거리 안인가 — 회전 마무리 토크 유지.

        lookahead 때문에 목표가 코너 출구 차선으로 먼저 넘어가면서 회전 한중간에
        스로틀이 차선 값으로 떨어지는 것을 막는다 (2026-08-04 동쪽 삼거리 우회전 실측).
        """
        s_car = self._cum[self._idx] - math.hypot(tx - x, ty - y)
        j = self._idx - 1
        while j >= 0:
            passed = s_car - self._cum[j]
            if passed > self._exit_hold:
                return False        # 더 뒤는 전부 이 거리 밖 (cum 단조성)
            if not self._wps[j][3]:
                return True
            j -= 1
        return False

    def _connector_within(self, x: float, y: float, tx: float, ty: float) -> bool:
        """차량에서 경로상 corner_slowdown 거리 안에 커넥터(seg_ok=False) waypoint가 있는가."""
        dist_to_target = math.hypot(tx - x, ty - y)
        j = self._idx
        last = len(self._wps) - 1
        while j <= last:
            along = (self._cum[j] - self._cum[self._idx]) + dist_to_target
            if along > self._corner_slow:
                return False
            if not self._wps[j][3]:
                return True
            j += 1
        return False

    def _slowdown_ratio(self, remaining: float) -> float:
        """taxi _throttle_for_distance — 감속 구간에서 near_ratio→1.0 선형."""
        if remaining >= self._slowdown:
            return 1.0
        usable = max(self._slowdown - self._stop_radius, 1e-9)
        progress = max(0.0, min(1.0, (remaining - self._stop_radius) / usable))
        return self._near_ratio + (1.0 - self._near_ratio) * progress
