"""HeadingEstimator — 연속 헤딩 추정 (2026-08-08, HY 프로젝트 방식 이식).

왜 필요한가 (0808 실측)
    마커 heading 이 주행마다 크게 뒤집힌다:
        7번  GPS  19.2° vs 실제 진행 190.3°  (-171.1°)
        22번 GPS  71.2° vs 실제 진행 240.9°  (-169.7°)
        19번 GPS 138.9° vs 실제 진행 329.4°  (+169.5°)
    기존 방어(control/lane_follower._trusted_heading)는 **이진 전환**이다 — 임계 25° 를
    넘으면 GPS 를 버리고 코스각으로 갈아탄다. 15:10 주행 19->20 구간에서 그 전환이
    **30초에 19회** 일어났고, 두 값이 최대 169° 차이라 전환할 때마다 추종 목표가
    통째로 뒤집혀 중심선 좌우로 **진폭 0.37m 왕복**(최서단 x=0.016 = 맵 경계 1.6cm)이 났다.

    이 추정기는 **전환하지 않는다.** 항상 도는 상태 하나를 유지하고 관측을 섞기만 한다.
    그래서 토글이 구조적으로 일어날 수 없다.

추정 우선순위 (헤딩보정.md §6)
    1. 이전 추정 헤딩
    2. 자전거 모델 예측 — 적용된 제어 명령 + 추정 속도
    3. GPS 위치 변화가 그린 코스각 — course_alpha 로 혼합 (최소 이동거리 필요)
    4. 마커 heading — 초기화 때, 그리고 marker_alpha(아주 작게)로만

    4번이 핵심이다. 마커 heading 을 매 프레임 강하게 반영하면 정지 상태의 자세 노이즈를
    그대로 따라간다 (0807 실측: 정지 중 heading 출렁임 최대 27°).

각도 결합
    반드시 wrap 오차로 한다. 359° 와 1° 의 산술 평균은 180° 가 되어 정반대를 가리킨다.
        angle_error = (target - current + 180) % 360 - 180
        blended     = current + alpha * angle_error

모델
    DeadReckoner 와 같은 실측 기반이지만 **별도 구현**이다 — DR 은 터널 모드의 앵커·
    맹목 한도에 묶여 있어 상시 추정에 쓰면 그쪽 안전 로직이 흔들린다. 수치는 같은
    config(tunnel) 를 공유한다: 스로틀→속도 실측 보간 / 서보 속도 제한 / 바퀴각 편향.
    부호 규약: 바퀴각 +(우) → heading 감소 (§2.7 반시계 +).

⚠️ enabled=false 면 마커 heading 을 그대로 돌려준다 (기존 동작). 키가 없으면 꺼짐이다.
"""
from __future__ import annotations

import math


def blend_angle(current_deg: float, target_deg: float, alpha: float) -> float:
    """wrap 안전 각도 혼합. alpha=0 이면 current, 1 이면 target."""
    err = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    return (current_deg + alpha * err) % 360.0


class HeadingEstimator:
    _STEP_S = 0.02          # 적분 간격 — DeadReckoner 와 동일

    def __init__(self, cfg: dict, motion_cfg: dict, wheelbase_m: float):
        cfg = dict(cfg or {})
        self._enabled = bool(cfg.get("enabled", False))
        self._course_min = float(cfg.get("course_min_distance_m", 0.04))
        self._a_course = float(cfg.get("course_alpha", 0.45))
        self._a_marker = float(cfg.get("marker_alpha", 0.03))
        # 자전거 모델 — tunnel 과 같은 실측값을 공유한다
        self._wheelbase = float(wheelbase_m)
        self._bias = float(motion_cfg["wheel_bias_deg"])
        self._servo_rate = float(motion_cfg["servo_rate_dps"])
        self._accel = float(motion_cfg.get("accel_mps2", 0.5))
        self._pts = [(float(a), float(b)) for a, b in motion_cfg["speed_points"]]
        self.reset()

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def initialized(self) -> bool:
        return self._heading is not None

    def reset(self) -> None:
        """유실 확정 등으로 상태를 버린다 — 다음 pose 에서 마커 heading 으로 재초기화."""
        self._heading = None
        self._ax = self._ay = None      # 코스각 기준점 (anchor)
        self._t = 0.0
        self._wheel = 0.0               # 서보 추정 현재각 (명령이 아니라 추종 결과)
        self._v = 0.0
        self._last_course_deg = None    # 관측용 — 마지막으로 섞은 코스각

    @property
    def last_course_deg(self):
        return self._last_course_deg

    def update(self, x: float, y: float, marker_heading_deg: float,
               throttle: float, wheel_cmd_deg: float, monotonic_s: float) -> float:
        """새 pose 마다 호출. 추정 heading(0~360)을 돌려준다.

        enabled=false 면 마커 값을 그대로 통과시킨다 — 호출측이 분기할 필요가 없다.
        """
        if not self._enabled:
            return marker_heading_deg

        if self._heading is None:                       # ① 초기화 = 마커 heading
            self._heading = float(marker_heading_deg) % 360.0
            self._ax, self._ay = float(x), float(y)
            self._t = float(monotonic_s)
            return self._heading

        self._propagate(throttle, wheel_cmd_deg, monotonic_s)   # ② 자전거 모델

        dx, dy = x - self._ax, y - self._ay
        moved = math.hypot(dx, dy)
        if moved >= self._course_min:                   # ③ 코스각 혼합
            course = math.degrees(math.atan2(dy, dx)) % 360.0
            self._heading = blend_angle(self._heading, course, self._a_course)
            self._last_course_deg = course
            self._ax, self._ay = float(x), float(y)
        else:                                           # ④ 마커 heading 약한 보정
            self._heading = blend_angle(
                self._heading, float(marker_heading_deg), self._a_marker)
        return self._heading

    # ---------- 내부 ----------

    def _propagate(self, throttle: float, wheel_cmd_deg: float,
                   monotonic_s: float) -> None:
        """마지막 시각 → monotonic_s 까지 heading 을 자전거 모델로 전진."""
        if monotonic_s <= self._t:
            return
        remain = monotonic_s - self._t
        target_v = self._speed(throttle)
        while remain > 1e-9:
            dt = min(self._STEP_S, remain)
            remain -= dt
            dw = wheel_cmd_deg - self._wheel                    # 서보 속도 제한
            step = self._servo_rate * dt
            self._wheel += max(-step, min(step, dw))
            dv = self._accel * dt                               # 가감속 슬루
            self._v = (min(target_v, self._v + dv) if target_v > self._v
                       else max(target_v, self._v - dv))
            self._heading -= math.degrees(                      # +우 → heading 감소
                self._v / self._wheelbase
                * math.tan(math.radians(self._wheel + self._bias)) * dt)
        self._heading %= 360.0
        self._t = monotonic_s

    def _speed(self, throttle: float) -> float:
        """스로틀→속도 실측 보간. 첫 점 이하(데드존)는 0. DeadReckoner 와 동일."""
        pts = self._pts
        if throttle <= pts[0][0]:
            return 0.0
        for (a, va), (b, vb) in zip(pts, pts[1:]):
            if throttle <= b:
                return va + (vb - va) * (throttle - a) / (b - a)
        return pts[-1][1]
