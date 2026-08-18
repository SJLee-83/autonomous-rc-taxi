"""DeadReckoner — 명령 기반 추측 항법 (터널 모드의 코어, 2026-08-06).

동편 측위 유실 구역(8/4·8/5 실측 — 관찰기록 0805)에서 서버 pose 대신, 마지막 신뢰
pose를 앵커로 잡고 **적용된 제어 명령**(SafetySupervisor가 store에 남긴 값)을
자전거 모델로 적분해 pose를 이어간다. 순수 계산기 — 스레드·로깅·store를 모른다.

모델은 tools/turn_param_sim.py 와 같은 실측 기반이다:
- 스로틀→속도: 실측점 선형 보간 (2026-08-04 실차: 데드존 ~0.24 / 0.3→0.052 / 1.0→0.199)
- 조향: 서보 속도 제한 + 잔여 바퀴각 편향 (트림 118 잔차 -1.6°)
- 부호 규약: 바퀴각 +(우) → heading 감소 (§2.7 반시계+)

한도: 앵커 이후 누적 맹목 거리·시간이 상한을 넘으면 within_budget=False.
호출측(터널 모드)은 공급을 멈추고 기존 유실 사슬(§7.2 정지)로 떨어진다 —
규약 §3.4 예외는 이 한도 안에서만 열린다. 수치는 전부 config (실차 튜닝 예정).
"""
from __future__ import annotations

import math


class DeadReckoner:
    _STEP_S = 0.02          # 적분 간격 — 제어 주기와 동일 (회전 충실도)

    def __init__(self, cfg: dict, wheelbase_m: float):
        self._wheelbase = float(wheelbase_m)
        self._bias = float(cfg["wheel_bias_deg"])
        self._servo_rate = float(cfg["servo_rate_dps"])
        self._accel = float(cfg.get("accel_mps2", 0.5))
        self._pts = [(float(a), float(b)) for a, b in cfg["speed_points"]]
        self._max_dist = float(cfg["max_blind_distance_m"])
        self._max_time = float(cfg["max_blind_time_s"])
        self.reset()

    def reset(self) -> None:
        """앵커 해제 — 유실 확정 후 새 신뢰 pose가 올 때까지 예측 불가."""
        self._x = self._y = self._heading = None
        self._t = 0.0            # 마지막 적분 시각 (monotonic)
        self._ts0 = 0.0          # 앵커 pose의 서버 timestamp — 합성 pose 순서 보장용
        self._t0 = 0.0           # 앵커 시각 (monotonic)
        self._wheel = 0.0        # 서보 추정 현재각 (명령 아님)
        self._v = 0.0
        self._blind_dist = 0.0
        self._blind_time = 0.0

    # ---------- 상태 ----------

    @property
    def anchored(self) -> bool:
        return self._x is not None

    @property
    def within_budget(self) -> bool:
        return (self._blind_dist <= self._max_dist
                and self._blind_time <= self._max_time)

    @property
    def blind_distance_m(self) -> float:
        return self._blind_dist

    @property
    def max_blind_distance_m(self) -> float:
        return self._max_dist

    @property
    def blind_time_s(self) -> float:
        return self._blind_time

    def predict(self):
        """(x, y, heading_deg 0~360, 합성 source_timestamp). anchored 전 호출 금지.

        합성 timestamp = 앵커 서버시각 + 경과 - 50ms. 50ms 양보는 복귀 실측 pose의
        서버시각이 합성치보다 근소하게 뒤질 때 store 순서 검사(§3.4)에 밀리지 않게 한다.
        """
        ts = self._ts0 + (self._t - self._t0) - 0.05
        return self._x, self._y, self._heading % 360.0, ts

    def deviation_m(self, x: float, y: float) -> float:
        return math.hypot(x - self._x, y - self._y)

    # ---------- 갱신 ----------

    def anchor(self, x: float, y: float, heading_deg: float,
               monotonic_s: float, source_timestamp_s: float) -> None:
        """신뢰 pose 채택 — 기준 재설정 + 맹목 한도 리셋. 서보 추정각은 유지한다."""
        self._x, self._y, self._heading = float(x), float(y), float(heading_deg)
        self._t = self._t0 = float(monotonic_s)
        self._ts0 = float(source_timestamp_s)
        self._blind_dist = 0.0
        self._blind_time = 0.0

    def propagate(self, throttle: float, steering_wheel_deg: float,
                  monotonic_s: float) -> None:
        """마지막 적분 시각 → monotonic_s 까지 전진. 미앵커·과거 시각이면 무시."""
        if not self.anchored or monotonic_s <= self._t:
            return
        remain = monotonic_s - self._t
        target_v = self._speed(throttle)
        while remain > 1e-9:
            dt = min(self._STEP_S, remain)
            remain -= dt
            dw = steering_wheel_deg - self._wheel
            step = self._servo_rate * dt
            self._wheel += max(-step, min(step, dw))
            dv = self._accel * dt
            self._v = (min(target_v, self._v + dv) if target_v > self._v
                       else max(target_v, self._v - dv))
            self._heading -= math.degrees(
                self._v / self._wheelbase
                * math.tan(math.radians(self._wheel + self._bias)) * dt)
            self._x += self._v * math.cos(math.radians(self._heading)) * dt
            self._y += self._v * math.sin(math.radians(self._heading)) * dt
            self._blind_dist += self._v * dt
            self._blind_time += dt
        self._t = monotonic_s

    # ---------- 내부 ----------

    def _speed(self, throttle: float) -> float:
        """스로틀→속도 실측 보간. 첫 점 이하(데드존)는 0."""
        pts = self._pts
        if throttle <= pts[0][0]:
            return 0.0
        for (a, va), (b, vb) in zip(pts, pts[1:]):
            if throttle <= b:
                return va + (vb - va) * (throttle - a) / (b - a)
        return pts[-1][1]
