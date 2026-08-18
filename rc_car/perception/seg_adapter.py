"""SegAdapter — seg 모델과 주행 코드 사이의 경계 (계약 v0.3 §5·§6 임베디드 측).

계약 v0.3 (2026-08-03 개정): 카메라는 vision 소유 — vision이 자체 캡처·추론 루프를
돌리고, 임베디드는 model.latest()로 최신 결과를 논블로킹 조회한다(§3, pull).
infer(frame) 동기 호출과 임베디드 캡처 가정(v0.2 §4)은 폐기.

책임:
- SegResult 해석: 필수 4필드(valid·offset·heading·timestamp) 검증,
  **모르는 필드 무시**(§5), 비정상 값 → invalid
- **신선도 판정**(§5 개정): pull은 같은 결과를 중복 조회할 수 있다 —
  timestamp가 freshness_max_s(잠정 0.5s)보다 오래되면 stale = invalid
- §6.1 방어: 계약상 latest()는 예외를 던지지 않지만, 위반해도 주행 스레드가 죽지 않게 삼킨다
- **부호 반전**: 계약 +(좌측 보정 필요) → rc_car 바퀴각 −(좌) (§5.1 "어댑터 책임")
- 보정 계산·제한: correction = −(k_off·offset + k_head·err), ±max 클램프 (명세서 §18-3
  "보정값은 반드시 제한하고, 신뢰도가 낮으면 사용하지 않는다")
- §6.2: invalid 지속 시 GPS 폴백 — 융합 구조상 폴백 = 보정 0 (기저 조향은 항상 GPS 추종).
  연속 invalid 가 임계에 닿으면 전환 로그를 남긴다

seg 관측은 미션·차량 상태가 아니라 조향 입력원의 내부 신호이므로 StateStore가 아닌
어댑터가 보관한다 (perception worker가 쓰고 DrivingWorker가 읽는다 — 자체 락).
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass

log = logging.getLogger("perception.seg")


@dataclass(frozen=True)
class SegObservation:
    valid: bool
    lateral_offset_m: float
    heading_error_deg: float


_INVALID = SegObservation(False, 0.0, 0.0)


class SegAdapter:
    def __init__(self, model, correction_cfg: dict):
        """model = 계약 §3.1 형태 객체 (mock 또는 실모델). cfg = perception.yaml seg_correction."""
        self._model = model
        self._k_off = float(correction_cfg["offset_gain_wheel_deg_per_m"])
        self._k_head = float(correction_cfg["heading_gain_wheel_deg_per_deg"])
        self._max_corr = float(correction_cfg["max_correction_wheel_deg"])
        self._fallback_after = int(correction_cfg["invalid_fallback_after"])
        self._freshness_max = float(correction_cfg["freshness_max_s"])
        self._lock = threading.Lock()
        self._latest = _INVALID
        self._invalid_streak = 0
        self._fallback_logged = False

    # ---------- 모델 수명 (Runtime 이 호출) ----------

    def load(self, config: dict | None = None) -> None:
        self._model.load(config or {})   # load 실패는 예외 그대로 — 기동 거부 (§6.1)

    def close(self) -> None:
        self._model.close()

    # ---------- 관측 (perception worker 스레드, 10Hz pull) ----------

    def observe(self) -> SegObservation:
        try:
            raw = self._model.latest()   # v0.3 §3: vision 자체 루프의 최신값 조회 (논블로킹)
        except Exception:                # §6.1 위반 모델 방어 — 주행 스레드 보호
            log.exception("seg latest 예외 (계약 §6.1 위반) — invalid 처리")
            raw = None
        obs = self._parse(raw)
        with self._lock:
            self._latest = obs
            if obs.valid:
                if self._invalid_streak >= self._fallback_after:
                    log.info("seg 복귀 — 보정 재개 (§6.2)")
                self._invalid_streak = 0
                self._fallback_logged = False
            else:
                self._invalid_streak += 1
                if (self._invalid_streak == self._fallback_after
                        and not self._fallback_logged):
                    self._fallback_logged = True
                    log.info("seg invalid %d회 연속 — GPS 폴백 유지 (§6.2)",
                             self._invalid_streak)
        return obs

    # ---------- 소비 (DrivingWorker, 50Hz) ----------

    def latest(self) -> SegObservation:
        with self._lock:
            return self._latest

    def correction_wheel_deg(self, obs: SegObservation) -> float:
        """§5 부호(+ = 좌측 보정 필요) → 바퀴각(+우) 보정. invalid 는 0 (폴백 = 기저 GPS)."""
        if not obs.valid:
            return 0.0
        corr = -(self._k_off * obs.lateral_offset_m
                 + self._k_head * obs.heading_error_deg)
        return max(-self._max_corr, min(self._max_corr, corr))

    # ---------- 내부 ----------

    def _parse(self, raw) -> SegObservation:
        """dict 권장(§5) — 속성 객체도 수용. 필수 4필드 없거나 비정상이면 invalid."""
        if raw is None:
            return _INVALID
        get = raw.get if isinstance(raw, dict) else lambda k, d=None: getattr(raw, k, d)
        valid = get("valid", None)
        if valid is not True:
            return _INVALID
        try:
            offset = float(get("lateral_offset_m"))
            err = float(get("heading_error_deg"))
            ts = float(get("timestamp"))          # v0.3 §5: timestamp 필수 (pull 신선도)
        except (TypeError, ValueError):
            log.warning("SegResult 필드 형식 오류 — invalid 처리 (§5)")
            return _INVALID
        if not (math.isfinite(offset) and math.isfinite(err)) or abs(err) >= 90.0:
            return _INVALID              # §5.2 값 범위 위반
        if time.time() - ts > self._freshness_max:
            return _INVALID              # stale — vision 루프 정지·중복 조회 (§5 개정)
        return SegObservation(True, offset, err)
