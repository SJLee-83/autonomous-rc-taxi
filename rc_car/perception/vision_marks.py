"""MarkAdapter — 노면표시 검출(2차 회전 트리거) 소비자 (2026-08-06 재설계 §0-45).

SegAdapter 와 나란히 있는 **별도 소비자**다. 같은 게시 파일을 읽지만 쓰임이 다르다:
    SegAdapter  : seg(차선 중앙 오프셋) → 직진 중 미세 횡보정
    MarkAdapter : 노면표시 검출 → **회전 개시 시점 결정**  ← 이 모듈

왜 나눴나: seg 는 0805 실측에서 유효율이 drive3 39% / drive4 0.3% 로 사실상 죽어 있었고,
반대로 노면표시는 direction_arrow 71~83% / stop_line 71~82% 로 잘 잡혔다.
"비전 활용도가 낮았던 건 모델 탓이 아니라 쓰는 방식 탓"이라는 것이 §0-45 의 결론이고,
그래서 잘 나오는 쪽(노면표시)을 독립 신호로 승격시킨다. SegAdapter 는 손대지 않는다
(정차 교차검증 오차 1.1cm 로 검증된 코드다).

트리거 판정 (TurnTable.TriggerSpec 기준)
    ① 클래스 일치        — crosswalk | stop_line | direction_arrow | dashed_line
    ② conf >= min_conf
    ③ 상자 **하단** y >= 높이 x near_row_frac   → 표식이 차에 그만큼 가까이 왔다
    ④ 상자 중심의 횡거리 <= max_lateral_m       → 남의 차선 표식 기각
       (0805: stop_line 검출 횡거리 최대 1.44m = 차선 5~6개 밖까지 잡혔다)

무중단 폴백: vision 프로세스가 죽으면 게시가 낡고 신선도 판정이 invalid → 트리거는
영원히 발화하지 않는다 → LaneFollower 가 좌표 폴백(차선 끝 개시)으로 회전한다.
즉 비전이 0이어도 주행은 계속된다.
"""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

log = logging.getLogger("perception.marks")


@dataclass(frozen=True)
class Mark:
    cls: str
    conf: float
    bottom_px: float       # 상자 하단 y — 클수록 차에 가깝다 (버드아이 아래가 근거리)
    center_x_px: float


@dataclass(frozen=True)
class MarkObservation:
    valid: bool
    marks: tuple[Mark, ...] = ()
    height_px: float = 0.0
    pixels_per_meter: float = 0.0
    axis_px: float = 0.0

    def lateral_m(self, mark: Mark) -> float:
        if self.pixels_per_meter <= 0:
            return 0.0
        return abs(mark.center_x_px - self.axis_px) / self.pixels_per_meter


_INVALID = MarkObservation(False)


# ---------- 소스 ----------

class FileMarkSource:
    """vision_runner 게시 파일 클라이언트 (RealSegModel 과 같은 파일, 다른 필드)."""

    def __init__(self, publish_path: str):
        self._path = Path(publish_path)

    def latest_payload(self) -> dict | None:
        try:
            return json.loads(self._path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None      # 원자적 게시라 드묾 — 파일 소실·경합 방어


class NullMarkSource:
    """비전 미가동 — 항상 검출 없음. seg_mode=off 나 시뮬에서 쓴다."""

    def latest_payload(self) -> dict | None:
        return None


class StaticMarkSource:
    """테스트·시뮬용 — 넣어둔 payload 를 그대로 돌려준다."""

    def __init__(self, payload: dict | None = None):
        self.payload = payload

    def latest_payload(self) -> dict | None:
        return self.payload


# ---------- 어댑터 ----------

class MarkAdapter:
    def __init__(self, source, freshness_max_s: float = 0.5,
                 axis_px: float | None = None):
        """axis_px = 차량 진행축 열. None 이면 게시된 버드아이 폭의 절반을 쓴다
        (현 보정 기준 954px 폭에 축 477px — 정확히 절반이다)."""
        self._source = source
        self._freshness_max = float(freshness_max_s)
        self._axis_override = axis_px
        self._lock = threading.Lock()
        self._latest = _INVALID
        self._stale_logged = False

    # ---------- 관측 (perception worker 스레드) ----------

    def observe(self) -> MarkObservation:
        try:
            payload = self._source.latest_payload()
        except Exception:                     # 소스 계약 위반 방어 — 주행 스레드 보호
            log.exception("mark 소스 예외 — invalid 처리")
            payload = None
        obs = self._parse(payload)
        with self._lock:
            self._latest = obs
            if obs.valid:
                self._stale_logged = False
            elif not self._stale_logged:
                self._stale_logged = True
                log.info("노면표시 관측 invalid — 회전은 좌표 폴백으로 진행")
        return obs

    # ---------- 소비 (DrivingWorker / LaneFollower) ----------

    def latest(self) -> MarkObservation:
        with self._lock:
            return self._latest

    def triggered(self, spec, obs: MarkObservation | None = None) -> bool:
        """TurnTable.TriggerSpec 기준 발화 여부. 관측이 invalid 면 항상 False."""
        if not spec.uses_vision:
            return False
        o = self.latest() if obs is None else obs
        if not o.valid or o.height_px <= 0:
            return False
        row = o.height_px * spec.near_row_frac
        for m in o.marks:
            if m.cls != spec.cls or m.conf < spec.min_conf:
                continue
            if m.bottom_px < row:
                continue                       # 아직 멀다
            if o.lateral_m(m) > spec.max_lateral_m:
                continue                       # 남의 차선 표식
            return True
        return False

    # ---------- 내부 ----------

    def _parse(self, payload) -> MarkObservation:
        if not isinstance(payload, dict):
            return _INVALID
        ts = payload.get("timestamp")
        try:
            if ts is None or time.time() - float(ts) > self._freshness_max:
                return _INVALID                # stale — vision 루프 정지·중복 조회
        except (TypeError, ValueError):
            return _INVALID

        size = payload.get("birdseye_size")
        if not (isinstance(size, (list, tuple)) and len(size) == 2):
            return _INVALID
        try:
            width, height = float(size[0]), float(size[1])
            ppm = float(payload.get("pixels_per_meter") or 0.0)
        except (TypeError, ValueError):
            return _INVALID
        if width <= 0 or height <= 0:
            return _INVALID

        axis = self._axis_override
        if axis is None:
            axis = payload.get("vehicle_axis_px")
        axis = float(axis) if axis is not None else width / 2.0

        model = payload.get("model")
        dets = model.get("detections") if isinstance(model, dict) else None
        marks: list[Mark] = []
        for d in dets or []:
            if not isinstance(d, dict):
                continue
            box = d.get("xyxy_px")
            cls = d.get("cls")
            if not (isinstance(box, (list, tuple)) and len(box) == 4) or not cls:
                continue
            try:
                x1, _y1, x2, y2 = (float(v) for v in box)
                conf = float(d.get("conf", 0.0))
            except (TypeError, ValueError):
                continue
            if not (math.isfinite(x1) and math.isfinite(x2) and math.isfinite(y2)):
                continue
            marks.append(Mark(str(cls), conf, y2, (x1 + x2) / 2.0))
        return MarkObservation(True, tuple(marks), height, ppm, axis)
