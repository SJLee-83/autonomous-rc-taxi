"""MockSegModel — seg 계약 §8의 "정답 seg" (임베디드 제공 mock).

계약 v0.3 §3.1 형태(load/latest/close)를 그대로 따르되, 프레임 대신 **시뮬레이션 상
차량 pose와 차선 그래프에서 §5 값을 계산**해 반환한다. latest()는 pull이므로 호출
시점 pose로 즉석 계산 = 항상 신선(timestamp=now). 실모델과의 교체는 config 1줄
(perception.seg_mode) — 드라이버 mock↔real과 동일 패턴.

§5 부호 (계약 그림 기준):
- lateral_offset_m: 차선 중심이 차량 **왼쪽**이면 + (차량 진행 방향 기준)
- heading_error_deg: 차선 진행 − 차량 진행, 반시계 + (−90 < e < +90 정규화, §5.2)
둘 다 + = "좌측 보정 필요". 바퀴각(+우)으로의 반전은 어댑터 책임 (§5.1 참고).

valid=false 반환 (§6.1 — 예외 금지):
- pose 없음/stale (카메라로 치면 프레임 이상)
- 어느 차선 중심에서도 멀다 (차선 미검출 모사, 기본 0.25m)
- 차선 역방향 (|heading 오차| ≥ 90° — §5.2 역방향 해석 금지)
- 강제 invalid 스위치 on (폴백 전환 테스트용 — 계약 §8)
"""
from __future__ import annotations

import math
import time

from core.state_store import StateStore
from mapping import geometry
from mapping.lane_map import LaneMap

_DETECT_RANGE_M = 0.25   # 이보다 차선 중심에서 멀면 "차선 미검출"


class MockSegModel:
    def __init__(self, lane_map: LaneMap, store: StateStore):
        self._map = lane_map
        self._store = store
        self._force_invalid = False
        self._loaded = False

    # ---------- 계약 §3.1 인터페이스 ----------

    def load(self, config: dict) -> None:
        """실모델은 여기서 가중치 로드 (수 초 허용). mock은 즉시 완료."""
        self._loaded = True

    def latest(self) -> dict:
        """v0.3 §3 pull — mock은 pose가 곧 '완벽한 카메라'라 호출 시점에 즉석 계산."""
        if not self._loaded or self._force_invalid:
            return self._invalid()
        snap = self._store.snapshot()
        if snap.pose is None or snap.pose_stale:
            return self._invalid()

        x, y, heading = snap.pose.x, snap.pose.y, snap.pose.heading_deg
        best = None   # (거리, 투영점, 차선 진행 방향, lane_id)
        for lid, lane in self._map.lanes.items():
            s, dist, q = geometry.project_point(lane.centerline, (x, y))
            if best is None or dist < best[0]:
                best = (dist, q, geometry.heading_at(lane.centerline, s), lid)
        dist, (qx, qy), lane_heading, lane_id = best
        if dist > _DETECT_RANGE_M:
            return self._invalid()
        err = geometry.heading_diff_deg(lane_heading, heading)
        if abs(err) >= 90.0:
            return self._invalid()          # 역방향 — §5.2

        # 부호: 차량 heading 기준 왼쪽(+y 회전 방향)에 중심이 있으면 +
        h = math.radians(heading)
        cross = (math.cos(h) * (qy - y) - math.sin(h) * (qx - x))
        offset = dist if cross > 0 else -dist
        return {
            "valid": True,
            "lateral_offset_m": offset,
            "heading_error_deg": err,
            "timestamp": time.time(),       # v0.3 §5 필수 — pull 신선도 판정용
            "debug_lane_id": lane_id,       # 모르는 필드 — 임베디드는 무시 (§5)
        }

    def close(self) -> None:
        self._loaded = False

    # ---------- mock 전용 (계약 §8 스위치) ----------

    def set_force_invalid(self, on: bool) -> None:
        self._force_invalid = on

    @staticmethod
    def _invalid() -> dict:
        return {"valid": False, "lateral_offset_m": 0.0, "heading_error_deg": 0.0,
                "timestamp": time.time()}
