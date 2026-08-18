"""ArrivalChecker — 목적지 도착 판단 (명세서 §19, WBS D4).

도착 조건 (§19, 전부 만족해야 게이트 통과):
1. 마커 위치가 StopPose 반경(`control.arrival_radius_m`) 이내
2. 목표 차선에 매칭되어 있음
3. heading 오차가 `max_heading_error_deg` 이내
4. throttle 명령이 0
5. 추정 속도 ≤ `max_estimated_speed_mps` **또는** 게이트 유지 `settle_time_sec` 경과

5번이 OR 라는 점이 핵심 — 실 ArUco 노이즈로 추정 속도가 임계 아래로 안 내려가도
(워크로그 리스크 1) settle 시간 경과로 도착이 확정된다. A5 노이즈 실측은 임계 튜닝
문제로 남을 뿐 도착 실패를 만들지 않는다.

- pose 유실(stale) 중에는 판정을 진행하지 않는다 — 마지막 값으로 도착을 선언하면
  §3.4(이전 pose 계속 사용 금지) 위반이다. stale 이면 유지 시계를 리셋한다.
- 판정은 래치된다: 한 번 도착이면 start_mission()/clear() 전까지 유지
  (complete 재전송 중 조건이 흔들려도 번복하지 않는다).

호출자(단계 5 주행 로직): start_mission(StopPose) → 매 tick update(...) → True 시
CommandPolicy.notify_arrived() 1회 호출.
"""
from __future__ import annotations

import math

from mapping import geometry
from navigation.destination_resolver import StopPose


class ArrivalChecker:
    def __init__(self, radius_m: float, cfg: dict):
        """cfg = control.yaml `arrival` 섹션."""
        self._radius = float(radius_m)
        self._max_speed = float(cfg["max_estimated_speed_mps"])
        self._settle_sec = float(cfg["settle_time_sec"])
        self._max_heading_err = float(cfg["max_heading_error_deg"])
        self._stop: StopPose | None = None
        self._held_since: float | None = None
        self._arrived = False

    # ---------- 미션 수명 ----------

    def start_mission(self, stop_pose: StopPose) -> None:
        self._stop = stop_pose
        self._held_since = None
        self._arrived = False

    def clear(self) -> None:
        self._stop = None
        self._held_since = None
        self._arrived = False

    @property
    def arrived(self) -> bool:
        return self._arrived

    # ---------- 판정 ----------

    def update(self, x: float, y: float, heading_deg: float,
               estimated_speed_mps: float, matched_lane_ids: frozenset[str] | None,
               throttle_cmd: float, pose_stale: bool, now_s: float) -> bool:
        """매 제어 tick 호출. True = 도착 확정 (래치).

        `matched_lane_ids` = 지금 차가 "있다고 인정"되는 차선 집합
        (`LaneMap.arrival_lane_candidates`). 이음매 위에서는 진출·진입 차선이 함께
        들어오므로 단일 값 비교가 아니라 포함 검사다. 매칭 실패는 빈 집합/None.
        """
        if self._stop is None:
            return False
        if self._arrived:
            return True

        gate = (
            not pose_stale
            and math.hypot(x - self._stop.x, y - self._stop.y) <= self._radius  # §19-1
            and self._stop.lane_id in (matched_lane_ids or ())                  # §19-2
            and abs(geometry.heading_diff_deg(
                heading_deg, self._stop.heading_deg)) <= self._max_heading_err  # §19-3
            and abs(throttle_cmd) <= 1e-6                                       # §19-4
        )
        if not gate:
            self._held_since = None
            return False

        if self._held_since is None:
            self._held_since = now_s

        # §19-5: 속도 낮음 OR 유지 시간 경과
        if (estimated_speed_mps <= self._max_speed
                or now_s - self._held_since >= self._settle_sec):
            self._arrived = True
        return self._arrived
