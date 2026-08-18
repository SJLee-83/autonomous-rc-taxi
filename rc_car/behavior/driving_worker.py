"""DrivingWorker — 주행 판단의 배선 (WBS ⑤, 단계 5).

50Hz tick마다 상태에 따라:
  PLANNING            → 매칭(§12) → 목적지 스냅(§13) → 경로(§14) → FOLLOWING_ROUTE
  FOLLOWING_ROUTE /
  APPROACHING_DESTINATION → follower 조향 → SafetySupervisor.apply
                            → ArrivalChecker(§19) → 도착 시 CommandPolicy.notify_arrived()
  그 외(WAITING·ARRIVED·ERROR·BOOT) → 정지 명령 유지 (이중 안전 — stage1과 동일)

원칙:
- 모터 출력은 매 tick SafetySupervisor.apply() 한 번 — 다른 경로 없음
- pose 없음/stale 이면 계획도 주행도 하지 않는다 (Supervisor 가 어차피 차단하지만 명시)
- 오류 복귀(§7.2): ERROR 동안 경로·도착 판정 상태는 그대로 두므로, 직전 state 로
  복귀하면 같은 경로를 이어서 달린다. 미션이 사라졌을 때(stop 수락)만 정리한다.
- 매칭 실패 시 PLANNING 에 머문다 (§12.3 — 출발 금지). 주기적으로 경고만 남긴다.

follower 는 주입된 것을 그대로 쓴다 (타입 고정 없음) — 기본은 LaneFollower
(차선 번호 point-to-point + 2단 회전, 2026-08-06 §0-45), 환경변수로만 켜지는
폐기 보류 WaypointFollower(원호 추종)도 같은 인터페이스라 이 파일은 바뀌지 않는다:
    set_route(route) / compute(x, y, heading) / seg_allowed / has_route / clear()
"""
import logging
import threading
import time
from dataclasses import replace

from core.enums import DrivingState
from core.models import STOP_COMMAND
from core.state_store import StateStore
from mapping.map_matcher import MapMatcher
from navigation.lane_route import ID2NUM
from navigation.arrival_checker import ArrivalChecker
from navigation.destination_resolver import DestinationResolver
from navigation.route_planner import RoutePlanner
from safety.safety_supervisor import SafetySupervisor
from workers.periodic_worker import PeriodicWorker

log = logging.getLogger("behavior.driving")

_WARN_EVERY_SEC = 2.0   # 매칭 실패 경고 간격 (50Hz 스팸 방지)


class DrivingWorker(PeriodicWorker):
    def __init__(self, stop_event: threading.Event, store: StateStore,
                 supervisor: SafetySupervisor, policy, lane_map, matcher: MapMatcher,
                 resolver: DestinationResolver, planner: RoutePlanner,
                 arrival: ArrivalChecker, follower,
                 loop_hz: float, approach_distance_m: float, on_error=None,
                 seg_adapter=None, forced_routes=None):
        super().__init__("DrivingWorker", 1.0 / loop_hz, stop_event, on_error)
        self._seg = seg_adapter          # None = seg 미사용 (GPS 폴백만)
        # 강제 지정 번호 노선 (config/routes.yaml — 시나리오 확정 노선, 2026-08-06 안 1).
        # 플래너는 순수 길이 최단이라 사용자 확정 노선(안 1)을 스스로 고르지 않는다
        self._forced = tuple(tuple(int(n) for n in r) for r in (forced_routes or ()))
        self._store = store
        self._supervisor = supervisor
        self._policy = policy
        self._lane_map = lane_map
        self._matcher = matcher
        self._resolver = resolver
        self._planner = planner
        self._arrival = arrival
        self._follower = follower
        self._approach = approach_distance_m
        self._last_warn = 0.0

    def tick(self) -> None:
        snap = self._store.snapshot()
        state = snap.driving_state
        if state == DrivingState.PLANNING:
            self._plan(snap)
            self._supervisor.apply(STOP_COMMAND)          # 계획 중엔 정지 유지
        elif state in (DrivingState.FOLLOWING_ROUTE,
                       DrivingState.APPROACHING_DESTINATION):
            self._drive(snap)
        else:
            if snap.mission is None and self._follower.has_route:
                # stop 수락으로 미션이 사라짐 — 경로·도착 판정 정리
                self._follower.clear()
                self._arrival.clear()
            self._supervisor.apply(STOP_COMMAND)

    # ---------- PLANNING ----------

    def _plan(self, snap) -> None:
        if snap.mission is None:            # 방어 — 미션 없는 PLANNING은 없다
            self._store.set_driving_state(DrivingState.WAITING)
            return
        pose = snap.pose
        if pose is None or snap.pose_stale:
            return                          # 위치 확보 후 다음 tick에 재시도

        match = self._matcher.match(pose.x, pose.y, pose.heading_deg)
        if match is None:
            now = time.monotonic()
            if now - self._last_warn >= _WARN_EVERY_SEC:
                self._last_warn = now
                log.warning("차선 매칭 실패 — 출발 보류 (§12.3): (%.2f, %.2f) %.0f°",
                            pose.x, pose.y, pose.heading_deg)
            return
        if match.kind == "lane":
            start_lane, start_s = match.segment_id, match.progress_s
        else:
            # 커넥터 위 시작 (§12.3 "명확한 connector") — 진출 차선 시작을 경로의
            # 기점으로 삼고, 남은 커넥터 구간은 첫 waypoint 를 향한 조향으로 소화한다
            conn = self._lane_map.connectors[match.segment_id]
            start_lane, start_s = conn.successor, 0.0

        mission = snap.mission
        stop = self._resolver.resolve(mission.dest_x, mission.dest_y)
        forced = self._forced_route(start_lane, stop.lane_id)
        if forced is not None:
            route = self._planner.plan_via_numbers(forced, start_s, stop.progress_s)
            log.info("강제 노선 적용: %s (Dijkstra 우회 — config/routes.yaml)",
                     "->".join(str(n) for n in forced))
        else:
            route = self._planner.plan(start_lane, start_s, stop.lane_id, stop.progress_s)
        self._follower.set_route(route)
        self._arrival.start_mission(stop)
        self._store.set_driving_state(DrivingState.FOLLOWING_ROUTE)
        log.info("경로 확정: %s(s=%.2f) → %s(s=%.2f), %.2fm / 구간 %d개",
                 start_lane, start_s, stop.lane_id, stop.progress_s,
                 route.total_length_m, len(route.segments))

    def _forced_route(self, start_lane_id: str, dest_lane_id: str):
        """등록된 강제 노선 중 (목적 차선 = 종점) ∧ (현 차선이 노선 위) 인 것의 부분 노선.

        중간 차선에서 재호출·오류 복귀되면 그 지점부터 이어 탄다. 어느 노선에도
        해당하지 않으면 None → 기존 Dijkstra (§14). 시작 차선이 곧 종점이면
        자명 경로·한 바퀴 판단이 필요하므로 플래너에 맡긴다.
        """
        start, dest = ID2NUM.get(start_lane_id), ID2NUM.get(dest_lane_id)
        if start is None or dest is None:
            return None
        for nums in self._forced:
            if nums[-1] == dest and start in nums[:-1]:
                return nums[nums.index(start):]
        return None

    # ---------- FOLLOWING / APPROACHING ----------

    def _drive(self, snap) -> None:
        pose = snap.pose
        if pose is None or snap.pose_stale:
            # §3.4 — 이전 pose 로 주행 금지. 게이트 유지 시계도 리셋한다
            self._supervisor.apply(STOP_COMMAND)
            if pose is not None:
                self._arrival.update(pose.x, pose.y, pose.heading_deg,
                                     snap.estimated_speed_mps, frozenset(), 0.0,
                                     True, time.monotonic())
            return
        if not self._follower.has_route:    # 방어 — 주행 상태인데 경로 없음 → 재계획
            self._store.set_driving_state(DrivingState.PLANNING)
            return

        cmd, remaining = self._follower.compute(pose.x, pose.y, pose.heading_deg)
        if self._seg is not None and self._follower.seg_allowed:
            # 시연 조향 구조 (명세서 §18-3): 최종 조향 = 경로 추종 + 차선 중앙 보정.
            # 보정은 차선·follow 구간에서만, 그리고 **차량이 코너 창 밖일 때만** —
            # 목표(lookahead) 기준 게이트는 코너 중 보정 누수를 만든다 (2026-08-05 저녁).
            # invalid 면 보정 0 = GPS 폴백 그대로 (모델 실패는 정지 사유가 아니다 — 계약 §6.2)
            obs = self._seg.latest()
            corr = self._seg.correction_wheel_deg(obs)
            if corr != 0.0:
                cmd = replace(cmd, steering_wheel_deg=cmd.steering_wheel_deg + corr)
        if (snap.driving_state == DrivingState.FOLLOWING_ROUTE
                and remaining <= self._approach):
            self._store.set_driving_state(DrivingState.APPROACHING_DESTINATION)
        applied = self._supervisor.apply(cmd)

        # §19-2 차선 매칭 게이트. 이음매 위라면 진출·진입 차선을 모두 인정한다
        # (LaneMap.arrival_lane_candidates — 근거는 그쪽 docstring).
        # 반경 0.10m 게이트가 남아 있어 오용은 차단된다.
        match = self._matcher.match(pose.x, pose.y, pose.heading_deg)
        matched_lanes = (frozenset() if match is None
                         else self._lane_map.arrival_lane_candidates(match.segment_id))
        arrived = self._arrival.update(
            pose.x, pose.y, pose.heading_deg, snap.estimated_speed_mps,
            matched_lanes, applied.throttle, False, time.monotonic())
        if arrived:
            self._policy.notify_arrived()   # ARRIVED 전이 + 정지 + complete 재전송 (§8)
