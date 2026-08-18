"""StateStore — 최신 상태의 단일 관리 지점 (명세서 §9, SW 3원칙 2).

- threading.RLock 동기화, 스냅샷은 불변 객체 → 외부에 mutable을 노출하지 않는다.
- 오래된 pose 거부: source_timestamp_s가 직전 채택분 이하이면 반영하지 않는다 (§3.4).
"""
import threading
import time
from dataclasses import replace

from .enums import DrivingState, ServiceKind
from .models import ActiveMission, ControlCommand, LocalizationPose, VehicleSnapshot


class StateStore:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._snap = VehicleSnapshot()

    # ---------- 읽기 ----------

    def snapshot(self) -> VehicleSnapshot:
        with self._lock:
            return self._snap  # 불변 객체라 그대로 반환해도 안전

    # ---------- pose ----------

    def update_pose(self, pose: LocalizationPose) -> bool:
        """직전 채택 pose보다 source_timestamp_s가 큰 경우에만 반영. 성공 여부 반환."""
        with self._lock:
            prev = self._snap.pose
            if prev is not None and pose.source_timestamp_s <= prev.source_timestamp_s:
                return False
            self._snap = replace(self._snap, pose=pose, pose_stale=False)
            return True

    def mark_pose_stale(self) -> None:
        """위치 유실 — 마지막 값은 유지하되 stale 표시 (protocol_2 §4.2)."""
        with self._lock:
            self._snap = replace(self._snap, pose_stale=True)

    def set_estimated_speed(self, mps: float) -> None:
        with self._lock:
            self._snap = replace(self._snap, estimated_speed_mps=max(0.0, mps))

    # ---------- 상태 전이 ----------

    def set_driving_state(self, state: DrivingState) -> DrivingState:
        """전이 수행. ERROR 진입 시 직전 state를 자동 보관한다 (protocol_2 §7 복귀)."""
        with self._lock:
            prev = self._snap.driving_state
            keep = self._snap.prev_state_for_recovery
            if state == DrivingState.ERROR and prev != DrivingState.ERROR:
                keep = prev
            elif state != DrivingState.ERROR:
                keep = None
            self._snap = replace(self._snap, driving_state=state,
                                 prev_state_for_recovery=keep)
            return prev

    def recover_previous_state(self) -> DrivingState:
        """오류 정지 → 보관해 둔 직전 state로 복귀 (protocol_2 §7.2)."""
        with self._lock:
            target = self._snap.prev_state_for_recovery or DrivingState.WAITING
            self._snap = replace(self._snap, driving_state=target,
                                 prev_state_for_recovery=None)
            return target

    # ---------- 미션 ----------

    def set_mission(self, dest_x: float, dest_y: float, kind: ServiceKind) -> ActiveMission:
        with self._lock:
            m = ActiveMission(dest_x, dest_y, kind, time.time())
            self._snap = replace(self._snap, mission=m, service_kind=kind)
            return m

    def clear_mission(self, next_kind: "ServiceKind | None") -> None:
        """도착 확인(stop 수락) 후 폐기. next_kind는 stop 이후의 대기 성격."""
        with self._lock:
            self._snap = replace(self._snap, mission=None, service_kind=next_kind)

    # ---------- 제어 ----------

    def set_control_command(self, command: ControlCommand) -> None:
        with self._lock:
            self._snap = replace(self._snap, control_command=command)
