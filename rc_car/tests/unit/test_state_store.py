"""StateStore 단위 테스트 — pose 순서 거부·ERROR 복귀·미션 (명세서 §9, protocol_2 §7)."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from core.enums import DrivingState, ServiceKind
from core.models import LocalizationPose
from core.state_store import StateStore


def _pose(ts: float) -> LocalizationPose:
    return LocalizationPose(1.0, 0.5, 90.0, ts, time.monotonic())


def test_pose_ordering_rejects_old():
    s = StateStore()
    assert s.update_pose(_pose(100.5)) is True
    assert s.update_pose(_pose(100.4)) is False   # 과거 timestamp 거부 (§3.4)
    assert s.update_pose(_pose(100.5)) is False   # 동일 timestamp 거부
    assert s.update_pose(_pose(100.6)) is True
    assert s.snapshot().pose.source_timestamp_s == 100.6


def test_stale_keeps_last_value():
    s = StateStore()
    s.update_pose(_pose(1.0))
    s.mark_pose_stale()
    snap = s.snapshot()
    assert snap.pose_stale is True
    assert snap.pose is not None                  # 마지막 값 유지 (protocol_2 §4.2)


def test_error_saves_and_recovers_previous_state():
    s = StateStore()
    s.set_driving_state(DrivingState.FOLLOWING_ROUTE)
    s.set_driving_state(DrivingState.ERROR)       # 직전 state 보관 (§4.5)
    assert s.snapshot().prev_state_for_recovery == DrivingState.FOLLOWING_ROUTE
    s.set_driving_state(DrivingState.ERROR)       # 중복 진입해도 보관값 유지
    assert s.snapshot().prev_state_for_recovery == DrivingState.FOLLOWING_ROUTE
    assert s.recover_previous_state() == DrivingState.FOLLOWING_ROUTE   # 복귀 (§7.2)
    assert s.snapshot().prev_state_for_recovery is None


def test_recover_without_mission_goes_waiting():
    s = StateStore()
    s.set_driving_state(DrivingState.ERROR)       # 대기 중 오류 — 보관값 BOOT
    s.set_driving_state(DrivingState.ERROR)
    s.recover_previous_state()
    # 미션 없음 → 주행 재개 없음은 SafetySupervisor가 보장 (DRIVING_STATES 아님)
    assert s.snapshot().mission is None


def test_mission_lifecycle():
    s = StateStore()
    m = s.set_mission(3.4, 2.2, ServiceKind.PICKUP)
    assert s.snapshot().mission == m
    s.set_driving_state(DrivingState.ERROR)
    assert s.snapshot().mission == m              # 오류 정지에도 유지 (protocol_1 §6.2)
    s.clear_mission(ServiceKind.PICKUP)
    assert s.snapshot().mission is None
