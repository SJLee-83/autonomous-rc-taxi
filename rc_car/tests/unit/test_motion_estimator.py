"""MotionEstimator 단위 테스트 — pose 차분 속도 추정 (protocol_2 §4.3)."""
import math
import threading

import pytest

from core.models import LocalizationPose
from core.state_store import StateStore
from localization.localization_service import LocalizationService
from localization.motion_estimator import MotionEstimator
from localization.pose_validator import PoseValidator

DT = 0.1                      # 10Hz


def pose(x=0.0, y=0.0, t=1000.0, heading=90.0):
    return LocalizationPose(x=x, y=y, heading_deg=heading,
                            source_timestamp_s=t, received_monotonic_s=t)


def feed(est, points, t0=1000.0, dt=DT):
    """(x, y) 목록을 dt 간격으로 넣고 마지막 추정치를 돌려준다."""
    last = 0.0
    for i, (x, y) in enumerate(points):
        last = est.update(pose(x, y, t0 + i * dt))
    return last


# ---------- 기본 동작 ----------

def test_창_크기가_2_미만이면_거부():
    with pytest.raises(ValueError):
        MotionEstimator(window_poses=1, max_gap_sec=0.6)


def test_첫_pose는_속도가_0():
    est = MotionEstimator(window_poses=5, max_gap_sec=0.6)
    assert est.update(pose(1.0, 1.0)) == 0.0
    assert est.sample_count == 0


def test_등속_직진의_속도를_정확히_낸다():
    # 0.02m / 0.1s = 0.2 m/s (차량 최대 0.2076에 가까운 값)
    est = MotionEstimator(window_poses=5, max_gap_sec=0.6)
    speed = feed(est, [(1.0 + 0.02 * i, 1.0) for i in range(5)])
    assert speed == pytest.approx(0.2)
    assert est.sample_count == 4          # 창 5 pose = 구간 4개


def test_정지_상태는_0():
    est = MotionEstimator(window_poses=5, max_gap_sec=0.6)
    assert feed(est, [(2.0, 1.5)] * 5) == pytest.approx(0.0)


def test_대각선_이동은_유클리드_거리로_계산():
    est = MotionEstimator(window_poses=3, max_gap_sec=0.6)
    speed = feed(est, [(1.0, 1.0), (1.03, 1.04)])      # 0.05m / 0.1s
    assert speed == pytest.approx(0.5)


# ---------- 창(window) 동작 ----------

def test_오래된_구간은_창_밖으로_밀려난다():
    est = MotionEstimator(window_poses=3, max_gap_sec=0.6)   # 구간 2개만 유지
    # 빠르게 3구간 이동 후 정지하면, 창이 비워지며 0으로 수렴한다
    feed(est, [(0.0, 0.0), (0.05, 0.0), (0.10, 0.0), (0.15, 0.0)])
    assert est.speed_mps == pytest.approx(0.5)
    est.update(pose(0.15, 0.0, 1000.4))
    est.update(pose(0.15, 0.0, 1000.5))
    assert est.speed_mps == pytest.approx(0.0)   # 이동 구간이 전부 밀려남


def test_이동평균이_순간값보다_노이즈에_둔감하다():
    """§4.3의 존재 이유 — 한 프레임 튐이 그대로 speed가 되면 안 된다."""
    est = MotionEstimator(window_poses=5, max_gap_sec=0.6)
    # 정지 중 한 프레임만 2cm 튀었다가 제자리로 (순간값이라면 0.2 m/s가 두 번)
    feed(est, [(1.0, 1.0), (1.0, 1.0), (1.02, 1.0), (1.0, 1.0), (1.0, 1.0)])
    assert est.speed_mps == pytest.approx(0.1)   # 0.2+0.2를 4구간에 나눠 흡수


# ---------- 불연속 방어 ----------

def test_시간_역전이나_동일_시각은_창을_비운다():
    est = MotionEstimator(window_poses=5, max_gap_sec=0.6)
    feed(est, [(0.0, 0.0), (0.02, 0.0), (0.04, 0.0)])
    assert est.speed_mps > 0
    est.update(pose(0.06, 0.0, 1000.1))     # 시각 역전 (검증기가 걸러야 할 값)
    assert est.speed_mps == 0.0


def test_긴_공백_뒤에는_이어_붙이지_않는다():
    est = MotionEstimator(window_poses=5, max_gap_sec=0.6)
    feed(est, [(0.0, 0.0), (0.02, 0.0)])
    est.update(pose(1.0, 1.0, 1002.0))       # 2초 공백 + 큰 이동
    assert est.speed_mps == 0.0              # 없는 이동을 만들어내지 않는다


def test_reset은_창과_기준점을_모두_비운다():
    est = MotionEstimator(window_poses=5, max_gap_sec=0.6)
    feed(est, [(0.0, 0.0), (0.02, 0.0), (0.04, 0.0)])
    est.reset()
    assert est.speed_mps == 0.0 and est.sample_count == 0
    assert est.update(pose(5.0, 3.0, 2000.0)) == 0.0   # 기준점도 사라져 첫 pose 취급


# ---------- LocalizationService 통합 ----------

CFG = {
    "marker_id": 4, "interval_ms": 100, "marker_yaw_offset_deg": 0,
    "max_jump_m": 0.08, "max_heading_jump_deg": 15.0,
    "pose_timeout_sec": 0.3, "lost_hold_sec": 0.3,
}


def make_service():
    store = StateStore()
    svc = LocalizationService(
        threading.Event(), store, PoseValidator(CFG, 5.0, 3.0), CFG,
        on_lost=lambda: None, on_recovered=lambda: None,
        estimator=MotionEstimator(window_poses=5, max_gap_sec=0.3))
    return store, svc


def test_서비스가_채택_pose마다_속도를_갱신한다():
    import json
    store, svc = make_service()
    for i in range(5):
        svc.on_raw(json.dumps({
            "marker_id": 4, "found": True, "timestamp": 1000.0 + i * DT,
            "position": {"x": 1.0 + 0.02 * i, "y": 1.0, "z": 0.1}, "heading": 90.0}))
    assert store.snapshot().estimated_speed_mps == pytest.approx(0.2)


def test_유실되면_속도가_0으로_떨어진다():
    """마지막 속도를 들고 있으면 도착 판정·관제 표시가 거짓말을 한다."""
    import json
    import time
    store, svc = make_service()
    for i in range(3):
        svc.on_raw(json.dumps({
            "marker_id": 4, "found": True, "timestamp": 1000.0 + i * DT,
            "position": {"x": 1.0 + 0.02 * i, "y": 1.0, "z": 0.1}, "heading": 90.0}))
    assert store.snapshot().estimated_speed_mps > 0

    time.sleep(CFG["pose_timeout_sec"] + 0.05)
    svc.tick()
    assert store.snapshot().estimated_speed_mps == 0.0
    assert store.snapshot().pose_stale is True


def test_속도는_음수가_될_수_없다():
    """후진이 없다 (§4.3). 거리는 항상 양수라 구조적으로 보장된다."""
    est = MotionEstimator(window_poses=5, max_gap_sec=0.6)
    speed = feed(est, [(1.0, 1.0), (0.98, 1.0), (0.96, 1.0)])   # -x 방향 이동
    assert speed == pytest.approx(0.2) and speed >= 0.0
