"""⑥ 테스트 — MockSegModel(계약 §8 정답 seg) + SegAdapter(§5·§6) + 조향 융합(§18-3).

부호 사슬이 핵심 검증 대상:
  차선 중심이 차량 왼쪽 → offset + (계약 §5.1)
  → 보정 바퀴각 − (rc_car +우 — 어댑터가 반전)
  → 차량이 좌로 붙어 중심 복귀
"""
import math
import time
import unittest

from core.enums import DrivingState, ServiceKind
from core.models import LocalizationPose
from core.state_store import StateStore
from perception.mock_seg_model import MockSegModel
from perception.seg_adapter import SegAdapter
from tests.unit.test_driving import (_make_stack, _set_pose, MAX_SPEED, WHEELBASE, _map)

CORR_CFG = {"offset_gain_wheel_deg_per_m": 30.0,
            "heading_gain_wheel_deg_per_deg": 0.3,
            "max_correction_wheel_deg": 8.0,
            "invalid_fallback_after": 3,
            "freshness_max_s": 0.5}


def _model_with_pose(x, y, heading):
    store = StateStore()
    store.update_pose(LocalizationPose(x, y, heading, 1.0, 1.0))
    m = MockSegModel(_map(), store)
    m.load({})
    return m, store


class TestMockSegModel(unittest.TestCase):
    def test_centered_on_lane(self):
        m, _ = _model_with_pose(1.2, 2.61, 0.0)   # top_inner_eb_w 정중앙 정방향
        r = m.latest()
        self.assertTrue(r["valid"])
        self.assertAlmostEqual(r["lateral_offset_m"], 0.0, places=6)
        self.assertAlmostEqual(r["heading_error_deg"], 0.0, places=6)

    def test_offset_sign_center_left_is_positive(self):
        # 차량이 중심(2.61) 아래(2.55), 동향 → 중심이 왼쪽 → +0.06 (계약 §5.1)
        m, _ = _model_with_pose(1.2, 2.55, 0.0)
        r = m.latest()
        self.assertTrue(r["valid"])
        self.assertAlmostEqual(r["lateral_offset_m"], +0.06, places=6)

    def test_offset_sign_center_right_is_negative(self):
        m, _ = _model_with_pose(1.2, 2.67, 0.0)
        r = m.latest()
        self.assertAlmostEqual(r["lateral_offset_m"], -0.06, places=6)

    def test_heading_error_ccw_positive(self):
        # 차량이 오른쪽(시계)으로 10° 틀어짐 → 오차 +10 (좌보정 필요)
        m, _ = _model_with_pose(1.2, 2.61, 350.0)
        r = m.latest()
        self.assertAlmostEqual(r["heading_error_deg"], +10.0, places=6)

    def test_reverse_direction_invalid(self):
        m, _ = _model_with_pose(1.2, 2.61, 180.0)  # 동행 차선에서 서향
        self.assertFalse(m.latest()["valid"])

    def test_far_from_any_lane_invalid(self):
        m, _ = _model_with_pose(1.2, 2.2, 0.0)     # block_nw 내부 — 중심선에서 멀다
        self.assertFalse(m.latest()["valid"])

    def test_stale_pose_invalid(self):
        m, store = _model_with_pose(1.2, 2.61, 0.0)
        store.mark_pose_stale()
        self.assertFalse(m.latest()["valid"])

    def test_force_invalid_switch(self):
        m, _ = _model_with_pose(1.2, 2.61, 0.0)
        m.set_force_invalid(True)
        self.assertFalse(m.latest()["valid"])
        m.set_force_invalid(False)
        self.assertTrue(m.latest()["valid"])


class _BrokenModel:
    """§6.1 위반 모델 — 어댑터 방어 확인용."""
    def load(self, config): pass
    def latest(self): raise RuntimeError("boom")
    def close(self): pass


class TestSegAdapter(unittest.TestCase):
    def _adapter(self, model=None):
        if model is None:
            model, _ = _model_with_pose(1.2, 2.55, 0.0)
        a = SegAdapter(model, CORR_CFG)
        return a

    def test_correction_sign_and_scale(self):
        a = self._adapter()
        obs = a.observe()          # offset +0.06 → 보정 −1.8° (좌)
        self.assertTrue(obs.valid)
        corr = a.correction_wheel_deg(obs)
        self.assertAlmostEqual(corr, -1.8, places=3)

    def test_correction_clamped(self):
        a = self._adapter()
        from perception.seg_adapter import SegObservation
        big = SegObservation(True, 1.0, 45.0)   # 30 + 13.5 → 클램프 8
        self.assertEqual(a.correction_wheel_deg(big), -8.0)

    def test_invalid_gives_zero_correction(self):
        a = self._adapter()
        from perception.seg_adapter import SegObservation
        self.assertEqual(a.correction_wheel_deg(SegObservation(False, 9, 9)), 0.0)

    def test_unknown_fields_ignored_and_missing_rejected(self):
        a = self._adapter()
        now = time.time()
        ok = a._parse({"valid": True, "lateral_offset_m": 0.1, "heading_error_deg": 5.0,
                       "timestamp": now, "debug_mask": object()})
        self.assertTrue(ok.valid)                       # 모르는 필드 무시 (§5)
        self.assertFalse(a._parse({"valid": True}).valid)          # 필드 누락
        self.assertFalse(a._parse({"valid": True, "lateral_offset_m": 0.1,
                                   "heading_error_deg": 5.0}).valid)  # timestamp 누락 (v0.3 §5)
        self.assertFalse(a._parse({"valid": True, "lateral_offset_m": float("nan"),
                                   "heading_error_deg": 0.0, "timestamp": now}).valid)
        self.assertFalse(a._parse({"valid": True, "lateral_offset_m": 0.0,
                                   "heading_error_deg": 120.0,
                                   "timestamp": now}).valid)  # §5.2 범위

    def test_stale_timestamp_rejected_and_fresh_accepted(self):
        # v0.3 §5: pull 중복 조회 — timestamp가 0.5s보다 오래되면 stale=invalid
        a = self._adapter()
        base = {"valid": True, "lateral_offset_m": 0.1, "heading_error_deg": 5.0}
        self.assertTrue(a._parse({**base, "timestamp": time.time()}).valid)
        self.assertFalse(a._parse({**base, "timestamp": time.time() - 0.6}).valid)

    def test_mock_latest_carries_fresh_timestamp(self):
        m, _ = _model_with_pose(1.2, 2.61, 0.0)
        r = m.latest()
        self.assertLess(abs(time.time() - r["timestamp"]), 1.0)   # v0.3 §5 필수 필드

    def test_broken_model_defended(self):
        a = self._adapter(_BrokenModel())               # §6.1 위반 — 예외 삼킴
        obs = a.observe()
        self.assertFalse(obs.valid)

    def test_fallback_streak_and_recovery(self):
        model, store = _model_with_pose(1.2, 2.61, 0.0)
        a = SegAdapter(model, CORR_CFG)
        model.set_force_invalid(True)
        for _ in range(3):
            self.assertFalse(a.observe().valid)     # 연속 invalid → 폴백 (§6.2)
        model.set_force_invalid(False)
        self.assertTrue(a.observe().valid)          # 복귀 즉시 seg 재개


class TestRealSegModel(unittest.TestCase):
    """V1 클라이언트 — vision 게시 파일 읽기 + 어댑터 사슬 (계약 v0.3 §3)."""

    def _write(self, payload):
        import json, tempfile, os
        fd, p = tempfile.mkstemp(suffix=".json")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(payload, f)
        self.addCleanup(lambda: os.path.exists(p) and os.remove(p))
        return p

    def test_load_rejects_when_vision_absent(self):
        from perception.real_seg_model import RealSegModel
        m = RealSegModel("/nonexistent/vision_latest.json")
        with self.assertRaises(FileNotFoundError):   # §3 기동 거부
            m.load({"load_wait_s": 0.01})

    def test_v1_null_seg_is_invalid_but_carries_timestamp(self):
        from perception.real_seg_model import RealSegModel
        p = self._write({"timestamp": time.time(), "seg": None})
        m = RealSegModel(p)
        m.load({"load_wait_s": 0.01})
        r = m.latest()
        self.assertFalse(r["valid"])                 # V1: seg 미계산 = GPS 폴백
        self.assertIn("timestamp", r)
        a = SegAdapter(m, CORR_CFG)
        self.assertFalse(a.observe().valid)

    def test_seg_payload_flows_through_adapter(self):
        from perception.real_seg_model import RealSegModel
        p = self._write({"timestamp": time.time(),
                         "seg": {"valid": True, "lateral_offset_m": 0.06,
                                 "heading_error_deg": 0.0}})
        m = RealSegModel(p)
        obs = SegAdapter(m, CORR_CFG).observe()
        self.assertTrue(obs.valid)                   # vision이 seg를 채우면 즉시 활성
        self.assertAlmostEqual(obs.lateral_offset_m, 0.06, places=6)

    def test_stale_publish_goes_invalid(self):
        from perception.real_seg_model import RealSegModel
        p = self._write({"timestamp": time.time() - 5.0,
                         "seg": {"valid": True, "lateral_offset_m": 0.06,
                                 "heading_error_deg": 0.0}})
        m = RealSegModel(p)
        obs = SegAdapter(m, CORR_CFG).observe()
        self.assertFalse(obs.valid)                  # 실행체 사망 = stale = GPS 폴백


class TestFusedClosedLoop(unittest.TestCase):
    """mock seg 보정을 켠 폐루프 — 차선 이탈 출발에서도 완주 + 보정 개입 확인."""

    def test_fused_steering_differs_and_mission_completes(self):
        store, supervisor, policy, follower, worker = _make_stack()
        model = MockSegModel(_map(), store)
        model.load({})
        adapter = SegAdapter(model, CORR_CFG)
        worker._seg = adapter                     # 융합 활성 (runtime 배선과 동일 효과)

        dt, t = 0.02, 1000.0
        x, y, heading = 0.90, 2.55, 0.0           # 차선 중심(2.61)에서 6cm 이탈 출발
        _set_pose(store, x, y, heading, t)
        store.set_driving_state(DrivingState.WAITING)
        store.set_mission(1.90, 2.61, ServiceKind.PICKUP)
        store.set_driving_state(DrivingState.PLANNING)

        corrections = 0
        for i in range(int(60 / dt)):
            if i % 5 == 0:                        # 10Hz perception (계약 §3.2)
                adapter.observe()
            before = store.snapshot().control_command.steering_wheel_deg
            worker.tick()
            if policy.notified:
                break
            cmd = store.snapshot().control_command
            if adapter.latest().valid and abs(adapter.correction_wheel_deg(
                    adapter.latest())) > 0.1:
                corrections += 1
            v = cmd.throttle * MAX_SPEED
            heading -= math.degrees(v / WHEELBASE * math.tan(
                math.radians(cmd.steering_wheel_deg)) * dt)
            x += v * math.cos(math.radians(heading)) * dt
            y += v * math.sin(math.radians(heading)) * dt
            t += dt
            _set_pose(store, x, y, heading, t)
            store.set_estimated_speed(v)

        self.assertEqual(policy.notified, 1)
        self.assertLess(math.hypot(x - 1.90, y - 2.61), 0.15)
        self.assertGreater(corrections, 10)       # 보정이 실제로 개입했다


if __name__ == "__main__":
    unittest.main()
