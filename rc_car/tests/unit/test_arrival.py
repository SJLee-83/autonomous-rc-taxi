"""D4 테스트 — ArrivalChecker (명세서 §19).

핵심 검증: §19-5의 OR 조건. 추정 속도가 임계(0.03) 아래로 내려가지 못해도
settle 시간 경과로 도착이 확정된다 — 워크로그 리스크 1(노이즈로 도착 실패)의 해소 근거.
"""
import unittest

from navigation.arrival_checker import ArrivalChecker
from navigation.destination_resolver import StopPose

STOP = StopPose(lane_id="mid_wb1_w_b", x=0.80, y=1.88, heading_deg=180.0,
                progress_s=0.2, snap_distance_m=0.0)

CFG = {"max_estimated_speed_mps": 0.03, "settle_time_sec": 0.30,
       "max_heading_error_deg": 20.0}


def _checker() -> ArrivalChecker:
    c = ArrivalChecker(radius_m=0.10, cfg=CFG)
    c.start_mission(STOP)
    return c


def _tick(c, t, *, x=0.80, y=1.88, heading=180.0, speed=0.0,
          lane="mid_wb1_w_b", throttle=0.0, stale=False):
    # lane 은 편의상 문자열/None 로 쓰고 여기서 집합으로 바꾼다 —
    # 실제 인자는 "차가 있다고 인정되는 차선 집합"이다 (§19-2).
    lanes = frozenset() if lane is None else frozenset(
        {lane} if isinstance(lane, str) else lane)
    return c.update(x, y, heading, speed, lanes, throttle, stale, t)


class TestArrivalChecker(unittest.TestCase):
    def test_no_mission_never_arrives(self):
        c = ArrivalChecker(0.10, CFG)
        self.assertFalse(c.update(0.8, 1.88, 180.0, 0.0,
                                  frozenset({"mid_wb1_w_b"}), 0.0, False, 0.0))

    def test_low_speed_in_radius_arrives_immediately(self):
        c = _checker()
        self.assertTrue(_tick(c, 0.0, speed=0.01))

    def test_radius_boundary_inclusive(self):
        c = _checker()
        self.assertTrue(_tick(c, 0.0, x=0.80 + 0.10, speed=0.0))  # 정확히 반경 위

    def test_outside_radius_not_arrived(self):
        c = _checker()
        self.assertFalse(_tick(c, 0.0, x=0.80 + 0.11, speed=0.0))

    def test_noisy_speed_arrives_after_settle(self):
        # ⭐ 리스크 1 시나리오: 노이즈로 추정 속도가 0.05를 유지해도 settle 로 도착
        c = _checker()
        self.assertFalse(_tick(c, 0.00, speed=0.05))
        self.assertFalse(_tick(c, 0.15, speed=0.06))
        self.assertTrue(_tick(c, 0.31, speed=0.05))  # 0.30s 경과

    def test_radius_exit_resets_settle(self):
        c = _checker()
        _tick(c, 0.00, speed=0.05)
        _tick(c, 0.20, x=1.00, speed=0.05)           # 반경 이탈 → 리셋
        self.assertFalse(_tick(c, 0.35, speed=0.05))  # 재진입 직후 — 아직 0.30 미경과
        self.assertTrue(_tick(c, 0.66, speed=0.05))

    def test_stale_pose_blocks_and_resets(self):
        # §3.4 — 유실 중 마지막 값으로 도착 선언 금지
        c = _checker()
        _tick(c, 0.00, speed=0.05)
        self.assertFalse(_tick(c, 0.29, speed=0.05, stale=True))
        self.assertFalse(_tick(c, 0.35, speed=0.05))  # 복귀 후 유지 시계 처음부터
        self.assertTrue(_tick(c, 0.66, speed=0.05))

    def test_wrong_lane_blocks(self):
        c = _checker()
        self.assertFalse(_tick(c, 0.0, speed=0.0, lane="mid_wb2_w_b"))
        self.assertFalse(_tick(c, 0.0, speed=0.0, lane=None))  # 매칭 실패 깜빡임

    def test_any_candidate_lane_opens_gate(self):
        """이음매 위에서는 후보가 여럿 온다 — 목표가 그중 하나면 열린다 (§19-2)."""
        c = _checker()
        # 진출 차선은 다음 조각이지만 진입 차선이 목표라면 도착이다
        self.assertTrue(_tick(c, 0.0, speed=0.0,
                              lane={"mid_wb1_w_a", "mid_wb1_w_b"}))

    def test_candidate_set_without_target_blocks(self):
        """후보가 여럿이어도 목표가 없으면 닫혀 있어야 한다 — 느슨해진 게 아니다."""
        c = _checker()
        self.assertFalse(_tick(c, 0.0, speed=0.0,
                               lane={"mid_wb2_w_a", "mid_wb2_w_b"}))
        self.assertFalse(_tick(c, 0.0, speed=0.0, lane=frozenset()))

    def test_heading_error_circular(self):
        c = _checker()
        self.assertFalse(_tick(c, 0.0, speed=0.0, heading=210.0))  # 30° 초과
        self.assertTrue(_tick(c, 0.1, speed=0.0, heading=165.0))   # 15° 이내

    def test_throttle_must_be_zero(self):
        c = _checker()
        self.assertFalse(_tick(c, 0.0, speed=0.0, throttle=0.05))
        self.assertTrue(_tick(c, 0.1, speed=0.0, throttle=0.0))

    def test_latched_until_new_mission(self):
        c = _checker()
        self.assertTrue(_tick(c, 0.0, speed=0.0))
        # complete 재전송 중 반경 밖으로 밀려도 판정 번복 없음
        self.assertTrue(_tick(c, 1.0, x=1.5, speed=0.5, stale=True))
        self.assertTrue(c.arrived)
        c.start_mission(STOP)
        self.assertFalse(c.arrived)

    def test_clear_resets(self):
        c = _checker()
        _tick(c, 0.0, speed=0.0)
        c.clear()
        self.assertFalse(c.arrived)
        self.assertFalse(_tick(c, 1.0, speed=0.0))  # 미션 없음


if __name__ == "__main__":
    unittest.main()
