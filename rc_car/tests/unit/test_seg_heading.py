"""seg 방위 대체 말뚝 테스트 (2026-08-07 신설 — control.yaml seg_heading).

왜 말뚝인가: 이 기능은 **꺼져 있어도 주행이 되고, 켜져 있어도 로그 한 줄뿐**이라
재배포·config 되돌림으로 조용히 사라져도 아무도 모른다. 8/6 의 인도 침범(11회 중 6회)이
정확히 이 기능이 막는 사고라, 배포 상태를 테스트가 붙들어야 한다.

덮는 범위
    ① 기본 꺼짐 — 설정 없으면 기존 동작(GPS 방위) 그대로
    ② 차선 화이트리스트 — 허용 차선에서만 대체, 밖에서는 GPS
    ③ seg invalid → GPS 무중단 폴백
    ④ 회전 중에는 대체 없음 (표의 고정각을 건드리지 않는다)
    ⑤ 🔴 0806 1차 주행 실측 재현 — 조향 부호가 뒤집히는가
    ⑥ 레이트 리밋 — 출처 전환 시 계단 지령 차단
    ⑦ 배포 config 가 [4, 14] 인가

부호 규약: 바퀴각 +(우) → heading 감소. 계약 §5 seg heading_error +(좌측 보정 필요).
"""
import math
import unittest
from pathlib import Path

from control.lane_follower import TURNING, LaneFollower
from mapping.lane_map import load_lane_map
from navigation import lane_route
from navigation.turn_table import TurnTable
from perception.seg_adapter import SegObservation

MAP_PATH = Path(__file__).resolve().parents[3] / "map" / "main_track_map.yaml"
TABLE_PATH = Path(__file__).resolve().parents[2] / "config" / "turn_table.yaml"
CONTROL_PATH = Path(__file__).resolve().parents[2] / "config" / "control.yaml"

BASE_CFG = {
    "loop_hz": 50,
    "lookahead_min_m": 0.25,
    "destination_slowdown_distance_m": 0.40,
    "stop_trigger_radius_m": 0.05,
    "steer_full_lock_error_deg": 30.0,
    "align_heading_error_deg": 10.0,
    "align_throttle_ratio": 1.0,
    "cruise_throttle": 0.4,
    "turn_throttle": 0.5,
    "near_target_throttle_ratio": 1.0,
    "departure_ramp_distance_m": 0.30,
    "corner_exit_hold_distance_m": 0.20,
}
MAX_WHEEL, MAX_SPEED, WHEELBASE = 30.0, 0.22, 0.14


def _map():
    if not hasattr(_map, "cached"):
        _map.cached = load_lane_map(MAP_PATH)
    return _map.cached


def _table():
    return TurnTable.load(WHEELBASE, MAX_WHEEL, path=TABLE_PATH)


def _design_table():
    """설계 기하 표 (하드웨어 보정값 없음) — 이상 차량 모델 폐루프/거동 검증용.

    배포 turn_table.yaml 은 시연 하드웨어(실효 조향 ≈ 지령의 46%)에 캘리브레이션된
    값(개루프 호장 0.856, lead 등)이라 명령을 100% 순종하는 시뮬과 양립하지 않는다
    (§0-57 규명: 시뮬에선 190° 과회전). 배포값 자체는 TestDeployedConfig 말뚝이 지킨다.
    """
    return TurnTable({"turn_table": {"defaults": {}, "turns": {}}}, WHEELBASE, MAX_WHEEL)


class _SegStub:
    """SegAdapter.latest() 대역 — 이 테스트가 쓰는 것은 그 하나뿐이다."""

    def __init__(self, obs=None):
        self.obs = obs or SegObservation(False, 0.0, 0.0)

    def latest(self):
        return self.obs


def _cfg(**over):
    cfg = dict(BASE_CFG)
    cfg["seg_heading"] = {"lane_nums": [4, 14], "disagree_warn_deg": 15.0,
                          "steer_rate_deg_per_s": 0.0}
    cfg["seg_heading"].update(over)
    return cfg


def _follower(cfg, seg):
    return LaneFollower(cfg, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE, seg=seg)


class TestSegHeadingGate(unittest.TestCase):
    """① 기본 꺼짐 / ② 차선 화이트리스트 / ③ invalid 폴백."""

    def _drive_on(self, numbers, cfg, seg, along=0.30, heading=None):
        route = lane_route.from_lane_numbers(_map(), numbers)
        f = _follower(cfg, seg)
        f.set_route(route)
        leg = route.legs[0]
        (x0, y0), (x1, y1) = leg.points[0], leg.points[-1]
        n = math.hypot(x1 - x0, y1 - y0)
        x, y = x0 + (x1 - x0) / n * along, y0 + (y1 - y0) / n * along
        h = leg.heading_deg() if heading is None else heading
        cmd, _ = f.compute(x, y, h)
        return f, cmd

    def test_off_by_default(self):
        """설정이 없으면 seg 어댑터가 붙어 있어도 대체하지 않는다."""
        seg = _SegStub(SegObservation(True, 0.0, 20.0))
        f, cmd = self._drive_on([4, 29], dict(BASE_CFG), seg, heading=20.0)
        self.assertFalse(f.seg_heading_active)
        # GPS 방위 20°(차선보다 좌향) 를 그대로 쓰면 복귀하려고 **우** 조향이 나온다.
        # seg(=+20 오차 → 방위 −20°)를 썼다면 부호가 반대였을 것이다.
        self.assertGreater(cmd.steering_wheel_deg, 5.0)

    def test_empty_lane_list_is_off(self):
        seg = _SegStub(SegObservation(True, 0.0, 20.0))
        f, _ = self._drive_on([4, 29], _cfg(lane_nums=[]), seg)
        self.assertFalse(f.seg_heading_active)

    def test_allowed_lane_uses_seg(self):
        seg = _SegStub(SegObservation(True, 0.0, 8.0))
        f, _ = self._drive_on([4, 29], _cfg(), seg, heading=30.0)
        self.assertTrue(f.seg_heading_active, "4번은 허용 차선 — seg 방위를 써야 한다")

    def test_lane_outside_whitelist_keeps_gps(self):
        """7번은 목록 밖 — 북쪽 경계가 흰색이라 seg 신뢰도가 낮다 (0806 재처리 56%)."""
        seg = _SegStub(SegObservation(True, 0.0, 8.0))
        f, _ = self._drive_on([7, 22], _cfg(), seg, heading=30.0)
        self.assertFalse(f.seg_heading_active)

    def test_invalid_seg_falls_back_to_gps(self):
        seg = _SegStub(SegObservation(False, 0.0, 0.0))
        f, _ = self._drive_on([4, 29], _cfg(), seg)
        self.assertFalse(f.seg_heading_active)

    def test_no_adapter_is_safe(self):
        """seg_mode=off 로 어댑터가 None 이어도 기동·주행이 되어야 한다."""
        f, cmd = self._drive_on([4, 29], _cfg(), None)
        self.assertFalse(f.seg_heading_active)
        self.assertGreater(cmd.throttle, 0.0)


class TestSegHeadingSubstitution(unittest.TestCase):
    """⑤ 0806 1차 주행 실측 재현 — 이 테스트가 이 기능의 존재 이유다."""

    # 0806 1차, 차선 4 위 (14:53:45, 프레임 4248) 실측:
    #   GPS pose (3.608, 2.629) heading 37.3°  /  seg heading_error +6.8° (valid, rows 7)
    #   당시 명령: 목표방위 −1.5°, 오차 −38.8° → 조향 **+30.0° 우 풀락** → 인도 침범
    GPS_HEADING = 37.3
    SEG_ERR = 6.8
    POSE = (3.608, 2.629)

    def _cmd(self, cfg, seg):
        route = lane_route.from_lane_numbers(_map(), [4, 29])
        f = _follower(cfg, seg)
        f.set_route(route)
        cmd, _ = f.compute(self.POSE[0], self.POSE[1], self.GPS_HEADING)
        return f, cmd

    def test_gps_alone_commands_right_full_lock(self):
        """재현 확인 — 기능이 꺼져 있으면 8/6 과 같은 우 풀락이 나온다."""
        _, cmd = self._cmd(dict(BASE_CFG), _SegStub())
        self.assertGreaterEqual(cmd.steering_wheel_deg, MAX_WHEEL - 0.01,
                                "GPS 방위 37.3° 면 우 풀락이 나와야 재현이다")

    def test_seg_heading_flips_sign(self):
        """대체가 걸리면 같은 tick 에 **부호가 뒤집힌다** (우 풀락 → 완만한 좌)."""
        seg = _SegStub(SegObservation(True, 0.0, self.SEG_ERR))
        f, cmd = self._cmd(_cfg(), seg)
        self.assertTrue(f.seg_heading_active)
        self.assertLess(cmd.steering_wheel_deg, 0.0,
                        "seg 방위(−6.8°)면 목표보다 우향이므로 좌 조향이어야 한다")
        self.assertGreater(cmd.steering_wheel_deg, -15.0, "완만해야 한다 — 풀락이면 과보정")

    def test_substituted_heading_formula(self):
        """방위 = 차선 공칭 − seg 오차. 차선 4 공칭은 0°(동향)."""
        route = lane_route.from_lane_numbers(_map(), [4, 29])
        leg = route.legs[0]
        self.assertAlmostEqual(leg.heading_deg(), 0.0, delta=0.5)
        seg = _SegStub(SegObservation(True, 0.0, self.SEG_ERR))
        f = _follower(_cfg(), seg)
        f.set_route(route)
        self.assertAlmostEqual(f._pursue_heading(leg, self.GPS_HEADING),
                               leg.heading_deg() - self.SEG_ERR, places=6)

    def test_lateral_offset_is_not_used(self):
        """🔴 offset 부호 미검증 — heading 만 쓴다. offset 을 바꿔도 결과가 같아야 한다."""
        a = _SegStub(SegObservation(True, +0.15, self.SEG_ERR))
        b = _SegStub(SegObservation(True, -0.15, self.SEG_ERR))
        _, ca = self._cmd(_cfg(), a)
        _, cb = self._cmd(_cfg(), b)
        self.assertAlmostEqual(ca.steering_wheel_deg, cb.steering_wheel_deg, places=6)


class TestSegHeadingDuringTurn(unittest.TestCase):
    """④ 회전 중에는 표의 고정 조향각이 그대로 나가야 한다."""

    def test_turning_ignores_seg(self):
        route = lane_route.from_lane_numbers(_map(), [4, 29, 7])
        seg = _SegStub(SegObservation(True, 0.0, 25.0))
        f = _follower(_cfg(), seg)
        f.set_route(route)
        leg = route.legs[0]
        h = leg.heading_deg()
        # 차선 끝까지 밀어 회전을 연다
        for along in (0.60, 1.00, leg.length_m + 0.02):
            (x0, y0), (x1, y1) = leg.points[0], leg.points[-1]
            n = math.hypot(x1 - x0, y1 - y0)
            cmd, _ = f.compute(x0 + (x1 - x0) / n * along, y0 + (y1 - y0) / n * along, h)
        self.assertEqual(f.state, TURNING)
        self.assertFalse(f.seg_heading_active, "회전 중에는 대체가 걸리지 않는다")
        self.assertAlmostEqual(cmd.steering_wheel_deg,
                               math.degrees(math.atan(WHEELBASE / 0.26)), delta=0.2)


class TestSteerRateLimit(unittest.TestCase):
    """⑥ 출처 전환 계단 지령 차단."""

    def _run(self, rate):
        route = lane_route.from_lane_numbers(_map(), [4, 29])
        seg = _SegStub(SegObservation(True, 0.0, 0.0))
        f = _follower(_cfg(steer_rate_deg_per_s=rate), seg)
        f.set_route(route)
        leg = route.legs[0]
        (x0, y0), (x1, y1) = leg.points[0], leg.points[-1]
        n = math.hypot(x1 - x0, y1 - y0)
        x, y = x0 + (x1 - x0) / n * 0.30, y0 + (y1 - y0) / n * 0.30
        f.compute(x, y, leg.heading_deg())            # 정렬 상태 — 조향 ≈ 0
        seg.obs = SegObservation(False, 0.0, 0.0)     # seg 탈락 → GPS(크게 틀어진 값)
        cmd, _ = f.compute(x, y, leg.heading_deg() + 40.0)
        return cmd.steering_wheel_deg

    def test_rate_limit_caps_step(self):
        limited = self._run(120.0)      # 50Hz → 2.4°/tick
        self.assertLessEqual(abs(limited), 2.4 + 1e-6)

    def test_zero_rate_means_unlimited(self):
        free = self._run(0.0)
        self.assertGreater(abs(free), 20.0, "제한 0 이면 계단 지령이 그대로 나간다")


class TestHeadingGuard(unittest.TestCase):
    """🔴 heading 튐 방어 — GPS 방위가 실제 진행 방향과 어긋나면 후자를 쓴다.

    재현 대상 (2026-08-07 18:04 주행, 4번 차선):
      차는 동쪽(코스 ≈350°)으로 가는데 GPS 는 "10~45°(북동)를 향한다"고 보고했다.
      _pursue 가 그 값을 믿고 우측 풀락을 내 40cm 를 남쪽으로 끌고 갔고, 그 자세로
      회전에 들어가 인도를 밟았다.
    """

    def _guard_cfg(self, deg=25.0):
        cfg = dict(BASE_CFG)
        cfg["heading_guard"] = {"disagree_deg": deg, "window_m": 0.15}
        return cfg

    def _drive_east(self, cfg, reported_heading, steps=14, dx=0.03):
        """4번 차선(동향)을 실제로 동쪽으로 굴리면서 GPS 방위만 거짓으로 준다.

        위치를 직접 먹이므로 '실제 진행 방향'은 정확히 0°(동) 다 —
        방위만 틀린 18:04 상황과 같은 구조다.
        """
        route = lane_route.from_lane_numbers(_map(), [4, 29])
        f = LaneFollower(cfg, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        f.set_route(route)
        leg = route.legs[0]
        (x0, y0) = leg.points[0]
        cmd = None
        for i in range(steps):
            cmd, _ = f.compute(x0 + 0.10 + dx * i, y0, reported_heading)
        return f, cmd

    def test_spike_is_replaced_by_course(self):
        """GPS 가 +40° 틀어져도 조향이 풀락으로 가지 않는다."""
        _, cmd = self._drive_east(self._guard_cfg(), 40.0)
        self.assertLess(abs(cmd.steering_wheel_deg), 10.0,
                        "튐을 못 걸러 풀락이 나갔다 — 18:04 인도 침범이 재현된다")

    def test_without_guard_the_spike_drives_full_lock(self):
        """대조군: 방어가 꺼져 있으면 같은 입력이 풀락을 만든다 (사고 재현)."""
        cfg = dict(BASE_CFG)
        cfg["heading_guard"] = {"disagree_deg": 0.0}
        _, cmd = self._drive_east(cfg, 40.0)
        self.assertGreater(cmd.steering_wheel_deg, 20.0,
                           "이 대조군이 깨지면 테스트가 사고를 재현하지 못하는 것이다")

    def test_small_error_is_left_alone(self):
        """임계 미만(10°)은 그대로 둔다 — 정상 차선 복귀 조향을 죽이면 안 된다."""
        f, cmd = self._drive_east(self._guard_cfg(), 10.0)
        self.assertFalse(f.heading_guard_active)
        self.assertGreater(cmd.steering_wheel_deg, 1.0, "정상 복귀 조향까지 막으면 안 된다")

    def test_stopped_vehicle_keeps_gps(self):
        """정지 상태는 베이스라인을 못 채운다 — 판단하지 않고 GPS 원값을 쓴다."""
        route = lane_route.from_lane_numbers(_map(), [4, 29])
        f = LaneFollower(self._guard_cfg(), _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        f.set_route(route)
        (x0, y0) = route.legs[0].points[0]
        for _ in range(10):
            cmd, _ = f.compute(x0 + 0.10, y0, 40.0)       # 같은 자리 = 이동 0
        self.assertFalse(f.heading_guard_active)
        self.assertGreater(cmd.steering_wheel_deg, 20.0, "정지 시엔 기존 동작 그대로")

    def test_turn_does_not_feed_the_guard(self):
        """회전 원호는 자취에 쌓지 않는다 — 코스각이 구조적으로 뒤처져 오탐이 된다."""
        route = lane_route.from_lane_numbers(_map(), [4, 29])
        f = LaneFollower(self._guard_cfg(), _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        f.set_route(route)
        leg = route.legs[0]
        (x1, y1) = leg.points[-1]
        f.compute(x1, y1, leg.heading_deg())              # 차선 끝 → 회전 개시
        self.assertEqual(f.state, TURNING)
        self.assertEqual(f._follow_path, [], "회전 진입 시 직선 자취를 비워야 한다")
        self.assertFalse(f.heading_guard_active)

    def test_control_yaml_enables_the_guard(self):
        """🔴 배포 config 말뚝 — 이 방어가 조용히 꺼지면 인도 침범이 재발한다."""
        import yaml
        cfg = yaml.safe_load(CONTROL_PATH.read_text(encoding="utf-8"))
        hg = cfg["control"]["heading_guard"]
        self.assertGreater(float(hg["disagree_deg"]), 0.0,
                           "heading_guard 가 꺼졌다 — 18:04 과 같은 +80° 튐을 못 막는다")
        self.assertLessEqual(float(hg["disagree_deg"]), 30.0,
                             "임계가 너무 크면 실측 +30~80° 중 작은 쪽을 놓친다")


class TestStraightConnectorGate(unittest.TestCase):
    """🔴 교차로 직진 게이트 — 19->20 (2026-08-07 저녁 사용자 지시).

    사용자 실차 관찰: "삼거리에서 계속 사거리 방향으로 진입하려고 한다."
    직진 커넥터 1.28m 가 어느 다리에도 속하지 않아, 19번 끝(0.39,0.86)을 지나는
    순간부터 20번 **끝점**(0.39,2.35)을 겨눴다. 차가 차선에서 벗어나 있으면 그 먼
    목표가 대각선이 되고, 그 방향이 중앙 도로로 파고드는 방향과 같다.
    """

    LANE_X = 0.39            # 19·20번 중심선
    GATE = (0.39, 2.14)      # 커넥터 출구 = 20번 시작점
    LANE20_END = (0.39, 2.35)

    def _follower(self, gate=True):
        cfg = dict(BASE_CFG)
        cfg["straight_connector_gate"] = gate
        cfg["lane_line_following"] = gate    # 구 동작 대조군은 끝점 겨냥까지 함께 끈다
        f = LaneFollower(cfg, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        f.set_route(lane_route.from_lane_numbers(_map(), [14, 19, 20, 3]))
        return f

    def _cross_the_tee(self, f, x_off):
        """19번을 지나 커넥터로 들어간 상태를 만든다 — 차선에서 x_off 만큼 벗어난 채."""
        f._leg, f._pt, f._state = 1, 1, "FOLLOW"      # 19번 다리
        for y in (0.70, 0.78, 0.86, 0.92):
            cmd, _ = f.compute(self.LANE_X + x_off, y, 90.0)
        return cmd

    def test_gate_aims_at_lane20_entry_not_its_far_end(self):
        f = self._follower(gate=True)
        self._cross_the_tee(f, -0.20)
        self.assertEqual(f.leg_index, 2, "직진 커넥터를 지나 20번 다리로 넘어가야 한다")
        self.assertEqual(f._straight_gate, self.GATE,
                         "커넥터 출구(20번 시작점)를 겨눠야 한다 — 20번 끝점이 아니다")

    def test_without_gate_the_target_is_the_far_end(self):
        """대조군 — 게이트를 끄면 예전처럼 1.49m 앞 먼 점을 겨눈다 (사고 재현)."""
        f = self._follower(gate=False)
        self._cross_the_tee(f, -0.20)
        self.assertEqual(f.leg_index, 2)
        self.assertIsNone(f._straight_gate, "이 대조군이 깨지면 사고를 재현하지 못한다")

    def test_gate_steers_back_onto_the_line_harder(self):
        """같은 횡오차에서 새 방식이 더 강하게 선 위로 복귀시킨다.

        목표가 가까울수록 같은 횡오차가 만드는 방위 오차가 커지기 때문이다 —
        18:38 실측에서 x 오차 −0.20m 를 10초간 끌고 간 것이 구 동작(끝점 겨냥)이다.
        """
        on = self._cross_the_tee(self._follower(True), -0.20).steering_wheel_deg
        off = self._cross_the_tee(self._follower(False), -0.20).steering_wheel_deg
        self.assertGreater(on, 0.0, "차선 서쪽에 있으면 동쪽(우)으로 복귀해야 한다")
        self.assertGreaterEqual(on, off, "새 방식이 더 빠르게 선 위로 되돌려야 한다")

    def test_line_following_settles_without_overshoot(self):
        """🔴 차선 위 추종은 **점근 수렴**한다 — 중심선을 넘어가지 않는다.

        20:35 실측 재현: 19/20 을 x 0.00(서쪽 0.39m)에서 달리다 마지막 3초에 0.489 로
        31cm 동진해 중심선을 넘고 인도(block_nw x>=0.52)까지 갔다.
        끝점 겨냥이면 먼 목표가 대각선이 되어 사선 돌진 후 넘어간다.
        """
        cfg = dict(BASE_CFG)
        cfg["lane_line_following"] = True
        # 설계 기하 표 — 배포 표의 20->3 lead 0.285 는 차선 끝 0.285m 전 개시라
        # 이 직선 추종 검증 구간과 겹친다 (하드웨어 보정값은 TestDeployedConfig 소관)
        f = LaneFollower(cfg, _design_table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [19, 20, 3])
        f.set_route(route)
        leg = route.legs[1]                       # 20번 다리
        f._leg, f._pt = 1, 1
        seen_east = False
        for i in range(26):                       # 서쪽 0.39m 에서 출발해 북상
            x = 0.00 + 0.020 * i                  # 실측처럼 동쪽으로 수렴시킨다
            cmd, _ = f.compute(x, 2.14 + 0.008 * i, 90.0)
            if x > 0.39:
                seen_east = True
                self.assertLessEqual(cmd.steering_wheel_deg, 0.5,
                                     "중심선을 넘었으면 서쪽(좌)으로 되돌려야 한다")
        self.assertTrue(seen_east, "테스트가 중심선을 넘는 구간을 못 만들었다")

    def test_gate_blocks_arming_the_next_turn(self):
        """게이트가 살아 있는 동안 20->3 을 준비하지 않는다.

        차선에 들어오기도 전에 ARMED 가 되면 그때부터 3번을 향해 비스듬히 겨눈다 —
        사용자가 지적한 "사거리 쪽으로 파고듦" 과 같은 대각선이다.
        """
        f = self._follower(True)
        self._cross_the_tee(f, -0.20)
        self.assertEqual(f.state, "FOLLOW", "커넥터 위에서 ARMED 가 되면 안 된다")

    def test_gate_releases_at_lane20_entry(self):
        """출구에 닿으면 게이트를 풀고 평소 추종으로 돌아간다."""
        f = self._follower(True)
        self._cross_the_tee(f, 0.0)
        for y in (1.4, 1.8, 2.05, 2.14):
            f.compute(self.LANE_X, y, 90.0)
        self.assertIsNone(f._straight_gate, "20번 진입 후에는 게이트가 풀려야 한다")

    def test_pickup_leg_has_no_straight_connector(self):
        """픽업 노선에는 직진 커넥터가 없다 — 이 변경의 영향 지점은 19->20 하나뿐이다."""
        r = lane_route.from_lane_numbers(_map(), [4, 29, 7, 22, 14])
        self.assertEqual([t.maneuver for t in r.turns].count("straight"), 0)
        r2 = lane_route.from_lane_numbers(_map(), [14, 19, 20, 3])
        self.assertEqual([t.maneuver for t in r2.turns].count("straight"), 1)


class TestFusedTurn(unittest.TestCase):
    """🔴 4->29->7 을 한 번에 도는 180° 회전 (2026-08-07 저녁, 사용자 제안).

    90° 코너 두 개는 각 설계 반경이 0.26m 라 이 차(실측 0.47m)가 못 낸다.
    180° 한 번으로 보면 필요 반경이 0.49m 라 실측과 2cm 차이로 맞는다.
    """

    R = 0.47                      # 실측 실효 반경

    def _follower(self):
        f = LaneFollower(dict(BASE_CFG), _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        f.set_route(lane_route.from_lane_numbers(_map(), [4, 29, 7, 22, 14]))
        return f

    def _open_the_turn(self, f):
        x, y = 3.60, 2.61                      # 4번 위, 동향
        for _ in range(60):
            f.compute(x, y, 0.0)
            if f.state == TURNING:
                return x, y
            x += 0.02
        self.fail("회전이 열리지 않았다")

    def test_span_is_two_legs(self):
        f = self._follower()
        self._open_the_turn(f)
        self.assertEqual(f._fuse_span, 2, "29번을 건너뛰고 7번까지 묶어야 한다")

    def test_lands_on_lane7_after_180(self):
        """반경 0.47m 원호를 먹이면 7번(다리 index 2)에서 종료해야 한다."""
        f = self._follower()
        x, y = self._open_the_turn(f)
        cx, cy = x, y - self.R
        for i in range(1, 200):
            th = 90.0 - 1.5 * i
            px = cx + self.R * math.cos(math.radians(th))
            py = cy + self.R * math.sin(math.radians(th))
            f.compute(px, py, (th - 90.0) % 360)
            if f.state != TURNING:
                self.assertEqual(f.leg_index, 2, "7번 다리에서 끝나야 한다 (29번은 건너뜀)")
                self.assertLess(abs(py - 1.63), 0.15, "7번 중심선 근처여야 한다")
                return
        self.fail("180° 를 다 돌고도 종료하지 않았다")

    def test_does_not_exit_at_the_90_degree_point(self):
        """중간(남향) 지점에서 끊기면 예전과 같은 문제가 된다 — min_turn_m 이 막는다."""
        f = self._follower()
        x, y = self._open_the_turn(f)
        cx, cy = x, y - self.R
        for i in range(1, 61):                 # 90° 까지만 (1.5° x 60)
            th = 90.0 - 1.5 * i
            f.compute(cx + self.R * math.cos(math.radians(th)),
                      cy + self.R * math.sin(math.radians(th)), (th - 90.0) % 360)
        self.assertEqual(f.state, TURNING, "90° 지점에서 아직 돌고 있어야 한다")

    def test_arc_reach_is_known_and_pinned(self):
        """🔴 호 최동단(개시 x + R)이 어디까지 가는지 **알고 쓴다**.

        사용자가 개시를 끝점 5cm 앞으로 지시했다(lead 0.05). 그 결과 최동단이
        바깥 북행 차선(4.735~4.985) 안으로 들어간다 — 대향 차선을 문다는 뜻이다.
        실주행에서 물면 lead 를 0.11 로 되돌려야 한다. 이 테스트는 그 사실이
        조용히 잊히지 않게 박아 두는 것이지, 안전하다고 주장하는 게 아니다.
        """
        f = self._follower()
        x, _ = self._open_the_turn(f)
        reach = x + self.R
        self.assertGreater(reach, 4.61, "코너 한계는 넘는다 — 반경이 설계의 1.8배라 불가피")
        self.assertLess(reach, 4.985, "바깥 차선 **바깥**까지 나가면 맵 이탈이다")

    def test_table_carries_the_fuse_flag(self):
        s = _table().resolve(lane_route.from_lane_numbers(_map(), [4, 29, 7]).turns[0])
        self.assertTrue(s.fuse_next, "4->29 의 fuse_next 가 꺼졌다")
        self.assertGreaterEqual(s.min_turn_m, 0.80, "90° 지점(호 0.77m) 전 종료를 막아야 한다")
        self.assertGreaterEqual(s.max_turn_m, 2.13, "실측 반경 0.678 의 180° 호장 2.13m")

    def test_other_turns_are_not_fused(self):
        tt = _table()
        for pair in ([29, 7, 22], [7, 22, 14], [22, 14, 19], [14, 19, 20], [20, 3, 4]):
            s = tt.resolve(lane_route.from_lane_numbers(_map(), pair).turns[0])
            self.assertFalse(s.fuse_next, f"{s.key} 는 묶으면 안 된다")


class TestLaneDeadband(unittest.TestCase):
    """🔴 횡오차 불감대 — "정렬되면 그냥 가라" (2026-08-07 밤 사용자 지시).

    14번은 북쪽 가장자리(0.515) 바로 위가 인도(block_sw 0.52)라 중심선 복귀가
    곧 인도 방향이다. 20:35 실측에서 y −0.083 → +0.107 로 넘어가 인도까지 2.3cm.
    """

    def _steer(self, y, deadband):
        cfg = dict(BASE_CFG)
        cfg["lane_line_following"] = True
        cfg["lane_deadband_m"] = deadband
        f = LaneFollower(cfg, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [22, 14, 19])
        f.set_route(route)
        f._leg, f._pt = 1, 1
        cmd, _ = f.compute(1.60, y, 180.0)          # 14번 위, 차선 방향 정렬
        return cmd.steering_wheel_deg

    def test_inside_deadband_goes_straight(self):
        """차선 안(중심선 −8.3cm)이고 정렬돼 있으면 조향 0 — 인도 쪽으로 안 당긴다."""
        self.assertAlmostEqual(self._steer(0.307, 0.10), 0.0, delta=0.5)

    def test_outside_deadband_still_recovers(self):
        """불감대 밖(−15cm)이면 복귀 조향이 살아 있어야 한다 — 차선 이탈은 방치 안 한다."""
        self.assertGreater(self._steer(0.24, 0.10), 5.0)

    def test_deadband_off_pulls_to_centerline(self):
        """대조군 — 불감대 0 이면 같은 자리에서 중심선(=인도 방향)으로 당긴다."""
        self.assertGreater(self._steer(0.307, 0.0), 10.0)

    def test_deadband_smaller_than_half_lane_width(self):
        """불감대는 차선 반폭(0.125)보다 작아야 한다 — 크면 차선 밖을 허용한다."""
        import yaml
        cfg = yaml.safe_load(CONTROL_PATH.read_text(encoding="utf-8"))["control"]
        self.assertLess(float(cfg["lane_deadband_m"]), 0.125)
        self.assertGreater(float(cfg["lane_deadband_m"]), 0.0)


class TestCourseHeadingLanes(unittest.TestCase):
    """🔴 4번은 GPS 방위를 **아예 안 쓴다** (2026-08-07 밤 사용자 지시).

    "4번 도로에서는 좌표만 받고 heading 값 무시하고 가라."
    21:00 실측: 실제 진행 341~351°(남동) vs GPS 0~11°(동·약간 북).
    제어기가 GPS 를 믿고 우측 조향을 계속 내 y 가 0.54m 동안 12cm 떨어졌다.
    차이 19~21° 가 튐 방어 임계 25° 바로 아래라 안 걸렸다.
    """

    def _drive(self, lanes, gps_heading, along_dx=0.03, steps=14):
        cfg = dict(BASE_CFG)
        cfg["course_heading_lanes"] = lanes
        cfg["heading_guard"] = {"disagree_deg": 25.0, "window_m": 0.15}
        f = LaneFollower(cfg, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [4, 29, 7])
        f.set_route(route)
        (x0, y0) = route.legs[0].points[0]
        cmd = None
        for i in range(steps):                      # 정확히 동쪽으로 굴린다 (실제 진행 = 0°)
            cmd, _ = f.compute(x0 + 0.10 + along_dx * i, y0, gps_heading)
        return f, cmd

    def test_lane4_ignores_a_sub_threshold_gps_bias(self):
        """GPS 가 +11° 로 치우쳐도(임계 25° 미만) 4번에서는 우측 조향이 안 나간다."""
        f, cmd = self._drive([4], 11.0)
        self.assertTrue(f.heading_guard_active, "4번은 항상 진행 방향을 쓴다")
        self.assertLess(abs(cmd.steering_wheel_deg), 3.0,
                        "실제로 똑바로 가는 중이면 조향이 0 이어야 한다")

    def test_without_the_list_the_bias_leaks_through(self):
        """대조군 — 목록에서 빼면 같은 +11° 가 우측 조향으로 새어 나간다 (사고 재현)."""
        f, cmd = self._drive([], 11.0)
        self.assertFalse(f.heading_guard_active)
        self.assertGreater(cmd.steering_wheel_deg, 3.0,
                           "이 대조군이 깨지면 테스트가 사고를 재현하지 못한다")

    def test_stopped_uses_lane_nominal_not_gps(self):
        """🔴 정지 중에는 GPS 가 아니라 **차선 공칭 방위**를 쓴다.

        여기서 GPS 로 떨어지면 "출발하자마자 방위 보정한다고 우회전"이 재발한다 —
        21:09 실측: 정차 중 GPS 9~17°(북동)를 믿고 우측 조향이 나갔다.
        """
        cfg = dict(BASE_CFG)
        cfg["course_heading_lanes"] = [4]
        f = LaneFollower(cfg, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [4, 29, 7])
        f.set_route(route)
        (x0, y0) = route.legs[0].points[0]
        for _ in range(10):
            cmd, _ = f.compute(x0 + 0.10, y0, 11.0)      # 제자리, GPS 는 +11° 로 거짓
        self.assertTrue(f.heading_guard_active, "정지 중에도 GPS 를 쓰면 안 된다")
        self.assertAlmostEqual(cmd.steering_wheel_deg, 0.0, delta=1.0,
                               msg="차선 위·정렬 상태라면 정지 중 조향은 0 이어야 한다")

    def test_control_yaml_lists_lane4(self):
        import yaml
        cfg = yaml.safe_load(CONTROL_PATH.read_text(encoding="utf-8"))["control"]
        self.assertIn(4, list(cfg["course_heading_lanes"]),
                      "4번이 빠지면 GPS 방위 편향이 다시 우측 조향으로 샌다")


class TestStopSteerHoldAndMapGuard(unittest.TestCase):
    """🔴 정지 중 조향 금지 + 회전 중 맵 이탈 방지 (2026-08-07 밤 사용자 지시)."""

    def _cfg(self, **over):
        cfg = dict(BASE_CFG)
        cfg["course_heading_lanes"] = [4]
        cfg["hold_steer_lanes"] = [4]
        cfg["lane_deadband_m"] = 0.10
        cfg["map_bounds"] = {"x_max": 5.0, "y_max": 3.0, "turn_margin_m": 0.01}
        cfg.update(over)
        return cfg

    def test_no_steer_while_stopped(self):
        """대기 중 y 오차가 불감대 밖(+0.105)이어도 바퀴를 틀지 않는다.

        21:22 실측: pose 붙박이인데 남쪽 조향이 계속 나갔다.
        출발 순간 그 각도로 튀어나가는 것이 사용자가 지적한 문제다.
        """
        f = LaneFollower(self._cfg(), _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [4, 29, 7])
        f.set_route(route)
        (x0, y0) = route.legs[0].points[0]
        for _ in range(12):
            cmd, _ = f.compute(x0 + 0.10, y0 + 0.105, 358.0)     # 제자리, 불감대 밖
        self.assertAlmostEqual(cmd.steering_wheel_deg, 0.0, places=6)

    def test_steering_returns_once_moving(self):
        """움직이기 시작하면 조향이 되살아난다 — 정지 중에만 막는 것이다."""
        f = LaneFollower(self._cfg(), _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [4, 29, 7])
        f.set_route(route)
        (x0, y0) = route.legs[0].points[0]
        cmd = None
        for i in range(14):
            cmd, _ = f.compute(x0 + 0.10 + 0.03 * i, y0 + 0.20, 0.0)   # 크게 벗어난 채 주행
        self.assertGreater(abs(cmd.steering_wheel_deg), 3.0,
                           "주행 중에는 복귀 조향이 나가야 한다")

    def test_hold_can_be_disabled(self):
        """대조군 — 끄면 정지 중에도 조향이 나간다 (지금까지의 동작)."""
        f = LaneFollower(self._cfg(hold_steer_lanes=[]),
                         _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [4, 29, 7])
        f.set_route(route)
        (x0, y0) = route.legs[0].points[0]
        for _ in range(12):
            cmd, _ = f.compute(x0 + 0.10, y0 + 0.105, 358.0)
        self.assertGreater(abs(cmd.steering_wheel_deg), 1.0)

    def test_turn_aborts_at_map_edge(self):
        """🔴 회전 중 맵 경계 8cm 안에 닿으면 정렬과 무관하게 끊는다.

        21:24 실측 최서단 x = −0.034 (맵 밖). 회전은 개루프라 스스로 멈추지 않는다.
        """
        f = LaneFollower(self._cfg(), _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [14, 19, 20])
        f.set_route(route)
        leg = route.legs[0]
        (x1, y1) = leg.points[-1]
        f.compute(x1 - 0.16, y1, 180.0)                 # 끝점 지나 회전 개시 (lead −0.14)
        self.assertEqual(f.state, TURNING)
        f.compute(-0.03, 0.60, 140.0)                   # 맵 밖(x<0) — 21:24 실측과 같은 값
        self.assertNotEqual(f.state, TURNING, "맵을 벗어나면 회전을 끊어야 한다")

    def test_map_guard_does_not_kill_the_fused_turn(self):
        """🔴 융합 180° 는 x 4.84~4.98 까지 나가야 한다 — 여기서 끊기면 안 된다.

        22:12 실측: 여유 0.08 일 때 (4.920, 2.313) 에서 끊겨 정렬 68.3° 로 실패했다.
        그 구간은 맵 밖이 아니라 바깥 북행 차선(4.735~4.985) = 실제 도로다.
        """
        f = LaneFollower(self._cfg(), _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [4, 29, 7])
        f.set_route(route)
        (x1, y1) = route.legs[0].points[-1]
        f.compute(x1 - 0.04, y1, 0.0)                    # 4번 끝 부근 — 회전 개시
        self.assertEqual(f.state, TURNING)
        f.compute(4.92, 2.31, 300.0)                     # 22:12 에 끊겼던 그 지점
        self.assertEqual(f.state, TURNING, "바깥 차선 위인데 끊으면 융합 회전이 죽는다")
        f.compute(4.97, 2.20, 285.0)
        self.assertEqual(f.state, TURNING, "4.98 까지는 맵 안이다")

    def test_map_guard_does_not_fire_in_the_middle(self):
        """맵 한가운데서는 걸리면 안 된다 — 정상 회전을 죽이지 않는다."""
        f = LaneFollower(self._cfg(), _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [14, 19, 20])
        f.set_route(route)
        (x1, y1) = route.legs[0].points[-1]
        f.compute(x1 - 0.16, y1, 180.0)
        self.assertEqual(f.state, TURNING)
        f.compute(0.50, 0.55, 150.0)                    # 경계에서 충분히 안쪽
        self.assertEqual(f.state, TURNING, "한가운데서 끊기면 정상 회전이 죽는다")

    def test_control_yaml_has_the_guard(self):
        import yaml
        cfg = yaml.safe_load(CONTROL_PATH.read_text(encoding="utf-8"))["control"]
        self.assertEqual(list(cfg["hold_steer_lanes"]), [4],
                         "정지 조향금지는 출발 지점(4번)에만 — 19/20 에 걸면 복귀가 죽는다")
        self.assertGreater(float(cfg["map_bounds"]["turn_margin_m"]), 0.0)

    def test_hold_lanes_are_separate_from_course_lanes(self):
        """🔴 두 목록을 묶으면 안 된다 (2026-08-07 밤 실주행으로 드러난 결합 사고).

        19·20 을 course_heading_lanes 에 넣는 순간 정지 조향금지까지 같이 걸려,
        차가 x=0.02(중심선 서쪽 0.37m)에 선 채 조향 0 으로 95초간 직진만 했다.
        서편은 원래 기어가는 구간이라 베이스라인 미확보가 곧 '정지' 판정이 된다.
        """
        import yaml
        cfg = yaml.safe_load(CONTROL_PATH.read_text(encoding="utf-8"))["control"]
        course = set(int(n) for n in cfg["course_heading_lanes"])
        hold = set(int(n) for n in cfg["hold_steer_lanes"])
        self.assertIn(19, course, "19·20 은 GPS 방위를 안 쓴다")
        self.assertIn(20, course)
        self.assertFalse(hold & {19, 20}, "19·20 에는 정지 조향금지를 걸면 안 된다")

    def test_stopped_on_lane20_still_steers(self):
        """19/20 에서 기어가도 복귀 조향은 살아 있어야 한다."""
        cfg = self._cfg()
        cfg["course_heading_lanes"] = [4, 19, 20]
        f = LaneFollower(cfg, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [19, 20, 3])
        f.set_route(route)
        f._leg, f._pt = 1, 1
        for _ in range(12):                      # 제자리, 중심선 서쪽 0.37m
            cmd, _ = f.compute(0.02, 2.20, 90.0)
        self.assertGreater(abs(cmd.steering_wheel_deg), 3.0,
                           "20번에서 조향이 0 이면 95초 직진 사고가 재발한다")


class TestParallelStop(unittest.TestCase):
    """🔴 면사무소 평행 정차 (2026-08-07 저녁, 사용자 지시).

    "면사무소 좌표 들어갈 때 조향을 주고 정지한다" — 19:44 실측 방위 190~235°
    (14번 방위 180°). 정차점이 옆으로 비껴 있으면 마지막까지 조향이 들어간다.
    """

    def _run(self, window, y_off):
        cfg = dict(BASE_CFG)
        cfg["parallel_stop_distance_m"] = window
        f = LaneFollower(cfg, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [14])       # 14번 단독 = 최종 다리
        f.set_route(route)
        (gx, gy) = route.legs[0].points[-1]
        cmd, _ = f.compute(gx + 0.25, gy + y_off, 180.0)         # 정차점 0.25m 앞, 횡오차
        return f, cmd, (gx, gy)

    def test_off_by_default_keeps_old_behaviour(self):
        _, cmd, _ = self._run(0.0, 0.08)
        self.assertGreater(abs(cmd.steering_wheel_deg), 5.0,
                           "창이 0 이면 예전처럼 점을 맞히려 조향한다 (사고 재현)")

    def test_window_makes_the_approach_parallel(self):
        """창 안에서는 조향이 크게 줄어든다 — 차선 위로 올라타 달리기 때문."""
        _, on, _ = self._run(0.40, 0.08)
        _, off, _ = self._run(0.0, 0.08)
        self.assertLess(abs(on.steering_wheel_deg), abs(off.steering_wheel_deg),
                        "평행 정차가 켜지면 마지막 조향이 완만해져야 한다")

    def test_target_is_beyond_the_stop_point_along_the_lane(self):
        f, _, (gx, gy) = self._run(0.40, 0.08)
        leg = f._route.legs[0]
        par = f._parallel_stop_target(leg, gx + 0.25, gy + 0.08, 0.25)
        self.assertIsNotNone(par)
        tx, ty, along = par
        self.assertLess(tx, gx, "14번은 서향 — 당근은 정차점보다 서쪽이어야 한다")
        self.assertAlmostEqual(ty, gy, places=6, msg="당근은 차선 중심선 위에 있어야 한다")
        self.assertAlmostEqual(along, 0.25, places=2, msg="차선 방향 남은 거리")

    def test_stops_on_along_lane_distance_not_euclidean(self):
        """횡오차가 남아 있어도 세로 위치가 맞으면 선다."""
        cfg = dict(BASE_CFG)
        cfg["parallel_stop_distance_m"] = 0.40
        f = LaneFollower(cfg, _table(), MAX_WHEEL, MAX_SPEED, WHEELBASE)
        route = lane_route.from_lane_numbers(_map(), [14])
        f.set_route(route)
        (gx, gy) = route.legs[0].points[-1]
        f.compute(gx + 0.25, gy + 0.09, 180.0)                  # 창 안으로 먼저 들어간다
        cmd, _ = f.compute(gx + 0.02, gy + 0.09, 180.0)         # 세로는 도달, 횡 9cm 남음
        self.assertEqual(cmd.throttle, 0.0, "차선 방향으로 도달했으면 서야 한다")

    def test_control_yaml_enables_it(self):
        import yaml
        cfg = yaml.safe_load(CONTROL_PATH.read_text(encoding="utf-8"))["control"]
        self.assertGreater(float(cfg["parallel_stop_distance_m"]), 0.0,
                           "평행 정차가 꺼졌다 — 비스듬한 정차가 재발한다")


class TestDeployedConfig(unittest.TestCase):
    """⑦ 배포 config 말뚝 — 재배포로 조용히 꺼지면 여기서 걸린다."""

    def test_control_yaml_disables_seg_heading(self):
        """🔴 2026-08-07 사용자 결정 — seg 방위 대체는 **꺼져 있어야** 한다.

        카메라 자세가 틀어지면 rows 는 정상인데 heading 에만 상수 오차가 실려
        조용히 잘못된 방위로 주행한다. 되살리려면 자세 검증 수단이 먼저다.
        """
        import yaml
        cfg = yaml.safe_load(CONTROL_PATH.read_text(encoding="utf-8"))["control"]
        sh = cfg.get("seg_heading")
        self.assertIsNotNone(sh, "control.yaml 에 seg_heading 블록이 있어야 한다")
        self.assertEqual(list(sh["lane_nums"]), [],
                         "seg 방위 대체가 다시 켜졌다 — 카메라 자세 검증 없이는 위험")
        self.assertGreater(sh["steer_rate_deg_per_s"], 0.0,
                           "출처 전환 계단 지령 차단은 유지 (다른 전이에도 쓰인다)")

    def test_turn_table_is_rolled_back_to_run9(self):
        """🔴 회전표는 **8/6 9차 상태**다 (커밋 5196ecf4). 2026-08-07 사용자 결정으로 롤백.

        왜: 8/7 오후 수동 주행 데이터로 역산한 값을 전부 얹었더니 실주행이 나빠졌다.
        7->22 는 차선 끝 0.58m 앞에 조기 발화해 코너를 가로질렀고(사람은 0.08m 앞),
        22->14 는 길이 0.21m 차선에서 즉시 발화해 복구 구간을 0으로 만들었다.
        9차·10차는 이 표로 **무개입 완주**했다 — 검증된 유일한 상태다.

        이 표만 되돌리고 `turn_throttle 0.8` 과 코드 변경(코스각 종료·레이트 리밋)은
        남겼다 (사용자 선택 B). 9차의 상한 절단 2건을 없앤 것이 그쪽이기 때문이다.
        """
        tt = TurnTable.load(WHEELBASE, MAX_WHEEL, path=TABLE_PATH)
        route = lane_route.from_lane_numbers(_map(), [4, 29, 7])
        s429 = tt.resolve(route.turns[0])
        self.assertIsNone(s429.trigger.cls,
                          "4->29 는 8/6 6차 판정대로 좌표 폴백이다")
        self.assertAlmostEqual(s429.exit_align_deg, 12.0, places=3,
                               msg="9차 상태 = defaults 12.0 (8/7 의 15.0 은 롤백 대상)")
        self.assertAlmostEqual(s429.max_turn_m, 2.20, places=3,
                               msg="실측 반경 0.678 기준 180° 호장 2.13m 를 담아야 한다")
        s297 = tt.resolve(route.turns[1])
        self.assertEqual(s297.trigger.cls, "stop_line")
        self.assertAlmostEqual(s297.trigger.near_row_frac, 0.85, places=3,
                               msg="29->7 — 9차 값. 0.73·0.68 은 수동 역산이라 롤백했다")
        self.assertAlmostEqual(s297.trigger.max_lateral_m, 0.30, places=3,
                               msg="max_lateral 도 기본값으로 — 0.51/0.53 은 수동 역산분")

    def test_run9_turn_values(self):
        """9차로 되돌린 나머지 항목 — 하나라도 8/7 값이 남아 있으면 여기서 걸린다."""
        tt = TurnTable.load(WHEELBASE, MAX_WHEEL, path=TABLE_PATH)
        s722 = tt.resolve(lane_route.from_lane_numbers(_map(), [7, 22]).turns[0])
        self.assertIsNone(s722.trigger.cls,
                          "7->22 는 9차에도 좌표 폴백이었다 (crosswalk@0.87 은 8/7 신설분)")
        self.assertAlmostEqual(s722.fallback_lead_m, 0.12, places=3,
                               msg="fallback_lead 0.12 는 9차 이전 커밋(5196ecf4)이라 유지된다")
        s2214 = tt.resolve(lane_route.from_lane_numbers(_map(), [22, 14]).turns[0])
        self.assertIsNone(s2214.trigger.cls,
                          "22->14 트리거가 되살아났다 — 22번이 0.21m 라 진입 즉시 발화한다")
        self.assertAlmostEqual(s2214.exit_align_deg, 11.0, places=3,
                               msg="종료창 11.0 은 8/6 브래킷 결과 — 트리거와 별개로 유지")

    def test_14_19_starts_past_the_lane_end(self):
        """🔴 14->19 시연 확정값 — lead 0.10 + 개루프 호장 min=max 0.856 (2026-08-08).

        이력: −0.14(8/7 밤 "끝점 지나서 개시") → +0.285(8/8, "4->29 와 타이밍 통일")
        → 0.10(8/8 저녁 확정 — 0.285 는 인도 모서리 10.5cm 관통, 사용자 결정
        "대향 차선 좀 밟아도 된다" = 인도 회피 우선). 종료는 방위 판정 대신
        호장 min=max 0.856(= π/2 × 실효 R 0.545) 개루프 — GPS 방위 무관 (§0-49~50).
        이 값 그대로 8/10 시연 무개입 완주 3회. (구 말뚝 −0.14/1.20 은 8/7 시점 값 —
        2026-08-17 리팩터링에서 시연 확정값으로 갱신)
        """
        route = lane_route.from_lane_numbers(_map(), [14, 19, 20])
        s = TurnTable.load(WHEELBASE, MAX_WHEEL, path=TABLE_PATH).resolve(route.turns[0])
        self.assertFalse(s.inherited, "14->19 는 명시 항목이다 (4->29 상속 아님)")
        self.assertIsNone(s.trigger.cls, "14->19 는 트리거 없음 (좌표 폴백)")
        self.assertAlmostEqual(s.fallback_lead_m, 0.10, places=3,
                               msg="8/8 저녁 확정 lead (0.285 는 인도 관통으로 철회)")
        self.assertAlmostEqual(s.min_turn_m, 0.856, places=3,
                               msg="개루프 호장 = π/2 × 실효 R 0.545")
        self.assertAlmostEqual(s.max_turn_m, 0.856, places=3,
                               msg="min=max — 종료 판정이 낄 자리가 없어야 GPS 방위 무관")
        self.assertFalse(s.fuse_next, "19->20 은 직진이라 묶을 대상이 아니다")

    def test_4_29_blocks_the_symmetric_inheritance(self):
        """🔴 4->29 항목은 값이 아니라 **대칭 상속 차단**이 목적이다.

        14->19 를 신설하자 4->29 가 점대칭으로 lead 0.21 · max_turn 0.85 를 그대로 받았다.
        신고된 회전만 고치기로 했으므로 명시 항목으로 막는다. 이 테스트가 깨지면
        4->29 가 조용히 14->19 값을 따라간 것이다.
        """
        s = TurnTable.load(WHEELBASE, MAX_WHEEL, path=TABLE_PATH).resolve(
            lane_route.from_lane_numbers(_map(), [4, 29, 7]).turns[0])
        self.assertFalse(s.inherited, "4->29 가 14->19 를 상속받고 있다")
        self.assertAlmostEqual(s.max_turn_m, 2.20, places=3,
                               msg="14->19 의 1.20 을 따라가면 반 바퀴도 못 돈다")

    def test_4_29_starts_early_for_the_real_radius(self):
        """🔴 4->29 도 개시를 0.21m 앞당긴다 (2026-08-07 저녁 사용자 지시).

        14->19 와 같은 처방이다 — 두 회전은 점대칭 짝이고 반경 오차도 같다.
        동향에서 우회전해 29번(x=4.61)에 안착하려면 반경 R 앞에서 시작해야 하는데,
        맵은 설계 R=0.26 기준(시작점 x=4.35 = 차선 끝)이고 실측 R 은 0.47m 다.
        필요한 시작점 x = 4.61 − 0.47 = 4.14 → 차선 끝보다 0.21m 앞.
        그대로 두면 궤적 최동단이 x=4.808(코너 한계 4.61 대비 +0.20) 로 인도를 밟는다.
        """
        s = TurnTable.load(WHEELBASE, MAX_WHEEL, path=TABLE_PATH).resolve(
            lane_route.from_lane_numbers(_map(), [4, 29, 7]).turns[0])
        self.assertAlmostEqual(s.fallback_lead_m, 0.05, places=3,
                               msg="사용자 지시 '끝점 5cm 앞에서 풀로 우회전'")
        self.assertIsNone(s.trigger.cls, "비전 트리거는 여전히 없다 — 개시만 앞당긴 것")

    def test_symmetric_pair_intentionally_differ(self):
        """🔴 4->29 와 14->19 는 점대칭 짝이지만 **개시가 다르다** — 의도된 것이다.

        4->29 는 7번까지 묶어 180° 를 돌기 때문에 앞당기지 않으면 호가 바깥 북행
        차선(x=4.86)을 침범한다. 14->19 는 뒤가 직진이라 묶이지 않고, 앞당겨도
        효과가 없어 철회했다. 같은 값이 아니어야 맞다.
        """
        tt = TurnTable.load(WHEELBASE, MAX_WHEEL, path=TABLE_PATH)
        a = tt.resolve(lane_route.from_lane_numbers(_map(), [4, 29, 7]).turns[0])
        b = tt.resolve(lane_route.from_lane_numbers(_map(), [14, 19, 20]).turns[0])
        self.assertAlmostEqual(a.fallback_lead_m, 0.05, places=3, msg="끝점 5cm 앞 개시")
        self.assertAlmostEqual(b.fallback_lead_m, 0.10, places=3,
                               msg="8/8 저녁 확정 — 대칭 짝이지만 값이 달라야 맞다")
        self.assertNotAlmostEqual(a.fallback_lead_m, b.fallback_lead_m, places=3,
                                  msg="같은 값이면 대칭 상속 차단이 무의미해진 것")
        self.assertTrue(a.fuse_next)
        self.assertFalse(b.fuse_next)

    def test_proven_turns_untouched(self):
        """실측으로 굳힌 값은 건드리지 않는다 (22->14 종료창 11.0, 20->3 개루프 0.856).

        22->14 종료창 11.0 은 8/6 브래킷 실험(12=침범 / 10=클립)의 결과.
        20->3 은 8/8 밤 개루프화(min=max 0.856 + lead 0.285)가 최종 — 8/6 의 0.85 는
        방위 종료 상한이던 시절 값이다 (2026-08-17 리팩터링에서 시연 확정값으로 갱신).
        """
        tt = TurnTable.load(WHEELBASE, MAX_WHEEL, path=TABLE_PATH)
        s2214 = tt.resolve(lane_route.from_lane_numbers(_map(), [22, 14]).turns[0])
        self.assertAlmostEqual(s2214.exit_align_deg, 11.0, places=3)
        s203 = tt.resolve(lane_route.from_lane_numbers(_map(), [20, 3]).turns[0])
        self.assertAlmostEqual(s203.min_turn_m, 0.856, places=3)
        self.assertAlmostEqual(s203.max_turn_m, 0.856, places=3)
        self.assertAlmostEqual(s203.fallback_lead_m, 0.285, places=3,
                               msg="lead 0 이면 착지 y 2.895 = 대향 차선 (8/8 실측)")


if __name__ == "__main__":
    unittest.main()
