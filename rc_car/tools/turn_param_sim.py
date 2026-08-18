# -*- coding: utf-8 -*-
"""시나리오 1 리허설 폐루프 시뮬 — 실제 차량 코드(맵·강제 노선·LaneFollower) + 실측 차량 모델.

2026-08-06 개정: 주행 로직 교체(§0-46)에 맞춰 원호 추종(legacy WaypointFollower) 시뮬을
차선 번호 point-to-point + 2단 회전(LaneFollower) 시뮬로 전환.
- 코스 = 시나리오 1 확정 노선 (안 1 픽업 + 목적 원안) — config/routes.yaml 과 같은 번호열을
  plan_via_numbers 로 태워 보드와 동일한 경로 편성을 쓴다
- 비전 미가동(좌표 폴백) 기준 = 목·금 첫 주행과 같은 구성. 비전 발화 시뮬은
  세로 캘리브(y_px→m) 실측 후에나 의미가 있다 (§0-45 ③) — 표가 채워지면 여기에 얹는다
- 유닛 폐루프(test_lane_following)는 이상 운동학이다. 서보 속도 제한·물리 클램프·조향 편향·
  GPS 10Hz/파이프라인 지연/노이즈까지 모델해 실차 대조군 수치를 만드는 쪽은 여기다

관찰 지표
- 회전 종료 정렬 오차: 시뮬 11.8° vs 임계 12.0° 로 임계에 붙어 있다 (§0-46) — 첫 튜닝 항목.
  회전(TURNING 종료)마다 진짜 heading 과 다음 다리 방위의 차를 기록한다
- 정차 오차 / 완주 시간: 실차가 크게 벗어나면 모델 가정이 아니라 로직 문제를 의심 (워크로그)

모델 가정 (결과 해석 시 유의):
- 속도: 실측점 보간 (2026-08-04 실차: 데드존 ~0.24 / 0.3→0.052 / 1.0→0.199 m/s)
- 서보: 조향 속도 제한 120°/s(바퀴각), 물리 클램프 좌 -30°/우 +27° (트림 118 부작용 §0-42.
  우회전 기본각 28.3°가 우측 클램프에 걸리는 것까지 모델)
- 잔여 바퀴각 편향 -1.6° (2026-08-04 트림 118 GPS 실측 잔차)
- GPS: 10Hz 샘플 유지(제어는 50Hz), 지연 0.15s, 노이즈 위치 sd 1cm / heading sd 1.5°
"""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import yaml
from mapping import geometry
from mapping.lane_map import load_lane_map
from mapping.map_matcher import MapMatcher
from navigation import lane_route
from navigation.destination_resolver import DestinationResolver
from navigation.route_planner import RoutePlanner
from navigation.turn_table import TurnTable
from control.lane_follower import TURNING, LaneFollower

MAP = str(ROOT.parent / "map" / "main_track_map.yaml")
CTRL = str(ROOT / "config" / "control.yaml")
TABLE = str(ROOT / "config" / "turn_table.yaml")

WHEELBASE = 0.14
MAX_WHEEL = 30.0          # 소프트웨어 명령 한계 (follower 클램프)
WHEEL_RIGHT_MAX = 27.0    # 물리 우측 한계 — 트림 118 부작용 (§0-42)
MAX_SPEED = 0.22
DT = 0.02                 # 50Hz 제어 주기
GPS_DT = 0.10             # 10Hz pose
SERVO_RATE = 120.0        # deg/s (바퀴각)
ACCEL = 0.5               # m/s^2
NOISE_POS = 0.01
NOISE_HEAD = 1.5
WHEEL_BIAS = -1.6         # 트림 118 잔차 (2026-08-04 GPS 실측)
GPS_LATENCY = 0.15        # 촬영→서버→ws→차량 파이프라인 지연 (2026-08-05 실측 근사)
TIMEOUT = 300.0

with open(CTRL, encoding="utf-8") as f:
    BASE_CTRL = yaml.safe_load(f)["control"]
LM = load_lane_map(MAP)
TT = TurnTable.load(WHEELBASE, MAX_WHEEL, Path(TABLE))

# 스로틀→속도: 실측점 선형 보간 (2026-08-04). 데드존 밑은 0
SPEED_PTS = ((0.24, 0.0), (0.30, 0.052), (1.00, 0.199))

# 시나리오 2 (자료/시연시나리오.md 확정 2026-08-06 오전 — config/routes.yaml 과 동일 번호열)
COURSES = (
    dict(name="픽업 4중앙→면사무소", numbers=(4, 29, 7, 22, 14),
         start=(3.75, 2.61, 0.0), dest=(1.20, 0.39)),
    dict(name="목적 면사무소→우리집", numbers=(14, 19, 20, 3),
         start=(1.20, 0.39, 180.0), dest=(1.20, 2.61)),
)


def lateral_dev(leg, x: float, y: float) -> float:
    """다리 직선(무한선) 기준 횡방향 거리 — 차선 유지 품질.

    세그먼트 클램프 거리(project_point)를 쓰면 직진 커넥터로 교차로를 건너는 동안
    (leg 는 이미 다음 차선인데 차량은 그 시작점 앞) 진행방향 거리가 이탈로 잡힌다.
    다음 차선은 직전 차선과 일직선이므로 무한선 횡거리가 그 구간에서도 유효하다.
    """
    (x0, y0), (x1, y1) = leg.points[0], leg.points[-1]
    dx, dy = x1 - x0, y1 - y0
    ln = math.hypot(dx, dy)
    if ln < 1e-9:
        return math.hypot(x - x0, y - y0)
    return abs((x - x0) * dy - (y - y0) * dx) / ln


def throttle_to_speed(th: float) -> float:
    if th <= SPEED_PTS[0][0]:
        return 0.0
    for (a, va), (b, vb) in zip(SPEED_PTS, SPEED_PTS[1:]):
        if th <= b:
            return va + (vb - va) * (th - a) / (b - a)
    return SPEED_PTS[-1][1]


def build_route(numbers, start, dest):
    """보드와 같은 편성 — 시작 트림은 매칭, 목적 트림은 스냅, 몸통은 강제 번호열."""
    sx, sy, sh = start
    matcher = MapMatcher(LM, {"distance_weight": 1.0, "heading_weight_per_deg": 0.01,
                              "max_center_distance_m": 0.20, "max_heading_error_deg": 60.0})
    m = matcher.match(sx, sy, sh)
    assert m and m.kind == "lane", f"시작점 매칭 실패: ({sx}, {sy}) {sh}"
    assert lane_route.ID2NUM.get(m.segment_id) == numbers[0], \
        f"시작 차선 불일치: 매칭 {m.segment_id} vs 노선 {numbers[0]}"
    stop = DestinationResolver(LM, 0.30).resolve(dest[0], dest[1])
    assert lane_route.ID2NUM.get(stop.lane_id) == numbers[-1], \
        f"목적 차선 불일치: 스냅 {stop.lane_id} vs 노선 {numbers[-1]}"
    route = RoutePlanner(LM).plan_via_numbers(numbers, m.progress_s, stop.progress_s)
    return lane_route.from_route(route)


def run_once(course, seed):
    lr = build_route(course["numbers"], course["start"], course["dest"])
    fol = LaneFollower(BASE_CTRL, TT, MAX_WHEEL, MAX_SPEED, WHEELBASE)  # marks 없음 = 좌표 폴백
    fol.set_route(lr)

    rng = random.Random(seed)
    sx, sy, sh = course["start"]
    x, y, th = sx, sy, math.radians(sh)
    v, wheel = 0.0, 0.0
    gx, gy, gh = x, y, sh                      # GPS 샘플 (hold)
    next_gps, pending = 0.0, []                # (도착 시각, gx, gy, gh)
    t = 0.0
    align_errs = {}                            # 회전 키 → 종료 정렬 오차(진짜 heading 기준)
    max_dev_follow, dev_at = 0.0, None
    prev_state = fol.state
    while t < TIMEOUT:
        if t >= next_gps:
            pending.append((t + GPS_LATENCY,
                            x + rng.gauss(0, NOISE_POS),
                            y + rng.gauss(0, NOISE_POS),
                            math.degrees(th) + rng.gauss(0, NOISE_HEAD)))
            next_gps += GPS_DT
        while pending and pending[0][0] <= t:
            _, gx, gy, gh = pending.pop(0)
        cmd, rem = fol.compute(gx, gy, gh)
        # 회전 종료 관측 — TURNING 을 빠져나온 tick 에 다음 다리 방위와의 정렬 오차 기록
        if prev_state == TURNING and fol.state != TURNING:
            leg = lr.legs[fol.leg_index]
            turn = lr.turns[fol.leg_index - 1]
            err = abs(geometry.heading_diff_deg(math.degrees(th), leg.heading_deg()))
            align_errs[turn.key] = max(align_errs.get(turn.key, 0.0), err)
        prev_state = fol.state
        if cmd.throttle == 0.0 and rem <= BASE_CTRL["stop_trigger_radius_m"]:
            stop_err = math.hypot(x - course["dest"][0], y - course["dest"][1])
            return dict(done=True, time=t, stop_err=stop_err,
                        align=align_errs, max_dev_follow=max_dev_follow, dev_at=dev_at)
        # 서보 속도 제한 + 물리 클램프(좌 -30/우 +27) + 자전거 모델
        dw = cmd.steering_wheel_deg - wheel
        wheel += max(-SERVO_RATE * DT, min(SERVO_RATE * DT, dw))
        eff = max(-MAX_WHEEL, min(WHEEL_RIGHT_MAX, wheel))
        vt = throttle_to_speed(cmd.throttle)
        v = min(vt, v + ACCEL * DT) if vt > v else vt
        th += -v / WHEELBASE * math.tan(math.radians(eff + WHEEL_BIAS)) * DT
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        # 직선(FOLLOW/ARMED) 구간 이탈 — 현재 다리 직선 기준 횡방향 거리
        if fol.state != TURNING and fol.has_route:
            leg = lr.legs[fol.leg_index]
            dev = lateral_dev(leg, x, y)
            if dev > max_dev_follow:
                max_dev_follow = dev
                dev_at = (x, y, leg.lane_num, fol.state)
        t += DT
    return dict(done=False, time=TIMEOUT, stop_err=float("nan"),
                align=align_errs, max_dev_follow=max_dev_follow, dev_at=dev_at)


if __name__ == "__main__":
    print(f"우회전 기본각 atan(축거/0.26) = {math.degrees(math.atan(WHEELBASE / 0.26)):.1f}° / "
          f"물리 우측 한계 {WHEEL_RIGHT_MAX:.0f}° / 조향 편향 {WHEEL_BIAS:+.1f}°")
    print("비전 미가동(좌표 폴백) 기준. 정렬 오차가 표의 exit_align 임계를 넘으면 [!] 표시\n")
    for course in COURSES:
        lr = build_route(course["numbers"], course["start"], course["dest"])
        thresholds = {tn.key: TT.resolve(tn).exit_align_deg
                      for tn in lr.turns if TT.resolve(tn).is_turn}
        worst = dict(done=True, time=0.0, stop_err=0.0, max_dev_follow=0.0, dev_at=None)
        align = {}
        for seed in (1, 2, 3):
            r = run_once(course, seed)
            worst["done"] = worst["done"] and r["done"]
            for k in ("time", "stop_err"):
                worst[k] = max(worst[k], r[k])
            if r["max_dev_follow"] > worst["max_dev_follow"]:
                worst["max_dev_follow"] = r["max_dev_follow"]
                worst["dev_at"] = r["dev_at"]
            for key, err in r["align"].items():
                align[key] = max(align.get(key, 0.0), err)
        state = "완주" if worst["done"] else "!!미완주"
        at = worst["dev_at"]
        at_txt = f" @({at[0]:.2f},{at[1]:.2f}) {at[3]} 다리{at[2]}" if at else ""
        print(f"[{course['name']}] {lr.describe()}")
        print(f"  {state}  시간 {worst['time']:.1f}s  정차 오차 {worst['stop_err']*100:.1f}cm  "
              f"직선 최대 이탈 {worst['max_dev_follow']*100:.1f}cm{at_txt}  (seed 3회 최악치)")
        for key, err in align.items():
            limit = thresholds.get(key, 12.0)
            mark = " [!]" if err >= limit else ""
            print(f"  회전 {key:>7} 종료 정렬 오차 {err:5.1f}° (임계 {limit:.1f}°){mark}")
        print()
