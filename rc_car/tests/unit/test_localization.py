"""단계 2 A2 — 위치 수신 검증·유실 판정 단위 테스트 (protocol_2 §3.4·§7.2·§7.4)."""
import json
import threading
import time

import pytest

from core.state_store import StateStore
from localization.localization_service import LocalizationService
from localization.pose_validator import (JUMP, MALFORMED, NOT_FOUND, OTHER_MARKER,
                                         OUT_OF_MAP, OUT_OF_ORDER, PARSE, PoseValidator)

MAP_X, MAP_Y = 5.0, 3.0

CFG = {
    "marker_id": 4,
    "interval_ms": 100,
    "marker_yaw_offset_deg": 0,
    "max_jump_m": 0.08,
    "max_heading_jump_deg": 15.0,
    "pose_timeout_sec": 0.3,     # 테스트 가속 — 규약값 0.6의 동작을 그대로 축소
    "lost_hold_sec": 0.3,
}


def msg(x=1.0, y=1.0, heading=90.0, ts=1000.0, marker_id=4, found=True):
    m = {"marker_id": marker_id, "found": found, "timestamp": ts}
    if found:
        m["position"] = {"x": x, "y": y, "z": 0.12}
        m["heading"] = heading
    return json.dumps(m)


def make_validator(**over):
    cfg = dict(CFG)
    cfg.update(over)
    return PoseValidator(cfg, MAP_X, MAP_Y)


# ---------- 정상 채택 ----------

def test_정상_pose_채택하고_z는_읽지_않는다():
    v = make_validator()
    r = v.validate(msg(x=1.5, y=2.0, heading=90.0, ts=1000.0), received_monotonic_s=5.0)
    assert r.accepted
    assert (r.pose.x, r.pose.y) == (1.5, 2.0)
    assert r.pose.heading_deg == 90.0
    assert r.pose.source_timestamp_s == 1000.0
    assert r.pose.received_monotonic_s == 5.0     # 안전 판단 기준은 단조 시계 (§3.4)


def test_마커_yaw_오프셋이_적용되고_0_360으로_정규화된다():
    v = make_validator(marker_yaw_offset_deg=90)
    assert v.validate(msg(heading=350.0), 0.0).pose.heading_deg == pytest.approx(80.0)


# ---------- §3.4 거부 규칙 ----------

def test_다른_마커는_무시한다():
    assert make_validator().validate(msg(marker_id=7), 0.0).reason == OTHER_MARKER


def test_found_false는_좌표를_읽지_않고_not_found():
    # found=false면 position·heading 키 자체가 없다 — 키 접근으로 터지면 안 된다
    r = make_validator().validate(msg(found=False), 0.0)
    assert r.reason == NOT_FOUND and r.pose is None


@pytest.mark.parametrize("raw", [
    "{깨진 json",
    "[1, 2, 3]",
])
def test_해석_불가는_parse(raw):
    assert make_validator().validate(raw, 0.0).reason == PARSE


@pytest.mark.parametrize("bad", [
    {"marker_id": 4, "found": True, "timestamp": 1000.0, "heading": 90.0},          # position 없음
    {"marker_id": 4, "found": True, "timestamp": 1000.0,
     "position": {"x": 1.0, "y": 1.0}},                                             # heading 없음
    {"marker_id": 4, "found": True, "timestamp": 1000.0,
     "position": {"x": "1.0", "y": 1.0}, "heading": 90.0},                          # 타입 오류
    {"marker_id": 4, "found": True, "timestamp": 1000.0,
     "position": {"x": float("nan"), "y": 1.0}, "heading": 90.0},                    # NaN
    {"marker_id": 4, "found": True, "position": {"x": 1.0, "y": 1.0}, "heading": 9},  # ts 없음
])
def test_키_부재나_타입_오류는_malformed(bad):
    assert make_validator().validate(json.dumps(bad), 0.0).reason == MALFORMED


def test_timestamp_순서_역전은_거부():
    v = make_validator()
    assert v.validate(msg(ts=1000.0), 0.0).accepted
    assert v.validate(msg(ts=999.9), 0.1).reason == OUT_OF_ORDER
    assert v.validate(msg(ts=1000.0), 0.1).reason == OUT_OF_ORDER      # 같은 값도 거부


@pytest.mark.parametrize("x,y", [(-0.01, 1.0), (5.01, 1.0), (1.0, -0.01), (1.0, 3.01)])
def test_맵_범위_밖은_거부(x, y):
    assert make_validator().validate(msg(x=x, y=y), 0.0).reason == OUT_OF_MAP


def test_위치_점프_거부():
    v = make_validator()
    v.validate(msg(x=1.0, y=1.0, ts=1000.0), 0.0)
    assert v.validate(msg(x=1.5, y=1.0, ts=1000.1), 0.1).reason == JUMP


def test_heading_점프_거부하되_0_360_순환은_점프가_아니다():
    v = make_validator()
    v.validate(msg(heading=359.0, ts=1000.0), 0.0)
    assert v.validate(msg(heading=1.0, ts=1000.1), 0.1).accepted        # 실제 회전 2°
    assert v.validate(msg(heading=40.0, ts=1000.2), 0.2).reason == JUMP  # 39°


def test_프레임_누락_뒤_정상_이동은_점프가_아니다():
    # 허용치가 경과 시간에 비례한다 — 0.5초(5프레임 누락) 뒤 0.3m는 정상 주행 범위
    v = make_validator()
    v.validate(msg(x=1.0, y=1.0, ts=1000.0), 0.0)
    assert v.validate(msg(x=1.3, y=1.0, ts=1000.5), 0.5).accepted


def test_reset_후에는_점프_검사_기준이_사라진다():
    v = make_validator()
    v.validate(msg(x=1.0, y=1.0, ts=1000.0), 0.0)
    v.reset()
    assert v.validate(msg(x=4.0, y=2.5, ts=1000.1), 0.1).accepted


# ---------- LocalizationService — 유실·복귀 판정 ----------

class Hooks:
    def __init__(self):
        self.lost = 0
        self.recovered = 0

    def on_lost(self):
        self.lost += 1

    def on_recovered(self):
        self.recovered += 1


def make_service(**over):
    cfg = dict(CFG)
    cfg.update(over)
    store = StateStore()
    hooks = Hooks()
    svc = LocalizationService(threading.Event(), store, PoseValidator(cfg, MAP_X, MAP_Y),
                              cfg, hooks.on_lost, hooks.on_recovered)
    return store, svc, hooks


def test_첫_pose_전에는_유실을_오류로_격상하지_않는다():
    _, svc, hooks = make_service()
    svc.on_disconnected()
    svc.on_raw(msg(found=False))
    svc.tick()
    assert hooks.lost == 0 and not svc.lost


def test_found_false는_즉시_stale이지만_전이는_지속_후():
    store, svc, hooks = make_service()
    svc.on_raw(msg(ts=1000.0))
    assert store.snapshot().pose_stale is False

    svc.on_raw(msg(ts=1000.1, found=False))
    assert store.snapshot().pose_stale is True     # 이전 pose를 주행에 쓰지 않는다 (§3.4)
    assert hooks.lost == 0                          # 아직 오류 정지 아님 (§7.4 — 0.6초)

    svc.tick()
    assert hooks.lost == 0                          # 지속 시간 미달
    time.sleep(CFG["pose_timeout_sec"] + 0.05)
    svc.tick()
    svc.tick()
    assert hooks.lost == 1 and svc.lost              # 지속 후 1회만


def test_정상화되면_recovered_1회_후_복귀():
    store, svc, hooks = make_service()
    svc.on_raw(msg(ts=1000.0))
    time.sleep(CFG["pose_timeout_sec"] + 0.05)
    svc.tick()
    assert hooks.lost == 1

    svc.on_raw(msg(x=4.0, y=2.5, ts=1000.5))         # 유실 중 이동분 — reset 덕에 수용
    assert hooks.recovered == 1
    assert store.snapshot().pose_stale is False      # 복귀 시점에 유효 pose가 이미 있다
    assert not svc.lost

    svc.tick()
    assert hooks.recovered == 1                      # 중복 호출 없음


def test_연결_끊김은_지속_시간을_기다리지_않는다():
    _, svc, hooks = make_service()
    svc.on_raw(msg(ts=1000.0))
    svc.on_disconnected()
    assert hooks.lost == 1 and svc.lost               # §7.4 — 즉시 판정


def test_다른_마커_수신은_유실_타이머를_되돌리지_않는다():
    _, svc, hooks = make_service()
    svc.on_raw(msg(ts=1000.0))
    time.sleep(CFG["pose_timeout_sec"] + 0.05)
    svc.on_raw(msg(marker_id=99, ts=2000.0))
    svc.tick()
    assert hooks.lost == 1


# ---------- 큐브 4면 모드 (워크로그 §0-31) ----------

CUBE = {"enabled": True,
        "face_yaw_offset_deg": {6: 0.0, 7: 90.0, 8: 180.0, 9: 270.0},
        "center_offset_m": 0.06}


def test_큐브_4면은_전부_수용하고_그_외_마커는_거부한다():
    v = make_validator(cube=CUBE)
    assert v.validate(msg(marker_id=6, ts=1000.0), 0.0).accepted
    assert v.validate(msg(marker_id=4, ts=1001.0), 0.0).reason == OTHER_MARKER  # 초기화 마커
    assert v.validate(msg(marker_id=9, ts=1002.0, x=1.01), 0.0).accepted


def test_큐브_면별_yaw_오프셋으로_차량_전방을_만든다():
    v = make_validator(cube=CUBE)
    # 면 8(후면 가정, 오프셋 180): 보고 90 → 차량 전방 270
    r = v.validate(msg(marker_id=8, heading=90.0), 0.0)
    assert r.pose.heading_deg == pytest.approx(270.0)


def test_큐브_면_좌표를_바깥_법선_반대_방향으로_중심_보정한다():
    v = make_validator(cube=CUBE)
    # 면이 동쪽(heading 0)을 보면 차량 중심은 면에서 서쪽으로 6cm
    p = v.validate(msg(marker_id=6, x=1.0, y=1.0, heading=0.0), 0.0).pose
    assert p.x == pytest.approx(0.94)
    assert p.y == pytest.approx(1.0)


def test_큐브_면_전환_직후는_점프_판정을_건너뛰고_같은_면은_유지한다():
    v = make_validator(cube=CUBE)
    assert v.validate(msg(marker_id=6, x=1.0, y=1.0, heading=0.0, ts=1000.0), 0.0).accepted
    # 면 전환(6→7): 중심 보정 후에도 26cm 불연속 — 전환 1회는 흡수한다
    assert v.validate(msg(marker_id=7, x=1.2, y=1.0, heading=270.0, ts=1000.1), 0.0).accepted
    # 같은 면(7)에서의 큰 점프는 여전히 거부
    assert v.validate(msg(marker_id=7, x=1.5, y=1.0, heading=270.0, ts=1000.2), 0.0).reason == JUMP


def test_큐브_꺼져_있으면_기존_단일_마커_동작_그대로다():
    v = make_validator()   # cube 키 없음
    assert v.validate(msg(marker_id=6), 0.0).reason == OTHER_MARKER
    assert v.validate(msg(marker_id=4), 0.0).accepted


# ---------- 서버→맵 frame_transform (맵 실측) ----------

def test_frame_transform_스케일_오프셋이_적용되고_맵_범위는_변환_후_판정한다():
    ft = {"a": [[0.5, 0.0], [0.0, 0.5]], "t": [0.1, 0.2]}
    p = make_validator(frame_transform=ft).validate(msg(x=2.0, y=2.0), 0.0).pose
    assert p.x == pytest.approx(1.1)
    assert p.y == pytest.approx(1.2)
    # 서버 좌표 (9.6, 5.4)는 원시값으론 맵 밖이지만 변환하면 (4.9, 2.9) — 채택돼야 한다
    r = make_validator(frame_transform=ft).validate(msg(x=9.6, y=5.4), 0.0)
    assert r.accepted


def test_frame_transform_회전_성분이_heading에_더해진다():
    # 반시계 90° 회전 + 평행이동: server (1,1) → map (2,1), heading 0 → 90
    ft = {"a": [[0.0, -1.0], [1.0, 0.0]], "t": [3.0, 0.0]}
    p = make_validator(frame_transform=ft).validate(
        msg(x=1.0, y=1.0, heading=0.0), 0.0).pose
    assert p.x == pytest.approx(2.0)
    assert p.y == pytest.approx(1.0)
    assert p.heading_deg == pytest.approx(90.0)
