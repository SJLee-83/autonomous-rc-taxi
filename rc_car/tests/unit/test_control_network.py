"""단계 4 단위 테스트 — messages 검증 · 수용 매트릭스 · 상태 전이 · 재전송 (protocol_2 §2·§4~§7)."""
import json
import threading
import time

import pytest

from core.enums import DrivingState, ServiceKind, to_service_state
from core.models import LocalizationPose
from core.state_store import StateStore
from network.command_policy import CommandPolicy
from network.messages import build_info, build_report, parse_command
from network.report_manager import ReportManager
from network.telemetry import TelemetryWorker

MAP_X, MAP_Y = 5.0, 3.0
CS_CFG = {"complete_resend_interval_sec": 1.0, "error_resend_interval_sec": 1.0}


class FakeClient:
    """송신 기록만 하는 가짜 ControlClient."""
    def __init__(self):
        self.sent: list[dict] = []
        self.connected = True

    def send_json(self, obj: dict) -> bool:
        if self.connected:
            self.sent.append(obj)
        return self.connected

    def reports(self, kind: str) -> list[dict]:
        return [m for m in self.sent if m.get("header") == "report" and m.get("report") == kind]


class FakeSupervisor:
    def __init__(self):
        self.force_stop_count = 0

    def force_stop(self) -> None:
        self.force_stop_count += 1


def make_stack():
    store = StateStore()
    store.set_driving_state(DrivingState.WAITING)
    client = FakeClient()
    supervisor = FakeSupervisor()
    reports = ReportManager(threading.Event(), client, store, CS_CFG)
    policy = CommandPolicy(store, supervisor, reports, MAP_X, MAP_Y)
    return store, client, supervisor, reports, policy


def cmd(command: str, loc=None, **extra) -> str:
    msg = {"protocol_version": "2.3", "header": "command",
           "timestamp": time.time(), "command": command, **extra}
    if loc is not None:
        msg["loc"] = loc
    return json.dumps(msg, ensure_ascii=False)


def give_pose(store, x=1.0, y=1.0, heading=0.0, ts=None):
    store.update_pose(LocalizationPose(x, y, heading,
                                       ts if ts is not None else time.time(),
                                       time.monotonic()))


# ---------- messages.parse_command (§2.10 · §6.5) ----------

def test_parse_rejects_invalid_json_and_unknown():
    assert parse_command("{잘림", MAP_X, MAP_Y) is None
    assert parse_command(json.dumps({"header": "command", "command": "fly"}), MAP_X, MAP_Y) is None
    assert parse_command(json.dumps({"header": "info"}), MAP_X, MAP_Y) is None
    assert parse_command(json.dumps(["not", "dict"]), MAP_X, MAP_Y) is None


def test_parse_move_requires_valid_loc_in_map():
    assert parse_command(cmd("move"), MAP_X, MAP_Y) is None                       # loc 없음
    assert parse_command(cmd("move", {"x": 7.2, "y": -1}), MAP_X, MAP_Y) is None  # 맵 밖
    assert parse_command(cmd("move", {"x": "a", "y": 1}), MAP_X, MAP_Y) is None   # 타입 오류
    got = parse_command(cmd("move", {"x": 3.4, "y": 2.2}), MAP_X, MAP_Y)
    assert got == ("move", {"x": 3.4, "y": 2.2})


def test_parse_ignores_loc_on_stop_and_unknown_fields():
    # stop에 loc이 와도 오류가 아니라 loc만 무시 (§6.3) / 모르는 필드는 무시 (§2.4)
    assert parse_command(cmd("stop", {"x": 1, "y": 1}), MAP_X, MAP_Y) == ("stop", None)
    assert parse_command(cmd("resume", extra_field=123), MAP_X, MAP_Y) == ("resume", None)


def test_parse_version_rules():
    bad_major = json.dumps({"protocol_version": "3.0", "header": "command", "command": "stop"})
    assert parse_command(bad_major, MAP_X, MAP_Y) is None          # major 불일치 → 무시 (§2.4)
    minor = json.dumps({"protocol_version": "2.4", "header": "command", "command": "resume"})
    assert parse_command(minor, MAP_X, MAP_Y) == ("resume", None)  # minor → 경고 후 진행


# ---------- 수용 매트릭스 · 상태 전이 (§4.5 · §6.4) ----------

def test_full_normal_cycle_transitions():
    store, client, _, _, policy = make_stack()

    # 대기 중 + move → 호출 응답 이동 중, accept 1회
    policy.on_raw_message(cmd("move", {"x": 3.0, "y": 2.0}))
    snap = store.snapshot()
    assert to_service_state(snap.driving_state, snap.service_kind) == "호출 응답 이동 중"
    assert snap.mission.kind == ServiceKind.PICKUP
    assert len(client.reports("accept")) == 1

    # 도착 → complete 즉시 1회 + ARRIVED (state 표기는 아직 이동 중 유지)
    policy.notify_arrived()
    snap = store.snapshot()
    assert snap.driving_state == DrivingState.ARRIVED
    assert to_service_state(snap.driving_state, snap.service_kind) == "호출 응답 이동 중"
    assert len(client.reports("complete")) == 1

    # stop → 탑승 대기 중
    policy.on_raw_message(cmd("stop"))
    snap = store.snapshot()
    assert to_service_state(snap.driving_state, snap.service_kind) == "탑승 대기 중"
    assert snap.mission is None

    # 두 번째 move → 고객 탑승 이동 중 (직전 상태가 하차 운행을 결정, §4.5)
    policy.on_raw_message(cmd("move", {"x": 0.5, "y": 0.5}))
    snap = store.snapshot()
    assert to_service_state(snap.driving_state, snap.service_kind) == "고객 탑승 이동 중"
    assert snap.mission.kind == ServiceKind.CARRY

    # 도착 + stop → 대기 중
    policy.notify_arrived()
    policy.on_raw_message(cmd("stop"))
    snap = store.snapshot()
    assert to_service_state(snap.driving_state, snap.service_kind) == "대기 중"


def test_matrix_rejects_move_while_driving_and_in_error():
    store, client, _, _, policy = make_stack()
    policy.on_raw_message(cmd("move", {"x": 3.0, "y": 2.0}))
    dest_before = store.snapshot().mission

    # 주행 중 move 거부 — 목적지 불변, accept 추가 없음 (§6.4)
    policy.on_raw_message(cmd("move", {"x": 1.0, "y": 1.0}))
    assert store.snapshot().mission == dest_before
    assert len(client.reports("accept")) == 1

    # 오류 정지 중 move 거부
    policy.on_control_disconnected()
    policy.on_raw_message(cmd("move", {"x": 1.0, "y": 1.0}))
    assert store.snapshot().mission == dest_before
    assert len(client.reports("accept")) == 1


def test_matrix_ignores_stop_before_arrival_and_stray_resume():
    store, _, _, _, policy = make_stack()
    policy.on_raw_message(cmd("move", {"x": 3.0, "y": 2.0}))

    # 주행 중(도착 전) stop 무시 — 계약 위반 (§6.4)
    policy.on_raw_message(cmd("stop"))
    snap = store.snapshot()
    assert snap.driving_state in (DrivingState.PLANNING,)
    assert snap.mission is not None

    # 오류 아닌 상태의 resume 무시
    policy.on_raw_message(cmd("resume"))
    assert store.snapshot().driving_state == DrivingState.PLANNING


# ---------- 오류 · 복귀 (§7) ----------

def test_control_loss_recovery_resumes_driving_state():
    store, client, supervisor, _, policy = make_stack()
    policy.on_raw_message(cmd("move", {"x": 3.0, "y": 2.0}))

    policy.on_control_disconnected()
    snap = store.snapshot()
    assert snap.driving_state == DrivingState.ERROR
    assert to_service_state(snap.driving_state, snap.service_kind) == "오류 정지"
    assert supervisor.force_stop_count >= 1
    assert snap.mission is not None          # 목적지는 지우지 않는다 (§6.2 ActiveMission)

    # 재연결 → error 즉시 1회 (resume까지 반복은 tick이 담당)
    policy.on_control_connected()
    assert len(client.reports("error")) == 1

    # resume → 직전 state 복귀 + 미션 유지 (주행 재개 가능)
    policy.on_raw_message(cmd("resume"))
    snap = store.snapshot()
    assert snap.driving_state == DrivingState.PLANNING
    assert snap.mission is not None


def test_gps_loss_self_recovery_without_resume():
    store, client, _, _, policy = make_stack()
    policy.on_raw_message(cmd("move", {"x": 3.0, "y": 2.0}))

    policy.on_gps_lost()
    assert store.snapshot().driving_state == DrivingState.ERROR
    assert len(client.reports("error")) == 1      # GPS 유실 error는 1회 (§5.2)

    policy.on_gps_lost()                          # 중복 통지 → 추가 송신 없음
    assert len(client.reports("error")) == 1

    policy.on_gps_recovered()                     # recovered 후 자체 복귀 (§7.2)
    assert len(client.reports("recovered")) == 1
    assert store.snapshot().driving_state == DrivingState.PLANNING


def test_error_while_waiting_recovers_without_driving():
    store, _, _, _, policy = make_stack()
    policy.on_control_disconnected()
    policy.on_control_connected()
    policy.on_raw_message(cmd("resume"))
    snap = store.snapshot()
    assert snap.driving_state == DrivingState.WAITING
    assert snap.mission is None                   # 재개할 목적지가 없다 (§7.2)


def test_simultaneous_loss_requires_both_conditions():
    store, client, _, _, policy = make_stack()
    policy.on_raw_message(cmd("move", {"x": 3.0, "y": 2.0}))

    policy.on_control_disconnected()
    policy.on_gps_lost()
    policy.on_control_connected()
    policy.on_raw_message(cmd("resume"))          # 조건 ①만 충족
    assert store.snapshot().driving_state == DrivingState.ERROR   # 복귀 보류 (§7.5)

    policy.on_gps_recovered()                     # 조건 ② 충족 → 복귀
    assert len(client.reports("recovered")) == 1
    assert store.snapshot().driving_state == DrivingState.PLANNING


def test_simultaneous_loss_reverse_order():
    store, _, _, _, policy = make_stack()
    policy.on_raw_message(cmd("move", {"x": 3.0, "y": 2.0}))
    policy.on_control_disconnected()
    policy.on_gps_lost()
    policy.on_control_connected()
    policy.on_gps_recovered()                     # GPS 먼저 정상화
    assert store.snapshot().driving_state == DrivingState.ERROR
    policy.on_raw_message(cmd("resume"))          # 마지막 조건 충족 순간 복귀
    assert store.snapshot().driving_state == DrivingState.PLANNING


# ---------- 재전송 (§5.2 · §7.2 · §7.3) ----------

def test_complete_resend_keeps_first_timestamp_and_stops_on_stop():
    store, client, _, reports, policy = make_stack()
    policy.on_raw_message(cmd("move", {"x": 3.0, "y": 2.0}))
    policy.notify_arrived()

    reports.tick()
    reports.tick()
    completes = client.reports("complete")
    assert len(completes) == 3                    # 즉시 1회 + tick 2회
    assert len({m["timestamp"] for m in completes}) == 1   # 최초 발생 시각 유지

    policy.on_raw_message(cmd("stop"))
    reports.tick()
    assert len(client.reports("complete")) == 3   # stop 후 재전송 중단


def test_complete_resend_pauses_in_error_and_resumes_after_recovery():
    store, client, _, reports, policy = make_stack()
    policy.on_raw_message(cmd("move", {"x": 3.0, "y": 2.0}))
    policy.notify_arrived()
    first_ts = client.reports("complete")[0]["timestamp"]

    # complete 재전송 중 오류 (2차 검증 시나리오)
    policy.on_control_disconnected()
    reports.tick()
    assert len(client.reports("complete")) == 1   # 오류 정지 중 complete 정지 (§7.2)

    policy.on_control_connected()                 # error 재전송 시작
    reports.tick()
    assert len(client.reports("error")) >= 2

    policy.on_raw_message(cmd("resume"))          # ARRIVED 복귀 → complete 재개
    assert store.snapshot().driving_state == DrivingState.ARRIVED
    reports.tick()
    completes = client.reports("complete")
    assert len(completes) == 2
    assert completes[-1]["timestamp"] == first_ts # 복귀 후에도 사건 식별자 유지

    policy.on_raw_message(cmd("stop"))            # 정상 마무리
    assert to_service_state(store.snapshot().driving_state,
                            store.snapshot().service_kind) == "탑승 대기 중"


def test_error_resend_keeps_timestamp_until_resume():
    store, client, _, reports, policy = make_stack()
    policy.on_control_disconnected()
    policy.on_control_connected()
    reports.tick()
    reports.tick()
    errors = client.reports("error")
    assert len(errors) == 3
    assert len({m["timestamp"] for m in errors}) == 1

    policy.on_raw_message(cmd("resume"))
    reports.tick()
    assert len(client.reports("error")) == 3


# ---------- telemetry (§4) ----------

def test_telemetry_gates_until_first_pose_then_reports():
    store, client, _, _, _ = make_stack()
    tw = TelemetryWorker(threading.Event(), client, store,
                         {"center_deg": 108, "wheel_angle_ratio": 0.526}, rate_hz=5)
    tw.tick()
    assert client.sent == []                      # 첫 유효 pose 게이트 (§4.0)

    give_pose(store, x=1.2, y=0.85, heading=92.4)
    tw.tick()
    info = client.sent[-1]
    assert info["header"] == "info"
    assert info["loc"] == {"x": 1.2, "y": 0.85, "heading": 92.4, "stale": False}
    assert info["steer"] == 108                   # 중앙 = 서보 108 (§4.4)
    assert info["state"] == "대기 중"

    store.mark_pose_stale()                       # 유실 — 마지막 값 유지 + stale (§4.2)
    tw.tick()
    info = client.sent[-1]
    assert info["loc"]["stale"] is True
    assert info["loc"]["x"] == 1.2


def test_telemetry_steer_servo_conversion():
    store, client, _, _, _ = make_stack()
    tw = TelemetryWorker(threading.Event(), client, store,
                         {"center_deg": 108, "wheel_angle_ratio": 0.526}, rate_hz=5)
    give_pose(store)
    from core.models import ControlCommand
    store.set_control_command(ControlCommand(throttle=0.3, steering_wheel_deg=10.0))
    tw.tick()
    # 서보각 = 108 + 10 / 0.526 ≈ 127.0 (§4.4)
    assert client.sent[-1]["steer"] == pytest.approx(108 + 10 / 0.526, abs=0.05)


def test_build_info_clamps_negative_speed():
    info = build_info(1.0, 1, 1, 0, False, -0.02, 108, "대기 중")
    assert info["speed"] == 0.0                   # §4.3 — 음수 금지


def test_report_schema():
    r = build_report("accept", 1784000244.5)
    assert r == {"protocol_version": "2.3", "header": "report",
                 "timestamp": 1784000244.5, "report": "accept"}
