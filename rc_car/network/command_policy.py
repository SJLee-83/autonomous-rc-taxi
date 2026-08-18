"""CommandPolicy — 상태 전이의 단일 지점 (protocol_2 §4.5 전이표 · §6.4 매트릭스 · §7 오류/복구).

- 명령 수신(ControlClient 콜백)·연결 이벤트·GPS 이벤트·도착 통지가 전부 여기로 모인다.
- 오류는 사유(관제 끊김 / GPS 유실) 집합으로 관리한다 — §7.5 동시 유실은
  "모든 사유가 해소된 뒤에만 직전 state 복귀"로 자연스럽게 처리된다.
- GPS 이벤트(on_gps_lost/on_gps_recovered)는 단계 2 LocalizationClient가 호출한다.
"""
import logging
import threading

from core.enums import DRIVING_STATES, DrivingState, ServiceKind, to_service_state
from core.state_store import StateStore
from network.messages import parse_command
from network.report_manager import ReportManager
from safety.safety_supervisor import SafetySupervisor

log = logging.getLogger("network.policy")

REASON_CONTROL_LOST = "control_lost"   # §7.1 조건 3 — resume 수신으로만 해소
REASON_GPS_LOST = "gps_lost"           # §7.1 조건 1·2 — GPS 정상화로 자체 해소


class CommandPolicy:
    def __init__(self, store: StateStore, supervisor: SafetySupervisor,
                 reports: ReportManager, map_x_max: float, map_y_max: float):
        self._store = store
        self._supervisor = supervisor
        self._reports = reports
        self._map_x = map_x_max
        self._map_y = map_y_max
        self._lock = threading.RLock()
        self._reasons: set[str] = set()

    # ---------- 명령 수신 (네트워크 스레드) ----------

    def on_raw_message(self, raw: str) -> None:
        parsed = parse_command(raw, self._map_x, self._map_y)
        if parsed is None:
            return                     # 무시 + 로그는 parse_command가 수행 (§2.10)
        command, loc = parsed
        with self._lock:
            snap = self._store.snapshot()
            if command == "move":
                self._on_move(snap, loc)
            elif command == "stop":
                self._on_stop(snap)
            elif command == "resume":
                self._on_resume(snap)

    def _on_move(self, snap, loc: dict) -> None:
        if snap.driving_state != DrivingState.WAITING:
            # 주행 중·도착 후·오류 정지 전부 거부 — 응답 없이 로그만 (§6.4)
            log.warning("move 거부 (state=%s, §6.4)", self._service_state(snap))
            return
        # 픽업/하차 구분 = 직전 상태 (§4.5): 대기 중 → 픽업, 탑승 대기 중 → 하차 운행
        kind = (ServiceKind.CARRY if snap.service_kind == ServiceKind.PICKUP
                else ServiceKind.PICKUP)
        self._store.set_mission(loc["x"], loc["y"], kind)
        # PLANNING 진입 — 경로 계획·주행은 단계 5 ControlWorker가 이어받는다
        self._store.set_driving_state(DrivingState.PLANNING)
        self._reports.send_accept()
        log.info("move 수락 → %s, dest=(%.2f, %.2f)", kind.value, loc["x"], loc["y"])

    def _on_stop(self, snap) -> None:
        if snap.driving_state != DrivingState.ARRIVED:
            # stop 수락은 도착 후 complete 재전송 중뿐 — 주행 중 stop은 계약 위반 (§6.4)
            log.warning("stop 무시 (state=%s, §6.4)", self._service_state(snap))
            return
        self._reports.clear_complete()
        # §4.5 전이: 픽업 도착 확인 → 탑승 대기 중 / 하차 도착 확인 → 대기 중
        next_kind = ServiceKind.PICKUP if snap.service_kind == ServiceKind.PICKUP else None
        self._store.clear_mission(next_kind)
        self._store.set_driving_state(DrivingState.WAITING)
        log.info("stop 수락 → %s", self._service_state(self._store.snapshot()))

    def _on_resume(self, snap) -> None:
        if snap.driving_state != DrivingState.ERROR or REASON_CONTROL_LOST not in self._reasons:
            log.warning("resume 무시 (state=%s, §6.4)", self._service_state(snap))
            return
        log.info("resume 수락 — 관제 끊김 사유 해소")
        self._clear_reason(REASON_CONTROL_LOST)

    # ---------- 관제 연결 이벤트 (ControlClient 콜백) ----------

    def on_control_disconnected(self) -> None:
        """§7.1 조건 3 — 즉시 정지, 직전 state 보관."""
        with self._lock:
            self._enter_error(REASON_CONTROL_LOST)

    def on_control_connected(self) -> None:
        with self._lock:
            if REASON_CONTROL_LOST in self._reasons:
                # 재연결 성공 → resume 수신까지 error 반복 (§7.3)
                self._reports.start_error_resend()

    # ---------- GPS 이벤트 (단계 2 LocalizationClient가 호출) ----------

    def on_gps_lost(self) -> None:
        """§7.1 조건 1·2 — 즉시 정지. 관제가 살아 있으면 error 1회 (§5.2)."""
        with self._lock:
            if self._enter_error(REASON_GPS_LOST):
                self._reports.send_error_once()

    def on_gps_recovered(self) -> None:
        """위치 수신 정상화 — recovered 보고 후 자체 복귀 (§7.2). resume 불필요."""
        with self._lock:
            if REASON_GPS_LOST not in self._reasons:
                return
            self._reports.send_recovered()
            self._clear_reason(REASON_GPS_LOST)

    # ---------- 도착 통지 (주행 로직이 호출 — 단계 5) ----------

    def notify_arrived(self) -> None:
        """도착 → 스스로 정지 + complete 재전송 시작 (§8). stop 수신으로 전이가 완성된다."""
        with self._lock:
            snap = self._store.snapshot()
            if snap.driving_state not in DRIVING_STATES:
                log.warning("notify_arrived 무시 (state=%s)", snap.driving_state.value)
                return
            self._store.set_driving_state(DrivingState.ARRIVED)
            self._supervisor.force_stop()
            self._reports.mark_complete()
            log.info("도착 — complete 재전송 시작 (stop 대기)")

    # ---------- 오류 진입 · 복귀 ----------

    def _enter_error(self, reason: str) -> bool:
        """오류 사유 추가. 새 사유였으면 True. ERROR 최초 진입 시 직전 state 보관 + 즉시 정지."""
        newly = reason not in self._reasons
        self._reasons.add(reason)
        snap = self._store.snapshot()
        if snap.driving_state != DrivingState.ERROR:
            prev = self._store.set_driving_state(DrivingState.ERROR)
            self._supervisor.force_stop()   # 50Hz 제어 루프를 기다리지 않는 즉시 정지
            log.warning("오류 정지 진입 (사유=%s, 직전=%s)", reason, prev.value)
        return newly

    def _clear_reason(self, reason: str) -> None:
        self._reasons.discard(reason)
        if reason == REASON_CONTROL_LOST:
            self._reports.stop_error_resend()
        if self._reasons:
            log.info("사유 %s 해소 — 잔여 %s, 복귀 보류 (§7.5)", reason, self._reasons)
            return
        if self._store.snapshot().driving_state == DrivingState.ERROR:
            target = self._store.recover_previous_state()
            log.info("모든 사유 해소 — 직전 state %s 복귀 (§7.2)", target.value)
            # 직전이 이동 중이었으면 보관된 ActiveMission으로 주행이 재개된다
            # (SafetySupervisor 차단이 풀리고 제어 worker가 이어받는다 — 단계 5)
            # 직전이 ARRIVED였으면 ReportManager tick이 complete 재전송을 재개한다

    def _service_state(self, snap) -> str:
        return to_service_state(snap.driving_state, snap.service_kind)
