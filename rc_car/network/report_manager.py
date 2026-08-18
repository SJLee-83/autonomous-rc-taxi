"""ReportManager — report 4종 송신 + complete/error 재전송 (protocol_2 §5).

재전송 규칙 (§5.2):
- complete: stop 수신까지 1초 간격. 오류 정지 중에는 멈추고 복귀 시 재개한다 (§7.2).
- error(관제 끊김): 재연결 후 resume 수신까지 1초 간격 (§7.3).
- error(GPS 유실)·accept·recovered: 1회.
- 재전송 timestamp는 최초 발생 시각을 유지한다 — (report, timestamp)가 사건 식별자.
"""
import logging
import threading
import time

from core.enums import DrivingState
from core.state_store import StateStore
from network.messages import build_report
from workers.periodic_worker import PeriodicWorker

log = logging.getLogger("network.report")


class ReportManager(PeriodicWorker):
    def __init__(self, stop_event: threading.Event, client, store: StateStore,
                 cfg: dict, on_error=None):
        # complete·error 재전송 주기는 config상 동일(1초) — 짧은 쪽으로 tick 한다
        interval = min(cfg["complete_resend_interval_sec"], cfg["error_resend_interval_sec"])
        super().__init__("ReportManager", interval, stop_event, on_error)
        self._client = client
        self._store = store
        self._lock = threading.Lock()
        self._complete_ts: float | None = None   # 도착 최초 시각 (§5.2 고정)
        self._error_resend_ts: float | None = None

    # ---------- 1회성 보고 ----------

    def send_accept(self) -> None:
        """move 수락 시에만 (§5.2)."""
        self._send("accept", time.time())

    def send_recovered(self) -> None:
        """GPS 위치 수신 정상화 (§7.2)."""
        self._send("recovered", time.time())

    def send_error_once(self) -> None:
        """GPS 유실 error — 관제 연결이 살아 있으므로 1회로 충분 (§5.2)."""
        self._send("error", time.time())

    # ---------- complete 재전송 ----------

    def mark_complete(self) -> None:
        """도착 시 호출. 즉시 1회 송신하고 stop 수신까지 재전송한다."""
        with self._lock:
            if self._complete_ts is None:
                self._complete_ts = time.time()
            ts = self._complete_ts
        self._send("complete", ts)

    def clear_complete(self) -> None:
        """stop 수락 — 재전송 중단 (§5.2)."""
        with self._lock:
            self._complete_ts = None

    # ---------- error 재전송 (관제 끊김 전용) ----------

    def start_error_resend(self) -> None:
        """관제 재연결 직후 호출. 즉시 1회 송신하고 resume 수신까지 재전송한다 (§7.3)."""
        with self._lock:
            if self._error_resend_ts is None:
                self._error_resend_ts = time.time()
            ts = self._error_resend_ts
        self._send("error", ts)

    def stop_error_resend(self) -> None:
        """resume 수락 (§7.3)."""
        with self._lock:
            self._error_resend_ts = None

    # ---------- 재전송 tick ----------

    def tick(self) -> None:
        with self._lock:
            complete_ts = self._complete_ts
            error_ts = self._error_resend_ts
        state = self._store.snapshot().driving_state
        # complete는 ARRIVED에서만 재전송 — 오류 정지 중에는 멈추고 복귀 시 재개 (§7.2)
        if complete_ts is not None and state == DrivingState.ARRIVED:
            self._send("complete", complete_ts, resend=True)
        if error_ts is not None and state == DrivingState.ERROR:
            self._send("error", error_ts, resend=True)

    def _send(self, kind: str, ts: float, resend: bool = False) -> None:
        sent = self._client.send_json(build_report(kind, ts))
        if not resend:
            log.info("report:%s 송신%s", kind, "" if sent else " (미연결 — 유실)")
