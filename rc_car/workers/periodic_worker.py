"""PeriodicWorker 공통 구조 (명세서 §24.2·§24.3).

- stop_event 대기로 종료에 즉시 반응한다 (단순 sleep 금지).
- 최상위 예외는 잡아서 on_worker_error 콜백(→ SafetySupervisor)에 보고한다.
  예외를 조용히 무시하지 않는다.
"""
import logging
import threading
import time

log = logging.getLogger("worker")


class PeriodicWorker:
    def __init__(self, name: str, interval_sec: float, stop_event: threading.Event,
                 on_error: "Callable[[str, BaseException], None] | None" = None):
        self.name = name
        self.interval_sec = interval_sec
        self.stop_event = stop_event
        self.on_error = on_error
        self.last_tick_monotonic = 0.0     # SafetyWatchdog 감시용
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)

    # 하위 클래스가 구현
    def tick(self) -> None:
        raise NotImplementedError

    def setup(self) -> None:
        """스레드 시작 직후 1회. 필요 시 오버라이드."""

    def teardown(self) -> None:
        """종료 직전 1회. 필요 시 오버라이드."""

    # ---------- 생명주기 ----------

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float = 2.0) -> None:
        self._thread.join(timeout)
        if self._thread.is_alive():
            log.warning("[%s] join 시간 초과", self.name)

    def _run(self) -> None:
        log.info("[%s] 시작 (%.0f Hz)", self.name,
                 1.0 / self.interval_sec if self.interval_sec > 0 else 0)
        try:
            self.setup()
            while not self.stop_event.is_set():
                started = time.monotonic()
                self.tick()
                self.last_tick_monotonic = started
                elapsed = time.monotonic() - started
                self.stop_event.wait(max(0.0, self.interval_sec - elapsed))
        except BaseException as e:  # noqa: BLE001 — 최상위 방어 (명세서 §24.3)
            log.exception("[%s] worker 예외", self.name)
            if self.on_error:
                self.on_error(self.name, e)
        finally:
            self.teardown()
            log.info("[%s] 종료", self.name)
