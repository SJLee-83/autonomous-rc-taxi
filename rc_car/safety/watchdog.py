"""SafetyWatchdog — worker 생존 감시 (명세서 §24.1) + 프로세스 하트비트.

worker의 tick 기록이 일정 시간 없으면 시스템을 정지 래치한다.
pose 미수신 감시는 여기가 아니라 LocalizationService가 맡는다 — 유실은 정지 래치가
아니라 오류 정지·자체 복귀 절차(§7.2)로 흘러야 하기 때문이다. 이 watchdog은
그 LocalizationService 자체가 멈췄는지를 본다.

하트비트 (2026-08-05 콘솔 동결 사고 대책): 매 tick마다 tmpfs 파일에 monotonic을
기록한다. 프로세스 전체가 동결되면(이 스레드 포함) 기록이 멈추고, 독립 프로세스
tools/guard.py 가 이를 감지해 모터 레지스터를 직접 끈다 — 프로세스 내부 안전
장치(래치·finally)가 전부 무력화되는 '동결' 상황의 최후 방어선.
"""
import logging
import os
import tempfile
import threading
import time
from pathlib import Path

from workers.periodic_worker import PeriodicWorker

log = logging.getLogger("safety.watchdog")

STALL_FACTOR = 5.0     # interval의 5배 동안 tick이 없으면 멈춘 것으로 판정
MIN_STALL_SEC = 1.0

HEARTBEAT_PATH = (Path("/dev/shm/veh_heartbeat")
                  if Path("/dev/shm").is_dir()
                  else Path(tempfile.gettempdir()) / "veh_heartbeat")


class SafetyWatchdog(PeriodicWorker):
    def __init__(self, stop_event: threading.Event, supervisor,
                 watched: list[PeriodicWorker],
                 heartbeat_path: Path = HEARTBEAT_PATH):
        super().__init__("SafetyWatchdog", 0.05, stop_event)   # 20Hz
        self._supervisor = supervisor
        self._watched = watched
        self._hb_path = heartbeat_path

    def tick(self) -> None:
        now = time.monotonic()
        try:
            self._hb_path.write_text(f"{now:.3f}")   # guard.py 가 읽는다
        except OSError:
            pass                                      # 하트비트 실패로 감시를 멈추진 않는다
        for w in self._watched:
            if w.last_tick_monotonic == 0.0:
                continue        # 아직 첫 tick 전
            limit = max(w.interval_sec * STALL_FACTOR, MIN_STALL_SEC)
            if now - w.last_tick_monotonic > limit:
                log.error("[%s] %.1f초간 tick 없음 → 정지 래치", w.name,
                          now - w.last_tick_monotonic)
                self._supervisor.latch_stop(f"{w.name} 멈춤")

    @staticmethod
    def clear_heartbeat(path: Path = HEARTBEAT_PATH) -> None:
        """정상 종료 시 호출 — guard가 '동결'과 '종료'를 구분하게 한다."""
        try:
            os.remove(path)
        except OSError:
            pass
