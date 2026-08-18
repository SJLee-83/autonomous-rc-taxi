"""safe_logging — 로깅 비블로킹화 (2026-08-05 콘솔 동결 사고 대책).

사고: WiFi 순단 → 사용자 ssh 터미널 동결 → tee 파이프 역압 → 콘솔 핸들러의
write()가 블록 → logging 락을 쥔 채 정지 → 로그를 찍는 모든 스레드(제어 루프·
안전 워치독 포함)가 연쇄 동결 → 모터가 마지막 명령에 래치된 채 주행.

대책: 모든 레코드를 유한 큐에 넣고(가득 차면 조용히 버림), 실제 출력은 데몬
리스너 스레드가 전담한다. 콘솔이 어떻게 막혀도 블록되는 것은 리스너 하나뿐이고,
제어·안전 스레드는 절대 로깅으로 멈추지 않는다.
"""
from __future__ import annotations

import logging
import queue
import threading
from logging.handlers import QueueHandler, QueueListener


class DropQueueHandler(QueueHandler):
    """큐가 가득 차면 레코드를 버린다 — handleError(stderr 출력)로도 안 간다."""

    def __init__(self, q: queue.Queue):
        super().__init__(q)
        self.dropped = 0

    def enqueue(self, record) -> None:
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1     # 출력 정체 중 — 유실 감수 (§동결 대책)


class _DaemonQueueListener(QueueListener):
    """리스너 스레드를 데몬으로 — 콘솔이 영원히 막혀도 프로세스 종료를 안 막는다."""

    def start(self) -> None:
        self._thread = t = threading.Thread(target=self._monitor, daemon=True)
        t.start()


def make_nonblocking(root: logging.Logger | None = None,
                     maxsize: int = 2000) -> DropQueueHandler:
    """root의 기존 핸들러들을 큐 뒤로 옮긴다. basicConfig 직후 1회 호출."""
    root = root or logging.getLogger()
    q: queue.Queue = queue.Queue(maxsize=maxsize)
    listener = _DaemonQueueListener(q, *root.handlers,
                                    respect_handler_level=True)
    handler = DropQueueHandler(q)
    root.handlers = [handler]
    listener.start()
    return handler
