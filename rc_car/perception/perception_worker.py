"""PerceptionWorker — 비전 최신값 pull 주기 실행 (계약 v0.3 §3, 목표 10Hz).

계약 v0.3: 카메라·캡처·추론 루프는 전부 vision 소유 — 임베디드는 latest()를
논블로킹 조회만 한다. 이 worker는 10Hz로 어댑터 관측(신선도 판정·폴백 카운트)을
갱신하는 미러다. 프레임 공급 경로는 v0.2 §4 폐기와 함께 소멸.

2026-08-06(§0-45)부터 소비자가 둘이다 — 둘 다 같은 게시 파일을 읽지만 쓰임이 다르다:
    SegAdapter  : 차선 중앙 오프셋 → 직진 중 미세 횡보정
    MarkAdapter : 노면표시 검출   → 회전 개시 시점 결정 (2차 트리거)
어느 한쪽이 없어도(None) 나머지는 그대로 돈다.
"""
import threading

from workers.periodic_worker import PeriodicWorker


class PerceptionWorker(PeriodicWorker):
    def __init__(self, stop_event: threading.Event, adapter,
                 rate_hz: float, on_error=None, mark_adapter=None):
        super().__init__("PerceptionWorker", 1.0 / rate_hz, stop_event, on_error)
        self._adapter = adapter
        self._marks = mark_adapter

    def tick(self) -> None:
        if self._adapter is not None:
            self._adapter.observe()   # v0.3 pull — 모델 latest() 조회·검증·보관
        if self._marks is not None:
            self._marks.observe()     # 노면표시 — 회전 트리거 (§0-45)
