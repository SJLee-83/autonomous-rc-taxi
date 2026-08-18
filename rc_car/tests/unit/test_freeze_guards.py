"""2026-08-05 콘솔 동결 사고 대책 3종 검증.

① safe_logging: 콘솔 핸들러가 영원히 블록돼도 로깅 호출은 즉시 반환 (드롭 감수)
② SafetyWatchdog 하트비트: 매 tick tmpfs 기록 — 독립 guard의 감시 입력
③ guard 레지스터 산식: PCA9685 FULL_OFF 대상 (motor_channel+3·4·5)
"""
import logging
import tempfile
import threading
import time
import unittest
from pathlib import Path

from core.safe_logging import make_nonblocking
from safety.watchdog import SafetyWatchdog
from tools.guard import full_off_regs


class _BlockedHandler(logging.Handler):
    """콘솔 파이프 역압 모사 — emit이 영원히 블록.

    lock 없음(createLock 무효화): 실제 사고에선 락 보유가 문제지만, 테스트에선
    logging.shutdown()(atexit)이 이 락을 기다리며 pytest가 교착하는 것을 막는다.
    블록 자체는 리스너 스레드(데몬)만 잡아둔다.
    """

    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self._never = threading.Event()

    def createLock(self):
        self.lock = None

    def emit(self, record):
        self.entered.set()
        self._never.wait()          # 영원히


class TestNonblockingLogging(unittest.TestCase):
    def test_logging_never_blocks_caller(self):
        root = logging.getLogger("freeze_test_root")
        root.propagate = False
        root.setLevel(logging.INFO)
        blocked = _BlockedHandler()
        root.handlers = [blocked]
        qh = make_nonblocking(root, maxsize=50)

        t0 = time.monotonic()
        for i in range(5000):       # 큐(50)보다 훨씬 많이 — 드롭 강제
            root.info("tick %d", i)
        elapsed = time.monotonic() - t0

        self.assertLess(elapsed, 2.0)            # 호출측 무블록 (동결 사고 재현 방지)
        self.assertTrue(blocked.entered.wait(2)) # 리스너는 실제로 블록에 걸려 있음
        self.assertGreater(qh.dropped, 0)        # 유실 감수 방식 확인


class TestWatchdogHeartbeat(unittest.TestCase):
    def test_tick_writes_monotonic_heartbeat(self):
        hb = Path(tempfile.mkdtemp()) / "veh_heartbeat"
        wd = SafetyWatchdog(threading.Event(), supervisor=None, watched=[],
                            heartbeat_path=hb)
        wd.tick()
        age = time.monotonic() - float(hb.read_text())
        self.assertLess(abs(age), 0.5)
        SafetyWatchdog.clear_heartbeat(hb)       # 정상 종료 신호 = 파일 제거
        self.assertFalse(hb.exists())


class TestGuardRegs(unittest.TestCase):
    def test_full_off_registers_for_channel0(self):
        # vendor motor_controller: ch+3(IN2)·ch+4(IN1)·ch+5(PWM), LEDn_OFF_H=0x09+4n
        self.assertEqual(full_off_regs(0), [0x15, 0x19, 0x1D])


if __name__ == "__main__":
    unittest.main()
