"""main._install_signal_handlers — 시그널 즉사(finally 미실행 → PWM 래치) 방지 검증.

2026-07-31 주행 2: ssh 세션 절단(SIGHUP)이 프로세스를 finally 없이 죽이면
PCA9685가 마지막 PWM을 래치한 채 남는 사고 경로. 핸들러가 SystemExit로 변환해
main()의 finally(shutdown→force_stop)를 보장하는지 확인한다.
"""
import signal
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from main import _install_signal_handlers  # noqa: E402

_SIGS = [s for s in (getattr(signal, "SIGHUP", None), signal.SIGTERM) if s is not None]


@pytest.fixture
def restore_handlers():
    saved = {sig: signal.getsignal(sig) for sig in _SIGS}
    yield
    for sig, handler in saved.items():
        signal.signal(sig, handler)


def test_sigterm_becomes_system_exit(restore_handlers):
    _install_signal_handlers()
    handler = signal.getsignal(signal.SIGTERM)
    assert callable(handler)
    with pytest.raises(SystemExit) as exc:
        handler(int(signal.SIGTERM), None)
    assert exc.value.code == 128 + int(signal.SIGTERM)


@pytest.mark.skipif(not hasattr(signal, "SIGHUP"), reason="Windows에는 SIGHUP 없음")
def test_sighup_becomes_system_exit(restore_handlers):
    _install_signal_handlers()
    handler = signal.getsignal(signal.SIGHUP)
    assert callable(handler)
    with pytest.raises(SystemExit):
        handler(int(signal.SIGHUP), None)


def test_ignored_signal_stays_ignored(restore_handlers):
    # nohup 기동은 SIGHUP을 SIG_IGN으로 만든다 — 덮어쓰면 nohup 보호가 풀린다.
    # SIGHUP이 없는 플랫폼에서도 같은 로직을 SIGTERM으로 검증한다.
    sig = getattr(signal, "SIGHUP", signal.SIGTERM)
    signal.signal(sig, signal.SIG_IGN)
    _install_signal_handlers()
    assert signal.getsignal(sig) is signal.SIG_IGN
