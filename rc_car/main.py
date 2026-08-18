"""진입점 — 조립만 한다. 비즈니스 로직 금지 (명세서 SW 3원칙 1).

실행:
    python main.py                  # Ctrl+C까지 실행
    python main.py --run-seconds 3  # N초 후 자동 종료 (자동 테스트용)

로컬 검증 오버라이드 (config 파일은 건드리지 않는다 — 워크로그 교훈):
    python main.py --driver-mode sim --seg-mode off --sim-port 9100 \
        --localization-url wss://127.0.0.1:8000/ws/v1/localization
(config가 seg_mode: real 이어도 --seg-mode off 로 PC 로컬 기동이 된다 —
 real은 vision 게시 파일이 전제라 없는 기기에서는 기동이 거부된다)
"""
import argparse
import signal
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from app.runtime import Runtime  # noqa: E402


def _install_signal_handlers() -> None:
    """SIGHUP(ssh 세션 절단)·SIGTERM(외부 kill)을 SystemExit로 변환한다.

    기본 동작은 finally 없이 즉사 → PCA9685가 마지막 PWM을 래치한 채 남는다
    (2026-07-31 주행 2 사고 경로 후보). SystemExit로 바꾸면 main()의 finally가
    돌아 shutdown()→force_stop()이 보장된다.

    - nohup 기동 시 SIGHUP은 SIG_IGN 상태다 — 덮어쓰면 nohup 보호가 풀리므로 유지한다
    - Windows(개발 PC)에는 SIGHUP이 없다 — 있는 시그널만 설치한다
    """
    def _terminate(signum: int, _frame) -> None:
        raise SystemExit(128 + signum)

    for name in ("SIGHUP", "SIGTERM"):
        sig = getattr(signal, name, None)
        if sig is None:
            continue
        try:
            if signal.getsignal(sig) is signal.SIG_IGN:
                continue
            signal.signal(sig, _terminate)
        except (ValueError, OSError):
            pass  # 메인 스레드가 아니면 설치 불가 — 기존 동작 유지


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-seconds", type=float, default=None,
                    help="지정 시간 후 자동 종료 (미지정 시 Ctrl+C까지)")
    ap.add_argument("--driver-mode", choices=("mock", "real", "sim"), default=None,
                    help="config runtime.driver_mode 오버라이드 (sim은 통합 시뮬 전용)")
    ap.add_argument("--seg-mode", choices=("off", "mock", "real"), default=None,
                    help="config perception.seg_mode 오버라이드 "
                         "(vision 게시 파일이 없는 PC에서 real 기동 거부를 피할 때)")
    ap.add_argument("--sim-port", type=int, default=9100,
                    help="driver-mode sim의 UDP 명령 포트 (tools/sim_world.py)")
    ap.add_argument("--localization-url", default=None,
                    help="위치 서버 URL 오버라이드 (로컬 GPS 서버 검증용)")
    ap.add_argument("--control-url", default=None,
                    help="관제 서버 URL 오버라이드")
    args = ap.parse_args()

    _install_signal_handlers()
    overrides = {"driver_mode": args.driver_mode, "seg_mode": args.seg_mode,
                 "sim_port": args.sim_port,
                 "localization_url": args.localization_url,
                 "control_url": args.control_url}
    runtime = Runtime(overrides=overrides)
    try:
        runtime.initialize()
        runtime.start()
        runtime.wait(args.run_seconds)
    finally:
        runtime.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
