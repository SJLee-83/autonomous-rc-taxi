"""⑦ 통합 시뮬 오케스트레이터 - Phase 1 최종 리허설.

4개 프로세스를 띄우고 시나리오를 구동한 뒤 관제 서버 events.jsonl로 검증한다:

    GPS 서버(원본, 무수정) ← 합성 카메라 ← [sim_world: 운동학 적분] ← UDP ← 차량(sim 드라이버)
         ↓ wss                                                             ↑
         └────────────── pose ──────────────→ 차량(rc_car 전체 스택) ── ws ─┴─ 관제 서버

시나리오 (protocol_2 §8 정상 + 오류):
    S1 정상 픽업+하차     - move→accept→complete→stop→(탑승 대기)→move→…→(대기)
    S2 GPS 유실 자체복귀   - 주행 중 카메라 mute → error → unmute → recovered → 완주
    S3 관제 끊김 resume    - 주행 중 kill → 재접속 error 반복 → resume → 완주
    S4 대기 중 오류        - 정지 상태 mute → error → unmute → recovered (주행 없음)
    S5 complete 재전송 중 오류 - stop 유실 주입 + 재전송 중 GPS 유실 → 복귀 후 재전송 재개

실행 (rc_car/ 에서, 일반 파이썬):
    python tools/integration_sim.py                # 전체
    python tools/integration_sim.py --scenarios s1 s2
"""
import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

RC_CAR = Path(__file__).resolve().parent.parent
CODE = RC_CAR.parent
GPS_DIR = Path(r"<GPS서버경로>")
GPS_PY = GPS_DIR / ".venv" / "Scripts" / "python.exe"
EVENTS = CODE / "control_server" / "events.jsonl"
LOG_DIR = RC_CAR / "tools" / "sim_logs"

START = (0.9, 2.61, 0.0)   # top_inner_eb_w 위, 동향
UDP_PORT = 9100


class ScenarioFail(Exception):
    pass


class Stack:
    """프로세스 4개 + 이벤트 tail + 시나리오 헬퍼."""

    def __init__(self):
        LOG_DIR.mkdir(exist_ok=True)
        self.procs: list[tuple[str, subprocess.Popen]] = []
        self.cursor = 0            # events.jsonl 파싱 커서 (신규 이벤트만)
        self.events: list[dict] = []
        self._events_offset = EVENTS.stat().st_size if EVENTS.exists() else 0

    # ---------- 프로세스 기동 ----------

    def start_all(self) -> None:
        self._spawn("gps", [str(GPS_PY), "-m", "gps_server.main"], cwd=GPS_DIR)
        self._wait_port(8000, 30, "GPS 서버")

        self.sim = self._spawn(
            "sim", [str(GPS_PY), str(RC_CAR / "tools" / "sim_world.py"),
                    "--x", str(START[0]), "--y", str(START[1]),
                    "--heading", str(START[2]), "--udp-port", str(UDP_PORT)],
            cwd=RC_CAR, stdin=True)
        self._wait_output("sim", "SIM READY", 40)

        self.ctl = self._spawn("ctl", [sys.executable, "server.py"],
                               cwd=CODE / "control_server", stdin=True)
        self._wait_output("ctl", "기동", 15)

        self._spawn("veh", [sys.executable, "main.py",
                            "--driver-mode", "sim", "--sim-port", str(UDP_PORT),
                            "--localization-url",
                            "wss://127.0.0.1:8000/ws/v1/localization",
                            "--control-url", "ws://127.0.0.1:8002/ws/vehicle",
                            "--run-seconds", "900"], cwd=RC_CAR)
        # 차량 접속 + 첫 pose 게이트 통과 → info가 흐르기 시작할 때까지
        self.wait(lambda e: e["dir"] == "rx-info", 30, "차량 info 수신")
        print("== 스택 기동 완료 ==", flush=True)

    def _spawn(self, name, cmd, cwd, stdin=False) -> subprocess.Popen:
        import os
        env = dict(os.environ, PYTHONIOENCODING="utf-8")   # 자식 출력 인코딩 통일
        out = open(LOG_DIR / f"{name}.log", "w", encoding="utf-8")
        p = subprocess.Popen(cmd, cwd=str(cwd), stdout=out, stderr=subprocess.STDOUT,
                             stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
                             text=True, encoding="utf-8", errors="replace", env=env)
        self.procs.append((name, p))
        return p

    def _wait_port(self, port, timeout, what) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=1):
                    return
            except OSError:
                time.sleep(0.5)
        raise ScenarioFail(f"{what} 포트 {port} 기동 실패")

    def _wait_output(self, name, needle, timeout) -> None:
        path = LOG_DIR / f"{name}.log"
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if path.exists() and needle in path.read_text(encoding="utf-8",
                                                          errors="replace"):
                return
            time.sleep(0.5)
        raise ScenarioFail(f"{name}: '{needle}' 출력 대기 실패 ({path} 확인)")

    # ---------- 이벤트 ----------

    def _pump(self) -> None:
        if not EVENTS.exists():
            return
        with open(EVENTS, encoding="utf-8") as f:
            f.seek(self._events_offset)
            chunk = f.read()
            self._events_offset = f.tell()
        for line in chunk.splitlines():
            try:
                self.events.append(json.loads(line))
            except json.JSONDecodeError:
                pass

    def wait(self, pred, timeout, desc):
        """커서 이후 신규 이벤트 중 pred 매치 대기 - 매치 지점으로 커서 전진 (순서 보장)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self._pump()
            for i in range(self.cursor, len(self.events)):
                if pred(self.events[i]):
                    self.cursor = i + 1
                    print(f"  OK {desc}", flush=True)   # cp949 콘솔 - 특수문자 금지
                    return self.events[i]
            time.sleep(0.25)
        raise ScenarioFail(f"{desc} - {timeout}초 내 미발생")

    def wait_report(self, name, timeout=20):
        return self.wait(lambda e: e["dir"] == "rx"
                         and e["msg"].get("report") == name, timeout, f"report {name}")

    def wait_tx(self, command, timeout=20):
        return self.wait(lambda e: e["dir"] == "tx"
                         and e["msg"].get("command") == command, timeout, f"tx {command}")

    def wait_state(self, state, timeout=20):
        return self.wait(lambda e: e["dir"] == "rx-info"
                         and e["msg"].get("state") == state, timeout, f"state {state}")

    def absent_since(self, idx, pred, desc) -> None:
        self._pump()
        for e in self.events[idx:]:
            if pred(e):
                raise ScenarioFail(f"{desc} - 발생하면 안 되는 이벤트")

    # ---------- 조작 ----------

    def ctl_cmd(self, line: str) -> None:
        print(f"  관제> {line}", flush=True)
        self.ctl.stdin.write(line + "\n")
        self.ctl.stdin.flush()

    def sim_cmd(self, line: str) -> None:
        print(f"  sim> {line}", flush=True)
        self.sim.stdin.write(line + "\n")
        self.sim.stdin.flush()

    def teardown(self) -> None:
        for name, p in reversed(self.procs):
            if p.poll() is None:
                p.terminate()
        time.sleep(2)
        for name, p in self.procs:
            if p.poll() is None:
                p.kill()


# ---------- 시나리오 ----------

def s1_normal_cycle(st: Stack) -> None:
    """정상 픽업 + 하차 - §8 전이 사슬 (§4.5 상태 흐름 포함)."""
    st.ctl_cmd("move 1.8 2.61")
    st.wait_report("accept", 10)
    st.wait_state("호출 응답 이동 중", 10)
    st.wait_report("complete", 90)
    st.wait_tx("stop", 10)
    st.wait_state("탑승 대기 중", 10)
    st.ctl_cmd("move 3.9 2.61")
    st.wait_report("accept", 10)
    st.wait_state("고객 탑승 이동 중", 10)
    st.wait_report("complete", 120)
    st.wait_tx("stop", 10)
    st.wait_state("대기 중", 10)


def s2_gps_loss_self_recovery(st: Stack) -> None:
    """주행 중 GPS 유실 → 오류 정지 + error → 복귀 시 recovered 후 자체 재개 (§7.2)."""
    st.ctl_cmd("move 4.0 1.88")
    st.wait_report("accept", 10)
    time.sleep(4)                      # 주행 궤도에 오른 뒤
    st.sim_cmd("mute")
    st.wait_report("error", 15)        # camera_stale(≤1s) + 0.6s 판정
    st.wait_state("오류 정지", 10)
    time.sleep(1.5)
    st.sim_cmd("unmute")
    st.wait_report("recovered", 15)
    st.wait_report("complete", 120)    # 같은 경로 이어서 완주 (§7.2 자체 복귀)
    st.wait_tx("stop", 10)


def s3_control_kill_resume(st: Stack) -> None:
    """주행 중 관제 절단 → 재접속 후 error 반복 → resume 수신 시 재개 (§7.3)."""
    st.ctl_cmd("move 1.5 1.88")
    st.wait_report("accept", 10)
    time.sleep(3)
    st.ctl_cmd("kill")
    st.wait_report("error", 15)        # 재접속 후 error 재전송 시작
    st.wait_tx("resume", 10)           # auto 정책이 resume 회신
    st.wait_report("complete", 120)
    st.wait_tx("stop", 10)


def s4_error_while_waiting(st: Stack) -> None:
    """대기 중 GPS 유실 - error 1회 + 복귀만 하고 주행하지 않는다 (§7.2 '복귀만')."""
    mark = st.cursor
    st.sim_cmd("mute")
    st.wait_report("error", 15)
    st.wait_state("오류 정지", 10)
    time.sleep(1.5)
    st.sim_cmd("unmute")
    st.wait_report("recovered", 15)
    # S3(하차 운행) 도착 후라 '대기 중' - 복귀만 하고 주행하지 않는다 (§4.5 kind 교대)
    st.wait_state("대기 중", 15)
    st.absent_since(mark, lambda e: e["dir"] == "rx"
                    and e["msg"].get("report") == "complete", "대기 중 complete")


def s5_error_during_complete_resend(st: Stack) -> None:
    """complete 재전송 중 GPS 유실 - 재전송 일시정지 → 복귀 후 재개 (§7.2·§5.2)."""
    st.ctl_cmd("mute stop")            # 서버가 stop을 유실시킨다 → complete 재전송 지속
    st.ctl_cmd("move 1.5 2.61")
    st.wait_report("accept", 10)
    st.wait_report("complete", 120)
    st.wait_report("complete", 10)     # 재전송 확인 (1초 주기, timestamp 고정 §5.2)
    st.sim_cmd("mute")
    st.wait_report("error", 15)
    time.sleep(1.5)
    st.sim_cmd("unmute")
    st.wait_report("recovered", 15)
    st.wait_report("complete", 15)     # 복귀 후 complete 재전송 재개
    st.ctl_cmd("unmute")
    st.ctl_cmd("stop")
    st.wait_state("탑승 대기 중", 15)   # 이번 운행은 픽업(§4.5 kind 교대) - stop 후 탑승 대기


def s6_dual_loss(st: Stack) -> None:
    """주행 중 GPS+관제 동시 유실 - 사유 집합이 전부 빌 때만 복귀한다 (§7.5)."""
    st.ctl_cmd("move 3.0 2.61")
    st.wait_report("accept", 10)
    time.sleep(3)
    st.sim_cmd("mute")                 # GPS 유실
    time.sleep(0.5)
    st.ctl_cmd("kill")                 # 관제 절단까지 - 동시 유실
    st.wait_report("error", 20)        # 재접속 후 error 재전송
    st.wait_tx("resume", 10)           # auto resume → 관제 사유 해소 (GPS 사유는 잔존)
    time.sleep(1.0)                    # 복귀 보류 상태 확인 창 (§7.5)
    st.wait_state("오류 정지", 5)      # 사유 하나 남아 있으므로 여전히 오류 정지
    st.sim_cmd("unmute")               # GPS 복귀 → 마지막 사유 해소
    st.wait_report("recovered", 15)
    st.wait_report("complete", 120)    # 직전 state 복귀 + 같은 경로 완주
    st.wait_tx("stop", 10)


SCENARIOS = {"s1": s1_normal_cycle, "s2": s2_gps_loss_self_recovery,
             "s3": s3_control_kill_resume, "s4": s4_error_while_waiting,
             "s5": s5_error_during_complete_resend, "s6": s6_dual_loss}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="*", default=list(SCENARIOS),
                    choices=list(SCENARIOS))
    args = ap.parse_args()

    st = Stack()
    results: dict[str, str] = {}
    try:
        st.start_all()
        for name in args.scenarios:
            print(f"\n== {name}: {SCENARIOS[name].__doc__.splitlines()[0]}", flush=True)
            try:
                SCENARIOS[name](st)
                results[name] = "PASS"
                print(f"== {name} PASS", flush=True)
            except ScenarioFail as e:
                results[name] = f"FAIL: {e}"
                print(f"== {name} FAIL: {e}", flush=True)
                break                  # 상태가 어긋나면 후속 시나리오 무의미
    except ScenarioFail as e:
        print(f"기동 실패: {e}", flush=True)
        return 2
    finally:
        st.teardown()

    print("\n===== 통합 시뮬 결과 =====", flush=True)
    for name in args.scenarios:
        print(f"  {name}: {results.get(name, 'SKIP')}", flush=True)
    return 0 if all(v == "PASS" for v in results.values()) and results else 1


if __name__ == "__main__":
    raise SystemExit(main())
