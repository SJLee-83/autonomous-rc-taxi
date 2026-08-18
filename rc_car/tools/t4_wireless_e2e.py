# -*- coding: utf-8 -*-
"""T4 실차 무선 E2E 오케스트레이터 (PC측).

PC:    GPS 서버(:8000, TLS) + Java 백엔드(:8080)  [이미 기동돼 있거나 이 스크립트가 재기동]
Jetson: sim_world(합성 카메라+운동학) + rc_car 차량(sim 드라이버)

시나리오
  j1: 정상 픽업+하차 (id+좌표 혼합, board, 옛 데드락 좌표 하차) - 0-22 무선판
  j2: 주행 중 GPS 유실 - error/오류정지 - 복귀 - 자체재개 완주 (규약 7.2)
  j4: 대기 중 GPS 유실 - error/recovered만, 주행 없음 (규약 7.2)
  j3: 주행 중 관제(Java) 절단 - 정지 - 재기동 - error 재전송 - resume - 완주 (규약 7.3)
  j6: GPS+관제 동시 유실 - 사유 전부 해소 때만 복귀 (규약 7.5)
"""
import json
import subprocess
import sys
import time
import urllib.request
import urllib.error

PC_IP = "<PC-IP>"
JET = "<user>@<jetson-ip>"
API = "http://127.0.0.1:8080"
SP = r"<작업경로>"
JAVA = SP + r"\jdk-25.0.4+7\bin\java.exe"
BACKEND = SP + r"\backend"
JAR = BACKEND + r"\target\control-0.0.1-SNAPSHOT.jar"
JAVA_LOG = SP + r"\java_t4.log"

VEH_CMD = ("cd /home/<user>/rc_car; nohup python3 main.py --driver-mode sim"
           " --sim-port 9100"
           " --localization-url wss://%s:8000/ws/v1/localization"
           " --control-url ws://%s:8080/ws/vehicle"
           " --run-seconds 3600 >/tmp/veh.log 2>&1 </dev/null & echo $!" % (PC_IP, PC_IP))
SIM_CMD = ("rm -f /tmp/simctl; mkfifo /tmp/simctl; "
           "nohup bash -c 'tail -f /tmp/simctl | python3 /home/<user>/rc_car/tools/sim_world.py"
           " --url wss://%s:8000/ws/v1/camera --x 0.9 --y 2.61 --heading 0.0"
           " --udp-port 9100' >/tmp/sim.log 2>&1 </dev/null & echo SIM_SPAWNED" % PC_IP)


class Fail(Exception):
    pass


def say(msg):
    print(msg, flush=True)


def ssh(cmd, timeout=30):
    r = subprocess.run(["ssh", "-o", "BatchMode=yes", JET, cmd],
                       capture_output=True, text=True, timeout=timeout,
                       encoding="utf-8", errors="replace",
                       stdin=subprocess.DEVNULL)
    return r.stdout.strip()


def api(method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(API + path, method=method, data=data,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            raw = r.read().decode()
            return r.status, (json.loads(raw) if raw else None)
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()
    except Exception as e:
        return -1, str(e)


def ride_state():
    s, b = api("GET", "/api/rides/current")
    if s == 200 and isinstance(b, dict):
        return b.get("state")
    return None


def wait_ride(target, timeout, desc=""):
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < timeout:
        st = ride_state()
        if st != last:
            say("    ride: %s" % st)
            last = st
        if st == target:
            say("  OK ride=%s %s (%.0fs)" % (target, desc, time.monotonic() - t0))
            return
        time.sleep(1)
    raise Fail("ride=%s not reached in %ds (last=%s) %s" % (target, timeout, last, desc))


def veh_log(tail=5):
    return ssh("tail -n %d /tmp/veh.log" % tail)


def wait_veh(needle, timeout, desc="", fresh_after=None):
    """veh.log 에 needle 등장 대기. fresh_after: 그 마크 이후 로그만 본다."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if fresh_after:
            out = ssh("awk '/%s/{f=1} f' /tmp/veh.log | grep -c '%s' || true"
                      % (fresh_after, needle))
            hit = out and out.splitlines()[-1].strip() not in ("", "0")
        else:
            out = ssh("grep -c '%s' /tmp/veh.log || true" % needle)
            hit = out and out.splitlines()[-1].strip() not in ("", "0")
        if hit:
            say("  OK veh '%s' %s (%.0fs)" % (needle, desc, time.monotonic() - t0))
            return
        time.sleep(1.5)
    raise Fail("veh '%s' not seen in %ds %s || tail: %s"
               % (needle, timeout, desc, veh_log()))


def mark():
    """veh.log 에 시간 마크 없음 - 대신 현재 줄 수를 마크로 쓴다."""
    out = ssh("wc -l < /tmp/veh.log")
    return int(out.splitlines()[-1].strip() or 0)


def wait_veh_after(lineno, needle, timeout, desc=""):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        out = ssh("tail -n +%d /tmp/veh.log | grep -c '%s' || true" % (lineno + 1, needle))
        if out and out.splitlines()[-1].strip() not in ("", "0"):
            say("  OK veh '%s' %s (%.0fs)" % (needle, desc, time.monotonic() - t0))
            return
        time.sleep(1.5)
    raise Fail("veh '%s' not seen after line %d in %ds %s || tail: %s"
               % (needle, lineno, timeout, desc, veh_log()))


def absent_veh_after(lineno, needle, desc=""):
    out = ssh("tail -n +%d /tmp/veh.log | grep -c '%s' || true" % (lineno + 1, needle))
    if out and out.splitlines()[-1].strip() not in ("", "0"):
        raise Fail("veh '%s' happened but must not %s" % (needle, desc))
    say("  OK absent '%s' %s" % (needle, desc))


def sim_cmd(c):
    say("  sim> %s" % c)
    ssh("echo %s > /tmp/simctl" % c)


# ---------- 프로세스 제어 ----------

def stop_vehicle():
    ssh("pkill -f 'main.py --driver-mode sim' || true")
    time.sleep(2)


def start_vehicle():
    stop_vehicle()
    ssh("rm -f /tmp/veh.log")
    pid = ssh(VEH_CMD)
    say("  vehicle pid %s" % pid)
    wait_veh("stale=False", 40, "(GPS 체인 가동)")
    wait_veh("대기 중", 20, "(WAITING 진입)")


def start_sim():
    ssh("pkill -f sim_world.py || true; pkill -f 'tail -f /tmp/simctl' || true")
    time.sleep(1)
    ssh("rm -f /tmp/sim.log")
    say("  " + ssh(SIM_CMD))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 40:
        if "SIM READY" in ssh("grep -a 'SIM READY' /tmp/sim.log || true"):
            say("  OK sim ready")
            return
        time.sleep(1.5)
    raise Fail("sim_world not ready: " + ssh("tail -n 8 /tmp/sim.log"))


def java_pid():
    r = subprocess.run(["netstat", "-ano"], capture_output=True, text=True)
    for ln in r.stdout.splitlines():
        if ":8080" in ln and "LISTENING" in ln:
            return int(ln.split()[-1])
    return None


def kill_java():
    pid = java_pid()
    if pid is None:
        raise Fail("no java on :8080 to kill")
    say("  kill java pid %d" % pid)
    subprocess.run(["taskkill", "/F", "/PID", str(pid)], capture_output=True)
    time.sleep(2)


def start_java():
    if java_pid() is not None:
        say("  java already up")
        return
    log = open(JAVA_LOG, "a", encoding="utf-8")
    DETACHED = 0x00000008
    subprocess.Popen([JAVA, "-jar", JAR], cwd=BACKEND, stdout=log,
                     stderr=subprocess.STDOUT, creationflags=DETACHED)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 60:
        s, _ = api("GET", "/api/places")
        if s == 200:
            say("  OK java up (%.0fs)" % (time.monotonic() - t0))
            return
        time.sleep(1.5)
    raise Fail("java did not come up")


# ---------- 시나리오 ----------

def call_ride(origin, dest, desc):
    s, b = api("POST", "/api/rides", {"origin": origin, "destination": dest})
    if s != 200:
        raise Fail("POST /api/rides %s -> %s %s" % (desc, s, b))
    say("  OK call %s: state=%s pickup=(%.3f,%.3f,%s) dest=(%.3f,%.3f,%s)"
        % (desc, b["state"],
           b["pickup"]["x"], b["pickup"]["y"], b["pickup"]["lane"],
           b["destination"]["x"], b["destination"]["y"], b["destination"]["lane"]))
    return b


def j1():
    say("== J1 정상 픽업+하차 (id+좌표 혼합, 무선)")
    b = call_ride({"id": "home"}, {"x": 1.10, "y": 0.90}, "home->(1.10,0.90)")
    dx, dy = b["destination"]["x"], b["destination"]["y"]
    if abs(dx - 1.10) > 0.05 or abs(dy - 1.124) > 0.05:
        say("  주의: 목적지 정차점 (%.3f,%.3f) - 예상(1.10,1.124)과 차이" % (dx, dy))
    else:
        say("  OK 정차점 = 옛 데드락 구간 좌표 재현 (%.3f,%.3f)" % (dx, dy))
    wait_ride("TO_PICKUP", 20)
    wait_veh("호출 응답 이동 중", 25)
    wait_ride("BOARDING", 240, "(픽업 도착)")
    wait_veh("탑승 대기 중", 20, "(8.1 전제 상태)")
    s, _ = api("POST", "/api/rides/current/board")
    if s not in (200, 204):
        raise Fail("board -> %s" % s)
    say("  OK board")
    wait_ride("TO_DEST", 20, "(8.1: 차량 상태 확인 후 출발)")
    wait_veh("고객 탑승 이동 중", 25)
    wait_ride("COMPLETED", 300, "(옛 데드락 좌표 하차)")
    wait_veh("대기 중", 30, "(운행 종료 복귀)")
    say("== J1 PASS")


def j2():
    say("== J2 주행 중 GPS 유실 - 자체 복귀 (7.2)")
    m = mark()
    call_ride({"id": "market"}, {"id": "clinic"}, "market->clinic")
    wait_ride("TO_PICKUP", 20)
    time.sleep(6)
    sim_cmd("mute")
    wait_veh_after(m, "오류 정지", 25)
    time.sleep(1.5)
    sim_cmd("unmute")
    wait_veh_after(m, "호출 응답 이동 중", 40, "(자체 재개)")
    wait_ride("BOARDING", 300, "(유실 후 완주)")
    api("POST", "/api/rides/current/board")
    wait_ride("COMPLETED", 300)
    say("== J2 PASS")


def j4():
    say("== J4 대기 중 GPS 유실 - 복귀만 (7.2)")
    if ride_state() not in (None, "COMPLETED"):
        raise Fail("j4 는 대기 상태에서 시작해야 함: ride=%s" % ride_state())
    m = mark()
    sim_cmd("mute")
    wait_veh_after(m, "오류 정지", 25)
    time.sleep(1.5)
    sim_cmd("unmute")
    wait_veh_after(m, "대기 중", 30, "(복귀만)")
    absent_veh_after(m, "이동 중", "(대기 중 주행 금지)")
    say("== J4 PASS")


def j3():
    say("== J3 주행 중 관제 절단 - 재기동 - resume (7.3)")
    m = mark()
    call_ride({"id": "office"}, {"id": "bank"}, "office->bank")
    wait_ride("TO_PICKUP", 20)
    time.sleep(5)
    kill_java()
    wait_veh_after(m, "오류 정지", 30, "(관제 두절 감지)")
    time.sleep(2)
    start_java()
    time.sleep(4)                      # 차량 재접속 + error 재전송 창
    s, b = api("POST", "/api/vehicle/resume")
    say("  resume -> %s" % s)
    if s not in (200, 204):
        raise Fail("resume -> %s %s" % (s, b))
    wait_veh_after(m, "호출 응답 이동 중", 40, "(resume 후 재개)")
    wait_veh_after(m, "탑승 대기 중", 300, "(픽업 완주 - 백엔드는 라이드 소실이 정상)")
    say("  참고: 재기동한 백엔드 ride=%s (소실 문서화됨 HANDOFF 5)" % ride_state())
    say("== J3 PASS")


def j6():
    say("== J6 GPS+관제 동시 유실 (7.5)")
    m = mark()
    call_ride({"id": "center"}, {"id": "home"}, "center->home")
    wait_ride("TO_PICKUP", 20)
    time.sleep(5)
    sim_cmd("mute")
    time.sleep(0.7)
    kill_java()
    wait_veh_after(m, "오류 정지", 30, "(동시 유실)")
    time.sleep(2)
    start_java()
    time.sleep(4)
    s, _ = api("POST", "/api/vehicle/resume")
    say("  resume -> %s (관제 사유만 해소)" % s)
    time.sleep(3)
    m2 = mark()
    absent_veh_after(m2 - 1, "이동 중", "(GPS 사유 잔존 - 아직 주행 금지)")
    sim_cmd("unmute")
    wait_veh_after(m, "호출 응답 이동 중", 40, "(마지막 사유 해소 후 재개)")
    wait_veh_after(m, "탑승 대기 중", 300, "(완주)")
    say("== J6 PASS")


# ---------- 엔트리 ----------

def setup():
    say("== SETUP: java/gps 확인 + jetson sim/vehicle 기동")
    s, _ = api("GET", "/api/places")
    if s != 200:
        start_java()
    start_sim()
    start_vehicle()
    say("== SETUP DONE")


def teardown():
    stop_vehicle()
    ssh("pkill -f sim_world.py || true; pkill -f 'tail -f /tmp/simctl' || true; rm -f /tmp/simctl")
    say("== TEARDOWN DONE (java/gps 서버는 유지)")


def fetch_logs():
    say(ssh("tail -n 40 /tmp/veh.log"))


CMDS = {"setup": setup, "teardown": teardown, "logs": fetch_logs,
        "j1": j1, "j2": j2, "j3": j3, "j4": j4, "j6": j6,
        "restart-veh": start_vehicle}

if __name__ == "__main__":
    try:
        for name in sys.argv[1:]:
            CMDS[name]()
        sys.exit(0)
    except Fail as e:
        say("FAIL: %s" % e)
        sys.exit(1)
