"""sim_world — 통합 시뮬의 "세상" (⑦): 운동학 적분 + 합성 카메라 되먹임.

차량(driver_mode=sim)이 UDP로 보내는 제어 명령(throttle·바퀴각)을 받아 자전거 모델로
pose를 적분하고, 그 위치에 ArUco 마커를 그린 프레임을 GPS 서버 `/ws/v1/camera`로
push한다. 차량은 자기 명령의 결과를 **실제 GPS 경로(카메라→서버→wss)** 로 돌려받는다.

    차량 SafetySupervisor → UDP → [여기: 적분+렌더] → GPS 서버 → wss → 차량 Localization

실행 (cv2 필요 — GPS venv 파이썬):
    <GPS서버경로>\\.venv\\Scripts\\python.exe tools/sim_world.py \\
        --x 0.9 --y 2.61 --heading 0

stdin 명령 (오케스트레이터가 파이프로 구동):
    mute    카메라 중단 (GPS 유실 주입 — 적분은 계속된다)
    unmute  카메라 재개
    pos     현재 상태 출력
    quit    종료

운동학 부호: 바퀴각 +(우) → heading 감소 (§2.7 반시계+). 스로틀→속도는 선형 가정
(v = throttle × max_speed) — B3 실측 전의 잠정 모델이다.
"""
import argparse
import asyncio
import json
import math
import socket
import ssl
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fake_camera import MARKER_M, SPACE_H_M, SPACE_W_M, draw, send_frame  # noqa: E402

VEH_MARKER = 6   # 차량 마커 ID — network.yaml marker_id와 일치해야 한다
                 # (2026-07-31: 실물 서버가 큐브 4면을 융합해 6번으로 차량 중심을 보고 —
                 #  시뮬도 동일하게 6번 단일 마커로 차량을 표현한다)


class World:
    def __init__(self, args):
        self.x, self.y, self.heading = args.x, args.y, args.heading
        self.throttle, self.wheel = 0.0, 0.0
        self.v = 0.0
        self.max_speed = args.max_speed
        self.wheelbase = args.wheelbase
        self.muted = False
        self.last_t = time.monotonic()
        # 아핀 보정 (자가 캘리브레이션): 서버 보고 좌표 rep = R·q + t (q = 그린 위치).
        # 원하는 보고 좌표 p로 만들려면 q = R⁻¹(p − t) 에 그린다. 기본은 항등.
        self.r_inv = [[1.0, 0.0], [0.0, 1.0]]
        self.t = [0.0, 0.0]

    def draw_pos(self) -> tuple:
        """현재 진짜 pose가 서버에서 그대로 보고되도록 보정된 '그릴 위치'."""
        px, py = self.x - self.t[0], self.y - self.t[1]
        return (self.r_inv[0][0] * px + self.r_inv[0][1] * py,
                self.r_inv[1][0] * px + self.r_inv[1][1] * py)

    def apply_cmd(self, msg: dict) -> None:
        self.throttle = float(msg.get("throttle", 0.0))
        self.wheel = float(msg.get("wheel_deg", 0.0))

    def step(self) -> None:
        now = time.monotonic()
        dt = min(now - self.last_t, 0.2)      # 프레임 지연 시 폭주 방지
        self.last_t = now
        self.v = max(0.0, min(self.throttle, 1.0)) * self.max_speed
        self.heading -= math.degrees(
            self.v / self.wheelbase * math.tan(math.radians(self.wheel)) * dt)
        self.heading %= 360.0
        self.x += self.v * math.cos(math.radians(self.heading)) * dt
        self.y += self.v * math.sin(math.radians(self.heading)) * dt
        lim = MARKER_M * 1.5                  # 마커가 프레임 밖으로 나가지 않게
        self.x = max(lim, min(SPACE_W_M - lim, self.x))
        self.y = max(lim, min(SPACE_H_M - lim, self.y))

    def status(self) -> str:
        return (f"SIM pos=({self.x:.3f}, {self.y:.3f}) heading={self.heading:.1f} "
                f"v={self.v:.3f} throttle={self.throttle:.2f} wheel={self.wheel:+.1f} "
                f"muted={self.muted}")


async def calibrate(world: World, ws, loc_url: str, ssl_ctx) -> None:
    """합성 카메라의 계통 좌표 왜곡(수~십 cm)을 역산해 아핀 보정을 세운다.

    격자점에 마커를 그려 넣고 서버가 보고한 좌표를 읽어 rep = R·q + t 를 최소자승
    적합 → 이후 draw_pos()가 R⁻¹(p−t)에 그리므로 보고 좌표 ≈ 진짜 pose 가 된다.
    (⑦에서 실측: 보정 없이는 y −13cm 왜곡으로 차선 매칭이 블록 경계에 걸린다)
    """
    import numpy as np
    from websockets.asyncio.client import connect

    grid = [(1.0, 0.7), (2.5, 0.7), (4.0, 0.7),
            (1.0, 2.3), (2.5, 2.3), (4.0, 2.3)]
    drawn, reported = [], []
    async with connect(loc_url, ssl=ssl_ctx) as loc:
        await loc.send(json.dumps({"type": "subscribe", "marker_id": VEH_MARKER,
                                   "interval_ms": 100}))
        fid = 50
        for q in grid:
            frame = draw({VEH_MARKER: q}, rot_deg={VEH_MARKER: 90.0})   # heading 0
            last = None
            for _ in range(10):
                await send_frame(ws, frame, fid)
                fid += 1
                try:
                    while True:
                        msg = json.loads(await asyncio.wait_for(loc.recv(), timeout=0.05))
                        if msg.get("found"):
                            last = msg
                except asyncio.TimeoutError:
                    pass
                await asyncio.sleep(0.08)
            if last is None:
                # 무보정 진행은 이후 주행 실패를 혼란스럽게 만든다 - 즉시 실패가 낫다
                print(f"CAL: {q} 검출 실패 - 기동 중단", flush=True)
                raise SystemExit(1)
            pos = last["position"]
            drawn.append(q)
            reported.append((pos["x"], pos["y"]))
        q_mat = np.hstack([np.array(drawn), np.ones((len(drawn), 1))])
        sol, *_ = np.linalg.lstsq(q_mat, np.array(reported), rcond=None)  # 3x2
        r = sol[:2].T                                  # 2x2
        t = sol[2]
        r_inv = np.linalg.inv(r)
        world.r_inv = r_inv.tolist()
        world.t = t.tolist()
        resid = max(abs(np.array(reported) - (q_mat @ sol)).flatten())
        print(f"CAL: 아핀 보정 확정 t=({t[0]:+.3f}, {t[1]:+.3f}) "
              f"잔차 {resid * 100:.1f}cm", flush=True)


async def udp_listener(world: World, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    sock.setblocking(False)
    loop = asyncio.get_event_loop()
    while True:
        data = await loop.sock_recv(sock, 4096)
        try:
            world.apply_cmd(json.loads(data.decode()))
        except (ValueError, UnicodeDecodeError):
            pass


async def stdin_commands(world: World) -> None:
    loop = asyncio.get_event_loop()
    while True:
        try:
            line = await loop.run_in_executor(None, sys.stdin.readline)
        except Exception:
            return
        if not line:                          # EOF — 파이프 닫힘
            return
        cmd = line.strip()
        if cmd == "mute":
            world.muted = True
            print("SIM camera MUTED (GPS 유실 주입)", flush=True)
        elif cmd == "unmute":
            world.muted = False
            print("SIM camera UNMUTED", flush=True)
        elif cmd == "pos":
            print(world.status(), flush=True)
        elif cmd == "quit":
            raise SystemExit


async def run(args) -> int:
    from websockets.asyncio.client import connect

    world = World(args)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    ssl_ctx = ctx if args.url.startswith("wss://") else None

    margin = MARKER_M
    refs = {0: (margin, margin), 1: (SPACE_W_M - margin, margin),
            2: (SPACE_W_M - margin, SPACE_H_M - margin), 3: (margin, SPACE_H_M - margin)}

    async with connect(args.url, ssl=ssl_ctx, max_size=16 * 1024 * 1024) as ws:
        hello = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
        if hello.get("type") != "hello":
            print(f"카메라 소켓이 아니다: {hello}", flush=True)
            return 1
        if not args.skip_init:
            ref_frame = draw(refs)
            for i in range(5):
                await send_frame(ws, ref_frame, i)
            await ws.send(json.dumps({"type": "initialize"}))
            res = json.loads(await ws.recv())
            if res.get("type") != "space_built":
                print(f"Initialize 실패: {res}", flush=True)
                return 1
            sp = res["space"]
            print(f"공간 확정: {sp['width_m']:.3f} x {sp['depth_m']:.3f} m", flush=True)

        loc_url = args.url.rsplit("/", 1)[0] + "/localization"
        await calibrate(world, ws, loc_url, ssl_ctx)

        asyncio.ensure_future(udp_listener(world, args.udp_port))
        asyncio.ensure_future(stdin_commands(world))
        print(f"SIM READY start=({world.x}, {world.y}) heading={world.heading} "
              f"udp={args.udp_port} fps={args.fps}", flush=True)

        frame_id = 100
        last_report = 0.0
        while True:
            world.step()
            if not world.muted:
                rot = {VEH_MARKER: 90.0 - world.heading}
                frame = draw({VEH_MARKER: world.draw_pos()}, rot_deg=rot)   # 아핀 보정 반영
                await send_frame(ws, frame, frame_id)
                frame_id += 1
            if time.monotonic() - last_report >= 2.0:
                last_report = time.monotonic()
                print(world.status(), flush=True)
            await asyncio.sleep(1.0 / args.fps)


def main() -> int:
    ap = argparse.ArgumentParser(description="통합 시뮬 월드 (운동학 + 합성 카메라)")
    ap.add_argument("--url", default="wss://127.0.0.1:8000/ws/v1/camera")
    ap.add_argument("--x", type=float, default=0.9)
    ap.add_argument("--y", type=float, default=2.61)
    ap.add_argument("--heading", type=float, default=0.0)
    ap.add_argument("--udp-port", type=int, default=9100)
    ap.add_argument("--fps", type=float, default=12.0)
    ap.add_argument("--max-speed", type=float, default=0.25,
                    help="throttle 1.0의 속도 (선형 가정 — B3 실측 전 잠정)")
    ap.add_argument("--wheelbase", type=float, default=0.14)
    ap.add_argument("--skip-init", action="store_true")
    args = ap.parse_args()
    try:
        return asyncio.run(run(args))
    except (KeyboardInterrupt, SystemExit):
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
