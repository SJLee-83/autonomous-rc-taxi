#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""guard — 차량 프로세스 '동결' 대비 독립 모터 차단기 (2026-08-05 사고 대책).

차량 프로세스가 통째로 얼면(콘솔 파이프 역압 등) 내부 안전장치(워치독 래치·
finally force_stop)까지 같이 얼어 모터가 마지막 명령에 래치된 채 주행한다.
이 스크립트는 **별도 프로세스**로 돌며 SafetyWatchdog가 20Hz로 찍는 하트비트
(/dev/shm/veh_heartbeat, monotonic)를 감시하다가, 기록이 stale_sec 이상 끊기면
PCA9685 모터 채널을 레지스터 직접 쓰기(FULL_OFF)로 끈다.

- 하트비트 파일 없음 = 차량 미기동/정상 종료(clear_heartbeat) → 대기
- 차량이 되살아나 duty를 다시 쓰면 FULL_OFF는 덮인다 — 가드는 stale이 지속되는
  동안 매 주기 다시 끈다 (되살아난 차량은 자체 안전 사슬이 이어받는다)

실행 (보드, 차량 기동 전에 미리):
  nohup python3 ~/rc_car/tools/guard.py > ~/guard.log 2>&1 &

레지스터 근거: PCA9685 LEDn_OFF_H(0x09+4n)의 bit4 = FULL_OFF.
모터(0x40, motor_channel=0)는 vendor motor_controller가 채널 ch+3(IN2)·
ch+4(IN1)·ch+5(PWM)를 쓴다 — 셋 다 끄면 코스트 정지.
"""
from __future__ import annotations

import argparse
import subprocess
import time
from pathlib import Path


def full_off_regs(motor_channel: int) -> list[int]:
    return [0x09 + 4 * (motor_channel + k) for k in (3, 4, 5)]


def fire(bus: int, addr: int, regs: list[int]) -> bool:
    ok = True
    for reg in regs:
        r = subprocess.run(
            ["i2cset", "-y", str(bus), f"0x{addr:02x}", f"0x{reg:02x}", "0x10"],
            capture_output=True, text=True)
        ok = ok and r.returncode == 0
    return ok


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--heartbeat", type=Path, default=Path("/dev/shm/veh_heartbeat"))
    ap.add_argument("--stale-sec", type=float, default=0.7)
    ap.add_argument("--interval", type=float, default=0.2)
    ap.add_argument("--bus", type=int, default=7)
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0x40)
    ap.add_argument("--motor-channel", type=int, default=0)
    args = ap.parse_args()

    regs = full_off_regs(args.motor_channel)
    print(f"guard 시작 — hb={args.heartbeat} stale>{args.stale_sec}s "
          f"-> i2c bus{args.bus} 0x{args.addr:02x} regs {[hex(r) for r in regs]}",
          flush=True)
    firing = False
    while True:
        try:
            hb = float(args.heartbeat.read_text())
            age = time.monotonic() - hb
        except (OSError, ValueError):
            age = None                        # 미기동/정상 종료 — 대기
        if age is not None and age > args.stale_sec:
            ok = fire(args.bus, args.addr, regs)
            if not firing:
                print(f"[{time.strftime('%T')}] 하트비트 {age:.1f}s 정체 — "
                      f"모터 FULL_OFF {'성공' if ok else '실패'}", flush=True)
                firing = True
        elif firing and age is not None:
            print(f"[{time.strftime('%T')}] 하트비트 회복 (age {age:.2f}s)", flush=True)
            firing = False
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
