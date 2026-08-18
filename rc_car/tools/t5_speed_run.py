# -*- coding: utf-8 -*-
"""T5 실측 3 (스로틀-속도 곡선) 주행 러너.

조향을 중앙(108)에 고정하고 지정 스로틀로 지정 시간 직진 후 정지한다.
바닥 마크 2개(2m 간격) 통과 시간을 스톱워치로 재서 속도를 구한다.

    python3 t5_speed_run.py --throttle 0.3 --seconds 10

MOTOR_SIGN=-1 확정(2026-07-30 실측 1) 기준: 전진 raw = +throttle.
안전: 시간이 다 되면 무조건 정지. 예외가 나도 finally 에서 정지.
"""
import argparse
import time

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

SERVO_ADDR = 0x60
MOTOR_ADDR = 0x40
SERVO_CENTER = 108
RAMP_S = 0.5          # 급출발 방지 램프


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--throttle", type=float, required=True, help="0.05~1.0")
    ap.add_argument("--seconds", type=float, required=True, help="주행 시간")
    ap.add_argument("--servo", type=float, default=SERVO_CENTER,
                    help="직진 트림 탐색용 서보 각도 (기본 108)")
    args = ap.parse_args()
    th = max(0.0, min(1.0, args.throttle))
    servo_angle = max(48.0, min(168.0, args.servo))

    i2c = busio.I2C(board.SCL, board.SDA)
    kit = ServoKit(channels=16, i2c=i2c, address=SERVO_ADDR)
    pwm = PCA9685(i2c, address=MOTOR_ADDR)
    pwm.frequency = 60
    ch = pwm.channels

    def raw(t):
        # vendor motor_controller 로직, MOTOR_SIGN=-1 -> 전진 = 양수 raw
        pulse = int(0xFFFF * abs(t))
        if t > 0:
            ch[5].duty_cycle = pulse
            ch[4].duty_cycle = 0xFFFF
            ch[3].duty_cycle = 0
        else:
            ch[5].duty_cycle = 0
            ch[4].duty_cycle = 0
            ch[3].duty_cycle = 0

    try:
        kit.servo[0].angle = servo_angle
        time.sleep(0.3)
        print("RUN throttle=%.2f seconds=%.1f servo=%.1f"
              % (th, args.seconds, servo_angle), flush=True)
        steps = 10
        for i in range(1, steps + 1):          # 0.5초 램프
            raw(th * i / steps)
            time.sleep(RAMP_S / steps)
        time.sleep(max(0.0, args.seconds - RAMP_S))
    finally:
        raw(0.0)
        pwm.deinit()
        print("STOP", flush=True)


if __name__ == "__main__":
    main()
