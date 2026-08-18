# -*- coding: utf-8 -*-
"""PCA9685 DC 구동 모터 제어."""

from adafruit_pca9685 import PCA9685
import board
import busio

import config


class MotorController:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pwm = PCA9685(i2c, address=config.MOTOR_I2C_ADDRESS)
        self.pwm.frequency = 60
        self.channel = config.MOTOR_CHANNEL
        self.stop()

    def _set_raw_throttle(self, throttle):
        """기존 모터 보드의 -1.0~1.0 원시 입력을 설정한다."""
        throttle = max(-1.0, min(1.0, float(throttle)))
        pulse = int(0xFFFF * abs(throttle))
        pwm_channel = self.pwm.channels

        if throttle < 0:
            pwm_channel[self.channel + 5].duty_cycle = pulse
            pwm_channel[self.channel + 4].duty_cycle = 0
            pwm_channel[self.channel + 3].duty_cycle = 0xFFFF
        elif throttle > 0:
            pwm_channel[self.channel + 5].duty_cycle = pulse
            pwm_channel[self.channel + 4].duty_cycle = 0xFFFF
            pwm_channel[self.channel + 3].duty_cycle = 0
        else:
            pwm_channel[self.channel + 5].duty_cycle = 0
            pwm_channel[self.channel + 4].duty_cycle = 0
            pwm_channel[self.channel + 3].duty_cycle = 0

    def forward(self, speed):
        """0.0~1.0의 속도로 전진한다."""
        self._set_raw_throttle(-config.MOTOR_SIGN * abs(speed))

    def stop(self):
        self._set_raw_throttle(0.0)

    def close(self):
        self.stop()
        self.pwm.deinit()
