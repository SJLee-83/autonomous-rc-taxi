# -*- coding: utf-8 -*-
"""PCA9685 조향 서보 제어."""

import math

from adafruit_servokit import ServoKit
import board
import busio

import config


class SteeringController:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.kit = ServoKit(
            channels=16, i2c=i2c, address=config.SERVO_I2C_ADDRESS
        )
        self.channel = config.SERVO_CHANNEL
        self.center()

    def set_angle(self, angle):
        angle = float(angle)
        if not math.isfinite(angle):
            raise ValueError("조향각은 유한한 숫자여야 합니다.")
        angle = max(
            config.SERVO_RIGHT_ANGLE,
            min(config.SERVO_LEFT_ANGLE, angle),
        )
        self.kit.servo[self.channel].angle = angle

    def center(self):
        self.set_angle(config.SERVO_CENTER_ANGLE)
