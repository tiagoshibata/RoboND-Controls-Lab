# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0., ki = 0., kd = 0., max_windup = 10):
        self.reset()
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_windup = max_windup


    def reset(self):
        self.kp = 0.
        self.ki = 0.
        self.kd = 0.
        self.max_windup = 0.
        self.last_diff = self.last_time = self.set_point = self.error_integral = self.last_error = 0.


    def setTarget(self, target):
        self.set_point = target


    def setKP(self, kp):
        self.kp = kp


    def setKI(self, ki):
        self.ki = ki


    def setKD(self, kd):
        self.kd = kd


    def setMaxWindup(self, max_windup):
        self.max_windup = max_windup


    def update(self, measured_value, timestamp):
        error = self.set_point - measured_value
        delta = timestamp - self.last_time

        self.error_integral += error * delta
        self.error_integral = min(max(self.error_integral, -self.max_windup), self.max_windup)

        p = self.kp * error
        i = self.ki * self.error_integral

        diff = error - self.last_error
        alpha = .8
        filtered_diff = alpha * diff / delta + (1 - alpha) * self.last_diff
        d = self.kd * filtered_diff

        self.last_error = error
        self.last_time = timestamp
        self.last_diff = filtered_diff

        return p + i + d