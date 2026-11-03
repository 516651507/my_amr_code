# -*- coding: utf-8 -*-
"""差速底盘轮速 → 位姿积分。"""

from __future__ import division

import math


class DiffOdometry(object):
    def __init__(self, wheel_spacing):
        self.wheel_spacing = float(wheel_spacing)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.omega = 0.0

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def update_from_wheel_speeds(self, v_left, v_right, dt):
        dt = max(0.0, float(dt))
        spacing = max(self.wheel_spacing, 1e-6)
        self.vx = 0.5 * (v_left + v_right)
        self.omega = (v_right - v_left) / spacing
        if abs(self.omega) < 1e-9:
            self.x += self.vx * math.cos(self.theta) * dt
            self.y += self.vx * math.sin(self.theta) * dt
        else:
            radius = self.vx / self.omega
            dtheta = self.omega * dt
            self.x += radius * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + dtheta) - math.cos(self.theta))
            self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        return self.x, self.y, self.theta
