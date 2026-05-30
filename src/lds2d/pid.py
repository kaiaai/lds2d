# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""PID controller used to hold host-driven LiDAR motors at a target speed.

A faithful port of Brett Beauregard's Arduino PID v1 (derivative-on-measurement,
integral clamp) as used by kaiaai/LDS."""
from __future__ import annotations

import time
from typing import Optional


class PID:
    def __init__(self, kp, ki, kd, setpoint, out_min, out_max, sample_ms):
        sample_s = sample_ms / 1000.0
        self.kp = kp
        self.ki = ki * sample_s          # pre-scaled by the sample period
        self.kd = kd / sample_s if sample_s else 0.0
        self.setpoint = setpoint
        self.out_min, self.out_max = out_min, out_max
        self.sample_s = sample_s
        self.i_term = 0.0
        self.last_input = 0.0
        self.last_time = time.monotonic() - sample_s

    def initialize(self, current_input, current_output):
        """Bumpless transfer: seed the integral with the current output."""
        self.i_term = min(max(current_output, self.out_min), self.out_max)
        self.last_input = current_input

    def compute(self, current_input) -> Optional[float]:
        """Return a new output, or None if the sample period hasn't elapsed."""
        now = time.monotonic()
        if now - self.last_time < self.sample_s:
            return None
        error = self.setpoint - current_input
        self.i_term = min(max(self.i_term + self.ki * error, self.out_min), self.out_max)
        d_input = current_input - self.last_input
        output = self.kp * error + self.i_term - self.kd * d_input
        output = min(max(output, self.out_min), self.out_max)
        self.last_input = current_input
        self.last_time = now
        return output
