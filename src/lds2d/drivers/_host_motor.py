# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Base class for LiDARs whose motor the host must drive (LDS02RR, Delta-2A).

These sensors have no onboard speed control: they only stream data while the host
spins them at a target rate. This base embeds the control loop in the read loop —
iterating ``scans()``/``points()`` reads packets *and* runs a PID that trims the
adapter's MOT_EN PWM duty to hold the setpoint. The motor is stopped on close().

Subclasses set the motor/PID constants and implement ``_decode(buf)`` to yield
``(scan_freq_hz, [ScanPoint, ...])`` from a streaming byte buffer.
"""
from __future__ import annotations

from typing import Optional

from ..core import LidarDriver
from ..pid import PID
from .. import motor as _motor


class HostMotorLidar(LidarDriver):
    NEEDS_MOTOR = True

    # The PID runs in RPM (matching the kaiaai/LDS NEATO driver). Override per model.
    TARGET_RPM = 300.0
    PID_KP = 3.0e-3
    PID_KI = 1.0e-3
    PID_KD = 0.0
    PID_SAMPLE_MS = 20
    INITIAL_DUTY = 0.5
    DEFAULT_PWM_FREQ = 10000

    def __init__(self, transport, motor=None, *, pwm="software", pwm_pin=18,
                 pwm_channel=0, pwm_chip=None, pwm_freq=None,
                 target_hz=None, kp=None, ki=None, kd=None):
        super().__init__(transport)
        freq = pwm_freq if pwm_freq is not None else self.DEFAULT_PWM_FREQ
        # Inject a motor for tests; otherwise build one from the PWM settings.
        self._motor = motor if motor is not None else _motor.make_motor(
            pwm=pwm, pin=pwm_pin, channel=pwm_channel, chip=pwm_chip, freq=freq)
        setpoint_rpm = target_hz * 60.0 if target_hz else self.TARGET_RPM
        self._pid = PID(self.PID_KP if kp is None else kp,
                        self.PID_KI if ki is None else ki,
                        self.PID_KD if kd is None else kd,
                        setpoint_rpm, 0.0, 1.0, self.PID_SAMPLE_MS)
        self._measured_rpm = 0.0
        self._started = False

    # -- per-model parsing --
    def _decode(self, buf: bytearray):
        """Consume bytes from ``buf``; yield ``(scan_freq_hz, [ScanPoint, ...])``."""
        raise NotImplementedError

    # -- read loop that also drives the motor --
    def _ensure_started(self):
        if not self._started:
            self._pid.initialize(0.0, self.INITIAL_DUTY)
            self._motor.set_duty(self.INITIAL_DUTY)
            self._started = True

    def _packets(self):
        self._ensure_started()
        buf = bytearray()
        while True:
            chunk = self._t.read(256)
            if chunk:
                buf.extend(chunk)
                for freq_hz, points in self._decode(buf):
                    self._measured_rpm = freq_hz * 60.0
                    yield freq_hz, points
            out = self._pid.compute(self._measured_rpm)   # drive the motor each tick
            if out is not None:
                self._motor.set_duty(out)

    # -- motor control --
    def start(self) -> None:
        self._ensure_started()

    def stop(self) -> None:
        self._motor.set_duty(0.0)
        self._started = False

    def set_scan_freq(self, hz: float) -> None:
        if hz <= 0:
            raise ValueError("target scan rate must be > 0 Hz")
        self._pid.setpoint = hz * 60.0

    def get_scan_freq(self, listen_s: float = 1.0) -> Optional[float]:
        return self._measured_rpm / 60.0 if self._measured_rpm else None

    def close(self) -> None:
        try:
            self._motor.set_duty(0.0)
            self._motor.close()
        finally:
            super().close()
