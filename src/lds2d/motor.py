# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Motor PWM back ends for host-driven LiDARs (e.g. the Xiaomi LDS02RR).

A motor object only needs ``set_duty(0..1)`` and ``close()``. Drivers accept any
such object, so they can be unit-tested with a fake motor and no hardware. The
hardware libraries are imported lazily, so importing this module is always safe.
"""
from __future__ import annotations

_HINT = "install the PWM extra:  pip install 'lds2d[pwm]'"


class SoftwarePWM:
    """gpiozero software PWM on any GPIO pin. Simplest, more timing jitter."""

    def __init__(self, pin: int, freq: int = 10000):
        try:
            from gpiozero import PWMOutputDevice
        except ImportError as e:
            raise ImportError(f"gpiozero is required for software PWM — {_HINT}") from e
        self._dev = PWMOutputDevice(pin, frequency=freq)

    def set_duty(self, duty: float) -> None:
        self._dev.value = min(max(duty, 0.0), 1.0)

    def close(self) -> None:
        self._dev.value = 0.0
        self._dev.close()


class HardwarePWM:
    """Pi hardware PWM (clean multi-kHz). Needs 'dtoverlay=pwm-2chan' + reboot;
    on the Pi 5 you may need chip=2. (Not yet hardware-verified for the LDS02RR.)"""

    def __init__(self, channel: int = 0, freq: int = 10000, chip=None):
        try:
            from rpi_hardware_pwm import HardwarePWM as _HW
        except ImportError as e:
            raise ImportError(f"rpi-hardware-pwm is required for hardware PWM — {_HINT}") from e
        kwargs = {"pwm_channel": channel, "hz": freq}
        if chip is not None:
            kwargs["chip"] = chip
        self._pwm = _HW(**kwargs)
        self._pwm.start(0)

    def set_duty(self, duty: float) -> None:
        self._pwm.change_duty_cycle(min(max(duty, 0.0), 1.0) * 100.0)

    def close(self) -> None:
        self._pwm.change_duty_cycle(0)
        self._pwm.stop()


def make_motor(pwm: str = "software", pin: int = 18, channel: int = 0,
               chip=None, freq: int = 10000):
    """Build a motor PWM back end. ``pwm`` is 'software' or 'hardware'."""
    if pwm == "software":
        return SoftwarePWM(pin, freq)
    if pwm == "hardware":
        return HardwarePWM(channel, freq, chip)
    raise ValueError("pwm must be 'software' or 'hardware'")
