# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Xiaomi LDS02RR driver (Neato XV11 family).

The LDS02RR has no onboard motor controller: the host must spin it at ~5 Hz, or
it stops outputting data. This driver embeds the motor control in the read loop —
iterating ``scans()`` / ``points()`` reads packets *and* runs a PID that trims the
motor PWM duty to hold the target speed (the RPM the LiDAR reports in each packet).

    from lds2d import Lidar
    with Lidar.open("LDS02RR", "/dev/serial0", pwm="software", pwm_pin=18) as lidar:
        for scan in lidar.scans():
            ...   # the motor is driven for you; stopped on exit

Packet (22 bytes): 0xFA  index  speed(u16)  4x[dist(u16) signal(u16)]  CRC(u16).
RPM = speed / 64; angle = (index - 0xA0) * 4 (+0..3). Constants from kaiaai/LDS.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

from ..core import LidarDriver, ScanPoint, register
from ..pid import PID
from .. import motor as _motor

_COMMAND = 0xFA
_INDEX_LO = 0xA0
_PACKET_LEN = 22
_N_QUADS = 4
_INVALID_FLAG = 1 << 7
_WARNING_FLAG = 1 << 6

# motor control defaults (ported from kaiaai/LDS)
TARGET_RPM = 300.0          # 5 Hz
PID_KP = 3.0e-3
PID_KI = 1.0e-3
PID_KD = 0.0
PID_SAMPLE_MS = 20
INITIAL_DUTY = 0.5
DEFAULT_PWM_FREQ = 10000


def valid_packet(pkt: bytes) -> bool:
    """XV11 checksum: sum the ten 16-bit little-endian words, fold to 15 bits."""
    chk = 0
    for i in range(0, 20, 2):
        chk = (chk << 1) + (pkt[i] | (pkt[i + 1] << 8))
    checksum = ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF
    return checksum == (pkt[20] | (pkt[21] << 8))


def parse_packet(pkt: bytes) -> Tuple[float, List[ScanPoint]]:
    """Return (scan_freq_hz, points) for one validated 22-byte packet."""
    start_angle = (pkt[1] - _INDEX_LO) * _N_QUADS
    rpm = (pkt[2] | (pkt[3] << 8)) / 64.0
    points = []
    for q in range(_N_QUADS):
        off = 4 + q * 4
        msb = pkt[off + 1]
        quality = pkt[off + 2] | (pkt[off + 3] << 8)
        dist = 0 if (msb & (_INVALID_FLAG | _WARNING_FLAG)) else (pkt[off] | ((msb & 0x3F) << 8))
        points.append(ScanPoint(start_angle + q, dist, quality))  # LDS02RR scans CW
    return rpm / 60.0, points


def iter_packets(buf: bytearray):
    """Yield validated 22-byte packets from a bytearray, consuming what it uses."""
    while True:
        i = buf.find(b"\xFA")
        if i < 0:
            buf.clear()
            return
        if i > 0:
            del buf[:i]
        if len(buf) < _PACKET_LEN:
            return
        pkt = bytes(buf[:_PACKET_LEN])
        if valid_packet(pkt):
            del buf[:_PACKET_LEN]
            yield pkt
        else:
            del buf[:1]


@register("LDS02RR", "XIAOMI_LDS02RR")
class LDS02RR(LidarDriver):
    MODEL_NAME = "Xiaomi LDS02RR"
    DEFAULT_BAUD = 115200
    NEEDS_MOTOR = True

    def __init__(self, transport, motor=None, *, pwm="software", pwm_pin=18,
                 pwm_channel=0, pwm_chip=None, pwm_freq=DEFAULT_PWM_FREQ,
                 target_hz=TARGET_RPM / 60.0, kp=PID_KP, ki=PID_KI, kd=PID_KD):
        super().__init__(transport)
        # Inject a motor for tests; otherwise build one from the PWM settings.
        self._motor = motor if motor is not None else _motor.make_motor(
            pwm=pwm, pin=pwm_pin, channel=pwm_channel, chip=pwm_chip, freq=pwm_freq)
        self._pid = PID(kp, ki, kd, target_hz * 60.0, 0.0, 1.0, PID_SAMPLE_MS)
        self._measured_rpm = 0.0
        self._started = False

    def _ensure_started(self):
        if not self._started:
            self._pid.initialize(0.0, INITIAL_DUTY)
            self._motor.set_duty(INITIAL_DUTY)
            self._started = True

    def _packets(self):
        self._ensure_started()
        buf = bytearray()
        while True:
            chunk = self._t.read(256)
            if chunk:
                buf.extend(chunk)
                for pkt in iter_packets(buf):
                    freq_hz, points = parse_packet(pkt)
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
