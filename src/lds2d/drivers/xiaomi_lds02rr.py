# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Xiaomi LDS02RR driver (Neato XV11 family) — host-driven motor.

The LDS02RR has no onboard motor controller: the host must spin it at ~5 Hz, or
it stops outputting data. The motor loop lives in HostMotorLidar; this module adds
the XV11 packet parser.

    from lds2d import Lidar
    with Lidar.open("LDS02RR", "/dev/serial0", pwm="software", pwm_pin=18) as lidar:
        for scan in lidar.scans():
            ...   # the motor is driven for you; stopped on exit

Packet (22 bytes): 0xFA  index  speed(u16)  4x[dist(u16) signal(u16)]  CRC(u16).
RPM = speed / 64; angle = (index - 0xA0) * 4 (+0..3). Constants from kaiaai/LDS.
"""
from __future__ import annotations

from typing import List, Tuple

from ..core import ScanPoint, register
from ._host_motor import HostMotorLidar

_COMMAND = 0xFA
_INDEX_LO = 0xA0
_PACKET_LEN = 22
_N_QUADS = 4
_INVALID_FLAG = 1 << 7
_WARNING_FLAG = 1 << 6


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
class LDS02RR(HostMotorLidar):
    MODEL_NAME = "Xiaomi LDS02RR"
    DEFAULT_BAUD = 115200
    TARGET_RPM = 300.0          # 5 Hz
    PID_KP = 3.0e-3             # PID runs in RPM (kaiaai/LDS NEATO driver)
    PID_KI = 1.0e-3
    PID_KD = 0.0
    INITIAL_DUTY = 0.5

    def _decode(self, buf):
        for pkt in iter_packets(buf):
            yield parse_packet(pkt)
