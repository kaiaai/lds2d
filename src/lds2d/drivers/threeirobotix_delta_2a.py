# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""3irobotix Delta-2A driver — host-driven motor (same MOT_EN PWM idea as the
LDS02RR; the motor loop lives in HostMotorLidar). This module adds the Delta
packet parser.

    from lds2d import Lidar
    with Lidar.open("DELTA-2A", "/dev/serial0", pwm="software", pwm_pin=18) as lidar:
        for scan in lidar.scans():
            ...

Packet (big-endian multibyte fields):
  0xAA  packet_length(u16)  ver=0x01  type=0x61  data_type  data_length(u16)
  scan_freq_x20  offset_angle(i16)  start_angle_x100(u16)  N x [quality dist_x4(u16)]  checksum(u16)
data_type 0xAD = speed + measurements, 0xAE = speed only. scan_freq_hz = byte/20,
distance_mm = dist_x4/4, angle = start_angle + idx*360/(16*N). Checksum is the
16-bit sum of every byte before the 2-byte checksum. Constants from kaiaai/LDS.
"""
from __future__ import annotations

from ..core import ScanPoint, register
from ._host_motor import HostMotorLidar

_START = 0xAA
_VERSION = 0x01
_TYPE = 0x61
_DT_SPEED_MEAS = 0xAD
_DT_SPEED_ONLY = 0xAE
_HEADER_LEN = 13            # 0xAA..start_angle_x100, before the samples
_PACKETS_PER_SCAN = 16
_MAX_PACKET_LEN = 220       # 13 header + 3*61 samples (230400 variant) + margin


def _u16(b, i):             # big-endian
    return (b[i] << 8) | b[i + 1]


def decode_delta(buf: bytearray, packets_per_scan: int = _PACKETS_PER_SCAN):
    """Yield (scan_freq_hz, [ScanPoint, ...]) for each complete Delta packet."""
    while True:
        i = buf.find(b"\xAA")
        if i < 0:
            buf.clear()
            return
        if i > 0:
            del buf[:i]
        if len(buf) < 6:                       # need the header signature
            return
        if buf[3] != _VERSION or buf[4] != _TYPE or buf[5] not in (_DT_SPEED_MEAS, _DT_SPEED_ONLY):
            del buf[:1]
            continue
        packet_length = _u16(buf, 1)
        if not (_HEADER_LEN <= packet_length <= _MAX_PACKET_LEN):
            del buf[:1]
            continue
        total = packet_length + 2              # + 2-byte checksum
        if len(buf) < total:
            return                             # wait for the rest
        pkt = bytes(buf[:total])
        if (sum(pkt[:packet_length]) & 0xFFFF) != _u16(pkt, packet_length):
            del buf[:1]                        # bad checksum: resync
            continue
        del buf[:total]

        scan_freq_hz = pkt[8] * 0.05
        if pkt[5] != _DT_SPEED_MEAS:
            yield scan_freq_hz, []             # 0xAE: speed only, no points
            continue
        data_length = _u16(pkt, 6)
        sample_count = (data_length - 5) // 3
        if sample_count <= 0 or _HEADER_LEN + sample_count * 3 > packet_length:
            continue
        start_angle = _u16(pkt, 11) * 0.01
        coeff = 360.0 / (packets_per_scan * sample_count)
        points = []
        for idx in range(sample_count):
            off = _HEADER_LEN + idx * 3
            quality = pkt[off]
            dist_mm = int(round(_u16(pkt, off + 1) * 0.25))
            angle = (start_angle + idx * coeff) % 360.0
            points.append(ScanPoint(angle, dist_mm, quality))
        yield scan_freq_hz, points


@register("DELTA-2A", "DELTA_2A", "DELTA2A", "3IROBOTIX_DELTA_2A")
class Delta2A(HostMotorLidar):
    MODEL_NAME = "3irobotix Delta-2A"
    DEFAULT_BAUD = 115200       # the 230400 variant is the same parser at a higher baud
    TARGET_RPM = 360.0          # 6 Hz
    # C++ runs this PID in Hz (Kp=0.3, Ki=0.1); our base runs it in RPM, so / 60.
    PID_KP = 0.3 / 60.0
    PID_KI = 0.1 / 60.0
    PID_KD = 0.0
    INITIAL_DUTY = 0.6
    PACKETS_PER_SCAN = _PACKETS_PER_SCAN

    def _decode(self, buf):
        yield from decode_delta(buf, self.PACKETS_PER_SCAN)
