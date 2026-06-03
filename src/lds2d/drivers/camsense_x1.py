# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""Camsense X1 driver: parse the self-spinning sensor's 36-byte scan packets.

Ported from kaiaai/LDS ``LDS_CAMSENSE_X1.{h,cpp}``. The X1 free-runs at ~5.2 Hz
(its motor has no software speed control: ``setScanTargetFreqHz`` /``stop`` return
ERROR_NOT_IMPLEMENTED), so this is a plain :class:`LidarDriver` with no motor
methods, just a parser.

Packet (36 bytes, all multi-byte fields little-endian)::

    off  size  field
    0    1     0x55         start_byte0
    1    1     0xAA         start_byte1
    2    1     0x03         start_byte2
    3    1     0x08         samples_per_packet (8)
    4    2     u16          rotation_speed   (= Hz * 64 * 60)
    6    2     u16          start_angle      (angle_q6 + 0xA000)
    8    24    8 x sample   each: int16 distance_mm, u8 quality
    32   2     u16          end_angle        (angle_q6 + 0xA000)
    34   2     u16          crc16            (present, NOT verified by kaiaai/LDS)

Field math (faithful to the .cpp):
  * scan_freq_hz   = rotation_speed / 3840.0      (3840 = 64 * 60)
  * angle_deg      = (raw_angle - 0xA000) / 64.0
  * start/end angle MUST be >= 0xA000, else the packet is rejected (resync)
  * per-sample angle is linearly interpolated from start_angle_deg to
    end_angle_deg across the 8 samples (step = span / 7); if start > end the
    span wraps so end += 360 before stepping, and angles > 360 wrap back down.
  * distance is a *signed* int16 in millimetres (negative = invalid return).

Checksum note: the kaiaai/LDS parser carries the trailing ``crc16`` field but
does NOT validate it (the ``ERROR_CHECKSUM`` line is commented out). Its only
packet-integrity gate is the ``angle >= 0xA000`` range check, which this port
reproduces. We expose ``crc16`` from the parse but, like the C++, do not reject
on it.
"""
from __future__ import annotations

import struct
from typing import List, Tuple

from ..core import LidarDriver, ScanPoint, register

_START0 = 0x55
_START1 = 0xAA
_START2 = 0x03
_SAMPLES_PER_PACKET = 0x08
_ANGLE_MIN = 0xA000

_HEADER = bytes([_START0, _START1, _START2, _SAMPLES_PER_PACKET])
_PACKET_SIZE = 36                  # sizeof(scan_packet_t)
_ROT_SPEED_TO_HZ = 1.0 / 3840.0    # rotation_speed = Hz * 64 * 60
_ANGLE_TO_DEG = 1.0 / 64.0
_SAMPLE_OFF = 8                    # first sample byte offset
_END_ANGLE_OFF = 32


def parse_packet(packet: bytes) -> Tuple[float, List[ScanPoint], int]:
    """Parse one validated 36-byte packet into (scan_freq_hz, points, crc16).

    Raises ``ValueError`` if either angle field is below ``0xA000`` (the same
    rejection the kaiaai/LDS parser applies before producing any points).
    """
    rotation_speed, start_angle = struct.unpack_from("<HH", packet, 4)
    end_angle, crc16 = struct.unpack_from("<HH", packet, _END_ANGLE_OFF)

    if start_angle < _ANGLE_MIN or end_angle < _ANGLE_MIN:
        raise ValueError("Camsense X1 angle field below 0xA000")

    scan_freq_hz = rotation_speed * _ROT_SPEED_TO_HZ
    start_deg = (start_angle - _ANGLE_MIN) * _ANGLE_TO_DEG
    end_deg = (end_angle - _ANGLE_MIN) * _ANGLE_TO_DEG
    if start_angle > end_angle:        # rotation wrapped past 0 within the packet
        end_deg += 360.0
    step_deg = (end_deg - start_deg) / (_SAMPLES_PER_PACKET - 1)

    points: List[ScanPoint] = []
    for i in range(_SAMPLES_PER_PACKET):
        dist_mm, quality = struct.unpack_from("<hB", packet, _SAMPLE_OFF + i * 3)
        angle_deg = start_deg + step_deg * i
        if angle_deg > 360.0:
            angle_deg -= 360.0
        points.append(ScanPoint(angle_deg, dist_mm, quality))
    return scan_freq_hz, points, crc16


def _find_header(buf: bytearray) -> int:
    """Index of the next 0x55 0xAA 0x03 0x08 header in ``buf`` after pos 0, or -1."""
    idx = buf.find(_HEADER, 1)
    return idx


@register("CAMSENSE-X1", "CAMSENSE_X1", "CAMSENSE", "X1")
class CamsenseX1(LidarDriver):
    MODEL_NAME = "Camsense X1"
    DEFAULT_BAUD = 115200

    def _packets(self):
        buf = bytearray()
        while True:
            chunk = self._t.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= _PACKET_SIZE:
                if buf[:4] != _HEADER:
                    idx = _find_header(buf)
                    if idx < 0:
                        # keep only a possible partial header tail
                        del buf[:-3]
                        break
                    del buf[:idx]
                    if len(buf) < _PACKET_SIZE:
                        break
                packet = bytes(buf[:_PACKET_SIZE])
                try:
                    freq, points, _crc = parse_packet(packet)
                except ValueError:
                    del buf[0]          # bad angle field: drop one byte and resync
                    continue
                yield freq, points
                del buf[:_PACKET_SIZE]
