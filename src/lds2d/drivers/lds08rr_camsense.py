# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""LDS08RR, Camsense-protocol hardware revision.

There is more than one LDS08RR revision in the wild. ``3IROBOTIX-LDS08RR`` (see
``threeirobotix_delta_variants``) speaks the 3irobotix Delta protocol (0xAA
framing). This one speaks the Camsense X1 protocol family (0x55 0xAA framing)
and will produce noise if parsed by the Delta driver, and vice versa. If your
stream starts with 0x55 0xAA, use this model.

Ported from kaiaai/LDS ``LDS_LDS08RR_CAMSENSE.{h,cpp}``. Like the Camsense X1
the motor free-runs at a fixed ~5 Hz with no software speed control, so this is
a plain :class:`LidarDriver` with no motor methods, just a parser.

Packet (60 bytes, all multi-byte fields little-endian)::

    off  size  field
    0    1     0x55          start_byte0
    1    1     0xAA          start_byte1
    2    1     0x07          start_byte2          (Camsense X1: 0x03)
    3    1     0x0C          samples_per_packet   (12; Camsense X1: 8)
    4    2     u16           rotation_speed       (= Hz * 64 * 60)
    6    2     u16           start_angle          (angle_q6 + 0xA000)
    8    48    12 x sample   each: int16 distance_mm, u8 quality, u8 pad(=0)
    56   2     u16           end_angle            (angle_q6 + 0xA000)
    58   2     u16           checksum             (15-bit, see checksum())

Field math (faithful to the .cpp):
  * scan_freq_hz = rotation_speed / 3840.0      (3840 = 64 * 60)
  * angle_deg    = (raw_angle - 0xA000) / 64.0
  * per-sample angle is linearly interpolated from start to end across the 12
    samples (step = span / 11); if start > end the span wraps so end += 360
    before stepping, and angles > 360 wrap back down.
  * distance is a *signed* int16 in millimetres; negative means no return
    (0x8000 / -32768 is the marker seen in captures).

Validity gates: ``55 AA 07 0C`` also occurs inside sample data, so the header
match alone locks onto false packets, and the checksum is what rejects them.
Note this is *not* a standard CRC16 -- it is the vendor's own 15-bit fold (see
:func:`lds2d.crc.camsense_checksum`), which is why it appears as an unidentified
"crc16" in the community Camsense drivers.

Measured against the capture in https://github.com/kaiaai/LDS/issues/17: of the
packets a bare header match accepts, the checksum rejects 100% of the false
locks, and every surviving packet lands in a tight 10.84-11.27 deg span with
physically sane distances.

Protocol reverse engineered from that capture; thanks to Nelson (@npireso) for
the log. Verified against it, but not yet run against live hardware by us.
"""
from __future__ import annotations

import struct
from typing import List, Tuple

from ..core import LidarDriver, ScanPoint, register
from ..crc import camsense_checksum as checksum

_START0 = 0x55
_START1 = 0xAA
_START2 = 0x07
_SAMPLES_PER_PACKET = 0x0C
_ANGLE_MIN = 0xA000

_HEADER = bytes([_START0, _START1, _START2, _SAMPLES_PER_PACKET])
_PACKET_SIZE = 60
_ROT_SPEED_TO_HZ = 1.0 / 3840.0
_ANGLE_TO_DEG = 1.0 / 64.0
_SAMPLE_OFF = 8
_END_ANGLE_OFF = 56


def parse_packet(packet: bytes) -> Tuple[float, List[ScanPoint], int]:
    """Parse one 60-byte packet into (scan_freq_hz, points, checksum).

    Raises ``ValueError`` if the checksum fails or an angle field is below
    ``0xA000`` -- either indicates a false header lock or a corrupt packet.
    """
    rotation_speed, start_angle = struct.unpack_from("<HH", packet, 4)
    end_angle, packet_checksum = struct.unpack_from("<HH", packet, _END_ANGLE_OFF)

    if checksum(packet[:-2]) != packet_checksum:
        raise ValueError("LDS08RR checksum mismatch")

    if start_angle < _ANGLE_MIN or end_angle < _ANGLE_MIN:
        raise ValueError("LDS08RR angle field below 0xA000")

    start_deg = (start_angle - _ANGLE_MIN) * _ANGLE_TO_DEG
    end_deg = (end_angle - _ANGLE_MIN) * _ANGLE_TO_DEG
    if start_angle > end_angle:         # rotation wrapped past 0 within the packet
        end_deg += 360.0

    step_deg = (end_deg - start_deg) / (_SAMPLES_PER_PACKET - 1)

    points: List[ScanPoint] = []
    for i in range(_SAMPLES_PER_PACKET):
        dist_mm, quality, _pad = struct.unpack_from("<hBB", packet,
                                                    _SAMPLE_OFF + i * 4)
        angle_deg = start_deg + step_deg * i
        if angle_deg > 360.0:
            angle_deg -= 360.0
        points.append(ScanPoint(angle_deg, dist_mm, quality))
    return rotation_speed * _ROT_SPEED_TO_HZ, points, packet_checksum


@register("3IROBOTIX-LDS08RR-CAMSENSE", "3IROBOTIX_LDS08RR_CAMSENSE",
          "LDS08RR-CAMSENSE", "LDS08RR_CAMSENSE")
class LDS08RRCamsense(LidarDriver):
    MODEL_NAME = "LDS08RR (Camsense protocol)"
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
                    idx = buf.find(_HEADER, 1)
                    if idx < 0:
                        del buf[:-3]    # keep a possible partial header tail
                        break
                    del buf[:idx]
                    if len(buf) < _PACKET_SIZE:
                        break
                packet = bytes(buf[:_PACKET_SIZE])
                try:
                    freq, points, _crc = parse_packet(packet)
                except ValueError:
                    del buf[0]          # false lock: drop one byte and resync
                    continue
                yield freq, points
                del buf[:_PACKET_SIZE]
