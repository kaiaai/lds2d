# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""COIN-D4A (a.k.a. COIN-D4) driver: a small, cheap self-spinning ToF LiDAR.

Ported from the MicroPython driver by QuirkyCort (Cort Stratton), released
under the IoTy project: https://github.com/QuirkyCort/IoTy/blob/main/public/extensions/coind4.py
Requested in https://github.com/kaiaai/awesome-2d-lidars/issues/3 .
The wire protocol matches the CSPC "M1C1 Mini" family (https://www.cspctech.com/resources).

Frame (variable length, little-endian):
    0xAA 0x55  speed  n  start_angle(u16)  end_angle(u16)  cs_even  cs_odd
    n x [ mode/strength_hi  dist_lo/strength_lo  dist_hi ]
  - speed: if bit0 set, rev/s = (speed >> 1) / 10  (so scan_freq_hz directly)
  - start/end angle are in 1/128 deg; the driver shifts them to 1/64 deg
  - each 3-byte sample: distance_mm = b1 >> 2 | b2 << 6 (14-bit);
    strength = b2 >> 2 | (b1 & 0x03) << 6
  - two split XOR checksums cover the even- and odd-indexed bytes

Motor command frame (4 bytes): 0xAA 0x55 code (0xAA ^ 0x55 ^ code).
The device spins at a fixed ~10 Hz; the scan rate is not host-settable, but the
exposure can be toggled high/low.
"""
from __future__ import annotations

import struct
from typing import List, Tuple

from ..core import LidarDriver, ScanPoint, register

_H0 = 0xAA
_H1 = 0x55
_HEADER = bytes((_H0, _H1))
_HEADER_LEN = 10
_MAX_SAMPLES = 25          # frame length maxes out at 10 + 25*3 = 85 bytes
_DEG_PER_REV_X64 = 360 * 64  # 23040

CMD_START = 0xF0
CMD_HIGH_EXPOSURE = 0xF1
CMD_LOW_EXPOSURE = 0xF2
CMD_STOP = 0xF5


def build_command(code: int) -> bytes:
    """Build a 4-byte command frame with its XOR checksum."""
    return bytes((_H0, _H1, code, _H0 ^ _H1 ^ code))


def checksum_ok(frame: bytes, n: int) -> bool:
    """Verify the pair of split XOR checksums (bytes 8 and 9)."""
    cs_even = frame[0] ^ frame[2] ^ frame[4] ^ frame[6]
    cs_odd = frame[1] ^ frame[3] ^ frame[5] ^ frame[7]
    for i in range(n):
        base = _HEADER_LEN + i * 3
        cs_even ^= frame[base] ^ frame[base + 1]
        cs_odd ^= frame[base + 2]
    return cs_even == frame[8] and cs_odd == frame[9]


def parse_frame(frame: bytes, n: int) -> Tuple[float, List[ScanPoint]]:
    """Parse one validated frame into (scan_freq_hz, points)."""
    speed = frame[2]
    scan_freq_hz = (speed >> 1) / 10.0 if (speed & 1) else 0.0

    start_raw, end_raw = struct.unpack_from("<HH", frame, 4)
    start_angle = start_raw >> 1          # 1/64 deg
    end_angle = end_raw >> 1
    if end_angle < start_angle:           # wrapped through 360
        start_angle -= _DEG_PER_REV_X64
    step = (end_angle - start_angle) // (n - 1) if n > 1 else 0

    points: List[ScanPoint] = []
    for i in range(n):
        base = _HEADER_LEN + i * 3
        b1, b2 = frame[base + 1], frame[base + 2]
        dist_mm = (b1 >> 2) | (b2 << 6)                 # 14-bit millimetres
        strength = (b2 >> 2) | ((b1 & 0x03) << 6)
        angle_deg = ((start_angle + i * step) / 64.0) % 360.0
        points.append(ScanPoint(angle_deg, dist_mm, strength))
    return scan_freq_hz, points


@register("COIN-D4A", "COIN_D4A", "COIN-D4", "COIN_D4", "COIND4")
class CoinD4A(LidarDriver):
    MODEL_NAME = "COIN-D4A"
    DEFAULT_BAUD = 115200   # CSPC M1C1-Mini family default; confirm on hardware

    def _packets(self):
        buf = bytearray()
        while True:
            chunk = self._t.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                if len(buf) < _HEADER_LEN:
                    break
                if buf[0] != _H0 or buf[1] != _H1:
                    idx = buf.find(_HEADER, 1)
                    if idx < 0:
                        del buf[:-1]        # keep last byte (may be a lone 0xAA)
                        break
                    del buf[:idx]
                    continue
                n = buf[3]
                if n == 0 or n > _MAX_SAMPLES:
                    del buf[0]              # implausible count: resync
                    continue
                frame_len = _HEADER_LEN + n * 3
                if len(buf) < frame_len:
                    break
                frame = bytes(buf[:frame_len])
                if not checksum_ok(frame, n):
                    del buf[0]              # bad checksum: drop one byte and resync
                    continue
                yield parse_frame(frame, n)
                del buf[:frame_len]

    # -- command-driven motor / exposure control --
    def start(self) -> None:
        self._t.write(build_command(CMD_START))

    def stop(self) -> None:
        self._t.write(build_command(CMD_STOP))

    def set_high_exposure(self) -> None:
        self._t.write(build_command(CMD_HIGH_EXPOSURE))

    def set_low_exposure(self) -> None:
        self._t.write(build_command(CMD_LOW_EXPOSURE))
