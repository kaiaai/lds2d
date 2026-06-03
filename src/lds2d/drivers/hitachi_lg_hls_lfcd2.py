# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""Hitachi-LG HLS-LFCD2 driver (TurtleBot3 LDS-01).

Ported from kaiaai/LDS ``LDS_HLS_LFCD2.cpp``. The sensor self-spins once it
receives the ASCII boot command ``'b'`` and stops on ``'e'``; it has no onboard
speed control, so :meth:`HLSLFCD2.set_scan_freq` is a no-op for freq <= 0 and
raises otherwise (matching the C++ ``setScanTargetFreqHz``).

Packet (42 bytes, little-endian throughout)::

    off  size  field
    0    1     start_byte   0xFA
    1    1     index        0xA0 + packet_number   (packet_number 0..59)
    2    2     rpm          motor speed; scan_freq_hz = rpm / 60
    4    36    readings[6]  each 6 bytes: intensity(u16), range_mm(u16), reserved(u16)
    40   2     checksum     over bytes [0..39]   (the 20 little-endian words)

There are 60 packets per scan (PACKETS_PER_SCAN), 6 readings each = 360 points.
Each reading is INTENSITY FIRST, then range, then two reserved bytes -- the genuine
ROBOTIS LFCD layout. (The kaiaai C++ ``reading_t`` mistakenly omits the two
reserved bytes, making its packed struct 30 bytes rather than the 42 its parser
actually collects; this port follows the real 42-byte wire format and the kaiaai
field order / checksum, which is what produces a working parser.)

Angle. The C++ emits points clockwise / in reverse order::

    point_index = packet_number * 6 + i        (i = 0..5)
    angle_cw    = 359 - point_index            (+360 if negative)

so packet 0 carries 359, 358, 357, 356, 355, 354 and the last packet carries
5, 4, 3, 2, 1, 0. By default this driver instead emits *ascending* angles equal
to ``point_index`` (packet 0 -> 0..5, last packet -> 354..359): the C++ scan is
clockwise/descending, which never produces the >180 deg DROP that lds2d's
``scans()`` uses to split rotations (a descending ramp drops by 1 deg per point
and the rotation boundary is an *increase* 0 -> 359). Re-keying to ascending
``point_index`` makes the 359 -> 0 wrap land exactly on the rotation boundary so
``scans()`` groups cleanly. This is a monotonic re-ordering of the same points,
not the exact geometric mirror of the C++ angle. Pass ``cw=True`` to
:func:`parse_packet` (or set ``CW=True`` on a subclass) to get the raw,
byte-faithful clockwise C++ angle ``359 - point_index`` instead.

Checksum (``calcChecksum`` in the C++): walk the first 40 bytes as 20 little-
endian 16-bit words, ``chk32 = (chk32 << 1) + word`` for each, then fold:
``checksum = (chk32 & 0x7FFF) + (chk32 >> 15)`` and return ``checksum & 0x7FFF``.

Note on flags: the genuine LFCD protocol stores two status flags in the upper two
bits of the distance word -- bit 15 = invalid measurement, bit 14 = signal-strength
warning (``INVALID_FLAG`` / ``STRENGTH_WARN_FLAG``). The kaiaai C++ uses ``range_mm``
verbatim and never masks them. This port masks the two MSBs out of the reported
distance (so a flagged sample still yields a sane millimetre value, and an invalid
sample is reported as 0 -> ``ScanPoint.valid is False``). With no flags set the
behaviour is identical to kaiaai's.
"""
from __future__ import annotations

import struct
import time
from typing import List, Optional, Tuple

from ..core import LidarDriver, NotSupportedError, ScanPoint, register

START_BYTE = 0xFA
INDEX_BASE = 0xA0
PACKETS_PER_SCAN = 60
READINGS_PER_PACKET = 6
READING_SIZE = 6                          # intensity(u16) + range(u16) + reserved(u16)
PACKET_SIZE = 42
_CHECKSUM_LEN = PACKET_SIZE - 2          # bytes covered by the checksum (40)

INVALID_FLAG = 0x8000                     # bit 15 of the distance word
STRENGTH_WARN_FLAG = 0x4000               # bit 14 of the distance word
_RANGE_MASK = 0x3FFF                      # distance occupies the low 14 bits

_CMD_START = b"b"
_CMD_STOP = b"e"

_TARGET_HZ = 5.0                          # ~300 rpm nominal (getTargetScanFreqHz)


def calc_checksum(data: bytes) -> int:
    """Reproduce LDS_HLS_LFCD2::calcChecksum over ``len(data)`` bytes.

    ``data`` should be the first 40 bytes of the packet (everything but the
    trailing 2-byte checksum field).
    """
    chk32 = 0
    for i in range(len(data) // 2):
        word = data[i * 2] | (data[i * 2 + 1] << 8)
        chk32 = (chk32 << 1) + word
    checksum = (chk32 & 0x7FFF) + (chk32 >> 15)
    return checksum & 0x7FFF


def parse_packet(packet: bytes, cw: bool = False) -> Tuple[float, List[ScanPoint]]:
    """Parse one validated 42-byte packet into ``(scan_freq_hz, points)``.

    The caller is responsible for checksum validation; this only decodes.

    ``cw=False`` (default) emits ascending angles equal to ``point_index`` so
    lds2d's wrap-on-drop ``scans()`` groups rotations cleanly on the 359 -> 0
    wrap (the raw C++ angle is clockwise/descending and never triggers that
    split). ``cw=True`` emits the raw, byte-faithful clockwise C++ angle
    ``359 - point_index``. The point list is always in the sensor's transmission
    order (ascending ``point_index`` within the packet).
    """
    packet_number = packet[1] - INDEX_BASE
    (rpm,) = struct.unpack_from("<H", packet, 2)
    points: List[ScanPoint] = []
    for i in range(READINGS_PER_PACKET):
        # reading: intensity(u16), range(u16), reserved(u16) -- intensity FIRST
        intensity, raw_range = struct.unpack_from("<HH", packet, 4 + i * READING_SIZE)
        invalid = bool(raw_range & INVALID_FLAG)
        range_mm = 0 if invalid else (raw_range & _RANGE_MASK)
        point_index = packet_number * READINGS_PER_PACKET + i
        if cw:
            angle_deg = float((359 - point_index) % 360)
        else:
            angle_deg = float(point_index % 360)
        points.append(ScanPoint(angle_deg, range_mm, intensity & 0xFF))
    return rpm / 60.0, points


def _find_packet_start(buf: bytearray, start: int = 1) -> int:
    """Index of the next plausible 0xFA + valid-index pair, or -1."""
    for i in range(start, len(buf) - 1):
        if buf[i] == START_BYTE and INDEX_BASE <= buf[i + 1] <= INDEX_BASE + PACKETS_PER_SCAN - 1:
            return i
    return -1


@register("HLS-LFCD2", "HLS_LFCD2", "LFCD2", "LDS-01", "LDS01")
class HLSLFCD2(LidarDriver):
    """Hitachi-LG HLS-LFCD2 / TurtleBot3 LDS-01.

    Self-spinning: ``start()`` writes ``'b'`` and ``stop()`` writes ``'e'``,
    exactly as the C++ ``enableMotor`` does. NEEDS_MOTOR stays False because
    the host does not PWM the motor; the sensor runs its own.
    """

    MODEL_NAME = "Hitachi-LG HLS-LFCD2"
    DEFAULT_BAUD = 230400
    CW = False                  # emit CCW angles so scans() groups (see parse_packet)

    def _packets(self):
        buf = bytearray()
        while True:
            chunk = self._t.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= PACKET_SIZE:
                if not (buf[0] == START_BYTE
                        and INDEX_BASE <= buf[1] <= INDEX_BASE + PACKETS_PER_SCAN - 1):
                    idx = _find_packet_start(buf)
                    if idx < 0:
                        del buf[:-1]        # keep last byte; might be a lone 0xFA
                        break
                    del buf[:idx]
                    if len(buf) < PACKET_SIZE:
                        break
                packet = bytes(buf[:PACKET_SIZE])
                (recv_chk,) = struct.unpack_from("<H", packet, _CHECKSUM_LEN)
                if calc_checksum(packet[:_CHECKSUM_LEN]) != recv_chk:
                    del buf[0]              # bad checksum: drop one byte and resync
                    continue
                yield parse_packet(packet, cw=self.CW)
                del buf[:PACKET_SIZE]

    # -- motor control (ASCII boot/halt commands) --
    def start(self) -> None:
        self._t.write(_CMD_START)

    def stop(self) -> None:
        self._t.write(_CMD_STOP)

    def set_scan_freq(self, hz: float) -> None:
        # The HLS-LFCD2 has no speed control: only "leave it alone" (hz <= 0)
        # is accepted, mirroring the C++ setScanTargetFreqHz return code.
        if hz > 0:
            raise NotSupportedError(
                f"{self.MODEL_NAME} has no speed control (target is ~{_TARGET_HZ} Hz)")

    def get_scan_freq(self, listen_s: float = 1.0) -> Optional[float]:
        """Listen briefly and return scan_freq_hz from a packet's rpm, or None.

        Mirrors getCurrentScanFreqHz() = motor_rpm / 60 in the C++.
        """
        deadline = time.monotonic() + listen_s
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._t.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= PACKET_SIZE:
                if not (buf[0] == START_BYTE
                        and INDEX_BASE <= buf[1] <= INDEX_BASE + PACKETS_PER_SCAN - 1):
                    idx = _find_packet_start(buf)
                    if idx < 0:
                        del buf[:-1]
                        break
                    del buf[:idx]
                    if len(buf) < PACKET_SIZE:
                        break
                packet = bytes(buf[:PACKET_SIZE])
                (recv_chk,) = struct.unpack_from("<H", packet, _CHECKSUM_LEN)
                if calc_checksum(packet[:_CHECKSUM_LEN]) != recv_chk:
                    del buf[0]
                    continue
                (rpm,) = struct.unpack_from("<H", packet, 2)
                return rpm / 60.0
        return None
