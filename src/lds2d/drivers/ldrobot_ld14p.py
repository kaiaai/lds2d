# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""LDROBOT LD14P driver: parse scan packets and control the motor over UART.

Packet (47 bytes): 0x54 0x2C  speed(u16)  start_angle(u16)
                    12 x [dist(u16) intensity(u8)]  end_angle(u16) timestamp(u16)  CRC8
Angles are in centi-degrees; speed is in degrees/second.

Motor command frame (8 bytes): 0x54 cmd 0x04  4-byte payload  CRC8.
Command bytes and CRC match the kaiaai/LDS library.
"""
from __future__ import annotations

import struct
import time
from typing import List, Optional, Tuple

from ..core import LidarDriver, ScanPoint, register
from ..crc import crc8

_HEADER = 0x54
_VER_LEN = 0x2C
_POINTS = 12
_PACKET_SIZE = 47

_CMD_START = 0xA0
_CMD_STOP = 0xA1
_CMD_SET_SPEED = 0xA2

MIN_HZ = 2.0
MAX_HZ = 8.0


def build_command(cmd: int, payload: bytes = b"") -> bytes:
    """Build an 8-byte motor command frame with its CRC."""
    payload = (payload + b"\x00\x00\x00\x00")[:4]
    body = bytes([_HEADER, cmd, 0x04]) + payload
    return body + bytes([crc8(body)])


def parse_packet(packet: bytes) -> Tuple[float, List[ScanPoint]]:
    """Parse one validated 47-byte packet into (scan_freq_hz, points)."""
    speed_dps, start_a = struct.unpack_from("<HH", packet, 2)
    end_a, _ts = struct.unpack_from("<HH", packet, 6 + _POINTS * 3)
    span = (end_a - start_a) % 36000
    step = span / (_POINTS - 1)
    points = []
    for i in range(_POINTS):
        dist_mm, intensity = struct.unpack_from("<HB", packet, 6 + i * 3)
        angle_deg = ((start_a + i * step) % 36000) / 100.0
        points.append(ScanPoint(angle_deg, dist_mm, intensity))
    return speed_dps / 360.0, points


def _find_header(buf: bytearray) -> int:
    for i in range(1, len(buf) - 1):
        if buf[i] == _HEADER and buf[i + 1] == _VER_LEN:
            return i
    return -1


@register("LD14P", "LDROBOT_LD14P")
class LD14P(LidarDriver):
    MODEL_NAME = "LDROBOT LD14P"
    DEFAULT_BAUD = 230400

    def _packets(self):
        buf = bytearray()
        while True:
            chunk = self._t.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= _PACKET_SIZE:
                if buf[0] != _HEADER or buf[1] != _VER_LEN:
                    idx = _find_header(buf)
                    if idx < 0:
                        del buf[:-1]
                        break
                    del buf[:idx]
                    if len(buf) < _PACKET_SIZE:
                        break
                packet = bytes(buf[:_PACKET_SIZE])
                if crc8(packet[:-1]) != packet[-1]:
                    del buf[0]      # bad CRC: drop one byte and resync
                    continue
                yield parse_packet(packet)
                del buf[:_PACKET_SIZE]

    # -- motor control --
    def start(self) -> None:
        self._t.write(build_command(_CMD_START))

    def stop(self) -> None:
        self._t.write(build_command(_CMD_STOP))

    def set_scan_freq(self, hz: float) -> None:
        if not (MIN_HZ <= hz <= MAX_HZ):
            raise ValueError(f"scan rate must be {MIN_HZ}-{MAX_HZ} Hz")
        deg_per_sec = int(round(hz * 360))
        self._t.write(build_command(_CMD_SET_SPEED, struct.pack("<H", deg_per_sec)))

    def get_scan_freq(self, listen_s: float = 1.0) -> Optional[float]:
        """Listen briefly and return the reported scan rate, or None."""
        deadline = time.monotonic() + listen_s
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._t.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            i = 0
            while i + 4 <= len(buf):
                if buf[i] == _HEADER and buf[i + 1] == _VER_LEN:
                    deg_per_sec = struct.unpack_from("<H", buf, i + 2)[0]
                    if 0 < deg_per_sec < 36000:
                        return deg_per_sec / 360.0
                i += 1
            del buf[:-1]
        return None


# --- the rest of the LDROBOT LD-series ---------------------------------------
# The LD06, LD19 and STL19P stream the *identical* 47-byte packet at the same
# 230400 baud (in kaiaai/LDS the LD06/STL19P classes subclass LD19, which shares
# LD14P's scan_packet_t byte-for-byte). So they are the same parser under a
# different name. They are typically free-running at ~10 Hz; the inherited 0x54
# motor command frames match the LD14P's but are not hardware-verified on these.

@register("LD19", "LDROBOT_LD19")
class LD19(LD14P):
    MODEL_NAME = "LDROBOT LD19"


@register("LD06", "LDROBOT_LD06")
class LD06(LD19):
    MODEL_NAME = "LDROBOT LD06"


@register("STL19P", "LDROBOT_STL19P")
class STL19P(LD19):
    MODEL_NAME = "LDROBOT STL19P"
