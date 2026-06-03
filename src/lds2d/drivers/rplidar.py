# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""SLAMTEC RPLIDAR A1 / C1 driver — command-started, self-spinning LiDAR.

Ported faithfully from kaiaai/LDS ``LDS_RPLIDAR_A1.cpp`` (and ``..._C1.cpp``,
which subclasses A1). The host kicks the device off with a SCAN request, the
device replies with a 7-byte response descriptor, then streams 5-byte
measurement nodes until told to stop.

Request / response framing
--------------------------
* Commands are ``0xA5 <cmd>`` (two bytes, no payload for SCAN/STOP). SCAN is
  ``0xA5 0x20``, FORCE_SCAN ``0xA5 0x21``, STOP ``0xA5 0x25``. Commands with a
  payload append ``size`` and an XOR checksum, but none of ours carry a payload.
* The scan response descriptor is 7 bytes::

      0xA5 0x5A  size:30 | subType:2 (u32 LE)  type(u8)

  We validate the two sync bytes ``0xA5 0x5A`` and the trailing ``type`` byte
  ``0x81`` (ANS_TYPE_MEAS); the middle 4 bytes are skipped (size/subtype).

Measurement node (5 bytes, little-endian)
-----------------------------------------
::

    byte0  sync_quality   : syncbit(bit0), syncbit_inverse(bit1), quality(bits2..7)
    byte1  angle_lsb      : check_bit(bit0)=1, angle low bits
    byte2  angle_msb
    byte3  distance_lsb
    byte4  distance_msb

* byte0 check (C++): ``((b0 >> 1) ^ b0) & 0x01`` must be 1 — the syncbit (bit0)
  and its inverse (bit1) must differ. byte1 check: ``b1 & 0x01`` must be 1.
* ``angle_q6 = (u16(byte2<<8 | byte1)) >> 1`` ; ``angle_deg = angle_q6 / 64``.
* ``distance_q2 = u16(byte4<<8 | byte3)`` ; ``dist_mm = distance_q2 / 4``.
* ``quality = byte0 >> 2``.
* ``scan_completed = byte0 & 0x01`` (the syncbit) — marks the FIRST node of a
  fresh 360 deg scan.

Scan frequency is not encoded in the node; the C++ derives it from the wall-clock
period between successive start-flag nodes (``markScanTime`` / ``millis()``). We
have no device clock in the parser, so each node is yielded with
``scan_freq_hz = 0.0`` (best effort) and scan boundaries are taken from the
start-flag bit, exactly as the C++ ``postScanPoint(..., scan_completed)`` does.

Distances are emitted in millimetres rounded to ``int`` to satisfy the
``ScanPoint`` contract; quality 0-63 maps straight from the 6-bit field.

Classes
-------
``RPLIDAR_A1`` (115200 baud, internal motor enabled at start) and ``RPLIDAR_C1``
(460800 baud, fully internal motor control). Both send the SCAN command on
``start()`` and STOP on ``stop()``.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

from ..core import LidarDriver, ScanPoint, register

# Command framing
_CMD_SYNC_BYTE = 0xA5
_CMD_SCAN = 0x20
_CMD_FORCE_SCAN = 0x21
_CMD_STOP = 0x25

# Response descriptor
_ANS_SYNC_BYTE1 = 0xA5
_ANS_SYNC_BYTE2 = 0x5A
_ANS_TYPE_MEAS = 0x81
_DESCRIPTOR_SIZE = 7

# Node layout
_NODE_SIZE = 5
_RESP_MEAS_SYNCBIT = 0x01       # byte0 bit0
_RESP_MEAS_CHECKBIT = 0x01      # byte1 bit0
_RESP_MEAS_QUALITY_SHIFT = 2
_RESP_MEAS_ANGLE_SHIFT = 1


def build_scan_command(force: bool = False) -> bytes:
    """Build the 2-byte SCAN (or FORCE_SCAN) request: ``0xA5 0x20`` / ``0xA5 0x21``."""
    return bytes([_CMD_SYNC_BYTE, _CMD_FORCE_SCAN if force else _CMD_SCAN])


def build_stop_command() -> bytes:
    """Build the 2-byte STOP request: ``0xA5 0x25``."""
    return bytes([_CMD_SYNC_BYTE, _CMD_STOP])


def node_ok(node: bytes) -> bool:
    """Validate a 5-byte node's two check bits (C++ ``waitScanDot`` cases 0 & 1)."""
    if len(node) < _NODE_SIZE:
        return False
    b0 = node[0]
    b1 = node[1]
    # byte0: syncbit and syncbit_inverse must differ. The C++ waitScanDot accepts
    # byte0 when ((b0>>1) ^ b0) & 1 == 1 (the two low bits differ); reject otherwise.
    if not (((b0 >> 1) ^ b0) & 0x01):
        return False
    # byte1: low bit (check bit) must be set
    if not (b1 & _RESP_MEAS_CHECKBIT):
        return False
    return True


def parse_node(node: bytes) -> Tuple[ScanPoint, bool]:
    """Parse a validated 5-byte node into ``(ScanPoint, scan_completed)``.

    ``scan_completed`` True means this node is the first of a new 360 deg scan.
    """
    b0 = node[0]
    angle_word = node[1] | (node[2] << 8)       # u16 LE: angle_q6_checkbit
    distance_q2 = node[3] | (node[4] << 8)      # u16 LE: distance_q2

    angle_q6 = angle_word >> _RESP_MEAS_ANGLE_SHIFT
    angle_deg = angle_q6 * 0.015625             # / 64
    dist_mm = int(distance_q2 * 0.25)           # / 4, truncated like the C++ float->use
    quality = b0 >> _RESP_MEAS_QUALITY_SHIFT
    scan_completed = bool(b0 & _RESP_MEAS_SYNCBIT)
    return ScanPoint(angle_deg, dist_mm, quality), scan_completed


@register("RPLIDAR-A1", "RPLIDAR_A1", "A1")
class RPLIDAR_A1(LidarDriver):
    """SLAMTEC RPLIDAR A1: command-started, 115200 baud, internal motor."""

    MODEL_NAME = "RPLIDAR A1"
    DEFAULT_BAUD = 115200

    FORCE_SCAN = False

    def __init__(self, transport):
        super().__init__(transport)
        self._started = False

    # -- motor / scan control --
    def start(self) -> None:
        """Send the SCAN request that starts streaming."""
        self._t.write(build_scan_command(self.FORCE_SCAN))
        self._started = True

    def stop(self) -> None:
        """Send the STOP request (``0xA5 0x25``)."""
        self._t.write(build_stop_command())
        self._started = False

    # -- internal helpers --
    @staticmethod
    def _find_descriptor(buf: bytearray) -> int:
        """Return the index of the 0xA5 0x5A descriptor sync, or -1."""
        for i in range(len(buf) - 1):
            if buf[i] == _ANS_SYNC_BYTE1 and buf[i + 1] == _ANS_SYNC_BYTE2:
                return i
        return -1

    def _skip_descriptor(self, buf: bytearray) -> bool:
        """Locate & consume the 7-byte response descriptor.

        Returns True once a valid MEAS descriptor has been consumed. A wrong-type
        descriptor (e.g. a device-health response) is dropped and the search
        continues through whatever is already buffered, mirroring the C++
        waitResponseHeader loop that resyncs on the next 0xA5 0x5A.
        """
        while True:
            idx = self._find_descriptor(buf)
            if idx < 0:
                # keep the last byte in case 0xA5 straddles a read boundary
                if len(buf) > 1:
                    del buf[:-1]
                return False
            if len(buf) - idx < _DESCRIPTOR_SIZE:
                del buf[:idx]           # align, wait for the rest
                return False
            descriptor = bytes(buf[idx:idx + _DESCRIPTOR_SIZE])
            del buf[:idx + _DESCRIPTOR_SIZE]
            # descriptor[6] is the 'type' byte; must be ANS_TYPE_MEAS (0x81)
            if descriptor[6] == _ANS_TYPE_MEAS:
                return True
            # not a measurement stream; keep scanning the remaining buffer

    def _packets(self):
        # The Arduino start() also queries device info/health; for our serial
        # transport the meaningful trigger is the SCAN command + descriptor skip.
        if not self._started:
            self.start()

        buf = bytearray()
        got_descriptor = False

        while True:
            chunk = self._t.read(256)
            if not chunk:
                continue
            buf.extend(chunk)

            if not got_descriptor:
                if not self._skip_descriptor(buf):
                    continue
                got_descriptor = True

            while len(buf) >= _NODE_SIZE:
                node = bytes(buf[:_NODE_SIZE])
                if not node_ok(node):
                    del buf[0]          # bad check bits: drop one byte and resync
                    continue
                point, _scan_completed = parse_node(node)
                del buf[:_NODE_SIZE]
                # scan_freq is timing-derived in the C++; unavailable here -> 0.0
                yield 0.0, [point]


@register("RPLIDAR-C1", "RPLIDAR_C1", "C1")
class RPLIDAR_C1(RPLIDAR_A1):
    """SLAMTEC RPLIDAR C1: same node protocol, 460800 baud, fully internal motor."""

    MODEL_NAME = "RPLIDAR C1"
    DEFAULT_BAUD = 460800
