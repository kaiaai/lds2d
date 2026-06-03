# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Neato XV11 family drivers (host-driven motor): Neato XV11 and Xiaomi LDS01RR.

These sensors have no onboard speed control. The host must spin the motor at a
target rate (~5 Hz / 300 RPM) via PWM, or the unit stops streaming. The motor
loop lives in :class:`~lds2d.drivers._host_motor.HostMotorLidar`; this module
adds the XV11 packet parser, ported from kaiaai/LDS ``LDS_NEATO_XV11.cpp``.

    from lds2d import Lidar
    with Lidar.open("NEATO_XV11", "/dev/serial0", pwm="software", pwm_pin=18) as lidar:
        for scan in lidar.scans():
            ...   # the motor is driven for you; stopped on exit

Packet (22 bytes):
    0xFA  index  speed(u16 LE)  4 x [dist(u16 LE) signal(u16 LE)]  CRC(u16 LE)

Fields, per kaiaai/LDS LDS_NEATO_XV11:
  - ``index`` runs 0xA0..0xF9 (90 packets/scan). The first of the four angles in
    a packet is ``(index - 0xA0) * 4`` degrees; the four samples are +0..+3.
  - ``speed`` is RPM * 64, so ``rpm = speed / 64`` and ``scan_freq_hz = rpm/60``
    (``getCurrentScanFreqHz`` returns ``scan_rpm/60``).
  - Each distance quad's MSB byte holds two flags: INVALID_DATA (bit 7) and
    STRENGTH_WARNING (bit 6) (``BAD_DATA_MASK``). If either is set the distance
    is reported as 0; the C++ also leaves that quad's quality at 0 (it only
    calls processSignalStrength when the flag byte is 0). Otherwise distance is
    the low 14 bits in millimetres (max 0x3FFF = 16383 mm) and quality is the
    16-bit signal-strength word.
  - Checksum (``isValidPacket``): take the ten 16-bit LE words of bytes 0..19,
    fold ``chk = (chk << 1) + word`` over them, then
    ``checksum = ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF`` and compare to the
    little-endian trailer (bytes 20..21).

Scan direction: the Neato XV11 and Xiaomi LDS01RR spin counter-clockwise
(C++ ``cw == false``), so the reported angle is mirrored ``(360 - angle) % 360``.
This is the one protocol difference from the LDS02RR, which overrides
``cw = true`` (no mirroring) and is ported separately in ``xiaomi_lds02rr.py``.
Baud rate (115200), the RPM/64 speed scaling, the flags, the checksum and the
PID/motor constants are identical across the family.

Public helpers ``valid_packet`` / ``parse_packet`` / ``iter_packets`` mirror the
LDS02RR module so callers can parse XV11 frames without a transport.
"""
from __future__ import annotations

from typing import List, Tuple

from ..core import ScanPoint, register
from ._host_motor import HostMotorLidar

_COMMAND = 0xFA
_INDEX_LO = 0xA0
_INDEX_HI = 0xF9
_PACKET_LEN = 22
_N_QUADS = 4
_INVALID_FLAG = 1 << 7
_WARNING_FLAG = 1 << 6
_BAD_DATA_MASK = _INVALID_FLAG | _WARNING_FLAG


def valid_packet(pkt: bytes) -> bool:
    """XV11 checksum: sum the ten 16-bit little-endian words, fold to 15 bits."""
    chk = 0
    for i in range(0, 20, 2):
        chk = (chk << 1) + (pkt[i] | (pkt[i + 1] << 8))
    checksum = ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF
    return checksum == (pkt[20] | (pkt[21] << 8))


def parse_packet(pkt: bytes, cw: bool = False) -> Tuple[float, List[ScanPoint]]:
    """Return ``(scan_freq_hz, points)`` for one validated 22-byte packet.

    ``cw=False`` (Neato XV11, LDS01RR) mirrors each angle to ``(360 - a) % 360``;
    ``cw=True`` keeps angles ascending (the LDS02RR behaviour). A quad whose MSB
    flag byte has INVALID_DATA or STRENGTH_WARNING set reports dist=0, quality=0.
    """
    start_angle = (pkt[1] - _INDEX_LO) * _N_QUADS
    rpm = (pkt[2] | (pkt[3] << 8)) / 64.0
    points: List[ScanPoint] = []
    for q in range(_N_QUADS):
        off = 4 + q * 4
        msb = pkt[off + 1]
        if msb & _BAD_DATA_MASK:
            dist = 0
            quality = 0          # C++ leaves quality at 0 for a flagged quad
        else:
            dist = pkt[off] | ((msb & 0x3F) << 8)
            quality = pkt[off + 2] | (pkt[off + 3] << 8)
        angle = start_angle + q
        if not cw:
            angle = (360 - angle) % 360
        points.append(ScanPoint(float(angle), dist, quality))
    return rpm / 60.0, points


def iter_packets(buf: bytearray):
    """Yield validated 22-byte packets from a bytearray, consuming what it uses.

    On a bad checksum a single byte is dropped so the search resynchronises on
    the next 0xFA, mirroring the C++ ``eState`` find-COMMAND/build-packet loop.
    """
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


class _NeatoXV11Base(HostMotorLidar):
    """Shared host-motor + parser base for the Neato XV11 family.

    PID/motor constants match kaiaai/LDS ``LDS_NEATO_XV11::init``: setpoint
    300 RPM (5 Hz), Kp/Ki/Kd = 3.0e-3 / 1.0e-3 / 0.0 running in RPM, 20 ms
    sample period, initial PWM duty 0.5. (HostMotorLidar runs the PID in RPM,
    which is also the unit of the C++ ``scan_rpm`` PID input, so the gains carry
    over unchanged.)
    """

    DEFAULT_BAUD = 115200
    TARGET_RPM = 300.0          # 5 Hz; PID runs in RPM (kaiaai/LDS NEATO driver)
    PID_KP = 3.0e-3
    PID_KI = 1.0e-3
    PID_KD = 0.0
    INITIAL_DUTY = 0.5
    CW = False                  # Neato XV11 / LDS01RR scan counter-clockwise

    def _decode(self, buf):
        for pkt in iter_packets(buf):
            yield parse_packet(pkt, cw=self.CW)


@register("NEATO_XV11", "NEATO-XV11", "XV11")
class NeatoXV11(_NeatoXV11Base):
    MODEL_NAME = "Neato XV11"


@register("LDS01RR", "LDS-01RR", "XIAOMI_LDS01RR", "XIAOMI-LDS01RR")
class LDS01RR(_NeatoXV11Base):
    MODEL_NAME = "Xiaomi LDS01RR"
