# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""3irobotix Delta-2B / 2D / 2G and LDS08RR drivers — host-driven motor.

These are siblings of the Delta-2A: same 0xAA packet framing, same host-driven
MOT_EN PWM motor (no onboard speed control), same PID. In the kaiaai/LDS C++ they
all subclass ``LDS_DELTA_2A_115200`` (LDS08RR via DELTA_2D), overriding only a
handful of getters. We therefore reuse ``decode_delta`` from the Delta-2A module
and ``HostMotorLidar`` for the motor loop, varying just the per-model constants.

Packet format (identical to Delta-2A, big-endian multibyte fields):
  0xAA  packet_length(u16)  ver=0x01  type=0x61  data_type  data_length(u16)
  scan_freq_x20  offset_angle(i16)  start_angle_x100(u16)  N x [quality dist_x4(u16)]  checksum(u16)
data_type 0xAD = speed + measurements, 0xAE = speed only. scan_freq_hz = byte/20,
distance_mm = dist_x4/4, angle = start_angle + idx*360/(packets_per_scan*N).
Checksum is the 16-bit sum of every byte before the 2-byte checksum.

Per-model deltas vs Delta-2A (from the C++ getters):
  Delta-2B : getSerialBaudRate() = 230400   (everything else inherited)
  Delta-2D : get_max_data_sample_count() = 24 (bounds only; same parser output)
  Delta-2G : get_packets_per_scan() = 15     (changes per-sample angle step)
  LDS08RR  : subclasses Delta-2D (24 samples, 115200 baud)
All four keep get_default_scan_freq_hz()=6 Hz and PID Kp=0.3, Ki=0.1 (run in Hz
by the C++; HostMotorLidar runs the PID in RPM, so the gains are divided by 60).
"""
from __future__ import annotations

from ..core import register
from ._host_motor import HostMotorLidar
from .threeirobotix_delta_2a import Delta2A, decode_delta


class _DeltaVariant(HostMotorLidar):
    """Shared host-motor wiring for the Delta-2A-derived variants.

    Mirrors the C++ ``LDS_DELTA_2A_115200`` base: 6 Hz target, PID Kp=0.3/Ki=0.1
    (Hz in C++ -> /60 here for RPM), 0.6 initial PWM duty. Subclasses set only the
    baud, model name and ``PACKETS_PER_SCAN``.
    """

    DEFAULT_BAUD = Delta2A.DEFAULT_BAUD          # 115200 unless overridden
    TARGET_RPM = Delta2A.TARGET_RPM              # 6 Hz
    PID_KP = Delta2A.PID_KP                       # 0.3 / 60
    PID_KI = Delta2A.PID_KI                       # 0.1 / 60
    PID_KD = Delta2A.PID_KD
    INITIAL_DUTY = Delta2A.INITIAL_DUTY           # 0.6
    PACKETS_PER_SCAN = Delta2A.PACKETS_PER_SCAN   # 16

    def _decode(self, buf):
        yield from decode_delta(buf, self.PACKETS_PER_SCAN)


@register("DELTA-2B", "DELTA_2B", "DELTA2B", "3IROBOTIX_DELTA_2B")
class Delta2B(_DeltaVariant):
    MODEL_NAME = "3irobotix Delta-2B"
    DEFAULT_BAUD = 230400          # the only C++ override: getSerialBaudRate()
    PACKETS_PER_SCAN = 16


@register("DELTA-2D", "DELTA_2D", "DELTA2D", "3IROBOTIX_DELTA_2D")
class Delta2D(_DeltaVariant):
    MODEL_NAME = "3irobotix Delta-2D"
    DEFAULT_BAUD = 115200
    PACKETS_PER_SCAN = 16          # max_data_sample_count=24 affects bounds only


@register("DELTA-2G", "DELTA_2G", "DELTA2G", "3IROBOTIX_DELTA_2G")
class Delta2G(_DeltaVariant):
    MODEL_NAME = "3irobotix Delta-2G"
    DEFAULT_BAUD = 115200
    PACKETS_PER_SCAN = 15          # C++ override: get_packets_per_scan()=15


@register("LDS08RR", "LDS-08RR", "LDS_08RR", "3IROBOTIX_LDS08RR")
class Lds08RR(_DeltaVariant):
    MODEL_NAME = "LDS08RR"         # C++: "3irobotics LDS08RR"; subclasses Delta-2D
    DEFAULT_BAUD = 115200
    PACKETS_PER_SCAN = 16          # via Delta-2D: 24 samples, 16 packets/scan
