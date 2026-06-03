# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""YDLIDAR family driver: X4, X2/X2L, X3, X3 PRO, X4 PRO, SCL, T-mini.

A faithful port of the kaiaai/LDS YDLIDAR classes (themselves based on the
EAIBOT / YDLIDAR lidarCar Arduino code). One streaming parser implements the
shared "PH" packet; the variants subclass it and override only what differs.

Packet format ("PH" / point-cloud package)
-------------------------------------------
The sensor streams variable-length packages. Each has a 10-byte header
(``PACKAGE_PAID_BYTES``) followed by ``LSN`` samples::

    byte 0 : 0xAA              PH low  byte  (PH = 0x55AA)
    byte 1 : 0x55              PH high byte
    byte 2 : CT                package type; bit0 == 1 -> ring/scan start
    byte 3 : LSN               sample count in this package
    byte 4 : FSA low           first-sample angle, low  byte (bit0 = check bit)
    byte 5 : FSA high          first-sample angle, high byte
    byte 6 : LSA low           last-sample angle,  low  byte (bit0 = check bit)
    byte 7 : LSA high          last-sample angle,  high byte
    byte 8 : CS low            package checksum, low  byte
    byte 9 : CS high           package checksum, high byte
    then   : LSN samples (2 bytes each for X4/X2/X3/T-mini, 3 bytes for SCL)

Angles are *deg x 64*. The raw 16-bit FSA/LSA fields carry a check bit in the
low bit, so the real angle is ``raw >> 1``. The per-sample interpolation step is
``IntervalSampleAngle = (LSA - FSA) / (LSN - 1)`` (in deg x 64), with a wrap fix
when the package crosses 0 deg (23040 == 360 x 64). Each sample's angle is
``FSA + i * IntervalSampleAngle + AngleCorrectForDistance`` (the triangulation
correction is an ``atan`` term; T-mini, being a ToF sensor, applies none).

Distance:
  * X4 / X2 / X2L / X3 / X3 PRO: ``dist_mm = raw_u16 / 4``.
  * X4 PRO: the 2 sample bytes pack ``[interference:2][dist_low:6]`` then
    ``dist_high``; ``dist_mm = (dist_high << 6) + dist_low`` (already mm), and
    quality is the 2-bit interference flag.
  * SCL: 3-byte samples ``[intensity][dist_low][dist_high]`` with
    ``dist_mm = (dist_low >> 2) + (dist_high << 6)`` and intensity as quality.
  * T-mini (ToF): ``dist_mm = raw_u16`` (no /4, no angle correction).

Checksum: XOR of all 16-bit words in the package — ``PH`` ^ ``FSA_raw`` ^
(each sample word) ^ ``(CT | LSN<<8)`` ^ ``LSA_raw`` — compared against the CS
field. For SCL's 3-byte samples the first byte is XORed on its own and bytes 2-3
as a 16-bit word.

Scan frequency: on a ring-start package, ``scan_freq_hz = (CT >> 1) * 0.1``.

Motors
------
The X4 is *command driven*: ``start()`` / ``stop()`` send 0xA5 device-control
frames over serial (CMD_SCAN / CMD_FORCE_STOP). X2/X2L/X3/X3 PRO/SCL/X4 PRO/
T-mini are *self-spinning* once powered (the C++ only toggles a motor GPIO/PWM
pin, never a scan command), so they need no software start. SCL and X4 PRO
additionally regulate speed via a host PWM/PID in the C++, but they still stream
without it; speed regulation is out of scope for this serial-only driver.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from ..core import LidarDriver, ScanPoint, register

PH_LO = 0xAA
PH_HI = 0x55
PH = 0x55AA
PACKAGE_PAID_BYTES = 10            # 10-byte package header
PACKAGE_SAMPLE_MAX_LENGTH = 40     # LSN ceiling, per the C++
NODE_DEFAULT_QUALITY = 10
ANGLE_DEG64 = 23040                # 360 deg * 64

# 0xA5 device-control command frame bytes (X4 only)
CMD_SYNC_BYTE = 0xA5
CMD_STOP = 0x65
CMD_SCAN = 0x60
CMD_FORCE_STOP = 0x00


def _interval_sample_angle(fsa: int, lsa: int, n: int, last: float,
                           wrap_hi: int, wrap_lo: int) -> Tuple[float, float]:
    """IntervalSampleAngle (deg x 64) and the updated 'last package' value.

    ``fsa``/``lsa`` are already check-bit-shifted (>> 1). Mirrors the C++ wrap
    handling: if the package crosses 0 deg, add a full turn before dividing.
    """
    if n <= 1:
        return 0.0, last
    if lsa < fsa:
        if fsa > wrap_hi and lsa < wrap_lo:
            interval = float(ANGLE_DEG64 + lsa - fsa) / (n - 1)
            return interval, interval
        return last, last          # noisy wrap: reuse the previous step
    interval = float(lsa - fsa) / (n - 1)
    return interval, interval


class _YDLidarBase(LidarDriver):
    """Streaming PH-package parser shared by the whole YDLIDAR family.

    Subclasses override sample size / distance / angle-correction hooks. The
    parser keeps a rolling byte buffer, resynchronises on a bad header or
    checksum by dropping one byte, and yields one ``(scan_freq_hz, points)``
    tuple per validated package.
    """

    MODEL_NAME = "YDLIDAR"
    DEFAULT_BAUD = 128000

    SAMPLE_BYTES = 2               # bytes per distance sample
    # Wrap thresholds in deg x 64 (X4 uses raw 17280/5760; SCL uses 270*64/90*64,
    # which are identical numbers — 270*64 == 17280, 90*64 == 5760).
    WRAP_HI = 17280
    WRAP_LO = 5760

    def __init__(self, transport):
        super().__init__(transport)
        self._interval_last = 0.0   # IntervalSampleAngle_LastPackage

    # -- per-model hooks ---------------------------------------------------
    def _sample_distance_mm(self, sample: bytes) -> int:
        """Distance in mm from one raw sample (default: triangulation /4)."""
        raw = sample[0] | (sample[1] << 8)
        return raw // 4

    def _sample_quality(self, sample: bytes) -> int:
        return NODE_DEFAULT_QUALITY

    def _correction_distance(self, sample: bytes, dist_mm: int) -> float:
        """Full-precision distance (mm, float) fed to the atan correction.

        The C++ X4 path uses ``node.distance_q2 * 0.25`` -- i.e. the raw 16-bit
        sample scaled by 1/4 *without* truncating to whole millimetres -- so the
        sub-mm fraction must be preserved here (truncating ``dist_mm`` first
        would shift the correction by up to ~2 deg at short range).
        """
        raw = sample[0] | (sample[1] << 8)
        return raw * 0.25

    def _angle_correction_deg64(self, sample: bytes, dist_mm: int) -> float:
        """Triangulation angle correction (deg x 64). Override for ToF/SCL."""
        if dist_mm == 0:
            return 0.0
        d = self._correction_distance(sample, dist_mm)
        return float(int(math.atan(((21.8 * (155.3 - d)) / 155.3) / d) * 3666.93))

    def _sample_checksum_word(self, sample: bytes) -> int:
        """16-bit word folded into the XOR checksum for one sample."""
        return sample[0] | (sample[1] << 8)

    # -- packet engine -----------------------------------------------------
    def _try_parse(self, buf: bytearray):
        """Parse one package from the front of ``buf``.

        Returns ``(consumed, result)`` where ``result`` is ``(freq, points)`` or
        ``None`` (header/checksum reject). ``consumed`` is the number of leading
        bytes to drop; 0 means "need more data".
        """
        if len(buf) < PACKAGE_PAID_BYTES:
            return 0, None
        if buf[0] != PH_LO or buf[1] != PH_HI:
            return 1, None          # not a header here: drop one byte, resync

        ct = buf[2]
        lsn = buf[3]
        if lsn == 0 or lsn > PACKAGE_SAMPLE_MAX_LENGTH:
            return 1, None
        # Check bits: low bit of FSA low and LSA low must be set.
        if not (buf[4] & 0x01) or not (buf[6] & 0x01):
            return 1, None

        total = PACKAGE_PAID_BYTES + lsn * self.SAMPLE_BYTES
        if len(buf) < total:
            return 0, None          # wait for the samples

        pkt = bytes(buf[:total])
        fsa_raw = pkt[4] | (pkt[5] << 8)
        lsa_raw = pkt[6] | (pkt[7] << 8)
        check = pkt[8] | (pkt[9] << 8)

        # Reproduce the C++ XOR checksum exactly.
        csum = PH
        csum ^= fsa_raw
        for i in range(lsn):
            off = PACKAGE_PAID_BYTES + i * self.SAMPLE_BYTES
            csum ^= self._sample_checksum_word(pkt[off:off + self.SAMPLE_BYTES])
        csum ^= (ct | (lsn << 8))   # SampleNumlAndCTCal
        csum ^= lsa_raw             # LastSampleAngleCal
        csum &= 0xFFFF
        if csum != check:
            return 1, None          # bad checksum: drop one byte and resync

        fsa = fsa_raw >> 1
        lsa = lsa_raw >> 1
        interval, self._interval_last = _interval_sample_angle(
            fsa, lsa, lsn, self._interval_last, self.WRAP_HI, self.WRAP_LO)

        scan_start = (ct & 0x01) == 0x01
        scan_freq_hz = (ct >> 1) * 0.1 if scan_start else 0.0

        points: List[ScanPoint] = []
        for i in range(lsn):
            off = PACKAGE_PAID_BYTES + i * self.SAMPLE_BYTES
            sample = pkt[off:off + self.SAMPLE_BYTES]
            dist_mm = self._sample_distance_mm(sample)
            quality = self._sample_quality(sample)
            acd = self._angle_correction_deg64(sample, dist_mm)
            angle64 = fsa + interval * i + acd
            # Normalise into [0, 23040) the way the C++ does (one turn at a time).
            if angle64 < 0:
                angle64 += ANGLE_DEG64
            elif angle64 > ANGLE_DEG64:
                angle64 -= ANGLE_DEG64
            angle_deg = (angle64 / 64.0) % 360.0
            points.append(ScanPoint(angle_deg, dist_mm, quality))
        return total, (scan_freq_hz, points)

    def _packets(self):
        buf = bytearray()
        while True:
            chunk = self._t.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                consumed, result = self._try_parse(buf)
                if consumed == 0:
                    break           # need more bytes
                del buf[:consumed]
                if result is not None:
                    yield result


# ---------------------------------------------------------------------------
# X4 — the base part. Command-driven over serial (0xA5 device control).
# ---------------------------------------------------------------------------

def build_command(cmd: int) -> bytes:
    """Build a 2-byte 0xA5 device-control frame (no-payload commands only)."""
    return bytes([CMD_SYNC_BYTE, cmd & 0xFF])


@register("YDLIDAR-X4", "YDLIDAR_X4", "X4")
class YDLidarX4(_YDLidarBase):
    MODEL_NAME = "YDLIDAR X4"
    DEFAULT_BAUD = 128000

    def start(self) -> None:
        self._t.write(build_command(CMD_FORCE_STOP))
        self._t.write(build_command(CMD_SCAN))

    def stop(self) -> None:
        self._t.write(build_command(CMD_STOP))


# ---------------------------------------------------------------------------
# X2 / X2L / X3 / X3 PRO — same PH parser, self-spinning, lower baud.
# ---------------------------------------------------------------------------

class _SelfSpinningYDLidar(_YDLidarBase):
    """YDLIDAR variants that spin up on power with no scan command."""

    def start(self) -> None:
        pass        # self-spinning: nothing to send

    def stop(self) -> None:
        pass


@register("YDLIDAR-X2", "YDLIDAR_X2", "X2",
          "YDLIDAR-X2L", "YDLIDAR_X2L", "X2L",
          "YDLIDAR-X2_X2L", "X2_X2L")
class YDLidarX2X2L(_SelfSpinningYDLidar):
    MODEL_NAME = "YDLIDAR X2/X2L"
    DEFAULT_BAUD = 115200


@register("YDLIDAR-X3", "YDLIDAR_X3", "X3")
class YDLidarX3(YDLidarX2X2L):
    MODEL_NAME = "YDLIDAR X3"
    DEFAULT_BAUD = 115200


@register("YDLIDAR-X3-PRO", "YDLIDAR_X3_PRO", "X3-PRO", "X3_PRO")
class YDLidarX3Pro(YDLidarX3):
    MODEL_NAME = "YDLIDAR X3 PRO"
    DEFAULT_BAUD = 115200


# ---------------------------------------------------------------------------
# X4 PRO — separate base in the C++. Self-spinning PWM. Packed 6-bit distance,
# distance already in mm (no /4), interference bits as quality.
# ---------------------------------------------------------------------------

@register("YDLIDAR-X4-PRO", "YDLIDAR_X4_PRO", "X4-PRO", "X4_PRO")
class YDLidarX4Pro(_SelfSpinningYDLidar):
    MODEL_NAME = "YDLIDAR X4 PRO"
    DEFAULT_BAUD = 128000
    SAMPLE_BYTES = 2

    def _sample_distance_mm(self, sample: bytes) -> int:
        # byte0: [interference:2][distance_low:6], byte1: distance_high.
        dist_low = sample[0] >> 2
        dist_high = sample[1]
        return (dist_high << 6) + dist_low

    def _sample_quality(self, sample: bytes) -> int:
        return sample[0] & 0x03     # packageInterference

    def _correction_distance(self, sample: bytes, dist_mm: int) -> float:
        # The C++ uses node.distance directly (already integer mm, no /4).
        return float(dist_mm)


# ---------------------------------------------------------------------------
# SCL — 3-byte samples, intensity + 14-bit distance, separate angle correction.
# ---------------------------------------------------------------------------

@register("YDLIDAR-SCL", "YDLIDAR_SCL", "SCL")
class YDLidarSCL(_SelfSpinningYDLidar):
    MODEL_NAME = "YDLIDAR SCL"
    DEFAULT_BAUD = 115200
    SAMPLE_BYTES = 3
    WRAP_HI = 270 * 64
    WRAP_LO = 90 * 64

    def _sample_distance_mm(self, sample: bytes) -> int:
        # [intensity][distance_lsb][distance_msb]
        return (sample[1] >> 2) + (sample[2] << 6)

    def _sample_quality(self, sample: bytes) -> int:
        # The C++ computes ``quality_flag = distance_lsb && 0x03`` (a logical AND
        # — almost certainly a typo for bitwise ``&``) and posts *that* as the
        # point quality, so it is 1 when distance_lsb != 0 else 0. We reproduce
        # the shipped behaviour faithfully; ``sample[0]`` holds the real
        # intensity if a future fix wants it.
        return 1 if sample[1] else 0

    def _angle_correction_deg64(self, sample: bytes, dist_mm: int) -> float:
        if dist_mm == 0:
            return 0.0
        return float(int(math.atan(17.8 / float(dist_mm)) * 3666.929888837269))

    def _sample_checksum_word(self, sample: bytes) -> int:
        # The C++ XORs the intensity byte alone, then the 16-bit distance word.
        return sample[0] ^ (sample[1] | (sample[2] << 8))


# ---------------------------------------------------------------------------
# T-mini (T-mini Plus / Pro) — ToF: distance in mm, no angle correction.
# ---------------------------------------------------------------------------

@register("YDLIDAR-TMINI", "YDLIDAR_TMINI", "TMINI", "T-MINI", "T_MINI")
class YDLidarTmini(_SelfSpinningYDLidar):
    MODEL_NAME = "YDLIDAR T-mini"
    DEFAULT_BAUD = 230400
    SAMPLE_BYTES = 2

    def _sample_distance_mm(self, sample: bytes) -> int:
        return sample[0] | (sample[1] << 8)     # ToF: raw value is mm, no /4

    def _angle_correction_deg64(self, sample: bytes, dist_mm: int) -> float:
        return 0.0                              # ToF: no triangulation correction
