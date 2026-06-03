"""Hardware-free tests for the YDLIDAR family driver.

Builds valid PH packages in memory with correct XOR checksums, feeds them
through a fake transport, and checks angle interpolation, the check-bit shift,
distance scaling, scan-frequency from the CT byte, the scan/ring-start boundary,
checksum rejection + resync, and the per-variant distance/quality formulas.
"""
import struct
from itertools import islice

import pytest

from lds2d import Lidar, ScanPoint
from lds2d.core import driver_for
from lds2d.drivers.ydlidar import (
    PH, CMD_SYNC_BYTE, CMD_SCAN, CMD_STOP, CMD_FORCE_STOP,
    YDLidarX4, YDLidarX4Pro, YDLidarSCL, YDLidarTmini, build_command,
)


class FakeTransport:
    """Replays a fixed byte buffer; returns b'' once exhausted (like a timeout)."""

    def __init__(self, data=b""):
        self.data = bytes(data)
        self.pos = 0
        self.written = bytearray()

    def read(self, size):
        chunk = self.data[self.pos:self.pos + size]
        self.pos += len(chunk)
        return chunk

    def write(self, data):
        self.written += data
        return len(data)

    def close(self):
        pass


# --- packet builders (mirror the C++ wire format, with correct checksums) ---

def _angle_raw(deg):
    """deg -> 16-bit FSA/LSA field: (deg*64 << 1) | check-bit."""
    return ((int(round(deg * 64.0)) << 1) | 0x01) & 0xFFFF


def _frame(ct, n, fsa_raw, lsa_raw, csum, samples):
    head = struct.pack("<BBBBHHH", PH & 0xFF, PH >> 8, ct, n, fsa_raw, lsa_raw, csum)
    return head + samples


def make_x4(ct, fsa_deg, lsa_deg, dists_mm):
    """X4/X2/X3: 2-byte samples, distance = raw/4."""
    n = len(dists_mm)
    fsa_raw, lsa_raw = _angle_raw(fsa_deg), _angle_raw(lsa_deg)
    csum = PH ^ fsa_raw
    samples = b""
    for d in dists_mm:
        raw = d * 4
        samples += struct.pack("<H", raw)
        csum ^= raw
    csum ^= (ct | (n << 8))
    csum ^= lsa_raw
    return _frame(ct, n, fsa_raw, lsa_raw, csum & 0xFFFF, samples)


def make_x4pro(ct, fsa_deg, lsa_deg, dists_mm, interference=1):
    """X4 PRO: byte0 = [interference:2][dist_low:6], byte1 = dist_high (mm)."""
    n = len(dists_mm)
    fsa_raw, lsa_raw = _angle_raw(fsa_deg), _angle_raw(lsa_deg)
    csum = PH ^ fsa_raw
    samples = b""
    for d in dists_mm:
        b0 = ((interference & 0x03) | ((d & 0x3F) << 2)) & 0xFF
        b1 = d >> 6
        samples += struct.pack("<BB", b0, b1)
        csum ^= (b0 | (b1 << 8))
    csum ^= (ct | (n << 8))
    csum ^= lsa_raw
    return _frame(ct, n, fsa_raw, lsa_raw, csum & 0xFFFF, samples)


def make_scl(ct, fsa_deg, lsa_deg, dists_mm, intensity=77):
    """SCL: 3-byte samples [intensity][dist_lsb][dist_msb]."""
    n = len(dists_mm)
    fsa_raw, lsa_raw = _angle_raw(fsa_deg), _angle_raw(lsa_deg)
    csum = PH ^ fsa_raw
    samples = b""
    for d in dists_mm:
        dlsb = (d & 0x3F) << 2
        dmsb = d >> 6
        samples += struct.pack("<BBB", intensity, dlsb, dmsb)
        csum ^= intensity ^ (dlsb | (dmsb << 8))
    csum ^= (ct | (n << 8))
    csum ^= lsa_raw
    return _frame(ct, n, fsa_raw, lsa_raw, csum & 0xFFFF, samples)


def make_tmini(ct, fsa_deg, lsa_deg, dists_mm):
    """T-mini (ToF): 2-byte samples, distance = raw (mm), no correction."""
    n = len(dists_mm)
    fsa_raw, lsa_raw = _angle_raw(fsa_deg), _angle_raw(lsa_deg)
    csum = PH ^ fsa_raw
    samples = b""
    for d in dists_mm:
        samples += struct.pack("<H", d)
        csum ^= d
    csum ^= (ct | (n << 8))
    csum ^= lsa_raw
    return _frame(ct, n, fsa_raw, lsa_raw, csum & 0xFFFF, samples)


# CT byte for a ring-start package reporting 5 Hz: (CT>>1)*0.1 == 5 -> CT>>1 == 50.
CT_5HZ_START = (50 << 1) | 0x01     # ring-start bit set
CT_NORMAL = 0x00                    # non-start package


def first_packet(driver):
    return next(iter(driver._packets()))


# --- X4 base: angle interpolation, freq, distance ----------------------------

def test_x4_interpolates_angles_when_no_correction():
    # Zero distances -> the triangulation correction is skipped, so angles are
    # the pure linear interpolation FSA..LSA. This isolates the angle math.
    pkt = make_x4(CT_5HZ_START, 0.0, 11.0, [0] * 12)
    freq, pts = first_packet(YDLidarX4(FakeTransport(pkt)))
    assert freq == 5.0
    assert len(pts) == 12
    assert [round(p.angle_deg, 3) for p in pts] == [float(i) for i in range(12)]


def test_x4_distance_is_raw_over_four():
    pkt = make_x4(CT_5HZ_START, 0.0, 0.0, [100, 250, 1000])
    _freq, pts = first_packet(YDLidarX4(FakeTransport(pkt)))
    assert [p.dist_mm for p in pts] == [100, 250, 1000]
    assert all(p.quality == 10 for p in pts)     # NODE_DEFAULT_QUALITY


def test_x4_applies_angle_correction_for_real_distances():
    # With a real (non-zero) distance the atan correction shifts the angle away
    # from the bare interpolation, so this is not tautological.
    bare = make_x4(CT_5HZ_START, 0.0, 11.0, [0] * 12)
    real = make_x4(CT_5HZ_START, 0.0, 11.0, list(range(100, 112)))
    _f, pts_bare = first_packet(YDLidarX4(FakeTransport(bare)))
    _f, pts_real = first_packet(YDLidarX4(FakeTransport(real)))
    assert pts_real[0].angle_deg != pts_bare[0].angle_deg
    assert pts_real[0].dist_mm == 100


def _make_x4_raw(ct, fsa_deg, lsa_deg, raws):
    """Like make_x4 but takes raw 16-bit distance words (raw/4 == mm, with
    sub-mm fraction when raw % 4 != 0)."""
    n = len(raws)
    fsa_raw, lsa_raw = _angle_raw(fsa_deg), _angle_raw(lsa_deg)
    csum = PH ^ fsa_raw
    samples = b""
    for raw in raws:
        samples += struct.pack("<H", raw)
        csum ^= raw
    csum ^= (ct | (n << 8))
    csum ^= lsa_raw
    return _frame(ct, n, fsa_raw, lsa_raw, csum & 0xFFFF, samples)


def test_x4_angle_correction_uses_full_resolution_distance():
    # The C++ feeds node.distance_q2*0.25 (sub-mm precision) to the atan
    # correction, NOT the truncated whole-mm distance. raw=43 -> d=10.75 mm,
    # which gives a different correction than the truncated 10 mm would. This
    # guards against re-truncating dist_mm before the correction.
    import math
    raw = 43                       # 10.75 mm; truncates to 10 mm
    fsa_deg = 30.0
    pkt = _make_x4_raw(CT_5HZ_START, fsa_deg, fsa_deg, [raw])
    _freq, pts = first_packet(YDLidarX4(FakeTransport(pkt)))

    fsa64 = fsa_deg * 64.0
    d_full = raw * 0.25            # 10.75
    acd_full = float(int(math.atan(((21.8 * (155.3 - d_full)) / 155.3) / d_full)
                         * 3666.93))
    expected = ((fsa64 + acd_full) / 64.0) % 360.0
    assert round(pts[0].angle_deg, 6) == round(expected, 6)

    # And it must NOT match the (wrong) truncated-distance correction.
    d_trunc = float(raw // 4)      # 10.0
    acd_trunc = float(int(math.atan(((21.8 * (155.3 - d_trunc)) / 155.3) / d_trunc)
                          * 3666.93))
    wrong = ((fsa64 + acd_trunc) / 64.0) % 360.0
    assert round(pts[0].angle_deg, 6) != round(wrong, 6)
    assert pts[0].dist_mm == 10    # output distance is still truncated mm


def test_x4_check_bit_shift_decodes_first_angle():
    # FSA field carries deg*64 shifted left 1 with a check bit; a 90° start must
    # decode back to ~90° (zero distance => no correction).
    pkt = make_x4(CT_5HZ_START, 90.0, 90.0, [0])
    _freq, pts = first_packet(YDLidarX4(FakeTransport(pkt)))
    assert round(pts[0].angle_deg, 2) == 90.0


# --- scan/ring-start boundary ------------------------------------------------

def test_ring_start_reports_freq_normal_packet_reports_zero():
    start = make_x4(CT_5HZ_START, 0.0, 11.0, [0] * 12)
    normal = make_x4(CT_NORMAL, 12.0, 23.0, [0] * 12)
    gen = YDLidarX4(FakeTransport(start + normal))._packets()
    f0, _p0 = next(gen)
    f1, _p1 = next(gen)
    assert f0 == 5.0          # ring-start package carries the scan frequency
    assert f1 == 0.0          # a non-start package reports 0


def test_scans_split_on_angle_wrap():
    # Two ring-start packages 180° apart force a wrap so scans() yields a scan.
    a = make_x4(CT_5HZ_START, 0.0, 90.0, [0] * 4)
    b = make_x4(CT_NORMAL, 100.0, 190.0, [0] * 4)
    c = make_x4(CT_5HZ_START, 0.0, 90.0, [0] * 4)   # wraps back past 0
    lidar = YDLidarX4(FakeTransport(a + b + c + a))
    scans = list(islice(lidar.scans(), 1))
    assert len(scans) == 1
    assert scans[0].scan_freq_hz == 5.0
    assert len(scans[0]) >= 4


# --- checksum rejection and resync -------------------------------------------

def test_bad_checksum_is_skipped_and_parser_resyncs():
    good = make_x4(CT_5HZ_START, 0.0, 11.0, [200] * 12)
    bad = bytearray(make_x4(CT_5HZ_START, 0.0, 11.0, [200] * 12))
    bad[8] ^= 0xFF            # corrupt the checksum low byte
    lidar = YDLidarX4(FakeTransport(bytes(bad) + good))
    freq, pts = first_packet(lidar)     # must come from the good package
    assert freq == 5.0
    assert pts[0].dist_mm == 200


def test_resyncs_past_leading_garbage():
    pkt = make_x4(CT_5HZ_START, 0.0, 11.0, [0] * 12)
    lidar = YDLidarX4(FakeTransport(b"\x00\xff\xaa\x13garbage" + pkt + pkt))
    pts = list(islice(lidar.points(), 24))
    assert len(pts) == 24
    assert all(isinstance(p, ScanPoint) for p in pts)


def test_check_bit_violation_rejects_packet():
    pkt = bytearray(make_x4(CT_5HZ_START, 0.0, 11.0, [0] * 12))
    pkt[4] &= 0xFE           # clear the FSA check bit -> header must be rejected
    good = make_x4(CT_5HZ_START, 0.0, 11.0, [300] * 12)
    lidar = YDLidarX4(FakeTransport(bytes(pkt) + good))
    _freq, pts = first_packet(lidar)
    assert pts[0].dist_mm == 300


# --- X4 PRO variant: packed 6-bit distance, interference quality --------------

def test_x4pro_unpacks_distance_and_quality():
    pkt = make_x4pro(CT_5HZ_START, 0.0, 0.0, [300, 1000, 4095], interference=2)
    _freq, pts = first_packet(YDLidarX4Pro(FakeTransport(pkt)))
    assert [p.dist_mm for p in pts] == [300, 1000, 4095]
    assert all(p.quality == 2 for p in pts)      # interference bits


# --- SCL variant: 3-byte samples ---------------------------------------------

def test_scl_decodes_14bit_distance():
    pkt = make_scl(CT_5HZ_START, 0.0, 0.0, [500, 1234, 8191])
    freq, pts = first_packet(YDLidarSCL(FakeTransport(pkt)))
    assert freq == 5.0
    assert [p.dist_mm for p in pts] == [500, 1234, 8191]
    # SCL posts the (buggy) quality_flag == 1 when distance_lsb != 0.
    assert all(p.quality in (0, 1) for p in pts)


# --- T-mini variant: ToF, no /4, no angle correction -------------------------

def test_tmini_distance_is_raw_and_angle_uncorrected():
    pkt = make_tmini(CT_5HZ_START, 0.0, 11.0, [1500] * 12)
    _freq, pts = first_packet(YDLidarTmini(FakeTransport(pkt)))
    assert pts[0].dist_mm == 1500                # raw mm, no /4
    # No triangulation correction -> pure interpolation even with real distance.
    assert [round(p.angle_deg, 3) for p in pts] == [float(i) for i in range(12)]


# --- motor command frames (X4 is command-driven) -----------------------------

def test_x4_start_sends_force_stop_then_scan():
    t = FakeTransport()
    lidar = Lidar.open("YDLIDAR-X4", transport=t)
    lidar.start()
    assert bytes(t.written) == bytes([CMD_SYNC_BYTE, CMD_FORCE_STOP,
                                      CMD_SYNC_BYTE, CMD_SCAN])


def test_x4_stop_sends_stop_command():
    t = FakeTransport()
    Lidar.open("X4", transport=t).stop()
    assert bytes(t.written) == bytes([CMD_SYNC_BYTE, CMD_STOP])


def test_build_command():
    assert build_command(CMD_SCAN) == bytes([0xA5, 0x60])


def test_self_spinning_models_have_noop_motor():
    # X2/X3/SCL/T-mini spin on power: start()/stop() must not raise or write.
    for model in ("X2", "X3", "X3-PRO", "X4-PRO", "SCL", "TMINI"):
        t = FakeTransport()
        lidar = Lidar.open(model, transport=t)
        lidar.start()
        lidar.stop()
        assert bytes(t.written) == b""


# --- registry / model metadata -----------------------------------------------

@pytest.mark.parametrize("alias,model_name,baud", [
    ("YDLIDAR-X4", "YDLIDAR X4", 128000),
    ("YDLIDAR_X4", "YDLIDAR X4", 128000),
    ("X2", "YDLIDAR X2/X2L", 115200),
    ("X2L", "YDLIDAR X2/X2L", 115200),
    ("X3", "YDLIDAR X3", 115200),
    ("X3-PRO", "YDLIDAR X3 PRO", 115200),
    ("X4-PRO", "YDLIDAR X4 PRO", 128000),
    ("SCL", "YDLIDAR SCL", 115200),
    ("TMINI", "YDLIDAR T-mini", 230400),
    ("T-MINI", "YDLIDAR T-mini", 230400),
])
def test_registry_aliases(alias, model_name, baud):
    cls = driver_for(alias)
    assert cls is not None
    assert cls.MODEL_NAME == model_name
    assert cls.DEFAULT_BAUD == baud


def test_pro_variants_share_x4_parser():
    # X3 PRO must parse an X4-format package identically to the base.
    pkt = make_x4(CT_5HZ_START, 0.0, 11.0, [123] * 12)
    lidar = Lidar.open("X3-PRO", transport=FakeTransport(pkt + pkt))
    pts = list(islice(lidar.points(), 12))
    assert len(pts) == 12
    assert pts[0].dist_mm == 123
