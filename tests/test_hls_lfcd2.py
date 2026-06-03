"""Hardware-free tests for the Hitachi-LG HLS-LFCD2 (TurtleBot3 LDS-01) driver:
build valid 42-byte packets in memory with correct checksums, feed them through a
fake transport, and check parsing (CCW + raw CW angles, distance, intensity, invalid flag), the
checksum reject + resync path, scan grouping over a full set of indices, and the
ASCII motor commands."""
import struct
from itertools import islice

import pytest

from lds2d import Lidar, ScanPoint
from lds2d.core import driver_for, NotSupportedError
from lds2d.drivers.hitachi_lg_hls_lfcd2 import (
    INDEX_BASE,
    INVALID_FLAG,
    PACKET_SIZE,
    PACKETS_PER_SCAN,
    READINGS_PER_PACKET,
    STRENGTH_WARN_FLAG,
    calc_checksum,
    parse_packet,
)

START_BYTE = 0xFA


def make_packet(packet_number, readings, rpm=300, reserved=0):
    """Assemble a valid 42-byte HLS-LFCD2 packet with a correct checksum.

    ``readings`` is a list of 6 (intensity, raw_range) tuples; ``raw_range`` may
    carry INVALID_FLAG / STRENGTH_WARN_FLAG in its two MSBs. Each reading is
    6 bytes on the wire: intensity(u16), range(u16), reserved(u16).
    """
    assert len(readings) == READINGS_PER_PACKET
    body = struct.pack("<BBH", START_BYTE, INDEX_BASE + packet_number, rpm)
    for intensity, raw_range in readings:
        body += struct.pack("<HHH", intensity, raw_range, reserved)
    assert len(body) == PACKET_SIZE - 2, len(body)
    return body + struct.pack("<H", calc_checksum(body))


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


# --- checksum primitive ------------------------------------------------------

def test_checksum_matches_cpp_fold():
    # Independently recompute the C++ algorithm to prove calc_checksum is faithful.
    body = struct.pack("<BBH", START_BYTE, INDEX_BASE, 300)
    for k in range(READINGS_PER_PACKET):
        body += struct.pack("<HHH", 10 + k, 1000 + k, 0)
    assert len(body) == PACKET_SIZE - 2
    chk32 = 0
    for i in range(len(body) // 2):
        word = body[i * 2] | (body[i * 2 + 1] << 8)
        chk32 = (chk32 << 1) + word
    expected = ((chk32 & 0x7FFF) + (chk32 >> 15)) & 0x7FFF
    assert calc_checksum(body) == expected
    assert 0 <= calc_checksum(body) <= 0x7FFF


# --- packet parsing ----------------------------------------------------------

def test_parse_packet_ccw_angles_distance_quality():
    # readings = (intensity, range_mm); intensity comes FIRST in the wire format.
    readings = [(50 + i, 1000 + i) for i in range(READINGS_PER_PACKET)]
    freq, points = parse_packet(make_packet(0, readings))         # default CCW
    assert freq == 5.0                       # 300 rpm / 60
    assert len(points) == READINGS_PER_PACKET
    # Default CCW: packet 0 emits angles 0,1,2,3,4,5 (point_index order).
    assert [p.angle_deg for p in points] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert points[0].dist_mm == 1000 and points[0].quality == 50
    assert points[5].dist_mm == 1005 and points[5].quality == 55


def test_parse_packet_cw_matches_raw_cpp_angle():
    # cw=True reproduces the raw C++ angle 359 - point_index (reverse order).
    freq, points = parse_packet(make_packet(0, [(1, 7)] * 6), cw=True)
    assert [p.angle_deg for p in points] == [359.0, 358.0, 357.0, 356.0, 355.0, 354.0]
    # Last packet (59), cw: point_index 354..359 -> 5,4,3,2,1,0
    _f, last = parse_packet(make_packet(PACKETS_PER_SCAN - 1, [(1, 7)] * 6), cw=True)
    assert [p.angle_deg for p in last] == [5.0, 4.0, 3.0, 2.0, 1.0, 0.0]


def test_parse_last_packet_ccw_reaches_359():
    last = PACKETS_PER_SCAN - 1               # packet 59
    freq, points = parse_packet(make_packet(last, [(1, 7) for _ in range(6)]))
    # CCW: point_index 354..359 -> angle 354..359
    assert [p.angle_deg for p in points] == [354.0, 355.0, 356.0, 357.0, 358.0, 359.0]


def test_invalid_flag_zeroes_distance():
    # A range word with the invalid flag set must decode to dist 0 (no return),
    # while a clean neighbour keeps its value. The strength-warning flag alone
    # does not invalidate the sample but must be masked out of the distance.
    readings = [
        (40, 1500),                       # clean
        (40, 1500 | INVALID_FLAG),        # invalid -> 0
        (40, 1500 | STRENGTH_WARN_FLAG),  # warned but usable -> 1500
        (40, 1500), (40, 1500), (40, 1500),
    ]
    _freq, points = parse_packet(make_packet(0, readings))
    assert points[0].dist_mm == 1500 and points[0].valid
    assert points[1].dist_mm == 0 and not points[1].valid
    assert points[2].dist_mm == 1500 and points[2].valid


def test_intensity_first_not_range_first():
    # Guard against swapping the two u16 fields: a big range with tiny intensity
    # must decode as dist=big, quality=small (not the other way round).
    freq, points = parse_packet(make_packet(0, [(3, 5000)] * 6))
    assert points[0].dist_mm == 5000
    assert points[0].quality == 3


# --- driver iteration through a fake transport -------------------------------

def test_driver_resyncs_past_garbage():
    pkt = make_packet(0, [(10, 1000)] * 6)
    stream = b"\x00\xff\xfa\x01junk" + pkt + pkt
    lidar = Lidar.open("HLS-LFCD2", transport=FakeTransport(stream))
    pts = list(islice(lidar.points(), READINGS_PER_PACKET * 2))
    assert len(pts) == READINGS_PER_PACKET * 2
    assert all(isinstance(p, ScanPoint) for p in pts)
    assert pts[0].dist_mm == 1000


def test_driver_skips_bad_checksum_and_resyncs():
    good = make_packet(0, [(10, 1234)] * 6)
    bad = bytearray(make_packet(0, [(10, 9999)] * 6))
    bad[-1] ^= 0xFF                           # corrupt the checksum
    bad[-2] ^= 0xFF
    lidar = Lidar.open("HLS-LFCD2", transport=FakeTransport(bytes(bad) + good))
    first = next(iter(lidar.points()))        # must come from the good packet
    assert first.dist_mm == 1234


def test_scans_split_over_full_index_set():
    # Two complete rotations: all 60 packet indices, 6 readings each = 360 points.
    def rotation():
        out = b""
        for k in range(PACKETS_PER_SCAN):
            out += make_packet(k, [(20, 800 + k)] * 6, rpm=300)
        return out

    lidar = Lidar.open("LDS-01", transport=FakeTransport(rotation() + rotation()))
    scans = list(islice(lidar.scans(), 1))
    assert len(scans) == 1
    # A full rotation is 360 points; the first scan should hold all of them
    # (CCW emission splits exactly on the 359 -> 0 wrap at the rotation boundary).
    assert len(scans[0]) == PACKETS_PER_SCAN * READINGS_PER_PACKET
    assert scans[0].scan_freq_hz == 5.0
    # Angles inside one scan run upward from 0 to 359.
    assert scans[0].points[0].angle_deg == 0.0
    assert scans[0].points[-1].angle_deg == 359.0


# --- motor / capability ------------------------------------------------------

def test_motor_commands_are_ascii_b_and_e():
    t = FakeTransport()
    lidar = Lidar.open("HLS-LFCD2", transport=t)
    lidar.start()
    assert t.written == b"b"
    lidar.stop()
    assert t.written == b"be"


def test_set_scan_freq_no_speed_control():
    lidar = Lidar.open("HLS-LFCD2", transport=FakeTransport())
    lidar.set_scan_freq(0)        # "leave it alone" is accepted
    lidar.set_scan_freq(-1)
    with pytest.raises(NotSupportedError):
        lidar.set_scan_freq(5.0)  # any positive target is rejected


def test_get_scan_freq_reads_rpm_field():
    pkt = make_packet(0, [(10, 1000)] * 6, rpm=360)
    lidar = Lidar.open("HLS-LFCD2", transport=FakeTransport(pkt))
    assert lidar.get_scan_freq() == 6.0       # 360 rpm / 60


# --- registry / aliases ------------------------------------------------------

@pytest.mark.parametrize("alias", ["HLS-LFCD2", "HLS_LFCD2", "LFCD2", "LDS-01", "LDS01"])
def test_aliases_resolve_to_one_model(alias):
    cls = driver_for(alias)
    assert cls is not None
    assert cls.MODEL_NAME == "Hitachi-LG HLS-LFCD2"
    assert cls.DEFAULT_BAUD == 230400
