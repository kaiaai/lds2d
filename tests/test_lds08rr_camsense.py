"""Hardware-free tests for the LDS08RR (Camsense protocol) driver: build 60-byte
packets in memory, feed them through a fake transport, and check angle
interpolation, signed-distance/quality decoding, the validity gates (angle range
and angular span) plus resync, and scan grouping.

Protocol reverse engineered from the capture in kaiaai/LDS issue #17."""
import struct
from itertools import islice

import pytest

from lds2d import Lidar
from lds2d.core import driver_for
from lds2d.drivers.lds08rr_camsense import checksum, parse_packet

START0, START1, START2, SAMPLES = 0x55, 0xAA, 0x07, 0x0C
ANGLE_MIN = 0xA000
PACKET_SIZE = 60


def make_packet(start_deg, end_deg, dists, quality=33, speed=19335, bad_checksum=False):
    """Assemble a valid 60-byte packet with a correct 15-bit checksum.

    Angles in degrees -> 0xA000 + deg*64. speed default 19335 -> ~5.04 Hz.
    """
    start_raw = ANGLE_MIN + int(round(start_deg * 64))
    end_raw = ANGLE_MIN + int(round(end_deg * 64))
    body = struct.pack("<BBBBHH", START0, START1, START2, SAMPLES, speed, start_raw)
    assert len(dists) == SAMPLES
    for d in dists:
        body += struct.pack("<hBB", d, quality, 0)
    body += struct.pack("<H", end_raw)
    cks = checksum(body)
    if bad_checksum:
        cks ^= 0x0001
    body += struct.pack("<H", cks)
    assert len(body) == PACKET_SIZE
    return body


class FakeTransport:
    def __init__(self, data=b""):
        self.data = bytes(data)
        self.pos = 0

    def read(self, size):
        chunk = self.data[self.pos:self.pos + size]
        self.pos += len(chunk)
        return chunk

    def write(self, data):
        return len(data)

    def close(self):
        pass


# --- parsing ---

def test_parse_angles_distance_quality_freq():
    dists = list(range(600, 600 + SAMPLES))          # 600..611 mm
    pkt = make_packet(137.0, 148.0, dists)            # 11 deg over 12 samples -> 1 deg
    freq, points, _crc = parse_packet(pkt)
    assert freq == pytest.approx(19335 / 3840.0)      # ~5.04 Hz
    assert len(points) == SAMPLES
    assert points[0].angle_deg == pytest.approx(137.0)
    assert points[-1].angle_deg == pytest.approx(148.0)
    assert points[3].angle_deg == pytest.approx(140.0)
    assert [p.dist_mm for p in points] == dists
    assert all(p.quality == 33 for p in points)


def test_negative_distance_is_invalid_no_return():
    # 0x8000 as int16 = -32768, the "no return" marker seen in captures
    dists = [-32768] * 6 + [500] * 6
    _freq, points, _crc = parse_packet(make_packet(0.0, 11.0, dists))
    assert [p.dist_mm for p in points[:6]] == [-32768] * 6
    assert not any(p.valid for p in points[:6])
    assert all(p.valid for p in points[6:])


def test_parse_handles_angle_wrap():
    # start 355 deg, end 6 deg -> wraps through 360
    _freq, points, _crc = parse_packet(make_packet(355.0, 366.0, [500] * SAMPLES))
    assert points[0].angle_deg == pytest.approx(355.0)
    # angles past 360 wrap back down
    assert points[-1].angle_deg == pytest.approx(6.0)


# --- validity gates (these are what stop the "noise" in issue #17) ---

def test_checksum_matches_real_capture_packets():
    """Two verbatim packets from the issue #17 report must pass the checksum."""
    p1 = bytes.fromhex("55AA070C874B3FC24F050F006805030040050100710212006D021E00"
                       "6802210064022100620222005E0221005B02210059022100560222"
                       "0004C5D61A")
    p2 = bytes.fromhex("55AA070C884B45C5530222004F0221004D02220049022200460221"
                       "0043022300400222003B0222003702220033022200300223002B02"
                       "220007C89A6E")
    for p in (p1, p2):
        assert len(p) == PACKET_SIZE
        assert checksum(p[:-2]) == struct.unpack_from("<H", p, 58)[0]


def test_checksum_is_15_bit():
    # the fold masks to 0x7FFF, so the high bit is never set
    for start in (0.0, 90.0, 217.5):
        pkt = make_packet(start, start + 11.0, list(range(100, 100 + SAMPLES)))
        assert struct.unpack_from("<H", pkt, 58)[0] & 0x8000 == 0


def test_rejects_bad_checksum():
    with pytest.raises(ValueError):
        parse_packet(make_packet(10.0, 21.0, [500] * SAMPLES, bad_checksum=True))


def test_rejects_angle_below_0xA000():
    # corrupt start_angle, then re-checksum so only the angle gate can reject it
    pkt = bytearray(make_packet(10.0, 21.0, [500] * SAMPLES))
    struct.pack_into("<H", pkt, 6, 0x9000)      # start_angle below ANGLE_MIN
    struct.pack_into("<H", pkt, 58, checksum(bytes(pkt[:-2])))
    with pytest.raises(ValueError):
        parse_packet(bytes(pkt))


# --- driver iteration through a fake transport ---

def test_driver_resyncs_past_garbage():
    pkt = make_packet(137.0, 148.0, [600] * SAMPLES)
    stream = b"\x00\xffjunk\x55\xaanope" + pkt + pkt
    lidar = Lidar.open("LDS08RR-CAMSENSE", transport=FakeTransport(stream))
    pts = list(islice(lidar.points(), SAMPLES * 2))
    assert len(pts) == SAMPLES * 2
    assert all(p.dist_mm == 600 for p in pts)


def test_driver_skips_bad_checksum_then_recovers():
    bad = make_packet(10.0, 21.0, [500] * SAMPLES, bad_checksum=True)
    good = make_packet(137.0, 148.0, [777] * SAMPLES)
    lidar = Lidar.open("LDS08RR-CAMSENSE", transport=FakeTransport(bad + good))
    first = next(iter(lidar.points()))
    assert first.dist_mm == 777      # the corrupt packet was rejected


def test_scans_split_on_wrap():
    def rotation():
        out = b""
        for k in range(30):                     # 30 packets x 12 deg = 360 deg
            start = k * 12.0
            out += make_packet(start, start + 11.0, [1000] * SAMPLES)
        return out

    lidar = Lidar.open("LDS08RR-CAMSENSE", transport=FakeTransport(rotation() * 2))
    scans = list(islice(lidar.scans(), 1))
    assert len(scans) == 1
    assert len(scans[0]) >= SAMPLES
    assert scans[0].scan_freq_hz == pytest.approx(19335 / 3840.0)


# --- registry ---

def test_registered_and_distinct_from_delta_lds08rr():
    for name in ("LDS08RR-CAMSENSE", "3IROBOTIX-LDS08RR-CAMSENSE", "LDS08RR_CAMSENSE"):
        assert driver_for(name) is not None
        assert driver_for(name).MODEL_NAME == "LDS08RR (Camsense protocol)"
    # the bare LDS08RR name must still resolve to the Delta-protocol revision
    assert driver_for("LDS08RR").MODEL_NAME == "LDS08RR"
    assert driver_for("LDS08RR") is not driver_for("LDS08RR-CAMSENSE")
    assert driver_for("LDS08RR-CAMSENSE").DEFAULT_BAUD == 115200
