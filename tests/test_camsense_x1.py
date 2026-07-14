"""Hardware-free tests for the Camsense X1 driver: build 36-byte packets in
memory, feed them through a fake transport, and check angle interpolation,
distance/quality decoding, the checksum and angle-range validity gates + resync,
and scan grouping. Also checks the checksum against real captured packets."""
import struct
from itertools import islice

import pytest

from lds2d import Lidar, ScanPoint
from lds2d.core import driver_for
from lds2d.crc import camsense_checksum
from lds2d.drivers.camsense_x1 import parse_packet

START0, START1, START2, SAMPLES = 0x55, 0xAA, 0x03, 0x08
ANGLE_MIN = 0xA000

# Real packets captured from a Camsense X1, published at
# github.com/thijses/camsense-X1 ("a few data packets.txt").
REAL_PACKETS = [bytes.fromhex(h) for h in (
    "55AA0308EF4DB1C200800000800000800000800000800000800000800000800033C4D133",
    "55AA0308EC4D6CC40080000080000080000080000080008106147E06337D0633F6C5493C",
    "55AA0308EC4D2FC67A06357A06307B06357A0633790634780622080709ED0609BAC7DB39",
)]


def make_packet(start_deg, end_deg, dists, quality=200, speed=19968,
                bad_checksum=False):
    """Assemble a valid 36-byte Camsense X1 packet with a correct checksum.

    Angles are given in degrees and converted to the 0xA000 + deg*64 encoding.
    speed default 19968 -> 19968/3840 = 5.2 Hz.
    """
    start_raw = ANGLE_MIN + int(round(start_deg * 64))
    end_raw = ANGLE_MIN + int(round(end_deg * 64))
    body = struct.pack("<BBBBHH", START0, START1, START2, SAMPLES, speed, start_raw)
    assert len(dists) == SAMPLES
    for d in dists:
        body += struct.pack("<hB", d, quality)
    body += struct.pack("<H", end_raw)
    cks = camsense_checksum(body)
    if bad_checksum:
        cks ^= 0x0001
    body += struct.pack("<H", cks)
    assert len(body) == 36
    return body


# --- checksum, validated against real hardware captures ---

def test_checksum_matches_real_captured_packets():
    for pkt in REAL_PACKETS:
        assert len(pkt) == 36
        assert camsense_checksum(pkt[:34]) == struct.unpack_from("<H", pkt, 34)[0]


def test_checksum_is_15_bit():
    # the fold masks to 0x7FFF, so the stored high bit is never set
    for pkt in REAL_PACKETS:
        assert struct.unpack_from("<H", pkt, 34)[0] & 0x8000 == 0


def test_real_captured_packet_parses():
    freq, points, _cks = parse_packet(REAL_PACKETS[0])
    assert round(freq, 2) == 5.20            # X1 free-runs at ~5.2 Hz
    assert len(points) == SAMPLES
    assert all(not p.valid for p in points)  # this one is all 0x8000 "no return"


def test_rejects_bad_checksum():
    with pytest.raises(ValueError):
        parse_packet(make_packet(10.0, 17.0, [500] * SAMPLES, bad_checksum=True))


def test_parse_packet_angles_distance_quality_freq():
    dists = list(range(100, 100 + SAMPLES))      # 100..107 mm
    pkt = make_packet(10.0, 17.0, dists)          # span 7 deg over 8 samples -> 1 deg step
    freq, points, crc = parse_packet(pkt)
    assert round(freq, 4) == 5.2
    assert len(points) == SAMPLES
    # linear interpolation: angle i = 10 + i
    assert points[0].angle_deg == pytest.approx(10.0)
    assert points[-1].angle_deg == pytest.approx(17.0)
    assert points[3].angle_deg == pytest.approx(13.0)
    assert points[0].dist_mm == 100
    assert points[-1].dist_mm == 107
    assert all(p.quality == 200 for p in points)


def test_parse_handles_angle_wrap():
    # start near top of circle, end just past 0 -> end += 360 internally, then wraps
    freq, points, crc = parse_packet(make_packet(357.0, 4.0, [500] * SAMPLES))
    # span = (4 + 360) - 357 = 7 deg, step 1 deg: 357,358,359,360->0,1,2,3,4
    assert points[0].angle_deg == pytest.approx(357.0)
    assert points[3].angle_deg == pytest.approx(360.0)  # boundary stays at 360
    assert points[4].angle_deg == pytest.approx(1.0)    # 361 -> wrapped to 1
    assert points[-1].angle_deg == pytest.approx(4.0)


def test_signed_distance_is_negative():
    # distance is a signed int16 in the C++; a negative value must survive.
    pkt = make_packet(0.0, 7.0, [-3] + [200] * (SAMPLES - 1))
    _freq, points, _crc = parse_packet(pkt)
    assert points[0].dist_mm == -3


def test_angle_below_min_rejected():
    # Hand-build a packet whose start_angle is below 0xA000 -> ValueError.
    # The checksum is made valid so the angle gate is what rejects it.
    body = struct.pack("<BBBBHH", START0, START1, START2, SAMPLES, 19968, 0x0010)
    for _ in range(SAMPLES):
        body += struct.pack("<hB", 100, 50)
    body += struct.pack("<H", ANGLE_MIN + 100)
    body += struct.pack("<H", camsense_checksum(body))
    with pytest.raises(ValueError):
        parse_packet(bytes(body))


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


def test_driver_parses_two_packets():
    dists = list(range(100, 100 + SAMPLES))
    pkt = make_packet(10.0, 17.0, dists)
    lidar = Lidar.open("CAMSENSE-X1", transport=FakeTransport(pkt + pkt))
    pts = list(islice(lidar.points(), SAMPLES * 2))
    assert len(pts) == SAMPLES * 2
    assert all(isinstance(p, ScanPoint) for p in pts)
    assert pts[0].dist_mm == 100
    assert pts[SAMPLES - 1].angle_deg == pytest.approx(17.0)


def test_driver_resyncs_past_garbage():
    pkt = make_packet(10.0, 17.0, [200] * SAMPLES)
    stream = b"\xff\x00\x11garbage\x55\xaa" + pkt + pkt
    lidar = Lidar.open("CAMSENSE-X1", transport=FakeTransport(stream))
    pts = list(islice(lidar.points(), SAMPLES * 2))
    assert len(pts) == SAMPLES * 2
    assert pts[0].dist_mm == 200


def test_driver_skips_invalid_angle_packet():
    # A 36-byte block with the right header but a bad (low) angle must be
    # dropped via the angle-range gate; the following good packet wins.
    good = make_packet(10.0, 17.0, [300] * SAMPLES)
    bad = bytearray(make_packet(10.0, 17.0, [200] * SAMPLES))
    # corrupt start_angle to below 0xA000 (offset 6..7, little-endian), then
    # re-checksum so the angle gate (not the checksum) is what rejects it
    bad[6:8] = struct.pack("<H", 0x0010)
    bad[34:36] = struct.pack("<H", camsense_checksum(bytes(bad[:34])))
    lidar = Lidar.open("CAMSENSE-X1", transport=FakeTransport(bytes(bad) + good))
    first = next(iter(lidar.points()))
    assert first.dist_mm == 300


def test_scans_split_on_wrap():
    # Build two full rotations as 12 packets of 30 deg each.
    def rotation():
        out = b""
        for k in range(12):
            start = (k * 30.0) % 360.0
            end = start + 30.0
            dists = [1000] * SAMPLES
            out += make_packet(start, end if end < 360 else end - 360, dists)
        return out

    lidar = Lidar.open("CAMSENSE-X1", transport=FakeTransport(rotation() + rotation()))
    scans = list(islice(lidar.scans(), 1))
    assert len(scans) == 1
    assert len(scans[0]) >= SAMPLES
    assert scans[0].scan_freq_hz == pytest.approx(5.2)


def test_registered_aliases_and_baud():
    for name in ("CAMSENSE-X1", "CAMSENSE_X1", "CAMSENSE"):
        cls = driver_for(name)
        assert cls is not None
        assert cls.MODEL_NAME == "Camsense X1"
        assert cls.DEFAULT_BAUD == 115200


def test_no_motor_control():
    from lds2d.core import NotSupportedError
    lidar = Lidar.open("CAMSENSE-X1", transport=FakeTransport())
    with pytest.raises(NotSupportedError):
        lidar.set_scan_freq(5.0)
