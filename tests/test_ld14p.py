"""Hardware-free tests for the LD14P driver: build packets in memory, feed them
through a fake transport, and check parsing, CRC, scan grouping, and motor frames."""
import struct
from itertools import islice

from lds2d import Lidar, ScanPoint
from lds2d.crc import crc8
from lds2d.drivers.ldrobot_ld14p import build_command, parse_packet

HEADER, VER_LEN, POINTS = 0x54, 0x2C, 12


def make_packet(start_centideg, end_centideg, dists, intensity=200):
    """Assemble a valid 47-byte LD14P packet with a correct CRC."""
    speed_dps = 1800  # 5 Hz
    body = struct.pack("<BBHH", HEADER, VER_LEN, speed_dps, start_centideg)
    for d in dists:
        body += struct.pack("<HB", d, intensity)
    body += struct.pack("<HH", end_centideg, 0)
    assert len(body) == 46
    return body + bytes([crc8(body)])


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


# --- motor command frames (known-good bytes from the LDS library / README) ---

def test_motor_frames_match_known_bytes():
    assert build_command(0xA0).hex() == "54a004000000005e"   # start
    assert build_command(0xA1).hex() == "54a104000000004a"   # stop


def test_set_6hz_frame():
    # 6 Hz -> 2160 deg/s -> 0x0870 little-endian
    frame = build_command(0xA2, struct.pack("<H", 6 * 360))
    assert frame.hex() == "54a20470080000a1"


# --- packet parsing ---

def test_parse_packet_angles_and_freq():
    dists = list(range(100, 100 + POINTS))   # 100..111 mm
    pkt = make_packet(0, 1100, dists)         # 0.00° .. 11.00°
    freq, points = parse_packet(pkt)
    assert freq == 5.0
    assert len(points) == POINTS
    assert points[0].angle_deg == 0.0
    assert round(points[-1].angle_deg, 2) == 11.0
    assert points[0].dist_mm == 100
    assert points[-1].dist_mm == 111
    assert all(p.quality == 200 for p in points)


def test_parse_handles_angle_wrap():
    # start near the top of the circle, end just past 0 — span must wrap
    freq, points = parse_packet(make_packet(35900, 100, [500] * POINTS))
    assert points[0].angle_deg == 359.0
    assert points[-1].angle_deg == 1.0   # wrapped through 360


# --- driver iteration through a fake transport ---

def test_driver_resyncs_past_garbage():
    pkt = make_packet(0, 1100, [200] * POINTS)
    stream = b"\xff\x00garbage" + pkt + pkt
    lidar = Lidar.open("LD14P", transport=FakeTransport(stream))
    pts = list(islice(lidar.points(), POINTS * 2))
    assert len(pts) == POINTS * 2
    assert all(isinstance(p, ScanPoint) for p in pts)


def test_driver_skips_bad_crc():
    good = make_packet(0, 1100, [200] * POINTS)
    bad = bytearray(make_packet(0, 1100, [200] * POINTS))
    bad[-1] ^= 0xFF                      # corrupt the CRC
    lidar = Lidar.open("LD14P", transport=FakeTransport(bytes(bad) + good))
    first = next(iter(lidar.points()))   # must come from the good packet
    assert first.dist_mm == 200


def test_scans_split_on_wrap():
    # Two full rotations: 12 packets of 30° each = 360°, then again.
    def rotation():
        out = b""
        for k in range(12):
            start = (k * 3000) % 36000
            end = ((k + 1) * 3000) % 36000
            out += make_packet(start, end, [1000] * POINTS)
        return out

    lidar = Lidar.open("LD14P", transport=FakeTransport(rotation() + rotation()))
    scans = list(islice(lidar.scans(), 1))
    assert len(scans) == 1
    assert len(scans[0]) >= POINTS
    assert scans[0].scan_freq_hz == 5.0


def test_motor_methods_write_frames():
    t = FakeTransport()
    lidar = Lidar.open("LD14P", transport=t)
    lidar.stop()
    assert t.written.hex() == "54a104000000004a"


def test_set_scan_freq_rejects_out_of_range():
    lidar = Lidar.open("LD14P", transport=FakeTransport())
    for bad in (1.0, 9.0, 0):
        try:
            lidar.set_scan_freq(bad)
        except ValueError:
            continue
        raise AssertionError(f"{bad} Hz should have been rejected")
