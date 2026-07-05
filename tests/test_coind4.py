"""Hardware-free tests for the COIN-D4A driver: build frames in memory (with the
protocol's split XOR checksums), feed them through a fake transport, and check
parsing, checksum handling, resync, and command frames.

Protocol reference: QuirkyCort's MicroPython coind4.py (CSPC M1C1-Mini family)."""
import struct
from itertools import islice

from lds2d import Lidar, ScanPoint
from lds2d.core import driver_for
from lds2d.drivers.cspc_coind4 import build_command, checksum_ok, parse_frame

H0, H1 = 0xAA, 0x55


def make_sample(dist_mm, mode=0):
    """One 3-byte sample encoding the given 14-bit distance (strength derived)."""
    b0 = mode & 0x03
    b1 = (dist_mm & 0x3F) << 2
    b2 = (dist_mm >> 6) & 0xFF
    return bytes((b0, b1, b2))


def make_frame(start_x64, end_x64, dists, freq_hz=10.0):
    """Assemble a valid COIN-D4A frame with correct split checksums.

    Angles are given in 1/64-degree units (the value the driver works in)."""
    n = len(dists)
    speed = (int(round(freq_hz * 10)) << 1) | 1        # bit0 set => valid speed
    b = bytearray((H0, H1, speed, n))
    b += struct.pack("<HH", (start_x64 << 1) & 0xFFFF, (end_x64 << 1) & 0xFFFF)
    body = b"".join(make_sample(d) for d in dists)
    cs_even = b[0] ^ b[2] ^ b[4] ^ b[6]
    cs_odd = b[1] ^ b[3] ^ b[5] ^ b[7]
    for i in range(n):
        cs_even ^= body[i * 3] ^ body[i * 3 + 1]
        cs_odd ^= body[i * 3 + 2]
    b.append(cs_even)
    b.append(cs_odd)
    b += body
    return bytes(b)


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


# --- command frames (AA 55 code, checksum = AA ^ 55 ^ code) ---

def test_command_frames_match_protocol():
    assert build_command(0xF0).hex() == "aa55f00f"   # start
    assert build_command(0xF5).hex() == "aa55f50a"   # stop
    assert build_command(0xF1).hex() == "aa55f10e"   # high exposure
    assert build_command(0xF2).hex() == "aa55f20d"   # low exposure


def test_make_frame_self_checksum():
    frame = make_frame(0, 448, [1000] * 8)
    assert checksum_ok(frame, 8)


# --- frame parsing ---

def test_parse_angles_distance_and_freq():
    # 8 samples, 0deg .. 7deg (step = 64/64 = 1deg), distances 1000..1007
    dists = [1000 + i for i in range(8)]
    freq, points = parse_frame(make_frame(0, 448, dists), 8)
    assert freq == 10.0
    assert len(points) == 8
    assert [round(p.angle_deg) for p in points] == list(range(8))
    assert [p.dist_mm for p in points] == dists


def test_parse_handles_angle_wrap():
    # start at 359deg (359*64=22976), end at 1deg (64) -> wrap through 360
    start_x64, end_x64 = 359 * 64, 1 * 64
    _freq, points = parse_frame(make_frame(start_x64, end_x64, [500] * 3), 3)
    assert round(points[0].angle_deg) == 359
    assert round(points[-1].angle_deg) % 360 == 1


def test_large_distance_14bit():
    _freq, points = parse_frame(make_frame(0, 64, [12000, 12000], 10.0), 2)
    assert points[0].dist_mm == 12000


# --- driver iteration through a fake transport ---

def test_driver_resyncs_past_garbage():
    frame = make_frame(0, 448, [200] * 8)
    stream = b"\x00\xffjunk\xaanope" + frame + frame
    lidar = Lidar.open("COIN-D4A", transport=FakeTransport(stream))
    pts = list(islice(lidar.points(), 16))
    assert len(pts) == 16
    assert all(isinstance(p, ScanPoint) for p in pts)


def test_driver_skips_bad_checksum():
    good = make_frame(0, 448, [321] * 8)
    bad = bytearray(make_frame(0, 448, [111] * 8))
    bad[8] ^= 0xFF                       # corrupt a checksum byte
    lidar = Lidar.open("COIN-D4A", transport=FakeTransport(bytes(bad) + good))
    first = next(iter(lidar.points()))   # must come from the good frame
    assert first.dist_mm == 321


def test_scans_split_on_wrap():
    def rotation():
        out = b""
        for k in range(12):                     # 12 frames of 30deg = 360deg
            start = (k * 30 * 64)
            end = ((k + 1) * 30 * 64) % (360 * 64)
            out += make_frame(start, end, [1000] * 4)
        return out

    lidar = Lidar.open("COIN-D4A", transport=FakeTransport(rotation() + rotation()))
    scans = list(islice(lidar.scans(), 1))
    assert len(scans) == 1
    assert len(scans[0]) >= 12


# --- motor / exposure commands write the right bytes ---

def test_start_stop_write_frames():
    t = FakeTransport()
    lidar = Lidar.open("COIN-D4A", transport=t)
    lidar.start()
    lidar.stop()
    assert t.written.hex() == "aa55f00f" + "aa55f50a"


def test_exposure_commands():
    t = FakeTransport()
    lidar = Lidar.open("COIN-D4A", transport=t)
    lidar.set_high_exposure()
    lidar.set_low_exposure()
    assert t.written.hex() == "aa55f10e" + "aa55f20d"


# --- registry / aliases ---

def test_registered_under_aliases():
    for name in ("COIN-D4A", "COIN_D4A", "COIN-D4", "COIND4"):
        assert driver_for(name) is not None
        assert driver_for(name).MODEL_NAME == "COIN-D4A"
    assert driver_for("COIN-D4A").DEFAULT_BAUD == 115200
