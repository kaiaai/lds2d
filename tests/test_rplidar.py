"""Hardware-free tests for the RPLIDAR A1/C1 driver: build a response descriptor
and 5-byte measurement nodes in memory with correct check bits, feed them through
a fake transport, and check node parsing, check-bit rejection + resync, scan
grouping on the start-flag bit, command bytes, and per-model baud."""
from itertools import islice

import pytest

from lds2d import Lidar, ScanPoint
from lds2d.core import driver_for
from lds2d.drivers.rplidar import (
    build_scan_command,
    build_stop_command,
    node_ok,
    parse_node,
)

ANS_SYNC1, ANS_SYNC2, ANS_TYPE_MEAS = 0xA5, 0x5A, 0x81


def make_descriptor(type_byte=ANS_TYPE_MEAS):
    """7-byte scan response descriptor: 0xA5 0x5A, 4 size/subtype bytes, type."""
    return bytes([ANS_SYNC1, ANS_SYNC2, 0x05, 0x00, 0x00, 0x40, type_byte])


def make_node(angle_deg, dist_mm, quality, scan_start=False):
    """Assemble a valid 5-byte RPLIDAR node with correct check bits.

    byte0: syncbit(bit0)=scan_start, syncbit_inverse(bit1)=NOT scan_start, quality<<2
    byte1: angle low bits with check bit (bit0) forced to 1
    bytes1-2: angle_q6 = round(angle_deg*64), stored as (angle_q6<<1)|1
    bytes3-4: distance_q2 = dist_mm*4 (u16 LE)
    """
    syncbit = 1 if scan_start else 0
    sync_inv = 0 if scan_start else 1
    b0 = (quality << 2) | (sync_inv << 1) | syncbit

    angle_q6 = round(angle_deg * 64)
    angle_word = ((angle_q6 << 1) | 0x01) & 0xFFFF   # check bit = 1
    b1 = angle_word & 0xFF
    b2 = (angle_word >> 8) & 0xFF

    distance_q2 = round(dist_mm * 4) & 0xFFFF
    b3 = distance_q2 & 0xFF
    b4 = (distance_q2 >> 8) & 0xFF
    return bytes([b0, b1, b2, b3, b4])


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


# --- command frames (known-good bytes from the C++ CMD_* constants) ---

def test_scan_command_bytes():
    assert build_scan_command().hex() == "a520"
    assert build_scan_command(force=True).hex() == "a521"


def test_stop_command_bytes():
    assert build_stop_command().hex() == "a525"


# --- node check bits ---

def test_node_ok_accepts_valid():
    assert node_ok(make_node(90.0, 1000, 47))


def test_node_ok_rejects_bad_byte0():
    # break syncbit/inverse relation: both bits equal -> ((b0>>1)^b0)&1 == 0
    bad = bytearray(make_node(90.0, 1000, 47))
    bad[0] = (bad[0] & ~0x03) | 0x03   # syncbit=1, inverse=1 (same) -> invalid
    assert not node_ok(bytes(bad))


def test_node_ok_rejects_bad_byte1_checkbit():
    bad = bytearray(make_node(90.0, 1000, 47))
    bad[1] &= ~0x01                    # clear the byte1 check bit
    assert not node_ok(bytes(bad))


# --- node parsing math ---

def test_parse_node_angle_distance_quality():
    point, completed = parse_node(make_node(90.0, 1000, 47, scan_start=False))
    assert isinstance(point, ScanPoint)
    assert round(point.angle_deg, 3) == 90.0
    assert point.dist_mm == 1000
    assert point.quality == 47
    assert completed is False


def test_parse_node_scan_start_flag():
    _point, completed = parse_node(make_node(0.0, 500, 10, scan_start=True))
    assert completed is True


def test_parse_node_fractional_angle_and_distance():
    # angle_q6 stores 1/64 deg; distance_q2 stores 1/4 mm
    point, _ = parse_node(make_node(12.5, 123, 5))
    assert round(point.angle_deg, 3) == 12.5
    assert point.dist_mm == 123


# --- driver iteration through a fake transport ---

def _stream(nodes, descriptor=None, prefix=b""):
    if descriptor is None:
        descriptor = make_descriptor()
    return prefix + descriptor + b"".join(nodes)


def test_driver_skips_descriptor_then_parses_nodes():
    nodes = [make_node(float(a), 1000 + a, 30) for a in range(5)]
    lidar = Lidar.open("RPLIDAR-A1", transport=FakeTransport(_stream(nodes)))
    pts = list(islice(lidar.points(), 5))
    assert [round(p.angle_deg) for p in pts] == [0, 1, 2, 3, 4]
    assert [p.dist_mm for p in pts] == [1000, 1001, 1002, 1003, 1004]


def test_driver_writes_scan_command_on_iteration():
    t = FakeTransport(_stream([make_node(1.0, 1000, 10)]))
    lidar = Lidar.open("RPLIDAR-A1", transport=t)
    next(iter(lidar.points()))
    assert t.written.hex().startswith("a520")   # SCAN sent before streaming


def test_driver_resyncs_past_garbage_before_descriptor():
    nodes = [make_node(float(a), 800, 20) for a in range(4)]
    # leading junk that does not contain the descriptor sync pair
    stream = _stream(nodes, prefix=b"\x00\x11\x22\x33")
    lidar = Lidar.open("RPLIDAR-A1", transport=FakeTransport(stream))
    pts = list(islice(lidar.points(), 4))
    assert len(pts) == 4
    assert all(isinstance(p, ScanPoint) for p in pts)


def test_driver_resyncs_on_bad_node_check_bits():
    good = [make_node(float(a), 700, 15) for a in range(3)]
    # Insert a single byte with bad check bits right after the descriptor.
    # Its presence shifts the stream by one byte; the parser must drop it and
    # realign on the following good nodes.
    bad_byte = b"\x00"                  # byte0 with syncbit=inverse=0 -> invalid
    stream = make_descriptor() + bad_byte + b"".join(good)
    lidar = Lidar.open("RPLIDAR-A1", transport=FakeTransport(stream))
    pts = list(islice(lidar.points(), 3))
    assert [round(p.angle_deg) for p in pts] == [0, 1, 2]
    assert pts[0].dist_mm == 700


def test_scans_split_on_start_flag():
    # rotation 1: start-flag node then fillers; rotation 2: another start flag.
    rot1 = [make_node(0.0, 1000, 10, scan_start=True)] + \
           [make_node(float(a), 1000, 10) for a in (90, 180, 270)]
    rot2 = [make_node(0.0, 1000, 10, scan_start=True)] + \
           [make_node(float(a), 1000, 10) for a in (90, 180, 270)]
    stream = _stream(rot1 + rot2)
    lidar = Lidar.open("RPLIDAR-A1", transport=FakeTransport(stream))
    # scans() in core splits on angle wrap (drop > 180 deg); 270 -> 0 wraps.
    scans = list(islice(lidar.scans(), 1))
    assert len(scans) == 1
    assert len(scans[0]) >= 4


def test_bad_descriptor_type_is_rejected_then_real_one_used():
    # First descriptor has the wrong type byte; a correct one follows.
    nodes = [make_node(float(a), 600, 12) for a in range(3)]
    stream = (make_descriptor(type_byte=0x06) +      # DEV_HEALTH type, not MEAS
              make_descriptor() + b"".join(nodes))
    lidar = Lidar.open("RPLIDAR-A1", transport=FakeTransport(stream))
    pts = list(islice(lidar.points(), 3))
    assert [round(p.angle_deg) for p in pts] == [0, 1, 2]


# --- motor / command methods ---

def test_stop_writes_stop_command():
    t = FakeTransport()
    lidar = Lidar.open("RPLIDAR-A1", transport=t)
    lidar.stop()
    assert t.written.hex() == "a525"


# --- A1 vs C1 registration / baud ---

@pytest.mark.parametrize("alias", ["RPLIDAR-A1", "RPLIDAR_A1"])
def test_a1_aliases_resolve(alias):
    cls = driver_for(alias)
    assert cls.MODEL_NAME == "RPLIDAR A1"
    assert cls.DEFAULT_BAUD == 115200


@pytest.mark.parametrize("alias", ["RPLIDAR-C1", "RPLIDAR_C1"])
def test_c1_aliases_resolve(alias):
    cls = driver_for(alias)
    assert cls.MODEL_NAME == "RPLIDAR C1"
    assert cls.DEFAULT_BAUD == 460800


def test_c1_parses_like_a1():
    nodes = [make_node(float(a), 900, 25) for a in range(3)]
    lidar = Lidar.open("RPLIDAR-C1", transport=FakeTransport(_stream(nodes)))
    pts = list(islice(lidar.points(), 3))
    assert [p.dist_mm for p in pts] == [900, 900, 900]
    assert [round(p.angle_deg) for p in pts] == [0, 1, 2]
