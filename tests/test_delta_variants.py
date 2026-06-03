"""Hardware-free tests for the Delta-2B/2D/2G and LDS08RR variant drivers.

Build valid 0xAA Delta packets in memory, feed them through a fake transport with
a fake motor, and assert: angle/distance/quality parsing, the per-variant
packets-per-scan angle step (2G=15 vs 16), checksum rejection + resync, scan
grouping, and per-model baud/registration wiring.
"""
from itertools import islice

from lds2d import Lidar, ScanPoint
from lds2d.core import driver_for
from lds2d.drivers.threeirobotix_delta_variants import (
    Delta2B, Delta2D, Delta2G, Lds08RR,
)


def make_packet(scan_freq_hz, start_angle_deg, samples, data_type=0xAD):
    """Build a valid Delta packet. `samples` is a list of (quality, dist_mm)."""
    n = len(samples)
    body = bytearray()
    body.append(0xAA)                                  # start
    body += b"\x00\x00"                                # packet_length placeholder
    body += bytes([0x01, 0x61, data_type])             # version, type, data_type
    data_length = 5 + 3 * n
    body += bytes([(data_length >> 8) & 0xFF, data_length & 0xFF])
    body.append(int(round(scan_freq_hz * 20)) & 0xFF)  # scan_freq_x20
    body += b"\x00\x00"                                # offset_angle (ignored)
    sa = int(round(start_angle_deg * 100))
    body += bytes([(sa >> 8) & 0xFF, sa & 0xFF])       # start_angle_x100 BE
    for quality, dist_mm in samples:
        dx4 = int(round(dist_mm * 4))
        body += bytes([quality & 0xFF, (dx4 >> 8) & 0xFF, dx4 & 0xFF])
    packet_length = len(body)
    body[1] = (packet_length >> 8) & 0xFF
    body[2] = packet_length & 0xFF
    checksum = sum(body) & 0xFFFF
    body += bytes([(checksum >> 8) & 0xFF, checksum & 0xFF])
    return bytes(body)


class FakeTransport:
    def __init__(self, data=b""):
        self.data, self.pos = bytes(data), 0

    def read(self, n):
        chunk = self.data[self.pos:self.pos + n]
        self.pos += len(chunk)
        return chunk

    def close(self):
        pass


class FakeMotor:
    def __init__(self):
        self.duties = []
        self.closed = False

    def set_duty(self, d):
        self.duties.append(d)

    def close(self):
        self.closed = True


# -- registration / metadata --------------------------------------------------

def test_registration_and_baud():
    assert driver_for("DELTA-2B") is Delta2B
    assert driver_for("DELTA_2B") is Delta2B
    assert driver_for("3IROBOTIX_DELTA_2B") is Delta2B
    assert driver_for("delta-2d") is Delta2D
    assert driver_for("DELTA2G") is Delta2G
    assert driver_for("LDS08RR") is Lds08RR
    assert driver_for("LDS-08RR") is Lds08RR
    assert Delta2B.DEFAULT_BAUD == 230400          # only Delta-2B raises the baud
    assert Delta2D.DEFAULT_BAUD == 115200
    assert Delta2G.DEFAULT_BAUD == 115200
    assert Lds08RR.DEFAULT_BAUD == 115200


def test_packets_per_scan_constants():
    assert Delta2B.PACKETS_PER_SCAN == 16
    assert Delta2D.PACKETS_PER_SCAN == 16
    assert Delta2G.PACKETS_PER_SCAN == 15          # the 2G C++ override
    assert Lds08RR.PACKETS_PER_SCAN == 16
    # All host-driven: 6 Hz target, 0.6 initial duty, gains = C++/60.
    for cls in (Delta2B, Delta2D, Delta2G, Lds08RR):
        assert cls.TARGET_RPM == 360.0
        assert cls.INITIAL_DUTY == 0.6
        assert abs(cls.PID_KP - 0.3 / 60.0) < 1e-12
        assert abs(cls.PID_KI - 0.1 / 60.0) < 1e-12


# -- parsing through the live driver loop -------------------------------------

def _drive(model, data, motor=None):
    motor = motor or FakeMotor()
    return Lidar.open(model, transport=FakeTransport(data), motor=motor), motor


def test_decode_speed_and_points_16():
    data = make_packet(6.0, 0.0, [(200, 1000), (180, 1001), (210, 1002), (190, 1003)])
    lidar, _ = _drive("DELTA-2B", data)
    pts = list(islice(lidar.points(), 4))
    assert [p.dist_mm for p in pts] == [1000, 1001, 1002, 1003]
    assert [p.quality for p in pts] == [200, 180, 210, 190]
    # 16 packets/scan, 4 samples -> 360/(16*4) = 5.625 deg step
    assert pts[0].angle_deg == 0.0
    assert abs(pts[1].angle_deg - 5.625) < 1e-6
    assert lidar.get_scan_freq() == 6.0


def test_delta_2g_uses_15_packets_per_scan():
    # 2G differs from the others only in the angle step: 360/(15*4) = 6.0 deg.
    data = make_packet(6.0, 0.0, [(200, 500)] * 4)
    g, _ = _drive("DELTA-2G", data)
    d, _ = _drive("DELTA-2D", data)
    gp = list(islice(g.points(), 4))
    dp = list(islice(d.points(), 4))
    assert abs(gp[1].angle_deg - 6.0) < 1e-6        # 360/(15*4)
    assert abs(dp[1].angle_deg - 5.625) < 1e-6      # 360/(16*4)


def test_start_angle_offsets_points():
    data = make_packet(6.0, 90.0, [(200, 500)] * 4)
    lidar, _ = _drive("LDS08RR", data)
    pts = list(islice(lidar.points(), 4))
    assert pts[0].angle_deg == 90.0


def test_distance_quarter_mm_scaling():
    data = make_packet(6.0, 0.0, [(200, 1234)])
    lidar, _ = _drive("DELTA-2D", data)
    pts = list(islice(lidar.points(), 1))
    assert pts[0].dist_mm == 1234                    # dist*4 / 4 round-trips


def test_speed_only_packet_has_no_points_then_real_packet():
    # 0xAE speed-only yields no points; the following 0xAD packet still parses.
    data = (make_packet(6.0, 0.0, [], data_type=0xAE)
            + make_packet(6.0, 10.0, [(200, 777)] * 4))
    lidar, _ = _drive("DELTA-2B", data)
    pts = list(islice(lidar.points(), 4))
    assert all(p.dist_mm == 777 for p in pts)
    assert abs(pts[0].angle_deg - 10.0) < 1e-6


def test_bad_checksum_then_resync():
    good = make_packet(6.0, 0.0, [(200, 800)] * 4)
    bad = bytearray(make_packet(6.0, 0.0, [(150, 600)] * 4))
    bad[-1] ^= 0xFF                                  # corrupt the checksum
    data = bytes(bad) + good                         # bad dropped, good recovered
    lidar, _ = _drive("DELTA-2B", data)
    pts = list(islice(lidar.points(), 4))
    assert [p.dist_mm for p in pts] == [800, 800, 800, 800]


def test_resync_past_leading_garbage():
    good = make_packet(6.0, 0.0, [(200, 700)] * 4)
    data = b"\xAA\x00\x11garbage" + good             # bogus 0xAA header, then valid
    lidar, _ = _drive("DELTA-2G", data)
    pts = list(islice(lidar.points(), 4))
    assert [p.dist_mm for p in pts] == [700, 700, 700, 700]


def test_scan_grouping_on_angle_wrap():
    # Two packets: one near end-of-scan, one wrapping back to ~0 -> two scans.
    p1 = make_packet(6.0, 350.0, [(200, 100)] * 4)
    p2 = make_packet(6.0, 0.0, [(200, 200)] * 4)
    lidar, _ = _drive("DELTA-2B", p1 + p2)
    scans = list(islice(lidar.scans(), 1))
    assert len(scans) == 1                            # first scan closes at the wrap
    assert all(p.dist_mm == 100 for p in scans[0].points)


def test_driver_drives_motor_and_close_stops_it():
    motor = FakeMotor()
    data = make_packet(6.0, 0.0, [(200, 1000)] * 4) * 3
    lidar, motor = _drive("DELTA-2D", data, motor)
    list(islice(lidar.points(), 4))
    assert motor.duties and motor.duties[0] == 0.6   # initial duty
    lidar.close()
    assert motor.duties[-1] == 0.0 and motor.closed
