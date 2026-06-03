"""Hardware-free tests for the Neato XV11 family drivers (Neato XV11, LDS01RR).

Build XV11 packets in memory with correct checksums, feed them through a fake
transport with a fake motor, and assert parsing (angles, distances, quality),
CCW angle mirroring, RPM->Hz scaling, checksum rejection + resync, and scan
grouping. The packet builder computes a genuine checksum, so the parser is
exercised for real (not tautologically)."""
from itertools import islice

import pytest

from lds2d import Lidar, ScanPoint
from lds2d.drivers.neato_xv11 import valid_packet, parse_packet, iter_packets

N_QUADS = 4


def make_packet(index, rpm, dists, quality=200, flags=0):
    """Assemble a valid 22-byte XV11 packet with a correct checksum."""
    pkt = bytearray(22)
    pkt[0] = 0xFA
    pkt[1] = index                       # 0xA0..0xF9
    speed = int(round(rpm * 64))
    pkt[2], pkt[3] = speed & 0xFF, (speed >> 8) & 0xFF
    for q in range(N_QUADS):
        off = 4 + q * 4
        d = dists[q]
        pkt[off] = d & 0xFF
        pkt[off + 1] = ((d >> 8) & 0x3F) | flags
        pkt[off + 2] = quality & 0xFF
        pkt[off + 3] = (quality >> 8) & 0xFF
    chk = 0
    for i in range(0, 20, 2):
        chk = (chk << 1) + (pkt[i] | (pkt[i + 1] << 8))
    cs = ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF
    pkt[20], pkt[21] = cs & 0xFF, (cs >> 8) & 0xFF
    return bytes(pkt)


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


def test_checksum_and_parse_ccw():
    # Neato XV11 / LDS01RR spin CCW: angle = (360 - a) % 360.
    pkt = make_packet(0xA0, 300.0, [1000, 1001, 1002, 1003])
    assert valid_packet(pkt)
    freq, pts = parse_packet(pkt)                 # cw defaults to False
    assert round(freq, 3) == 5.0                  # 300 RPM = 5 Hz
    # raw angles 0,1,2,3 -> mirrored 0,359,358,357
    assert [p.angle_deg for p in pts] == [0.0, 359.0, 358.0, 357.0]
    assert [p.dist_mm for p in pts] == [1000, 1001, 1002, 1003]
    assert all(p.quality == 200 for p in pts)


def test_index_maps_to_mirrored_angle():
    # index 45 -> raw start angle 180; mirrored stays in the 180s, descending.
    _, pts = parse_packet(make_packet(0xA0 + 45, 300.0, [500] * 4))
    assert [p.angle_deg for p in pts] == [180.0, 179.0, 178.0, 177.0]


def test_cw_flag_keeps_ascending_angles():
    # cw=True is the LDS02RR behaviour: no mirroring.
    _, pts = parse_packet(make_packet(0xA0, 300.0, [500] * 4), cw=True)
    assert [p.angle_deg for p in pts] == [0.0, 1.0, 2.0, 3.0]


def test_rpm_to_hz_scaling():
    freq, _ = parse_packet(make_packet(0xB0, 360.0, [700] * 4))
    assert round(freq, 3) == 6.0                  # speed=360*64 -> 360 RPM = 6 Hz


def test_bad_checksum_rejected():
    pkt = bytearray(make_packet(0xA0, 300.0, [800] * 4))
    pkt[21] ^= 0xFF
    assert not valid_packet(bytes(pkt))


def test_invalid_flag_clears_distance_and_quality():
    _, pts = parse_packet(make_packet(0xA0, 300.0, [1234] * 4, flags=1 << 7))
    assert all(p.dist_mm == 0 for p in pts)       # INVALID_DATA flag -> 0 dist
    assert all(p.quality == 0 for p in pts)       # C++ leaves quality 0 too


def test_warning_flag_clears_distance():
    _, pts = parse_packet(make_packet(0xA0, 300.0, [4095] * 4, flags=1 << 6))
    assert all(p.dist_mm == 0 for p in pts)       # STRENGTH_WARNING flag -> 0


def test_iter_packets_resyncs_after_garbage():
    pkt = make_packet(0xA0, 300.0, [600] * 4)
    buf = bytearray(b"\x00\x11garbage" + pkt + pkt)
    assert len(list(iter_packets(buf))) == 2


def test_iter_packets_drops_one_byte_on_bad_checksum():
    good = make_packet(0xA0, 300.0, [600] * 4)
    bad = bytearray(make_packet(0xA0, 300.0, [600] * 4))
    bad[21] ^= 0xFF                                # corrupt the CRC trailer
    buf = bytearray(bytes(bad) + good)
    pkts = list(iter_packets(buf))
    # the corrupted leading frame is skipped byte-by-byte; the good one parses.
    assert len(pkts) == 1
    _, pts = parse_packet(pkts[0])
    assert [p.dist_mm for p in pts] == [600, 600, 600, 600]


def test_driver_reads_and_drives_motor_neato():
    motor = FakeMotor()
    data = make_packet(0xA0, 300.0, [1000, 2000, 0, 1500]) * 3
    lidar = Lidar.open("NEATO_XV11", transport=FakeTransport(data), motor=motor)
    pts = list(islice(lidar.points(), 4))         # one packet's worth
    assert all(isinstance(p, ScanPoint) for p in pts)
    assert [p.dist_mm for p in pts] == [1000, 2000, 0, 1500]
    # CCW mirroring: raw 0,1,2,3 -> 0,359,358,357
    assert [p.angle_deg for p in pts] == [0.0, 359.0, 358.0, 357.0]
    # the motor was enabled at the initial 50% duty as soon as reading began
    assert motor.duties and motor.duties[0] == 0.5
    assert lidar.get_scan_freq() == 5.0           # from the reported 300 RPM


def test_xv11_alias_registered():
    lidar = Lidar.open("XV11", transport=FakeTransport(), motor=FakeMotor())
    assert lidar.MODEL_NAME == "Neato XV11"
    assert lidar.DEFAULT_BAUD == 115200


def test_lds01rr_registered_and_ccw():
    motor = FakeMotor()
    data = make_packet(0xA0, 300.0, [111, 222, 333, 444]) * 2
    lidar = Lidar.open("LDS01RR", transport=FakeTransport(data), motor=motor)
    assert lidar.MODEL_NAME == "Xiaomi LDS01RR"
    pts = list(islice(lidar.points(), 4))
    assert [p.dist_mm for p in pts] == [111, 222, 333, 444]
    assert [p.angle_deg for p in pts] == [0.0, 359.0, 358.0, 357.0]


def test_scans_group_into_rotations():
    # Repeating one index produces a descending-then-repeating angle sequence;
    # scans() splits whenever the angle jumps back up (a wrap by more than 180).
    data = make_packet(0xA0, 300.0, [500] * 4) * 4
    lidar = Lidar.open("NEATO_XV11", transport=FakeTransport(data), motor=FakeMotor())
    scans = list(islice(lidar.scans(), 2))
    assert len(scans) == 2
    for s in scans:
        assert all(isinstance(p, ScanPoint) for p in s.points)
        assert s.scan_freq_hz == 5.0


def test_close_stops_motor():
    motor = FakeMotor()
    lidar = Lidar.open("NEATO_XV11", transport=FakeTransport(), motor=motor)
    lidar.start()
    lidar.close()
    assert motor.duties[-1] == 0.0 and motor.closed


def test_set_scan_freq_changes_setpoint():
    lidar = Lidar.open("NEATO_XV11", transport=FakeTransport(), motor=FakeMotor())
    lidar.set_scan_freq(6.0)
    assert lidar._pid.setpoint == 360.0           # 6 Hz -> 360 RPM
    for bad in (0, -1):
        with pytest.raises(ValueError):
            lidar.set_scan_freq(bad)
