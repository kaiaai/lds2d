"""Hardware-free tests for the LDS02RR driver: build XV11 packets in memory,
feed them through a fake transport with a fake motor, check parsing + wiring."""
import struct
from itertools import islice

from lds2d import Lidar, ScanPoint
from lds2d.drivers.xiaomi_lds02rr import valid_packet, parse_packet, iter_packets

N_QUADS = 4


def make_packet(index, rpm, dists, quality=200, flags=0):
    """Assemble a valid 22-byte XV11/LDS02RR packet with a correct checksum."""
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


def test_checksum_and_parse():
    pkt = make_packet(0xA0, 300.0, [1000, 1001, 1002, 1003])
    assert valid_packet(pkt)
    freq, pts = parse_packet(pkt)
    assert round(freq, 3) == 5.0                 # 300 RPM = 5 Hz
    assert [p.angle_deg for p in pts] == [0, 1, 2, 3]
    assert [p.dist_mm for p in pts] == [1000, 1001, 1002, 1003]
    assert all(p.quality == 200 for p in pts)


def test_index_maps_to_angle():
    _, pts = parse_packet(make_packet(0xA0 + 45, 300.0, [500] * 4))  # index 45 -> 180 deg
    assert [p.angle_deg for p in pts] == [180, 181, 182, 183]


def test_bad_checksum_rejected():
    pkt = bytearray(make_packet(0xA0, 300.0, [800] * 4))
    pkt[21] ^= 0xFF
    assert not valid_packet(bytes(pkt))


def test_flag_clears_distance():
    _, pts = parse_packet(make_packet(0xA0, 300.0, [1234] * 4, flags=1 << 7))
    assert all(p.dist_mm == 0 for p in pts)      # invalid-data flag -> 0


def test_iter_packets_resyncs():
    pkt = make_packet(0xA0, 300.0, [600] * 4)
    buf = bytearray(b"\x00\x11garbage" + pkt + pkt)
    assert len(list(iter_packets(buf))) == 2


def test_driver_reads_and_drives_motor():
    motor = FakeMotor()
    data = make_packet(0xA0, 300.0, [1000, 2000, 0, 1500]) * 3
    lidar = Lidar.open("LDS02RR", transport=FakeTransport(data), motor=motor)
    pts = list(islice(lidar.points(), 4))        # one packet's worth
    assert all(isinstance(p, ScanPoint) for p in pts)
    assert [p.dist_mm for p in pts] == [1000, 2000, 0, 1500]
    # the motor was enabled at the initial 50% duty as soon as reading began
    assert motor.duties and motor.duties[0] == 0.5
    assert lidar.get_scan_freq() == 5.0          # from the reported 300 RPM


def test_close_stops_motor():
    motor = FakeMotor()
    lidar = Lidar.open("LDS02RR", transport=FakeTransport(), motor=motor)
    lidar.start()
    lidar.close()
    assert motor.duties[-1] == 0.0 and motor.closed


def test_set_scan_freq_changes_setpoint():
    lidar = Lidar.open("LDS02RR", transport=FakeTransport(), motor=FakeMotor())
    lidar.set_scan_freq(6.0)
    assert lidar._pid.setpoint == 360.0          # 6 Hz -> 360 RPM
    for bad in (0, -1):
        try:
            lidar.set_scan_freq(bad)
        except ValueError:
            continue
        raise AssertionError("non-positive scan rate should be rejected")
