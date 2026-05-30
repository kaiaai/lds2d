"""Hardware-free tests for the Delta-2A driver: build Delta packets in memory,
feed them through a fake transport with a fake motor, check parsing + wiring."""
from itertools import islice

from lds2d import Lidar, ScanPoint
from lds2d.drivers.threeirobotix_delta_2a import decode_delta, _PACKETS_PER_SCAN


def make_packet(scan_freq_hz, start_angle_deg, samples, data_type=0xAD):
    """Build a valid Delta-2A packet. `samples` is a list of (quality, dist_mm)."""
    n = len(samples)
    body = bytearray()
    body.append(0xAA)                                  # start
    body += b"\x00\x00"                                # packet_length placeholder
    body += bytes([0x01, 0x61, data_type])             # version, type, data_type
    data_length = 5 + 3 * n                            # scan_freq + offset + start + samples
    body += bytes([(data_length >> 8) & 0xFF, data_length & 0xFF])
    body.append(int(round(scan_freq_hz * 20)) & 0xFF)  # scan_freq_x20
    body += b"\x00\x00"                                # offset_angle (ignored)
    sa = int(round(start_angle_deg * 100))
    body += bytes([(sa >> 8) & 0xFF, sa & 0xFF])       # start_angle_x100 (big-endian)
    for quality, dist_mm in samples:
        dx4 = int(round(dist_mm * 4))
        body += bytes([quality & 0xFF, (dx4 >> 8) & 0xFF, dx4 & 0xFF])
    packet_length = len(body)                          # everything before the checksum
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


def test_decode_speed_and_points():
    pkt = make_packet(6.0, 0.0, [(200, 1000), (180, 1001), (210, 1002), (190, 1003)])
    (freq, pts), = list(decode_delta(bytearray(pkt)))
    assert round(freq, 3) == 6.0
    assert len(pts) == 4
    assert [p.dist_mm for p in pts] == [1000, 1001, 1002, 1003]
    assert [p.quality for p in pts] == [200, 180, 210, 190]
    # angle spacing = 360 / (16 packets * 4 samples) = 5.625 deg
    assert pts[0].angle_deg == 0.0
    assert abs(pts[1].angle_deg - 5.625) < 1e-6


def test_start_angle_offsets_points():
    (_, pts), = list(decode_delta(bytearray(make_packet(6.0, 90.0, [(200, 500)] * 4))))
    assert pts[0].angle_deg == 90.0


def test_speed_only_packet_has_no_points():
    pkt = make_packet(6.0, 0.0, [], data_type=0xAE)
    (freq, pts), = list(decode_delta(bytearray(pkt)))
    assert round(freq, 3) == 6.0 and pts == []


def test_bad_checksum_rejected():
    pkt = bytearray(make_packet(6.0, 0.0, [(200, 800)] * 4))
    pkt[-1] ^= 0xFF
    assert list(decode_delta(pkt)) == []          # dropped, nothing yielded


def test_resync_past_garbage():
    pkt = make_packet(6.0, 0.0, [(200, 700)] * 4)
    buf = bytearray(b"\xAA\x00\x11rubbish" + pkt + pkt)
    assert len(list(decode_delta(buf))) == 2


def test_distance_quarter_mm_scaling():
    (_, pts), = list(decode_delta(bytearray(make_packet(6.0, 0.0, [(200, 1234)]))))
    assert pts[0].dist_mm == 1234                 # round-trips dist*4 / 4


def test_driver_reads_and_drives_motor():
    motor = FakeMotor()
    data = make_packet(6.0, 0.0, [(200, 1000), (200, 2000), (200, 1500), (200, 800)]) * 3
    lidar = Lidar.open("DELTA-2A", transport=FakeTransport(data), motor=motor)
    pts = list(islice(lidar.points(), 4))
    assert [p.dist_mm for p in pts] == [1000, 2000, 1500, 800]
    assert motor.duties and motor.duties[0] == 0.6   # Delta initial duty
    assert lidar.get_scan_freq() == 6.0


def test_close_stops_motor():
    motor = FakeMotor()
    lidar = Lidar.open("DELTA-2A", transport=FakeTransport(), motor=motor)
    lidar.start()
    lidar.close()
    assert motor.duties[-1] == 0.0 and motor.closed
