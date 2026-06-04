"""Tests for the synthetic DemoLidar: it needs no hardware, produces a full,
plausible scan, animates between frames, and speaks the normal driver API."""
from itertools import islice

from lds2d import ScanPoint
from lds2d.demo import DemoLidar


def test_render_scan_shape_and_ranges():
    pts = DemoLidar(seed=1).render_scan(0)
    assert len(pts) == 360
    assert all(isinstance(p, ScanPoint) for p in pts)
    # angles march 0..359 in order
    assert [round(p.angle_deg) for p in pts] == list(range(360))
    valid = [p for p in pts if p.valid]
    assert len(valid) > 320                       # mostly returns, a few dropouts
    assert all(0 < p.dist_mm <= 6000 for p in valid)   # inside the synthetic room
    assert all(0 <= p.quality <= 255 for p in valid)


def test_deterministic_for_seed_and_frame():
    a = DemoLidar(seed=7).render_scan(3)
    b = DemoLidar(seed=7).render_scan(3)
    assert [(p.dist_mm, p.quality) for p in a] == [(p.dist_mm, p.quality) for p in b]


def test_scene_animates_between_frames():
    d = DemoLidar(seed=1)
    moved = sum(1 for x, y in zip(d.render_scan(0), d.render_scan(40))
                if x.dist_mm != y.dist_mm)
    assert moved > 0                              # the walking person (and noise) move


def test_scans_yields_a_full_rotation():
    # rate_hz=0 disables the inter-scan sleep so the test is instant.
    lidar = DemoLidar(rate_hz=0)
    scan = next(islice(lidar.scans(), 1))
    assert len(scan) == 360
    assert len(scan.valid_points) > 320


def test_points_flat_stream():
    lidar = DemoLidar(rate_hz=0)
    pts = list(islice(lidar.points(), 500))
    assert len(pts) == 500
    assert all(isinstance(p, ScanPoint) for p in pts)


def test_driver_api_needs_no_hardware():
    lidar = DemoLidar()                           # no port, no transport
    assert lidar.NEEDS_TRANSPORT is False
    assert lidar.get_scan_freq() == 5.0
    lidar.close()                                 # must not raise (no transport)


def test_context_manager():
    with DemoLidar(rate_hz=0) as lidar:
        assert next(islice(lidar.scans(), 1)) is not None
