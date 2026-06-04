# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""A synthetic LiDAR so you can try lds2d — and especially ``lds2d viz`` — with
no hardware at all.

``DemoLidar`` ray-casts a small 2D scene (four walls, a couple of pillars, a desk,
and a person pacing back and forth) into a full 360° scan, frame by frame, so the
browser radar shows a recognisable room that *moves*. It speaks the exact same
``scans()`` / ``points()`` API as a real driver, needs no serial port or GPIO, and
is deterministic given a seed.

    from lds2d.demo import DemoLidar
    from lds2d.viz import serve
    serve(DemoLidar())                      # http://localhost:8080

On the CLI it's just a flag:  ``lds2d viz --demo``  /  ``lds2d read --demo``.
"""
from __future__ import annotations

import math
import random
import time
from typing import List, Optional

from .core import LidarDriver, ScanPoint

# --- the scene, in millimetres; the LiDAR sits at the origin -----------------
# A closed rectangular room (the LiDAR is inside it, off-centre), a desk, two
# pillars, plus a person who walks — added per frame in ``_moving_person``.

def _rect(x0: float, y0: float, x1: float, y1: float):
    """Four wall segments of an axis-aligned rectangle."""
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


_WALLS = _rect(-2200, -2400, 3000, 1600)           # the room
_DESK = _rect(1000, 200, 2000, 800)                # a desk block
_SEGMENTS = _WALLS + _DESK
_PILLARS = [(-1300, -700, 230), (2200, -1400, 170)]   # (cx, cy, radius)

_MAX_RANGE_MM = 6000


def _ray_segment(dx: float, dy: float, seg) -> Optional[float]:
    """Distance from the origin along unit dir (dx,dy) to a segment, or None."""
    (ax, ay), (bx, by) = seg
    ex, ey = bx - ax, by - ay
    det = ex * dy - dx * ey
    if abs(det) < 1e-9:                 # ray parallel to the segment
        return None
    t = (ex * ay - ey * ax) / det       # distance along the ray
    s = (dx * ay - dy * ax) / det       # position along the segment, 0..1
    if t > 0 and 0.0 <= s <= 1.0:
        return t
    return None


def _ray_circle(dx: float, dy: float, cx: float, cy: float, r: float) -> Optional[float]:
    """Distance from the origin along unit dir (dx,dy) to a circle, or None."""
    b = -2.0 * (dx * cx + dy * cy)
    c = cx * cx + cy * cy - r * r
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    t = (-b - sq) / 2.0
    if t > 0.0:
        return t
    t = (-b + sq) / 2.0
    return t if t > 0.0 else None


class DemoLidar(LidarDriver):
    """A hardware-free LiDAR that streams a synthetic, animated 2D scene."""

    MODEL_NAME = "Demo (synthetic scene)"
    NEEDS_TRANSPORT = False

    def __init__(self, rate_hz: float = 5.0, points_per_scan: int = 360, seed: int = 1):
        super().__init__(transport=None)        # no serial port
        self.rate_hz = rate_hz
        self.points_per_scan = points_per_scan
        self._seed = seed
        self._frame = 0

    # -- the moving actor --
    @staticmethod
    def _moving_person(frame: int):
        """(cx, cy, radius) of the pacing person for this frame."""
        phase = frame * 0.07
        cx = -200.0 + 1400.0 * (0.5 + 0.5 * math.sin(phase))
        cy = -1700.0 + 250.0 * math.sin(phase * 1.6)
        return (cx, cy, 200.0)

    def render_scan(self, frame: int) -> List[ScanPoint]:
        """Ray-cast one full 360° scan. Deterministic for a given (seed, frame)."""
        rng = random.Random(self._seed * 1_000_003 + frame)
        circles = _PILLARS + [self._moving_person(frame)]
        pts: List[ScanPoint] = []
        for i in range(self.points_per_scan):
            angle = i * 360.0 / self.points_per_scan
            rad = math.radians(angle)
            dx, dy = math.sin(rad), math.cos(rad)      # 0° points +y ("up")
            best = None
            for seg in _SEGMENTS:
                t = _ray_segment(dx, dy, seg)
                if t is not None and (best is None or t < best):
                    best = t
            for (cx, cy, r) in circles:
                t = _ray_circle(dx, dy, cx, cy, r)
                if t is not None and (best is None or t < best):
                    best = t
            # occasional dropout, like a real sensor missing a return
            if best is None or best > _MAX_RANGE_MM or rng.random() < 0.012:
                pts.append(ScanPoint(angle, 0, 0))
                continue
            dist = best * (1.0 + rng.uniform(-0.004, 0.004))   # ranging noise
            dist_mm = int(round(dist))
            quality = max(8, min(255, int(260 - dist_mm / 22) + rng.randint(-12, 12)))
            pts.append(ScanPoint(angle, dist_mm, quality))
        return pts

    # -- the lds2d driver interface --
    def _packets(self):
        while True:
            yield self.rate_hz, self.render_scan(self._frame)
            self._frame += 1
            if self.rate_hz > 0:
                time.sleep(1.0 / self.rate_hz)         # pace like real hardware

    def get_scan_freq(self, listen_s: float = 1.0) -> Optional[float]:
        return self.rate_hz

    def close(self) -> None:                            # no transport to close
        pass
