# See what your LiDAR sees: a live browser plot for `lds2d`

*Draft — announcement / blog post*

A spinning LiDAR is one of those parts that feels like magic right up until you
plug it in and get… a wall of numbers. `angle 182.40  dist_mm 1043  quality 71`,
forty thousand times a second. You believe it's working. You can't *see* that
it's working.

So the latest `lds2d` release adds a way to see it. One command, any browser:

```
pip install 'lds2d[viz]'
lds2d viz
```

Open `http://<your-pi>:8080` from your laptop and the room draws itself — a
live polar plot of every return, the walls and furniture sweeping into place at
5–6 Hz. No desktop on the Pi, no X11, no ROS, no rviz. Just a web page.

![the live plot]( ) <!-- TODO: drop in a screenshot of the polar view -->

## What `lds2d` is

If you're new here: [`lds2d`](https://github.com/kaiaai/lds2d) is a small,
pure-Python driver for 2D LiDARs on Linux and the Raspberry Pi — a Pythonic port
of the [kaiaai/LDS](https://github.com/kaiaai/LDS) C++ library. Where the C++
side targets Arduino with registered callbacks, `lds2d` gives you plain
iterators:

```python
from lds2d import Lidar

with Lidar.open("LD14P", "/dev/serial0") as lidar:
    for scan in lidar.scans():            # one full rotation at a time
        pts = scan.valid_points
        print(f"{scan.scan_freq_hz:.1f} Hz  {len(pts)} points")
```

It speaks **23 LiDAR models** today — across LDROBOT (LD14P/LD19/LD06/STL19P),
YDLIDAR (X2/X3/X4/SCL/T-mini), RPLIDAR (A1/C1), 3irobotix (Delta-2A/2B/2D/2G,
LDS08RR), Neato/Xiaomi (XV11, LDS01RR, LDS02RR), Camsense X1 and the Hitachi-LG
HLS-LFCD2 (TurtleBot3 LDS-01) — including running the PID + PWM motor loop for
the ones that have no onboard speed control and need the Pi to spin them. Each is
a faithful port of the kaiaai/LDS C++ driver, unit-tested against recorded byte
streams; the models we haven't yet re-checked on physical hardware are flagged as
such.

## The visualizer, end to end

The browser view is deliberately boring on the inside, which is the point. Three
small pieces:

- **A background reader thread** pulls full rotations off whatever driver you
  opened — `lidar.scans()` — and hands each one to a buffer.
- **A thread-safe latest-scan buffer** holds exactly one rotation, converted to a
  compact JSON-ready dict of valid points. The web request thread reads it; the
  reader thread writes it; a lock keeps them from tearing.
- **A tiny Flask app** serves two routes: `/scan.json` (the latest scan) and `/`
  (a single, dependency-free HTML page). The page polls every ~120 ms and redraws
  a `<canvas>` in polar coordinates — points coloured by signal strength, a range
  ring that auto-scales to the room, and a HUD with the live scan rate and point
  count.

Because the visualizer reads through the same `scans()` iterator every driver
already exposes, it works with *all 23* sensors for free — including the
host-driven-motor ones, where simply iterating keeps the motor spinning:

```
lds2d --model LDS02RR --pwm software viz
```

Prefer to wire it into your own app? The Python entry point is one call:

```python
from lds2d import Lidar
from lds2d.viz import serve

with Lidar.open("LD14P", "/dev/serial0") as lidar:
    serve(lidar, port=8080)
```

## Tested without touching hardware

`lds2d` has a rule: every driver is unit-tested against recorded byte streams, so
the whole suite runs on a laptop with no sensor attached. The visualizer keeps
that rule.

The serial port and the spinning motor are pushed to the edges; the logic in the
middle is plain, testable code. The scan→JSON conversion is a pure function. The
buffer is exercised by four threads hammering it at once to prove it never tears.
The reader is driven by a *fake* LiDAR that yields a fixed list of scans, and a
deliberately exploding one to prove a yanked cable ends the thread quietly instead
of crashing the server. The Flask routes are checked with Flask's test client —
no browser, no port. And an end-to-end test feeds real LD14P packet bytes through
the actual parser, through the buffer, out of the HTTP endpoint, and asserts the
points that come back.

The hardware-dependent surface is small enough to eyeball; everything else has a
test. That's the whole bet.

## Get it

```
pip install 'lds2d[viz]'
lds2d viz
```

It pairs with the step-by-step Raspberry Pi tutorials this library grew out of:

- [Connect & read the LD14P on a Raspberry Pi](https://makerspet.com/blog/tutorial-connect-ldrobot-ld14p-lidar-to-raspberry-pi-python/)
- [Control the LD14P motor](https://makerspet.com/blog/tutorial-control-ldrobot-ld14p-lidar-motor-raspberry-pi-python/)

The driver architecture is built to grow — and it has: from three models to 23 in
one pass, each ported from the kaiaai/LDS C++ and verified. If you've got a 2D
LiDAR and a Pi, there's a good chance it's already supported — give it a spin and
watch the room appear.

*`lds2d` is Apache-2.0. The LiDAR that inspired it lives at
[makerspet.com](https://makerspet.com/product/ldrobot-ld14p-lidar/).*
