# lds2d

Python driver for 2D LiDARs — a Pythonic port of the [kaiaai/LDS](https://github.com/kaiaai/LDS) C++ library.

Where the C++ library targets Arduino with registered callbacks, `lds2d` targets
Linux / Raspberry Pi and gives you plain iterators: loop over individual points
or over full 360° scans.

> **Supported: LDROBOT LD14P and Xiaomi LDS02RR.** The driver architecture is built
> to grow — more models from the LDS family (Delta-2A, YDLIDAR, …) are planned.

## Install

```
pip install lds2d

# for host-driven-motor LiDARs (the LDS02RR needs the Pi to spin it):
pip install 'lds2d[pwm]'
```

## Quick start

```python
from lds2d import Lidar

with Lidar.open("LD14P", "/dev/serial0") as lidar:
    for scan in lidar.scans():            # one full rotation at a time
        pts = scan.valid_points
        print(f"{scan.scan_freq_hz:.1f} Hz  {len(pts)} points")
        for p in pts:
            print(p.angle_deg, p.dist_mm, p.quality)
```

Want a flat stream instead of grouped scans? Use `lidar.points()`.

### Motor control (LD14P)

```python
with Lidar.open("LD14P", "/dev/serial0") as lidar:
    lidar.set_scan_freq(6)     # 2–8 Hz
    lidar.stop()               # stop the motor (data stream halts)
    lidar.start()              # spin back up
    print(lidar.get_scan_freq())
```

### Xiaomi LDS02RR (host-driven motor)

The LDS02RR has no onboard motor controller — it only streams data while the host
spins it at ~5 Hz. `lds2d` runs that PID + PWM loop for you: just iterating drives
the motor, and leaving the `with` block stops it.

```python
# needs the [pwm] extra and the Maker's Pet LDS02RR adapter
with Lidar.open("LDS02RR", "/dev/serial0", pwm="software", pwm_pin=18) as lidar:
    for scan in lidar.scans():            # the motor is held at 5 Hz for you
        print(f"{scan.scan_freq_hz:.1f} Hz  {len(scan.valid_points)} points")
```

`pwm="software"` drives any GPIO via gpiozero (tested). `pwm="hardware"` uses Pi
hardware PWM (`pwm_channel`/`pwm_chip`) and is supported but not yet hardware-verified.
Tune with `target_hz=`, `kp=`, `ki=`, `kd=`.

## Command line

```
lds2d read                 # summarized: one line per full scan
lds2d read --raw           # one line per measurement
lds2d --port /dev/ttyUSB0 read

lds2d motor status
lds2d motor stop
lds2d motor start
lds2d motor speed 6        # set 6 Hz

# LDS02RR: read drives the motor (software PWM on GPIO18)
lds2d --model LDS02RR --pwm software --pwm-pin 18 read
```

## Wiring & setup (Raspberry Pi)

See the step-by-step tutorials, which this library grew out of:

- [Connect & read the LD14P on a Raspberry Pi](https://makerspet.com/blog/tutorial-connect-ldrobot-ld14p-lidar-to-raspberry-pi-python/)
- [Control the LD14P motor](https://makerspet.com/blog/tutorial-control-ldrobot-ld14p-lidar-motor-raspberry-pi-python/)

In short: LiDAR 5V→Pin2, GND→Pin6, LiDAR TX→GPIO15/Pin10 (reading), LiDAR
RX→GPIO14/Pin8 (motor commands). 3.3 V logic, no level shifter.

## Extending

A driver subclasses `LidarDriver`, implements `_packets()` (yielding
`(scan_freq_hz, [ScanPoint, ...])`), and registers itself:

```python
from lds2d.core import LidarDriver, ScanPoint, register

@register("MY_MODEL")
class MyModel(LidarDriver):
    DEFAULT_BAUD = 115200
    def _packets(self):
        ...
```

`points()` and `scans()` come for free. The transport is any object with
`read(n)` / `write(data)` / `close()`, so drivers are unit-tested against
recorded byte streams — no hardware required (see `tests/`).

## Development

```
pip install -e ".[dev]"
pytest
```

## License

Apache License 2.0 — see [LICENSE](LICENSE). The LiDAR which inspired this
library is available at [makerspet.com](https://makerspet.com/product/ldrobot-ld14p-lidar/).
