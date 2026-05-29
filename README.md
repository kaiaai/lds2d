# lds2d

Python driver for 2D LiDARs — a Pythonic port of the [kaiaai/LDS](https://github.com/kaiaai/LDS) C++ library.

Where the C++ library targets Arduino with registered callbacks, `lds2d` targets
Linux / Raspberry Pi and gives you plain iterators: loop over individual points
or over full 360° scans.

> **v0.1 supports the LDROBOT LD14P.** The driver architecture is built to grow —
> more models from the LDS family (Delta-2A, LDS02RR, YDLIDAR, …) are planned.

## Install

```
pip install lds2d
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

## Command line

```
lds2d read                 # summarized: one line per full scan
lds2d read --raw           # one line per measurement
lds2d --port /dev/ttyUSB0 read

lds2d motor status
lds2d motor stop
lds2d motor start
lds2d motor speed 6        # set 6 Hz
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
