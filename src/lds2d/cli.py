# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Command-line interface: ``lds2d read``, ``lds2d viz``, and ``lds2d motor``."""
from __future__ import annotations

import argparse
import sys

from . import drivers  # noqa: F401  — populates the model registry
from .core import Lidar, available_models, driver_for

DEFAULT_PORT = "/dev/serial0"


def _open(args):
    """Open the LiDAR, forwarding PWM settings only for host-driven-motor models."""
    if getattr(args, "demo", False):
        from .demo import DemoLidar           # synthetic scene, no hardware
        return DemoLidar()
    cls = driver_for(args.model)
    kwargs = {}
    if cls is not None and getattr(cls, "NEEDS_MOTOR", False):
        kwargs = dict(pwm=args.pwm, pwm_pin=args.pwm_pin, pwm_channel=args.pwm_channel,
                      pwm_chip=args.pwm_chip, pwm_freq=args.pwm_freq)
    return Lidar.open(args.model, args.port, args.baud, **kwargs)


def _cmd_read(args) -> int:
    lidar = _open(args)
    if getattr(args, "demo", False):
        print("Demo: synthesizing scans — no hardware  (Ctrl-C to stop)", file=sys.stderr)
    else:
        print(f"{lidar.MODEL_NAME}: reading {args.port} @ {args.baud or lidar.DEFAULT_BAUD} "
              f"baud  (Ctrl-C to stop)", file=sys.stderr)
    try:
        if args.raw:
            print(f"{'angle':>7}  {'dist_mm':>7}  {'quality':>7}")
            for p in lidar.points():
                if p.valid:
                    print(f"{p.angle_deg:7.2f}  {p.dist_mm:7d}  {p.quality:7d}")
        else:
            print(f"{'scan':>5}  {'freq_hz':>7}  {'n_valid':>7}  "
                  f"{'min_mm':>7}  {'max_mm':>7}")
            for n, scan in enumerate(lidar.scans()):
                valid = scan.valid_points
                if not valid:
                    continue
                d = [p.dist_mm for p in valid]
                print(f"{n:5d}  {scan.scan_freq_hz:7.1f}  {len(valid):7d}  "
                      f"{min(d):7d}  {max(d):7d}")
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
    finally:
        lidar.close()
    return 0


def _cmd_viz(args) -> int:
    from .viz import serve
    lidar = _open(args)
    shown = "localhost" if args.host in ("0.0.0.0", "") else args.host
    tag = "Demo (synthetic)" if getattr(args, "demo", False) else lidar.MODEL_NAME
    print(f"{tag}: live plot at http://{shown}:{args.http_port}  "
          f"(Ctrl-C to stop)", file=sys.stderr)
    try:
        serve(lidar, host=args.host, port=args.http_port,
              rotation=args.rotation, fixed_range=args.range)
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
    finally:
        lidar.close()
    return 0


def _cmd_motor(args) -> int:
    import time
    lidar = _open(args)
    try:
        if args.action == "status":
            hz = lidar.get_scan_freq(1.0)
            print(f"Spinning at {hz:.2f} Hz" if hz else "No data — motor appears stopped.")
        elif args.action == "start":
            lidar.start()
            time.sleep(1.0)
            hz = lidar.get_scan_freq(1.5)
            print(f"Started — spinning at {hz:.2f} Hz." if hz else
                  "Sent start, but no data yet. Check the Pi-TX → LiDAR-RX wire.")
        elif args.action == "stop":
            lidar.stop()
            time.sleep(1.5)
            hz = lidar.get_scan_freq(1.0)
            print("Stopped." if hz is None else f"Still spinning at {hz:.2f} Hz — retry?")
        elif args.action == "speed":
            if args.hz is None:
                print("'speed' needs a value, e.g. 'motor speed 6'", file=sys.stderr)
                return 2
            lidar.set_scan_freq(args.hz)
            time.sleep(0.8)
            hz = lidar.get_scan_freq(1.5)
            print(f"Set {args.hz:.1f} Hz — now reading {hz:.2f} Hz." if hz else
                  f"Sent {args.hz:.1f} Hz, but no data yet (is the motor started?).")
    finally:
        lidar.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="lds2d", description="2D LiDAR tool")
    ap.add_argument("--model", default="LDROBOT-LD14P",
                    help=f"LiDAR model (default LDROBOT-LD14P); known: {', '.join(available_models())}")
    ap.add_argument("--port", default=DEFAULT_PORT, help="serial port")
    ap.add_argument("--baud", type=int, default=None, help="override the default baud")
    # Host-driven motor (LDS02RR) PWM options; ignored by self-spinning LiDARs.
    ap.add_argument("--pwm", choices=["hardware", "software"], default="software",
                    help="motor PWM method for host-driven LiDARs (e.g. LDS02RR)")
    ap.add_argument("--pwm-pin", type=int, default=18, help="BCM GPIO for software PWM")
    ap.add_argument("--pwm-channel", type=int, default=0, help="hardware PWM channel")
    ap.add_argument("--pwm-chip", type=int, default=None, help="hardware PWM chip (Pi 5: often 2)")
    ap.add_argument("--pwm-freq", type=int, default=10000, help="PWM frequency (Hz)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    _demo_help = "stream a synthetic moving scene — no hardware needed"

    r = sub.add_parser("read", help="print live scan data")
    r.add_argument("--raw", action="store_true", help="one line per measurement")
    r.add_argument("--demo", action="store_true", help=_demo_help)
    r.set_defaults(func=_cmd_read)

    v = sub.add_parser("viz", help="live polar plot in your browser")
    v.add_argument("--host", default="0.0.0.0", help="bind address (default all interfaces)")
    # Distinct dest from the global serial --port, or the subparser default would
    # clobber it and Lidar.open() would receive the HTTP port as the serial port.
    v.add_argument("--http-port", "--port", dest="http_port", type=int, default=8080,
                   help="HTTP port (default 8080)")
    v.add_argument("--rotation", choices=["cw", "ccw"], default="ccw",
                   help="your LiDAR's spin direction; flip it if the plot looks mirrored "
                        "(default ccw)")
    v.add_argument("--range", type=int, default=0, metavar="MM",
                   help="lock the rim distance in mm (default 0 = auto-scale)")
    v.add_argument("--demo", action="store_true", help=_demo_help)
    v.set_defaults(func=_cmd_viz)

    m = sub.add_parser("motor", help="control the motor (command-driven models)")
    m.add_argument("action", choices=["start", "stop", "speed", "status"])
    m.add_argument("hz", nargs="?", type=float, help="scan rate in Hz (for 'speed')")
    m.set_defaults(func=_cmd_motor)
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
