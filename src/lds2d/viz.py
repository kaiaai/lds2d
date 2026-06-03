# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Live web visualizer for lds2d.

Point any browser at a running ``lds2d viz`` and watch the LiDAR sweep in a
polar plot, in real time. A background thread reads scans from any driver and
keeps the most recent one; a tiny Flask app exposes it as JSON and serves a
dependency-free HTML canvas that polls and redraws.

Like the rest of lds2d, the moving parts are hardware-free testable: the
scan→dict conversion and the latest-scan buffer need neither Flask nor a sensor.
Flask itself is an optional extra (``pip install 'lds2d[viz]'``); it is imported
lazily so importing this module never requires it.

    from lds2d import Lidar
    from lds2d.viz import serve

    with Lidar.open("LD14P", "/dev/serial0") as lidar:
        serve(lidar)                      # http://0.0.0.0:8080
"""
from __future__ import annotations

import threading
from typing import Dict, Optional

from .core import Scan


def scan_to_dict(scan: Scan) -> Dict:
    """Convert a :class:`Scan` into a JSON-serializable dict of valid points.

    Only points with a real return (``dist_mm > 0``) are included, to keep the
    payload small. Angles and distances are passed through untouched so the
    browser can plot in the sensor's own coordinate frame.
    """
    pts = [
        {"angle": p.angle_deg, "dist": p.dist_mm, "quality": p.quality}
        for p in scan.points
        if p.valid
    ]
    return {"freq_hz": round(scan.scan_freq_hz, 2), "n": len(pts), "points": pts}


_EMPTY = {"freq_hz": 0.0, "n": 0, "points": []}


class ScanBuffer:
    """Thread-safe holder for the most recent scan, as a serializable dict.

    The reader thread calls :meth:`update`; the web request thread calls
    :meth:`latest`. A lock keeps the two from tearing a dict across threads.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: Dict = dict(_EMPTY)

    def update(self, scan: Scan) -> None:
        data = scan_to_dict(scan)
        with self._lock:
            self._latest = data

    def latest(self) -> Dict:
        with self._lock:
            return self._latest


class ScanReader:
    """Pumps ``lidar.scans()`` into a :class:`ScanBuffer` on a daemon thread.

    Works with any object exposing ``scans()`` (a real driver or a fake), so it
    is testable without hardware. Iteration errors — e.g. the transport being
    closed on shutdown — end the loop quietly rather than crashing the server.
    """

    def __init__(self, lidar, buffer: ScanBuffer) -> None:
        self._lidar = lidar
        self._buffer = buffer
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def _loop(self) -> None:
        try:
            for scan in self._lidar.scans():
                if self._stop.is_set():
                    break
                self._buffer.update(scan)
        except Exception:
            pass  # transport closed / driver stopped — let the thread exit

    def start(self) -> "ScanReader":
        self._thread = threading.Thread(
            target=self._loop, name="lds2d-scan-reader", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()


def make_app(buffer: ScanBuffer, model: str = "LiDAR"):
    """Build the Flask app serving the plot (``/``) and the data (``/scan.json``).

    Importing Flask is deferred to here so the rest of the module — and its
    tests — work with the base install.
    """
    try:
        from flask import Flask, Response, jsonify
    except ImportError as e:  # pragma: no cover - exercised only without the extra
        raise ImportError(
            "the web visualizer needs Flask — install the viz extra:  "
            "pip install 'lds2d[viz]'") from e

    app = Flask(__name__)

    @app.route("/")
    def index() -> "Response":
        return Response(INDEX_HTML.replace("{{MODEL}}", model), mimetype="text/html")

    @app.route("/scan.json")
    def scan_json():
        return jsonify(buffer.latest())

    return app


def serve(lidar, host: str = "0.0.0.0", port: int = 8080) -> None:
    """Start the reader thread and run the visualizer until interrupted.

    Blocks in Flask's development server. The reader thread is a daemon, so a
    Ctrl-C that unwinds the caller's ``with`` block still stops the motor.
    """
    buffer = ScanBuffer()
    reader = ScanReader(lidar, buffer).start()
    app = make_app(buffer, model=getattr(lidar, "MODEL_NAME", "LiDAR"))
    try:
        app.run(host=host, port=port, threaded=True)
    finally:
        reader.stop()


# Single-file page: a canvas, a polar grid, and a poll-and-redraw loop. No deps.
INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>lds2d — {{MODEL}}</title>
<style>
  :root { color-scheme: dark; }
  html, body { margin: 0; height: 100%; background: #0b0e13; color: #c9d4e3;
    font: 14px/1.4 ui-monospace, "SF Mono", Menlo, Consolas, monospace; }
  #wrap { display: flex; flex-direction: column; align-items: center;
    justify-content: center; height: 100%; gap: 12px; }
  #hud { letter-spacing: .04em; }
  #hud b { color: #6cf0c2; }
  canvas { background: radial-gradient(circle at center, #11161f, #0b0e13 70%);
    border-radius: 50%; box-shadow: 0 0 60px #00000080; }
  .dim { color: #5b6776; }
</style>
</head>
<body>
<div id="wrap">
  <div id="hud" class="dim">connecting…</div>
  <canvas id="c" width="720" height="720"></canvas>
  <div class="dim">{{MODEL}} · range ring = <span id="ring">—</span> · drag-free, just watching</div>
</div>
<script>
const cv = document.getElementById("c"), ctx = cv.getContext("2d");
const hud = document.getElementById("hud"), ringLabel = document.getElementById("ring");
const CX = cv.width / 2, CY = cv.height / 2, R = cv.width / 2 - 16;
let maxRange = 4000;            // mm shown at the rim; auto-scales to the scan

function niceRange(mm) {        // round up to a tidy ring distance
  const steps = [1000, 2000, 3000, 4000, 6000, 8000, 12000, 16000];
  for (const s of steps) if (mm <= s) return s;
  return Math.ceil(mm / 8000) * 8000;
}

function grid() {
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.strokeStyle = "#1d2733"; ctx.fillStyle = "#3a4656"; ctx.lineWidth = 1;
  for (let k = 1; k <= 4; k++) {
    ctx.beginPath(); ctx.arc(CX, CY, R * k / 4, 0, 2 * Math.PI); ctx.stroke();
  }
  ctx.beginPath();
  ctx.moveTo(CX - R, CY); ctx.lineTo(CX + R, CY);
  ctx.moveTo(CX, CY - R); ctx.lineTo(CX, CY + R); ctx.stroke();
  ctx.fillText("0°", CX + 4, CY - R + 12);          // forward (sensor 0°) at top
}

function draw(scan) {
  if (scan.points.length) {
    let mx = 0; for (const p of scan.points) if (p.dist > mx) mx = p.dist;
    maxRange = niceRange(mx);
  }
  ringLabel.textContent = (maxRange / 1000).toFixed(0) + " m";
  grid();
  for (const p of scan.points) {
    const a = (p.angle - 90) * Math.PI / 180;        // 0° up, clockwise
    const r = Math.min(p.dist / maxRange, 1) * R;
    const x = CX + r * Math.cos(a), y = CY + r * Math.sin(a);
    const t = Math.max(0.25, Math.min(1, p.quality / 200));
    ctx.fillStyle = "rgba(108,240,194," + t + ")";
    ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
  }
  hud.innerHTML = "<b>" + scan.freq_hz.toFixed(1) + " Hz</b> · <b>" +
    scan.n + "</b> points";
}

async function tick() {
  try {
    const r = await fetch("/scan.json", { cache: "no-store" });
    draw(await r.json());
  } catch (e) {
    hud.textContent = "waiting for data…";
  }
}
grid();
setInterval(tick, 120);
tick();
</script>
</body>
</html>
"""
