"""Hardware-free tests for the web visualizer: convert scans to JSON, pump a
fake lidar through the buffer on a thread, and (when Flask is installed) hit the
HTTP endpoints. No serial port, no motor, no browser required."""
import threading

import pytest

from lds2d.core import Scan, ScanPoint
from lds2d.viz import ScanBuffer, ScanReader, make_app, scan_to_dict


def make_scan(freq=5.0):
    return Scan(
        points=[
            ScanPoint(0.0, 1000, 200),
            ScanPoint(90.0, 0, 0),       # no return — must be dropped
            ScanPoint(180.0, 2500, 50),
        ],
        scan_freq_hz=freq,
    )


# --- scan_to_dict -----------------------------------------------------------

def test_scan_to_dict_drops_invalid_and_keeps_fields():
    d = scan_to_dict(make_scan(6.0))
    assert d["freq_hz"] == 6.0
    assert d["n"] == 2                       # the dist=0 point is excluded
    assert len(d["points"]) == 2
    first = d["points"][0]
    assert first == {"angle": 0.0, "dist": 1000, "quality": 200}
    assert all(p["dist"] > 0 for p in d["points"])


def test_scan_to_dict_rounds_freq():
    d = scan_to_dict(Scan(points=[], scan_freq_hz=5.12345))
    assert d["freq_hz"] == 5.12
    assert d["points"] == []


def test_scan_to_dict_is_json_serializable():
    import json
    json.dumps(scan_to_dict(make_scan()))    # must not raise


# --- ScanBuffer -------------------------------------------------------------

def test_buffer_starts_empty():
    assert ScanBuffer().latest() == {"freq_hz": 0.0, "n": 0, "points": []}


def test_buffer_update_returns_latest():
    buf = ScanBuffer()
    buf.update(make_scan(7.0))
    assert buf.latest()["freq_hz"] == 7.0
    assert buf.latest()["n"] == 2


def test_buffer_is_thread_safe_under_concurrent_writes():
    buf = ScanBuffer()

    def hammer():
        for _ in range(500):
            buf.update(make_scan())

    threads = [threading.Thread(target=hammer) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert buf.latest()["n"] == 2            # never torn / corrupted


# --- ScanReader against a fake lidar ----------------------------------------

class FakeLidar:
    """Yields a fixed list of scans from scans(), like a finite recording."""
    MODEL_NAME = "FAKE"

    def __init__(self, scans):
        self._scans = scans

    def scans(self):
        yield from self._scans


def test_reader_loop_fills_buffer_with_last_scan():
    buf = ScanBuffer()
    lidar = FakeLidar([make_scan(4.0), make_scan(5.0), make_scan(6.0)])
    ScanReader(lidar, buf)._loop()           # finite source returns promptly
    assert buf.latest()["freq_hz"] == 6.0


def test_reader_swallows_transport_errors():
    class Boom:
        def scans(self):
            raise OSError("port yanked")
            yield  # pragma: no cover

    buf = ScanBuffer()
    ScanReader(Boom(), buf)._loop()          # must not propagate
    assert buf.latest()["n"] == 0


def test_reader_thread_start_and_stop():
    buf = ScanBuffer()
    reader = ScanReader(FakeLidar([make_scan(5.0)]), buf).start()
    reader._thread.join(timeout=2.0)
    reader.stop()
    assert buf.latest()["freq_hz"] == 5.0


# --- Flask endpoints (only if the viz extra is installed) -------------------

def _client():
    pytest.importorskip("flask")
    buf = ScanBuffer()
    buf.update(make_scan(5.0))
    return make_app(buf, model="LD14P").test_client()


def test_index_serves_html_with_model_name():
    resp = _client().get("/")
    assert resp.status_code == 200
    body = resp.get_data(as_text=True)
    assert "<canvas" in body
    assert "LD14P" in body
    assert "{{MODEL}}" not in body           # placeholder was substituted


def test_scan_json_returns_latest():
    resp = _client().get("/scan.json")
    assert resp.status_code == 200
    data = resp.get_json()
    assert data["freq_hz"] == 5.0
    assert data["n"] == 2
    assert data["points"][0]["dist"] == 1000
