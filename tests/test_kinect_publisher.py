"""Публикатор глубины: уменьшение кадра и POST без железа."""
from __future__ import annotations

import array
import base64
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "kinect_depth_publisher", ROOT / "scripts" / "kinect_depth_publisher.py")
publisher = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(publisher)


def test_demo_frame_has_wall_and_floor():
    data, width, height = publisher.demo_frame(40, 30)
    assert len(data) == width * height
    assert data[15 * 40 + 20] == 2000
    assert data[28 * 40 + 20] == 1200


def test_resample_preserves_dimensions_and_values():
    data = array.array("H", range(8 * 4))
    small = publisher.resample_u16(data, 8, 4, 4, 2)
    assert len(small) == 8
    assert small[0] == data[0]
    assert small[-1] == data[2 * 8 + 6]


def test_post_sends_base64_uint16_and_marker(monkeypatch):
    captured = {}

    class Reply:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return b'{"ok":true,"points":3}'

    def fake_urlopen(req, timeout):
        captured["url"] = req.full_url
        captured["headers"] = dict(req.headers)
        captured["body"] = json.loads(req.data.decode("utf-8"))
        return Reply()

    monkeypatch.setattr(publisher.urllib.request, "urlopen", fake_urlopen)
    samples = array.array("H", [2000] * 16)
    reply = publisher.post("http://127.0.0.1:8090/api/kinect/depth", samples, 4, 4, 12.5)
    assert reply == {"ok": True, "points": 3}
    body = captured["body"]
    assert body["width"] == 4 and body["height"] == 4 and body["units"] == "mm"
    assert body["stamp"] == 12.5
    raw = base64.b64decode(body["depth"])
    assert len(raw) == 4 * 4 * 2
    assert captured["headers"]["X-robot-control"] == "kinect"


def test_demo_once_end_to_end(monkeypatch):
    """--source demo --once проходит через grab → resample → post."""
    calls = []

    class Reply:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return b'{"ok":true,"points":10}'

    def fake_urlopen(req, timeout):
        calls.append(json.loads(req.data.decode("utf-8")))
        return Reply()

    monkeypatch.setattr(publisher.urllib.request, "urlopen", fake_urlopen)
    code = publisher.main(["--source", "demo", "--once"])
    assert code == 0
    assert len(calls) == 1
    assert calls[0]["width"] == publisher.DEFAULT_WIDTH
