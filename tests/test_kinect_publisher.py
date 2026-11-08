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
    reply = publisher.post("http://127.0.0.1:8090/api/kinect/depth", samples, 4, 4, 12.5,
                           capture_source="freenect", capture_port="usb:1-2.3")
    assert reply == {"ok": True, "points": 3}
    body = captured["body"]
    assert body["width"] == 4 and body["height"] == 4 and body["units"] == "mm"
    assert body["stamp"] == 12.5
    assert body["capture_source"] == "freenect"
    assert body["capture_port"] == "usb:1-2.3"
    raw = base64.b64decode(body["depth"])
    assert len(raw) == 4 * 4 * 2
    assert captured["headers"]["X-robot-control"] == "kinect"


def test_rear_unit_is_passed_in_url_and_body(monkeypatch):
    captured = {}

    class Reply:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return b'{"ok":true,"unit":"rear"}'

    def fake_urlopen(req, timeout):
        captured["url"] = req.full_url
        captured["body"] = json.loads(req.data.decode("utf-8"))
        return Reply()

    monkeypatch.setattr(publisher.urllib.request, "urlopen", fake_urlopen)
    samples = array.array("H", [2000] * 16)
    reply = publisher.post("http://127.0.0.1:8090/api/kinect/depth",
                           samples, 4, 4, 12.5, unit="rear")
    assert reply["unit"] == "rear"
    assert captured["url"].endswith("?unit=rear")
    assert captured["body"]["unit"] == "rear"
    assert publisher.main(["--source", "demo", "--once", "--unit", "rear"]) == 0


def test_auto_without_drivers_is_honest(monkeypatch, capsys):
    """Без драйвера auto не подсовывает демо-кадр вместо настоящей глубины."""
    monkeypatch.setattr(publisher, "detect_backends", lambda: [])
    source, hint = publisher.detect_source()
    assert source == "" and "не найден ни один драйвер" in hint
    assert publisher.main(["--once"]) == 2
    assert "requirements-kinect.txt" in capsys.readouterr().err
    assert publisher.detect_source(allow_demo=True)[0] == "demo"
    monkeypatch.setattr(publisher, "detect_backends", lambda: ["freenect", "ros2"])
    assert publisher.detect_source() == ("freenect", "")


def test_list_backends_reports_installed_drivers(monkeypatch, capsys):
    monkeypatch.setattr(publisher, "detect_backends", lambda: ["ros2"])
    assert publisher.main(["--list-backends"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["installed"] == ["ros2"]
    assert "freenect" in report["missing"] and "openni" in report["missing"]


def test_kinect_wrapper_sources_ros_before_backend_detection():
    script = (ROOT / "scripts" / "kinect_depth.sh").read_text(encoding="utf-8")
    source = script.index('source "$RC_ROS_PREFIX/setup.bash"')
    listing = script.index('exec "$PY" scripts/kinect_depth_publisher.py --list-backends')
    check = script.index('INSTALLED="$(')
    assert source < listing < check


def test_kinect_status_reports_drivers():
    from robot_control import kinect
    drivers = kinect.available_backends()
    assert set(drivers["backends"]) == {"freenect", "openni", "ros2"}
    assert drivers["installed"] == [name for name, yes in drivers["backends"].items() if yes]
    service = kinect.KinectDepthService()
    assert service.status()["drivers"]["ok"] is True


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
    assert calls[0]["capture_source"] == "demo"


def test_selected_usb_topology_maps_to_freenect_camera_index():
    import types

    def usb(vendor, product, bus, ports):
        return types.SimpleNamespace(idVendor=vendor, idProduct=product,
                                     bus=bus, port_numbers=ports)

    devices = [
        usb(0x045E, 0x02AD, 1, (1,)),       # NUI Audio is not a freenect camera
        usb(0x045E, 0x02AE, 1, (2,)),       # first NUI Camera
        usb(0x045E, 0x02BF, 1, (3,)),       # selected second camera
    ]
    assert publisher.freenect_index_for_port("usb:1-3", devices) == 1
    assert publisher.freenect_index_for_port("usb:1-2", devices) == 0
    try:
        publisher.freenect_index_for_port("usb:1-9", devices)
    except RuntimeError as exc:
        assert "не найден" in str(exc)
    else:
        raise AssertionError("неизвестный выбранный порт нельзя молча заменить индексом 0")


def test_usb_device_node_is_resolved_to_sysfs_topology(tmp_path):
    import pytest

    camera = tmp_path / "1-4.2"
    camera.mkdir()
    (camera / "busnum").write_text("1", encoding="ascii")
    (camera / "devnum").write_text("7", encoding="ascii")
    assert publisher.usb_topology_for_port("/dev/bus/usb/001/007", tmp_path) == "1-4.2"


def test_selected_port_requires_freenect_in_auto_mode(monkeypatch, capsys):
    monkeypatch.setattr(publisher, "detect_backends", lambda: ["openni"])
    source, message = publisher.detect_source(require_usb_binding=True)
    assert source == "" and "freenect" in message
    assert publisher.main(["--source", "auto", "--usb-port", "usb:1-2", "--once"]) == 2
    assert "привязки RC_KINECT_PORT" in capsys.readouterr().err


def test_freenect_raw_disparity_is_converted_to_mm_and_selected_index_passed(monkeypatch):
    import math
    import sys
    import types

    class FakeDepth:
        shape = (1, 4)
        @property
        def flat(self):
            return [500, 800, 1000, 2047]

    calls = []
    monkeypatch.setitem(sys.modules, 'freenect', types.SimpleNamespace(
        sync_get_depth=lambda index=0: (calls.append(index) or FakeDepth(), 123)))
    samples, width, height = publisher.freenect_frame(index=2)
    assert calls == [2]
    assert (width, height) == (4, 1)
    for raw, actual in zip((500, 800, 1000), samples):
        assert actual == int(1000 * 0.1236 * math.tan(raw / 2842.5 + 1.1863))
    assert samples[3] == 0
    assert publisher.freenect_raw_to_mm(2047) == 0
    assert publisher.freenect_raw_to_mm(1093) == 0  # past tangent asymptote
