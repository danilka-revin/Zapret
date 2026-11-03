"""API автообследования, Kinect и геометрии на борту и через прокси пульта."""
from __future__ import annotations

import array
import base64
import io
import json
from urllib.parse import urlparse

from conftest import login
from robot_control import map_peer


class Reply(io.BytesIO):
    def __init__(self, payload, code=200):
        super().__init__(json.dumps(payload).encode("utf-8"))
        self.code = code


def depth_payload(width=40, height=30):
    samples = array.array("H", [0]) * (width * height)
    for y in range(height // 3, 2 * height // 3):
        for x in range(width // 4, 3 * width // 4):
            samples[y * width + x] = 2000
    return {"width": width, "height": height,
            "depth": base64.b64encode(samples.tobytes()).decode("ascii")}


def test_kiosk_status_layout_and_kinect(kiosk_client):
    status = kiosk_client.get("/api/exploration/status")
    assert status.status_code == 200
    assert status.json["ok"] is True and status.json["enabled"] is False
    layout = kiosk_client.get("/api/sensors/layout")
    assert layout.status_code == 200
    assert layout.json["cameraCount"] == 4
    assert layout.json["kinect"]["model"] == "Kinect 360"
    kinect = kiosk_client.get("/api/kinect/status")
    assert kinect.status_code == 200 and kinect.json["connected"] is False
    assert kiosk_client.application.extensions["rc_exploration"] is not None
    assert kiosk_client.application.extensions["rc_kinect"] is not None


def test_kiosk_kinect_feed_requires_marker_and_feeds_map(kiosk_client):
    assert kiosk_client.post("/api/kinect/depth", json=depth_payload()).status_code == 400
    reply = kiosk_client.post("/api/kinect/depth", json=depth_payload(),
                              headers={"X-Robot-Control": "kinect"})
    assert reply.status_code == 200 and reply.json["ok"] is True
    assert reply.json["points"] > 0
    assert kiosk_client.get("/api/kinect/status").json["connected"] is True
    # Фоновая нить карты выключена в testing-режиме; проверяем саму связь
    # сервис → карта тем же вызовом, который делает _map_worker.
    service = kiosk_client.application.extensions["rc_kinect"]
    frame = service.consume()
    assert frame is None or frame[0]
    map_service = kiosk_client.application.extensions["rc_map"]
    assert map_service.ingest_kinect([{"a": 0.0, "r": 1.2}]) > 0
    assert map_service.snapshot(with_png=False)["kinectScans"] == 1


def test_kiosk_exploration_commands_require_marker(kiosk_client):
    assert kiosk_client.post("/api/exploration/start", json={}).status_code == 400
    reply = kiosk_client.post("/api/exploration/start", json={"blindZoneChecked": True},
                              headers={"X-Robot-Control": "exploration"})
    assert reply.status_code == 409
    assert reply.json["code"] == "disabled", "по умолчанию режим выключен"
    stop = kiosk_client.post("/api/exploration/stop", json={},
                             headers={"X-Robot-Control": "exploration"})
    assert stop.status_code == 200 and stop.json["ok"] is True
    landmarks = kiosk_client.get("/api/exploration/landmarks")
    assert landmarks.status_code == 200 and isinstance(landmarks.json["landmarks"], list)
    cleared = kiosk_client.post("/api/exploration/landmarks/clear", json={},
                                headers={"X-Robot-Control": "exploration"})
    assert cleared.status_code == 200 and cleared.json["ok"] is True
    assert kiosk_client.get("/api/exploration/landmark/../../etc/passwd").status_code in (400, 404)
    # Валидный кадр отдаётся с борта как JPEG.
    directory = kiosk_client.application.extensions["rc_exploration"].landmark_directory
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "step003-cam2.jpg").write_bytes(b"\xff\xd8test\xff\xd9")
    shot = kiosk_client.get("/api/exploration/landmark/step003-cam2.jpg")
    assert shot.status_code == 200 and shot.data.startswith(b"\xff\xd8")


def test_web_landmark_proxy_serves_jpeg(client, operator, monkeypatch):
    login(client)

    class ImageReply(io.BytesIO):
        status = 200

        def __init__(self):
            super().__init__(b"\xff\xd8proxy\xff\xd9")

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    monkeypatch.setattr(map_peer, "urlopen", lambda req, timeout: ImageReply())
    reply = client.get("/api/exploration/landmark/step003-cam2.jpg")
    assert reply.status_code == 200
    assert reply.mimetype == "image/jpeg" and reply.data == b"\xff\xd8proxy\xff\xd9"
    assert client.get("/api/exploration/landmark/not-a-frame.jpg").status_code == 400


def test_map_and_route_pages_serve_exploration_ui(kiosk_client, client, operator):
    login(client)
    for web in (kiosk_client, client):
        for path in ("/map", "/route"):
            page = web.get(path)
            assert page.status_code == 200, path
            assert "explore.js" in page.text
            assert 'id="explore-start"' in page.text
            assert 'id="explore-blind"' in page.text
        assert web.get("/explore.js").status_code == 200


def test_web_proxy_forwards_exploration_with_csrf(client, operator, kiosk_client, monkeypatch):
    login(client)

    def forward(req, timeout):
        path = urlparse(req.full_url).path
        headers = {}
        marker = req.headers.get("X-robot-control")
        if marker:
            headers["X-Robot-Control"] = marker
        response = kiosk_client.open(path, method=req.method, data=req.data,
                                     content_type="application/json", headers=headers)
        return Reply(response.json, response.status_code)

    monkeypatch.setattr(map_peer, "urlopen", forward)
    status = client.get("/api/exploration/status")
    assert status.status_code == 200 and status.json["ok"] is True
    token = status.json["csrfToken"]
    assert token
    assert client.get("/api/sensors/layout").json["cameraCount"] == 4
    assert client.get("/api/kinect/status").json["ok"] is True

    headers = {"X-Robot-Control": "exploration"}
    assert client.post("/api/exploration/start", json={"blindZoneChecked": True},
                       headers=headers).status_code == 400  # без CSRF
    assert client.post("/api/exploration/start", json={"blindZoneChecked": True},
                       headers={"X-CSRF-Token": token}).status_code == 400  # без маркера
    reply = client.post("/api/exploration/start", json={"blindZoneChecked": True},
                        headers={**headers, "X-CSRF-Token": token})
    assert reply.status_code == 409 and reply.json["code"] == "disabled"
    stop = client.post("/api/exploration/stop", json={},
                       headers={**headers, "X-CSRF-Token": token})
    assert stop.status_code == 200 and stop.json["ok"] is True
