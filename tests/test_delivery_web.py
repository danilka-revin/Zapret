"""Экран /delivery и API: обе стороны, маркеры, CSRF и отсутствие тяги."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from conftest import login

ROOT = Path(__file__).resolve().parents[1]


class FakeReply:
    def __init__(self, payload, code=200):
        self.payload = payload
        self.code = code
        self.status = code

    def read(self, limit):
        raw = json.dumps(self.payload).encode("utf-8")
        assert len(raw) <= limit
        return raw

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def test_delivery_page_and_assets_on_both_sides(kiosk_client, client, operator):
    login(client)
    for web in (kiosk_client, client):
        page = web.get("/delivery")
        assert page.status_code == 200
        html = page.text
        for marker in ('id="delivery-canvas"', 'id="delivery-goal"', 'id="delivery-simulate"',
                       "СУХОЙ ПРОГОН", "ТЯГА ОТКЛЮЧЕНА"):
            assert marker in html
        assert 'id="delivery-start-drive"' not in html
        for name in ("delivery.js", "delivery.css", "delivery_reference.png"):
            assert web.get("/" + name).status_code == 200, name


def test_kiosk_delivery_api_flow(kiosk_client):
    assert kiosk_client.get("/api/delivery/map").status_code == 200
    status = kiosk_client.get("/api/delivery/status").get_json()
    assert status["drivePermitted"] is False
    assert status["tractionConnected"] is False
    body = {"start": {"x": 22, "y": 140, "yawDeg": 80}, "goalNodeId": "ntc_delivery",
            "scenario": "clear", "draftAcknowledged": True}
    assert kiosk_client.post("/api/delivery/setup", json=body).status_code == 400
    saved = kiosk_client.post("/api/delivery/setup", json=body, headers={"X-Robot-Control": "delivery"})
    assert saved.status_code == 200
    assert saved.get_json()["plan"]["hardwareEnabled"] is False
    simulated = kiosk_client.post("/api/delivery/simulate", json={"scenario": "pothole_blocked"},
                                  headers={"X-Robot-Control": "delivery"})
    assert simulated.status_code == 200
    payload = simulated.get_json()["simulation"]
    assert payload["ok"] is False
    assert payload["finalState"] == "blocked_by_road_hazard"
    assert payload["tractionConnected"] is False
    reset = kiosk_client.post("/api/delivery/reset", json={}, headers={"X-Robot-Control": "delivery"})
    assert reset.status_code == 200


def test_remote_delivery_requires_login(client):
    assert client.get("/delivery").status_code == 302
    assert client.get("/api/delivery/status").status_code == 302


def test_remote_delivery_proxy_guards_and_csrf(client, operator, monkeypatch):
    login(client)
    assert client.post("/api/delivery/setup", json={}).status_code == 400
    calls = []

    def fake_urlopen(request, timeout=0):
        calls.append(request)
        if request.full_url.endswith("/api/delivery/status"):
            return FakeReply({"ok": True, "checks": {}, "drivePermitted": False})
        return FakeReply({"ok": True, "setup": {}, "plan": {"hardwareEnabled": False}, "status": {"ok": True}})

    monkeypatch.setattr("robot_control.map_peer.urlopen", fake_urlopen)
    status = client.get("/api/delivery/status")
    assert status.status_code == 200
    token = status.get_json()["csrfToken"]
    response = client.post("/api/delivery/setup", json={"goalNodeId": "ntc_delivery"},
                           headers={"X-Robot-Control": "delivery", "X-CSRF-Token": token})
    assert response.status_code == 200
    assert calls[-1].full_url.endswith("/api/delivery/setup")
    assert calls[-1].get_header("X-robot-control") == "delivery"
    bad = client.post("/api/delivery/simulate", json={}, headers={"X-CSRF-Token": token})
    assert bad.status_code == 400


def test_main_and_kiosk_link_delivery_screen():
    main = (ROOT / "slam_gui" / "main.html").read_text(encoding="utf-8")
    robot = (ROOT / "slam_gui" / "robot.html").read_text(encoding="utf-8")
    assert 'href="/delivery" id="nav-delivery"' in main
    assert 'id="k-delivery-open"' in robot and 'href="/delivery"' in robot
    assert 'id="swipe-delivery-open"' in robot
