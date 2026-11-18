"""HTTP вкладки «Автономность»: страница, маркеры, CSRF и прокси пульта."""
from __future__ import annotations

import io
import json
import threading
from dataclasses import replace

from conftest import login
from robot_control import map_peer
from robot_control.track_autonomy import AUTONOMY_HEADER


class FakeReply(io.BytesIO):
    def __init__(self, payload, code=200):
        super().__init__(json.dumps(payload).encode("utf-8"))
        self.code = code

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def _upstream(monkeypatch, payloads=None):
    calls = []
    payloads = payloads or {}

    def fake_urlopen(request, timeout=0):
        calls.append(request)
        path = request.full_url.rsplit("/", 1)[-1]
        body = payloads.get(path, {"ok": True, "status": {"ok": True}})
        return FakeReply(body)

    monkeypatch.setattr(map_peer, "urlopen", fake_urlopen)
    return calls


def test_kiosk_autonomy_page_assets_and_status(kiosk_client):
    page = kiosk_client.get("/autonomy")
    assert page.status_code == 200
    html = page.text
    for marker in ('id="aut-canvas"', 'id="aut-start"', 'id="aut-stop"',
                   'id="aut-blind"', 'id="aut-simulate"', "Сухой прогон",
                   "ждём 3 с", "только на зелёный"):
        assert marker in html
    for asset in ("autonomy.js", "autonomy.css", "map.css", "slam_auth.js"):
        assert kiosk_client.get("/" + asset).status_code == 200, asset
    status = kiosk_client.get("/api/autonomy/status")
    assert status.status_code == 200
    body = status.get_json()
    assert body["ok"] is True and body["enabled"] is True
    assert body["active"] is False and body["canStart"] is False
    assert body["requireBlindCheck"] is True
    assert "RC_TRACK_ENABLE" not in " ".join(body["blockers"])
    assert body["navigationMode"] == "forward" and body["corridor"]["required"] is False
    assert body["drive"]["commands"] == 0 and body["drive"]["percent"] == 0
    assert kiosk_client.application.extensions["rc_track"] is not None


def test_kiosk_autonomy_commands_require_marker(kiosk_client):
    marker = {"X-Robot-Control": AUTONOMY_HEADER}
    assert kiosk_client.post("/api/autonomy/start", json={"blindZoneChecked": True}).status_code == 400
    blocked = kiosk_client.post("/api/autonomy/start", json={"blindZoneChecked": True}, headers=marker)
    assert blocked.status_code == 409 and blocked.get_json()["code"] == "not_ready"
    assert kiosk_client.post("/api/autonomy/stop", json={}).status_code == 400
    stopped = kiosk_client.post("/api/autonomy/stop", json={}, headers=marker)
    assert stopped.status_code == 200 and stopped.get_json()["ok"] is True
    assert kiosk_client.post("/api/autonomy/simulate", json={"scenario": "full"}).status_code == 400
    simulated = kiosk_client.post("/api/autonomy/simulate", json={"scenario": "full"}, headers=marker)
    assert simulated.status_code == 200
    report = simulated.get_json()
    assert report["ok"] is True and report["phase"] == "done"
    assert report["counters"]["stop"] >= 1 and report["counters"]["bump"] >= 1
    assert report["counters"]["traffic"] >= 1 and report["counters"]["powerBoosts"] >= 1
    closed = kiosk_client.post("/api/autonomy/simulate", json={"scenario": "blocked_gate"}, headers=marker)
    assert closed.status_code == 200 and closed.json["passed"] and closed.json["distanceM"] == 0
    dead_end = kiosk_client.post("/api/autonomy/simulate", json={"scenario": "dead_end"}, headers=marker)
    assert dead_end.status_code == 200 and dead_end.json["passed"] and dead_end.json["expectedStop"]
    assert 0 < dead_end.json["distanceM"] < 8 and dead_end.json["finalPercent"] == 0
    track = kiosk_client.application.extensions["rc_track"]
    assert not track.active and track.motion.command_count == 0  # simulations never arm the real service
    bad = kiosk_client.post("/api/autonomy/simulate", json={"scenario": "sideways"}, headers=marker)
    assert bad.status_code == 400


def test_simulated_autonomy_requires_blind_check_without_braking_other_modes(kiosk_client, monkeypatch):
    track = kiosk_client.application.extensions["rc_track"]
    # Тестовый киоск работает без шасси: включаем только логику пуска, а не
    # реальные команды Serial (симуляция внутри TrackMotion).
    track.cfg = replace(track.cfg, enabled=True, simulation=True,
                        require_depth=False, require_imu=False)
    track.motion.simulation = True
    def fake_preflight(*, blind_zone_checked=False, for_start=False):
        blockers = ([] if blind_zone_checked or not for_start else
                    ["Осмотрите зону вокруг робота и поставьте галочку подтверждения"])
        return {"ok": True, "blockers": blockers, "warnings": [], "checks": {},
                "canStart": not blockers, "requireBlindCheck": True}

    monkeypatch.setattr(track, "preflight", fake_preflight)
    preempted = []
    monkeypatch.setattr(track, "_preempt", lambda: preempted.append(True))
    monkeypatch.setattr(track, "_run", lambda _cancel: track._session.update(finished=True))

    marker = {"X-Robot-Control": AUTONOMY_HEADER}
    no_blind = kiosk_client.post("/api/autonomy/start", json={}, headers=marker)
    assert no_blind.status_code == 409
    assert "галочку" in no_blind.get_json()["error"]
    # preflight() подменён, поэтому галочку проверит сам start() через for_start.
    started = kiosk_client.post("/api/autonomy/start",
                                json={"blindZoneChecked": True}, headers=marker)
    assert started.status_code == 202, started.get_json()
    assert started.get_json()["ok"] is True and not preempted
    stopped = kiosk_client.post("/api/autonomy/stop", json={}, headers=marker)
    assert stopped.status_code == 200 and stopped.get_json()["ok"] is True


def test_remote_autonomy_requires_login(client):
    assert client.get("/autonomy").status_code == 302
    assert client.get("/api/autonomy/status").status_code == 302


def test_remote_autonomy_page_assets_and_proxy(client, operator, monkeypatch):
    login(client)
    page = client.get("/autonomy")
    assert page.status_code == 200 and 'id="aut-canvas"' in page.text
    for asset in ("autonomy.js", "autonomy.css", "map.css"):
        assert client.get("/" + asset).status_code == 200, asset
    calls = _upstream(monkeypatch, {
        "status": {"ok": True, "active": False, "checks": {}, "canStart": False},
        "start": {"ok": True, "status": {"phase": "starting"}},
        "stop": {"ok": True, "status": {"phase": "cancelled"}},
        "simulate": {"ok": True, "phase": "done", "scenario": "full",
                     "counters": {"stop": 1}, "events": []},
    })
    status = client.get("/api/autonomy/status")
    assert status.status_code == 200
    token = status.get_json()["csrfToken"]
    assert token
    assert calls[-1].full_url.endswith("/api/autonomy/status")

    headers = {"X-Robot-Control": AUTONOMY_HEADER, "X-CSRF-Token": token}
    assert client.post("/api/autonomy/start", json={}).status_code == 400
    assert client.post("/api/autonomy/start", json={},
                       headers={"X-CSRF-Token": token}).status_code == 400
    assert client.post("/api/autonomy/start", json={}, headers=headers).status_code == 200
    assert calls[-1].full_url.endswith("/api/autonomy/start")
    assert calls[-1].get_header("X-robot-control") == AUTONOMY_HEADER

    assert client.post("/api/autonomy/stop", json={}, headers=headers).status_code == 200
    assert calls[-1].full_url.endswith("/api/autonomy/stop")
    simulated = client.post("/api/autonomy/simulate", json={"scenario": "full"}, headers=headers)
    assert simulated.status_code == 200 and simulated.get_json()["phase"] == "done"
    assert calls[-1].full_url.endswith("/api/autonomy/simulate")
    assert client.post("/api/autonomy/simulate", json={"scenario": "full"},
                       headers={"X-CSRF-Token": token}).status_code == 400


def test_other_autonomous_modes_cannot_take_track_drive(kiosk_client, monkeypatch):
    track = kiosk_client.application.extensions["rc_track"]
    monkeypatch.setattr(type(track), "active", property(lambda _self: True))
    response = kiosk_client.post("/api/scan_mode/enable", json={})
    assert response.status_code == 409 and response.json["reason"] == "autonomy_active"
    response = kiosk_client.post("/api/navigation/start", json={}, headers={"X-Robot-Control": "navigation"})
    assert response.status_code == 409 and response.json["reason"] == "autonomy_active"


def test_stop_retires_an_autonomy_start_during_preflight(kiosk_client, monkeypatch):
    app = kiosk_client.application
    track = app.extensions["rc_track"]
    track.motion.simulation = True  # no command can touch a port
    requested, release = threading.Event(), threading.Event()
    responses = []
    marker = {"X-Robot-Control": AUTONOMY_HEADER}

    def preflight(*, for_start=False, **_kw):
        if for_start:
            requested.set()
            assert release.wait(3.0)
        return {"blockers": [], "canStart": True}

    monkeypatch.setattr(track, "preflight", preflight)

    def start():
        with app.test_client() as client:
            responses.append(client.post("/api/autonomy/start", json={"blindZoneChecked": True}, headers=marker))

    worker = threading.Thread(target=start)
    worker.start()
    try:
        assert requested.wait(1.0)
        stopped = kiosk_client.post("/api/autonomy/stop", json={}, headers=marker)
        assert stopped.status_code == 200
    finally:
        release.set()
        worker.join(3.0)
    assert not worker.is_alive()
    assert not track.active and track.motion.command_count == 0
    assert responses[0].status_code == 409 and responses[0].json["code"] == "start_interrupted"
