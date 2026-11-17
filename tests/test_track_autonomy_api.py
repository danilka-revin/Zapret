"""HTTP вкладки «Автономность»: страница, маркеры, CSRF и прокси пульта."""
from __future__ import annotations

import io
import json
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
    assert body["ok"] is True and body["enabled"] is False
    assert body["active"] is False and body["canStart"] is False
    assert body["requireBlindCheck"] is True
    assert "RC_TRACK_ENABLE" in " ".join(body["blockers"])
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
    bad = kiosk_client.post("/api/autonomy/simulate", json={"scenario": "sideways"}, headers=marker)
    assert bad.status_code == 400


def test_kiosk_autonomy_start_requires_blind_check_and_preempts_rivals(kiosk_client, monkeypatch):
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
    assert started.get_json()["ok"] is True and preempted == [True]
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
