"""HTTP-API сводки работы: /api/stats, выгрузки CSV/JSON и сброс с CSRF-токеном."""

import json

from conftest import login


def test_stats_requires_login(client, operator):
    resp = client.get("/api/stats")
    assert resp.status_code in (302, 401, 403)
    assert client.get("/api/stats/report.csv").status_code in (302, 401, 403)
    assert client.get("/api/stats/report.json").status_code in (302, 401, 403)


def test_stats_summary_shape_and_csrf_token(client, operator):
    login(client)
    body = client.get("/api/stats").get_json()
    assert body["ok"] is True
    assert body["csrfToken"]
    data = body["data"]
    assert {"since", "totals", "distanceKm", "whPerKm", "avgSpeedKmh", "live",
            "recentTrips", "unavailable"} <= set(data)
    assert {"trips", "tripsAborted", "energyUsedWh", "energyChargedWh", "movingSec",
            "maxSpeedMps", "chargeCycles", "faults", "lockOpen", "lockDenied",
            "pinChanged"} <= set(data["totals"])
    # без робота сводка пустая: ни одного выдуманного рейса или пробега
    assert data["totals"]["trips"] == 0
    assert data["totals"]["distanceM"] == 0.0
    assert data["live"]["linkOk"] is False
    assert data["whPerKm"] is None
    assert data["recentTrips"] == []


def test_stats_reports_download(client, operator):
    login(client)
    csv_resp = client.get("/api/stats/report.csv")
    assert csv_resp.status_code == 200
    assert csv_resp.mimetype == "text/csv"
    assert "rus_slam_stats.csv" in csv_resp.headers["Content-Disposition"]
    assert "Пробег, м" in csv_resp.get_data(as_text=True)

    json_resp = client.get("/api/stats/report.json")
    assert json_resp.status_code == 200
    assert json_resp.mimetype == "application/json"
    doc = json.loads(json_resp.get_data(as_text=True))
    assert doc["summary"]["totals"]["trips"] == 0
    assert "unavailable" in doc


def test_reset_requires_csrf_token(client, operator):
    login(client)
    resp = client.post("/api/stats/reset", json={})
    assert resp.status_code == 400
    assert resp.get_json()["ok"] is False
    bad = client.post("/api/stats/reset", headers={"X-CSRF-Token": "не-токен"}, json={})
    assert bad.status_code == 400


def test_reset_with_csrf_token_clears_summary(client, app, operator):
    login(client)
    token = client.get("/api/stats").get_json()["csrfToken"]
    store = app.extensions["rc_stats"]
    store.data["totals"]["trips"] = 3
    store.data["totals"]["distanceM"] = 1234.0
    resp = client.post("/api/stats/reset", headers={"X-CSRF-Token": token}, json={})
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["ok"] is True
    assert body["data"]["totals"]["trips"] == 0
    assert body["data"]["totals"]["distanceM"] == 0.0
    assert client.get("/api/stats").get_json()["data"]["totals"]["trips"] == 0


def test_stats_sampler_is_not_started_under_tests(app):
    assert "rc_stats_sampler" not in app.extensions


def test_main_page_has_summary_tab(client, operator):
    login(client)
    html = client.get("/").get_data(as_text=True)
    assert 'id="nav-summary"' in html
    assert 'id="summary"' in html
    assert "summary.js" in html and "summary.css" in html
    js = client.get("/summary.js").get_data(as_text=True)
    assert "api/stats" in js and "api/stats/reset" in js
    assert "X-CSRF-Token" in js
