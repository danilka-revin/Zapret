"""Режим «Скан»: объезд помещения с картой и включаемым вручную управлением.

Что закреплено:

* управление включается только явно и снимается кнопкой, молчанием страницы
  (deadman) или сменой токена: «выключено» действительно значит «не поедет»;
* выключение и включение всегда отправляют STOP, причём до смены состояния —
  даже если страница закрылась посреди движения;
* тяга ограничена отдельным потолком (``RC_SCAN_MAX_PERCENT``, по умолчанию
  20 %), ноль разрешён всегда;
* карту можно сохранить и потом загрузить: PGM читается так же, как пишется;
* страница ``/scan`` и её скрипт отдаются, а движение с пульта требует CSRF и
  заголовка ``X-Robot-Control: scan``.
"""
from __future__ import annotations

from pathlib import Path

import pytest

from robot_control import scan_mode as scan
from robot_control.scan_mode import ScanModeError, ScanModeService
from robot_control.slam_map import SlamMapService, UNKNOWN, parse_pgm

ROOT = Path(__file__).resolve().parent.parent


class Clock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


class FakeChassis:
    """Шасси, которое записывает команды и умеет отказывать."""

    def __init__(self, *, brake_ok=True):
        self.calls = []
        self.brake_ok = brake_ok

    def drive(self, source, body):
        self.calls.append(("drive", dict(body)))
        if body.get("percent") == 0 and not self.brake_ok:
            return {"ok": False, "reason": "serial_unavailable"}, 503
        return {"ok": True, "percent": body.get("percent"), "maxPercent": 100}, 200

    def steer(self, source, body):
        self.calls.append(("steer", dict(body)))
        return {"ok": True, "module": body.get("module"), "angle": body.get("angle")}, 200

    def service(self, *, clock=None, estop=None, max_percent=20, deadman_s=5.0):
        return ScanModeService(
            lambda: object(), self.drive, self.steer,
            estop or (lambda: None),
            clock=clock if clock is not None else Clock(),
            max_percent=max_percent,
            deadman_s=deadman_s,
        )


# ------------------------------------------------------------- состояние ---
def test_starts_disabled_and_refuses_motion():
    """По умолчанию управление выключено — тяга из режима не проходит."""
    chassis = FakeChassis()
    service = chassis.service()
    assert service.status()["enabled"] is False
    with pytest.raises(ScanModeError) as excinfo:
        service.drive({"percent": 8})
    assert excinfo.value.reason == "scan_disabled" and excinfo.value.status == 409
    # В шасси не ушло ни одной команды тяги: отказ случился до железа.
    assert ("drive", {"percent": 8}) not in chassis.calls


def test_enable_brakes_first_then_allows_drive():
    chassis = FakeChassis()
    service = chassis.service()
    enabled = service.enable()
    assert enabled["enabled"] is True and enabled["token"]
    # Первое, что уходит на шасси при включении, — нулевая тяга.
    assert chassis.calls[0] == ("drive", {"percent": 0})
    result, status = service.drive({"percent": 8}, enabled["token"])
    assert status == 200 and result["ok"] is True
    assert chassis.calls[-1] == ("drive", {"percent": 8})


def test_enable_is_refused_when_the_chassis_will_not_confirm_the_brake():
    chassis = FakeChassis(brake_ok=False)
    service = chassis.service()
    with pytest.raises(ScanModeError) as excinfo:
        service.enable()
    assert excinfo.value.reason == "brake_failed"
    assert service.status()["enabled"] is False


def test_disable_brakes_even_when_motion_was_active():
    chassis = FakeChassis()
    service = chassis.service()
    token = service.enable()["token"]
    service.drive({"percent": 10}, token)
    result = service.disable()
    assert result["enabled"] is False and result["braked"] is True
    assert chassis.calls[-1] == ("drive", {"percent": 0})
    with pytest.raises(ScanModeError):
        service.drive({"percent": 10})


def test_stop_is_allowed_while_disabled():
    """Ноль всегда можно: «стоп» не должен требовать включённого управления."""
    chassis = FakeChassis()
    service = chassis.service()
    result, status = service.drive({"percent": 0})
    assert status == 200 and result["ok"] is True
    assert chassis.calls[-1] == ("drive", {"percent": 0})


def test_max_percent_is_enforced_and_zero_allowed():
    chassis = FakeChassis()
    service = chassis.service(max_percent=20)
    token = service.enable()["token"]
    with pytest.raises(ScanModeError) as excinfo:
        service.drive({"percent": 21}, token)
    assert excinfo.value.reason == "drive_limit" and excinfo.value.status == 400
    assert service.drive({"percent": 20}, token)[0]["ok"] is True
    assert service.drive({"percent": -20}, token)[0]["ok"] is True


def test_bad_percent_is_rejected_not_treated_as_zero():
    chassis = FakeChassis()
    service = chassis.service()
    token = service.enable()["token"]
    for bad in ({"percent": "много"}, {"percent": True}, {}, {"percent": 1000}):
        with pytest.raises(ScanModeError) as excinfo:
            service.drive(bad, token)
        assert excinfo.value.reason in {"invalid_drive", "payload"}
    # Неизвестный формат не должен превратиться в команду «стоп» на шасси.
    assert all(call[1].get("percent") != 0 for call in chassis.calls[1:])


def test_estop_blocks_enable():
    chassis = FakeChassis()
    service = chassis.service(estop=lambda: True)
    with pytest.raises(ScanModeError) as excinfo:
        service.enable()
    assert excinfo.value.reason == "estop"
    assert service.status()["estop"] is True


def test_silent_page_loses_control():
    """Страница молчала дольше deadman — управление снимается и тяга гаснет."""
    clock = Clock(100.0)
    chassis = FakeChassis()
    service = chassis.service(clock=clock, deadman_s=5.0)
    token = service.enable()["token"]
    clock.now += 4.0
    service.heartbeat(token)
    clock.now += 6.0
    status = service.status()
    assert status["enabled"] is False and status["reason"] == "deadman"
    assert chassis.calls[-1] == ("drive", {"percent": 0})


def test_motion_after_deadman_is_refused():
    clock = Clock(0.0)
    chassis = FakeChassis()
    service = chassis.service(clock=clock, deadman_s=2.0)
    token = service.enable()["token"]
    clock.now += 5.0
    with pytest.raises(ScanModeError) as excinfo:
        service.drive({"percent": 5}, token)
    assert excinfo.value.reason in {"scan_disabled", "scan_timeout"}


def test_another_tab_cannot_drive_with_a_stale_token():
    chassis = FakeChassis()
    service = chassis.service()
    first = service.enable()["token"]
    second = service.enable()["token"]
    assert first != second
    with pytest.raises(ScanModeError) as excinfo:
        service.drive({"percent": 5}, first)
    assert excinfo.value.reason == "scan_token"
    assert service.drive({"percent": 5}, second)[0]["ok"] is True


def test_steer_goes_through_the_same_gate():
    chassis = FakeChassis()
    service = chassis.service()
    with pytest.raises(ScanModeError):
        service.steer({"module": "FL", "angle": 20})
    token = service.enable()["token"]
    result, status = service.steer({"module": "FL", "angle": 20}, token)
    assert status == 200 and result["ok"] is True
    assert chassis.calls[-1] == ("steer", {"module": "FL", "angle": 20})


def test_default_limit_is_20_percent(monkeypatch):
    monkeypatch.delenv("RC_SCAN_MAX_PERCENT", raising=False)
    assert scan.env_max_percent() == 20
    monkeypatch.setenv("RC_SCAN_MAX_PERCENT", "35")
    assert scan.env_max_percent() == 35
    monkeypatch.setenv("RC_SCAN_MAX_PERCENT", "1000")
    assert scan.env_max_percent() == 100


def test_missing_source_is_a_refusal_not_a_crash():
    service = ScanModeService(lambda: None, lambda s, b: ({}, 200), lambda s, b: ({}, 200))
    result, status = service.drive({"percent": 0})
    assert status == 503 and result["reason"] == "serial_unavailable"


# ----------------------------------------------------------------- карта ---
def test_map_saves_and_loads_back(tmp_path):
    """«Посканировал — сохранил — загрузил»: сетка не теряется по дороге."""
    service = SlamMapService(directory=tmp_path)
    service._grid.data[0] = 100
    service._grid.data[7] = 0
    assert service.save()["ok"] is True

    restored = SlamMapService(directory=tmp_path)
    result = restored.load()
    assert result["ok"] is True
    assert restored._grid.data[0] == 100
    assert restored._grid.data[7] == 0
    # Поза после загрузки обнулена: робот должен найти себя заново.
    assert restored._pose.pose == (0.0, 0.0, 0.0)


def test_loading_a_missing_map_says_so(tmp_path):
    service = SlamMapService(directory=tmp_path)
    result = service.load()
    assert result["ok"] is False and "нет файла карты" in result["error"]


def test_truncated_pgm_is_rejected(tmp_path):
    service = SlamMapService(directory=tmp_path)
    service.save()
    pgm = Path(service.snapshot()["paths"]["pgm"])
    data = pgm.read_bytes()
    pgm.write_bytes(data[:len(data) // 2])
    result = SlamMapService(directory=tmp_path).load()
    assert result["ok"] is False and "обрезан" in result["error"]


def test_parse_pgm_handles_comments_and_unknown_cells():
    body = b"P5\n# comment\n2 2\n255\n" + bytes([0, 254, 205, 100])
    meta, pixels = parse_pgm(body)
    assert meta == {"width": 2, "height": 2, "maxval": 255}
    assert list(pixels) == [0, 254, 205, 100]
    with pytest.raises(ValueError):
        parse_pgm(b"P2\n2 2\n255\n")


# ------------------------------------------------------------------- API ---
def test_scan_page_and_assets_are_served(kiosk_client):
    page = kiosk_client.get("/scan")
    assert page.status_code == 200
    html = page.text
    for marker in ('id="scan-enable"', 'id="scan-disable"', 'id="map-canvas"',
                   'id="cam1-canvas"', 'id="scan-stop"', 'scan_mode.js'):
        assert marker in html, marker
    # Камеры не должны ходить на localhost: только относительные запросы.
    assert "127.0.0.1" not in html and "localhost" not in html
    assert kiosk_client.get("/scan_mode.js").status_code == 200
    assert kiosk_client.get("/scan.css").status_code == 200


def test_scan_api_status_is_read_only(kiosk_client):
    body = kiosk_client.get("/api/scan_mode")
    assert body.status_code == 200
    payload = body.json
    assert payload["ok"] is True and payload["enabled"] is False
    assert payload["maxPercent"] >= 1
    assert "map" in payload


def test_scan_api_refuses_drive_while_disabled(kiosk_client):
    body = kiosk_client.post("/api/scan_mode/drive", json={"percent": 8})
    assert body.status_code == 409
    assert body.json["reason"] == "scan_disabled"
    # Стоп разрешён всегда, но без шасси честно отвечает отказом.
    stop = kiosk_client.post("/api/scan_mode/drive", json={"percent": 0})
    assert stop.status_code in (200, 409, 503)


def test_scan_api_enable_reports_the_missing_chassis(kiosk_client):
    body = kiosk_client.post("/api/scan_mode/enable")
    assert body.status_code == 409
    assert body.json["reason"] == "brake_failed"


def test_scan_api_map_load_reports_a_missing_file(kiosk_client, tmp_path):
    body = kiosk_client.post("/api/scan_mode/map/load")
    assert body.status_code == 400
    assert "нет файла карты" in body.json["error"]


def test_scan_screen_script_runs_in_node():
    """Кнопки экрана: блокировка без управления, тяга, стоп, Esc, камеры."""
    import shutil
    import subprocess
    node = shutil.which("node")
    if node is None:
        pytest.skip("нет node: экранные скрипты проверяются только с ним")
    result = subprocess.run([node, str(ROOT / "tests" / "js" / "scan_screen.js")],
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "scan screen: OK" in result.stdout


def test_scan_assets_are_registered_for_the_remote_console():
    """Файлы режима отдаются и бортовым сервером, и веб-пультом."""
    kiosk = (ROOT / "robot_control" / "kiosk_app.py").read_text(encoding="utf-8")
    web = (ROOT / "robot_control" / "slam.py").read_text(encoding="utf-8")
    assert '"scan_mode.js"' in kiosk and '"scan.css"' in kiosk
    assert '"scan_mode.js": "scan_mode_js"' in web
    assert '"/scan"' in kiosk and '"/scan"' in web


def test_remote_proxy_requires_csrf_and_scan_header(client, operator):
    """Пульт не должен становиться обходным путём: нужен CSRF и маркер scan."""
    from conftest import login
    login(client)
    # Без CSRF движение не пересылается вовсе.
    body = client.post("/api/scan_mode/drive", json={"percent": 5},
                       headers={"X-Robot-Control": "scan"})
    assert body.status_code == 400 and body.json["reason"] == "csrf_required"
    # С CSRF, но без маркера — тот же отказ.
    token = client.get("/api/control/session").json["csrfToken"]
    body = client.post("/api/scan_mode/drive", json={"percent": 5},
                       headers={"X-CSRF-Token": token})
    assert body.status_code == 400 and body.json["reason"] == "request_header_required"
