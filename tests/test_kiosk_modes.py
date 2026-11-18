"""Стартовый экран бортового дисплея: PIN выбирает режим экрана.

Проверяем:
  * PIN 7232 открывает обычный киоск, PIN 5656 — гостевой «бортовой монитор»;
  * режим живёт в подписанной cookie, а не в адресе: подделка не проходит;
  * монитор не получает настройку, руление, тягу и остановку — это закрывает
    сам сервер, одного скрытия кнопок на странице мало;
  * кейс на мониторе открывается только своим PIN, а закрывается не с него.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from robot_control import kiosk_app as kiosk_module
from robot_control.kiosk_app import KIOSK_MODE_COOKIE


MODE_HEADER = {"X-Robot-Control": "kiosk-mode"}


def open_mode(client, pin):
    return client.post("/api/kiosk_mode", json={"pin": pin}, headers=MODE_HEADER)


def mode_cookie(client):
    cookie = client.get_cookie(KIOSK_MODE_COOKIE)
    return cookie.value if cookie is not None else None


# ---------------------------------- стартовый экран ------------------------- #
def test_start_screen_is_served(kiosk_client):
    for path in ("/start", "/start.html"):
        response = kiosk_client.get(path)
        assert response.status_code == 200, path
        assert "start.js" in response.get_data(as_text=True)


def test_start_screen_assets_are_available(kiosk_client):
    for name in ("start.css", "start.js", "monitor.css", "monitor.js"):
        assert kiosk_client.get(f"/{name}").status_code == 200, name


def test_monitor_page_needs_the_chosen_mode(kiosk_client):
    """Прямая ссылка на монитор в обход PIN не открывает его."""
    response = kiosk_client.get("/monitor")
    assert response.status_code == 302
    assert response.headers["Location"].startswith("/?choose=")


def test_mode_status_starts_empty(kiosk_client):
    body = kiosk_client.get("/api/kiosk_mode").get_json()
    assert body["ok"] is True
    assert body["mode"] is None
    assert body["monitorReadOnly"] is False


# ------------------------------------- PIN ---------------------------------- #
def test_main_pin_opens_the_kiosk(kiosk_client):
    response = open_mode(kiosk_client, "7232")
    assert response.status_code == 200
    body = response.get_json()
    assert body["mode"] == "main" and body["page"] == "/kiosk"
    assert mode_cookie(kiosk_client)
    assert kiosk_client.get("/monitor").status_code == 302      # монитор не открыт
    assert kiosk_client.get("/kiosk").status_code == 200


def test_monitor_pin_opens_the_guest_monitor(kiosk_client):
    response = open_mode(kiosk_client, "5656")
    assert response.status_code == 200
    assert response.get_json()["mode"] == "monitor"
    page = kiosk_client.get("/monitor")
    assert page.status_code == 200
    text = page.get_data(as_text=True)
    assert "monitor.js" in text
    assert 'id="mon-case-open"' in text
    assert kiosk_client.get("/").status_code == 302             # корень сразу на монитор


@pytest.mark.parametrize("pin, reason", [("0000", "wrong"), ("", "format"), ("12", "format")])
def test_wrong_pin_is_rejected_without_a_mode(kiosk_client, pin, reason):
    response = open_mode(kiosk_client, pin)
    assert response.status_code == 403
    assert response.get_json()["reason"] == reason
    assert mode_cookie(kiosk_client) is None


def test_mode_endpoint_requires_its_header(kiosk_client):
    """Сторонний POST без заголовка не может сменить режим экрана."""
    response = kiosk_client.post("/api/kiosk_mode", json={"pin": "7232"})
    assert response.status_code == 400
    assert response.get_json()["reason"] == "request_header_required"


def test_forged_cookie_is_ignored(kiosk_client):
    kiosk_client.set_cookie(KIOSK_MODE_COOKIE, "monitor")
    body = kiosk_client.get("/api/kiosk_mode").get_json()
    assert body["mode"] is None
    assert kiosk_client.get("/monitor").status_code == 302


def test_too_many_wrong_pins_lock_the_start_screen(kiosk_client):
    modes = kiosk_client.application.extensions["rc_kiosk_modes"]
    for _ in range(modes.max_attempts):
        open_mode(kiosk_client, "1111")
    locked = open_mode(kiosk_client, "7232")
    assert locked.status_code == 403
    assert locked.get_json()["reason"] == "blocked"
    assert mode_cookie(kiosk_client) is None


def test_chosen_mode_is_forgotten_after_a_server_restart(kiosk_client, config):
    """Новый запуск ярлыка снова показывает стартовый экран с PIN."""
    open_mode(kiosk_client, "7232")
    old_cookie = mode_cookie(kiosk_client)
    assert kiosk_client.get("/kiosk").status_code == 200

    # «Перезапуск»: новое приложение того же робота — своя соль подписи.
    fresh = kiosk_module.create_kiosk_app(config).test_client()
    fresh.set_cookie(KIOSK_MODE_COOKIE, old_cookie)
    assert fresh.get("/api/kiosk_mode").get_json()["mode"] is None
    assert fresh.get("/monitor").status_code == 302


def test_exit_returns_to_the_pin_screen(kiosk_client):
    open_mode(kiosk_client, "5656")
    assert kiosk_client.get("/monitor").status_code == 200
    exit_response = kiosk_client.post("/api/kiosk_mode/exit")
    assert exit_response.status_code == 200
    assert exit_response.get_json()["page"] == "/"
    assert exit_response.headers["Set-Cookie"].count(KIOSK_MODE_COOKIE) == 1
    assert kiosk_client.get("/monitor").status_code == 302


# ----------------------------- ограничения монитора ------------------------- #
def test_monitor_cannot_steer_drive_or_stop(kiosk_client):
    open_mode(kiosk_client, "5656")
    steering = {"X-Robot-Control": "steering"}
    drive = {"X-Robot-Control": "drive"}
    for path, headers in (
        ("/api/kiosk_steer", steering),
        ("/api/kiosk_drive", drive),
        ("/api/kiosk_keyboard", drive),
        ("/api/kiosk_neutral", drive),
        ("/api/kiosk_full_zero", steering),
        ("/api/kiosk_maneuver", steering),
        ("/api/estop/release", drive),
        ("/api/system/stop", {"X-Robot-Control": "stop"}),
        ("/api/devices/ports", {}),
        ("/api/wheels/settings", {}),
        ("/api/lidar/settings", {}),
        ("/api/scan_mode/enable", {}),
    ):
        response = kiosk_client.post(path, json={}, headers=headers)
        assert response.status_code == 403, path
        assert response.get_json()["reason"] == "monitor_mode", path


def test_monitor_keeps_the_common_information(kiosk_client):
    open_mode(kiosk_client, "5656")
    for path in ("/api/kiosk_state", "/api/battery", "/api/lidar", "/api/kiosk_logs",
                 "/api/kiosk_lock", "/api/pc_state", "/api/connections"):
        assert kiosk_client.get(path).status_code == 200, path


def test_monitor_displays_jbd_independently_of_chassis():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node is needed for the monitor screen test")
    subprocess.run([node, str(Path(__file__).parent / "js/monitor_battery.js")], check=True, timeout=15)


def test_monitor_opens_the_case_with_its_pin(kiosk_client):
    open_mode(kiosk_client, "5656")
    wrong = kiosk_client.post("/api/kiosk_lock/open", json={"pin": "0000"})
    assert wrong.status_code == 403
    opened = kiosk_client.post("/api/kiosk_lock/open", json={"pin": "7232"})
    assert opened.status_code == 200 and opened.get_json()["lock"]["open"] is True


def test_monitor_does_not_close_the_case(kiosk_client):
    """Закрытие кейса с монитора — команда обычного экрана."""
    open_mode(kiosk_client, "5656")
    response = kiosk_client.post("/api/kiosk_lock/close", json={})
    assert response.status_code == 403
    assert response.get_json()["reason"] == "monitor_mode"


def test_main_mode_keeps_all_commands(kiosk_client):
    open_mode(kiosk_client, "7232")
    closed = kiosk_client.post("/api/kiosk_lock/close", json={})
    assert closed.status_code == 200
    assert kiosk_client.get("/api/wheels/settings").status_code == 200
    assert kiosk_client.get("/api/devices/ports").status_code == 200


def test_pin_codes_come_from_the_environment(config, monkeypatch):
    """Коды можно задать в .env: длинные, свои — не только заводские."""
    from robot_control.config import Config

    custom = Config(testing=True, slam_source="none",
                    slam_lock_file=config.slam_lock_file, stats_file=config.stats_file,
                    kiosk_pin_main="1357", kiosk_pin_monitor="2468")
    application = kiosk_module.create_kiosk_app(custom)
    application.config.update(TESTING=True)
    client = application.test_client()

    assert open_mode(client, "7232").status_code == 403
    assert open_mode(client, "2468").get_json()["mode"] == "monitor"


# ------------------------- встроенный предпросмотр (без cookie) ------------- #
def _token(client, pin="5656"):
    body = open_mode(client, pin).get_json()
    assert body["ok"] and body["token"], body
    return body


def test_monitor_opens_by_signed_link_without_cookie(kiosk_client):
    """В iframe cookie не сохраняется: режим подтверждается ссылкой из ответа."""
    body = _token(kiosk_client)
    client = kiosk_client.application.test_client()          # совсем новый «браузер»
    assert body["param"] == "m"
    page = body["page"]
    response = client.get(page)
    assert response.status_code == 200
    assert "monitor.js" in response.get_data(as_text=True)


def test_signed_link_keeps_monitor_restrictions(kiosk_client):
    token = _token(kiosk_client)["token"]
    client = kiosk_client.application.test_client()
    state = client.get("/api/kiosk_state?m=" + token)
    assert state.status_code == 200
    assert "battery" in client.get("/api/battery?m=" + token).get_json()
    steer = client.post("/api/kiosk_steer?m=" + token, json={"module": "FL", "angle": 5},
                        headers={"X-Robot-Control": "steering"})
    assert steer.status_code == 403
    assert steer.get_json()["reason"] == "monitor_mode"


def test_root_follows_signed_link(kiosk_client):
    token = _token(kiosk_client)["token"]
    client = kiosk_client.application.test_client()
    response = client.get("/?m=" + token)
    assert response.status_code == 302
    assert response.headers["Location"].startswith("/monitor?m=")


def test_forged_link_is_refused(kiosk_client):
    """Не подписанный ссылкой режим не открывается и прав монитора не даёт."""
    client = kiosk_client.application.test_client()
    assert client.get("/monitor?m=monitor").status_code == 302
    assert client.get("/api/kiosk_mode?m=monitor").get_json()["mode"] is None
    close = client.post("/api/kiosk_lock/close?m=monitor", json={})
    assert close.status_code == 200          # это обычный клиент без режима, не монитор


def test_start_screen_does_not_print_the_codes(kiosk_client):
    """На стартовом экране нет подписи, какой PIN что открывает."""
    page = kiosk_client.get("/start").get_data(as_text=True)
    js = kiosk_client.get("/start.js").get_data(as_text=True)
    for text in (page, js):
        assert "7232" not in text
        assert "5656" not in text
        assert "Коды режимов" not in text and "коды режимов" not in text
