"""Бортовой экран робота (киоск): страница на 8090, открытие экрана, флаги.

Проверяем три вещи:
  * публичная телеметрия и экран киоска работают без логина;
  * функции киоска доступны без PIN, а грузовой кейс открывается только с PIN;
  * запуск экрана в браузере работает без графики — «браузером» в тестах
    служит скрипт-заглушка, который только отмечает свой запуск в файле.
"""

from __future__ import annotations

import os
import shutil
import signal
import socket
import stat
import sys
import threading
import time
from pathlib import Path

import pytest
from werkzeug.serving import make_server

from robot_control import kiosk_app as kiosk_module
from robot_control.config import Config
from robot_control.kiosk_app import (
    DEFAULT_PORT,
    PROFILE_DIR_NAME,
    _install_stop_signals,
    _probe_hosts,
    _reopen_delay,
    browser_command,
    create_kiosk_app,
    display_available,
    display_claimed,
    find_browser,
    kiosk_port,
    kiosk_profile_dir,
    kiosk_url,
    open_kiosk_screen,
    parse_args,
    port_in_use,
    prepare_browser_profile,
    snap_name,
    wait_for_server,
)


#: Ход руля, общий для прошивки, сервера и экранов (STEER_LIMIT_DEG).
STEER_LIMIT_DEG = 100


@pytest.fixture
def kiosk_app(config: Config):
    application = create_kiosk_app(config)
    application.config.update(TESTING=True)
    return application


@pytest.fixture
def kiosk_client(kiosk_app):
    return kiosk_app.test_client()


# ------------------------------- страница и API ------------------------------ #
def test_root_opens_the_screen_itself(kiosk_client):
    """Корень без запроса — сразу на экран: браузер робота открывает /."""
    response = kiosk_client.get("/")
    assert response.status_code == 302
    assert response.headers["Location"] == "/kiosk"


@pytest.mark.parametrize("path", ["/kiosk", "/robot.html", "/main.html"])
def test_screen_is_served_without_login(kiosk_client, path):
    response = kiosk_client.get(path)
    assert response.status_code == 200
    text = response.get_data(as_text=True)
    assert "Бортовой экран" in text      # заголовок киоска
    assert "k-soc" in text               # крупный заряд АКБ
    assert "robot.js" in text


def test_screen_assets_are_served(kiosk_client):
    for name in ("robot.css", "robot.js"):
        response = kiosk_client.get(f"/{name}")
        assert response.status_code == 200, name


def test_screen_contains_individual_steering_controls(kiosk_client):
    text = kiosk_client.get("/kiosk").get_data(as_text=True)
    js = kiosk_client.get("/robot.js").get_data(as_text=True)
    assert 'id="swipe-motors-view"' in text
    assert 'class="swipe-tabs"' not in text
    assert 'id="swipe-tab-cameras"' not in text
    assert 'id="swipe-tab-motors"' not in text
    assert 'class="swipe-handle' not in text
    for module in ("FL", "FR", "RL", "RR"):
        assert f'id="steer-slider-{module}"' in text
        assert f'data-steer-send="{module}"' in text
    assert "api/kiosk_steer" in js
    assert "X-Robot-Control" in js
    assert "selectSwipeView(view || 'motors')" in js
    assert "openSwipe('motors')" in js  # переход к панели руля и тяги
    assert "openSwipe('cameras')" in js
    assert "if (dx > 65) openSwipe('motors');" in js
    assert "else if (dx < -65) openSwipe('cameras');" in js
    assert 'id="swipe-open-cameras"' not in text
    assert 'поворот модулей →' in text and '← камеры · логи' in text


def test_telemetry_api_returns_only_public_values(kiosk_client):
    for path in ("/api/kiosk_state", "/api/state"):
        response = kiosk_client.get(path)
        assert response.status_code == 200, path
        body = response.get_json()
        assert body["ok"] is True
        assert set(body["data"]) == {
            "battery", "motors", "lidar", "speedMps", "powerKw", "mode", "linkOk", "ts",
            "source", "steeringControl", "drive", "lidarStatus", "lidarDemo", "masterMessages",
            # состояние E-STOP публично: экран обязан показывать защёлку
            "estop",
            # какой порт мастера настроен/открыт: экран объясняет «нет связи»
            "masterPort",
        }
        # причина «нет скана» видна на экране, а не только в журнале
        status = body["data"]["lidarStatus"]
        assert status["ok"] is True
        assert isinstance(status["reason"], str) and status["reason"]


def test_pin_is_required_only_when_opening_case(kiosk_client):
    # В киоске больше нет отдельной PIN-проверки/сессии для доступа к функциям.
    assert kiosk_client.post("/api/kiosk_lock/verify", json={"pin": "7232"}).status_code == 404
    no_pin = kiosk_client.post("/api/kiosk_lock/open", json={})
    assert no_pin.status_code == 403
    assert no_pin.get_json()["reason"] == "format"

    wrong = kiosk_client.post("/api/kiosk_lock/open", json={"pin": "0000"})
    assert wrong.status_code == 403 and wrong.get_json()["reason"] == "wrong"

    opened = kiosk_client.post("/api/kiosk_lock/open", json={"pin": "7232"})
    assert opened.status_code == 200 and opened.get_json()["lock"]["open"] is True
    closed = kiosk_client.post("/api/kiosk_lock/close", json={})
    assert closed.status_code == 200 and closed.get_json()["lock"]["open"] is False


def test_steering_endpoint_is_available_without_pin_but_requires_command_header(kiosk_client):
    body = {"module": "FL", "angle": 10}
    headers = {"X-Robot-Control": "steering"}
    assert kiosk_client.get("/api/kiosk_lock").get_json()["steeringAuthorized"] is True
    assert kiosk_client.post("/api/kiosk_steer", json=body).status_code == 400
    # Эта фикстура работает на симуляторе, поэтому она заведомо не может двигать моторы.
    response = kiosk_client.post("/api/kiosk_steer", json=body, headers=headers)
    assert response.status_code == 409
    assert response.get_json()["reason"] == "control_not_supported"


def test_steering_endpoint_dispatches_exactly_one_module_and_angle(kiosk_app, kiosk_client):
    class FakeSteeringSource:
        name = "master"
        supports_individual_steer = True

        def __init__(self):
            self.commands = []

        def steer_module(self, module, angle):
            self.commands.append((module, angle))
            return {"ok": True, "angle": round(angle, 1)}

    source = FakeSteeringSource()
    kiosk_app.extensions["rc_slam"].source = source
    headers = {"X-Robot-Control": "steering"}

    response = kiosk_client.post("/api/kiosk_steer", json={"module": "fr", "angle": 12.5}, headers=headers)
    assert response.status_code == 200
    assert response.get_json()["module"] == "FR"
    assert source.commands == [("FR", 12.5)]

    for bad_body in (
        {"module": "FR", "angle": 12, "pwm": 100},
        {"module": "drive", "angle": 12},
        {"module": "FL", "angle": 101},  # за пределом ±100° (STEER_LIMIT_DEG)
    ):
        response = kiosk_client.post("/api/kiosk_steer", json=bad_body, headers=headers)
        assert response.status_code == 400
    assert source.commands == [("FR", 12.5)]

    # Следующая команда тоже доступна без PIN-сессии.
    assert kiosk_client.post("/api/kiosk_steer", json={"module": "FR", "angle": 10}, headers=headers).status_code == 200


def test_steering_api_reports_the_limit_it_enforces(kiosk_app, kiosk_client):
    """limitDeg в телеметрии и отказ по пределу говорят одно и то же число."""
    from robot_control import slam as slam_module

    class LimitedSource:
        name = "master"
        supports_individual_steer = True
        supports_calibration = True
        MAX_STEER_DEG = STEER_LIMIT_DEG

        def __init__(self):
            self.commands = []

        def steer_module(self, module, angle):
            self.commands.append((module, angle))
            return {"ok": True, "queued": True, "module": module, "angle": angle}

    source = LimitedSource()
    kiosk_app.extensions["rc_slam"].source = source
    assert slam_module.steering_control_info(source)["limitDeg"] == STEER_LIMIT_DEG

    headers = {"X-Robot-Control": "steering"}
    ok = kiosk_client.post("/api/kiosk_steer", json={"module": "FL", "angle": 90}, headers=headers)
    assert ok.status_code == 200, ok.get_json()

    over = kiosk_client.post("/api/kiosk_steer", json={"module": "FL", "angle": 101}, headers=headers)
    assert over.status_code == 400
    body = over.get_json()
    assert body["reason"] == "angle_limit"
    assert body["limit"] == STEER_LIMIT_DEG


def test_steering_api_adopts_the_source_limit(kiosk_app, kiosk_client):
    """Старая прошивка режет ход до ±32° — API не обещает больше, чем примет плата."""
    class OldSource:
        name = "master"
        supports_individual_steer = True
        MAX_STEER_DEG = 32.0

        def __init__(self):
            self.commands = []

        def steer_module(self, module, angle):
            self.commands.append((module, angle))
            return {"ok": True, "queued": True, "module": module, "angle": angle}

    source = OldSource()
    kiosk_app.extensions["rc_slam"].source = source

    from robot_control import slam as slam_module
    assert slam_module.steering_control_info(source)["limitDeg"] == 32.0

    headers = {"X-Robot-Control": "steering"}
    assert kiosk_client.post("/api/kiosk_steer", json={"module": "FL", "angle": 45},
                             headers=headers).status_code == 400
    assert kiosk_client.post("/api/kiosk_steer", json={"module": "FL", "angle": 32},
                             headers=headers).status_code == 200


def test_manual_software_zero_is_available_without_pin_and_targets_one_module(kiosk_app, kiosk_client):
    class FakeSteeringSource:
        name = "master"
        supports_individual_steer = True

        def __init__(self):
            self.zeroed = []

        def zero_module(self, module):
            self.zeroed.append(module)
            return {"ok": True}

    source = FakeSteeringSource()
    kiosk_app.extensions["rc_slam"].source = source
    headers = {"X-Robot-Control": "steering"}
    body = {"module": "rr"}

    assert kiosk_client.post("/api/kiosk_zero", json=body).status_code == 400
    response = kiosk_client.post("/api/kiosk_zero", json=body, headers=headers)
    assert response.status_code == 200
    assert response.get_json()["zeroMode"] == "manual"
    assert source.zeroed == ["RR"]

    for bad_body in ({"module": "FL", "angle": 0}, {"module": "drive"}, {}):
        assert kiosk_client.post("/api/kiosk_zero", json=bad_body, headers=headers).status_code == 400
    assert source.zeroed == ["RR"]


def test_opto_calibration_is_available_without_pin_and_accepts_module_or_all(kiosk_app, kiosk_client):
    class FakeCalibrationSource:
        name = "master"
        supports_individual_steer = True
        supports_calibration = True

        def __init__(self):
            self.calibrated = []

        def calibrate_module(self, module):
            self.calibrated.append(module)
            return {"ok": True, "offline": []}

    source = FakeCalibrationSource()
    kiosk_app.extensions["rc_slam"].source = source
    headers = {"X-Robot-Control": "steering"}

    assert kiosk_client.post("/api/kiosk_calibrate", json={"module": "FR"}).status_code == 400
    response = kiosk_client.post("/api/kiosk_calibrate", json={"module": "fr"}, headers=headers)
    assert response.status_code == 200 and response.get_json()["module"] == "FR"
    response = kiosk_client.post("/api/kiosk_calibrate", json={"module": "all"}, headers=headers)
    assert response.status_code == 200 and response.get_json()["module"] == "ALL"
    for bad_body in ({"module": "XX"}, {"module": "FR", "angle": 0}, {}):
        assert kiosk_client.post("/api/kiosk_calibrate", json=bad_body, headers=headers).status_code == 400
    assert source.calibrated == ["FR", "ALL"]
    entries = kiosk_client.get("/api/kiosk_logs?limit=50").get_json()["data"]["entries"]
    text = "\n".join(entry["msg"] for entry in entries)
    assert "калибровка руля FR по оптодатчику запущена" in text
    assert "калибровка руля всех колёс по оптодатчику запущена" in text


def test_opto_calibration_is_refused_by_sources_without_it(kiosk_client):
    """Симулятор и пульт-наблюдатель не калибруют: честный отказ 409."""
    response = kiosk_client.post("/api/kiosk_calibrate", json={"module": "ALL"},
                                 headers={"X-Robot-Control": "steering"})
    assert response.status_code == 409
    assert response.get_json()["reason"] == "control_not_supported"


def test_kiosk_drive_is_available_without_pin_and_accepts_only_bounded_percent(kiosk_app, kiosk_client):
    class FakeDriveSource:
        name = "master"
        supports_drive = True

        def __init__(self):
            self.commands = []

        def drive(self, percent):
            self.commands.append(percent)
            return {"ok": True, "percent": round(percent)}

    source = FakeDriveSource()
    kiosk_app.extensions["rc_slam"].source = source
    header = {"X-Robot-Control": "drive"}
    body = {"percent": -12.5}

    assert kiosk_client.post("/api/kiosk_drive", json=body).status_code == 400

    response = kiosk_client.post("/api/kiosk_drive", json=body, headers=header)
    assert response.status_code == 200
    assert response.get_json()["percent"] == -12
    assert source.commands == [-12.5]

    for invalid in ({"percent": 35.1}, {"percent": True}, {"percent": 0, "direction": "forward"}, {}):
        assert kiosk_client.post("/api/kiosk_drive", json=invalid, headers=header).status_code == 400
    assert source.commands == [-12.5]

    stopped = kiosk_client.post("/api/kiosk_drive", json={"percent": 0}, headers=header)
    assert stopped.status_code == 200
    assert source.commands == [-12.5, 0]


def test_kiosk_drive_single_wheel_uses_its_own_command(kiosk_app, kiosk_client):
    """Тяга одного колеса: {module, percent} на борт, как {module, angle} у руля."""
    class FakeWheelSource:
        name = "master"
        supports_drive = True
        supports_individual_drive = True

        def __init__(self):
            self.calls = []

        def drive(self, percent):
            self.calls.append(("all", percent))
            return {"ok": True, "percent": round(percent)}

        def drive_module(self, module, percent):
            self.calls.append((module, percent))
            if module == "RL":
                return {"ok": False, "reason": "module_offline"}
            return {"ok": True, "queued": True, "module": module, "percent": round(percent)}

    source = FakeWheelSource()
    kiosk_app.extensions["rc_slam"].source = source
    header = {"X-Robot-Control": "drive"}

    # Заголовок команды обязателен и здесь.
    assert kiosk_client.post("/api/kiosk_drive",
                             json={"module": "FL", "percent": 8}).status_code == 400
    response = kiosk_client.post("/api/kiosk_drive", json={"module": "fl", "percent": 6},
                                 headers=header)
    assert response.status_code == 200 and response.get_json()["module"] == "FL"
    assert source.calls == [("FL", 6)]

    # Ноль колеса проходит всегда и останавливает только его.
    assert kiosk_client.post("/api/kiosk_drive", json={"module": "FL", "percent": 0},
                             headers=header).status_code == 200
    assert source.calls[-1] == ("FL", 0)

    # Отказы: неизвестное колесо, лишнее поле, отказ платы.
    for invalid in ({"module": "XX", "percent": 6}, {"module": "FL"},
                    {"module": "FL", "percent": 6, "direction": "forward"}):
        assert kiosk_client.post("/api/kiosk_drive", json=invalid, headers=header).status_code == 400
    refused = kiosk_client.post("/api/kiosk_drive", json={"module": "RL", "percent": 6},
                                headers=header)
    assert refused.status_code == 503 and refused.get_json()["reason"] == "module_offline"

    # Общая команда осталась прежней.
    assert kiosk_client.post("/api/kiosk_drive", json={"percent": 4},
                             headers=header).status_code == 200
    assert source.calls[-1] == ("all", 4)


def test_kiosk_state_exposes_the_current_estop_latch(kiosk_client):
    from robot_control import estop

    expected = estop.latch("test E-STOP")
    body = kiosk_client.get("/api/kiosk_state").get_json()
    assert body["data"]["estop"] == expected


def test_telemetry_has_no_cargo_or_audit_data(kiosk_client):
    """В киоске нет ни замка отсека, ни журнала — только АКБ, шасси, лидар."""
    body = kiosk_client.get("/api/kiosk_state").get_json()
    assert "lock" not in body["data"]
    assert "audit" not in body["data"]
    assert "cargo" not in str(body["data"]).lower()


def test_healthz_reports_kiosk_mode(kiosk_client):
    body = kiosk_client.get("/healthz").get_json()
    assert body["ok"] is True
    assert body["mode"] == "kiosk"


# ---------------------- журнал событий на борту ----------------------- #
def test_journal_endpoint_is_empty_and_honest_on_a_fresh_screen(kiosk_client):
    """Событий не было — так и написано; выдуманных строк в журнале нет."""
    body = kiosk_client.get("/api/kiosk_logs").get_json()
    assert body["ok"] is True
    data = body["data"]
    assert data["demo"] is False              # это не демо-поток
    assert data["entries"] == []
    assert data["count"] == 0
    assert data["levels"]["warn"] == 0


def test_journal_records_real_pin_and_command_events(kiosk_app, kiosk_client):
    """Настоящие события робота: неудачный PIN, принятый PIN, команда руления."""
    class FakeSteeringSource:
        name = "master"
        supports_individual_steer = True

        def steer_module(self, module, angle):
            return {"ok": True, "angle": angle}

    kiosk_app.extensions["rc_slam"].source = FakeSteeringSource()
    kiosk_client.post("/api/kiosk_lock/open", json={"pin": "0000"})
    kiosk_client.post("/api/kiosk_lock/open", json={"pin": "7232"})
    kiosk_client.post("/api/kiosk_steer", json={"module": "FL", "angle": 12.5},
                      headers={"X-Robot-Control": "steering"})
    # Угол за пределом хода (±100°): команда обязана быть отклонена и попасть
    # в журнал — иначе оператор не поймёт, почему колесо не поехало.
    kiosk_client.post("/api/kiosk_steer", json={"module": "FL", "angle": 101},
                      headers={"X-Robot-Control": "steering"})

    entries = kiosk_client.get("/api/kiosk_logs?limit=50").get_json()["data"]["entries"]
    text = "\n".join(entry["msg"] for entry in entries)
    assert "PIN не принят" in text and "неверный PIN" in text
    assert "PIN принят" in text
    assert "руление FL → +12.5°" in text
    assert "отклонено: угол вне допуска" in text
    # Сам код PIN в журнал не попадает ни в каком виде.
    assert "7232" not in text and "0000" not in text
    # Демо-строк прежней панели здесь больше нет.
    for fake in ("маршрут: склад", "навигация: коррекция курса", "SLAM: скан лидара 360°"):
        assert fake not in text


def test_journal_shows_why_the_scan_is_missing(kiosk_client):
    """Причина «скана нет» попадает в журнал — без неё экран молчит о лидаре."""
    kiosk_client.get("/api/kiosk_state")
    entries = kiosk_client.get("/api/kiosk_logs").get_json()["data"]["entries"]
    assert any(entry["source"] == "лидар" for entry in entries), entries


def test_journal_limit_is_bounded(kiosk_client):
    for bad in ("0", "abc", "100000"):
        body = kiosk_client.get(f"/api/kiosk_logs?limit={bad}").get_json()
        assert body["ok"] is True
        assert isinstance(body["data"]["entries"], list)


def test_screen_has_no_demo_log_stream(kiosk_client):
    """Панель «Логи» берёт журнал робота, а не выдуманный поток строк."""
    js = kiosk_client.get("/robot.js").get_data(as_text=True)
    assert "api/kiosk_logs" in js
    assert "api/audit" not in js
    assert "demoMsgs" not in js                  # демо-строк нет
    assert "demoLogsOn" not in js
    html = kiosk_client.get("/kiosk").get_data(as_text=True)
    assert "/api/kiosk_logs" in html
    assert "/api/audit" not in html


def test_screen_can_be_embedded_in_preview(kiosk_client):
    """Экран показывают во встроенном предпросмотре — запрет iframe не ставим."""
    response = kiosk_client.get("/kiosk")
    assert "X-Frame-Options" not in response.headers
    assert response.headers["Cache-Control"] == "no-store"


# ------------------------------ адрес и браузер ------------------------------ #
def test_screen_lives_on_port_8090(monkeypatch):
    monkeypatch.delenv("RC_KIOSK_PORT", raising=False)
    assert DEFAULT_PORT == 8090
    assert kiosk_port() == 8090
    assert kiosk_url() == "http://127.0.0.1:8090/kiosk"


def test_port_comes_from_environment(monkeypatch):
    monkeypatch.setenv("RC_KIOSK_PORT", "8091")
    assert kiosk_port() == 8091
    assert kiosk_url(port=8091) == "http://127.0.0.1:8091/kiosk"


def test_broken_port_value_falls_back_to_default(monkeypatch):
    monkeypatch.setenv("RC_KIOSK_PORT", "не число")
    assert kiosk_port() == DEFAULT_PORT


@pytest.mark.parametrize(
    "env, expected",
    [
        ({}, False),
        ({"DISPLAY": ":0"}, True),
        ({"WAYLAND_DISPLAY": "wayland-0"}, True),
    ],
)
def test_display_detection(env, expected):
    assert display_available(env) is expected


def test_chromium_opens_without_frames():
    args = browser_command("/usr/bin/chromium-browser", "http://127.0.0.1:8090/kiosk")
    assert args[0] == "/usr/bin/chromium-browser"
    assert "--kiosk" in args
    assert "--app=http://127.0.0.1:8090/kiosk" in args


def test_chromium_uses_its_own_profile():
    """Свой профиль: повторный запуск не «передаёт адрес» чужой копии браузера.

    Без --user-data-dir процесс Chromium сразу выходит (окно открыла уже
    работающая копия), сторожевой таймер считает это закрытием окна и
    открывает экран снова и снова — окна плодятся.
    """
    args = browser_command(
        "/usr/bin/chromium-browser", "http://127.0.0.1:8090/kiosk",
        profile_dir="/tmp/robot-kiosk-profile",
    )
    assert "--user-data-dir=/tmp/robot-kiosk-profile" in args


def test_firefox_opens_in_kiosk_mode():
    args = browser_command(
        "/usr/bin/firefox", "http://127.0.0.1:8090/kiosk",
        profile_dir="/tmp/robot-kiosk-profile",
    )
    assert args == [
        "/usr/bin/firefox", "--kiosk", "--no-remote",
        "--profile", "/tmp/robot-kiosk-profile",
        "http://127.0.0.1:8090/kiosk",
    ]


def test_snap_name_from_path_and_wrapper(tmp_path: Path):
    """Ubuntu кладёт в PATH обёртку, а не сам бинарник snap."""
    assert snap_name("/snap/bin/firefox") == "firefox"
    assert snap_name("/snap/bin/firefox.geckodriver") == "firefox"
    assert snap_name("/snap/firefox/current/usr/lib/firefox/firefox") == "firefox"
    assert snap_name(str(tmp_path / "no-such-firefox")) is None

    wrapper = tmp_path / "firefox"
    wrapper.write_text("#!/bin/sh\nexec /snap/bin/firefox \"$@\"\n", encoding="utf-8")
    assert snap_name(str(wrapper)) == "firefox"

    via_run = tmp_path / "chromium-browser"
    via_run.write_text("#!/bin/sh\nexec snap run chromium \"$@\"\n", encoding="utf-8")
    assert snap_name(str(via_run)) == "chromium"

    native = tmp_path / "firefox-bin"
    native.write_bytes(b"\x7fELF")
    assert snap_name(str(native)) is None


def test_prepare_creates_missing_firefox_profile(tmp_path, monkeypatch):
    """Каталога нет — Firefox сам его не создаст и покажет «profile cannot be loaded»."""
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    monkeypatch.setenv("XDG_CACHE_HOME", str(home / "cache"))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    assert profile == (home / "cache" / PROFILE_DIR_NAME).resolve()
    assert profile.is_dir()
    assert oct(profile.stat().st_mode & 0o777) == "0o700"
    text = (profile / "user.js").read_text(encoding="utf-8")
    assert "browser.shell.checkDefaultBrowser" in text
    assert "browser.sessionstore.resume_from_crash" in text


def test_prepare_does_not_clobber_existing_user_js(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("XDG_CACHE_HOME", str(home / "cache"))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    profile = prepare_browser_profile(str(exe))
    (profile / "user.js").write_text("user_pref(\"custom\", true);\n", encoding="utf-8")

    again = prepare_browser_profile(str(exe))

    assert again == profile
    assert (profile / "user.js").read_text(encoding="utf-8") == "user_pref(\"custom\", true);\n"


def test_snap_firefox_profile_is_not_in_a_hidden_dir(tmp_path, monkeypatch):
    """Snap не читает ~/.cache — оттуда и диалог «profile cannot be loaded»."""
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    monkeypatch.delenv("XDG_CACHE_HOME", raising=False)
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexec /snap/bin/firefox \"$@\"\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    expected = home / "snap" / "firefox" / "common" / PROFILE_DIR_NAME
    assert profile == expected.resolve()
    assert profile.is_dir()
    assert (profile / "user.js").is_file()
    assert not any(part.startswith(".") for part in profile.relative_to(home.resolve()).parts)


def test_snap_keeps_explicit_visible_profile(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    visible = home / "kiosk-profile"
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("RC_KIOSK_PROFILE_DIR", str(visible))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexec snap run firefox \"$@\"\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    assert profile == visible.resolve()
    assert profile.is_dir()


def test_snap_relocates_tmp_and_hidden_profiles(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexec /snap/bin/firefox \"$@\"\n", encoding="utf-8")
    expected = (home / "snap" / "firefox" / "common" / PROFILE_DIR_NAME).resolve()

    from_tmp = prepare_browser_profile(str(exe), profile_dir=tmp_path / "outside")
    from_hidden = prepare_browser_profile(
        str(exe), profile_dir=home / ".cache" / PROFILE_DIR_NAME,
    )

    assert from_tmp == expected
    assert from_hidden == expected


def test_snap_falls_back_to_visible_home_dir_when_snap_tree_is_blocked(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    (home / "snap").write_text("not a directory", encoding="utf-8")
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexec /snap/bin/firefox \"$@\"\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    assert profile == (home / PROFILE_DIR_NAME).resolve()
    assert profile.is_dir()
    assert (profile / "user.js").is_file()


def test_chromium_snap_gets_a_visible_profile_without_firefox_prefs(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    exe = tmp_path / "chromium-browser"
    exe.write_text("#!/bin/sh\nexec /snap/bin/chromium \"$@\"\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    assert profile == (home / "snap" / "chromium" / "common" / PROFILE_DIR_NAME).resolve()
    assert profile.is_dir()
    assert not (profile / "user.js").exists()


def test_stale_firefox_lock_is_cleared_and_live_lock_is_kept(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("XDG_CACHE_HOME", str(home / "cache"))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))
    (profile / "lock").symlink_to("127.0.0.1:+999999999")
    (profile / ".parentlock").write_text("", encoding="utf-8")
    prepare_browser_profile(str(exe))
    assert not (profile / "lock").exists()
    assert not (profile / ".parentlock").exists()

    (profile / "lock").symlink_to(f"127.0.0.1:+{os.getpid()}")
    prepare_browser_profile(str(exe))
    assert (profile / "lock").is_symlink()


def test_screen_launches_snap_firefox_with_a_readable_profile(config, tmp_path, monkeypatch):
    """Окно ошибки больше не из чего рисовать: каталог есть и snap его видит."""
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    monkeypatch.delenv("XDG_CACHE_HOME", raising=False)
    marker = tmp_path / "opened.txt"
    script = tmp_path / "firefox"
    script.write_text(
        "#!/bin/sh\n"
        "# Ubuntu wrapper: exec /snap/bin/firefox \"$@\"\n"
        f'echo "$@" >> "{marker}"\n',
        encoding="utf-8",
    )
    script.chmod(script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(script))

    server = make_server("127.0.0.1", 0, create_kiosk_app(config), threaded=True)
    port = server.server_port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    stop = threading.Event()
    try:
        open_kiosk_screen(port, watch=False, stop_event=stop)
        opened = _wait_for(marker, 1)
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    expected = (home / "snap" / "firefox" / "common" / PROFILE_DIR_NAME).resolve()
    assert opened, "экран не открылся"
    assert "--profile" in opened[0]
    assert str(expected) in opened[0]
    assert expected.is_dir()
    assert (expected / "user.js").is_file()
    assert "--kiosk" in opened[0]


def test_kiosk_profile_dir_from_environment(monkeypatch):
    monkeypatch.setenv("RC_KIOSK_PROFILE_DIR", "/tmp/custom-kiosk")
    assert kiosk_profile_dir() == Path("/tmp/custom-kiosk")
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR")
    monkeypatch.setenv("XDG_CACHE_HOME", "/tmp/cache")
    assert kiosk_profile_dir() == Path("/tmp/cache/robot-kiosk-browser")


def test_reopen_delay_grows_for_instantly_exiting_browser():
    """Браузер выходит сразу — не долбим его каждые 2 секунды очередями окон."""
    assert _reopen_delay(0) == 2.0
    assert _reopen_delay(1) == 2.0
    assert _reopen_delay(2) == 4.0
    assert _reopen_delay(3) == 8.0
    assert _reopen_delay(100) == 30.0  # потолок


def test_browser_can_be_set_explicitly(monkeypatch):
    monkeypatch.setenv("RC_KIOSK_BROWSER", sys.executable)
    assert find_browser() == sys.executable


def test_missing_browser_is_not_an_error(monkeypatch):
    """Нет браузера — не падаем: сервер живёт, подсказка уходит в лог."""
    monkeypatch.setenv("RC_KIOSK_BROWSER", "/nonexistent/browser")
    monkeypatch.setattr(shutil, "which", lambda _name: None)
    assert find_browser() is None


# --------------------------------- порт и сервер ----------------------------- #
def test_port_in_use_detects_listener():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        sock.listen(1)
        busy = sock.getsockname()[1]
        assert port_in_use(busy) is True
    assert port_in_use(busy) is False


def test_wait_for_server_times_out_on_dead_port():
    with socket.socket() as sock:          # заведомо свободный порт
        sock.bind(("127.0.0.1", 0))
        free = sock.getsockname()[1]
    assert wait_for_server(f"http://127.0.0.1:{free}", timeout=0.3) is False


@pytest.fixture
def fake_browser(tmp_path: Path) -> Path:
    """«Браузер»: пишет строку в файл и сразу выходит (графики в тестах нет)."""
    marker = tmp_path / "opened.txt"
    script = tmp_path / "fake-browser.sh"
    script.write_text(
        "#!/bin/sh\n"
        f'echo "$@" >> "{marker}"\n',
        encoding="utf-8",
    )
    script.chmod(script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return script


def _wait_for(marker: Path, lines: int, timeout: float = 20.0) -> list[str]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if marker.exists():
            got = [line for line in marker.read_text(encoding="utf-8").splitlines() if line]
            if len(got) >= lines:
                return got
        time.sleep(0.2)
    return []


def test_screen_opens_itself_when_server_starts(config, fake_browser, monkeypatch):
    """Главное требование: экран на 8090 открывается сам, без команд в браузере."""
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(fake_browser))
    server = make_server("127.0.0.1", 0, create_kiosk_app(config), threaded=True)
    port = server.server_port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    stop = threading.Event()
    try:
        open_kiosk_screen(port, watch=False, stop_event=stop)
        opened = _wait_for(fake_browser.parent / "opened.txt", 1)
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
    assert opened, "экран не открылся в браузере"
    assert "/kiosk" in opened[0]


def test_screen_reopens_if_browser_was_closed(config, fake_browser, monkeypatch):
    """Киоск-режим: закрыли браузер — экран возвращается на дисплей сам."""
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(fake_browser))
    server = make_server("127.0.0.1", 0, create_kiosk_app(config), threaded=True)
    port = server.server_port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    stop = threading.Event()
    try:
        open_kiosk_screen(port, watch=True, stop_event=stop)
        opened = _wait_for(fake_browser.parent / "opened.txt", 2)
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
    assert len(opened) >= 2, f"экран не открылся заново: {opened}"


@pytest.fixture
def long_lived_browser(tmp_path: Path) -> Path:
    """«Браузер», который не выходит сразу: окно как будто открыто."""
    marker = tmp_path / "opened.txt"
    script = tmp_path / "long-browser.sh"
    script.write_text(
        "#!/bin/sh\n"
        f'echo "$@" >> "{marker}"\n'
        "sleep 60\n",
        encoding="utf-8",
    )
    script.chmod(script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return script


def test_screen_is_not_opened_twice(config, long_lived_browser, monkeypatch):
    """Главное: повторный запуск не плодит окна — экран строго в одном экземпляре.

    Автозапуск рабочего стола, ярлык и ручной kiosk.sh приводят сюда же:
    кто первый занял показ, тот окно и держит, остальные получают отказ.
    """
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(long_lived_browser))
    server = make_server("127.0.0.1", 0, create_kiosk_app(config), threaded=True)
    port = server.server_port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    stop = threading.Event()
    stop2 = threading.Event()
    marker = long_lived_browser.parent / "opened.txt"
    try:
        first = open_kiosk_screen(port, watch=True, stop_event=stop)
        assert first is not None
        assert _wait_for(marker, 1), "первый экран не открылся"
        assert display_claimed(port) is True

        # Второй и третий показы того же порта — отказ, а не новое окно.
        assert open_kiosk_screen(port, watch=True, stop_event=threading.Event()) is None
        assert open_kiosk_screen(port, watch=False, stop_event=threading.Event()) is None
        assert len(_wait_for(marker, 1)) == 1, "экран открылся повторно"

        # Показ остановлен — замок отпущен, экран можно открыть снова.
        stop.set()
        first.join(timeout=10)
        assert display_claimed(port) is False
        assert open_kiosk_screen(port, watch=True, stop_event=stop2) is not None
        assert len(_wait_for(marker, 2)) == 2
    finally:
        stop.set()
        stop2.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


# ------------------- остановка сервера гасит экран на мониторе --------------- #
@pytest.fixture
def fast_server_checks(monkeypatch):
    """Проверки «жив ли сервер» чаще и короче: тест не ждёт боевые 15 секунд."""
    monkeypatch.setattr(kiosk_module, "SERVER_CHECK_SEC", 0.1)
    monkeypatch.setattr(kiosk_module, "SERVER_GONE_SEC", 0.5)


@pytest.fixture
def window_browser(tmp_path: Path) -> Path:
    """«Браузер», который держит окно открытым и пишет свой pid (как настоящее окно)."""
    script = tmp_path / "window-browser.sh"
    script.write_text(
        "#!/bin/sh\n"
        f'echo "$@" >> "{tmp_path / "opened.txt"}"\n'
        f'echo $$ > "{tmp_path / "browser.pid"}"\n'
        "exec sleep 60\n",  # exec: TERM убивает и «окно», и его сон
        encoding="utf-8",
    )
    script.chmod(script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return script


def _start_kiosk_server(config: Config):
    """Настоящий сервер киоска на случайном порту + поток обслуживания."""
    server = make_server("127.0.0.1", 0, create_kiosk_app(config), threaded=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def _stop_kiosk_server(server, thread) -> None:
    server.shutdown()
    server.server_close()
    thread.join(timeout=5)


def _browser_pid(tmp_path: Path) -> int:
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        pid_file = tmp_path / "browser.pid"
        if pid_file.exists():
            try:
                return int(pid_file.read_text(encoding="utf-8").strip())
            except ValueError:
                pass
        time.sleep(0.05)
    raise AssertionError("окно киоска так и не открылось")


def _process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:  # pragma: no cover - чужой процесс
        return True
    return True


def _wait_dead(pid: int, timeout: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _process_alive(pid):
            return True
        time.sleep(0.05)
    return False


def test_screen_watchdog_gives_up_when_server_is_stopped(
        config, fake_browser, monkeypatch, fast_server_checks):
    """Баг «монитор запускается снова и снова»: сервер остановлен — показ должен
    прекратиться, а не переоткрывать окно вечно.

    Именно так робот и жил раньше: ``./scripts/stop.sh`` (или кнопка «Остановить
    всё», или ``systemctl stop robot-kiosk``) гасила сервер и окно, а сторож
    экрана в отдельном процессе продолжал открывать браузер заново — уже на
    остановленном роботе и на мёртвый адрес.
    """
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(fake_browser))
    server, thread = _start_kiosk_server(config)
    port = server.server_port
    marker = fake_browser.parent / "opened.txt"
    stop = threading.Event()
    display = None
    try:
        display = open_kiosk_screen(port, watch=True, stop_event=stop)
        assert display is not None
        assert _wait_for(marker, 1), "экран не открылся при живом сервере"

        _stop_kiosk_server(server, thread)
        server = None

        display.join(timeout=15)
        assert not display.is_alive(), "сторож экрана не остановился вместе с сервером"
        assert display_claimed(port) is False, "замок показа не отпущен"

        opened = len(_wait_for(marker, 1))
        time.sleep(1.5)  # раньше за это время окно открывалось ещё раз-другой
        assert len(_wait_for(marker, 1)) == opened, \
            "экран открылся заново уже после остановки сервера"
    finally:
        stop.set()
        if display is not None:
            display.join(timeout=10)
        if server is not None:
            _stop_kiosk_server(server, thread)


def test_screen_window_is_closed_when_server_is_stopped(
        config, window_browser, monkeypatch, fast_server_checks):
    """Окно на мониторе гаснет вместе с сервером, а не висит на мёртвом адресе."""
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(window_browser))
    tmp_path = window_browser.parent
    server, thread = _start_kiosk_server(config)
    port = server.server_port
    stop = threading.Event()
    display = None
    browser_pid = 0
    try:
        display = open_kiosk_screen(port, watch=True, stop_event=stop)
        assert display is not None
        browser_pid = _browser_pid(tmp_path)
        assert _process_alive(browser_pid), "окно киоска не открыто"

        _stop_kiosk_server(server, thread)
        server = None

        assert _wait_dead(browser_pid), "окно киоска осталось на мониторе"
        display.join(timeout=10)
        assert not display.is_alive()
        assert display_claimed(port) is False
    finally:
        stop.set()
        if display is not None:
            display.join(timeout=10)
        if browser_pid and _process_alive(browser_pid):
            os.kill(browser_pid, signal.SIGKILL)
        if server is not None:
            _stop_kiosk_server(server, thread)


def test_screen_survives_short_server_restart(
        config, window_browser, monkeypatch):
    """Перезапуск сервера (обновление, ``systemctl restart``) экран не гасит.

    Сокет на пару секунд освобождается, и без запаса терпения сторож решил бы,
    что робот остановлен, — монитор гас среди работы.
    """
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(window_browser))
    monkeypatch.setattr(kiosk_module, "SERVER_CHECK_SEC", 0.1)
    monkeypatch.setattr(kiosk_module, "SERVER_GONE_SEC", 5.0)
    tmp_path = window_browser.parent
    server, thread = _start_kiosk_server(config)
    port = server.server_port
    stop = threading.Event()
    display = None
    browser_pid = 0
    try:
        display = open_kiosk_screen(port, watch=True, stop_event=stop)
        assert display is not None
        browser_pid = _browser_pid(tmp_path)

        _stop_kiosk_server(server, thread)          # «сервер перезапускается»
        time.sleep(0.8)                             # порт свободен меньше запаса
        server, thread = _start_kiosk_server_on(port, config)

        time.sleep(1.5)
        assert _process_alive(browser_pid), "экран погас при перезапуске сервера"
        assert display.is_alive(), "сторож экрана вышел при перезапуске сервера"
        assert display_claimed(port) is True
    finally:
        stop.set()
        if display is not None:
            display.join(timeout=10)
        if browser_pid and _process_alive(browser_pid):
            os.kill(browser_pid, signal.SIGKILL)
        if server is not None:
            _stop_kiosk_server(server, thread)


def _start_kiosk_server_on(port: int, config: Config):
    """Тот же сервер, но на уже известном порту (имитация возвращения после restart)."""
    server = make_server("127.0.0.1", port, create_kiosk_app(config), threaded=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def test_screen_is_not_opened_without_a_server_at_all(
        config, fake_browser, monkeypatch, fast_server_checks):
    """Сервера нет вовсе — окно не открывается: нечего показывать."""
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(fake_browser))
    with socket.socket() as sock:          # заведомо свободный порт
        sock.bind(("127.0.0.1", 0))
        free = sock.getsockname()[1]
    marker = fake_browser.parent / "opened.txt"
    stop = threading.Event()
    display = None
    try:
        # HEALTH_TIMEOUT_SEC — значение по умолчанию аргумента, его не подменить;
        # поэтому stub-ом делаем саму проверку «сервер ответил».
        monkeypatch.setattr(kiosk_module, "wait_for_server",
                            lambda _base_url, timeout=0.3: False)
        display = open_kiosk_screen(free, watch=True, stop_event=stop)
        assert display is not None
        display.join(timeout=10)
        assert not display.is_alive()
        assert not marker.exists(), "браузер открылся на мёртвый адрес"
        assert display_claimed(free) is False
    finally:
        stop.set()
        if display is not None:
            display.join(timeout=10)


def test_probe_hosts_cover_explicit_bind_address():
    """Сервер на конкретном адресе виден сторожу, а не только 127.0.0.1."""
    assert _probe_hosts(None) == ("127.0.0.1",)
    assert _probe_hosts("0.0.0.0") == ("127.0.0.1",)
    assert _probe_hosts("::") == ("127.0.0.1",)
    assert _probe_hosts("192.168.4.1") == ("127.0.0.1", "192.168.4.1")


def test_stop_signals_close_the_screen_instead_of_dying(monkeypatch):
    """SIGTERM/SIGINT: окно закрывается, а процесс не обрывается мгновенно.

    ``kill`` из ``./scripts/stop.sh`` и Ctrl+C в ярлыке «Запустить робот»
    раньше просто убивали процесс — окно киоска оставалось на мониторе.
    """
    stop = threading.Event()
    shutdowns: list[bool] = []

    class FakeServer:
        def shutdown(self) -> None:
            shutdowns.append(True)

    saved = {name: signal.getsignal(getattr(signal, name))
             for name in ("SIGINT", "SIGTERM")}
    try:
        _install_stop_signals(stop, FakeServer())
        os.kill(os.getpid(), signal.SIGTERM)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not (stop.is_set() and shutdowns):
            time.sleep(0.02)
        assert stop.is_set(), "сигнал не остановил показ экрана"
        assert shutdowns == [True], "сигнал не остановил сервер киоска"
    finally:
        for name, handler in saved.items():
            signal.signal(getattr(signal, name), handler)


# ----------------------------------- запуск ---------------------------------- #
def test_default_flags_mean_open_the_screen():
    args = parse_args([])
    assert args.open_screen is None       # «как настроено/есть ли дисплей»
    assert args.port is None
    assert args.source is None


def test_flags_disable_and_force_opening():
    assert parse_args(["--no-open"]).open_screen is False
    assert parse_args(["--open"]).open_screen is True
    args = parse_args(["--open", "--port", "8091", "--source", "none", "--no-watch"])
    assert (args.port, args.source, args.watch) == (8091, "none", False)


@pytest.mark.parametrize("name", ["kiosk.sh", "robot-kiosk.service"])
def test_kiosk_launch_files_are_in_place(name):
    path = Path(__file__).resolve().parent.parent / "scripts" / name
    assert path.is_file(), name


def test_kiosk_script_is_executable():
    path = Path(__file__).resolve().parent.parent / "scripts" / "kiosk.sh"
    assert os.access(path, os.X_OK), "scripts/kiosk.sh должен быть исполняемым"


def test_pot_mode_api(kiosk_app, kiosk_client):
    class Source:
        def set_pot_mode(self, action):
            return {"ok": True, "queued": True}
    kiosk_app.extensions['rc_slam'].source = Source()
    headers = {'X-Robot-Control': 'steering'}
    assert kiosk_client.post('/api/kiosk_pot_mode', json={'action': 'off'}).status_code == 400
    for body in ({}, {'action': []}, {'action': 'invalid'}, {'action': 'off', 'extra': 1}):
        assert kiosk_client.post('/api/kiosk_pot_mode', json=body, headers=headers).status_code == 400
    for action in ('off', 'on', 'calibrate'):
        assert kiosk_client.post('/api/kiosk_pot_mode', json={'action': action}, headers=headers).status_code == 200


def test_drive_settings_api_is_gone(kiosk_app, kiosk_client):
    """Настройки привода больше нет: маршрут отвечает 404, а не «нельзя»."""
    headers = {'X-Robot-Control': 'steering'}
    for body in ({'action': 'trim', 'value': 1500}, {'action': 'limit', 'value': 30},
                 {'action': 'direction', 'module': 'FL', 'value': 'reverse'}):
        assert kiosk_client.post('/api/kiosk_drive_config', json=body,
                                 headers=headers).status_code in (404, 405), body


def test_maneuver_api(kiosk_app, kiosk_client):
    """Пресеты-манёвры: «метр вперёд», «90° на месте» — старт, стоп, правка чисел."""
    class Source:
        def __init__(self):
            self.calls = []

        def start_maneuver(self, name):
            self.calls.append(('start', name))
            if name == 'телепорт':
                return {"ok": False, "reason": "invalid_maneuver"}
            return {"ok": True, "maneuver": {"name": name, "state": "steering"}}

        def stop_maneuver(self):
            self.calls.append(('stop',))
            return {"ok": True, "maneuver": {"state": "cancelled"}}

        def set_maneuver_config(self, name, values):
            self.calls.append(('config', name, values))
            return {"ok": True}

    source = Source()
    kiosk_app.extensions['rc_slam'].source = source
    headers = {'X-Robot-Control': 'steering'}
    # Без заголовка команды — отказ: это команда движения, а не телеметрия.
    assert kiosk_client.post('/api/kiosk_maneuver',
                             json={'action': 'start', 'name': 'forward'}).status_code == 400
    for body in ({}, {'action': 'start'}, {'action': 'поехали', 'name': 'forward'},
                 {'action': 'start', 'name': 'forward', 'extra': 1}):
        assert kiosk_client.post('/api/kiosk_maneuver', json=body,
                                 headers=headers).status_code == 400, body
    started = kiosk_client.post('/api/kiosk_maneuver',
                                json={'action': 'start', 'name': 'forward'}, headers=headers)
    assert started.status_code == 200, started.get_json()
    assert started.get_json()['maneuver']['state'] == 'steering'
    assert source.calls[-1] == ('start', 'forward')
    assert kiosk_client.post('/api/kiosk_maneuver', json={'action': 'stop'},
                             headers=headers).status_code == 200
    assert source.calls[-1] == ('stop',)
    refused = kiosk_client.post('/api/kiosk_maneuver',
                                json={'action': 'start', 'name': 'телепорт'}, headers=headers)
    assert refused.status_code == 400
    assert refused.get_json()['reason'] == 'invalid_maneuver'

    # Числа пресета правятся отдельным запросом и живут на сервере.
    saved = kiosk_client.post('/api/kiosk_maneuver_config',
                              json={'name': 'forward', 'values': {'distanceM': 1.5, 'percent': 10}},
                              headers=headers)
    assert saved.status_code == 200
    assert source.calls[-1] == ('config', 'forward', {'distanceM': 1.5, 'percent': 10})
    assert kiosk_client.post('/api/kiosk_maneuver_config',
                             json={'name': 'forward'}, headers=headers).status_code == 400
    assert kiosk_client.post('/api/kiosk_maneuver_config',
                             json={'name': 'forward', 'values': {'distanceM': 1.5}}).status_code == 400


def test_firmware_flash_button_api_starts_the_flash(kiosk_app, kiosk_client, monkeypatch):
    """Кнопка «Прошить модули»: состояние читается, запуск — только с заголовком."""
    from robot_control import firmware_flash

    calls = []

    class Source:
        name = "master"

        def suspend_ports(self):
            calls.append("suspend_ports")

    kiosk_app.extensions["rc_slam"].source = Source()
    monkeypatch.setattr(firmware_flash, "flash_state",
                        lambda root=None: {"status": "idle", "pct": 0, "lines": [],
                                           "phase": "", "error": "", "summary": ""})
    monkeypatch.setattr(firmware_flash, "start_flash",
                        lambda source=None, reason="", **kwargs: (
                            calls.append("start_flash") or
                            {"started": True, "status": "busy", "pct": 2, "lines": [],
                             "phase": "Готовлюсь", "error": "", "summary": ""}))

    state = kiosk_client.get("/api/kiosk_firmware_flash")
    assert state.status_code == 200
    assert state.get_json()["ok"] is True and state.get_json()["status"] == "idle"

    # Без заголовка — отказ: прошивку нельзя запустить случайным POST из браузера.
    assert kiosk_client.post("/api/kiosk_firmware_flash", json={}).status_code == 400
    # Лишние поля тоже отказ: у кнопки нет параметров.
    assert kiosk_client.post("/api/kiosk_firmware_flash", json={"port": "/dev/ttyUSB0"},
                             headers={"X-Robot-Control": "firmware-flash"}).status_code == 400

    started = kiosk_client.post("/api/kiosk_firmware_flash", json={},
                                headers={"X-Robot-Control": "firmware-flash"})
    assert started.status_code == 202
    body = started.get_json()
    assert body["ok"] is True and body["started"] is True and body["status"] == "busy"
    assert calls == ["start_flash"]


def test_firmware_flash_is_refused_when_the_board_is_not_master(kiosk_app, kiosk_client):
    """Прошивать порты может только бортовой процесс: пульт получает отказ."""
    class Remote:
        name = "peer"

    kiosk_app.extensions["rc_slam"].source = Remote()
    response = kiosk_client.post("/api/kiosk_firmware_flash", json={},
                                 headers={"X-Robot-Control": "firmware-flash"})
    assert response.status_code == 409
    assert "master" in response.get_json()["error"]


def test_second_flash_click_reports_that_it_is_already_running(kiosk_app, kiosk_client,
                                                               monkeypatch):
    from robot_control import firmware_flash

    class Source:
        name = "master"

    kiosk_app.extensions["rc_slam"].source = Source()
    monkeypatch.setattr(firmware_flash, "start_flash",
                        lambda source=None, reason="", **kwargs: {
                            "started": False, "status": "busy", "pct": 40,
                            "lines": [], "phase": "Прошиваю", "error": "", "summary": ""})
    response = kiosk_client.post("/api/kiosk_firmware_flash", json={},
                                 headers={"X-Robot-Control": "firmware-flash"})
    assert response.status_code == 200
    body = response.get_json()
    assert body["ok"] is True and body["started"] is False and body["pct"] == 40
