"""Проверки веб-интерфейса RUS SLAM: страницы, API, замок отсека, статика."""

from __future__ import annotations

import re

from conftest import login
from robot_control import slam

DEFAULT_PIN = "7232"  # заводской PIN грузового отсека из backend.py


# ------------------------------- доступ ---------------------------------- #
def test_main_screen_requires_login(client):
    response = client.get("/")
    assert response.status_code == 302
    assert response.headers["Location"].endswith("/login?next=/")


def test_console_requires_login(client):
    response = client.get("/console")
    assert response.status_code == 302
    assert "/login" in response.headers["Location"]


def test_api_requires_login(client):
    assert client.get("/api/state").status_code == 302
    assert client.get("/api/health").status_code == 302


# ------------------------------- страницы -------------------------------- #
def test_main_screen_after_login(client, operator):
    login(client)
    response = client.get("/")
    assert response.status_code == 200
    text = response.get_data(as_text=True)
    # это основной экран робота из rus_slam, а не наша заглушка
    assert 'href="main.css"' in text
    assert 'src="main.js"' in text
    assert "Управление роботом ещё не подключено" not in text


def test_main_screen_also_at_main_html(client, operator):
    login(client)
    assert client.get("/main.html").status_code == 200


def test_console_after_login(client, operator):
    login(client)
    response = client.get("/console")
    assert response.status_code == 200
    text = response.get_data(as_text=True)
    for asset in ("console-core.js", "vision.js", "app.js", "console.js", "styles.css"):
        assert asset in text


def test_pages_not_cached(client, operator):
    login(client)
    assert client.get("/").headers["Cache-Control"] == "no-store"


# --------------------------------- API ----------------------------------- #
def test_api_state_shape(client, operator):
    login(client)
    body = client.get("/api/state").get_json()
    assert body["ok"] is True
    data = body["data"]
    assert data["source"] == "none"
    assert len(data["motors"]) == 4
    for motor in data["motors"]:
        assert {"id", "title", "angle", "rpm", "temp", "homed"} <= set(motor)
    battery = data["battery"]
    assert {"soc", "volts", "amps", "rangeKm", "thresholds"} <= set(battery)
    # без робота значения честно пустые (null), а не выдуманные проценты
    assert battery["soc"] is None or 0 <= battery["soc"] <= 100
    assert data["cargo"]["closed"] is True
    assert data["lock"]["open"] is False


def test_kiosk_steering_endpoint_uses_pin_session_without_operator_login(app, client):
    """Киоск открыт без логина оператора, но физическая команда закрыта PIN-сессией."""
    class FakeSteeringSource:
        name = "master"
        supports_individual_steer = True

        def steer_module(self, module, angle):
            return {"ok": True, "angle": angle}

    app.extensions["rc_slam"].source = FakeSteeringSource()
    command = {"module": "RL", "angle": -8}
    header = {"X-Robot-Control": "steering"}
    assert client.post("/api/kiosk_steer", json=command, headers=header).status_code == 401
    assert client.post("/api/kiosk_lock/verify", json={"pin": DEFAULT_PIN}).status_code == 200
    response = client.post("/api/kiosk_steer", json=command, headers=header)
    assert response.status_code == 200
    assert response.get_json()["module"] == "RL"


def test_operator_drive_requires_login_pin_and_drive_header(app, client, operator):
    class FakeDriveSource:
        name = "master"
        supports_drive = True

        def __init__(self):
            self.commands = []

        def drive(self, percent):
            self.commands.append(percent)
            return {"ok": True, "percent": percent}

    source = FakeDriveSource()
    app.extensions["rc_slam"].source = source
    body = {"percent": 8}
    header = {"X-Robot-Control": "drive"}

    assert client.post("/api/drive", json=body, headers=header).status_code == 302
    login(client)
    assert client.post("/api/drive", json=body, headers=header).status_code == 401
    assert client.post("/api/kiosk_lock/verify", json={"pin": DEFAULT_PIN}).status_code == 200
    assert client.post("/api/drive", json=body).status_code == 400

    response = client.post("/api/drive", json=body, headers=header)
    assert response.status_code == 200
    assert response.get_json()["percent"] == 8
    assert source.commands == [8.0]

    client.post("/api/kiosk_lock/session/end", json={})
    assert client.post("/api/drive", json={"percent": 0}, headers=header).status_code == 401


def test_api_health_reports_source(client, operator):
    login(client)
    body = client.get("/api/health").get_json()
    assert body["ok"] is True
    assert body["source"] == "none"
    assert body["uptimeSec"] >= 0


def test_healthz_reports_interface(app):
    body = app.test_client().get("/healthz").get_json()
    assert body["panel"] == "rus_slam"
    assert body["source"] == "none"


# --------------------------- грузовой отсек ------------------------------ #
def test_lock_opens_with_default_pin(client, operator):
    login(client)
    response = client.post("/api/lock/open", json={"pin": DEFAULT_PIN})
    assert response.status_code == 200
    assert response.get_json()["ok"] is True

    state = client.get("/api/state").get_json()["data"]
    assert state["lock"]["open"] is True
    assert state["cargo"]["closed"] is False


def test_lock_rejects_wrong_pin(client, operator):
    login(client)
    response = client.post("/api/lock/open", json={"pin": "0000"})
    assert response.status_code == 403
    assert response.get_json()["ok"] is False

    state = client.get("/api/state").get_json()["data"]
    assert state["lock"]["open"] is False
    assert state["lock"]["attemptsLeft"] == 4  # попытка учтена


def test_lock_closes(client, operator):
    login(client)
    client.post("/api/lock/open", json={"pin": DEFAULT_PIN})
    response = client.post("/api/lock/close")
    assert response.status_code == 200
    assert response.get_json()["ok"] is True
    assert client.get("/api/state").get_json()["data"]["cargo"]["closed"] is True


def test_audit_records_attempts(client, operator):
    login(client)
    client.post("/api/lock/open", json={"pin": "0000"})
    body = client.get("/api/audit?limit=5").get_json()
    assert body["ok"] is True
    assert body["audit"], "журнал пуст"
    assert body["integrity"]["ok"] is True  # хеш-цепочка не повреждена


def test_kiosk_logs_are_real_and_need_login(client, operator):
    """Журнал событий для панели «Логи»: настоящие записи и только после входа.

    На борту (8090) панель публична вместе с телеметрией, а на пульте журнал
    процесса закрыт входом оператора — как и остальные служебные данные.
    """
    assert client.get("/api/kiosk_logs").status_code in (302, 401)

    login(client)
    body = client.get("/api/kiosk_logs?limit=20").get_json()
    assert body["ok"] is True
    data = body["data"]
    assert data["demo"] is False           # выдуманных строк в журнале нет

    # Настоящее событие робота: неверный PIN отсечён — в журнале это видно.
    client.post("/api/lock/open", json={"pin": "0000"})
    client.post("/api/kiosk_lock/verify", json={"pin": "0000"})
    entries = client.get("/api/kiosk_logs?limit=50").get_json()["data"]["entries"]
    text = "\n".join(entry["msg"] for entry in entries)
    assert "PIN" in text and "неверный PIN" in text
    assert "0000" not in text


# -------------------------------- статика -------------------------------- #
def test_assets_are_served(client, operator):
    login(client)
    for name in ("main.css", "main.js", "styles.css", "app.js",
                 "console.js", "console-core.js", "vision.js"):
        response = client.get(f"/{name}")
        assert response.status_code == 200, name
        assert len(response.data) > 100, name


def test_assets_are_public_without_login(client):
    """CSS/JS отдаются без входа: в iframe-предпросмотре cookie нет, и подзапросы
    не могут передать сессию — иначе страница приезжает сломанной."""
    for name in ("main.css", "main.js", "styles.css", "app.js",
                 "console.js", "console-core.js", "vision.js", "slam_auth.js"):
        assert client.get(f"/{name}").status_code == 200, name


def test_data_endpoints_still_closed_without_login(client):
    assert client.get("/api/state").status_code == 302
    assert client.get("/").status_code == 302
    assert client.get("/console").status_code == 302


def test_auth_shim_is_loaded_before_page_scripts(client, operator):
    """Скрипт, пробрасывающий токен st в fetch, должен подключаться первым.

    Сравниваем именно теги <script src=…>: слова вроде "main.js" встречаются
    в тексте страницы и в комментариях.
    """
    def order_of(text: str) -> list[str]:
        return re.findall(r'<script src="([^"]+)"', text)

    login(client)
    main_scripts = order_of(client.get("/").get_data(as_text=True))
    assert "/slam_auth.js" in main_scripts
    assert main_scripts.index("/slam_auth.js") < main_scripts.index("main.js")

    console_scripts = order_of(client.get("/console").get_data(as_text=True))
    assert "/slam_auth.js" in console_scripts
    assert console_scripts.index("/slam_auth.js") < console_scripts.index("console-core.js")


def test_unknown_asset_is_404(client, operator):
    login(client)
    assert client.get("/backend.py").status_code == 404
    assert client.get("/UPSTREAM_COMMIT.txt").status_code == 404


# ------------------------------- заголовки ------------------------------- #
def test_csp_allows_inline_styles_for_console(client, operator):
    """В index.html есть атрибуты style="…" — CSP не должен их резать."""
    login(client)
    policy = client.get("/console").headers["Content-Security-Policy"]
    assert "style-src 'self' 'unsafe-inline'" in policy
    assert "script-src 'self'" in policy


def test_exit_link_logs_out(client, operator):
    login(client)
    assert client.get("/").status_code == 200
    assert client.get("/exit").status_code == 302
    assert client.get("/").status_code == 302  # интерфейс снова закрыт


# ------------------------ основной экран (киоск) ------------------------- #
def test_main_screen_has_no_cargo_compartment(client, operator):
    """Грузовой отсек с PIN-клавиатурой убран с главного экрана. Ячейка и /api/lock
    остались у инженерного пульта (их проверяют тесты замка выше)."""
    login(client)
    text = client.get("/").get_data(as_text=True)
    for phrase in ("Грузовой отсек", "Открыть отсек", "Введите PIN"):
        assert phrase not in text, phrase
    assert not re.search(r"\bPIN\b", text)

    js = client.get("/main.js").get_data(as_text=True)
    assert "api/lock" not in js
    assert "api/audit" not in js
    assert "cargo" not in js
    assert not re.search(r"\b(lock|pin|PIN)\b", js)


def test_main_screen_shows_four_drive_modules(client, operator):
    login(client)
    text = client.get("/").get_data(as_text=True)
    for module in ("FL", "FR", "RL", "RR"):
        assert f'id="mod-{module}"' in text, module
        assert f'id="wheel-{module}"' in text, module


def test_main_screen_has_logout_and_console_links(client, operator):
    login(client)
    text = client.get("/").get_data(as_text=True)
    assert 'href="/exit"' in text
    assert 'href="/console"' in text


def test_main_screen_side_menu_points_to_real_sections(client, operator):
    """Меню слева: два пункта — «Пульт» (наверх экрана) и «Сервис» (инженерный пульт);
    якоря, если появятся, должны вести к существующим разделам экрана."""
    login(client)
    text = client.get("/").get_data(as_text=True)
    for label in ("Пульт", "Сервис"):
        assert label in text, label
    for label in ("Ходовая", "Миссия", "Сенсоры", "Безопасность", "Журнал"):
        assert f'<span class="nav-text">{label}</span>' not in text, label
    for target in re.findall(r'href="#([^"]+)"', text):
        assert f'id="{target}"' in text, target


def test_main_screen_polls_state_with_relative_url(client, operator):
    """Относительный путь нужен, чтобы экран работал и под /main.html,
    и за прокси предпросмотра, где префикс может отличаться."""
    login(client)
    js = client.get("/main.js").get_data(as_text=True)
    assert "fetch('api/state'" in js
    assert "'/api/state'" not in js


def test_main_screen_has_no_demo_mode(client, operator):
    """Демо-режима нет: без робота экран показывает «нет данных», а не выдуманные значения."""
    login(client)
    js = client.get("/main.js").get_data(as_text=True)
    assert "get('demo')" not in js
    assert "demoData" not in js
    assert "demoLidar" not in js
    assert "drawDemoCamera" not in js


def test_main_screen_keeps_session_token_for_console(client, operator):
    """Ссылка на пульт сохраняет st, иначе в iframe-предпросмотре вход потеряется."""
    login(client)
    js = client.get("/main.js").get_data(as_text=True)
    assert "'/console?st='" in js


def _load_backend():
    """slam_gui/backend.py грузится по пути, как в robot_control/slam.py."""
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "slam_gui" / "backend.py"
    spec = importlib.util.spec_from_file_location("rus_slam_backend_test", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scan_points_skips_bad_beams_and_normalizes_angles():
    """LaserScan -> точки: битые лучи пропускаются, угол в [-pi, pi]."""
    import math
    from types import SimpleNamespace

    backend = _load_backend()
    msg = SimpleNamespace(
        angle_min=0.0,
        angle_increment=math.pi / 2,
        range_min=0.1,
        range_max=10.0,
        ranges=[2.0, float("inf"), float("nan"), 0.05, 4.0, 12.0],
    )
    pts = backend.scan_points(msg)
    assert [p["r"] for p in pts] == [2.0, 4.0]
    # луч с индексом 4: 4 * pi/2 = 2pi, нормализован к 0
    assert abs(pts[1]["a"]) < 1e-3
    assert all(-math.pi <= p["a"] <= math.pi for p in pts)


def test_ros_source_exposes_fresh_lidar_only():
    """В /api/state попадает свежий скан; старый или отсутствующий даёт None."""
    import time

    backend = _load_backend()
    src = object.__new__(backend.RosSource)
    src.ok = True
    src.fallback = None
    src.data = {}
    src.scan = [{"a": 0.0, "r": 2.5}]
    src.scan_at = time.time()
    assert src.read()["lidar"] == [{"a": 0.0, "r": 2.5}]

    src.scan_at = time.time() - 10
    assert src.read()["lidar"] is None


def test_main_screen_manual_drive_toggle_on_r(client, operator):
    """R включает режим WASD на главном экране: лидар обводится красным, команда не уходит на сервер."""
    login(client)
    js = client.get("/main.js").get_data(as_text=True)
    assert "'KeyR'" in js and "'KeyW'" in js and "'Space'" in js
    assert "fetch(" not in js.split("function driveKey", 1)[1].split("function boot", 1)[0]
    css = client.get("/main.css").get_data(as_text=True)
    assert ".lidar.is-control" in css


def test_master_line_parses_tlm_and_ignores_text():
    """Строка "@TLM ..." разбирается; обычный текст монитора мастера игнорируется."""
    backend = _load_backend()
    frame = backend.parse_master_line(
        "@TLM mod=fl deg=-9.25 tgt=0.00 moving=1 cal=1 cycle=0 calib=0 opto=0 t=12345\r\n")
    assert frame["mod"] == "FL"
    assert frame["deg"] == -9.25
    assert frame["moving"] is True and frame["cal"] is True
    assert frame["cycle"] is False and frame["opto"] is False
    assert backend.parse_master_line("Цель достигнута. Угол: 12.00°") is None
    assert backend.parse_master_line("@TLM mod=FL deg=abc") is None
    assert backend.parse_master_line("@TLM mod=DRIVE deg=0") is None
    assert backend.parse_master_line("@TLM mod=FL deg=nan") is None


def test_master_source_queues_only_safe_module_angle_commands():
    """Команда отправляется только свежему откалиброванному и остановленному модулю."""
    import queue
    import threading

    backend = _load_backend()
    src = object.__new__(backend.MasterSource)
    src.lock = threading.Lock()
    src.frames = {}
    src.drive_percent = 0
    src.drive_last_command = 0.0
    src.commands = queue.Queue(maxsize=4)
    src.serial_ready = threading.Event()
    src.serial_ready.set()
    src.feed("@TLM mod=FR deg=0 tgt=0 moving=0 cal=1 cycle=0 calib=0", now=100.0)

    result = src.steer_module("fr", 12.5, now=100.2)
    assert result == {"ok": True, "queued": True, "module": "FR", "angle": 12.5}
    assert src.commands.get_nowait() == b"w FR 12.5\n"

    assert src.steer_module("FR", 101, now=100.2)["reason"] == "angle_limit"
    assert src.steer_module("FR", 100, now=100.2)["ok"] is True   # потолок хода разрешён
    assert src.steer_module("FL", 10, now=100.2)["reason"] == "module_offline"
    src.feed("@TLM mod=RL deg=0 tgt=0 moving=0 cal=0 cycle=0 calib=0", now=100.0)
    assert src.steer_module("RL", 10, now=100.2)["reason"] == "not_homed"
    src.feed("@TLM mod=RR deg=0 tgt=0 moving=1 cal=1 cycle=0 calib=0", now=100.0)
    assert src.steer_module("RR", 10, now=100.2)["reason"] == "module_busy"
    assert src.steer_module("FR", 10, now=101.2)["reason"] == "module_offline"


def test_master_telemetry_parses_bldc_rpm_and_drive_percent():
    backend = _load_backend()
    frame = backend.parse_master_line(
        "@TLM mod=FL deg=0.00 tgt=0.00 moving=0 cal=1 cycle=0 calib=0 opto=0 rpm=245.5 drive=-12 t=42")
    assert frame["rpm"] == 245.5
    assert frame["drivePercent"] == -12
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 rpm=20 drive=21") is None


def test_master_source_drive_requires_aligned_zeroed_modules_and_prioritizes_stop():
    import queue
    import threading

    backend = _load_backend()
    src = object.__new__(backend.MasterSource)
    src.lock = threading.Lock()
    src.frames = {}
    src.drive_percent = 0
    src.drive_last_command = 0.0
    src.commands = queue.Queue(maxsize=2)
    src.serial_ready = threading.Event()
    src.serial_ready.set()

    for module in ("FL", "FR", "RL", "RR"):
        src.feed("@TLM mod=%s deg=0.0 tgt=0.0 moving=0 cal=1 rpm=0.0 drive=0" % module, now=100.0)
    status = src.drive_status(now=100.2)
    assert status["canDrive"] is True
    assert status["reason"] is None

    # Тяга — «t N»: «d» в прошивке осталась поворотом на угол (как в проверенном скетче).
    assert src.drive(12, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"t 12\n"
    assert src.drive(-8, now=100.3)["ok"] is True
    assert src.commands.get_nowait() == b"t -8\n"
    assert src.steer_module("FL", 5, now=100.3)["reason"] == "module_busy"

    # Очередь заполнена поворотами — STOP вытесняет их, а не пропадает.
    src.commands.put_nowait(b"w FL 5\n")
    src.commands.put_nowait(b"w FR 5\n")
    assert src.stop_drive(now=100.4)["ok"] is True
    assert src.commands.get_nowait() == b"b\n"
    assert src.commands.empty()

    src.feed("@TLM mod=RR deg=4.0 tgt=4.0 moving=0 cal=1 rpm=0 drive=0", now=100.5)
    assert src.drive_status(now=100.6)["reason"] == "steering_misaligned"


def test_master_line_v2_reports_zero_source_calibration_error_and_firmware():
    """Прошивка v2: откуда ноль (датчик/вручную), код ошибки калибровки, версия."""
    backend = _load_backend()
    frame = backend.parse_master_line(
        "@TLM mod=RR deg=0.00 tgt=0.00 moving=0 cal=1 cycle=0 calib=0 opto=1 home=opto "
        "err=0 rpm=0.0 drive=0 fw=2 t=5000")
    assert frame["cal"] is True and frame["home"] == "opto" and frame["manualZero"] is False
    assert frame["opto"] is True and frame["firmware"] == 2 and frame["calibError"] == 0
    failed = backend.parse_master_line(
        "@TLM mod=FR deg=-75.00 tgt=-75.00 moving=0 cal=0 cycle=0 calib=0 opto=0 home=none "
        "err=1 rpm=0.0 drive=0 fw=2 t=9000")
    assert failed["cal"] is False and failed["home"] == "none" and failed["calibError"] == 1
    manual = backend.parse_master_line("@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 home=manual fw=2")
    assert manual["manualZero"] is True and manual["home"] == "manual"
    # Старая строка без home= — ноль мог быть только ручным.
    legacy = backend.parse_master_line("@TLM mod=FL deg=0 tgt=0 moving=0 cal=1")
    assert legacy["home"] == "manual" and legacy["firmware"] is None
    assert backend.parse_master_line("@TLM mod=FL deg=0 home=elsewhere") is None
    assert backend.parse_master_line("@TLM mod=FL deg=0 err=99") is None


def _bare_master_source(backend, maxsize=4):
    import queue
    import threading

    src = object.__new__(backend.MasterSource)
    src.lock = threading.Lock()
    src.frames = {}
    src.drive_percent = 0
    src.drive_last_command = 0.0
    src.commands = queue.Queue(maxsize=maxsize)
    src.serial_ready = threading.Event()
    src.serial_ready.set()
    src.messages = []
    src.on_message = None
    return src


def test_master_source_calibration_command_is_guarded():
    """Калибровка по датчику: один модуль «w FR calib», все — «k»; не при тяге и не на ходу."""
    backend = _load_backend()
    src = _bare_master_source(backend)
    line = "@TLM mod=%s deg=0 tgt=0 moving=0 cal=0 cycle=0 calib=0 opto=0 home=none err=0 rpm=0 drive=0 fw=2"
    src.feed(line % "FR", now=100.0)
    assert src.calibrate_module("fr", now=100.2) == {"ok": True, "queued": True, "module": "FR",
                                                     "offline": []}
    assert src.commands.get_nowait() == b"w FR calib\n"
    assert src.calibrate_module("FL", now=100.2)["reason"] == "module_offline"
    assert src.calibrate_module("XX", now=100.2)["reason"] == "invalid_module"
    result = src.calibrate_module("all", now=100.2)
    assert result["ok"] is True and result["offline"] == ["FL", "RL", "RR"]
    assert src.commands.get_nowait() == b"k\n"
    src.feed(line.replace("calib=0", "calib=1") % "FR", now=100.3)
    assert src.calibrate_module("FR", now=100.4)["reason"] == "module_busy"
    src.feed(line % "FR", now=100.5)
    src.drive_percent = 8
    assert src.calibrate_module("FR", now=100.6)["reason"] == "module_busy"
    src.drive_percent = 0
    src.serial_ready.clear()
    assert src.calibrate_module("FR", now=100.6)["reason"] == "serial_unavailable"


def test_master_source_keeps_firmware_messages_for_the_journal():
    backend = _load_backend()
    src = _bare_master_source(backend)
    seen = []
    src.on_message = seen.append
    assert src.feed("# FR: Калибровка ЗАВЕРШЕНА! Точка 0 установлена по датчику.\r\n", now=5.0) is False
    assert src.feed("просто текст", now=5.1) is False
    assert seen == ["FR: Калибровка ЗАВЕРШЕНА! Точка 0 установлена по датчику."]
    assert src.recent_messages() == [{"ts": 5.0, "text": seen[0]}]
    for index in range(60):
        src.feed("# FL: строка %d" % index, now=6.0 + index)
    assert len(src.messages) == backend.MasterSource.MESSAGE_LIMIT
    assert src.recent_messages(2)[-1]["text"] == "FL: строка 59"


def test_steering_control_info_reports_opto_calibration_for_master():
    from robot_control.slam import steering_control_info

    backend = _load_backend()
    info = steering_control_info(_bare_master_source(backend))
    assert info["optoUsed"] is True and info["calibration"] is True and info["zeroMode"] == "opto"
    assert info["manualZero"] is True
    peer = steering_control_info(backend.MasterPeerSource.__new__(backend.MasterPeerSource))
    assert peer["available"] is False and peer["calibration"] is False


def test_master_source_reports_each_module_and_empty_for_silent_ones():
    """Мастер передаёт FL и узлы FR/RL/RR; модуль, который молчит, — пустой, без выдумок."""
    backend = _load_backend()
    src = backend.MasterSource("/dev/null")                 # порт не открывается в тесте
    src.feed("@TLM mod=FL deg=30.00 tgt=30.00 moving=0 cal=1 cycle=0 calib=0 opto=0 t=1", now=100.0)
    src.feed("@TLM mod=FR deg=-5.50 tgt=0.00 moving=1 cal=0 cycle=0 calib=1 opto=0 t=2", now=100.0)
    out = src.read(now=100.5)
    fl = next(m for m in out["motors"] if m["id"] == "FL")
    fr = next(m for m in out["motors"] if m["id"] == "FR")
    rl = next(m for m in out["motors"] if m["id"] == "RL")
    assert fl["angle"] == 30.0 and fl["online"] is True and fl["homed"] is True
    assert fl["rpm"] is None and fl["temp"] is None
    assert fr["angle"] == -5.5 and fr["moving"] is True and fr["homed"] is False
    assert rl["online"] is False and rl["angle"] is None
    assert out["linkOk"] is True and out["mode"] == "МАСТЕР НА СВЯЗИ"
    assert out["battery"]["volts"] is None and out["speedMps"] is None
    stale = src.read(now=102.0)                             # старше STALE_S — связи нет
    assert stale["linkOk"] is False
    assert all(m["online"] is False for m in stale["motors"])


def _source_with_runner(backend):
    """Источник с настоящим раннером манёвров, но без портов."""
    src = _bare_master_source(backend)
    core = backend._load_maneuvers()
    src.maneuver_core = core
    src.maneuvers = core.ManeuverRunner(brake=src._brake_all)
    return src


def test_maneuvers_need_a_runner_and_fresh_wheels():
    """Без ядра манёвров функция честно «не поддерживается», а не падает."""
    backend = _load_backend()
    bare = _bare_master_source(backend)
    assert bare.maneuver_status()["supported"] is False
    assert bare.start_maneuver("forward") == {"ok": False,
                                              "reason": "control_not_supported"}

    src = _source_with_runner(backend)
    assert src.maneuver_status()["supported"] is True
    src.serial_ready.clear()
    assert src.start_maneuver("forward") == {"ok": False, "reason": "serial_unavailable"}
    src.serial_ready.set()
    # Ни одного кадра — колёс нет: манёвр не запускается «вслепую».
    refused = src.start_maneuver("forward")
    assert refused["ok"] is False and refused["reason"] == "module_offline"
    assert src.maneuver_status()["active"] is False


def test_manual_drive_and_stop_cancel_the_maneuver():
    """Ручная тяга и СТОП отменяют манёвр: оператор всегда главнее пресета."""
    import time as _time

    backend = _load_backend()
    src = _source_with_runner(backend)
    assert src.maneuvers.start("forward", _time.monotonic(), limit_percent=20)["ok"] is True
    assert src.maneuver_status()["active"] is True
    assert src.stop_maneuver()["ok"] is True
    assert src.maneuver_status()["state"] == "cancelled"

    assert src.maneuvers.start("spin_left", _time.monotonic(), limit_percent=20)["ok"] is True
    src.drive(0)                                   # ручной СТОП
    assert src.maneuver_status()["state"] == "cancelled"


def test_maneuver_presets_are_editable_and_validated():
    backend = _load_backend()
    src = _source_with_runner(backend)
    config = src.maneuver_config()
    assert config["supported"] is True
    assert config["presets"]["forward"]["distanceM"] == 1.0
    assert "distanceM" in config["limits"]
    assert src.set_maneuver_config("forward", {"distanceM": 2.5})["ok"] is True
    assert src.maneuver_config()["presets"]["forward"]["distanceM"] == 2.5
    assert src.set_maneuver_config("forward", {"distanceM": 0}) == {
        "ok": False, "reason": "invalid_value"}
    assert src.set_maneuver_config("телепорт", {"distanceM": 1}) == {
        "ok": False, "reason": "invalid_maneuver"}


def test_pot_mode_requires_all_boards_and_idle_telemetry():
    backend = _load_backend()
    src = _bare_master_source(backend)
    line = '@TLM mod=%s deg=0 tgt=0 moving=0 cal=1 cycle=0 calib=0 drive=0 fw=2 pm=1'
    for mid in ('FL', 'FR', 'RL'):
        src.feed(line % mid, now=100)
    assert src.set_pot_mode('off', now=100.2)['reason'] == 'module_offline'
    src.feed((line % 'RR').replace(' pm=1', ''), now=100)
    assert src.set_pot_mode('off', now=100.2)['reason'] == 'firmware_update_required'
    src.feed(line % 'RR', now=100)
    assert src.set_pot_mode('off', now=100.2)['ok']
    assert src.commands.get_nowait() == b'pm 0\n'
    assert src.set_pot_mode('on', now=100.2)['ok']
    assert src.commands.get_nowait() == b'pm 1\n'
    assert src.set_pot_mode('calibrate', now=100.2)['ok']
    assert src.commands.get_nowait() == b'q\n'
    src.feed((line % 'FR').replace('pm=1', 'pm=0'), now=100)
    assert src.set_pot_mode('calibrate', now=100.2)['reason'] == 'zero_required'
    src.feed((line % 'FR').replace('moving=0', 'moving=1'), now=100)
    assert src.set_pot_mode('off', now=100.2)['reason'] == 'module_busy'
    assert src.set_pot_mode('off', now=102)['reason'] == 'module_offline'
    src.serial_ready.clear()
    assert src.set_pot_mode('off', now=100.2)['reason'] == 'serial_unavailable'


# --------------------------- гибкая настройка тяги --------------------------- #
def _drive_frame(module, *, tach=1, dmax=20, trim=1500, span=500, us=1500, stp=1,
                 brk=1, dr=0, deg=0.0, drive=0, ppr=1, rpm=0.0):
    return ("@TLM mod=%s deg=%.2f tgt=0.00 moving=0 cal=1 cycle=0 calib=0 rpm=%.1f "
            "drive=%d fw=6 pot=1 pm=1 tach=%d dmax=%d trim=%d span=%d ppr=%d brk=%d stp=%d dir=%d us=%d"
            % (module, deg, rpm, drive, tach, dmax, trim, span, ppr, brk, stp, dr, us))


def test_master_telemetry_parses_flexible_drive_configuration():
    """Прошивка с командой m: тахометр, предел, нейтраль/размах серво-импульса."""
    backend = _load_backend()
    frame = backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 rpm=0 drive=8 fw=6 tach=0 "
        "dmax=40 trim=1620 span=300 brk=0 stp=1 dir=1 us=1770 t=1")
    assert frame["tach"] is False
    assert frame["driveMaxPercent"] == 40
    assert (frame["driveTrimUs"], frame["driveSpanUs"]) == (1620, 300)
    assert frame["drivePulseUs"] == 1770
    assert frame["brake"] is False and frame["stopInput"] is True
    assert frame["dirReverse"] is True
    # Импульсов тахометра на оборот: без него «об/мин» врут ровно в ppr раз.
    assert frame["tachPpr"] is None
    with_ppr = backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 rpm=60.0 drive=8 fw=7 tach=1 "
        "dmax=40 trim=1500 span=500 ppr=60 brk=1 stp=1 dir=0 us=1500 t=1")
    assert with_ppr["tachPpr"] == 60 and with_ppr["rpm"] == 60.0
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 ppr=0") is None
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 ppr=201") is None
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 ppr=abc") is None

    # Прошивка без команды m: поля отсутствуют, а не выдуманы нулями.
    legacy = backend.parse_master_line("@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 rpm=0 drive=0 fw=4")
    assert legacy["tach"] is None and legacy["driveTrimUs"] is None
    assert legacy["driveMaxPercent"] is None and legacy["drivePulseUs"] is None

    # Мусор и противоречия не превращаются в «настройку по умолчанию».
    assert backend.parse_master_line("@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 tach=7") is None
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 trim=900 span=100") is None
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 trim=1620 span=480") is None  # 2100 мкс
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 trim=1500 span=600") is None  # шире стандарта
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 us=2500") is None
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 dmax=101") is None
    # Тяга не может превышать собственный предел платы.
    assert backend.parse_master_line("@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=30 dmax=20") is None
    assert backend.parse_master_line("@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=30 dmax=50") is not None


def test_drive_limit_follows_the_plates_and_server_ceiling():
    """Предел тяги берётся из телеметрии плат, но не выше потолка сервера."""
    backend = _load_backend()
    src = _bare_master_source(backend)
    for module in ("FL", "FR", "RL", "RR"):
        src.feed(_drive_frame(module), now=100.0)
    assert src.drive_limit_percent(now=100.2) == 20
    assert src.drive_status(now=100.2)["maxPercent"] == 20

    for module in ("FL", "FR", "RL", "RR"):
        src.feed(_drive_frame(module, dmax=50), now=100.0)
    assert src.drive_limit_percent(now=100.2) == 50
    assert src.drive(30, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"t 30\n"

    # Одна плата осторожнее — общий предел по ней: сервер не обещает больше.
    src.feed(_drive_frame("RR", dmax=10), now=100.0)
    assert src.drive_limit_percent(now=100.2) == 10
    assert src.drive(30, now=100.3)["reason"] == "drive_limit"
    assert src.drive(30, now=100.3)["limit"] == 10


def test_drive_config_is_mixed_when_the_plates_disagree():
    backend = _load_backend()
    src = _bare_master_source(backend)
    for module in ("FL", "FR", "RL"):
        src.feed(_drive_frame(module), now=100.0)
    src.feed(_drive_frame("RR", dmax=40, trim=1520, span=300, ppr=60), now=100.0)
    config = src.drive_config(now=100.2)
    assert config["supported"] is True and config["mixed"] is True
    assert config["maxPercent"] is None and config["trimUs"] is None
    assert config["perModule"]["RR"]["trimUs"] == 1520
    assert config["perModule"]["RR"]["spanUs"] == 300
    # Импульсы на оборот — свойство конкретной платы, поэтому в perModule оно
    # есть всегда, а общее значение пустое, когда платы разошлись.
    assert config["perModule"]["RR"]["pulsesPerRev"] == 60
    assert config["perModule"]["FL"]["pulsesPerRev"] == 1
    assert config["pulsesPerRev"] is None
    assert config["outputMode"] == "servo"
    # Смешанный режим — не повод разрешить тягу: общий предел неизвестен.
    assert src.drive_limit_percent(now=100.2) == backend.MAX_DRIVE_PERCENT


def test_individual_drive_targets_only_its_own_wheel():
    """Тяга одного колеса: «t FL 8» на своё колесо, без выравнивания всех."""
    backend = _load_backend()
    src = _bare_master_source(backend)
    src.drive_modules = {}
    src.feed(_drive_frame("FL"), now=100.0)
    src.feed(_drive_frame("FR", deg=9.0), now=100.0)      # колесо смотрит в сторону
    src.feed(_drive_frame("RL"), now=100.0)
    src.feed(_drive_frame("RR"), now=100.0)

    status = src.drive_status(now=100.2)
    # Общая тяга запрещена (колёса не выровнены), одиночная — нет: именно так
    # крутят одно колесо на стенде и делают разворот на месте.
    assert status["canDrive"] is False and status["reason"] == "steering_misaligned"
    assert status["individual"] is True
    assert status["perModule"]["FL"]["canDrive"] is True
    assert set(status["perModule"]) == {"FL", "FR", "RL", "RR"}

    result = src.drive_module("fl", 8, now=100.2)
    assert result == {"ok": True, "queued": True, "module": "FL", "percent": 8,
                      "maxPercent": 20}
    assert src.commands.get_nowait() == b"t FL 8\n"
    assert src.drive_status(now=100.2)["commandedModules"] == {"FL": 8}

    # Разные колёса — разные значения и свои команды.
    assert src.drive_module("FR", -6, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"t FR -6\n"
    assert src.drive_status(now=100.2)["commandedModules"] == {"FL": 8, "FR": -6}
    assert src.drive_module("FL", 0, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"t FL 0\n"
    assert src.drive_status(now=100.2)["commandedModules"] == {"FL": 0, "FR": -6}

    # Общая команда адресована всем: одиночные значения больше не значат ничего.
    src.feed(_drive_frame("FR", deg=0.0), now=100.0)
    assert src.drive(5, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"t 5\n"
    assert src.drive_status(now=100.2)["commandedModules"] == {}


def test_individual_drive_guards():
    backend = _load_backend()
    src = _bare_master_source(backend)
    src.drive_modules = {}
    for module in ("FL", "FR", "RL", "RR"):
        src.feed(_drive_frame(module), now=100.0)

    assert src.drive_module("XX", 8, now=100.2)["reason"] == "invalid_module"
    assert src.drive_module("FL", "abc", now=100.2)["reason"] == "invalid_drive"
    assert src.drive_module("FL", 21, now=100.2)["reason"] == "drive_limit"
    assert src.drive_module("FL", 21, now=100.2)["limit"] == 20
    # Своё колесо должно быть на связи, с нулём и стоять.
    src.feed(_drive_frame("RL", deg=0.0).replace("cal=1", "cal=0"), now=100.0)
    assert src.drive_module("RL", 8, now=100.2)["reason"] == "not_homed"
    src.feed(_drive_frame("RL", deg=0.0), now=100.0)
    assert src.drive_module("RL", 8, now=103.0)["reason"] == "module_offline"
    src.feed(_drive_frame("RL", deg=0.0), now=100.0)
    src.frames["RL"] = (dict(src.frames["RL"][0], moving=True), 100.0)
    assert src.drive_module("RL", 8, now=100.2)["reason"] == "module_busy"
    assert src.commands.empty()

    # Остановка колеса разрешена и «не на связи»: 0 обязан проходить всегда.
    src.serial_ready.clear()
    assert src.drive_module("RL", 0, now=100.2)["reason"] == "serial_unavailable"
    src.serial_ready.set()
    assert src.drive_module("RL", 0, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"t RL 0\n"

    # Старая прошивка без поля drive= одиночную тягу не получает.
    legacy = _bare_master_source(backend)
    legacy.drive_modules = {}
    for module in ("FL", "FR", "RL", "RR"):
        legacy.feed("@TLM mod=%s deg=0 tgt=0 moving=0 cal=1 rpm=0 fw=4" % module, now=100.0)
    assert legacy.drive_module("FL", 8, now=100.2)["reason"] == "drive_firmware_required"

    # Очередь занята — понятный отказ, а не тишина.
    full = _bare_master_source(backend, maxsize=1)
    full.drive_modules = {}
    for module in ("FL", "FR", "RL", "RR"):
        full.feed(_drive_frame(module), now=100.0)
    full.commands.put_nowait(b"w 1\n")
    assert full.drive_module("FL", 8, now=100.2)["reason"] == "command_queue_full"


def test_module_direction_is_per_wheel_and_guarded():
    """Сторона вращения — настройка одного колеса, а не всех четырёх.

    Колёса стоят зеркально, поэтому «вперёд» у левого и правого борта — разные
    DIR. Сервер должен слать «m FL dir 1» только на порт этого колеса, не
    трогать остальные и не менять направление на ходу.
    """
    backend = _load_backend()
    src = _bare_master_source(backend)
    src.drive_modules = {}
    for module in ("FL", "FR", "RL", "RR"):
        src.feed(_drive_frame(module), now=100.0)

    assert src.set_drive_config("direction", "reverse", "FL", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m FL dir 1\n"
    assert src.set_drive_config("direction", "normal", "FL", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m FL dir 0\n"
    answer = src.set_drive_config("direction", True, "FR", now=100.2)
    assert answer["ok"] is True and answer["reverse"] is True and answer["module"] == "FR"
    assert src.commands.get_nowait() == b"m FR dir 1\n"

    # Некорректные значения и имена отклоняются, а не «примерно применяются».
    assert src.set_drive_config("direction", "maybe", "FL", now=100.2)["reason"] == "invalid_value"
    assert src.set_drive_config("direction", "reverse", None, now=100.2)["reason"] == "invalid_module"
    assert src.set_drive_config("direction", "reverse", "XX", now=100.2)["reason"] == "invalid_module"
    assert src.commands.empty()

    # Направление меняют у стоящего колеса: реверс DIR под тягой — рывок.
    assert src.drive_module("FL", 8, now=100.2)["ok"] is True
    src.commands.get_nowait()
    assert src.set_drive_config("direction", "reverse", "FL", now=100.2)["reason"] == "module_busy"
    # ... а соседнее колесо, которое стоит, — можно: у него своя плата.
    assert src.set_drive_config("direction", "reverse", "FR", now=100.2)["ok"] is True
    src.commands.get_nowait()
    assert src.drive_module("FL", 0, now=100.2)["ok"] is True
    src.commands.get_nowait()

    # Нет свежей телеметрии этого колеса — понятный отказ.
    assert src.set_drive_config("direction", "reverse", "RL", now=103.5)["reason"] == "module_offline"
    src.serial_ready.clear()
    assert src.set_drive_config("direction", "reverse", "FL", now=100.2)["reason"] == "serial_unavailable"
    src.serial_ready.set()

    # Старая прошивка (без команды m) направление не умеет.
    legacy = _bare_master_source(backend)
    legacy.drive_modules = {}
    for module in ("FL", "FR", "RL", "RR"):
        legacy.feed("@TLM mod=%s deg=0 tgt=0 moving=0 cal=1 rpm=0 fw=4" % module, now=100.0)
    assert legacy.set_drive_config("direction", "reverse", "FL",
                                   now=100.2)["reason"] == "firmware_update_required"

    # Очередь занята — отказ, а не потерянная настройка.
    full = _bare_master_source(backend, maxsize=1)
    full.drive_modules = {}
    for module in ("FL", "FR", "RL", "RR"):
        full.feed(_drive_frame(module), now=100.0)
    full.commands.put_nowait(b"w 1\n")
    assert full.set_drive_config("direction", "reverse", "FL",
                                 now=100.2)["reason"] == "command_queue_full"


def test_drive_status_reports_direction_of_each_wheel():
    """Экран рисует «обычное/зеркальное» по телеметрии, а не по своей памяти."""
    backend = _load_backend()
    src = _bare_master_source(backend)
    src.drive_modules = {}
    for module in ("FL", "FR", "RL", "RR"):
        src.feed(_drive_frame(module, dr=1 if module in ("FL", "RL") else 0), now=100.0)

    status = src.drive_status(now=100.2)
    assert status["perModule"]["FL"]["dirReverse"] is True
    assert status["perModule"]["FR"]["dirReverse"] is False
    assert all(item["canSetDirection"] for item in status["perModule"].values())
    assert status["config"]["perModule"]["RL"]["dirReverse"] is True

    # Пока колесо едет, его кнопка направления объясняет, почему нельзя.
    assert src.drive_module("FR", 8, now=100.2)["ok"] is True
    item = src.drive_status(now=100.2)["perModule"]["FR"]
    assert item["canSetDirection"] is False and item["directionReason"] == "module_busy"
    assert src.drive(0, now=100.2)["ok"] is True

    # Колёса не сошлись в направлении — это норма (зеркальная установка),
    # поэтому общая настройка не должна считаться «различается» из-за dir.
    assert src.set_drive_config("direction", "normal", "FL", now=100.2)["ok"] is True
    src.commands.get_nowait()


def test_journal_names_the_wheel_that_was_driven():
    """В журнале видно колесо: FL вперёд и FR назад — две разные записи, не «0»."""
    from robot_control import eventlog

    eventlog.reset()
    try:
        slam.note_drive_command(8, "FL")
        slam.note_drive_command(-8, "FR")
        slam.note_drive_command(0, "FL")
        slam.note_drive_command(0)
        messages = [item["msg"] for item in eventlog.get_event_log().entries()]
        assert "тяга колесо FL +8% (вперёд)" in messages
        assert "тяга колесо FR -8% (назад)" in messages
        assert "тяга колесо FL: 0% — тормоз" in messages
        assert "тяга BLDC: 0% — тормоз" in messages
        # Heartbeat того же колеса в журнал не попадает.
        slam.note_drive_command(8, "FL")
        assert len(eventlog.get_event_log().entries()) == 4
    finally:
        eventlog.reset()


def test_steering_stays_allowed_for_a_stopped_wheel_next_to_a_driving_one():
    backend = _load_backend()
    src = _bare_master_source(backend)
    src.drive_modules = {}
    for module in ("FL", "FR", "RL", "RR"):
        src.feed(_drive_frame(module), now=100.0)
    assert src.drive_module("FL", 8, now=100.2)["ok"] is True
    src.commands.get_nowait()
    assert src.steer_module("FL", 10, now=100.2)["reason"] == "module_busy"
    assert src.steer_module("FR", 10, now=100.2)["ok"] is True
    # Калибровка и настройка привода — только когда не едет ни одно колесо.
    assert src.set_drive_config("limit", 30, now=100.2)["reason"] == "module_busy"
    assert src.calibrate_module("FR", now=100.2)["reason"] == "module_busy"
    # Общая команда «стоп» гасит и одиночную тягу.
    src.feed(_drive_frame("FR", deg=0.0), now=100.0)
    assert src.drive(0, now=100.2)["ok"] is True
    assert src.drive_status(now=100.2)["commandedModules"] == {}
    assert src.steer_module("FL", 10, now=100.2)["ok"] is True


def test_dispatch_individual_drive_payload_and_statuses():
    class Source:
        supports_drive = True
        supports_individual_drive = True

        def __init__(self, result=None):
            self.result = result if result is not None else {"ok": True, "queued": True, "percent": 8,
                                                             "module": "FL", "maxPercent": 20}
            self.calls = []

        def drive(self, percent):
            self.calls.append(("all", percent))
            return self.result

        def drive_module(self, module, percent):
            self.calls.append((module, percent))
            return self.result

    source = Source()
    result, status = slam.dispatch_drive_command(source, {"module": "fl", "percent": 8})
    assert status == 200 and result["ok"] is True
    assert result["module"] == "FL" and result["percent"] == 8
    assert source.calls == [("FL", 8)]
    # Без модуля — прежняя общая команда.
    result, status = slam.dispatch_drive_command(source, {"percent": -4})
    assert status == 200 and source.calls[-1] == ("all", -4)

    for body, code, reason in (
            ({"module": "XX", "percent": 8}, 400, "invalid_module"),
            ({"module": "FL"}, 400, "payload"),
            ({"module": "FL", "percent": 8, "angle": 0}, 400, "payload"),
            ({"percent": 8, "module": None}, 400, "invalid_module")):
        result, status = slam.dispatch_drive_command(Source(), body)
        assert (status, result["reason"]) == (code, reason), body

    # Источник без адресной тяги честно отказывает, а не шлёт команду всем.
    class AllOnly(Source):
        supports_individual_drive = False

    all_only = AllOnly()
    result, status = slam.dispatch_drive_command(all_only, {"module": "FL", "percent": 8})
    assert (status, result["reason"]) == (409, "control_not_supported")
    assert all_only.calls == []

    # Отказы платы → HTTP-коды.
    for reason, code in (("module_offline", 503), ("serial_unavailable", 503),
                         ("command_queue_full", 429), ("invalid_module", 400),
                         ("invalid_drive", 400), ("drive_limit", 400),
                         ("module_busy", 409), ("not_homed", 409)):
        result, status = slam.dispatch_drive_command(
            Source({"ok": False, "reason": reason}), {"module": "FL", "percent": 8})
        assert result["reason"] == reason and status == code, reason
    result, status = slam.dispatch_drive_command(
        Source({"ok": False, "reason": "drive_limit", "limit": 12}), {"module": "FL", "percent": 8})
    assert result["limit"] == 12


def test_set_drive_config_commands_and_guards():
    backend = _load_backend()
    src = _bare_master_source(backend, maxsize=8)
    for module in ("FL", "FR", "RL", "RR"):
        src.feed(_drive_frame(module), now=100.0)

    assert src.set_drive_config("tach", "off", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m tach 0\n"
    assert src.set_drive_config("tach", "on", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m tach 1\n"
    assert src.set_drive_config("brake", "off", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m brake 0\n"
    assert src.set_drive_config("stop", "off", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m stop 0\n"
    assert src.set_drive_config("stop", "on", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m stop 1\n"
    assert src.set_drive_config("limit", 45, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m max 45\n"
    # Нейтраль и размах «серво»-импульса: края 1000…2000 мкс не переступаем.
    assert src.set_drive_config("span", 300, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m span 300\n"
    assert src.set_drive_config("trim", 1620, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m trim 1620\n"
    # Импульсов на оборот — отдельная настройка: у драйверов бывает и 1, и 60.
    assert src.set_drive_config("ppr", 60, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m ppr 60\n"
    assert src.set_drive_config("defaults", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m reset\n"

    # Недопустимые значения не уходят на платы.
    for action, value in (("stop", "yes"), ("tach", "yes"), ("limit", 0),
                          ("limit", 101), ("limit", "abc"), ("trim", 900), ("trim", 2100),
                          ("span", 0), ("span", 600), ("span", "abc"),
                          ("ppr", 0), ("ppr", 201), ("ppr", "abc"), ("ppr", None),
                          ("unknown", None)):
        result = src.set_drive_config(action, value, now=100.2)
        expected = "invalid_action" if action == "unknown" else "invalid_value"
        assert result["ok"] is False and result["reason"] == expected, (action, value)
    assert src.commands.empty()

    # Настройка — только при выключенной тяге и стоящих колёсах.
    src.drive_percent = 8
    assert src.set_drive_config("limit", 30, now=100.2)["reason"] == "module_busy"
    # Импульсы на оборот — исключение: они не меняют выходной импульс, а
    # подбирать их надо именно на крутящемся колесе.
    assert src.set_drive_config("ppr", 60, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"m ppr 60\n"
    src.drive_percent = 0
    src.feed(_drive_frame("FR", deg=4.0), now=100.0)
    src.frames["FR"] = (dict(src.frames["FR"][0], moving=True), 100.0)
    assert src.set_drive_config("limit", 30, now=100.2)["reason"] == "module_busy"

    src.feed(_drive_frame("FR"), now=100.0)
    src.serial_ready.clear()
    assert src.set_drive_config("limit", 30, now=100.2)["reason"] == "serial_unavailable"
    src.serial_ready.set()
    assert src.set_drive_config("limit", 30, now=100.3)["ok"] is True

    # Старая прошивка (без dm/tach/dmax) настройку не принимает.
    legacy = _bare_master_source(backend)
    for module in ("FL", "FR", "RL", "RR"):
        legacy.feed("@TLM mod=%s deg=0 tgt=0 moving=0 cal=1 rpm=0 drive=0 fw=4" % module, now=100.0)
    assert legacy.set_drive_config("limit", 30, now=100.2)["reason"] == "firmware_update_required"


def test_dispatch_drive_config_maps_reasons_to_http_codes():
    class Source:
        def __init__(self, result):
            self.result = result
            self.calls = []

        def set_drive_config(self, action, value=None, module=None):
            self.calls.append((action, value, module))
            return self.result

    for reason, code in (("module_offline", 503), ("serial_unavailable", 503),
                         ("command_queue_full", 429), ("invalid_value", 400),
                         ("invalid_action", 400), ("payload", 400), ("module_busy", 409),
                         ("firmware_update_required", 409)):
        source = Source({"ok": False, "reason": reason})
        result, status = slam.dispatch_drive_config(source, {"action": "limit", "value": 30})
        assert result["reason"] == reason and status == code, reason

    source = Source({"ok": True, "queued": True})
    result, status = slam.dispatch_drive_config(source, {"action": "limit", "value": 30})
    assert result["ok"] is True and status == 200
    assert source.calls == [("limit", 30, None)]

    # Сторона вращения одного колеса: имя уходит в источник вместе с действием.
    source = Source({"ok": True, "queued": True})
    result, status = slam.dispatch_drive_config(
        source, {"action": "direction", "module": "fl", "value": "reverse"})
    assert result["ok"] is True and status == 200
    assert source.calls == [("direction", "reverse", "FL")]

    for body in ({"action": "direction", "module": "FX", "value": "reverse"},
                 {"action": "direction", "module": 4, "value": "reverse"}):
        result, status = slam.dispatch_drive_config(Source({}), body)
        assert status == 400 and result["reason"] == "invalid_module", body
    # Имя колеса имеет смысл только для адресной настройки.
    result, status = slam.dispatch_drive_config(Source({}), {"action": "limit", "module": "FL", "value": 30})
    assert status == 400 and result["reason"] == "payload"
    result, status = slam.dispatch_drive_config(Source({"ok": False, "reason": "invalid_module"}),
                                               {"action": "direction", "module": "FL", "value": "reverse"})
    assert status == 400 and result["reason"] == "invalid_module"

    result, status = slam.dispatch_drive_config(Source({}), {"action": "limit", "oops": 1})
    assert status == 400 and result["reason"] == "payload"
    result, status = slam.dispatch_drive_config(Source({}), {"action": "limit", "value": 30})
    assert status == 409 and result["reason"] == "command_failed"
    result, status = slam.dispatch_drive_config(object(), {"action": "limit", "value": 30})
    assert status == 409 and result["reason"] == "control_not_supported"
