"""Экран «Модуль датчиков» бортового экрана.

Страница ``/sensors`` — такая же, как «Карта» и «Устройства»: отдельный адрес,
свой скрипт и стили, ссылка с бортового экрана и с инженерного пульта. Здесь
закреплено то, что действительно ломается:

* страница открывается без подключённой платы — данные подтягивает скрипт;
* кадр телеметрии заполняет полосу состояния, компас, барометр, свет, дверь,
  карты и журнал платы;
* без ленты светодиодов габариты честно помечены как недоступные, а не
  выглядят рабочими;
* свежий ACK подтверждает USB-связь отдельно от наличия телеметрии;
* открытие замка и правка карт не уходят на сервер без PIN кейса;
* ошибка сервера видна оператору, а не остаётся пустым экраном.
"""

from __future__ import annotations

import io
import json
import shutil
import subprocess
from pathlib import Path
from urllib.parse import urlsplit

import pytest

from conftest import login

ROOT = Path(__file__).resolve().parent.parent
HARNESS = Path(__file__).resolve().parent / "js" / "sensors_screen.js"
NODE = shutil.which("node")

pytestmark = pytest.mark.skipif(
    NODE is None, reason="нет node: экранные скрипты проверяются только с ним")


def snapshot(**kwargs) -> dict:
    """Ответ ``GET /api/sensor_module`` в том виде, в каком его отдаёт сервер."""
    data = {
        "ok": True,
        "configured": True,
        "port": "/dev/ttyUSB0",
        "protocol": 5,
        "doorClosed": True,
        "ageMs": 420,
        "stale": False,
        "error": None,
        "heading": 123.4,
        "pitch": 1.2,
        "roll": -0.8,
        "gyroZ": 0.3,
        "pressurePa": 100420.0,
        "pressureHpa": 1004.2,
        "temperatureC": 23.5,
        "altitudeM": 77,
        "lockOpen": False,
        "lightMask": 0,
        "lights": {"head": False, "stop": False, "left": False, "right": False, "dim": False},
        "dim": 80,
        "stripEnabled": False,
        "stripLeds": None,
        "rfid": "ready",
        "rfidOn": True,
        "cards": 3,
        "cardList": [{"idx": 1, "uid": "A1B2C3D4", "role": "admin"},
                     {"idx": 2, "uid": "01020304", "role": "user"}],
        "cardListKnown": True,
        "windowKind": "none",
        "windowActive": False,
        "lastCard": None,
        "linkOk": True,
        "errors": [],
        "events": [{"kind": "note", "text": "SENSORS: карта записана"}],
    }
    data.update(kwargs)
    return {"ok": True, "data": data}


def screen(**kwargs) -> dict:
    """Payload для harness: ответ API целиком (``{"ok": true, "data": {...}}``)."""
    return {"snapshot": snapshot(**kwargs)}


def run_screen(payload: dict, tmp_path: Path) -> dict:
    path = tmp_path / "payload.json"
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    proc = subprocess.run([NODE, str(HARNESS), str(path)], capture_output=True, text=True,
                          timeout=120)
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout)


# --------------------------------------------------------------- страница
def test_sensors_screen_is_reachable_like_the_other_screens(kiosk_client, client, operator):
    login(client)
    for web in (kiosk_client, client):
        page = web.get("/sensors")
        assert page.status_code == 200
        html = page.text
        assert "sensors.js" in html and "sensors.css" in html
        assert 'href="/map"' in html and 'href="/devices"' in html
        assert 'aria-current="page"' in html
        # Статика страницы доступна через оба сервера.
        assert web.get("/sensors.js").status_code == 200
        assert web.get("/sensors.css").status_code == 200
    # Ссылка есть на бортовом экране и на инженерном пульте.
    assert "k-sensors-open" in kiosk_client.get("/kiosk").text
    main_page = (ROOT / "slam_gui" / "main.html").read_text(encoding="utf-8")
    assert 'href="/sensors" id="nav-sensors"' in main_page
    console = (ROOT / "slam_gui" / "index.html").read_text(encoding="utf-8")
    assert 'id="console-sensors-link"' in console and 'href="/sensors"' in console


def test_remote_sensor_api_proxies_calibration_with_auth_and_csrf(client, operator, kiosk_client,
                                                                   monkeypatch):
    from robot_control import map_peer

    class FakeModule:
        def __init__(self):
            self.actions = []

        def state(self):
            return {"configured": True, "ok": True, "connected": True,
                    "port": "/dev/ttyUSB0", "heading": 12.3, "events": []}

        def mag_calibration(self, action):
            self.actions.append(action)

    module = FakeModule()
    kiosk_client.application.extensions["rc_sensor_module"] = module
    monkeypatch.setenv("RC_SLAM_PEER_URL", "http://owner.example:8090/api/kiosk_state")
    forwarded = []

    def forward(req, timeout):
        parsed = urlsplit(req.full_url)
        forwarded.append((req.full_url, req.get_method(), dict(req.header_items())))
        response = kiosk_client.open(parsed.path, method=req.get_method(), data=req.data,
                                     headers=dict(req.header_items()))
        result = io.BytesIO(response.data)
        result.code = response.status_code
        result.headers = response.headers
        return result

    monkeypatch.setattr(map_peer, "urlopen", forward)
    assert client.get("/sensors").status_code in (302, 401)
    assert client.post("/api/sensor_module/mag", json={"action": "start"}).status_code in (302, 401)
    login(client)
    status = client.get("/api/sensor_module")
    assert status.status_code == 200 and status.json["data"]["heading"] == 12.3
    assert client.post("/api/sensor_module/mag", json={"action": "start"},
                       headers={"X-Robot-Control": "sensors"}).status_code == 400
    token = client.get("/api/control/session").json["csrfToken"]
    headers = {"X-Robot-Control": "sensors", "X-CSRF-Token": token}
    response = client.post("/api/sensor_module/mag", json={"action": "start"}, headers=headers)
    assert response.status_code == 200, response.get_json()
    assert module.actions == ["start"]
    assert forwarded[-1][0] == "http://owner.example:8090/api/sensor_module/mag"
    assert forwarded[-1][1] == "POST"
    assert forwarded[-1][2].get("X-robot-control") == "sensors"
    assert all("Cookie" not in headers for _, _, headers in forwarded)


def test_sensor_api_stays_reachable_and_says_so_when_the_board_is_absent(kiosk_client):
    body = kiosk_client.get("/api/sensor_module")
    assert body.status_code == 200
    payload = body.json
    assert payload["ok"] is True
    assert payload["data"]["configured"] is False      # без RC_SENSOR_PORT
    assert payload["data"]["ok"] is False
    assert payload["data"]["stripEnabled"] is False    # лента пока не подключена


# ------------------------------------------------------------- отрисовка
def test_telemetry_frame_fills_the_whole_screen(tmp_path):
    out = run_screen(screen(heading=123.4, lights={"head": True, "stop": False,
                                                   "left": False, "right": True,
                                                   "dim": True},
                            dim=120, lockOpen=True, cards=3), tmp_path)
    assert out["errors"] == []
    assert out["status"] == {
        "tone": "live", "title": "на связи",
        "sub": "Плата отвечает и присылает кадры",
        "port": "/dev/ttyUSB0", "age": "0.4 с назад", "errors": "нет",
        "warnings": "нет",
    }
    assert out["compass"]["heading"] == "123°"
    assert out["compass"]["cardinal"] == "ЮВ"
    assert out["compass"]["needle"] == "rotate(123.4deg)"
    assert "крен -0.8°" in out["compass"]["tilt"]
    assert "тангаж 1.2°" in out["compass"]["tilt"]
    assert out["baro"]["pressure"].startswith("1004.2")
    assert out["baro"]["temp"] == "23.5"
    assert out["baro"]["alt"] == "77"
    assert out["lights"]["head"]["pressed"] == "true"
    assert out["lights"]["right"]["pressed"] == "true"
    assert out["lights"]["stop"]["pressed"] == "false"
    assert out["door"] == "закрыта"
    assert out["lock"] == {"text": "открыт", "state": "open"}
    assert out["cards"]["count"] == "3"
    assert out["log"] == ["SENSORS: карта записана"]
    assert out["calls"] == [{"url": "api/sensor_module", "method": "GET", "body": None}]


def test_without_a_led_strip_the_ambient_light_is_marked_unavailable(tmp_path):
    """Лента светодиодов пока не подключена — экран не должен это скрывать."""
    out = run_screen(screen(stripEnabled=False, dim=80), tmp_path)
    assert out["strip"]["noteHidden"] is False
    assert out["lights"]["dim"]["disabled"] is True
    assert out["lights"]["dim"]["title"] == "лента не подключена"
    # Реле при этом полностью рабочие.
    assert out["lights"]["head"]["disabled"] is False


def test_wired_strip_enables_the_ambient_controls(tmp_path):
    out = run_screen(screen(stripEnabled=True, dim=140, stripLeds=73), tmp_path)
    assert out["strip"]["noteHidden"] is True
    assert out["lights"]["dim"]["disabled"] is False
    assert out["strip"]["dim"] == "140"
    # Плата сообщила 73 диода — экран подтверждает монтаж, а не молчит.
    assert "73" in out["strip"]["info"]
    assert "проверьте монтаж" not in out["strip"]["info"]
    # Все четыре режима ленты доступны, когда лента есть.
    assert [m["mode"] for m in out["strip"]["modes"]] == ["auto", "floor", "wave", "off"]
    assert all(not m["disabled"] for m in out["strip"]["modes"])


def test_a_different_led_count_is_reported_not_hidden(tmp_path):
    """Лента физически на 73 диода: другое число — повод проверить монтаж."""
    out = run_screen(screen(stripEnabled=True, stripLeds=30), tmp_path)
    assert "30" in out["strip"]["info"] and "проверьте монтаж" in out["strip"]["info"]
    # Старая прошивка без счётчика не должна выдавать ошибку монтажа.
    legacy = run_screen(screen(stripEnabled=True), tmp_path)
    assert "проверьте монтаж" not in legacy["strip"]["info"]


def test_strip_mode_button_sends_the_mode(tmp_path):
    """Кнопка режима уходит на сервер тем же маршрутом, что и свет."""
    payload = screen(stripEnabled=True, stripLeds=73)
    payload["actions"] = {"stripMode": "floor"}
    out = run_screen(payload, tmp_path)
    posts = [c for c in out["callsAfter"] if c["method"] == "POST"]
    assert {"url": "api/sensor_module/strip", "method": "POST",
            "body": {"mode": "floor"}} in posts
    assert out["afterStripMode"]["tone"] == "ok"


def test_a_disabled_mode_button_does_not_post(tmp_path):
    payload = screen(stripEnabled=False)
    payload["actions"] = {"stripMode": "wave"}
    out = run_screen(payload, tmp_path)
    assert not [c for c in out["callsAfter"] if c["method"] == "POST"]


def test_without_a_strip_the_modes_are_disabled_too(tmp_path):
    out = run_screen(screen(stripEnabled=False), tmp_path)
    assert all(m["disabled"] for m in out["strip"]["modes"])


def test_stale_or_missing_board_is_shown_honestly(tmp_path):
    stale = run_screen(screen(ok=False, stale=True, ageMs=5200), tmp_path)
    assert stale["status"]["tone"] == "warn"
    assert stale["status"]["title"] == "данные устарели"
    assert stale["status"]["age"] == "5.2 с назад"

    unconfigured = run_screen({"snapshot": {"ok": True, "data": {
        "configured": False, "ok": False, "error": "не настроен (RC_SENSOR_PORT)"}}},
        tmp_path)
    assert unconfigured["status"]["tone"] == "err"
    assert unconfigured["status"]["title"] == "не подключён"
    assert "RC_SENSOR_PORT" in unconfigured["status"]["sub"]
    assert unconfigured["compass"]["heading"] == "—"
    assert unconfigured["lock"]["text"] == "—"

    port_error = run_screen(screen(ok=False, stale=False, error="[Errno 5] ввод/вывод"),
                            tmp_path)
    assert port_error["status"]["title"] == "ошибка порта"
    assert "Errno 5" in port_error["status"]["sub"]


def test_ack_confirms_command_link_when_sensor_telemetry_is_missing(tmp_path):
    out = run_screen(screen(ok=False, connected=True, connectionSource="ack",
                            stale=False, ageMs=None, heading=None), tmp_path)
    assert out["status"]["tone"] == "warn"
    assert out["status"]["title"] == "команда подтверждена"
    assert "ответила по USB" in out["status"]["sub"]
    assert out["compass"]["heading"] == "—"


def test_server_failure_is_reported_instead_of_a_blank_screen(tmp_path):
    out = run_screen({"fail": [{"match": "^api/sensor_module$", "status": 500,
                                "body": {"ok": False, "error": "сервер лежит"}}]}, tmp_path)
    assert out["status"]["tone"] == "err"
    assert out["status"]["title"] == "нет связи"
    assert "нет ответа сервера" in out["status"]["sub"]


# --------------------------------------------------------------- действия
def test_lights_go_to_the_api_the_operator_pressed(tmp_path):
    off = {"head": False, "stop": False, "left": False, "right": False, "dim": False}
    payload = screen(lights=dict(off, head=True))
    payload["after"] = dict(payload["snapshot"]["data"], lights=off, lightMask=0)
    payload["actions"] = {"toggle": "head"}
    out = run_screen(payload, tmp_path)
    posts = [call for call in out["callsAfter"] if call["method"] == "POST"]
    assert posts == [{"url": "api/sensor_module/lights", "method": "POST",
                      "body": {"light": "head", "on": False}}]
    assert out["afterToggle"]["head"]["pressed"] == "false"
    assert "передний свет" in out["afterToggleMsg"]["text"]
    assert out["afterToggleMsg"]["tone"] == "ok"


def test_open_lock_warning_is_shown_apart_from_sensor_errors(tmp_path):
    """«Замок открыт слишком долго» — предупреждение, а не отказ датчика."""
    payload = screen()
    payload["snapshot"]["data"].update(warnMask=1, warnings=["замок открыт слишком долго"],
                                       lockOpen=True, errors=[])
    out = run_screen(payload, tmp_path)
    assert out["status"]["warnings"] == "замок открыт слишком долго"
    assert out["status"]["errors"] == "нет"
    clean = run_screen(screen(), tmp_path)
    assert clean["status"]["warnings"] == "нет"


def test_automatic_brake_and_near_obstacle_are_shown_with_their_reason(tmp_path):
    """Стоп-сигнал и тревога — состояние сервера, а не кнопки оператора."""
    payload = screen()
    payload["snapshot"]["data"]["driveSignals"] = {
        "enabled": True, "brakeLight": True, "alert": True, "buzz": True,
        "distanceM": 0.42, "nearM": 0.5,
        "reason": "препятствие 0.42 м по курсу", "error": ""}
    out = run_screen(payload, tmp_path)
    assert "стоп-сигнал" in out["signals"]
    assert "тревога" in out["signals"] and "0.42" in out["signals"]
    assert out["signalsTone"] == "err"
    assert out["errors"] == []

    # Тревоги нет — показываем дистанцию по курсу, а не пустую строку.
    payload["snapshot"]["data"]["driveSignals"].update(alert=False, brakeLight=False, buzz=False,
                                                       distanceM=3.1, reason="по курсу свободно")
    calm = run_screen(payload, tmp_path)
    assert "по курсу 3.10 м" in calm["signals"] and calm["signalsTone"] == ""

    # Выключенные автосигналы честно помечены, а не выглядят рабочими.
    payload["snapshot"]["data"]["driveSignals"] = {"enabled": False}
    off = run_screen(payload, tmp_path)
    assert "выключены" in off["signals"]


def test_buzzer_check_button_sends_buzz_without_pin(tmp_path):
    """Кнопка «Проверить сигнал» — короткий тон зуммера, не доступ к отсеку."""
    payload = screen()
    payload["actions"] = {"buzz": True}
    out = run_screen(payload, tmp_path)
    posts = [call for call in out["callsAfter"] if call["method"] == "POST"]
    assert posts == [{"url": "api/sensor_module/buzz", "method": "POST", "body": {}}]
    assert out["afterBuzz"]["tone"] == "ok"
    assert "сигнал зуммера" in out["afterBuzz"]["text"]
    # Кнопка есть на странице и ничего не включает: только звук.
    page = (ROOT / "slam_gui" / "sensors.html").read_text(encoding="utf-8")
    assert 'id="sm-buzz"' in page and "Проверить сигнал" in page


def test_rfid_reader_can_be_switched_off_for_heat_test(tmp_path):
    """Считыватель гасят кнопкой: карты не читаются, RC522 не греется."""
    out = run_screen(screen(rfidOn=False), tmp_path)
    assert out["cards"]["reader"] == "выключен"
    assert out["cards"]["readerState"] == "off"

    payload = screen(rfidOn=False)
    payload["actions"] = {"rfidOff": True}
    out = run_screen(payload, tmp_path)
    posts = [call for call in out["callsAfter"] if call["method"] == "POST"]
    assert posts == [{"url": "api/sensor_module/rfid", "method": "POST", "body": {"on": False}}]
    assert out["afterRfid"]["tone"] == "ok"
    assert "считыватель карт" in out["afterRfid"]["text"]
    # Кнопки есть на странице и не требуют PIN: это режим железа.
    page = (ROOT / "slam_gui" / "sensors.html").read_text(encoding="utf-8")
    assert 'id="sm-rfid-off"' in page and 'id="sm-rfid-on"' in page
    assert 'id="sm-rfid-state"' in page


def test_rfid_reader_switch_on_sends_on(tmp_path):
    payload = screen(rfidOn=False)
    payload["actions"] = {"rfidOn": True}
    out = run_screen(payload, tmp_path)
    posts = [call for call in out["callsAfter"] if call["method"] == "POST"]
    assert posts == [{"url": "api/sensor_module/rfid", "method": "POST", "body": {"on": True}}]


def test_empty_rfid_list_is_distinguished_from_not_loaded(tmp_path):
    out = run_screen(screen(cardList=[], cardListKnown=True), tmp_path)
    assert out["cards"]["empty"] == ["карт нет"]

    unknown = run_screen(screen(cardList=[], cardListKnown=False), tmp_path)
    assert unknown["cards"]["empty"] == ["список не загружен"]


def test_compass_calibration_buttons_send_all_three_actions(tmp_path):
    for action in ("start", "stop", "reset"):
        payload = screen()
        payload["actions"] = {"mag": action}
        out = run_screen(payload, tmp_path)
        posts = [call for call in out["callsAfter"] if call["method"] == "POST"]
        assert posts == [{"url": "api/sensor_module/mag", "method": "POST",
                          "body": {"action": action}}]
        assert out["postHeaders"] == [{"Content-Type": "application/json",
                                       "X-Robot-Control": "sensors"}]
        assert out["afterMagMsg"]["tone"] == "ok"
        assert "команда отправлена" in out["afterMagMsg"]["text"]


def test_remote_operator_gets_csrf_header_for_calibration(tmp_path):
    payload = screen()
    payload["csrfToken"] = "signed-test-token"
    payload["actions"] = {"mag": "start"}
    out = run_screen(payload, tmp_path)
    assert out["postHeaders"] == [{"Content-Type": "application/json",
                                   "X-Robot-Control": "sensors",
                                   "X-CSRF-Token": "signed-test-token"}]


def test_lock_and_cards_require_the_case_pin(tmp_path):
    # Без PIN замок и правка карт не уходят на сервер вовсе.
    out = run_screen(dict(screen(), actions={"lockOpen": True, "uid": "A1B2C3D4",
                                             "cardAdd": True}), tmp_path)
    posts = [call for call in out["callsAfter"] if call["method"] == "POST"
             and "/sensor_module" in call["url"]]
    assert posts == []
    assert out["afterLock"]["tone"] == "err"
    assert "PIN" in out["afterLock"]["text"]
    assert out["afterCardAdd"]["tone"] == "err"
    assert "PIN" in out["afterCardAdd"]["text"]

    # С PIN — уходят, причём UID нормализуется и секунды замка заданы.
    out = run_screen(dict(screen(), actions={"pin": "1234", "lockOpen": True,
                                             "uid": "a1b2c3d4", "cardAdd": True,
                                             "cardLearn": True, "mag": "start"}), tmp_path)
    posts = [call for call in out["callsAfter"] if call["method"] == "POST"]
    # Замок теперь «держится» открытым до повторной карты или кнопки «Закрыть».
    assert posts[0] == {"url": "api/sensor_module/lock", "method": "POST",
                        "body": {"hold": True, "pin": "1234"}}
    assert posts[1] == {"url": "api/sensor_module/cards", "method": "POST",
                        "body": {"action": "add", "uid": "A1B2C3D4", "role": "user",
                                 "pin": "1234"}}
    assert posts[2]["body"] == {"action": "learn", "role": "user", "pin": "1234"}
    assert posts[3] == {"url": "api/sensor_module/mag", "method": "POST",
                        "body": {"action": "start"}}


def test_switch_and_servo_are_shown_independently(tmp_path):
    closed = run_screen(screen(lockOpen=False, doorClosed=False), tmp_path)
    assert closed["lock"] == {"text": "закрыт", "state": "closed"}
    assert closed["door"] == "открыта"
    opened = run_screen(screen(lockOpen=True, doorClosed=True), tmp_path)
    assert opened["lock"] == {"text": "открыт", "state": "open"}
    assert opened["door"] == "закрыта"
    for payload in [screen(ok=False, stale=True, doorClosed=True), screen(protocol=4, doorClosed=None)]:
        assert run_screen(payload, tmp_path)["door"] == "нет данных"


def test_choose_an_unknown_scan_without_granting_rights_to_the_next_card(tmp_path):
    selected = "04112233445566"
    other = "04112233445566778899"
    scans = [{"uid": uid, "role": "none", "reason": "unknown", "granted": False, "atMs": t}
             for t, uid in enumerate([selected, other])]
    payload = screen(recentCards=scans, lastCard=scans[-1])
    payload["actions"] = {"selectScan": selected, "cardRole": "admin", "cardAdd": True}
    out = run_screen(payload, tmp_path)
    assert out["scans"][1:] == [f"{other} · не записана", f"{selected} · не записана"]
    assert [c for c in out["callsAfter"] if c["method"] == "POST"] == []
    payload["actions"]["pin"] = "1234"
    out = run_screen(payload, tmp_path)
    assert [c["body"] for c in out["callsAfter"] if c["method"] == "POST"] == [
        {"uid": selected, "role": "admin", "pin": "1234", "action": "add"}]


def test_cargo_window_is_shown_and_started_with_the_case_pin(tmp_path):
    payload = screen(windowKind="load", windowActive=True)
    payload["actions"] = {"pin": "1234", "cargo": "unload"}
    out = run_screen(payload, tmp_path)
    assert out["window"]["label"] == "окно: погрузка"
    assert out["window"]["state"] == "погрузка"
    assert out["window"]["tone"] == "ok"
    posts = [call for call in out["callsAfter"] if call["method"] == "POST"]
    assert posts == [{"url": "api/sensor_module/cargo", "method": "POST",
                      "body": {"kind": "unload", "pin": "1234"}}]
    assert "окно выгрузки" in out["afterCargo"]["text"]

    # Без PIN окно не открывается, но закрывается: закрывать нечего охранять.
    closed = run_screen(dict(screen(windowKind="load"), actions={"cargo": "end"}), tmp_path)
    posts = [call for call in closed["callsAfter"] if call["method"] == "POST"]
    assert posts == [{"url": "api/sensor_module/cargo", "method": "POST",
                      "body": {"kind": "end"}}]
    blocked = [call for call in closed["callsAfter"] if call["method"] == "POST"
               and call["body"].get("kind") != "end"]
    assert blocked == []


def test_card_list_shows_roles_and_lets_the_operator_change_them(tmp_path):
    out = run_screen(screen(), tmp_path)
    rows = out["cards"]["rows"]
    assert [row["uid"] for row in rows] == ["A1B2C3D4", "01020304"]
    assert [row["role"] for row in rows] == ["admin", "user"]
    assert "администратор" in rows[0]["text"]
    assert "сделать пользователем" in rows[0]["text"]
    assert "пользователь" in rows[1]["text"]
    assert out["cards"]["role"] == ""

    payload = screen()
    payload["actions"] = {"pin": "1234", "roleChange": "A1B2C3D4",
                          "cardDel": "01020304", "cardRole": "admin"}
    changed = run_screen(payload, tmp_path)
    posts = [call for call in changed["callsAfter"] if call["method"] == "POST"]
    assert {"url": "api/sensor_module/cards", "method": "POST",
            "body": {"action": "role", "uid": "A1B2C3D4", "role": "user",
                     "pin": "1234"}} in posts
    assert {"url": "api/sensor_module/cards", "method": "POST",
            "body": {"action": "del", "uid": "01020304", "pin": "1234"}} in posts
    assert "роль A1B2C3D4: команда отправлена" in changed["afterRoleChange"]["text"]
    assert changed["afterRoleChange"]["tone"] == "ok"


def test_role_change_without_the_pin_never_reaches_the_server(tmp_path):
    payload = screen()
    payload["actions"] = {"roleChange": "A1B2C3D4", "cardDel": "01020304"}
    out = run_screen(payload, tmp_path)
    assert [call for call in out["callsAfter"] if call["method"] == "POST"] == []
    assert "PIN" in out["afterRoleChange"]["text"]


def test_short_uid_is_reported_before_the_request(tmp_path):
    out = run_screen(dict(screen(), actions={"pin": "1234", "uid": "A1B2",
                                             "cardAdd": True}), tmp_path)
    assert [call for call in out["callsAfter"] if call["method"] == "POST"] == []
    assert "8, 14 или 20 hex" in out["afterCardAdd"]["text"]
