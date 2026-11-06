"""Экран «Модуль датчиков» бортового экрана.

Страница ``/sensors`` — такая же, как «Карта» и «Устройства»: отдельный адрес,
свой скрипт и стили, ссылка с бортового экрана и с инженерного пульта. Здесь
закреплено то, что действительно ломается:

* страница открывается без подключённой платы — данные подтягивает скрипт;
* кадр телеметрии заполняет полосу состояния, компас, барометр, свет, дверь,
  карты и журнал платы;
* без ленты светодиодов габариты честно помечены как недоступные, а не
  выглядят рабочими;
* телеметрия без свежего кадра не выдаёт себя за «на связи»;
* открытие замка и правка карт не уходят на сервер без PIN кейса;
* ошибка сервера видна оператору, а не остаётся пустым экраном.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

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
        "protocol": 2,
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
        "doorOpen": False,
        "lockOpen": False,
        "lightMask": 0,
        "lights": {"head": False, "stop": False, "left": False, "right": False, "dim": False},
        "dim": 80,
        "stripEnabled": False,
        "rfid": "ready",
        "cards": 3,
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
def test_sensors_screen_is_reachable_like_the_other_screens(kiosk_client):
    page = kiosk_client.get("/sensors")
    assert page.status_code == 200
    html = page.text
    assert "sensors.js" in html and "sensors.css" in html
    assert 'href="/map"' in html and 'href="/devices"' in html
    assert 'aria-current="page"' in html
    # Файлы страницы отдаются тем же сервером, что и остальные экраны.
    assert kiosk_client.get("/sensors.js").status_code == 200
    assert kiosk_client.get("/sensors.css").status_code == 200
    # Ссылка есть на бортовом экране и на инженерном пульте.
    assert "k-sensors-open" in kiosk_client.get("/kiosk").text
    main_page = (ROOT / "slam_gui" / "main.html").read_text(encoding="utf-8")
    assert 'href="/sensors" id="nav-sensors"' in main_page
    console = (ROOT / "slam_gui" / "index.html").read_text(encoding="utf-8")
    assert 'id="console-sensors-link"' in console and 'href="/sensors"' in console


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
                            dim=120, doorOpen=False, lockOpen=True, cards=3), tmp_path)
    assert out["errors"] == []
    assert out["status"] == {
        "tone": "live", "title": "на связи",
        "sub": "Плата отвечает и присылает кадры",
        "port": "/dev/ttyUSB0", "age": "0.4 с назад", "errors": "нет",
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
    assert out["door"] == {"door": "закрыта", "lock": "открыт"}
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
    out = run_screen(screen(stripEnabled=True, dim=140), tmp_path)
    assert out["strip"]["noteHidden"] is True
    assert out["lights"]["dim"]["disabled"] is False
    assert out["strip"]["dim"] == "140"


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
    assert unconfigured["door"]["door"] == "—"

    port_error = run_screen(screen(ok=False, stale=False, error="[Errno 5] ввод/вывод"),
                            tmp_path)
    assert port_error["status"]["title"] == "ошибка порта"
    assert "Errno 5" in port_error["status"]["sub"]


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
    assert posts[0] == {"url": "api/sensor_module/lock", "method": "POST",
                        "body": {"open": True, "seconds": 3, "pin": "1234"}}
    assert posts[1] == {"url": "api/sensor_module/cards", "method": "POST",
                        "body": {"action": "add", "uid": "A1B2C3D4", "pin": "1234"}}
    assert posts[2]["body"] == {"action": "learn", "pin": "1234"}
    assert posts[3] == {"url": "api/sensor_module/mag", "method": "POST",
                        "body": {"action": "start"}}


def test_short_uid_is_reported_before_the_request(tmp_path):
    out = run_screen(dict(screen(), actions={"pin": "1234", "uid": "A1B2",
                                             "cardAdd": True}), tmp_path)
    assert [call for call in out["callsAfter"] if call["method"] == "POST"] == []
    assert "8 hex" in out["afterCardAdd"]["text"]
