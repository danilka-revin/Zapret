"""Активное удержание угла по потенциометру (команда прошивки ``ph``, fw24).

Проверяется весь путь одного переключателя:

* парсер телеметрии: бит ``0x8000`` в коротком кадре и поле ``ph=`` в ``@TLM``,
  причём у прошивок до fw24 ``potHold`` = ``None`` (экран обязан показать
  «нужна прошивка», а не выдать отсутствие за «выключено»);
* сервер: ``set_pot_hold`` (связь, прошивка, занятость, датчик) и
  ``dispatch_pot_hold`` (строгий payload, E-STOP, отсутствие метода);
* маршрут бортового экрана ``POST /api/kiosk_pot_hold`` и прокси пульта;
* сам переключатель — в mini-DOM через ``tests/js/pot_hold.js``.

Симуляция прошивки (``tests/firmware_sim``, сценарии ``pot_hold*``) проверяет
саму петлю удержания: возврат после ручного сдвига, потолки и выключение
командами. Здесь — только то, что видит и отправляет сервер и экран.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_control import slam

ROOT = Path(__file__).resolve().parent.parent
HARNESS = ROOT / "tests" / "js" / "pot_hold.js"
NODE = shutil.which("node")
MODULES = ("FL", "FR", "RL", "RR")


_BACKEND = None


def _load_backend():
    """slam_gui/backend.py грузится по пути, как в robot_control/slam.py."""
    global _BACKEND
    if _BACKEND is None:
        import importlib.util

        path = ROOT / "slam_gui" / "backend.py"
        spec = importlib.util.spec_from_file_location("rus_slam_backend_hold", str(path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _BACKEND = module
    return _BACKEND


def _bare_master_source(backend):
    import queue
    import threading

    src = object.__new__(backend.MasterSource)
    src.lock = threading.Lock()
    src.frames = {}
    src.drive_percent = 0
    src.drive_last_command = 0.0
    src.commands = queue.Queue(maxsize=4)
    src.serial_ready = threading.Event()
    src.serial_ready.set()
    src.messages = []
    src.on_message = None
    return src


def _frame(module, *, fw=24, pot="1", pm="1", moving="0", cal="1", ph="0"):
    """Полный кадр @TLM fw24 с настоящей CRC; ph=None — кадр прошивки до fw24."""
    fields = ["@TLM", "mod=%s" % module, "deg=0.00", "tgt=0.00", "moving=%s" % moving,
              "cal=%s" % cal, "cycle=0", "calib=0", "opto=0", "home=opto", "err=0",
              "drive=0", "fw=%d" % fw, "pot=%s" % pot, "pm=%s" % pm, "zfault=0",
              "tach=0", "dmax=100", "sv=0", "duty=0", "sc_rpm=0.0", "sc_hz=0",
              "sc_ppr=90", "sc_count=0", "sc_recent=0", "brake_id=0", "neutral_id=0",
              "neutral_lock=0", "live_id=0", "live_seq=0", "t=1000"]
    if ph is not None:
        fields.append("ph=%s" % ph)
    payload = " ".join(fields)
    return "%s crc=%04X" % (payload, _load_backend().crc16(payload.encode("ascii")))


def _feed_all(src, **kwargs):
    for module in MODULES:
        src.feed(_frame(module, **kwargs), now=100.0)


# --------------------------------- парсер телеметрии ------------------------- #
def test_telemetry_reports_pot_hold_only_from_firmware_24():
    backend = _load_backend()
    frame = backend.parse_master_line(_frame("FR", ph="1"))
    assert frame["potHold"] is True
    assert frame["firmware"] == 24 and frame["pot"] is True
    assert backend.parse_master_line(_frame("FR", ph="0"))["potHold"] is False
    # fw24 без поля ph= — это не «выключено», а испорченный/чужой кадр.
    without = _frame("FR", ph=None)                  # fw24 без поля ph=
    assert backend.parse_master_line(without) is None
    assert backend.parse_master_line(_frame("FR", ph="2")) is None
    # Прошивка до fw24 поля не знает: состояние неизвестно, ключ есть, но None.
    old = backend.parse_master_line(_frame("FR", fw=23, ph=None))
    assert old["potHold"] is None


def test_fast_telemetry_carries_the_hold_bit():
    """Бит 0x8000 в коротком кадре — состояние удержания; код ошибки не сдвинут."""
    backend = _load_backend()

    def checked(status):
        payload = "@F 24 2 0 0 %d 0 100 0 0 0 90 0 0 0 0 0 0 0 5" % status
        return "%s %04X" % (payload, backend.crc16(payload.encode("ascii")))

    hold = backend.parse_master_line(checked(0x40 | 0x2000 | 0x8000))
    plain = backend.parse_master_line(checked(0x40 | 0x2000))
    assert hold["potHold"] is True and plain["potHold"] is False
    assert hold["calibError"] == 0, "бит удержания не должен попадать в код ошибки"
    # Бита нет, а прошивка обещает fw24 — значит, кадр собран не этой платой.
    assert backend.parse_master_line(checked(0x40 | 0x2000 | 0x8000))["firmware"] == 24
    assert backend.parse_master_line(checked(0x40 | 0x2000))["potHold"] is False


def test_state_snapshot_exposes_pot_hold_per_wheel():
    backend = _load_backend()
    src = _bare_master_source(backend)
    _feed_all(src, ph="1")
    motors = {motor["id"]: motor for motor in src.read(now=100.2)["motors"]}
    assert all(motors[module]["potHold"] is True for module in MODULES)
    src.feed(_frame("FR", fw=23, ph=None), now=100.0)
    motors = {motor["id"]: motor for motor in src.read(now=100.2)["motors"]}
    assert motors["FR"]["potHold"] is None
    offline = {motor["id"]: motor for motor in src.read(now=200.0)["motors"]}
    assert all(offline[module]["potHold"] is None for module in MODULES)


# ----------------------------------- сервер ---------------------------------- #
def test_set_pot_hold_requires_fresh_calibrated_idle_boards():
    backend = _load_backend()
    src = _bare_master_source(backend)
    assert src.set_pot_hold("on", now=100.2)["reason"] == "module_offline"
    for module in ("FL", "FR", "RL"):
        src.feed(_frame(module), now=100.0)
    assert src.set_pot_hold("on", now=100.2)["reason"] == "module_offline"
    # Старая прошивка: команду ph она не знает, подтверждения не будет.
    src.feed(_frame("RR", fw=23, ph=None), now=100.0)
    assert src.set_pot_hold("on", now=100.2)["reason"] == "firmware_update_required"
    assert src.set_pot_hold("off", now=100.2)["reason"] == "firmware_update_required"
    # Без режима с потенциометром включить нельзя, выключить — можно.
    src.feed(_frame("RR", pm="0"), now=100.0)
    assert src.set_pot_hold("on", now=100.2)["reason"] == "pot_mode_off"
    assert src.set_pot_hold("off", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"ph 0\n"
    # Не откалиброван датчик (никогда не было q) — включать нечего.
    src.feed(_frame("RR", pot="0"), now=100.0)
    assert src.set_pot_hold("on", now=100.2)["reason"] == "pot_not_calibrated"
    src.feed(_frame("RR"), now=100.0)
    result = src.set_pot_hold("on", now=100.2)
    assert result["ok"] is True and result["potHold"] is True
    assert src.commands.get_nowait() == b"ph 1\n"
    # Движение и тяга запрещают включение, но не выключение: снять удержание
    # оператор должен мочь и в тот момент, когда колесо само подтягивает угол.
    src.feed(_frame("FR", moving="1"), now=100.0)
    assert src.set_pot_hold("on", now=100.2)["reason"] == "module_busy"
    assert src.set_pot_hold("off", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"ph 0\n"
    src.feed(_frame("FR"), now=100.0)
    src.drive_percent = 8
    assert src.set_pot_hold("on", now=100.2)["reason"] == "module_busy"
    assert src.set_pot_hold("off", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"ph 0\n"
    src.drive_percent = 0
    # Выключение не требует и калибровки: иначе после сброса pm удержание
    # осталось бы включённым без способа его снять.
    src.feed(_frame("FR", pot="0", pm="0"), now=100.0)
    assert src.set_pot_hold("off", now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"ph 0\n"
    src.feed(_frame("FR"), now=100.0)
    assert src.set_pot_hold("calibrate", now=100.2)["reason"] == "invalid_action"
    # Устаревшая телеметрия — снова офлайн.
    assert src.set_pot_hold("off", now=102.0)["reason"] == "module_offline"
    src.serial_ready.clear()
    assert src.set_pot_hold("off", now=100.2)["reason"] == "serial_unavailable"


class _HoldSource:
    name = "master"
    supports_individual_drive = True
    supports_individual_steer = True
    supports_calibration = True
    supports_drive = True
    MAX_STEER_DEG = 100

    def __init__(self):
        self.calls = []

    def set_pot_hold(self, action):
        self.calls.append(action)
        return {"ok": True, "queued": True, "potHold": action == "on"}


def test_dispatch_pot_hold_payload_estop_and_support():
    from robot_control import estop

    source = _HoldSource()
    assert slam.dispatch_pot_hold(source, {"action": "on"}) == (
        {"ok": True, "queued": True, "potHold": True}, 200)
    assert source.calls == ["on"]
    assert slam.dispatch_pot_hold(source, {"action": "off"})[1] == 200
    # Строгий payload: только action со значениями on/off.
    for body in (None, {}, {"action": "calibrate"}, {"action": 1}, {"action": "on", "module": "FL"},
                 [], "on"):
        assert slam.dispatch_pot_hold(source, body) == ({"ok": False, "reason": "payload"}, 400)
    assert source.calls == ["on", "off"]
    # Нет метода — не 500 и не «сделали вид»: честный отказ.
    class NoHold:
        name = "none"
    assert slam.dispatch_pot_hold(NoHold(), {"action": "on"}) == (
        {"ok": False, "reason": "control_not_supported"}, 409)
    # E-STOP отклоняет до любых команд платам.
    estop.latch("test")
    try:
        assert slam.dispatch_pot_hold(source, {"action": "on"}) == (
            {"ok": False, "reason": "estop"}, 409)
        assert source.calls == ["on", "off"]
    finally:
        estop.release()


def test_dispatch_pot_hold_maps_offline_to_unavailable():
    class Offline:
        name = "master"

        def set_pot_hold(self, action):
            return {"ok": False, "reason": "module_offline"}

    assert slam.dispatch_pot_hold(Offline(), {"action": "on"}) == (
        {"ok": False, "reason": "module_offline"}, 503)


# ---------------------------------- маршруты --------------------------------- #
def test_kiosk_pot_hold_route(kiosk_client):
    source = _HoldSource()
    kiosk_client.application.extensions["rc_slam"].source = source
    headers = {"X-Robot-Control": "steering"}
    assert kiosk_client.get("/api/kiosk_pot_hold").status_code == 405
    assert kiosk_client.post("/api/kiosk_pot_hold", json={"action": "on"}).status_code == 400
    assert kiosk_client.post("/api/kiosk_pot_hold", json={"action": "on"},
                             headers={"X-Robot-Control": "drive"}).status_code == 400
    for body in ({}, {"action": "x"}, {"action": "on", "extra": 1}):
        assert kiosk_client.post("/api/kiosk_pot_hold", json=body, headers=headers).status_code == 400
    assert source.calls == []
    assert kiosk_client.post("/api/kiosk_pot_hold", json={"action": "on"},
                             headers=headers).status_code == 200
    assert kiosk_client.post("/api/kiosk_pot_hold", json={"action": "off"},
                             headers=headers).status_code == 200
    assert source.calls == ["on", "off"]


def test_kiosk_pot_hold_respects_estop(kiosk_client):
    from robot_control import estop

    source = _HoldSource()
    kiosk_client.application.extensions["rc_slam"].source = source
    estop.latch("test E-STOP")
    try:
        response = kiosk_client.post("/api/kiosk_pot_hold", json={"action": "on"},
                                     headers={"X-Robot-Control": "steering"})
        assert response.status_code == 409 and response.get_json()["reason"] == "estop"
        assert source.calls == []
    finally:
        estop.release()


# ----------------------------------- экран ----------------------------------- #
def _motor(module, **overrides):
    motor = {"id": module, "online": True, "homed": True, "angle": 0.0, "targetAngle": 0.0,
             "moving": False, "calibrating": False, "cycling": False, "drivePercent": 0,
             "pot": True, "potMode": True, "potHold": False, "firmware": 24}
    motor.update(overrides)
    return motor


def _motors(**overrides):
    return [_motor(module, **overrides) for module in MODULES]


def run_screen(tmp_path, body) -> dict:
    assert NODE, "node нужен для проверки экрана"
    fixture = tmp_path / "payload.json"
    fixture.write_text(json.dumps(body, ensure_ascii=False), encoding="utf-8")
    result = subprocess.run([NODE, str(HARNESS), str(fixture)], cwd=ROOT,
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(result.stdout)
    assert "fatal" not in report, report.get("fatal")
    return report


def test_switch_renders_state_from_telemetry(tmp_path):
    report = run_screen(tmp_path, {"steps": []})
    first = report["seen"][0]
    assert first["tag"] == "BUTTON" and first["role"] == "switch" and first["type"] == "button"
    assert first["ariaChecked"] == "false"
    assert first["text"] == "Активная доводка — выкл"
    assert "само возвращается к заданному углу" in first["hint"]
    assert "после перезапуска платы — выключено" in first["hint"]
    assert "больше 15°" in first["hint"]
    assert first["requests"] == 0
    assert report["parentChildren"] == 1, "компонент вставляется один раз, а не каждый кадр"


@pytest.mark.parametrize("step,status_part", [
    ({"motors": _motors(online=False)}, "связь со всеми"),
    ({"motors": _motors(potHold=None)}, "прошивка fw24"),
    ({"motors": _motors(pot=False)}, "потенциометра"),
    ({"motors": _motors(potMode=False)}, "потенциометра"),
    ({"motors": _motors(moving=True)}, "Идёт движение"),
])
def test_switch_is_disabled_by_state(tmp_path, step, status_part):
    report = run_screen(tmp_path, {"steps": [dict(step, name="blocked", click=True)]})
    seen = report["seen"][-1]
    assert seen["disabled"] is True
    assert status_part in seen["status"]
    assert report["requests"] == [], "при disabled POST не уходит"


def test_unknown_hold_is_not_exposed_as_a_confirmed_off_switch(tmp_path):
    report = run_screen(tmp_path, {"steps": [
        {"name": "no-telemetry", "motors": _motors(potHold=None)},
        {"name": "restored", "motors": _motors(potHold=True)},
    ]})
    unknown, restored = report["seen"][1:]
    assert unknown["disabled"] is True
    assert unknown["role"] is None and unknown["ariaChecked"] is None
    assert "нет подтверждения" in unknown["text"]
    assert restored["role"] == "switch" and restored["ariaChecked"] == "true"


def test_switch_stays_usable_while_the_hold_moves_the_wheel(tmp_path):
    """Пока удержание само подтягивает угол, выключатель должен работать.

    Плата в этот момент занята (`moving`), и запрет «идёт движение» оставил бы
    оператора без выключателя — а ему нужно снять удержание как раз тогда.
    """
    report = run_screen(tmp_path, {"steps": [
        {"name": "moving", "motors": _motors(potHold=True, moving=True)},
        {"name": "click", "motors": _motors(potHold=True, moving=True), "click": True},
    ]})
    moving = report["seen"][1]
    assert moving["disabled"] is False
    assert "подтягивает" in moving["status"]
    assert [item["action"] for item in report["requests"]] == ["off"]


def test_switch_resolves_boards_that_disagree(tmp_path):
    """Расхождение плат лечится выключением, а не заблокированным экраном."""
    motors = _motors()
    for index in (2, 3):
        motors[index]["potHold"] = True
    report = run_screen(tmp_path, {"steps": [
        {"name": "mixed", "motors": motors},
        {"name": "click", "motors": motors, "click": True},
    ]})
    mixed = report["seen"][1]
    assert mixed["disabled"] is False
    # Нельзя объявлять расходящиеся платы выключенным ARIA-переключателем.
    assert mixed["role"] is None and mixed["ariaChecked"] is None
    assert "расходятся" in mixed["status"]
    assert [item["action"] for item in report["requests"]] == ["off"]


def test_switch_is_disabled_without_pin_estop_or_neutral(tmp_path):
    steps = [
        {"name": "no-pin", "authorized": False, "click": True},
        {"name": "estop", "state": {"estop": {"latched": True, "unknown": False}}, "click": True},
        {"name": "neutral", "state": {"neutral": {"locked": True}}, "click": True},
    ]
    report = run_screen(tmp_path, {"steps": steps})
    by_name = {item["step"]: item for item in report["seen"]}
    for name in ("no-pin", "estop", "neutral"):
        assert by_name[name]["disabled"] is True, name
        assert by_name[name]["requests"] == 0, name
    assert "PIN" in by_name["no-pin"]["status"]
    assert "E-STOP" in by_name["estop"]["status"]
    assert "тормоз" in by_name["neutral"]["status"]


def test_switch_waits_for_telemetry_instead_of_optimistic_toggle(tmp_path):
    report = run_screen(tmp_path, {"steps": [
        {"name": "click", "click": True},
        {"name": "confirmed", "motors": _motors(potHold=True)},
    ]})
    assert len(report["requests"]) == 1
    assert report["requests"][0]["url"] == "api/kiosk_pot_hold"
    assert report["requests"][0]["method"] == "POST"
    assert report["requests"][0]["header"] == "steering"
    assert report["requests"][0]["action"] == "on"
    clicked = report["seen"][1]
    # Оптимистичного переключения нет: до подтверждения плат — «выкл» и ожидание.
    assert clicked["ariaChecked"] == "false"
    assert clicked["text"] == "Активная доводка — выкл"
    assert "Ждём подтверждения" in clicked["status"]
    confirmed = report["seen"][2]
    assert confirmed["ariaChecked"] == "true"
    assert confirmed["text"] == "Активная доводка — вкл"
    assert "Подтверждено" in confirmed["status"]
    assert len(report["requests"]) == 1, "подтверждение пришло из телеметрии, а не вторым POST"


def test_switch_reports_missing_confirmation_after_five_seconds(tmp_path):
    report = run_screen(tmp_path, {"steps": [
        {"name": "click", "click": True},
        {"name": "no-answer", "advanceMs": 6000},
    ]})
    waiting = report["seen"][1]
    assert "Ждём подтверждения" in waiting["status"]
    expired = report["seen"][2]
    assert "не подтвердили" in expired["alert"]
    assert expired["ariaChecked"] == "false"


def test_switch_off_needs_no_confirmation_dialog(tmp_path):
    report = run_screen(tmp_path, {"steps": [
        {"name": "on", "motors": _motors(potHold=True)},
        # Выключение подтверждения не требует: снять удержание можно всегда.
        {"name": "click", "motors": _motors(potHold=True), "click": True},
    ]})
    assert report["seen"][1]["ariaChecked"] == "true"
    assert report["requests"][0]["action"] == "off"
    # После POST состояние всё равно ждёт телеметрию, а не переключается само.
    assert report["seen"][2]["ariaChecked"] == "true"
    assert "Ждём подтверждения" in report["seen"][2]["status"]


def test_switch_works_from_keyboard(tmp_path):
    """Enter и пробел на переключателе — то же нажатие, и ровно одна команда."""
    report = run_screen(tmp_path, {"steps": [
        {"name": "enter", "key": "Enter"},
        # Платы подтвердили — второй шаг идёт уже с включённым удержанием.
        {"name": "space", "motors": _motors(potHold=True), "key": " "},
    ]})
    actions = [item["action"] for item in report["requests"]]
    # На одно нажатие — одна команда: без синтетического click от браузера
    # вторая не задвоилась бы.
    assert actions == ["on", "off"]
    assert report["seen"][1]["status"].startswith("Ждём подтверждения")
    # Телеметрия ещё говорит «включено», поэтому переключатель не врёт и ждёт.
    assert report["seen"][2]["ariaChecked"] == "true"
    assert report["seen"][2]["status"].startswith("Ждём подтверждения")
    assert report["seen"][2]["disabled"] is True


def test_switch_keyboard_is_ignored_when_disabled(tmp_path):
    report = run_screen(tmp_path, {"steps": [
        {"name": "no-pin", "authorized": False, "key": "Enter"},
    ]})
    assert report["requests"] == []


def test_switch_does_not_send_when_confirmation_cancelled(tmp_path):
    report = run_screen(tmp_path, {"confirm": False, "steps": [{"name": "click", "click": True}]})
    assert report["requests"] == []
    assert report["seen"][-1]["ariaChecked"] == "false"


def test_switch_shows_reason_from_server(tmp_path):
    report = run_screen(tmp_path, {
        "responses": [{"status": 409, "json": {"ok": False, "reason": "pot_not_calibrated"}}],
        "steps": [{"name": "click", "click": True}],
    })
    assert "не откалибровано по потенциометру" in report["seen"][-1]["alert"]


def test_screens_mount_the_switch():
    robot_js = (ROOT / "slam_gui" / "robot.js").read_text(encoding="utf-8")
    steering_js = (ROOT / "slam_gui" / "steering.js").read_text(encoding="utf-8")
    assert "RCPotHold.render(document.querySelector('.steer-calib-bar')" in robot_js
    assert "RCPotHold.render(document.querySelector('.rc-steering-toolbar')" in steering_js
    for page in ("robot.html", "index.html", "main.html"):
        html = (ROOT / "slam_gui" / page).read_text(encoding="utf-8")
        assert '<script src="pot_hold.js"></script>' in html, page
    # Компонент не переключает состояние сам: только POST и ожидание телеметрии.
    component = (ROOT / "slam_gui" / "pot_hold.js").read_text(encoding="utf-8")
    assert "aria-checked" in component and "role', 'switch'" in component
    assert "Ждём подтверждения плат" in component


def test_switch_names_missing_sensor_large_deviation_and_live_wheel(tmp_path):
    motors = _motors(firmware=26, potAngle=0, fullZeroFault=False)
    missing = [dict(m, potAngle=None if m['id'] == 'RL' else 0) for m in motors]
    far = [dict(m, potAngle=15.1 if m['id'] == 'FR' else 0) for m in motors]
    live = [dict(m, liveId=42 if m['id'] == 'RR' else 0) for m in motors]
    memory = [dict(m, fullZeroFault=m['id'] == 'FL') for m in motors]
    old_on_with_fault = _motors(potHold=True)
    old_on_with_fault[1]['pot'] = False
    mixed_with_fault = [dict(m, potHold=m['id'] != 'FR', pot=m['id'] != 'FR',
                             potAngle=None if m['id'] == 'FR' else 0) for m in motors]
    report = run_screen(tmp_path, {"steps": [
        {"name": "missing", "motors": missing, "click": True},
        {"name": "far", "motors": far, "click": True},
        {"name": "live", "motors": live, "click": True},
        {"name": "memory", "motors": memory, "click": True},
        {"name": "old_on_fault", "motors": old_on_with_fault},
        {"name": "mixed_fault", "motors": mixed_with_fault},
        {"name": "ready", "motors": motors},
    ]})
    by_name = {x["step"]: x for x in report["seen"]}
    assert by_name["missing"]["disabled"] and 'RL' in by_name["missing"]["status"]
    assert by_name["far"]["disabled"] and 'FR' in by_name["far"]["status"] and '15°' in by_name["far"]["status"]
    assert by_name["live"]["disabled"] and 'RR' in by_name["live"]["status"] and 'WASD' in by_name["live"]["status"]
    assert by_name["memory"]["disabled"] and 'FL' in by_name["memory"]["status"] and 'памяти' in by_name["memory"]["status"]
    assert not by_name["old_on_fault"]["disabled"] and by_name["old_on_fault"]["ariaChecked"] == 'true'
    assert 'FR' in by_name["old_on_fault"]["status"] and 'датчик не готов' in by_name["old_on_fault"]["status"]
    assert not by_name["mixed_fault"]["disabled"] and by_name["mixed_fault"]["ariaChecked"] is None
    assert 'FR' in by_name["mixed_fault"]["status"] and 'датчик не готов' in by_name["mixed_fault"]["status"]
    assert not by_name["ready"]["disabled"] and report["requests"] == []


def test_partial_confirmation_names_refusing_board_and_off_is_available(tmp_path):
    mixed = [_motor(mid, potHold=mid != 'RL') for mid in MODULES]
    report = run_screen(tmp_path, {"steps": [
        {"name": "click", "click": True},
        {"name": "partial", "motors": mixed, "advanceMs": 6000},
        {"name": "switch-off", "motors": mixed, "click": True},
    ]})
    partial = report["seen"][2]
    assert partial["role"] is None and partial["ariaChecked"] is None
    assert 'RL' in partial["alert"] and 'Часть плат включена' in partial["alert"]
    assert not partial["disabled"]
    assert [item["action"] for item in report["requests"]] == ['on', 'off']


def test_server_reason_contains_the_specific_modules(tmp_path):
    report = run_screen(tmp_path, {
        "responses": [{"status": 409, "json": {"ok": False, "reason": "pot_hold_far", "modules": ["RL"]}}],
        "steps": [{"name": "click", "click": True}],
    })
    assert '15°' in report["seen"][-1]["alert"] and 'RL' in report["seen"][-1]["alert"]
