"""Руление на бортовом экране: пресеты, предел угла и команды — без браузера.

``slam_gui/robot.js`` написан под браузер и ничего не экспортирует, поэтому
экран выполняется в минимальном DOM (``tests/js/steering_screen.js``, тот же
приём, что для карты лидара), а проверяется то, что видит и делает оператор:

  * какие пресеты нарисованы, какие нажимаемы и что они отправляют;
  * ход ползунков берётся из ответа сервера (``steeringControl.limitDeg``),
    а не зашит в разметку — иначе экран разрешает угол, который плата режет;
  * пресет, который железо не тянет, честно недоступен и ничего не отправляет;
  * отказ на одном колесе останавливает пресет, а не доворачивает остальные;
  * движение — удерживаемое: нажал → тяга, отпустил → 0.
"""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
HARNESS = ROOT / "tests" / "js" / "steering_screen.js"
NODE = shutil.which("node")

pytestmark = pytest.mark.skipif(NODE is None, reason="node нужен для проверки экрана")

MODULES = ["FL", "FR", "RL", "RR"]


def motor(module, *, angle=0.0, target=None, online=True, homed=True, moving=False):
    return {
        "id": module, "online": online, "homed": homed, "homeSource": "opto" if homed else "none",
        "angle": angle, "targetAngle": angle if target is None else target,
        "moving": moving, "calibrating": False,
        "cycling": False, "opto": False, "calibError": 0, "drivePercent": 0, "rpm": 0.0,
        "firmware": 6, "pot": True,
    }


def drive_config(**overrides):
    """Настройка привода в ответе сервера: серво-ШИМ, нейтраль, размах, STOP."""
    config = {
        "supported": True, "confirmed": True, "mixed": False,
        "outputMode": "servo", "tach": True, "brake": True, "stopInput": True,
        "maxPercent": 20, "limitPercent": 20,
        "trimUs": 1500, "spanUs": 500, "pulseUs": 0, "pulsesPerRev": 1,
        "dirReverse": False, "perModule": {},
    }
    config.update(overrides)
    return config


def maneuver_config(**overrides):
    """Пресеты манёвров в ответе сервера: цель в метрах/градусах и процент."""
    presets = {
        "forward": {"title": "Вперёд", "kind": "distance", "distanceM": 1.0,
                    "percent": 12, "toleranceM": 0.05},
        "backward": {"title": "Назад", "kind": "distance", "distanceM": 0.5,
                     "percent": 10, "toleranceM": 0.05},
        "left": {"title": "Боком влево", "kind": "distance", "distanceM": 0.5,
                 "percent": 10, "toleranceM": 0.05},
        "right": {"title": "Боком вправо", "kind": "distance", "distanceM": 0.5,
                  "percent": 10, "toleranceM": 0.05},
        "spin_left": {"title": "Разворот влево", "kind": "angle", "angleDeg": 90.0,
                      "percent": 10, "toleranceDeg": 3.0},
        "spin_right": {"title": "Разворот вправо", "kind": "angle", "angleDeg": 90.0,
                       "percent": 10, "toleranceDeg": 3.0},
    }
    for name, values in overrides.pop("presets", {}).items():
        presets[name] = dict(presets[name], **values)
    config = {"supported": True, "presets": presets,
              "limits": {"distanceM": [0.05, 20.0], "angleDeg": [5.0, 360.0],
                         "percent": [1.0, 100.0]}}
    config.update(overrides)
    return config


def per_module_state(**overrides):
    """Готовность одиночной тяги по колёсам (drive.perModule от сервера)."""
    state = {"canDrive": True, "reason": None, "percent": 0, "commandedPercent": 0,
             "pulseUs": 1500, "rpm": 0.0, "tach": True,
             "dirReverse": False, "canSetDirection": True, "directionReason": None}
    state.update(overrides)
    return {module: dict(state) for module in MODULES}


def payload(*, limit=90.0, motors=None, available=True, can_drive=True, drive_reason=None,
            actions=None, drive=None, per_module=None, maneuver=None, **extra):
    data = {

        "linkOk": True,
        "steeringControl": {
            "available": available, "source": "master" if available else "none",
            "zeroMode": "opto", "optoUsed": True, "calibration": True,
            "manualZero": True, "limitDeg": limit,
        },
        "drive": {"available": available, "canDrive": can_drive, "reason": drive_reason,
                  "maxPercent": 20, "commandedPercent": 0, "active": False,
                  "individual": True, "commandedModules": {},
                  "perModule": per_module if per_module is not None else per_module_state(),
                  "config": drive_config(), "alignmentToleranceDeg": 2.5,
                  "maneuver": maneuver or {"supported": True, "active": False,
                                           "state": "idle", "mode": "odometry"}},
        "motors": motors if motors is not None else [motor(m) for m in MODULES],
        "masterMessages": [],
        "maneuverConfig": maneuver_config(),
    }
    return {"ok": True, "data": data, "actions": actions or [], **extra}


def run_screen(tmp_path, body) -> dict:
    fixture = tmp_path / "payload.json"
    fixture.write_text(json.dumps(body, ensure_ascii=False), encoding="utf-8")
    result = subprocess.run([NODE, str(HARNESS), str(fixture)], cwd=ROOT,
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(result.stdout)
    assert not report["fatal"] if "fatal" in report else True
    return report


def preset(report, preset_id):
    matches = [item for item in report["presets"] if item["id"] == preset_id]
    assert matches, "пресет %s не нарисован: %s" % (preset_id, report["presets"])
    return matches[0]


def angles_sent(report):
    return [(item["module"], item["angle"]) for item in report["sent"]]


def test_screen_renders_without_errors(tmp_path):
    report = run_screen(tmp_path, payload())
    assert report["renderErrors"] == []
    assert report["dataChip"] == "в реальном времени"


def wheel_drive(mod):
    return next(item for item in mod["wheelDrive"] if item["id"] == "FL") if isinstance(mod, dict) else None


def test_each_wheel_card_has_its_own_drive_controls(tmp_path):
    """В карточке колеса — своя тяга: кнопки, ползунок и свой предел."""
    report = run_screen(tmp_path, payload())
    assert report["renderErrors"] == []
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert sorted(cards) == MODULES
    # Все колёса на связи и с нулём — кнопки доступны, ползунок тоже.
    assert all(not item["backDisabled"] and not item["forwardDisabled"] for item in cards.values())
    assert all(item["sliderMax"] == "20" for item in cards.values())
    assert all(item["state"] == "стоит" for item in cards.values())


def test_single_wheel_drive_sends_only_that_wheel(tmp_path):
    """Удержание «вперёд» в карточке FL: только {module: FL}, без общей тяги."""
    report = run_screen(tmp_path, payload(actions=[
        {"type": "wheel-drive", "id": "FL", "direction": 1, "percent": 6}]))
    assert report["drives"], report["netlog"]
    assert all(item["module"] == "FL" for item in report["drives"])
    assert report["drives"][0]["percent"] == 6
    # Отпустили — 0 именно этому колесу, а не «стоп всем».
    assert report["drives"][-1] == {"module": "FL", "percent": 0}
    assert all(item["percent"] in (0, 6) for item in report["drives"])


def test_two_wheels_drive_opposite_directions(tmp_path):
    """Разворот на месте: FL вперёд, FR назад — у каждого своя команда."""
    report = run_screen(tmp_path, payload(actions=[
        {"type": "wheel-drive", "id": "FL", "direction": 1, "keep": True},
        {"type": "wheel-drive", "id": "FR", "direction": -1, "keep": True},
        {"type": "stop"}]))
    per_wheel = [item for item in report["drives"] if item["module"]]
    assert {item["module"] for item in per_wheel} == {"FL", "FR"}
    assert any(item["module"] == "FL" and item["percent"] > 0 for item in per_wheel)
    assert any(item["module"] == "FR" and item["percent"] < 0 for item in per_wheel)
    # Общий СТОП гасит всё: одиночные нули каждому колесу и общая нулевая.
    assert {"module": "FL", "percent": 0} in report["drives"]
    assert {"module": "FR", "percent": 0} in report["drives"]
    # Общая нулевая тоже уходит: на плате это «b» — тормоз всем колёсам.
    assert any(item["module"] is None and item["percent"] == 0 for item in report["drives"])


def test_single_wheel_drive_is_blocked_when_that_wheel_is_not_ready(tmp_path):
    """Колесо без нуля не крутится одно — кнопки серые и ничего не отправляют."""
    ready = per_module_state()
    ready["RL"] = dict(ready["RL"], canDrive=False, reason="not_homed")
    report = run_screen(tmp_path, payload(per_module=ready, actions=[
        {"type": "wheel-drive", "id": "RL", "direction": 1},
        {"type": "wheel-drive", "id": "FL", "direction": 1}]))
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert cards["RL"]["backDisabled"] and cards["RL"]["forwardDisabled"]
    assert cards["RL"]["state"] == "Сначала откалибруйте колёса по датчику (или задайте 0° вручную)."
    assert cards["FL"]["forwardDisabled"] is False
    # Команда ушла только готовому колесу.
    assert {item["module"] for item in report["drives"]} == {"FL"}


def test_wheel_drive_stop_button_stops_only_that_wheel(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "wheel-drive", "id": "FR", "direction": 1, "keep": True},
        {"type": "wheel-stop", "id": "FR"}]))
    assert all(item["module"] == "FR" for item in report["drives"])
    assert report["drives"][-1] == {"module": "FR", "percent": 0}


def test_wheel_drive_refusal_stops_the_heartbeat_and_is_shown(tmp_path):
    """Отказ платы виден в карточке, и экран не повторяет его 5 раз в секунду."""
    report = run_screen(tmp_path, payload(driveRefuse="module_busy", actions=[
        {"type": "wheel-drive", "id": "FL", "direction": 1, "keep": True}]))
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert cards["FL"]["state"] == "Дождитесь остановки рулевых модулей."
    assert len(report["drives"]) == 1, report["drives"]


def test_wheel_drive_shows_reported_output_and_honest_rpm(tmp_path):
    """В карточке видно, что реально уходит на мотор; без тахометра — «rpm нет»."""
    motors = [motor(m) for m in MODULES]
    motors[0] = dict(motors[0], drivePercent=6, drivePulseUs=1650, tach=False, rpm=0.0)
    report = run_screen(tmp_path, payload(motors=motors, actions=[
        {"type": "wheel-drive", "id": "FL", "direction": 1}]))
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert "1650 мкс" in cards["FL"]["output"]
    assert "rpm нет" in cards["FL"]["output"]
    assert cards["FL"]["state"].startswith("вперёд")


def test_each_wheel_card_shows_its_own_direction(tmp_path):
    """Сторона вращения — по телеметрии: «обычное» / «зеркальное» у каждого колеса."""
    per_module = per_module_state()
    per_module["FL"] = dict(per_module["FL"], dirReverse=True)
    per_module["RR"] = dict(per_module["RR"], dirReverse=False)
    report = run_screen(tmp_path, payload(per_module=per_module))
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert cards["FL"]["dirState"] == "зеркальное"
    assert cards["FL"]["dirPressed"] == "true"
    assert cards["RR"]["dirState"] == "обычное"
    assert cards["RR"]["dirPressed"] == "false"
    assert all(not item["dirDisabled"] for item in cards.values())
    # Кнопка называет то, что сделает: у зеркального — «вернуть».
    assert cards["FL"]["dirLabel"] == "⇄ вернуть"
    assert cards["RR"]["dirLabel"] == "⇄ зеркалить"


def test_wheel_direction_toggle_sends_only_that_wheel(tmp_path):
    """«⇄» в карточке FR: одна адресная команда, без тяги и без остальных колёс."""
    report = run_screen(tmp_path, payload(actions=[{"type": "wheel-dir", "id": "FR"}]))
    assert report["renderErrors"] == [], report["renderErrors"]
    assert report["drives"] == [], "направление не должно включать тягу"
    assert report["driveCfgSent"] == [
        {"action": "direction", "value": "reverse", "module": "FR", "header": "steering"}]
    # Платы подтвердили — карточка показывает то, что прислала телеметрия,
    # а не то, что оператор попросил.
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert cards["FL"]["dirState"] == "обычное"


def test_wheel_direction_can_be_returned_to_normal(tmp_path):
    per_module = per_module_state()
    per_module["RL"] = dict(per_module["RL"], dirReverse=True)
    report = run_screen(tmp_path, payload(per_module=per_module,
                                          actions=[{"type": "wheel-dir", "id": "RL"}]))
    assert report["driveCfgSent"] == [
        {"action": "direction", "value": "normal", "module": "RL", "header": "steering"}]


def test_wheel_direction_is_blocked_while_that_wheel_is_not_ready(tmp_path):
    """На ходу сторону не меняют: кнопка серая и ничего не отправляет."""
    per_module = per_module_state()
    per_module["FR"] = dict(per_module["FR"], canSetDirection=False, directionReason="module_busy")
    report = run_screen(tmp_path, payload(per_module=per_module,
                                          actions=[{"type": "wheel-dir", "id": "FR"}]))
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert cards["FR"]["dirDisabled"] is True
    assert cards["FL"]["dirDisabled"] is False
    assert report["driveCfgSent"] == []


def test_wheel_direction_refusal_is_shown_in_the_card(tmp_path):
    """Отказ платы виден там, где нажали, а не «тихо ничего»."""
    report = run_screen(tmp_path, payload(driveCfgRefuse="module_busy",
                                          actions=[{"type": "wheel-dir", "id": "FL"}]))
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert cards["FL"]["state"] == "Сначала остановите это колесо: на ходу направление не меняется."


def test_all_presets_are_rendered_in_order(tmp_path):
    report = run_screen(tmp_path, payload())
    assert [item["id"] for item in report["presets"]] == [
        "straight", "veer-left", "veer-right", "crab-left", "crab-right",
        "side-left", "side-right", "pivot",
    ]
    # У каждой кнопки — иконка с четырьмя колёсами, название и подсказка.
    assert all(item["wheels"] == 4 and item["title"] and item["hint"] for item in report["presets"])
    # Разворот на месте одной тягой не делается: кнопка есть, но помечена недоступной.
    assert preset(report, "pivot")["unavailable"] is True
    assert preset(report, "pivot")["title"] == "Разворот"


def test_sliders_follow_the_limit_reported_by_the_server(tmp_path):
    report = run_screen(tmp_path, payload(limit=90))
    assert report["sharedLimit"] == 90
    assert report["masterSlider"]["min"] == "-90" and report["masterSlider"]["max"] == "90"
    assert all(item["min"] == "-90" and item["max"] == "90" for item in report["wheelSliders"])


def test_side_preset_sends_ninety_degrees_to_all_wheels(tmp_path):
    report = run_screen(tmp_path, payload(actions=[{"type": "preset", "id": "side-right"}]))
    assert angles_sent(report) == [(module, 90) for module in MODULES]
    assert all(item["header"] == "steering" for item in report["sent"])
    assert "Боком вправо" in report["status"]
    assert "в самом низу экрана" in report["status"]


def test_left_crab_preset_keeps_the_firmware_sign(tmp_path):
    report = run_screen(tmp_path, payload(actions=[{"type": "preset", "id": "crab-left"}]))
    assert angles_sent(report) == [(module, -45) for module in MODULES]
    assert "Краб влево" in report["status"]


def test_master_slider_applies_its_angle_to_every_wheel(tmp_path):
    report = run_screen(tmp_path, payload(actions=[{"type": "master", "angle": -75}]))
    assert angles_sent(report) == [(module, -75) for module in MODULES]


def test_wheel_slider_sends_one_command_on_release(tmp_path):
    report = run_screen(tmp_path, payload(actions=[{"type": "wheel", "id": "RL", "angle": 90}]))
    assert angles_sent(report) == [("RL", 90)]


def test_unavailable_pivot_sends_nothing_and_explains_why(tmp_path):
    """Пресет «Разворот» не включает тягу, а отправляет к одиночной тяге колёс."""
    report = run_screen(tmp_path, payload(actions=[{"type": "preset", "id": "pivot"}]))
    assert report["sent"] == []
    assert "по отдельности" in report["status"]


def test_old_firmware_limit_shrinks_sliders_and_blocks_wide_presets(tmp_path):
    """Плата с прежним ходом ±32°: экран не обещает больше, чем она примет."""
    report = run_screen(tmp_path, payload(limit=32, actions=[{"type": "preset", "id": "side-right"}]))
    assert report["sharedLimit"] == 32
    assert report["masterSlider"]["max"] == "32"
    assert all(item["max"] == "32" for item in report["wheelSliders"])
    for wide in ("crab-left", "crab-right", "side-left", "side-right"):
        assert preset(report, wide)["unavailable"] is True, wide
    assert preset(report, "straight")["unavailable"] is False
    assert preset(report, "veer-left")["unavailable"] is False
    # Нажатие недоступного пресета не отправляет команду, а объясняет причину.
    assert report["sent"] == []
    assert "±32°" in report["status"]


def test_preset_is_blocked_until_every_wheel_is_homed(tmp_path):
    motors = [motor("FL"), motor("FR"), motor("RL", homed=False), motor("RR")]
    report = run_screen(tmp_path, payload(motors=motors,
                                          actions=[{"type": "preset", "id": "straight"}]))
    assert report["sent"] == []
    assert "RL" in report["status"]
    assert all(item["disabled"] for item in report["presets"] if item["id"] != "pivot")


def test_preset_stops_at_the_first_refused_wheel(tmp_path):
    """FR отказал — остальные колёса не доворачиваются: робот не уходит в дугу."""
    report = run_screen(tmp_path, payload(
        steerFailOn="FR", steerFailReason="module_busy",
        actions=[{"type": "preset", "id": "side-left"}]))
    assert angles_sent(report) == [("FL", -90), ("FR", -90)]
    assert "Остановлено на FR" in report["status"]


def test_angle_beyond_the_limit_is_refused_by_the_server(tmp_path):
    report = run_screen(tmp_path, payload(actions=[{"type": "wheel", "id": "FL", "angle": 150}]))
    # Ползунок физически не даёт выставить больше предела: значение обрезано.
    assert angles_sent(report) == [("FL", 90)]


def test_drive_is_hold_to_move_and_release_to_stop(tmp_path):
    report = run_screen(tmp_path, payload(actions=[{"type": "drive", "direction": 1}]))
    percents = [item["percent"] for item in report["drives"]]
    assert percents[0] == 8, percents
    assert percents[-1] == 0, percents


def test_drive_refusal_is_shown_to_the_operator(tmp_path):
    report = run_screen(tmp_path, payload(
        can_drive=False, drive_reason="steering_misaligned",
        actions=[{"type": "drive", "direction": 1}]))
    assert report["drives"] == []
    assert "в одном направлении" in report["driveStatus"]


def test_presets_are_blocked_while_the_robot_is_moving(tmp_path):
    """На ходу руль не перекладывается: сначала СТОП, иначе робот уйдёт в дугу."""
    body = payload(actions=[{"type": "preset", "id": "crab-right"}])
    body["data"]["drive"].update(active=True, commandedPercent=10, canDrive=False,
                                 reason="drive_active")
    report = run_screen(tmp_path, body)
    assert report["sent"] == []
    assert "остановите тягу" in report["status"]
    assert all(item["disabled"] for item in report["presets"] if item["id"] != "pivot")


def test_stop_button_sends_zero_thrust(tmp_path):
    report = run_screen(tmp_path, payload(actions=[{"type": "drive", "direction": 1},
                                                   {"type": "stop"}]))
    percents = [item["percent"] for item in report["drives"]]
    # Пока кнопку держат, экран повторяет команду: прошивка сбрасывает тягу
    # через 650 мс без подтверждения. СТОП и отпускание дают ноль — и после
    # первого нуля никаких «8» быть не должно, иначе робот поедет сам.
    assert percents[0] == 8
    assert percents[-1] == 0
    first_zero = percents.index(0)
    assert percents[first_zero:] == [0] * (len(percents) - first_zero)


def test_wheel_that_stopped_short_does_not_hang_the_screen(tmp_path):
    """Колесо, вставшее в паре градусов от цели, не должно вешать экран.

    Раньше команда считалась исполненной только при попадании в ±1° от
    заданного угла. Если колесо останавливалось рядом, но не в цель (остаток
    после доводки по потенциометру, упор), экран держал его — а вместе с ним и
    все пресеты — заблокированным ровно 15 секунд, показывая лишь «команда
    отправлена». Так и получалось: «одно колесо зависает, потом рулится только
    отдельно».
    """
    frames = [
        [motor(module) for module in MODULES],
        [motor(module, angle=45.0, target=90.0, moving=True) for module in MODULES],
        # Три колеса доехали, а RR остановилось в 87,4° при цели 90°.
        [motor(module, angle=90.0, target=90.0) for module in MODULES[:3]]
        + [motor("RR", angle=87.4, target=90.0)],
    ]
    report = run_screen(tmp_path, payload(
        actions=[{"type": "preset", "id": "side-right", "rounds": 8}],
        motorFrames=frames,
    ))
    assert angles_sent(report) == [(module, 90) for module in MODULES]
    wheels = {item["id"]: item for item in report["wheels"]}
    # Как только колесо остановилось, им снова можно рулить.
    assert wheels["RR"]["sliderDisabled"] is False
    # И оператор видит, что угол не тот, а не молчаливую блокировку.
    assert "не доехало до 90" in wheels["RR"]["state"]
    assert "87.4" in wheels["RR"]["state"]
    # Одно недоехавшее колесо не выключает пресеты всему роботу.
    assert preset(report, "straight")["disabled"] is False
    assert preset(report, "side-left")["disabled"] is False
    # Доехавшим колёсам лишних сообщений не пишем.
    assert "не доехало" not in wheels["FL"]["state"]


def test_wheel_that_never_started_is_reported_and_unlocked(tmp_path):
    """Команда не дошла до платы: ждём недолго, потом говорим и отпускаем."""
    body = payload(actions=[{"type": "preset", "id": "side-right", "rounds": 24}],
                   virtualClock=True)
    report = run_screen(tmp_path, body)
    wheels = {item["id"]: item for item in report["wheels"]}
    assert wheels["RR"]["sliderDisabled"] is False
    assert "не доехало до 90" in wheels["RR"]["state"]
    assert "стоит на 0.0" in wheels["RR"]["state"]


def test_master_refusal_is_marked_as_a_warning_on_the_panel(tmp_path):
    """Отказ платы виден на панели руления как отказ, а не обычная строка.

    Сервер пишет «команда «w 90.0» не выполнена: <причина>», когда плата не
    подтвердила поворот. Именно это сообщение объясняет, почему колесо не
    поехало, — оно обязано выделяться, иначе теряется среди обычных строк.
    """
    body = payload()
    body["data"]["masterMessages"] = [
        {"ts": 1000, "text": "RR: порт колеса на связи"},
        {"ts": 1001, "text": "RR: команда «w 90.0» не выполнена: включена тяга — сначала b"},
    ]
    report = run_screen(tmp_path, body)
    html = report["masterMessages"]
    assert "не выполнена" in html
    assert 'data-warn="true"' in html
    # Обычное сообщение об связи остаётся без пометки.
    assert html.count('data-warn="true"') == 1


def with_drive_config(body, **overrides):
    """Подменить настройку привода в готовом ответе сервера."""
    body["data"]["drive"].update(overrides.pop("drive", {}))
    body["data"]["drive"]["config"] = drive_config(**overrides)
    return body


def test_drive_panel_shows_the_confirmed_drive_setup(tmp_path):
    """Веб-пульт и борт видят то же, что подтвердили платы: серво-ШИМ, без тахометра."""
    body = with_drive_config(
        payload(), tach=False, brake=False, stopInput=False, maxPercent=60,
        trimUs=1620, spanUs=300, pulseUs=1770)
    body["data"]["drive"]["maxPercent"] = 60
    body["data"]["motors"] = [dict(motor(m), tach=False, drivePulseUs=1770) for m in MODULES]
    report = run_screen(tmp_path, body)
    cfg = report["driveCfg"]
    assert cfg["tach"] == "off" and cfg["brake"] == "off" and cfg["stop"] == "off"
    assert cfg["limit"] == "60" and cfg["trim"] == "1620" and cfg["span"] == "300"
    assert cfg["applyDisabled"] is False
    assert "серво-ШИМ" in cfg["status"] and "нейтраль 1620 мкс" in cfg["status"]
    assert "тахометр нет" in cfg["status"] and "стоп нет" in cfg["status"]
    # Ползунок тяги идёт за платой, а не за зашитыми 20 %.
    assert report["driveSpeedMax"] == "60"
    # Без тахометра оборотов нет — карточка пишет «нет», а не 0.
    assert report["rpmText"] == ["нет"] * 4


def test_drive_setup_is_sent_as_individual_commands(tmp_path):
    """Настройка уходит по одной команде: семь подряд в порт — это очередь."""
    report = run_screen(tmp_path, with_drive_config(
        payload(actions=[{"type": "drivecfg", "trim": 1620, "span": 300, "tach": "off",
                          "brake": "on", "stop": "off", "limit": 35, "ppr": 60}])))
    sent = report["driveCfgSent"]
    # Нейтраль и размах — первыми: прошивка проверяет края каждой команды.
    assert [item["action"] for item in sent] == ["span", "trim", "ppr", "tach", "brake",
                                                 "stop", "limit"]
    assert sent[0]["value"] == 300
    assert sent[1]["value"] == 1620
    # Импульсов на оборот — отдельной командой m ppr: от неё зависят об/мин и путь.
    assert sent[2]["value"] == 60
    assert sent[3]["value"] == "off"
    assert sent[5]["value"] == "off"
    assert sent[6]["value"] == 35
    assert all(item["header"] == "steering" for item in sent)
    assert report["renderErrors"] == []


def test_drive_setup_is_blocked_while_the_traction_is_active(tmp_path):
    """Смена режима под работающим мотором — рывок: экран не отправляет её."""
    body = with_drive_config(payload(actions=[{"type": "drivecfg", "limit": 30}]),
                             maxPercent=20)
    body["data"]["drive"].update(active=True, commandedPercent=8, canDrive=False)
    report = run_screen(tmp_path, body)
    assert report["driveCfgSent"] == []
    assert report["driveCfg"]["applyDisabled"] is True
    assert "остановите тягу" in report["driveCfg"]["status"]


def test_drive_setup_is_honest_about_old_firmware(tmp_path):
    body = payload(actions=[{"type": "drivecfg", "limit": 40}])
    body["data"]["drive"]["config"] = {"supported": False, "outputMode": None, "tach": None,
                                       "maxPercent": None, "trimUs": None, "spanUs": None,
                                       "pulseUs": None, "brake": None, "stopInput": None,
                                       "dirReverse": None}
    report = run_screen(tmp_path, body)
    assert report["driveCfgSent"] == []
    assert report["driveCfg"]["applyDisabled"] is True
    assert "не умеет" in report["driveCfg"]["status"]


def test_drive_setup_refusal_is_explained(tmp_path):
    report = run_screen(tmp_path, with_drive_config(
        payload(actions=[{"type": "drivecfg", "limit": 30}], driveCfgRefuse="module_busy")))
    assert report["driveCfgSent"]
    assert "остановите тягу" in report["driveCfg"]["status"].lower()


def test_drive_setup_fits_the_span_instead_of_refusing_the_neutral(tmp_path):
    """Нейтраль 2000 плюс размах 500 — это 2500 мкс.

    Раньше экран молча отказывался отправлять такую настройку, и нейтраль нельзя
    было сдвинуть вообще. Теперь отправляется подрезанная пара: импульс остаётся
    в 1000…2000 мкс, а человек видит, что размах уменьшен.
    """
    report = run_screen(tmp_path, with_drive_config(
        payload(actions=[{"type": "drivecfg", "trim": 2000, "span": 500}])))
    sent = report["driveCfgSent"]
    actions = {item["action"]: item["value"] for item in sent}
    assert actions["trim"] == 1950
    assert actions["span"] == 50
    assert "размах уменьшен" in report["driveCfg"]["status"].lower()


def test_drive_panel_shows_pulses_per_rev_and_the_measured_rpm(tmp_path):
    """Импульсов на оборот видно на панели: именно оно делает об/мин правдой."""
    body = with_drive_config(payload(), pulsesPerRev=60)
    body["data"]["motors"] = [dict(motor(m), rpm=60.0) for m in MODULES]
    report = run_screen(tmp_path, body)
    cfg = report["driveCfg"]
    assert cfg["ppr"] == "60"
    assert "импульсов на оборот 60" in cfg["status"]
    # Рядом — то, что платы реально насчитали, чтобы было с чем сверять.
    assert "60 об/мин" in cfg["rpmNow"]
    assert cfg["fitDisabled"] is False


def test_drive_fit_button_recomputes_pulses_from_the_real_rpm(tmp_path):
    """«Один оборот в секунду — это 60 об/мин»: подбор даёт ppr 60 и уходит на платы."""
    body = with_drive_config(payload(actions=[
        {"type": "drivecfg-fit", "fact": 60},
        {"type": "drivecfg-apply"}]), pulsesPerRev=1)
    # Платы врут ровно как у оператора: 1 оборот в секунду показан как 3600.
    body["data"]["motors"] = [dict(motor(m), rpm=3600.0) for m in MODULES]
    report = run_screen(tmp_path, body)
    sent = report["driveCfgSent"]
    assert {"action": "ppr", "value": 60, "module": None, "header": "steering"} in sent
    assert report["driveCfg"]["ppr"] == "60"
    assert report["renderErrors"] == []


def test_drive_fit_works_while_the_wheel_is_spinning(tmp_path):
    """Подбор импульсов доступен под тягой: крутящееся колесо — единственный источник правды."""
    body = with_drive_config(payload(actions=[{"type": "drivecfg-fit", "fact": 60}]),
                             pulsesPerRev=1)
    body["data"]["drive"].update(active=True, commandedPercent=8, canDrive=False)
    body["data"]["motors"] = [dict(motor(m), rpm=3600.0) for m in MODULES]
    report = run_screen(tmp_path, body)
    cfg = report["driveCfg"]
    # Обычная настройка заблокирована, а подбор — нет.
    assert cfg["applyDisabled"] is True and cfg["fitDisabled"] is False
    assert [item["action"] for item in report["driveCfgSent"]] == ["ppr"]
    assert report["driveCfgSent"][0]["value"] == 60
    assert "отправлено платам" in cfg["status"]
    assert report["renderErrors"] == []


def test_drive_ppr_control_is_off_for_firmware_without_it(tmp_path):
    """Прошивка без ppr в телеметрии: поле выключено, команду m ppr не шлём."""
    body = with_drive_config(payload(actions=[{"type": "drivecfg", "limit": 30}]),
                             pulsesPerRev=None)
    report = run_screen(tmp_path, body)
    cfg = report["driveCfg"]
    assert cfg["fitDisabled"] is True
    assert [item["action"] for item in report["driveCfgSent"]] == \
        ["span", "trim", "tach", "brake", "stop", "limit"]
    # Без команд в статусе видно подтверждённую настройку — и что ppr прошивка
    # не сообщает, то есть обновлять его нечем.
    quiet = run_screen(tmp_path, with_drive_config(payload(), pulsesPerRev=None))
    assert "нет в прошивке" in quiet["driveCfg"]["status"]


def test_drive_setup_defaults_are_one_command(tmp_path):
    body = with_drive_config(payload(actions=[{"type": "drivecfg-reset"}]),
                             tach=False, maxPercent=60, trimUs=1620, spanUs=300)
    report = run_screen(tmp_path, body)
    assert [item["action"] for item in report["driveCfgSent"]] == ["defaults"]


# ------------------------------- манёвры ----------------------------------- #
def test_maneuver_numbers_come_from_the_server(tmp_path):
    """Числа пресетов — с сервера, а не зашиты в разметку."""
    body = payload()
    body["data"]["maneuverConfig"] = maneuver_config(
        presets={"forward": {"distanceM": 2.5, "percent": 15},
                 "spin_left": {"angleDeg": 180, "percent": 8}})
    report = run_screen(tmp_path, body)
    assert report["renderErrors"] == []
    buttons = {item["id"]: item for item in report["maneuvers"]["buttons"]}
    assert buttons["forward"]["target"] == "2.5"
    assert buttons["forward"]["percent"] == "15"
    assert buttons["spin_left"]["target"] == "180"
    assert buttons["spin_left"]["percent"] == "8"


def test_maneuver_saves_numbers_then_starts(tmp_path):
    """«Ехать» сначала сохраняет числа пресета, потом просит старт — в этом порядке."""
    report = run_screen(tmp_path, payload(actions=[
        {"type": "maneuver", "id": "forward", "target": 1.5, "percent": 10}]))
    sent = report["maneuversSent"]
    assert [item["action"] for item in sent] == ["config", "start"], sent
    assert sent[0]["name"] == "forward"
    assert sent[0]["values"] == {"distanceM": 1.5, "percent": 10}
    assert sent[1]["name"] == "forward"
    assert report["renderErrors"] == []


def test_maneuver_start_is_refused_before_numbers_are_saved(tmp_path):
    """Если числа не сохранились, старт не запрашивается вовсе."""
    report = run_screen(tmp_path, payload(
        maneuverConfigRefuse="invalid_value",
        actions=[{"type": "maneuver", "id": "spin_left", "target": 90}]))
    assert [item["action"] for item in report["maneuversSent"]] == ["config"]
    assert "числа" in report["maneuvers"]["status"].lower()


def test_maneuver_buttons_need_ready_drive(tmp_path):
    """Колёса не выровнены или нет PIN — кнопки манёвров серые."""
    report = run_screen(tmp_path, payload(can_drive=False, drive_reason="steering_misaligned"))
    assert all(item["disabled"] for item in report["maneuvers"]["buttons"])
    assert "в одном направлении" in report["maneuvers"]["status"]

    old = payload(motors=payload()["data"]["motors"])
    for module in MODULES:
        old["data"]["drive"]["perModule"][module]["canDrive"] = False
    report = run_screen(tmp_path, old)
    assert all(not item["disabled"] for item in report["maneuvers"]["buttons"])


def test_running_maneuver_shows_progress_and_stop(tmp_path):
    """Идущий манёвр: прогресс, чем меряем, и рабочая кнопка СТОП."""
    body = payload(maneuver={"supported": True, "active": True, "state": "driving",
                             "name": "forward", "title": "Вперёд", "kind": "distance",
                             "mode": "odometry", "progress": 0.42,
                             "value": 0.42, "target": 1.0, "percent": 12})
    report = run_screen(tmp_path, body)
    status = report["maneuvers"]["status"]
    assert "42%" in status and "0.42 м из 1.00 м" in status and "по тахометру" in status
    assert report["maneuvers"]["active"] == "true"
    assert report["maneuvers"]["stopDisabled"] is False
    buttons = {item["id"]: item for item in report["maneuvers"]["buttons"]}
    assert buttons["forward"]["label"] == "Едем…"
    assert all(item["disabled"] for item in report["maneuvers"]["buttons"])


def test_maneuver_without_tachometer_says_it_goes_by_time(tmp_path):
    body = payload(maneuver={"supported": True, "active": True, "state": "driving",
                             "name": "spin_left", "title": "Разворот влево", "kind": "angle",
                             "mode": "time", "progress": 0.5,
                             "value": 45.0, "target": 90.0, "percent": 10})
    report = run_screen(tmp_path, body)
    assert "по времени" in report["maneuvers"]["status"]
    assert "45°" in report["maneuvers"]["status"]


def test_maneuver_stop_button_sends_stop(tmp_path):
    body = payload(maneuver={"supported": True, "active": True, "state": "driving",
                             "name": "forward", "title": "Вперёд", "kind": "distance",
                             "mode": "odometry", "progress": 0.1,
                             "value": 0.1, "target": 1.0},
                   actions=[{"type": "maneuver-stop"}])
    report = run_screen(tmp_path, body)
    assert report["maneuversSent"] == [{"url": "api/kiosk_maneuver", "action": "stop",
                                        "name": None}]


def test_finished_maneuver_reports_the_result(tmp_path):
    body = payload(maneuver={"supported": True, "active": False, "state": "done",
                             "name": "forward", "title": "Вперёд", "kind": "distance",
                             "mode": "odometry", "progress": 1.0,
                             "value": 1.02, "target": 1.0,
                             "message": "Готово: 1.02 из 1.00."})
    report = run_screen(tmp_path, body)
    assert "1.02" in report["maneuvers"]["status"]
    assert report["maneuvers"]["stopDisabled"] is True


def test_maneuver_controls_exist_in_the_onboard_page():
    """Разметка борта: строки пресетов и СТОП манёвра на месте (не только в JS)."""
    page = (ROOT / "slam_gui" / "robot.html").read_text(encoding="utf-8")
    assert 'id="drive-maneuvers"' in page
    for name in ("forward", "backward", "left", "right", "spin_left", "spin_right"):
        assert f'id="maneuver-{name}"' in page
        assert f'id="maneuver-target-{name}"' in page
        assert f'id="maneuver-percent-{name}"' in page
    assert 'id="maneuver-stop"' in page and 'id="maneuver-status"' in page


def test_maneuver_controls_exist_in_the_web_remote():
    """Веб-пульт рисует те же строки пресетов из конфига сервера."""
    script = (ROOT / "slam_gui" / "steering.js").read_text(encoding="utf-8")
    assert "data-maneuver-start" in script and "data-maneuver-target" in script
    assert "rc-maneuver-stop" in script and "api/kiosk_maneuver" in script
    assert "api/kiosk_maneuver_config" in script
