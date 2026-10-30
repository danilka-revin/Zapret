"""Руление на бортовом экране: пресеты, предел угла и команды — без браузера.

``slam_gui/robot.js`` написан под браузер и ничего не экспортирует, поэтому
экран выполняется в минимальном DOM (``tests/js/steering_screen.js``, тот же
приём, что для карты лидара), а проверяется то, что видит и делает оператор:

  * какие пресеты нарисованы, какие нажимаемы и что они отправляют;
  * ход ползунков берётся из ответа сервера (``steeringControl.limitDeg``),
    а не зашит в разметку — иначе экран разрешает угол, который плата режет;
  * пресет, который железо не тянет, честно недоступен и ничего не отправляет;
  * отказ на одном колесе останавливает пресет, а не доворачивает остальные;
  * движение с фиксацией: нажал → тяга, отпустил → тяга, СТОП → 0;
  * изменение ползунка на ходу передаёт новое точное значение.
"""
from __future__ import annotations

import json
import re
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
        "firmware": 14, "pot": True, "tach": True, "driveMaxPercent": 30,
        "driveDutyPermille": 0, "driveServoValue": 0,
        "tachHz": 90,
    }


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
             "dutyPermille": 0, "rpm": 0.0, "tach": True,
             "dirReverse": False, "canSetDirection": True, "directionReason": None}
    state.update(overrides)
    return {module: dict(state) for module in MODULES}


def payload(*, limit=90.0, motors=None, available=True, can_drive=True, drive_reason=None,
            actions=None, drive=None, per_module=None, servo=None, maneuver=None, **extra):
    data = {

        "linkOk": True,
        "steeringControl": {
            "available": available, "source": "master" if available else "none",
            "zeroMode": "opto", "optoUsed": True, "calibration": True,
            "manualZero": True, "limitDeg": limit,
        },
        "drive": {"available": available, "canDrive": can_drive, "reason": drive_reason,
                  "maxPercent": 30, "commandedPercent": 0, "active": False,
                  "individual": True, "commandedModules": {},
                  "perModule": per_module if per_module is not None else per_module_state(),
                  "servo": servo if servo is not None else {
                      "maxValue": 150, "canDrive": available, "reason": None,
                      "perModule": {m: {"canDrive": available, "reason": None} for m in MODULES}},
                  "reportedDuty": {}, "alignmentToleranceDeg": 2.5,
                  # Колёса стоят ровно: справка для подписи, тягу не запирает.
                  "aligned": True, "spreadDeg": 0.0,
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


def test_each_wheel_card_has_its_own_direct_slider(tmp_path):
    report = run_screen(tmp_path, payload())
    assert report["renderErrors"] == []
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert sorted(cards) == MODULES
    assert all(not item["sliderDisabled"] for item in cards.values())
    assert all(item["sliderMin"] == "0" and item["sliderMax"] == "150" for item in cards.values())
    assert all(item["sliderValue"] == "0" and item["state"] == "стоит" for item in cards.values())


def test_single_wheel_continues_after_button_is_released(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "wheel-drive", "id": "FL", "value": 40, "release": True},
        {"type": "wait", "rounds": 12}]))
    assert len(report["drives"]) > 1, report["netlog"]
    assert all(item == {"module": "FL", "value": 40, "direction": "forward"}
               for item in report["drives"])
    card = next(c for c in report["wheelDrive"] if c["id"] == "FL")
    assert card["sliderValue"] == "40" and card["forwardPressed"] == "true"
    assert card["backDisabled"]


def test_two_wheels_have_independent_values_and_global_stop(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "wheel-drive", "id": "FL", "value": 40},
        {"type": "wheel-drive", "id": "FR", "value": 150},
        {"type": "stop"}]))
    assert {"module": "FL", "value": 40, "direction": "forward"} in report["drives"]
    assert {"module": "FR", "value": 150, "direction": "forward"} in report["drives"]
    assert report["drives"][-1] == {"module": None, "value": 0}
    assert {item["id"]: item["sliderValue"] for item in report["wheelDrive"]} == {"FL": "40", "FR": "150", "RL": "0", "RR": "0"}


def test_only_offline_wheel_slider_is_blocked(tmp_path):
    states = {m: {"canDrive": True, "reason": None} for m in MODULES}
    states["RL"] = {"canDrive": False, "reason": "module_offline"}
    servo = {"maxValue": 150, "canDrive": False, "reason": "module_offline", "perModule": states}
    report = run_screen(tmp_path, payload(servo=servo, actions=[
        {"type": "wheel-drive", "id": "RL", "value": 75},
        {"type": "wheel-drive", "id": "FL", "value": 40}]))
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert cards["RL"]["sliderDisabled"] and not cards["FL"]["sliderDisabled"]
    assert {item["module"] for item in report["drives"]} == {"FL"}
    assert all(item["value"] == 40 for item in report["drives"])


def test_wheel_drive_stop_button_stops_only_that_wheel(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "wheel-drive", "id": "FR", "value": 75},
        {"type": "wheel-stop", "id": "FR"}]))
    assert all(item["module"] == "FR" for item in report["drives"])
    assert report["drives"][-1] == {"module": "FR", "value": 0}


def test_wheel_drive_refusal_stops_heartbeat_and_is_shown(tmp_path):
    report = run_screen(tmp_path, payload(driveRefuse="servo_firmware_required", actions=[
        {"type": "wheel-drive", "id": "FL", "value": 40}]))
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert "v15" in cards["FL"]["state"]
    assert report["drives"][0] == {"module": "FL", "value": 40, "direction": "forward"}
    assert report["drives"][-1] == {"module": "FL", "value": 0}
    assert sum(item["value"] != 0 for item in report["drives"]) == 1


def test_common_panel_shows_each_boards_reported_write_value_not_rpm(tmp_path):
    motors = [dict(motor(module), driveServoValue=value, rpm=None, tach=False)
              for module, value in zip(MODULES, [2, 150, 75, 40])]
    report = run_screen(tmp_path, payload(motors=motors))
    assert report["driveReported"] == (
        "Платы сообщили Servo.write: FL: 2 · FR: 150 · RL: 75 · RR: 40 (не обороты)")


def test_common_panel_does_not_label_stale_values_as_fresh(tmp_path):
    motors = [dict(motor(module), driveServoValue=150) for module in MODULES]
    states = {m: {"canDrive": True, "reason": None} for m in MODULES}
    states["FR"] = {"canDrive": False, "reason": "module_offline"}
    servo = {"maxValue": 150, "canDrive": False, "reason": "module_offline", "perModule": states}
    report = run_screen(tmp_path, payload(motors=motors, servo=servo))
    assert "FL: 150 · FR: — · RL: 150" in report["driveReported"]


def test_sc_rpm_is_shown_on_chassis_common_and_wheel_cards_only(tmp_path):
    motors = [dict(motor(module), firmware=16, rpm=None, tach=False, scRpm=value)
              for module, value in zip(MODULES, [60.2, 120.0, 0.0, 15.5])]
    report = run_screen(tmp_path, payload(motors=motors))
    assert report["rpmText"] == ["60.2", "120.0", "0.0", "15.5"]
    assert report["driveRpmReported"] == (
        "Обороты SC, об/мин: FL: 60.2 · FR: 120.0 · RL: 0.0 · RR: 15.5")
    assert [card["rpm"] for card in report["wheelDrive"]] == report["rpmText"]
    assert report["drives"] == [], "показания не запускают моторы"
    assert not report["driveSpeedDisabled"]


def test_rpm_readout_hides_stale_and_missing_values_without_disabling_drive(tmp_path):
    motors = [dict(motor(module), firmware=16, rpm=None, tach=False, scRpm=70.0)
              for module in MODULES]
    motors[1]["online"] = False
    motors[2]["scRpm"] = None
    motors[3]["scRpm"] = "bad"
    report = run_screen(tmp_path, payload(motors=motors, actions=[{"type": "drive", "value": 40}]))
    assert report["rpmText"] == ["70.0", "—", "—", "—"]
    assert [card["rpm"] for card in report["wheelDrive"]] == report["rpmText"]
    assert report["drives"][0] == {"module": None, "value": 40, "direction": "forward"}


def test_rpm_readouts_do_not_remain_frozen_when_server_is_unreachable(tmp_path):
    motors = [dict(motor(module), firmware=16, rpm=None, tach=False, scRpm=70.0)
              for module in MODULES]
    body = payload(motors=motors, virtualClock=True,
                   actions=[{"type": "server-offline", "rounds": 20}])
    report = run_screen(tmp_path, body)
    assert report["dataChip"] == "НЕТ СВЯЗИ С СЕРВЕРОМ"
    assert report["rpmText"] == ["—"] * 4
    assert [card["rpm"] for card in report["wheelDrive"]] == ["—"] * 4
    assert report["driveRpmReported"] == "Обороты SC, об/мин: FL: — · FR: — · RL: — · RR: —"

    body["actions"].append({"type": "server-online", "rounds": 4})
    restored = run_screen(tmp_path, body)
    assert restored["dataChip"] == "в реальном времени"
    assert restored["rpmText"] == ["70.0"] * 4


def test_wheel_drive_shows_actual_servo_value_without_rpm(tmp_path):
    motors = [motor(m) for m in MODULES]
    motors[0] = dict(motors[0], driveServoValue=40, tach=True, rpm=95.0)
    report = run_screen(tmp_path, payload(motors=motors))
    card = next(c for c in report["wheelDrive"] if c["id"] == "FL")
    assert "Servo.write(40)" in card["output"]
    assert "об/мин" not in card["output"]
    assert card["state"] == "Servo.write(40)"


def test_wheel_cards_have_movement_buttons_but_no_mirror_setting(tmp_path):
    """Вперёд/назад доступны, а зеркальность по-прежнему задана платой."""
    report = run_screen(tmp_path, payload())
    cards = {item["id"]: item for item in report["wheelDrive"]}
    for card in cards.values():
        assert not card["backDisabled"] and not card["forwardDisabled"]
        assert "dirState" not in card and "dirLabel" not in card
    assert not report["driveBackDisabled"] and not report["driveForwardDisabled"]
    assert report["driveCfgSent"] == []


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


@pytest.mark.parametrize("module", [None, "FL", "FR", "RL", "RR"])
def test_slider_by_itself_never_starts_motion(tmp_path, module):
    report = run_screen(tmp_path, payload(actions=[{"type": "speed", "id": module, "value": 75}]))
    assert report["drives"] == []
    if module is None:
        assert report["driveSpeedValue"] == "75"
    else:
        assert next(card for card in report["wheelDrive"] if card["id"] == module)["sliderValue"] == "75"


@pytest.mark.parametrize("direction", ["forward", "backward"])
@pytest.mark.parametrize("value", [1, 2, 40, 75, 150])
@pytest.mark.parametrize("module", [None, "FL", "FR", "RL", "RR"])
def test_clicked_direction_keeps_exact_value_until_stop(tmp_path, direction, value, module):
    kind = "drive" if module is None else "wheel-drive"
    stop = {"type": "stop"} if module is None else {"type": "wheel-stop", "id": module}
    report = run_screen(tmp_path, payload(actions=[
        {"type": kind, "id": module, "direction": direction, "value": value, "release": True},
        {"type": "wait", "rounds": 12}, stop]))
    expected = {"module": module, "value": value, "direction": direction}
    assert len(report["drives"]) > 2
    assert report["drives"][0] == expected
    assert report["drives"][-1] == {"module": module, "value": 0}
    assert all(item == expected for item in report["drives"][:-1])


def test_common_speed_is_not_replaced_by_an_individual_wheel_setting(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "speed", "value": 40},
        {"type": "speed", "id": "FL", "value": 150},
        {"type": "press", "direction": "backward"},
        {"type": "stop"}]))
    assert report["drives"][0] == {"module": None, "value": 40, "direction": "backward"}
    assert report["drives"][-1] == {"module": None, "value": 0}
    assert report["driveSpeedValue"] == "40"
    assert next(w for w in report["wheelDrive"] if w["id"] == "FL")["sliderValue"] == "40"


def test_reversing_requires_stop_and_uses_the_same_saved_speed(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "speed", "value": 40},
        {"type": "press", "direction": "forward"},
        {"type": "release", "direction": "forward"},
        {"type": "press", "direction": "backward"},  # отключено до СТОП
        {"type": "stop"},
        {"type": "press", "direction": "backward"},
        {"type": "stop"}]))
    assert report["drives"][0] == {"module": None, "value": 40, "direction": "forward"}
    first_stop = next(i for i, item in enumerate(report["drives"]) if item["value"] == 0)
    assert all(item.get("direction") == "forward" for item in report["drives"][:first_stop])
    assert {"module": None, "value": 40, "direction": "backward"} in report["drives"][first_stop + 1:]
    assert sum(item["value"] == 0 for item in report["drives"]) == 2
    assert report["drives"][-1] == {"module": None, "value": 0}


@pytest.mark.parametrize("direction", ["forward", "backward"])
@pytest.mark.parametrize("module", [None, "FL", "FR", "RL", "RR"])
def test_slider_updates_exact_value_during_latched_motion(tmp_path, module, direction):
    kind = "drive" if module is None else "wheel-drive"
    stop = {"type": "stop"} if module is None else {"type": "wheel-stop", "id": module}
    report = run_screen(tmp_path, payload(actions=[
        {"type": kind, "id": module, "value": 2, "direction": direction},
        {"type": "release", "id": module, "direction": direction},
        {"type": "speed", "id": module, "value": 150},
        {"type": "speed", "id": module, "value": 2},
        {"type": "wait", "rounds": 6}, stop]))
    expected_2 = {"module": module, "value": 2, "direction": direction}
    expected_150 = {"module": module, "value": 150, "direction": direction}
    assert report["drives"][0] == expected_2
    high = report["drives"].index(expected_150)
    assert expected_2 in report["drives"][high + 1:-1]
    assert all(item["value"] != 0 for item in report["drives"][:-1])
    assert report["drives"][-1] == {"module": module, "value": 0}
    if module is None:
        assert report["driveSpeedValue"] == "2"
    else:
        assert next(card for card in report["wheelDrive"] if card["id"] == module)["sliderValue"] == "2"


def test_keyboard_release_and_pointer_cancel_do_not_stop_a_run(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "speed", "value": 75},
        {"type": "key", "direction": "backward", "event": "keydown"},
        {"type": "key", "direction": "backward", "event": "keyup"},
        {"type": "release", "direction": "backward", "cancel": True},
        {"type": "wait", "rounds": 8}, {"type": "stop"}]))
    expected = {"module": None, "value": 75, "direction": "backward"}
    assert len(report["drives"]) > 2
    assert all(item == expected for item in report["drives"][:-1])
    assert report["drives"][-1] == {"module": None, "value": 0}
    assert report["driveSpeedValue"] == "75"


@pytest.mark.parametrize("value", [1, 2, 40, 75, 150])
def test_common_slider_sends_exact_servo_value(tmp_path, value):
    report = run_screen(tmp_path, payload(actions=[{"type": "drive", "value": value}]))
    expected = {"module": None, "value": value, "direction": "forward"}
    assert report["drives"][0] == expected
    assert all(item == expected for item in report["drives"])
    assert report["driveSpeedMin"] == "0" and report["driveSpeedMax"] == "150"


def test_zero_speed_does_not_arm_a_direction_or_auto_start(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "press", "direction": "forward"},
        {"type": "speed", "value": 40}, {"type": "wait", "rounds": 6}]))
    assert report["drives"] == []
    assert report["driveForwardPressed"] == "false"


def test_slider_zero_stops_and_needs_a_new_direction_click(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "drive", "value": 40}, {"type": "speed", "value": 0},
        {"type": "speed", "value": 150}, {"type": "wait", "rounds": 6}]))
    assert report["drives"][0] == {"module": None, "value": 40, "direction": "forward"}
    first_zero = next(i for i, item in enumerate(report["drives"]) if item["value"] == 0)
    assert all(item["value"] == 0 for item in report["drives"][first_zero:])
    assert report["driveSpeedValue"] == "150" and report["driveForwardPressed"] == "false"


def test_common_slider_refuses_old_firmware(tmp_path):
    states = {m: {"canDrive": True, "reason": None} for m in MODULES}
    states["RR"] = {"canDrive": False, "reason": "servo_firmware_required"}
    servo = {"maxValue": 150, "canDrive": False, "reason": "servo_firmware_required", "perModule": states}
    report = run_screen(tmp_path, payload(servo=servo, actions=[{"type": "drive", "value": 40}]))
    assert report["drives"] == [] and report["driveSpeedDisabled"]
    assert "v15" in report["driveStatus"]


def test_direct_slider_ignores_legacy_steering_and_tach_readiness(tmp_path):
    motors = [motor(m, homed=False, moving=True) for m in MODULES]
    for item in motors:
        item.update(tach=False, rpm=0)
    report = run_screen(tmp_path, payload(motors=motors, can_drive=False,
                                          drive_reason="not_homed",
                                          actions=[{"type": "drive", "value": 75}]))
    assert report["drives"][0] == {"module": None, "value": 75, "direction": "forward"}
    assert not report["driveSpeedDisabled"]


def test_blur_stops_direct_drive_without_resuming(tmp_path):
    report = run_screen(tmp_path, payload(actions=[{"type": "drive", "value": 40}, {"type": "blur"}]))
    assert report["drives"][0] == {"module": None, "value": 40, "direction": "forward"}
    assert report["drives"][-1] == {"module": None, "value": 0}
    first_zero = next(i for i, item in enumerate(report["drives"]) if item["value"] == 0)
    assert all(item["value"] == 0 for item in report["drives"][first_zero:])
    assert report["driveSpeedValue"] == "40"


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
    report = run_screen(tmp_path, payload(actions=[{"type": "drive", "value": 40}, {"type": "stop"}]))
    values = [item["value"] for item in report["drives"]]
    assert values[0] == 40 and values[-1] == 0
    first_zero = values.index(0)
    assert all(value == 0 for value in values[first_zero:])


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


def test_drive_panel_has_no_settings_at_all(tmp_path):
    """Настроек тяги нет ни в разметке, ни в запросах: драйвер ZS-Z11H их не имеет."""
    report = run_screen(tmp_path, payload())
    assert report["driveCfgSent"] == []
    assert "driveCfg" not in report
    assert report["renderErrors"] == []
    # Прямой предел независим от прежнего процентного лимита платы.
    assert report["driveSpeedMax"] == "150"
    assert "нажмите" in report["driveStatus"]
    # Пока показывается только значение Servo, а не обороты или импульсы SC.
    cards = {item["id"]: item for item in report["wheelDrive"]}
    assert cards["FL"]["output"] == "Servo.write(0)"


def test_drive_screen_has_no_drive_config_controls():
    """В самой странице нет ни настроек привода, ни кнопки прошивки плат."""
    html = (ROOT / "slam_gui" / "robot.html").read_text(encoding="utf-8")
    for gone in ("drive-cfg-trim", "drive-cfg-span", "drive-cfg-ppr", "drive-cfg-tach",
                 "drive-cfg-brake", "drive-cfg-stop", "drive-cfg-limit", "drive-cfg-apply",
                 "drive-cfg-reset", "drive-setup-grid", "data-drive-module-dir",
                 # прошивки плат из программы больше нет: только Arduino IDE
                 "drive-flash", "Прошить модули", "kiosk_firmware_flash"):
        assert gone not in html, gone


# ------------------------------- манёвры ----------------------------------- #
def test_geometry_markup_has_measured_defaults(tmp_path):
    report = run_screen(tmp_path, payload())
    assert report["geometry"]["wheelDiameterMm"] == "215.9"
    assert report["geometry"]["wheelbaseMm"] == "500"
    assert report["geometry"]["trackMm"] == "380"
    assert report["maneuversSent"] == []


def test_geometry_form_displays_saved_values_without_automatic_overwrite(tmp_path):
    body = payload()
    body["data"]["maneuverConfig"] = maneuver_config(chassis={
        "wheelDiameterMm": 207.5, "wheelbaseMm": 500, "trackMm": 450,
    })
    report = run_screen(tmp_path, body)
    assert report["geometry"]["wheelDiameterMm"] == "207.5"
    assert report["geometry"]["wheelbaseMm"] == "500"
    assert report["geometry"]["trackMm"] == "450"
    assert report["maneuversSent"] == []
    assert report["renderErrors"] == []


def test_geometry_change_requires_explicit_save_before_a_maneuver(tmp_path):
    body = payload(actions=[
        {"type": "geometry", "values": {"trackMm": 380}, "save": False},
        {"type": "maneuver", "id": "spin_left"},
    ])
    body["data"]["maneuverConfig"] = maneuver_config(chassis={
        "wheelDiameterMm": 207.5, "wheelbaseMm": 500, "trackMm": 450,
    })
    report = run_screen(tmp_path, body)
    assert report["geometry"]["trackMm"] == "380"  # телеметрия не затёрла правку
    assert report["maneuversSent"] == []
    assert "сохраните" in report["maneuvers"]["status"].lower()
    assert report["renderErrors"] == []

    body["actions"] = [{"type": "geometry", "values": {"trackMm": 380}}]
    report = run_screen(tmp_path, body)
    assert report["maneuversSent"] == [{
        "url": "api/kiosk_maneuver_config", "action": "config", "name": "chassis",
        "values": {"wheelDiameterMm": 207.5, "wheelbaseMm": 500, "trackMm": 380},
    }]
    assert report["geometry"]["trackMm"] == "380"
    assert "сохранены" in report["geometry"]["status"].lower()
    assert report["renderErrors"] == []


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


def test_maneuver_buttons_stay_pressable_without_ready_drive(tmp_path):
    """Пресеты доступны всегда: нет связи с платами — кнопки не гаснут.

    Нажатие «Ехать» всё равно уходит на сервер, а причина видна словами под
    списком. Углы колёс манёвры не блокируют давно, теперь не блокирует и
    отсутствие телеметрии: оператор видит не серую кнопку, а объяснение.
    """
    report = run_screen(tmp_path, payload(can_drive=False, drive_reason="module_offline"))
    buttons = report["maneuvers"]["buttons"]
    assert buttons and all(not item["disabled"] for item in buttons), buttons
    status = report["maneuvers"]["status"]
    assert "телеметрии" in status
    assert "«Ехать» покажет точную причину" in status
    assert report["maneuvers"]["stopDisabled"] is False

    angled = payload()
    angled["data"]["drive"].update(aligned=False, spreadDeg=45.0)
    report = run_screen(tmp_path, angled)
    assert all(not item["disabled"] for item in report["maneuvers"]["buttons"])

    old = payload(motors=payload()["data"]["motors"])
    for module in MODULES:
        old["data"]["drive"]["perModule"][module]["canDrive"] = False
    report = run_screen(tmp_path, old)
    assert all(not item["disabled"] for item in report["maneuvers"]["buttons"])


def test_pressing_to_go_when_drive_is_not_ready_shows_the_reason(tmp_path):
    """Кнопка не серая: нажатие спрашивает сервер и печатает его отказ словами."""
    report = run_screen(tmp_path, payload(
        can_drive=False, drive_reason="module_offline",
        maneuverRefuse="module_offline",
        actions=[{"type": "maneuver", "id": "forward", "target": 1.0, "percent": 10}]))
    sent = report["maneuversSent"]
    assert [item["action"] for item in sent] == ["config", "start"], sent
    status = report["maneuvers"]["status"]
    assert "свежая телеметрия" in status, status


def test_maneuvers_are_separate_and_never_greyed_in_the_markup():
    """Манёвры свёрнуты и не мешают простому ползунку; кнопки доступны."""
    page = (ROOT / "slam_gui" / "robot.html").read_text(encoding="utf-8")
    detail = re.search(r'<details[^>]*id="drive-maneuvers"[^>]*>', page)
    assert detail and "open" not in detail.group(0), detail and detail.group(0)
    for name in ("forward", "backward", "left", "right", "spin_left", "spin_right"):
        button = re.search(r'<button[^>]*id="maneuver-%s"[^>]*>' % name, page)
        assert button and "disabled" not in button.group(0), button and button.group(0)
    stop = re.search(r'<button[^>]*id="maneuver-stop"[^>]*>', page)
    assert stop and "disabled" not in stop.group(0), stop and stop.group(0)


def test_running_maneuver_shows_progress_and_stop(tmp_path):
    """Идущий манёвр: прогресс, чем меряем, и рабочая кнопка СТОП."""
    body = payload(maneuver={"supported": True, "active": True, "state": "driving",
                             "name": "forward", "title": "Вперёд", "kind": "distance",
                             "mode": "odometry", "sensor": "sc_count", "progress": 0.42,
                             "value": 0.42, "target": 1.0, "percent": 12})
    report = run_screen(tmp_path, body)
    status = report["maneuvers"]["status"]
    assert "42%" in status and "0.42 м из 1.00 м" in status and "по счётчикам SC" in status
    assert report["maneuvers"]["active"] == "true"
    assert report["maneuvers"]["stopDisabled"] is False
    buttons = {item["id"]: item for item in report["maneuvers"]["buttons"]}
    assert buttons["forward"]["label"] == "Едем…"
    # Идущий манёвр тоже не гасит пресеты: повторное нажатие получает честный
    # ответ «другой манёвр уже идёт — сначала СТОП», а не мёртвую кнопку.
    assert all(not item["disabled"] for item in report["maneuvers"]["buttons"])


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
    # СТОП манёвра доступен всегда — даже когда манёвр уже закончился.
    assert report["maneuvers"]["stopDisabled"] is False


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


def test_web_remote_also_drives_at_any_angle():
    """Веб-пульт и борт говорят одно и то же: угол колёс тягу не запирает."""
    console = (ROOT / "slam_gui" / "steering.js").read_text(encoding="utf-8")
    assert "spreadDeg" in console and "поедет под этим углом" in console
    # Блокировка кнопок движения смотрит только на готовность привода, не на углы.
    assert "button.disabled = !authorizedForDrive || !drive.canDrive;" in console


def test_drive_heartbeats_expire_but_stop_never_expires(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "drive", "value": 40}, {"type": "wait", "rounds": 8}, {"type": "stop"}]))
    assert len(report["driveDeadlines"]) == len(report["drives"])
    assert all(float(deadline) > 0 for deadline in report["driveDeadlines"][:-1])
    assert report["driveDeadlines"][-1] is None


def test_stop_overtakes_queued_heartbeat_after_delayed_request(tmp_path):
    report = run_screen(tmp_path, payload(deferFirstDrive=True, actions=[
        {"type": "drive", "value": 40}, {"type": "stop"}, {"type": "finish-drive"}]))
    assert report["drives"] == [{"module": None, "value": 40, "direction": "forward"},
                                {"module": None, "value": 0}]
    assert report["driveSpeedValue"] == "40"


def test_lost_active_wheel_does_not_restart_after_connection_returns(tmp_path):
    report = run_screen(tmp_path, payload(actions=[
        {"type": "wheel-drive", "id": "FL", "value": 40},
        {"type": "servo-offline", "id": "FL"},
        {"type": "servo-online", "id": "FL"}]))
    values = [item["value"] for item in report["drives"]]
    assert values[0] == 40 and values[-1] == 0
    first_zero = values.index(0)
    assert all(value == 0 for value in values[first_zero:])
    assert {card["id"]: card["sliderValue"] for card in report["wheelDrive"]} == {"FL": "40", "FR": "0", "RL": "0", "RR": "0"}
