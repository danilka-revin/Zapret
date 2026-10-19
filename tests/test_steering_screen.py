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


def motor(module, *, angle=0.0, online=True, homed=True, moving=False):
    return {
        "id": module, "online": online, "homed": homed, "homeSource": "opto" if homed else "none",
        "angle": angle, "targetAngle": angle, "moving": moving, "calibrating": False,
        "cycling": False, "opto": False, "calibError": 0, "drivePercent": 0, "rpm": 0.0,
        "firmware": 3, "pot": True,
    }


def payload(*, limit=90.0, motors=None, available=True, can_drive=True, drive_reason=None,
            actions=None, **extra):
    data = {
        "linkOk": True,
        "steeringControl": {
            "available": available, "source": "master" if available else "none",
            "zeroMode": "opto", "optoUsed": True, "calibration": True,
            "manualZero": True, "limitDeg": limit,
        },
        "drive": {"available": available, "canDrive": can_drive, "reason": drive_reason,
                  "maxPercent": 20, "commandedPercent": 0, "active": False,
                  "alignmentToleranceDeg": 2.5},
        "motors": motors if motors is not None else [motor(m) for m in MODULES],
        "masterMessages": [],
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
    report = run_screen(tmp_path, payload(actions=[{"type": "preset", "id": "pivot"}]))
    assert report["sent"] == []
    assert "одна тяга" in report["status"]


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
    assert [item["percent"] for item in report["drives"]] == [8, 0, 0]
