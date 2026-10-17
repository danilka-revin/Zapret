"""Зум и вид карты лидара (slam_gui/lidar_view.js) — в node, без браузера.

Экраны рисуют карту через общий модуль RCLidarView. Здесь проверяется то, что
оператор ощущает при работе колесом, щипком и кнопками: точка под курсором не
уезжает, пределы зума соблюдаются, сброс возвращает обзор 10 м.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
SCRIPT = Path(__file__).resolve().parent / "js" / "lidar_view_zoom.js"
NODE = shutil.which("node")

pytestmark = pytest.mark.skipif(NODE is None, reason="нет node: модуль карты проверяется с ним")


def run_zoom_check() -> dict:
    proc = subprocess.run([NODE, str(SCRIPT)], capture_output=True, text=True, timeout=60, cwd=ROOT)
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout)


def test_wheel_zoom_keeps_point_under_cursor():
    result = run_zoom_check()
    assert result["wheelRadius"] < 10, result          # колесо вверх приблизило
    assert result["anchorDrift"] < 1e-6, result        # точка под курсором на месте


def test_buttons_zoom_and_limits_hold():
    result = run_zoom_check()
    assert result["zoomInRadius"] == pytest.approx(10 / 1.25)
    assert result["zoomBackRadius"] == pytest.approx(10)
    assert result["minRadius"] == pytest.approx(0.5)
    assert result["maxRadius"] == pytest.approx(40)


def test_reset_returns_default_view():
    result = run_zoom_check()
    assert result["resetRadius"] == 10
    assert result["resetPan"] == [0, 0]


def test_display_settings_are_sanitized():
    result = run_zoom_check()
    settings = result["settings"]
    assert settings["size"] == 0.5                      # верхняя граница Size
    assert settings["maxValue"] > settings["minValue"]  # Max Value не ниже Min Value
    assert settings["color"] == "range"                 # мусор — значение по умолчанию
    assert settings["minColor"] == "#ff5a36"            # не HEX — значение по умолчанию
    assert result["settingsReset"] == "range"
