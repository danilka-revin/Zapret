"""Страница «Карта лидара» бортового экрана: node-прогон slam_gui/map_view.js.

Скрипт страницы написан под браузер, поэтому проверяется через
``tests/js/map_screen.js``: поднимается минимальный DOM, скрипт выполняется
как в странице, ``fetch`` отдаёт заготовленные ответы API карты, а результат
читается из тех же элементов, которые видит оператор.

Закреплено то, что ломалось по-настоящему:

* карта приезжает с сервера и заполняет статистику (размер, сетка, изучено,
  препятствия, поза, сканов, путь к файлу карты);
* плашка MAPPING/IDLE соответствует состоянию карты;
* кнопка сброса зовёт ``api/kiosk_map/reset`` (несуществующий ``api/map/reset``
  экран раньше звал молча);
* сохранение зовёт ``api/kiosk_map/save``;
* настройки уходят на сервер (``api/kiosk_map/settings``) вместе с выбранной
  чувствительностью и дальностью;
* ошибка сервера не остаётся незамеченной — экран пишет об этом внизу панели.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
HARNESS = Path(__file__).resolve().parent / "js" / "map_screen.js"
SWIPE_HARNESS = Path(__file__).resolve().parent / "js" / "lidar_screen.js"
NODE = shutil.which("node")

pytestmark = pytest.mark.skipif(
    NODE is None, reason="нет node: экранные скрипты проверяются только с ним")

PATHS = {
    "dir": "/home/robot/Desktop",
    "pgm": "/home/robot/Desktop/rus_slam_map.pgm",
    "yaml": "/home/robot/Desktop/rus_slam_map.yaml",
    "png": "/home/robot/Desktop/rus_slam_map.png",
}

SETTINGS = {
    "resolution": 0.1,
    "size_m": 30.0,
    "max_range": 8.0,
    "min_range": 0.3,
    "hit": 0.65,
    "miss": 0.3,
    "decay": 0.0,
    "matching": True,
    "autosave": True,
    "autosave_s": 60.0,
}


def snapshot(scans: int = 42, occupied: int = 310) -> dict:
    """Ответ GET /api/kiosk_map — как отдаёт сервер карты."""
    return {
        "ok": True,
        "settings": SETTINGS,
        "stats": {
            "width": 300, "height": 300, "cells": 90000,
            "free": 9000, "occupied": occupied, "unknown": 77000,
            "known": 10000, "coverage": 0.1111, "sizeM": 30.0,
            "resolution": 0.1,
        },
        "pose": {"x": 1.25, "y": -0.5, "yaw": 0.5236},
        "origin": {"x": -15.0, "y": -15.0},
        "quality": 0.9,
        "matched": 30,
        "scans": scans,
        "points": 15120,
        "updatedAt": 1_800_000_000.0,
        "savedAt": 1_800_000_000.0,
        "paths": PATHS,
        "resolution": 0.1,
        "width": 300,
        "height": 300,
        "png": "",
    }


def run(tmp_path: Path, payload: dict) -> dict:
    payload_file = tmp_path / "map_payload.json"
    payload_file.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    result = subprocess.run(
        [NODE, str(HARNESS), str(payload_file)],
        cwd=str(ROOT), capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, \
        "страница карты упала:\n%s\n%s" % (result.stdout, result.stderr)
    return json.loads(result.stdout)


def test_map_page_shows_stats_and_file(tmp_path):
    report = run(tmp_path, {"snapshot": snapshot(), "paths": PATHS})
    stats = report["stats"]
    assert stats["size"] == "30.0 × 30.0 м"
    assert "0.10" in stats["resolution"] and "300×300" in stats["resolution"]
    assert "11.1" in stats["coverage"]
    assert stats["occupied"] == "310"
    assert "1.25" in stats["pose"] and "30°" in stats["pose"]
    assert stats["scans"] == "42 · точек 15120"
    assert stats["file"] == "файл: " + PATHS["pgm"]
    assert "90" in stats["quality"]
    assert report["renderErrors"] == [], report["renderErrors"]


def test_map_badge_switches_to_mapping(tmp_path):
    report = run(tmp_path, {"snapshot": snapshot(), "paths": PATHS})
    assert report["badge"] == "MAPPING"
    assert report["badgeTone"] == "live"
    assert report["emptyHidden"] is True

    report = run(tmp_path, {"snapshot": snapshot(scans=0, occupied=0), "paths": PATHS})
    assert report["badge"] == "IDLE"
    assert report["emptyHidden"] is False


def test_reset_button_calls_map_reset_api(tmp_path):
    report = run(tmp_path, {"snapshot": snapshot(), "paths": PATHS})
    urls = [call["url"] for call in report["calls"]]
    assert any("kiosk_map/reset" in url for url in urls), urls
    # старый адрес api/map/reset на сервере не существует — экран его не зовёт
    assert not any(url.endswith("api/map/reset") for url in urls), urls
    reset_calls = [c for c in report["calls"] if "kiosk_map/reset" in c["url"]]
    assert reset_calls[0]["method"] == "POST"
    assert "файл" in report["afterReset"]["msg"]
    assert report["afterReset"]["tone"] == "ok"


def test_save_button_calls_map_save_api(tmp_path):
    report = run(tmp_path, {"snapshot": snapshot(), "paths": PATHS})
    saves = [c for c in report["calls"] if "kiosk_map/save" in c["url"]]
    assert saves, report["calls"]
    assert saves[0]["method"] == "POST"
    assert "сохран" in report["afterSave"]["msg"].lower()


def test_settings_are_sent_to_server(tmp_path):
    report = run(tmp_path, {"snapshot": snapshot(), "paths": PATHS})
    settings_calls = [c for c in report["calls"] if "kiosk_map/settings" in c["url"]]
    assert settings_calls, report["calls"]
    sent = settings_calls[0]["body"]["settings"]
    assert sent["max_range"] == 6
    assert sent["hit"] == 0.85 and sent["miss"] == 0.45   # пресет «жёстко»
    assert report["afterApply"]["tone"] in ("ok", "warn")


def test_server_error_is_reported_to_operator(tmp_path):
    payload = {
        "snapshot": snapshot(),
        "paths": PATHS,
        "reset": {"ok": False, "error": "диск занят"},
    }
    report = run(tmp_path, payload)
    assert "не удалось" in report["afterReset"]["msg"].lower()
    assert report["afterReset"]["tone"] == "err"


# ----------------------------------------------------------- свайп-навигация ---
def swipe_map(tmp_path: Path) -> dict:
    """Прогон бортового экрана: свайп вправо с «камер и логов» на карту."""
    payload_file = tmp_path / "swipe_payload.json"
    payload_file.write_text(json.dumps({"ok": True, "data": {}, "mapSwipe": True}),
                             encoding="utf-8")
    result = subprocess.run(
        [NODE, str(SWIPE_HARNESS), "robot", str(payload_file)],
        cwd=str(ROOT), capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, \
        "бортовой экран упал:\n%s\n%s" % (result.stdout, result.stderr)
    return json.loads(result.stdout)


def test_swipe_right_from_cameras_opens_map_page(tmp_path):
    """Страница карты открывается свайпом вправо после «камер и логов»."""
    states = swipe_map(tmp_path)["mapNavigation"]
    assert len(states) == 3, states
    # 1) открыта страница «камеры и логи»
    assert states[0]["cameras"] is True and states[0]["map"] is False
    assert states[0]["title"] == "Камеры и логи"
    # 2) свайп вправо — карта лидара
    assert states[1]["map"] is True, states[1]
    assert states[1]["cameras"] is False
    assert states[1]["title"] == "Карта лидара"
    assert states[1]["direction"] == "right"
    assert "карта" in states[1]["hint"].lower()
    # 3) свайп влево — назад к камерам и логам
    assert states[2]["cameras"] is True and states[2]["map"] is False
    assert states[2]["title"] == "Камеры и логи"


def test_map_page_is_after_cameras_in_markup(kiosk_client):
    """В разметке карта стоит сразу после «камер и логов»."""
    page = kiosk_client.get("/kiosk").get_data(as_text=True)
    cameras = page.index('id="swipe-cameras-view"')
    map_view = page.index('id="swipe-map-view"')
    motors = page.index('id="swipe-motors-view"')
    assert cameras < map_view < motors
