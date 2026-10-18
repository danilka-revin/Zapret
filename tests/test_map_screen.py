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
* ошибка сервера не остаётся незамеченной — экран пишет об этом внизу панели;
* карта открывается только кнопкой: свайп с «камер и логов» её не открывает,
  а страница камер использует общую чёрно-красную тему;
* у карты есть тёмная и светлая тема, выбор сохраняется в localStorage.
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


def test_navigation_goal_pick_and_plan_preview(tmp_path):
    plan = {
        "ok": True,
        "path": [{"x": 1.25, "y": -0.5}, {"x": 1.0, "y": 0.0}],
        "goal": {"x": 1.0, "y": 0.0},
        "goalRequested": {"x": 1.0, "y": 0.0},
        "distanceM": 0.56,
        "waypoints": 1,
        "footprintM": 0.42,
    }
    payload = {
        "snapshot": snapshot(), "paths": PATHS, "navigationTest": True,
        "navigationPlanReply": {
            "ok": True, "plan": plan,
            "status": {
                "ok": True, "phase": "planned", "message": "Маршрут готов",
                "canStart": True, "enabled": True, "reason": "", "plan": plan,
                "progress": 0.0, "csrfToken": "signed-test-token",
            },
        },
    }
    report = run(tmp_path, payload)
    nav = report["navigation"]
    assert nav["goalX"] == "1.00" and nav["goalY"] == "0.00"
    assert "0.56 м" in nav["summary"]
    assert nav["phase"] == "маршрут готов"
    assert nav["startDisabled"] is False
    assert nav["planCalls"][0]["body"]["goal"] == {"x": 1, "y": 0}


def test_direct_map_click_builds_route_in_same_map(tmp_path):
    plan = {
        "ok": True,
        "path": [{"x": 1.25, "y": -0.5}, {"x": 1.0, "y": 0.0}],
        "goal": {"x": 1.0, "y": 0.0},
        "goalRequested": {"x": 1.0, "y": 0.0},
        "distanceM": 0.56,
        "waypoints": 1,
        "footprintM": 0.42,
    }
    payload = {
        "snapshot": snapshot(), "paths": PATHS,
        "navigationTest": True, "directClickTest": True,
        "navigationPlanReply": {
            "ok": True, "plan": plan,
            "status": {
                "ok": True, "phase": "planned", "message": "Маршрут готов",
                "canStart": True, "enabled": True, "reason": "", "plan": plan,
                "progress": 0.0, "csrfToken": "signed-test-token",
            },
        },
    }
    report = run(tmp_path, payload)
    nav = report["navigation"]
    assert nav["goalX"] == "1.00" and nav["goalY"] == "0.00"
    assert nav["routeWinGoalX"] == "1.00" and nav["routeWinGoalY"] == "0.00"
    assert nav["routeWindowOpen"] is False
    assert nav["pointCount"] == 1
    assert "0.56 м" in nav["summary"]
    assert nav["planCalls"][0]["body"]["goals"] == [{"x": 1, "y": 0}]
    assert report["renderErrors"] == [], report["renderErrors"]


# ----------------------------------------------------------- свайп-навигация ---
def run_map_swipe(tmp_path: Path) -> dict:
    """Прогон бортового экрана: свайп вправо на странице «камеры и логи»."""
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


def test_swipe_does_not_open_map_page(tmp_path):
    """Карта открывается только кнопкой: свайп с камер её не открывает."""
    states = run_map_swipe(tmp_path)["mapNavigation"]
    assert len(states) == 2, states
    # 1) открыта страница «камеры и логи»
    assert states[0]["open"] is True
    assert states[0]["cameras"] is True and states[0]["map"] is False
    assert states[0]["title"] == "Камеры и логи"
    assert "карта" not in states[0]["hint"].lower()
    # 2) свайп вправо просто закрывает страницу — карта так и не появилась
    assert states[1]["open"] is False
    assert states[1]["map"] is False, states[1]
    assert states[1]["title"] == "Камеры и логи"


def test_map_view_stays_available_for_functions_button(kiosk_client):
    """Страница карты остаётся в разметке: её открывает кнопка «Функции»."""
    page = kiosk_client.get("/kiosk").get_data(as_text=True)
    assert 'id="swipe-map-view"' in page
    assert 'id="btn-map-view"' in page
    # Отдельная кнопка «Карта» в шапке ведёт на самостоятельную страницу.
    assert 'id="k-map-open" class="k-ctrl-btn" href="/map"' in page


def test_camera_page_uses_common_black_red_theme():
    """У «камер и логов» нет отдельной синей темы — общий чёрно-красный экран."""
    css = (ROOT / "slam_gui" / "robot.css").read_text(encoding="utf-8")
    assert "#swipe-page.is-camera-page" not in css
    assert "#91c8ff" not in css
    assert ".swipe-camera-view { color: var(--text); }" in css


def test_map_has_dark_and_light_theme_switch(tmp_path):
    """Карта переключается между тёмной и светлой, выбор сохраняется."""
    report = run(tmp_path, {"snapshot": snapshot(), "paths": PATHS})
    assert report["themeBefore"]["attr"] == "dark"
    assert report["themeBefore"]["label"] == "Светлая карта"
    assert report["themeBefore"]["pressed"] == "false"
    assert report["themeAfter"]["attr"] == "light"
    assert report["themeAfter"]["label"] == "Тёмная карта"
    assert report["themeAfter"]["pressed"] == "true"
    css = (ROOT / "slam_gui" / "map.css").read_text(encoding="utf-8")
    assert ':root[data-map-theme="light"]' in css
    script = (ROOT / "slam_gui" / "map_view.js").read_text(encoding="utf-8")
    assert "kiosk_map_theme" in script


@pytest.mark.parametrize("render_failure", [False, True])
def test_functions_map_button_opens_before_rendering(tmp_path, render_failure):
    payload_file = tmp_path / "map_opening.json"
    payload_file.write_text(json.dumps({
        "ok": True, "data": {}, "mapOpening": True,
        "mapRenderFailure": render_failure,
    }), encoding="utf-8")
    result = subprocess.run([NODE, str(SWIPE_HARNESS), "robot", str(payload_file)],
                            cwd=ROOT, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(result.stdout)
    for index, state in enumerate(report["mapOpening"], 1):
        assert state["open"] is True
        assert state["hidden"] is False
        assert state["mapHidden"] is False
        assert state["overlayHidden"] is True
        assert state["title"] == "Карта лидара"
        assert state["shows"] == index
        if render_failure:
            assert "test canvas unavailable" in state["message"]


def test_route_points_can_be_reordered_removed_and_cleared(tmp_path):
    report = run(tmp_path, {"snapshot": snapshot(), "paths": PATHS,
                            "navigationTest": True, "multiPointTest": True})
    edited = report["navigation"]["editedRoute"]
    assert edited["added"] == 2
    assert edited["invalidated"] is True
    assert "2.00 / 0.20" in edited["reordered"][0]
    assert edited["removed"] == 1
    assert edited["cleared"] == 0
    assert report["renderErrors"] == []


def test_map_reads_actual_independent_wheel_telemetry(tmp_path):
    report = run(tmp_path, {"snapshot": snapshot(), "paths": PATHS, "robotState": {
        "ok": True, "data": {"linkOk": True, "source": "master", "speedMps": 0.2,
                              "drive": {"active": True},
                              "motors": [{"id": id_, "angle": angle, "targetAngle": 80,
                                          "online": True, "homed": True}
                                         for id_, angle in zip(["FL", "FR", "RL", "RR"],
                                                               [20, -15, 8, 0])]}}})
    model = report["robotModel"]
    assert [w["angle"] for w in model["wheels"]] == [20, -15, 8, 0]
    assert model["driving"] is True
    assert any(call["url"] == "api/state" for call in report["calls"])
    assert all(call["method"] == "GET" for call in report["calls"]
               if 'steer' in call["url"] or 'drive' in call["url"])
    assert report["renderErrors"] == []
