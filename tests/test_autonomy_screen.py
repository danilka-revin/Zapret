"""Экран /autonomy: разметка, ссылки и настоящий JS в минимальном DOM."""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tests" / "js" / "autonomy_screen.js"
NODE = shutil.which("node")


def status_payload() -> dict:
    checks = {key: True for key in (
        "enabled", "estopClear", "visionOk", "visionRunning", "camerasFresh",
        "conesVisible", "corridorOk", "geometryCalibrated", "driveReady",
        "steerReady", "chassisLink", "wheelsOnline", "depthFresh", "imuFresh",
        "rivalsIdle", "standstill")}
    checks["blindZone"] = False
    return {
        "ok": True, "active": False, "enabled": True, "simulation": False,
        "config": {"lookahead_m": 2.5}, "phase": "idle",
        "message": "Автономность не запущена.", "distanceM": 0.0,
        "percent": 0.0, "steerLeftDeg": 0.0, "boostPercent": 0.0,
        "counters": {"stop": 0, "bump": 0, "traffic": 0, "powerBoosts": 0},
        "cones": [{"x": 1.0, "y": -0.8}, {"x": 1.0, "y": 0.8},
                  {"x": 2.4, "y": -0.8}, {"x": 2.4, "y": 0.8}],
        "signs": [], "corridor": {"ok": True, "centerM": 0.05,
                                  "centerAheadM": 0.1, "widthM": 1.6,
                                  "leftCount": 2, "rightCount": 2},
        "traffic": {"visible": False, "state": "none", "hold": False},
        "ground": {"ok": False, "nearestBumpM": None, "status": "unknown"},
        "imu": {"fresh": True, "pitch": 0.0, "roll": 0.0, "gyroZ": 0.0},
        "events": [{"atS": 1.0, "distanceM": 0.0, "phase": "idle",
                    "message": "Вкладка открыта", "tone": "info"}],
        "blockers": [], "warnings": ["Перед пуском осмотрите зону"],
        "checks": checks, "canStart": True, "requireBlindCheck": True,
        "csrfToken": "test-token",
    }


def test_autonomy_html_links_and_safety_labels():
    html = (ROOT / "slam_gui" / "autonomy.html").read_text(encoding="utf-8")
    assert 'id="aut-canvas"' in html and 'id="aut-start"' in html
    assert 'id="aut-stop"' in html and 'id="aut-estop"' in html
    assert 'id="aut-blind"' in html and 'id="aut-simulate"' in html
    assert "Сухой прогон" in html and "без железа и тяги" in html
    assert "ждём 3 с" in html and "только на зелёный" in html
    assert "ждём 3 с" in (ROOT / "docs" / "TRACK_AUTONOMY.md").read_text(encoding="utf-8")
    main = (ROOT / "slam_gui" / "main.html").read_text(encoding="utf-8")
    robot = (ROOT / "slam_gui" / "robot.html").read_text(encoding="utf-8")
    index = (ROOT / "slam_gui" / "index.html").read_text(encoding="utf-8")
    assert 'href="/autonomy" id="nav-autonomy"' in main
    assert 'id="k-autonomy-open"' in robot and 'href="/autonomy"' in robot
    assert 'id="btn-autonomy-view"' in robot
    assert 'id="console-autonomy-link"' in index
    env = (ROOT / ".env.example").read_text(encoding="utf-8")
    for key in ("RC_TRACK_ENABLE=0", "RC_TRACK_STOP_WAIT_S=3.0",
                "RC_TRACK_MAX_DISTANCE_M=200", "RC_TRACK_REQUIRE_DEPTH=1"):
        assert key in env
    docs = (ROOT / "docs" / "TRACK_AUTONOMY.md").read_text(encoding="utf-8")
    assert "navigation_frame()" in docs and "notForNavigation" in docs
    assert "уже влито в эту ветку" in docs and "PR #158" in docs
    module_doc = (ROOT / "robot_control" / "track_autonomy.py").read_text(encoding="utf-8")
    assert "влитого из ``main`` / PR #158" in module_doc
    assert "дорабатывается на соседней ветке" not in module_doc


@pytest.mark.skipif(NODE is None, reason="нет node")
def test_autonomy_screen_js_flow(tmp_path):
    simulate = {
        "ok": True, "scenario": "bump", "phase": "done", "steps": 400,
        "message": "Трасса пройдена", "distanceM": 44.1, "elapsedS": 85.4,
        "counters": {"stop": 0, "bump": 1, "traffic": 0, "powerBoosts": 3},
        "events": [
            {"atS": 42.4, "distanceM": 21.1, "phase": "bump_climb",
             "message": "Переезжаю кочку", "tone": "warn"},
            {"atS": 45.6, "distanceM": 22.0, "phase": "bump_exit",
             "message": "Кочка пройдена", "tone": "ok"},
        ],
        "commands": 441, "maxBoostPercent": 9.0,
    }
    payload_file = tmp_path / "payload.json"
    payload_file.write_text(json.dumps({"status": status_payload(),
                                        "simulate": simulate}, ensure_ascii=False),
                            encoding="utf-8")
    result = subprocess.run([NODE, str(HARNESS), str(payload_file)], cwd=str(ROOT),
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
    data = json.loads(result.stdout)
    assert data["beforeBlind"] is True and data["afterBlind"] is False
    assert data["checks"] >= 15 and data["warnings"] >= 1
    assert data["events"] >= 1 and data["simEvents"] >= 2
    assert data["canvasCalls"] > 0
    assert data["csrf"] == "test-token"
    assert "api/vision/camera/0/frame" in data["cameraSrc"]
    assert data["phase"] == "ОСТАНОВЛЕНО" and data["badge"] == "ОСТАНОВЛЕНО"

    start = data["startCalls"][0]
    assert start["init"]["method"] == "POST"
    assert start["init"]["headers"]["X-Robot-Control"] == "autonomy"
    assert start["init"]["headers"]["X-CSRF-Token"] == "test-token"
    assert json.loads(start["init"]["body"])["blindZoneChecked"] is True
    assert data["stopCalls"][0]["init"]["headers"]["X-Robot-Control"] == "autonomy"
    sim_call = data["simulateCalls"][0]
    assert json.loads(sim_call["init"]["body"])["scenario"] == "bump"
    assert sim_call["init"]["headers"]["X-CSRF-Token"] == "test-token"
    assert data["estopCalls"][0]["init"]["headers"]["X-Robot-Control"] == "drive"
    assert not any("/api/kiosk_drive" in call["url"] or
                   "/api/navigation/start" in call["url"] for call in data["calls"])
