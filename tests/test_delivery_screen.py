"""Экран доставки: разметка, ссылки и настоящий JS в минимальном DOM."""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_control.delivery_service import DeliveryService

ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tests" / "js" / "delivery_screen.js"
NODE = shutil.which("node")
DRAFT = ROOT / "docs" / "delivery_map_draft.json"


def test_delivery_html_has_no_real_drive_button():
    html = (ROOT / "slam_gui" / "delivery.html").read_text(encoding="utf-8")
    assert 'id="delivery-simulate"' in html
    assert 'id="delivery-start-drive"' not in html
    assert "ТЯГА ОТКЛЮЧЕНА" in html
    assert 'href="/delivery"' in (ROOT / "slam_gui" / "main.html").read_text(encoding="utf-8")


@pytest.mark.skipif(NODE is None, reason="нет node")
def test_delivery_screen_js_flow(tmp_path):
    service = DeliveryService(map_path=DRAFT, directory=tmp_path / "delivery")
    body = {"start": {"x": 22, "y": 140, "yawDeg": 80}, "goalNodeId": "ntc_delivery",
            "scenario": "clear", "draftAcknowledged": True}
    saved = service.save_setup(body)
    simulated = service.simulate()
    status = dict(simulated["status"], csrfToken="test-token")
    payload = {
        "map": service.map_payload(),
        "status": status,
        "plan": {"ok": True, "plan": saved["plan"], "status": status},
        "simulate": dict(simulated, status=status),
    }
    payload_file = tmp_path / "payload.json"
    payload_file.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    result = subprocess.run([NODE, str(HARNESS), str(payload_file)], cwd=str(ROOT),
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
    data = json.loads(result.stdout)
    assert data["goalOptions"] >= 4
    assert data["maneuvers"] > 0
    assert data["events"] > 0
    assert data["simulateDisabled"] is False
    assert data["simulationOk"] is True
    assert data["start"]["yawDeg"] == pytest.approx(80)
    simulate_calls = [call for call in data["calls"] if call["url"] == "/api/delivery/simulate"]
    assert simulate_calls
    assert simulate_calls[0]["init"]["headers"]["X-Robot-Control"] == "delivery"
    assert not any("/api/navigation/start" in call["url"] or "/api/kiosk_drive" in call["url"] for call in data["calls"])
