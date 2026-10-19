"""Пресеты руления: общий модуль, бортовой экран и веб-пульт.

Проверяем то, что нельзя увидеть в обычном браузерном тесте без железа:
  * знаки углов совпадают с прошивкой (+ вправо, − влево);
  * «разворот на месте» честно помечен недоступным, а не имитируется;
  * экран киоска действительно содержит панель пресетов и общий ползунок;
  * общий модуль подключён и к киоску, и к пульту оператора.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
NODE = shutil.which("node")


@pytest.mark.skipif(NODE is None, reason="node нужен для проверки JS-модуля")
def test_steering_presets_signs_and_limits():
    result = subprocess.run(["node", "tests/js/steer_presets.js"], cwd=ROOT,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_kiosk_has_preset_panel_and_shared_module(kiosk_client):
    html = kiosk_client.get("/kiosk").get_data(as_text=True)
    js = kiosk_client.get("/robot.js").get_data(as_text=True)
    presets = kiosk_client.get("/steer_presets.js")

    assert presets.status_code == 200
    assert "RCSteerPresets" in presets.get_data(as_text=True)
    assert 'id="steer-preset-grid"' in html
    assert 'id="steer-master-slider"' in html
    assert 'id="steer-apply-all"' in html
    assert 'id="steer-hero-angle"' in html
    assert 'data-preview-wheel="FL"' in html
    assert "steer_presets.js" in html
    assert "api/kiosk_steer" in js
    assert "RCSteerPresets" in js
    assert "steerPresetBlockedReason" in js


def test_operator_pages_load_shared_presets():
    for name in ("main.html", "index.html"):
        text = (ROOT / "slam_gui" / name).read_text(encoding="utf-8")
        assert "steer_presets.js" in text, name
        assert "steering.js" in text, name
    overlay = (ROOT / "slam_gui" / "steering.js").read_text(encoding="utf-8")
    assert "rc-preset-grid" in overlay
    assert "api/kiosk_steer" in overlay
    assert "RCSteerPresets" in overlay
