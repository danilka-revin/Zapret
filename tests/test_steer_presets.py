"""Пресеты руления: общий модуль, бортовой экран и веб-пульт.

Проверяем то, что нельзя увидеть в обычном браузерном тесте без железа:
  * знаки углов совпадают с прошивкой (+ вправо, − влево);
  * предел хода руля — одно число на прошивку, сервер и экраны (иначе экран
    показывает один ход, а команда отклоняется: «руление работает криво»);
  * «разворот на месте» честно помечен недоступным, а не имитируется;
  * экран киоска действительно содержит панель пресетов и общий ползунок;
  * кнопки движения — в самом низу панели руления;
  * общий модуль подключён и к киоску, и к пульту оператора.
"""
from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
NODE = shutil.which("node")

#: Ход руля, который должен быть одинаковым во всём стеке.
STEER_LIMIT_DEG = 100


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _py_const(text: str, name: str) -> float:
    match = re.search(r"^%s\s*=\s*([0-9.]+)" % re.escape(name), text, re.MULTILINE)
    assert match, name
    return float(match.group(1))


def _ino_const(text: str, name: str) -> float:
    match = re.search(r"\bconst\s+[\w ]+?\b%s\s*=\s*([0-9.]+)f?" % re.escape(name), text)
    assert match, name
    return float(match.group(1))


def _js_const(text: str, name: str) -> float:
    match = re.search(r"\bconst\s+%s\s*=\s*([0-9.]+)" % re.escape(name), text)
    assert match, name
    return float(match.group(1))


@pytest.mark.skipif(NODE is None, reason="node нужен для проверки JS-модуля")
def test_steering_presets_signs_and_limits():
    result = subprocess.run(["node", "tests/js/steer_presets.js"], cwd=ROOT,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_steer_limit_is_one_number_across_the_stack():
    """Прошивка, драйвер, API и экраны ограничивают руль одним и тем же углом.

    Раньше число было записано в семи местах независимо: расхождение выглядело
    как «кривое руление» — ползунок шёл дальше, чем принимала плата.
    """
    firmware = [ROOT / "firmware" / "motor_wheel" / "motor_wheel.ino"]
    for path in firmware:
        text = _text(path)
        assert _ino_const(text, "STEER_LIMIT_DEG") == STEER_LIMIT_DEG, path.name
        # Предел команды меньше механического хода — колесо не упрётся в край.
        assert _ino_const(text, "STEER_TRAVEL_LIMIT_DEG") > STEER_LIMIT_DEG, path.name
        assert "±32" not in text, path.name

    assert _py_const(_text(ROOT / "slam_gui" / "backend.py"), "STEER_LIMIT_DEG") == STEER_LIMIT_DEG
    assert _py_const(_text(ROOT / "robot_control" / "slam.py"),
                     "STEERING_ANGLE_LIMIT_DEG") == STEER_LIMIT_DEG
    assert _py_const(_text(ROOT / "robot_control" / "rus_bridge.py"),
                     "STEER_LIMIT_DEG") == STEER_LIMIT_DEG
    assert _js_const(_text(ROOT / "slam_gui" / "steer_presets.js"), "LIMIT_DEG") == STEER_LIMIT_DEG

    for name in ("slam_gui/robot.js", "slam_gui/steering.js"):
        text = _text(ROOT / name)
        assert "Math.min(32" not in text and "-32, Math.min" not in text, name
        assert "±32" not in text, name

    # Ползунки в разметке сразу нарисованы с настоящим ходом, до ответа сервера.
    kiosk = _text(ROOT / "slam_gui" / "robot.html")
    assert kiosk.count('min="-100" max="100"') == 5
    assert 'min="-32"' not in kiosk
    console = _text(ROOT / "slam_gui" / "steering.js")
    assert console.count('min="-100" max="100"') == 2


def test_kiosk_movement_block_is_at_the_bottom_of_the_steering_panel():
    """Кнопки «Движение» — последний блок панели руления, как просил оператор."""
    html = _text(ROOT / "slam_gui" / "robot.html")
    panel = html[html.index('<section id="swipe-motors-view"'):]
    panel = panel[:panel.index("</section>\n    </div>")]

    order = [
        'id="steer-preset-grid"',          # пресеты
        'id="steer-master-slider"',        # общий угол
        'class="steer-hero"',              # схема колёс
        'class="steer-calib-bar"',         # калибровка
        'class="steer-fine"',              # точная настройка колёс
        'class="steer-footnote',           # сноска прямого режима
        'class="traction-controls"',       # движение — самое последнее
    ]
    positions = [panel.index(marker) for marker in order]
    assert positions == sorted(positions), order
    assert positions[-1] == max(positions)

    # Тексты панели больше не зовут «кнопками ниже» — движение в самом низу.
    assert "в самом низу экрана" in panel


def test_console_movement_block_is_below_the_wheel_cards():
    """В пульте оператора тяга — тоже внизу окна, под карточками колёс."""
    text = _text(ROOT / "slam_gui" / "steering.js")
    assert text.index('class="rc-steering-grid"') < text.index('class="rc-drive-panel"')
    assert text.index('id="rc-preset-grid"') < text.index('class="rc-steering-grid"')


def test_kiosk_steering_follows_the_server_limit():
    """Экран берёт предел из ответа сервера, а не из зашитого числа."""
    js = _text(ROOT / "slam_gui" / "robot.js")
    assert "syncSteerLimit(control)" in js
    assert "limitDeg" in js
    assert "data-steer-range-labels" in js
    # Схема колёс возвращается к живой телеметрии после ручного касания.
    assert js.count("scheduleSteerPreviewLive()") >= 3

    console = _text(ROOT / "slam_gui" / "steering.js")
    assert "syncLimit(control)" in console
    assert "data-rc-steer-limit" in console


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

