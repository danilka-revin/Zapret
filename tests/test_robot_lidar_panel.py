"""Карточка лидара на бортовом экране (slam_gui/robot.html, robot.css, robot.js).

Закреплено то, что оператор видит на борту:

* в карточке лидара остаются точки и дальность; «ближайшее» убрано;
* «дальность» — живой радиус обзора карты, а не статичные «10 м»;
* подпись «скана лидара нет» не перекрывает канву: без этого щипок и колесо
  до карты не доходят и приблизить пальцами нельзя.
"""

from __future__ import annotations

import re
from pathlib import Path

GUI = Path(__file__).resolve().parent.parent / "slam_gui"


def _read(name: str) -> str:
    return (GUI / name).read_text(encoding="utf-8")


def test_lidar_card_has_points_and_range_only():
    html = _read("robot.html")
    card = html[html.index('<section class="lidar"'):]
    card = card[:card.index("</section>")]
    assert 'id="lidar-count"' in card
    assert 'id="lidar-range"' in card
    assert "ближайшее" not in card
    assert 'id="lidar-near"' not in card


def test_robot_js_has_no_nearest_readout_and_updates_range():
    js = _read("robot.js")
    assert "lidar-near" not in js
    assert "'rc-lidar-view'" in js or '"rc-lidar-view"' in js
    assert "lidar-range" in js


def test_empty_hint_does_not_block_canvas():
    css = _read("robot.css")
    rule = re.search(r"\.lidar-empty\s*\{[^}]*\}", css)
    assert rule and "pointer-events: none" in rule.group(0)
    assert re.search(r"\.lidar-empty\[hidden\]\s*\{[^}]*display:\s*none", css)


def test_view_emits_radius_and_persists_zoom():
    js = _read("lidar_view.js")
    assert "new CustomEvent('rc-lidar-view'" in js
    assert "rc_lidar_pos_v1" in js
