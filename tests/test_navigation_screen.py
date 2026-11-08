"""Отдельный экран навигации: навигация ушла с панели карты.

Проверяется то, что видит оператор:

* на ``/map`` больше нет панели навигации (точек, «Построить маршрут»,
  «Поехать», E-STOP) — там остались карта, её настройки и автообследование;
* ``/navigation`` отдаётся и на пульте, и на борту, все три шага на месте,
  а статика (``navigation.css``/``navigation_screen.js``) доступна;
* экран навигации действительно управляет тем же ``map_view.js``: клик по
  карте ставит цель, кнопка «Построить маршрут» уходит на сервер, «Поехать»
  открывается только по ``canStart``;
* проверки готовности показывают, что именно мешает ехать.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from conftest import login

ROOT = Path(__file__).resolve().parent.parent
HARNESS = ROOT / "tests" / "js" / "navigation_screen.js"
NODE = shutil.which("node")

# Элементы панели навигации, которые убраны с экрана карты.
MAP_PANEL_MARKERS = (
    'id="nav-goal-x"', 'id="nav-goal-y"', 'id="nav-plan"', 'id="nav-start"',
    'id="nav-cancel"', 'id="nav-estop"', 'id="nav-point-list"',
    'id="nav-add-point"', 'id="nav-route-summary"',
)

# Что должно быть на отдельном экране навигации.
SCREEN_MARKERS = (
    'id="map-canvas"', 'id="nav-goal-x"', 'id="nav-goal-y"', 'id="nav-plan"',
    'id="nav-start"', 'id="nav-resume"', 'id="nav-cancel"', 'id="nav-estop"',
    'id="nav-route-summary"', 'id="navscreen-check-list"', 'id="navscreen-blockers"',
)


def _html(name: str) -> str:
    return (ROOT / "slam_gui" / name).read_text(encoding="utf-8")


# --------------------------------------------------------------- страницы ---
def test_map_page_has_no_navigation_panel(kiosk_client, client, operator) -> None:
    """С экрана карты навигация убрана: шаги живут в отдельном окне."""
    login(client)
    for web in (kiosk_client, client):
        page = web.get("/map").text
        for marker in MAP_PANEL_MARKERS:
            assert marker not in page, marker
        # Вместо панели — ссылка на отдельный экран навигации.
        assert 'id="map-open-navigation" href="/navigation"' in page
        # Карта, её настройки и автообследование остаются как были.
        for marker in ('id="map-canvas"', 'id="map-settings"',
                       'id="explore-start"', 'id="explore-nav"'):
            assert marker in page, marker


def test_navigation_screen_is_served_on_both_sides(kiosk_client, client, operator) -> None:
    login(client)
    for web in (kiosk_client, client):
        response = web.get("/navigation")
        assert response.status_code == 200
        page = response.text
        # Три шага в правильном порядке: цель → маршрут → поездка.
        order = [page.index(marker) for marker in
                 ('data-navstep="goal"', 'data-navstep="plan"', 'data-navstep="drive"')]
        assert order == sorted(order)
        for marker in SCREEN_MARKERS:
            assert marker in page, marker
        for name in ("navigation.css", "navigation_screen.js"):
            asset = web.get("/" + name)
            assert asset.status_code == 200, (name, asset.status_code)


def test_navigation_page_explains_why_it_cannot_drive() -> None:
    """Экран сам объясняет причину: «почему нельзя ехать» не нужно угадывать."""
    html = _html("navigation.html")
    assert 'id="navscreen-checks-list"' not in html
    assert 'id="navscreen-check-list"' in html
    assert 'id="navscreen-checks-state"' in html
    assert 'id="navscreen-blockers"' in html
    # Шаги подписаны по-человечески, а не «nav-start / nav-plan».
    for label in ("Куда ехать", "Построить маршрут", "Поехали"):
        assert label in html, label
    script = _html("navigation_screen.js")
    for key in ("masterReady", "estopClear", "scanReady", "poseReady"):
        assert key in script, key


def test_navigation_panel_removed_from_kiosk_map_view() -> None:
    """Бортовая «Функции → Карта» тоже без панели навигации."""
    robot = _html("robot.html")
    assert 'id="nav-point-list"' not in robot
    assert 'id="nav-estop"' not in robot
    assert 'id="map-nav-screen" href="/navigation"' in robot
    # Кнопка-иконка «Навигация» в шапке киоска ведёт на отдельный экран.
    start = robot.index('id="k-route-open"')
    assert 'href="/navigation"' in robot[start:start + 400]
    assert 'aria-label="Навигация: цель, маршрут, поездка"' in robot[start:start + 400]


def test_main_menu_links_the_navigation_screen() -> None:
    main = _html("main.html")
    assert 'href="/navigation" id="nav-navigation"' in main
    # Порядок меню: карта → скан → навигация (карта строится первой).
    assert main.index('id="nav-map"') < main.index('id="nav-scan"') < main.index('id="nav-navigation"')


# ----------------------------------------------------------------- node ---
pytestmark_node = pytest.mark.skipif(
    NODE is None, reason="нет node: экранные скрипты проверяются только с ним")

SNAPSHOT = {
    "ok": True,
    "settings": {"resolution": 0.1, "size_m": 30.0},
    "stats": {"width": 300, "height": 300, "coverage": 0.25},
    "pose": {"x": 1.25, "y": -0.5, "yaw": 0.5236},
    "scans": 42,
    "width": 300,
    "height": 300,
    "resolution": 0.1,
}

PLAN = {
    "path": [{"x": 1.25, "y": -0.5}, {"x": 2.0, "y": 0.0}],
    "goal": {"x": 2.0, "y": 0.0},
    "goalRequested": {"x": 2.0, "y": 0.0},
    "goalsRequested": [{"x": 2.0, "y": 0.0}],
    "distanceM": 0.89,
    "waypoints": 1,
    "footprintM": 0.42,
}


def _checks(**overrides: bool) -> dict:
    base = {key: True for key in (
        "enabled", "speedCalibrated", "speedWithinLimit", "masterReady",
        "driveStopped", "mapFresh", "mapReady", "mapTrusted", "mapIntegrity",
        "scanReady", "poseReady", "mapDecayDisabled", "footprintSufficient",
        "estopClear")}
    base.update(overrides)
    return base


def run(tmp_path: Path, payload: dict) -> dict:
    payload_file = tmp_path / "navigation_payload.json"
    payload_file.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    result = subprocess.run([NODE, str(HARNESS), str(payload_file)],
                            cwd=str(ROOT), capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, \
        "экран навигации упал:\n%s\n%s" % (result.stdout, result.stderr)
    return json.loads(result.stdout)


def payload_blocked() -> dict:
    return {
        "snapshot": SNAPSHOT,
        "navigationStatus": {
            "ok": True, "phase": "idle", "message": "Ожидание цели",
            "canStart": False, "canResume": False, "enabled": False,
            "reason": "Автономная тяга выключена. Для включения задайте RC_NAV_ENABLE=1 после проверки системы.",
            "plan": None,
            "diagnostics": {"checks": _checks(enabled=False, scanReady=False)},
        },
        "navigationPlanReply": {
            "ok": True, "plan": PLAN,
            "status": {"ok": True, "phase": "planned", "message": "Маршрут готов",
                       "canStart": True, "enabled": False, "reason": "", "plan": PLAN,
                       "diagnostics": {"checks": _checks()}},
        },
    }


@pytestmark_node
def test_screen_shows_what_blocks_the_start(tmp_path: Path) -> None:
    report = run(tmp_path, payload_blocked())
    initial = report["initial"]
    assert initial["checksState"] == "мешают: 2"
    bad = [row for row in initial["checks"] if not row["ok"]]
    assert len(bad) == 2
    texts = " ".join(row["text"] for row in bad)
    assert "Автономная тяга разрешена" in texts
    assert "Скан полный" in texts
    assert "RC_NAV_ENABLE=1" in initial["blockers"]
    assert initial["startDisabled"] is True
    assert initial["steps"] == {"goal": "todo", "plan": "todo", "drive": "todo"}
    assert "Шаг 1" in initial["reason"]
    # Карта и поза подписаны по-человечески, а не «—».
    assert "300 × 300" in initial["mapinfo"] and "25%" in initial["mapinfo"]
    assert "1.25" in initial["posemeta"] and "сканов: 42" in initial["posemeta"]
    assert report["renderErrors"] == [], report["renderErrors"]


@pytestmark_node
def test_map_click_sets_one_goal_and_plan_opens_the_drive_step(tmp_path: Path) -> None:
    report = run(tmp_path, payload_blocked())
    click = report["afterClick"]
    # Клик по карте — это цель, а не точка маршрута (как было на /map).
    assert click["goalX"] == "0.95" and click["goalY"] == "0.00"
    assert "цель: X 0.95" in click["note"]
    assert click["planCalls"] == 1
    assert click["steps"]["goal"] == "done"
    assert "Шаг 2" in click["reason"]

    plan = report["afterPlan"]
    # Явные координаты уходят на сервер как одна цель.
    assert plan["planBody"]["goal"] == {"x": 2, "y": 0}
    assert plan["planBody"]["goals"] == [{"x": 2, "y": 0}]
    assert "0.89 м" in plan["summary"]
    assert plan["steps"] == {"goal": "done", "plan": "done", "drive": "next"}
    # canStart=true от сервера и построенный маршрут снимают блокировку «Поехать».
    assert plan["startDisabled"] is False
    assert plan["reason"] == ""


@pytestmark_node
def test_robot_pose_button_and_estop(tmp_path: Path) -> None:
    report = run(tmp_path, payload_blocked())
    robot = report["afterRobot"]
    assert robot["goalX"] == "1.25" and robot["goalY"] == "-0.50"
    assert "1.25" in robot["note"]
    assert report["estop"]["calls"] == 1
    assert "E-STOP" in report["estop"]["message"]


# ------------------------------------------------------- безопасность ---
def test_navigation_screen_keeps_confirmation_and_estop() -> None:
    """Подтверждение перед стартом и E-STOP остались в общем map_view.js."""
    script = _html("map_view.js")
    assert "window.confirm('Запустить автономное движение (" in script
    assert "api/navigation/estop" in script
    assert "X-Robot-Control': 'navigation'" in script
    html = _html("navigation.html")
    for marker in ('id="nav-resume"', 'id="nav-cancel"', 'id="nav-estop"',
                   'id="navscreen-route"'):
        assert marker in html, marker


# ------------------------------------------- режим SLAM и иконки на борту ---
def test_map_scan_and_navigation_share_the_slam_mode_switch(kiosk_client,
                                                            client,
                                                            operator) -> None:
    """Тумблер режима SLAM есть на всех экранах с картой и подписан по-русски."""
    login(client)
    for web in (kiosk_client, client):
        for path in ("/map", "/scan", "/navigation"):
            page = web.get(path).text
            assert 'data-slam-mode="localize"' in page, (path, "localize")
            assert 'data-slam-mode="mapping"' in page, (path, "mapping")
            assert "Только локализация" in page
            assert "Активный SLAM" in page
            assert 'id="map-slam-hint"' in page


def test_slam_mode_is_stored_on_the_map_service(kiosk_client) -> None:
    """Переключение режима видно серверу и переживает перечитывание снимка."""
    app = kiosk_client.application
    map_service = app.extensions["rc_map"]
    try:
        response = kiosk_client.post("/api/kiosk_map/settings",
                                     json={"settings": {"slam_mode": "localize"}})
        assert response.status_code == 200
        assert response.json["settings"]["slam_mode"] == "localize"
        assert response.json["mapReset"] is False
        snapshot = kiosk_client.get("/api/kiosk_map").json
        assert snapshot["slamMode"] == "localize"
        assert snapshot["mapping"] is False
        # Режим скана показывает тот же тумблер.
        assert kiosk_client.get("/api/scan_mode").json["slamMode"] == "localize"
    finally:
        map_service.set_settings({"slam_mode": "mapping"})
        assert kiosk_client.get("/api/kiosk_map").json["slamMode"] == "mapping"


def test_kiosk_header_buttons_are_icons() -> None:
    """Функции/Карта/Навигация/Устройства/Датчики/Камеры — иконки, не текст."""
    robot = _html("robot.html")
    for button_id, title in (("k-map-open", "Карта лидара"),
                             ("k-route-open", "Навигация: цель, маршрут, поездка"),
                             ("k-devices-open", "Устройства робота"),
                             ("k-sensors-open", "Модуль датчиков"),
                             ("k-cameras-open", "Камеры и логи")):
        start = robot.index('id="%s"' % button_id)
        block = robot[start:robot.index("</a>", start) if "</a>" in robot[start:start + 900]
                      else robot.index("</button>", start)]
        assert "k-ctrl-btn--icon" in block, button_id
        assert "<svg" in block and 'aria-hidden="true"' in block, button_id
        assert 'aria-label="%s"' % title in block, button_id
        assert 'title="%s"' % title in block, button_id
        # Текста внутри кнопки больше нет: подпись живёт в tooltip и aria-label.
        inner = block[block.index(">", block.index("<svg")):]
        assert ">" not in inner or not any(ch.isalpha() for ch in inner.split("</svg>")[-1]), \
            button_id
    assert "k-ctrl-btn--icon" in _html("robot.css")


def test_localization_is_visible_on_every_map_screen(kiosk_client, client, operator) -> None:
    """«Робот знает, где он»: строка локализации есть на карте, скане и навигации."""
    login(client)
    for web in (kiosk_client, client):
        for path in ("/map", "/scan", "/navigation"):
            page = web.get(path).text
            assert 'id="map-localize"' in page, path
    script = (ROOT / "slam_gui" / "map_view.js").read_text(encoding="utf-8")
    for marker in ("нашёл себя на карте", "не совпал со стенами карты", "Карта пуста"):
        assert marker in script, marker
    assert "drawLocalization" in script


def test_localization_state_reaches_the_api(kiosk_client) -> None:
    """Снимок карты отдаёт результат сравнения скана со стенами."""
    payload = kiosk_client.get("/api/kiosk_map").json
    assert "localize" in payload
    info = payload["localize"]
    for key in ("score", "matched", "checked", "hasMap", "active", "localized", "reason"):
        assert key in info, key
    scan = kiosk_client.get("/api/scan_mode").json
    assert "localize" in scan["map"]


def test_localize_mode_does_not_paint_the_page_white() -> None:
    """Режим локализации не перекрашивает интерфейс.

    Класс режима навешивается на <body>, а плашка локализации — отдельный
    элемент с тем же именем. Одно правило на оба красило всю страницу
    (белый фон, отступы) — интерфейс «ломался» при нажатии «Только локализация».
    """
    css = (ROOT / "slam_gui" / "map.css").read_text(encoding="utf-8")
    # Правила плашки привязаны к самому элементу, а не к классу на <body>.
    assert "p.map-localize {" in css
    assert "\n.map-localize {" not in css
    # Класс состояния тела называется отдельно.
    assert "body.map-mode-localize" in css
    for name in ("map_view.js", "scan_mode.js"):
        script = _html(name)
        assert "'map-mode-localize'" in script, name
        # Старое имя на <body> больше не навешивается.
        assert "classList.toggle('map-localize'" not in script, name
        assert "classList.toggle('map-mode-localize'" in script, name
    # Плашка и её подписи на месте во всех трёх экранах.
    for page in ("map.html", "scan.html", "navigation.html"):
        html = _html(page)
        assert '<p class="map-localize" id="map-localize"' in html, page
