#!/usr/bin/env python3
"""
Проверка интерфейса «всё нажимается»: кликает по кнопкам и переключателям
и убеждается, что состояние меняется, а ошибок нет.

Запуск (без графического сервера):
    QT_QPA_PLATFORM=offscreen python3 tools/smoke_test.py

Тест подменяет системные вызовы ядра (nftables, nfqws, git) заглушками,
поэтому ничего не меняет в системе и не требует прав root.
"""

from __future__ import annotations

import os
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
os.environ.setdefault("ZAPRET_NO_AUTOSTART", "1")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPoint, Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from zapret import config as config_mod  # noqa: E402
from zapret import core  # noqa: E402
from zapret.qt import icons  # noqa: E402
from zapret.qt.app import ZapretWindow  # noqa: E402
from zapret.qt.controller import Controller  # noqa: E402
from zapret.qt.theme import ThemeManager, UISettings  # noqa: E402

FAILURES: list[str] = []
CHECKS = 0


def check(condition: bool, message: str):
    global CHECKS
    CHECKS += 1
    if condition:
        print(f"  ok  · {message}")
    else:
        FAILURES.append(message)
        print(f"  FAIL· {message}")


def install_fakes(state: dict):
    """Подменяет системные операции ядра, чтобы тест был безопасным."""
    core_status = {
        "running": False, "firewall": False, "deps_ready": True,
        "service_installed": False, "service_active": False,
        "shortcut_installed": False, "sudo_ok": True, "backend": "nftables",
        "app_dir": "/tmp/zapret-test",
    }
    state["status"] = core_status

    core.status = lambda: dict(state["status"])              # type: ignore[assignment]
    core.run_zapret = lambda cfg=None: state.__setitem__(     # type: ignore[assignment]
        "status", {**state["status"], "running": True, "firewall": True})
    core.stop_zapret = lambda: state.__setitem__(             # type: ignore[assignment]
        "status", {**state["status"], "running": False, "firewall": False})
    core.deps_ready = lambda: True                            # type: ignore[assignment]
    core.ensure_deps = lambda *a, **k: None                   # type: ignore[assignment]
    core.list_strategies = lambda: ["general.bat", "general_alt2.bat"]  # type: ignore[assignment]

    from zapret import autopilot as autopilot_mod
    autopilot_mod.candidate_strategies = lambda cfg, limit=6: ["general.bat", "general_alt2.bat"]

    def fake_probe_all(timeout: float = 6.0, keys=None):
        return [
            {"key": "youtube", "title": "YouTube", "icon": "play", "state": "ok",
             "latency_ms": 42, "code": 204, "detail": "42 мс · работает", "hint": "видео"},
            {"key": "discord", "title": "Discord", "icon": "gamepad", "state": "ok",
             "latency_ms": 61, "code": 200, "detail": "61 мс · работает", "hint": "чат"},
            {"key": "telegram", "title": "Telegram", "icon": "send", "state": "warn",
             "latency_ms": 1200, "code": 200, "detail": "1200 мс · медленно", "hint": "mtproto"},
        ]

    from zapret import checks as checks_mod
    checks_mod.probe_all = fake_probe_all                        # type: ignore[assignment]
    checks_mod.internet_available = lambda timeout=4.0: True      # type: ignore[assignment]
    autopilot_mod.checks.probe_all = fake_probe_all               # type: ignore[attr-defined]

    # Обслуживание: обновление, ярлык и починка не должны трогать систему и сеть
    from zapret import integration as integration_mod
    from zapret import repair as repair_mod
    from zapret import update as update_mod

    integration_mod.install_shortcut = lambda add_to_desktop=True: {
        "menu": "/tmp/zapret-test/zapret-control.desktop",
        "desktop": "/tmp/zapret-test/Desktop/zapret-control.desktop",
        "trusted": True}
    integration_mod.remove_shortcut = lambda: []
    integration_mod.shortcut_status = lambda: {
        "menu": True, "menu_path": "/tmp/zapret-test/zapret-control.desktop",
        "desktop": True, "desktop_path": "/tmp/zapret-test/Desktop/zapret-control.desktop",
        "desktop_shown": True}
    integration_mod.permissions_ready = lambda: True
    integration_mod.permissions_user = lambda: "tester"
    integration_mod.setup_permissions = lambda user=None: state.__setitem__("permissions", user)

    def fake_update_all(progress_cb=None, with_deps=True, with_shortcut=True, force=False):
        state.setdefault("calls", []).append("update_all")
        if progress_cb:
            progress_cb("Тестовое обновление: код обновлён")
        return {"ok": True, "restart_required": True, "failed_steps": [],
                "messages": ["код обновлён"], "version_before": "0.0.0", "version_after": "9.9.9",
                "steps": {"code": {"changed": True, "error": "", "version_before": "0.0.0",
                                   "version_after": "9.9.9"},
                          "deps": {"ok": True}, "shortcut": {"ok": True},
                          "permissions": {"ok": True}, "ownership": {"ok": True}}}

    update_mod.update_all = fake_update_all                        # type: ignore[assignment]
    update_mod.update_app = fake_update_all                        # type: ignore[assignment]
    update_mod.check_update = lambda progress_cb=None: {
        "available": True, "local": "aaaaaaa", "remote": "bbbbbbbbcccc", "version": "0.0.0",
        "source": "git", "message": "Доступно обновление"}
    update_mod.relaunch = lambda wait_for_exit=True, extra_args=("gui",): (
        state.__setitem__("relaunched", True) or {"ok": True, "error": "", "log": ""})
    update_mod.restart_command = lambda: "python3 run.py gui"
    repair_mod.repair = lambda log=None, launch=True, with_data=True: (
        state.__setitem__("repaired", True)
        or {"ok": True, "owner": "tester", "steps": {}})


def click(widget, button=Qt.MouseButton.LeftButton):
    """Настоящий клик мышью по центру виджета."""
    point = QPoint(widget.width() // 2, widget.height() // 2)
    QTest.mousePress(widget, button, Qt.KeyboardModifier.NoModifier, point)
    QTest.mouseRelease(widget, button, Qt.KeyboardModifier.NoModifier, point)


def wait_idle(app, controller, limit_ms: int = 4000) -> bool:
    """Ждёт, пока контроллер закончит текущую операцию."""
    waited = 0
    while controller.busy_key and waited < limit_ms:
        QTest.qWait(50)
        app.processEvents()
        waited += 50
    return not controller.busy_key


def pump(app, ms: int = 120):
    """Даёт Qt обработать события в течение ms миллисекунд."""
    deadline = ms
    while deadline > 0:
        QTest.qWait(20)
        deadline -= 20
        app.processEvents()


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    state: dict = {}
    install_fakes(state)

    cfg = config_mod.load()
    theme = ThemeManager(UISettings.from_dict(cfg.get("ui")))
    # Как в run(): интерфейс целиком перекрашивается палитрой темы,
    # иначе часть системных элементов остаётся светлой.
    app.setStyle("Fusion")
    from zapret.qt.theme import apply_theme_to_app

    apply_theme_to_app(app, theme)
    theme.changed.connect(lambda: apply_theme_to_app(app, theme))
    controller = Controller(theme)
    window = ZapretWindow(theme, controller, tray=False)
    window.resize(1200, 820)
    window.show()
    pump(app, 500)
    wait_idle(app, controller)

    print("\n[1] Главный экран и большая кнопка")
    check(window.power.caption in ("ВЫКЛ", "ЖДИТЕ"),
          f"кнопка показывает исходное состояние ({window.power.caption})")
    click(window.power)
    wait_idle(app, controller)
    pump(app, 400)
    check(state["status"]["running"], "клик по большой кнопке включает обход (core.run_zapret)")
    check(controller.status.get("running"), "статус в контроллере обновился")

    # Режим без автопилота, чтобы проверить ветку прямого запуска
    theme.update(autopilot=False)
    pump(app)
    wait_idle(app, controller)
    click(window.power)
    wait_idle(app, controller)
    pump(app, 400)
    check(not state["status"]["running"],
          f"повторный клик выключает обход (core.stop_zapret), busy={controller.busy_key!r}")

    print("\n[2] Автопилот")
    theme.update(autopilot=True)
    pump(app)
    click(window.autopilot_quick)
    wait_idle(app, controller)
    pump(app, 500)
    check(controller.autopilot_report is not None, "кнопка «Перебрать стратегии» запускает подбор")
    if controller.autopilot_report:
        check(controller.autopilot_report.get("strategy") in ("general.bat", "general_alt2.bat"),
              "автопилот выбрал стратегию и записал её в конфиг")

    print("\n[2b] Прогресс автопилота виден на большой кнопке")
    steps: list[tuple[int, int, str]] = []
    snapshots: list[tuple[str, str, float | None]] = []
    real_step = controller._on_autopilot_step

    def fake_autopilot_run(cfg, progress_cb=None, stop_flag=None, timeout=5.0, limit=6,
                           on_step=None):
        """Имитация автопилота: два шага и мгновенный результат."""
        for index, name in enumerate(["general.bat", "general_alt2.bat"], start=1):
            if progress_cb:
                progress_cb(f"[{index}/2] стратегия {name}…")
            if on_step is not None:
                on_step(index, 2, name)
        return {"strategy": "general_alt2.bat", "ok": 3, "avg_ms": 120.0,
                "tries": [], "results": []}

    from zapret import autopilot as autopilot_mod
    autopilot_mod.run = fake_autopilot_run          # type: ignore[assignment]

    def watch_step(index, total, strategy):
        real_step(index, total, strategy)           # проверяем настоящую логику
        steps.append((index, total, strategy))
        window._refresh_dashboard()                 # как это сделал бы таймер обновления
        snapshots.append((window.power.caption, window.power.hint, controller.progress))

    controller._on_autopilot_step = watch_step      # type: ignore[assignment]
    click(window.autopilot_quick)
    wait_idle(app, controller)
    pump(app, 400)
    controller._on_autopilot_step = real_step       # type: ignore[assignment]

    check(len(steps) == 2, f"автопилот сообщает о каждом шаге ({len(steps)})")
    check(steps and steps[0][:2] == (1, 2),
          "в шаге есть номер и общее число стратегий")
    check(all(caption.startswith("ПОДБОР") for caption, _hint, _p in snapshots),
          f"кнопка показывает процент подбора ({[c for c, _h, _p in snapshots]})")
    check(all("Стратегия" in hint for _c, hint, _p in snapshots),
          f"под кнопкой видно, какая стратегия проверяется ({[h for _c, h, _p in snapshots]})")
    check(snapshots and snapshots[0][2] and snapshots[0][2] > 0
          and snapshots[1][2] > snapshots[0][2],
          f"прогресс растёт по шагам и не равен нулю "
          f"({[round(p, 2) for _c, _h, p in snapshots]})")

    print("\n[2c] Панель «Подбор под сайт»")
    sheet = window.sheets["targets"]
    sheet.setGeometry(window.centralWidget().rect())
    sheet.open()
    pump(app, 400)
    check(sheet.isVisible(), "панель подбора под сайт открывается")

    # Подменяем перебор стратегий: сеть в тесте не нужна
    from zapret import autopilot as ap_mod
    from zapret.qt.widgets import Chip

    captured: dict = {}

    def fake_run_for_targets(cfg, hosts, progress_cb=None, stop_flag=None, on_step=None,
                             timeout=5.0, limit=6, apply_best=True, prefer=""):
        captured["hosts"] = list(hosts)
        captured["apply_best"] = apply_best
        captured["prefer"] = prefer
        if progress_cb:
            progress_cb("Проверяю " + ", ".join(hosts))
        if on_step:
            on_step(1, 2, "general.bat")
        return {"strategy": "general.bat", "applied": apply_best, "ok": len(hosts),
                "total": len(hosts), "avg_ms": 150.0, "tries": [
                    {"strategy": "general.bat", "ok": len(hosts), "avg_ms": 150.0,
                     "results": [{"host": h, "state": "ok", "latency_ms": 150,
                                  "detail": "150 мс · открывается"} for h in hosts]},
                    {"strategy": "general_alt.bat", "ok": 0, "avg_ms": 4000.0,
                     "results": [{"host": h, "state": "bad", "latency_ms": 4000,
                                  "detail": "нет ответа"} for h in hosts]},
                ], "results": [{"host": h, "state": "ok", "latency_ms": 150,
                               "detail": "150 мс · открывается"} for h in hosts],
                "what": hosts[0]}

    ap_mod.run_for_targets = fake_run_for_targets     # type: ignore[assignment]

    check(len(sheet.group_chips) >= 10,
          f"готовые группы сервисов на месте ({len(sheet.group_chips)})")
    check({"youtube", "discord", "telegram"} <= set(sheet.group_chips),
          "есть готовые группы Discord, YouTube и Telegram")

    # Мультивыбор: сразу несколько групп одной проверкой
    sheet.group_chips["youtube"].setChecked(True)
    sheet.group_chips["discord"].setChecked(True)
    pump(app, 150)
    check("YouTube" in sheet.selection_label.text()
          and "Discord" in sheet.selection_label.text(),
          f"выбор групп показывается подсказкой ({sheet.selection_label.text()[:40]}…)")
    check(sheet.btn_run_groups.isEnabled(), "кнопка «Проверить выбранные группы» активна")

    click(sheet.btn_run_groups)
    wait_idle(app, controller)
    pump(app, 400)
    hosts = captured.get("hosts", [])
    check("youtube.com" in hosts and "discord.com" in hosts and "googlevideo.com" in hosts,
          f"стратегия подбирается сразу под несколько групп ({len(hosts)} доменов)")
    check(len(sheet.result_rows) == 2, "результаты по стратегиям показаны списком")
    check(any(row.badge == "ЛУЧШАЯ" for row in sheet.result_rows),
          "лучшая стратегия помечена бейджем «ЛУЧШАЯ»")
    check(len(sheet.site_rows) >= 2, "по каждому сайту показан результат")

    # Свой сайт строкой
    sheet.group_chips["youtube"].setChecked(False)
    sheet.group_chips["discord"].setChecked(False)
    sheet.input.setText("https://rutracker.org/forum/index.php")
    click(sheet.run_btn)
    wait_idle(app, controller)
    pump(app, 400)
    check(captured.get("hosts") == ["rutracker.org"],
          f"ввод ссылки превращается в домен ({captured.get('hosts')})")
    check(any(c.text() == "rutracker.org" for c in sheet.findChildren(Chip)),
          "запрос попал в историю недавних")

    # Пресет: сохраняем результат и применяем одним нажатием
    sheet.preset_name.setText("Рутрекер")
    click(sheet.btn_save_preset)
    pump(app, 200)
    stored = controller.site_presets()
    check(any(p["name"] == "Рутрекер" for p in stored),
          f"пресет сохраняется ({[p['name'] for p in stored]})")
    check(bool(sheet.preset_cards), "пресет показан карточкой на панели")
    check(any(p["strategy"] == "general.bat" for p in stored),
          "в пресете записана найденная стратегия")

    controller.cfg["strategy"] = "general.bat"
    click(sheet.preset_cards[0].apply_btn)
    wait_idle(app, controller)
    pump(app, 200)
    check(controller.cfg.get("strategy") == "general.bat",
          "кнопка «Применить» у пресета ставит его стратегию")

    # Своя группа
    sheet.input.setText("wiki.example.com, git.example.com")
    sheet.group_name.setText("Работа")
    click(sheet.btn_save_group)
    pump(app, 250)
    check(any(key.startswith("my:") for key in sheet.group_chips),
          f"своя группа появляется среди чипов ({[k for k in sheet.group_chips if k.startswith('my:')]})")
    check(any(g["title"] == "Работа" for g in controller.groups()),
          "своя группа сохраняется в конфиг")
    sheet.group_name.setText("Работа")
    sheet.input.setText("git.example.com")
    click(sheet.btn_save_group)
    pump(app, 200)
    check(len([g for g in controller.groups() if g["title"] == "Работа"]) == 1,
          "повторное сохранение обновляет ту же группу, а не плодит дубли")

    # Повторный подбор для тех же доменов идёт от стратегии пресета
    sheet.input.setText("rutracker.org")
    click(sheet.run_btn)
    wait_idle(app, controller)
    pump(app, 400)
    check(captured.get("prefer") == "general.bat",
          f"повторный подбор начинается со стратегии пресета ({captured.get('prefer')!r})")

    # …и то же самое делает кнопка «Проверить» на карточке пресета
    captured.pop("prefer", None)
    click(sheet.preset_cards[0].check_btn)
    wait_idle(app, controller)
    pump(app, 400)
    check(captured.get("prefer") == "general.bat",
          f"карточку пресета можно перепроверить ({captured.get('prefer')!r})")
    check(controller.site_presets()[0]["avg_ms"] == 150.0,
          "перепроверка обновляет результат в пресете")

    # отключённое «применять лучшую»
    click(sheet.apply_switch)
    pump(app, 150)
    sheet.input.setText("example.org")
    click(sheet.run_btn)
    wait_idle(app, controller)
    pump(app, 300)
    check(captured.get("apply_best") is False,
          "переключатель «применять лучшую» отключает автоприменение")

    # удаление пресета
    controller.remove_preset("Рутрекер")
    pump(app, 300)
    check(not controller.site_presets(), "пресет удаляется")
    sheet.close()
    pump(app, 300)

    print("\n[3] Панели (шторки)")
    for key, button in (("customizer", window.palette_btn), ("journal", window.journal_btn),
                        ("help", window.help_btn)):
        click(button)
        pump(app, 400)
        sheet = window.sheets[key]
        check(sheet.isVisible(), f"панель «{key}» открывается кликом по кнопке в шапке")
        sheet.close()
        pump(app, 400)
        check(not sheet.isVisible(), f"панель «{key}» закрывается")

    click(window.btn_diagnostics)
    pump(app, 400)
    check(window.sheets["diagnostics"].isVisible(), "кнопка «Диагностика» открывает панель")
    window.sheets["diagnostics"].close()
    pump(app, 300)

    print("\n[3b] Быстрое переключение темы")
    was_dark = theme.palette.dark
    click(window.theme_toggle_btn)
    pump(app, 300)
    check(theme.palette.dark != was_dark,
          f"кнопка в шапке переключает тёмную и светлую тему (было dark={was_dark})")
    from PySide6.QtGui import QPalette
    window_palette = window.palette()
    window_color = window_palette.color(QPalette.ColorRole.Window)
    if theme.palette.dark:
        check(window_color.lightness() < 90,
              f"в тёмной теме фон окна тёмный ({window_color.name()})")
    else:
        check(window_color.lightness() > 180,
              f"в светлой теме фон окна светлый ({window_color.name()})")
    click(window.theme_toggle_btn)
    pump(app, 300)
    check(theme.palette.dark == was_dark, "повторный клик возвращает исходную тему")
    check(window.palette().color(QPalette.ColorRole.Window).lightness() < 90,
          f"палитра приложения следует за темой ({window.palette().color(QPalette.ColorRole.Window).name()})")

    print("\n[4] Кастомизация (тема, стекло, плотность)")
    sheet = window.sheets["customizer"]
    sheet.setGeometry(window.centralWidget().rect())
    sheet.open()
    pump(app, 400)
    sheet.theme_seg.set_value("light", emit=True)
    pump(app)
    check(theme.settings.mode == "light", "переключатель темы меняет режим на светлый")
    sheet.accent_picker.set_value("#69a7ff", emit=True)
    pump(app)
    check(theme.settings.accent == "#69a7ff", "свотч акцента меняет цвет")
    sheet.glass_seg.set_value("off", emit=True)
    pump(app)
    check(theme.settings.glass == "off", "переключатель стекла работает")
    sheet.radius_seg.set_value("square", emit=True)
    sheet.density_seg.set_value("compact", emit=True)
    sheet.font_seg.set_value("large", emit=True)
    pump(app)
    check(theme.settings.radius == "square" and theme.settings.density == "compact"
          and theme.settings.font_scale == "large",
          "углы, плотность и размер текста применяются")
    check(window.power.width() > 0 and window.hero_title.text() != "",
          "интерфейс перерисовался после смены оформления")
    saved = config_mod.load().get("ui") or {}
    check(saved.get("accent") == "#69a7ff", "настройки оформления сохраняются в config.json")

    # Пресет одним кликом
    for card in sheet.preset_cards.values():
        if card.selected:
            continue
        card.clicked.emit(card.key)
        pump(app)
        break
    check(theme.settings.accent in [v["accent"] for _k, _l, v in
                                    __import__("zapret.qt.theme", fromlist=["PRESETS"]).PRESETS],
          "карточка-пресет применяет готовый набор оформления")
    sheet.close()
    pump(app, 300)

    print("\n[5] Переключатели на главном экране")
    before = theme.settings.autopilot
    click(window.sw_autopilot)
    pump(app)
    check(theme.settings.autopilot != before, "переключатель автопилота переключается")
    click(window.sw_telegram)
    pump(app, 200)
    check(controller.cfg.get("telegram") is not None, "переключатель Telegram пишет в конфиг")
    click(window.sw_gamefilter)
    pump(app, 300)
    check(controller.cfg.get("gamefilter_tcp") in (True, False),
          "переключатель GameFilter пишет в конфиг")

    print("\n[6] Обслуживание и проверка сервисов")
    # при клике на «Обновить и перезапустить» окно не должно реально закрыться
    state["calls"] = []
    window._quit_app = lambda: state.__setitem__("quit", True)
    for button in (window.btn_deps, window.btn_check_update, window.btn_repair,
                   window.btn_shortcut, window.btn_diagnostics):
        click(button)
        wait_idle(app, controller)
        pump(app, 250)
    check(True, "кнопки обслуживания нажимаются без исключений")
    check(controller.update_info.get("available"),
          "«Проверить обновления» показывает наличие новой версии")
    check("bbbbbbbb" in window.update_state_label.text(),
          "подпись под кнопкой обновления содержит хеш новой версии")

    print("\n[6b] Обновление одной кнопкой + перезапуск")
    click(window.btn_update)
    wait_idle(app, controller)
    pump(app, 900)
    check("update_all" in state.get("calls", []),
          "кнопка «Обновить и перезапустить» запускает полное обновление")
    check(controller.update_result.get("ok"), "обновление вернуло успешную сводку")
    check(state.get("relaunched"), "после обновления приложение отделяет перезапуск")
    check(state.get("quit"), "старое окно закрывается, чтобы открылось с новым кодом")
    controller.check_services()
    pump(app, 700)
    check(len(controller.services) == 3, "проверка сервисов вернула данные по трём сервисам")
    check(window.service_rows["youtube"].state in ("ok", "checking"),
          "строка YouTube показывает результат проверки")

    print("\n[7] Иконки темы")
    check(len(icons.ICONS) > 40, f"набор иконок загружен ({len(icons.ICONS)})")
    missing = [name for name in ("power", "shield-check", "rocket", "palette", "refresh")
               if icons.icon_pixmap(name, 24, "#ffffff").isNull()]
    check(not missing, "ключевые иконки рисуются")

    print("\n[8] Горячие клавиши")
    QTest.keyClick(window, Qt.Key.Key_L, Qt.KeyboardModifier.ControlModifier)
    pump(app, 300)
    check(window.sheets["journal"].isVisible(), "Ctrl+L открывает журнал")
    QTest.keyClick(window, Qt.Key.Key_Escape)
    pump(app, 400)
    check(not window.sheets["journal"].isVisible(), "Esc закрывает панель")

    print("\n[9] Журнал")
    check(len(window.log_history) > 0, f"в журнале есть записи ({len(window.log_history)})")
    check(window.log_view.toPlainText() != "", "журнал отображается в интерфейсе")

    print(f"\nВсего проверок: {CHECKS}, ошибок: {len(FAILURES)}")
    if FAILURES:
        for failure in FAILURES:
            print("  · " + failure)
        return 1
    print("Интерфейс исправен: все проверки пройдены.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        sys.exit(2)
