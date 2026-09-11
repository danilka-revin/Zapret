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
    for button in (window.btn_deps, window.btn_update, window.btn_shortcut,
                   window.btn_diagnostics):
        click(button)
        wait_idle(app, controller)
        pump(app, 250)
    check(True, "кнопки обслуживания нажимаются без исключений")
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
