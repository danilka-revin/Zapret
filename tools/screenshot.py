#!/usr/bin/env python3
"""
Сборка скриншотов интерфейса без графического сервера.

Используется для документации и быстрой проверки вёрстки: приложение
запускается в демо-режиме (с фиктивным статусом) и сохраняет PNG-файлы.

Примеры:
    python3 tools/screenshot.py --out /tmp/shots
    python3 tools/screenshot.py --out /tmp/shots --theme light --preset lime
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

os.environ.setdefault("ZAPRET_NO_AUTOSTART", "1")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# Герметичность: демо-пресеты и тема для кадров не должны утекать в реальный
# конфиг пользователя (и наоборот — чужой конфиг не должен портить кадры).
if "ZAPRET_APP_DIR" not in os.environ:
    import tempfile as _tempfile

    os.environ["ZAPRET_APP_DIR"] = _tempfile.mkdtemp(prefix="zapret-shots-")

from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from zapret import config as config_mod  # noqa: E402
from zapret.qt.app import ZapretWindow  # noqa: E402
from zapret.qt.controller import Controller  # noqa: E402
from zapret.qt.theme import (  # noqa: E402
    PRESETS,
    ThemeManager,
    UISettings,
    apply_theme_to_app,
)

TARGET_REPORT = {
    "strategy": "general_alt2.bat",
    "applied": True,
    "ok": 2,
    "total": 2,
    "avg_ms": 210.0,
    "what": "rutracker.org, youtube.com",
    "tries": [
        {"strategy": "general_alt2.bat", "ok": 2, "avg_ms": 210.0, "results": [
            {"host": "rutracker.org", "state": "ok", "latency_ms": 180,
             "detail": "180 мс · открывается"},
            {"host": "youtube.com", "state": "ok", "latency_ms": 240,
             "detail": "240 мс · открывается"},
        ]},
        {"strategy": "general.bat", "ok": 1, "avg_ms": 640.0, "results": []},
        {"strategy": "general_alt.bat", "ok": 0, "avg_ms": 4200.0, "results": []},
    ],
    "results": [
        {"host": "rutracker.org", "state": "ok", "latency_ms": 180,
         "detail": "180 мс · открывается"},
        {"host": "youtube.com", "state": "ok", "latency_ms": 240,
         "detail": "240 мс · открывается"},
    ],
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Скриншоты Zapret Control")
    parser.add_argument("--out", default="/tmp/shots", help="каталог для PNG")
    parser.add_argument("--theme", default="dark", choices=["dark", "light", "system"])
    parser.add_argument("--accent", default="#d5ff45")
    parser.add_argument("--glass", default="vivid", choices=["off", "soft", "vivid"])
    parser.add_argument("--size", default="1200x820")
    args = parser.parse_args()

    width, height = (int(x) for x in args.size.lower().split("x"))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    app = QApplication.instance() or QApplication(sys.argv)
    cfg = config_mod.load()
    # motion=off: панели встают мгновенно, кадры не ловят анимацию на середине.
    settings = UISettings.from_dict({**(cfg.get("ui") or {}), "mode": args.theme,
                                     "accent": args.accent, "glass": args.glass,
                                     "motion": "off"})
    theme = ThemeManager(settings)
    # Красим и системные элементы (поля ввода, подсказки, меню) — как в run().
    app.setStyle("Fusion")
    apply_theme_to_app(app, theme)
    theme.changed.connect(lambda: apply_theme_to_app(app, theme))
    controller = Controller(theme)
    window = ZapretWindow(theme, controller, tray=False)
    window.resize(width, height)
    window.show()
    window.demo_mode("on")

    def shot(name: str, close_sheets: bool = False):
        def inner():
            if close_sheets:
                for sheet in window.sheets.values():
                    sheet.hide()
                    sheet.overlay.hide()
            window.grab().save(str(out / name))
            print("сохранено:", out / name)
        return inner

    def open_targets(name: str):
        def inner():
            for other_key, other in window.sheets.items():
                if other_key != "targets":
                    other.hide()
                    other.overlay.hide()
            # Демонстрационные данные: своя группа и сохранённые пресеты
            now = time.time()
            controller.cfg["site_groups"] = [
                {"key": "work", "title": "Работа", "icon": "star",
                 "hosts": ["wiki.example.com", "git.example.com"]}]
            controller.cfg["site_presets"] = [
                {"name": "Видео и чат", "strategy": "general_alt2.bat",
                 "hosts": ["youtube.com", "youtu.be", "googlevideo.com", "discord.com",
                           "discordapp.com"],
                 "ok": 5, "total": 5, "avg_ms": 210.0, "checked_at": now - 240},
                {"name": "Работа VPN", "strategy": "general.bat",
                 "hosts": ["wiki.example.com", "git.example.com"],
                 "ok": 2, "total": 2, "avg_ms": 140.0, "checked_at": now - 5400},
            ]
            sheet = window.sheets["targets"]
            sheet._build_groups()
            sheet._rebuild_presets()
            sheet.group_chips["youtube"].setChecked(True)
            sheet.group_chips["discord"].setChecked(True)
            sheet.setGeometry(window.centralWidget().rect())
            sheet.input.setText("rutracker.org, youtube.com")
            sheet.open()
            sheet.offset = 0.0
            sheet._apply_offset(0.0)
            controller.targets_started.emit(["rutracker.org", "youtube.com"])
            controller.targets_done.emit(TARGET_REPORT)
            QTimer.singleShot(200, shot(name))
        return inner

    def open_sheet(key: str, name: str):
        def inner():
            if key == "journal":
                window.sheets[key].fill(window.log_history)
            for other_key, other in window.sheets.items():
                if other_key != key:
                    other.hide()
                    other.overlay.hide()
            sheet = window.sheets[key]
            sheet.setGeometry(window.centralWidget().rect())
            sheet.open()
            sheet.offset = 0.0
            sheet._apply_offset(0.0)
            QTimer.singleShot(200, shot(name))
        return inner

    def demo_strategies():
        """Демонстрационные стратегии для панели — без скачанных файлов."""
        now = time.time()
        controller.strategies = lambda: [  # type: ignore[assignment]
            {"name": "general.bat", "tcp": "80,443", "udp": "443,50000-50099",
             "filters": 4, "size": 8192},
            {"name": "general_alt.bat", "tcp": "80,443", "udp": "443",
             "filters": 3, "size": 6144},
            {"name": "general_alt2.bat", "tcp": "80,443", "udp": "443,3478",
             "filters": 5, "size": 9728},
            {"name": "discord.bat", "tcp": "443", "udp": "50000-50100",
             "filters": 2, "size": 4096},
            {"name": "telegram.bat", "tcp": "80,443", "udp": "", "filters": 2,
             "size": 3584},
        ]
        controller.cfg["strategy"] = "general_alt2.bat"
        controller.strategy_preview = (  # type: ignore[assignment]
            lambda name, max_lines=40, max_chars=4000:
            f"@echo off\nREM {name} — демонстрационное содержимое\n"
            "set ARGS=--wf-tcp=80,443 --wf-udp=443 ^\n"
            " --dpi-desync=fake,multisplit --dpi-desync-ttl=4 --new ^\n"
            " --filter-tcp=443 --dpi-desync=fake --new ^\n"
            " --filter-udp=443 --dpi-desync=fake")
        controller.strategy_info = (  # type: ignore[assignment]
            lambda name: {"name": name, "exists": True, "size": 8192,
                          "lines": 24, "tcp": "80,443", "udp": "443",
                          "filters": 4})
        sheet = window.sheets["strategies"]
        sheet._selected = "general_alt2.bat"
        sheet.refresh()

    lime_preset = dict(next(p[2] for p in PRESETS if p[0] == "lime"))
    sequence = [
        (200, shot("01-main-dark.png", close_sheets=True)),
        (500, open_sheet("customizer", "02-customizer.png")),
        (1000, open_sheet("journal", "03-journal.png")),
        (1450, open_sheet("diagnostics", "04-diagnostics.png")),
        (1900, open_sheet("help", "05-help.png")),
        (2250, open_targets("05b-targets.png")),
        (2700, lambda: (demo_strategies(), open_sheet("strategies", "05c-strategies.png")())),
        (3300, open_sheet("network", "05d-network.png")),
        (3800, open_sheet("about", "05e-about.png")),
        (4300, lambda: theme.update(mode="light", glass="soft", accent="#69a7ff")),
        (4900, shot("06-main-light.png", close_sheets=True)),
        (5300, lambda: theme.update(mode="dark", accent="#ffad4d", glass="off",
                                    density="compact", radius="square", motion="off")),
        (5700, shot("07-main-compact-square.png")),
        (6100, lambda: theme.update(accent="#34e0a1", glass="vivid", radius="soft",
                                    density="comfortable", font_scale="large")),
        (6500, shot("08-main-large-text.png")),
        (6800, lambda: theme.update(font_scale="normal", **lime_preset)),
        (7200, shot("09-preset-lime.png")),
    ]
    for delay, action in sequence:
        QTimer.singleShot(delay, action)
    QTimer.singleShot(7700, app.quit)
    app.exec()
    return 0


if __name__ == "__main__":
    sys.exit(main())
