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
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

os.environ.setdefault("ZAPRET_NO_AUTOSTART", "1")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from zapret import config as config_mod  # noqa: E402
from zapret.qt.app import ZapretWindow  # noqa: E402
from zapret.qt.controller import Controller  # noqa: E402
from zapret.qt.theme import PRESETS, ThemeManager, UISettings  # noqa: E402


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
    settings = UISettings.from_dict({**(cfg.get("ui") or {}), "mode": args.theme,
                                     "accent": args.accent, "glass": args.glass})
    theme = ThemeManager(settings)
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

    sequence = [
        (200, shot("01-main-dark.png", close_sheets=True)),
        (500, open_sheet("customizer", "02-customizer.png")),
        (1000, open_sheet("journal", "03-journal.png")),
        (1450, open_sheet("diagnostics", "04-diagnostics.png")),
        (1900, open_sheet("help", "05-help.png")),
        (2400, lambda: theme.update(mode="light", glass="soft", accent="#69a7ff")),
        (2800, shot("06-main-light.png", close_sheets=True)),
        (3100, lambda: theme.update(mode="dark", accent="#ffad4d", glass="off",
                                    density="compact", radius="square", motion="off")),
        (3500, shot("07-main-compact-square.png")),
        (3800, lambda: theme.update(accent="#34e0a1", glass="vivid", radius="soft",
                                    density="comfortable", font_scale="large")),
        (4200, shot("08-main-large-text.png")),
        (4500, lambda: theme.update(font_scale="normal", **dict(PRESETS[1][2]))),
        (4900, shot("09-preset-lime.png")),
    ]
    for delay, action in sequence:
        QTimer.singleShot(delay, action)
    QTimer.singleShot(5400, app.quit)
    app.exec()
    return 0


if __name__ == "__main__":
    sys.exit(main())
