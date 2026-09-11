#!/usr/bin/env python3
"""
Точка входа Zapret Control.

Использование:
  python3 run.py gui                     — запустить графический интерфейс (Qt6)
  python3 run.py daemon                  — демон (для systemd-службы)
  python3 run.py start | stop | restart  — запуск/остановка zapret
  python3 run.py status                  — вывести статус
  python3 run.py ensure-deps             — скачать nfqws и стратегии
  python3 run.py shortcut install|remove — ярлык приложения
  python3 run.py service install|remove|start|stop — системная служба
  python3 run.py permissions install|remove — NOPASSWD для nft/nfqws
  python3 run.py update                  — самообновление
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _log(msg: str) -> None:
    print(msg, flush=True)


LIB_HINT = ("Установите системные библиотеки Qt6:\n"
            "    Debian/Ubuntu: sudo apt install libgl1 libegl1 libxkbcommon0 libdbus-1-3\n"
            "    Fedora:        sudo dnf install mesa-libGL mesa-libEGL libxkbcommon dbus-libs\n"
            "    Arch:          sudo pacman -S libglvnd libxkbcommon")


def _pyside_state() -> tuple[bool, str]:
    """Возвращает (готов ли Qt, текст ошибки). Различает «нет модуля» и «нет библиотек»."""
    try:
        import PySide6  # noqa: F401
        return True, ""
    except ModuleNotFoundError:
        return False, ""
    except ImportError as exc:      # сам модуль есть, а системных библиотек нет
        return False, f"PySide6 установлен, но не хватает системных библиотек: {exc}"


def _ensure_pyside() -> bool:
    """Проверяет наличие PySide6 и при отсутствии ставит его автоматически."""
    ready, error = _pyside_state()
    if ready:
        return True
    if error:
        _log(error)
        _log(LIB_HINT)
        return False

    _log("Интерфейс работает на Qt6 (PySide6) — устанавливаю его автоматически.")
    import subprocess

    attempts = [
        [sys.executable, "-m", "pip", "install", "--user", "--quiet",
         "PySide6-Essentials"],
        [sys.executable, "-m", "pip", "install", "--user", "--quiet",
         "--break-system-packages", "PySide6-Essentials"],
    ]
    for cmd in attempts:
        try:
            if subprocess.call(cmd) == 0:
                break
        except OSError:
            continue

    ready, error = _pyside_state()
    if ready:
        return True
    if error:
        _log(error)
        _log(LIB_HINT)
        return False

    _log("Не удалось установить PySide6 автоматически. Установите вручную:")
    _log("    python3 -m pip install --user PySide6-Essentials")
    _log("    (Ubuntu/Debian: sudo apt install python3-pip libgl1 libegl1 libxkbcommon0)")
    _log("    (Arch: sudo pacman -S pyside6  |  Fedora: sudo dnf install python3-pyside6)")
    return False


def _run_gui() -> int:
    if not _ensure_pyside():
        return 1
    try:
        from zapret.qt.app import run as qt_run
    except ModuleNotFoundError as exc:
        _log(f"Не удалось загрузить Qt-интерфейс: {exc}")
        _log("Установите зависимости: python3 -m pip install --user PySide6-Essentials")
        return 1
    except ImportError as exc:
        _log(f"Не удалось запустить Qt-интерфейс: {exc}")
        _log(LIB_HINT)
        return 1
    return qt_run()


def main() -> int:
    from zapret import core, integration

    args = sys.argv[1:]
    cmd = args[0] if args else "gui"

    if cmd in ("gui", "qt"):
        return _run_gui()

    if cmd == "daemon":
        core.daemon()
        return 0

    if cmd == "start":
        try:
            core.run_zapret()
            _log("zapret запущен.")
            return 0
        except Exception as exc:  # noqa: BLE001
            _log(f"Ошибка: {exc}")
            return 1

    if cmd == "stop":
        core.stop_zapret()
        _log("zapret остановлен.")
        return 0

    if cmd == "restart":
        core.stop_zapret()
        core.run_zapret()
        _log("zapret перезапущен.")
        return 0

    if cmd == "status":
        import json
        _log(json.dumps(core.status(), ensure_ascii=False, indent=2))
        return 0

    if cmd == "ensure-deps":
        try:
            core.ensure_deps("latest", "", _log)
            _log("Зависимости установлены.")
            return 0
        except Exception as exc:  # noqa: BLE001
            _log(f"Ошибка: {exc}")
            return 1

    if cmd == "shortcut":
        sub = args[1] if len(args) > 1 else "install"
        if sub == "install":
            integration.install_shortcut()
            _log("Ярлык приложения создан.")
        elif sub == "remove":
            integration.remove_shortcut()
            _log("Ярлык приложения удалён.")
        return 0

    if cmd == "service":
        sub = args[1] if len(args) > 1 else "status"
        if sub == "install":
            integration.install_service()
            _log("Служба установлена и запущена.")
        elif sub == "remove":
            integration.remove_service()
            _log("Служба удалена.")
        elif sub == "start":
            integration.start_service()
        elif sub == "stop":
            integration.stop_service()
        elif sub == "status":
            _log("installed=%s active=%s" % (integration.service_installed(),
                                             integration.service_active()))
        return 0

    if cmd == "permissions":
        sub = args[1] if len(args) > 1 else "install"
        if sub == "install":
            try:
                integration.setup_permissions()
                _log("Права NOPASSWD настроены.")
            except Exception as exc:  # noqa: BLE001
                _log(f"Ошибка: {exc}")
                return 1
        elif sub == "remove":
            integration.remove_permissions()
        return 0

    if cmd == "update":
        from zapret import update
        res = update.update_app(_log)
        _log(f"Обновление: {res}")
        return 0

    if cmd in ("--version", "version"):
        from zapret import APP_VERSION
        _log(APP_VERSION)
        return 0

    _log(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
