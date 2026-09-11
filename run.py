#!/usr/bin/env python3
"""
Точка входа Zapret Control.

Использование:
  python3 run.py gui                     — запустить графический интерфейс
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


def main() -> int:
    from zapret import core, integration

    args = sys.argv[1:]
    cmd = args[0] if args else "gui"

    if cmd == "gui":
        try:
            from zapret import gui
        except ModuleNotFoundError as exc:
            if getattr(exc, "name", None) == "tkinter":
                _log("Модуль tkinter не установлен. Выполните: sudo apt install python3-tk")
                return 1
            raise
        return gui.main()

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
