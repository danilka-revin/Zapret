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
  python3 run.py doctor                  — самодиагностика (что мешает работать)
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _log(msg: str) -> None:
    print(msg, flush=True)


LIB_HINT = ("Установите системные библиотеки Qt6:\n"
            "    Debian/Ubuntu: sudo apt install libgl1 libegl1 libxkbcommon0 libdbus-1-3 "
            "libxcb-cursor0\n"
            "    Fedora:        sudo dnf install mesa-libGL mesa-libEGL libxkbcommon dbus-libs "
            "xcb-util-cursor\n"
            "    Arch:          sudo pacman -S libglvnd libxkbcommon xcb-util-cursor")


def _distro_id() -> str:
    """Определяет семейство дистрибутива для точной подсказки."""
    try:
        text = open("/etc/os-release", encoding="utf-8").read().lower()
    except OSError:
        return "unknown"
    for family, keys in (("debian", ("debian", "ubuntu", "mint", "pop")),
                         ("fedora", ("fedora", "rhel", "centos", "rocky", "almalinux")),
                         ("arch", ("arch", "manjaro", "endeavouros")),
                         ("suse", ("opensuse", "suse", "sles")),
                         ("alpine", ("alpine",)),
                         ("gentoo", ("gentoo",))):
        if any(key in text for key in keys):
            return family
    return "unknown"


XCB_HINT = {
    "debian": "sudo apt install libxcb-cursor0 libxcb-xinerama0 libxcb-icccm4 libxcb-image0 "
              "libxcb-keysyms1 libxcb-render-util0 libxkbcommon-x11-0",
    "fedora": "sudo dnf install xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms "
              "xcb-util-renderutil libxkbcommon-x11",
    "arch": "sudo pacman -S xcb-util-cursor xcb-util-wm xcb-util-image xcb-util-keysyms "
            "xcb-util-renderutil libxkbcommon-x11",
    "suse": "sudo zypper install libxkbcommon-x11-0 xcb-util-cursor",
    "alpine": "sudo apk add libxcb xcb-util-cursor libxkbcommon",
    "gentoo": "sudo emerge x11-libs/xcb-util-cursor x11-libs/xcb-util-wm x11-libs/libxkbcommon",
    "unknown": "установите пакеты xcb-util-* и libxkbcommon-x11 для вашего дистрибутива",
}


def _qt_preflight() -> tuple[bool, str]:
    """Проверяет, что Qt действительно может открыть окно (в отдельном процессе).

    Если не хватает плагина xcb или библиотек, диагностируем это заранее и
    подсказываем точную команду установки — вместо непонятного падения Qt.
    """
    import subprocess

    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return False, ("нет графической среды: не заданы DISPLAY и WAYLAND_DISPLAY.\n"
                       "Запустите приложение из-под рабочего стола "
                       "(или выполните: python3 run.py doctor)")

    probe = ("from PySide6.QtWidgets import QApplication;"
             "app = QApplication([]);"
             "print('QT_OK')")
    try:
        proc = subprocess.run([sys.executable, "-c", probe], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return True, ""      # проверить не удалось — пробуем запустить как есть
    output = proc.stdout or ""
    if proc.returncode == 0 and "QT_OK" in output:
        return True, ""
    if "xcb" in output.lower() or "platform plugin" in output.lower():
        hint = XCB_HINT.get(_distro_id(), XCB_HINT["unknown"])
        return False, ("Qt не смог загрузить плагин xcb.\n"
                       f"Установите пакеты: {hint}\n"
                       "Если у вас Wayland, попробуйте: QT_QPA_PLATFORM=wayland python3 run.py gui")
    return False, f"Qt не запустился:\n{output.strip()[:600]}"


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


def _doctor() -> int:
    """Самодиагностика: показывает, что установлено и что мешает работе."""
    import platform
    import shutil

    from zapret import APP_NAME, APP_VERSION, app_dir, core, integration

    problems: list[str] = []

    def report(title: str, value: str, ok: bool | None = None) -> None:
        mark = " " if ok is None else ("+" if ok else "!")
        print(f"  [{mark}] {title}: {value}")
        if ok is False:
            problems.append(title)

    print(f"{APP_NAME} v{APP_VERSION} — самодиагностика")
    print("\nОкружение")
    report("система", f"{platform.system()} {platform.release()} ({platform.machine()})")
    report("python", platform.python_version(),
           tuple(int(x) for x in platform.python_version_tuple()[:2]) >= (3, 9))
    report("каталог данных", str(app_dir()))

    print("\nИнтерфейс (Qt6)")
    try:
        import PySide6
        from PySide6 import QtCore
        report("PySide6", QtCore.__version__, True)
    except Exception as exc:  # noqa: BLE001
        PySide6 = None
        report("PySide6", f"не установлен ({exc})", False)
        print("      исправление: python3 -m pip install --user PySide6-Essentials")
    if PySide6 is not None:
        ready, reason = _qt_preflight()
        first_line, _sep, rest = reason.partition("\n")
        report("открытие окна", "работает" if ready else first_line, ready)
        if not ready and rest:
            print("      " + rest.replace("\n", "\n      "))

    print("\nОбход DPI")
    report("nfqws", str(core.nfqws_path()) if core.nfqws_path().exists() else "не скачан",
           core.nfqws_path().exists())
    strategies = core.list_strategies()
    report("стратегии", f"{len(strategies)} шт." if strategies else "не найдены",
           bool(strategies))
    backend = None
    for name in ("nft", "iptables"):
        if shutil.which(name):
            backend = name
            break
    report("файрвол", backend or "не найден (нужен nftables или iptables)", bool(backend))
    report("права без пароля", "настроены" if integration.permissions_ready()
           else "нужен пароль sudo", integration.permissions_ready())

    print("\nСистемная интеграция")
    report("служба автозапуска", "установлена" if integration.service_installed() else "нет")
    report("ярлык в меню", "есть" if integration.shortcut_installed() else "нет")
    if PySide6 is not None:
        try:
            from PySide6.QtWidgets import QSystemTrayIcon
            report("системный трей", "доступен" if QSystemTrayIcon.isSystemTrayAvailable()
                   else "недоступен (окно просто не будет скрываться)")
        except Exception:  # noqa: BLE001
            pass

    if problems:
        print("\nЧто мешает работе:")
        for item in problems:
            print(f"  · {item}")
        print("\nЧаще всего кнопка не срабатывает без прав: сначала настройте их —")
        print("  python3 run.py permissions install   (один раз, спросит пароль)")
    else:
        print("\nВсё на месте: приложение должно запускаться и работать.")
    return 1 if problems else 0


def _run_gui() -> int:
    if not _ensure_pyside():
        return 1
    ready, reason = _qt_preflight()
    if not ready:
        _log("Не удалось открыть графический интерфейс.")
        _log(reason)
        _log("Подробная диагностика: python3 run.py doctor")
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

    if cmd in ("doctor", "check", "diag"):
        return _doctor()

    if cmd in ("--version", "version"):
        from zapret import APP_VERSION
        _log(APP_VERSION)
        return 0

    _log(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
