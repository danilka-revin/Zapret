#!/usr/bin/env python3
"""
Точка входа Zapret Control.

Использование:
  python3 run.py gui                     — запустить графический интерфейс (Qt6)
  python3 run.py launch                  — запустить интерфейс «как установщик»:
                                           окружение рабочего стола + проверка окна
  python3 run.py daemon                  — демон (для systemd-службы)
  python3 run.py start | stop | restart  — запуск/остановка zapret
  python3 run.py status                  — вывести статус
  python3 run.py ensure-deps             — скачать nfqws и стратегии
  python3 run.py autopilot --sites a,b   — подобрать стратегию под сайты
                                           (--apply — сразу применить лучшую)
  python3 run.py shortcut install|remove — ярлык приложения (меню + рабочий стол)
  python3 run.py service install|remove|start|stop — системная служба
  python3 run.py permissions install|remove — NOPASSWD для nft/nfqws
  python3 run.py check-update            — есть ли новая версия
  python3 run.py update [--restart]      — обновить код+зависимости и перезапуститься
  python3 run.py repair [--no-deps]      — починить установку (root → пользователь,
                                           ярлык на столе, права, запуск окна)
  python3 run.py doctor                  — самодиагностика (что мешает работать)
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _log(msg: str) -> None:
    print(msg, flush=True)


def _crash_log(message: str) -> None:
    """Пишет в лог рядом с приложением — его видно и пользователю, и root."""
    from pathlib import Path

    from zapret import app_dir

    for candidate in (app_dir() / "zapret-control.log", Path("/tmp/zapret-control.log")):
        try:
            candidate.parent.mkdir(parents=True, exist_ok=True)
            with open(candidate, "a", encoding="utf-8") as handle:
                handle.write(message.rstrip() + "\n")
            return
        except OSError:
            continue


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
    # всё ниже — опционально: «нет» не считается неисправностью, поэтому ok=None
    report("служба автозапуска", "установлена" if integration.service_installed() else "нет",
           True if integration.service_installed() else None)
    status = integration.shortcut_status()
    report("ярлык в меню", ("есть: " + status["menu_path"]) if status["menu"] else "нет",
           True if status["menu"] else None)
    report("значок на рабочем столе",
           ("есть: " + status["desktop_path"]) if status["desktop"]
           else ("нет" + (" (каталога Рабочий стол нет)" if not status["desktop_shown"] else "")),
           True if status["desktop"] else None)
    if PySide6 is not None:
        try:
            from PySide6.QtWidgets import QSystemTrayIcon
            available = QSystemTrayIcon.isSystemTrayAvailable()
            report("системный трей", "доступен" if available
                   else "недоступен (окно просто не будет скрываться)",
                   True if available else None)
        except Exception:  # noqa: BLE001
            pass

    print("\nКто и откуда запускает приложение")
    from zapret import session

    owner = session.desktop_user()
    report("пользователь рабочего стола", owner, bool(owner))
    report("этот процесс", f"uid={os.getuid()} ({session.current_user()})", True)
    if session.is_root() and owner != "root":
        report("запуск от root",
               f"да — приложение, ярлык и права должны принадлежать {owner}", False)
    if session.root_install_leftovers() is not None:
        report("копия в /root", str(session.root_install_leftovers())
               + " — перенести в домашний каталог пользователя", False)
    try:
        st = app_dir().stat()
        owner_uid = session.uid_gid(owner)[0]
        report("владелец каталога приложения",
               f"uid {st.st_uid}" + ("" if st.st_uid == owner_uid else f" (нужен {owner_uid})"),
               st.st_uid == owner_uid)
    except OSError as exc:
        report("каталог приложения", f"не читается: {exc}", False)
    env = session.session_env(owner)
    display = env.get("DISPLAY") or env.get("WAYLAND_DISPLAY") or "не найден"
    report("дисплей", display + (f" (XAUTHORITY: {'есть' if env.get('XAUTHORITY') else 'нет'})"),
           session.has_display(env))

    if problems:
        print("\nЧто мешает работе:")
        for item in problems:
            print(f"  · {item}")
        print("\nБыстрый путь — починка одной командой (нужен пароль sudo):")
        print("  python3 run.py repair")
        print("Часто кнопка не срабатывает только из-за прав:")
        print("  python3 run.py permissions install   (один раз, спросит пароль)")
    else:
        print("\nВсё на месте: приложение должно запускаться и работать.")
    return 1 if problems else 0


def _relaunch_as_user() -> int | None:
    """GUI под root — типичная причина «окна нет»: перепускаем окно пользователю.

    Возвращает код выхода, если перезапуск от имени пользователя состоялся
    (или честно не удался), и None, если трогать ничего не нужно.
    """
    from pathlib import Path

    from zapret import session

    if not session.is_root():
        return None
    owner = session.desktop_user()
    if not owner or owner == "root":
        return None
    _log(f"Запущено от root, а рабочий стол у пользователя «{owner}».")
    _log("Окно от root на его дисплей обычно не выходит, а ярлык и права уезжают в /root.")
    here = Path(__file__).resolve().parent
    _log(f"Перезапускаю интерфейс от имени {owner}…")
    result = session.launch_gui(here, probe=False, settle=3.0)
    if result.get("ok"):
        _log(f"[+] {result.get('message')}")
        _log(f"    Лог: {result.get('log_path')}")
        _log("    Если нужно перенести установку из /root и поправить права: "
             f"python3 {here / 'run.py'} repair")
        return 0
    _log("[-] Не удалось запустить окно от имени пользователя: "
         + str(result.get("message") or "неизвестная причина"))
    if result.get("log"):
        _log("  последние строки лога:")
        for line in str(result["log"]).splitlines():
            _log("    " + line)
    _log(f"    Починка установки одной командой: python3 {here / 'run.py'} repair")
    return 1


def _launch_gui_from_cli() -> int:
    """Запуск интерфейса с проверкой, что окно действительно живое."""
    from pathlib import Path

    from zapret import session

    here = Path(__file__).resolve().parent
    owner = session.desktop_user()
    env = session.session_env(owner)
    _log(f"Пользователь рабочего стола: {owner}")
    display = env.get("DISPLAY") or env.get("WAYLAND_DISPLAY") or "не найден"
    _log(f"Дисплей: {display}")
    if not session.has_display(env):
        _log("[-] Графической сессии не видно: нет DISPLAY и WAYLAND_DISPLAY.")
        _log("    Запустите приложение из-под рабочего стола или выполните:")
        _log(f"    python3 {here / 'run.py'} doctor")
        return 1
    result = session.launch_gui(here, log_path=here / "zapret-control.log", settle=3.5)
    if result.get("ok"):
        _log(f"[+] {result.get('message')}")
        _log(f"    Лог: {result.get('log_path')}")
        return 0
    _log("[-] " + str(result.get("message") or "Окно не запустилось"))
    if result.get("log"):
        _log("  лог запуска:")
        for line in str(result["log"]).splitlines():
            _log("    " + line)
    _log("  Самодиагностика: python3 run.py doctor   |   Починка: python3 run.py repair")
    return 1


def _relaunch_waiter(argv: list[str]) -> int:
    """Служебная команда: дождаться выхода старого окна и поднять новое.

    Так приложение перезапускается после обновления: дочерний процесс отделён
    (setsid), поэтому переживает выход родителя и запускает уже новый код.
    """
    import argparse
    import time

    from zapret import session

    parser = argparse.ArgumentParser(prog="run.py relaunch", add_help=False)
    parser.add_argument("--pid", type=int, default=0)
    parser.add_argument("--settle", type=float, default=0.6)
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument("--args", default="gui")
    options, _unknown = parser.parse_known_args(argv)

    target = os.path.dirname(os.path.abspath(__file__))
    deadline = time.time() + max(1.0, options.timeout)
    if options.pid > 0:
        while time.time() < deadline:
            try:
                os.kill(options.pid, 0)
            except ProcessLookupError:
                break
            except PermissionError:
                break
            time.sleep(0.2)
    time.sleep(max(0.0, options.settle))
    extra = tuple(part for part in options.args.split(",") if part) or ("gui",)
    result = session.launch_gui(target, log_path=os.path.join(target, "zapret-control.log"),
                                settle=3.0, extra_args=extra)
    if result.get("ok"):
        _log("[+] " + str(result.get("message")))
        return 0
    _log("[-] " + str(result.get("message") or "окно не запустилось"))
    if result.get("log"):
        _log("  " + str(result["log"]).replace("\n", "\n  "))
    return 1


def _run_gui() -> int:
    if not _ensure_pyside():
        return 1
    rerouted = _relaunch_as_user()
    if rerouted is not None:
        return rerouted
    ready, reason = _qt_preflight()
    if not ready:
        _log("Не удалось открыть графический интерфейс.")
        _log(reason)
        _crash_log("[qt-preflight] " + reason)
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
    try:
        return qt_run()
    except Exception:  # noqa: BLE001 — окно не должно исчезать без следа
        import traceback

        detail = traceback.format_exc()
        _log("Интерфейс аварийно завершился:")
        _log(detail.strip()[-1500:])
        _crash_log("[gui-crash]\n" + detail)
        _log("Полный лог: ~/.local/share/zapret-control/zapret-control.log")
        return 1



def main() -> int:
    from zapret import checks, core, integration

    args = sys.argv[1:]
    cmd = args[0] if args else "gui"

    if cmd in ("gui", "qt"):
        return _run_gui()

    if cmd in ("launch", "start-gui", "open"):
        return _launch_gui_from_cli()

    if cmd == "relaunch":
        return _relaunch_waiter(args[1:])

    if cmd == "repair":
        from zapret import repair as repair_mod

        launch = "--no-launch" not in args
        with_data = "--no-deps" not in args
        result = repair_mod.repair(_log, launch=launch, with_data=with_data)
        _log("")
        _log(repair_mod.summary_text(result))
        return 0 if result.get("ok") else 1

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

    if cmd == "autopilot":
        from zapret import autopilot as autopilot_mod
        from zapret import config as config_mod
        from zapret import presets as presets_mod

        sites = ""
        groups = ""
        preset_name = ""
        save_as = ""
        apply_best = False
        limit = 6
        index = 1
        while index < len(args):
            item = args[index]
            if item in ("--sites", "-s") and index + 1 < len(args):
                sites = args[index + 1]
                index += 2
            elif item in ("--groups", "-g") and index + 1 < len(args):
                groups = args[index + 1]
                index += 2
            elif item in ("--preset", "-p") and index + 1 < len(args):
                preset_name = args[index + 1]
                index += 2
            elif item in ("--save-preset",) and index + 1 < len(args):
                save_as = args[index + 1]
                index += 2
            elif item in ("--limit", "-l") and index + 1 < len(args):
                limit = int(args[index + 1])
                index += 2
            elif item in ("--apply", "-a"):
                apply_best = True
                index += 1
            elif item in ("--help", "-h"):
                _log('python3 run.py autopilot --sites "rutracker.org, youtube.com" '
                     "[--apply] [--limit N]")
                _log("  или по группам:  --groups youtube,discord")
                _log("  или пресетом:    --preset \"Видео и чат\" "
                     "[--save-preset \"Видео и чат\"]")
                return 0
            else:
                sites = (sites + " " + item).strip()
                index += 1

        cfg = config_mod.load()
        hosts: list[str] = []
        prefer = ""
        if preset_name:
            preset = presets_mod.preset_by_name(cfg, preset_name)
            if not preset:
                _log(f"Пресет «{preset_name}» не найден. Сохранённые: "
                     + (", ".join(p["name"] for p in presets_mod.presets(cfg)) or "нет"))
                return 1
            hosts = list(preset["hosts"])
            prefer = preset["strategy"]
            save_as = save_as or preset["name"]
            _log(f"Пресет «{preset['name']}»: {len(hosts)} домен(ов), "
                 f"стратегия {prefer or 'ещё не подобрана'}")
        if groups:
            keys = [key.strip() for key in groups.split(",") if key.strip()]
            known = {group["key"] for group in presets_mod.all_groups(cfg)}
            unknown = [key for key in keys if key not in known]
            if unknown:
                _log("Не знаю такие группы: " + ", ".join(unknown))
                _log("Доступные: " + ", ".join(sorted(known)))
                return 1
            for host in presets_mod.hosts_for_selection(cfg, keys):
                if host not in hosts:
                    hosts.append(host)
        if sites:
            for host in checks.parse_targets(sites):
                if host not in hosts:
                    hosts.append(host)
        if not hosts:
            _log('Укажите сайты или группы: python3 run.py autopilot --sites "rutracker.org" '
                 "| --groups youtube,discord | --preset \"Видео и чат\"")
            return 1
        _log(f"Проверяю {len(hosts)} домен(ов): {', '.join(hosts[:6])}"
             + ("…" if len(hosts) > 6 else ""))
        try:
            report = autopilot_mod.run_for_targets(
                cfg, hosts, progress_cb=_log, limit=limit, apply_best=apply_best,
                prefer=prefer)
        except Exception as exc:  # noqa: BLE001
            _log(f"Ошибка: {exc}")
            return 1
        for item in report.get("tries", []):
            if "error" in item:
                continue
            _log(f"  {item['strategy']:<28} сайтов ок: {item.get('ok')}"
                 f"/{report.get('total')} · средняя задержка {item.get('avg_ms', 0):.0f} мс")
        _log(f"Лучшая стратегия: {report.get('strategy')}"
             + (" (применена)" if report.get("applied") else " (только измерение)"))
        if save_as:
            preset = presets_mod.save_preset(
                cfg, save_as, hosts, report.get("strategy", ""),
                ok=report.get("ok", 0), total=report.get("total", len(hosts)),
                avg_ms=report.get("avg_ms", 0.0))
            _log(f"Пресет «{preset['name']}» сохранён: {preset['strategy']}")
        return 0

    if cmd == "shortcut":
        sub = args[1] if len(args) > 1 else "install"
        if sub == "install":
            placed = integration.install_shortcut()
            _log("Ярлык в меню приложений: " + placed.get("menu", ""))
            if placed.get("desktop"):
                _log("Значок на рабочем столе: " + placed["desktop"])
            else:
                _log("Каталог «Рабочий стол» не найден — ярлык только в меню приложений.")
        elif sub == "remove":
            removed = integration.remove_shortcut()
            _log(f"Удалено ярлыков: {len(removed)}" if removed else "Удалять нечего.")
        elif sub == "status":
            import json
            _log(json.dumps(integration.shortcut_status(), ensure_ascii=False, indent=2))
        else:
            _log("Не знаю команду shortcut " + sub)
            return 1
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
                user = integration.permissions_user()
                integration.setup_permissions(user)
                _log(f"Права NOPASSWD настроены для {user}.")
            except Exception as exc:  # noqa: BLE001
                _log(f"Ошибка: {exc}")
                return 1
        elif sub == "remove":
            integration.remove_permissions()
        return 0

    if cmd in ("check-update", "updates"):
        import json

        from zapret import update as update_mod

        info = update_mod.check_update()
        if "--json" in args:
            _log(json.dumps(info, ensure_ascii=False, indent=2))
        else:
            _log(f"Текущая версия: v{info.get('version')} "
                 f"(код: {info.get('local') or info.get('known') or 'неизвестно'})")
            if info.get("remote"):
                _log(f"Версия в репозитории: {info['remote']}")
            if info.get("message"):
                _log(str(info["message"]))
            if info.get("available"):
                _log("Выполните: python3 run.py update --restart")
        return 0 if info.get("remote") else 1

    if cmd == "record-version":
        from zapret import update as update_mod

        state = update_mod.record_installed_version()
        _log("Записана установленная версия: " + str(state.get("commit", ""))
             + " (v" + str(state.get("version", "")) + ")")
        return 0

    if cmd == "repair-gui":
        # быстрый путь: только ярлык + запуск окна (без переноса установки)
        from zapret import repair as repair_mod

        repair_mod.ensure_launch_entry(_log)
        return _launch_gui_from_cli()

    if cmd == "update":
        import json

        from zapret import update as update_mod

        with_deps = "--no-deps" not in args
        restart = "--restart" in args or "--relaunch" in args
        result = update_mod.update_all(
            _log, with_deps=with_deps, with_shortcut="--no-shortcut" not in args)
        if "--json" in args:
            _log(json.dumps({key: value for key, value in result.items()
                             if key != "messages"}, ensure_ascii=False, indent=2))
        else:
            _log("Обновление: код — %s, зависимости — %s, перезапуск нужен — %s"
                 % ("изменён" if result["steps"]["code"].get("changed") else "без изменений",
                    result["steps"]["deps"].get("ok"), bool(result.get("restart_required"))))
            for message in result.get("messages", []):
                _log("  · " + message)
            if result.get("failed_steps"):
                _log("  не удалось: " + ", ".join(result["failed_steps"]))
        if restart:
            relaunch = update_mod.relaunch(wait_for_exit=False)
            _log("Интерфейс перезапущен с новым кодом." if relaunch.get("ok")
                 else "Перезапустите интерфейс вручную: " + update_mod.restart_command())
        return 0 if result.get("ok") else 1

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
