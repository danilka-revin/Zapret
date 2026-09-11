"""
Починка установки Zapret Control.

Команда `python3 run.py repair` (и кнопка в интерфейсе) чинит ровно то, что
ломается при «установке через sudo»: код и зависимости переезжают из /root в
домашний каталог пользователя, заново создаются ярлык в меню и значок на
рабочем столе, права NOPASSWD подписываются на живого пользователя, после
чего интерфейс запускается уже от его имени.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from . import APP_SLUG, app_dir, integration, session

IGNORED_ON_MOVE = {"__pycache__", "*.pyc", ".pytest_cache"}
CODE_ITEMS = ("run.py", "install.sh", "README.md", "LICENSE", "zapret", "extras", "assets",
              "tests", "tools")
DATA_ITEMS = ("nfqws", "deps", "cache", "config.json", "update.json", "zapret-control.log")


def _note(log, message: str) -> None:
    if log:
        log(message)


def _copy_code(source: Path, destination: Path) -> list[str]:
    """Кладёт исходники в destination, не трогая зависимости и конфиг."""
    copied: list[str] = []
    destination.mkdir(parents=True, exist_ok=True)
    for name in CODE_ITEMS:
        src = source / name
        if not src.exists():
            continue
        dst = destination / name
        try:
            if src.is_dir():
                shutil.copytree(src, dst, dirs_exist_ok=True,
                                ignore=shutil.ignore_patterns(*IGNORED_ON_MOVE))
            else:
                shutil.copy2(src, dst)
            copied.append(name)
        except OSError:
            continue
    return copied


def relocate_root_install(log=None) -> dict:
    """Переносит установку из /root в домашний каталог пользователя стола."""
    result = {"moved": False, "target": "", "notes": []}
    if not session.is_root():
        return result
    owner = session.desktop_user()
    if not owner or owner == "root":
        return result
    source = session.root_install_leftovers()
    if source is None:
        return result
    target = session.user_home(owner) / ".local" / "share" / APP_SLUG
    _note(log, f"Установка найдена в {source} — переношу в {target}")
    moved = session.migrate_root_install(source, target)
    if moved:
        result["notes"].append("перенесено: " + ", ".join(moved))
    copied = _copy_code(source, target)
    result["notes"].append("код скопирован: " + (", ".join(copied) or "не требуется"))
    if session.chown_tree(target, owner):
        result["notes"].append(f"владелец {target} → {owner}")
    # хвосты в /root: ярлык, который «не появлялся» на столе
    try:
        root_apps = Path("/root/.local/share/applications") / integration.DESKTOP_NAME
        if root_apps.exists():
            root_apps.unlink()
            result["notes"].append("ярлык из /root удалён")
    except OSError:
        pass
    try:
        shutil.rmtree(source, ignore_errors=True)
        result["notes"].append(f"копия {source} удалена")
    except OSError:
        pass
    os.environ["ZAPRET_APP_DIR"] = str(target)
    result["moved"] = True
    result["target"] = str(target)
    return result


def check_ownership(log=None) -> dict:
    """Каталог приложения должен принадлежать тому, кто будет в нём работать."""
    owner = session.desktop_user()
    target = app_dir()
    info = {"user": owner, "path": str(target), "fixed": False, "problem": ""}
    if not target.exists():
        info["problem"] = f"{target} не найден — сначала установка"
        return info
    try:
        st = target.stat()
    except OSError as exc:
        info["problem"] = str(exc)
        return info
    uid, _gid = session.uid_gid(owner)
    if st.st_uid == uid:
        return info
    _note(log, f"Каталог {target} принадлежит uid {st.st_uid}, а работать будет {owner}")
    if session.chown_tree(target, owner):
        info["fixed"] = True
        _note(log, "Права на каталог исправлены.")
    else:
        info["problem"] = (f"нет прав менять владельца: выполните "
                           f"sudo chown -R {owner} {target}")
    return info


def ensure_launch_entry(log=None) -> dict:
    """Ярлык в меню приложений + значок на рабочем столе."""
    placed = integration.install_shortcut()
    status = integration.shortcut_status()
    _note(log, "Ярлык в меню: " + (status["menu_path"] if status["menu"] else "не создан"))
    if status["desktop"]:
        _note(log, "Значок на рабочем столе: " + status["desktop_path"]
              + ("" if placed.get("trusted") else " (нужно разрешить в GNOME)"))
    elif not status["desktop_shown"]:
        _note(log, "Каталога «Рабочий стол» нет — ярлык только в меню приложений.")
    return {"placed": placed, "status": status}


def ensure_permissions(log=None) -> dict:
    owner = integration.permissions_user()
    if integration.permissions_ready():
        _note(log, f"Права без пароля для {owner} уже настроены.")
        return {"ok": True, "user": owner, "already": True}
    try:
        integration.setup_permissions(owner)
        _note(log, f"Права NOPASSWD прописаны для {owner}.")
        return {"ok": True, "user": owner}
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        _note(log, f"Права без пароля не настроены: {exc}")
        return {"ok": False, "user": owner, "error": str(exc)}


def ensure_data(log=None) -> dict:
    from . import core

    if core.deps_ready():
        return {"ok": True, "ready": True}
    _note(log, "Зависимости (nfqws + стратегии) не скачаны — скачиваю…")
    try:
        core.ensure_deps("latest", "", lambda msg: _note(log, msg))
        return {"ok": True, "ready": True}
    except Exception as exc:  # noqa: BLE001
        _note(log, f"Не удалось скачать зависимости: {exc}")
        return {"ok": False, "ready": False, "error": str(exc)}


def repair(log=None, launch: bool = True, with_data: bool = True) -> dict:
    """Полный цикл починки. Возвращает сводку по шагам."""
    result: dict = {"owner": session.desktop_user(), "steps": {}, "ok": True}

    result["steps"]["relocate"] = relocate_root_install(log)
    result["steps"]["ownership"] = check_ownership(log)
    if with_data:
        result["steps"]["data"] = ensure_data(log)
    result["steps"]["shortcut"] = ensure_launch_entry(log)
    result["steps"]["permissions"] = ensure_permissions(log)
    result["ok"] = (result["steps"]["ownership"].get("problem", "") == ""
                    and result["steps"]["permissions"].get("ok", True))

    if launch:
        launch_result = session.launch_gui(app_dir())
        result["steps"]["launch"] = {key: value for key, value in launch_result.items()
                                     if key != "env"}
        if not launch_result.get("ok"):
            result["ok"] = False
    return result


def summary_text(result: dict) -> str:
    """Человекочитаемое резюме для терминала."""
    lines = [f"Владелец установки: {result.get('owner', '?')} ({app_dir()})"]
    for key, step in (result.get("steps") or {}).items():
        if not isinstance(step, dict):
            continue
        if key == "relocate":
            lines.append("перенос из /root: " + ("готово" if step.get("moved") else "не требовался"))
            for note in step.get("notes", []):
                lines.append("  · " + note)
        elif key == "ownership":
            problem = step.get("problem")
            lines.append(f"права на каталог: {'есть замечание: ' + problem if problem else 'в порядке'}")
        elif key == "data":
            lines.append("зависимости: " + ("готовы" if step.get("ready") else "не скачаны"))
        elif key == "shortcut":
            status = step.get("status", {})
            lines.append("ярлык: меню — {}, рабочий стол — {}".format(
                "есть" if status.get("menu") else "нет",
                "есть" if status.get("desktop") else "нет"))
        elif key == "permissions":
            lines.append("права без пароля: " + ("настроены" if step.get("ok") else
                                                 "нужен пароль sudo"))
        elif key == "launch":
            lines.append("запуск интерфейса: " + (step.get("message") or ""))
            if step.get("log"):
                lines.append("  лог:")
                lines.extend("    " + line for line in str(step["log"]).splitlines())
    return "\n".join(lines)
