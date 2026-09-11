"""
Самообновление: git pull в установленной копии + обновление зависимостей.
"""

import subprocess
from pathlib import Path

from . import APP_VERSION, app_dir
from .core import ensure_deps


def is_git_repo(path: Path) -> bool:
    return (path / ".git").exists()


def update_app(progress_cb=None) -> dict:
    """Обновляет код приложения (git pull) и зависимости. Возвращает сводку."""
    root = app_dir()
    result = {"code_updated": False, "deps_updated": False, "version": APP_VERSION}

    if is_git_repo(root):
        if progress_cb:
            progress_cb("Обновление кода (git pull)…")
        p = subprocess.run(["git", "-C", str(root), "pull", "--ff-only"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        result["code_updated"] = p.returncode == 0
        if p.returncode != 0:
            result["git_output"] = p.stdout
            if progress_cb:
                progress_cb(f"git pull не удался:\n{p.stdout}")
    else:
        if progress_cb:
            progress_cb("Установленная копия не является git-репозиторием — пропускаю git pull.")

    try:
        ensure_deps("latest", "", progress_cb)
        result["deps_updated"] = True
    except Exception as exc:  # noqa: BLE001
        result["deps_error"] = str(exc)
        if progress_cb:
            progress_cb(f"Ошибка обновления зависимостей: {exc}")

    return result
