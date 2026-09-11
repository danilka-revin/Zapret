"""
Самообновление Zapret Control: код + зависимости + ярлык + перезапуск.

Кнопка «Обновить и перезапустить» в интерфейсе и `python3 run.py update`
попадают сюда. Логика рассчитана на то, что приложение может быть установлено
двумя способами: git-клон (обычный путь установщика) или распакованный архив
(если git не установлен) — и в том, и в другом случае код меняется на месте,
а пользовательские данные (config.json, deps/, кэш, nfqws) не трогаются.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.request
from pathlib import Path

from . import APP_VERSION, app_dir, session

REPO_URL = os.environ.get("ZAPRET_REPO_URL", "https://github.com/danilka-revin/Zapret")
BRANCH = os.environ.get("ZAPRET_BRANCH", "main")
REPO_SLUG = "danilka-revin/Zapret"

# Что переносим при обновлении из архива (всё пользовательское остаётся на месте)
CODE_ITEMS = ("run.py", "install.sh", "README.md", "LICENSE", "zapret", "extras", "assets")
STATE_FILE = "update.json"
_VERSION_RE = re.compile(r'^APP_VERSION\s*=\s*["\']([^"\']+)["\']', re.MULTILINE)


def root() -> Path:
    return app_dir()


def is_git_repo(path: Path | None = None) -> bool:
    path = path or root()
    return (path / ".git").exists() and shutil.which("git") is not None


def _git(args: list[str], cwd: Path | None = None, timeout: int = 120) -> subprocess.CompletedProcess:
    return subprocess.run(["git", *args], cwd=str(cwd or root()), stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, timeout=timeout)


def local_commit(path: Path | None = None) -> str:
    if not (path or root()).joinpath(".git").exists() or not shutil.which("git"):
        return ""
    try:
        return _git(["rev-parse", "--short", "HEAD"], cwd=path, timeout=20).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def _http_get(url: str, timeout: int = 30) -> bytes:
    request = urllib.request.Request(url, headers={
        "User-Agent": "Mozilla/5.0 (X11; Linux x86_64) ZapretControl/" + APP_VERSION})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


_REMOTE_CACHE: dict = {"at": 0.0, "value": ""}


def remote_commit() -> str:
    """HEAD ветки в репозитории: сначала git, затем GitHub API.

    Результат кэшируется на 5 минут, чтобы частые проверки не долбили сеть.
    """
    import time as _time

    if _REMOTE_CACHE["value"] and _TIME_NOW() - _REMOTE_CACHE["at"] < 300:
        return _REMOTE_CACHE["value"]

    def _remember(value: str) -> str:
        _REMOTE_CACHE["value"] = value
        _REMOTE_CACHE["at"] = _TIME_NOW()
        return value

    if shutil.which("git"):
        try:
            out = subprocess.run(["git", "ls-remote", REPO_URL, BRANCH],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                 text=True, timeout=40).stdout
            head = out.strip().split("\t", 1)[0].strip()
            if re.fullmatch(r"[0-9a-f]{7,40}", head or ""):
                return _remember(head[:12])
        except (OSError, subprocess.SubprocessError):
            pass
    try:
        data = json.loads(_http_get(
            f"https://api.github.com/repos/{REPO_SLUG}/commits/{BRANCH}").decode("utf-8", "replace"))
        return _remember((data.get("sha") or "")[:12])
    except Exception:  # noqa: BLE001 — сеть может быть недоступна, это не ошибка обновления
        return _REMOTE_CACHE["value"]


def _TIME_NOW() -> float:
    return time.time()


def compare_versions(left: str, right: str) -> int:
    """Сравнение версий вида 3.0.0: -1/0/1. Мусор считается нулём."""

    def parts(value: str) -> list[int]:
        out = []
        for chunk in re.split(r"[.\-+_]", str(value or "")):
            digits = "".join(ch for ch in chunk if ch.isdigit())
            out.append(int(digits) if digits else 0)
        return (out + [0, 0, 0])[:3]

    left_parts, right_parts = parts(left), parts(right)
    return (left_parts > right_parts) - (left_parts < right_parts)


def fetch_changelog(limit: int = 5, timeout: int = 20) -> list[dict]:
    """Последние релизы с GitHub: [{tag, name, body, url}]. Пусто при ошибке сети."""
    try:
        raw = _http_get(
            f"https://api.github.com/repos/{REPO_SLUG}/releases?per_page={max(1, limit)}",
            timeout=timeout)
        data = json.loads(raw.decode("utf-8", "replace"))
    except Exception:  # noqa: BLE001 — чейнджлог не критичен
        return []
    if not isinstance(data, list):
        return []
    out = []
    for item in data[:limit]:
        if not isinstance(item, dict):
            continue
        out.append({"tag": str(item.get("tag_name", "")),
                    "name": str(item.get("name", "") or item.get("tag_name", "")),
                    "body": str(item.get("body", "") or "")[:2000],
                    "url": str(item.get("html_url", ""))})
    return out


def code_version(path: Path | None = None) -> str:
    """Версия, которая лежит в коде по указанному пути (не в памяти процесса)."""
    try:
        text = (path or root()).joinpath("zapret", "__init__.py").read_text(encoding="utf-8")
    except OSError:
        return APP_VERSION
    match = _VERSION_RE.search(text)
    return match.group(1) if match else APP_VERSION


# ---------------------------------------------------------------------------
# Состояние последнего обновления
# ---------------------------------------------------------------------------

def state_path() -> Path:
    return root() / STATE_FILE


def read_state() -> dict:
    try:
        data = json.loads(state_path().read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def write_state(**values) -> dict:
    state = read_state()
    state.update(values)
    state["updated_at"] = time.time()
    try:
        state_path().write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n",
                                encoding="utf-8")
    except OSError:
        pass
    return state


def check_update(progress_cb=None) -> dict:
    """Есть ли новая версия и что для этого нужно сделать.

    Сеть — единственная часть операции, которая может зависнуть, поэтому
    всё остальное приложение делает и без неё.
    """
    before = local_commit()
    remote = remote_commit()
    known = read_state().get("commit", "")
    baseline = before or known
    result = {"available": None, "local": before, "known": known, "remote": remote,
              "version": APP_VERSION, "source": "git" if is_git_repo() else "archive",
              "message": ""}
    if not remote:
        result["message"] = ("Не удалось связаться с репозиторием: обновлений не видно. "
                             "Обычно мешает DNS или медленный GitHub — попробуйте ещё раз.")
        return result
    if not baseline:
        result["message"] = ("Не знаю, какая версия кода установлена (нет git и метки) — "
                             "при обновлении скачаю актуальную.")
        return result
    result["available"] = not remote.startswith(baseline)
    result["message"] = ("Доступно обновление: " + remote[:8] if result["available"]
                         else "Обновление не требуется: стоит " + baseline[:8])
    if progress_cb:
        progress_cb(result["message"])
    return result


def record_installed_version(commit: str = "", version: str = "") -> dict:
    """Записывает, какая версия установлена (использует установщик)."""
    return write_state(commit=commit or local_commit() or remote_commit(),
                       version=version or code_version(), branch=BRANCH)


# ---------------------------------------------------------------------------
# Обновление кода
# ---------------------------------------------------------------------------

def update_code(progress_cb=None) -> dict:
    """Обновляет исходники приложения. Возвращает {changed, source, error}."""
    target = root()
    before_commit = local_commit(target)
    before_version = code_version(target)
    info = {"changed": False, "source": "none", "error": "",
            "commit_before": before_commit, "commit_after": before_commit,
            "version_before": before_version, "version_after": before_version}

    def log(message: str) -> None:
        if progress_cb:
            progress_cb(message)

    if is_git_repo(target):
        info["source"] = "git"
        try:
            fetch = _git(["fetch", "--depth", "1", "origin", BRANCH], cwd=target, timeout=300)
            if fetch.returncode == 0:
                head = _git(["rev-parse", "FETCH_HEAD"], cwd=target, timeout=30).stdout.strip()
                current = _git(["rev-parse", "HEAD"], cwd=target, timeout=30).stdout.strip()
                if head and head != current:
                    log("Обновляю код: " + current[:8] + " → " + head[:8])
                    reset = _git(["reset", "--hard", head], cwd=target, timeout=120)
                    if reset.returncode != 0:
                        info["error"] = ((reset.stdout or "").strip()[-400:]
                                         or f"git reset не удался (код {reset.returncode})")
                else:
                    log("Код уже актуален.")
            else:
                log("git fetch не удался — пробую git pull…")
                pull = _git(["pull", "--ff-only"], cwd=target, timeout=300)
                if pull.returncode != 0:
                    info["error"] = ((pull.stdout or "").strip()[-400:]
                                     or f"git pull не удался (код {pull.returncode})")
        except (OSError, subprocess.SubprocessError) as exc:
            info["error"] = str(exc)
    else:
        info["source"] = "archive"
        try:
            _update_from_archive(target, log)
        except Exception as exc:  # noqa: BLE001 — показываем причину в интерфейсе
            info["error"] = str(exc)

    info["commit_after"] = local_commit(target)
    if info["source"] == "archive" and not info["commit_after"]:
        info["commit_after"] = remote_commit()
    info["version_after"] = code_version(target)
    info["changed"] = (info["commit_after"] != before_commit
                       or info["version_after"] != before_version)
    if info["changed"]:
        write_state(commit=info["commit_after"], version=info["version_after"],
                    source=info["source"])
    return info


def _update_from_archive(target: Path, log) -> None:
    """Ставит код из tar.gz репозитория — вариант для систем без git."""
    url = f"https://codeload.github.com/{REPO_SLUG}/tar.gz/{BRANCH}"
    log("Скачиваю архив с кодом…")
    tmp = Path(tempfile.mkdtemp(prefix="zc-update-"))
    try:
        archive = tmp / "source.tar.gz"
        archive.write_bytes(_http_get(url, timeout=180))
        extract = tmp / "extract"
        extract.mkdir()
        with tarfile.open(archive, "r:gz") as tf:
            _safe_extract(tf, extract)
        top = [p for p in extract.iterdir() if p.is_dir()]
        if not top:
            raise RuntimeError("Архив пустой или повреждён")
        source = top[0]
        for name in CODE_ITEMS:
            src = source / name
            if not src.exists():
                continue
            try:
                if src.is_dir():
                    log("Обновляю " + name + "/")
                    _copy_tree(src, target / name)
                else:
                    log("Обновляю " + name)
                    _copy_file(src, target / name)
            except OSError as exc:
                log("Не удалось обновить " + name + ": " + str(exc)[:120])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _safe_extract(tar: tarfile.TarFile, destination: Path) -> None:
    try:
        tar.extractall(destination, filter="data")     # type: ignore[call-arg]
    except TypeError:                                  # Python < 3.12
        members = [m for m in tar.getmembers()
                   if not m.name.startswith(("/", "..")) and not m.issym()]
        tar.extractall(destination, members=members)


def _copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def _copy_tree(src: Path, dst: Path) -> None:
    """Обновляет каталог, не трогая *.pyc и локальные файлы пользователя."""
    dst.mkdir(parents=True, exist_ok=True)
    for item in src.iterdir():
        if item.name in ("__pycache__", ".git"):
            continue
        target = dst / item.name
        if item.is_dir():
            _copy_tree(item, target)
        else:
            _copy_file(item, target)


# ---------------------------------------------------------------------------
# Полное обновление «одной кнопкой»
# ---------------------------------------------------------------------------

def update_all(progress_cb=None, with_deps: bool = True, with_shortcut: bool = True,
               force: bool = False) -> dict:
    """Код + зависимости + ярлык + права. То самое «сама всё установит».

    Возвращает сводку, по которой интерфейс решает, нужно ли перезапускаться.
    """
    from . import config as config_mod
    from . import core, integration

    def log(message: str) -> None:
        if progress_cb:
            progress_cb(message)

    result: dict = {"ok": False, "steps": {}, "messages": [], "restart_required": False,
                    "version_before": APP_VERSION, "version_after": APP_VERSION}

    def note(message: str) -> None:
        result["messages"].append(message)
        log(message)

    cfg = config_mod.load()

    # 1. Код
    note("Обновляю код приложения…")
    code = update_code(log)
    result["steps"]["code"] = code
    if code.get("error"):
        note("Код обновить не удалось: " + str(code["error"])[:200])
    result["restart_required"] = bool(code.get("changed")) or force
    result["version_after"] = code.get("version_after", APP_VERSION)

    # 2. Зависимости (nfqws + стратегии Flowseal)
    if with_deps:
        note("Проверяю зависимости (nfqws, стратегии)…")
        try:
            core.ensure_deps(cfg.get("nfqws_version", "latest"), cfg.get("strategy_rev", ""), log)
            result["steps"]["deps"] = {"ok": True}
        except Exception as exc:  # noqa: BLE001 — зависимости не должны ронять обновление
            result["steps"]["deps"] = {"ok": False, "error": str(exc)}
            note("Зависимости не обновились: " + str(exc)[:200])
    else:
        result["steps"]["deps"] = {"ok": None, "skipped": True}

    # 3. Ярлыки: меню приложений + рабочий стол
    if with_shortcut:
        try:
            placed = integration.install_shortcut()
            result["steps"]["shortcut"] = {"ok": True, **placed}
            note("Ярлык обновлён: " + (placed.get("desktop") or placed.get("menu", "")))
        except OSError as exc:
            result["steps"]["shortcut"] = {"ok": False, "error": str(exc)}

    # 4. Права без пароля (переподписываем на нужного пользователя)
    try:
        if not integration.permissions_ready():
            user = integration.permissions_user()
            integration.setup_permissions(user)
            note("Права NOPASSWD настроены для " + user)
            result["steps"]["permissions"] = {"ok": True, "user": user}
        else:
            result["steps"]["permissions"] = {"ok": True, "already": True}
    except Exception as exc:  # noqa: BLE001 — без прав приложение работает, просто спросит пароль
        result["steps"]["permissions"] = {"ok": False, "error": str(exc)}

    # 5. Права на файлы: после установки из-под root всё должно принадлежать пользователю
    try:
        owner = session.desktop_user()
        chowned = session.chown_tree(root(), owner)
        result["steps"]["ownership"] = {"ok": chowned, "user": owner}
    except OSError as exc:
        result["steps"]["ownership"] = {"ok": False, "error": str(exc)}

    result["failed_steps"] = [key for key, value in result["steps"].items()
                              if isinstance(value, dict) and value.get("ok") is False]
    result["ok"] = not code.get("error")
    if result["ok"]:
        write_state(commit=code.get("commit_after", "") or local_commit(),
                    version=result["version_after"], checked_at=time.time())
    return result


def update_app(progress_cb=None) -> dict:
    """Совместимость со старым вызовом: то же самое, что update_all."""
    return update_all(progress_cb)


# ---------------------------------------------------------------------------
# Перезапуск интерфейса
# ---------------------------------------------------------------------------

def relaunch(wait_for_exit: bool = True, extra_args: tuple[str, ...] = ("gui",)) -> dict:
    """Отделяется от текущего процесса и поднимает новое окно.

    Новый экземпляр запускает отдельный «ждущий» процесс: он переживает выход
    текущего приложения и не даёт обновлённому коду остаться незагруженным.
    """
    target = root()
    others = [pid for pid in session.running_gui_pids() if pid != os.getpid()]
    if others:
        return {"ok": False, "waiting": False,
                "error": "интерфейс уже запущен (pid " + ", ".join(map(str, others)) + ")",
                "log": ""}
    python = sys.executable or shutil.which("python3") or "python3"
    command = [python, str(target / "run.py"), "relaunch"]
    if wait_for_exit:
        command += ["--pid", str(os.getpid())]
    command += ["--args", ",".join(extra_args)]
    log_path = target / "zapret-control.log"
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        handle = open(log_path, "ab")      # noqa: SIM115 - fd наследуется потомком
    except OSError:
        handle = subprocess.DEVNULL
    try:
        subprocess.Popen(command, cwd=str(target), stdin=subprocess.DEVNULL,
                         stdout=handle, stderr=subprocess.STDOUT,
                         start_new_session=True, env=session.session_env())
        ok = True
    except OSError as exc:
        ok = False
        relaunch_error = str(exc)
    else:
        relaunch_error = ""
    finally:
        if handle is not subprocess.DEVNULL:
            handle.close()
    return {"ok": ok, "error": relaunch_error, "log": str(log_path)}


def restart_command() -> str:
    """Команда ручного перезапуска — показываем её, если самоперезапуск не удался."""
    return f"python3 {root() / 'run.py'} gui"
