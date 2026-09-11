"""
Пользователь рабочего стола, его графическое окружение и запуск интерфейса.

Модуль решает один класс проблем: Zapret Control установили из-под root
(`sudo -i` / `sudo su`), и тогда приложение, ярлык и правила sudo уезжают
в /root, а окно не может подключиться к X/Wayland сессии обычного
пользователя. Здесь собрано «правильное» определение того, ЧЬИ должен быть
ярлык и В КАКОЙ сессии показывать окно — и единая точка запуска GUI, которой
пользуются и установщик, и интерфейс.
"""

from __future__ import annotations

import os
import pwd
import shutil
import subprocess
import time
from functools import lru_cache
from pathlib import Path

# Переменные, без которых Qt не найдёт дисплей или сессионную шину
GUI_VARS = ("DISPLAY", "WAYLAND_DISPLAY", "XAUTHORITY", "XDG_RUNTIME_DIR",
            "DBUS_SESSION_BUS_ADDRESS", "XDG_CURRENT_DESKTOP", "DESKTOP_SESSION",
            "XDG_SESSION_TYPE", "XDG_DATA_DIRS", "QT_QPA_PLATFORM", "LANG", "LANGUAGE")

# Всё с такими префиксами тоже полезно передать в пользовательскую сессию
GUI_PREFIXES = ("XDG_", "WAYLAND_", "DISPLAY", "DBUS_", "QT_", "GDK_", "MESA_",
                "SDL_VIDEODRIVER", "LC_")

# Процессы-«якоря» графической сессии: из их /proc/<pid>/environ берём окружение
SESSION_MARKERS = (
    "gnome-session-binary", "gnome-session", "cinnamon-session", "mate-session",
    "xfce4-session", "lxqt-session", "ukui-session", "deepin-session", "lxsession",
    "startplasma-wayland", "startplasma-x11", "plasmashell", "kwin_wayland",
    "kwin_x11", "mutter-x11-fragments", "Xwayland", "weston", "sway", "hyprland",
    "labwc", "river", "openbox", "xfwm4", "enlightenment", "wayfire",
)

MIN_UID = 1000


# ---------------------------------------------------------------------------
# Пользователи
# ---------------------------------------------------------------------------

def is_root() -> bool:
    return hasattr(os, "geteuid") and os.geteuid() == 0


def current_user() -> str:
    try:
        return pwd.getpwuid(os.getuid()).pw_name
    except (KeyError, OSError):
        return os.environ.get("USER") or "root"


def passwd_user(uid: int) -> str:
    try:
        return pwd.getpwuid(uid).pw_name
    except (KeyError, OSError):
        return ""


def user_info(user: str):
    try:
        return pwd.getpwnam(user)
    except KeyError:
        return None


def user_home(user: str) -> Path:
    """Домашний каталог пользователя (для root — его реальный home, не HOME)."""
    info = user_info(user)
    if info is not None and info.pw_dir:
        return Path(info.pw_dir)
    return Path("~").expanduser()


def uid_gid(user: str) -> tuple[int, int]:
    info = user_info(user)
    if info is None:
        return os.getuid(), os.getgid()
    return info.pw_uid, info.pw_gid


def _looks_like_human(user: str) -> bool:
    if not user or user == "root":
        return False
    info = user_info(user)
    if info is None:
        return False
    if info.pw_uid < MIN_UID:
        return False
    shell = (info.pw_shell or "").strip()
    if not shell or shell.endswith(("/nologin", "/false", "/sync")):
        return False
    return bool(info.pw_dir) and info.pw_dir not in ("/", "/nonexistent", "")


def _from_sudo_env() -> str:
    for key in ("ZAPRET_USER", "SUDO_USER"):
        candidate = (os.environ.get(key) or "").strip()
        if candidate and _looks_like_human(candidate):
            return candidate
    sudo_uid = (os.environ.get("SUDO_UID") or "").strip()
    if sudo_uid.isdigit():
        name = passwd_user(int(sudo_uid))
        if _looks_like_human(name):
            return name
    return ""


def _from_loginctl() -> str:
    """Владелец активной графической сессии (x11/wayland) по данным logind."""
    loginctl = shutil.which("loginctl")
    if not loginctl:
        return ""
    try:
        listing = subprocess.run([loginctl, "list-sessions", "--no-legend"],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                 text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""
    best = ""
    best_age = -1
    for line in listing.splitlines():
        session_id = line.strip().split(" ", 1)[0]
        if not session_id or not session_id[0].isdigit():
            continue
        try:
            details = subprocess.run([loginctl, "show-session", session_id],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                     text=True, timeout=10).stdout
        except (OSError, subprocess.TimeoutExpired):
            continue
        fields = {}
        for entry in details.splitlines():
            key, _sep, value = entry.partition("=")
            fields[key.strip()] = value.strip()
        if fields.get("Remote") == "yes":
            continue
        if fields.get("Type") not in ("x11", "wayland", "mir"):
            continue
        if fields.get("Active") != "yes":
            continue
        user = fields.get("User", "")
        if not _looks_like_human(user):
            continue
        # самая свежая сессия — та, на которой пользователь сейчас сидит
        age = _age_of(Path(f"/run/user/{uid_gid(user)[0]}/bus")) if user else 0
        if age >= best_age:
            best, best_age = user, age
    return best


def _age_of(path: Path) -> float:
    try:
        return max(0.0, time.time() - path.stat().st_mtime)
    except OSError:
        return 0.0


def _from_who() -> str:
    """Запасной вариант: кто сидит на консоли/в X (работает без logind)."""
    for tool in (["w", "-h"], ["who"]):
        if not shutil.which(tool[0]):
            continue
        try:
            out = subprocess.run(tool, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                 text=True, timeout=10).stdout
        except (OSError, subprocess.TimeoutExpired):
            continue
        for line in out.splitlines():
            fields = line.split()
            if len(fields) >= 2 and _looks_like_human(fields[0]):
                return fields[0]
    return ""


def _from_run_user() -> str:
    """Каталоги /run/user/<uid> создаются только у живых сеансов."""
    found = []
    try:
        entries = list(Path("/run/user").iterdir())
    except OSError:
        return ""
    for entry in entries:
        if not entry.name.isdigit():
            continue
        uid = int(entry.name)
        if uid < MIN_UID:
            continue
        has_bus = (entry / "bus").exists() or any(entry.glob("wayland-*"))
        user = passwd_user(uid)
        if has_bus and _looks_like_human(user):
            found.append((_age_of(entry), user))
    if not found:
        return ""
    found.sort()
    return found[-1][1]


def _from_passwd() -> str:
    """Совсем крайний случай: единственный обычный пользователь в системе."""
    candidates = []
    try:
        entries = list(pwd.getpwall())
    except OSError:
        return ""
    for entry in entries:
        if entry.pw_uid >= MIN_UID and entry.pw_shell not in ("/usr/sbin/nologin",
                                                              "/sbin/nologin", "/bin/false"):
            if entry.pw_dir.startswith("/home/"):
                candidates.append(entry.pw_name)
    return candidates[0] if len(candidates) == 1 else ""


@lru_cache(maxsize=1)
def desktop_user() -> str:
    """Кто должен владеть приложением, ярлыком и правами NOPASSWD.

    Под обычным пользователем это он сам; под root — реальный человек, чья
    графическая сессия сейчас активна (sudo -i и «установка от root» больше не
    ломают установку).
    """
    if not is_root():
        return current_user()
    for finder in (_from_sudo_env, _from_loginctl, _from_run_user, _from_who, _from_passwd):
        user = finder()
        if user:
            return user
    return "root"


def app_owner() -> str:
    """Владелец каталога приложения: тот, чей ~/.local/share его содержит."""
    return desktop_user()


def root_install_leftovers(root_home: str | Path | None = None) -> Path | None:
    """Каталог «/root/.local/share/zapret-control», если установка уехала в /root."""
    if not is_root():
        return None
    base = Path(root_home) if root_home else Path("/root/.local/share")
    candidate = base / "zapret-control"
    return candidate if (candidate / "run.py").exists() else None


def chown_tree(path: Path, user: str) -> bool:
    """Отдаёт каталог пользователю (нужен после установки из-под root)."""
    path = Path(path)
    if not path.exists():
        return False
    uid, gid = uid_gid(user)
    try:
        if path.stat().st_uid == uid and os.getuid() == uid:
            return True
    except OSError:
        return False
    if not is_root():
        return False
    ok = True
    try:
        targets = [path]
        for item in path.rglob("*"):
            targets.append(item)
        for item in targets:
            try:
                os.lchown(item, uid, gid)
            except OSError:
                ok = False
    except OSError:
        ok = False
    return ok


def migrate_root_install(source: Path, destination: Path) -> list[str]:
    """Переносит «корневую» установку в домашний каталог пользователя.

    Перемещаем только тяжелые и ценные вещи: nfqws, зависимости, кэш и
    config.json. Код поверх кладёт установщик.
    """
    source, destination = Path(source), Path(destination)
    moved: list[str] = []
    if not source.is_dir() or source.resolve() == destination.resolve():
        return moved
    destination.mkdir(parents=True, exist_ok=True)
    for name in ("nfqws", "deps", "cache", "config.json", "strategies"):
        src = source / name
        dst = destination / name
        if not src.exists() or dst.exists():
            continue
        try:
            shutil.move(str(src), str(dst))
            moved.append(name)
        except OSError:
            try:
                if src.is_dir():
                    shutil.copytree(src, dst, dirs_exist_ok=True)
                else:
                    shutil.copy2(src, dst)
                moved.append(name)
            except OSError:
                continue
    return moved


# ---------------------------------------------------------------------------
# Окружение графической сессии
# ---------------------------------------------------------------------------

def _environ_of(pid: int) -> dict[str, str]:
    try:
        raw = Path(f"/proc/{pid}/environ").read_bytes()
    except OSError:
        return {}
    result = {}
    for chunk in raw.split(b"\0"):
        if not chunk:
            continue
        text = chunk.decode("utf-8", "replace")
        key, sep, value = text.partition("=")
        if sep and value:
            result[key] = value
    return result


def _relevant(source: dict[str, str]) -> dict[str, str]:
    """Фильтр окружения: только то, что нужно графике и пользовательским путям."""
    out = {}
    for key, value in source.items():
        if value and (key in GUI_VARS or key.startswith(GUI_PREFIXES)):
            out[key] = value
    return out


def _user_processes(user: str) -> list[tuple[int, str]]:
    try:
        out = subprocess.run(["ps", "-u", user, "-o", "pid=,comm="],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    result = []
    for line in out.splitlines():
        fields = line.split(None, 1)
        if len(fields) == 2 and fields[0].isdigit():
            result.append((int(fields[0]), fields[1].strip()))
    return result


def _is_session_process(name: str) -> bool:
    return any(marker in name for marker in SESSION_MARKERS)


def has_session_process(user: str | None = None) -> bool:
    """Видим ли процессы, которые бывают только внутри графической сессии."""
    user = user or desktop_user()
    return any(_is_session_process(name) for _pid, name in _user_processes(user))


def _pids_of(user: str) -> list[int]:
    """pid пользователя: сначала процессы самой сессии, потом остальные."""
    processes = _user_processes(user)
    ranked = [(0 if _is_session_process(name) else 1, pid) for pid, name in processes]
    ranked.sort()
    return [pid for _rank, pid in ranked]


def _candidate_xauth(user: str, runtime: Path) -> Path | None:
    home = user_home(user)
    candidates = [runtime / "gdm" / "Xauthority", runtime / "xauth_file",
                  runtime / "Xauthority", home / ".Xauthority"]
    for pattern in (".mutter-Xwaylandauth.*", "xauth-*"):
        try:
            candidates.extend(sorted(runtime.glob(pattern)))
        except OSError:
            pass
    lightdm = Path(f"/run/lightdm/root/{os.environ.get('DISPLAY', ':0')}")
    candidates.append(lightdm)
    for candidate in candidates:
        try:
            if candidate.is_file():
                return candidate
        except OSError:
            continue
    return None


def session_env(user: str | None = None) -> dict[str, str]:
    """Окружение, в котором окно пользователя действительно появится.

    Под своим пользователем возвращаем текущее окружение как есть. Под root —
    собираем окружение целевого пользователя: его HOME и PATH, переменные живой
    сессии из /proc/<pid>/environ (DISPLAY, XAUTHORITY, Wayland-сокет, D-Bus) и
    типичные запасные значения. Именно этот dict решает проблему «установил
    через sudo -i — и окно не появилось».
    """
    user = user or desktop_user()
    switching = is_root() and user not in ("", current_user(), "root")
    uid, _gid = uid_gid(user)
    home = user_home(user)
    runtime = Path(f"/run/user/{uid}")
    info = user_info(user)
    env: dict[str, str] = dict(os.environ)
    if switching or not env.get("HOME"):
        env["HOME"] = str(home)
    if switching or not env.get("USER"):
        env["USER"] = user
        env["LOGNAME"] = user
    if switching:
        env["SHELL"] = (info.pw_shell if info is not None and info.pw_shell else "/bin/bash")
        env["XDG_RUNTIME_DIR"] = str(runtime)
    if not env.get("PATH"):
        env["PATH"] = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    user_bin = str(home / ".local" / "bin")
    if user_bin not in env["PATH"].split(":"):
        env["PATH"] = env["PATH"] + ":" + user_bin

    # Дисплей/шина берётся из живого процесса сессии: только так окно видно,
    # когда запуск пришёл из sudo/TTY, где DISPLAY и XAUTHORITY потеряны.
    pids = _pids_of(user)[:12]
    if not env.get("DISPLAY") or not env.get("WAYLAND_DISPLAY"):
        for pid in pids:
            for key, value in _relevant(_environ_of(pid)).items():
                if value and not env.get(key):
                    env[key] = value

    if switching and not Path(env.get("XDG_RUNTIME_DIR", "")).is_dir():
        env["XDG_RUNTIME_DIR"] = str(runtime)
    if not env.get("DISPLAY") and not env.get("WAYLAND_DISPLAY") and has_session_process(user):
        env["DISPLAY"] = ":0"        # у пользователя есть сессия, просто env пустой
    runtime_dir = Path(env.get("XDG_RUNTIME_DIR") or str(runtime))
    if not env.get("XAUTHORITY"):
        xauth = _candidate_xauth(user, runtime_dir)
        if xauth is not None:
            env["XAUTHORITY"] = str(xauth)
    if not env.get("DBUS_SESSION_BUS_ADDRESS") and (runtime_dir / "bus").exists():
        env["DBUS_SESSION_BUS_ADDRESS"] = f"unix:path={runtime_dir / 'bus'}"
    # X-сокет принадлежит пользователю: root к нему часто не допускается, поэтому
    # окно и запускается от его пользователя, а не от root
    return {key: value for key, value in env.items() if value}


def has_display(env: dict[str, str] | None = None) -> bool:
    env = env if env is not None else os.environ
    return bool(env.get("DISPLAY") or env.get("WAYLAND_DISPLAY"))


def qt_platform_fallbacks(env: dict[str, str] | None = None) -> list[str]:
    """Порядок перебора Qt-платформ: текущая → xcb → wayland.

    Пустая строка — «как есть» (выбор пользователя или автовыбор Qt).
    На Wayland-сессии без wayland-плагина спасает xcb через XWayland
    (в Ubuntu он всегда под рукой), и наоборот.
    """
    env = env if env is not None else os.environ
    forced = (env.get("QT_QPA_PLATFORM") or "").strip().lower()
    if forced in ("offscreen", "minimal"):
        return [forced]       # тестовые платформы не перебираем
    ordered = [forced] if forced else [""]
    if forced != "xcb":
        ordered.append("xcb")
    if (env.get("WAYLAND_DISPLAY") or forced == "wayland") and forced != "wayland":
        ordered.append("wayland")
    return ordered


# ---------------------------------------------------------------------------
# Ярлыки пользователя
# ---------------------------------------------------------------------------

def xdg_user_dir(user: str, key: str, default: str = "") -> Path | None:
    """Каталог из ~/.config/user-dirs.dirs (например DESKTOP → «Рабочий стол»)."""
    home = user_home(user)
    config = home / ".config" / "user-dirs.dirs"
    try:
        text = config.read_text(encoding="utf-8", errors="replace")
    except OSError:
        text = ""
    for line in text.splitlines():
        if line.strip().startswith(f"XDG_{key}_DIR="):
            match = line.split("=", 1)[1].strip().strip('"')
            match = match.replace("$HOME", str(home)).replace("${HOME}", str(home))
            return Path(match)
    return home / default if default else None


def desktop_dir(user: str | None = None) -> Path:
    """Каталог «Рабочий стол»: существующий, а если его нет — предлагаемый."""
    user = user or desktop_user()
    home = user_home(user)
    candidates = []
    declared = xdg_user_dir(user, "DESKTOP")
    if declared is not None:
        candidates.append(declared)
    candidates.extend(home / name for name in ("Desktop", "Рабочий стол", "Рабочий_стол"))
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    return home / "Desktop"


# ---------------------------------------------------------------------------
# Выполнение от имени пользователя и запуск интерфейса
# ---------------------------------------------------------------------------

def _drop_privileges(uid: int, gid: int) -> None:      # pragma: no cover - fork-only
    import os as _os

    _os.setgid(gid)
    try:
        _os.setgroups([gid])
    except OSError:
        pass
    _os.setuid(uid)


def popen_as(command: list[str], user: str | None = None, env: dict[str, str] | None = None,
             cwd: str | Path | None = None, log_path: str | Path | None = None,
             stdin_null: bool = True) -> subprocess.Popen:
    """Запускает команду от имени пользователя рабочего стола (root → user).

    Никаких runuser/su: права сбрасываются в потомке через setuid, поэтому
    работает даже там, где этих утилит нет.
    """
    kwargs: dict = {"cwd": str(cwd) if cwd else None, "env": env}
    if stdin_null:
        kwargs["stdin"] = subprocess.DEVNULL
    handle = None
    if log_path:
        path = Path(log_path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            handle = open(path, "ab")     # noqa: SIM115 - наследуется потомком
        except OSError:
            handle = None
    if handle is not None:
        kwargs["stdout"] = handle
        kwargs["stderr"] = subprocess.STDOUT
    kwargs["start_new_session"] = True

    if user and is_root() and user != current_user():
        uid, gid = uid_gid(user)
        kwargs["preexec_fn"] = lambda: _drop_privileges(uid, gid)
    try:
        return subprocess.Popen(list(command), **kwargs)
    finally:
        if handle is not None:
            handle.close()


def run_as_user(command: list[str], user: str | None = None, timeout: int = 120,
                env: dict[str, str] | None = None, cwd: str | Path | None = None,
                capture: bool = True) -> subprocess.CompletedProcess:
    """Короткая команда от имени пользователя (проверки, pip, ярлык)."""
    kwargs: dict = {"cwd": str(cwd) if cwd else None, "env": env,
                    "timeout": timeout, "stdin": subprocess.DEVNULL}
    if capture:
        kwargs["stdout"] = subprocess.PIPE
        kwargs["stderr"] = subprocess.STDOUT
        kwargs["text"] = True
    if user and is_root() and user != current_user():
        uid, gid = uid_gid(user)
        kwargs["preexec_fn"] = lambda: _drop_privileges(uid, gid)
    return subprocess.run(list(command), **kwargs)


def running_gui_pids() -> list[int]:
    """pid запущенных экземпляров интерфейса (кроме текущего процесса)."""
    try:
        out = subprocess.run(["ps", "-eo", "pid=,args="],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             text=True, timeout=3).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    me = os.getpid()
    pids = []
    for line in out.splitlines():
        fields = line.strip().split(None, 1)
        if len(fields) != 2 or not fields[0].isdigit():
            continue
        pid = int(fields[0])
        command = fields[1]
        if pid == me or "run.py" not in command:
            continue
        if command.rstrip().endswith(" gui") or " run.py gui" in command:
            pids.append(pid)
    return pids


def read_log_tail(log_path: str | Path, lines: int = 12) -> str:
    try:
        text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    return "\n".join([line for line in text.splitlines() if line.strip()][-lines:])


def launch_gui(app_dir: str | Path, log_path: str | Path | None = None,
               settle: float = 2.5, extra_args: tuple[str, ...] = ("gui",),
               probe: bool = True) -> dict:
    """Запускает интерфейс и ПРОВЕРЯЕТ, что он выжил.

    Возвращает dict(ok, pid, message, log, user, env) — установщик и кнопка
    «перезапустить» опираются на ok, а не на «мы успешно нажали spawn».
    """
    app_dir = Path(app_dir)
    user = desktop_user()
    env = session_env(user)
    python = shutil.which("python3") or shutil.which("python") or "python3"
    log_path = Path(log_path) if log_path else app_dir / "zapret-control.log"
    result = {"ok": False, "pid": None, "message": "", "log": "", "user": user,
              "log_path": str(log_path)}

    if not (app_dir / "run.py").exists():
        result["message"] = f"Не найден {app_dir / 'run.py'} — установка не завершена."
        return result
    headless = (env.get("QT_QPA_PLATFORM") or "").strip().lower() in (
        "offscreen", "minimal", "vnc")
    if not has_display(env) and not headless:
        result["message"] = ("Графическая сессия не найдена (нет DISPLAY/WAYLAND_DISPLAY). "
                             f"Запустите из-под рабочего стола: python3 {app_dir / 'run.py'} gui")
        return result

    picked_platform = ""
    forced_platform = (env.get("QT_QPA_PLATFORM") or "").strip().lower()
    if probe:
        # os._exit(0) в конце: иначе PySide6 иногда падает с SIGSEGV при
        # завершении процесса — проверка «открывается ли окно» ложно краснела.
        probe_code = ("import os, sys\n"
                      "from PySide6.QtWidgets import QApplication\n"
                      "QApplication(sys.argv)\n"
                      "print('QT_OK', flush=True)\n"
                      "os._exit(0)\n")
        tried: list[str] = []
        last_output = ""
        give_up_checking = False
        for candidate in qt_platform_fallbacks(env):
            attempt_env = dict(env)
            if candidate:
                attempt_env["QT_QPA_PLATFORM"] = candidate
            tried.append(candidate or "авто")
            try:
                check = run_as_user([python, "-c", probe_code], user=user,
                                    env=attempt_env, cwd=app_dir, timeout=25)
            except FileNotFoundError:
                result["message"] = f"Не найден {python}."
                return result
            except subprocess.TimeoutExpired:
                # Зависшая проверка — не приговор: запускаем как есть.
                give_up_checking = True
                break
            output = (getattr(check, "stdout", "") or "").strip()
            if check.returncode == 0 and "QT_OK" in output:
                env = attempt_env
                picked_platform = candidate
                break
            last_output = output
        else:
            result["message"] = ("Qt не может открыть окно в этом окружении "
                                 f"(перебраны платформы: {', '.join(tried)}).")
            result["log"] = last_output[-1200:]
            return result
        if give_up_checking:
            result["message"] = "Проверка Qt зависла — пробуем запустить интерфейс как есть."

    try:
        proc = popen_as([python, str(app_dir / "run.py"), *extra_args], user=user, env=env,
                        cwd=app_dir, log_path=log_path)
    except OSError as exc:
        result["message"] = f"Не удалось запустить интерфейс: {exc}"
        return result

    result["pid"] = proc.pid
    deadline = time.time() + max(0.4, settle)
    while time.time() < deadline:
        if proc.poll() is not None:
            break
        time.sleep(0.15)
    code = proc.poll()
    if code is None:
        result["ok"] = True
        result["message"] = f"Интерфейс запущен (pid {proc.pid})."
        if picked_platform and picked_platform != forced_platform:
            result["message"] += f" Qt-платформа: {picked_platform} (запасная)."
        return result
    result["message"] = (f"Интерфейс завершился сразу (код {code})."
                         if code is not None else "Интерфейс запущен.")
    result["log"] = read_log_tail(log_path)
    return result
