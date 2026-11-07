"""Драйвер лидара ROS 2 поднимается сам — без терминала.

Ярлык «Запустить робот» (``scripts/start_robot.sh``) ставил ROS 2 и запускал
супервизор лидара, но это был единственный путь, на котором драйвер появлялся
сам. Стоило запустить приложение иначе (автозапуск, systemd, ``kiosk.sh``,
второй экран, «Обновить с GitHub») — и ``/scan`` никто не публиковал: на экранах
«скана лидара нет», а исправить это можно было только руками в терминале
(``./scripts/ros2_lidar.sh`` или ``./scripts/ros2_setup.sh``). Оператор робота не
должен ничего печатать в консоли — приложение держит драйвер так же, как ярлык.

Что делает сторож (раз в :data:`TICK_S` секунд, в фоновом потоке):

  * скан идёт (``ros.fresh()``) — не вмешиваемся;
  * супервизор уже запущен (живой ``var/lidar.pid`` или служба ``robot-lidar``) —
    только наблюдаем, второй не поднимаем;
  * ROS 2 готов (есть префикс, видны пакеты драйверов) — запускаем
    ``scripts/ros2_lidar.sh --keep-alive --no-setup`` отдельной сессией, журнал
    ``var/lidar.log``; он сам выбирает драйвер под лидар и следит за ``/scan``;
  * ROS 2 не готов — запускаем ``scripts/ros2_setup.sh --background-build
    --no-wait --quiet`` (докачает ROS 2, драйверы, соберёт ``ros2_ws``). Пароль
    администратора спрашивает окно системы (``scripts/askpass_gui.sh``), а не
    терминал; без окна состояние честно пишется в ``var/ros2_setup.json``.

Почему сторож нужен и при источнике ``ros``: профиль LDS-01 (``RC_LIDAR_PROTOCOL
=lds01``) намеренно не отдаёт порт приложению. Если драйвер не поднялся,
приложение не имеет права отбирать USB-адаптер — значит, обязано поднять
драйвер само.

Выключается: ``RC_LIDAR_AUTOSTART=0`` (весь лидар) и ``RC_ROS_AUTO=0`` (только
автоустановка; запуск готового драйвера остаётся). Тесты сторожа не поднимают
(``PYTEST_CURRENT_TEST``).
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

from . import ros_env

log = logging.getLogger("robot_control.lidar_boot")

#: Как часто проверять, нужен ли драйвер (секунды).
TICK_S = 20.0
#: Не запускать супервизор чаще, чем раз в столько секунд (защита от циклов).
SPAWN_EVERY_S = 60.0
#: Не повторять автоустановку ROS 2 чаще, чем раз в столько секунд.
INSTALL_EVERY_S = 600.0
#: Как часто перепроверять службу ``robot-lidar`` (systemctl — не бесплатный).
SERVICE_CHECK_S = 60.0
#: Журнал супервизора и установки — в ``var/``, как у ярлыка «Запустить робот».
LIDAR_LOG = Path("var") / "lidar.log"
BOOT_LOG = Path("var") / "lidar-boot.log"
SETUP_LOG = Path("var") / "ros2_setup.log"
PID_FILE = Path("var") / "lidar.pid"
LOG_SOURCES = (
    ("Журнал драйвера ROS 2", LIDAR_LOG),
    ("Предыдущий журнал драйвера", LIDAR_LOG.with_suffix(".log.1")),
    ("Автоподготовка ROS 2", BOOT_LOG),
    ("Предыдущий журнал автоподготовки", BOOT_LOG.with_suffix(".log.1")),
    ("Установка и сборка ROS 2", SETUP_LOG),
    ("Предыдущий журнал установки ROS 2", SETUP_LOG.with_suffix(".log.1")),
)
LOCK_FILE = Path("var") / "lidar-boot.lock"
#: Так узнаём «наше» приложение/цикл по командной строке процесса.
_LIDAR_CMDLINE = "ros2_lidar.sh"
_SETUP_CMDLINE = "ros2_setup.sh"

#: Состояние сторожа для ``GET /api/lidar`` (``ros.boot``) и журнала.
_state: dict[str, Any] = {
    "enabled": True,
    "state": "idle",        # idle | fresh | supervisor | service | spawn | install | off
    "action": "",
    "note": "",
    "at": 0.0,
    "spawned_pid": None,
    "installed_at": 0.0,
    "spawn_at": 0.0,
}
_thread: Optional[threading.Thread] = None
_lock = threading.Lock()
_last_logged_action: Optional[str] = None


def in_tests() -> bool:
    """Идёт ли тестовый прогон: тогда сторож не поднимается и ничего не ставит.

    ``PYTEST_CURRENT_TEST`` pytest выставляет на время каждого теста, но
    отдельные тесты его убирают, чтобы проверить рабочую ветку приложения.
    Поэтому смотрим ещё и на сам загруженный модуль ``pytest``: в рабочем
    процессе робота его нет никогда.
    """
    return bool(os.environ.get("PYTEST_CURRENT_TEST")) or "pytest" in sys.modules


def project_root() -> Path:
    """Корень проекта (там, где ``scripts/`` и ``.env``)."""
    return Path(__file__).resolve().parent.parent


def _truthy(value: Any, default: bool = True) -> bool:
    text = str(value if value is not None else "").strip().lower()
    if not text:
        return default
    return text not in {"0", "false", "no", "off", "нет"}


def enabled() -> bool:
    """Включён ли автозапуск лидара (``RC_LIDAR_AUTOSTART``)."""
    return _truthy(os.environ.get("RC_LIDAR_AUTOSTART"), True)


def install_allowed() -> bool:
    """Можно ли ставить и собирать ROS 2 самим (``RC_ROS_AUTO``)."""
    return _truthy(os.environ.get("RC_ROS_AUTO"), True)


def _alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _cmdline(pid: int) -> str:
    try:
        raw = Path("/proc/%d/cmdline" % pid).read_bytes()
    except OSError:
        return ""
    return " ".join(part.decode("utf-8", "replace") for part in raw.split(b"\0") if part)


def supervisor_pid(root: Optional[Path] = None) -> Optional[int]:
    """Pid цикла ``ros2_lidar.sh --keep-alive`` из ``var/lidar.pid`` (если жив)."""
    path = Path(root or project_root()) / PID_FILE
    try:
        pid = int(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return None
    if not _alive(pid):
        return None
    # Pid мог достаться чужому процессу: проверяем командную строку, иначе
    # «супервизор запущен» было бы ложью, и драйвер бы не поднялся никогда.
    if _LIDAR_CMDLINE not in _cmdline(pid):
        return None
    return pid


def supervisor_running(root: Optional[Path] = None) -> bool:
    """Работает ли супервизор лидара (его цикл или сам узел)."""
    return supervisor_pid(root) is not None


_service_checked = 0.0
_service_active = False


def service_active(*, fresh: bool = False) -> bool:
    """Активна ли служба ``robot-lidar`` (тогда свой супервизор не нужен)."""
    global _service_checked, _service_active
    now = time.monotonic()
    if not fresh and now - _service_checked < SERVICE_CHECK_S:
        return _service_active
    _service_checked = now
    _service_active = False
    from shutil import which
    if which("systemctl") is None:
        return False
    try:
        result = subprocess.run(["systemctl", "is-active", "--quiet", "robot-lidar"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                timeout=3)
    except (OSError, subprocess.SubprocessError):
        return False
    _service_active = result.returncode == 0
    return _service_active


def readiness(protocol: str = "auto") -> tuple[bool, str]:
    """Готов ли ROS 2 к запуску драйвера: ``(готов, причина если нет)``.

    Проверяем ровно то, без чего супервизор не стартует: префикс ROS 2 и
    поставленный пакет драйвера. ``rclpy`` из процесса приложения не требуется —
    супервизор запускается скриптом, который сам выставляет пути ROS 2.
    """
    if ros_env.prefix() is None:
        return False, "ROS 2 не установлен"
    packages = ros_env.installed_packages()
    if not packages:
        return False, "пакеты ROS 2 не видны (нет ament-индекса)"
    wanted = ros_env.driver_packages(protocol or "auto")
    if not wanted:
        wanted = tuple(dict.fromkeys(
            name for group in ros_env.LIDAR_DRIVER_PACKAGES.values() for name in group))
    if not set(wanted) & packages:
        return False, "нет ROS 2-пакетов драйверов лидара (%s)" % ", ".join(sorted(set(wanted)))
    return True, ""


def _rotate(path: Path, limit: int = 2_000_000) -> None:
    try:
        if path.stat().st_size > limit:
            path.replace(path.with_suffix(path.suffix + ".1"))
    except OSError:
        pass


def _open_log(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    _rotate(path)
    return open(path, "ab")


def read_logs(lines: Any = 120, root: Optional[Path] = None) -> dict[str, Any]:
    """Return bounded tails of the lidar/setup logs (never accepts a file path)."""
    try:
        count = max(10, min(500, int(lines)))
    except (TypeError, ValueError):
        count = 120
    base = Path(root or project_root()).resolve()
    files = []
    for title, relative in LOG_SOURCES:
        path = (base / relative).resolve()
        # All sources are fixed above; this assertion also guards future edits.
        if not path.is_relative_to(base):
            continue
        entry: dict[str, Any] = {
            "title": title, "path": str(relative), "exists": path.is_file(),
            "lines": [], "updated": None,
        }
        if path.is_file():
            try:
                size = path.stat().st_size
                with path.open("rb") as stream:
                    stream.seek(max(0, size - 192 * 1024))
                    text = stream.read().decode("utf-8", "replace")
                tail = text.splitlines()[-count:]
                entry["lines"] = [line[:2000] for line in tail]
                entry["updated"] = time.strftime(
                    "%Y-%m-%d %H:%M:%S", time.localtime(path.stat().st_mtime))
            except OSError as exc:
                entry["error"] = str(exc)
        files.append(entry)
    return {"ok": True, "files": files, "lines": count}


def _flock(path: Path):
    """Межпроцессный замок: два приложения (8080 и 8090) не поднимут два цикла."""
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = open(path, "a+b")
    try:
        import fcntl
    except ImportError:  # pragma: no cover - не-POSIX
        return handle
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        return None
    return handle


def start_supervisor(root: Optional[Path] = None) -> Optional[int]:
    """Запускает ``ros2_lidar.sh --keep-alive`` отдельной сессией (pid или None)."""
    base = Path(root or project_root())
    script = base / "scripts" / "ros2_lidar.sh"
    if not script.is_file():
        log.warning("Лидар: нет scripts/ros2_lidar.sh — драйвер ROS 2 не запустить")
        return None
    lock = _flock(base / LOCK_FILE)
    if lock is None:
        return None      # другой процесс уже решает этот вопрос
    try:
        if supervisor_running(base) or service_active(fresh=True):
            return None
        logger = base / "scripts" / "lidar_log.sh"
        command = ["bash", str(script), "--keep-alive", "--no-setup"]
        if logger.is_file():
            command = ["bash", str(logger), *command]
        process = subprocess.Popen(
            command, cwd=str(base), stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True, close_fds=True)
    except OSError as exc:
        log.warning("Лидар: не удалось запустить драйвер ROS 2: %s", exc)
        return None
    finally:
        try:
            lock.close()
        except OSError:
            pass
    log.info("Лидар: запускаю драйвер ROS 2 сам (pid %s, журнал %s)",
             process.pid, LIDAR_LOG)
    return process.pid


def start_setup(root: Optional[Path] = None) -> Optional[int]:
    """Запускает автоустановку ROS 2 в фоне (без терминала)."""
    base = Path(root or project_root())
    script = base / "scripts" / "ros2_setup.sh"
    if not script.is_file():
        return None
    lock = _flock(base / LOCK_FILE)
    if lock is None:
        return None
    try:
        state = ros_env.setup_state()
        if str(state.get("state") or "") in {"running", "background"}:
            return None
        with _open_log(base / BOOT_LOG) as out:
            process = subprocess.Popen(
                ["bash", str(script), "--background-build", "--no-wait", "--quiet"],
                cwd=str(base), stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True)
    except OSError as exc:
        log.warning("Лидар: не удалось запустить автоустановку ROS 2: %s", exc)
        return None
    finally:
        try:
            lock.close()
        except OSError:
            pass
    log.info("Лидар: ROS 2 не готов — запускаю установку сама (pid %s, журнал %s)",
             process.pid, BOOT_LOG)
    return process.pid


def status() -> dict[str, Any]:
    """Состояние сторожа для ``GET /api/lidar`` (``ros.boot``) и журнала."""
    data = dict(_state)
    data["enabled"] = enabled()
    data["supervisor_pid"] = supervisor_pid()
    data["install_allowed"] = install_allowed()
    if data.get("at"):
        data["age_s"] = round(time.time() - float(data["at"]), 1)
    return data


def _note(action: str) -> str:
    if action == "fresh":
        return ""
    if action == "supervisor":
        return ""
    if action == "service":
        return ""
    if action == "spawn":
        return "запускаю драйвер ROS 2 сам — открой журнал var/lidar.log, если скана нет"
    if action == "install":
        return "ставлю ROS 2 и драйверы сама — это первый запуск, несколько минут"
    if action.startswith("wait"):
        return action[5:] or ""
    return ""


def step(manager: Any = None, root: Optional[Path] = None) -> str:
    """Один проход сторожа. Возвращает действие (для журнала и тестов)."""
    now = time.time()
    _state["at"] = now
    _state["enabled"] = enabled()
    if not enabled():
        _state["state"] = "off"
        _state["action"] = "off"
        _state["note"] = ""
        return "off"
    ros = getattr(manager, "ros", None)
    source_mode = str(getattr(manager, "source_mode", "auto") or "auto")
    if source_mode == "serial":
        _state["state"] = "serial"
        _state["action"] = "serial"
        _state["note"] = ""
        return "serial"
    if ros is not None and getattr(ros, "fresh", None) and ros.fresh():
        _state["state"] = "fresh"
        _state["action"] = "fresh"
        _state["note"] = ""
        return "fresh"
    if supervisor_running(root):
        _state["state"] = "supervisor"
        _state["action"] = "supervisor"
        _state["note"] = ""
        return "supervisor"
    if service_active():
        _state["state"] = "service"
        _state["action"] = "service"
        _state["note"] = ""
        return "service"

    protocol = str(getattr(manager, "protocol", "auto") or "auto")
    ready, why = readiness(protocol)
    if ready:
        last = float(_state.get("spawn_at") or 0.0)
        if last and now - last < SPAWN_EVERY_S:
            _state["state"] = "wait"
            _state["action"] = "wait spawn"
            _state["note"] = ""
            return "wait spawn"
        pid = start_supervisor(root)
        if pid:
            # Отметку «когда запускали» ставит step, а не сам запуск: иначе
            # подменённый в тестах запуск не ограничивал бы повторы.
            _state["spawned_pid"] = pid
            _state["spawn_at"] = now
        _state["state"] = "spawn" if pid else "wait"
        _state["action"] = "spawn" if pid else "wait"
        _state["note"] = _note(_state["action"])
        return _state["action"]

    _state["state"] = "not-ready"
    _state["note"] = why
    if not install_allowed():
        _state["action"] = "wait install off"
        return "wait install off"
    last = float(_state.get("installed_at") or 0.0)
    if last and now - last < INSTALL_EVERY_S:
        _state["action"] = "wait install"
        _state["note"] = why
        return "wait install"
    pid = start_setup(root)
    if pid:
        _state["installed_at"] = now
    _state["state"] = "install" if pid else "not-ready"
    _state["action"] = "install" if pid else "wait install"
    _state["note"] = _note(_state["action"]) or why
    return _state["action"]


def _loop(manager: Any, root: Optional[Path], tick: float) -> None:
    global _last_logged_action
    while True:
        try:
            action = step(manager, root)
            if action != _last_logged_action:
                detail = str(_state.get("note") or _state.get("action") or "")
                log.info("Лидар: автоподготовка action=%s state=%s%s",
                         action, _state.get("state"), (" — " + detail) if detail else "")
                _last_logged_action = action
        except Exception:  # noqa: BLE001 — сторож не должен ронять приложение
            log.exception("Лидар: исключение в стороже драйвера ROS 2")
        time.sleep(max(1.0, tick))


def start(manager: Any = None, *, root: Optional[Path] = None,
          testing: bool = False, tick: float = TICK_S) -> Optional[threading.Thread]:
    """Поднимает сторож драйвера (в тестах и при ``RC_LIDAR_AUTOSTART=0`` — нет)."""
    global _thread
    if testing or in_tests():
        return None
    if not enabled():
        _state["state"] = "off"
        return None
    with _lock:
        if _thread is not None and _thread.is_alive():
            return _thread
        _thread = threading.Thread(target=_loop, args=(manager, root, tick),
                                   name="lidar-boot", daemon=True)
        _thread.start()
    log.info("Лидар: сторож драйвера ROS 2 запущен (проверка каждые %.0f с; журналы %s, %s)",
             tick, LIDAR_LOG, BOOT_LOG)
    return _thread


def reset_state() -> None:
    """Сбрасывает состояние сторожа (тесты и повторный запуск после остановки)."""
    global _last_logged_action
    _last_logged_action = None
    _state.update({"state": "idle", "action": "", "note": "", "at": 0.0,
                   "spawned_pid": None, "installed_at": 0.0, "spawn_at": 0.0})


if __name__ == "__main__":  # pragma: no cover - отладка вручную
    # По умолчанию только показываем состояние: проход сторожа может запустить
    # установку ROS 2, а делать это случайно (из любопытства в терминале) нельзя.
    if "--step" in sys.argv[1:]:
        print(step(), file=sys.stderr)
    print(json.dumps(status(), ensure_ascii=False, indent=1))
