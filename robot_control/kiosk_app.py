"""Отдельное приложение для дисплея на борту робота.

Корень / и /main.html сразу отдают экран киоска с АКБ, шасси и лидаром.
Телеметрия и функции бортового киоска доступны без PIN; PIN запрашивается
только при открытии грузового отсека. Руление и управление BLDC доступны
источнику MasterSource. На борту направление включается нажатием до СТОП, а
ползунок задаёт прямое Servo.write(0…150); обороты SC выводятся без обратной связи.
Аппаратный watchdog включает тормоз, если heartbeat не приходит 650 мс.
Веб-пульт оператора работает отдельно и сохраняет собственную авторизацию.

Порт по умолчанию — 8090 (переменная ``RC_KIOSK_PORT``). Запуск одной
командой — ``./run_kiosk.py`` или ``./scripts/kiosk.sh``: сервер поднимается на
8090 и сам открывает экран в браузере в киоск-режиме (без рамок, на весь
экран). Если браузер закрыли — экран открывается заново, чтобы на дисплее
робота всегда была телеметрия. Сторож экрана живёт только пока жив сервер
киоска: сервер остановили (``./scripts/stop.sh``, кнопка «Остановить всё»,
``systemctl stop robot-kiosk``) — окно закрывается и больше не открывается.

Экран при этом открывается строго в одном экземпляре, сколько бы раз ни
запустили команду (автозапуск рабочего стола, ярлык, вручную): повторный
запуск видит, что окно киоска уже открыто, и просто завершается. Для этого
держат flock-замок на время жизни окна, а браузеру дают отдельный профиль —
иначе Chromium просто передаёт адрес уже запущенной копии и сразу выходит,
а сторожевой таймер принимал бы это за «браузер закрыли» и плодил окна.

Firefox с ``--profile`` каталог сам не создаёт. На Ubuntu он ещё и snap:
скрытый ``~/.cache`` и чужой ``/tmp`` ему недоступны. И то и другое
заканчивается окном «Your Firefox profile cannot be loaded. It may be
missing or inaccessible.» Перед запуском профиль создаётся там, где
браузер его действительно прочитает.
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import re
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import replace
from pathlib import Path
from typing import Mapping, Sequence

try:
    import fcntl  # POSIX (Ubuntu на роботе): межпроцессный замок экрана.
except ImportError:  # pragma: no cover - не-POSIX платформы
    fcntl = None  # type: ignore[assignment]

from flask import Flask, Response, g, jsonify, redirect, request
from werkzeug.serving import make_server

from robot_control import estop as estop_latch
from robot_control import eventlog
from robot_control import lidar_boot
from robot_control import shutdown
from robot_control.camera import CameraManager
from robot_control.config import PROJECT_ROOT, Config
from robot_control.lidar import LidarManager, set_self_port
from robot_control.lidar_settings import apply_lidar_settings, settings_payload
from robot_control.keyboard import KeyboardControl
from robot_control.full_zero import FullZeroControl
from robot_control.neutral import NeutralControl
from robot_control.slam import (
    GUI_DIR,
    KIOSK_PAGE,
    KIOSK_URL_PATH,
    STEERING_COMMAND_HEADER,
    STEERING_COMMAND_HEADER_VALUE,
    DRIVE_COMMAND_HEADER_VALUE,
    STOP_COMMAND_HEADER_VALUE,
    build_state,
    dispatch_drive_command,
    dispatch_steering_calibration,
    dispatch_pot_mode,
    dispatch_pot_hold,
    dispatch_maneuver,
    dispatch_maneuver_config,
    engage_estop,
    release_estop,
    note_pin_result,
    dispatch_steering_command,
    dispatch_steering_zero,
    kiosk_log_payload,
    lidar_for_screen,
    lidar_status_payload,
    start_lidar,
    steering_control_info,
)
from robot_control.slam_map import SlamMapService, map_paths
from robot_control.navigation import NavigationError, NavigationService
from robot_control.kinect import DepthError, KinectDepthService
from robot_control.exploration import ExplorationError, ExplorationService
from robot_control.sensor_layout import layout_payload, kinect_spec, rear_kinect_enabled
from robot_control import sensor_module as sensor_module_client
from robot_control import calibration as geometry_calibration

log = logging.getLogger("robot_control.kiosk")

#: Как часто карта лидара впитывает очередной скан, с.
MAP_POLL_S = 0.35

#: Порт бортового экрана. 8080 занят пультом оператора, поэтому киоск живёт рядом.
DEFAULT_PORT = 8090

#: Имя каталога профиля киоска — и в кэше, и в ~/snap/<имя>/common.
PROFILE_DIR_NAME = "robot-kiosk-browser"

# Обёртка Ubuntu («/usr/bin/firefox» → snap) и прямой бинарник snap.
_SNAP_BIN_RE = re.compile(rb"/snap/bin/([A-Za-z0-9][A-Za-z0-9._-]*)")
_SNAP_RUN_RE = re.compile(
    rb"\bsnap\s+run\s+(?:--\S+\s+)*([A-Za-z0-9][A-Za-z0-9._-]*)"
)
_FIREFOX_LOCK_PID_RE = re.compile(r"\+(\d+)\s*$")

# Глушит окна первого запуска, которые иначе закрывают телеметрию.
# Пишется только если своего user.js ещё нет — ручные правки не затираем.
_FIREFOX_USER_JS = """\
// Профиль бортового экрана робота. Firefox читает файл при каждом старте.
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("browser.aboutwelcome.enabled", false);
user_pref("browser.startup.homepage_override.mstone", "ignore");
user_pref("startup.homepage_welcome_url", "");
user_pref("startup.homepage_welcome_url.additional", "");
user_pref("browser.startup.firstrunSkipsHomepage", true);
user_pref("trailhead.firstrun.didSeeAboutWelcome", true);
user_pref("datareporting.policy.dataSubmissionEnabled", false);
user_pref("datareporting.healthreport.uploadEnabled", false);
user_pref("toolkit.telemetry.reportingpolicy.firstRun", false);
user_pref("app.normandy.enabled", false);
user_pref("app.normandy.first_run", false);
user_pref("browser.messaging-system.whatsNewPanel.enabled", false);
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("toolkit.startup.max_resumed_crashes", -1);
"""

#: Браузеры, которые умеют киоск-режим (первый найденный и используется).
BROWSER_CANDIDATES = (
    "chromium-browser",
    "chromium",
    "google-chrome",
    "google-chrome-stable",
    "microsoft-edge",
    "firefox",
)

#: Сколько секунд ждать, пока сервер ответит, прежде чем открывать браузер.
HEALTH_TIMEOUT_SEC = 20.0
#: Пауза перед повторной попыткой, если браузер не найден.
BROWSER_RETRY_SEC = 30.0
#: Пауза перед повторным открытием закрытого браузера.
BROWSER_REOPEN_SEC = 2.0
#: Браузер, проживший меньше этого, не «закрыт пользователем», а скорее всего
#: сразу вышел (передал адрес другой копии/упал): пауза перед повтором растёт.
BROWSER_QUICK_EXIT_SEC = 10.0
#: Верхняя граница растущей паузы между быстрыми перезапусками браузера.
BROWSER_REOPEN_MAX_SEC = 30.0
#: Сколько ждать закрытия окна киоска, прежде чем добить его SIGKILL.
BROWSER_CLOSE_TIMEOUT_SEC = 3.0
#: Сколько ждать освобождения замка экрана, прежде чем решить, что окно киоска
#: уже открыто другим запуском (небольшая пауза — на случай смены владельца:
#: старый показ как раз завершается, когда стартует новый).
DISPLAY_CLAIM_TIMEOUT_SEC = 3.0
#: Как часто сторож экрана проверяет, жив ли сервер киоска, пока окно открыто.
SERVER_CHECK_SEC = 2.0
#: Сколько сервер экрана может не слушать порт, прежде чем показ прекращается.
#: Запас нужен на перезапуск: обновление (``SIGHUP`` → ``os.execv``) рвёт сокет
#: на пару секунд, и это не остановка — гасить экран в этот момент нельзя.
SERVER_GONE_SEC = 15.0


# ---------------------------------------------------------------- окружение --- #
def _env_flag(name: str, default: bool) -> bool:
    """Флаг из окружения: 1/true/yes/on → True, 0/false/no/off → False."""
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on", "да"}


def kiosk_port() -> int:
    """Порт бортового экрана (``RC_KIOSK_PORT``, по умолчанию 8090)."""
    raw = os.environ.get("RC_KIOSK_PORT", "").strip()
    try:
        return int(raw) if raw else DEFAULT_PORT
    except ValueError:
        log.warning("RC_KIOSK_PORT=%r не число — беру %s", raw, DEFAULT_PORT)
        return DEFAULT_PORT


def display_available(env: Mapping[str, str] | None = None) -> bool:
    """Есть ли графическая сессия: только тогда браузер имеет смысл открывать."""
    env = os.environ if env is None else env
    return bool(env.get("WAYLAND_DISPLAY") or env.get("DISPLAY"))


def find_browser(explicit: str | None = None) -> str | None:
    """Путь к браузеру для киоск-режима (``RC_KIOSK_BROWSER`` переопределяет)."""
    candidate = (explicit or os.environ.get("RC_KIOSK_BROWSER") or "").strip()
    if candidate:
        path = Path(candidate).expanduser()
        if path.is_file() and os.access(path, os.X_OK):
            return str(path)
        found = shutil.which(candidate)
        if found:
            return found
        # Ошиблись в имени — экран всё равно должен открыться: ищем сами.
        log.warning("RC_KIOSK_BROWSER=%s не найден — ищу браузер сам", candidate)
    for name in BROWSER_CANDIDATES:
        found = shutil.which(name)
        if found:
            return found
    return None


def kiosk_profile_dir(env: Mapping[str, str] | None = None) -> Path:
    """Отдельный профиль браузера киоска (``RC_KIOSK_PROFILE_DIR``).

    Свой профиль жизненно важен, чтобы экран не дублировался: без него
    Chromium/Chrome при повторном запуске не открывает новое окно сам, а
    передаёт адрес уже работающей копии браузера и тут же завершается.
    Сторожевой таймер видел «процесс вышел» → «браузер закрыли» → открывал
    экран заново — и так без конца, плодя окна. С отдельным профилем
    запущенный процесс и есть окно киоска: живёт, пока окно открыто.

    Это желаемый путь. Перед запуском ``prepare_browser_profile`` создаёт
    каталог и, если браузер — snap, переносит профиль из скрытого
    ``~/.cache`` в ``~/snap/<имя>/common``: иначе Firefox не может его
    прочитать.
    """
    env = os.environ if env is None else env
    explicit = (env.get("RC_KIOSK_PROFILE_DIR") or "").strip()
    if explicit:
        return Path(explicit).expanduser()
    cache = (env.get("XDG_CACHE_HOME") or "").strip()
    base_dir = Path(cache).expanduser() if cache else Path.home() / ".cache"
    return base_dir / PROFILE_DIR_NAME


def _absolute(path: Path | str) -> Path:
    """Абсолютный путь без «~» и без хвостовых ссылок.

    Firefox с относительным ``--profile`` ищет каталог от своего cwd и,
    не найдя его, показывает то же окно «profile cannot be loaded».
    """
    expanded = os.path.expanduser(str(path))
    try:
        return Path(os.path.realpath(expanded))
    except OSError:
        return Path(os.path.abspath(expanded))


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def snap_name(exe: str) -> str | None:
    """Имя snap-пакета браузера или None, если это обычная сборка.

    На Ubuntu ``firefox`` в PATH — это ``/usr/bin/firefox``: скрипт, который
    запускает ``/snap/bin/firefox`` (или ``snap run firefox``). По одному
    пути этого не видно, а snap не читает скрытые каталоги домашнего
    каталога. Тот же приём у ``chromium-browser``.
    """
    raw = os.path.expanduser(exe)
    candidates = [raw]
    try:
        resolved = os.path.realpath(raw)
    except OSError:
        resolved = raw
    if resolved not in candidates:
        candidates.append(resolved)
    for candidate in candidates:
        name = _snap_name_from_path(candidate)
        if name:
            return name
    try:
        with open(raw, "rb") as handle:
            head = handle.read(8192)
    except OSError:
        return None
    if not head.startswith(b"#!"):
        return None
    match = _SNAP_BIN_RE.search(head) or _SNAP_RUN_RE.search(head)
    if not match:
        return None
    token = match.group(1).decode("ascii", errors="ignore").split(".", 1)[0]
    return token or None


def _snap_name_from_path(path: str) -> str | None:
    parts = path.split("/")
    # ['', 'snap', 'bin', 'firefox'] или ['', 'snap', 'firefox', 'current', ...]
    if len(parts) >= 4 and parts[1] == "snap":
        if parts[2] == "bin" and parts[3]:
            return parts[3].split(".", 1)[0]
        if parts[2] not in {"bin", "core", "snapd"}:
            return parts[2]
    return None


def _snap_can_read(path: Path, snap: str) -> bool:
    """Видит ли строго ограниченный snap этот каталог.

    Домашний интерфейс snap не открывает скрытые каталоги (``.cache``,
    ``.local``) и чужой ``/tmp`` (у snap свой). Гарантированно доступен
    ``~/snap/<имя>/``; обычные нескрытые пути в $HOME — тоже, если подключён
    интерфейс home. Чужой ``~/snap/<другой>/`` — нет.
    """
    home = _absolute(Path.home())
    path = _absolute(path)
    own = home / "snap" / snap
    if _is_relative_to(path, own):
        return True
    if not _is_relative_to(path, home):
        return False
    rel = path.relative_to(home)
    if not rel.parts or rel.parts[0] == "snap":
        return False
    return all(not part.startswith(".") for part in rel.parts)


def _writable_profile(path: Path) -> Path | None:
    """Создать каталог профиля с правами только владельца. None — не вышло."""
    if path.exists() and not path.is_dir():
        log.error("профиль %s существует, но это не каталог — браузер его не откроет", path)
        return None
    try:
        path.mkdir(parents=True, exist_ok=True)
        os.chmod(path, 0o700)
    except OSError as exc:
        log.warning("не удалось создать профиль %s: %s", path, exc)
        return None
    if not os.access(path, os.R_OK | os.W_OK | os.X_OK):
        log.warning("профиль %s недоступен на запись", path)
        return None
    return path


def prepare_browser_profile(exe: str, profile_dir: Path | str | None = None) -> Path:
    """Каталог профиля, который этот браузер сможет открыть. Каталог создаётся.

    Без этого шага Firefox показывает: «Your Firefox profile cannot be
    loaded. It may be missing or inaccessible.» Каталог ``--profile`` должен
    уже существовать, а у snap-сборки он ещё и не может лежать в ``~/.cache``
    или в ``/tmp``.
    """
    requested = _absolute(profile_dir if profile_dir is not None else kiosk_profile_dir())
    snap = snap_name(exe)
    candidates = [requested]
    if snap and not _snap_can_read(requested, snap):
        # Сначала каталог самого snap (виден даже без интерфейса home),
        # затем обычная нескрытая папка в $HOME — её home-интерфейс читает.
        candidates = [
            _absolute(Path.home() / "snap" / snap / "common" / PROFILE_DIR_NAME),
            _absolute(Path.home() / PROFILE_DIR_NAME),
        ]
    chosen: Path | None = None
    for candidate in candidates:
        chosen = _writable_profile(candidate)
        if chosen is not None:
            break
    if chosen is None:
        log.error(
            "не удалось подготовить профиль %s — Firefox покажет: "
            "Your Firefox profile cannot be loaded. It may be missing or inaccessible.",
            requested,
        )
        return candidates[0]
    if chosen != requested:
        log.warning(
            "профиль %s этому браузеру недоступен — беру %s. "
            "Иначе Firefox пишет: Your Firefox profile cannot be loaded. "
            "It may be missing or inaccessible.",
            requested, chosen,
        )
    if "firefox" in os.path.basename(exe).lower():
        _ensure_firefox_profile(chosen)
    return chosen


def _ensure_firefox_profile(profile: Path) -> None:
    """Минимальный профиль: без мастера первого запуска и без мёртвого замка."""
    user_js = profile / "user.js"
    if not user_js.exists():
        try:
            user_js.write_text(_FIREFOX_USER_JS, encoding="utf-8")
            os.chmod(user_js, 0o600)
        except OSError as exc:
            log.warning("не удалось записать %s: %s", user_js, exc)
    _clear_stale_firefox_lock(profile)


def _firefox_lock_pid(lock: Path) -> int | None:
    try:
        raw = os.readlink(lock) if lock.is_symlink() else lock.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    match = _FIREFOX_LOCK_PID_RE.search(raw.strip())
    if not match:
        return None
    return int(match.group(1))


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # процесс есть, просто чужой
    except OSError:
        return False
    return True


def _clear_stale_firefox_lock(profile: Path) -> None:
    """Снять замок упавшего Firefox, чтобы киоск не упёрся в «already running».

    Живой процесс не трогаем: замок с его pid остаётся.
    """
    lock = profile / "lock"
    pid = _firefox_lock_pid(lock)
    if pid is None or _pid_alive(pid):
        return
    for name in ("lock", ".parentlock", "parent.lock"):
        try:
            (profile / name).unlink()
        except FileNotFoundError:
            pass
        except OSError as exc:
            log.debug("не удалось снять замок %s: %s", profile / name, exc)
    log.info("снят устаревший замок профиля Firefox (процесс %s уже не жив)", pid)


def browser_command(exe: str, url: str, profile_dir: Path | str | None = None) -> list[str]:
    """Команда запуска браузера без рамок и адресной строки (киоск-режим).

    Каталог профиля должен уже существовать и быть браузеру виден — это
    делает ``prepare_browser_profile`` (Firefox сам каталог не создаёт).
    """
    profile = str(profile_dir) if profile_dir is not None else str(kiosk_profile_dir())
    base = os.path.basename(exe).lower()
    if "firefox" in base:
        # --no-remote + свой профиль: не цепляться к чужому окну Firefox
        # (иначе процесс-лаунчер сразу выходит, как и у Chromium).
        return [exe, "--kiosk", "--no-remote", "--profile", profile, url]
    return [
        exe,
        "--kiosk",
        "--app=" + url,
        f"--user-data-dir={profile}",
        "--no-first-run",
        "--noerrdialogs",
        "--disable-infobars",
        "--autoplay-policy=no-user-gesture-required",
    ]


# ------------------------------------------------------------------- сервер -- #
def create_kiosk_app(config: Config | None = None) -> Flask:
    config = config or Config.from_env()
    if not config.testing and config.slam_source == "master":
        os.environ["RC_USB_DISCOVERY_MODE"] = "1"
    app = Flask(__name__)
    from .device_registry import DeviceRegistry
    device_registry = DeviceRegistry()
    app.extensions["rc_devices"] = device_registry
    # Configure the onboard owner before any lazy camera manager is created.
    CameraManager.configure_registry(device_registry)
    app.config.update(
        SECRET_KEY=config.secret_key or secrets.token_urlsafe(48),
        SESSION_COOKIE_HTTPONLY=True,
        SESSION_COOKIE_SAMESITE=config.cookie_samesite,
        SESSION_COOKIE_SECURE=config.cookie_secure,
    )

    # Панель «Логи» показывает настоящий журнал процесса: записи логирования
    # robot_control.* и явные события маршрутов. Подключаем его первым делом,
    # чтобы в журнал попало всё, включая неудачный старт источника данных.
    eventlog.install()

    from .battery import BatteryManager, report_response
    from . import bms_setup
    battery = BatteryManager(os.environ.get("RC_BMS_PORT", "").strip())
    app.extensions["rc_battery"] = battery
    usb_assignment_lock = threading.RLock()

    from .connections import ConnectionsMonitor
    from .system_monitor import SystemMonitor, SystemState
    # Один читатель /proc и /sys на процесс: экран «Подключения» и маленький
    # индикатор на бортовом экране показывают одни и те же числа.
    system_state = SystemState()
    system = SystemMonitor(system_state)
    app.extensions["rc_system"] = system
    connections = ConnectionsMonitor(system=system_state)
    app.extensions["rc_connections"] = connections

    @app.get("/api/connections")
    def api_connections():
        response = jsonify(connections.snapshot())
        response.headers["Cache-Control"] = "no-store"
        return response

    @app.get("/api/pc_state")
    def api_system():
        # Только чтение: температура, память, диск и загрузка процессора.
        response = jsonify(system.snapshot())
        response.headers["Cache-Control"] = "no-store"
        return response

    @app.get("/api/battery")
    def api_battery():
        snap = battery.snapshot()
        snap["setupAvailable"] = True
        return jsonify({"ok": True, "battery": snap}), 200, {"Cache-Control": "no-store"}

    @app.get("/api/battery/report.csv")
    def api_battery_report():
        return report_response(battery.snapshot())

    # ------------------- Подключение BMS со страницы «АКБ» ------------------- #
    # Настройка порта и разовая проверка BMS — сервисные операции, не команды
    # движения. На бортовом экране они доступны без PIN; мастер/лидар/шасси
    # защищены от выбора на уровне bms_setup.
    @app.get("/api/battery/setup")
    def api_battery_setup():
        payload = bms_setup.setup_payload(battery)
        payload["requiresPin"] = False  # совместимость со старыми клиентами
        return jsonify(payload), 200, {"Cache-Control": "no-store"}

    @app.post("/api/battery/probe")
    def api_battery_probe():
        body = request.get_json(silent=True) or {}
        port = str(body.get("port") or "").strip()
        if (port and battery._requires_wheel_check and
                (battery._manual_source is None or not battery._manual_source.manual_bms_port_allowed(port))):
            return jsonify({"ok": False, "error": "Порт ещё не прошёл проверку USB (или его читает "
                                                   "колесо/лидар). Сначала назначьте его BMS на "
                                                   "странице «Устройства» — там выбор подтверждает "
                                                   "оператор"}), 409
        with usb_assignment_lock:
            result = bms_setup.probe_port(port)
        eventlog.record("info" if result.get("found") else "warn",
                        "BMS: проверка %s — %s" % (port or "?", result.get("summary") or result.get("error")
                                                   or "JBD не ответила"),
                        source="АКБ")
        return jsonify(result), 200 if result.get("ok") else 409

    @app.post("/api/battery/scan")
    def api_battery_scan():
        return jsonify({"ok": False, "error": "Автопоиск BMS отключён: выберите порт вручную"}), 410

    @app.post("/api/battery/port")
    def api_battery_port():
        body = request.get_json(silent=True) or {}
        if "port" not in body or not isinstance(body.get("port"), str):
            return jsonify({"ok": False, "error": "нужно поле port"}), 400
        try:
            with usb_assignment_lock:
                result = bms_setup.apply_port(battery, body["port"])
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        except OSError as exc:
            return jsonify({"ok": False, "error": "не удалось записать .env: %s" % exc}), 500
        eventlog.record("ok", "BMS: порт %s сохранён в .env, подключение…" % (result["port"] or "—")
                        if result["port"] else "BMS отключена: RC_BMS_PORT очищен", source="АКБ")
        return jsonify(result)

    # Телеметрия и команды бортового киоска открыты без PIN; отдельный
    # веб-пульт оператора использует свой механизм авторизации.
    try:
        state = build_state(
            mode=config.slam_source,
            ports=config.slam_ports,
            wheel_ports=config.wheel_ports,
            lock_file=config.slam_lock_file,
            pin=config.slam_pin,
            max_attempts=config.slam_max_attempts,
            lock_sec=config.slam_lock_seconds,
            device_registry=device_registry,
        )
        log.info("киоск: интерфейс подключён (источник: %s)",
                 getattr(state.source, "name", "?"))
    except Exception as exc:  # noqa: BLE001
        log.error("киоск: не удалось подключить источник: %s", exc)
        state = None
    app.extensions["rc_slam"] = state
    if config.slam_source == "master":
        wheel_source = getattr(state, "source", None)
        battery.use_manual_discovery(wheel_source if getattr(wheel_source, "auto_discovery", False) else None)
        if not getattr(wheel_source, "auto_discovery", False):
            log.error("BMS: жду владельца проверки USB-портов колёс")
    if not config.testing:
        battery.start()

    # Printer is owned by the onboard process. Inventory/preview are passive;
    # only an explicit request starts a single bounded USB-UART print job.
    from .printer import PrinterError, PrinterService
    printer = PrinterService(device_registry, battery, lambda: getattr(state, "source", None))
    app.extensions["rc_printer"] = printer

    # A silent LDS-01 cannot be positively auto-identified. The operator may
    # request exactly one start only after a physical cable/model check; CRC
    # frames are still required before the USB owner can publish an identity.
    from .lidar_start import LidarStartError, LidarStartService
    lidar_start = LidarStartService(device_registry, battery, lambda: getattr(state, "source", None))
    app.extensions["rc_lidar_start"] = lidar_start

    def printer_body():
        import json
        if request.headers.get("X-Robot-Control") != "printer":
            raise PrinterError("Требуется запрос со страницы принтера.", 400)
        if request.content_length is not None and request.content_length > 4096:
            raise PrinterError("Слишком большой запрос.", 413)
        raw = request.stream.read(4097)
        if len(raw) > 4096:
            raise PrinterError("Слишком большой запрос.", 413)
        try:
            body = json.loads(raw)
        except (ValueError, UnicodeError):
            raise PrinterError("Ожидается JSON-объект.", 400) from None
        if not isinstance(body, dict):
            raise PrinterError("Ожидается JSON-объект.", 400)
        return body

    @app.get("/api/printer")
    def api_printer_status():
        return jsonify(printer.status()), 200, {"Cache-Control": "no-store"}

    @app.get("/api/printer/preview")
    def api_printer_preview():
        try:
            return jsonify(printer.preview(request.args.get("level", "medium"))), 200, {"Cache-Control": "no-store"}
        except PrinterError as exc:
            return jsonify(ok=False, error=str(exc)), exc.status

    @app.post("/api/printer/config")
    def api_printer_config():
        try:
            body = printer_body()
            with usb_assignment_lock:
                result = printer.configure(body)
        except PrinterError as exc:
            return jsonify(ok=False, error=str(exc)), exc.status
        except OSError as exc:
            return jsonify(ok=False, error="Не удалось сохранить настройки принтера: " + str(exc)), 500
        eventlog.record("info", "Принтер: настройки сохранены; USB-UART требует физической проверки", source="АКБ")
        return jsonify(result), 200, {"Cache-Control": "no-store"}

    @app.post("/api/printer/print")
    @app.post("/api/printer/test")
    def api_printer_print():
        try:
            result = printer.start(printer_body(), test=request.path.endswith("/test"))
        except PrinterError as exc:
            return jsonify(ok=False, error=str(exc)), exc.status
        eventlog.record("info", "Принтер: задание принято; результат печати не подтверждён", source="АКБ")
        return jsonify(result), 202, {"Cache-Control": "no-store"}

    @app.post("/api/printer/cancel")
    def api_printer_cancel():
        try:
            if printer_body() != {}:
                raise PrinterError("Неверный запрос остановки.", 400)
            return jsonify(printer.cancel()), 200, {"Cache-Control": "no-store"}
        except PrinterError as exc:
            return jsonify(ok=False, error=str(exc)), exc.status

    def note_case_pin_result(result: dict, *, where: str) -> None:
        """Журналировать PIN кейса без самого кода и без обещаний авторизации руля."""
        reasons = {
            "wrong": "неверный PIN",
            "format": "PIN должен быть 4…8 цифр",
            "blocked": "ввод заблокирован после серии ошибок",
        }
        if result.get("ok"):
            eventlog.record("ok", "PIN принят (%s)" % where, source="pin")
        else:
            reason = reasons.get(str(result.get("reason") or ""), "проверка не пройдена")
            eventlog.record("warn", "PIN не принят (%s): %s" % (where, reason), source="pin")

    # Лидар стартует сразу, а не по первому запросу экрана: .env уже прочитан,
    # а USB-порт должен занять именно этот процесс (пульт возьмёт скан по HTTP).
    app.extensions["rc_lidar"] = start_lidar(
        testing=config.testing, port=kiosk_port(),
    )
    if app.extensions["rc_lidar"] is not None:
        app.extensions["rc_lidar"].device_registry = device_registry

    # --------------------- Карта лидара (страница экрана) ------------------ #
    # Карта живёт в сервере: её строит тот же лидар, что читает экран, а файл
    # карты перезаписывается на рабочем столе. Сброс карты файл не удаляет —
    # он просто перезаписывается пустым, путь и имя остаются прежними.
    map_service = SlamMapService()
    map_service.load_settings_file()
    app.extensions["rc_map"] = map_service
    # Владелец карты и UART — только бортовой процесс (8090). Пульт 8080
    # использует прокси и не создаёт второго исполнителя моторов.
    nav_lidar = None if config.testing else (app.extensions.get("rc_lidar") or LidarManager.get_instance())
    navigation = NavigationService(map_service, getattr(state, "source", None), nav_lidar)
    app.extensions["rc_navigation"] = navigation
    # Модуль датчиков (свет, замок двери, карты доступа, компас) — отдельная
    # плата на своём USB. Порт задаётся вручную (RC_SENSOR_PORT), автопоиска нет:
    # без порта экран показывает «не подключён», а не чужое устройство.
    sensor_module = None
    if not config.testing:
        sensor_module = sensor_module_client.open_sensor_module(
            log=lambda message: eventlog.record("warn", message, source="sensor"))
        if sensor_module is not None:
            sensor_module.start()
            eventlog.record("ok", "модуль датчиков: %s" % sensor_module.port, source="sensor")
    app.extensions["rc_sensor_module"] = sensor_module
    # PIN-хранилище кейса нужно и замку двери модуля датчиков: открытие двери
    # так же требует PIN, иначе плата стала бы обходным путём.
    app.extensions["rc_vault"] = getattr(state, "vault", None)

    # Kinect 360 и автообследование живут только здесь, на владельце железа:
    # пульт 8080 получает статус и команды через proxy, а не второго исполнителя.
    kinect_service = KinectDepthService()
    app.extensions["rc_kinect"] = kinect_service
    # Второй Kinect сзади — опция (RC_KINECT_REAR_ENABLE=1). Передний всегда
    # доступен под прежним ключом rc_kinect, чтобы старые API и тесты не менялись.
    kinect_services = {"front": kinect_service}
    if rear_kinect_enabled():
        kinect_services["rear"] = KinectDepthService(spec=kinect_spec("rear"))
    app.extensions["rc_kinects"] = kinect_services
    camera_provider = lambda: CameraManager.get_instance()  # noqa: E731 — ленивый синглтон
    exploration = ExplorationService(
        map_service, navigation, camera_provider=camera_provider, kinect=kinect_service,
        landmarks=None,
    )
    app.extensions["rc_exploration"] = exploration
    keyboard = KeyboardControl(lambda: getattr(state, "source", None), dispatch_drive_command,
                               dispatch_steering_command, estop_latch.latched)
    app.extensions["rc_keyboard"] = keyboard
    neutral = NeutralControl(lambda: getattr(state, "source", None), interrupt_keyboard=keyboard.interrupt)
    app.extensions["rc_neutral"] = neutral
    full_zero = FullZeroControl(lambda: getattr(state, "source", None), estop_latch.latched,
                                neutral.status, lambda: navigation.active)
    app.extensions["rc_full_zero"] = full_zero

    def device_snapshot(*, refresh=False):
        # Reuse each subsystem's single owner; this service never opens a
        # serial/video port or actively probes an unknown USB device.
        errors = []
        def read(name, callback, fallback):
            try:
                return callback()
            except Exception as exc:  # noqa: BLE001 — one broken driver must not hide all devices
                errors.append(name + ": " + (str(exc) or type(exc).__name__))
                return fallback
        source = getattr(state, "source", None) if state is not None else None
        wheels = read("колёса", lambda: source.read() if source else {}, {})
        cameras = read("камеры", lambda: CameraManager.get_instance().list_cameras(), [])
        lidar_manager = app.extensions.get("rc_lidar") or LidarManager._instance
        lidar_status = read("лидар", lambda: lidar_manager.get_status() if lidar_manager else {}, {})
        bms_status = read("BMS", battery.snapshot, {})
        depth = read("Kinect", kinect_service.status, {})
        result = device_registry.snapshot(wheels=wheels, cameras=cameras, lidar=lidar_status,
                                          bms=bms_status, kinect=depth, refresh=refresh)
        result["producerErrors"] = errors
        return result

    if not config.testing:
        device_stop = threading.Event()
        app.extensions["rc_devices_stop"] = device_stop
        def device_worker():
            while not device_stop.is_set():
                try:
                    device_snapshot()
                except Exception:  # noqa: BLE001 — monitoring cannot stop device owners
                    log.exception("Не удалось обновить реестр устройств")
                device_stop.wait(2.0)
        threading.Thread(target=device_worker, name="device-registry", daemon=True).start()

    motion_gate = threading.RLock()
    motion_routes = {
        "/api/kiosk_neutral", "/api/kiosk_drive", "/api/kiosk_keyboard", "/api/kiosk_steer",
        "/api/kiosk_zero", "/api/kiosk_calibrate", "/api/kiosk_pot_mode",
        "/api/kiosk_pot_hold", "/api/kiosk_maneuver",
        "/api/kiosk_maneuver_config", "/api/navigation/start", "/api/navigation/resume",
        "/api/navigation/cancel", "/api/navigation/estop", "/api/estop", "/api/estop/release",
        "/api/exploration/start", "/api/exploration/stop", "/api/system/stop",
    }

    @app.before_request
    def guard_neutral_motion():
        # A request that passed the interlock must finish before brake
        # recovery; commands arriving while locked must not run afterwards.
        if request.method != "POST" or request.path not in motion_routes:
            return None
        arrival_epoch, arrival_locked = neutral.ticket()
        zero_epoch, zero_locked = full_zero.ticket()
        body = request.get_json(silent=True)
        body = body if isinstance(body, dict) else {}
        if request.path == "/api/kiosk_drive":
            value = body.get("value") if "value" in body else body.get("percent")
            is_stop = type(value) in (int, float) and value == 0
        elif request.path in {"/api/kiosk_keyboard", "/api/kiosk_maneuver"}:
            action = body.get("action")
            is_stop = isinstance(action, str) and action in {"stop", "end"}
        else:
            is_stop = request.path in {"/api/estop", "/api/navigation/cancel", "/api/navigation/estop",
                                       "/api/exploration/stop", "/api/system/stop"}
        # STOP gets an immediate best-effort brake even if an earlier HTTP
        # motion request is still finishing. The normal STOP runs again under
        # the gate so that this in-flight request cannot overtake the final b.
        marker = ("steering" if request.path == "/api/kiosk_maneuver" else
                  "navigation" if request.path.startswith("/api/navigation/") else
                  "exploration" if request.path.startswith("/api/exploration/") else
                  STOP_COMMAND_HEADER_VALUE if request.path == "/api/system/stop" else "drive")
        global_stop = (request.path != "/api/kiosk_drive" or body.get("module") in (None, "ALL", "all")
                       or arrival_locked or zero_locked)
        if is_stop and global_stop and request.headers.get(STEERING_COMMAND_HEADER) == marker:
            if request.path == "/api/kiosk_keyboard" and keyboard.preempt_stop(body):
                g.keyboard_preempt = body
            brake = getattr(getattr(state, "source", None), "_brake_all", None)
            if callable(brake):
                try:
                    brake()
                except Exception:  # noqa: BLE001 — firmware watchdog is the backstop
                    pass
            if zero_locked:
                # The ordinary traction STOP does not stop a calibration stepper.
                # Revoke the wizard and send s + tagged brake immediately.
                full_zero.interrupt("stop")
                g.full_zero_stop = True
        motion_gate.acquire()
        g.neutral_motion_gate = True
        if request.path == "/api/kiosk_neutral":
            # Recovery is always allowed; the endpoint rejects legacy release
            # commands independently of the full-zero session state.
            return None
        current = neutral.status()
        if is_stop:
            if current["locked"]:
                neutral.interrupt()
            return None
        if zero_locked or full_zero.ticket()[1] or zero_epoch != full_zero.ticket()[0]:
            return jsonify(ok=False, reason="full_zero_active"), 409
        if current["locked"] or arrival_locked or arrival_epoch != neutral.ticket()[0]:
            return jsonify(ok=False, reason="neutral_locked"), 409
        return None

    @app.after_request
    def repeat_full_zero_stop(response):
        # The endpoint's normal b/STOP may have cleared the early command.
        # Send the final s + tagged b after that endpoint, still under the gate.
        if getattr(g, "full_zero_stop", False):
            full_zero.interrupt("stop", repeat=True)
        return response

    @app.teardown_request
    def release_neutral_motion_gate(_error):
        if getattr(g, "keyboard_preempt", None):
            keyboard.finish_preempted_stop(g.keyboard_preempt)
            g.keyboard_preempt = None
        if getattr(g, "neutral_motion_gate", False):
            g.neutral_motion_gate = False
            motion_gate.release()

    @app.before_request
    def interrupt_keyboard_on_other_motion():
        # STOP, E-STOP, ручные и автономные команды всегда старше WASD.
        # Отзыв под тем же замком не даст запоздавшему heartbeat снова поехать.
        if request.method == "POST" and request.path in {
            "/api/kiosk_drive", "/api/kiosk_steer", "/api/kiosk_zero",
            "/api/kiosk_calibrate", "/api/kiosk_pot_mode", "/api/kiosk_pot_hold",
            # Geometry must never change underneath an armed keyboard pivot.
            "/api/kiosk_maneuver", "/api/kiosk_maneuver_config",
            "/api/estop", "/api/navigation/start", "/api/navigation/resume",
            "/api/navigation/cancel", "/api/navigation/estop",
            "/api/exploration/start", "/api/exploration/stop", "/api/system/stop",
        }:
            # A global system STOP must retire live steering under the motion
            # gate too; the early best-effort brake alone can be overtaken by
            # an HTTP heartbeat that was already executing.
            if request.path != "/api/system/stop" or request.headers.get(STEERING_COMMAND_HEADER) == STOP_COMMAND_HEADER_VALUE:
                keyboard.interrupt()

    def _map_worker(stop_event: threading.Event) -> None:
        """Фоновая нить карты: впитывает сканы и перезаписывает файл карты.

        Каждый скан лидара вписывается ровно один раз (по времени прихода), а
        не по каждому тику опроса: иначе одна и та же стена считалась бы
        десятки раз подряд и карта «зарастала» лишним.
        """
        last_stamp = 0.0
        while not stop_event.is_set():
            try:
                manager = LidarManager.get_instance()
                if manager is not None:
                    scan_info = manager.get_scan_info()
                    points, stamp = scan_info["points"], scan_info["stamp"]
                    if points and stamp and stamp != last_stamp:
                        last_stamp = stamp
                        # Сохраняем время получения скана, не время окончания
                        # медленного scan matching: устаревшая поза не свежая.
                        map_service.ingest(points, stamp=stamp, trusted=scan_info["trusted"])
                # Kinect 360: отдельный псевдоскан в ту же сетку, ровно один
                # раз на кадр. Задний датчик (если включён) вносит точки со
                # своим смещением и курсом. Позу задаёт только лидар.
                for unit, service in kinect_services.items():
                    depth = service.consume()
                    if depth is None:
                        continue
                    depth_points, depth_stamp = depth
                    map_service.ingest_kinect(depth_points, stamp=depth_stamp,
                                              spec=kinect_spec(unit))
                map_service.maybe_autosave()
            except Exception as exc:  # noqa: BLE001 — карта не должна ронять сервер
                log.debug("карта лидара: %s", exc)
            stop_event.wait(MAP_POLL_S)

    if not config.testing:
        map_stop = threading.Event()
        app.extensions["rc_map_stop"] = map_stop
        threading.Thread(target=_map_worker, args=(map_stop,),
                         name="kiosk-map", daemon=True).start()

    def _send(name: str):
        from flask import send_from_directory
        response = send_from_directory(GUI_DIR, name)
        response.headers["Cache-Control"] = "no-store"
        return response

    # ---------- маршруты ----------
    @app.get("/")
    def root():
        return redirect(KIOSK_URL_PATH)

    @app.get(KIOSK_URL_PATH)
    def kiosk():
        return _send(KIOSK_PAGE)

    @app.get("/devices")
    def device_page():
        return _send("devices.html")

    @app.get("/sensors")
    def sensor_page():
        # Отдельный экран модуля датчиков: связь, свет, замок, карты, компас.
        # Плата может быть не подключена — страница всё равно открывается.
        return _send("sensors.html")

    @app.get("/api/devices")
    def api_devices():
        return jsonify(device_snapshot(refresh=request.args.get("refresh") == "1")), 200, {"Cache-Control": "no-store"}

    @app.post("/api/devices/hint")
    def api_device_hint():
        import json
        if request.headers.get("X-Robot-Control") != "devices":
            return jsonify(ok=False, error="Требуется запрос со страницы устройств"), 400
        if request.content_length is not None and request.content_length > 4096:
            return jsonify(ok=False, error="Слишком большой запрос"), 413
        raw = request.stream.read(4097)
        if len(raw) > 4096:
            return jsonify(ok=False, error="Слишком большой запрос"), 413
        try:
            body = json.loads(raw)
            if not isinstance(body, dict) or set(body) != {"role", "port"}:
                raise ValueError("Нужны только поля role и port")
            result = device_registry.set_hint(body["role"], body["port"])
        except (ValueError, TypeError, UnicodeError) as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except OSError as exc:
            return jsonify(ok=False, error="Не удалось сохранить подсказку: " + str(exc)), 500
        if body["role"].startswith("camera_"):
            # Stop old streams and discard their frames before re-resolving.
            CameraManager.get_instance().refresh()
        return jsonify(result), 200, {"Cache-Control": "no-store"}

    @app.post("/api/devices/camera/confirm")
    def api_device_camera_confirm():
        import json
        if request.headers.get("X-Robot-Control") != "devices":
            return jsonify(ok=False, error="Требуется запрос со страницы устройств"), 400
        if request.content_length is not None and request.content_length > 4096:
            return jsonify(ok=False, error="Слишком большой запрос"), 413
        raw = request.stream.read(4097)
        if len(raw) > 4096:
            return jsonify(ok=False, error="Слишком большой запрос"), 413
        try:
            body = json.loads(raw)
            if not isinstance(body, dict) or set(body) != {"role"} or not isinstance(body["role"], str):
                raise ValueError("Укажите роль камеры")
            result = device_registry.confirm_camera(body["role"], CameraManager.get_instance().list_cameras())
        except (ValueError, TypeError, UnicodeError) as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except OSError as exc:
            return jsonify(ok=False, error="Не удалось сохранить подтверждение: " + str(exc)), 500
        return jsonify(result), 200, {"Cache-Control": "no-store"}

    # --------------------- Ручное назначение портов --------------------- #
    # Автопоиск отключён: оператор сам указывает порты всех 12 устройств на
    # странице «Устройства». Сохранённое назначение — это его решение, а не
    # доказательство типа устройства: колесо всё равно требует свежей
    # телеметрии с CRC, камеру — свежего кадра, принтер — физического
    # подтверждения в диалоге.
    from . import device_ports

    def _wheel_status():
        try:
            source = getattr(state, "source", None) if state is not None else None
            info = source.read().get("masterPort") if source is not None else None
            return info if isinstance(info, dict) else {}
        except Exception:  # noqa: BLE001 — экран настроек без телеметрии
            return {}

    def _ports_payload():
        return device_ports.payload(registry=device_registry, wheel_status=_wheel_status())

    def _camera_assign(role, port):
        from .camera_settings import (apply_camera_settings, current_camera_settings,
                                      normalize_camera_settings)
        role_key = device_ports.CAMERA_SETTING[role]
        settings = current_camera_settings()
        settings[role_key] = port
        normalized = normalize_camera_settings(settings)
        # Подсказка реестра фиксирует USB-идентичность открытого устройства.
        # «Сброс» (пустой порт) оставляет отключённую подсказку: камера
        # выключена до нового назначения, а не открывается по старому пути.
        try:
            if not port:
                device_registry.set_hint(role, "")
            else:
                # Подсказку не выбрасываем, даже если устройства сейчас нет в
                # топологии: иначе подтвердить камеру по изображению нельзя
                # никогда (раньше это был тупик «Сначала укажите порт камеры»).
                # Порт остаётся назначением оператора, камера открывается по
                # .env, а подтверждение требует свежего кадра с этого порта.
                device_registry.set_hint(role, port)
        except ValueError as exc:
            raise device_ports.PortError(str(exc)) from exc
        result = apply_camera_settings(normalized)
        return "Камера %s: %s" % (device_ports.ROLE_TITLES[role].split("·")[-1].strip(),
                                  port or "порт не назначен — камера выключена"), result

    def _lidar_assign(port):
        from .lidar_settings import apply_lidar_settings
        settings = LidarManager.get_instance().current_settings()
        settings["port"] = port
        settings["autoprobe"] = False
        result = apply_lidar_settings(settings)
        return "Лидар: %s" % (port or "порт не назначен"), result

    def _kinect_assign(port):
        device_ports.save_role("kinect", port)
        return ("Kinect: %s. Публикатор глубины подхватит порт при следующем запуске."
                % (port or "порт не назначен")), {"port": port}

    def _bms_assign(port):
        result = bms_setup.apply_port(battery, port, operator_confirmed=True)
        return "BMS: %s" % (port or "порт не назначен"), result

    def _printer_assign(port):
        from .printer import DEFAULTS, PrinterError
        previous = device_ports.role_port("printer")
        device_ports.save_role("printer", port)
        body = {key: printer.settings().get(key) for key in DEFAULTS if key != "identity"}
        body["port"] = port
        body["confirmed"] = True
        try:
            result = printer.configure(body)
        except PrinterError:
            device_ports.save_role("printer", previous)
            raise
        except Exception:
            device_ports.save_role("printer", previous)
            raise
        return "Принтер: %s" % (port or "порт не назначен"), result

    def _ports_body(limit=8192):
        import json
        if request.headers.get("X-Robot-Control") != "devices":
            raise device_ports.PortError("Требуется запрос со страницы устройств")
        if request.content_length is not None and request.content_length > limit:
            raise device_ports.PortError("Слишком большой запрос")
        raw = request.stream.read(limit + 1)
        if len(raw) > limit:
            raise device_ports.PortError("Слишком большой запрос")
        try:
            body = json.loads(raw)
        except (ValueError, UnicodeError):
            raise device_ports.PortError("Ожидается JSON-объект") from None
        if not isinstance(body, dict):
            raise device_ports.PortError("Ожидается JSON-объект")
        return body

    @app.get("/api/devices/ports")
    def api_device_ports():
        payload = _ports_payload()
        payload["canEdit"] = True
        return jsonify(payload), 200, {"Cache-Control": "no-store"}

    @app.post("/api/devices/ports")
    def api_device_ports_save():
        from .printer import PrinterError
        try:
            body = _ports_body()
            role = body.get("role")
            if not isinstance(role, str) or (role != "wheels" and role not in device_ports.ROLE_ENV):
                raise device_ports.PortError("Неизвестное устройство")
            with usb_assignment_lock:
                if role == "wheels":
                    ports = device_ports.save_wheels(body.get("ports") or [])
                    message = "Порты колёс: %s. Роли FL/FR/RL/RR определит телеметрия плат." % (
                        ", ".join(p for p in ports if p) or "не назначены")
                    result = {"wheels": ports}
                else:
                    if not isinstance(body.get("port"), str):
                        raise device_ports.PortError("Нужен путь порта")
                    port = device_ports.validate(role, body["port"])
                    if role.startswith("camera_"):
                        message, result = _camera_assign(role, port)
                    elif role == "lidar":
                        message, result = _lidar_assign(port)
                    elif role == "kinect":
                        message, result = _kinect_assign(port)
                    elif role == "bms":
                        message, result = _bms_assign(port)
                    else:
                        message, result = _printer_assign(port)
            eventlog.record("ok", message, source="устройства")
            payload = _ports_payload()
            return jsonify(ok=True, message=message, result=result, ports=payload), 200, {
                "Cache-Control": "no-store"}
        except device_ports.PortError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except PrinterError as exc:
            return jsonify(ok=False, error=str(exc)), getattr(exc, "status", 409)
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except OSError as exc:
            return jsonify(ok=False, error="Не удалось записать .env: " + str(exc)), 500

    def lidar_start_body():
        import json
        if request.headers.get("X-Robot-Control") != "devices":
            raise LidarStartError("Требуется запрос со страницы устройств.", 400)
        if request.content_length is not None and request.content_length > 4096:
            raise LidarStartError("Слишком большой запрос.", 413)
        raw = request.stream.read(4097)
        if len(raw) > 4096:
            raise LidarStartError("Слишком большой запрос.", 413)
        try:
            body = json.loads(raw)
        except (ValueError, UnicodeError):
            raise LidarStartError("Ожидается JSON-объект.", 400) from None
        if not isinstance(body, dict):
            raise LidarStartError("Ожидается JSON-объект.", 400)
        return body

    @app.post("/api/devices/lidar/prepare")
    def api_device_lidar_prepare():
        try:
            with usb_assignment_lock:
                result = lidar_start.prepare(lidar_start_body())
            return jsonify(result), 200, {"Cache-Control": "no-store"}
        except LidarStartError as exc:
            return jsonify(ok=False, error=str(exc)), exc.status

    @app.post("/api/devices/lidar/start")
    def api_device_lidar_start():
        try:
            with usb_assignment_lock:
                result = lidar_start.start(lidar_start_body())
        except LidarStartError as exc:
            eventlog.record("warn", "LDS-01: проверка не завершена — " + str(exc), source="Лидар")
            return jsonify(ok=False, error=str(exc)), exc.status
        eventlog.record("ok", "LDS-01: получены проверенные кадры с " + result["port"], source="Лидар")
        # Stop navigation before restarting a driver. A valid UART stream does
        # not yet mean ROS is publishing a usable scan or navigation is safe.
        navigation.cancel("Лидар пересканируется — навигация остановлена.")
        try:
            rescan = LidarManager.get_instance().restart_search()
            result["rosRescan"] = rescan.get("ros_rescan") if isinstance(rescan, dict) else None
        except Exception as exc:  # noqa: BLE001 — frame proof remains true, driver is still unconfirmed
            result["rescanError"] = str(exc)
        return jsonify(result), 200, {"Cache-Control": "no-store"}

    @app.get("/map")
    def kiosk_map_page():
        # Separate page works even when telemetry/swipe initialization fails.
        return _send("map.html")

    @app.get("/route")
    def kiosk_route_page():
        return _send("route.html")

    @app.get("/api/navigation/status")
    def api_navigation_status():
        return jsonify(navigation.status()), 200, {"Cache-Control": "no-store"}

    @app.post("/api/navigation/plan")
    def api_navigation_plan():
        try:
            return jsonify(navigation.plan(request.get_json(silent=True)))
        except NavigationError as exc:
            return jsonify(exc.payload()), exc.status

    @app.post("/api/navigation/start")
    def api_navigation_start():
        if request.headers.get("X-Robot-Control") != "navigation":
            return jsonify({"ok": False, "reason": "request_header_required",
                            "error": "Запрос без заголовка X-Robot-Control: navigation"}), 400
        try:
            payload, status = navigation.start(request.get_json(silent=True))
        except NavigationError as exc:
            return jsonify(exc.payload()), exc.status
        eventlog.record("ok", "автономная навигация запущена к (%.2f, %.2f) м" % (
            payload["status"].get("goal", {}).get("x", 0),
            payload["status"].get("goal", {}).get("y", 0)), source="навигация")
        return jsonify(payload), status

    @app.post("/api/navigation/resume")
    def api_navigation_resume():
        if request.headers.get("X-Robot-Control") != "navigation":
            return jsonify({"ok": False, "reason": "request_header_required",
                            "error": "Запрос без заголовка X-Robot-Control: navigation"}), 400
        try:
            payload, status = navigation.resume(request.get_json(silent=True))
        except NavigationError as exc:
            return jsonify(exc.payload()), exc.status
        goal = payload["status"].get("goal") or {}
        eventlog.record("ok", "маршрут навигации продолжен к (%.2f, %.2f) м" % (
            goal.get("x", 0), goal.get("y", 0)), source="навигация")
        return jsonify(payload), status

    @app.post("/api/navigation/cancel")
    def api_navigation_cancel():
        if request.headers.get("X-Robot-Control") != "navigation":
            return jsonify({"ok": False, "reason": "request_header_required",
                            "error": "Запрос без заголовка X-Robot-Control: navigation"}), 400
        result = navigation.cancel("Маршрут отменён оператором.")
        eventlog.record("warn", "маршрут навигации отменён оператором", source="навигация")
        return jsonify(result), 200, {"Cache-Control": "no-store"}

    @app.post("/api/navigation/estop")
    def api_navigation_estop():
        if request.headers.get("X-Robot-Control") != "navigation":
            return jsonify({"ok": False, "reason": "request_header_required",
                            "error": "Запрос без заголовка X-Robot-Control: navigation"}), 400
        navigation.cancel("E-STOP: маршрут отменён.")
        result = engage_estop(getattr(state, "source", None))
        result["status"] = navigation.status()
        return jsonify(result), 200, {"Cache-Control": "no-store"}

    # ---------------- Автономное обследование и Kinect ------------------- #
    # Владелец моторов — только этот процесс. Команды требуют маркера
    # X-Robot-Control: exploration, как и остальные команды движения.
    @app.get("/api/exploration/status")
    def api_exploration_status():
        return jsonify(exploration.status()), 200, {"Cache-Control": "no-store"}

    @app.post("/api/exploration/start")
    def api_exploration_start():
        if request.headers.get("X-Robot-Control") != "exploration":
            return jsonify({"ok": False, "error": "Запрос без заголовка X-Robot-Control: exploration"}), 400
        try:
            payload, status = exploration.start(request.get_json(silent=True) or {})
        except ExplorationError as exc:
            return jsonify(exc.payload()), exc.status
        return jsonify(payload), status

    @app.post("/api/exploration/stop")
    def api_exploration_stop():
        if request.headers.get("X-Robot-Control") != "exploration":
            return jsonify({"ok": False, "error": "Запрос без заголовка X-Robot-Control: exploration"}), 400
        return jsonify(exploration.stop()), 200, {"Cache-Control": "no-store"}

    @app.get("/api/exploration/landmarks")
    def api_exploration_landmarks():
        try:
            limit = int(request.args.get("limit", 20))
        except (TypeError, ValueError):
            limit = 20
        return jsonify({"ok": True, "landmarks": exploration.landmarks(limit=limit)}), 200, \
            {"Cache-Control": "no-store"}

    @app.post("/api/exploration/landmarks/clear")
    def api_exploration_landmarks_clear():
        if request.headers.get("X-Robot-Control") != "exploration":
            return jsonify({"ok": False, "error": "Запрос без заголовка X-Robot-Control: exploration"}), 400
        return jsonify({"ok": True, "removed": exploration.clear_landmarks()})

    @app.get("/api/exploration/landmark/<name>")
    def api_exploration_landmark_file(name: str):
        import re
        from flask import send_from_directory

        if not re.fullmatch(r"step\d{3}-cam[0-3]\.jpg", name):
            return jsonify({"ok": False, "error": "Неверное имя файла"}), 400
        return send_from_directory(exploration.landmark_directory, name,
                                   mimetype="image/jpeg", max_age=0)

    def _kinect_for_request(body=None):
        unit = (request.args.get("unit") or (body or {}).get("unit") or "front").strip().lower()
        if unit not in {"front", "rear"}:
            return None, None
        service = kinect_services.get(unit)
        if service is None:
            return None, unit
        return service, unit

    @app.get("/api/kinect/status")
    def api_kinect_status():
        service, unit = _kinect_for_request()
        if service is None:
            return jsonify({"ok": False, "error": "Kinect %s не включён (RC_KINECT_REAR_ENABLE=1)" % unit}), 404
        payload = service.status()
        payload["unit"] = unit
        payload["units"] = {name: item.status()["connected"] for name, item in kinect_services.items()}
        return jsonify(payload), 200, {"Cache-Control": "no-store"}

    @app.post("/api/kinect/depth")
    def api_kinect_depth():
        """Принять кадр глубины от scripts/kinect_depth_publisher.py."""
        if request.headers.get("X-Robot-Control") != "kinect":
            return jsonify({"ok": False, "error": "Запрос без заголовка X-Robot-Control: kinect"}), 400
        body = request.get_json(silent=True)
        if not isinstance(body, dict):
            return jsonify({"ok": False, "error": "Ожидается JSON-объект с кадром глубины"}), 400
        service, unit = _kinect_for_request(body)
        if service is None:
            return jsonify({"ok": False, "error": "Kinect %s не включён" % unit}), 404
        try:
            result = service.feed(body)
        except DepthError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        result["unit"] = unit
        return jsonify(result)

    @app.get("/api/sensors/layout")
    def api_sensors_layout():
        """Четыре C930e + Kinect 360: порты, углы, готовность калибровки."""
        return jsonify(layout_payload()), 200, {"Cache-Control": "no-store"}

    @app.get("/api/calibration/geometry")
    def api_calibration_geometry():
        """Поля для внесения замеров геометрии сенсоров."""
        return jsonify(geometry_calibration.geometry_payload()), 200, {"Cache-Control": "no-store"}

    @app.post("/api/calibration/geometry")
    def api_calibration_geometry_save():
        """Сохранить замеры в .env (только известные ключи, с проверкой диапазонов)."""
        body = request.get_json(silent=True) or {}
        values = body.get("values", body)
        try:
            result = geometry_calibration.save_geometry(values)
        except geometry_calibration.CalibrationError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        except OSError as exc:
            return jsonify({"ok": False, "error": "не удалось записать .env: %s" % exc}), 500
        eventlog.record("ok", "Геометрия сенсоров: сохранено полей %d" % len(result["savedKeys"]),
                        source="Калибровка")
        return jsonify(result)

    @app.get("/api/kiosk_logs")
    def api_kiosk_logs():
        """Настоящий журнал событий робота для панели «Логи» на борту.

        Публичен вместе с остальной телеметрией экрана: это журнал самого
        процесса — подключение лидара, ошибки портов, PIN, команды руления и
        тяги. Выдуманных строк в нём нет; если событий не было, ``entries``
        пустой, и экран так и пишет.
        """
        limit = eventlog.clamp_limit(request.args.get("limit"))
        return jsonify({"ok": True, "data": kiosk_log_payload(limit, state=state)})

    @app.get("/main.html")
    @app.get("/robot.html")
    def kiosk_alt():
        return _send(KIOSK_PAGE)

    @app.get("/api/state")
    @app.get("/api/kiosk_state")
    def api_state():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        manager = LidarManager.get_instance()
        snap = state.snapshot()
        points, demo = lidar_for_screen(manager, snap)
        estop_info = estop_latch.latched()
        # Телеметрия и доступность управления публичны; PIN используется только для кейса.
        return jsonify({"ok": True, "data": {
            "battery": snap.get("battery"),
            "motors": snap.get("motors"),
            # The peer displays these snapshots, but never opens cameras.
            "cameras": CameraManager.get_instance().list_cameras(),
            "lidar": points,
            # Демо (стенд без железа) помечено отдельно: экран пишет «демо»,
            # а не «в реальном времени».
            "lidarDemo": demo,
            # Причина, если скана нет: экран пишет её прямо под картой,
            # а не молчит (порт не найден, скорость не та, порт занят и т. д.).
            "lidarStatus": lidar_status_payload(manager, points, demo),
            "speedMps": snap.get("speedMps"),
            "wheelSpeedMps": snap.get("wheelSpeedMps"),
            "powerKw": snap.get("powerKw"),
            "estop": estop_info,
            "neutral": neutral.status(),
            "mode": snap.get("mode"),
            "linkOk": snap.get("linkOk"),
            "ts": snap.get("ts"),
            "source": snap.get("source"),
            "steeringControl": steering_control_info(state.source),
            "drive": snap.get("drive"),
            "maneuverConfig": snap.get("maneuverConfig"),
            # Какой порт мастера настроен/открыт: экран объясняет «нет связи».
            "masterPort": snap.get("masterPort"),
            # Последние сообщения прошивки мастера («# FR: Калибровка ЗАВЕРШЕНА!»).
            "masterMessages": snap.get("masterMessages") or [],
            # Свет, замок, карты и компас отдельной платы (может не быть).
            "sensorModule": sensor_state_payload(),
        }})

    @app.get("/api/kiosk_wheel_state")
    def api_kiosk_wheel_state():
        """Small, current wheel snapshot for active WASD; no lidar/map payload."""
        if state is None:
            return jsonify(ok=False, error="no source"), 503
        source = getattr(state, "source", None)
        try:
            snap = source.read()
        except Exception:  # noqa: BLE001 — fail closed on a broken source
            return jsonify(ok=False, error="wheel source unavailable"), 503
        if not isinstance(snap, dict):
            return jsonify(ok=False, error="wheel source unavailable"), 503
        return jsonify(ok=True, data={
            "motors": snap.get("motors"), "drive": snap.get("drive"),
            "linkOk": snap.get("linkOk"), "estop": estop_latch.latched(),
            "neutral": neutral.status(), "steeringControl": steering_control_info(source),
        }), 200, {"Cache-Control": "no-store"}

    @app.get("/api/lidar")
    @app.get("/api/scan")
    def api_kiosk_lidar_get():
        """Скан и статус лидара бортового экрана.

        По этому адресу пульт (8080) берёт скан, когда порт лидара держит
        бортовой экран. Демо-скана в системе нет, флаг ``demo`` всегда ``False``.
        """
        mgr = LidarManager.get_instance()
        scan = mgr.get_scan()
        demo = False
        if scan is None and state and hasattr(state, "source"):
            try:
                src_data = getattr(state.source, "read", lambda: {})()
                scan = src_data.get("lidar")  # реальный скан источника; демо-режима нет
            except Exception:  # noqa: BLE001 — источник может не уметь read()
                scan = None
        return jsonify(lidar_status_payload(mgr, scan, demo))

    @app.get("/api/lidar/logs")
    def api_kiosk_lidar_logs():
        """Журнал лидара на бортовом экране: только хвост фиксированных файлов."""
        return jsonify(lidar_boot.read_logs(request.args.get("lines", 120)))

    @app.post("/api/lidar")
    @app.post("/api/scan")
    def api_kiosk_lidar_post():
        body = request.get_json(silent=True) or {}
        mgr = LidarManager.get_instance()
        if "points" in body or "scan" in body:
            pts = body.get("points") or body.get("scan") or []
            count = mgr.feed_points(pts, source_label="api")
            return jsonify({"ok": True, "count": count})
        elif "ranges" in body:
            import math
            count = mgr.feed_scan_msg(
                ranges=body.get("ranges", []),
                angle_min=float(body.get("angle_min", -math.pi)),
                angle_max=float(body.get("angle_max", math.pi)),
                angle_increment=float(body.get("angle_increment", 0.0174)),
                range_min=float(body.get("range_min", 0.05)),
                range_max=float(body.get("range_max", 12.0)),
                source_label="api_ranges",
            )
            return jsonify({"ok": True, "count": count})
        return jsonify({"ok": False, "error": "expected 'points' or 'ranges'"}), 400

    @app.get("/api/cameras")
    def api_kiosk_cameras():
        mgr = CameraManager.get_instance()
        return jsonify({"ok": True, "cameras": mgr.list_cameras()})

    @app.get("/api/camera/<int:cam_id>/stream")
    def api_kiosk_camera_stream(cam_id: int):
        mgr = CameraManager.get_instance()
        return Response(
            mgr.generate_mjpeg_stream(cam_id),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/api/camera/front/stream")
    def api_kiosk_camera_front_stream():
        return api_kiosk_camera_stream(0)

    @app.get("/api/camera/rear/stream")
    def api_kiosk_camera_rear_stream():
        return api_kiosk_camera_stream(1)

    @app.get("/api/camera/<int:cam_id>/frame")
    def api_kiosk_camera_frame(cam_id: int):
        mgr = CameraManager.get_instance()
        frame = mgr.get_frame(cam_id)
        if frame:
            return Response(frame, mimetype="image/jpeg")
        return Response(status=404)

    @app.get("/api/camera/front/frame")
    def api_kiosk_camera_front_frame():
        return api_kiosk_camera_frame(0)

    @app.get("/api/camera/rear/frame")
    def api_kiosk_camera_rear_frame():
        return api_kiosk_camera_frame(1)

    @app.post("/api/camera/<int:cam_id>/frame")
    def api_kiosk_camera_feed(cam_id: int):
        mgr = CameraManager.get_instance()
        data = request.get_data()
        if data:
            mgr.feed_frame(cam_id, data)
            return jsonify({"ok": True})
        return jsonify({"ok": False, "error": "empty data"}), 400

    @app.get("/api/kiosk_lock")
    def api_kiosk_lock():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        # Поле оставлено для совместимости со старым клиентом; в киоске команды
        # руля/тяги не зависят от PIN. Только open_lock проверяет PIN кейса.
        return jsonify({"ok": True, "lock": state.vault.state(state.lock_open),
                        "steeringAuthorized": True})

    @app.post("/api/kiosk_lock/open")
    def api_kiosk_lock_open():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        body = request.get_json(silent=True) or {}
        pin = str(body.get("pin", ""))
        result = state.open_lock(pin)
        note_case_pin_result(result, where="открытие кейса")
        return jsonify(result), 200 if result.get("ok") else 403

    @app.post("/api/kiosk_lock/close")
    def api_kiosk_lock_close():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        return jsonify(state.close_lock())

    @app.post("/api/kiosk_full_zero")
    def api_kiosk_full_zero():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify(ok=False, reason="request_header_required"), 400
        body = request.get_json(silent=True)
        if not isinstance(body, dict):
            return jsonify(ok=False, reason="payload"), 400
        action = body.get("action")
        # Cancellation and status must not wait behind a queued motion request.
        if action in {"cancel", "status"}:
            result, status = full_zero.handle(body)
            return jsonify(result), status, {"Cache-Control": "no-store"}
        with motion_gate:
            if action == "begin":
                keyboard.interrupt()
                # No late navigation heartbeat may overtake the calibration STOP.
                dispatched = navigation.manual_override(lambda: full_zero.handle(body))
                if dispatched is None:
                    return jsonify(ok=False, reason="navigation_stopping"), 409
                result, status = dispatched
            else:
                result, status = full_zero.handle(body)
        return jsonify(result), status, {"Cache-Control": "no-store"}

    @app.post("/api/kiosk_steer")
    def api_kiosk_steer():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        navigation.cancel("Ручное руление — автономный маршрут остановлен.")
        result, status = dispatch_steering_command(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @app.post("/api/kiosk_zero")
    def api_kiosk_zero():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        navigation.cancel("Ручная калибровка — автономный маршрут остановлен.")
        result, status = dispatch_steering_zero(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @app.post("/api/kiosk_pot_mode")
    def api_kiosk_pot_mode():
        if state is None:
            return jsonify({"ok": False, "reason": "serial_unavailable"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        navigation.cancel("Смена режима потенциометра — маршрут остановлен.")
        result, status = dispatch_pot_mode(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @app.post("/api/kiosk_pot_hold")
    def api_kiosk_pot_hold():
        """Активное удержание угла (ph 0/1) со всех четырёх плат.

        Как и смена режима потенциометра, это команда руления: заголовок
        X-Robot-Control обязателен, маршрут отменяется, PIN на киоске не нужен.
        """
        if state is None:
            return jsonify({"ok": False, "reason": "serial_unavailable"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        navigation.cancel("Активное удержание угла — маршрут остановлен.")
        result, status = dispatch_pot_hold(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @app.post("/api/kiosk_maneuver")
    def api_kiosk_maneuver():
        """Манёвр-пресет тяги: «метр вперёд», «развернуться на 90°» (start/stop).

        Правила те же, что у ручной тяги: заголовок команды обязателен, PIN
        проверяется сессией руления. Маршрут отменяется — манёвр и автономная
        езда одновременно несовместимы.
        """
        if state is None:
            return jsonify({"ok": False, "reason": "serial_unavailable"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        body = request.get_json(silent=True) or {}
        if isinstance(body, dict) and body.get("action") == "start":
            navigation.cancel("Манёвр — автономный маршрут остановлен.")
        result, status = dispatch_maneuver(state.source, body)
        return jsonify(result), status

    @app.post("/api/kiosk_maneuver_config")
    def api_kiosk_maneuver_config():
        """Настройка пресетов манёвров: метры, градусы, рабочий процент, шасси."""
        if state is None:
            return jsonify({"ok": False, "reason": "serial_unavailable"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        result, status = dispatch_maneuver_config(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @app.post("/api/kiosk_calibrate")
    def api_kiosk_calibrate():
        """Калибровка руля по оптодатчику D18: один модуль или ``ALL``."""
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        navigation.cancel("Калибровка руля — автономный маршрут остановлен.")
        result, status = dispatch_steering_calibration(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @app.post("/api/estop")
    def api_estop():
        """E-STOP с борта: без PIN (остановить может любой), снятие — только с PIN."""
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        navigation.cancel("E-STOP: маршрут отменён.")
        result = engage_estop(state.source)
        return jsonify(result)

    @app.post("/api/estop/release")
    def api_estop_release():
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        body = request.get_json(silent=True) or {}
        result = state.vault.verify(str(body.get("pin", "")))
        note_pin_result(result, where="снятие E-STOP")
        if not result.get("ok"):
            return jsonify({"ok": False, "reason": result.get("reason", "pin")}), 403
        return jsonify(release_estop())

    @app.post("/api/kiosk_neutral")
    def api_kiosk_neutral():
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify(ok=False, reason="request_header_required"), 400
        body = request.get_json(silent=True)
        if not isinstance(body, dict):
            return jsonify(ok=False, reason="payload"), 400
        action = body.get("action")
        if not isinstance(action, str):
            return jsonify(ok=False, reason="payload"), 400
        if action in {"begin", "heartbeat"}:
            # Old/cached clients must never be able to request freewheel. Still
            # detect an orphaned legacy state and initiate its safe brake.
            neutral.status()
            return jsonify(ok=False, reason="neutral_disabled"), 410
        if action == "restore" and set(body) == {"action"}:
            result, status = neutral.restore()
            return jsonify(result), status
        return jsonify(ok=False, reason="payload"), 400

    @app.post("/api/kiosk_keyboard")
    def api_kiosk_keyboard():
        if state is None:
            return jsonify({"ok": False, "reason": "serial_unavailable"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        body = request.get_json(silent=True)
        action = body.get("action") if isinstance(body, dict) else None
        if not isinstance(action, str):
            return jsonify(ok=False, reason="payload"), 400
        expires = None
        if action not in {"stop", "end"}:
            try:
                expires = float(request.headers.get("X-Robot-Command-Deadline", ""))
                now = time.time()
                if not math.isfinite(expires) or expires < now or expires > now + 0.75:
                    raise ValueError("expired or clock skew")
            except (TypeError, ValueError):
                if keyboard.is_preempted(body):
                    return jsonify(ok=False, reason="stale_command"), 409
                keyboard.interrupt()
                return jsonify(ok=False, reason="command_expired"), 408

        def send_keyboard():
            # Время могло истечь, пока навигация освобождала моторы.
            if expires is not None and time.time() > expires:
                if keyboard.is_preempted(body):
                    return {"ok": False, "reason": "stale_command"}, 409
                keyboard.interrupt()
                return {"ok": False, "reason": "command_expired"}, 408
            deadline = (time.monotonic() + max(0.0, expires - time.time())
                        if expires is not None else None)
            return keyboard.handle(body, deadline=deadline)

        # Сервер навигации освобождает тягу прежде чем разрешить WASD.
        sent = navigation.manual_override(send_keyboard, wait_for_exit=action not in {"stop", "end"})
        if sent is None:
            if keyboard.is_preempted(body):
                return jsonify(ok=False, reason="stale_command"), 409
            keyboard.interrupt()
            return jsonify(ok=False, reason="navigation_stopping"), 409
        result, status = sent
        return jsonify(result), status

    @app.post("/api/kiosk_drive")
    def api_kiosk_drive():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        deadline = request.headers.get("X-Robot-Command-Deadline")
        body = request.get_json(silent=True)
        is_stop = isinstance(body, dict) and (body.get("value") == 0 or body.get("percent") == 0)
        expires = None
        if deadline is not None and isinstance(body, dict) and not is_stop:
            try:
                expires = float(deadline)
                if not math.isfinite(expires) or time.time() > expires:
                    raise ValueError("expired")
            except (TypeError, ValueError):
                return jsonify(ok=False, reason="command_expired"), 408

        def send_manual():
            # Время могло истечь, пока завершался автономный worker.
            if expires is not None and time.time() > expires:
                return {"ok": False, "reason": "command_expired"}, 408
            return dispatch_drive_command(state.source, body)

        sent = navigation.manual_override(send_manual, wait_for_exit=not is_stop)
        if sent is None:
            return jsonify(ok=False, reason="navigation_stopping"), 409
        result, status = sent
        return jsonify(result), status

    # ------------------------- Настройки лидара ------------------------- #
    # Настроить и пересканировать лидар можно без PIN: эти операции не дают
    # команд движения. Набор параметров проходит строгую валидацию, а порт
    # лидара синхронизируется с источником шасси общим межпроцессным замком.
    @app.get("/api/lidar/settings")
    def api_kiosk_lidar_settings():
        result = settings_payload(LidarManager.get_instance())
        result["canEdit"] = True
        result["requiresPin"] = False  # совместимость со старыми клиентами
        return jsonify(result)

    @app.post("/api/lidar/settings")
    def api_kiosk_lidar_settings_save():
        body = request.get_json(silent=True) or {}
        navigation.cancel("Настройки лидара меняются — навигация остановлена.")
        try:
            result = apply_lidar_settings(body.get("settings") or body)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        return jsonify(result)

    @app.post("/api/lidar/rescan")
    def api_kiosk_lidar_rescan():
        navigation.cancel("Лидар перезапускается — навигация остановлена.")
        manager = LidarManager.get_instance()
        result = manager.restart_search()
        return jsonify({"ok": True, "settings": manager.current_settings(),
                        # При источнике ros «пересканировать» значит перезапустить
                        # драйвер через супервизор: его ответ виден в панели.
                        "ros_rescan": result.get("ros_rescan") if isinstance(result, dict) else None,
                        "status": manager.get_status()})

    # --------------------------- Модуль датчиков ------------------------- #
    # Свет, замок двери, карты доступа и компас живут на отдельной плате
    # (firmware/sensor_module). Сервер только показывает состояние и передаёт
    # команды; безопасное состояние держит прошивка. Порт может отсутствовать
    # или отвалиться — тогда экран показывает «не подключён», а не ошибку.
    def current_sensor_module():
        return app.extensions.get("rc_sensor_module")

    def sensor_state_payload():
        module = current_sensor_module()
        if module is None:
            return {"configured": False, "ok": False, "stripEnabled": False,
                    "error": "не настроен (RC_SENSOR_PORT)"}
        try:
            return module.state()
        except Exception as exc:  # noqa: BLE001 — экран не должен падать из-за платы
            return {"configured": True, "ok": False, "error": str(exc) or type(exc).__name__}

    def sensor_command(call, *, needs_pin=False):
        """Общий путь: проверка PIN (если нужен), команда, журнал."""
        module = current_sensor_module()
        if module is None:
            return jsonify(ok=False, error="модуль датчиков не подключён"), 503
        body = request.get_json(silent=True) or {}
        if needs_pin:
            vault = app.extensions.get("rc_vault")
            pin = str(body.get("pin", ""))
            verdict = vault.verify(pin) if vault is not None else {"ok": False, "reason": "format"}
            note_case_pin_result(verdict, where="замок двери")
            if not verdict.get("ok"):
                return jsonify(ok=False, error="PIN не принят", lock=verdict.get("lock")), 403
        try:
            call(module, body)
        except (ValueError, RuntimeError) as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except Exception as exc:  # noqa: BLE001 — обрыв USB не повод ронять экран
            return jsonify(ok=False, error=str(exc) or type(exc).__name__), 503
        return jsonify(ok=True, data=sensor_state_payload()), 200

    @app.get("/api/sensor_module")
    def api_sensor_module():
        """Снимок модуля датчиков: курс, свет, замок, карты, ошибки платы."""
        return jsonify(ok=True, data=sensor_state_payload()), 200, {"Cache-Control": "no-store"}

    @app.post("/api/sensor_module/lights")
    def api_sensor_module_lights():
        """Свет: {"light": "head"|"stop"|"left"|"right"|"dim", "on": bool}.

        Дополнительно принимается {"all": true, "on": false} и {"dim": 0…255}.
        """
        def act(module, body):
            if "dim" in body:
                module.set_dim(int(body["dim"]))
            if body.get("all"):
                module.lights_all(bool(body.get("on", True)))
            if body.get("light") is not None:
                module.light(str(body["light"]), bool(body.get("on", True)))
        return sensor_command(act)

    @app.post("/api/sensor_module/lock")
    def api_sensor_module_lock():
        """Замок отсека: {"open": true, "seconds": 3} | {"hold": true} | {"open": false}.

        Открытие только по PIN кейса, закрытие — свободно.
        """
        body = request.get_json(silent=True) or {}
        hold = bool(body.get("hold"))

        def act(module, payload):
            if hold:
                # Открыть и держать: так же, как это делает карта администратора.
                module.lock(True, hold=True)
            elif payload.get("open"):
                seconds = payload.get("seconds")
                module.lock(True, None if seconds is None else float(seconds))
            else:
                module.lock(False)
            eventlog.record("ok", "замок отсека: %s" % (
                "открыт и держится" if hold else "открыт" if payload.get("open") else "закрыт"),
                source="sensor")
        return sensor_command(act, needs_pin=bool(body.get("open") or hold))

    @app.post("/api/sensor_module/cards")
    def api_sensor_module_cards():
        """Карты доступа: list, learn, add, del, role, clear (кроме list — с PIN).

        Роль карты (`role`): admin открывает отсек всегда, user — только в окно
        погрузки или выгрузки. По умолчанию user: меньше прав по умолчанию.
        """
        body = request.get_json(silent=True) or {}
        action = str(body.get("action", "list")).strip().lower()
        role = body.get("role")

        def act(module, payload):
            module.card(action, payload.get("uid"), role)
            eventlog.record("ok", "карты доступа: %s (%s)" % (action, role or "user"),
                            source="sensor")
        return sensor_command(act, needs_pin=action not in ("list",))

    @app.post("/api/sensor_module/cargo")
    def api_sensor_module_cargo():
        """Окно погрузки/выгрузки: {"kind": "load"|"unload"|"end", "minutes": 15}.

        Пока окно открыто, карты пользователей тоже открывают отсек. Открытие
        окна требует PIN кейса: это понижение защиты, а не чтение состояния.
        """
        body = request.get_json(silent=True) or {}
        kind = str(body.get("kind", "")).strip().lower()
        minutes = body.get("minutes")

        def act(module, payload):
            module.cargo(kind, None if minutes is None else int(minutes))
            eventlog.record("ok", "окно погрузки: %s" % (kind or "end"), source="sensor")
        return sensor_command(act, needs_pin=kind not in ("end", "stop", "none", ""))

    @app.post("/api/sensor_module/mag")
    def api_sensor_module_mag():
        """Калибровка компаса: start (оборот), stop (сохранить), reset."""
        def act(module, body):
            module.mag_calibration(str(body.get("action", "")))
        return sensor_command(act)

    # --------------------------- Карта лидара ---------------------------- #
    # Страница «Карта лидара» на бортовом экране: живая карта (как в ROS),
    # сброс, настройки и файл карты на рабочем столе. Всё это — чтение и
    # запись файла, команд движения карта не даёт, поэтому PIN не нужен.
    @app.get("/api/kiosk_map")
    def api_kiosk_map():
        """Снимок карты: настройки, статистика, поза и сама картинка (PNG).

        Рядом с позой отдаётся курс с магнитометра модуля датчиков: карта
        строит лидар, но «куда смотрит робот» знает GY-87. Платы может не быть —
        тогда ``compass`` = ``None``, и экран показывает только лидар.
        """
        payload = map_service.snapshot()
        module = app.extensions.get("rc_sensor_module")
        payload["compass"] = None
        if module is not None:
            try:
                payload["compass"] = module.compass()
            except Exception as exc:  # noqa: BLE001 — карта важнее компаса
                payload["compass"] = {"ok": False, "error": str(exc) or type(exc).__name__}
        return jsonify(payload), 200, {"Cache-Control": "no-store"}

    @app.get("/api/kiosk_map/settings")
    def api_kiosk_map_settings():
        return jsonify({"ok": True, "settings": map_service.settings.to_dict(),
                        "paths": map_paths()})

    @app.post("/api/kiosk_map/settings")
    def api_kiosk_map_settings_save():
        body = request.get_json(silent=True) or {}
        navigation.cancel("Настройки карты меняются — навигация остановлена.")
        navigation.clear_plan()
        result = map_service.set_settings(body.get("settings") or body)
        settings = result["settings"]
        eventlog.record("ok", "карта: сетка %.2f м, дальность %.1f м%s" % (
            settings["resolution"], settings["max_range"],
            " — карта очищена, сетка новая" if result.get("mapReset") else ""),
            source="карта")
        return jsonify({"ok": True, "settings": settings,
                        "mapReset": bool(result.get("mapReset")),
                        "paths": map_paths()})

    @app.post("/api/kiosk_map/reset")
    def api_kiosk_map_reset():
        """Сброс карты: сетка очищается, файл карты перезаписывается пустым."""
        navigation.cancel("Карта сбрасывается — навигация остановлена.")
        navigation.clear_plan()
        result = map_service.reset()
        paths = result.get("paths") or map_paths()
        eventlog.record("ok", "карта лидара сброшена, файл обновлён: %s" % paths.get("pgm"),
                        source="карта")
        return jsonify({"ok": True, "paths": paths,
                        "snapshot": map_service.snapshot(with_png=False)})

    @app.post("/api/kiosk_map/save")
    def api_kiosk_map_save():
        """Сохранить карту на рабочий стол (PGM + YAML как у ROS и PNG)."""
        result = map_service.save()
        if not result.get("ok"):
            eventlog.record("err", "карта: не удалось сохранить файл — %s" % result.get("error"),
                            source="карта")
            return jsonify(result), 500
        eventlog.record("ok", "карта сохранена: %s" % result["paths"]["pgm"], source="карта")
        return jsonify(result)

    @app.get("/api/wheels/settings")
    def api_wheel_settings():
        from .wheel_settings import discovery_payload
        return jsonify(discovery_payload(getattr(state, "source", None)))

    @app.post("/api/wheels/settings")
    def api_wheel_settings_save():
        return jsonify({"ok": False, "error": "Порты колёс определяются автоматически; ручное назначение отключено"}), 410

    # ------------------------ Настройки камер ------------------------- #
    # Отдельные порты передней и задней камер. На борту (8090) можно менять
    # без PIN и без перезапуска; пульт (8080) берёт скан по HTTP и только читает.
    @app.get("/api/camera/settings")
    def api_kiosk_camera_settings():
        from .camera_settings import settings_payload as cam_settings_payload

        result = cam_settings_payload()
        result["canEdit"] = True
        result["requiresPin"] = False
        return jsonify(result)

    @app.post("/api/camera/settings")
    def api_kiosk_camera_settings_save():
        from .camera_settings import apply_camera_settings

        hints, _ = device_registry.records()
        if any(key.startswith("camera_") for key in hints):
            return jsonify(ok=False, error="Камеры настроены на странице «Устройства». Измените или очистите подсказки там."), 409
        body = request.get_json(silent=True) or {}
        try:
            result = apply_camera_settings(body.get("settings") or body)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        except OSError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500
        ports = result["settings"]
        eventlog.record("ok", "камеры (4): FL %s, FR %s, RL %s, RR %s — сохранено в .env" % (
            ports.get("front") or "авто /dev/video0",
            ports.get("front_right") or "авто /dev/video2",
            ports.get("rear") or "авто /dev/video1",
            ports.get("rear_right") or "авто /dev/video3"), source="камеры")
        return jsonify(result)

    @app.post("/api/camera/rescan")
    def api_kiosk_camera_rescan():
        from .camera_settings import describe_camera_ports
        from .camera import CameraManager

        mgr = CameraManager.get_instance()
        # Обновляем список камер (проверяем наличие устройств)
        ports = describe_camera_ports()
        return jsonify({"ok": True, "ports": ports, "cameras": mgr.list_cameras()})

    # ---------------------- Остановка робота кнопкой --------------------- #
    # Кнопка на бортовом экране гасит всё то же, что ./scripts/stop.sh: пульт
    # 8080, этот экран 8090, супервизор лидара и окно киоска. Входа оператора
    # на киоске нет, поэтому запрос помечается заголовком «от кнопки» (как
    # команды руления и тяги) — сторонний POST такой заголовок поставить не
    # может, а случайное нажатие всё равно требует подтверждения в окне.
    @app.get("/api/system/stop")
    def api_kiosk_stop_status():
        return jsonify({
            "ok": True,
            "canStop": True,
            "requiresCsrf": False,
            "targets": list(shutdown.TARGETS),
            "script": str(PROJECT_ROOT / shutdown.STOP_SCRIPT),
            "restart": "вернуть всё: ярлык «Запустить робот» или ./scripts/start.sh",
            "autostart": "автозапуск не выключается: ./scripts/autostart.sh off — если не нужен",
        })

    @app.post("/api/system/stop")
    def api_kiosk_stop():
        if request.headers.get(STEERING_COMMAND_HEADER) != STOP_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "error": "Запрос не от кнопки остановки."}), 400
        navigation.cancel("Система останавливается — маршрут отменён.")
        result = shutdown.stop_all(PROJECT_ROOT)
        if result.get("ok"):
            eventlog.record(
                "warn",
                "остановка по кнопке на бортовом экране: пульт, экран и лидар",
                source="остановка",
            )
            return jsonify(result), 202
        return jsonify(result), 500

    @app.get("/healthz")
    def healthz():
        return jsonify({
            "ok": True,
            "mode": "kiosk",
            "source": getattr(getattr(state, "source", None), "name", None) if state else None,
        })

    # статика (robot.css, robot.js, карта лидара и панель настроек лидара)
    for filename in ("steer_dial.js", "steer_dial.css", "calibration.css", "wheel_settings.js", "pot_mode.js",
                     "wheel_angle.js", "pot_hold.js", "devices.css", "devices.js", "sensors.css", "sensors.js", "robot.css", "robot.js", "pc_chip.js", "full_zero.js", "keyboard.js", "brake_recovery.js", "battery.js", "receipt.js", "receipt.css", "lidar_settings.css",
                     "lidar_settings.js", "lidar_view.js", "robot_model.js", "wheel_ports.js", "map_view.js",
                     "explore.js", "map.css", "viewport.js", "camera_view.js", "camera_stitch.js", "camera_frames.js", "slam_auth.js",
                     "steer_presets.js", "power.css", "power.js"):
        # нужен closure, чтобы цикл не переиспользовал последнее значение
        def _view(name=filename):
            return _send(name)
        endpoint = f"asset_{filename.replace('.', '_')}"
        app.add_url_rule(f"/{filename}", endpoint=endpoint, view_func=_view)

    # заголовки безопасности (разрешены iframe для превью)
    @app.after_request
    def _headers(response):
        response.headers.setdefault("X-Content-Type-Options", "nosniff")
        response.headers.setdefault("Referrer-Policy", "no-referrer")
        response.headers.setdefault("X-Robots-Tag", "noindex, nofollow")
        response.headers.setdefault("Cache-Control", "no-store")
        return response

    return app


# ------------------------------------------------------------ открытие экрана -- #
def kiosk_url(port: int | None = None, host: str = "127.0.0.1") -> str:
    """Адрес бортового экрана (для браузера на самом роботе — 127.0.0.1)."""
    return f"http://{host}:{port or kiosk_port()}{KIOSK_URL_PATH}"


def port_in_use(port: int, host: str = "127.0.0.1", timeout: float = 0.5) -> bool:
    """Слушает ли кто-то порт уже сейчас (тогда сервер поднимать не надо)."""
    with socket.socket() as sock:
        sock.settimeout(timeout)
        try:
            sock.connect((host, port))
        except OSError:
            return False
    return True


def wait_for_server(base_url: str, timeout: float = HEALTH_TIMEOUT_SEC) -> bool:
    """Ждёт, пока сервер ответит на /healthz (браузеру нельзя открываться раньше)."""
    deadline = time.monotonic() + max(0.0, timeout)
    while True:
        try:
            with urllib.request.urlopen(base_url + "/healthz", timeout=1.0) as response:
                if response.status == 200:
                    return True
        except (urllib.error.URLError, OSError):
            pass
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.25)


# ------------------------------------------------------ единственный экран -- #
# Экран должен быть один, сколько бы запусков ни случилось одновременно:
# автозапуск рабочего стола после входа, клик по ярлыку, ручной kiosk.sh.
# Поэтому показ экрана держит межпроцессный замок (flock) на весь срок жизни
# окна; второй запуск замок не получает и браузер не открывает вовсе.
_display_ports_lock = threading.Lock()
_display_ports: set[int] = set()  # порты, чей экран уже показывает этот процесс


def _display_lock_file(port: int) -> Path:
    return PROJECT_ROOT / "var" / f"kiosk-display-{port}.lock"


class _DisplayClaim:
    """Занятый показ экрана: держит flock, пока окно киоска открыто."""

    def __init__(self, port: int, handle) -> None:
        self.port = port
        self._handle = handle  # открытый файл замка (None — замка нет, не POSIX)

    def release(self) -> None:
        handle, self._handle = self._handle, None
        if handle is not None:
            try:
                if fcntl is not None:
                    fcntl.flock(handle, fcntl.LOCK_UN)
            except OSError:
                pass
            try:
                handle.close()
            except OSError:
                pass
        with _display_ports_lock:
            _display_ports.discard(self.port)


def _claim_display(port: int) -> _DisplayClaim | None:
    """Занять показ экрана на ``port``; None — экран уже показывает кто-то ещё.

    Замок один на порт, поэтому повторный вызов из этого же процесса тоже
    получит отказ — второй сторожевой поток нам не нужен.
    """
    with _display_ports_lock:
        if port in _display_ports:
            return None
        _display_ports.add(port)
    handle = None
    if fcntl is not None:
        try:
            path = _display_lock_file(port)
            path.parent.mkdir(parents=True, exist_ok=True)
            handle = path.open("a+")
        except OSError as exc:  # без файла замка живём на внутрипроцессной защите
            log.debug("не удалось открыть %s: %s", _display_lock_file(port), exc)
            handle = None
    if handle is not None:
        deadline = time.monotonic() + DISPLAY_CLAIM_TIMEOUT_SEC
        while True:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    handle.close()
                    with _display_ports_lock:
                        _display_ports.discard(port)
                    return None
                time.sleep(0.1)
    return _DisplayClaim(port, handle)


def display_claimed(port: int | None = None) -> bool:
    """Показывает ли кто-нибудь экран этого порта прямо сейчас (для тестов)."""
    port = port or kiosk_port()
    with _display_ports_lock:
        if port in _display_ports:
            return True
    if fcntl is None:
        return False
    try:
        path = _display_lock_file(port)
        if not path.exists():
            return False
        with path.open("a+") as handle:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                return True
            fcntl.flock(handle, fcntl.LOCK_UN)
    except OSError:
        return False
    return False


def _probe_hosts(host: str | None) -> tuple[str, ...]:
    """Куда стучаться сторожу экрана, чтобы узнать, жив ли сервер киоска.

    Сервер может слушать как все адреса (``0.0.0.0``), так и один конкретный
    (``RC_BIND_HOST=192.168.4.1``): во втором случае проверка только
    ``127.0.0.1`` решила бы, что сервер остановлен, и погасила бы экран.
    """
    hosts = ["127.0.0.1"]
    if host and host not in {"0.0.0.0", "::", "localhost", "127.0.0.1"}:
        hosts.append(host)
    return tuple(hosts)


def _server_listening(port: int, hosts: Sequence[str] = ("127.0.0.1",)) -> bool:
    """Слушает ли сервер экрана порт прямо сейчас (дешёвая проверка сторожа)."""
    return any(port_in_use(port, host) for host in hosts)


def _wait_for_kiosk_port(port: int, hosts: Sequence[str], timeout: float,
                         stop_event: threading.Event) -> bool:
    """Ждёт, пока порт экрана снова начнёт слушаться; False — остановка показа.

    Проверка нужна перед каждым открытием окна: экран показывает данные сервера
    киоска, и без сервера показывать нечего. Пауза (``SERVER_GONE_SEC``) — на
    перезапуск обновления: сокет в это время свободен, но робот не остановлен.
    """
    deadline = time.monotonic() + max(0.0, timeout)
    while not stop_event.is_set():
        if _server_listening(port, hosts):
            return True
        if time.monotonic() >= deadline:
            return False
        stop_event.wait(0.25)
    return False


class _ServerWatch:
    """Следит, жив ли сервер киоска, пока окно экрана открыто.

    Одна проверка порта врёт на перезапуске (обновление поднимает
    ``run_kiosk.py`` заново за секунду-две), поэтому «сервер остановлен» — это
    ``SERVER_GONE_SEC`` без слушателя подряд. Как только остановка подтверждена,
    показ прекращается: окно закрывается и больше не открывается. Иначе сторож
    продолжал бы возвращать экран на монитор уже после остановки всего робота.
    """

    def __init__(self, port: int, hosts: Sequence[str] = ("127.0.0.1",)) -> None:
        self.port = port
        self.hosts = tuple(hosts)
        self._due = time.monotonic() + SERVER_CHECK_SEC
        self._gone_since: float | None = None

    def stopped(self) -> bool:
        """True, если сервер экрана не слушает порт уже ``SERVER_GONE_SEC``."""
        now = time.monotonic()
        if now < self._due:
            return False
        self._due = now + SERVER_CHECK_SEC
        if _server_listening(self.port, self.hosts):
            if self._gone_since is not None:
                log.info("сервер экрана (порт %s) снова отвечает — показ продолжается",
                         self.port)
            self._gone_since = None
            return False
        if self._gone_since is None:
            self._gone_since = now
            log.info("сервер экрана (порт %s) не отвечает — жду %.0f с: остановка "
                     "или перезапуск", self.port, SERVER_GONE_SEC)
            return False
        return now - self._gone_since >= SERVER_GONE_SEC


def _reopen_delay(quick_exits: int) -> float:
    """Пауза перед переоткрытием: растёт, если браузер выходит почти сразу."""
    if quick_exits <= 1:
        return BROWSER_REOPEN_SEC
    return min(BROWSER_REOPEN_SEC * 2 ** (quick_exits - 1), BROWSER_REOPEN_MAX_SEC)


def _close_window(proc: "subprocess.Popen[bytes]",
                  timeout: float = BROWSER_CLOSE_TIMEOUT_SEC) -> None:
    """Закрывает окно киоск-браузера и дожидается его.

    Дождаться нужно по двум причинам: без ``wait`` закрытый браузер остаётся
    зомби в таблице процессов, а упорное окно (snap-обёртка не всегда умирает
    от SIGTERM) продолжало бы светить на мониторе уже после остановки робота.
    """
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    log.warning("окно киоска не закрылось за %.0f с — завершаю принудительно", timeout)
    proc.kill()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:  # pragma: no cover - ядро держит процесс
        log.error("окно киоска не закрывается даже по SIGKILL (pid %s)", proc.pid)


def _display_worker(port: int, url: str, base_url: str, watch: bool,
                    stop_event: threading.Event, claim: _DisplayClaim,
                    hosts: Sequence[str] = ("127.0.0.1",)) -> None:
    """Открывает экран в браузере и, если ``watch``, держит его открытым.

    Показ живёт только пока жив сервер киоска: сервер остановили (служба
    ``robot-kiosk``, ``./scripts/stop.sh``, кнопка «Остановить всё» или Ctrl+C
    в ярлыке «Запустить робот») — окно закрывается и заново не открывается.
    Без этой проверки сторож переоткрывал экран вечно, и «остановленный» робот
    продолжал светить монитором.
    """
    try:
        if not wait_for_server(base_url):
            log.warning("сервер не ответил за %.0f с — открываю %s как есть",
                        HEALTH_TIMEOUT_SEC, url)
        quick_exits = 0
        while not stop_event.is_set():
            # Нет сервера — нет и данных для экрана: окно не открываем вовсе.
            if not _wait_for_kiosk_port(port, hosts, SERVER_GONE_SEC, stop_event):
                log.info("сервер экрана (порт %s) остановлен — показ прекращён, "
                         "экран больше не открывается", port)
                return
            browser = find_browser()
            if browser is None:
                log.warning(
                    "браузер не найден — откройте %s вручную "
                    "(поставить: sudo apt install chromium-browser)", url)
                if not watch or stop_event.wait(BROWSER_RETRY_SEC):
                    return
                continue
            try:
                # Каталог создаём до запуска: иначе Firefox рисует
                # «Your Firefox profile cannot be loaded…» и экран пустой.
                profile = prepare_browser_profile(browser)
                proc = subprocess.Popen(
                    browser_command(browser, url, profile_dir=profile),
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
            except OSError as exc:
                log.error("не удалось открыть браузер %s: %s", browser, exc)
                return
            log.info(
                "экран робота открыт в браузере (%s), профиль %s: %s",
                os.path.basename(browser), profile, url,
            )
            server_watch = _ServerWatch(port, hosts)
            if not watch:
                # Не переоткрываем, но замок показа держим, пока окно живо:
                # иначе следующий запуск решит, что экран никто не показывает.
                while not stop_event.wait(0.5):
                    if proc.poll() is not None:
                        return
                    if server_watch.stopped():
                        _close_window(proc)
                        log.info("сервер экрана (порт %s) остановлен — окно закрыто",
                                 port)
                        return
                _close_window(proc)
                return
            # Ждём закрытия браузера и открываем заново: экран всегда на месте.
            # Но если процесс вышел почти сразу, это не «окно закрыли», а
            # лаунчер передал адрес другой копии/браузер упал: паузу между
            # попытками увеличиваем, чтобы не плодить окна очередями.
            started = time.monotonic()
            server_stopped = False
            try:
                while not stop_event.wait(0.5):
                    if proc.poll() is not None:
                        break
                    if server_watch.stopped():
                        server_stopped = True
                        break
                else:
                    return
            finally:
                if stop_event.is_set() or server_stopped:
                    _close_window(proc)  # показ остановлен — своё окно закрываем
            if server_stopped:
                log.info("сервер экрана (порт %s) остановлен — окно закрыто, "
                         "показ прекращён", port)
                return
            lived = time.monotonic() - started
            quick_exits = quick_exits + 1 if lived < BROWSER_QUICK_EXIT_SEC else 0
            delay = _reopen_delay(quick_exits)
            log.info("браузер закрыт — открываю экран заново через %.0f с", delay)
            if stop_event.wait(delay):
                return
    finally:
        claim.release()


def open_kiosk_screen(port: int | None = None, *, watch: bool = True,
                      stop_event: threading.Event | None = None,
                      host: str | None = None) -> threading.Thread | None:
    """Открыть бортовой экран в отдельном потоке (сервер не блокируется).

    Возвращает поток показа или None, если экран этого порта уже открыт
    (этим или другим запуском киоска): второй браузер не запускается никогда.
    Поток завершается сам, когда сервер экрана остановлен.
    """
    port = port or kiosk_port()
    claim = _claim_display(port)
    if claim is None:
        log.info("экран робота (порт %s) уже открыт другим запуском — "
                 "второй браузер не открываю", port)
        return None
    base_url = f"http://127.0.0.1:{port}"
    thread = threading.Thread(
        target=_display_worker,
        args=(port, kiosk_url(port), base_url, watch,
              stop_event or threading.Event(), claim, _probe_hosts(host)),
        name="kiosk-display",
        daemon=True,
    )
    try:
        thread.start()
    except Exception:
        claim.release()
        raise
    return thread


# -------------------------------------------------------------- точка входа --- #
def _pid_file() -> Path:
    return PROJECT_ROOT / "var" / "kiosk.pid"


def _write_pid_file() -> None:
    try:
        path = _pid_file()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"{os.getpid()}\n", encoding="utf-8")
    except OSError as exc:  # не критично: файл нужен только для stop.sh
        log.debug("не удалось записать %s: %s", _pid_file(), exc)


def _remove_pid_file() -> None:
    try:
        _pid_file().unlink()
    except OSError:
        pass


def _display_pid_file(port: int) -> Path:
    """Pid-файл сторожа экрана (рядом с замком показа ``kiosk-display-*.lock``).

    Нужен, когда сервер киоска поднимает не этот процесс (служба
    ``robot-kiosk`` или прошлый запуск): тогда ``var/kiosk.pid`` принадлежит
    серверу, а процесс, который держит окно на мониторе, ``./scripts/stop.sh``
    раньше не видел вовсе — и сторож переоткрывал экран уже после остановки.
    """
    return PROJECT_ROOT / "var" / f"kiosk-display-{port}.pid"


def _write_display_pid_file(port: int) -> None:
    try:
        path = _display_pid_file(port)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"{os.getpid()}\n", encoding="utf-8")
    except OSError as exc:  # не критично: файл нужен только для stop.sh
        log.debug("не удалось записать %s: %s", path, exc)


def _remove_display_pid_file(port: int) -> None:
    try:
        _display_pid_file(port).unlink()
    except OSError:
        pass


def _install_stop_signals(stop_event: threading.Event, server) -> None:
    """SIGTERM/SIGINT: сначала закрыть окно и сервер, и только потом выйти.

    Без обработчика ``kill`` из ``./scripts/stop.sh`` (или Ctrl+C в ярлыке
    «Запустить робот») обрывал процесс мгновенно: окно киоска оставалось на
    мониторе сиротой, а сторож экрана не успевал его закрыть.

    Обработчик намеренно ничего не логирует: сигнал приходит в главный поток
    между байткодами, и журналирование из него могло бы deadlock-нуть процесс,
    если сигнал застал его внутри другого вызова logging.
    """

    def handler(_signum, _frame):
        stop_event.set()
        if server is not None:
            # shutdown() блокируется до выхода serve_forever(), а тот работает в
            # этом же потоке: из обработчика сигнала зовём его отдельным потоком.
            threading.Thread(target=server.shutdown, daemon=True,
                             name="kiosk-shutdown").start()

    for name in ("SIGINT", "SIGTERM"):
        signum = getattr(signal, name, None)
        if signum is None:  # pragma: no cover - не-POSIX платформы
            continue
        try:
            signal.signal(signum, handler)
        except (ValueError, OSError, RuntimeError):  # не главный поток
            pass


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="run_kiosk.py",
        description="Бортовой экран (8090): функции без PIN, кейс по PIN",
    )
    parser.add_argument("--host", default=None,
                        help="адрес прослушивания (по умолчанию RC_BIND_HOST или 0.0.0.0)")
    parser.add_argument("--port", type=int, default=None,
                        help=f"порт экрана (по умолчанию RC_KIOSK_PORT или {DEFAULT_PORT})")
    parser.add_argument("--source", choices=["auto", "none", "serial", "ros", "master"], default=None,
                        help="источник данных: master (по умолчанию), auto, sim, serial или ros")
    parser.add_argument("--open", dest="open_screen", action="store_true", default=None,
                        help="открыть экран в браузере (по умолчанию — если есть дисплей)")
    parser.add_argument("--no-open", dest="open_screen", action="store_false",
                        help="только сервер, браузер не открывать")
    parser.add_argument("--watch", dest="watch", action="store_true", default=None,
                        help="снова открывать экран, если браузер закрыли (по умолчанию — да)")
    parser.add_argument("--no-watch", dest="watch", action="store_false",
                        help="не переоткрывать браузер")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    # Настройки (заодно подхватываем .env — например RC_KIOSK_OPEN).
    # По умолчанию используются четыре колеса по своему USB; стенд включается
    # явно: RC_SLAM_SOURCE=none ./run_kiosk.py
    cfg = Config.from_env()
    if args.source:
        cfg = replace(cfg, slam_source=args.source)

    host = args.host or cfg.bind_host
    port = args.port or kiosk_port()
    # Свой HTTP-порт запоминаем до старта менеджера лидара: у себя же скан
    # брать не нужно (иначе в подсказку попадает собственный статус).
    set_self_port(port)
    url = kiosk_url(port)
    base_url = f"http://127.0.0.1:{port}"

    open_screen = args.open_screen
    if open_screen is None:
        open_screen = _env_flag("RC_KIOSK_OPEN", display_available())
    watch = args.watch
    if watch is None:
        watch = open_screen and _env_flag("RC_KIOSK_WATCHDOG", True)
    if open_screen and not display_available():
        log.warning("дисплей не найден (нет DISPLAY/WAYLAND_DISPLAY) — "
                    "экран будет доступен по адресу %s", url)

    stop_event = threading.Event()

    # Сервер поднимаем сами, если порт свободен: так сначала связываем сокет,
    # а браузер открываем уже по живому адресу.
    server = None
    if port_in_use(port):
        log.info("порт %s уже обслуживается (сервер уже запущен) — сервер не поднимаю", port)
    else:
        try:
            server = make_server(host, port, create_kiosk_app(cfg), threaded=True)
        except OSError as exc:
            log.error("не удалось занять порт %s: %s", port, exc)
            return 1

    # Остановка по сигналу (Ctrl+C, kill из ./scripts/stop.sh или от ярлыка
    # «Запустить робот»): сначала взводим stop_event — сторож экрана закрывает
    # окно киоска, — и только потом выходит процесс. Без этого окно оставалось
    # на мониторе сиротой.
    _install_stop_signals(stop_event, server)

    if server is None:
        # Порт уже занят: экран отдаёт кто-то другой (служба robot-kiosk или
        # прошлый запуск). Наша задача — просто показать его на дисплее.
        print(f"Экран робота: {url} (сервер уже запущен отдельно)")
        if not open_screen:
            return 0
        # Свой pid-файл: сервер этому процессу не принадлежит, var/kiosk.pid
        # записан не им, и ./scripts/stop.sh сторожа экрана раньше не видел —
        # тот переживал остановку и открывал окно на мониторе снова и снова.
        _write_display_pid_file(port)
        try:
            display = open_kiosk_screen(port, watch=watch, stop_event=stop_event,
                                        host=host)
            if display is None:
                # Экран уже на дисплее (автозапуск/прошлый запуск его держит).
                # Мы — лишний дублирующий запуск: не висим и окна не плодим.
                print("Экран уже открыт другим запуском — выхожу, чтобы не дублировать окно.")
                return 0
            try:
                while display.is_alive():
                    display.join(timeout=0.5)
            except KeyboardInterrupt:
                pass
            stop_event.set()
            display.join(timeout=BROWSER_REOPEN_SEC + 1.0)
            log.info("показ экрана остановлен")
            return 0
        finally:
            _remove_display_pid_file(port)

    display = None
    if open_screen:
        # None (экран уже открыт) — не беда: сервер всё равно должен работать,
        # а открытое окно показывает именно этот адрес.
        display = open_kiosk_screen(port, watch=watch, stop_event=stop_event,
                                    host=host)

    _write_pid_file()
    log.info("экран робота (киоск) слушает http://%s:%s/ · источник: %s",
             host, port, cfg.slam_source)
    log.info("на роботе: http://127.0.0.1:%s%s · без авторизации", port, KIOSK_URL_PATH)
    log.info("Ctrl+C — остановить сервер")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("остановлено")
    finally:
        stop_event.set()
        if display is not None:
            # Поток показа — демон: не дождавшись его, процесс вышел бы раньше,
            # чем окно киоска закрылось, и экран остался бы на мониторе.
            display.join(timeout=BROWSER_CLOSE_TIMEOUT_SEC * 2 + 2.0)
        server.server_close()
        _remove_pid_file()
    return 0


if __name__ == "__main__":
    sys.exit(main())
