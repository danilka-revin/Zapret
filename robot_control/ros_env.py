"""Мост к ROS 2: rclpy из ``.venv`` проекта и проверка установленных пакетов.

Зачем это отдельным модулем. Лидар на роботе теперь берётся из ROS 2: драйвер
(``rplidar_ros``, ``hls_lfcd_lds_driver``, ``ldlidar_stl_ros2`` или
``turtlebot3_node``) публикует ``sensor_msgs/LaserScan`` в ``/scan``, а пульт и
бортовой экран читают топик. Но приложение запускается интерпретатором из
``.venv`` (там Flask и pyserial, см. ``scripts/ensure_deps.sh``), а ROS 2 ставится
системно в ``/opt/ros/<дистрибутив>``. Обычный ``.venv`` не видит системные
пакеты, поэтому ``import rclpy`` в нём молча падал и ROS-режим лидара **никогда**
не включался — ни на пульте, ни на борту, хотя ``ros2 topic echo /scan`` в
терминале работал. Здесь этот разрыв закрывается: пути ROS 2 добавляются в
``sys.path`` и переменные окружения процесса, после чего ``rclpy`` импортируется
как обычно.

Второе назначение — диагностика. ``GET /api/lidar`` отдаёт блок ``ros``
(:func:`describe`): какой дистрибутив найден, доступен ли ``rclpy``, какие
пакеты драйверов лидара установлены и что делать, если нужного нет. Оператор
видит не «скана нет», а причину с командой.

Настройки (``.env``):

  * ``RC_LIDAR_SOURCE`` — ``auto`` (по умолчанию), ``ros`` или ``serial``;
  * ``RC_LIDAR_ROS_TOPIC`` — топик скана, по умолчанию ``/scan``;
  * ``RC_ROS_PREFIX`` — префикс ROS 2, если он не в ``/opt/ros/<distro>``;
  * ``ROS_DISTRO`` — обычная переменная ROS 2, читается как есть.
"""

from __future__ import annotations

import logging
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

log = logging.getLogger("robot_control.ros_env")

#: Дистрибутивы ROS 2, которые имеет смысл искать на роботе (от новых к старым).
ROS_DISTROS = ("kilted", "jazzy", "iron", "humble", "rolling", "galactic", "foxy")

#: Где обычно лежит ROS 2, если он поставлен пакетами Ubuntu.
DEFAULT_PREFIX = Path("/opt/ros")

#: Рабочее пространство ROS 2 проекта (пакет ``zapret_lidar``).
WORKSPACE = Path(__file__).resolve().parent.parent / "ros2_ws"

#: Топик скана по умолчанию: так его называют все драйверы лидаров.
DEFAULT_SCAN_TOPIC = "/scan"

#: Топик со состоянием супервизора драйверов (пакет ``zapret_lidar``).
SUPERVISOR_STATUS_TOPIC = "/zapret/lidar/status"

#: ROS-пакеты, которые умеют отдавать ``/scan`` для конкретного лидара.
#: Порядок = порядок предпочтения. Значения сверяются с таблицей драйверов
#: ``ros2_ws/src/zapret_lidar/zapret_lidar/drivers.py`` (это проверяет тест).
LIDAR_DRIVER_PACKAGES: dict[str, tuple[str, ...]] = {
    "lds01": ("hls_lfcd_lds_driver",),
    "rplidar": ("rplidar_ros", "sllidar_ros2"),
    "ld06": ("ldlidar_stl_ros2",),
    "ld19": ("ldlidar_stl_ros2",),
    "stl27l": ("ldlidar_stl_ros2",),
    "opencr": ("turtlebot3_node",),
}

#: apt-имена пакетов драйверов (``{d}`` — дистрибутив: humble, jazzy, …).
APT_PACKAGES: dict[str, str] = {
    "hls_lfcd_lds_driver": "ros-{d}-hls-lfcd-lds-driver",
    "rplidar_ros": "ros-{d}-rplidar-ros",
    "sllidar_ros2": "ros-{d}-sllidar-ros2",
    "turtlebot3_node": "ros-{d}-turtlebot3-node",
}

#: Пакеты, которых нет в apt: их собирают из исходников в ``ros2_ws/src``.
GIT_SOURCES: dict[str, str] = {
    "ldlidar_stl_ros2": "https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2",
}

#: Подсказка оператору: чем поднять драйвер, если скана нет.
SETUP_COMMAND = "./scripts/ros2_setup.sh"
LAUNCH_COMMAND = "./scripts/ros2_lidar.sh"

#: Сколько секунд держится кэш :func:`describe`. Статус лидара читают оба
#: экрана несколько раз в секунду, а разбор ament-индекса — это обход каталога;
#: раз в 3 с достаточно, чтобы оператор видел изменения сразу после установки
#: пакета.
DESCRIBE_TTL_S = 3.0

_lock = threading.Lock()
_rclpy: Optional[Any] = None
_rclpy_error: str = ""
_prepared_prefix: Optional[Path] = None
_describe_cache: tuple[float, str, dict[str, Any]] = (0.0, "", {})


class RosUnavailable(RuntimeError):
    """ROS 2 или ``rclpy`` недоступны; текст исключения — подсказка оператору."""


# --------------------------------------------------------------------------- #
# Поиск установленного ROS 2
# --------------------------------------------------------------------------- #


def _is_prefix(path: Path) -> bool:
    """Похож ли каталог на префикс ROS 2 (есть setup.bash или библиотеки)."""
    try:
        if not path.is_dir():
            return False
    except OSError:
        return False
    return (path / "setup.bash").is_file() or (path / "lib").is_dir() and (
        path / "share" / "ament_index"
    ).is_dir()


def candidate_prefixes() -> list[Path]:
    """Префиксы ROS 2 в порядке проверки: явный, ``ROS_DISTRO``, ``/opt/ros/*``.

    Явный путь (``RC_ROS_PREFIX``) уважается первым: на роботах ROS 2 иногда
    распакован не в ``/opt/ros``, а, например, в домашний каталог.
    """
    out: list[Path] = []

    def add(value: Any) -> None:
        if not value:
            return
        path = Path(str(value)).expanduser()
        if path not in out:
            out.append(path)

    add(os.environ.get("RC_ROS_PREFIX"))
    distro = (os.environ.get("ROS_DISTRO") or "").strip()
    if distro:
        add(DEFAULT_PREFIX / distro)
    for name in ROS_DISTROS:
        add(DEFAULT_PREFIX / name)
    try:
        for path in sorted(DEFAULT_PREFIX.iterdir()):
            if path.is_dir():
                add(path)
    except OSError:
        pass
    for entry in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        add(entry)
    return out


def prefix() -> Optional[Path]:
    """Первый существующий префикс ROS 2 или ``None``, если ROS 2 не установлен."""
    for path in candidate_prefixes():
        if _is_prefix(path):
            return path
    return None


def install_prefix() -> Optional[Path]:
    """Каталог ``install`` собранного рабочего пространства ``ros2_ws`` (если есть)."""
    for name in ("install",):
        candidate = WORKSPACE / name
        if (candidate / "setup.bash").is_file() or (candidate / "local_setup.bash").is_file():
            return candidate
    return None


def distro() -> Optional[str]:
    """Имя дистрибутива (``humble``, ``jazzy``…): из ``ROS_DISTRO`` или из пути."""
    env = (os.environ.get("ROS_DISTRO") or "").strip()
    if env:
        return env
    path = prefix()
    return path.name if path else None


def site_packages_dirs(root: Path) -> list[Path]:
    """Каталоги с python-пакетами ROS 2 внутри префикса (``rclpy`` и соседи)."""
    version = "%d.%d" % sys.version_info[:2]
    out: list[Path] = []
    for tail in (
        Path("lib") / ("python%s" % version) / "site-packages",
        Path("lib") / "python3" / "dist-packages",
        Path("lib") / "python3" / "site-packages",
    ):
        candidate = root / tail
        if candidate.is_dir() and candidate not in out:
            out.append(candidate)
    return out


def python_path_entries(root: Optional[Path] = None) -> list[str]:
    """Пути, которые нужно добавить в ``sys.path``, чтобы ``import rclpy`` сработал."""
    entries: list[str] = []
    for base in (root, install_prefix()):
        if base is None:
            continue
        for directory in site_packages_dirs(base):
            text = str(directory)
            if text not in entries:
                entries.append(text)
    # Системные пакеты Ubuntu: часть зависимостей ROS 2 ставится apt-ом в python3.
    for extra in ("/usr/lib/python3/dist-packages",):
        if Path(extra).is_dir() and extra not in entries:
            entries.append(extra)
    return entries


def _prepend_env(name: str, values: list[str]) -> None:
    """Добавляет значения в переменную окружения, не дублируя существующие."""
    if not values:
        return
    current = [item for item in (os.environ.get(name) or "").split(os.pathsep) if item]
    merged = [item for item in values if item] + [item for item in current if item not in values]
    os.environ[name] = os.pathsep.join(merged)


def ensure_python_path(root: Optional[Path] = None) -> bool:
    """Готовит процесс к импорту ``rclpy``. True — что-то изменилось.

    Кроме ``sys.path`` выставляются переменные, которые нужны уже загруженным
    библиотекам ROS 2: ``AMENT_PREFIX_PATH`` (поиск пакетов), ``LD_LIBRARY_PATH``
    (``librcl.so`` и типы сообщений), ``ROS_DISTRO`` и ``PYTHONPATH`` для
    дочерних процессов. Без них ``import rclpy`` либо падает, либо работает до
    первого создания узла.
    """
    global _prepared_prefix
    base = root or prefix()
    entries = python_path_entries(base)
    changed = False
    for entry in entries:
        if entry not in sys.path:
            sys.path.insert(0, entry)
            changed = True
    _prepend_env("PYTHONPATH", entries)

    prefixes: list[str] = []
    for item in (base, install_prefix()):
        if item is not None and str(item) not in prefixes:
            prefixes.append(str(item))
    if prefixes:
        _prepend_env("AMENT_PREFIX_PATH", prefixes)
        libs: list[str] = []
        for item in (base, install_prefix()):
            if item is None:
                continue
            for tail in ("lib", "lib/x86_64-linux-gnu"):
                path = item / tail
                if path.is_dir():
                    libs.append(str(path))
        _prepend_env("LD_LIBRARY_PATH", libs)
        if base is not None and not os.environ.get("ROS_DISTRO"):
            os.environ["ROS_DISTRO"] = base.name
    if changed or base != _prepared_prefix:
        _prepared_prefix = base
        changed = True
    return changed


def workspace_scripts_dir() -> Optional[Path]:
    """``lib`` собранного рабочего пространства: там лежат исполняемые файлы пакета."""
    install = install_prefix()
    if install is None:
        return None
    for tail in ("lib/zapret_lidar", "lib"):
        path = install / tail
        if path.is_dir():
            return path
    return None


def launch_command(model: str = "auto", port: str = "", baud: str = "") -> str:
    """Команда запуска супервизора драйверов лидара — её показывает статус."""
    parts = ["%s" % LAUNCH_COMMAND]
    if model and model not in ("auto", ""):
        parts.append("--model %s" % model)
    if port:
        parts.append("--port %s" % port)
    if baud:
        parts.append("--baud %s" % baud)
    return " ".join(parts)


# --------------------------------------------------------------------------- #
# rclpy
# --------------------------------------------------------------------------- #


def rclpy_available() -> bool:
    """Импортируется ли ``rclpy`` в этом процессе (после правки путей)."""
    try:
        import_rclpy()
    except RosUnavailable:
        return False
    return True


def import_rclpy():
    """Импортирует ``rclpy``, подготавливая окружение ROS 2.

    Держит модуль в кэше: ``import rclpy`` тяжёлый, а спрашивают его часто
    (статус, старт моста, ``auto``-источник). При отсутствии ROS 2 поднимается
    :class:`RosUnavailable` с готовой подсказкой оператору.
    """
    global _rclpy, _rclpy_error
    if _rclpy is not None:
        return _rclpy
    ensure_python_path()
    try:
        import rclpy  # noqa: PLC0415 — импорт нужен именно в момент вызова
    except Exception as exc:  # noqa: BLE001 — нет rclpy / несовместимая версия python
        _rclpy_error = "%s: %s" % (type(exc).__name__, exc)
        raise RosUnavailable(missing_reason()) from exc
    _rclpy = rclpy
    _rclpy_error = ""
    return rclpy


def rclpy_error() -> str:
    """Текст последней ошибки импорта ``rclpy`` (пусто, если её не было)."""
    return _rclpy_error


def missing_reason(root: Optional[Path] = None) -> str:
    """Подсказка, почему ROS 2 недоступен и что с этим делать."""
    base = root if root is not None else prefix()
    if base is None:
        return ("ROS 2 не установлен (нет /opt/ros/<дистрибутив>): поставь его и "
                "пакеты драйвера лидара командой %s, затем перезапусти приложения"
                % SETUP_COMMAND)
    where = site_packages_dirs(base)
    if not where:
        return ("в префиксе ROS 2 %s нет python-пакетов (lib/python3.*/site-packages): "
                "похоже, поставлен только C++-часть — нужен пакет ros-%s-rclpy "
                "или полный ros-base (%s)" % (base, base.name, SETUP_COMMAND))
    return ("rclpy не импортируется из %s (python %s): проверь поставку ROS 2 "
            "(%s%s)" % (base, "%d.%d" % sys.version_info[:2], SETUP_COMMAND,
                        ("; последняя ошибка: " + _rclpy_error) if _rclpy_error else ""))


def reset_cache() -> None:
    """Сбрасывает кэши: нужны тестам и переключению дистрибутива/пакетов."""
    global _rclpy, _rclpy_error, _prepared_prefix, _describe_cache
    with _lock:
        _rclpy = None
        _rclpy_error = ""
        _prepared_prefix = None
        _describe_cache = (0.0, "", {})


# --------------------------------------------------------------------------- #
# Диагностика для веб-панели
# --------------------------------------------------------------------------- #


def installed_packages(root: Optional[Path] = None) -> set[str]:
    """Пакеты ROS 2, видимые в ament-индексе префиксов (по одному файлу на пакет).

    Так видно, поставлен ли пакет драйвера лидара, не запуская его: панель сама
    говорит «нет ros-humble-rplidar-ros», вместо того чтобы оператор искал
    причину в журнале.
    """
    bases = [item for item in (root, install_prefix(), prefix()) if item is not None]
    names: set[str] = set()
    for base in bases:
        index = base / "share" / "ament_index" / "resource_index" / "packages"
        try:
            for entry in index.iterdir():
                if entry.is_file():
                    names.add(entry.name)
        except OSError:
            continue
    return names


#: Синонимы названий лидаров (как их пишут на корпусе и в .env) → наш ключ.
_LIDAR_ALIASES = {
    "lds": "lds01", "lds-01": "lds01", "lds_01": "lds01", "lds1": "lds01",
    "lds02": "lds01", "lds-02": "lds01", "lds2": "lds01",
    "hls": "lds01", "hls-lfcd2": "lds01", "hls_lfcd2": "lds01", "lfcd2": "lds01",
    "turtlebot": "lds01", "turtlebot3": "lds01", "burger": "opencr",
    "rplidar_ros": "rplidar", "sllidar": "rplidar", "ld-06": "ld06",
    "ld-19": "ld19", "d500": "ld06", "stl-27l": "stl27l",
}


def driver_packages(protocol: str) -> tuple[str, ...]:
    """ROS-пакеты, которые умеют отдавать ``/scan`` для этого лидара.

    Понимает синонимы: ``lds-01``, ``hls-lfcd2``, ``turtlebot3`` → ``lds01``;
    ``rplidar_ros`` → ``rplidar``. Так подсказка «что поставить» не зависает на
    пустом ответе, когда лидар назван как в .env.
    """
    key = (protocol or "").strip().lower()
    key = _LIDAR_ALIASES.get(key, key)
    return LIDAR_DRIVER_PACKAGES.get(key, ())


def install_hint(protocol: str = "auto", *, distro_name: Optional[str] = None) -> str:
    """Что поставить, чтобы у лидара этого типа появился ROS 2-драйвер."""
    name = distro_name or distro() or "humble"
    packages = driver_packages(protocol)
    if not packages and protocol in ("", "auto"):
        packages = tuple(dict.fromkeys(
            package for group in LIDAR_DRIVER_PACKAGES.values() for package in group
        ))
    apt: list[str] = []
    git: list[str] = []
    for package in packages:
        template = APT_PACKAGES.get(package)
        if template:
            apt.append(template.format(d=name))
        elif package in GIT_SOURCES:
            git.append(GIT_SOURCES[package])
    chunks: list[str] = []
    if apt:
        chunks.append("apt: sudo apt install " + " ".join(sorted(set(apt))))
    if git:
        chunks.append("из исходников: " + ", ".join(sorted(set(git))))
    chunks.append("затем: %s (сборка пакета zapret_lidar и запуск)" % SETUP_COMMAND)
    return "; ".join(chunks)


def describe(topic: Optional[str] = None) -> dict[str, Any]:
    """Блок ``ros`` для ``GET /api/lidar``: что есть и чего не хватает.

    Никаких догадок: ``available`` — это результат реального ``import rclpy``,
    ``packages`` — содержимое ament-индекса. Он дешёвый (список каталогов), но
    всё равно вызывается раз в секунду, а не на каждый запрос скана.
    """
    scan_topic = topic or os.environ.get("RC_LIDAR_ROS_TOPIC") or DEFAULT_SCAN_TOPIC
    global _describe_cache
    cached_at, cached_topic, cached = _describe_cache
    if cached and cached_topic == scan_topic and (time.time() - cached_at) < DESCRIBE_TTL_S:
        return dict(cached)
    base = prefix()
    packages = installed_packages()
    available = rclpy_available()
    drivers = {
        name: [package for package in group if package in packages]
        for name, group in LIDAR_DRIVER_PACKAGES.items()
    }
    any_driver = any(drivers.values())
    if not available:
        hint = missing_reason(base)
    elif not any_driver:
        hint = ("ROS 2 есть, пакетов драйверов лидара нет: " + install_hint("auto"))
    else:
        hint = ""
    info = {
        "available": available,
        "distro": distro(),
        "prefix": str(base) if base else None,
        "topic": scan_topic,
        "status_topic": SUPERVISOR_STATUS_TOPIC,
        "packages": sorted(packages) if packages else [],
        "drivers": drivers,
        "workspace": str(install_prefix()) if install_prefix() else None,
        "launch": launch_command(),
        "error": rclpy_error() or None,
        "hint": hint,
    }
    _describe_cache = (time.time(), scan_topic, dict(info))
    return info


def summary_line(topic: Optional[str] = None) -> str:
    """Одна строка для журнала запуска: с чем лидар будет работать."""
    info = describe(topic)
    if info["available"]:
        drivers = ", ".join(sorted(name for name, found in info["drivers"].items() if found))
        return ("ROS 2 %s: rclpy доступен, топик %s, драйверы: %s"
                % (info["distro"] or "?", info["topic"], drivers or "не найдены"))
    return "ROS 2: недоступен — %s" % (info["hint"] or missing_reason())
