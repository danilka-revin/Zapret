"""
Zapret Control — удобное приложение для обхода DPI-блокировок/замедлений
на Linux (YouTube, Discord, Telegram и другие сервисы).

Построено на базе zapret (bol-van) и стратегий Flowseal/zapret-discord-youtube.
"""

import os
import sys
from pathlib import Path

APP_NAME = "Zapret Control"
APP_VERSION = "2.9.0"
APP_SLUG = "zapret-control"

# Репозитории-источники (загружаются автоматически при установке зависимостей)
ZAPRET_REPO = "bol-van/zapret"                 # nfqws
STRATEGIES_REPO = "Flowseal/zapret-discord-youtube"  # стратегии (.bat + lists + bin)
STRATEGIES_DEFAULT_REV = "ef19845a801e4e743f7bdfdbd58f9745c6adbd60"  # проверенный коммит

# nftables/очередь (совместимо со стратегиями Flowseal)
NFT_TABLE = "inet zapretunix"
NFT_CHAIN_PRE = "pre"
NFT_CHAIN_POST = "post"
NFT_QUEUE_NUM = 220
NFT_MARK = "0x40000000"

# Порт-заглушка для gamefilter, когда он выключен (ничего не матчит)
GAME_FILTER_PORTS = "1024-65535"
GAME_FILTER_OFF_PORTS = "12"


def app_dir() -> Path:
    """Корневая директория приложения (код + зависимости + конфиг).

    Обычно это ~/.local/share/zapret-control. Отдельно обрабатывается случай
    «всё установили через sudo -i»: каталога в /root нет, а в домашнем каталоге
    пользователя рабочего стола он есть — тогда работаем с копией пользователя,
    иначе ярлык и конфиг уезжали бы в /root и «приложение не появлялось».
    """
    override = os.environ.get("ZAPRET_APP_DIR")
    if override:
        return Path(override).expanduser()
    base = os.environ.get("XDG_DATA_HOME")
    if not base:
        base = str(Path.home() / ".local" / "share")
    candidate = Path(base) / APP_SLUG
    if candidate.exists() or getattr(os, "geteuid", lambda: 1)() != 0:
        return candidate
    try:
        from . import session as _session

        user = _session.desktop_user()
        if user and user != "root":
            other = _session.user_home(user) / ".local" / "share" / APP_SLUG
            if (other / "run.py").exists():
                return other
    except Exception:  # noqa: BLE001 — диагностика окружения не должна ломать запуск
        pass
    return candidate


def is_installed_mode() -> bool:
    """True, если код выполняется из установленной копии (~/.local/share/...)."""
    return os.environ.get("ZAPRET_APP_DIR") is not None or __file__.startswith(str(app_dir()))


def ensure_on_path():
    """Добавляет корень установленного приложения в sys.path (для systemd-режима)."""
    root = str(app_dir())
    if root not in sys.path:
        sys.path.insert(0, root)
