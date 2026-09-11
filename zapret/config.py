"""
Конфигурация приложения (JSON в APP_DIR/config.json).
"""

import json
from pathlib import Path

from . import app_dir

CONFIG_FILE_NAME = "config.json"

DEFAULTS = {
    "interface": "any",
    "strategy": "general.bat",
    "gamefilter_tcp": False,
    "gamefilter_udp": False,
    "firewall_backend": "auto",      # auto | nftables | iptables
    "telegram": True,                 # добавить обход Telegram
    "autostart": False,               # системная служба
    "nfqws_version": "latest",        # latest | vXX.Y
    "strategy_rev": "",               # пусто = рекомендованный коммит
    # Оформление и автономика (см. zapret/qt/theme.py -> UISettings)
    "ui": {},
}


def config_path() -> Path:
    return app_dir() / CONFIG_FILE_NAME


def load() -> dict:
    cfg = dict(DEFAULTS)
    p = config_path()
    if p.exists():
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                for k in DEFAULTS:
                    if k in data:
                        cfg[k] = data[k]
        except (OSError, ValueError):
            pass
    return cfg


def save(cfg: dict) -> None:
    app_dir().mkdir(parents=True, exist_ok=True)
    config_path().write_text(
        json.dumps(cfg, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
