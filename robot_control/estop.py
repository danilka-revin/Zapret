"""Аварийный останов (E-STOP) — защёлка, общая для пульта (8080) и борта (8090).

Два экрана — два процесса, поэтому защёлка живёт в файле (``data/estop.json``),
а не в памяти: нажатие на одном экране блокирует тягу и руление на обоих.

Правила безопасности:

* нажатие E-STOP шлёт нулевую тягу в источник и взводит защёлку;
* пока защёлка взведена, любая ненулевая тяга и любое движение руля отклоняются
  с причиной ``estop`` (нулевая тяга — останов — разрешена всегда);
* снимает защёлку только оператор с PIN-сессией (см. ``slam.py``);
* файл, который не удаётся прочитать, считается **взведённым** (fail-safe):
  повреждение файла не должно молча снимать останов.
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
import time
from pathlib import Path
from typing import Any, Optional

log = logging.getLogger("robot_control.estop")

DEFAULT_PATH = Path(__file__).resolve().parents[1] / "data" / "estop.json"


def latch_path() -> Path:
    """Путь к файлу защёлки; переопределяется ``RC_ESTOP_FILE`` (читается на каждом вызове)."""
    raw = os.environ.get("RC_ESTOP_FILE")
    return Path(raw) if raw else DEFAULT_PATH


def latched() -> Optional[dict[str, Any]]:
    """Сведения о защёлке, если она взведена; ``None`` — движение разрешено.

    Если файла нет — защёлки нет. Если файл есть, но не читается, возвращается
    сведения с ``corrupt=True`` (считается взведённой).
    """
    path = latch_path()
    try:
        raw = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    except OSError as exc:  # нет прав, диск и т. п. — не снимаем останов
        log.error("защёлка E-STOP не читается (%s): %s", path, exc)
        return {"reason": "unreadable", "corrupt": True, "since": None}
    try:
        data = json.loads(raw)
    except ValueError:
        log.error("защёлка E-STOP повреждена: %s", path)
        return {"reason": "corrupt", "corrupt": True, "since": None}
    if not isinstance(data, dict) or not data.get("latched", False):
        return None
    return {
        "reason": str(data.get("reason") or "оператор"),
        "since": data.get("since"),
        "corrupt": False,
    }


def _write(payload: dict[str, Any]) -> None:
    path = latch_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".estop-", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def latch(reason: str = "оператор") -> dict[str, Any]:
    """Взвести защёлку (идемпотентно; первое время остаётся в файле)."""
    current = latched()
    if current and not current.get("corrupt"):
        return current
    info = {"latched": True, "reason": reason, "since": time.time()}
    _write(info)
    return {"reason": reason, "since": info["since"], "corrupt": False}


def release() -> None:
    """Снять защёлку (вызывать только после проверки PIN-сессии)."""
    _write({"latched": False, "released_at": time.time()})
