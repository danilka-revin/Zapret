"""Автопрошивка плат колёс после обновления с GitHub.

Порядок такой:

1. Кнопка «Обновить с GitHub» ставит код и **оставляет заявку** — файл
   ``var/firmware-pending.json``. Прошивать прямо в момент обновления нельзя:
   USB-порты колёс держит процесс бортового экрана, а не тот, что обновляет код.
2. Экран робота при следующем старте (после обновления он перезапускается сам)
   видит заявку **до** открытия портов и запускает ``scripts/flash_firmware.sh``.
   Порты берутся из настроек: ``RC_WHEEL_FL_PORT … RC_WHEEL_RR_PORT`` в ``.env``,
   то есть ровно те, что выбраны на бортовом экране.
3. Скрипт собирает ``motor_wheel_<КОЛЕСО>.ino`` с вписанным именем колеса и
   прошивает плату через ``arduino-cli``. «Актуальная версия» проверяется по
   самой плате: она сообщает ``fw=`` в каждой строке ``@TLM``, и скрипт сверяет
   это с ``FIRMWARE_PROTOCOL`` из скетча (``scripts/firmware_probe.py``). Плата с
   другой прошивкой перепрошивается, даже если локальная отметка говорит
   «уже прошито» — её могли заменить или прошить старой копией с рабочего стола.
   Отметка по хешу скетча (``var/firmware-flashed``) только избавляет от лишнего
   стирания платы, когда всё и так совпало.

Если ``arduino-cli`` нет, скрипт пробует поставить его сам
(``scripts/install_arduino_cli.sh``, без sudo); не вышло — платы прошиваются
вручную копиями ``motor_wheel_*.ino`` с рабочего стола, об этом пишется в журнал.

Копии прошивок на рабочем столе (``scripts/sync_firmware_desktop.sh``) остаются
на месте: их можно открыть в Arduino IDE и прошить вручную, если ``arduino-cli``
на машине нет.
"""

from __future__ import annotations

import json
import logging
import os
import re
import subprocess
import time
from pathlib import Path

log = logging.getLogger(__name__)

#: Сколько ждём автопрошивку четырёх плат. arduino-cli на каждую тратит
#: несколько секунд (сборка кешируется между платами), запас — на медленный USB.
FLASH_TIMEOUT_S = 300

PENDING_NAME = "firmware-pending.json"

#: Версия прошивки из скетча (``FIRMWARE_PROTOCOL``) — та, которая должна стоять
#: на платах. Платы сообщают её же полем ``fw=`` в ``@TLM``.
PROTOCOL_RE = re.compile(r"FIRMWARE_PROTOCOL\s*=\s*(\d+)")

#: Коды возврата scripts/flash_firmware.sh (см. его шапку).
EXIT_TOOL_MISSING = 2
EXIT_NO_PORTS = 3


def _root(root: Path | str | None) -> Path:
    if root is not None:
        return Path(root).resolve()
    from .config import PROJECT_ROOT

    return Path(PROJECT_ROOT).resolve()


def pending_path(root: Path | str | None = None) -> Path:
    return _root(root) / "var" / PENDING_NAME


def expected_protocol(root: Path | str | None = None) -> int | None:
    """Версия прошивки, которая лежит в коде (``FIRMWARE_PROTOCOL`` в скетче)."""
    sketch = _root(root) / "firmware" / "motor_wheel" / "motor_wheel.ino"
    try:
        text = sketch.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    match = PROTOCOL_RE.search(text)
    if match is None:
        return None
    try:
        return int(match.group(1))
    except ValueError:                        # pragma: no cover
        return None


def mark_pending(reason: str = "", sha: str = "", root: Path | str | None = None) -> Path:
    """Оставить заявку на автопрошивку: её выполнит бортовой экран при старте."""
    path = pending_path(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"reason": reason, "sha": sha, "at": time.time()}
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    tmp.replace(path)
    log.info("автопрошивка плат: заявка оставлена (%s)", reason or "без причины")
    return path


def pending(root: Path | str | None = None) -> dict | None:
    """Заявка на автопрошивку или ``None``, если прошивать нечего."""
    path = pending_path(root)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else {}


def clear_pending(root: Path | str | None = None) -> None:
    try:
        pending_path(root).unlink()
    except OSError:
        pass


def flash_if_pending(root: Path | str | None = None,
                     timeout: float = FLASH_TIMEOUT_S,
                     always: bool = False) -> dict | None:
    """Прошить платы в порты из настроек.

    Без ``always`` работает только по заявке (кнопка «Обновить с GitHub»).
    С ``always`` — при каждом старте экрана: скрипт сам сверяет хеш прошивки и
    прошивает только изменившиеся платы, а когда прошивать нечего, выходит за
    доли секунды. Так платы получают код и после обычного `git pull`.

    Возвращает отчёт ``{"ok", "code", "output", "reason"}`` или ``None``, если
    заявки нет и ``always`` не задан. Заявка снимается в любом случае: повторять
    её на каждом старте бессмысленно — если прошивка не удалась, в журнале
    останется причина, а человек увидит её на экране.
    """
    if os.environ.get("PYTEST_CURRENT_TEST"):
        return None                      # тесты не трогают железо
    request = pending(root)
    if request is None and not always:
        return None
    project = _root(root)
    script = project / "scripts" / "flash_firmware.sh"
    reason = str((request or {}).get("reason") or "").strip()
    report: dict = {"ok": False, "code": None, "output": "", "reason": reason}
    if not script.is_file():
        report["output"] = "нет scripts/flash_firmware.sh"
        clear_pending(root)
        log.warning("автопрошивка плат: %s", report["output"])
        return report
    command = ["bash", str(script)]
    if reason:
        command += ["--reason", reason]
    log.info("автопрошивка плат: прошиваю колёса в порты из настроек…")
    try:
        result = subprocess.run(command, cwd=str(project), capture_output=True,
                                text=True, timeout=timeout, check=False)
    except (OSError, subprocess.SubprocessError) as exc:
        report["output"] = str(exc)
        clear_pending(root)
        log.warning("автопрошивка плат не запустилась: %s", exc)
        return report
    output = (result.stdout or "") + (result.stderr or "")
    report["code"] = result.returncode
    report["output"] = output.strip()
    report["ok"] = result.returncode == 0
    clear_pending(root)
    for line in report["output"].splitlines()[-12:]:
        log.info("%s", line)
    if result.returncode == 0:
        log.info("автопрошивка плат: готово")
    elif result.returncode == EXIT_TOOL_MISSING:
        log.warning("автопрошивка плат: arduino-cli не установлен — прошейте "
                    "motor_wheel_*.ino с рабочего стола вручную")
    elif result.returncode == EXIT_NO_PORTS:
        log.warning("автопрошивка плат: не выбраны порты колёс — настройка портов "
                    "на бортовом экране, затем обновление ещё раз")
    else:
        log.warning("автопрошивка плат: не все платы прошились (код %s)",
                    result.returncode)
    report["summary"] = summarize(report)
    return report


def summarize(report: dict) -> str:
    """Одна фраза для журнала экрана: что вышло с прошивкой плат."""
    if not report:
        return ""
    code = report.get("code")
    output = str(report.get("output") or "")
    if code == 0:
        # «итог: прошито 4, уже было актуально 0, ошибок 0…» — самая полезная строка.
        for line in reversed(output.splitlines()):
            if "итог:" in line:
                return "платы: " + line.split("итог:", 1)[1].strip()
        if "прошивать нечего" in output:
            return "платы уже на актуальной прошивке"
        return "платы прошиты"
    if code == EXIT_TOOL_MISSING:
        return ("платы не прошиты: нет arduino-cli — прошейте motor_wheel_*.ino "
                "с рабочего стола в Arduino IDE")
    if code == EXIT_NO_PORTS:
        return ("платы не прошиты: не выбраны USB-порты колёс "
                "(«Тяга и руление» → «USB-порты колёс»)")
    if code is None:
        return "платы не прошиты: " + (report.get("output") or "ошибка запуска")[:160]
    return "не все платы прошились — подробности в журнале прошивки"
