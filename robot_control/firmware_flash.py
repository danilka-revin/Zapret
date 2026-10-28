"""Прошивка плат колёс — только по кнопке «Прошить модули».

Ни старт бортового экрана, ни обновление с GitHub платы не прошивают: USB-порты
колёс заняты процессом экрана, а железо должно меняться тогда, когда этого
хочет человек. Поэтому здесь один путь: :func:`start_flash` запускает в фоне
``scripts/flash_firmware.sh``, освободив порты у источника данных, а экран
показывает ход и итог по ``var/firmware-flash-state.json``.

Скрипт берёт порты из настроек (``RC_WHEEL_FL_PORT … RC_WHEEL_RR_PORT`` в
``.env`` — те же, что выбраны на бортовом экране), собирает
``motor_wheel_<КОЛЕСО>.ino`` с вписанным именем колеса и проверяет, что на
плате уже стоит нужная прошивка: она сообщает ``fw=`` в каждой строке ``@TLM``,
скрипт сверяет это с ``FIRMWARE_PROTOCOL`` из скетча
(``scripts/firmware_probe.py``). Плата с другой версией прошивается заново.

``arduino-cli`` не обязателен: без него сборка идёт обычным ``avr-gcc`` из
пакета ``gcc-avr`` + ``avr-libc``, а заливка — ``avrdude``
(``scripts/avr_upload.py``, ``RC_FIRMWARE_BACKEND=auto|cli|avr``). Копии
``motor_wheel_*.ino`` на рабочем столе остаются: их можно открыть в Arduino IDE
и прошить руками, если на машине нет ни того, ни другого.
"""

from __future__ import annotations

import json
import logging
import re
import subprocess
import threading
import time
from pathlib import Path

log = logging.getLogger(__name__)

#: Сколько ждём прошивку четырёх плат. arduino-cli на каждую тратит
#: несколько секунд (сборка кешируется между платами), запас — на медленный USB.
FLASH_TIMEOUT_S = 300

STATE_NAME = "firmware-flash-state.json"

#: Один запуск прошивки на процесс: второй щелчок по кнопке не должен
#: открывать те же USB-порты параллельно.
_run_lock = threading.Lock()
_run_thread: threading.Thread | None = None

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
        return ("платы не прошиты: нет инструментов — sudo apt install gcc-avr "
                "avr-libc avrdude (arduino-cli не нужен)")
    if code == EXIT_NO_PORTS:
        return ("платы не прошиты: не выбраны USB-порты колёс "
                "(«Тяга и руление» → «USB-порты колёс»)")
    if code is None:
        return "платы не прошиты: " + (report.get("output") or "ошибка запуска")[:160]
    return "не все платы прошились — подробности в журнале прошивки"


# ---------------------------------------------------------------------------
# Кнопка «Прошить модули» на бортовом экране: прошивка по запросу, с прогрессом.
# ---------------------------------------------------------------------------


def state_path(root: Path | str | None = None) -> Path:
    return _root(root) / "var" / STATE_NAME


def flash_state(root: Path | str | None = None) -> dict:
    """Что сейчас с прошивкой: ``{"status": idle|busy|done|error, ...}``."""
    path = state_path(root)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        data = None
    out = {"status": "idle", "phase": "", "pct": 0, "lines": [], "error": "",
           "summary": "", "started": None, "done": None, "reason": "", "backend": ""}
    if isinstance(data, dict):
        out.update({key: data.get(key, out[key]) for key in out})
    if out["status"] == "busy" and not running():
        # Процесс перезапустили посреди прошивки (потока нет вовсе) — не врём
        # «идёт»: файл состояния остался от прошлого запуска.
        out["status"] = "error"
        out["error"] = out["error"] or "прошивка прервана перезапуском программы"
    return out


def _write_state(root: Path | str | None, **patch) -> dict:
    path = state_path(root)
    data = flash_state(root)
    data.update(patch)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
        tmp.replace(path)
    except OSError:
        pass
    return data


def running() -> bool:
    """Идёт ли прошивка прямо сейчас."""
    return _run_thread is not None and _run_thread.is_alive()


def start_flash(source=None, root: Path | str | None = None,
                reason: str = "кнопка «Прошить модули»",
                extra_args: tuple[str, ...] = ()) -> dict:
    """Запустить прошивку в фоне (для кнопки на экране).

    ``source`` — источник данных (``MasterSource``): у него на время прошивки
    освобождаются USB-порты и берётся остановка тяги, если она доступна.
    """
    global _run_thread
    with _run_lock:
        if running():
            return dict(flash_state(root), started=False)
        thread = threading.Thread(target=_flash_worker,
                                  args=(source, root, reason, extra_args),
                                  daemon=True, name="firmware-flash")
        # Поток запоминаем до записи «busy»: иначе экран успел бы прочитать
        # файл, не увидеть потока и решить, что прошивка прервана.
        _run_thread = thread
        _write_state(root, status="busy", phase="Готовлюсь: останавливаю колёса…",
                     pct=2, lines=[], error="", summary="", reason=reason,
                     started=time.time(), done=None)
        thread.start()
        return dict(flash_state(root), started=True)


def _flash_worker(source, root, reason, extra_args) -> None:
    project = _root(root)
    script = project / "scripts" / "flash_firmware.sh"
    lines: list[str] = []

    def step(phase: str, pct: int) -> None:
        _write_state(root, status="busy", phase=phase, pct=pct, lines=lines[-60:])

    def resume() -> None:
        if source is None:
            return
        for name in ("resume_ports",):
            callback = getattr(source, name, None)
            if callable(callback):
                try:
                    callback()
                except Exception:            # noqa: BLE001 — колёса важнее отчёта
                    log.exception("прошивка: не удалось вернуть подключение колёс")

    try:
        # Тяга должна быть выключена: прошивка перезагружает плату.
        for name in ("stop_drive", "stop_maneuver"):
            callback = getattr(source, name, None)
            if callable(callback):
                try:
                    callback()
                except Exception:            # noqa: BLE001
                    log.exception("прошивка: не удалось вызвать %s", name)
        suspend = getattr(source, "suspend_ports", None)
        if callable(suspend):
            step("Освобождаю USB-порты колёс…", 6)
            try:
                suspend()
            except Exception:                # noqa: BLE001
                log.exception("прошивка: не удалось освободить порты")
        if not script.is_file():
            raise RuntimeError("нет scripts/flash_firmware.sh")
        command = ["bash", str(script), "--reason", reason, *extra_args]
        step("Собираю и прошиваю платы…", 15)
        process = subprocess.Popen(command, cwd=str(project), stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1)
        assert process.stdout is not None
        for line in process.stdout:
            text = line.rstrip("\n")
            if not text.strip():
                continue
            lines.append(text)
            # Прогресс по строкам: 4 платы — примерно по 20 % на каждую.
            pct = min(90, 15 + 15 * sum(1 for item in lines if "прошиваю" in item))
            if "прошито" in text:
                pct = min(92, pct + 8)
            _write_state(root, status="busy", phase=_phase_from(text), pct=pct,
                         lines=lines[-60:])
        code = process.wait()
        summary = _summary_from(lines, code)
        _write_state(root, status="done" if code == 0 else "error", phase=summary or "Готово",
                     pct=100, lines=lines[-60:], summary=summary, backend=_backend_from(lines),
                     error="" if code == 0 else (summary or "платы не прошились"),
                     done=time.time())
        log.info("прошивка по кнопке: %s", summary or code)
    except Exception as exc:                 # noqa: BLE001 — экран должен знать причину
        log.exception("прошивка по кнопке не удалась")
        _write_state(root, status="error", phase="Прошивка не удалась", pct=0,
                     lines=lines[-60:], error=str(exc)[:300], done=time.time())
    finally:
        resume()


def _phase_from(line: str) -> str:
    """Строка скрипта → короткая фаза для экрана."""
    text = line.replace("[firmware]", "").strip()
    lowered = text.lower()
    if "прошиваю" in lowered:
        return text
    if "прошито" in lowered or "загружено" in lowered:
        return text
    if "собира" in lowered or "скачиваю" in lowered or "ставлю" in lowered:
        return text
    if "нет порт" in lowered or "не выбран" in lowered:
        return "Порты колёс не выбраны в настройках"
    return "Прошиваю платы…"


def _backend_from(lines: list[str]) -> str:
    for line in lines:
        if "без arduino-cli" in line:
            return "avr"
        if "arduino-cli" in line and "прошиваю" in line:
            return "cli"
    return ""


def _summary_from(lines: list[str], code: int) -> str:
    for line in reversed(lines):
        if "итог:" in line:
            return "платы: " + line.split("итог:", 1)[1].strip()
    for line in reversed(lines):
        text = line.replace("[firmware]", "").strip()
        if any(word in text.lower() for word in ("не удалось", "нечем", "нет порт", "не прошит")):
            return text
    return "" if code == 0 else "платы не прошились (код %s)" % code
