"""Остановка робота одной кнопкой: пульт, бортовой экран, лидар и окно киоска.

Кнопка **«Остановить всё»** на экранах гасит ровно то же, что делает человек
руками командой ``./scripts/stop.sh``:

  * пульт оператора (8080) и бортовой экран (8090) — по pid-файлам из ``var/``
    или как службы ``robot-control``/``robot-kiosk``;
  * супервизор лидара ROS 2 (``var/lidar.pid`` и служба ``robot-lidar``) — он
    сам отпускает USB-порт;
  * оставшиеся окна киоск-браузера на дисплее робота.

Порядок важен: сначала браузер получает ответ «останавливаю», и только потом
работает остановка. Поэтому процесс-помощник запускается **отсоединённым**
(``start_new_session=True``): его не убьёт ни остановка группы процессов
приложений, ни падение самого сервера, а вывод идёт в ``var/stop.log``.

Сам сервер, который принял нажатие, гасится отдельно: если приложение
запущено не через ``scripts/start.sh`` (тогда у него нет pid-файла) или
работает под gunicorn, где мастер поднимает воркеров заново, ``stop.sh`` его
не найдёт. Поэтому помощник получает pid сервера явно и перед сигналом
проверяет, что по этому pid всё ещё живёт именно наше приложение — иначе
``kill`` мог бы попасть в чужой процесс, занявший освободившийся pid.

Автозапуск при этом **не выключается** (это отдельное решение оператора:
``./scripts/autostart.sh off``) — после ярлыка «Запустить робот» или загрузки
Ubuntu всё поднимется снова.
"""

from __future__ import annotations

import logging
import os
import shlex
import subprocess
import time
from pathlib import Path

log = logging.getLogger("robot_control.shutdown")

#: Скрипт, который останавливает приложения (то же, что делает человек руками).
STOP_SCRIPT = Path("scripts") / "stop.sh"
#: Куда пишется вывод помощника: по нему видно, что именно остановилось.
LOG_FILE = Path("var") / "stop.log"

#: Пауза перед остановкой: браузер должен успеть получить ответ «останавливаю».
DEFAULT_DELAY_S = 1.5
#: Сколько ждём «мягкую» остановку сервера, прежде чем добить его SIGKILL.
TERM_GRACE_S = 3.0

#: Что кнопка останавливает — списком для окна подтверждения и ответа API.
TARGETS = (
    "пульт оператора (8080)",
    "бортовой экран на дисплее робота (8090)",
    "супервизор лидара ROS 2 (USB-порт освобождается)",
    "окно киоск-браузера на дисплее",
)

#: По этим словам в командной строке узнаём «своё» приложение перед сигналом.
_OUR_COMMANDS = r"(run\.py|run_kiosk\.py|gunicorn)"


def _read_cmdline(pid: int) -> str:
    """Командная строка процесса (пустая строка, если его уже нет)."""
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return ""
    return " ".join(part.decode("utf-8", "replace") for part in raw.split(b"\0") if part)


def server_pid(pid: int | None = None, *, parent: int | None = None,
               parent_cmdline: str | None = None) -> int:
    """Pid сервера, который надо погасить, если ``scripts/stop.sh`` его не найдёт.

    Под gunicorn гасить нужно мастера, а не воркера: воркер мастер поднимет
    заново. Во всех остальных случаях это собственный процесс приложения
    (``run.py --dev``, ``run_kiosk.py``).
    """
    pid = os.getpid() if pid is None else pid
    parent = os.getppid() if parent is None else parent
    cmdline = _read_cmdline(parent) if parent_cmdline is None else parent_cmdline
    if parent > 1 and "gunicorn" in cmdline.lower():
        return parent
    return pid


def _kill_snippet(target: int, signal_name: str) -> str:
    """``kill`` только по нашему процессу: pid мог достаться кому-то другому.

    Проверка «жив и командная строка наша» закрывает редкий, но неприятный
    случай: сервер уже остановлен, его pid ядро успело выдать новому процессу,
    и слепой ``kill`` погасил бы постороннего.
    """
    return (
        "pid=%d; "
        "if kill -0 \"$pid\" 2>/dev/null && "
        "grep -qaE %s /proc/$pid/cmdline 2>/dev/null; "
        "then kill -%s \"$pid\" 2>/dev/null || true; fi"
    ) % (target, shlex.quote(_OUR_COMMANDS), signal_name)


def stop_command(root: Path | str, *, pid: int | None = None,
                 delay: float = DEFAULT_DELAY_S, target: int | None = None) -> str:
    """Команда помощника: пауза → ``scripts/stop.sh`` → гашение самого сервера.

    Сервер гасится **после** скрипта: тот снимает pid-файлы и службы, и только
    затем видно, остался ли процесс, запущенный вручную.
    """
    project_root = Path(root).resolve()
    script = project_root / STOP_SCRIPT
    server = server_pid(pid=pid) if target is None else target
    return (
        "sleep %s; "
        "echo \"[stop] %s — остановка по кнопке\"; "
        "bash %s; echo \"[stop] scripts/stop.sh завершён\"; "
        "%s; sleep %s; %s; echo \"[stop] готово\""
    ) % (
        "%.1f" % max(0.0, delay),
        time.strftime("%Y-%m-%d %H:%M:%S"),
        shlex.quote(str(script)),
        _kill_snippet(server, "TERM"),
        "%.1f" % TERM_GRACE_S,
        _kill_snippet(server, "KILL"),
    )


def stop_all(root: Path | str, *, pid: int | None = None,
             delay: float = DEFAULT_DELAY_S, log_path: Path | None = None) -> dict:
    """Запускает отсоединённого помощника, который гасит приложения.

    Возвращает ответ для API. ``stopping`` — помощник запущен: остановка идёт
    уже после того, как браузер получит этот ответ.
    """
    project_root = Path(root).resolve()
    script = project_root / STOP_SCRIPT
    log_file = Path(log_path) if log_path else project_root / LOG_FILE
    server = server_pid(pid=pid)
    if not script.is_file():
        return {
            "ok": False,
            "stopping": False,
            "error": "не найден %s — останови вручную: ./scripts/stop.sh" % script,
            "log": str(log_file),
        }
    command = stop_command(project_root, delay=delay, target=server)
    try:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        handle = open(log_file, "a", encoding="utf-8")  # noqa: SIM115 — fd остаётся у помощника
    except OSError as exc:
        return {
            "ok": False,
            "stopping": False,
            "error": "не удалось открыть %s: %s" % (log_file, exc),
            "log": str(log_file),
        }
    try:
        handle.write("\n[stop] кнопка «Остановить всё»\n")
        handle.flush()
        process = subprocess.Popen(  # noqa: S603 — команда собирается здесь же
            ["bash", "-c", command],
            cwd=str(project_root),
            stdin=subprocess.DEVNULL,
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )
    except OSError as exc:
        return {
            "ok": False,
            "stopping": False,
            "error": "не удалось запустить остановку: %s" % exc,
            "log": str(log_file),
        }
    finally:
        # Файл остаётся открытым у помощника: закрываем только свою копию.
        try:
            handle.close()
        except OSError:  # pragma: no cover
            pass
    log.warning(
        "остановка робота запущена (помощник pid %s, сервер pid %s, журнал %s)",
        process.pid, server, log_file,
    )
    return {
        "ok": True,
        "stopping": True,
        "message": "останавливаю: " + ", ".join(TARGETS),
        "targets": list(TARGETS),
        "script": str(script),
        "log": str(log_file),
        "helperPid": process.pid,
        "serverPid": server,
        "delayS": max(0.0, delay),
        "autostart": "автозапуск не выключается (./scripts/autostart.sh off — если не нужен)",
        "restart": "вернуть всё: ярлык «Запустить робот» или ./scripts/start.sh",
    }
