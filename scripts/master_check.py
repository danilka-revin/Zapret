#!/usr/bin/env python3
"""Проверка USB-колёс Zapret: на каком порту Arduino и почему «нет связи».

Колесо — плата с прошивкой ``firmware/motor_wheel`` (Arduino Mega / Mega Mini,
USB 115200 бод, имя колеса в ``MODULE_ID``). Каждые 250 мс она печатает
``@TLM mod=FL …``; по этой строке скрипт узнаёт колесо, а не просто «порт
открылся». Порт из ``RC_SLAM_PORT`` — первый кандидат; сервер ведёт карту
«колесо → порт» сам, скрипт помогает найти и закрепить кандидатов.

    .venv/bin/python scripts/master_check.py              # найти мастер сам
    .venv/bin/python scripts/master_check.py --list       # только список портов
    .venv/bin/python scripts/master_check.py --port /dev/ttyUSB0
    .venv/bin/python scripts/master_check.py --seconds 8  # дольше ждать ответа
    .venv/bin/python scripts/master_check.py --save-env   # записать порт в .env
    .venv/bin/python scripts/master_check.py --json       # для скриптов

Почему не ``/dev/ttyACM0``: родная Mega видна как ``ttyACM0``, а клоны на
CH340/FT232/CP2102 — как ``ttyUSB0``. Номер после переподключения меняется,
поэтому ``--save-env`` пишет постоянное имя ``/dev/serial/by-id/…``, если оно
есть.

Порт открывается монопольно, и Arduino при этом перезагружается (DTR). После
проверки/записи порта перезапустите робота и нажмите на экране «Калибровать все
колёса»: ноль руля после перезапуска платы не сохраняется. Если пульт/экран уже
занимает порт, скрипт честно скажет «занят» — это не поломка.

Коды возврата: 0 — колесо найдено, 1 — не найдено (причина в выводе).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_control.master_port import (  # noqa: E402
    ENV_KEY,
    TELEMETRY_PREFIX,
    diagnostics,
    master_ports,
    normalize_port,
    serial_ports,
    stable_port_path,
)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
ENV_FILE = PROJECT_ROOT / ".env"
BAUD = 115200


def log(text: str = "") -> None:
    print(text, flush=True)


def load_env_file(path: Path = ENV_FILE) -> None:
    """Прочитать ``.env`` как приложение: уже заданные переменные главнее."""
    if not path.is_file():
        return
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


def classify_error(exc: Exception) -> tuple[str, str]:
    """Тип ошибки pyserial → (код, понятный текст)."""
    text = str(exc) or type(exc).__name__
    low = text.lower()
    if "permission denied" in low or "access is denied" in low:
        return "permission", ("нет прав на порт: sudo usermod -aG dialout $USER, "
                              "затем выйдите из Ubuntu и войдите снова")
    if "exclusively lock" in low or "resource temporarily unavailable" in low or "busy" in low:
        return "busy", ("порт занят другой программой (пульт/экран, монитор порта "
                        "Arduino IDE или второй скрипт) — закройте её и повторите")
    if "no such file" in low or "could not open port" in low and "no such" in low:
        return "missing", "порт исчез: проверьте USB-кабель и питание платы"
    return "error", text


def probe_port(port: str, *, seconds: float = 5.0, baud: int = BAUD) -> dict:
    """Открыть порт и ждать ``@TLM``; мастером считается только ответивший."""
    port = normalize_port(port)
    result = {"port": port, "ok": False, "error": "", "detail": "",
              "modules": [], "firmware": None, "lines": 0}
    if not port:
        result.update(error="missing", detail="пустое имя порта")
        return result
    if not os.path.exists(port):
        result.update(error="missing", detail="порт не найден (устройство не подключено)")
        return result
    if not os.access(port, os.R_OK | os.W_OK):
        result.update(error="permission",
                      detail="нет прав; sudo usermod -aG dialout $USER и войдите заново")
        return result
    try:
        import serial
    except ImportError:
        result.update(error="pyserial", detail="pyserial не установлен: ./scripts/ensure_deps.sh")
        return result

    dev = None
    try:
        try:
            dev = serial.Serial(port, baud, timeout=0.2, exclusive=True)
        except TypeError:                     # очень старый pyserial без exclusive
            dev = serial.Serial(port, baud, timeout=0.2)
        deadline = time.monotonic() + max(0.5, float(seconds))
        modules: list[str] = []
        firmware = None
        lines = 0
        while time.monotonic() < deadline:
            raw = dev.readline()
            if not raw:
                continue
            lines += 1
            text = raw.decode("utf-8", "replace").strip()
            if not text.startswith(TELEMETRY_PREFIX):
                continue
            fields = {}
            for token in text[len(TELEMETRY_PREFIX):].split():
                key, sep, value = token.partition("=")
                if sep:
                    fields[key] = value
            module = (fields.get("mod") or "").upper()
            if module and module not in modules:
                modules.append(module)
            try:
                firmware = int(fields.get("fw") or 0) or firmware
            except ValueError:
                pass
            result.update(ok=True, modules=modules, firmware=firmware, lines=lines)
            # Каждая плата печатает своё колесо: первого ответа достаточно.
            break
        if not result["ok"]:
            result["detail"] = ("порт открылся, но строк @TLM нет: это не колесо "
                                "Zapret или прошита не motor_wheel")
    except Exception as exc:                  # noqa: BLE001 — pyserial SerialException
        code, detail = classify_error(exc)
        result.update(error=code, detail=detail)
    finally:
        if dev is not None:
            try:
                dev.close()
            except Exception:                 # noqa: BLE001 — закрытие не критично
                pass
    return result


def describe(configured: str = "") -> list[str]:
    """Строки отчёта «что видит система» без открытия портов."""
    info = diagnostics(configured or None)
    lines = ["Настройка: %s=%s%s" % (
        ENV_KEY, info["configured"],
        "" if info["configuredExists"] else "  (не найден)",
    )]
    ports = info["ports"]
    if ports:
        lines.append("USB-последовательные порты:")
        for path in ports:
            real = os.path.realpath(path)
            note = "" if real == path else " → %s" % real
            if not os.access(path, os.R_OK | os.W_OK):
                note += "  [нет прав]"
            lines.append("  • %s%s" % (path, note))
    else:
        lines.append("USB-последовательные порты: не найдены")
    if info["reserved"]:
        lines.append("Исключены (лидар/BMS): %s" % ", ".join(info["reserved"]))
    if info["hint"]:
        lines.append("Подсказка: %s" % info["hint"])
    return lines


def save_env(port: str, path: Path = ENV_FILE) -> dict:
    """Записать ``RC_SLAM_PORT`` в ``.env``, не трогая остальные строки."""
    raw = str(port or "").strip()
    if not raw.startswith("/"):
        raise ValueError("нужен полный путь к порту: /dev/ttyUSB0 или /dev/serial/by-id/…")
    if any(ch.isspace() or ord(ch) < 32 or ord(ch) == 127 or ch in "\"'`$;" for ch in raw):
        raise ValueError("недопустимые символы в имени порта")
    port = normalize_port(raw)
    lines = path.read_text(encoding="utf-8").splitlines() if path.is_file() else []
    out: list[str] = []
    old = None
    seen = False
    for raw in lines:
        key = raw.strip().split("=", 1)[0].strip() if "=" in raw else ""
        if key == ENV_KEY and not raw.lstrip().startswith("#"):
            old = raw.split("=", 1)[1].strip() if "=" in raw else ""
            if not seen:
                out.append("%s=%s" % (ENV_KEY, port))
            seen = True
        else:
            out.append(raw)
    if not seen:
        out.append("%s=%s" % (ENV_KEY, port))
    if not path.is_file() and not any(line.startswith("RC_SECRET_KEY=") for line in out):
        import secrets
        out.insert(0, "RC_SECRET_KEY=%s" % secrets.token_urlsafe(48))
    text = "\n".join(out).rstrip("\n") + "\n"
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    try:
        mode = path.stat().st_mode & 0o777 if path.is_file() else 0o600
        os.chmod(tmp, mode or 0o600)
    except OSError:
        pass
    os.replace(tmp, path)
    return {"key": ENV_KEY, "old": old, "new": port}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Найти USB-колесо Zapret (@TLM), проверить порт и записать его в .env",
    )
    parser.add_argument("--port", default="", help="порт или glob (по умолчанию — из RC_SLAM_PORT и автопоиск)")
    parser.add_argument("--seconds", type=float, default=5.0,
                        help="сколько секунд ждать строку @TLM на каждом порту (по умолчанию 5)")
    parser.add_argument("--list", action="store_true", help="только показать порты и выйти")
    parser.add_argument("--json", action="store_true", help="вывод в JSON (для скриптов)")
    parser.add_argument("--save-env", action="store_true",
                        help="записать найденный порт в .env (постоянное имя by-id, если есть)")
    parser.add_argument("--env-file", default=str(ENV_FILE),
                        help="файл .env для --save-env (по умолчанию .env проекта)")
    args = parser.parse_args(argv)

    load_env_file(Path(args.env_file))
    configured = normalize_port(args.port) or diagnostics()["configured"]
    ports = master_ports(args.port or None) or [configured]

    if args.list:
        for line in describe(args.port):
            log(line)
        return 0 if (os.path.exists(configured) or serial_ports()) else 1

    if not args.json:
        log("Проверка USB-колёс Zapret (Arduino, 115200 бод)")
        for line in describe(args.port):
            log(line)
        log()

    reports = []
    for port in ports:
        if not args.json:
            log("• проверяю %s …" % port)
        report = probe_port(port, seconds=args.seconds)
        reports.append(report)
        if args.json:
            continue
        if report["ok"]:
            modules = ", ".join(report["modules"]) or "—"
            log("  колесо ответило: @TLM, модулей: %d (%s), fw=%s"
                % (len(report["modules"]), modules, report["firmware"] or "?"))
            if (report["firmware"] or 0) < 4:
                log("  внимание: прошивка до v4 (fw=%s) — перепрошите плату файлом "
                    "motor_wheel_%s.ino с рабочего стола." % (report["firmware"] or "?",
                                                              report["modules"][0] if report["modules"] else "XX"))
        else:
            log("  не колесо: %s" % (report["detail"] or report["error"] or "нет ответа"))
    found = next((report for report in reports if report["ok"]), None)

    if args.json:
        payload = {"ok": bool(found), "reports": reports}
        if found:
            payload["port"] = stable_port_path(found["port"])
        if args.save_env and found:
            try:
                payload["env"] = save_env(stable_port_path(found["port"]), Path(args.env_file))
            except (OSError, ValueError) as exc:
                payload["envError"] = str(exc)
        log(json.dumps(payload, ensure_ascii=False))
        return 0 if found else 1

    log()
    if not found:
        log("Колесо не ответило ни на одном порту.")
        log("• проверьте, что прошит именно firmware/motor_wheel "
            "(motor_wheel_FL/FR/RL/RR.ino — каждая плата своим файлом), 115200 бод;")
        log("• закройте монитор порта Arduino IDE; если порт «занят» — остановите "
            "робота (./scripts/stop.sh): один UART нельзя открыть двум программам;")
        log("• проверьте кабель (с данными), питание платы и `dmesg | tail` после подключения;")
        log("• права: sudo usermod -aG dialout $USER, затем выйдите из Ubuntu и войдите снова.")
        return 1

    stable = stable_port_path(found["port"])
    log("Колесо найдено: %s (%s)" % (found["port"], ", ".join(found["modules"]) or "?"))
    if stable != found["port"]:
        log("Постоянное имя (не меняется после переподключения): %s" % stable)
    if args.save_env:
        try:
            change = save_env(stable, Path(args.env_file))
        except (OSError, ValueError) as exc:
            log("Не удалось записать .env: %s" % exc)
            return 1
        log("Записано в %s: %s=%s%s" % (
            args.env_file, ENV_KEY, change["new"],
            "" if change["old"] in (None, change["new"]) else " (было %s)" % change["old"],
        ))
        log("Перезапустите робота: ./scripts/stop.sh && ./scripts/start_robot.sh")
    else:
        log("Записать в .env: %s=%s" % (ENV_KEY, stable))
        log("Или запустите: .venv/bin/python scripts/master_check.py --save-env")
    log("После перезапуска платы нажмите на экране «Калибровать все колёса»: "
        "ноль руля хранится до выключения.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
