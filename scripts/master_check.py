#!/usr/bin/env python3
"""Check only a manually selected wheel port; never scan/open other devices.

  .venv/bin/python scripts/master_check.py --list
  .venv/bin/python scripts/master_check.py --port /dev/ttyUSB0
  .venv/bin/python scripts/master_check.py --port /dev/ttyUSB0 --wheel FL --save-env

Stop the robot before testing a port. No wheel identity is inferred from
firmware; --wheel is the operator's assignment. A complete @TLM confirms data.
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
    TELEMETRY_PREFIX,
    normalize_port,
    serial_ports,
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
    """List kernel USB port names without opening devices."""
    ports = serial_ports(patterns=("/dev/ttyUSB*", "/dev/ttyACM*"))
    return ["USB-последовательные порты:"] + ["  • " + p for p in ports]


def save_env(port: str, path: Path = ENV_FILE, *, wheel: str) -> dict:
    """Persist exactly the operator-selected port and wheel, never infer either."""
    from robot_control.wheel_settings import save_settings, current_settings, ENV_KEYS
    from robot_control.lidar import read_env_file
    if wheel not in ENV_KEYS:
        raise ValueError("Выберите колесо FL, FR, RL или RR")
    values = read_env_file(path)
    settings = {m: values.get(key, current_settings()[m]) for m, key in ENV_KEYS.items()}
    key = ENV_KEYS[wheel]
    old = values.get(key)
    settings[wheel] = port
    result = save_settings(settings, env_path=path)
    return {"key": key, "old": old, "new": result["settings"][wheel]}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Проверить вручную выбранный USB-порт колеса без автопоиска")
    parser.add_argument("--port", default="", help="точное имя порта, например /dev/ttyUSB0")
    parser.add_argument("--wheel", choices=("FL", "FR", "RL", "RR"), help="назначение порта оператором")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--list", action="store_true", help="только список имён портов; ничего не открывать")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--save-env", action="store_true")
    parser.add_argument("--env-file", default=str(ENV_FILE))
    args = parser.parse_args(argv)
    load_env_file(Path(args.env_file))
    if args.list:
        for line in describe():
            log(line)
        return 0
    if not args.port:
        parser.error("Укажите --port. Автоопределение отключено. Список: --list")
    if args.save_env and not args.wheel:
        parser.error("Для сохранения укажите --wheel FL|FR|RL|RR")
    if any(c in args.port for c in '*?['):
        parser.error("Нужно точное имя порта, не шаблон")
    report = probe_port(args.port, seconds=args.seconds)
    payload = {"ok": report["ok"], "reports": [report]}
    if args.save_env and report["ok"]:
        try:
            payload["env"] = save_env(args.port, Path(args.env_file), wheel=args.wheel)
        except (OSError, ValueError) as exc:
            payload.update(ok=False, envError=str(exc))
    if args.json:
        log(json.dumps(payload, ensure_ascii=False))
    else:
        log("%s: %s" % (args.port, "телеметрия получена" if report["ok"] else report["detail"]))
        if "envError" in payload:
            log(payload["envError"])
        if "env" in payload:
            log("Сохранено. Остановите робот и перезапустите программу для применения.")
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
