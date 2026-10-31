# Copyright 2026 Zapret robot control
#
# Быстрая проверка порта перед запуском драйвера: читает байты и говорит, какой
# это лидар. Нужна для ``model:=auto`` — иначе супервизор тратил бы по таймауту
# на каждый драйвер, а оператор ждал бы скан десятки секунд.
"""Проверка последовательного порта: какой лидар на нём виден."""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any, Optional

from .frames import (PROTOCOL_BAUDS, RPLIDAR_CMD_SCAN, SCORE_STRONG, SCORE_WEAK,
                     score_bytes)
from . import usb_guard

log = logging.getLogger("zapret_lidar.probe")

#: Сколько читаем на одной скорости, с. Меньше — не увидим кадры LDS-01 (они
#: идут ~5 Гц), больше — перебор портов станет заметным на глаз. Это именно
#: окно времени: непрерывный поток лидара не должен растягивать проверку на
#: десятки секунд (раньше чтение шло до 64 КБ и «зависало» на живом порту).
READ_WINDOW_S = 0.45
#: Предел прочитанного: 16 КБ хватает на десятки кадров LDS-01/LD06, а живой
#: поток не заставляет проверку читать до бесконечности.
MAX_READ_BYTES = 16 * 1024
#: Сколько секунд максимум тратить на все порты в одном пересмотре плана.
PROBE_BUDGET_S = 2.5


def _open(port: str, baud: int, timeout: float = 0.15):
    """Открывает порт через pyserial; None, если библиотеки нет или порт занят."""
    try:
        import serial  # pyserial
    except ImportError:  # pragma: no cover — на роботе pyserial стоит в .venv
        log.debug("нет pyserial: проверка порта пропускается")
        return None
    try:
        # exclusive: не отнимаем байты у уже работающего драйвера. На старом
        # pyserial параметра нет — тогда открываем как раньше.
        return serial.Serial(port, baud, timeout=timeout, exclusive=True)
    except TypeError:
        try:
            return serial.Serial(port, baud, timeout=timeout)
        except Exception as exc:  # noqa: BLE001 — порт занят/нет прав/нет устройства
            log.debug("порт %s: не открылся на %s бод: %s", port, baud, exc)
            return None
    except Exception as exc:  # noqa: BLE001 — порт занят/нет прав/нет устройства
        log.debug("порт %s: не открылся на %s бод: %s", port, baud, exc)
        return None


def read_bytes(port: str, baud: int, window: float = READ_WINDOW_S,
               *, command: Optional[bytes] = None) -> bytes:
    """Читает байты порта за окно времени (без исключений: пустая строка — молчит).

    ``window`` — жёсткий предел по стенным часам, а не «пока в порту есть
    данные»: лидар сыплет кадры непрерывно, и чтение до опустошения буфера
    занимало секунды на каждой скорости и каждом порту. Для проверки нам достаточно 0,45 с.
    """
    timeout = max(0.02, min(0.1, window))
    token = usb_guard.claim(port)
    if token is None:
        return b""
    handle = None
    try:
        if not usb_guard.permitted(port):
            return b""
        handle = _open(port, baud, timeout=timeout)
        if handle is None:
            return b""
        if command:
            try:
                handle.dtr = False
                handle.rts = False
            except Exception:  # noqa: BLE001 — у части адаптеров линий нет
                pass
            try:
                handle.write(command)
                handle.flush()
            except Exception as exc:  # noqa: BLE001 — передатчик залип
                log.debug("порт %s: команда не ушла: %s", port, exc)
                return b""
        deadline = time.monotonic() + max(0.02, window)
        chunks: list[bytes] = []
        total = 0
        while total < MAX_READ_BYTES:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                # pyserial читает не дольше таймаута; не даём одному read()
                # выйти за окно проверки.
                handle.timeout = min(0.1, max(0.01, remaining))
            except (AttributeError, OSError, ValueError):
                pass
            try:
                data = handle.read(4096)
            except Exception as exc:  # noqa: BLE001 — обрыв кабеля на ходу
                log.debug("порт %s: чтение оборвалось: %s", port, exc)
                break
            if not data:
                break
            chunks.append(data)
            total += len(data)
        return b"".join(chunks)
    finally:
        if handle is not None:
            try:
                handle.close()
            except Exception:  # noqa: BLE001
                pass
        usb_guard.release(token)


def probe_port(port: str, *, bauds: Optional[dict[str, tuple[int, ...]]] = None,
               window: float = READ_WINDOW_S) -> dict[str, Any]:
    """Что видно на порту: протокол, оценка, найденная скорость, что проверяли.

    Возвращает ``{"port":…, "protocol": "", "score": 0.0, "baud": 0,
    "bauds": {...}, "scores": {...}}``. Пустой ``protocol`` — порт молчит или
    на нём не лидар; это не ошибка, а результат проверки. ``baud`` — скорость,
    на которой лучший протокол реально отдал кадры: RPLIDAR A2/A3/C1 нельзя
    запускать на скорости A1.
    """
    rates = bauds or PROTOCOL_BAUDS
    scores: dict[str, float] = {}
    protocol_bauds: dict[str, int] = {}
    for protocol, candidates in rates.items():
        best = 0.0
        best_baud = 0
        for baud in candidates:
            # RPLIDAR сам молчит: его надо попросить о скане командой SCAN.
            command = RPLIDAR_CMD_SCAN if protocol == "rplidar" else None
            data = read_bytes(port, baud, window, command=command)
            if not data:
                continue
            score = score_bytes(data, protocol)
            if score > best:
                best = score
                best_baud = baud
            if score >= SCORE_STRONG:
                # Уверенно опознали протокол: и другие скорости этого
                # протокола, и остальные протоколы уже не нужны — экономим
                # до секунды на каждой проверке (а их несколько за запуск).
                break
            if protocol in ("lds01", "ld06") and score >= SCORE_WEAK:
                break  # поток односторонний: убедились и хватит
        if best > 0:
            scores[protocol] = best
            protocol_bauds[protocol] = best_baud
        if best >= SCORE_STRONG:
            break
    protocol, score = best_protocol_from(scores)
    return {"port": port, "protocol": protocol, "score": round(score, 3),
            "baud": protocol_bauds.get(protocol, 0),
            "protocol_bauds": dict(protocol_bauds),
            "bauds": {name: list(values) for name, values in rates.items()},
            "scores": {name: round(value, 3) for name, value in scores.items()}}


def best_protocol_from(scores: dict[str, float]) -> tuple[str, float]:
    """Протокол с наибольшей оценкой (пусто, если все оценки ниже порога)."""
    if not scores:
        return "", 0.0
    name = max(scores, key=lambda key: scores[key])
    score = float(scores[name])
    if score < SCORE_WEAK:
        return "", 0.0
    return name, score


def probe_ports(ports: list[str], *, limit: int = 4,
                budget_s: float = PROBE_BUDGET_S) -> dict[str, dict[str, Any]]:
    """Проверяет порты по очереди и возвращает отчёт по каждому (для статуса).

    Общий бюджет ``budget_s`` не даёт супервизору «зависнуть» на минуту, пока
    он перебирает все скорости всех портов: проверка идёт до первого уверенно
    опознанного лидара, а остальные порты проверятся при следующем пересмотре.
    """
    reports: dict[str, dict[str, Any]] = {}
    started = time.monotonic()
    for port in ports[:limit]:
        if not Path(port).exists():
            continue
        if reports and time.monotonic() - started >= budget_s:
            break
        reports[port] = probe_port(port)
        if reports[port].get("score", 0.0) >= SCORE_STRONG:
            break
    return reports


def scores_for_plan(reports: dict[str, dict[str, Any]]) -> dict[str, float]:
    """Оценки в виде, который понимает :func:`zapret_lidar.drivers.build_plan`."""
    out: dict[str, float] = {}
    for port, report in reports.items():
        for protocol, score in (report.get("scores") or {}).items():
            key = "%s@%s" % (protocol, port)
            out[key] = max(out.get(key, 0.0), float(score))
    return out


def bauds_for_plan(reports: dict[str, dict[str, Any]]) -> dict[str, int]:
    """Найденные скорости в виде ``{"протокол@порт": бод}`` для build_plan."""
    out: dict[str, int] = {}
    for port, report in reports.items():
        protocol = str(report.get("protocol") or "")
        baud = int(report.get("baud") or 0)
        if protocol and baud:
            out["%s@%s" % (protocol, port)] = baud
    return out
