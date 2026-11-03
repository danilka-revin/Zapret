"""Состояние бортового компьютера: температура, загрузка, память, диск.

Только чтение ``/proc`` и ``/sys`` — без внешних программ, без ``sudo``, без
записи в систему. Один фоновый сборщик на процесс: HTTP-запрос оператора и
моторные команды никогда не ждут чтения датчиков.

Правила те же, что у инвентаря подключений (``connections.py``):

* отсутствующий датчик — это «нет данных», а не выдуманная цифра;
* снимок копируется при выдаче, чтобы браузер не менял кэш сервера;
* ошибки чтения попадают в ``warnings`` и не роняют страницу.

Значения температур берутся из ``/sys/class/thermal`` (там их публикует ядро
для ARM-плат вроде Raspberry Pi) и, если есть, из ``/sys/class/hwmon``
(x86-платы). Счётчик троттлинга читается из ``thermal_throttle``, когда он есть;
иначе поле не показывается.
"""
from __future__ import annotations

import copy
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional

#: Порог предупреждения по самому горячему датчику. Ядро ARM обычно начинает
#: снижать частоту около 80–85 °C, поэтому «внимание» ставим заранее.
WARN_TEMP_C = 75.0
#: Порог «критично»: тягу в этот момент лучше не запускать.
CRIT_TEMP_C = 85.0
WARN_MEMORY_PERCENT = 90.0
CRIT_MEMORY_PERCENT = 97.0
WARN_DISK_PERCENT = 90.0
CRIT_DISK_PERCENT = 97.0
#: Минимальная пауза между фоновыми чтениями: чаще нет смысла, а нагрузка лишняя.
DEFAULT_INTERVAL_S = 2.0
#: Дольше этого снимок считается устаревшим (экран покажет предупреждение).
STALE_AFTER_S = 15.0


def _env_float(name: str, default: float) -> float:
    try:
        value = float(os.environ.get(name, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _text(path: Path, limit: int = 4096) -> str:
    try:
        return path.read_text(errors="replace")[:limit].strip()
    except OSError:
        return ""  # нет файла — нормальная ситуация для опциональных датчиков


def _number(text: str) -> Optional[float]:
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _celsius(raw: Optional[float]) -> Optional[float]:
    """Ядро отдаёт миллиградусы, но не все драйверы это соблюдают."""
    if raw is None:
        return None
    value = raw / 1000.0 if abs(raw) >= 1000 else raw
    # Мусорные значения (0, −273, 1000) не выдаём за измерение.
    if not -40.0 <= value <= 150.0:
        return None
    return round(value, 1)


def _meminfo(proc: Path) -> dict[str, int]:
    """``/proc/meminfo`` в килобайтах; неизвестные поля просто отсутствуют."""
    result: dict[str, int] = {}
    for line in _text(proc / "meminfo", 64_000).splitlines():
        name, _, rest = line.partition(":")
        parts = rest.split()
        if not parts:
            continue
        try:
            result[name.strip()] = int(parts[0])
        except ValueError:
            continue
    return result


class SystemState:
    """Один снимок состояния компьютера из ``/proc`` и ``/sys``."""

    def __init__(self, procroot: str | Path = "/proc", sysroot: str | Path = "/sys",
                 root: str | Path = "/", clock: Callable[[], float] = time.time,
                 hostname: Optional[Callable[[], str]] = None):
        self.proc = Path(procroot)
        self.sys = Path(sysroot)
        self.root = Path(root)
        self.clock = clock
        self._hostname = hostname or (lambda: os.uname().nodename)
        self.warn_temp_c = _env_float("RC_PC_TEMP_WARN_C", WARN_TEMP_C)
        self.crit_temp_c = _env_float("RC_PC_TEMP_CRIT_C", CRIT_TEMP_C)
        if self.crit_temp_c <= self.warn_temp_c:
            self.warn_temp_c, self.crit_temp_c = WARN_TEMP_C, CRIT_TEMP_C
        # Для процента загрузки нужны два замера /proc/stat подряд.
        self._previous_cpu: Optional[tuple[float, float]] = None
        # Один и тот же снимок читают поток экрана и поток инвентаря.
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ cpu --
    def _cpu_times(self) -> Optional[tuple[float, float]]:
        """(занятое время, всё время) по первой строке ``/proc/stat``."""
        for line in _text(self.proc / "stat", 64_000).splitlines():
            if not line.startswith("cpu "):
                continue
            values = []
            for token in line.split()[1:9]:
                try:
                    values.append(float(token))
                except ValueError:
                    return None
            if len(values) < 4:
                return None
            idle = values[3] + (values[4] if len(values) > 4 else 0.0)
            return sum(values) - idle, sum(values)
        return None

    def cpu(self) -> dict[str, Any]:
        cores, model, frequency_mhz = None, "", None
        cpuinfo = _text(self.proc / "cpuinfo", 128_000)
        for line in cpuinfo.splitlines():
            name, _, value = line.partition(":")
            key = name.strip().lower()
            value = value.strip()
            if key == "processor":
                cores = (cores or 0) + 1
                # ARM пишет в «processor» название ядра, x86 — номер ядра.
                if not model and value and not value.isdigit():
                    model = value
            elif not model and key in ("model name", "hardware", "cpu model"):
                model = value
        if not cores:
            cores = cpuinfo.count("processor\t") or None
        if not cores:
            try:
                cores = len(os.sched_getaffinity(0))
            except (AttributeError, OSError):
                cores = None
        # Частота: современный cpufreq, старый cpuinfo_cur_freq, затем /proc/cpuinfo.
        for candidate in (self.sys / "devices/system/cpu/cpu0/cpufreq/scaling_cur_freq",
                          self.sys / "devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq"):
            khz = _number(_text(candidate))
            if khz and khz > 0:
                frequency_mhz = round(khz / 1000.0)
                break
        if frequency_mhz is None:
            for line in cpuinfo.splitlines():
                name, _, value = line.partition(":")
                if name.strip().lower() == "cpu mhz":
                    mhz = _number(value.strip())
                    frequency_mhz = round(mhz) if mhz else None
                    break

        usage = None
        current = self._cpu_times()
        if current is not None:
            with self._lock:
                if self._previous_cpu is not None:
                    busy, total = current
                    was_busy, was_total = self._previous_cpu
                    delta_total = total - was_total
                    delta_busy = busy - was_busy
                    if delta_total > 0:
                        usage = round(max(0.0, min(100.0, delta_busy / delta_total * 100.0)), 1)
                self._previous_cpu = current

        load1 = load5 = load15 = None
        loadavg = _text(self.proc / "loadavg").split()
        if len(loadavg) >= 3:
            load1, load5, load15 = (_number(loadavg[0]), _number(loadavg[1]), _number(loadavg[2]))
        running = None
        if len(loadavg) >= 4 and "/" in loadavg[3]:
            try:
                running = int(loadavg[3].split("/")[0])
            except ValueError:
                running = None
        return {
            "available": bool(cores or model or load1 is not None),
            "model": model,
            "cores": cores,
            "load1": load1,
            "load5": load5,
            "load15": load15,
            "runningProcesses": running,
            "usagePercent": usage,
            "frequencyMHz": frequency_mhz,
        }

    # ---------------------------------------------------------- temperature --
    def _thermal_throttle(self) -> Optional[int]:
        total = 0
        found = False
        for name in ("core_throttle_count", "package_throttle_count"):
            for cpu in ("cpu0", "cpu1"):
                raw = _number(_text(self.sys / f"devices/system/cpu/{cpu}/thermal_throttle/{name}"))
                if raw is not None and raw >= 0:
                    total += int(raw)
                    found = True
        return total if found else None

    def temperature(self, warnings: Optional[list[str]] = None) -> dict[str, Any]:
        zones: list[dict[str, Any]] = []
        seen: set[tuple[str, float]] = set()
        thermal = self.sys / "class/thermal"
        try:
            entries = sorted(thermal.glob("thermal_zone*"))
        except OSError:
            entries = []
        for zone in entries:
            value = _celsius(_number(_text(zone / "temp")))
            if value is None:
                continue
            name = _text(zone / "type") or zone.name
            if (name, value) in seen:
                continue
            seen.add((name, value))
            zones.append({"name": name, "celsius": value, "source": "thermal"})
        # x86: плата может не иметь thermal_zone, зато отдаёт hwmon (k10temp, coretemp).
        hwmon = self.sys / "class/hwmon"
        try:
            chips = sorted(hwmon.glob("hwmon*"))
        except OSError:
            chips = []
        for chip in chips:
            chip_name = _text(chip / "name") or chip.name
            try:
                inputs = sorted(chip.glob("temp*_input"))
            except OSError:
                continue
            for path in inputs:
                value = _celsius(_number(_text(path)))
                if value is None:
                    continue
                label = _text(path.with_name(path.name.replace("_input", "_label")))
                name = f"{chip_name} {label}".strip()
                if (name, value) in seen:
                    continue
                seen.add((name, value))
                zones.append({"name": name, "celsius": value, "source": "hwmon"})

        hottest = max(zones, key=lambda z: z["celsius"]) if zones else None
        result: dict[str, Any] = {
            "available": bool(zones),
            "maxC": hottest["celsius"] if hottest else None,
            "hottest": hottest["name"] if hottest else "",
            "zones": zones,
            "throttleCount": self._thermal_throttle(),
        }
        if not zones:
            result["reason"] = "датчики температуры не найдены (/sys/class/thermal и hwmon пусты)"
            if warnings is not None:
                warnings.append("Температура недоступна: " + result["reason"])
        return result

    # --------------------------------------------------------------- memory --
    def memory(self) -> dict[str, Any]:
        info = _meminfo(self.proc)
        total_kb = info.get("MemTotal")
        if not total_kb:
            return {"available": False, "reason": "/proc/meminfo недоступен"}
        available_kb = info.get("MemAvailable")
        if available_kb is None:
            available_kb = (info.get("MemFree", 0) + info.get("Buffers", 0)
                            + info.get("Cached", 0))
        used_kb = max(0, total_kb - available_kb)
        swap_total = info.get("SwapTotal") or 0
        swap_free = info.get("SwapFree") or 0
        return {
            "available": True,
            "totalMB": round(total_kb / 1024),
            "usedMB": round(used_kb / 1024),
            "availableMB": round(available_kb / 1024),
            "usedPercent": round(used_kb / total_kb * 100.0, 1),
            "swapTotalMB": round(swap_total / 1024),
            "swapUsedMB": round(max(0, swap_total - swap_free) / 1024),
            "swapUsedPercent": (round(max(0, swap_total - swap_free) / swap_total * 100.0, 1)
                                if swap_total else None),
        }

    # ----------------------------------------------------------------- disk --
    def disk(self) -> dict[str, Any]:
        try:
            stats = os.statvfs(str(self.root))
        except OSError as exc:
            return {"available": False, "reason": "statvfs: %s" % exc.strerror}
        total = stats.f_blocks * stats.f_frsize
        free = stats.f_bavail * stats.f_frsize
        if total <= 0:
            return {"available": False, "reason": "размер раздела не определяется"}
        used = max(0, total - free)
        return {
            "available": True,
            "path": str(self.root),
            "totalGB": round(total / 1e9, 1),
            "freeGB": round(free / 1e9, 1),
            "usedPercent": round(used / total * 100.0, 1),
        }

    def uptime(self) -> Optional[float]:
        parts = _text(self.proc / "uptime").split()
        return _number(parts[0]) if parts else None

    # -------------------------------------------------------------- snapshot --
    def collect(self) -> dict[str, Any]:
        warnings: list[str] = []
        sections: dict[str, Any] = {}
        for name in ("cpu", "memory", "disk"):
            try:
                sections[name] = getattr(self, name)()
            except Exception as exc:  # noqa: BLE001 — экран не должен падать из-за /proc
                sections[name] = {"available": False, "reason": str(exc)[:120]}
                warnings.append("%s: %s" % (name, str(exc)[:120]))
        try:
            sections["temperature"] = self.temperature(warnings)
        except Exception as exc:  # noqa: BLE001
            sections["temperature"] = {"available": False, "reason": str(exc)[:120]}
            warnings.append("temperature: %s" % str(exc)[:120])
        try:
            uptime = self.uptime()
        except Exception:  # noqa: BLE001
            uptime = None

        status = "ok"
        cpu, memory, disk = sections["cpu"], sections["memory"], sections["disk"]
        temp = sections["temperature"]
        hottest = temp.get("maxC")
        if isinstance(hottest, (int, float)):
            if hottest >= self.crit_temp_c:
                status = "critical"
                warnings.append("Процессор %.1f °C — выше %.0f °C, нагрузку лучше снять"
                                % (hottest, self.crit_temp_c))
            elif hottest >= self.warn_temp_c:
                status = "warn" if status == "ok" else status
                warnings.append("Процессор %.1f °C — выше %.0f °C, проверьте охлаждение"
                                % (hottest, self.warn_temp_c))
        throttle = temp.get("throttleCount")
        if isinstance(throttle, int) and throttle > 0:
            status = "warn" if status == "ok" else status
            warnings.append("Зафиксирован троттлинг процессора: счётчик %d" % throttle)

        used = memory.get("usedPercent")
        if isinstance(used, (int, float)):
            if used >= CRIT_MEMORY_PERCENT:
                status = "critical"
                warnings.append("Занято %.0f %% памяти — система может начать сбрасывать кэш" % used)
            elif used >= WARN_MEMORY_PERCENT:
                status = "warn" if status == "ok" else status
                warnings.append("Занято %.0f %% памяти" % used)

        used_disk = disk.get("usedPercent")
        if isinstance(used_disk, (int, float)):
            if used_disk >= CRIT_DISK_PERCENT:
                status = "critical"
                warnings.append("Диск заполнен на %.0f %% — записи карты и журналов могут не пройти"
                                % used_disk)
            elif used_disk >= WARN_DISK_PERCENT:
                status = "warn" if status == "ok" else status
                warnings.append("Диск заполнен на %.0f %%" % used_disk)

        load1, cores = cpu.get("load1"), cpu.get("cores")
        if isinstance(load1, (int, float)) and isinstance(cores, int) and cores > 0:
            if load1 > cores * 2.0:
                status = "warn" if status == "ok" else status
                warnings.append("Средняя загрузка %.2f при %d ядрах — очередь задач" % (load1, cores))

        return {
            "ok": True,
            "hostname": self._hostname(),
            # Пороги отдаём вместе с числами: экран не повторяет их своей копией.
            "thresholds": {"warnTempC": self.warn_temp_c, "critTempC": self.crit_temp_c,
                           "warnMemoryPercent": WARN_MEMORY_PERCENT,
                           "critMemoryPercent": CRIT_MEMORY_PERCENT,
                           "warnDiskPercent": WARN_DISK_PERCENT,
                           "critDiskPercent": CRIT_DISK_PERCENT},
            "collectedAt": self.clock(),
            "status": status,
            "warnings": warnings,
            "cpu": cpu,
            "memory": memory,
            "disk": disk,
            "temperature": temp,
            "uptimeS": uptime,
        }


class SystemMonitor:
    """Один кэшированный читатель на процесс, как у инвентаря подключений."""

    def __init__(self, reader: Optional[SystemState] = None,
                 interval: float = DEFAULT_INTERVAL_S, clock: Callable[[], float] = time.monotonic,
                 thread_factory: Optional[Callable[..., threading.Thread]] = None):
        self.reader = reader or SystemState()
        self.interval = max(0.5, float(interval))
        self.clock = clock
        self._thread = thread_factory or threading.Thread
        self._lock = threading.Lock()
        self._data: Optional[dict[str, Any]] = None
        self._refreshing = False
        self._last_attempt = float("-inf")
        self._error: Optional[str] = None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            if not self._refreshing and self.clock() - self._last_attempt >= self.interval:
                self._refreshing = True
                self._last_attempt = self.clock()
                self._thread(target=self.refresh, name="system-state", daemon=True).start()
            data = copy.deepcopy(self._data)
            error = self._error
            refreshing = self._refreshing
        if data is not None:
            age = time.time() - float(data.get("collectedAt") or 0.0)
            data["ageS"] = round(max(0.0, age), 1)
            data["stale"] = age > STALE_AFTER_S
        return {"ok": True, "data": data, "refreshing": refreshing, "error": error}

    def refresh(self) -> None:
        try:
            data = self.reader.collect()
            with self._lock:
                self._data = data
                self._error = None
        except Exception:  # noqa: BLE001 — экран живёт и без свежих датчиков
            with self._lock:
                self._error = "Не удалось прочитать состояние компьютера"
        finally:
            with self._lock:
                self._refreshing = False
