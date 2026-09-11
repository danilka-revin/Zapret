"""
Автопилот: сам подбирает рабочую стратегию обхода.

Идея простая — «минимум настроек, максимум автономики». Приложение по очереди
включает стратегии из набора Flowseal, проверяет YouTube, Discord и Telegram и
оставляет ту, которая даёт лучший результат. Пользователю не нужно ничего
выбирать вручную.
"""

from __future__ import annotations

import re
from typing import Callable

from . import checks, config as config_mod, core

MAX_CANDIDATES = 6
PROBE_TIMEOUT = 5.0
IDEAL_LATENCY_MS = 350.0


def _natural_key(name: str):
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", name)]


def candidate_strategies(cfg: dict, limit: int = MAX_CANDIDATES) -> list[str]:
    """Список стратегий в порядке проверки: текущая, general, general_alt*…"""
    available = core.list_strategies()
    if not available:
        return []
    ordered: list[str] = []

    def add(name: str):
        if name and name in available and name not in ordered:
            ordered.append(name)

    current = cfg.get("strategy", "general.bat")
    add(current)
    add("general.bat")
    for prefix in ("general_alt", "general_simple_fake", "general_simple", "general_"):
        for name in sorted(available, key=_natural_key):
            if name.startswith(prefix):
                add(name)
    for name in sorted(available, key=_natural_key):
        if name.startswith("general"):
            add(name)
    return ordered[:limit]


def evaluate(cfg: dict, strategy: str, timeout: float = PROBE_TIMEOUT) -> dict:
    """Включает стратегию и измеряет результат по трём сервисам."""
    trial = dict(cfg)
    trial["strategy"] = strategy
    core.run_zapret(trial)
    results = checks.probe_all(timeout)
    ok, avg = checks.score(results)
    return {"strategy": strategy, "results": results, "ok": ok, "avg_ms": avg}


def run(cfg: dict,
        progress_cb: Callable[[str], None] | None = None,
        stop_flag: Callable[[], bool] | None = None,
        timeout: float = PROBE_TIMEOUT,
        limit: int = MAX_CANDIDATES) -> dict:
    """
    Перебирает стратегии и оставляет лучшую.

    Возвращает отчёт: выбранная стратегия, результаты по каждой попытке,
    сохранён ли выбор в конфиг.
    """
    log = progress_cb or (lambda _msg: None)

    if not core.deps_ready():
        raise RuntimeError("Сначала скачайте зависимости — без nfqws обход не работает.")

    if not checks.internet_available():
        raise RuntimeError("Нет доступа в интернет. Проверьте подключение и повторите.")

    names = candidate_strategies(cfg, limit)
    if not names:
        raise RuntimeError("Не найдено ни одной стратегии. Обновите зависимости.")

    log(f"Автопилот: проверяю {len(names)} стратегий, это займёт около "
        f"{int(len(names) * (timeout + 3))} с.")

    report: list[dict] = []
    best: dict | None = None
    total = len(names)
    last_error: str | None = None
    repeated_errors = 0
    for index, name in enumerate(names, start=1):
        if stop_flag and stop_flag():
            log("Автоподбор остановлен пользователем.")
            break
        log(f"[{index}/{total}] стратегия {name}…")
        try:
            result = evaluate(cfg, name, timeout)
        except Exception as exc:  # noqa: BLE001 — одна стратегия не должна ломать подбор
            message = str(exc)
            log(f"[{index}/{total}] {name}: ошибка — {message}")
            report.append({"strategy": name, "ok": 0, "avg_ms": 0.0, "error": message})
            # Одинаковая ошибка на всех стратегиях означает проблему не в
            # стратегии, а в системе: нет nftables, нет прав, нет сети.
            repeated_errors = repeated_errors + 1 if message == last_error else 1
            last_error = message
            if repeated_errors >= 2:
                log("Ошибка повторяется на всех стратегиях — перебор остановлен.")
                raise RuntimeError(f"{message} (повторяется для всех стратегий)") from exc
            continue
        report.append({k: v for k, v in result.items() if k != "results"})
        detail = " · ".join(
            f"{r['title']}: {'ок' if r['state'] == 'ok' else ('медленно' if r['state'] == 'warn' else 'нет')}"
            f" {r['latency_ms']} мс" for r in result["results"]
        )
        log(f"[{index}/{total}] {name}: работает {result['ok']}/3, в среднем "
            f"{result['avg_ms']:.0f} мс — {detail}")

        if best is None or (result["ok"], -result["avg_ms"]) > (best["ok"], -best["avg_ms"]):
            best = result
        if result["ok"] == len(checks.SERVICES) and result["avg_ms"] < IDEAL_LATENCY_MS:
            log("Найдена отличная стратегия — прекращаю перебор.")
            break
    if best is None:
        raise RuntimeError(
            "Ни одна стратегия не дала результата. Проверьте права sudo и соединение."
        )

    cfg["strategy"] = best["strategy"]
    config_mod.save(cfg)
    log(f"Автопилот выбрал {best['strategy']} "
        f"(сервисов ок: {best['ok']}/3, средняя задержка {best['avg_ms']:.0f} мс).")

    # Возвращаем систему в стабильное состояние с лучшей стратегией
    try:
        core.run_zapret(cfg)
    except Exception as exc:  # noqa: BLE001
        log(f"Не удалось применить выбранную стратегию: {exc}")

    return {
        "strategy": best["strategy"],
        "ok": best["ok"],
        "avg_ms": best["avg_ms"],
        "tries": report,
        "results": best["results"],
    }
