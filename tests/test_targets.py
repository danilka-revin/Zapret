"""
Тесты подбора стратегии под конкретные сайты: разбор ввода, проверка доменов,
режим подбора в автопилоте и настройки темы.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import autopilot, checks  # noqa: E402


# ---------------------------------------------------------------------------
# Разбор пользовательского ввода
# ---------------------------------------------------------------------------

def test_clean_host_accepts_any_common_input():
    assert checks.clean_host("rutracker.org") == "rutracker.org"
    assert checks.clean_host("https://rutracker.org/forum/index.php") == "rutracker.org"
    assert checks.clean_host("http://www.example.com:8080/path?x=1") == "www.example.com"
    assert checks.clean_host("  HTTPS://Site.RU  ") == "site.ru"
    assert checks.clean_host("user:pass@host.ru") == "host.ru"


def test_clean_host_rejects_garbage():
    assert checks.clean_host("") == ""
    assert checks.clean_host("привет") == ""
    assert checks.clean_host("не сайт") == ""
    assert checks.clean_host("http://") == ""


def test_parse_targets_handles_lists_and_groups():
    hosts = checks.parse_targets("rutracker.org, https://youtube.com/watch?v=1")
    assert hosts == ["rutracker.org", "youtube.com"]

    group = checks.parse_targets("youtube")
    assert "googlevideo.com" in group
    assert len(group) == len(checks.SITE_GROUPS_BY_KEY["youtube"][2])

    mixed = checks.parse_targets("ai rutracker.org")
    assert "chatgpt.com" in mixed and "rutracker.org" in mixed


def test_parse_targets_deduplicates_and_skips_junk():
    hosts = checks.parse_targets("site.ru site.ru мусор !!!")
    assert hosts == ["site.ru"]


# ---------------------------------------------------------------------------
# Проверка конкретных доменов
# ---------------------------------------------------------------------------

def test_probe_host_marks_fast_site_ok(monkeypatch):
    monkeypatch.setattr(checks, "_probe_full", lambda url, timeout: (200, 180.0, 90.0))
    result = checks.probe_host("example.com", timeout=1.0)
    assert result["state"] == "ok"
    assert result["latency_ms"] == 180
    assert "открывается" in result["detail"]


def test_probe_host_detects_slow_tls(monkeypatch):
    """Замедление DPI проявляется именно в долгом TLS-рукопожатии."""
    monkeypatch.setattr(checks, "_probe_full", lambda url, timeout: (200, 900.0, 850.0))
    result = checks.probe_host("youtube.com", timeout=1.0)
    assert result["state"] == "warn"
    assert "TLS" in result["detail"]


def test_probe_host_marks_blocked_site(monkeypatch):
    monkeypatch.setattr(checks, "_probe_full", lambda url, timeout: (0, 6000.0, 0.0))
    result = checks.probe_host("rutracker.org", timeout=0.2)
    assert result["state"] == "bad"
    assert result["code"] == 0
    assert "заблокирован" in result["detail"]


def test_probe_hosts_returns_row_per_site(monkeypatch):
    monkeypatch.setattr(checks, "_probe_full", lambda url, timeout: (200, 200.0, 100.0))
    results = checks.probe_hosts(["a.ru", "b.ru", "c.ru"], timeout=1.0)
    assert [r["host"] for r in results] == ["a.ru", "b.ru", "c.ru"]


# ---------------------------------------------------------------------------
# Режим подбора в автопилоте
# ---------------------------------------------------------------------------

def _prepare(monkeypatch, results_by_strategy):
    monkeypatch.setattr(autopilot, "candidate_strategies",
                        lambda cfg, limit=6: list(results_by_strategy))
    monkeypatch.setattr(autopilot.checks, "internet_available", lambda timeout=4.0: True)
    monkeypatch.setattr(autopilot.core, "deps_ready", lambda: True)
    monkeypatch.setattr(autopilot.core, "run_zapret", lambda cfg=None: None)
    saved = {}

    def fake_evaluate(cfg, strategy, timeout=5.0, prober=None):
        rows = prober() if prober else []
        ok = sum(1 for r in rows if r["state"] == "ok")
        avg = sum(r["latency_ms"] for r in rows) / max(1, len(rows))
        return {"strategy": strategy, "results": rows, "ok": ok, "avg_ms": avg}

    monkeypatch.setattr(autopilot, "evaluate", fake_evaluate)
    monkeypatch.setattr(autopilot.config_mod, "save", lambda cfg: saved.update(cfg))
    return saved


def _host_results(monkeypatch, quality: dict):
    """quality: {стратегия: задержка} — меньше = лучше."""
    current = {"strategy": ""}

    def fake_probe_hosts(hosts, timeout=6.0):
        delay = quality.get(current["strategy"], 5000.0)
        state = "ok" if delay < 500 else "bad"
        return [{"host": host, "state": state, "latency_ms": delay,
                 "detail": "тест", "code": 200 if state == "ok" else 0,
                 "title": host} for host in hosts]

    monkeypatch.setattr(autopilot.checks, "probe_hosts", fake_probe_hosts)
    original_eval = autopilot.evaluate

    def evaluate(cfg, strategy, timeout=5.0, prober=None):
        current["strategy"] = strategy
        return original_eval(cfg, strategy, timeout, prober)

    return evaluate


def test_autopilot_picks_strategy_for_specific_site(monkeypatch):
    saved = _prepare(monkeypatch, {"general.bat": 0, "general_alt2.bat": 0})
    monkeypatch.setattr(autopilot, "evaluate", _host_results(
        monkeypatch, {"general.bat": 1500.0, "general_alt2.bat": 120.0}))
    cfg = {"strategy": "general.bat"}
    report = autopilot.run_for_targets(cfg, ["rutracker.org"], timeout=1.0,
                                       limit=2, apply_best=True)
    assert report["strategy"] == "general_alt2.bat"
    assert report["applied"] is True
    assert report["what"] == "rutracker.org"
    assert cfg["strategy"] == "general_alt2.bat"
    assert saved["strategy"] == "general_alt2.bat"


def test_autopilot_measure_only_does_not_change_config(monkeypatch):
    # Первая стратегия отвечает медленнее «идеального» порога, иначе перебор
    # остановится на ней досрочно и до второй дело не дойдёт.
    _prepare(monkeypatch, {"general.bat": 0, "general_alt2.bat": 0})
    monkeypatch.setattr(autopilot, "evaluate", _host_results(
        monkeypatch, {"general.bat": 480.0, "general_alt2.bat": 100.0}))
    cfg = {"strategy": "general.bat"}
    report = autopilot.run_for_targets(cfg, ["youtube.com"], timeout=1.0, limit=2,
                                       apply_best=False)
    assert report["strategy"] == "general_alt2.bat"
    assert report["applied"] is False
    assert cfg["strategy"] == "general.bat"          # конфиг не тронут


def test_autopilot_reports_percent_for_multiple_sites(monkeypatch):
    _prepare(monkeypatch, {"only.bat": 0})
    monkeypatch.setattr(autopilot, "evaluate", _host_results(
        monkeypatch, {"only.bat": 100.0}))
    steps = []
    report = autopilot.run_for_targets({"strategy": "only.bat"},
                                       ["a.ru", "b.ru", "c.ru"], timeout=1.0, limit=1,
                                       on_step=lambda i, t, n: steps.append((i, t, n)))
    assert report["total"] == 3
    assert steps == [(1, 1, "only.bat")]


def test_saved_strategy_is_checked_first(monkeypatch):
    """Пресет проверяется своей же стратегией — она идёт первой в переборе."""
    _prepare(monkeypatch, {"general.bat": 0, "general_alt2.bat": 0})
    monkeypatch.setattr(autopilot, "evaluate", _host_results(
        monkeypatch, {"general.bat": 100.0, "general_alt2.bat": 450.0}))
    seen: list[str] = []
    original = autopilot.evaluate

    def spy(cfg, strategy, timeout=5.0, prober=None):
        seen.append(strategy)
        return original(cfg, strategy, timeout, prober)

    monkeypatch.setattr(autopilot, "evaluate", spy)
    report = autopilot.run_for_targets({"strategy": "general.bat"}, ["a.ru"], timeout=1.0,
                                       limit=2, apply_best=False,
                                       prefer="general_alt2.bat")
    assert seen[0] == "general_alt2.bat"          # сохранённая — первой
    assert report["strategy"] == "general.bat"    # но выбрана лучшая по факту


def test_working_saved_strategy_stops_the_search(monkeypatch):
    _prepare(monkeypatch, {"general.bat": 0, "general_alt2.bat": 0})
    monkeypatch.setattr(autopilot, "evaluate", _host_results(
        monkeypatch, {"general.bat": 900.0, "general_alt2.bat": 120.0}))
    seen: list[str] = []
    original = autopilot.evaluate

    def spy(cfg, strategy, timeout=5.0, prober=None):
        seen.append(strategy)
        return original(cfg, strategy, timeout, prober)

    monkeypatch.setattr(autopilot, "evaluate", spy)
    report = autopilot.run_for_targets({"strategy": "general.bat"}, ["a.ru"], timeout=1.0,
                                       limit=2, apply_best=False,
                                       prefer="general_alt2.bat")
    assert seen == ["general_alt2.bat"]           # перебор закончился сразу
    assert report["strategy"] == "general_alt2.bat"


def test_run_for_targets_requires_hosts():
    try:
        autopilot.run_for_targets({}, ["мусор"])
    except RuntimeError:
        pass
    else:  # pragma: no cover
        raise AssertionError("ожидали ошибку про отсутствие сайтов")


# ---------------------------------------------------------------------------
# Тема: полностью тёмная / полностью светлая
# ---------------------------------------------------------------------------

def test_qpalette_matches_theme():
    from PySide6.QtGui import QPalette  # noqa: F401

    from zapret.qt.theme import Palette, UISettings, build_qpalette

    dark = Palette(UISettings(mode="dark"))
    light = Palette(UISettings(mode="light"))
    dark_qp = build_qpalette(dark)
    light_qp = build_qpalette(light)

    from PySide6.QtGui import QPalette as QP
    dark_window = dark_qp.color(QP.ColorRole.Window)
    light_window = light_qp.color(QP.ColorRole.Window)
    assert dark_window.lightness() < 60          # тёмное окно, без светлых полос
    assert light_window.lightness() > 200
    assert dark_qp.color(QP.ColorRole.Base).lightness() < 80
    assert light_qp.color(QP.ColorRole.Base).lightness() > 200


def test_style_sheet_has_no_light_backgrounds_in_dark_theme():
    from zapret.qt.theme import Palette, UISettings, style_sheet

    qss = style_sheet(Palette(UISettings(mode="dark")))
    assert "transparent" in qss
    assert "#efefef" not in qss.lower()
    assert "QToolTip" in qss and "QMenu" in qss
