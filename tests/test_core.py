"""
Тесты ядра: разбор стратегий, порты, автопилот.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import core  # noqa: E402

SAMPLE = """
@echo off
start "zapret" /min "%BIN%winws.exe" --wf-tcp=80,443,%GameFilterTCP% --wf-udp=443,%GameFilterUDP% ^
--filter-tcp=80,443 --hostlist="%LISTS%list-general.txt" --dpi-desync=fake --new ^
--filter-udp=443 --dpi-desync=fake-quic
"""


def test_parse_strategy_replaces_gamefilter_placeholders():
    tcp, udp, blocks = core.parse_strategy_text(SAMPLE, True, True)
    assert "1024-65535" in tcp
    assert "1024-65535" in udp
    assert len(blocks) == 2


def test_parse_strategy_without_gamefilter_keeps_ports_clean():
    tcp, udp, blocks = core.parse_strategy_text(SAMPLE, False, False)
    assert tcp == "80,443"
    assert udp == "443"
    assert all("1024-65535" not in block for block in blocks)


def test_parse_strategy_substitutes_paths():
    _tcp, _udp, blocks = core.parse_strategy_text(SAMPLE, False, False)
    assert "%BIN%" not in " ".join(blocks)
    assert "lists/list-general.txt" in blocks[0]


def test_merge_ports_sorts_and_dedupes():
    merged = core.merge_ports("443,80,1024-65535", "80,50000-50100")
    assert merged.startswith("80,443")
    assert "50000-50100" in merged
    assert merged.count("80") == 1


def test_platform_dir_returns_known_platform():
    value = core.platform_dir()
    assert value.startswith("linux-") or value.startswith("freebsd-")


def test_nfqws_running_ignores_unrelated_processes():
    """Процесс с «nfqws» в аргументах не должен считаться запущенным обходом."""
    import subprocess

    proc = subprocess.Popen(["sleep", "30", "nfqws"], )
    try:
        assert core.nfqws_running() is False
    finally:
        proc.terminate()
        proc.wait(timeout=5)


def test_autopilot_orders_strategies(monkeypatch):
    monkeypatch.setattr(core, "list_strategies", lambda: [
        "general.bat", "general_alt2.bat", "general_alt.bat",
        "general_simple_fake.bat", "discord.bat", "telegram.bat",
    ])
    from zapret import autopilot

    cfg = {"strategy": "general_alt.bat"}
    names = autopilot.candidate_strategies(cfg, limit=4)
    assert names[0] == "general_alt.bat"      # текущая — первой
    assert "general.bat" in names
    assert "discord.bat" not in names
    assert len(names) == 4


def test_autopilot_stops_on_repeated_system_error(monkeypatch):
    from zapret import autopilot

    monkeypatch.setattr(autopilot, "candidate_strategies",
                        lambda cfg, limit=6: ["a.bat", "b.bat", "c.bat"])
    monkeypatch.setattr(autopilot.checks, "internet_available", lambda timeout=4.0: True)
    monkeypatch.setattr(autopilot.core, "deps_ready", lambda: True)

    def boom(cfg, strategy, timeout=5.0, prober=None):
        raise RuntimeError("Не найден nftables или iptables. Установите один из них.")

    monkeypatch.setattr(autopilot, "evaluate", boom)
    messages = []
    try:
        autopilot.run({"strategy": "a.bat"}, progress_cb=messages.append, limit=3)
    except RuntimeError as exc:
        assert "nftables" in str(exc)
    else:  # pragma: no cover — должны были упасть
        raise AssertionError("ожидали RuntimeError")
    assert any("перебор остановлен" in m for m in messages)


def test_autopilot_picks_best_and_saves(monkeypatch, tmp_path):
    from zapret import autopilot

    saved = {}
    monkeypatch.setattr(autopilot, "candidate_strategies",
                        lambda cfg, limit=6: ["weak.bat", "best.bat"])
    monkeypatch.setattr(autopilot.checks, "internet_available", lambda timeout=4.0: True)
    monkeypatch.setattr(autopilot.core, "deps_ready", lambda: True)
    monkeypatch.setattr(autopilot.core, "run_zapret", lambda cfg=None: None)
    monkeypatch.setattr(autopilot.config_mod, "save", lambda cfg: saved.update(cfg))

    def fake_evaluate(cfg, strategy, timeout=5.0, prober=None):
        if strategy == "best.bat":
            return {"strategy": strategy, "results": [], "ok": 3, "avg_ms": 120.0}
        return {"strategy": strategy, "results": [], "ok": 1, "avg_ms": 900.0}

    monkeypatch.setattr(autopilot, "evaluate", fake_evaluate)
    cfg = {"strategy": "weak.bat"}
    report = autopilot.run(cfg, limit=2)
    assert report["strategy"] == "best.bat"
    assert cfg["strategy"] == "best.bat"
    assert saved.get("strategy") == "best.bat"
