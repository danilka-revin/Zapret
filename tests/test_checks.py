"""
Тесты проверок сервисов и подсчёта трафика (без сети и без Qt).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zapret import checks  # noqa: E402


def test_probe_url_bad_host_returns_zero():
    code, ms = checks.probe_url("http://127.0.0.1:9/", timeout=1.0)
    assert code == 0
    assert ms >= 0


def test_probe_service_ok(monkeypatch):
    service = checks.SERVICES_BY_KEY["youtube"]
    monkeypatch.setattr(checks, "probe_url", lambda url, timeout=6.0: (204, 42.0))
    result = checks.probe_service(service, timeout=1.0)
    assert result["state"] == "ok"
    assert result["latency_ms"] == 42
    assert "работает" in result["detail"]


def test_probe_service_slow(monkeypatch):
    service = checks.SERVICES_BY_KEY["discord"]
    monkeypatch.setattr(checks, "probe_url", lambda url, timeout=6.0: (200, 1500.0))
    result = checks.probe_service(service, timeout=1.0)
    assert result["state"] == "warn"
    assert "медленно" in result["detail"]


def test_probe_service_blocked(monkeypatch):
    service = checks.SERVICES_BY_KEY["telegram"]
    monkeypatch.setattr(checks, "probe_url", lambda url, timeout=6.0: (0, 6000.0))
    result = checks.probe_service(service, timeout=1.0)
    assert result["state"] == "bad"
    assert result["code"] == 0
    assert "блокировку" in result["detail"]


def test_probe_service_accepts_auth_codes(monkeypatch):
    """403/401 — сервер ответил, значит обход работает."""
    service = checks.SERVICES_BY_KEY["discord"]
    monkeypatch.setattr(checks, "probe_url", lambda url, timeout=6.0: (403, 120.0))
    assert checks.probe_service(service, timeout=1.0)["state"] == "ok"


def test_score_counts_ok_services(monkeypatch):
    results = [
        {"state": "ok", "latency_ms": 100},
        {"state": "ok", "latency_ms": 200},
        {"state": "bad", "latency_ms": 5000},
    ]
    ok, avg = checks.score(results)
    assert ok == 2
    assert avg == 150.0


def test_read_iface_counters_from_proc(monkeypatch):
    """Разбор /proc/net/dev без обращения к реальной системе."""
    fake = (
        "Inter-|   Receive                                                |  Transmit\n"
        " face |bytes    packets errs drop fifo frame compressed multicast|bytes    packets\n"
        "  eth0: 1000 5 0 0 0 0 0 0 2000 7 0 0 0 0 0 0\n"
        "wlan0:  300 1 0 0 0 0 0 0  400 2 0 0 0 0 0 0\n"
    )
    original = checks.Path.read_text
    monkeypatch.setattr(
        checks.Path, "read_text",
        lambda self, *a, **k: fake if str(self).endswith("net/dev")
        else original(self, *a, **k),
    )
    counters = checks.read_iface_counters()
    assert counters["eth0"] == (1000, 2000)
    assert counters["wlan0"] == (300, 400)


def test_traffic_monitor_skips_first_and_counts_delta(monkeypatch):
    values = [1000, 1000, 2_000_000]
    state = {"i": 0}

    def fake_counters():
        value = values[min(state["i"], len(values) - 1)]
        state["i"] += 1
        return {"eth0": (value, value // 2)}

    monkeypatch.setattr(checks, "read_iface_counters", fake_counters)
    monkeypatch.setattr(checks, "default_route_iface", lambda: "eth0")
    monitor = checks.TrafficMonitor("eth0")
    first = monitor.sample()
    assert first["rx_mbps"] == 0.0        # первый замер — только база
    monitor.sample()
    third = monitor.sample()
    assert third["rx_mbps"] > 0.0
    assert third["total_rx"] > 0
