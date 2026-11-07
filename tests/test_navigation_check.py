"""Read-only onboard diagnostic must fail closed on missing API checks."""
import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "navigation_check.py"
spec = importlib.util.spec_from_file_location("navigation_check_cli", SCRIPT)
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def payloads():
    return {
        "/api/lidar": {"ok": True, "connected": True, "points_count": 360, "source": "serial:ttyUSB0", "age_s": 0.1},
        "/api/kiosk_map": {"ok": True, "scans": 10, "quality": 1.0},
        "/api/navigation/status": {"ok": True, "enabled": True, "canStart": False, "reason": "",
                                   "diagnostics": {"checks": {k: True for k in check.REQUIRED_CHECKS}}},
    }


def test_complete_read_only_report_does_not_require_a_plan(monkeypatch, capsys):
    responses = payloads()
    calls = []

    def get(url):
        calls.append(url)
        return responses[url.removeprefix("http://robot")]

    monkeypatch.setattr(check, "fetch", get)
    assert check.main(["--url", "http://robot"]) == 0
    assert len(calls) == 3
    assert "физические E-STOP" in capsys.readouterr().out


def test_failure_or_missing_check_never_reports_ready(monkeypatch):
    responses = payloads()
    del responses["/api/navigation/status"]["diagnostics"]["checks"]["mapIntegrity"]
    monkeypatch.setattr(check, "fetch", lambda url: responses[url.removeprefix("http://robot")])
    report = check.inspect("http://robot")
    assert not report["ok"] and "mapIntegrity" in report["missingChecks"]
    assert check.main(["--url", "http://robot", "--json"]) == 1
    responses["/api/lidar"] = {"error": "offline"}
    assert check.main(["--url", "http://robot", "--json"]) == 2


def test_report_prints_gap_and_speed_when_present(monkeypatch, capsys):
    responses = payloads()
    details = responses['/api/navigation/status']['diagnostics']
    details.update(scanPoints=287, scanMaxGapDeg=33.0, scanMaxGapCenterDeg=-128.5,
                   scanGapLimitDeg=30.0, speedMpsRequested=0.05,
                   speedPercentRequested=None, speedMaxPercent=4)
    details['checks']['scanReady'] = False
    monkeypatch.setattr(check, 'fetch', lambda url: responses[url.removeprefix('http://robot')])
    assert check.main(['--url', 'http://robot']) == 1
    output = capsys.readouterr().out
    assert '33.0°' in output and '-128.5°' in output
    assert 'нет калибровки' in output and 'предел 4%' in output
