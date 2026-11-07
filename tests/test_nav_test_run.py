"""Тест A→B не должен двигать робота без явного подтверждения оператора.

Скрипт ``scripts/nav_test_run.py`` — тонкая обёртка над бортовым API, поэтому
здесь подменяется только ``call``: проверяется порядок и набор запросов, а
исполнитель навигации уже покрыт ``tests/test_navigation_executor.py``.
"""
import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "nav_test_run.py"
spec = importlib.util.spec_from_file_location("nav_test_run_cli", SCRIPT)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def ready_status(**overrides):
    checks = {key: True for key in runner.REQUIRED_CHECKS}
    checks.update(overrides)
    return {
        "ok": True, "phase": "idle", "progress": 0.0,
        "diagnostics": {"checks": checks, "mapAgeS": 0.2, "poseQuality": 0.95,
                        "scanPoints": 360},
        "reason": "",
    }


class Api:
    """Журнал запросов вместо бортового сервера."""

    def __init__(self, status=None, plan=None, start_error=None):
        self.status = status or ready_status()
        self.plan = plan if plan is not None else {
            "ok": True, "plan": {"path": [{"x": 0, "y": 0}, {"x": 2, "y": 0}],
                                 "distanceM": 2.0, "waypoints": 1,
                                 "manualBlindZone": True,
                                 "blindInspectionRadiusM": 0.48}}
        self.start_error = start_error
        self.calls = []
        self.status_reads = 0

    def __call__(self, base, path, body=None, *, headers=None, timeout=6.0):
        self.calls.append((path, body, headers))
        if path == "/api/navigation/status":
            self.status_reads += 1
            # Первый опрос — предполётный, затем поездка завершилась успехом.
            if self.status_reads > 1:
                done = dict(self.status, phase="succeeded", progress=1.0)
                self.status = done
            return self.status
        if path == "/api/kiosk_map":
            return {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}}
        if path == "/api/navigation/plan":
            return self.plan
        if path == "/api/navigation/start":
            if self.start_error:
                raise self.start_error
            return {"ok": True, "status": {"message": "Запуск"}}
        if path == "/api/navigation/cancel":
            return {"ok": True}
        raise AssertionError(path)

    def paths(self):
        return [path for path, _body, _headers in self.calls]


def install(monkeypatch, api):
    monkeypatch.setattr(runner, "call", api)
    return api


def test_go_without_yes_never_touches_the_robot(monkeypatch):
    api = install(monkeypatch, Api())
    assert runner.main(["--goal", "2,0", "--go"]) == 2
    assert api.calls == []


def test_read_only_run_prints_checks_and_sends_no_motion(monkeypatch, capsys):
    api = install(monkeypatch, Api())
    assert runner.main([]) == 0
    assert api.paths() == ["/api/navigation/status", "/api/kiosk_map"]
    assert "Готовность навигации" in capsys.readouterr().out


def test_plan_only_does_not_start(monkeypatch, capsys):
    api = install(monkeypatch, Api())
    assert runner.main(["--goal", "2,0"]) == 0
    assert "/api/navigation/start" not in api.paths()
    assert "--go --yes" in capsys.readouterr().out


def test_red_check_blocks_the_start(monkeypatch):
    api = install(monkeypatch, Api(status=ready_status(enabled=False)))
    assert runner.main(["--goal", "2,0", "--go", "--yes", "--blind-zone-checked"]) == 1
    assert "/api/navigation/start" not in api.paths()


def test_start_requires_blind_zone_confirmation_and_motion_header(monkeypatch):
    api = install(monkeypatch, Api())
    assert runner.main(["--goal", "2,0", "--go", "--yes"]) == 0
    starts = [(body, headers) for path, body, headers in api.calls
              if path == "/api/navigation/start"]
    assert len(starts) == 1
    body, headers = starts[0]
    assert body == {"goal": {"x": 2.0, "y": 0.0}, "blindZoneChecked": False}
    assert headers == {"X-Robot-Control": "navigation"}


def test_successful_ride_reports_goal_reached(monkeypatch, capsys):
    install(monkeypatch, Api())
    code = runner.main(["--goal", "2,0", "--go", "--yes", "--blind-zone-checked",
                        "--poll", "0.01"])
    out = capsys.readouterr().out
    assert code == 0 and "Цель достигнута" in out


def test_plan_changed_is_not_retried_automatically(monkeypatch, capsys):
    error = runner.ApiError("Карта изменилась", 409, {
        "error": "plan_changed",
        "details": {"plan": {"path": [{"x": 0, "y": 0}, {"x": 1, "y": 1}],
                             "distanceM": 1.4, "waypoints": 1}}})
    api = install(monkeypatch, Api(start_error=error))
    assert runner.main(["--goal", "2,0", "--go", "--yes", "--blind-zone-checked"]) == 1
    assert api.paths().count("/api/navigation/start") == 1
    assert "запустите команду ещё раз" in capsys.readouterr().out


def test_unreachable_board_api_fails_closed(monkeypatch):
    def offline(*_args, **_kwargs):
        raise runner.ApiError("connection refused", 0)

    monkeypatch.setattr(runner, "call", offline)
    assert runner.main(["--goal", "2,0", "--go", "--yes"]) == 3
