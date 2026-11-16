"""CLI сухого прогона доставки работает без робота и без тяги."""
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("delivery_sim", ROOT / "scripts" / "delivery_sim.py")
sim = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(sim)


def test_delivery_sim_all_scenarios():
    report = sim.run_simulation()
    assert report["ok"] is True, report
    assert report["tractionConnected"] is False
    assert len(report["results"]) == len(sim.EXPECTED)
    assert {item["scenario"] for item in report["results"]} == set(sim.EXPECTED)


def test_delivery_sim_cli_json(capsys):
    import json
    assert sim.main(["--scenario", "clear", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["ok"] is True
    assert payload["tractionConnected"] is False
