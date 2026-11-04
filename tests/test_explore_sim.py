"""Сухой прогон автоцикла: карта должна расти без лидара и шасси."""
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("explore_sim", ROOT / "scripts" / "explore_sim.py")
sim = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(sim)


def test_simulation_grows_map_and_records_landmarks():
    report = sim.run_simulation(steps=4)
    assert report["ok"] is True, report
    assert report["steps"] == 4
    assert report["knownCellsAfter"] > report["knownCellsBefore"]
    assert report["coverageAfterPct"] > report["coverageBeforePct"]
    assert len(report["goals"]) == report["steps"]
    assert report["mapSaves"] >= report["steps"]
    assert report["landmarks"] >= report["steps"]


def test_simulation_cli_json_output():
    assert sim.main(["--steps", "2", "--json"]) == 0
