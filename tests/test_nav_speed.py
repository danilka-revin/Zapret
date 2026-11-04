"""Помощник замера RC_NAV_MPS_PER_PERCENT и запись в .env."""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "measure_nav_speed", ROOT / "scripts" / "measure_nav_speed.py")
measure = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(measure)


@pytest.fixture(autouse=True)
def restore_speed_env():
    """save_geometry намеренно применяет ключ к процессу — не заражаем соседей.

    Чистим os.environ напрямую: monkeypatch.undo() вернул бы записанное
    тестом значение обратно, и соседний тест увидел бы «уже измеренную»
    скорость.
    """
    key = "RC_NAV_MPS_PER_PERCENT"
    before = os.environ.get(key)
    yield
    if before is None:
        os.environ.pop(key, None)
    else:
        os.environ[key] = before


def test_compute_is_speed_per_percent():
    # 2 м за 8 с при t=4% → 0,25 м/с → 0,0625 м/с на 1%.
    assert measure.compute(2.0, 8.0, 4.0) == pytest.approx(0.0625)
    assert measure.compute(1.0, 10.0, 2.0) == pytest.approx(0.05)


def test_compute_rejects_unphysical_measurements():
    for args in ((0, 8, 4), (2, 0, 4), (2, 8, 0), (2, 8, 200)):
        with pytest.raises(ValueError):
            measure.compute(*args)


def test_save_env_writes_measured_speed(tmp_path, monkeypatch):
    from robot_control import calibration

    env = tmp_path / ".env"
    monkeypatch.setattr(calibration, "env_file_path", lambda: env)
    report = measure.run(2.0, 8.0, 4.0, save_env=True)
    assert report["saved"] is True
    assert "RC_NAV_MPS_PER_PERCENT=0.0625" in env.read_text(encoding="utf-8")


def test_cli_json():
    assert measure.main(["--distance", "2", "--seconds", "8",
                         "--percent", "4", "--json"]) == 0
    assert measure.main(["--distance", "0", "--seconds", "8",
                         "--percent", "4", "--json"]) == 2
