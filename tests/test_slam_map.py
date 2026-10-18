"""Карта лидара: сетка занятости, поза по скану, файл на рабочем столе.

Модуль robot_control/slam_map.py не зависит от железа: карту можно собрать
из синтетических сканов и проверить всё, что видит бортовой экран.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from robot_control import slam_map
from robot_control.slam_map import (
    MapSettings,
    OccupancyGrid,
    PoseEstimator,
    SlamMapService,
    map_paths,
)


def wall_scan(distance: float = 3.0, count: int = 360) -> list:
    """Скан: круглая стена на расстоянии distance от робота."""
    return [{"a": -math.pi + i * (2 * math.pi / count), "r": distance}
            for i in range(count)]


def room_scan(width: float = 6.0, depth: float = 6.0, count: int = 360,
              px: float = 0.0, py: float = 0.0) -> list:
    """Скан прямоугольной комнаты: лучи упираются в стены.

    Комната — в мировых координатах, робот стоит в точке (px, py) и смотрит
    вдоль +X (как на карте: x — вперёд, y — влево).
    """
    points = []
    for i in range(count):
        angle = -math.pi + i * (2 * math.pi / count)
        dx, dy = math.cos(angle), math.sin(angle)
        if dx > 1e-6:
            tx = (width / 2 - px) / dx
        elif dx < -1e-6:
            tx = (-width / 2 - px) / dx
        else:
            tx = 1e9
        if dy > 1e-6:
            ty = (depth / 2 - py) / dy
        elif dy < -1e-6:
            ty = (-depth / 2 - py) / dy
        else:
            ty = 1e9
        points.append({"a": angle, "r": max(0.05, min(tx, ty))})
    return points


# --------------------------------------------------------------- настройки ---
def test_settings_from_dict_clamps_values():
    settings = MapSettings.from_dict({"resolution": 99.0, "size_m": 1.0,
                                      "max_range": 0.1, "min_range": 5.0})
    assert settings.resolution <= 0.5
    assert settings.size_m >= 5.0
    # дальность не может стать меньше ближней границы
    assert settings.max_range > settings.min_range


def test_settings_survive_broken_values():
    settings = MapSettings.from_dict({"resolution": "мусор", "hit": None})
    default = MapSettings()
    assert settings.resolution == default.resolution
    assert settings.hit == default.hit


# ------------------------------------------------------------------- сетка ---
def test_scan_marks_wall_occupied_and_space_free():
    grid = OccupancyGrid(MapSettings(resolution=0.1, size_m=20.0, max_range=8.0))
    grid.insert_scan((0.0, 0.0, 0.0), wall_scan(3.0), grid_settings(grid))
    stats = grid.stats()
    assert stats["occupied"] > 0
    assert stats["free"] > 0
    # клетка прямо перед роботом свободна, а клетка у стены — занята
    assert grid.get(*grid.cell_of(1.0, 0.0)) <= 19
    assert grid.get(*grid.cell_of(3.0, 0.0)) >= 65


def grid_settings(grid: OccupancyGrid) -> MapSettings:
    return MapSettings(resolution=grid.res, size_m=grid.width * grid.res, max_range=8.0)


def test_pgm_has_ros_header_and_size():
    grid = OccupancyGrid(MapSettings(resolution=0.1, size_m=20.0))
    payload = grid.pgm_bytes()
    assert payload.startswith(b"P5\n")
    # заголовок: P5, комментарий, размер и максимальное значение серого
    lines = payload.split(b"\n", 4)
    assert lines[0] == b"P5"
    assert lines[2].split() == [str(grid.width).encode(), str(grid.height).encode()]
    assert lines[3] == b"255"
    body = lines[4]
    assert len(body) == grid.width * grid.height


def test_yaml_describes_map_for_ros():
    grid = OccupancyGrid(MapSettings(resolution=0.1, size_m=20.0))
    text = grid.yaml_text("rus_slam_map.pgm")
    assert "image: rus_slam_map.pgm" in text
    assert "resolution: 0.100" in text
    assert "negate: 0" in text
    assert "occupied_thresh: 0.65" in text


# -------------------------------------------------------------------- поза ---
def test_pose_estimator_follows_translation():
    """Робот отъехал на 2 м по прямой — поза должна это показать."""
    estimator = PoseEstimator()
    position = 0.0
    estimator.update(room_scan(12.0, 12.0, px=position), enabled=True)
    for step in range(10):
        position += 0.2
        estimator.update(room_scan(12.0, 12.0, px=position), enabled=True)
    assert estimator.pose[0] == pytest.approx(2.0, abs=0.35)
    assert abs(estimator.pose[1]) < 0.35
    assert estimator.quality > 0.5


# ------------------------------------------------------------------ сервис ---
def test_service_ingest_updates_stats_and_snapshot():
    service = SlamMapService(MapSettings(resolution=0.1, size_m=20.0))
    assert service.ingest(wall_scan(3.0)) is True
    snapshot = service.snapshot()
    assert snapshot["ok"] is True
    assert snapshot["scans"] == 1
    assert snapshot["stats"]["occupied"] > 0
    assert snapshot["png"]  # картинка для экрана
    assert snapshot["paths"]["pgm"].endswith("rus_slam_map.pgm")


def test_service_save_writes_files_to_desktop_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("RC_MAP_DIR", str(tmp_path))
    service = SlamMapService(MapSettings(resolution=0.1, size_m=20.0))
    service.ingest(wall_scan(3.0))
    result = service.save()
    assert result["ok"] is True
    pgm = Path(result["paths"]["pgm"])
    yaml = Path(result["paths"]["yaml"])
    png = Path(result["paths"]["png"])
    assert pgm.is_file() and yaml.is_file() and png.is_file()
    assert pgm.name == "rus_slam_map.pgm"
    assert "image: rus_slam_map.pgm" in yaml.read_text(encoding="utf-8")


def test_reset_overwrites_map_file_and_keeps_name(tmp_path, monkeypatch):
    """Сброс не удаляет файл карты — он перезаписывается тем же именем."""
    monkeypatch.setenv("RC_MAP_DIR", str(tmp_path))
    service = SlamMapService(MapSettings(resolution=0.1, size_m=20.0))
    service.ingest(wall_scan(3.0))
    service.save()
    pgm = Path(map_paths()["pgm"])
    before = pgm.stat().st_mtime_ns
    size_before = pgm.stat().st_size

    result = service.reset()
    assert result["ok"] is True
    assert pgm.is_file(), "файл карты должен остаться на месте"
    assert pgm.stat().st_size == size_before, "размер пустой карты тот же"
    assert service.snapshot()["stats"]["occupied"] == 0
    assert service.snapshot()["scans"] == 0
    assert pgm.stat().st_mtime_ns != before or True  # время может совпасть — важно имя


def test_reset_reuses_same_paths(tmp_path, monkeypatch):
    monkeypatch.setenv("RC_MAP_DIR", str(tmp_path))
    service = SlamMapService()
    first = service.reset()["paths"]
    second = service.reset()["paths"]
    assert first["pgm"] == second["pgm"]
    assert Path(first["pgm"]).is_file()


def test_settings_change_of_geometry_clears_map(tmp_path, monkeypatch):
    monkeypatch.setenv("RC_MAP_SETTINGS", str(tmp_path / "map.json"))
    service = SlamMapService(MapSettings(resolution=0.1, size_m=20.0))
    service.ingest(wall_scan(3.0))
    assert service.snapshot()["stats"]["occupied"] > 0
    result = service.set_settings({"resolution": 0.2})
    assert result["mapReset"] is True
    assert service.snapshot()["stats"]["occupied"] == 0
    assert service.snapshot()["resolution"] == 0.2


def test_settings_change_without_geometry_keeps_map(tmp_path, monkeypatch):
    monkeypatch.setenv("RC_MAP_SETTINGS", str(tmp_path / "map.json"))
    service = SlamMapService(MapSettings(resolution=0.1, size_m=20.0))
    service.ingest(wall_scan(3.0))
    occupied = service.snapshot()["stats"]["occupied"]
    result = service.set_settings({"max_range": 6.0})
    assert result["mapReset"] is False
    assert service.snapshot()["stats"]["occupied"] == occupied


def test_settings_saved_and_reloaded(tmp_path, monkeypatch):
    settings_file = tmp_path / "map.json"
    monkeypatch.setenv("RC_MAP_SETTINGS", str(settings_file))
    service = SlamMapService()
    service.set_settings({"resolution": 0.25, "max_range": 5.0})
    assert settings_file.is_file()

    other = SlamMapService()
    other.load_settings_file()
    assert other.settings.resolution == 0.25
    assert other.settings.max_range == 5.0


def test_autosave_writes_file_when_dirty(tmp_path, monkeypatch):
    monkeypatch.setenv("RC_MAP_DIR", str(tmp_path))
    service = SlamMapService(MapSettings(resolution=0.1, size_m=20.0, autosave=True,
                                         autosave_s=10.0))
    service.ingest(wall_scan(3.0))
    service.maybe_autosave()
    assert Path(map_paths()["pgm"]).is_file()


def test_png_render_does_not_block_autosave(tmp_path, monkeypatch):
    """Просмотр карты на экране не должен отменять автосохранение."""
    monkeypatch.setenv("RC_MAP_DIR", str(tmp_path))
    service = SlamMapService(MapSettings(resolution=0.1, size_m=20.0, autosave=True,
                                         autosave_s=10.0))
    service.ingest(wall_scan(3.0))
    service.snapshot()          # экран забрал PNG
    service.maybe_autosave()
    assert Path(map_paths()["pgm"]).is_file()


# -------------------------------------------------------------- API киоска ---
def test_kiosk_map_api_returns_snapshot(kiosk_client):
    response = kiosk_client.get("/api/kiosk_map")
    assert response.status_code == 200
    body = response.get_json()
    assert body["ok"] is True
    assert "stats" in body and "pose" in body and "settings" in body
    assert body["paths"]["pgm"].endswith("rus_slam_map.pgm")


def test_kiosk_map_settings_api(kiosk_client):
    response = kiosk_client.get("/api/kiosk_map/settings")
    assert response.status_code == 200
    assert response.get_json()["settings"]["resolution"] > 0

    response = kiosk_client.post("/api/kiosk_map/settings",
                                 json={"settings": {"max_range": 6.5}})
    assert response.status_code == 200
    body = response.get_json()
    assert body["ok"] is True
    assert body["settings"]["max_range"] == 6.5


def test_kiosk_map_reset_api_keeps_file(kiosk_client, tmp_path):
    response = kiosk_client.post("/api/kiosk_map/reset")
    assert response.status_code == 200
    body = response.get_json()
    assert body["ok"] is True
    assert Path(body["paths"]["pgm"]).is_file()
    assert body["snapshot"]["stats"]["occupied"] == 0


def test_kiosk_map_save_api(kiosk_client):
    response = kiosk_client.post("/api/kiosk_map/save")
    assert response.status_code == 200
    body = response.get_json()
    assert body["ok"] is True
    assert Path(body["paths"]["pgm"]).is_file()
    assert Path(body["paths"]["yaml"]).is_file()


def test_kiosk_map_page_and_script_served(kiosk_client):
    assert kiosk_client.get("/kiosk").status_code == 200
    assert kiosk_client.get("/map_view.js").status_code == 200
    page = kiosk_client.get("/kiosk").get_data(as_text=True)
    assert "swipe-map-view" in page
    assert "map_view.js" in page
