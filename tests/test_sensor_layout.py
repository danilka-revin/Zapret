"""Геометрия 4 × C930e + Kinect и настройки портов без железа."""
from __future__ import annotations

import pytest

from robot_control import camera_settings as cs
from robot_control import sensor_layout as layout
from robot_control.camera import CameraManager


def clean_camera_env(monkeypatch):
    for keys in cs.ENV_KEYS_BY_ROLE.values():
        for key in keys:
            monkeypatch.delenv(key, raising=False)
    for keys in layout.CAMERA_ENV_KEYS.values():
        for key in keys:
            monkeypatch.delenv(key, raising=False)
    for key in layout.KINECT_ENV_KEYS.values():
        monkeypatch.delenv(key, raising=False)
    for key in layout.KINECT_REAR_ENV_KEYS.values():
        monkeypatch.delenv(key, raising=False)
    monkeypatch.delenv(layout.REAR_KINECT_ENABLE_KEY, raising=False)


def test_four_cameras_have_stable_ids_roles_and_angles(monkeypatch):
    clean_camera_env(monkeypatch)
    specs = layout.camera_specs()
    assert [s["id"] for s in specs] == [0, 1, 2, 3]
    assert [s["role"] for s in specs] == ["front_left", "rear_left", "front_right", "rear_right"]
    assert all(s["model"] == "Logitech C930e" for s in specs)
    assert specs[0]["port"] == "/dev/video0" and specs[3]["port"] == "/dev/video3"
    assert not layout.geometry_calibrated()
    payload = layout.layout_payload()
    assert payload["cameraCount"] == 4
    assert payload["kinect"]["model"] == "Kinect 360"
    assert payload["duplicatePorts"] == []


def test_env_overrides_geometry_and_calibration(monkeypatch):
    clean_camera_env(monkeypatch)
    monkeypatch.setenv("RC_CAMERA_FL_YAW_DEG", "-37.5")
    monkeypatch.setenv("RC_CAMERA_FR_PORT", "/dev/v4l/by-id/cam-fr-video-index0")
    monkeypatch.setenv("RC_KINECT_X_M", "0.42")
    monkeypatch.setenv("RC_KINECT_YAW_DEG", "12")
    specs = {s["role"]: s for s in layout.camera_specs()}
    assert specs["front_left"]["yaw_deg"] == -37.5
    assert specs["front_right"]["port"] == "/dev/v4l/by-id/cam-fr-video-index0"
    assert layout.kinect_spec()["x_m"] == 0.42
    # Одного-двух замеров мало: калибровка считается законченной только когда
    # заданы углы всех четырёх камер и положение/оптика Kinect.
    assert layout.geometry_calibrated() is False
    for role in layout.CAMERA_ROLES:
        monkeypatch.setenv(layout.camera_env_keys(role)["yaw_deg"], "-37.5")
    monkeypatch.setenv("RC_KINECT_HFOV_DEG", "57")
    assert layout.geometry_calibrated() is True


def test_rear_kinect_is_optional_and_has_own_geometry(monkeypatch):
    clean_camera_env(monkeypatch)
    monkeypatch.delenv("RC_KINECT_REAR_ENABLE", raising=False)
    assert layout.rear_kinect_enabled() is False
    assert list(layout.kinect_specs()) == ["front"]
    payload = layout.layout_payload()
    assert payload["rearKinectEnabled"] is False and list(payload["kinects"]) == ["front"]
    monkeypatch.setenv("RC_KINECT_REAR_ENABLE", "1")
    monkeypatch.setenv("RC_KINECT_REAR_X_M", "-0.42")
    monkeypatch.setenv("RC_KINECT_REAR_YAW_DEG", "175")
    assert layout.rear_kinect_enabled() is True
    specs = layout.kinect_specs()
    assert set(specs) == {"front", "rear"}
    assert specs["rear"]["x_m"] == -0.42 and specs["rear"]["yaw_deg"] == 175.0
    assert specs["front"]["x_m"] == pytest.approx(0.35), "передний не должен меняться"
    assert layout.layout_payload()["rearKinectEnabled"] is True


def test_bad_env_values_are_clamped_not_crashing(monkeypatch):
    clean_camera_env(monkeypatch)
    monkeypatch.setenv("RC_KINECT_HFOV_DEG", "9999")
    monkeypatch.setenv("RC_KINECT_MIN_RANGE_M", "banana")
    spec = layout.kinect_spec()
    assert spec["hfov_deg"] <= 170.0
    assert spec["min_range_m"] == pytest.approx(0.45)


def test_camera_manager_exposes_four_roles(monkeypatch):
    clean_camera_env(monkeypatch)
    CameraManager.reset_instance()
    try:
        cameras = CameraManager.get_instance().list_cameras()
        assert len(cameras) == 4
        assert [c["role"] for c in cameras] == [
            "front_left", "rear_left", "front_right", "rear_right"]
        assert all("yaw_deg" in c and "hfov_deg" in c for c in cameras)
    finally:
        CameraManager.reset_instance()


def test_normalize_four_ports_rejects_duplicates_and_bad_paths():
    values = cs.normalize_camera_settings({
        "front": "/dev/video0", "rear": "/dev/video1",
        "front_right": "/dev/video2", "rear_right": "/dev/v4l/by-id/cam4-video-index0",
    })
    assert values["front_right"] == "/dev/video2"
    assert set(values) == set(cs.SETTING_KEYS)
    with pytest.raises(ValueError):
        cs.normalize_camera_settings({"front": "/dev/video0", "rear": "/dev/video0"})
    with pytest.raises(ValueError):
        cs.normalize_camera_settings({"front": "/etc/passwd"})
    with pytest.raises(ValueError):
        cs.normalize_camera_settings({"front": "/dev/video0; rm -rf /"})


def test_save_camera_settings_writes_four_keys_and_keeps_other_env(tmp_path, monkeypatch):
    clean_camera_env(monkeypatch)
    env_file = tmp_path / ".env"
    env_file.write_text("RC_NAV_ENABLE=1\nRC_CAMERA_FRONT_PORT=/dev/video9\n", encoding="utf-8")
    result = cs.save_camera_settings({
        "front": "/dev/v4l/by-id/fl-video-index0",
        "rear": "/dev/v4l/by-id/rl-video-index0",
        "front_right": "/dev/v4l/by-id/fr-video-index0",
        "rear_right": "/dev/v4l/by-id/rr-video-index0",
    }, env_path=env_file)
    text = env_file.read_text(encoding="utf-8")
    assert "RC_NAV_ENABLE=1" in text
    assert "RC_CAMERA_FRONT_PORT" not in text       # алиас удалён
    for key in cs.ENV_PRIMARY_KEYS.values():
        assert key in text
    assert result["front_right"].endswith("fr-video-index0")


def test_apply_settings_reconfigures_manager_without_hardware(tmp_path, monkeypatch):
    clean_camera_env(monkeypatch)
    CameraManager.reset_instance()
    try:
        result = cs.apply_camera_settings({
            "front": "/dev/video0", "rear": "/dev/video1",
            "front_right": "/dev/video2", "rear_right": "/dev/video3",
        }, env_path=tmp_path / ".env")
        assert result["ok"] is True
        assert len(result["cameras"]) == 4
        assert cs.current_camera_settings()["front_right"] == "/dev/video2"
        payload = cs.settings_payload()
        assert len(payload["cameras"]) == 4
        assert payload["layout"]["calibrated"] is False
    finally:
        CameraManager.reset_instance()
