"""Мастер калибровки: валидация, атомарная запись .env и API на борту/пульте."""
from __future__ import annotations

import json
import os
from urllib.parse import urlparse

import pytest
from conftest import login

from robot_control import calibration, map_peer, sensor_layout


class Reply:
    def __init__(self, payload, code=200):
        self.raw = json.dumps(payload).encode("utf-8")
        self.code = code
        self.status = code

    def read(self, limit=None):
        return self.raw

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


@pytest.fixture
def clean_geometry():
    """Изолировать .env-переменные геометрии: save_geometry пишет в os.environ."""
    prefixes = ("RC_CAMERA_", "RC_KINECT_")
    saved = {key: value for key, value in os.environ.items() if key.startswith(prefixes)}
    for key in saved:
        os.environ.pop(key, None)
    try:
        yield
    finally:
        for key in [key for key in os.environ if key.startswith(prefixes)]:
            os.environ.pop(key, None)
        os.environ.update(saved)


def test_payload_describes_all_measured_fields(clean_geometry):
    payload = calibration.geometry_payload()
    assert payload["ok"] is True and payload["calibrated"] is False
    assert len(payload["groups"]) == 6  # Kinect + 4 камеры + тяга/скорость
    assert payload["fieldCount"] == 39  # + RC_NAV_ENABLE с новым тумблером
    keys = {field["key"] for group in payload["groups"] for field in group["fields"]}
    assert "RC_KINECT_HFOV_DEG" in keys and "RC_CAMERA_RR_YAW_DEG" in keys
    assert "RC_NAV_MPS_PER_PERCENT" in keys
    assert keys == set(calibration.ALLOWED_KEYS)
    drive = payload["groups"][-1]
    assert drive["id"] == "drive"
    # Первое поле — тумблер автотяги, второе — обязательный замер скорости.
    assert drive["fields"][0]["key"] == "RC_NAV_ENABLE"
    speed = drive["fields"][1]
    assert speed["value"] is None and speed["measured"] is True, \
        "скорость нельзя подставлять по умолчанию — её измеряют"


def test_validate_rejects_unknown_out_of_range_and_bool(clean_geometry):
    assert calibration.validate({"RC_KINECT_HFOV_DEG": 57})["RC_KINECT_HFOV_DEG"] == "57"
    assert calibration.validate({"RC_KINECT_SCAN_COLUMNS": 80.6})["RC_KINECT_SCAN_COLUMNS"] == "81"
    for bad in ({"RC_SECRET_KEY": "x"}, {"RC_KINECT_HFOV_DEG": 999},
                {"RC_KINECT_HFOV_DEG": "banana"}, {"RC_KINECT_HFOV_DEG": True},
                {"RC_KINECT_HFOV_DEG": float("nan")},
                {"RC_NAV_MPS_PER_PERCENT": ""}, {}):
        with pytest.raises(calibration.CalibrationError):
            calibration.validate(bad)
    assert calibration.validate({"RC_NAV_MPS_PER_PERCENT": 0.00625}) == {
        "RC_NAV_MPS_PER_PERCENT": "0.00625"}


def test_save_geometry_writes_only_known_keys_and_applies_env(tmp_path, clean_geometry, monkeypatch):
    env = tmp_path / ".env"
    env.write_text("# комментарий\nRC_SECRET_KEY=keep-me\nRC_NAV_ENABLE=1\nRC_CAMERA_FL_YAW_DEG=old\n",
                   encoding="utf-8")
    monkeypatch.setattr(calibration, "env_file_path", lambda: env)
    values = {"RC_KINECT_HFOV_DEG": 57, "RC_CAMERA_FL_YAW_DEG": -30,
              "RC_KINECT_X_M": 0.4, "RC_KINECT_YAW_DEG": 0}
    result = calibration.save_geometry(values)
    assert result["ok"] is True and len(result["savedKeys"]) == 4
    text = env.read_text(encoding="utf-8")
    assert "RC_SECRET_KEY=keep-me" in text and "RC_NAV_ENABLE=1" in text
    assert "# комментарий" in text
    assert "RC_CAMERA_FL_YAW_DEG=-30" in text and "old" not in text
    assert text.count("RC_CAMERA_FL_YAW_DEG=") == 1
    assert os.environ["RC_CAMERA_FL_YAW_DEG"] == "-30"
    assert sensor_layout.camera_specs()[0]["yaw_deg"] == -30.0


def test_calibrated_only_when_every_angle_is_set(tmp_path, clean_geometry, monkeypatch):
    env = tmp_path / ".env"
    monkeypatch.setattr(calibration, "env_file_path", lambda: env)
    # RC_NAV_MPS_PER_PERCENT намеренно приходит пустым: его измеряют на
    # площадке, поэтому в «заполнить всё» он не участвует.
    values = {field["key"]: field["value"]
              for group in calibration.geometry_payload()["groups"]
              for field in group["fields"] if field["value"] is not None}
    result = calibration.save_geometry(values)
    assert result["calibrated"] is True
    assert sensor_layout.geometry_calibrated() is True


def test_kiosk_calibration_api(kiosk_client, tmp_path, clean_geometry, monkeypatch):
    env = tmp_path / ".env"
    monkeypatch.setattr(calibration, "env_file_path", lambda: env)
    page = kiosk_client.get("/api/calibration/geometry")
    assert page.status_code == 200 and page.json["fieldCount"] == 39
    assert kiosk_client.post("/api/calibration/geometry",
                             json={"values": {"RC_NOPE": 1}}).status_code == 400
    reply = kiosk_client.post(
        "/api/calibration/geometry",
        json={"values": {"RC_KINECT_HFOV_DEG": 57, "RC_CAMERA_FL_YAW_DEG": -30}})
    assert reply.status_code == 200 and reply.json["ok"] is True
    assert "RC_KINECT_HFOV_DEG=57" in env.read_text(encoding="utf-8")


def test_web_calibration_proxy_requires_csrf(client, operator, kiosk_client, monkeypatch,
                                             tmp_path, clean_geometry):
    login(client)
    env = tmp_path / ".env"
    monkeypatch.setattr(calibration, "env_file_path", lambda: env)

    def forward(req, timeout):
        path = urlparse(req.full_url).path
        marker = req.headers.get("X-robot-control")
        headers = {"X-Robot-Control": marker} if marker else {}
        response = kiosk_client.open(path, method=req.method, data=req.data,
                                     content_type="application/json", headers=headers)
        return Reply(response.json, response.status_code)

    monkeypatch.setattr(map_peer, "urlopen", forward)
    token = client.get("/api/exploration/status").json["csrfToken"]
    page = client.get("/api/calibration/geometry")
    assert page.status_code == 200 and page.json["fieldCount"] == 39
    assert page.json["csrfToken"] == token
    payload = {"values": {"RC_KINECT_HFOV_DEG": 57}}
    assert client.post("/api/calibration/geometry", json=payload).status_code == 400
    assert client.post("/api/calibration/geometry", json=payload,
                       headers={"X-Robot-Control": "calibration"}).status_code == 400
    saved = client.post("/api/calibration/geometry", json=payload,
                        headers={"X-Robot-Control": "calibration", "X-CSRF-Token": token})
    assert saved.status_code == 200 and saved.json["ok"] is True
    assert "RC_KINECT_HFOV_DEG=57" in env.read_text(encoding="utf-8")


def test_status_exposes_preflight_blockers(kiosk_client):
    status = kiosk_client.get("/api/exploration/status").json
    assert status["canStart"] is False
    assert any("RC_EXPLORE_ENABLE" in reason for reason in status["blockers"])
    assert any("RC_NAV_ENABLE" in reason for reason in status["blockers"])
    assert any("Kinect" in reason for reason in status["blockers"])
    assert any("Калибровка" in reason or "геометрия" in reason.lower()
               for reason in status["warnings"])


# --------------------------------------------------------------------------- #
# Автотяга из формы: включение без перезапуска борта
# --------------------------------------------------------------------------- #
def test_drive_form_exposes_the_autopilot_switch_and_accepts_it(tmp_path, monkeypatch):
    """Форма должна уметь включить автотягу — иначе «Поехать» не оживёт."""
    monkeypatch.delenv("RC_NAV_ENABLE", raising=False)
    payload = calibration.geometry_payload()
    drive = next(group for group in payload["groups"] if group["id"] == "drive")
    switch = drive["fields"][0]
    assert switch["key"] == "RC_NAV_ENABLE" and switch["checkbox"] is True
    assert switch["value"] == 0          # по умолчанию выключено

    target = tmp_path / ".env"
    calibration.save_geometry({"RC_NAV_ENABLE": 1}, env_path=target)
    assert "RC_NAV_ENABLE=1" in target.read_text(encoding="utf-8")
    assert os.environ["RC_NAV_ENABLE"] == "1"
    # Только 0 или 1: значение вне диапазона не должно попасть в .env.
    with pytest.raises(calibration.CalibrationError):
        calibration.validate({"RC_NAV_ENABLE": 3})


def test_navigation_applies_the_saved_drive_settings_without_restart(monkeypatch):
    """Сохранение формы перечитывает RC_NAV_* в самом сервисе навигации."""
    from robot_control.navigation import NavigationService

    service = NavigationService(None, None, enabled=False, mps_per_percent=None)
    # Без карты status() не может собрать проверки — смотрим сами поля сервиса.
    assert service.enabled is False and service.mps_per_percent is None

    monkeypatch.setenv("RC_NAV_ENABLE", "1")
    monkeypatch.setenv("RC_NAV_MPS_PER_PERCENT", "0.03749")
    applied = service.reload_drive_settings()

    assert applied["applied"] is True
    assert applied["enabled"] is True and applied["speedCalibrated"] is True
    assert service.enabled is True and service.mps_per_percent == pytest.approx(0.03749)
    # 0,05 м/с при 0,03749 м/с на 1% — это 1% тяги, предел 4% не нарушен.
    assert 1 <= int(round(service.speed_mps / service.mps_per_percent)) <= service.max_drive_percent
    # Перечитывание не запускает движение.
    assert service.active is False
