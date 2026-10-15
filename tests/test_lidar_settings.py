"""Настройки лидара из веб-панели и peer-скан бортового экрана.

Закрывает три жалобы оператора:
  * «настройки лидара только в .env руками» — теперь есть API + сохранение;
  * «на вебе лидар подключён, а на бортовом экране нет» — peer-процесс бросал
    готовый скан и уходил перебирать чужие порты, скан успевал протухнуть;
  * «где кнопка обновить» — цикл обновления всегда переустанавливает
    зависимости и собирает код, а не только при смене requirements.txt.
"""

from __future__ import annotations

import os

import pytest

from robot_control import lidar
from robot_control.lidar import LidarManager
from robot_control.lidar_settings import save_lidar_settings

from conftest import login


@pytest.fixture(autouse=True)
def _preserve_lidar_env():
    """reconfigure() пишет RC_LIDAR_* в os.environ — не даём им утечь в другие тесты.

    Явный снапшот, а не monkeypatch.delenv: тот не записывает отсутствующий
    ключ, и прямая запись в os.environ во время теста пережила бы откат.
    """
    saved = {key: os.environ.get(key) for key in lidar.LIDAR_ENV_KEYS}
    for key in lidar.LIDAR_ENV_KEYS:
        os.environ.pop(key, None)
    yield
    for key, value in saved.items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value


@pytest.fixture
def manager():
    previous = LidarManager._instance
    instance = LidarManager()
    LidarManager._instance = instance
    try:
        yield instance
    finally:
        instance.stop()
        LidarManager._instance = previous


def test_settings_are_validated_and_saved_without_touching_other_env(tmp_path):
    env = tmp_path / ".env"
    env.write_text("RC_SECRET_KEY=secret\nRC_SLAM_PORT=/dev/ttyACM0\n"
                   "RC_LIDAR_PORT=/dev/old\n", encoding="utf-8")
    saved = save_lidar_settings({
        "port": "/dev/serial/by-id/usb-lidar", "protocol": "lds-01",
        "baud": "230400", "motor": "none", "autoprobe": False,
    }, env_path=env)
    assert saved == {"port": "/dev/serial/by-id/usb-lidar", "protocol": "lds01",
                     "baud": "230400", "motor": "none", "autoprobe": False}
    text = env.read_text(encoding="utf-8")
    assert "RC_SECRET_KEY=secret" in text
    assert "RC_SLAM_PORT=/dev/ttyACM0" in text
    assert "RC_LIDAR_PORT=/dev/serial/by-id/usb-lidar" in text
    assert text.count("RC_LIDAR_PORT=") == 1


@pytest.mark.parametrize("payload,field", [
    ({"port": "ttyUSB0"}, "Порт"),
    ({"port": "/dev/ttyUSB0; rm -rf /"}, "Порт"),
    ({"protocol": "unknown-proto"}, "Протокол"),
    ({"baud": "fast"}, "Скорость"),
    ({"motor": "spin"}, "Мотор"),
])
def test_bad_settings_are_refused(payload, field):
    with pytest.raises(ValueError, match=field):
        lidar.normalize_lidar_settings(payload)


def test_manager_reconfigure_changes_search_parameters(manager):
    applied = manager.reconfigure({
        "port": "/dev/ttyUSB9", "protocol": "rplidar", "baud": "256000",
        "motor": "dtr-low", "autoprobe": False,
    })
    assert applied == {"port": "/dev/ttyUSB9", "protocol": "rplidar",
                       "baud": "256000", "motor": "dtr-low", "autoprobe": False}
    assert manager.get_status()["settings"] == applied
    assert manager.get_scan() is None


def test_web_settings_api_requires_login_csrf_and_applies(client, app, operator, monkeypatch, tmp_path):
    manager = LidarManager()
    previous = LidarManager._instance
    LidarManager._instance = manager
    saved = {}
    monkeypatch.setattr(
        "robot_control.app.apply_lidar_settings",
        lambda payload: saved.update(payload) or {
            "ok": True, "settings": manager.reconfigure(payload),
            "status": manager.get_status()})
    try:
        assert client.get("/api/lidar/settings").status_code == 302
        assert login(client).status_code == 302
        page = client.get("/api/lidar/settings")
        assert page.status_code == 200
        body = page.get_json()
        assert body["canEdit"] is True and body["csrfToken"]
        assert client.post("/api/lidar/settings", json={}).status_code == 400
        ok = client.post("/api/lidar/settings", json={
            "port": "", "protocol": "ld06", "baud": "", "motor": "auto",
            "autoprobe": True,
        }, headers={"X-CSRF-Token": body["csrfToken"]})
        assert ok.status_code == 200
        assert ok.get_json()["settings"]["protocol"] == "ld06"
        assert saved["protocol"] == "ld06"
    finally:
        manager.stop()
        LidarManager._instance = previous


def test_kiosk_settings_read_open_but_write_needs_pin(config, monkeypatch):
    from robot_control.kiosk_app import create_kiosk_app
    manager = LidarManager()
    previous = LidarManager._instance
    LidarManager._instance = manager
    monkeypatch.setattr(
        "robot_control.kiosk_app.apply_lidar_settings",
        lambda payload: {"ok": True, "settings": manager.reconfigure(payload),
                         "status": manager.get_status()})
    try:
        client = create_kiosk_app(config).test_client()
        page = client.get("/api/lidar/settings")
        assert page.status_code == 200
        assert page.get_json()["requiresPin"] is True
        payload = {"port": "", "protocol": "auto", "baud": "",
                   "motor": "auto", "autoprobe": True}
        assert client.post("/api/lidar/settings", json=payload).status_code == 401
        ok = client.post("/api/lidar/settings", json={**payload, "pin": "7232"})
        assert ok.status_code == 200
        # На борту подхватились и новые статические файлы панели.
        assert client.get("/lidar_settings.js").status_code == 200
        assert client.get("/lidar_settings.css").status_code == 200
        assert "lidar_settings.js" in client.get("/kiosk").get_data(as_text=True)
    finally:
        manager.stop()
        LidarManager._instance = previous


def test_peer_port_becomes_sticky_so_scan_stays_fresh(manager, monkeypatch):
    """После peer-скана следующий круг начинается с порта соседа.

    Раньше ротация уводила перебор на следующий порт: борт подбирал параметры
    чужого USB дольше секунды свежести скана, и экран показывал «нет скана»
    при живом лидаре на веб-пульте.
    """
    ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]
    manager._sticky = ports[1]
    monkeypatch.setattr(lidar, "discover_ports", lambda **kw: list(ports))
    monkeypatch.setattr(lidar, "excluded_ports", lambda: set())
    manager.excluded = set()
    assert manager.candidate_ports()[0] == ports[1]
    assert manager.candidate_ports()[0] == ports[1]


def test_env_file_change_is_picked_up_by_other_process(manager, tmp_path):
    env = tmp_path / ".env"
    env.write_text("RC_LIDAR_PORT=/dev/ttyUSB7\nRC_LIDAR_PROTOCOL=lds01\n"
                   "RC_LIDAR_BAUD=230400\nRC_LIDAR_MOTOR=none\n"
                   "RC_LIDAR_AUTOPROBE=0\n", encoding="utf-8")
    manager._env_file = env
    manager._env_mtime = 0.0
    manager._env_checked = 0.0
    manager._sync_settings_from_env_file()
    assert manager.current_settings() == {
        "port": "/dev/ttyUSB7", "protocol": "lds01", "baud": "230400",
        "motor": "none", "autoprobe": False,
    }
