"""Подтверждение расположения камеры и «Сброс» на странице «Устройства».

Оператор смотрит на изображение камеры и подтверждает, что она стоит там,
куда её назначили. Раньше подтверждение требовало «живой» подсказки реестра:
если камеру назначили до того, как она появилась в sysfs (или by-id ещё не
создан), подсказка удалялась — и подтвердить камеру было нельзя уже никогда
(«Сначала укажите порт камеры»), хотя кадры с назначенного порта шли.

Здесь закреплено:

* подтверждение принимается по свежему кадру с назначенного оператором порта;
* USB-идентичность обновляется при подтверждении, если её видно;
* устаревший кадр, кадр с другого порта и подменённое USB-устройство
  по-прежнему не подтверждаются;
* «Сброс» выключает камеру, снимает подтверждение и работает при любом
  состоянии реестра, в том числе когда камеры нет в топологии.
"""

from __future__ import annotations

import copy

import pytest

from robot_control.camera import CameraManager

HEADERS = {"X-Robot-Control": "devices"}


def _node(path, kind, *, interface="1.0", index=""):
    return {"path": path, "kind": kind, "aliases": [], "interface": interface,
            "index": index, "name": ""}


def _usb(topology, vidpid, serial, nodes):
    return {"topology": topology, "vidPid": vidpid, "serial": serial,
            "manufacturer": "Logitech", "product": "Logitech C930e", "class": "00",
            "interfaces": [], "busPath": "", "nodes": list(nodes),
            "connection": "через хаб · 1-2", "parent": "1-2", "isHub": False,
            "printerClass": False}


class Topology:
    """Минимальная замена sysfs-обхода: список устройств и счётчик чтений."""

    def __init__(self, devices):
        self.devices = devices
        self.calls = 0

    def collect(self):
        self.calls += 1
        return {"available": True, "devices": copy.deepcopy(self.devices),
                "collectedAt": 1000}


class FakeCameras:
    """CameraManager без V4L2: кадры включаются руками в тесте."""

    def __init__(self):
        self.opened: dict[int, str] = {}
        self.age = 0.1

    def reconfigure(self, settings):
        return settings

    def list_cameras(self):
        result = []
        for cam_id in range(4):
            path = self.opened.get(cam_id)
            result.append({"id": cam_id, "device": path or "",
                           "openedPath": path, "frameSource": "capture" if path else "",
                           "frameAgeS": self.age if path else None,
                           "exists": bool(path), "active": bool(path)})
        return result


def four_cameras():
    return [
        _usb("1-4", "046d:0843", "CAM-FL", [_node("/dev/video0", "video", index="0")]),
        _usb("1-5", "046d:0843", "CAM-RL", [_node("/dev/video2", "video", index="0")]),
        _usb("1-6", "046d:0843", "CAM-FR", [_node("/dev/video4", "video", index="0")]),
        _usb("1-7", "046d:0843", "CAM-RR", [_node("/dev/video6", "video", index="0")]),
    ]


@pytest.fixture
def station(kiosk_client, monkeypatch, tmp_path):
    """Бортовая страница устройств с подменёнными USB-топологией и камерами."""
    monkeypatch.setenv("RC_CAMERA_ENV_FILE", str(tmp_path / "camera-ports.env"))
    registry = kiosk_client.application.extensions["rc_devices"]
    registry.topology = Topology([])
    cameras = FakeCameras()
    monkeypatch.setattr(CameraManager, "get_instance", classmethod(lambda cls: cameras))
    return kiosk_client, registry, cameras


def assign(client, role, port):
    return client.post("/api/devices/ports", json={"role": role, "port": port},
                       headers=HEADERS)


def confirm(client, role):
    return client.post("/api/devices/camera/confirm", json={"role": role}, headers=HEADERS)


def card(client, role):
    snapshot = client.get("/api/devices?refresh=1").get_json()
    return {item["id"]: item for item in snapshot["devices"]}[role]


def test_camera_assigned_before_it_appears_can_still_be_confirmed(station):
    """Камеру назначили, подключили позже — подтверждение доступно."""
    client, registry, cameras = station
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    # Устройства ещё нет в USB-топологии, но назначение оператора не теряется.
    assert registry.records()[0]["camera_FL"]["port"] == "/dev/video0"

    # Камера уже отдаёт кадры по назначенному пути, хотя sysfs её ещё не показал.
    cameras.opened[0] = "/dev/video0"

    item = card(client, "camera_FL")
    assert item["port"] == "/dev/video0"
    assert item["canConfirm"] is True, "кадр с назначенного порта можно подтвердить"
    assert "подтвердите расположение" in item["detail"]

    response = confirm(client, "camera_FL")
    assert response.status_code == 200, response.get_json()
    assert response.get_json()["usbMatched"] is False  # USB-путь ещё не сверен
    item = card(client, "camera_FL")
    assert item["label"] == "Кадры · подтверждено оператором"
    assert "USB-путь ещё не сверен" in item["detail"]

    # Устройство появилось в sysfs — повторное подтверждение фиксирует серийник.
    registry.topology = Topology(four_cameras())
    registry.usb(refresh=True)  # как 2,5-секундный опрос страницы
    response = confirm(client, "camera_FL")
    assert response.status_code == 200
    assert response.get_json()["usbMatched"] is True
    assert response.get_json()["confidence"] == "serial"
    assert registry.records()[0]["camera_FL"]["identity"]["serial"] == "CAM-FL"


def test_confirmed_camera_keeps_identity_after_usb_renumbering(station):
    """Подтверждение хранит USB-серийник, а не номер /dev/videoN."""
    client, registry, cameras = station
    registry.topology = Topology(four_cameras())
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    cameras.opened[0] = "/dev/video0"
    assert confirm(client, "camera_FL").status_code == 200

    moved = four_cameras()
    moved[0]["nodes"] = [_node("/dev/video8", "video", index="0")]
    registry.topology = Topology(moved)
    cameras.opened[0] = "/dev/video8"
    assert confirm(client, "camera_FL").status_code == 200
    assert registry.records()[0]["camera_FL"]["identity"]["serial"] == "CAM-FL"
    assert card(client, "camera_FL")["status"] == "verified"


@pytest.mark.parametrize("state, message", [
    ("stale", "дождитесь изображения"),
    ("other port", "не с назначенного порта"),
])
def test_confirmation_rejects_frames_it_cannot_trust(station, state, message):
    client, registry, cameras = station
    registry.topology = Topology(four_cameras())
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    cameras.opened[0] = "/dev/video0"
    if state == "stale":
        cameras.age = 5.0
    else:
        cameras.opened[0] = "/dev/video2"
    response = confirm(client, "camera_FL")
    assert response.status_code == 400
    assert message in response.get_json()["error"]


def test_replaced_camera_is_confirmed_again_by_the_operator(station):
    """Камеру заменили: старое подтверждение снимается, новое даёт оператор."""
    client, registry, cameras = station
    registry.topology = Topology(four_cameras())
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    cameras.opened[0] = "/dev/video0"
    assert confirm(client, "camera_FL").status_code == 200
    assert registry.camera_role_confirmed("front_left") is True

    replaced = four_cameras()
    replaced[0]["serial"] = "SOMETHING-ELSE"
    registry.topology = Topology(replaced)
    registry.usb(refresh=True)
    assert registry.camera_role_confirmed("front_left") is False, "серийник больше не тот"
    assert registry.camera_target("front_left", "/dev/video0") == ""

    response = confirm(client, "camera_FL")
    assert response.status_code == 200, response.get_json()
    assert registry.records()[0]["camera_FL"]["identity"]["serial"] == "SOMETHING-ELSE"
    assert registry.camera_role_confirmed("front_left") is True


def test_frames_from_another_camera_cannot_be_confirmed(station):
    """Подтверждать чужое изображение нельзя, даже если оно свежее."""
    client, registry, cameras = station
    registry.topology = Topology(four_cameras())
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    cameras.opened[0] = "/dev/video0"
    assert confirm(client, "camera_FL").status_code == 200
    # Камера «передняя правая» отдаёт кадр в слот FL: путь не тот.
    cameras.opened[0] = "/dev/video4"
    response = confirm(client, "camera_FL")
    assert response.status_code == 400
    assert "не с назначенного порта" in response.get_json()["error"]


def test_reset_turns_the_camera_off_and_drops_the_confirmation(station):
    client, registry, cameras = station
    registry.topology = Topology(four_cameras())
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    cameras.opened[0] = "/dev/video0"
    assert confirm(client, "camera_FL").status_code == 200

    response = assign(client, "camera_FL", "")
    assert response.status_code == 200, response.get_json()
    assert "камера выключена" in response.get_json()["message"]

    hint = registry.records()[0]["camera_FL"]
    assert hint["port"] == "" and hint["disabled"] is True
    assert "confirmedAt" not in hint, "сброс снимает подтверждение расположения"
    assert response.get_json()["ports"]["roles"]["camera_FL"]["port"] == ""
    # Камера не открывается по старому пути, пока порт не назначат снова.
    assert registry.camera_target("front_left", "/dev/video0") == ""
    # И подтвердить выключенную камеру нельзя — но с понятным текстом.
    response = confirm(client, "camera_FL")
    assert response.status_code == 400
    assert "Сначала укажите порт" in response.get_json()["error"]


def test_reset_works_when_the_camera_is_unplugged(station):
    """«Сброс» не зависит от того, видно ли сейчас устройство в USB."""
    client, registry, cameras = station
    registry.topology = Topology(four_cameras())
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    registry.topology = Topology([])  # камеру выдернули
    cameras.opened.clear()

    response = assign(client, "camera_FL", "")
    assert response.status_code == 200, response.get_json()
    assert registry.records()[0]["camera_FL"]["disabled"] is True


def test_port_can_be_assigned_and_confirmed_again_after_reset(station):
    client, registry, cameras = station
    registry.topology = Topology(four_cameras())
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    cameras.opened[0] = "/dev/video0"
    assert confirm(client, "camera_FL").status_code == 200
    assert assign(client, "camera_FL", "").status_code == 200

    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    cameras.opened[0] = "/dev/video0"
    response = confirm(client, "camera_FL")
    assert response.status_code == 200
    assert registry.records()[0]["camera_FL"]["identity"]["serial"] == "CAM-FL"


def test_camera_assigned_by_documented_env_key_is_confirmable(station, monkeypatch):
    """.env по документации (``RC_CAMERA_FL_PORT``) — камера видна и подтверждается."""
    client, registry, cameras = station
    monkeypatch.setenv("RC_CAMERA_FL_PORT", "/dev/video0")
    registry.topology = Topology(four_cameras())
    cameras.opened[0] = "/dev/video0"

    ports = client.get("/api/devices/ports").get_json()
    assert ports["roles"]["camera_FL"]["port"] == "/dev/video0"
    item = card(client, "camera_FL")
    assert item["canConfirm"] is True
    assert confirm(client, "camera_FL").status_code == 200


def test_hint_without_identity_does_not_open_a_guessed_camera(station):
    """Неразрешённая подсказка не открывает камеру по «своему» пути."""
    client, registry, cameras = station
    registry.topology = Topology([])
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    # Порт из .env — решение оператора, его открываем; чужой путь — нет.
    assert registry.camera_target("front_left", "/dev/video0") == "/dev/video0"
    # Замену устройства с известным серийником по-прежнему не открываем.
    registry.topology = Topology(four_cameras())
    assert assign(client, "camera_FL", "/dev/video0").status_code == 200
    replaced = four_cameras()
    replaced[0]["serial"] = "REPLACEMENT"
    replaced[0]["nodes"] = [_node("/dev/video0", "video", index="0")]
    registry.topology = Topology(replaced)
    registry.usb(refresh=True)
    assert registry.camera_target("front_left", "/dev/video0") == ""
