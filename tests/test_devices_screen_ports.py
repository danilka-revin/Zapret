"""Выбор портов на странице «Устройства»: реальные USB-порты вместо дублей.

Оператор жаловался, что в настройке каждого устройства висит стена
задвоенных последовательных портов. Причина в том, что один физический порт
виден трижды: ``/dev/ttyUSB0``, ``/dev/serial/by-id/…`` и
``/dev/serial/by-path/…``, а ещё раз — как ``usb:1-2``. Здесь закреплено:

* в каждом списке одна строка на физический порт, подходящий тип — первым;
* строка начинается с настоящего порта (``1-2.1 · /dev/ttyUSB0 · CH340``);
* выбор из списка заполняет поле пути для ``.env``;
* отключённый, но назначенный порт виден и помечен «не подключено»;
* списка-подсказки ``<datalist>`` больше нет — есть обычный ``<select>``.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
HARNESS = Path(__file__).resolve().parent / "js" / "devices_ports.js"
NODE = shutil.which("node")

pytestmark = pytest.mark.skipif(
    NODE is None, reason="нет node: экранные скрипты проверяются только с ним")

ROLES = ("camera_FL", "camera_FR", "camera_RL", "camera_RR", "lidar", "kinect", "bms", "printer")


def option(port, device, name, usb_port, *, by_id="", by_path="", exists=True, kind=""):
    data = {"port": port, "device": device, "name": name, "usbPort": usb_port,
            "byId": by_id, "byPath": by_path, "exists": exists}
    if kind:
        data["kind"] = kind
    return data


SERIAL = [
    option("/dev/serial/by-id/usb-1a86_A-if00-port0", "/dev/ttyUSB0", "Maker · CH340", "1-2.1",
           by_id="/dev/serial/by-id/usb-1a86_A-if00-port0",
           by_path="/dev/serial/by-path/pci-0:1.2-port0"),
    option("/dev/serial/by-id/usb-1a86_B-if00-port0", "/dev/ttyUSB1", "Maker · CH340", "1-2.2",
           by_id="/dev/serial/by-id/usb-1a86_B-if00-port0"),
]
VIDEO = [option("/dev/v4l/by-id/usb-Cam-video-index0", "/dev/video0", "Maker · Camera", "1-4",
                by_id="/dev/v4l/by-id/usb-Cam-video-index0")]
USB = [{"port": "usb:1-6", "device": "usb:1-6", "name": "Kinect", "usbPort": "1-6",
        "vidPid": "045e:02ae", "kinect": True, "exists": True}]


def ports_payload(*, serial=None, extra_roles=None):
    roles = {role: {"title": role, "port": (extra_roles or {}).get(role, ""),
                    "envKey": "RC_X", "kind": "serial", "assigned": True, "note": ""}
             for role in ROLES}
    return {
        "ok": True, "manual": True, "roles": roles,
        "wheels": {"envKey": "RC_WHEEL_PORTS", "slots": ["/dev/ttyUSB0", "", "", ""],
                   "titles": ["Переднее левое", "Переднее правое", "Заднее левое", "Заднее правое"],
                   "resolved": {}, "ready": False, "candidates": [], "error": None},
        "options": {"serial": SERIAL if serial is None else serial,
                    "video": VIDEO, "usb": USB},
    }


def snapshot_payload(options=None):
    devices = [{"id": role, "name": role, "status": "missing", "label": "Нет подтверждения",
                "port": None, "assignedPort": "", "canConfirm": False}
               for role in ("wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR", *ROLES)]
    return {"ok": True, "devices": devices, "generatedAt": 1_800_000_000, "usbAvailable": True,
            "options": options or {"serial": SERIAL, "video": VIDEO, "usb": USB},
            "wheelDiscovery": {"ready": False, "candidates": [], "manualPorts": ["/dev/ttyUSB0"]}}


def run_screen(tmp_path: Path, *, ports=None, snapshot=None, pick=None) -> dict:
    payload = {"ports": ports or ports_payload(), "snapshot": snapshot or snapshot_payload()}
    if pick is not None:
        payload["pickValue"] = pick
    path = tmp_path / "devices.json"
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    proc = subprocess.run([NODE, str(HARNESS), str(path)], capture_output=True, text=True,
                          timeout=120)
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout)


# ------------------------------------------------------------------ списки
def test_card_lists_offer_each_physical_port_once(tmp_path):
    out = run_screen(tmp_path)
    assert out["errors"] == []
    assert out["status"] == "Связь с бортовым сервером"

    lidar = out["lidar"]
    values = lidar["values"]
    assert values == ["/dev/serial/by-id/usb-1a86_A-if00-port0",
                      "/dev/serial/by-id/usb-1a86_B-if00-port0",
                      "/dev/v4l/by-id/usb-Cam-video-index0", "usb:1-6"]
    # Ни одного повтора: by-id и by-path одного порта — не две строки.
    assert len(values) == len(set(values))
    assert not any("by-path" in value for value in values)

    # Подпись начинается с настоящего порта, а не с длинного by-id.
    assert lidar["labels"][1] == "1-2.1 · /dev/ttyUSB0 · Maker · CH340"
    assert lidar["labels"][2].startswith("1-2.2 · /dev/ttyUSB1")


def test_matching_kind_goes_first_in_its_own_group(tmp_path):
    out = run_screen(tmp_path)
    lidar_groups = out["lidar"]["groups"]
    assert lidar_groups[0]["label"] == "Подходящие · последовательный"
    assert lidar_groups[0]["values"] == ["/dev/serial/by-id/usb-1a86_A-if00-port0",
                                         "/dev/serial/by-id/usb-1a86_B-if00-port0"]
    # Группы не перемешиваются: тип идёт одним блоком.
    labels = [group["label"] for group in lidar_groups]
    assert len(labels) == len(set(labels))

    camera_groups = out["camera"]["groups"]
    assert camera_groups[0]["label"] == "Подходящие · видео"
    assert camera_groups[0]["values"] == ["/dev/v4l/by-id/usb-Cam-video-index0"]

    kinect_groups = out["kinect"]["groups"]
    assert kinect_groups[0]["label"] == "Подходящие · только USB"
    assert out["kinect"]["labels"][1] == "1-6 · usb:1-6 · Kinect"


def test_wheel_pickers_offer_the_same_physical_ports(tmp_path):
    out = run_screen(tmp_path)
    for picker in out["wheels"]:
        assert picker["values"] == out["lidar"]["values"]
        assert picker["groups"][0]["label"] == "Подходящие · последовательный"
    assert out["inputs"][2] == "/dev/ttyUSB0"  # сохранённое значение на месте


def test_found_list_shows_real_ports_not_aliases(tmp_path):
    out = run_screen(tmp_path)
    rows = out["found"]["rows"]
    assert [row["port"] for row in rows] == ["1-2.1 · /dev/ttyUSB0", "1-2.2 · /dev/ttyUSB1",
                                             "1-4 · /dev/video0", "1-6 · usb:1-6"]
    assert rows[0]["name"] == "Maker · CH340"
    assert out["found"]["text"].startswith("Найдено физических портов: 4")
    assert out["found"]["tone"] == "good"


def test_disconnected_assigned_port_is_marked_and_last(tmp_path):
    serial = SERIAL + [option("/dev/serial/by-id/usb-gone", "/dev/ttyUSB9",
                              "назначено в настройках", "", exists=False,
                              by_id="/dev/serial/by-id/usb-gone")]
    out = run_screen(tmp_path, ports=ports_payload(serial=serial),
                     snapshot=snapshot_payload({"serial": serial, "video": VIDEO, "usb": USB}))
    assert out["lidar"]["missing"] == ["/dev/serial/by-id/usb-gone"]
    labels = out["lidar"]["labels"]
    assert any("не подключено" in label for label in labels)
    # Отключённый порт не лезет вперёд рабочих.
    assert labels[1].startswith("1-2.1") and labels[2].startswith("1-2.2")
    assert out["found"]["text"].startswith("Найдено физических портов: 4")


def test_busy_port_is_marked_with_its_owner_before_saving(tmp_path):
    """Оператор видит «занят: лидар» ещё в списке, а не после ошибки."""
    serial = [dict(SERIAL[0], owner="lidar", ownerTitle="лидар", ownerEnv="RC_LIDAR_PORT"),
              SERIAL[1]]
    out = run_screen(tmp_path, ports=ports_payload(serial=serial),
                     snapshot=snapshot_payload({"serial": serial, "video": VIDEO, "usb": USB}))
    labels = out["lidar"]["labels"]
    assert "занят: лидар" in labels[1]
    assert "занят" not in labels[2]
    # В общем списке найденных портов владелец тоже виден.
    assert "назначен: лидар" in out["found"]["rows"][0]["owner"]
    assert out["found"]["rows"][1]["owner"] == ""


def test_choosing_a_port_fills_the_env_field(tmp_path):
    out = run_screen(tmp_path, pick="/dev/serial/by-id/usb-1a86_B-if00-port0")
    assert out["afterPick"]["value"] == "/dev/serial/by-id/usb-1a86_B-if00-port0"
    assert out["afterPick"]["input"] == "/dev/serial/by-id/usb-1a86_B-if00-port0"


def test_same_port_from_two_sources_is_listed_once(tmp_path):
    """Пульт 8080 видит и /api/devices, и /api/devices/ports: порт не двоится."""
    ports = ports_payload()
    # Второй источник назвал тот же порт узлом, а не by-id.
    snapshot = snapshot_payload({"serial": [option("/dev/ttyUSB0", "/dev/ttyUSB0",
                                                   "Maker · CH340", "1-2.1")],
                                 "video": VIDEO, "usb": USB})
    out = run_screen(tmp_path, ports=ports, snapshot=snapshot)
    values = out["lidar"]["values"]
    assert len(values) == len(set(values))
    assert values.count("/dev/serial/by-id/usb-1a86_A-if00-port0") == 1
    assert "/dev/ttyUSB0" not in values  # тот же порт, второй раз — не нужен


# ------------------------------------------------------------------ разметка
def test_page_has_a_real_picker_and_no_datalist_flood(kiosk_client):
    page = kiosk_client.get("/devices").get_data(as_text=True)
    assert 'id="ports-found-list"' in page
    assert "wheel-slots-options" not in page        # даталист с сотнями строк убран
    js = kiosk_client.get("/devices.js").get_data(as_text=True)
    assert "datalist" not in js
    assert "port-pick" in js and "wheel-pick-" in js
    # Список строится по физическому порту, а не по каталогу /dev.
    assert "usbPort" in js and "byPath" in js


# ------------------------------------------------- камера: подтверждение и сброс
def camera_snapshot(can_confirm=True):
    """Снимок, где камера FL уже даёт кадры с назначенного порта."""
    snapshot = snapshot_payload()
    for item in snapshot["devices"]:
        if item["id"] != "camera_FL":
            continue
        item.update(status="warning", label="Кадры идут · роль проверить",
                    port="/dev/video0", assignedPort="/dev/v4l/by-id/usb-Cam-video-index0",
                    canConfirm=can_confirm,
                    detail="Проверьте изображение и подтвердите расположение камеры.")
    return snapshot


def run_actions(tmp_path: Path, **payload_extra) -> dict:
    payload = {"ports": ports_payload(), "snapshot": camera_snapshot()}
    payload.update(payload_extra)
    path = tmp_path / "devices.json"
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    proc = subprocess.run([NODE, str(HARNESS), str(path)], capture_output=True, text=True,
                          timeout=120)
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout)


def test_camera_confirm_button_posts_without_a_browser_dialog(tmp_path):
    """Кнопка подтверждения не зависит от window.confirm (в киоске он молчит)."""
    out = run_actions(tmp_path, cameraConfirm="camera_FL")
    assert out["actions"]["confirm"]["found"] is True
    assert out["actions"]["confirm"]["hidden"] is False
    posts = [call for call in out["posts"] if "camera/confirm" in call["url"]]
    assert len(posts) == 1
    assert json.loads(posts[0]["body"]) == {"role": "camera_FL"}
    assert out["confirms"] == [], "системный диалог не должен требоваться"
    assert "Расположение подтверждено оператором" in out["actions"]["confirm"]["message"]


def test_camera_reset_needs_two_clicks_and_posts_an_empty_port(tmp_path):
    """«Сброс» сначала взводится, вторым нажатием выключает камеру."""
    first = run_actions(tmp_path, cameraReset="camera_FL", cameraResetClicks=1)
    assert first["actions"]["reset"] == ["Точно сбросить?"]
    assert not [call for call in first["posts"] if call["url"].endswith("/api/devices/ports")]
    assert "Камера остановится" in first["actions"]["resetMessage"]
    assert first["confirms"] == []

    second = run_actions(tmp_path, cameraReset="camera_FL", cameraResetClicks=2)
    assert second["actions"]["reset"] == ["Точно сбросить?", "Сброс"]
    posts = [call for call in second["posts"] if call["url"].endswith("/api/devices/ports")]
    assert len(posts) == 1
    assert json.loads(posts[0]["body"]) == {"role": "camera_FL", "port": ""}
    assert second["confirms"] == []


def test_confirm_button_is_hidden_until_a_frame_arrives(tmp_path):
    out = run_actions(tmp_path, cameraConfirm="camera_FL", snapshot=camera_snapshot(False))
    assert out["actions"]["confirm"]["found"] is True
    assert out["actions"]["confirm"]["hidden"] is True
