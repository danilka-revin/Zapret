"""Robot device inventory and *untrusted* persistent discovery hints.

Only the onboard process owns the drivers. This module reads Linux sysfs and
existing driver snapshots; it never opens a USB device, probes a protocol, or
sends a command. A remembered port is not proof of a device's current role.
"""
from __future__ import annotations

import copy
import json
import math
import os
import re
import tempfile
import threading
import time
from pathlib import Path
from typing import Any

from .config import PROJECT_ROOT

WHEELS = ("FL", "FR", "RL", "RR")
CAMERA_ROLES = {"FL": "front_left", "FR": "front_right", "RL": "rear_left", "RR": "rear_right"}
CAMERA_IDS = {"FL": 0, "FR": 2, "RL": 1, "RR": 3}
ROLES = tuple([f"wheel_{x}" for x in WHEELS] + [f"camera_{x}" for x in WHEELS] +
              ["kinect", "lidar", "printer", "bms"])
ROLE_TITLES = {
    "wheel_FL": "Колесо · переднее левое", "wheel_FR": "Колесо · переднее правое",
    "wheel_RL": "Колесо · заднее левое", "wheel_RR": "Колесо · заднее правое",
    "camera_FL": "Камера · передняя левая", "camera_FR": "Камера · передняя правая",
    "camera_RL": "Камера · задняя левая", "camera_RR": "Камера · задняя правая",
    "kinect": "Kinect · глубина", "lidar": "Лидар", "printer": "Чековый принтер", "bms": "BMS · аккумулятор",
}
SERIAL_RE = re.compile(r"^/dev/(?:tty(?:USB|ACM)\d+|serial/by-(?:id|path)/[A-Za-z0-9_.:+-]+)$")
VIDEO_RE = re.compile(r"^/dev/(?:video\d+|v4l/by-(?:id|path)/[A-Za-z0-9_.:+-]+)$")
USB_RE = re.compile(r"^(?:usb:\d+-\d+(?:\.\d+)*|/dev/bus/usb/\d{3}/\d{3}|/dev/usb/lp\d+)$")
TOPOLOGY_RE = re.compile(r"^\d+-\d+(?:\.\d+)*$")
# libfreenect's NUI/K4W camera IDs; v2 is recognized as a candidate only.
# USB descriptors alone do not bind a particular depth stream to the device.
KINECT_CAMERA_IDS = {"045e:02ae", "045e:02bf", "045e:02d9"}
MAX_ENTRIES = 256


def _finite_number(value: Any) -> bool:
    # bool is an int in Python, and converting an arbitrarily large JSON int
    # to a float can itself raise OverflowError.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False


def _text(path: Path, limit: int = 200) -> str:
    try:
        # sysfs attributes are short. Never allow control characters in the UI.
        with path.open("r", encoding="utf-8", errors="replace") as handle:
            return "".join(c for c in handle.read(limit + 1).strip()[:limit] if c.isprintable())
    except OSError:
        return ""


def _children(path: Path) -> list[Path]:
    try:
        return sorted(path.iterdir(), key=lambda item: item.name)[:MAX_ENTRIES]
    except OSError:
        return []


def _real(path: str) -> str:
    return os.path.realpath(path) if path else ""


def _meaningful_serial(value: str) -> bool:
    compact = re.sub(r"[^a-z0-9]", "", str(value or "").lower())
    return bool(compact and set(compact) != {"0"} and compact not in {"none", "unknown", "noserial"})


def _interface_name(path: Path) -> str:
    for parent in (path, *path.parents):
        if re.fullmatch(r"\d+-\d+(?:\.\d+)*:\d+\.\d+", parent.name):
            return parent.name.rsplit(":", 1)[-1]
    return ""


class UsbTopology:
    """Snapshot of physical USB devices and their tty/video/lp nodes via sysfs."""

    def __init__(self, sysroot: str | Path = "/sys", devroot: str | Path = "/dev"):
        self.sysroot = Path(sysroot)
        self.devroot = Path(devroot)

    def collect(self) -> dict[str, Any]:
        usb_dir = self.sysroot / "bus/usb/devices"
        available = usb_dir.is_dir()
        paths = _children(usb_dir)
        devices: dict[str, dict[str, Any]] = {}
        real_usb: dict[str, str] = {}
        for path in paths:
            vid, pid = _text(path / "idVendor").lower(), _text(path / "idProduct").lower()
            if not re.fullmatch(r"[0-9a-f]{4}", vid) or not re.fullmatch(r"[0-9a-f]{4}", pid):
                continue
            name = path.name
            bus, num = _text(path / "busnum"), _text(path / "devnum")
            bus_path = (str(self.devroot / "bus/usb" / bus.zfill(3) / num.zfill(3))
                        if bus.isdigit() and num.isdigit() and int(bus) <= 999 and int(num) <= 999 else "")
            devices[name] = {
                "topology": name, "vidPid": f"{vid}:{pid}", "serial": _text(path / "serial"),
                "manufacturer": _text(path / "manufacturer"), "product": _text(path / "product"),
                "class": _text(path / "bDeviceClass").lower(), "interfaces": [],
                "busPath": bus_path, "nodes": [],
            }
            real_usb[_real(str(path))] = name
        for path in paths:
            if ":" not in path.name:
                continue
            parent_name = path.name.split(":", 1)[0]
            if parent_name in devices:
                cls = _text(path / "bInterfaceClass").lower()
                if cls and cls not in devices[parent_name]["interfaces"]:
                    devices[parent_name]["interfaces"].append(cls)

        def parent_usb(path: Path) -> str:
            resolved = Path(_real(str(path)))
            for part in (resolved, *resolved.parents):
                if str(part) in real_usb:
                    return real_usb[str(part)]
            return ""

        classes = (("class/tty", "serial", re.compile(r"tty(?:USB|ACM)\d+")),
                   ("class/video4linux", "video", re.compile(r"video\d+")),
                   ("class/usb", "printer", re.compile(r"lp\d+")))
        for folder, kind, pattern in classes:
            for path in _children(self.sysroot / folder):
                if not pattern.fullmatch(path.name):
                    continue
                device_path = path / "device"
                usb_name = parent_usb(device_path)
                if not usb_name:
                    continue
                node_path = self.devroot / ("usb" if kind == "printer" else "") / path.name
                if not node_path.exists():
                    continue
                node = {"path": str(node_path), "kind": kind, "aliases": [],
                        "interface": _interface_name(Path(_real(str(device_path)))),
                        "index": _text(path / "index") if kind == "video" else "",
                        "name": _text(path / "name") if kind == "video" else ""}
                devices[usb_name]["nodes"].append(node)

        nodes = [node for device in devices.values() for node in device["nodes"]]
        by_real = {_real(node["path"]): node for node in nodes}
        for folder in ("serial/by-id", "serial/by-path", "v4l/by-id", "v4l/by-path"):
            for path in _children(self.devroot / folder):
                if not path.exists():
                    continue
                node = by_real.get(_real(str(path)))
                if node is not None:
                    node["aliases"].append(str(path))
        for device in devices.values():
            name = device["topology"]
            if "." in name and TOPOLOGY_RE.fullmatch(name):
                parent = name.rsplit(".", 1)[0]
            elif TOPOLOGY_RE.fullmatch(name):
                parent = "usb" + name.split("-", 1)[0]
            else:
                parent = ""
            upstream = devices.get(parent)
            device["parent"] = parent
            device["connection"] = (
                ("через " + (upstream["product"] or "USB-хаб") + " · " + parent)
                if upstream and parent and not parent.startswith("usb") else
                ("порт компьютера · " + parent if parent else "USB-топология неизвестна"))
            device["isHub"] = device["class"] == "09" or "09" in device["interfaces"]
            device["printerClass"] = device["class"] == "07" or "07" in device["interfaces"]
        return {"available": available, "devices": list(devices.values()), "collectedAt": time.time()}


def _find_port(topology: dict, port: str) -> tuple[dict | None, dict | None]:
    """Find a live node or physical USB path; no stale path is treated as present."""
    for device in topology.get("devices", []):
        if port == "usb:" + device["topology"] or (device.get("busPath") and port == device["busPath"]):
            return device, None
        for node in device["nodes"]:
            if port == node["path"] or port in node["aliases"]:
                return device, node
            if port and os.path.exists(port) and _real(port) == _real(node["path"]):
                return device, node
    return None, None


def _identity(device: dict | None, node: dict | None) -> dict | None:
    if not device:
        return None
    return {"vidPid": device.get("vidPid", ""), "serial": device.get("serial", ""),
            "topology": device.get("topology", ""), "kind": (node or {}).get("kind", "usb"),
            "interface": (node or {}).get("interface", ""), "index": (node or {}).get("index", "")}


def _resolve_identity(topology: dict, identity: Any) -> tuple[dict | None, dict | None, str]:
    """Re-match an old observation against current USB; duplicated serials are weak.

    A VID:PID or a socket never proves physical role. For cameras a port match
    is usable for a stream only after the operator associated the camera; the
    UI explicitly calls out that identical devices could have been swapped.
    """
    if not isinstance(identity, dict) or not re.fullmatch(r"[0-9a-f]{4}:[0-9a-f]{4}", str(identity.get("vidPid", ""))):
        return None, None, "unverified"
    candidates = [d for d in topology.get("devices", []) if d.get("vidPid") == identity["vidPid"]]
    serial = identity.get("serial") if isinstance(identity.get("serial"), str) else ""
    if _meaningful_serial(serial):
        matches = [d for d in candidates if d.get("serial") == serial]
        if len(matches) == 1:
            device, confidence = matches[0], "serial"
        else:
            # Cloned/duplicated serials can distinguish a socket, not a device.
            by_socket = [d for d in matches if d.get("topology") == identity.get("topology")]
            if len(by_socket) != 1:
                return None, None, "ambiguous" if matches else "changed"
            device, confidence = by_socket[0], "socket"
    else:
        by_socket = [d for d in candidates if d.get("topology") == identity.get("topology")]
        if len(by_socket) != 1:
            return None, None, "changed"
        device, confidence = by_socket[0], "socket"
    kind = identity.get("kind")
    if kind == "usb":
        return device, None, confidence
    matches = [n for n in device.get("nodes", []) if n.get("kind") == kind and
               n.get("interface", "") == identity.get("interface", "") and
               n.get("index", "") == identity.get("index", "")]
    if len(matches) != 1:
        return None, None, "ambiguous" if matches else "changed"
    return device, matches[0], confidence


def _valid_hint(role: str, value: Any) -> str:
    if role not in ROLES or not isinstance(value, str):
        raise ValueError("Неизвестное устройство или неверный тип порта")
    if any(ord(c) < 32 for c in value):
        raise ValueError("Недопустимые управляющие символы в пути")
    port = value.strip()
    if not port:
        return ""
    if len(port) > 256 or ".." in port:
        raise ValueError("Недопустимый путь порта")
    allowed = ((VIDEO_RE,) if role.startswith("camera_") else
               (SERIAL_RE,) if role.startswith("wheel_") or role in {"lidar", "bms"} else
               (USB_RE, SERIAL_RE) if role == "printer" else (USB_RE,))
    if not any(pattern.fullmatch(port) for pattern in allowed):
        raise ValueError("Укажите подходящий путь /dev/… или выберите обнаруженное USB-устройство")
    return port


class DeviceRegistry:
    def __init__(self, path: str | Path | None = None, *, topology: UsbTopology | None = None,
                 clock=time.time):
        self.path = Path(path or os.environ.get("RC_DEVICE_REGISTRY_FILE") or
                         PROJECT_ROOT / "data" / "device_registry.json")
        self.topology = topology or UsbTopology()
        self.clock = clock
        self._lock = threading.RLock()
        self._topology_lock = threading.Lock()
        self._topology_data: dict | None = None
        self._topology_time = 0.0
        self._data: dict[str, Any] = {"version": 1, "hints": {}, "verified": {}}
        # Read-only published snapshots: the wheel dispatcher and camera
        # readers must never wait for a registry fsync or a sysfs scan.
        self._fast_hints: dict[str, Any] = {}
        self._fast_priorities: dict[str, frozenset[str]] = {}
        self.error = ""
        self._load()
        self._fast_hints = copy.deepcopy(self._data["hints"])
        try:
            self._refresh_priorities(self.usb(refresh=True))
        except Exception:  # noqa: BLE001 — full discovery works without a cache
            self._fast_priorities = {}

    def _load(self) -> None:
        try:
            if not self.path.exists():
                return
            if self.path.stat().st_size > 256_000:
                raise ValueError("файл слишком большой")
            value = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(value, dict) or value.get("version") != 1:
                raise ValueError("неизвестный формат")
            for section in ("hints", "verified"):
                if not isinstance(value.get(section), dict):
                    raise ValueError("неверный раздел")
                self._data[section] = {role: item for role, item in value[section].items()
                                       if role in ROLES and isinstance(item, dict)}
        except (OSError, ValueError, TypeError, UnicodeError) as exc:
            self.error = "Не удалось прочитать сохранённые подсказки: " + str(exc)
            self._data = {"version": 1, "hints": {}, "verified": {}}

    def _save_locked(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".devices-", suffix=".tmp", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(self._data, handle, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.chmod(tmp, 0o600)
            os.replace(tmp, self.path)
            self.error = ""
        finally:
            try:
                os.unlink(tmp)
            except FileNotFoundError:
                pass

    def usb(self, *, refresh: bool = False) -> dict:
        with self._topology_lock:
            if refresh or self._topology_data is None or time.monotonic() - self._topology_time > 0.75:
                self._topology_data = self.topology.collect()
                self._topology_time = time.monotonic()
            return self._topology_data

    def set_hint(self, role: str, value: Any) -> dict:
        port = _valid_hint(role, value)
        topology = self.usb(refresh=True)
        device, node = _find_port(topology, port) if port else (None, None)
        if role.startswith("camera_") and device and (not node or node.get("kind") != "video"):
            raise ValueError("Для камеры нужен видеопорт этого USB-устройства")
        if role.startswith("camera_") and device and device.get("vidPid") in KINECT_CAMERA_IDS:
            raise ValueError("Этот видеопорт относится к Kinect, а не к обзорной камере")
        hint = ({"port": port, "identity": _identity(device, node), "updatedAt": self.clock()} if port else
                {"port": "", "identity": None, "disabled": True, "updatedAt": self.clock()}
                if role.startswith("camera_") else None)
        with self._lock:
            previous = copy.deepcopy(self._data)
            if role.startswith("camera_") and device:
                for other, old in self._data["hints"].items():
                    if other == role or not other.startswith("camera_"):
                        continue
                    other_device, _, _ = _resolve_identity(topology, old.get("identity"))
                    if other_device and other_device["topology"] == device["topology"]:
                        raise ValueError("Эта физическая камера уже указана для другой роли")
            if hint is None:
                self._data["hints"].pop(role, None)
            else:
                self._data["hints"][role] = hint
            try:
                self._save_locked()
            except OSError:
                self._data = previous
                raise
            self._fast_hints = copy.deepcopy(self._data["hints"])
        self._refresh_priorities(topology)
        return {"ok": True, "role": role, "hint": copy.deepcopy(hint)}

    def confirm_camera(self, role: str, cameras: list) -> dict:
        """Record the operator's visual confirmation, never an inferred role."""
        if role not in {"camera_" + key for key in WHEELS}:
            raise ValueError("Неизвестная камера")
        key = role.removeprefix("camera_")
        cam = next((c for c in cameras if isinstance(c, dict) and c.get("id") == CAMERA_IDS[key]), {})
        frame_age = cam.get("frameAgeS")
        topology = self.usb(refresh=True)
        with self._lock:
            hint = self._data["hints"].get(role)
            if not isinstance(hint, dict):
                raise ValueError("Сначала укажите порт камеры")
            device, node, _ = _resolve_identity(topology, hint.get("identity"))
            if (not device or not node or node.get("kind") != "video" or
                    not cam.get("openedPath") or _real(cam["openedPath"]) != _real(node["path"]) or
                    cam.get("frameSource") != "capture" or not _finite_number(frame_age) or
                    not 0 <= frame_age < 2):
                raise ValueError("Нет свежего кадра с указанной камеры; дождитесь изображения")
            previous = copy.deepcopy(self._data)
            hint["confirmedAt"] = self.clock()
            try:
                self._save_locked()
            except OSError:
                self._data = previous
                raise
            self._fast_hints = copy.deepcopy(self._data["hints"])
            return {"ok": True, "role": role, "confirmedAt": hint["confirmedAt"]}

    def camera_role_confirmed(self, role: str) -> bool:
        """Conservative identity for recording orientation-dependent landmarks."""
        key = next((key for key, value in CAMERA_ROLES.items() if value == role), None)
        hints = self._fast_hints
        hint = hints.get("camera_" + str(key)) if key else None
        confirmed_at = hint.get("confirmedAt") if isinstance(hint, dict) else None
        if not _finite_number(confirmed_at) or not 0 < confirmed_at <= self.clock() + 2:
            return False
        topology = self._topology_data or {"devices": []}
        device, node, confidence = _resolve_identity(topology, hint.get("identity"))
        return bool(device and node and confidence == "serial")

    def records(self) -> tuple[dict, dict]:
        with self._lock:
            return copy.deepcopy(self._data["hints"]), copy.deepcopy(self._data["verified"])

    def remember(self, updates: dict[str, tuple[str, str]], topology: dict) -> None:
        """Persist only observations made by live drivers, not inferred hints."""
        now = self.clock()
        with self._lock:
            previous = copy.deepcopy(self._data)
            changed = False
            for role, (port, check) in updates.items():
                if role not in ROLES or not port:
                    continue
                device, node = _find_port(topology, port)
                candidate = {"port": port, "identity": _identity(device, node), "check": check, "verifiedAt": now}
                old = self._data["verified"].get(role) or {}
                old_at = old.get("verifiedAt")
                if (_finite_number(old_at) and old.get("port") == port and old.get("identity") == candidate["identity"] and
                        old.get("check") == check and 0 <= now - old_at < 60):
                    continue
                self._data["verified"][role] = candidate
                changed = True
            if changed:
                try:
                    self._save_locked()
                except (OSError, ValueError, TypeError) as exc:
                    self._data = previous
                    self.error = "Не удалось сохранить реестр: " + str(exc)
        self._refresh_priorities(topology)

    def _refresh_priorities(self, topology: dict) -> None:
        # Called only by the inventory worker / hint request, never by the
        # motion dispatcher. Publish a completely new immutable snapshot.
        hints, verified = self.records()
        priorities: dict[str, frozenset[str]] = {}
        for role in (*('wheel_' + x for x in WHEELS), 'lidar'):
            priority: set[str] = set()
            for record in (hints.get(role), verified.get(role)):
                if not isinstance(record, dict):
                    continue
                device, node, _ = _resolve_identity(topology, record.get("identity"))
                if device and node and node.get("kind") == "serial":
                    priority.add(_real(node["path"]))
                # An exact existing name is still only a priority.
                port = record.get("port")
                if isinstance(port, str) and SERIAL_RE.fullmatch(port) and os.path.exists(port):
                    priority.add(_real(port))
            priorities[role] = frozenset(priority)
        self._fast_priorities = priorities

    def order_serial(self, ports: list[str], *, roles: tuple[str, ...] | None = None) -> list[str]:
        """Stable priority only: never add, remove, classify or authorize a port.

        This is called by the wheel dispatch thread, so it neither reads the
        registry file nor waits for inventory locks, sysfs, or disk fsync.
        """
        roles = roles or tuple(f"wheel_{x}" for x in WHEELS)
        published = self._fast_priorities
        priority = set().union(*(published.get(role, frozenset()) for role in roles))
        return sorted(ports, key=lambda path: 0 if _real(path) in priority else 1)

    def camera_target(self, role: str, default: str, *, refresh: bool = False) -> str:
        """Resolve a hint; refresh sysfs before opening and fail closed on mismatch."""
        short = next((key for key, value in CAMERA_ROLES.items() if value == role), None)
        if not short:
            return default
        topology = self.usb(refresh=True) if refresh else (self._topology_data or {"devices": []})
        hints = self._fast_hints
        own = hints.get("camera_" + short)
        if isinstance(own, dict):
            device, node, _ = _resolve_identity(topology, own.get("identity"))
            return node["path"] if device and node and node.get("kind") == "video" else ""
        # A legacy/default path must not steal a hinted camera via its second
        # V4L2 interface. Otherwise retain existing behaviour for compatibility.
        default_device, _ = _find_port(topology, default)
        if default_device:
            for key, hint in hints.items():
                if not key.startswith("camera_") or not isinstance(hint, dict):
                    continue
                other, _, _ = _resolve_identity(topology, hint.get("identity"))
                if other and other.get("topology") == default_device.get("topology"):
                    return ""
        return default

    def snapshot(self, *, wheels: dict | None = None, cameras: list | None = None,
                 lidar: dict | None = None, bms: dict | None = None,
                 kinect: dict | None = None, refresh: bool = False) -> dict:
        """Combine producer snapshots. Missing/error snapshots are never cached as live."""
        now = self.clock()
        topology = self.usb(refresh=refresh)
        hints, remembered = self.records()
        wheels = wheels if isinstance(wheels, dict) else {}
        lidar = lidar if isinstance(lidar, dict) else {}
        bms = bms if isinstance(bms, dict) else {}
        kinect = kinect if isinstance(kinect, dict) else {}
        motors_raw = wheels.get("motors") if isinstance(wheels.get("motors"), list) else []
        motors = {str(m.get("id")): m for m in motors_raw if isinstance(m, dict)}
        camera_map = {c.get("id"): c for c in (cameras if isinstance(cameras, list) else []) if isinstance(c, dict)}
        master = wheels.get("masterPort") if isinstance(wheels.get("masterPort"), dict) else {}
        entries, updates = [], {}

        def age(value: Any) -> float | None:
            return round(float(value), 2) if _finite_number(value) and value >= 0 else None

        def base(role: str) -> dict:
            hint = hints.get(role) if isinstance(hints.get(role), dict) else None
            prior = remembered.get(role) if isinstance(remembered.get(role), dict) else None
            match, match_node, confidence = _resolve_identity(topology, (hint or prior or {}).get("identity"))
            return {"id": role, "name": ROLE_TITLES[role], "status": "missing", "label": "Нет подтверждения",
                    "port": None, "usb": None, "ageS": None, "check": "Ожидание данных драйвера",
                    "detail": "", "hint": hint, "remembered": prior,
                    "rememberedMatch": {"port": (match_node or {}).get("path") or
                                        ("usb:" + match["topology"] if match else None),
                                        "confidence": confidence}}

        def connect(entry: dict, port: str | None) -> None:
            if not port:
                return
            entry["port"] = port
            device, _ = _find_port(topology, port)
            if device:
                entry["usb"] = {key: device.get(key) for key in
                                ("topology", "vidPid", "serial", "manufacturer", "product", "connection")}

        for key in WHEELS:
            role, motor = "wheel_" + key, motors.get(key) or {}
            item = base(role)
            # Only MasterSource auto mode supplies confirmed, fresh CRC role/port.
            telemetry_age = age(motor.get("telemetryAgeS"))
            firmware = motor.get("firmware")
            port = (motor.get("port") if master.get("auto") is True and motor.get("online") is True and
                    isinstance(firmware, int) and not isinstance(firmware, bool) and firmware >= 22 and
                    telemetry_age is not None and telemetry_age < 1 else None)
            if port:
                # The source itself suppresses port on identity faults. A
                # separate unknown USB candidate may still block readiness.
                connect(item, port)
                item.update(status="verified", label="Связь подтверждена", ageS=age(motor.get("telemetryAgeS")),
                            check="Свежая телеметрия fw22+ с CRC и ID колеса")
                updates[role] = (port, item["check"])
            elif master.get("error"):
                item.update(status="conflict", label="Ошибка поиска", detail=str(master["error"]),
                            check="Порт не назначается до устранения ошибки")
            elif not master.get("auto"):
                item["detail"] = "Нужен бортовой автоматический источник колёс master."
            else:
                item["detail"] = "Порт ещё не подтвердил роль свежей телеметрией."
            entries.append(item)

        for key in WHEELS:
            role, cam = "camera_" + key, camera_map.get(CAMERA_IDS[key]) or {}
            item = base(role)
            item["configuredPort"] = cam.get("device") or None
            port = cam.get("openedPath")
            target = self.camera_target(CAMERA_ROLES[key], str(cam.get("device") or "")) if cam else ""
            frame_age = age(cam.get("frameAgeS"))
            frame_ok = frame_age is not None and frame_age < 2 and cam.get("frameSource") == "capture"
            if frame_ok and port and target and _real(str(port)) == _real(target):
                connect(item, str(port))
            else:
                port = None
            hint = hints.get(role)
            _, _, confidence = _resolve_identity(topology, (hint or {}).get("identity"))
            if frame_ok and port:
                confirmed_at = hint.get("confirmedAt") if isinstance(hint, dict) else None
                confirmed = _finite_number(confirmed_at) and 0 < confirmed_at <= now + 2
                verified_role = confirmed and confidence == "serial"
                item.update(status="verified" if verified_role else "warning",
                            label="Кадры · роль подтверждена" if verified_role else "Кадры идут · роль проверить",
                            ageS=frame_age, check="Свежий кадр V4L2; направление подтверждает оператор",
                            canConfirm=bool(hint and confidence in {"serial", "socket"}))
                if not hint:
                    item["detail"] = "Порт по настройкам/умолчанию; физическая роль камеры не подтверждена."
                elif not confirmed:
                    item["detail"] = "Проверьте изображение и подтвердите расположение камеры."
                elif confidence == "socket":
                    item["detail"] = "Совпало гнездо и модель, но одинаковые камеры могли поменять местами."
                else:
                    item["detail"] = "Совпал уникальный USB-серийный номер; расположение ранее подтвердил оператор."
                if verified_role:
                    updates[role] = (str(port), item["check"])
            elif cam.get("frameSource") == "external" and frame_age is not None and frame_age < 2:
                item.update(status="warning", label="Внешний кадр", ageS=frame_age,
                            check="Внешний поток не подтверждает USB-камеру")
            elif hint and not target:
                item.update(status="warning", label="Камера отключена" if hint.get("disabled") else "Подсказка не совпала",
                            port=None, usb=None,
                            detail="Укажите порт и проверьте изображение." if hint.get("disabled") else
                                   "Камера не открыта: проверьте порт и укажите её снова.")
            else:
                item["detail"] = str(cam.get("captureError") or "Нет свежих кадров. Проверьте камеру, драйвер и права доступа.")
            entries.append(item)

        item = base("lidar")
        detected = lidar.get("detected") if isinstance(lidar.get("detected"), dict) else {}
        # ROS topic may come from a different host; detected serial path is not
        # bound to that topic. Only show an actual serial producer's port.
        source = str(lidar.get("source") or "")
        port = detected.get("port") if source.startswith("serial:") else None
        scan_age = age(lidar.get("age_s"))
        if (lidar.get("scan_trusted") is True and lidar.get("connected") is True and
                scan_age is not None and scan_age < 1.0 and (source == "ros" or source.startswith("serial:"))):
            connect(item, str(port) if port else None)
            item.update(status="verified", label="Свежий скан", ageS=age(lidar.get("age_s")),
                        check="Свежий скан от драйвера " + source,
                        detail="Порт ROS не подтверждён" if not port else "")
            if port:
                updates["lidar"] = (str(port), item["check"])
            elif source == "ros":
                ros = lidar.get("ros") if isinstance(lidar.get("ros"), dict) else {}
                supervisor = ros.get("supervisor") if isinstance(ros.get("supervisor"), dict) else {}
                reported = supervisor.get("port")
                if (supervisor.get("backend") == "zapret_lidar" and isinstance(reported, str)
                        and SERIAL_RE.fullmatch(reported)):
                    item["reportedPort"] = reported
                    reported_usb, _ = _find_port(topology, reported)
                    if reported_usb:
                        item["usbCandidate"] = {k: reported_usb.get(k) for k in
                                                ("topology", "vidPid", "product", "connection")}
                        updates["lidar"] = (reported, "Порт сообщил ROS-супервизор; источник скана не привязан")
                    item["detail"] = "ROS-супервизор сообщает порт %s; связь порта с топиком не доказана." % reported
        else:
            ros = lidar.get("ros") if isinstance(lidar.get("ros"), dict) else {}
            supervisor = ros.get("supervisor") if isinstance(ros.get("supervisor"), dict) else {}
            reasons = [str(lidar.get("reason") or "Нет свежего проверенного скана.")]
            if supervisor.get("state"):
                reasons.append("ROS: " + str(supervisor["state"]))
            if supervisor.get("reason") and str(supervisor["reason"]) not in reasons:
                reasons.append(str(supervisor["reason"]))
            if supervisor.get("hint") and str(supervisor["hint"]) not in reasons:
                reasons.append(str(supervisor["hint"]))
            item["detail"] = " · ".join(reasons)
        entries.append(item)

        item = base("kinect")
        # The existing publisher does not bind frames to a USB identity; even
        # fresh data and a visible Kinect cannot prove which device produced it.
        live = kinect.get("connected") is True
        stamp = kinect.get("lastStamp")
        spec = kinect.get("spec") if isinstance(kinect.get("spec"), dict) else {}
        raw_limit = spec.get("max_age_s", 1.5)
        limit = float(raw_limit) if _finite_number(raw_limit) and raw_limit > 0 else 1.5
        capture_age = now - stamp if _finite_number(stamp) else None
        fresh_capture = capture_age is not None and 0 <= capture_age <= limit
        if live and fresh_capture:
            item.update(status="warning", label="Кадры глубины идут", ageS=age(kinect.get("ageS")),
                        check="Свежий кадр; USB-источник драйвером не подтверждён",
                        detail="Демо и внешний источник без идентификации нельзя принять за проверенный Kinect.")
        elif live:
            item.update(status="warning", label="Кадр устарел или время неизвестно",
                        check="Время захвата не подтверждает свежесть")
        else:
            item["detail"] = "Нет свежих кадров глубины; проверьте драйвер Kinect."
        hint = hints.get("kinect")
        kd, _, _ = _resolve_identity(topology, (hint or {}).get("identity"))
        candidates = [d for d in topology.get("devices", []) if d.get("vidPid") in KINECT_CAMERA_IDS]
        if not hint and len(candidates) == 1:
            kd = candidates[0]
        if kd:
            item["usbCandidate"] = {k: kd.get(k) for k in ("topology", "vidPid", "product", "connection")}
            if item["status"] == "missing":
                item.update(status="warning", label="USB-кандидат · нет кадров")
            if kd.get("vidPid") in KINECT_CAMERA_IDS:
                updates["kinect"] = ("usb:" + kd["topology"], "USB-дескриптор Kinect; источник кадров не подтверждён")
        elif len(candidates) > 1 and not hint:
            item.update(status="conflict", label="Несколько Kinect USB", detail="Выберите устройство; источник потока всё равно требует проверки.")
        entries.append(item)

        item = base("bms")
        port = bms.get("port")
        if port:
            # The selected port is configuration until a fresh JBD response.
            item["configuredPort"] = str(port)
        bms_age = age(bms.get("ageS"))
        if bms.get("connected") is True and port and bms_age is not None and bms_age <= 5.0:
            connect(item, str(port))
            item.update(status="verified", label="Данные BMS получены", ageS=age(bms.get("ageS")),
                        check="Свежий ответ JBD на вручную выбранном порту",
                        detail=str(bms.get("reason") or ""))
            updates["bms"] = (str(port), item["check"])
        else:
            item["detail"] = str(bms.get("reason") or "Порт BMS выбирается вручную на экране АКБ.")
        entries.append(item)

        item = base("printer")
        hint = hints.get("printer")
        printer_candidates = [d for d in topology.get("devices", []) if d.get("printerClass")]
        selected, selected_node, _ = _resolve_identity(topology, (hint or {}).get("identity"))
        printer = selected if selected and selected.get("printerClass") else (
            printer_candidates[0] if len(printer_candidates) == 1 and not hint else None)
        if selected and selected_node and selected_node.get("kind") == "serial":
            # A USB-UART only identifies its adapter, not the peripheral on
            # its wires. A hint is never proof of a printer or permission.
            item["usb"] = {k: selected.get(k) for k in
                           ("topology", "vidPid", "serial", "manufacturer", "product", "connection")}
            item.update(port=selected_node.get("path"), status="warning", label="USB-UART · подсказка",
                        check="Адаптер совпал; принтер и печать не проверены",
                        detail="Физически проверьте подключение и подтвердите порт в настройках печати на странице АКБ.")
        elif printer:
            item["usb"] = {k: printer.get(k) for k in
                           ("topology", "vidPid", "serial", "manufacturer", "product", "connection")}
            lp = next((n["path"] for n in printer["nodes"] if n["kind"] == "printer"), None)
            item.update(port=lp or printer.get("busPath") or "usb:" + printer["topology"], status="warning",
                        label="USB-класс принтера", check="Только USB-дескриптор, печать не проверена",
                        detail="Нужны модель, интерфейс и драйвер; тестовые команды не отправлялись.")
            updates["printer"] = (item["port"], item["check"])
        elif len(printer_candidates) > 1:
            item.update(status="conflict", label="Несколько принтеров", detail="Укажите нужное USB-устройство.")
        else:
            item["detail"] = "Принтер не распознан по USB-классу; модель и способ подключения неизвестны."
        entries.append(item)

        self.remember(updates, topology)
        # Expose candidate lists, never as proof or a whitelist. Suppress hubs
        # and root controllers; one USB device may have many V4L2 interfaces.
        options = {"serial": [], "video": [], "usb": [], "printer": []}
        for device in topology.get("devices", []):
            if not device.get("isHub") and TOPOLOGY_RE.fullmatch(device.get("topology", "")):
                usb = {"port": "usb:" + device["topology"], "name": device.get("product") or "USB-устройство",
                       "connection": device.get("connection"), "vidPid": device.get("vidPid")}
                options["usb"].append(usb)
                if device.get("printerClass"):
                    options["printer"].append(usb)
            for node in device.get("nodes", []):
                option = {"port": node["path"], "name": node.get("name") or device.get("product") or node["kind"],
                          "connection": device.get("connection"), "vidPid": device.get("vidPid"),
                          "aliases": node.get("aliases", []), "topology": device["topology"],
                          "index": node.get("index", "")}
                if node["kind"] in options and not (node["kind"] == "video" and device.get("vidPid") in KINECT_CAMERA_IDS):
                    options[node["kind"]].append(option)
        return {"ok": True, "generatedAt": now, "usbAvailable": bool(topology.get("available")),
                "storageError": self.error or None, "devices": entries, "options": options,
                "wheelDiscovery": {"ready": bool(master.get("ready")), "error": master.get("error"),
                                   "unclassified": master.get("unclassified") or [],
                                   "excluded": master.get("excluded") or [],
                                   "candidates": master.get("candidates") or []}}
