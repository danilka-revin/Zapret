#!/usr/bin/env python3
"""Отправлять кадры глубины Kinect 360 на бортовой сервер (8090).

Запускается отдельным процессом на том же компьютере, что и киоск: приложение
не зависит от libfreenect / OpenNI2 / ROS 2 и не открывает USB само.

    # демо-кадр (без железа, проверка всей цепочки)
    .venv/bin/python scripts/kinect_depth_publisher.py --source demo --once

    # libfreenect с привязкой к выбранному NUI Camera (RC_KINECT_PORT в .env)
    ./scripts/kinect_depth.sh --foreground

    # OpenNI2/ROS 2 пока не запускаются с RC_KINECT_PORT: для них ещё нет
    # проверяемой привязки USB-топологии к URI/topic.

Кадр уменьшается до 160×120 и уходит как base64 uint16 (мм) в
`POST /api/kinect/depth`. Геометрия и фильтры лежат в `.env` борта
(`RC_KINECT_*`), поэтому калибровка не дублируется в драйвере.
"""
from __future__ import annotations

import argparse
import array
import base64
import json
import math
import os
import re
import struct
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any, Optional

DEFAULT_URL = "http://127.0.0.1:8090/api/kinect/depth"
CONTROL_HEADER = "X-Robot-Control"
DEFAULT_WIDTH, DEFAULT_HEIGHT = 160, 120
# libfreenect v1 supports Xbox NUI Camera and Kinect-for-Windows camera only.
FREENECT_CAMERA_IDS = {(0x045E, 0x02AE), (0x045E, 0x02BF)}
USB_TOPOLOGY_RE = re.compile(r"^usb:(\d+-\d+(?:\.\d+)*)$")
USB_DEVICE_NODE_RE = re.compile(r"^/dev/bus/usb/(\d{3})/(\d{3})$")
SYS_USB_ROOT = Path("/sys/bus/usb/devices")


def configured_kinect_port() -> str:
    """RC_KINECT_PORT from process environment or project .env."""
    value = os.environ.get("RC_KINECT_PORT", "").strip()
    if value:
        return value
    env_path = Path(__file__).resolve().parents[1] / ".env"
    try:
        for line in env_path.read_text(encoding="utf-8").splitlines():
            if line.startswith("RC_KINECT_PORT="):
                return line.partition("=")[2].strip().strip("\\\"").strip("'")
    except OSError:
        pass
    return ""


def usb_topology_for_port(port: str, sys_usb_root: Path = SYS_USB_ROOT) -> str:
    """Normalize the saved selector value to sysfs topology (e.g. ``1-2.3``)."""
    value = str(port or "").strip()
    match = USB_TOPOLOGY_RE.fullmatch(value)
    if match:
        return match.group(1)
    node = USB_DEVICE_NODE_RE.fullmatch(value)
    if node:
        bus, address = (int(part, 10) for part in node.groups())
        try:
            paths = sorted(sys_usb_root.iterdir(), key=lambda item: item.name)
        except OSError:
            paths = []
        for path in paths:
            try:
                candidate_bus = int((path / "busnum").read_text().strip())
                candidate_address = int((path / "devnum").read_text().strip())
            except (OSError, ValueError):
                continue
            if candidate_bus == bus and candidate_address == address and re.fullmatch(
                    r"\d+-\d+(?:\.\d+)*", path.name):
                return path.name
    raise RuntimeError("RC_KINECT_PORT должен указывать на выбранный NUI Camera (usb:1-2)")


def freenect_index_for_port(port: str, devices=None, *, sys_usb_root: Path = SYS_USB_ROOT) -> int:
    """Map selected sysfs USB topology to libfreenect's own camera index.

    PyUSB preserves libusb_get_device_list ordering, the same list libfreenect
    enumerates. NUI Audio is excluded; index counts only freenect camera IDs.
    """
    target = usb_topology_for_port(port, sys_usb_root)
    if devices is None:
        try:
            import usb.core  # type: ignore
            import usb.backend.libusb1  # type: ignore
        except ImportError as exc:
            raise RuntimeError("для привязки USB нужен PyUSB: установите pyusb") from exc
        backend = usb.backend.libusb1.get_backend()
        if backend is None:
            raise RuntimeError("PyUSB не нашёл backend libusb-1.0; невозможно безопасно сопоставить индекс")
        devices = usb.core.find(find_all=True, backend=backend)
    candidates = []
    for device in devices or ():
        try:
            vendor, product = int(device.idVendor), int(device.idProduct)
        except (AttributeError, TypeError, ValueError):
            continue
        if (vendor, product) not in FREENECT_CAMERA_IDS:
            continue
        try:
            bus = int(device.bus)
            ports = tuple(int(port_number) for port_number in device.port_numbers)
        except (AttributeError, TypeError, ValueError, NotImplementedError):
            raise RuntimeError("USB-драйвер не сообщает топологию Kinect; выберите libfreenect с libusb 1.0")
        if not ports:
            raise RuntimeError("libusb не сообщил USB-путь NUI Camera")
        topology = str(bus) + "-" + ".".join(str(port_number) for port_number in ports)
        candidates.append((topology, device))
    for index, (topology, _device) in enumerate(candidates):
        if topology == target:
            return index
    raise RuntimeError("выбранный порт %s не найден среди камер, поддерживаемых libfreenect" % port)


def resample_u16(samples: array.array, width: int, height: int,
                 target_w: int, target_h: int) -> array.array:
    """Простое прореживание кадра без numpy (сохраняем 16-битные значения)."""
    out = array.array("H", [0]) * (target_w * target_h)
    for y in range(target_h):
        src_y = min(height - 1, y * height // target_h)
        row = src_y * width
        for x in range(target_w):
            src_x = min(width - 1, x * width // target_w)
            out[y * target_w + x] = samples[row + src_x]
    return out


def demo_frame(width: int = DEFAULT_WIDTH, height: int = DEFAULT_HEIGHT,
               wall_mm: int = 2000) -> tuple[array.array, int, int]:
    """Стена впереди и пол внизу кадра — для проверки фильтра высот."""
    data = array.array("H", [0]) * (width * height)
    for y in range(height):
        for x in range(width):
            if height // 3 < y < height * 2 // 3 and width // 4 < x < width * 3 // 4:
                data[y * width + x] = wall_mm
            elif y > height * 2 // 3:
                # «Пол»: точки ближе к нижнему краю должны быть отфильтрованы.
                data[y * width + x] = 1200
    return data, width, height


def freenect_raw_to_mm(raw: int) -> int:
    """Approximate Kinect v1 11-bit disparity to millimetres, 0 = invalid.

    libfreenect's default stream is not millimetres. This empirical formula
    is uncalibrated; no depth or obstacle detection is guaranteed.
    """
    if not 0 <= raw < 2047:
        return 0
    angle = raw / 2842.5 + 1.1863
    if angle >= math.pi / 2:
        return 0
    depth = 0.1236 * math.tan(angle)
    if not math.isfinite(depth) or depth <= 0 or depth * 1000 > 65535:
        return 0
    return max(1, int(depth * 1000))


_FREENECT_MM = tuple(freenect_raw_to_mm(i) for i in range(2048))


def freenect_frame(index: int = 0) -> tuple[array.array, int, int]:
    import freenect  # type: ignore

    depth, _ = freenect.sync_get_depth(index=index)
    width, height = int(depth.shape[1]), int(depth.shape[0])
    try:
        raw = depth.astype("<u2").tobytes()
    except AttributeError:
        raw = array.array("H", [int(v) for v in depth.flat]).tobytes()
    samples = array.array("H")
    samples.frombytes(raw)
    if sys.byteorder != "little":
        samples.byteswap()
    samples = array.array("H", (_FREENECT_MM[value] if value < 2048 else 0
                                for value in samples))
    return samples, width, height


def openni_frame() -> tuple[array.array, int, int]:
    import openni2  # type: ignore  # noqa: F401  (openni2 регистрирует драйвер)

    from openni2 import Device  # type: ignore

    dev = Device.open_any()
    stream = dev.create_depth_stream()
    stream.start()
    try:
        frame = stream.read_frame()
        width, height = int(frame.width), int(frame.height)
        raw = bytes(frame.get_buffer_as_uint16())
        samples = array.array("H")
        samples.frombytes(raw[: width * height * 2])
        if sys.byteorder != "little":
            samples.byteswap()
        return samples, width, height
    finally:
        stream.stop()
        dev.close()


def ros2_frame(topic: str, timeout_s: float = 3.0) -> tuple[array.array, int, int]:
    """Подписаться на один кадр 16UC1 (мм) или 32FC1 (м)."""
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from sensor_msgs.msg import Image  # type: ignore

    rclpy.init()
    holder: dict[str, Any] = {}

    class Listener(Node):
        def __init__(self):
            super().__init__("zapret_kinect_depth")
            self.create_subscription(Image, topic, self._on_frame, 5)

        def _on_frame(self, msg: Any):
            holder["msg"] = msg

    node = Listener()
    deadline = time.monotonic() + timeout_s
    try:
        while "msg" not in holder and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    msg = holder.get("msg")
    if msg is None:
        raise RuntimeError("нет кадра в топике %s за %.1f с" % (topic, timeout_s))
    width, height = int(msg.width), int(msg.height)
    raw = bytes(msg.data)
    samples = array.array("H")
    if msg.encoding in {"32FC1", "32FC1 "}:
        floats = struct.unpack("<%df" % (len(raw) // 4), raw[: (len(raw) // 4) * 4])
        for value in floats:
            mm = int(value * 1000.0) if value > 0 else 0
            samples.append(max(0, min(65535, mm)))
    else:
        samples.frombytes(raw[: width * height * 2])
        if sys.byteorder != "little":
            samples.byteswap()
    return samples, width, height


def grab(source: str, topic: str, device_index: int = 0) -> tuple[array.array, int, int]:
    if source == "demo":
        return demo_frame()
    if source == "freenect":
        return freenect_frame(device_index)
    if source == "openni":
        return openni_frame()
    if source == "ros2":
        return ros2_frame(topic)
    raise ValueError("неизвестный источник: %s" % source)


def post(url: str, samples: array.array, width: int, height: int,
         stamp: float, timeout_s: float = 3.0, unit: str = "front",
         capture_source: str = "", capture_port: str = "") -> dict[str, Any]:
    if sys.byteorder != "little":
        samples = array.array("H", samples)
        samples.byteswap()
    payload = {
        "width": width, "height": height,
        "depth": base64.b64encode(samples.tobytes()).decode("ascii"),
        "units": "mm", "stamp": stamp,
    }
    if capture_source:
        payload["capture_source"] = capture_source
    if capture_port:
        payload["capture_port"] = capture_port
    if unit != "front":
        payload["unit"] = unit
        url = url + ("&" if "?" in url else "?") + "unit=" + unit
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST", headers={
        "Content-Type": "application/json",
        CONTROL_HEADER: "kinect",
    })
    with urllib.request.urlopen(req, timeout=timeout_s) as reply:
        return json.loads(reply.read().decode("utf-8"))


BACKEND_MODULES: tuple[tuple[str, str], ...] = (
    ("freenect", "freenect"),
    ("openni", "openni2"),
    ("ros2", "rclpy"),
)


def detect_backends() -> list[str]:
    """Какие драйверы Kinect реально импортируются (без чтения кадров)."""
    found: list[str] = []
    for name, module in BACKEND_MODULES:
        try:
            __import__(module)
        except ImportError:
            continue
        found.append(name)
    return found


def detect_source(allow_demo: bool = False, *, require_usb_binding: bool = False) -> tuple[str, str]:
    """Choose a backend; a saved physical port requires libfreenect's index API."""
    found = detect_backends()
    if require_usb_binding:
        if "freenect" in found:
            return "freenect", ""
        return "", ("Kinect: для точной привязки RC_KINECT_PORT нужен backend freenect "
                    "и PyUSB. OpenNI/ROS 2 не позволяют этому publisher-у выбрать USB-топологию; "
                    "источник не подменён.")
    if found:
        return found[0], ""
    if allow_demo:
        return "demo", "Kinect: драйверы не найдены, явно разрешён демо-кадр (--allow-demo)"
    return "", ("Kinect: не найден ни один драйвер (freenect / openni2 / rclpy). "
                "См. requirements-kinect.txt и docs/AUTONOMY.md; для проверки "
                "связи без железа запустите --source demo.")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Kinect 360 → карта робота")
    parser.add_argument("--source", choices=("auto", "demo", "freenect", "openni", "ros2"),
                        default="auto", help="auto выбирает установленный драйвер Kinect")
    parser.add_argument("--allow-demo", action="store_true",
                        help="в auto разрешить демо-кадр, если драйверов нет")
    parser.add_argument("--list-backends", action="store_true",
                        help="показать установленные драйверы и выйти")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--usb-port", default=configured_kinect_port(),
                        help="selected Kinect USB topology (default: RC_KINECT_PORT from .env)")
    parser.add_argument("--topic", default="/camera/depth/image_rect_raw")
    parser.add_argument("--unit", choices=("front", "rear"), default="front",
                        help="какой Kinect публикует кадр (rear требует RC_KINECT_REAR_ENABLE=1)")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--once", action="store_true", help="отправить один кадр и выйти")
    args = parser.parse_args(argv)

    if args.list_backends:
        installed = detect_backends()
        print(json.dumps({
            "installed": installed,
            "missing": [name for name, _module in BACKEND_MODULES if name not in installed],
            "hint": "requirements-kinect.txt · docs/AUTONOMY.md",
        }, ensure_ascii=False))
        return 0

    source = args.source
    if source == "auto":
        detected, hint = detect_source(args.allow_demo,
                                       require_usb_binding=bool(args.usb_port))
        if hint:
            print(hint, file=sys.stderr)
        if not detected:
            return 2
        source = detected

    device_index = 0
    capture_port = ""
    if source != "demo":
        if args.unit == "rear":
            print("Kinect: задний датчик пока не поддержан привязкой к USB-порту; используйте один передний Kinect.",
                  file=sys.stderr)
            return 2
        if not args.usb_port:
            print("Kinect: сначала выберите NUI Camera во вкладке «Устройства» (RC_KINECT_PORT).",
                  file=sys.stderr)
            return 2
        if source != "freenect":
            print("Kinect: привязка выбранного USB-порта пока поддерживается только backend freenect; "
                  "не запускаю другой источник вместо выбранного устройства.", file=sys.stderr)
            return 2
        try:
            topology = usb_topology_for_port(args.usb_port)
            device_index = freenect_index_for_port(args.usb_port)
        except RuntimeError as exc:
            print("Kinect: %s" % exc, file=sys.stderr)
            return 2
        capture_port = "usb:" + topology

    period = 1.0 / max(0.1, min(30.0, args.fps))
    frames = 0
    try:
        while True:
            samples, raw_w, raw_h = grab(source, args.topic, device_index=device_index)
            small = resample_u16(samples, raw_w, raw_h, args.width, args.height)
            reply = post(args.url, small, args.width, args.height, time.time(),
                         unit=args.unit, capture_source=source, capture_port=capture_port)
            frames += 1
            if args.once:
                print(json.dumps({"source": source, "reply": reply}, ensure_ascii=False))
                break
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 — драйвер не должен падать молча
        print("Kinect: %s" % exc, file=sys.stderr)
        return 1
    if not args.once:
        print("Kinect: отправлено кадров: %d" % frames, file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
