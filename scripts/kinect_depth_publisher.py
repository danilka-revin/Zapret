#!/usr/bin/env python3
"""Отправлять кадры глубины Kinect 360 на бортовой сервер (8090).

Запускается отдельным процессом на том же компьютере, что и киоск: приложение
не зависит от libfreenect / OpenNI2 / ROS 2 и не открывает USB само.

    # демо-кадр (без железа, проверка всей цепочки)
    .venv/bin/python scripts/kinect_depth_publisher.py --source demo --once

    # libfreenect (Kinect 360 / Xbox 360)
    .venv/bin/python scripts/kinect_depth_publisher.py --source freenect

    # OpenNI2 (если Kinect виден через openni2)
    .venv/bin/python scripts/kinect_depth_publisher.py --source openni

    # ROS 2: топик sensor_msgs/Image с глубиной 16UC1 (мм) или 32FC1 (м)
    .venv/bin/python scripts/kinect_depth_publisher.py --source ros2 \
        --topic /camera/depth/image_rect_raw

Кадр уменьшается до 160×120 и уходит как base64 uint16 (мм) в
`POST /api/kinect/depth`. Геометрия и фильтры лежат в `.env` борта
(`RC_KINECT_*`), поэтому калибровка не дублируется в драйвере.
"""
from __future__ import annotations

import argparse
import array
import base64
import json
import struct
import sys
import time
import urllib.request
from typing import Any, Optional

DEFAULT_URL = "http://127.0.0.1:8090/api/kinect/depth"
CONTROL_HEADER = "X-Robot-Control"
DEFAULT_WIDTH, DEFAULT_HEIGHT = 160, 120


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


def freenect_frame() -> tuple[array.array, int, int]:
    import freenect  # type: ignore

    depth, _ = freenect.sync_get_depth()
    width, height = int(depth.shape[1]), int(depth.shape[0])
    try:
        raw = depth.astype("<u2").tobytes()
    except AttributeError:
        raw = array.array("H", [int(v) for v in depth.flat]).tobytes()
    samples = array.array("H")
    samples.frombytes(raw)
    if sys.byteorder != "little":
        samples.byteswap()
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


def grab(source: str, topic: str) -> tuple[array.array, int, int]:
    if source == "demo":
        return demo_frame()
    if source == "freenect":
        return freenect_frame()
    if source == "openni":
        return openni_frame()
    if source == "ros2":
        return ros2_frame(topic)
    raise ValueError("неизвестный источник: %s" % source)


def post(url: str, samples: array.array, width: int, height: int,
         stamp: float, timeout_s: float = 3.0) -> dict[str, Any]:
    if sys.byteorder != "little":
        samples = array.array("H", samples)
        samples.byteswap()
    body = json.dumps({
        "width": width, "height": height,
        "depth": base64.b64encode(samples.tobytes()).decode("ascii"),
        "units": "mm", "stamp": stamp,
    }).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST", headers={
        "Content-Type": "application/json",
        CONTROL_HEADER: "kinect",
    })
    with urllib.request.urlopen(req, timeout=timeout_s) as reply:
        return json.loads(reply.read().decode("utf-8"))


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Kinect 360 → карта робота")
    parser.add_argument("--source", choices=("auto", "demo", "freenect", "openni", "ros2"),
                        default="auto", help="auto пробует freenect, openni, затем demo")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--topic", default="/camera/depth/image_rect_raw")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--once", action="store_true", help="отправить один кадр и выйти")
    args = parser.parse_args(argv)

    source = args.source
    if source == "auto":
        for candidate in ("freenect", "openni"):
            try:
                __import__(candidate if candidate != "openni" else "openni2")
                source = candidate
                break
            except ImportError:
                continue
        else:
            source = "demo"
            print("Kinect не найден: отправляю демо-кадр (--source demo)", file=sys.stderr)

    period = 1.0 / max(0.1, min(30.0, args.fps))
    frames = 0
    try:
        while True:
            samples, raw_w, raw_h = grab(source, args.topic)
            small = resample_u16(samples, raw_w, raw_h, args.width, args.height)
            reply = post(args.url, small, args.width, args.height, time.time())
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
