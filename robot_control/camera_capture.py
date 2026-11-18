"""V4L2 readers for a single *already selected* device. No discovery here."""
from __future__ import annotations

import os
import re
import select
import subprocess
import time
from typing import NamedTuple


FFMPEG_FRAME_TIMEOUT_S = 4.0


class Mode(NamedTuple):
    backend: str
    fourcc: str
    width: int
    height: int
    fps: int

    @property
    def label(self) -> str:
        settings = f"{self.fourcc} {self.width}×{self.height}@{self.fps}" if self.fourcc else "режим драйвера"
        return f"{self.backend} · {settings}"


def modes(has_cv: bool, has_ffmpeg: bool) -> list[Mode]:
    # First try compressed UVC at USB-friendly rates (4 cameras on one hub).
    # Some webcams are raw-only; others reject all property changes.
    profiles = [("MJPG", 640, 480, 15), ("MJPG", 320, 240, 10),
                ("YUYV", 320, 240, 10), ("", 0, 0, 0)]
    backends = (["OpenCV"] if has_cv else []) + (["FFmpeg"] if has_ffmpeg else [])
    return [Mode(backend, *profile) for backend in backends for profile in profiles]


class Frame(NamedTuple):
    jpeg: bytes
    width: int
    height: int
    fps: float | None
    fourcc: str | None
    dark: bool | None


def opencv_frames(cv, path, mode, current):
    cap = cv.VideoCapture(path, cv.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise OSError("не удалось открыть видеовход; проверьте, не занят ли он")
        if not current(revalidate=True):
            return
        if mode.fourcc:
            # set() returning False is common (e.g. fixed FPS); only real pixels
            # validate the negotiation. Never insist on an exact property echo.
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*mode.fourcc))
            cap.set(cv.CAP_PROP_FRAME_WIDTH, mode.width)
            cap.set(cv.CAP_PROP_FRAME_HEIGHT, mode.height)
            cap.set(cv.CAP_PROP_FPS, mode.fps)
        fps = cap.get(cv.CAP_PROP_FPS)
        fps = fps if 0 < fps <= 240 else None
        code = cap.get(cv.CAP_PROP_FOURCC)
        fourcc = int(code).to_bytes(4, 'little').decode('ascii', 'replace') if 0 < code <= 0xffffffff else ''
        fourcc = fourcc if fourcc.isascii() and fourcc.isprintable() else None
        failures = 0
        last_ok = time.monotonic()
        while current():
            ok, image = cap.read()
            if ok and image is not None and image.size:
                ok, encoded = cv.imencode('.jpg', image, [cv.IMWRITE_JPEG_QUALITY, 75])
                if ok and encoded.size:
                    height, width = image.shape[:2]
                    # Evidence only; never reject a real dark scene or covered lens.
                    dark = bool(image[::16, ::16].max() < 12)
                    last_ok = time.monotonic()
                    failures = 0
                    yield Frame(encoded.tobytes(), width, height, fps, fourcc, dark)
                    continue  # blocking read() already paces us at the device FPS
            failures += 1
            if failures >= 4 or time.monotonic() - last_ok >= 2:
                raise OSError("вход открыт, но нет кадра JPEG; проверьте режим, питание и полосу USB")
            time.sleep(.05)
    finally:
        cap.release()


def _multipart_jpegs(buf):
    # FFmpeg's mpjpeg muxer gives the exact packet length. Looking for an EOI
    # byte sequence in the payload is not sufficient (JPEG segments are binary).
    while True:
        end = buf.find(b'\r\n\r\n', 0, 512)
        if end < 0:
            if len(buf) > 512:
                raise OSError("некорректный заголовок MJPEG от FFmpeg")
            return
        length = re.search(br'\r\nContent-length: (\d{1,7})\r\n', buf[:end + 2], re.I)
        if not length or not 4 <= int(length[1]) <= 2_000_000:
            raise OSError("некорректная длина JPEG от FFmpeg")
        stop = end + 4 + int(length[1])
        if len(buf) < stop:
            return
        jpeg = bytes(buf[end + 4:stop])
        del buf[:stop]
        if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise OSError("повреждённый JPEG от FFmpeg")
        yield jpeg


def ffmpeg_frames(binary, path, mode, current):
    # libavdevice opens the exact V4L2 path, never an index, URL or client camera.
    # Decode and re-encode: a UVC MJPG packet need not be a standalone browser JPEG.
    size = (mode.width or 320, mode.height or 240)
    args = [binary, '-nostdin', '-hide_banner', '-loglevel', 'error', '-probesize', '256k',
            '-analyzeduration', '200000', '-f', 'v4l2']
    if mode.fourcc:
        fmt = 'mjpeg' if mode.fourcc == 'MJPG' else 'yuyv422'
        args += ['-input_format', fmt, '-video_size', '%dx%d' % size, '-framerate', str(mode.fps)]
    args += ['-threads', '1', '-i', path, '-an']
    output = ['-s', '%dx%d' % size, '-c:v', 'mjpeg', '-threads', '1', '-q:v', '5',
              '-f', 'mpjpeg', '-flush_packets', '1', 'pipe:1']
    # Pass frames through without synthesizing duplicates. New FFmpeg no
    # longer recognizes -vsync, while older Ubuntu doesn't have -fps_mode.
    try:
        yield from _ffmpeg_pipe(args + ['-fps_mode', 'passthrough'] + output, size, current)
    except OSError as exc:
        if "Unrecognized option 'fps_mode'" not in str(exc):
            raise
        if current():
            yield from _ffmpeg_pipe(args + ['-vsync', '0'] + output, size, current)


def _ffmpeg_pipe(args, size, current):
    proc = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
    buf = bytearray()
    error = b''
    # Read diagnostics before handling simultaneous stdout EOF (e.g. an
    # unsupported option reported by a version that exited immediately).
    pipes = [proc.stderr, proc.stdout]
    deadline = time.monotonic() + FFMPEG_FRAME_TIMEOUT_S
    try:
        while current():
            if time.monotonic() >= deadline:
                raise OSError(error.decode(errors='replace').strip() or "нет кадра за отведённое время")
            ready, _, _ = select.select(pipes, [], [], .15)
            for pipe in ready:
                chunk = os.read(pipe.fileno(), 65536)
                if pipe is proc.stderr:
                    error = (error + chunk)[-512:]
                    if not chunk:
                        pipes.remove(pipe)
                elif not chunk:
                    # stderr and stdout can hit EOF together. Read the remaining
                    # diagnostic only when the child has exited (never block).
                    if proc.poll() is not None:
                        error = (error + (proc.stderr.read(512) or b''))[-512:]
                    raise OSError(error.decode(errors='replace').strip() or "FFmpeg завершил видеопоток")
                else:
                    buf.extend(chunk)
                    for jpeg in _multipart_jpegs(buf):
                        deadline = time.monotonic() + FFMPEG_FRAME_TIMEOUT_S
                        yield Frame(jpeg, *size, None, None, None)
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.wait()
        proc.stdout.close()
        proc.stderr.close()
