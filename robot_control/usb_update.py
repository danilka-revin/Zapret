"""Read a signed BULKA update from USB or local staging, without executing the input.

A .bulka package is a magic line, signed JSON line, base64 OpenSSH signature
line, then a raw Git bundle. The Releases ZIP wraps one such package. Its
member is streamed, never extracted. Git only sees a verified copy on disk.
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import tempfile
import zipfile
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Iterator

REPOSITORY = "danilka-revin/Zapret"
PACKAGE_NAME = "robot-update.bulka"
ZIP_NAME = "robot-update.zip"
USB_FILENAMES = (ZIP_NAME, PACKAGE_NAME)
TRUST_FILE = Path("data/usb-update-signers")
SIGNER = "bulka-update"
NAMESPACE = "bulka-usb-update"
MAGIC = b"BULKA-USB-1\n"
MANIFEST_LIMIT = 2048
SIGNATURE_LIMIT = 8192
# Keep the whole file below FAT32's 4 GiB limit.
BUNDLE_LIMIT = (4 << 30) - (16 << 10)
# Leave room below both GitHub's asset limit and zipfile's ZIP64 threshold.
ZIP_LIMIT = 2_000_000_000
# The web route has a separate limit; larger signed packages can use USB.
UPLOAD_LIMIT = 512 << 20
CHUNK_SIZE = 1 << 20
SHA_RE = re.compile(r"[0-9a-f]{40}")
BRANCH_RE = re.compile(r"[A-Za-z0-9._/-]{1,120}")


class UsbUpdateError(ValueError):
    """USB update is unavailable, damaged, or not trusted."""


@dataclass(frozen=True)
class UsbManifest:
    branch: str
    revision: str
    size: int
    sha256: str


def find_package(mount: Path | None = None) -> Path | None:
    """Look only in roots of mounted volumes, never recursively or on the robot's disk."""
    result = subprocess.run(
        ["findmnt", "--kernel", "--list", "--json", "--output", "TARGET,SOURCE"],
        capture_output=True, text=True, check=False, timeout=5,
    )
    if result.returncode != 0:
        raise UsbUpdateError("Не удалось проверить подключённые накопители (findmnt).")
    mounts = json.loads(result.stdout).get("filesystems") or []
    packages = set()
    for volume in mounts:
        if not (volume.get("source") or "").startswith("/dev/"):
            continue
        target = Path(volume["target"])
        if mount is not None:
            if target != mount:
                continue
        elif not (Path("/media") in target.parents or Path("/run/media") in target.parents):
            continue
        for name in USB_FILENAMES:
            candidate = target / name
            try:
                candidate.lstat()
            except (FileNotFoundError, PermissionError):
                continue
            packages.add(candidate)
    if len(packages) > 1:
        raise UsbUpdateError("Найдено несколько пакетов обновления. Оставьте только один файл обновления.")
    return next(iter(packages), None)


def _verify_signature(raw: bytes, signature: bytes, trust_file: Path) -> None:
    with tempfile.TemporaryDirectory(prefix="bulka-verify-") as tmp:
        sig = Path(tmp) / "manifest.sig"
        sig.write_bytes(signature)
        try:
            result = subprocess.run(
                ["ssh-keygen", "-Y", "verify", "-q", "-n", NAMESPACE, "-I", SIGNER,
                 "-f", str(trust_file), "-s", str(sig)],
                input=raw, capture_output=True, check=False, timeout=10,
            )
        except FileNotFoundError as exc:
            raise UsbUpdateError("Для проверки подписи нужен ssh-keygen (openssh-client).") from exc
    if result.returncode != 0:
        raise UsbUpdateError("Подпись файла обновления не подтверждена доверенным ключом робота.")


class _BoundedZipInput:
    """Do not let an unsigned ZIP directory request a large read/allocation."""

    def __init__(self, stream: BinaryIO, size: int) -> None:
        self.stream = stream
        self.size = size
        self.seek = stream.seek
        self.tell = stream.tell
        self.seekable = stream.seekable

    def read(self, size: int = -1) -> bytes:
        if size < 0:
            size = max(0, self.size - self.tell())
        if size > CHUNK_SIZE:
            raise UsbUpdateError(f"{ZIP_NAME}: слишком большой каталог архива.")
        return self.stream.read(size)


@contextmanager
def _open_zip_package(raw: BinaryIO, size: int) -> Iterator[tuple[BinaryIO, int]]:
    if not 0 < size <= ZIP_LIMIT:
        raise UsbUpdateError(f"{ZIP_NAME}: пустой файл или размер больше 2 ГБ.")
    try:
        with zipfile.ZipFile(_BoundedZipInput(raw, size)) as archive:
            entries = archive.infolist()
            if len(entries) != 1 or entries[0].orig_filename != PACKAGE_NAME:
                raise UsbUpdateError(f"Нужен {ZIP_NAME} из GitHub Releases. «Code → Download ZIP» не подходит.")
            member = entries[0]
            # The bundle is already compressed; a stored member avoids ZIP bombs.
            if (member.compress_type != zipfile.ZIP_STORED or member.flag_bits & 1
                    or not 0 < member.file_size == member.compress_size < size
                    or stat.S_IFMT(member.external_attr >> 16) not in (0, stat.S_IFREG)):
                raise UsbUpdateError(f"{ZIP_NAME}: неподдерживаемый формат. Не перепаковывайте архив.")
            with archive.open(member) as payload:
                yield payload, member.file_size
    except (zipfile.BadZipFile, EOFError, NotImplementedError) as exc:
        raise UsbUpdateError(f"{ZIP_NAME}: архив повреждён. Скачайте файл заново.") from exc


@contextmanager
def _open_package(path: Path) -> Iterator[tuple[BinaryIO, int]]:
    invalid_file = f"{path.name}: не является обычным файлом обновления BULKA."
    if not stat.S_ISREG(path.lstat().st_mode):
        raise UsbUpdateError(invalid_file)
    # Re-check the descriptor in case the source changed since lstat.
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        file = os.fstat(fd)
        if not stat.S_ISREG(file.st_mode):
            raise UsbUpdateError(invalid_file)
        with os.fdopen(fd, "rb", closefd=False) as stream:
            if path.name == ZIP_NAME or stream.peek(4)[:4] == b"PK\x03\x04":
                with _open_zip_package(stream, file.st_size) as payload:
                    yield payload
            else:
                yield stream, file.st_size
    finally:
        os.close(fd)


@contextmanager
def verified_package(path: Path, trust_file: Path) -> Iterator[tuple[BinaryIO, UsbManifest]]:
    """Verify the header and keep the same open stream for payload verification."""
    if not trust_file.is_file():
        raise UsbUpdateError(f"На роботе не установлен доверенный ключ ({TRUST_FILE}).")
    with _open_package(path) as (stream, size):
        if stream.readline(len(MAGIC) + 1) != MAGIC:
            raise UsbUpdateError(f"{path.name}: не является файлом обновления BULKA.")
        manifest = _read_header(stream, size, trust_file)
        yield stream, manifest


def _read_header(stream: BinaryIO, size: int, trust_file: Path) -> UsbManifest:
    raw = stream.readline(MANIFEST_LIMIT + 1)
    encoded_sig = stream.readline(SIGNATURE_LIMIT + 1)
    try:
        if not (0 < len(raw) <= MANIFEST_LIMIT and raw.endswith(b"\n")
                and 0 < len(encoded_sig) <= SIGNATURE_LIMIT and encoded_sig.endswith(b"\n")):
            raise ValueError
        signature = base64.b64decode(encoded_sig[:-1], validate=True)
        data = json.loads(raw.decode("utf-8"))
        if not isinstance(data, dict) or data.get("repository") != REPOSITORY:
            raise ValueError
        manifest = UsbManifest(data["branch"], data["revision"], data["size"], data["sha256"])
        if (not BRANCH_RE.fullmatch(manifest.branch) or not SHA_RE.fullmatch(manifest.revision)
                or not re.fullmatch(r"[0-9a-f]{64}", manifest.sha256)
                or type(manifest.size) is not int or not 0 < manifest.size <= BUNDLE_LIMIT
                or size != stream.tell() + manifest.size):
            raise ValueError
    except (ValueError, TypeError, KeyError, RecursionError) as exc:
        raise UsbUpdateError(f"{PACKAGE_NAME}: повреждённый или неполный пакет.") from exc
    _verify_signature(raw, signature, trust_file)
    return manifest


def verify_bundle_bytes(stream: BinaryIO, manifest: UsbManifest, *, target: BinaryIO | None = None) -> None:
    """Read the authenticated length and check every byte, optionally copying it."""
    digest = hashlib.sha256()
    left = manifest.size
    while left:
        block = stream.read(min(CHUNK_SIZE, left))
        if not block:
            raise UsbUpdateError("Файл обновления не дочитан. Подготовьте файл заново.")
        if target is not None:
            target.write(block)
        digest.update(block)
        left -= len(block)
    if stream.read(1) or digest.hexdigest() != manifest.sha256:
        raise UsbUpdateError("Контрольная сумма пакета не совпала. Скачайте или запишите файл заново.")


@contextmanager
def staged_bundle(stream: BinaryIO, manifest: UsbManifest, work_dir: Path) -> Iterator[Path]:
    """Copy and hash once, then let Git read only the local copy."""
    if shutil.disk_usage(work_dir).free < manifest.size:
        raise UsbUpdateError("На диске робота недостаточно места для файла обновления.")
    with tempfile.TemporaryDirectory(prefix="usb-update-", dir=work_dir) as tmp:
        bundle = Path(tmp) / "update.bundle"
        with bundle.open("wb") as target:
            verify_bundle_bytes(stream, manifest, target=target)
        yield bundle
