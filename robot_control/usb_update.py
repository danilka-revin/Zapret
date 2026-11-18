"""Read a signed BULKA Git bundle from a mounted drive, without running USB code.

File format: magic line, signed JSON line, base64 OpenSSH signature line, then
one raw Git bundle. The signature covers the target branch/revision and the
size and SHA-256 of the bundle. Git only sees a verified copy on local disk.
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
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Iterator

REPOSITORY = "danilka-revin/Zapret"
PACKAGE_NAME = "robot-update.bulka"
TRUST_FILE = Path("data/usb-update-signers")
SIGNER = "bulka-update"
NAMESPACE = "bulka-usb-update"
MAGIC = b"BULKA-USB-1\n"
MANIFEST_LIMIT = 2048
SIGNATURE_LIMIT = 8192
# Keep the whole file below FAT32's 4 GiB limit.
BUNDLE_LIMIT = (4 << 30) - (16 << 10)
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
        candidate = target / PACKAGE_NAME
        try:
            candidate.lstat()
        except (FileNotFoundError, PermissionError):
            continue
        packages.add(candidate)
    if len(packages) > 1:
        raise UsbUpdateError("Найдено несколько файлов обновления. Оставьте одну флешку.")
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


@contextmanager
def verified_package(path: Path, trust_file: Path) -> Iterator[tuple[BinaryIO, UsbManifest]]:
    """Verify the small header and keep the same open file for payload verification."""
    if not trust_file.is_file():
        raise UsbUpdateError(f"На роботе не установлен доверенный ключ ({TRUST_FILE}).")
    invalid_file = f"{PACKAGE_NAME}: не является обычным файлом обновления BULKA."
    if not stat.S_ISREG(path.lstat().st_mode):
        raise UsbUpdateError(invalid_file)
    # Re-check the opened descriptor in case the medium changed since lstat.
    # O_NOFOLLOW/O_NONBLOCK guard against symlinks and FIFOs.
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        file = os.fstat(fd)
        if not stat.S_ISREG(file.st_mode):
            raise UsbUpdateError(invalid_file)
        with os.fdopen(fd, "rb", closefd=False) as stream:
            if stream.readline(len(MAGIC) + 1) != MAGIC:
                raise UsbUpdateError(f"{PACKAGE_NAME}: не является файлом обновления BULKA.")
            manifest = _read_header(stream, file.st_size, trust_file)
            yield stream, manifest
    finally:
        os.close(fd)


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


@contextmanager
def staged_bundle(stream: BinaryIO, manifest: UsbManifest, work_dir: Path) -> Iterator[Path]:
    """Copy and hash once, then use only the local copy (the USB may be removed)."""
    if shutil.disk_usage(work_dir).free < manifest.size:
        raise UsbUpdateError("На диске робота недостаточно места для файла обновления.")
    with tempfile.TemporaryDirectory(prefix="usb-update-", dir=work_dir) as tmp:
        bundle = Path(tmp) / "update.bundle"
        digest = hashlib.sha256()
        left = manifest.size
        with bundle.open("wb") as target:
            while left:
                block = stream.read(min(CHUNK_SIZE, left))
                if not block:
                    raise UsbUpdateError("Файл обновления не дочитан. Не вынимайте флешку при копировании.")
                target.write(block)
                digest.update(block)
                left -= len(block)
        if stream.read(1) or digest.hexdigest() != manifest.sha256:
            raise UsbUpdateError("Контрольная сумма пакета не совпала. Запишите файл на флешку заново.")
        yield bundle
