#!/usr/bin/env python3
"""Build one signed USB update from the project's GitHub repository."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_control.usb_update import (  # noqa: E402
    BRANCH_RE, BUNDLE_LIMIT, CHUNK_SIZE, MAGIC, NAMESPACE, PACKAGE_NAME, REPOSITORY, SHA_RE,
    UsbUpdateError,
)

GITHUB_URL = f"https://github.com/{REPOSITORY}.git"


def make_package(repo: Path, output: Path, key: Path, *, branch: str, base: str | None) -> tuple[Path, str]:
    def git(*args: str) -> str:
        return subprocess.run(["git", "-C", str(repo), *args], check=True,
                              stdout=subprocess.PIPE, text=True).stdout.strip()

    if not BRANCH_RE.fullmatch(branch) or branch.startswith("-") or ".." in branch:
        raise UsbUpdateError("Некорректное имя ветки GitHub.")
    git("check-ref-format", f"refs/heads/{branch}")
    if base and not SHA_RE.fullmatch(base):
        raise UsbUpdateError("--base: укажите полный хеш коммита робота (git rev-parse HEAD).")
    if not output.is_dir() or not key.is_file():
        raise UsbUpdateError("Укажите существующую папку назначения и закрытый ключ SSH.")
    if git("rev-parse", "--is-shallow-repository") == "true":
        raise UsbUpdateError("На компьютере нужен Git-клон с историей. Сначала выполните git fetch --unshallow.")

    # Fetch the selected GitHub branch into a private ref. Do not package the
    # current worktree, local edits, or whatever happens to be in FETCH_HEAD.
    ref = f"refs/bulka-usb/{uuid4().hex}"
    destination = output / PACKAGE_NAME
    try:
        git("fetch", "--no-tags", "--no-recurse-submodules", GITHUB_URL, f"refs/heads/{branch}:{ref}")
        revision = git("rev-parse", "--verify", f"{ref}^{{commit}}")
        if base == revision:
            raise UsbUpdateError("На GitHub тот же коммит: обновлять пока нечего.")
        if base:
            try:
                git("merge-base", "--is-ancestor", base, revision)
            except subprocess.CalledProcessError as exc:
                raise UsbUpdateError("Коммит --base не входит в историю выбранной ветки GitHub.") from exc
        with tempfile.TemporaryDirectory(prefix="bulka-pack-") as tmp:
            bundle = Path(tmp) / "update.bundle"
            git("bundle", "create", str(bundle), ref, *([f"^{base}"] if base else []))
            size = bundle.stat().st_size
            if size > BUNDLE_LIMIT:
                raise UsbUpdateError("Пакет больше 4 ГБ. Укажите --base с текущим коммитом робота.")
            digest = hashlib.sha256()
            with bundle.open("rb") as stream:
                for block in iter(lambda: stream.read(CHUNK_SIZE), b""):
                    digest.update(block)
            raw = (json.dumps({"repository": REPOSITORY, "branch": branch, "revision": revision,
                               "size": size, "sha256": digest.hexdigest()}) + "\n").encode("utf-8")
            signature = subprocess.run(
                ["ssh-keygen", "-Y", "sign", "-q", "-n", NAMESPACE, "-f", str(key)],
                input=raw, stdout=subprocess.PIPE, check=True,
            ).stdout
            # Publish atomically, even if the destination is a FAT32 drive.
            temporary = output / f".bulka-update-{uuid4().hex}.tmp"
            target = temporary.open("xb")
            try:
                with target, bundle.open("rb") as stream:
                    target.write(MAGIC + raw + base64.b64encode(signature) + b"\n")
                    shutil.copyfileobj(stream, target, CHUNK_SIZE)
                    target.flush()
                    os.fsync(target.fileno())
                os.replace(temporary, destination)
            finally:
                temporary.unlink(missing_ok=True)
    finally:
        git("update-ref", "-d", ref)
    return destination, revision


def main() -> int:
    parser = argparse.ArgumentParser(description="Скачать обновление BULKA с GitHub и подготовить файл для флешки.")
    parser.add_argument("output", type=Path, help="папка назначения (например, корень флешки)")
    parser.add_argument("--key", required=True, type=Path, help="закрытый ключ SSH для подписи")
    parser.add_argument("--branch", default="main", help="ветка на GitHub (по умолчанию main)")
    parser.add_argument("--base", help="полный текущий хеш робота, чтобы собрать пакет только с изменениями")
    args = parser.parse_args()
    try:
        path, revision = make_package(ROOT, args.output.expanduser().resolve(), args.key.expanduser().resolve(),
                                      branch=args.branch, base=args.base)
    except (UsbUpdateError, subprocess.CalledProcessError, OSError) as exc:
        print(f"Не удалось подготовить пакет: {exc}", file=sys.stderr)
        return 1
    print(f"Готово: {path}\nВерсия: {revision}\nНа робота переносится только {PACKAGE_NAME}, не закрытый ключ.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
