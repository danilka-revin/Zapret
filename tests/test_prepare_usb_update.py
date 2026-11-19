"""Сборка офлайн-флешки: пакет, ключ, инструкция и скрипт настройки робота."""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from scripts import prepare_usb_update


def _fake_package(tmp_path, monkeypatch):
    """Не тянем GitHub и не проверяем подпись: проверяем сборку комплекта."""

    def make_package(repo, output, key, *, branch, base, as_zip=False, target_revision=None):
        output.mkdir(parents=True, exist_ok=True)
        path = output / ("robot-update.zip" if as_zip else "robot-update.bulka")
        path.write_bytes(b"BULKA-USB-1\n")
        return path, "a" * 40

    monkeypatch.setattr(prepare_usb_update.make_usb_update, "make_package", make_package)
    monkeypatch.setattr(prepare_usb_update, "verify_package", lambda path, pub: None)


def test_kit_contains_package_key_instruction_and_robot_script(tmp_path, monkeypatch):
    _fake_package(tmp_path, monkeypatch)
    key = tmp_path / "keys/bulka-usb-update"
    key.parent.mkdir(parents=True, exist_ok=True)
    key.write_text("private\n")
    key.with_suffix(".pub").write_text("ssh-ed25519 AAAA test\n")
    out = tmp_path / "flash"
    monkeypatch.setattr("sys.argv", ["prepare_usb_update.py", str(out), "--key", str(key)])
    assert prepare_usb_update.main() == 0

    assert sorted(p.name for p in out.iterdir()) == [
        "INSTALL.txt", "bulka-usb-update.pub", "install-on-robot.sh", "robot-update.zip"]
    text = (out / "INSTALL.txt").read_text(encoding="utf-8")
    assert "a" * 40 in text and "robot-update.zip" in text
    script = (out / "install-on-robot.sh").read_text(encoding="utf-8")
    assert "data/usb-update-signers" in script
    assert "RC_USB_UPDATE_AUTO=1" in script
    assert (out / "install-on-robot.sh").stat().st_mode & 0o111


def test_robot_script_installs_trust_and_auto(tmp_path):
    if not shutil.which("ssh-keygen") or not shutil.which("findmnt") or not shutil.which("git"):
        pytest.skip("нужны ssh-keygen, findmnt и git")
    flash = tmp_path / "flash"
    flash.mkdir()
    (flash / "bulka-usb-update.pub").write_text("ssh-ed25519 AAAA test\n", encoding="utf-8")
    (flash / "robot-update.zip").write_bytes(b"dummy")
    (flash / "install-on-robot.sh").write_text(prepare_usb_update.ROBOT_SCRIPT, encoding="utf-8")

    repo = tmp_path / "robot"
    (repo / ".git").mkdir(parents=True)
    (repo / ".env").write_text("RC_PORT=8080\n", encoding="utf-8")

    result = subprocess.run(["sh", str(flash / "install-on-robot.sh"), "--auto"],
                            cwd=repo, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    trust = (repo / "data" / "usb-update-signers").read_text(encoding="utf-8").strip()
    assert trust == "bulka-update ssh-ed25519 AAAA test"
    assert "RC_USB_UPDATE_AUTO=1" in (repo / ".env").read_text(encoding="utf-8")

    # Повторный запуск не дублирует доверенный ключ.
    subprocess.run(["sh", str(flash / "install-on-robot.sh"), "--auto"],
                   cwd=repo, check=True, capture_output=True)
    assert len((repo / "data" / "usb-update-signers").read_text(encoding="utf-8").strip().splitlines()) == 1
