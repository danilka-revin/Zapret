"""
Системная интеграция: systemd-служба автозапуска и ярлык приложения.
"""

import subprocess
from pathlib import Path

from . import APP_NAME, APP_SLUG, app_dir
from .core import elevation_prefix, run_privileged, which

SERVICE_NAME = "zapret-control"
SERVICE_FILE = f"/etc/systemd/system/{SERVICE_NAME}.service"


def run_py_path() -> Path:
    return app_dir() / "run.py"


def _unit_text() -> str:
    return f"""[Unit]
Description=Zapret Control — обход DPI (YouTube/Discord/Telegram)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=root
WorkingDirectory={app_dir()}
ExecStart=/usr/bin/env python3 {run_py_path()} daemon
ExecStop=/usr/bin/env python3 {run_py_path()} stop
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
"""


def service_installed() -> bool:
    return Path(SERVICE_FILE).exists()


def service_active() -> bool:
    if which("systemctl") is None:
        return False
    r = subprocess.run(["systemctl", "is-active", SERVICE_NAME],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return r.returncode == 0


def install_service() -> None:
    run_privileged(["bash", "-c", f"cat > {SERVICE_FILE}"],
                   input_text=_unit_text())
    run_privileged(["systemctl", "daemon-reload"])
    run_privileged(["systemctl", "enable", SERVICE_NAME])
    run_privileged(["systemctl", "start", SERVICE_NAME])


def remove_service() -> None:
    run_privileged(["systemctl", "stop", SERVICE_NAME], check=False)
    run_privileged(["systemctl", "disable", SERVICE_NAME], check=False)
    run_privileged(["rm", "-f", SERVICE_FILE], check=False)
    run_privileged(["systemctl", "daemon-reload"], check=False)


def start_service() -> None:
    run_privileged(["systemctl", "start", SERVICE_NAME])


def stop_service() -> None:
    run_privileged(["systemctl", "stop", SERVICE_NAME])


# ----------------------------------------------------------------------------
# Ярлык приложения (.desktop)
# ----------------------------------------------------------------------------

DESKTOP_NAME = "zapret-control.desktop"


def icon_path() -> Path:
    """Иконка приложения: установленная копия или файл рядом с кодом."""
    for p in (app_dir() / "assets" / "zapret-control.png",
              Path(__file__).resolve().parent.parent / "assets" / "zapret-control.png"):
        if p.exists():
            return p
    return app_dir() / "assets" / "zapret-control.png"


def desktop_file_path() -> Path:
    return Path.home() / ".local" / "share" / "applications" / DESKTOP_NAME


def _desktop_text() -> str:
    return f"""[Desktop Entry]
Version=1.0
Type=Application
Name={APP_NAME}
Comment=Обход замедления YouTube, Discord и Telegram (zapret)
Exec=/usr/bin/env python3 {run_py_path()} gui
Icon={icon_path()}
Terminal=false
Categories=Network;Utility;System;
Keywords=zapret;youtube;discord;telegram;dpi;vpn;
StartupWMClass={APP_SLUG}
"""


def shortcut_installed() -> bool:
    return desktop_file_path().exists()


def install_shortcut() -> None:
    p = desktop_file_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(_desktop_text(), encoding="utf-8")
    p.chmod(0o755)
    if which("update-desktop-database"):
        subprocess.run(["update-desktop-database",
                        str(Path.home() / ".local" / "share" / "applications")],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def remove_shortcut() -> None:
    p = desktop_file_path()
    if p.exists():
        p.unlink()
    if which("update-desktop-database"):
        subprocess.run(["update-desktop-database",
                        str(Path.home() / ".local" / "share" / "applications")],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ----------------------------------------------------------------------------
# NOPASSWD для nft/nfqws (работа GUI без постоянного ввода пароля)
# ----------------------------------------------------------------------------

SUDOERS_FILE = "/etc/sudoers.d/zapret-control"


def _sudoers_text(user: str) -> str:
    import shutil as _shutil

    defaults = {"nft": "/usr/sbin/nft", "iptables": "/usr/sbin/iptables",
                "ip6tables": "/usr/sbin/ip6tables", "pkill": "/usr/bin/pkill",
                "systemctl": "/usr/bin/systemctl"}

    def path_of(cmd):
        # ищем с расширенным PATH (nft/iptables обычно в /usr/sbin)
        extended = "/usr/local/sbin:/usr/sbin:/sbin:/usr/local/bin:/usr/bin:/bin"
        found = _shutil.which(cmd, path=extended)
        return found or defaults.get(cmd, f"/usr/bin/{cmd}")

    lines = [f"# Zapret Control — NOPASSWD для {user}", f"# Файл: {SUDOERS_FILE}", ""]
    for cmd in ("nft", "iptables", "ip6tables", "pkill", "systemctl"):
        lines.append(f"{user} ALL=(root) NOPASSWD: {path_of(cmd)}")
    lines.append(f"{user} ALL=(root) NOPASSWD: {app_dir() / 'nfqws'} *")
    lines.append("")
    return "\n".join(lines)


def setup_permissions(user: str | None = None) -> None:
    import getpass
    user = user or getpass.getuser()
    if which("sudo") is None:
        raise RuntimeError("sudo не найден. Настройте права вручную.")
    content = _sudoers_text(user)
    # Запись через интерактивный sudo (запросит пароль, если нужно)
    p = subprocess.run(
        ["sudo", "bash", "-c", f"cat > {SUDOERS_FILE} && chmod 440 {SUDOERS_FILE}"],
        input=content, text=True,
    )
    if p.returncode != 0:
        raise RuntimeError("Не удалось настроить sudoers (нужен пароль sudo).")
    # Проверка синтаксиса
    if which("visudo"):
        subprocess.run(["sudo", "visudo", "-c", "-f", SUDOERS_FILE],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def remove_permissions() -> None:
    run_privileged(["rm", "-f", SUDOERS_FILE], check=False)


def permissions_ready() -> bool:
    """Проверяет, может ли приложение работать без пароля."""
    try:
        r = subprocess.run(elevation_prefix() + ["true"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return r.returncode == 0
    except OSError:
        return False
