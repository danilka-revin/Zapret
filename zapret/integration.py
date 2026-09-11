"""
Системная интеграция: systemd-служба автозапуска, ярлык приложения и права NOPASSWD.

Ярлык пишется в каталоги ПОЛЬЗОВАТЕЛЯ рабочего стола (меню приложений + сам
рабочий стол), даже если установка или починка запускались из-под root — иначе
«иконки нет на столе» при `sudo -i` было бы нормой.
"""

import os
import shutil
import subprocess
from pathlib import Path

from . import APP_NAME, APP_SLUG, app_dir, session
from .core import elevation_prefix, run_privileged, which

SERVICE_NAME = "zapret-control"
SERVICE_FILE = f"/etc/systemd/system/{SERVICE_NAME}.service"

DESKTOP_NAME = f"{APP_SLUG}.desktop"
ICON_NAME = APP_SLUG


def run_py_path() -> Path:
    return app_dir() / "run.py"


# ---------------------------------------------------------------------------
# Кто владелец ярлыков и прав
# ---------------------------------------------------------------------------

def owner_user() -> str:
    """Пользователь, чьё меню приложений/рабочий стол/sudoers мы правим."""
    return session.desktop_user()


def user_home() -> Path:
    user = owner_user()
    if user == session.current_user():
        return Path.home()
    return session.user_home(user)


def _own_to_owner(path: Path) -> None:
    """Отдаёт файл владельцу рабочего стола (актуально при запуске из-под root)."""
    user = owner_user()
    if not session.is_root() or user == "root":
        return
    uid, gid = session.uid_gid(user)
    try:
        os.chown(path, uid, gid)
    except OSError:
        pass


def _write_text(path: Path, text: str, mode: int = 0o644) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    try:
        path.chmod(mode)
    except OSError:
        pass
    _own_to_owner(path)


# ---------------------------------------------------------------------------
# systemd-служба (корневая, системная)
# ---------------------------------------------------------------------------

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
    run_privileged(["bash", "-c", f"cat > {SERVICE_FILE}"], input_text=_unit_text())
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


# ---------------------------------------------------------------------------
# Ярлык приложения (.desktop): меню приложений + рабочий стол
# ---------------------------------------------------------------------------

def applications_dir() -> Path:
    """Каталог пользовательских записей меню приложений."""
    base = os.environ.get("XDG_DATA_HOME") if owner_user() == session.current_user() else ""
    root = Path(base) if base else user_home() / ".local" / "share"
    return root / "applications"


def desktop_dir() -> Path:
    """Рабочий стол пользователя («Desktop» или «Рабочий стол»)."""
    return session.desktop_dir(owner_user())


def desktop_file_path() -> Path:
    return applications_dir() / DESKTOP_NAME


def desktop_icon_path() -> Path:
    return desktop_dir() / DESKTOP_NAME


def icons_dir() -> Path:
    return user_home() / ".local" / "share" / "icons" / "hicolor"


def source_icon_path() -> Path | None:
    for candidate in (app_dir() / "assets" / f"{ICON_NAME}.png",
                      Path(__file__).resolve().parent.parent / "assets" / f"{ICON_NAME}.png"):
        if candidate.exists():
            return candidate
    return None


def install_icon() -> str:
    """Кладёт иконку в пользовательскую тему hicolor — так её видит GNOME/KDE.

    Возвращает значение для строки Icon= (имя темы или абсолютный путь).
    """
    source = source_icon_path()
    if source is None:
        return str(app_dir() / "assets" / f"{ICON_NAME}.png")
    copied = False
    for size in ("256x256", "128x128", "48x48", "scalable"):
        target = icons_dir() / size / "apps" / f"{ICON_NAME}.png"
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            _own_to_owner(target)
            copied = True
        except OSError:
            continue
    return ICON_NAME if copied else str(source)


def refresh_icon_caches() -> None:
    for tool, args in (("update-desktop-database", (str(applications_dir()),)),
                       ("gtk-update-icon-cache", ("-qtf", str(icons_dir())))):
        path = which(tool)
        if not path:
            continue
        try:
            subprocess.run([path, *args], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            pass


def _desktop_text(icon: str) -> str:
    return f"""[Desktop Entry]
Version=1.0
Type=Application
Name={APP_NAME}
GenericName=DPI bypass control
Comment=Обход замедления YouTube, Discord и Telegram (zapret)
Exec={_exec_command()}
Icon={icon}
Terminal=false
Categories=Network;Utility;System;
Keywords=zapret;youtube;discord;telegram;dpi;vpn;
StartupWMClass={APP_SLUG}
StartupNotify=true
"""


def _exec_command() -> str:
    """Команда запуска. От своего пользователя: под root окно всё равно не видно."""
    python = shutil.which("python3") or shutil.which("python") or "python3"
    return f"{python} {run_py_path()} gui"


def desktop_entry_ready(path: Path | None = None) -> bool:
    """Актуален ли ярлык: он про наше приложение и ведёт в текущий каталог кода.

    После переноса установки из /root старый файл .desktop остаётся «на месте»,
    но запускает несуществующий путь — такой ярлык считается битым и пересоздаётся.
    """
    path = path or desktop_file_path()
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    if "Exec=" not in text or APP_NAME not in text:
        return False
    return str(run_py_path()) in text


def shortcut_installed() -> bool:
    return desktop_entry_ready()


def shortcut_status() -> dict:
    """Короткая сводка для интерфейса: где ярлык есть, а где нет."""
    desktop = desktop_icon_path()
    return {"menu": desktop_entry_ready(),
            "menu_path": str(desktop_file_path()),
            "desktop": desktop.exists() and desktop_entry_ready(desktop),
            "desktop_path": str(desktop),
            "desktop_shown": desktop.parent.is_dir()}


def _mark_trusted(path: Path) -> bool:
    """GNOME показывает значок на рабочем столе только у «доверенных» .desktop."""
    try:
        path.chmod(0o755)
    except OSError:
        pass
    gio = which("gio")
    if not gio:
        return False
    try:
        result = subprocess.run([gio, "set", str(path), "metadata::trusted", "true"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def install_shortcut(add_to_desktop: bool = True) -> dict:
    """Создаёт ярлык в меню приложений и (по возможности) на рабочем столе."""
    icon = install_icon()
    entry = _desktop_text(icon)
    menu_path = desktop_file_path()
    _write_text(menu_path, entry)
    placed = {"menu": str(menu_path), "desktop": "", "trusted": False}

    if add_to_desktop:
        target_dir = desktop_dir()
        try:
            target_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            target_dir = None
        if target_dir is not None and target_dir.is_dir():
            copy_path = target_dir / DESKTOP_NAME
            try:
                _write_text(copy_path, entry, mode=0o755)
                placed["desktop"] = str(copy_path)
                placed["trusted"] = _mark_trusted(copy_path)
            except OSError:
                placed["desktop"] = ""
    refresh_icon_caches()
    return placed


def remove_shortcut() -> list[str]:
    """Убирает ярлык из меню приложений и с рабочего стола."""
    removed = []
    for path in (desktop_file_path(), desktop_icon_path()):
        if path.exists():
            try:
                path.unlink()
                removed.append(str(path))
            except OSError:
                continue
    for size in ("256x256", "128x128", "48x48", "scalable"):
        icon = icons_dir() / size / "apps" / f"{ICON_NAME}.png"
        if icon.exists():
            try:
                icon.unlink()
            except OSError:
                pass
    refresh_icon_caches()
    return removed


# ---------------------------------------------------------------------------
# NOPASSWD для nft/nfqws (работа GUI без постоянного ввода пароля)
# ---------------------------------------------------------------------------

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


def permissions_user() -> str:
    """Кому нужны права: живому пользователю сессии, а не root/LOGNAME из sudo."""
    return owner_user()


def setup_permissions(user: str | None = None) -> None:
    # Важно: не getpass.getuser() — под sudo -i это «root», и права
    # доставались бы root, а не человеку, который кликает кнопку.
    user = user or permissions_user()
    if which("sudo") is None:
        raise RuntimeError("sudo не найден. Настройте права вручную.")
    content = _sudoers_text(user)
    if session.is_root():
        p = subprocess.run(["bash", "-c",
                            f"cat > {SUDOERS_FILE} && chmod 440 {SUDOERS_FILE} "
                            f"&& chown root:root {SUDOERS_FILE}"],
                           input=content, text=True)
    else:
        p = subprocess.run(
            ["sudo", "bash", "-c", f"cat > {SUDOERS_FILE} && chmod 440 {SUDOERS_FILE}"],
            input=content, text=True,
        )
    if p.returncode != 0:
        raise RuntimeError("Не удалось настроить sudoers (нужен пароль sudo).")
    # Проверка синтаксиса
    if which("visudo"):
        check = (["visudo", "-c", "-f", SUDOERS_FILE] if session.is_root()
                 else ["sudo", "visudo", "-c", "-f", SUDOERS_FILE])
        subprocess.run(check, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


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
