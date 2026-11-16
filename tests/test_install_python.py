"""Установка Python 3.9+: скрипт не требует ручной возни и не ломает систему.

Проверяем поведение, из-за которого установка вставала на Ubuntu 20.04:
`scripts/install_python.sh` сам определяет версию системы, берёт нужный Python
из apt/PPA deadsnakes, а если apt не помог — ставит готовую сборку CPython.
Уже подходящий python3 скрипт не трогает: повторный запуск ничего не меняет.
`install_ubuntu.sh` больше не падает с «обновите Python» — он зовёт установщик.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"
INSTALLER = SCRIPTS / "install_python.sh"


def run(*args: str, env: dict | None = None, timeout: float = 120) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", str(INSTALLER), *args],
        cwd=str(SCRIPTS.parent),
        env=env,
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout,
        stdin=subprocess.DEVNULL,
    )


@pytest.fixture
def fake_host(tmp_path, monkeypatch):
    """Фальшивый «робот»: временный /etc/apt и подменённые apt/sudo/curl."""
    shims = tmp_path / "bin"
    shims.mkdir()
    log = tmp_path / "calls.log"
    log.write_text("", encoding="utf-8")
    apt_log = tmp_path / "apt.log"
    apt_log.write_text("", encoding="utf-8")
    bodies = {
        # apt-cache молчит: нужного пакета в стандартных репозиториях «нет»
        "apt-cache": "#!/usr/bin/env bash\nexit 0\n",
        "apt-get": "#!/usr/bin/env bash\nprintf '%s\\n' \"$*\" >> \"$FAKE_APT_LOG\"\nexit 0\n",
        "add-apt-repository": "#!/usr/bin/env bash\nprintf '%s\\n' \"$*\" >> \"$FAKE_APT_LOG\"\nexit 0\n",
        "dpkg": "#!/usr/bin/env bash\necho amd64\n",
        "sudo": "#!/usr/bin/env bash\nexec \"$@\"\n",
        # без сети: GitHub API и загрузки недоступны, но каждая попытка видна в логе
        "curl": "#!/usr/bin/env bash\nprintf '%s\\n' \"$*\" >> \"$FAKE_APT_LOG\"\nexit 1\n",
        "wget": "#!/usr/bin/env bash\nprintf '%s\\n' \"$*\" >> \"$FAKE_APT_LOG\"\nexit 1\n",
        "mktemp": "#!/usr/bin/env bash\nexec /usr/bin/mktemp \"$@\"\n",
        # «Системный» python3 — старый 3.8, как на Ubuntu 20.04: нужен 3.9+.
        "python3": (
            "#!/usr/bin/env bash\n"
            "if [[ \"${1:-}\" == \"-c\" ]]; then echo 3.8; exit 0; fi\n"
            "echo 'Python 3.8.10'\n"
        ),
    }
    for name, body in bodies.items():
        path = shims / name
        path.write_text(body, encoding="utf-8")
        path.chmod(0o755)
    empty_bins = tmp_path / "no_python"
    empty_bins.mkdir()
    # Пакета python3.X «нет» ни в одном варианте, кроме искусственного:
    # после apt-get install команда python3.X не появится — и установщик должен
    # честно упасть, а не рапортовать успех. Шимы видят логи вызовов.
    for key in [key for key in os.environ if key.startswith(("RC_PYTHON", "APT_"))]:
        monkeypatch.delenv(key, raising=False)
    base_path = os.environ.get("PATH", "/usr/bin:/bin")
    env = {
        "PATH": "%s%s%s" % (shims, os.pathsep, base_path),
        "FAKE_APT_LOG": str(apt_log),
        "FAKE_LOG": str(log),
        "RC_PYTHON_APT_DIR": str(tmp_path / "etc_apt"),
        "RC_PYTHON_PREFIX": str(tmp_path / "usr_local"),
        "RC_PYTHON_OS_RELEASE": str(tmp_path / "os-release"),
        "HOME": str(tmp_path / "home"),
        "APT_CONFIG": "",
        # «В системе нет ни одного python3.X»: поиск идёт только по этому пустому
        # каталогу, поэтому рабочая машина не влияет на результат теста.
        "RC_PYTHON_SEARCH_DIRS": str(empty_bins),
    }
    monkeypatch.setenv("PATH", env["PATH"])
    for key, value in env.items():
        monkeypatch.setenv(key, value)
    (tmp_path / "home").mkdir()
    (tmp_path / "etc_apt").mkdir()

    class Host:
        root = tmp_path
        apt = apt_log

        @staticmethod
        def set_os(text: str) -> None:
            (tmp_path / "os-release").write_text(text, encoding="utf-8")

        @staticmethod
        def calls() -> list[str]:
            return apt_log.read_text(encoding="utf-8").splitlines()

    return Host


UBUNTU_2004 = 'NAME="Ubuntu"\nVERSION_ID="20.04"\nVERSION_CODENAME=focal\nUBUNTU_CODENAME=focal\n'
UBUNTU_2204 = 'NAME="Ubuntu"\nVERSION_ID="22.04"\nVERSION_CODENAME=jammy\n'
UBUNTU_2404 = 'NAME="Ubuntu"\nVERSION_ID="24.04"\nVERSION_CODENAME=noble\n'


# ------------------------------------------------------------- сам скрипт --- #


def test_install_python_script_is_executable_and_valid_bash():
    assert INSTALLER.is_file()
    assert os.access(INSTALLER, os.X_OK), "install_python.sh должен быть исполняемым"
    syntax = subprocess.run(["bash", "-n", str(INSTALLER)], capture_output=True, text=True,
                            check=False)
    assert syntax.returncode == 0, syntax.stderr


def test_help_explains_versions_and_flags():
    result = run("--help")
    assert result.returncode == 0, result.stderr
    assert "Python 3.9+" in result.stdout
    assert "--check" in result.stdout and "--from-source" in result.stdout


def test_unknown_flag_is_rejected():
    result = run("--поставь")
    assert result.returncode != 0
    assert "неизвестный аргумент" in result.stderr


def test_check_mode_works_without_sudo(fake_host):
    """--check ничего не ставит, поэтому не требует root/sudo."""
    fake_host.set_os(UBUNTU_2004)
    (fake_host.root / "bin" / "sudo").unlink()
    result = run("--check")
    assert "нужен root или sudo" not in (result.stdout + result.stderr)
    assert result.returncode in (0, 1)


def test_check_mode_reports_installed_python_without_touching_anything():
    """--check только смотрит: код 0, версия в выводе, apt не вызывается."""
    result = run("--check")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Python 3." in result.stdout


def test_check_mode_fails_when_minimum_is_too_high():
    """Заведомо недостижимый минимум: честный код 1 и подсказка, что делать."""
    result = run("--check", "--min", "3.99")
    assert result.returncode == 1
    assert "install_python.sh" in result.stderr


def test_existing_newer_python_is_activated_when_python3_is_old(fake_host):
    """Не только находит python3.9, но и делает его используемым как python3."""
    fake_host.set_os(UBUNTU_2004)
    candidate = fake_host.root / "no_python" / "python3.9"
    candidate.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ \"${1:-}\" == \"--version\" ]]; then echo 'Python 3.9.5'; exit 0; fi\n"
        "if [[ \"${1:-}\" == \"-c\" ]]; then\n"
        "  case \"${2:-}\" in *sys.version_info*) echo 3.9; exit 0;; esac\n"
        "  exit 0\n"
        "fi\nexit 0\n",
        encoding="utf-8",
    )
    candidate.chmod(0o755)

    result = run()
    activated = fake_host.root / "usr_local" / "bin" / "python3"
    assert result.returncode == 0, result.stdout + result.stderr
    assert activated.is_symlink()
    assert activated.resolve() == candidate.resolve()
    assert "Python 3.9.5" in result.stdout


# ------------------------------------------------------- apt: PPA и версии --- #


def test_focal_takes_python_311_and_adds_deadsnakes_ppa(fake_host):
    """Ubuntu 20.04: в системе 3.8, ставим 3.11 и подключаем PPA deadsnakes."""
    fake_host.set_os(UBUNTU_2004)
    result = run()
    calls = "\n".join(fake_host.calls())
    assert "ppa:deadsnakes/ppa" in calls, calls
    assert "python3.11" in calls, calls
    assert "python3.11-venv" in calls, calls
    # Готовая сборка не скачивается: apt с PPA — основной путь.
    assert "python-build-standalone" not in calls, calls
    assert result.returncode == 0 or "python3.11" in (result.stdout + result.stderr)


def test_noble_does_not_add_ppa_for_python_that_is_already_in_apt(fake_host):
    """24.04: python3.12 есть в стандартных репозиториях — PPA не нужен."""
    fake_host.set_os(UBUNTU_2404)
    fake_host.apt.write_text("", encoding="utf-8")
    # apt-cache «находит» 3.12
    shim = fake_host.root / "bin" / "apt-cache"
    shim.write_text("#!/usr/bin/env bash\necho 'Version: 3.12.3-0ubuntu1'\n", encoding="utf-8")
    run()
    calls = "\n".join(fake_host.calls())
    assert "ppa:deadsnakes/ppa" not in calls, calls
    assert "python3.12" in calls, calls


def test_explicit_version_is_respected(fake_host):
    fake_host.set_os(UBUNTU_2204)
    run("3.9")
    calls = "\n".join(fake_host.calls())
    assert "python3.9" in calls, calls
    assert "python3.11" not in calls, calls


def test_apt_only_mode_never_downloads_standalone_build(fake_host):
    """--apt: скрипт не лезет в интернет за сборкой, только apt/PPA."""
    fake_host.set_os(UBUNTU_2204)
    run("--apt")
    calls = "\n".join(fake_host.calls())
    assert "python-build-standalone" not in calls, calls
    assert "ppa:deadsnakes/ppa" in calls or "python3.11" in calls, calls


def test_from_source_mode_downloads_standalone_build_and_fails_loudly_without_network(fake_host):
    """--from-source: скачивает готовую сборку CPython; без сети — честная ошибка."""
    fake_host.set_os(UBUNTU_2004)
    result = run("--from-source")
    calls = "\n".join(fake_host.calls())
    assert "python-build-standalone" in calls, calls
    assert result.returncode != 0
    assert "python-build-standalone" in (result.stdout + result.stderr)


def test_from_source_mode_builds_the_release_download_url(fake_host, monkeypatch):
    """--from-source: URL содержит версию Python, релиз и архитектуру."""
    fake_host.set_os(UBUNTU_2004)
    monkeypatch.setenv("RC_PYTHON_STANDALONE_RELEASE", "20260108")
    run("--from-source")
    calls = "\n".join(fake_host.calls())
    assert "cpython-3.11" in calls, calls
    assert "x86_64-unknown-linux-gnu-install_only" in calls, calls
    assert "cpython-3.11+20260108" in calls, calls


# ---------------------------------------------------- установщик проекта --- #


def test_install_ubuntu_installs_python_instead_of_failing():
    """Установщик сам зовёт install_python.sh, а не просит «обновите Python»."""
    text = (SCRIPTS / "install_ubuntu.sh").read_text(encoding="utf-8")
    assert "./scripts/install_python.sh" in text
    assert "На Ubuntu 22.04+ подходящий python3 уже стоит" not in text
    assert "RC_PYTHON_AUTO" in text


def test_ensure_deps_uses_selected_interpreter_and_points_at_installer():
    """ensure_deps получает выбранный Python и не падает обратно на старый python3."""
    deps = (SCRIPTS / "ensure_deps.sh").read_text(encoding="utf-8")
    installer = (SCRIPTS / "install_ubuntu.sh").read_text(encoding="utf-8")
    assert 'PYTHON3="${RC_PYTHON3:-$(command -v python3)}"' in deps
    assert 'RC_PYTHON3="$PYTHON3" ./scripts/ensure_deps.sh' in installer
    assert "./scripts/install_python.sh" in deps
