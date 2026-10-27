"""Прошивка колеса motor_wheel: совпадение с проверенным скетчем и симуляция.

* Пины, уровни и скорости берутся из скетча, проверенного на железе
  (firmware/reference/steering_tested) — расхождение ловится здесь, а не на роботе.
* I²C вырезан: каждое колесо — своя плата на своём USB, порты назначает
  оператор; новая телеметрия не содержит роль колеса.
* tests/firmware_sim — хост-симуляция четырёх плат (TB6600, оптодатчик D18):
  калибровка из разных положений, ошибки датчика, команды монитора, тяга и
  watchdog. Нужен g++; без него тест пропускается.
* Сборка настоящим avr-gcc — если есть arduino-cli или AVR_ROOT/CORE_ROOT
  (см. firmware/tools/avr_build.py).
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
WHEEL = ROOT / "firmware" / "motor_wheel" / "motor_wheel.ino"
REFERENCE = ROOT / "firmware" / "reference" / "steering_tested" / "steering_tested.ino"
SIM_DIR = ROOT / "tests" / "firmware_sim"
CORE_BEGIN = "// =========================== STEERING CORE BEGIN ==========================="
CORE_END = "// ============================ STEERING CORE END ============================"
MODULE_IDS = ("FL", "FR", "RL", "RR")


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _core(path: Path) -> str:
    text = _text(path)
    assert text.count(CORE_BEGIN) == 1 and text.count(CORE_END) == 1, path
    return text[text.index(CORE_BEGIN):text.index(CORE_END) + len(CORE_END)]


def _const(text: str, name: str) -> str:
    match = re.search(r"\bconst\s+[\w ]+?\b%s\s*=\s*([^;]+);" % re.escape(name), text)
    assert match, name
    return match.group(1).strip().rstrip("f").rstrip("UL").rstrip("L")


def test_sketch_compiles_for_avr_headers_only():
    """В avr-libc нет <strings.h>: прежняя прошивка из-за него не собиралась."""
    text = _text(WHEEL)
    assert "#include <strings.h>" not in text
    assert "String " not in text          # без кучи: только char-буферы
    assert "Serial.print" not in text     # печать только через неблокирующий буфер
    assert "#include <Wire.h>" not in text and "Wire." not in text  # I²C вырезан
    assert "NODE_ADDRESS" not in text     # адреса шины больше нет


def test_pins_levels_and_speeds_match_the_tested_sketch():
    reference = _text(REFERENCE)
    core = _core(WHEEL)
    assert _const(reference, "PUL") == _const(core, "STEP_PUL_PIN") == "6"
    assert _const(reference, "DIR") == _const(core, "STEP_DIR_PIN") == "7"
    assert _const(reference, "ENA") == _const(core, "STEP_ENA_PIN") == "8"
    assert _const(reference, "OPTO_PIN") == _const(core, "OPTO_PIN") == "18"
    # Потенциометр положения колеса: A0 на всех четырёх платах (Mega: 54),
    # аналоговый вход свободен от D2–D8 и D18.
    assert _const(core, "POT_PIN") == "A0"
    assert int(_const(core, "STATUS_POT"), 0) == 0x80
    assert int(_const(core, "POT_ADC_MAX")) == 1023
    assert float(_const(core, "STEER_TRAVEL_LIMIT_DEG")) == 125.0
    assert _const(reference, "STEP_PERIOD_US") == _const(core, "STEP_PERIOD_US") == "100"
    assert _const(reference, "CALIB_STEP_PERIOD_US") == _const(core, "CALIB_STEP_PERIOD_US") == "2000"
    assert float(_const(reference, "STEPS_PER_DEGREE")) == float(_const(core, "STEPS_PER_DEGREE"))
    # Эталон проверен на железе с поиском вправо 100° / влево 200°. У нового руля
    # потенциометр с упором ±130°: ищем ±125°, чтобы не упираться в его край.
    assert float(_const(reference, "CALIB_SEARCH_DEGREES")) == 100.0
    assert float(_const(core, "CALIB_HALF_RANGE_DEG")) == 125.0
    assert int(_const(core, "CALIB_SLOW_PASSES")) == 3          # подходов на кромку, ноль — среднее
    assert float(_const(core, "CALIB_MAX_EDGE_SPREAD_DEG")) == 2.0
    assert float(_const(reference, "CALIB_SLOW_SEARCH_DEGREES")) == float(
        _const(core, "CALIB_SLOW_SEARCH_DEGREES"))
    assert _const(reference, "OPTO_ACTIVE_LOW") == _const(core, "OPTO_ACTIVE_LOW") == "true"
    # DIR LOW — шаги растут («f»), ENA = HIGH с момента включения, D18 с подтяжкой.
    assert "digitalWrite(DIR, LOW)" in reference and _const(core, "STEP_DIR_PLUS_LEVEL") == "LOW"
    assert "digitalWrite(ENA, HIGH)" in reference and _const(core, "STEP_ENABLE_LEVEL") == "HIGH"
    assert "pinMode(OPTO_PIN, INPUT_PULLUP)" in reference and "pinMode(OPTO_PIN, INPUT_PULLUP)" in core
    # Шаги — Timer1: USB и печать не сбивают ход мотора.
    assert "setupStepTimer" in core and "ISR(TIMER1_COMPA_vect)" in core


def test_firmware_has_no_usb_role_discovery():
    text = _text(WHEEL)
    assert 'printIdentity' not in text
    assert '@ID' not in text
    assert 'PSTR("@TLM deg=%s' in text
    assert 'mod=%s' not in text
    # Legacy addressed monitor commands remain compatible; the application
    # sends local commands to the manually selected port.
    assert 'classifyModuleToken' in text
    # v6 — серво-ШИМ на D3, линия STOP на D2, тахометр на D19.
    assert _const(text, 'FIRMWARE_PROTOCOL') == '6'


def test_telemetry_period_and_safety_fields_are_present():
    text = _text(WHEEL)
    assert int(_const(text, "TELEMETRY_PERIOD_MS")) == 250
    # @TLM выходит целиком: обрезанный блок сервер не разберёт.
    assert "if (txFree() >= 210) appendTelemetry();" in text
    # Строка @TLM несёт все поля, которые ждёт сервер (parse_master_line).
    for field in ("deg=", "tgt=", "moving=", "cal=", "cycle=", "calib=",
                  "opto=", "home=", "err=", "rpm=", "drive=", "fw=", "pot=", "pm=", "t=",
                  # гибкая настройка тяги: тахометр, предел, нейтраль, размах,
                  # тормоз, линия STOP, сторона вращения, длительность импульса
                  "tach=", "dmax=", "trim=", "span=", "brk=", "stp=", "dir=", "us="):
        assert field in text, field


def test_monitor_commands_of_the_tested_sketch():
    text = _text(WHEEL)
    core = _core(WHEEL)
    for command in ("k", "q", "s", "g", "h", "z", "d", "c"):
        assert 'PSTR("%s")' % command in core
    for message in ("Калибровка уже запущена!", "Цель достигнута. Угол:", "ПАУЗА. Угол:",
                    "Шаблон: d [градусы] [f/b]", "Ошибка направления! f или b", "ОБНУЛЕНА.",
                    "ЦИКЛ качания на", "ЦИКЛ ВЫКЛ. Угол:", "Цикл и так выключен.", "Уже в цели."):
        assert message in core, message
    # Тяга — отдельная команда «t»: «d» осталась поворотом, как в проверенном скетче.
    assert 'strcasecmp_P(command, PSTR("t"))' in core
    assert 'strcasecmp_P(command, PSTR("b"))' in core
    # «w» принимает и безымянную форму (с руки), и адресную (от сервера).
    assert 'PSTR("Шаблон: w [FL] 12.5 | w zero | w calib")' in text
    assert 'PSTR("zero"))' in core and 'PSTR("calib"))' in core   # w zero / w calib


def test_calibration_messages_fit_the_line_buffers_in_utf8():
    """Строка монитора режется по буферу: vnote — 176 байт, текст ошибки — 112.

    Кириллица — 2 байта на символ. Обрезка посреди символа отдаёт в USB битый UTF-8
    (так и было при первом варианте длинных сообщений калибровки), поэтому длину
    считаем в байтах, с учётом подстановок и префикса строки.
    """
    note_re = re.compile(r'note\(PSTR\("((?:[^"\\]|\\.)*)"')
    # noteModule(...) добавляет префикс «# RR: » — те же 6 байт, что и «# FL: ».
    module_re = re.compile(r'noteModule\(\s*[^,]+,\s*PSTR\("((?:[^"\\]|\\.)*)"\)')
    text = _text(WHEEL)
    for fmt in note_re.findall(text) + module_re.findall(text):
        fmt = fmt.replace('\\"', '"')
        # Подстановка %s занимает ~8 байт (например «-135.00»), префикс «# FL: » — 6,
        # перевод строки — 2; буфер строки — 176.
        size = len(fmt.encode("utf-8")) + 8 * fmt.count("%") + 6 + 2
        assert size <= 176, (fmt, size)
    block = text[text.index("void copyCalibErrorText"):]
    block = block[:block.index("\n}\n")]
    for fmt in re.findall(r'PSTR\("((?:[^"\\]|\\.)*)"\)', block):
        assert len(fmt.encode("utf-8")) <= 110, fmt


def test_pot_correction_cannot_drag_the_wheel_after_a_lying_sensor():
    """Доводка доворачивает колесо, а не возит его за врущим датчиком.

    Расхождение с целью больше ``POT_TRUST_DEG`` означает, что сбился ноль
    датчика или не сделана ``q``. Раньше доводка догоняла именно датчик и
    утаскивала колесо на десятки градусов от команды («повернулось на 4°
    вместо 35°»), а привязка счётчика шагов к датчику заставляла телеметрию
    отрапортовать успех при колесе совсем не там.
    """
    core = _core(WHEEL)
    trust = float(_const(core, "POT_TRUST_DEG"))
    deadband = float(_const(core, "POT_CORRECTION_DEADBAND_DEG"))
    # Предел доверия обязан быть заметно больше мёртвой зоны, иначе доводка
    # не успеет сделать ни одной полезной поправки.
    assert trust > deadband * 4
    # И заметно меньше хода руля: иначе датчик по-прежнему сможет увезти
    # колесо через полшкалы.
    assert trust < float(_const(core, "STEER_LIMIT_DEG")) / 10
    assert "POT_TRUST_DEG" in core
    # И доводка, и привязка счётчика шагов к датчику обязаны его соблюдать.
    assert core.count("POT_TRUST_DEG") >= 3
    # Большое расхождение — это либо сбившийся ноль датчика, либо
    # заклинившее колесо (пропущенные шаги). Сообщение обязано называть
    # обе причины, а не отправлять оператора перекалибровывать исправный датчик.
    assert "доводка отключена; проверьте колесо или q" in core
    assert "нужна q" not in core


def test_pot_slope_survives_reboot_via_eeprom():
    """Наклон потенциометра обязан переживать перезагрузку.

    Пока ``pot_counts_per_degree`` жил только в RAM, ``pot_calibrated`` после
    каждого включения снова был ``false``, ``potFeedbackAllowed()`` возвращал
    ``false``, и вся доводка (включая защиту от врущего датчика) молча
    выключалась. Колесо ездило вслепую, а пропущенные шаги никто не замечал —
    ровно тот случай, когда «все тесты проходят без потенциометра».
    """
    text, core = _text(WHEEL), _core(WHEEL)
    assert int(_const(core, "EEPROM_POT_MAGIC_SLOT")) + 1 == int(
        _const(core, "EEPROM_POT_SLOPE_SLOT"))
    # Измеренный наклон сохраняется, провалившаяся q — стирается.
    assert "potSlopeToEeprom(slope)" in core
    assert "potSlopeForget()" in core
    # Загрузка обязана проверять правдоподобие, иначе битая ячейка EEPROM
    # включит доводку с мусорным наклоном.
    load = core[core.index("float potSlopeFromEeprom()"):]
    load = load[:load.index("\n}\n")]
    assert "POT_CAL_MIN_COUNTS_PER_DEG" in load and "POT_CAL_MAX_COUNTS_PER_DEG" in load
    assert "EEPROM_POT_MAGIC" in load
    # Загрузка вызывается при старте, иначе персистентность бесполезна.
    assert "potSlopeFromEeprom()" in core[core.index("void potBootNote()"):]
    assert "#include <avr/eeprom.h>" in text and "eeprom_update_byte" in core


def test_steering_core_is_self_contained():
    """Блок руля не зависит от способа связи: USB-хвост живёт за его пределами."""
    text = _text(WHEEL)
    core = _core(WHEEL)
    assert "appendTelemetry" not in core
    assert "handleWheelCommand" not in core
    assert "handleModuleCommand" in core        # команды монитора — внутри ядра
    tail = text[text.index(CORE_END) + len(CORE_END):]
    assert "appendTelemetry" in tail and "handleWheelCommand" in tail
    assert "steeringSetup()" in tail and "steeringLoop()" in tail


def _compiler() -> str | None:
    return shutil.which(os.environ.get("CXX", "g++")) or shutil.which("clang++")


@pytest.fixture(scope="module")
def simulator(tmp_path_factory):
    compiler = _compiler()
    if not compiler:
        pytest.skip("нет g++/clang++ — симуляция прошивок пропущена")
    binary = tmp_path_factory.mktemp("firmware-sim") / "sim"
    result = subprocess.run(
        [compiler, "-std=c++17", "-O1", "-w", "-I", str(SIM_DIR / "stubs"),
         str(SIM_DIR / "sim.cpp"), "-o", str(binary)],
        capture_output=True, text=True, timeout=300,
    )
    assert result.returncode == 0, result.stderr[-4000:]
    return binary


def test_firmware_simulation_scenarios(simulator):
    result = subprocess.run([str(simulator)], capture_output=True, text=True, timeout=600)
    report = result.stdout + "\n" + result.stderr[-6000:]
    assert result.returncode == 0, report
    # Число сценариев меняется вместе с прошивкой: важно, что все прошли.
    summary = re.search(r"(\d+)/(\d+) сценариев прошли", result.stdout)
    assert summary and summary.group(1) == summary.group(2), report


def test_server_parses_simulated_wheel_telemetry(simulator):
    """Строки @TLM настоящей прошивки (из симуляции) понимает сервер."""
    import importlib.util

    spec = importlib.util.spec_from_file_location("slam_backend_fw", ROOT / "slam_gui" / "backend.py")
    backend = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)

    result = subprocess.run([str(simulator), "tlm_dump"], capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stderr[-4000:]
    frames = {}
    messages = []
    module = None
    for line in result.stdout.splitlines():
        if line.startswith("PORT "):
            module = line.split()[1]
            continue
        frame = backend.parse_master_line(line, module=module)
        if frame:
            frames[frame["mod"]] = frame
        elif line.startswith("# "):
            messages.append(line)
    assert set(frames) == set(MODULE_IDS)
    for module, frame in frames.items():
        assert frame["cal"] is True and frame["home"] == "opto", (module, frame)
        assert frame["firmware"] == 6 and frame["calibError"] == 0
        assert frame["drivePercent"] == 0 and frame["rpm"] == 0.0
        assert frame["pot"] is True
    assert frames["FR"]["deg"] == 12.5 and frames["FR"]["moving"] is False
    assert any("FR: Калибровка ЗАВЕРШЕНА!" in line for line in messages)

    import queue
    import threading

    source = object.__new__(backend.MasterSource)
    source.lock = threading.Lock()
    source.frames = {}
    source.drive_percent = 0
    source.drive_last_command = 0.0
    source.commands = queue.Queue(maxsize=4)
    source.serial_ready = threading.Event()
    source.serial_ready.set()
    module = None
    for line in result.stdout.splitlines():
        if line.startswith("PORT "):
            module = line.split()[1]
            continue
        source.feed(line, now=100.0, module=module)
    status = source.drive_status(now=100.2)
    assert status["reason"] == "steering_misaligned"   # FR стоит на 12.5°
    motors = {motor["id"]: motor for motor in source.read(now=100.2)["motors"]}
    assert all(motor["homed"] and motor["online"] for motor in motors.values())
    assert motors["FR"]["homeSource"] == "opto"
    assert motors["FR"]["pot"] is True


def test_server_routes_commands_by_module(simulator):
    """Адресная команда «w FL …» идёт на порт колеса, рассылаемая — на все."""
    import queue
    import threading

    import importlib.util
    spec = importlib.util.spec_from_file_location("slam_backend_fw", ROOT / "slam_gui" / "backend.py")
    backend = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)

    assert backend.MasterSource._command_module(b"w FL 12.5\n") == "FL"
    assert backend.MasterSource._command_module(b"w rr -3.0\n") == "RR"
    assert backend.MasterSource._command_module(b"w FL calib\n") == "FL"
    assert backend.MasterSource._command_module(b"w ALL k\n") is None   # рассылаемая
    assert backend.MasterSource._command_module(b"t 8\n") is None
    # Тяга одного колеса адресуется так же, как «w FL 12.5».
    assert backend.MasterSource._command_module(b"t FL 8\n") == "FL"
    assert backend.MasterSource._command_module(b"t rr -3\n") == "RR"
    assert backend.MasterSource._command_module(b"t ALL 8\n") is None
    # Сторона вращения адресуется так же: имя уходит, на плату летит «m dir 1».
    assert backend.MasterSource._command_module(b"m FL dir 1\n") == "FL"
    assert backend.MasterSource._command_module(b"m rr dir 0\n") == "RR"
    assert backend.MasterSource._command_module(b"m dir 1\n") is None
    assert backend.MasterSource._command_module(b"m ALL dir 1\n") is None
    assert backend.MasterSource._command_module(b"k\n") is None
    assert backend.MasterSource._command_module(b"b\n") is None
    assert not hasattr(backend.MasterSource, '_line_module')


def _avr_toolchain_available() -> bool:
    if shutil.which("arduino-cli"):
        return True
    return bool(os.environ.get("AVR_ROOT")) and bool(os.environ.get("CORE_ROOT"))


@pytest.mark.skipif(not _avr_toolchain_available(),
                    reason="нет arduino-cli и AVR_ROOT/CORE_ROOT — сборка avr-gcc пропущена")
def test_sketch_builds_with_avr_gcc(tmp_path):
    if shutil.which("arduino-cli") and not os.environ.get("AVR_ROOT"):
        command = ["arduino-cli", "compile", "--fqbn", "arduino:avr:mega", "--warnings", "all",
                   str(WHEEL.parent)]
    else:
        command = ["python3", str(ROOT / "firmware" / "tools" / "avr_build.py"), str(WHEEL)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=600)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    assert "warning" not in (result.stdout + result.stderr).lower()
