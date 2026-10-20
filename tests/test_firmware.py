"""Прошивки руля: совпадение с проверенным скетчем, общий блок и симуляция.

* Пины, уровни и скорости берутся из скетча, проверенного на железе
  (firmware/reference/steering_tested) — расхождение ловится здесь, а не на роботе.
* Блок STEERING CORE одинаков в мастере и слейве: правка в одном месте
  без второго — ошибка.
* tests/firmware_sim — хост-симуляция всех четырёх плат (TB6600, оптодатчик,
  I2C): калибровка из разных положений, ошибки датчика, команды монитора,
  тяга и watchdog. Нужен g++; без него тест пропускается.
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
MASTER = ROOT / "firmware" / "motor_master" / "motor_master.ino"
NODE = ROOT / "firmware" / "motor_node" / "motor_node.ino"
REFERENCE = ROOT / "firmware" / "reference" / "steering_tested" / "steering_tested.ino"
SIM_DIR = ROOT / "tests" / "firmware_sim"
CORE_BEGIN = "// =========================== STEERING CORE BEGIN ==========================="
CORE_END = "// ============================ STEERING CORE END ============================"


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


def test_steering_core_is_identical_in_master_and_node():
    assert _core(MASTER) == _core(NODE)


def test_sketches_compile_for_avr_headers_only():
    """В avr-libc нет <strings.h>: прежняя прошивка из-за него не собиралась."""
    for path in (MASTER, NODE):
        text = _text(path)
        assert "#include <strings.h>" not in text
        assert "String " not in text          # без кучи: только char-буферы
        assert "Serial.print" not in text     # печать только через неблокирующий буфер


def test_pins_levels_and_speeds_match_the_tested_sketch():
    reference = _text(REFERENCE)
    core = _core(MASTER)
    assert _const(reference, "PUL") == _const(core, "STEP_PUL_PIN") == "6"
    assert _const(reference, "DIR") == _const(core, "STEP_DIR_PIN") == "7"
    assert _const(reference, "ENA") == _const(core, "STEP_ENA_PIN") == "8"
    assert _const(reference, "OPTO_PIN") == _const(core, "OPTO_PIN") == "18"
    # Потенциометр положения колеса: A0 на всех четырёх платах (Mega: 54),
    # аналоговый вход свободен от D2–D8, D18 и I2C D20/D21.
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
    assert "Wire.setClock(400000)" in reference and _const(_text(MASTER), "I2C_CLOCK_HZ") == "400000"


def test_wire_timeout_is_actually_enabled_on_avr():
    """Таймаут I2C должен доходить до железа, а не вырезаться препроцессором.

    Ядро arduino:avr (проверено на 1.8.8) макрос WIRE_HAS_TIMEOUT не
    определяет — есть только WIRE_HAS_END, а метод setWireTimeout существует
    с ядра 1.8.3. Под `#if defined(WIRE_HAS_TIMEOUT)` вызов молча исчезал, и
    twi оставался с twi_timeout_us = 0: зависшая шина вешала loop() навсегда.
    Симуляция этого не ловила: её заглушка Wire метод имеет в любом случае.
    """
    for path in (MASTER, NODE):
        text = _text(path)
        assert "#if defined(WIRE_HAS_TIMEOUT)" not in text, path
        assert "Wire.setWireTimeout(" in text and ", true)" in text, path
    # На AVR twi_readFrom() отсчитывает таймаут на ВСЮ передачу, а не на байт:
    # 32 байта кадра на 100 кГц — это ~3,2 мс, на самой медленной ступени
    # (31 кГц) — уже ~9,5 мс. Таймаут обязан покрывать всю передачу с запасом,
    # иначе «переход на медленную скорость» обрывал бы нормальный ответ.
    master = _text(MASTER)
    assert "I2C_TIMEOUT_US = 25000" in master
    assert "Wire.setWireTimeout(I2C_TIMEOUT_US, true)" in master
    assert "Wire.setWireTimeout(3000, true)" in _text(NODE)


def test_i2c_bus_can_be_diagnosed_and_slaves_get_their_own_address():
    """«Виден только мастер»: у трёх плат один адрес или шина удерживается.

    Поэтому адрес слейва задаётся с его USB и живёт в EEPROM (одна прошивка на
    все три платы), мастер умеет опросить шину и расклинить зависшую SDA.
    """
    node = _text(NODE)
    master = _text(MASTER)
    assert "#include <avr/eeprom.h>" in node
    assert "eeprom_update_byte" in node
    assert 'PSTR("a")' in node and 'PSTR("addr")' in node
    # «a» не рассылается слейвам — иначе все три получат один адрес.
    assert 'PSTR("a")) == 0 || strcasecmp_P(command, PSTR("addr")) == 0' in master
    assert "i2cScanBus" in master and "i2cRecoverBus" in master
    assert 'PSTR("i2c")' in master
    assert "I2C_SPEED_TIERS" in master


def test_i2c_frame_counter_is_not_packed_into_status_bits():
    """Счётчик кадров — отдельное поле, а не биты 12–15 состояния.

    В v2 он складывался по ИЛИ с битами режима потенциометра (0x1000/0x2000).
    Тогда pm= мигал 0/1 (переключение режима не подтверждалось), а мастер,
    прочитав «тот же» счётчик после паузы, объявлял живой модуль потерянным.
    Тест держит обе стороны: слейв не пакует счётчик в состояние, мастер
    понимает старый кадр (5 полей) и новый (6) и не выдумывает режим из
    испорченных старых битов.
    """
    node = _text(NODE)
    master = _text(MASTER)
    assert "(frame_seq & 0x0F) << 12" not in node
    assert 'PSTR("%ld,%ld,%04X,%lu,%d,%u\\n")' in node
    assert "values[2] & 0x0FFF" in master    # v2: биты 12–15 — счётчик, не режим
    assert "fields != 5 && fields != 6" in master
    assert "slave_protocol" in master


def test_master_adapts_the_i2c_speed_when_not_every_module_answers():
    """400 кГц — не догма: дальняя плата (RR) на них может не отвечать.

    Переход на медленную ступень теперь по лестнице 400 → 100 → 50 → 31 кГц
    (медленнее TWBR не помещается) и срабатывает не только при полном молчании
    шины, но и при «плавающих» потерях: три пропуска в окне последних восьми
    опросов. Счётчик пропусков «подряд» сбрасывается каждым удачным кадром, и
    при кадрах через раз модули мигали «то 4, то 2, то 1» вечно.
    """
    master = _text(MASTER)
    assert "i2cMissingModules" in master
    assert "i2c_auto_slow" in master
    assert "slave_missed" in master
    assert "I2C_SLOW_RETRY_MISSES" in master
    assert "I2C_SPEED_TIERS" in master and "31250UL" in master
    assert "slave_loss8" in master and "I2C_LOSS_LIMIT" in master
    assert "speedDemotionDue" in master and "trySlowerSpeed" in master
    # Диагностика называет молчащий модуль, а не только «ответили не все».
    assert "НЕТ СВЯЗИ: адрес %d не отвечает" in master
    assert "лишний адрес %u" in master


def test_master_holds_module_telemetry_through_short_bus_outages():
    """Одиночная помеха на шине не должна мигать «нет связи» на экране.

    Пока кадр модуля не старше SLAVE_HOLD_MS, мастер шлёт его последние
    значения в @TLM — сервер не объявляет модуль потерянным. Тяга от этого не
    «залёживается»: driveReady смотрит на slaveFresh (настоящий кадр), а не на
    удержание. Журнал при этом не мигает: одна и та же смена состояния
    печатается не чаще I2C_NOTE_REPEAT_MS.
    """
    master = _text(MASTER)
    assert "SLAVE_HOLD_MS" in master and "slaveHeld" in master
    assert "driveReady" in master and "slaveFresh" in master
    assert "I2C_NOTE_REPEAT_MS" in master
    node = _text(NODE)
    # Слейв всегда отдаёт кадр целиком (32 байта): хвост после «\\n» добит
    # нулями, и мастер не читает мусор из старого буфера Wire.
    assert "memset(buffer, 0, sizeof(buffer))" in node


def test_bus_work_is_time_budgeted_and_never_starves_the_loop():
    """Обращения к Wire блокирующие: без бюджета опрос зависшей шины вставал
    поперёк телеметрии и heartbeat тяги.

    На удерживаемой SDA один запрос стоит полного таймаута (25 мс), а опрос
    трёх слейвов с повторами, пробы медленных ступеней и скан 1…119 адресов шли
    одним куском: симуляция показывала 1,34 с на один проход loop(). Сервер
    считает модуль живым 1 с (STALE_S), watchdog тяги ждёт команду 650 мс —
    на экране это выглядело как «шина отвалилась, колёса пропали».
    """
    master = _text(MASTER)
    assert "I2C_LOOP_BUDGET_US = 20000" in master
    assert "poll_cursor" in master                 # недопрошенных допрашиваем в следующий раз
    assert "I2C_SWEEP_BUDGET_US" in master         # скан адресов тоже ограничен по времени
    # Расклинка — по времени молчания, а не по числу опросов: на удерживаемой
    # шине опрос стоит таймаута, и «8 опросов» растягивались больше чем на 3 с.
    assert "I2C_SILENT_BEFORE_RECOVERY_MS = 2000" in master
    assert "I2C_SILENT_ROUNDS_BEFORE_RECOVERY" not in master
    # Самое дорогое (пробы ступеней, скан) — только на стоянке: heartbeat тяги
    # приходит раз в 650 мс, и терять его ради журнала нельзя.
    assert "if (drive_active) return false;" in master
    assert "sweep_requested && !drive_active" in master
    assert "i2cDiagnose(false)" in master and "i2cDiagnose(true)" in master
    # Мёртвой плате хватает одного запроса: четыре попытки по таймауту — 100 мс.
    assert "I2C_RETRY_WHILE_MISSED" in master


def test_master_reports_a_command_the_slave_silently_refused():
    """ACK на шине — это «байты дошли», а не «колесо поехало».

    Слейв отказывает молча: ноль не задан, идёт калибровка, включена тяга или
    прошивка платы старая и режет угол (после подъёма предела до ±90° — ровно
    так выглядит «крутится только одно колесо»). Мастер обязан проверить
    выполнение по кадру и назвать причину.
    """
    master = _text(MASTER)
    assert "expectFromSlave" in master and "confirmSlaveCommand" in master
    assert "COMMAND_CONFIRM_MS" in master
    assert "slave_expect" in master
    assert "команда «%s» не выполнена: %s" in master
    # Причина берётся из первого неподтвердившего кадра: к концу ожидания
    # плата могла освободиться, и объяснение оказалось бы неверным.
    assert "refused_seen" in master
    for reason in ("ноль не задан", "идёт калибровка", "включена тяга", "перепрошить"):
        assert reason in master, reason


def test_live_frame_is_not_only_a_new_counter():
    """Счётчик 4-битный: опрос через ровно 16 кадров видел тот же номер.

    Слейв обновляет кадр каждые 50 мс, поэтому пауза в 800 мс (вокруг
    диагностики шины) давала тот же счётчик — и живой модуль объявлялся
    потерянным. Живым кадр считает смена счётчика ИЛИ любого поля; застывший
    слейв (loop() встал, TWI отдаёт старый буфер) не меняется ничем.
    """
    master = _text(MASTER)
    assert "slaveFrameChanged" in master
    assert "if (seq != slave_seq[i]) {" not in master


def test_master_accepts_monitor_commands_of_the_tested_sketch():
    master = _text(MASTER)
    core = _core(MASTER)
    for command in ("k", "q", "s", "g", "h", "z", "d", "c"):
        assert 'PSTR("%s")' % command in master
    for message in ("Калибровка уже запущена!", "Цель достигнута. Угол:", "ПАУЗА. Угол:",
                    "Шаблон: d [градусы] [f/b]", "Ошибка направления! f или b", "ОБНУЛЕНА.",
                    "ЦИКЛ качания на", "ЦИКЛ ВЫКЛ. Угол:", "Цикл и так выключен.", "Уже в цели."):
        assert message in core, message
    # Тяга — отдельная команда «t»: «d» осталась поворотом, как в проверенном скетче.
    assert 'PSTR("t %d")' in master
    assert '"d %d"' not in master


def test_calibration_messages_fit_the_line_buffers_in_utf8():
    """Строка монитора режется по буферу: vnote — 176 байт, текст ошибки — 112.

    Кириллица — 2 байта на символ. Обрезка посреди символа отдаёт в USB битый UTF-8
    (так и было при первом варианте длинных сообщений калибровки), поэтому длину
    считаем в байтах, с учётом подстановок и префикса строки.
    """
    note_re = re.compile(r'note\(PSTR\("((?:[^"\\]|\\.)*)"')
    # noteModule(...) добавляет префикс «# RR: » — те же 6 байт, что и «# FL: ».
    module_re = re.compile(r'noteModule\(\s*[^,]+,\s*PSTR\("((?:[^"\\]|\\.)*)"')
    for path in (MASTER, NODE):
        text = _text(path)
        for fmt in note_re.findall(text) + module_re.findall(text):
            fmt = fmt.replace('\\"', '"')
            # Подстановка %s занимает ~8 байт (например «-135.00»), префикс «# FL: » — 6,
            # перевод строки — 2; буфер строки — 176.
            size = len(fmt.encode("utf-8")) + 8 * fmt.count("%") + 6 + 2
            assert size <= 176, (path.name, fmt, size)
        block = text[text.index("void copyCalibErrorText"):]
        block = block[:block.index("\n}\n")]
        for fmt in re.findall(r'PSTR\("((?:[^"\\]|\\.)*)"\)', block):
            assert len(fmt.encode("utf-8")) <= 110, (path.name, fmt)


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


def test_server_parses_simulated_master_telemetry(simulator):
    """Строки @TLM настоящей прошивки (из симуляции) понимает сервер."""
    import importlib.util

    spec = importlib.util.spec_from_file_location("slam_backend_fw", ROOT / "slam_gui" / "backend.py")
    backend = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)

    result = subprocess.run([str(simulator), "tlm_dump"], capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stderr[-4000:]
    frames = {}
    messages = []
    for line in result.stdout.splitlines():
        frame = backend.parse_master_line(line)
        if frame:
            frames[frame["mod"]] = frame
        elif line.startswith("# "):
            messages.append(line)
    assert set(frames) == {"FL", "FR", "RL", "RR"}
    for module, frame in frames.items():
        assert frame["cal"] is True and frame["home"] == "opto", (module, frame)
        assert frame["firmware"] == 3 and frame["calibError"] == 0
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
    for line in result.stdout.splitlines():
        source.feed(line, now=100.0)
    status = source.drive_status(now=100.2)
    assert status["reason"] == "steering_misaligned"   # FR стоит на 12.5°
    motors = {motor["id"]: motor for motor in source.read(now=100.2)["motors"]}
    assert all(motor["homed"] and motor["online"] for motor in motors.values())
    assert motors["FR"]["homeSource"] == "opto"
    assert motors["FR"]["pot"] is True


def test_pot_correction_cannot_drag_the_wheel_after_a_lying_sensor():
    """Доводка доворачивает колесо, а не возит его за врущим датчиком.

    Расхождение с целью больше ``POT_TRUST_DEG`` означает, что сбился ноль
    датчика или не сделана ``q``. Раньше доводка догоняла именно датчик и
    утаскивала колесо на десятки градусов от команды («повернулось на 4°
    вместо 35°»), а привязка счётчика шагов к датчику заставляла телеметрию
    отрапортовать успех при колесе совсем не там.
    """
    for path in (MASTER, NODE):
        core = _core(path)
        trust = float(_const(core, "POT_TRUST_DEG"))
        deadband = float(_const(core, "POT_CORRECTION_DEADBAND_DEG"))
        # Предел доверия обязан быть заметно больше мёртвой зоны, иначе доводка
        # не успеет сделать ни одной полезной поправки.
        assert trust > deadband * 4, path.name
        # И заметно меньше хода руля: иначе датчик по-прежнему сможет увезти
        # колесо через полшкалы.
        assert trust < float(_const(core, "STEER_LIMIT_DEG")) / 10, path.name
        assert "POT_TRUST_DEG" in core, path.name
        # И доводка, и привязка счётчика шагов к датчику обязаны его соблюдать.
        assert core.count("POT_TRUST_DEG") >= 3, path.name
        assert "доводка отключена, нужна q" in core, path.name

    # Логика одинакова в обеих прошивках: мастер и слейв крутят колёса одним кодом.
    assert _core(MASTER).count("POT_TRUST_DEG") == _core(NODE).count("POT_TRUST_DEG")


def _avr_toolchain_available() -> bool:
    if shutil.which("arduino-cli"):
        return True
    return bool(os.environ.get("AVR_ROOT")) and bool(os.environ.get("CORE_ROOT"))


@pytest.mark.skipif(not _avr_toolchain_available(),
                    reason="нет arduino-cli и AVR_ROOT/CORE_ROOT — сборка avr-gcc пропущена")
@pytest.mark.parametrize("sketch", [MASTER, NODE])
def test_sketch_builds_with_avr_gcc(sketch, tmp_path):
    if shutil.which("arduino-cli") and not os.environ.get("AVR_ROOT"):
        command = ["arduino-cli", "compile", "--fqbn", "arduino:avr:mega", "--warnings", "all",
                   str(sketch.parent)]
    else:
        command = ["python3", str(ROOT / "firmware" / "tools" / "avr_build.py"), str(sketch)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=600)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    assert "warning" not in (result.stdout + result.stderr).lower()
