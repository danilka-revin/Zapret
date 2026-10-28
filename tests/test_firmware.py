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

import math
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
    # Шаги — отдельный таймер (Timer2): USB и печать не сбивают ход мотора.
    assert "setupStepTimer" in core and "ISR(TIMER2_COMPA_vect)" in core


def test_step_timer_does_not_fight_the_servo_library():
    """Шаги не на Timer1/3/4/5: на Mega библиотека Servo занимает их целиком.

    Servo (avr/Servo.cpp) для Mega объявляет обработчики Timer5, Timer1, Timer3
    и Timer4 всегда, даже если подключена одна серва. Свой ``ISR(TIMER1_COMPA)``
    рядом с ней — это «multiple definition of `__vector_17`» и отказ линковки,
    который видит оператор в Arduino IDE. Поэтому шаги идут на Timer2: он
    8-битный, 16 МГц / 8 / 100 = 20 кГц — те же 50 мкс.
    """
    text = _text(WHEEL)
    assert "ISR(TIMER2_COMPA_vect)" in text
    assert "TCCR2B = (1 << CS21)" in text and "OCR2A = 99" in text
    assert "TIMSK2 |= (1 << OCIE2A)" in text
    for vector in ("TIMER1_COMPA_vect", "TIMER3_COMPA_vect",
                   "TIMER4_COMPA_vect", "TIMER5_COMPA_vect"):
        assert "ISR(%s)" % vector not in text, vector
    # Timer2 в Arduino делится только с tone() и analogWrite на D9/D10 —
    # ни того, ни другого в коде нет (скорость — серво-импульс на D3).
    # Проверяем именно код: в комментариях эти имена упоминаются по делу.
    code = "\n".join(line.split("//", 1)[0] for line in text.splitlines())
    assert "analogWrite(" not in code
    assert "tone(" not in code


def test_drive_uses_a_servo_pulse_and_a_five_kmh_ceiling():
    """ШИМ на входе «P» — «серво»-импульс 50 Гц, а сверху потолок 5 км/ч.

    Драйвер RioRand ZS-Z11H понимает на «P» ШИМ 50 Гц…20 кГц: 1500 мкс —
    мотор не тянет, 2000 мкс — верхний ход прошивки. Полной скважности (а с ней
    и 18 км/ч) нет вовсе: библиотека Servo, 50 Гц, импульс 1500…2000 мкс.
    """
    text = _text(WHEEL)
    code = "\n".join(line.split("//", 1)[0] for line in text.splitlines())
    # Библиотека Servo вернулась: заголовок, объект и вызов на D3.
    assert "#include <Servo.h>" in text
    assert "Servo driveServo;" in code
    assert "driveServo.attach(BLDC_SPEED_PIN);" in code
    assert "driveServo.writeMicroseconds(" in code
    assert int(_const(text, "BLDC_SPEED_PIN")) == 3
    # Импульсы: нейтраль 1500 мкс, верхний ход 2000 мкс, частота 50 Гц.
    assert int(_const(text, "DRIVE_NEUTRAL_US")) == 1500
    assert int(_const(text, "DRIVE_FULL_US")) == 2000
    assert int(_const(text, "DRIVE_SERVO_HZ")) == 50
    assert int(_const(text, "DRIVE_FULL_US")) - int(_const(text, "DRIVE_NEUTRAL_US")) == 500
    # Прежнего прямого ШИМ регистрами больше нет: Timer3 ушёл библиотеке Servo.
    for gone in ("TCCR3A", "TCCR3B", "OCR3C", "DRIVE_PWM_HZ"):
        assert gone not in code, gone
    # Формула: процент → импульс 1500…2000 мкс с округлением до микросекунды.
    assert "DRIVE_NEUTRAL_US + ((uint32_t)percent * span + 50UL) / 100UL" in code
    for percent in range(1, 101):
        us = 1500 + (percent * 500 + 50) // 100
        assert 1500 < us <= 2000, percent
        assert abs(us - (1500 + percent * 5)) <= 3, percent
    assert 1500 + (100 * 500 + 50) // 100 == 2000      # 100 % — 2000 мкс
    assert 1500 + (8 * 500 + 50) // 100 == 1540        # 8 % — 1540 мкс

    # Потолок 5 км/ч: колесо 216 мм, 5 км/ч = 1,389 м/с = 2,047 об/с = 122,8 об/мин.
    kmh = float(_const(text, "DRIVE_MAX_KMH"))
    diameter_mm = float(_const(text, "DRIVE_WHEEL_DIAMETER_MM"))
    assert kmh == 5.0 and diameter_mm == 216.0
    rpm = kmh / 3.6 / (math.pi * diameter_mm / 1000.0) * 60.0
    assert abs(rpm - 122.8) < 0.5, rpm
    assert int(_const(text, "DRIVE_MAX_RPM_X10")) == round(rpm * 10)
    # Ограничитель стоит в том же месте, где считается скорость, и умеет только
    # подрезать импульс — не поднимать его выше заказанного.
    assert "updateSpeedGovernor();" in code.split("void updateRpm()")[1]
    assert "rpm_x10 > DRIVE_MAX_RPM_X10" in code
    assert "drive_cap_us < drive_cmd_pulse_us" in code
    # Уровни драйвера: STOP HIGH — выходы включены, LOW — выбег; BRAKE HIGH — тормоз.
    assert _const(text, "BLDC_STOP_ENABLE_LEVEL") == "HIGH"
    assert _const(text, "BLDC_STOP_DISABLE_LEVEL") == "LOW"
    assert _const(text, "BLDC_BRAKE_ACTIVE_LEVEL") == "HIGH"
    # Python-зеркало того же: предел тяги, границы импульса и потолок км/ч.
    mirror = _text(ROOT / "slam_gui" / "backend.py")
    assert re.search(r"^MAX_DRIVE_PERCENT = 100$", mirror, re.M)
    assert re.search(r"^DRIVE_PULSE_MIN_US = 1000$", mirror, re.M)
    assert re.search(r"^DRIVE_PULSE_MAX_US = 2000$", mirror, re.M)
    assert re.search(r"^DRIVE_MAX_KMH_X10 = 50$", mirror, re.M)


def test_tach_counts_edge_intervals_and_rejects_noise():
    """Обороты — по среднему интервалу между переходами «SC», а не по окну.

    Главная жалоба «показывает 3600, а по факту 60» лечится двумя вещами:
    обороты считаются по среднему интервалу (окно 500 мс квантовало медленное
    вращение шагами по 20 %), а невозможные значения — помеха на проводе, а не
    скорость. Тогда прошивка честно говорит «нет» и пишет в журнал, что смотреть.
    """
    text = _text(WHEEL)
    code = "\n".join(line.split("//", 1)[0] for line in text.splitlines())
    assert int(_const(text, "DRIVE_TACH_PPR")) == 90
    assert int(_const(text, "DRIVE_TACH_MIN_EDGE_US")) == 150
    assert int(_const(text, "DRIVE_TACH_MAX_RPM_X10")) == 6000
    assert int(_const(text, "DRIVE_BLIND_MAX_US")) == 1600
    # Интервалы накапливаются в прерывании, обороты — по среднему интервалу.
    assert "tach_interval_sum_us" in code and "tach_interval_count" in code
    assert "600000000ULL * (uint64_t)count" in code
    assert "dt < DRIVE_TACH_MIN_EDGE_US" in code
    # Помеха не превращается в обороты: rpm обнуляется, tach=0, журнал подсказывает.
    assert "rpm > DRIVE_TACH_MAX_RPM_X10" in code
    assert "Проверьте провод SC" in text
    assert "tachTrustedFlag()" in code
    # Формула среднего интервала сходится с жизнью: 90 переходов за секунду — 60 об/мин.
    for per_second in (90, 180, 270):
        mean_us = 1_000_000 / per_second
        rpm = 60_000_000 / (90 * mean_us)
        assert abs(rpm - per_second / 90 * 60) <= 1, (per_second, rpm)
    assert abs(60_000_000 / (90 * (1_000_000 / 90)) - 60) < 0.01


def test_mirrored_wheels_are_fixed_by_the_wheel_name():
    """FR и RR крутятся зеркально: сторона вращения — в прошивке, не настройка.

    Корпус симметричный, поэтому одинаковый уровень DIR на левом и правом борту
    крутит колёса в разные стороны. Прошивка знает об этом по имени колеса
    (MODULE_ID), которое уже вписано в каждую из четырёх копий.
    """
    text = _text(WHEEL)
    code = "\n".join(line.split("//", 1)[0] for line in text.splitlines())
    # Имя колеса — уже готовый признак: FR и RR зеркальные.
    assert 'strcasecmp_P(MODULE_ID, PSTR("FR")) == 0' in code
    assert 'strcasecmp_P(MODULE_ID, PSTR("RR")) == 0' in code
    # Ручная фиксация возможна, но по умолчанию — автоопределение по имени.
    assert "#ifndef DRIVE_MIRRORED" in code and "#define DRIVE_MIRRORED -1" in code
    # «Вперёд» у зеркального колеса — перевёрнутый уровень DIR.
    assert "uint8_t driveForwardLevel()" in code
    assert "const uint8_t direction = forward ? ahead : (ahead == HIGH ? LOW : HIGH);" in code
    # И это единственное место, где решается сторона: команд направления нет.
    assert "PSTR(\"dir\")" not in text
    # Скетчи для рабочего стола по-прежнему различаются только именем колеса.
    script = _text(ROOT / "scripts" / "sync_firmware_desktop.sh")
    assert "MODULE_ID" in script


def test_firmware_has_no_usb_role_discovery():
    text = _text(WHEEL)
    assert 'printIdentity' not in text
    assert '@ID' not in text
    assert 'PSTR("@TLM deg=%s' in text
    assert 'mod=%s' not in text
    # Legacy addressed monitor commands remain compatible; the application
    # sends local commands to the manually selected port.
    assert 'classifyModuleToken' in text
    # v8 — драйвер ZS-Z11H: «серво»-импульс на D3 (вход «P»), линия STOP на D2,
    # тахометр «SC» на D19, потолок 5 км/ч и без настроек привода.
    assert _const(text, 'FIRMWARE_PROTOCOL') == '8'


def test_telemetry_period_and_safety_fields_are_present():
    text = _text(WHEEL)
    assert int(_const(text, "TELEMETRY_PERIOD_MS")) == 250
    # @TLM выходит целиком: обрезанный блок сервер не разберёт.
    assert "if (txFree() >= 210) appendTelemetry();" in text
    # Строка @TLM несёт все поля, которые ждёт сервер (parse_master_line).
    for field in ("deg=", "tgt=", "moving=", "cal=", "cycle=", "calib=",
                  "opto=", "home=", "err=", "rpm=", "drive=", "fw=", "pot=", "pm=", "t=",
                  # тяга драйвера ZS-Z11H: тахометр, предел прошивки,
                  # фактический серво-импульс, переходов «SC» на оборот и
                  # потолок скорости в десятых долях км/ч
                  # hz= — сколько переходов «SC» приходит в секунду: при одном
                  # обороте в секунду это и есть число импульсов на оборот.
                  "tach=", "dmax=", "us=", "ppr=", "kmh=", "hz="):
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
        assert frame["firmware"] == 8 and frame["calibError"] == 0
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
    # FR стоит на 12.5° — но общая тяга на любом угле разрешена, разброс идёт
    # справкой для экрана. Так едут крабом и боком.
    assert status["canDrive"] is True and status["reason"] is None
    assert status["aligned"] is False and status["spreadDeg"] == 12.5
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
