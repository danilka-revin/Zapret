// motor_wheel — прошивка ОДНОГО колеса руля (Arduino Mega 2560 / Mega Mini).
// Каждая плата подключена отдельным USB-кабелем (115200 бод).
// Порт для каждого колеса оператор выбирает вручную в программе.
// MODULE_ID задаёт имя в журнале, фильтр адресных команд и зеркалирование DIR.
// Программа назначает порты вручную и посылает локальные команды (w 12.5);
// @TLM раз в 250 мс не определяет роль порта. При прошивке укажите имя колеса.
//
// Команды с USB (как в проверенном скетче): k, q, c, d, h, s, g, z, p, w, v, t, b.
//   w 12.5 | w FL 12.5 — поворот колеса; w zero / w calib — ноль и калибровка;
//   v N [f/b] — прямое значение Servo.write для тяги (0…150) и направление;
//   t N — прежняя команда тяги BLDC ±N% для других пультов;
//   обе команды повторяются чаще 650 мс, иначе плата сама тормозит.
//   b — тормоз; p — состояние; ? — справка.
//
// Выход на регулятор скорости — «PWM» драйвера RioRand ZS-Z11H (hoverboard,
// разбор входов: mad-ee.com/easy-inexpensive-hoverboard-motor-controller).
// Вход «P» рассчитан на ШИМ по скважности. По требованию оператора используется
// штатная Servo: экран передаёт целое 0…150 напрямую в Servo.write() на D3.
// При стандартных настройках Servo импульс 544…2400 мкс повторяется каждые
// ~20 мс: это лишь 2,7…12 % скважности, не весь диапазон входа P. Значение 0
// выключает тягу линиями BRAKE/STOP, а не нулевым импульсом. Фактические
// обороты не измеряются. Прежняя t оставлена старым пультам и манёврам.
//   * P (D3) — сигнал Servo, около 50 Гц; при команде 0 драйвер выключен;
//   * DIR — уровень: для этого колеса «вперёд» (у FR и RR он перевёрнут —
//     колёса стоят зеркально), инверсия — назад;
//   * BRAKE — 1 = тормоз включён, 0 = отпущен;
//   * STOP — 1 = драйвер разрешён, 0 = выходы выключены (свободный выбег).
// Всё это — константы прошивки, а не настройки: подбирать на экране нечего.
// Если конкретное колесо крутится не в ту сторону — поменяйте местами два
// фазных провода мотора или константы уровней ниже.
//
// Считывание тахометра SC пока отключено: обороты не измеряются и не влияют
// на тягу. Для проверки мотора нужен независимый физический аварийный выключатель.
//
// На каждой плате: свой TB6600 (шаговый руль), оптодатчик нуля на D18,
// потенциометр положения колеса на A0 и интерфейс BLDC-тяги:
// DIR (D4), BRAKE (D5), STOP (D2), SPEED/ШИМ (D3) и общая земля.
// Сигнал на D3 формирует штатная библиотека Servo.
// D19 (SC) сейчас не считывается и не настраивается как прерывание.
#include <Servo.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Имя колеса задаёт зеркалирование DIR; USB-порт оператор назначает вручную.
// Скрипт sync_firmware_desktop.sh подставляет сюда FL/FR/RL/RR.
#ifndef MODULE_ID
#define MODULE_ID "FL"
#endif
// Буфер ответов в USB: «p» печатает состояние, сведения о приводе и имя
// колеса; 640 байт оставляют запас, чтобы последняя строка не терялась.
#define TX_BUFFER_SIZE 640

// =========================== STEERING CORE BEGIN ===========================
// Этот блок — руль и датчики, он не зависит от способа связи с сервером.
//
// Руль повторяет скетч, проверенный на железе (firmware/reference/steering_tested):
// те же пины и уровни, 133.33 шага/°, 100 мкс на шаг в работе и 2000 мкс при
// точном подходе к оптодатчику D18, те же команды монитора (k c d h s g p z).
// Отличия от него:
//   * шаги выдаёт таймер (тик 50 мкс), а не loop(): чтение USB и печать
//     не сбивают ход мотора;
//   * датчик проверяется в том же прерывании — кромка ловится с точностью до
//     шага, без запаздывания loop();
//   * калибровка доведена: выход из флажка перед медленным подходом, поиск
//     влево действительно включается, ноль ставится без зависаний, есть
//     таймаут и отмена командой s.

// ---- распиновка (одинакова на всех четырёх платах) ----
// STOP — выход на драйвер. Вход тахометра SC на D19 временно не используется.
const uint8_t BLDC_STOP_PIN = 2;     // BLDC STOP — вход «разрешить» драйвера
const uint8_t BLDC_SPEED_PIN = 3;    // сигнал Servo на вход «P» драйвера
const uint8_t BLDC_DIR_PIN = 4;      // BLDC DIR
const uint8_t BLDC_BRAKE_PIN = 5;    // BLDC BRAKE
const uint8_t STEP_PUL_PIN = 6;      // TB6600 PUL
const uint8_t STEP_DIR_PIN = 7;      // TB6600 DIR
const uint8_t STEP_ENA_PIN = 8;      // TB6600 ENA
const uint8_t OPTO_PIN = 18;         // оптодатчик нуля руля (INPUT_PULLUP)
const uint8_t POT_PIN = A0;          // потенциометр положения колеса (ADC0; на Mega это вывод 54)

// ---- уровни сигналов (как в проверенном скетче) ----
const uint8_t STEP_DIR_PLUS_LEVEL = LOW;       // LOW — «вправо» (f), шаги растут
const uint8_t STEP_DIR_MINUS_LEVEL = HIGH;     // HIGH — «влево» (b), шаги убывают
const uint8_t STEP_ENABLE_LEVEL = HIGH;        // ENA = HIGH сразу при включении и всегда
const bool OPTO_ACTIVE_LOW = true;             // датчик сработал → на D18 LOW
// Уровни — по документации драйвера (RioRand ZS-Z11H, см. шапку файла):
// BRAKE активен высоким, DIR меняет сторону низким, STOP разрешает высоким.
// Пока тяги нет (старт платы, «b», watchdog), драйвер выключен, тормоз взведён.
const uint8_t BLDC_BRAKE_ACTIVE_LEVEL = HIGH;  // BRAKE = 1 → мотор заторможен
const uint8_t BLDC_DIR_FORWARD_LEVEL = HIGH;   // DIR = 1 (в покое) — штатное вращение
const uint8_t BLDC_STOP_ENABLE_LEVEL = HIGH;   // STOP = 1 → драйвер разрешён
const uint8_t BLDC_STOP_DISABLE_LEVEL = LOW;   // STOP = 0 → выходы выключены (выбег)

// ---- механика и скорость руля ----
const float STEPS_PER_DEGREE = 133.333333f;    // шагов_на_оборот × микрошаг × редуктор / 360
const uint16_t STEP_PERIOD_US = 100;           // рабочий ход: 10 кГц ≈ 75°/с
const uint16_t CALIB_STEP_PERIOD_US = 2000;    // точный подход к датчику: 500 Гц ≈ 3,75°/с
// Предел абсолютной команды «w <угол>». ОДНО число на весь стек: столько же
// в slam_gui/backend.py (STEER_LIMIT_DEG / MasterSource.MAX_STEER_DEG),
// в robot_control/slam.py и в экранах (slam_gui/steer_presets.js). 90° дают
// чистый боковой ход, а 100° — запас для диагоналей на неровном полу; до упора
// потенциометра (±130°) и до программного предела хода
// STEER_TRAVEL_LIMIT_DEG (±125°) запас остаётся.
const float STEER_LIMIT_DEG = 100.0f;
const float MANUAL_MOVE_LIMIT_DEG = 360.0f;    // предел одной ручной команды d / c
const float STEER_TRAVEL_LIMIT_DEG = 125.0f;   // предел хода от нуля: упор потенциометра ±130°

// ---- калибровка по оптодатчику (команда k) ----
const float CALIB_HALF_RANGE_DEG = 125.0f;     // поиск вправо 125°, затем влево 250°: 5° запаса до упора потенциометра ±130°
const float CALIB_SLOW_SEARCH_DEGREES = 20.0f; // медленный подход к кромке — не дальше
const uint8_t CALIB_SLOW_PASSES = 3;           // подходов к КАЖДОЙ кромке; ноль — среднее по всем подходам
const float CALIB_MAX_EDGE_SPREAD_DEG = 2.0f;  // разброс кромок между подходами больше — ошибка 6, ноль не ставим
const float CALIB_CLEARANCE_DEGREES = 2.0f;    // отъезд за кромку перед медленным подходом
const float CALIB_MAX_FLAG_DEGREES = 30.0f;    // шире — датчик «залип» или ошибка проводки
// true: ноль = середина между кромками, найденными медленным подходом справа и
// слева (так задумано в проверенном скетче: «Точка 0 = (right + left) / 2»).
// false: ноль = кромка при подходе вправо — так скетч работал фактически,
// потому что медленные фазы сразу срабатывали на той же кромке.
const bool CALIB_ZERO_AT_CENTER = true;
const float CALIB_ZERO_OFFSET_DEG = 0.0f;      // поправка, если «прямо» ≠ нулю датчика
const uint32_t CALIB_TIMEOUT_MS = 120000UL;    // страховка: дольше — ошибка (3 подхода на кромку)
const uint8_t OPTO_DEBOUNCE_TICKS = 3;         // 3 тика по 50 мкс подряд = срабатывание

// ---- потенциометр положения колеса (команда q) ----
// Линейный 10 кОм, 270°: крайние выводы — 5 В и GND платы, движок — A0.
// Питание от 5 В той же платы: измерение ратиометрическое, просадка 5 В не
// меняет угол. A0 свободен на всех четырёх платах (заняты только D2–D8 и D18).
const int16_t POT_ADC_MAX = 1023;
const int16_t POT_RAIL_MARGIN = 4;               // 0…4 и 1019…1023 — обрыв, КЗ или упор
const float POT_NOMINAL_COUNTS_PER_DEG = 1023.0f / 270.0f;  // 270° на весь ход
const uint16_t POT_SAMPLE_PERIOD_MS = 10;        // один отсчёт АЦП раз в 10 мс
const uint8_t POT_FILTER_SHIFT = 2;              // EMA 1/4: шум АЦП падает вчетверо
const float POT_CORRECTION_DEADBAND_DEG = 0.7f;  // меньше — мотор не дёргаем
const uint8_t POT_CORRECTION_MAX_PASSES = 3;     // поправок на один поворот
const uint16_t POT_CORRECTION_SETTLE_MS = 60;    // механика успокаивается после стопа
const float POT_RESYNC_DEG = 0.25f;              // счётчик шагов привязать к датчику
// Насколько датчику вообще позволено спорить с целью. Доводка убирает
// люфт и увод на единицы градусов; расхождение в десятки градусов означает,
// что врёт датчик (сбился ноль, изношена дорожка, не сделана q), а не колесо.
const float POT_TRUST_DEG = 5.0f;
const float POT_CAL_POINT_DEG = 60.0f;           // автокалибровка наклона: −60° и +60°
const float POT_CAL_MIN_COUNTS_PER_DEG = 1.5f;   // проверка правдоподобия наклона
const float POT_CAL_MAX_COUNTS_PER_DEG = 8.0f;
const uint32_t POT_CAL_TIMEOUT_MS = 30000UL;

// Наклон потенциометра — свойство железа, а не сессии: между включениями он не
// меняется. Без сохранения доводка после каждой перезагрузки молча выключалась
// (pot_calibrated = false), колесо снова ехало вслепую, и пропущенные шаги
// никто не замечал. Слоты 0…1 свободны.
const uint16_t EEPROM_POT_MAGIC_SLOT = 2;
const uint16_t EEPROM_POT_SLOPE_SLOT = 3;   // 4 байта, float, little-endian
const uint8_t EEPROM_POT_MAGIC = 0x51;      // «q» — наклон измерен

// ---- тяга BLDC: драйвер RioRand ZS-Z11H (hoverboard-мотор) ----
// Штатная библиотека Servo выдаёт на вход P сигнал около 50 Гц. Значение
// write() — число из шкалы Servo, а не измеренные обороты и не скважность P.
// При команде 0 драйвер выключается по BRAKE/STOP; контура оборотов нет.
const uint8_t DRIVE_SERVO_MAX_VALUE = 180;      // шкала прежней команды t
const uint8_t DRIVE_DIRECT_MAX_VALUE = 150;     // прямое значение с бортового экрана
// 1 % = 1,8 значения шкалы (180 / 100). В целых десятых, чтобы не тащить float
// в горячую часть: 18 десятых на процент.
const uint16_t DRIVE_SERVO_TENTHS_PER_PERCENT = 18;
// Предел прежней команды t — 100 % шкалы Servo, а не скважность и не обороты:
// 100 % = значение write(180), 1 % ≈ write(1,8). Константа прошивки.
const uint8_t DRIVE_MAX_PERCENT = 100;
const uint16_t DRIVE_COMMAND_TIMEOUT_MS = 650; // нет heartbeat → значение 0 и тормоз
// Зеркальные колёса. Корпус симметричный, поэтому правое колесо при том же
// уровне DIR крутится в другую сторону относительно робота. По умолчанию
// прошивка сама понимает это по имени колеса: FR и RR — зеркальные.
// Значение можно зафиксировать вручную: 1 — зеркалить всегда, 0 — никогда.
#ifndef DRIVE_MIRRORED
#define DRIVE_MIRRORED -1
#endif

// v15: v принимает направление и точное значение 0…150; SC не считывается.
// t остаётся совместимой со старыми пультами и манёврами.
const uint8_t FIRMWARE_PROTOCOL = 15;          // поле fw= в @TLM

// ---- состояние руля (общие с прерыванием поля — volatile) ----
volatile int32_t current_step = 0;
volatile int32_t target_step = 0;
volatile bool is_moving = false;
volatile bool pulse_high = false;
volatile int8_t step_dir = 1;
volatile uint8_t half_period_ticks = 1;
volatile uint8_t tick_count = 0;

const uint8_t OPTO_WATCH_NONE = 0;
const uint8_t OPTO_WATCH_ACTIVE = 1;   // остановиться, когда датчик сработает
const uint8_t OPTO_WATCH_CLEAR = 2;    // остановиться, когда датчик освободится
volatile uint8_t opto_watch = OPTO_WATCH_NONE;
volatile uint8_t opto_match = 0;
volatile bool opto_event = false;
volatile int32_t opto_event_step = 0;

const uint8_t HOME_NONE = 0;
const uint8_t HOME_MANUAL = 1;
const uint8_t HOME_OPTO = 2;

const uint8_t CAL_IDLE = 0;
const uint8_t CAL_FAST_PLUS = 1;       // быстрый поиск вправо
const uint8_t CAL_FAST_MINUS = 2;      // быстрый поиск влево
const uint8_t CAL_EXIT_PLUS = 3;       // из флажка вправо, пока датчик не освободится
const uint8_t CAL_CLEAR_PLUS = 4;      // ещё запас вправо
const uint8_t CAL_SLOW_MINUS = 5;      // медленно влево до срабатывания: кромка справа
const uint8_t CAL_EXIT_MINUS = 6;      // из флажка влево
const uint8_t CAL_CLEAR_MINUS = 7;     // ещё запас влево
const uint8_t CAL_SLOW_PLUS = 8;       // медленно вправо до срабатывания: кромка слева
const uint8_t CAL_GO_ZERO = 9;         // на найденный ноль

const uint8_t CAL_OK = 0;
const uint8_t CAL_ERR_NOT_FOUND = 1;   // датчик не сработал в зоне поиска
const uint8_t CAL_ERR_STUCK = 2;       // датчик не освобождается
const uint8_t CAL_ERR_EDGE = 3;        // кромка не найдена при медленном подходе
const uint8_t CAL_ERR_ABORTED = 4;     // прервано командой s
const uint8_t CAL_ERR_TIMEOUT = 5;     // дольше CALIB_TIMEOUT_MS
const uint8_t CAL_ERR_SPREAD = 6;      // кромки между подходами расходятся больше CALIB_MAX_EDGE_SPREAD_DEG

// Биты состояния модуля: поля телеметрии @TLM.
const uint16_t STATUS_MOVING = 0x01;
const uint16_t STATUS_ZERO = 0x02;
const uint16_t STATUS_CALIB = 0x04;
const uint16_t STATUS_OPTO = 0x08;
const uint16_t STATUS_CYCLE = 0x10;
const uint16_t STATUS_HOME_OPTO = 0x20;
const uint16_t STATUS_V2 = 0x40;       // признак кадра этой версии
const uint16_t STATUS_POT_MODE = 0x1000;  // режим с потенциометром
const uint16_t STATUS_POT_MODE_SUPPORTED = 0x2000;
const uint16_t STATUS_POT = 0x80;      // потенциометр откалиброван, доводка включена
// биты 8–11: код ошибки калибровки; биты 12–15 — режим потенциометра.
// Счётчик кадров в состояние НЕ входит: в v3 это отдельное поле кадра.

bool zero_set = false;
uint8_t home_source = HOME_NONE;
uint8_t calib_state = CAL_IDLE;
uint8_t calib_error = CAL_OK;
uint32_t calib_started_ms = 0;
int32_t calib_edges_plus[CALIB_SLOW_PASSES];   // кромки при подходе справа (движение влево), по подходам
int32_t calib_edges_minus[CALIB_SLOW_PASSES];  // кромки при подходе слева (движение вправо), по подходам
uint8_t calib_plus_count = 0;
uint8_t calib_minus_count = 0;
bool cycle_active = false;
bool cycle_to_end = true;
int32_t cycle_start_step = 0;
int32_t cycle_end_step = 0;
bool move_report_pending = false;      // напечатать «Цель достигнута» после поворота
bool resume_allowed = false;           // g продолжит поворот, прерванный командой s

bool drive_active = false;
int8_t drive_percent = 0;
uint32_t last_drive_command_ms = 0;

// Тяга: какие проценты просили и какое значение сейчас на входе «P».
// Значение — шкала Servo.write: 0…DRIVE_SERVO_MAX_VALUE (0 — мотор не тянет).
uint8_t drive_servo_value = 0;     // что реально написано в Servo.write
bool drive_mirrored = false;       // это колесо стоит зеркально

char tx_buf[TX_BUFFER_SIZE];
uint16_t tx_length = 0;
uint16_t tx_position = 0;

// ---------------------------------------------------------------------------
// Вывод в USB без блокировки loop(): строка копируется в буфер и уходит в порт
// по мере освобождения передатчика. Нет места — строка пропускается.
void txAppend(const char* data, uint16_t length) {
  if (tx_position >= tx_length) {
    tx_position = 0;
    tx_length = 0;
  }
  if (length == 0 || (uint32_t)tx_length + length > sizeof(tx_buf)) return;
  memcpy(tx_buf + tx_length, data, length);
  tx_length += length;
}

uint16_t txFree() {
  if (tx_position >= tx_length) return sizeof(tx_buf);
  return (uint16_t)(sizeof(tx_buf) - tx_length);
}

void txPump() {
  while (tx_position < tx_length && Serial.availableForWrite() > 0) {
    Serial.write((uint8_t)tx_buf[tx_position++]);
  }
}

// Сообщение для человека: «# FL: текст». Телеметрия начинается с @TLM, поэтому
// сервер такие строки не путает с данными, а пишет в журнал бортового экрана.
void vnote(const char* module, const char* format_P, va_list args) {
  char line[176];
  int n = snprintf_P(line, sizeof(line) - 2, PSTR("# %s: "), module);
  if (n < 0) return;
  if (n > (int)sizeof(line) - 3) n = (int)sizeof(line) - 3;
  const int m = vsnprintf_P(line + n, sizeof(line) - 2 - (size_t)n, format_P, args);
  if (m < 0) return;
  n += m;
  if (n > (int)sizeof(line) - 3) n = (int)sizeof(line) - 3;
  line[n++] = '\r';
  line[n++] = '\n';
  txAppend(line, (uint16_t)n);
}

void noteModule(const char* module, const char* format_P, ...) {
  va_list args;
  va_start(args, format_P);
  vnote(module, format_P, args);
  va_end(args);
}

void note(const char* format_P, ...) {
  va_list args;
  va_start(args, format_P);
  vnote(MODULE_ID, format_P, args);
  va_end(args);
}

// value в десятых (decimals = 1) или сотых (decimals = 2) → "-12.50"
void formatFixed(long value, uint8_t decimals, char* out, size_t out_size) {
  const unsigned long scale = decimals == 1 ? 10UL : 100UL;
  const unsigned long magnitude = value < 0 ? (unsigned long)(-value) : (unsigned long)value;
  if (decimals == 1) {
    snprintf_P(out, out_size, PSTR("%s%lu.%01lu"), value < 0 ? "-" : "",
               magnitude / scale, magnitude % scale);
  } else {
    snprintf_P(out, out_size, PSTR("%s%lu.%02lu"), value < 0 ? "-" : "",
               magnitude / scale, magnitude % scale);
  }
}

void formatDegrees(int32_t steps, char* out, size_t out_size) {
  formatFixed(lroundf((float)steps * 100.0f / STEPS_PER_DEGREE), 2, out, out_size);
}

int32_t degreesToSteps(float degrees) {
  return (int32_t)lroundf(degrees * STEPS_PER_DEGREE);
}

bool parseFloatToken(const char* text, float* out) {
  if (!text || !*text) return false;
  char* end = nullptr;
  const double value = strtod(text, &end);
  if (end == text || *end != '\0' || !isfinite(value)) return false;
  *out = (float)value;
  return true;
}

bool parseLongToken(const char* text, long* out) {
  if (!text || !*text) return false;
  char* end = nullptr;
  const long value = strtol(text, &end, 10);
  if (end == text || *end != '\0') return false;
  *out = value;
  return true;
}

// ---------------------------------------------------------------------------
// Шаги: Timer2 в режиме CTC, 16 МГц / 8 / 100 = 20 кГц, тик 50 мкс.
// Тяга идёт через библиотеку Servo (на Mega она берёт Timer5), руль — Timer2:
// таймеры не пересекаются.
bool optoActive() {
  const int level = digitalRead(OPTO_PIN);
  return OPTO_ACTIVE_LOW ? level == LOW : level == HIGH;
}

void setupStepTimer() {
  noInterrupts();
  TCCR2A = (1 << WGM21);            // CTC: сброс счётчика по совпадению с OCR2A
  TCCR2B = (1 << CS21);             // предделитель 8: 16 МГц / 8 / 100 = 20 кГц
  OCR2A = 99;                       // 100 тиков по 50 мкс, ровно как раньше
  TCNT2 = 0;
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}

// Полпериода шага = STEP_PERIOD_US / 100 тиков: 100 мкс → 50 мкс HIGH и 50 мкс
// LOW, ровно 10 кГц, как micros()-генератор проверенного скетча. Шаг считается
// по спаду PUL; начатый импульс при остановке доводится до конца и засчитывается.
ISR(TIMER2_COMPA_vect) {
  if (opto_watch != OPTO_WATCH_NONE) {
    if (optoActive() == (opto_watch == OPTO_WATCH_ACTIVE)) {
      if (++opto_match >= OPTO_DEBOUNCE_TICKS) {
        opto_watch = OPTO_WATCH_NONE;
        opto_event = true;
        opto_event_step = current_step + (pulse_high ? step_dir : 0);
        is_moving = false;
      }
    } else {
      opto_match = 0;
    }
  }
  if (!is_moving) {
    tick_count = 0;
    if (pulse_high) {
      digitalWrite(STEP_PUL_PIN, LOW);
      pulse_high = false;
      current_step += step_dir;
    }
    return;
  }
  if (current_step == target_step) {
    is_moving = false;
    tick_count = 0;
    return;
  }
  if (++tick_count < half_period_ticks) return;
  tick_count = 0;
  if (!pulse_high) {
    digitalWrite(STEP_PUL_PIN, HIGH);
    pulse_high = true;
  } else {
    digitalWrite(STEP_PUL_PIN, LOW);
    pulse_high = false;
    current_step += step_dir;
    if (current_step == target_step) is_moving = false;
  }
}

int32_t stepNow() {
  noInterrupts();
  const int32_t value = current_step;
  interrupts();
  return value;
}

int32_t targetNow() {
  noInterrupts();
  const int32_t value = target_step;
  interrupts();
  return value;
}

bool motorBusy() {
  noInterrupts();
  const bool busy = is_moving || pulse_high;
  interrupts();
  return busy;
}

// Остановить ход из loop(). Начатый импульс прерывание доведёт за ≤ 50 мкс.
void haltMotor() {
  noInterrupts();
  is_moving = false;
  opto_watch = OPTO_WATCH_NONE;
  interrupts();
  for (uint8_t i = 0; i < 30 && motorBusy(); ++i) delayMicroseconds(10);
}

void startMotor(int32_t target, uint16_t period_us, uint8_t watch) {
  haltMotor();
  const int32_t current = stepNow();
  const int8_t direction = target >= current ? 1 : -1;
  digitalWrite(STEP_ENA_PIN, STEP_ENABLE_LEVEL);
  digitalWrite(STEP_DIR_PIN, direction > 0 ? STEP_DIR_PLUS_LEVEL : STEP_DIR_MINUS_LEVEL);
  delayMicroseconds(10);  // DIR раньше первого фронта PUL (TB6600: не меньше 5 мкс)
  uint16_t ticks = (uint16_t)((period_us + 50U) / 100U);
  if (ticks < 1) ticks = 1;
  if (ticks > 255) ticks = 255;
  noInterrupts();
  step_dir = direction;
  half_period_ticks = (uint8_t)ticks;
  tick_count = 0;
  target_step = target;
  opto_match = 0;
  opto_event = false;
  opto_watch = watch;
  is_moving = target != current;
  interrupts();
}

// ---------------------------------------------------------------------------
// Потенциометр положения колеса — абсолютный датчик угла на A0.
//
// Ноль запоминается автоматически, когда задан ноль руля (k или z). Команда q
// измеряет наклон: колесо едет в 0°, −60° и +60°, по двум точкам считается
// «отсчётов АЦП на градус». Только после q включается доводка: после каждого
// поворота прошивка сравнивает угол с потенциометром и делает до
// POT_CORRECTION_MAX_PASSES поправок. Без q датчик показывает угол по номиналу
// 1023/270, но в управление не вмешивается.
const uint16_t POT_CAL_SETTLE_MS = 120;   // механика успокаивается после остановки
const uint8_t POT_CAL_SAMPLES = 8;        // отсчётов АЦП на точку калибровки

const uint8_t POT_CAL_IDLE = 0;
const uint8_t POT_CAL_TO_ZERO = 1;        // в 0° перед замерами
const uint8_t POT_CAL_TO_MINUS = 2;       // в −60°
const uint8_t POT_CAL_TO_PLUS = 3;        // в +60°
const uint8_t POT_CAL_BACK = 4;           // обратно в 0°

int16_t pot_raw = 0;                      // фильтрованное значение АЦП 0…1023
bool pot_enabled = true;                 // pm 0: управление только по шагам
bool pot_sampled = false;
bool pot_railed = false;                  // 0/1023 — обрыв, КЗ или упор
bool pot_zero_valid = false;              // есть привязка к нулю руля
bool pot_calibrated = false;              // наклон измерен командой q
float pot_zero_raw = 0.0f;                // АЦП в 0° руля
float pot_counts_per_degree = POT_NOMINAL_COUNTS_PER_DEG;
uint32_t last_pot_sample_ms = 0;
uint8_t pot_cal_state = POT_CAL_IDLE;
uint32_t pot_cal_started_ms = 0;
uint32_t pot_cal_wait_ms = 0;
float pot_cal_zero_raw = 0.0f;
float pot_cal_minus_raw = 0.0f;
float pot_cal_plus_raw = 0.0f;
bool pot_cal_ok = false;
bool feedback_pending = false;            // после поворота ждём доводку
uint8_t feedback_passes = 0;
uint32_t feedback_wait_ms = 0;
int32_t feedback_target_step = 0;

bool potRawOnRail(int16_t raw) {
  return raw <= POT_RAIL_MARGIN || raw >= POT_ADC_MAX - POT_RAIL_MARGIN;
}

int16_t potReadAverage(uint8_t count) {
  if (count == 0) count = 1;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < count; ++i) sum += (uint16_t)analogRead(POT_PIN);
  return (int16_t)((sum + count / 2) / count);
}

// Свежий замер после остановки: фильтр не успевает за ходом, поэтому для
// доводки берём среднее из четырёх отсчётов сразу, без EMA.
int16_t potMeasureRaw() {
  const int16_t raw = potReadAverage(4);
  pot_raw = raw;
  pot_sampled = true;
  pot_railed = potRawOnRail(raw);
  return raw;
}

// Один отсчёт раз в POT_SAMPLE_PERIOD_MS + экспоненциальный фильтр 1/4.
// 10 мс при 75°/с — это 0,75° запаздывания, а шум АЦП падает вчетверо.
void updatePot() {
  if (!pot_enabled) return;
  const uint32_t now = millis();
  if ((uint32_t)(now - last_pot_sample_ms) < POT_SAMPLE_PERIOD_MS) return;
  last_pot_sample_ms = now;
  const int16_t raw = (int16_t)analogRead(POT_PIN);
  if (!pot_sampled) {
    pot_raw = raw;
    pot_sampled = true;
  } else {
    pot_raw = (int16_t)(((int32_t)pot_raw * ((1 << POT_FILTER_SHIFT) - 1) + raw +
                         (1 << (POT_FILTER_SHIFT - 1))) >> POT_FILTER_SHIFT);
  }
  pot_railed = potRawOnRail(pot_raw);
}

float potAngleDeg() {
  if (!pot_zero_valid || pot_counts_per_degree == 0.0f) return 0.0f;
  return ((float)pot_raw - pot_zero_raw) / pot_counts_per_degree;
}

bool potFeedbackAllowed() {
  return pot_enabled && pot_calibrated && pot_zero_valid && !pot_railed && zero_set;
}

// Ноль руля задан (k или z) — запомнить, где стоит движок потенциометра.
// Наклон (отсчёты/градус), измеренный q, при повторном нуле сохраняется.
void potCaptureZero() {
  if (!pot_enabled) return;
  const int16_t raw = potReadAverage(POT_CAL_SAMPLES);
  pot_raw = raw;
  pot_sampled = true;
  if (potRawOnRail(raw)) {
    pot_zero_valid = false;
    pot_calibrated = false;
    pot_railed = true;
    note(PSTR("потенциометр A0: %d — на краю, доводка выключена"), (int)raw);
    return;
  }
  pot_zero_raw = raw;
  pot_zero_valid = true;
  pot_railed = false;
  if (!pot_calibrated) pot_counts_per_degree = POT_NOMINAL_COUNTS_PER_DEG;
  note(PSTR("потенциометр A0: 0° = %d, %s"), (int)raw,
       pot_calibrated ? "наклон сохранён" : "q — точная калибровка");
}

float potSlopeFromEeprom() {
  if (eeprom_read_byte((const uint8_t*)(uint16_t)EEPROM_POT_MAGIC_SLOT) != EEPROM_POT_MAGIC) return 0.0f;
  uint8_t bytes[4];
  for (uint8_t i = 0; i < sizeof(bytes); ++i) {
    bytes[i] = eeprom_read_byte((const uint8_t*)(uintptr_t)(EEPROM_POT_SLOPE_SLOT + i));
  }
  float slope;
  memcpy(&slope, bytes, sizeof(slope));
  if (slope < POT_CAL_MIN_COUNTS_PER_DEG || slope > POT_CAL_MAX_COUNTS_PER_DEG) return 0.0f;
  return slope;
}

void potSlopeToEeprom(float slope) {
  uint8_t bytes[4];
  memcpy(bytes, &slope, sizeof(slope));
  for (uint8_t i = 0; i < sizeof(bytes); ++i) {
    eeprom_update_byte((uint8_t*)(uintptr_t)(EEPROM_POT_SLOPE_SLOT + i), bytes[i]);
  }
  eeprom_update_byte((uint8_t*)(uint16_t)EEPROM_POT_MAGIC_SLOT, EEPROM_POT_MAGIC);
}

// Провал q значит, что датчик себя не подтвердил: прежний наклон доверия
// больше не заслуживает, иначе следующая загрузка достанет его из памяти.
void potSlopeForget() {
  eeprom_update_byte((uint8_t*)(uint16_t)EEPROM_POT_MAGIC_SLOT, 0);
}

void potBootNote() {
  const float saved_slope = potSlopeFromEeprom();
  if (saved_slope > 0.0f) {
    pot_counts_per_degree = saved_slope;
    pot_calibrated = true;
  }
  const int16_t raw = potReadAverage(POT_CAL_SAMPLES);
  pot_raw = raw;
  pot_sampled = true;
  pot_railed = potRawOnRail(raw);
  if (pot_railed) {
    note(PSTR("потенциометр A0: %d — на краю, проверьте провод"), (int)raw);
  } else {
    note(PSTR("потенциометр A0: %d — ноль: k/z%s"), (int)raw,
         pot_calibrated ? ", наклон из EEPROM" : ", калибровка: q");
  }
}

void potPrintStatus() {
  if (!pot_enabled) { note(PSTR("Без потенциометра: угол по шагам")); return; }
  potMeasureRaw();                     // p — свежий замер, а не фильтр с задержкой
  if (pot_zero_valid && !pot_railed) {
    char angle[16];
    formatFixed(lroundf(potAngleDeg() * 100.0f), 2, angle, sizeof(angle));
    note(PSTR("Потенциометр A0: %d = %s°%s"), (int)pot_raw, angle,
         pot_calibrated ? " (наклон измерен)" : " (номинал, q)");
  } else if (pot_railed) {
    note(PSTR("Потенциометр A0: %d — на краю, проверьте провод"), (int)pot_raw);
  } else {
    note(PSTR("Потенциометр A0: %d — нет нуля (k или z), затем q"), (int)pot_raw);
  }
}

void startPotCalMove(uint8_t state, float degrees) {
  pot_cal_state = state;
  pot_cal_wait_ms = 0;
  startMotor(degreesToSteps(degrees), STEP_PERIOD_US, OPTO_WATCH_NONE);
}

bool startPotCalibration() {
  if (!pot_enabled) { note(PSTR("Сначала pm 1 — включить потенциометр")); return false; }
  if (drive_active) {
    note(PSTR("потенциометр: включена тяга — сначала b или s"));
    return false;
  }
  if (calib_state != CAL_IDLE) {
    note(PSTR("потенциометр: идёт калибровка руля"));
    return false;
  }
  if (pot_cal_state != POT_CAL_IDLE) {
    note(PSTR("калибровка потенциометра уже идёт"));
    return false;
  }
  if (!zero_set) {
    note(PSTR("потенциометр: сначала ноль — k (по датчику) или z"));
    return false;
  }
  if (motorBusy() || cycle_active) {
    note(PSTR("потенциометр: сначала остановите колесо (s)"));
    return false;
  }
  if (pot_railed) {
    note(PSTR("потенциометр A0: %d — на краю, проверьте провод"), (int)pot_raw);
    return false;
  }
  feedback_pending = false;
  resume_allowed = false;
  move_report_pending = false;
  pot_cal_ok = false;
  pot_cal_started_ms = millis();
  note(PSTR("Калибровка потенциометра: 0°, −%d°, +%d°..."),
       (int)POT_CAL_POINT_DEG, (int)POT_CAL_POINT_DEG);
  startPotCalMove(POT_CAL_TO_ZERO, 0.0f);
  return true;
}

void updatePotCalibration() {
  if (pot_cal_state == POT_CAL_IDLE) return;
  if ((uint32_t)(millis() - pot_cal_started_ms) > POT_CAL_TIMEOUT_MS) {
    pot_cal_state = POT_CAL_IDLE;
    haltMotor();
    noInterrupts();
    target_step = current_step;
    interrupts();
    note(PSTR("ОШИБКА потенциометра: не уложилась по времени"));
    return;
  }
  if (motorBusy()) {
    pot_cal_wait_ms = 0;
    return;
  }
  if (pot_cal_wait_ms == 0) {
    pot_cal_wait_ms = millis() + POT_CAL_SETTLE_MS;
    if (pot_cal_wait_ms == 0) pot_cal_wait_ms = 1;
    return;
  }
  if ((int32_t)(millis() - pot_cal_wait_ms) < 0) return;

  switch (pot_cal_state) {
    case POT_CAL_TO_ZERO:
      pot_cal_zero_raw = (float)potReadAverage(POT_CAL_SAMPLES);
      startPotCalMove(POT_CAL_TO_MINUS, -POT_CAL_POINT_DEG);
      break;
    case POT_CAL_TO_MINUS:
      pot_cal_minus_raw = (float)potReadAverage(POT_CAL_SAMPLES);
      startPotCalMove(POT_CAL_TO_PLUS, POT_CAL_POINT_DEG);
      break;
    case POT_CAL_TO_PLUS: {
      pot_cal_plus_raw = (float)potReadAverage(POT_CAL_SAMPLES);
      const float slope = (pot_cal_plus_raw - pot_cal_minus_raw) /
                          (2.0f * POT_CAL_POINT_DEG);
      const float magnitude = fabsf(slope);
      const bool rails = potRawOnRail((int16_t)pot_cal_zero_raw) ||
                         potRawOnRail((int16_t)pot_cal_minus_raw) ||
                         potRawOnRail((int16_t)pot_cal_plus_raw);
      if (rails || magnitude < POT_CAL_MIN_COUNTS_PER_DEG ||
          magnitude > POT_CAL_MAX_COUNTS_PER_DEG) {
        pot_cal_ok = false;
        pot_calibrated = false;
        potSlopeForget();
        pot_zero_valid = false;
        note(PSTR("ОШИБКА потенциометра: нет сигнала или неверный ход"));
      } else {
        pot_cal_ok = true;
        pot_zero_raw = pot_cal_zero_raw;
        pot_counts_per_degree = slope;
        pot_zero_valid = true;
        pot_calibrated = true;
        potSlopeToEeprom(slope);
        char text[16];
        formatFixed(lroundf(slope * 100.0f), 2, text, sizeof(text));
        note(PSTR("Потенциометр: %s отсч/°, 0° = %d"), text, (int)pot_cal_zero_raw);
      }
      startPotCalMove(POT_CAL_BACK, 0.0f);
      break;
    }
    case POT_CAL_BACK:
      pot_cal_state = POT_CAL_IDLE;
      if (pot_cal_ok) {
        note(PSTR("Потенциометр: калибровка завершена, доводка включена"));
      }
      break;
    default:
      pot_cal_state = POT_CAL_IDLE;
      break;
  }
}

// После поворота: сравнить угол с потенциометром и до 3 раз довернуть.
// error = измерено − цель; чтобы уменьшить измеренное, колесо едет на −error.
// Затем счётчик шагов привязывается к измеренному углу, чтобы deg= в
// телеметрии было реальным положением колеса.
void updateFeedback() {
  if (!feedback_pending) return;
  if (calib_state != CAL_IDLE || pot_cal_state != POT_CAL_IDLE || cycle_active) {
    feedback_pending = false;
    return;
  }
  if (motorBusy()) {
    feedback_wait_ms = 0;
    return;
  }
  const uint32_t now = millis();
  if (feedback_wait_ms == 0) {
    feedback_wait_ms = now + POT_CORRECTION_SETTLE_MS;
    if (feedback_wait_ms == 0) feedback_wait_ms = 1;
    return;
  }
  if ((int32_t)(now - feedback_wait_ms) < 0) return;

  potMeasureRaw();
  const float actual = potAngleDeg();
  const float wanted = (float)feedback_target_step / STEPS_PER_DEGREE;
  const float error = actual - wanted;
  // Датчик доворачивает колесо на сантиметры, а не возит его по всей шкале.
  // Если он расходится с целью сильнее POT_TRUST_DEG, верить ему нельзя:
  // доводка догоняла бы именно датчик и утаскивала колесо на десятки градусов
  // от команды (оператор видит «повернулось на 4° вместо 35°»), а последовавшая
  // привязка счётчика шагов к датчику заставляла телеметрию отрапортовать
  // успех при колесе совсем не там. Колесо остаётся там, куда его привели
  // шаги, а расхождение честно печатается.
  if (!pot_railed && fabsf(error) > POT_TRUST_DEG) {
    char text[16];
    formatFixed(lroundf(error * 100.0f), 2, text, sizeof(text));
    note(PSTR("Потенциометр: расхождение %s° — доводка отключена; проверьте колесо или q"), text);
    noInterrupts();
    target_step = feedback_target_step;
    interrupts();
    feedback_pending = false;
    feedback_passes = 0;
    return;
  }
  if (!pot_railed && fabsf(error) > POT_CORRECTION_DEADBAND_DEG &&
      feedback_passes < POT_CORRECTION_MAX_PASSES) {
    ++feedback_passes;
    char text[16];
    formatFixed(lroundf(error * 100.0f), 2, text, sizeof(text));
    note(PSTR("Потенциометр: доводка %s° (проход %d/%d)"), text,
         (int)feedback_passes, (int)POT_CORRECTION_MAX_PASSES);
    const int32_t correction = degreesToSteps(-error);
    if (correction != 0) {
      target_step = stepNow() + correction;
      startMotor(target_step, STEP_PERIOD_US, OPTO_WATCH_NONE);
      feedback_wait_ms = 0;
      return;
    }
  }

  const float step_angle = (float)stepNow() / STEPS_PER_DEGREE;
  // Счётчик шагов привязываем к датчику только в пределах доверия: иначе одна
  // врущая q молча перепишет положение колеса и телеметрия начнёт врать следом.
  if (pot_zero_valid && !pot_railed &&
      fabsf(actual - step_angle) > POT_RESYNC_DEG &&
      fabsf(actual - step_angle) <= POT_TRUST_DEG) {
    noInterrupts();
    current_step = degreesToSteps(actual);
    interrupts();
  }
  noInterrupts();
  target_step = feedback_target_step;
  interrupts();
  if (feedback_passes > 0) {
    char text[16];
    formatFixed(lroundf(error * 100.0f), 2, text, sizeof(text));
    note(PSTR("Потенциометр: остаток %s°"), text);
  }
  feedback_pending = false;
  feedback_passes = 0;
}

// ---------------------------------------------------------------------------
// Калибровка по оптодатчику. Флажок находится быстрым ходом в пределах ±125°
// от старта (вправо 125°, потом влево 250°; упор потенциометра — ±130°).
// Затем каждая кромка снимается
// CALIB_SLOW_PASSES раз медленным подходом снаружи: справа налево и слева направо,
// вперемешку. Ноль — середина между средними кромками (или кромка, см.
// CALIB_ZERO_AT_CENTER) плюс CALIB_ZERO_OFFSET_DEG. Разброс кромок больше
// CALIB_MAX_EDGE_SPREAD_DEG — ошибка 6, ноль не ставится.
void copyCalibErrorText(uint8_t error, char* out, size_t out_size) {
  const char* text;
  switch (error) {
    case CAL_ERR_NOT_FOUND: text = PSTR("датчик не найден в ±125° от старта"); break;
    case CAL_ERR_STUCK: text = PSTR("датчик не освобождается (залип или флажок шире 30°)"); break;
    case CAL_ERR_EDGE: text = PSTR("кромка не найдена при медленном подходе"); break;
    case CAL_ERR_SPREAD: text = PSTR("кромки расходятся между подходами"); break;
    case CAL_ERR_ABORTED: text = PSTR("прервана командой s"); break;
    case CAL_ERR_TIMEOUT: text = PSTR("не уложилась по времени"); break;
    default: text = PSTR("нет ошибок"); break;
  }
  strncpy_P(out, text, out_size - 1);
  out[out_size - 1] = '\0';
}

void calibrationStep(uint8_t state, float degrees, uint16_t period_us, uint8_t watch) {
  calib_state = state;
  startMotor(stepNow() + degreesToSteps(degrees), period_us, watch);
}

bool startCalibration() {
  if (drive_active) {
    note(PSTR("калибровка отклонена: включена тяга (сначала b или s)"));
    return false;
  }
  if (calib_state != CAL_IDLE) {
    note(PSTR("Калибровка уже запущена!"));
    return false;
  }
  if (pot_cal_state != POT_CAL_IDLE) {
    note(PSTR("сначала дождитесь калибровки потенциометра или s"));
    return false;
  }
  cycle_active = false;
  resume_allowed = false;
  move_report_pending = false;
  feedback_pending = false;
  zero_set = false;
  pot_zero_valid = false;
  home_source = HOME_NONE;
  calib_error = CAL_OK;
  calib_started_ms = millis();
  calib_plus_count = 0;
  calib_minus_count = 0;
  if (optoActive()) {
    note(PSTR("Калибровка: датчик уже закрыт — выхожу из флажка вправо"));
    calibrationStep(CAL_EXIT_PLUS, CALIB_MAX_FLAG_DEGREES, STEP_PERIOD_US, OPTO_WATCH_CLEAR);
  } else {
    note(PSTR("Калибровка: быстрый поиск вправо на %d°..."), (int)CALIB_HALF_RANGE_DEG);
    calibrationStep(CAL_FAST_PLUS, CALIB_HALF_RANGE_DEG, STEP_PERIOD_US, OPTO_WATCH_ACTIVE);
  }
  return true;
}

void failCalibration(uint8_t error) {
  haltMotor();
  noInterrupts();
  target_step = current_step;
  interrupts();
  calib_state = CAL_IDLE;
  calib_error = error;
  zero_set = false;
  home_source = HOME_NONE;
  pot_zero_valid = false;
  feedback_pending = false;
  char text[112];
  copyCalibErrorText(error, text, sizeof(text));
  note(PSTR("ОШИБКА калибровки: %s"), text);
}

// Среднее по подходам (округление к ближайшему, половины — от нуля).
int32_t meanSteps(const int32_t* values, uint8_t count) {
  if (count == 0) return 0;
  int32_t sum = 0;
  for (uint8_t i = 0; i < count; ++i) sum += values[i];
  return sum >= 0 ? (sum + count / 2) / count : -((-sum + count / 2) / count);
}

// Разброс кромок между подходами (макс − мин).
int32_t spreadSteps(const int32_t* values, uint8_t count) {
  if (count == 0) return 0;
  int32_t low = values[0];
  int32_t high = values[0];
  for (uint8_t i = 1; i < count; ++i) {
    if (values[i] < low) low = values[i];
    if (values[i] > high) high = values[i];
  }
  return high - low;
}

uint8_t calibrationPassesDone() {
  return (uint8_t)(calib_plus_count + calib_minus_count);
}

void finishCalibrationEdges() {
  haltMotor();
  // Кромки с разных подходов должны совпадать: иначе датчик или механика врут,
  // и ноль ставить нельзя.
  const int32_t limit = degreesToSteps(CALIB_MAX_EDGE_SPREAD_DEG);
  const int32_t spread_plus = spreadSteps(calib_edges_plus, calib_plus_count);
  const int32_t spread_minus = spreadSteps(calib_edges_minus, calib_minus_count);
  if (spread_plus > limit || spread_minus > limit) {
    failCalibration(CAL_ERR_SPREAD);
    return;
  }
  const int32_t edge_plus = meanSteps(calib_edges_plus, calib_plus_count);
  const int32_t edge_minus = meanSteps(calib_edges_minus, calib_minus_count);
  int32_t zero_at = edge_minus;
  if (CALIB_ZERO_AT_CENTER) {
    const int32_t sum = edge_plus + edge_minus;
    zero_at = sum >= 0 ? (sum + 1) / 2 : -((1 - sum) / 2);
  }
  zero_at += degreesToSteps(CALIB_ZERO_OFFSET_DEG);
  char width[16];
  char shift[16];
  char spread_a[16];
  char spread_b[16];
  formatDegrees(edge_plus - edge_minus, width, sizeof(width));
  formatDegrees(zero_at, shift, sizeof(shift));
  formatDegrees(spread_plus, spread_a, sizeof(spread_a));
  formatDegrees(spread_minus, spread_b, sizeof(spread_b));
  note(PSTR("Калибровка: кромки найдены, ширина флажка %s°, разброс %s°/%s°, ноль на %s°"),
       width, spread_a, spread_b, shift);
  noInterrupts();
  current_step -= zero_at;   // среднее положение флажка становится нулём
  target_step = current_step;
  interrupts();
  calib_state = CAL_GO_ZERO;
  startMotor(0, STEP_PERIOD_US, OPTO_WATCH_NONE);
}

void updateCalibration() {
  if (calib_state == CAL_IDLE) return;
  if ((uint32_t)(millis() - calib_started_ms) > CALIB_TIMEOUT_MS) {
    failCalibration(CAL_ERR_TIMEOUT);
    return;
  }
  noInterrupts();
  const bool event = opto_event;
  const int32_t event_step = opto_event_step;
  const bool busy = is_moving || pulse_high;
  opto_event = false;
  interrupts();
  if (!event && busy) return;

  // Порядок подходов: медленно справа налево, затем слева направо, и так
  // CALIB_SLOW_PASSES раз каждый. Между подходами колесо выходит из флажка
  // на сторону, откуда пришло, — следующий подход снова идёт от края.
  switch (calib_state) {
    case CAL_FAST_PLUS:
      if (event) {
        note(PSTR("Калибровка: датчик найден справа, уточняю кромки..."));
        calibrationStep(CAL_EXIT_PLUS, CALIB_MAX_FLAG_DEGREES, STEP_PERIOD_US, OPTO_WATCH_CLEAR);
      } else {
        note(PSTR("Калибровка: справа датчика нет, ищу влево на %d°..."),
             (int)(2.0f * CALIB_HALF_RANGE_DEG));
        calibrationStep(CAL_FAST_MINUS, -2.0f * CALIB_HALF_RANGE_DEG, STEP_PERIOD_US,
                        OPTO_WATCH_ACTIVE);
      }
      break;
    case CAL_FAST_MINUS:
      if (event) {
        note(PSTR("Калибровка: датчик найден слева, уточняю кромки..."));
        calibrationStep(CAL_EXIT_PLUS, CALIB_MAX_FLAG_DEGREES, STEP_PERIOD_US, OPTO_WATCH_CLEAR);
      } else {
        failCalibration(CAL_ERR_NOT_FOUND);
      }
      break;
    case CAL_EXIT_PLUS:
      if (event) {
        calibrationStep(CAL_CLEAR_PLUS, CALIB_CLEARANCE_DEGREES, STEP_PERIOD_US, OPTO_WATCH_NONE);
      } else {
        failCalibration(CAL_ERR_STUCK);
      }
      break;
    case CAL_CLEAR_PLUS:
      calibrationStep(CAL_SLOW_MINUS, -(CALIB_CLEARANCE_DEGREES + CALIB_SLOW_SEARCH_DEGREES),
                      CALIB_STEP_PERIOD_US, OPTO_WATCH_ACTIVE);
      break;
    case CAL_SLOW_MINUS:
      if (event && calib_plus_count < CALIB_SLOW_PASSES) {
        calib_edges_plus[calib_plus_count++] = event_step;
        note(PSTR("Калибровка: кромка справа найдена (подход %d из %d)"),
             (int)calib_plus_count, (int)CALIB_SLOW_PASSES);
        calibrationStep(CAL_EXIT_MINUS, -CALIB_MAX_FLAG_DEGREES, STEP_PERIOD_US, OPTO_WATCH_CLEAR);
      } else {
        failCalibration(CAL_ERR_EDGE);
      }
      break;
    case CAL_EXIT_MINUS:
      if (event) {
        calibrationStep(CAL_CLEAR_MINUS, -CALIB_CLEARANCE_DEGREES, STEP_PERIOD_US, OPTO_WATCH_NONE);
      } else {
        failCalibration(CAL_ERR_STUCK);
      }
      break;
    case CAL_CLEAR_MINUS:
      calibrationStep(CAL_SLOW_PLUS, CALIB_CLEARANCE_DEGREES + CALIB_SLOW_SEARCH_DEGREES,
                      CALIB_STEP_PERIOD_US, OPTO_WATCH_ACTIVE);
      break;
    case CAL_SLOW_PLUS:
      if (event && calib_minus_count < CALIB_SLOW_PASSES) {
        calib_edges_minus[calib_minus_count++] = event_step;
        note(PSTR("Калибровка: кромка слева найдена (подход %d из %d)"),
             (int)calib_minus_count, (int)CALIB_SLOW_PASSES);
        if (calibrationPassesDone() >= 2 * CALIB_SLOW_PASSES) {
          finishCalibrationEdges();
        } else {
          calibrationStep(CAL_EXIT_PLUS, CALIB_MAX_FLAG_DEGREES, STEP_PERIOD_US, OPTO_WATCH_CLEAR);
        }
      } else {
        failCalibration(CAL_ERR_EDGE);
      }
      break;
    case CAL_GO_ZERO:
      calib_state = CAL_IDLE;
      calib_error = CAL_OK;
      zero_set = true;
      home_source = HOME_OPTO;
      potCaptureZero();
      note(PSTR("Калибровка ЗАВЕРШЕНА! Точка 0 установлена по датчику."));
      break;
    default:
      failCalibration(CAL_ERR_ABORTED);
      break;
  }
}


// ---------------------------------------------------------------------------
// Качание (c), обычные повороты (d, h, w, g), ноль (z), стоп (s).
bool steeringAllowed() {
  if (drive_active) {
    note(PSTR("руль заблокирован: включена тяга — сначала b или s"));
    return false;
  }
  if (calib_state != CAL_IDLE) {
    note(PSTR("идёт калибровка — дождитесь конца или отправьте s"));
    return false;
  }
  if (pot_cal_state != POT_CAL_IDLE) {
    note(PSTR("идёт калибровка потенциометра — s для отмены"));
    return false;
  }
  return true;
}

void startPlainMove(int32_t target, bool report) {
  const int32_t limit = degreesToSteps(STEER_TRAVEL_LIMIT_DEG);
  if (target > limit || target < -limit) {
    char text[16];
    formatDegrees(target, text, sizeof(text));
    note(PSTR("цель %s° вне хода ±%d°"), text, (int)STEER_TRAVEL_LIMIT_DEG);
    return;
  }
  cycle_active = false;
  feedback_pending = potFeedbackAllowed();
  feedback_target_step = target;
  feedback_passes = 0;
  feedback_wait_ms = 0;
  startMotor(target, STEP_PERIOD_US, OPTO_WATCH_NONE);
  resume_allowed = true;
  move_report_pending = report && motorBusy();
}

void startCycle(float degrees) {
  haltMotor();
  const int32_t start = stepNow();
  const int32_t delta = degreesToSteps(degrees);
  const int32_t limit = degreesToSteps(STEER_TRAVEL_LIMIT_DEG);
  if (start + delta > limit || start - delta < -limit) {
    note(PSTR("качание %d° выходит за ход ±%d°"), (int)degrees, (int)STEER_TRAVEL_LIMIT_DEG);
    return;
  }
  cycle_start_step = start;
  cycle_end_step = start + delta;
  cycle_to_end = true;
  cycle_active = true;
  resume_allowed = false;
  move_report_pending = false;
  feedback_pending = false;
  startMotor(cycle_end_step, STEP_PERIOD_US, OPTO_WATCH_NONE);
  char text[16];
  formatFixed(lroundf(degrees * 100.0f), 2, text, sizeof(text));
  note(PSTR("ЦИКЛ качания на %s°"), text);
}

void stopCycle() {
  if (!cycle_active) {
    note(PSTR("Цикл и так выключен."));
    return;
  }
  cycle_active = false;
  haltMotor();
  noInterrupts();
  target_step = current_step;
  interrupts();
  char angle[16];
  formatDegrees(stepNow(), angle, sizeof(angle));
  note(PSTR("ЦИКЛ ВЫКЛ. Угол: %s°"), angle);
}

void updateCycle() {
  if (!cycle_active || motorBusy()) return;
  cycle_to_end = !cycle_to_end;
  startMotor(cycle_to_end ? cycle_end_step : cycle_start_step, STEP_PERIOD_US, OPTO_WATCH_NONE);
}

void updateMoveReport() {
  if (!move_report_pending || motorBusy()) return;
  move_report_pending = false;
  char angle[16];
  formatDegrees(stepNow(), angle, sizeof(angle));
  note(PSTR("Цель достигнута. Угол: %s°"), angle);
}

bool setZeroHere() {
  if (drive_active || calib_state != CAL_IDLE || pot_cal_state != POT_CAL_IDLE) {
    note(PSTR("ноль не задан: %s"),
         drive_active ? "включена тяга"
                      : (calib_state != CAL_IDLE ? "идёт калибровка" : "идёт калибровка потенциометра"));
    return false;
  }
  cycle_active = false;
  feedback_pending = false;
  haltMotor();
  noInterrupts();
  current_step = 0;
  target_step = 0;
  interrupts();
  zero_set = true;
  home_source = HOME_MANUAL;
  calib_error = CAL_OK;
  resume_allowed = false;
  move_report_pending = false;
  potCaptureZero();
  note(PSTR("ОБНУЛЕНА. Ноль задан вручную в текущем положении."));
  return true;
}

// ---------------------------------------------------------------------------
// Выход на драйвер: аппаратный ШИМ (вход «P» на D3), DIR, BRAKE, STOP.
// Настроек нет — только константы железа (см. блок «тяга BLDC»).

// Зеркальное ли это колесо. FR и RR стоят на другой стороне корпуса, поэтому
// «вперёд» у них — противоположный уровень DIR. Имя колеса уже вписано в
// прошивку (MODULE_ID), так что определяется само, без настроек.
bool driveMirrorByModule() {
#if DRIVE_MIRRORED == 1
  return true;
#elif DRIVE_MIRRORED == 0
  return false;
#else
  return strcasecmp_P(MODULE_ID, PSTR("FR")) == 0 ||
         strcasecmp_P(MODULE_ID, PSTR("RR")) == 0;
#endif
}

// Уровень DIR для «вперёд» у этого колеса: у зеркального он перевёрнут.
uint8_t driveForwardLevel() {
  const bool flip = drive_mirrored;
  return flip ? (BLDC_DIR_FORWARD_LEVEL == HIGH ? LOW : HIGH) : BLDC_DIR_FORWARD_LEVEL;
}

// Библиотека Servo даёт на D3 сигнал около 50 Гц; драйвер использует его
// скважность. Выбранное значение передаётся в write() без пересчёта.
Servo drive_servo;

void setupDriveServo() {
  drive_mirrored = driveMirrorByModule();
  drive_servo.attach(BLDC_SPEED_PIN);   // «P» драйвера, D3
  drive_servo.write(0);                 // ещё есть импульсы; выходы отключены отдельно
  drive_servo_value = 0;
}

// Значение Servo.write, а не скважность на P и не фактические обороты.
void driveWriteServo(uint8_t value) {
  if (value > DRIVE_SERVO_MAX_VALUE) value = DRIVE_SERVO_MAX_VALUE;
  drive_servo.write(value);
  drive_servo_value = value;
}

// Процент команды → значение шкалы Servo: 1 % = 1,8, 100 % = 180.
uint8_t driveValueForPercent(int8_t percent) {
  const uint32_t magnitude = (uint32_t)abs((int)percent);
  uint32_t value = (magnitude * DRIVE_SERVO_TENTHS_PER_PERCENT + 5) / 10;
  if (value > DRIVE_SERVO_MAX_VALUE) value = DRIVE_SERVO_MAX_VALUE;
  return (uint8_t)value;
}

// Значение шкалы в промилле — для телеметрии (duty=) и журнала.
uint16_t driveValuePermille() {
  return (uint16_t)((uint32_t)drive_servo_value * 1000UL / DRIVE_SERVO_MAX_VALUE);
}

// Тормоз: BRAKE = 1 включает тормоз драйвера, 0 — отпускает.
void driveSetBrake(bool active) {
  digitalWrite(BLDC_BRAKE_PIN, active ? BLDC_BRAKE_ACTIVE_LEVEL
                                      : (BLDC_BRAKE_ACTIVE_LEVEL == HIGH ? LOW : HIGH));
}

// STOP драйвера: 1 — выходы разрешены, 0 — выключены (свободный выбег).
// Пока тяги нет (старт платы, «b», watchdog), драйвер выключен.
void driveSetStop(bool stopped) {
  digitalWrite(BLDC_STOP_PIN, stopped ? BLDC_STOP_DISABLE_LEVEL
                                      : BLDC_STOP_ENABLE_LEVEL);
}

void printDriveInfo() {
  // Строка должна влезть в буфер строки (176 байт с кириллицей по 2 байта):
  // поэтому проценты словами, без «%%» — они и так висят на экране.
  note(PSTR("Двигатель: Servo D3 50 Гц · значение %u из %u · тяга до %u %% · %s"),
       (unsigned)drive_servo_value, (unsigned)DRIVE_SERVO_MAX_VALUE,
       (unsigned)DRIVE_MAX_PERCENT, drive_mirrored ? "зеркальное" : "прямое");
}

// ---------------------------------------------------------------------------
// Тяга BLDC: только при заданном нуле и стоящем руле, с watchdog 650 мс.
bool localDriveAllowed() {
  return zero_set && !motorBusy() && calib_state == CAL_IDLE &&
         pot_cal_state == POT_CAL_IDLE && !cycle_active;
}

void applyDriveCommand(int8_t percent) {
  // Процент > 0 — вперёд: DIR в «своём» уровне колеса (у зеркального FR/RR он
  // перевёрнут). Процент < 0 — назад: противоположный уровень.
  const bool forward = percent >= 0;
  const uint8_t ahead = driveForwardLevel();
  const uint8_t direction = forward ? ahead : (ahead == HIGH ? LOW : HIGH);
  const bool reversing = drive_active && ((percent > 0) != (drive_percent > 0));
  if (!drive_active || reversing) {
    // Сначала снимаем тягу и тормозим, потом меняем DIR — иначе смена
    // направления приходится на вращающийся мотор (у драйвера это рывок).
    driveWriteServo(0);
    driveSetBrake(true);
    driveSetStop(true);
    digitalWrite(BLDC_DIR_PIN, direction);
    delayMicroseconds(100);
  }
  digitalWrite(BLDC_DIR_PIN, direction);
  driveSetBrake(false);                       // тормоз отпущен
  driveSetStop(false);                        // драйвер разрешён на время тяги
  // Как в проверенном скетче: значение на «P» пишется сразу, без разгона и без
  // контура. Плавность набора даёт сам драйвер — ему этот вход привычен.
  driveWriteServo(driveValueForPercent(percent));
  drive_percent = percent;
  drive_active = percent != 0;
  last_drive_command_ms = millis();
  if (!drive_active) {                // «t 0» — сразу снять тягу и затормозить
    driveSetBrake(true);
    driveSetStop(true);
  }
}

void stopDriveLocal() {
  driveWriteServo(0);                // Servo продолжает давать короткий импульс
  driveSetBrake(true);               // тормоз: колесо не катится
  driveSetStop(true);                // выходы драйвера выключены
  drive_percent = 0;
  drive_active = false;
}

// v: число ползунка без пересчёта идёт в Servo.write на D3. Направление
// задаётся отдельно через DIR; ноль руля и тахометр в этом режиме не участвуют.
void applyDirectDrive(uint8_t value, bool forward) {
  if (value == 0) {
    stopDriveLocal();
    return;
  }
  const uint8_t ahead = driveForwardLevel();
  const uint8_t direction = forward ? ahead : (ahead == HIGH ? LOW : HIGH);
  if (drive_active && digitalRead(BLDC_DIR_PIN) != direction) {
    // Прежде чем менять DIR, снять предыдущее значение и включить тормоз.
    driveWriteServo(0);
    driveSetBrake(true);
    driveSetStop(true);
    delayMicroseconds(100);
  }
  digitalWrite(BLDC_DIR_PIN, direction);
  driveSetBrake(false);
  driveSetStop(false);
  driveWriteServo(value);
  // Старое поле телеметрии: только индикация, не участвует в Servo.write.
  drive_percent = (int8_t)(((uint16_t)value * 100U + 90U) / 180U);
  if (!forward) drive_percent = -drive_percent;
  drive_active = true;
  last_drive_command_ms = millis();
}

bool applyLocalDrive(int8_t percent) {
  if (percent == 0) {
    stopDriveLocal();
    return true;
  }
  if (abs((int)percent) > (int)DRIVE_MAX_PERCENT || !localDriveAllowed() ||
      (drive_active && ((percent > 0) != (drive_percent > 0)))) {
    stopDriveLocal();
    return false;
  }
  applyDriveCommand(percent);
  return true;
}

void updateDriveWatchdog() {
  if (drive_active && (uint32_t)(millis() - last_drive_command_ms) > DRIVE_COMMAND_TIMEOUT_MS) {
    stopDriveLocal();
    note(PSTR("тяга: нет команды дольше 650 мс — выходы выключены, тормоз"));
  }
}

// ---------------------------------------------------------------------------
uint16_t moduleStatus() {
  uint16_t status = STATUS_V2;
  if (motorBusy()) status |= STATUS_MOVING;
  if (zero_set) status |= STATUS_ZERO;
  if (calib_state != CAL_IDLE || pot_cal_state != POT_CAL_IDLE) status |= STATUS_CALIB;
  if (optoActive()) status |= STATUS_OPTO;
  if (cycle_active) status |= STATUS_CYCLE;
  if (zero_set && home_source == HOME_OPTO) status |= STATUS_HOME_OPTO;
  if (potFeedbackAllowed()) status |= STATUS_POT;
  status |= STATUS_POT_MODE_SUPPORTED;  // поддержка переключения режима
  if (pot_enabled) status |= STATUS_POT_MODE;
  status |= (uint16_t)(calib_error & 0x0F) << 8;
  return status;
}

void printModuleStatus() {
  char current[16];
  char target[16];
  formatDegrees(stepNow(), current, sizeof(current));
  formatDegrees(targetNow(), target, sizeof(target));
  const char* zero = !zero_set ? "не задан" : (home_source == HOME_OPTO ? "по датчику" : "вручную");
  const char* state = calib_state != CAL_IDLE ? "КАЛИБРОВКА" : (cycle_active ? "ЦИКЛ" : (motorBusy() ? "ДВИЖ" : "СТОИТ"));
  note(PSTR("Текущий: %s° | Целевой: %s° [%s] · ноль %s · датчик %s · тяга %d%%"),
       current, target, state, zero, optoActive() ? "закрыт" : "свободен", (int)drive_percent);
  potPrintStatus();
  printDriveInfo();
}

// Команды одного рулевого модуля (проверенный скетч). Плата принимает их со
// своего USB — сервер шлёт адресную команду на порт нужного колеса, а любую
// плату можно проверить на столе отдельно от остальных.
//   k | calib      калибровка по оптодатчику
//   q              калибровка потенциометра A0: 0°, −60°, +60° (доводка по датчику)
//   z | zero       ноль в текущем положении (вручную)
//   h              в ноль           s   стоп всего (руль, калибровка, цикл, тяга)
//   g              продолжить поворот, прерванный s
//   d <°> f|b      повернуть на угол: f — вправо (+), b — влево (−)
//   c <°> | c c    качание на угол / выключить качание
//   w <°>          абсолютный угол (нужен ноль, предел ±90°) — команда сервера
//   v <0…150>      прямое значение Servo.write с бортового экрана
//   t <%> | b      прежняя команда тяги / тормоз
//   p              состояние
void stopEverything() {
  const bool cycling = cycle_active;
  const bool pot_cal = pot_cal_state != POT_CAL_IDLE;
  if (calib_state != CAL_IDLE) failCalibration(CAL_ERR_ABORTED);
  cycle_active = false;
  haltMotor();
  stopDriveLocal();
  move_report_pending = false;
  feedback_pending = false;
  if (pot_cal) {
    pot_cal_state = POT_CAL_IDLE;
    note(PSTR("калибровка потенциометра прервана"));
  }
  if (cycling) resume_allowed = false;
  char angle[16];
  formatDegrees(stepNow(), angle, sizeof(angle));
  note(PSTR("ПАУЗА. Угол: %s° (руль стоит, тяга выключена)"), angle);
}

void handleModuleCommand(char* line) {
  char* save = nullptr;
  char* command = strtok_r(line, " \t", &save);
  if (!command) return;
  char* arg1 = strtok_r(nullptr, " \t", &save);
  char* arg2 = strtok_r(nullptr, " \t", &save);
  char* extra = strtok_r(nullptr, " \t", &save);

  if (strcasecmp_P(command, PSTR("k")) == 0 || strcasecmp_P(command, PSTR("calib")) == 0) {
    startCalibration();
  } else if (strcasecmp_P(command, PSTR("pm")) == 0) {
    if (!arg1 || arg2 || (strcmp(arg1, "0") && strcmp(arg1, "1"))) {
      note(PSTR("Шаблон: pm 0|1")); return;
    }
    if (motorBusy() || drive_percent != 0 || calib_state != CAL_IDLE ||
        pot_cal_state != POT_CAL_IDLE || cycle_active) {
      note(PSTR("Смена режима: сначала s")); return;
    }
    const bool enabled = arg1[0] == '1';
    if (enabled != pot_enabled) {
      pot_enabled = enabled;
      pot_calibrated = false;
      pot_zero_valid = false;
      pot_sampled = false;
      feedback_pending = false;
      resume_allowed = false;
      if (enabled) potMeasureRaw();  // ноль АЦП измерит q в положении 0°
    }
    note(pot_enabled ? PSTR("С потенциометром: q для доводки") : PSTR("Без потенциометра: угол по шагам"));
  } else if (strcasecmp_P(command, PSTR("q")) == 0) {
    startPotCalibration();
  } else if (strcasecmp_P(command, PSTR("s")) == 0) {
    stopEverything();
  } else if (strcasecmp_P(command, PSTR("z")) == 0 || strcasecmp_P(command, PSTR("zero")) == 0) {
    setZeroHere();
  } else if (strcasecmp_P(command, PSTR("h")) == 0) {
    if (!steeringAllowed()) return;
    if (stepNow() == 0 && !motorBusy()) {
      note(PSTR("Уже в 0°."));
      return;
    }
    if (!zero_set) note(PSTR("ноль не задан — еду в положение включения (k — калибровка по датчику)"));
    note(PSTR("Домой: %s..."), stepNow() < 0 ? "ВПЕРЕД" : "НАЗАД");
    startPlainMove(0, true);
  } else if (strcasecmp_P(command, PSTR("g")) == 0) {
    if (!steeringAllowed()) return;
    if (!resume_allowed || motorBusy() || targetNow() == stepNow()) {
      note(PSTR("Уже в цели."));
      return;
    }
    note(PSTR("ПРОДОЛЖЕНИЕ..."));
    startPlainMove(targetNow(), true);
  } else if (strcasecmp_P(command, PSTR("d")) == 0) {
    float degrees = 0.0f;
    if (!arg1 || !arg2 || extra) {
      note(PSTR("Шаблон: d [градусы] [f/b]"));
      return;
    }
    if (!parseFloatToken(arg1, &degrees) || degrees <= 0.0f) {
      note(PSTR("Ошибка: Угол > 0!"));
      return;
    }
    if (degrees > MANUAL_MOVE_LIMIT_DEG) {
      note(PSTR("Ошибка: не больше 360° за одну команду"));
      return;
    }
    const char direction = (char)tolower((unsigned char)arg2[0]);
    if (direction != 'f' && direction != 'b') {
      note(PSTR("Ошибка направления! f или b"));
      return;
    }
    if (!steeringAllowed()) return;
    haltMotor();
    const int32_t steps = degreesToSteps(degrees);
    char text[16];
    formatFixed(lroundf(degrees * 100.0f), 2, text, sizeof(text));
    note(direction == 'f' ? PSTR("ВПЕРЕД (F) на %s°") : PSTR("НАЗАД (B) на %s°"), text);
    startPlainMove(stepNow() + (direction == 'f' ? steps : -steps), true);
  } else if (strcasecmp_P(command, PSTR("c")) == 0) {
    if (!arg1 || arg2) {
      note(PSTR("Шаблон: c [градусы] или c c"));
      return;
    }
    if (strcasecmp_P(arg1, PSTR("c")) == 0) {
      stopCycle();
      return;
    }
    float degrees = 0.0f;
    if (!parseFloatToken(arg1, &degrees) || degrees <= 0.0f) {
      note(PSTR("Ошибка: Угол > 0!"));
      return;
    }
    if (degrees > MANUAL_MOVE_LIMIT_DEG) {
      note(PSTR("Ошибка: не больше 360° за одну команду"));
      return;
    }
    if (!steeringAllowed()) return;
    startCycle(degrees);
  } else if (strcasecmp_P(command, PSTR("w")) == 0) {
    float degrees = 0.0f;
    if (!arg1 || arg2) {
      note(PSTR("Шаблон: w [градусы]"));
      return;
    }
    if (strcasecmp_P(arg1, PSTR("zero")) == 0) {
      setZeroHere();
      return;
    }
    if (strcasecmp_P(arg1, PSTR("calib")) == 0) {
      startCalibration();
      return;
    }
    if (!parseFloatToken(arg1, &degrees) || fabsf(degrees) > STEER_LIMIT_DEG) {
      note(PSTR("w: угол вне ±%d°"), (int)STEER_LIMIT_DEG);
      return;
    }
    if (!zero_set) {
      note(PSTR("w: ноль не задан — сначала k (по датчику) или z"));
      return;
    }
    if (!steeringAllowed()) return;
    startPlainMove(degreesToSteps(degrees), false);
  } else if (strcasecmp_P(command, PSTR("t")) == 0) {
    long percent = 0;
    if (!arg1 || arg2 || !parseLongToken(arg1, &percent) ||
        percent < -(long)DRIVE_MAX_PERCENT || percent > (long)DRIVE_MAX_PERCENT) {
      stopDriveLocal();
      return;
    }
    applyLocalDrive((int8_t)percent);
  } else if (strcasecmp_P(command, PSTR("v")) == 0) {
    long value = 0;
    const bool forward = !arg2 || strcasecmp_P(arg2, PSTR("f")) == 0;
    const bool backward = arg2 && strcasecmp_P(arg2, PSTR("b")) == 0;
    if (!arg1 || extra || !parseLongToken(arg1, &value) ||
        value < 0 || value > (long)DRIVE_DIRECT_MAX_VALUE || (!forward && !backward)) {
      stopDriveLocal();
      return;
    }
    applyDirectDrive((uint8_t)value, forward);
  } else if (strcasecmp_P(command, PSTR("b")) == 0) {
    stopDriveLocal();
  } else if (strcasecmp_P(command, PSTR("p")) == 0) {
    printModuleStatus();
  } else {
    note(PSTR("Неизв. команда: %s"), command);
  }
}

void steeringSetup() {
  // Сначала безопасные уровни в выходных регистрах, потом включаем выходы:
  // при старте нет ни тяги, ни лишнего шага, драйвер выключен, тормоз взведён.
  digitalWrite(BLDC_SPEED_PIN, LOW);
  digitalWrite(BLDC_DIR_PIN, BLDC_DIR_FORWARD_LEVEL);
  digitalWrite(BLDC_BRAKE_PIN, BLDC_BRAKE_ACTIVE_LEVEL);
  digitalWrite(BLDC_STOP_PIN, BLDC_STOP_DISABLE_LEVEL);
  pinMode(BLDC_DIR_PIN, OUTPUT);
  pinMode(BLDC_BRAKE_PIN, OUTPUT);
  pinMode(BLDC_STOP_PIN, OUTPUT);
  // Servo работает с 0, а BRAKE/STOP отдельно держат выходы выключенными.
  setupDriveServo();
  driveSetBrake(true);
  driveSetStop(true);

  pinMode(OPTO_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);          // без подтяжки: потенциометр — делитель 5 В/GND
  analogReference(DEFAULT);         // опора АЦП = 5 В платы, как питание потенциометра
  digitalWrite(STEP_PUL_PIN, LOW);
  digitalWrite(STEP_DIR_PIN, STEP_DIR_PLUS_LEVEL);
  digitalWrite(STEP_ENA_PIN, STEP_ENABLE_LEVEL);  // как в проверенном скетче: сразу HIGH
  pinMode(STEP_PUL_PIN, OUTPUT);
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_ENA_PIN, OUTPUT);

  setupStepTimer();
  last_drive_command_ms = millis();
  last_pot_sample_ms = millis();
}

void steeringLoop() {
  updateDriveWatchdog();
  updatePot();

  updateCalibration();
  updatePotCalibration();
  updateCycle();
  updateMoveReport();
  updateFeedback();
  txPump();
}
// ============================ STEERING CORE END ============================

// ==================== USB: команды и телеметрия колеса =====================
// Каждое колесо — своя плата со своим USB: сервер открывает все четыре порта,
// использует только вручную выбранный USB-порт. Здесь же
// страховка прошивки: команда с именем чужого колеса не выполняется.
char serial_line[48];
uint8_t serial_length = 0;
bool serial_overflow = false;
uint32_t serial_last_char_ms = 0;
// Обрывок строки без \n дольше секунды — мусор (наводка, сброс порта, эхо):
// выбрасываем, иначе он склеится со следующей командой и «съест» её.
const uint16_t SERIAL_LINE_TIMEOUT_MS = 1000;
const uint16_t TELEMETRY_PERIOD_MS = 250;   // как часто печатаем @TLM
uint32_t last_telemetry_ms = 0;

// Имя модуля в команде «w <имя> …»: своё колесо, ALL, чужое колесо или не имя.
const uint8_t MOD_UNKNOWN = 0;
const uint8_t MOD_MINE = 1;
const uint8_t MOD_OTHER = 2;
const uint8_t MOD_ALL = 3;

uint8_t classifyModuleToken(const char* token) {
  if (!token) return MOD_UNKNOWN;
  if (strcasecmp_P(token, PSTR(MODULE_ID)) == 0) return MOD_MINE;
  if (strcasecmp_P(token, PSTR("ALL")) == 0) return MOD_ALL;
  if (strcasecmp_P(token, PSTR("FL")) == 0 || strcasecmp_P(token, PSTR("FR")) == 0 ||
      strcasecmp_P(token, PSTR("RL")) == 0 || strcasecmp_P(token, PSTR("RR")) == 0) {
    return MOD_OTHER;
  }
  return MOD_UNKNOWN;
}

void printHelp() {
  note(PSTR("команды: k, q, c [°], c c, d [°] [f/b], h, s, g, z, p, ?"));
  note(PSTR("поворот: w [FL] 12.5 | w zero | w calib · тяга: v [FL] 0…%u [f/b], b"),
       (unsigned)DRIVE_DIRECT_MAX_VALUE);
  note(PSTR("Servo D3 50 Гц; t [FL] %% — прежняя команда; SC не считывается"));
}

// Телеметрия не содержит роли колеса: порт назначает оператор в программе.
void appendTelemetry() {
  const uint16_t status = moduleStatus();
  char deg[16];
  char tgt[16];
  char line[240];
  formatDegrees(stepNow(), deg, sizeof(deg));
  formatDegrees(targetNow(), tgt, sizeof(tgt));
  const char* home = !(status & STATUS_ZERO) ? "none"
      : ((status & STATUS_HOME_OPTO) ? "opto" : "manual");
  // sv — точное значение Servo.write; tach=0 обозначает отключённое измерение.
  // rpm и частоту SC не отправляем: ноль означал бы измеренную остановку.
  const int n = snprintf_P(line, sizeof(line),
      PSTR("@TLM deg=%s tgt=%s moving=%d cal=%d cycle=%d calib=%d opto=%d home=%s err=%u drive=%d fw=%u pot=%d pm=%d tach=0 dmax=%u sv=%u duty=%u t=%lu\r\n"),
      deg, tgt, (status & STATUS_MOVING) ? 1 : 0, (status & STATUS_ZERO) ? 1 : 0,
      (status & STATUS_CYCLE) ? 1 : 0, (status & STATUS_CALIB) ? 1 : 0, (status & STATUS_OPTO) ? 1 : 0,
      home, (unsigned)((status >> 8) & 0x0F), (int)drive_percent, (unsigned)FIRMWARE_PROTOCOL,
      (status & STATUS_POT) ? 1 : 0,
      (status & STATUS_POT_MODE_SUPPORTED) ? ((status & STATUS_POT_MODE) ? 1 : 0) : -1,
      (unsigned)DRIVE_MAX_PERCENT, (unsigned)drive_servo_value,
      (unsigned)driveValuePermille(), (unsigned long)millis());
  if (n <= 0 || n >= (int)sizeof(line)) return;
  txAppend(line, (uint16_t)n);
}

// Общая точка входа USB: «w [FL] …» разбирается на имя колеса и команду ядра,
// остальное уходит в handleModuleCommand как есть.
void handleWheelCommand(char* line) {
  char work[48];
  strncpy(work, line, sizeof(work) - 1);
  work[sizeof(work) - 1] = '\0';
  char* save = nullptr;
  char* command = strtok_r(work, " \t", &save);
  if (!command) return;
  if (strcasecmp_P(command, PSTR("?")) == 0 || strcasecmp_P(command, PSTR("help")) == 0) {
    printHelp();
    return;
  }
  // v [колесо] значение [f/b]. Имя снимается, число не преобразуется.
  if (strcasecmp_P(command, PSTR("v")) == 0) {
    char* value = strtok_r(nullptr, " \t", &save);
    const uint8_t kind = classifyModuleToken(value);
    if (kind == MOD_OTHER) return;
    if (kind == MOD_MINE || kind == MOD_ALL) value = strtok_r(nullptr, " \t", &save);
    char* direction = strtok_r(nullptr, " \t", &save);
    char* extra = strtok_r(nullptr, " \t", &save);
    if (!value || extra) {
      stopDriveLocal();
      return;
    }
    char local[32];
    const int written = direction
        ? snprintf_P(local, sizeof(local), PSTR("v %s %s"), value, direction)
        : snprintf_P(local, sizeof(local), PSTR("v %s"), value);
    if (written < 0 || written >= (int)sizeof(local)) {
      stopDriveLocal();
      return;
    }
    handleModuleCommand(local);
    return;
  }
  // Адресная прежняя команда t принимает своё имя, чужое — игнорирует.
  if (strcasecmp_P(command, PSTR("t")) == 0) {
    char* arg1 = strtok_r(nullptr, " \t", &save);
    const uint8_t kind = classifyModuleToken(arg1);
    if (kind == MOD_OTHER) return;
    if (kind == MOD_MINE || kind == MOD_ALL) {
      char* arg2 = strtok_r(nullptr, " \t", &save);
      char* extra = strtok_r(nullptr, " \t", &save);
      if (!arg2 || extra) {
        note(PSTR("Шаблон: t [FL] -%u…%u (0 — стоп); b — тормоз"),
             (unsigned)DRIVE_MAX_PERCENT, (unsigned)DRIVE_MAX_PERCENT);
        return;
      }
      char local[16];
      const int written = snprintf_P(local, sizeof(local), PSTR("t %s"), arg2);
      if (written < 0 || written >= (int)sizeof(local)) {
        stopDriveLocal();
        return;
      }
      handleModuleCommand(local);
      return;
    }
    if (kind == MOD_UNKNOWN && arg1 != nullptr && isalpha((unsigned char)arg1[0]) &&
        strtok_r(nullptr, " \t", &save)) {
      // «t FX 8» — не имя колеса и не число: подсказываем, какие имена есть.
      note(PSTR("неизвестный модуль %s (есть FL, FR, RL, RR, ALL)"), arg1);
      return;
    }
    // «t», «t 8» — прежняя команда на это колесо.
    handleModuleCommand(line);
    return;
  }
  if (strcasecmp_P(command, PSTR("w")) == 0) {
    char* arg1 = strtok_r(nullptr, " \t", &save);
    char* arg2 = strtok_r(nullptr, " \t", &save);
    char* extra = strtok_r(nullptr, " \t", &save);
    const uint8_t kind = classifyModuleToken(arg1);
    if (kind == MOD_OTHER) return;   // команда чужого колеса — молча пропускаем
    if (kind == MOD_MINE || kind == MOD_ALL) {
      if (!arg2 || extra) {
        note(PSTR("Шаблон: w [FL] 12.5 | w zero | w calib"));
        return;
      }
      // «w FL z» и «w FL k» — короткие имена, как в прежнем мастере.
      char local[32];
      if (strcasecmp_P(arg2, PSTR("z")) == 0) {
        strcpy_P(local, PSTR("w zero"));
      } else if (strcasecmp_P(arg2, PSTR("k")) == 0) {
        strcpy_P(local, PSTR("w calib"));
      } else {
        snprintf_P(local, sizeof(local), PSTR("w %s"), arg2);
      }
      handleModuleCommand(local);
      return;
    }
    if (kind == MOD_UNKNOWN && arg2) {
      note(PSTR("неизвестный модуль %s (есть FL, FR, RL, RR, ALL)"), arg1);
      return;
    }
    // «w 12.5», «w zero», «w calib» — своя команда без имени колеса.
    handleModuleCommand(line);
    return;
  }
  handleModuleCommand(line);
  if (strcasecmp_P(command, PSTR("p")) == 0) {
    note(PSTR("колесо %s, прошивка v%u (USB)"), MODULE_ID, (unsigned)FIRMWARE_PROTOCOL);
  }
}

void readSerialLines() {
  if ((serial_length > 0 || serial_overflow) &&
      (uint32_t)(millis() - serial_last_char_ms) > SERIAL_LINE_TIMEOUT_MS) {
    serial_length = 0;
    serial_overflow = false;
  }
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    serial_last_char_ms = millis();
    if (ch == '\n' || ch == '\r') {
      if (serial_overflow) {
        note(PSTR("слишком длинная строка — пропущена"));
      } else if (serial_length > 0) {
        serial_line[serial_length] = '\0';
        if (serial_line[0] == '@') {
          // Любой @ — входное эхо/мусор, не запрос определения колеса.
        } else if (serial_line[0] != '#') {
          handleWheelCommand(serial_line);
        }
      }
      serial_length = 0;
      serial_overflow = false;
    } else if (serial_length < sizeof(serial_line) - 1) {
      serial_line[serial_length++] = ch;
    } else {
      serial_overflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  steeringSetup();
  last_telemetry_ms = millis();
  note(PSTR("--- КОЛЕСО %s: руль v%u, USB 115200; ноль не задан — k или z ---"),
       MODULE_ID, (unsigned)FIRMWARE_PROTOCOL);
  potBootNote();
  printHelp();
}

void loop() {
  readSerialLines();
  steeringLoop();
  const uint32_t now = millis();
  if ((uint32_t)(now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now;
    // Блок @TLM должен уйти целиком: обрезанный — сервер не разберёт.
    if (txFree() >= 210) appendTelemetry();
  }
  txPump();
}
