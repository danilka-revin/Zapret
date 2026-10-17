#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// motor_master — Arduino Mega 2560 / Mega Mini: рулевой модуль FL и мастер
// I2C для трёх слейвов FR/RL/RR (прошивка motor_node, адреса 2/3/4).
//
// USB 115200, строки заканчиваются \n (в мониторе порта — «NL» или «NL & CR»).
//
// Команды монитора — как в проверенном скетче, сразу для ВСЕХ четырёх колёс:
//   k              калибровка по оптодатчику D18
//   q              калибровка потенциометра A0 и доводка положения колеса
//   c <°> | c c    качание на угол / выключить качание
//   d <°> f|b      повернуть на угол: f — вправо, b — влево
//   h  в ноль · s  стоп всего · g  продолжить · z  ноль здесь · p  состояние
//
// Команды сервера (бортовой экран, RC_SLAM_SOURCE=master):
//   w FL 12.5      абсолютный угол одного модуля (FL/FR/RL/RR, предел ±32°)
//   w FR zero      ноль одного модуля в текущем положении
//   w RL calib     калибровка одного модуля по датчику
//   t 8            тяга всех BLDC, −20…20 %; heartbeat не реже 650 мс
//   b              тормоз всех BLDC
//
// Телеметрия каждые 250 мс, по строке на модуль:
//   @TLM mod=FL deg=0.00 tgt=0.00 moving=0 cal=1 cycle=0 calib=0 opto=1
//        home=opto err=0 rpm=0.0 drive=0 fw=2 pot=1 t=12345
// Строки «# FL: …» — сообщения для человека (сервер пишет их в журнал экрана).

#define MODULE_ID "FL"
#define TX_BUFFER_SIZE 768

// =========================== STEERING CORE BEGIN ===========================
// Этот блок одинаков в motor_master.ino и motor_node.ino (тест
// tests/test_firmware.py сверяет его побайтно): меняете — меняйте в обоих.
//
// Руль повторяет скетч, проверенный на железе (firmware/reference/steering_tested):
// те же пины и уровни, 133.33 шага/°, 100 мкс на шаг в работе и 2000 мкс при
// точном подходе к оптодатчику D18, те же команды монитора (k c d h s g p z).
// Отличия от него:
//   * шаги выдаёт Timer1 (тик 50 мкс), а не loop(): опрос I2C и печать в порт
//     не сбивают ход мотора;
//   * датчик проверяется в том же прерывании — кромка ловится с точностью до
//     шага, без запаздывания loop();
//   * калибровка доведена: выход из флажка перед медленным подходом, поиск
//     влево действительно включается, ноль ставится без зависаний, есть
//     таймаут и отмена командой s.

// ---- распиновка (одинакова на всех четырёх платах) ----
const uint8_t TACH_PIN = 2;          // BLDC SC — тахометр (прерывание INT4)
const uint8_t BLDC_SPEED_PIN = 3;    // BLDC speed — PWM (Timer3, Timer1 не трогает)
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
const uint8_t BLDC_BRAKE_ACTIVE_LEVEL = HIGH;  // проверьте по своему контроллеру BLDC
const uint8_t BLDC_DIR_FORWARD_LEVEL = LOW;

// ---- механика и скорость руля ----
const float STEPS_PER_DEGREE = 133.333333f;    // шагов_на_оборот × микрошаг × редуктор / 360
const uint16_t STEP_PERIOD_US = 100;           // рабочий ход: 10 кГц ≈ 75°/с
const uint16_t CALIB_STEP_PERIOD_US = 2000;    // точный подход к датчику: 500 Гц ≈ 3,75°/с
const float STEER_LIMIT_DEG = 32.0f;           // предел абсолютной команды w (как у сервера)
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
// меняет угол. A0 свободен на всех четырёх платах (D2–D8, D18, I2C D20/D21 заняты).
const int16_t POT_ADC_MAX = 1023;
const int16_t POT_RAIL_MARGIN = 4;               // 0…4 и 1019…1023 — обрыв, КЗ или упор
const float POT_NOMINAL_COUNTS_PER_DEG = 1023.0f / 270.0f;  // 270° на весь ход
const uint16_t POT_SAMPLE_PERIOD_MS = 10;        // один отсчёт АЦП раз в 10 мс
const uint8_t POT_FILTER_SHIFT = 2;              // EMA 1/4: шум АЦП падает вчетверо
const float POT_CORRECTION_DEADBAND_DEG = 0.7f;  // меньше — мотор не дёргаем
const uint8_t POT_CORRECTION_MAX_PASSES = 3;     // поправок на один поворот
const uint16_t POT_CORRECTION_SETTLE_MS = 60;    // механика успокаивается после стопа
const float POT_RESYNC_DEG = 0.25f;              // счётчик шагов привязать к датчику
const float POT_CAL_POINT_DEG = 60.0f;           // автокалибровка наклона: −60° и +60°
const float POT_CAL_MIN_COUNTS_PER_DEG = 1.5f;   // проверка правдоподобия наклона
const float POT_CAL_MAX_COUNTS_PER_DEG = 8.0f;
const uint32_t POT_CAL_TIMEOUT_MS = 30000UL;

// ---- тяга BLDC ----
const uint8_t TACH_PULSES_PER_REV = 1;         // импульсов SC на оборот колеса
const uint16_t RPM_SAMPLE_PERIOD_MS = 500;
const int8_t DRIVE_MAX_PERCENT = 20;           // программный предел PWM
const uint16_t DRIVE_COMMAND_TIMEOUT_MS = 650; // нет heartbeat → PWM 0 и тормоз

const uint8_t FIRMWARE_PROTOCOL = 2;           // поле fw= в телеметрии @TLM

// ---- состояние руля (общие с прерыванием поля — volatile) ----
volatile int32_t current_step = 0;
volatile int32_t target_step = 0;
volatile bool is_moving = false;
volatile bool pulse_high = false;
volatile int8_t step_dir = 1;
volatile uint8_t half_period_ticks = 1;
volatile uint8_t tick_count = 0;
volatile uint32_t tach_edges = 0;

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

// Биты состояния модуля: кадр слейва по I2C и поля телеметрии @TLM.
const uint16_t STATUS_MOVING = 0x01;
const uint16_t STATUS_ZERO = 0x02;
const uint16_t STATUS_CALIB = 0x04;
const uint16_t STATUS_OPTO = 0x08;
const uint16_t STATUS_CYCLE = 0x10;
const uint16_t STATUS_HOME_OPTO = 0x20;
const uint16_t STATUS_V2 = 0x40;       // признак кадра этой версии
const uint16_t STATUS_POT = 0x80;      // потенциометр откалиброван, доводка включена
// биты 8–11: код ошибки калибровки, биты 12–15: счётчик кадров слейва

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
uint32_t last_rpm_edges = 0;
uint32_t rpm_x10 = 0;
uint32_t last_rpm_ms = 0;

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
// Шаги: Timer1 в режиме CTC, 16 МГц / 8 / 100 = 20 кГц, тик 50 мкс.
void tachometerISR() {
  ++tach_edges;
}

bool optoActive() {
  const int level = digitalRead(OPTO_PIN);
  return OPTO_ACTIVE_LOW ? level == LOW : level == HIGH;
}

void setupStepTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A = 99;
  TCNT1 = 0;
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

// Полпериода шага = STEP_PERIOD_US / 100 тиков: 100 мкс → 50 мкс HIGH и 50 мкс
// LOW, ровно 10 кГц, как micros()-генератор проверенного скетча. Шаг считается
// по спаду PUL; начатый импульс при остановке доводится до конца и засчитывается.
ISR(TIMER1_COMPA_vect) {
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
  return pot_calibrated && pot_zero_valid && !pot_railed && zero_set;
}

// Ноль руля задан (k или z) — запомнить, где стоит движок потенциометра.
// Наклон (отсчёты/градус), измеренный q, при повторном нуле сохраняется.
void potCaptureZero() {
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

void potBootNote() {
  const int16_t raw = potReadAverage(POT_CAL_SAMPLES);
  pot_raw = raw;
  pot_sampled = true;
  pot_railed = potRawOnRail(raw);
  if (pot_railed) {
    note(PSTR("потенциометр A0: %d — на краю, проверьте провод"), (int)raw);
  } else {
    note(PSTR("потенциометр A0: %d — ноль: k/z, калибровка: q"), (int)raw);
  }
}

void potPrintStatus() {
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
        pot_zero_valid = false;
        note(PSTR("ОШИБКА потенциометра: нет сигнала или неверный ход"));
      } else {
        pot_cal_ok = true;
        pot_zero_raw = pot_cal_zero_raw;
        pot_counts_per_degree = slope;
        pot_zero_valid = true;
        pot_calibrated = true;
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
  if (pot_zero_valid && !pot_railed &&
      fabsf(actual - step_angle) > POT_RESYNC_DEG) {
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
// Тяга BLDC: только при заданном нуле и стоящем руле, с watchdog 650 мс.
bool localDriveAllowed() {
  return zero_set && !motorBusy() && calib_state == CAL_IDLE &&
         pot_cal_state == POT_CAL_IDLE && !cycle_active;
}

void applyDriveCommand(int8_t percent) {
  const uint8_t direction = percent >= 0
      ? BLDC_DIR_FORWARD_LEVEL
      : (BLDC_DIR_FORWARD_LEVEL == HIGH ? LOW : HIGH);
  const bool reversing = drive_active && ((percent > 0) != (drive_percent > 0));
  if (!drive_active || reversing) {
    analogWrite(BLDC_SPEED_PIN, 0);
    digitalWrite(BLDC_BRAKE_PIN, BLDC_BRAKE_ACTIVE_LEVEL);
    digitalWrite(BLDC_DIR_PIN, direction);
    delayMicroseconds(100);
  }
  const uint8_t pwm = (uint8_t)((abs((int)percent) * 255 + 50) / 100);
  analogWrite(BLDC_SPEED_PIN, pwm);
  digitalWrite(BLDC_BRAKE_PIN, BLDC_BRAKE_ACTIVE_LEVEL == HIGH ? LOW : HIGH);
  drive_percent = percent;
  drive_active = percent != 0;
  last_drive_command_ms = millis();
}

void stopDriveLocal() {
  analogWrite(BLDC_SPEED_PIN, 0);
  digitalWrite(BLDC_BRAKE_PIN, BLDC_BRAKE_ACTIVE_LEVEL);
  drive_percent = 0;
  drive_active = false;
}

bool applyLocalDrive(int8_t percent) {
  if (percent == 0) {
    stopDriveLocal();
    return true;
  }
  if (percent > DRIVE_MAX_PERCENT || percent < -DRIVE_MAX_PERCENT || !localDriveAllowed() ||
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
    note(PSTR("тяга: нет команды дольше 650 мс — PWM 0, тормоз"));
  }
}

void updateRpm() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - last_rpm_ms;
  if (elapsed < RPM_SAMPLE_PERIOD_MS) return;
  noInterrupts();
  const uint32_t edges = tach_edges;
  interrupts();
  const uint32_t delta = edges - last_rpm_edges;
  last_rpm_edges = edges;
  last_rpm_ms = now;
  if (TACH_PULSES_PER_REV == 0) {
    rpm_x10 = 0;
  } else {
    rpm_x10 = (uint32_t)(((uint64_t)delta * 600000ULL) /
                         ((uint64_t)elapsed * TACH_PULSES_PER_REV));
  }
}

// ---------------------------------------------------------------------------
uint16_t moduleStatus() {
  uint16_t status = STATUS_V2;
  if (motorBusy()) status |= STATUS_MOVING;
  if (zero_set) status |= STATUS_ZERO;
  if (calib_state != CAL_IDLE) status |= STATUS_CALIB;
  if (optoActive()) status |= STATUS_OPTO;
  if (cycle_active) status |= STATUS_CYCLE;
  if (zero_set && home_source == HOME_OPTO) status |= STATUS_HOME_OPTO;
  if (potFeedbackAllowed()) status |= STATUS_POT;
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
}

// Команды одного рулевого модуля. Мастер выполняет их для FL и пересылает те
// же строки слейвам (как проверенный скетч), слейв принимает их по I2C и со
// своего USB — так любой модуль можно проверить на столе по одному.
//   k | calib      калибровка по оптодатчику
//   q              калибровка потенциометра A0: 0°, −60°, +60° (доводка по датчику)
//   z | zero       ноль в текущем положении (вручную)
//   h              в ноль           s   стоп всего (руль, калибровка, цикл, тяга)
//   g              продолжить поворот, прерванный s
//   d <°> f|b      повернуть на угол: f — вправо (+), b — влево (−)
//   c <°> | c c    качание на угол / выключить качание
//   w <°>          абсолютный угол (нужен ноль, предел ±32°) — команда сервера
//   t <%> | b      тяга BLDC −20…20 % / тормоз
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
      note(PSTR("w: угол вне ±32°"));
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
        percent < -DRIVE_MAX_PERCENT || percent > DRIVE_MAX_PERCENT) {
      stopDriveLocal();
      return;
    }
    applyLocalDrive((int8_t)percent);
  } else if (strcasecmp_P(command, PSTR("b")) == 0) {
    stopDriveLocal();
  } else if (strcasecmp_P(command, PSTR("p")) == 0) {
    printModuleStatus();
  } else {
    note(PSTR("Неизв. команда: %s"), command);
  }
}

void steeringSetup() {
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachometerISR, RISING);

  // Сначала безопасные уровни в выходных регистрах, потом включаем выходы:
  // при старте нет ни импульса тяги, ни лишнего шага.
  digitalWrite(BLDC_SPEED_PIN, LOW);
  digitalWrite(BLDC_DIR_PIN, BLDC_DIR_FORWARD_LEVEL);
  digitalWrite(BLDC_BRAKE_PIN, BLDC_BRAKE_ACTIVE_LEVEL);
  pinMode(BLDC_SPEED_PIN, OUTPUT);
  pinMode(BLDC_DIR_PIN, OUTPUT);
  pinMode(BLDC_BRAKE_PIN, OUTPUT);
  analogWrite(BLDC_SPEED_PIN, 0);

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
  last_rpm_ms = millis();
  last_drive_command_ms = millis();
  last_pot_sample_ms = millis();
}

void steeringLoop() {
  updateDriveWatchdog();
  updatePot();
  updateRpm();
  updateCalibration();
  updatePotCalibration();
  updateCycle();
  updateMoveReport();
  updateFeedback();
  txPump();
}
// ============================ STEERING CORE END ============================

// ============================== мастер I2C ==================================
const uint8_t SLAVE_ADDRESSES[] = {2, 3, 4};
const char* const SLAVE_MODULE_IDS[] = {"FR", "RL", "RR"};
const uint8_t NUM_SLAVES = 3;
const uint32_t I2C_CLOCK_HZ = 400000UL;       // как в проверенном скетче
const uint16_t TELEMETRY_PERIOD_MS = 250;
const uint16_t SLAVE_STALE_MS = 1000;
const float DRIVE_ALIGNMENT_TOLERANCE_DEG = 2.5f;

int32_t slave_current[NUM_SLAVES] = {0, 0, 0};
int32_t slave_target[NUM_SLAVES] = {0, 0, 0};
uint16_t slave_status[NUM_SLAVES] = {0, 0, 0};
uint32_t slave_rpm_x10[NUM_SLAVES] = {0, 0, 0};
int8_t slave_drive[NUM_SLAVES] = {0, 0, 0};
uint32_t slave_last_seen_ms[NUM_SLAVES] = {0, 0, 0};
uint8_t slave_seq[NUM_SLAVES] = {0xFF, 0xFF, 0xFF};
bool slave_online[NUM_SLAVES] = {false, false, false};
bool slave_fresh[NUM_SLAVES] = {false, false, false};

uint32_t last_telemetry_ms = 0;
char serial_line[48];
uint8_t serial_length = 0;
bool serial_overflow = false;
uint32_t serial_last_char_ms = 0;
// Обрывок строки без \n дольше секунды — мусор (наводка, сброс порта, эхо):
// выбрасываем, иначе он склеится со следующей командой и «съест» её.
const uint16_t SERIAL_LINE_TIMEOUT_MS = 1000;

bool sendToSlave(uint8_t index, const char* command) {
  Wire.beginTransmission(SLAVE_ADDRESSES[index]);
  const size_t length = strlen(command);
  const size_t written = Wire.write((const uint8_t*)command, length);
  const uint8_t result = Wire.endTransmission();
  return written == length && result == 0;
}

bool slaveFresh(uint8_t index, uint32_t now) {
  return slave_online[index] && (uint32_t)(now - slave_last_seen_ms[index]) <= SLAVE_STALE_MS;
}

// Кадр слейва: "ток,цель,состояние,об/мин×10,тяга\n", не длиннее 32 байт.
bool parseSlaveFrame(char* line, int32_t* current, int32_t* target, uint16_t* status,
                     uint32_t* module_rpm_x10, int8_t* module_drive) {
  long fields[5];
  char* p = line;
  for (uint8_t i = 0; i < 5; ++i) {
    char* end = nullptr;
    fields[i] = strtol(p, &end, 10);
    if (end == p) return false;
    p = end;
    if (i < 4) {
      if (*p != ',') return false;
      ++p;
    }
  }
  if (*p != '\0') return false;
  if (fields[2] < 0 || fields[2] > 0xFFFFL || !(fields[2] & STATUS_V2) || fields[3] < 0 ||
      fields[4] < -DRIVE_MAX_PERCENT || fields[4] > DRIVE_MAX_PERCENT) return false;
  *current = (int32_t)fields[0];
  *target = (int32_t)fields[1];
  *status = (uint16_t)fields[2];
  *module_rpm_x10 = (uint32_t)fields[3];
  *module_drive = (int8_t)fields[4];
  return true;
}

void reportSlaveChange(uint8_t index, uint16_t before, uint16_t now_status) {
  const char* module = SLAVE_MODULE_IDS[index];
  if (!slave_online[index]) {
    noteModule(module, PSTR("на связи по I2C (адрес %d)"), (int)SLAVE_ADDRESSES[index]);
    return;
  }
  if (!(before & STATUS_CALIB) && (now_status & STATUS_CALIB)) {
    noteModule(module, PSTR("Калибровка: начата"));
  } else if ((before & STATUS_CALIB) && !(now_status & STATUS_CALIB)) {
    const uint8_t error = (uint8_t)((now_status >> 8) & 0x0F);
    if (error == CAL_OK && (now_status & STATUS_ZERO)) {
      noteModule(module, PSTR("Калибровка ЗАВЕРШЕНА! Точка 0 установлена по датчику."));
    } else {
      char text[112];
      copyCalibErrorText(error, text, sizeof(text));
      noteModule(module, PSTR("ОШИБКА калибровки: %s"), text);
    }
  }
  if (!(before & STATUS_POT) && (now_status & STATUS_POT)) {
    noteModule(module, PSTR("Потенциометр откалиброван — доводка включена"));
  } else if ((before & STATUS_POT) && !(now_status & STATUS_POT)) {
    noteModule(module, PSTR("Потенциометр: доводка выключена"));
  }
}

void pollSlaves() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    slave_fresh[i] = false;
    const uint8_t received = Wire.requestFrom(SLAVE_ADDRESSES[i], (uint8_t)32);
    char buffer[40];
    uint8_t length = 0;
    bool complete = false;
    while (Wire.available()) {
      const char ch = (char)Wire.read();
      if (complete) continue;
      if (ch == '\n') {
        complete = true;
      } else if (ch != '\r' && length < sizeof(buffer) - 1) {
        buffer[length++] = ch;
      }
    }
    buffer[length] = '\0';
    int32_t current = 0;
    int32_t target = 0;
    uint16_t status = 0;
    uint32_t module_rpm_x10 = 0;
    int8_t module_drive = 0;
    const bool parsed = received > 0 && complete &&
        parseSlaveFrame(buffer, &current, &target, &status, &module_rpm_x10, &module_drive);
    if (parsed) {
      // Кадр с тем же номером — loop() слейва не обновляет его: связи нет.
      const uint8_t seq = (uint8_t)(status >> 12);
      if (seq != slave_seq[i]) {
        slave_seq[i] = seq;
        reportSlaveChange(i, slave_status[i], status);
        slave_current[i] = current;
        slave_target[i] = target;
        slave_status[i] = status;
        slave_rpm_x10[i] = module_rpm_x10;
        slave_drive[i] = module_drive;
        slave_last_seen_ms[i] = now;
        slave_online[i] = true;
        slave_fresh[i] = true;
      }
    }
    if (slave_online[i] && (uint32_t)(now - slave_last_seen_ms[i]) > SLAVE_STALE_MS) {
      slave_online[i] = false;
      noteModule(SLAVE_MODULE_IDS[i], PSTR("нет связи по I2C (адрес %d)"), (int)SLAVE_ADDRESSES[i]);
    }
  }
}

void appendTelemetry(const char* module, int32_t current, int32_t target, uint16_t status,
                     uint32_t module_rpm_x10, int8_t module_drive) {
  char deg[16];
  char tgt[16];
  char rpm[16];
  char line[176];
  formatDegrees(current, deg, sizeof(deg));
  formatDegrees(target, tgt, sizeof(tgt));
  formatFixed((long)module_rpm_x10, 1, rpm, sizeof(rpm));
  const char* home = !(status & STATUS_ZERO) ? "none" : ((status & STATUS_HOME_OPTO) ? "opto" : "manual");
  const int n = snprintf_P(line, sizeof(line),
      PSTR("@TLM mod=%s deg=%s tgt=%s moving=%d cal=%d cycle=%d calib=%d opto=%d home=%s err=%u rpm=%s drive=%d fw=%u pot=%d t=%lu\r\n"),
      module, deg, tgt, (status & STATUS_MOVING) ? 1 : 0, (status & STATUS_ZERO) ? 1 : 0,
      (status & STATUS_CYCLE) ? 1 : 0, (status & STATUS_CALIB) ? 1 : 0, (status & STATUS_OPTO) ? 1 : 0,
      home, (unsigned)((status >> 8) & 0x0F), rpm, (int)module_drive, (unsigned)FIRMWARE_PROTOCOL,
      (status & STATUS_POT) ? 1 : 0, (unsigned long)millis());
  if (n <= 0 || n >= (int)sizeof(line)) return;
  txAppend(line, (uint16_t)n);
}

void buildTelemetry() {
  appendTelemetry(MODULE_ID, stepNow(), targetNow(), moduleStatus(), rpm_x10, drive_percent);
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    if (!slave_fresh[i]) continue;
    appendTelemetry(SLAVE_MODULE_IDS[i], slave_current[i], slave_target[i], slave_status[i],
                    slave_rpm_x10[i], slave_drive[i]);
  }
}

// ------------------------------- тяга ---------------------------------------
bool driveReady() {
  if (!localDriveAllowed()) return false;
  const int32_t reference = stepNow();
  const uint32_t now = millis();
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    if (!slaveFresh(i, now)) return false;
    const uint16_t status = slave_status[i];
    if (!(status & STATUS_ZERO) || (status & (STATUS_MOVING | STATUS_CALIB | STATUS_CYCLE))) return false;
    const float difference = (float)(slave_current[i] - reference) / STEPS_PER_DEGREE;
    if (fabsf(difference) > DRIVE_ALIGNMENT_TOLERANCE_DEG) return false;
  }
  return true;
}

void stopAllDrive() {
  stopDriveLocal();
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    if (!sendToSlave(i, "b")) sendToSlave(i, "b");
  }
}

void setDriveCommand(int8_t percent) {
  if (percent == 0 || percent > DRIVE_MAX_PERCENT || percent < -DRIVE_MAX_PERCENT || !driveReady()) {
    stopAllDrive();
    return;
  }
  char command[8];
  snprintf_P(command, sizeof(command), PSTR("t %d"), (int)percent);
  bool all_nodes_acknowledged = true;
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    if (!sendToSlave(i, command)) all_nodes_acknowledged = false;
  }
  // Fail closed: ни один BLDC не едет, если хоть один слейв не принял команду.
  if (!all_nodes_acknowledged || !applyLocalDrive(percent)) stopAllDrive();
}

// ----------------------------- команды USB ----------------------------------
// Как проверенный скетч: одна строка уходит всем трём слейвам и выполняется на FL.
void broadcastCommand(const char* command) {
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    if (!sendToSlave(i, command) && !sendToSlave(i, command)) {
      noteModule(SLAVE_MODULE_IDS[i], PSTR("команда «%s» не доставлена по I2C (адрес %d)"),
                 command, (int)SLAVE_ADDRESSES[i]);
    }
  }
  char local[48];
  strncpy(local, command, sizeof(local) - 1);
  local[sizeof(local) - 1] = '\0';
  handleModuleCommand(local);
}

void printReport() {
  printModuleStatus();
  const uint32_t now = millis();
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    const char* module = SLAVE_MODULE_IDS[i];
    if (!slaveFresh(i, now)) {
      noteModule(module, PSTR("нет связи по I2C (адрес %d)"), (int)SLAVE_ADDRESSES[i]);
      continue;
    }
    const uint16_t status = slave_status[i];
    char current[16];
    char target[16];
    formatDegrees(slave_current[i], current, sizeof(current));
    formatDegrees(slave_target[i], target, sizeof(target));
    const char* zero = !(status & STATUS_ZERO) ? "не задан" : ((status & STATUS_HOME_OPTO) ? "по датчику" : "вручную");
    const char* state = (status & STATUS_CALIB) ? "КАЛИБРОВКА" : ((status & STATUS_CYCLE) ? "ЦИКЛ" : ((status & STATUS_MOVING) ? "ДВИЖ" : "СТОИТ"));
    noteModule(module, PSTR("Текущий: %s° | Целевой: %s° [%s] · ноль %s · датчик %s · тяга %d%%"),
               current, target, state, zero, (status & STATUS_OPTO) ? "закрыт" : "свободен",
               (int)slave_drive[i]);
    noteModule(module, PSTR("Потенциометр: %s"),
               (status & STATUS_POT) ? "калиброван, доводка включена" : "не калиброван (q)");
  }
}

void printHelp() {
  note(PSTR("все колёса: k, c [°], c c, d [°] [f/b], h, s, g, z, p · один модуль: w FL 12.5 | w FL zero | w FL calib · тяга: t [%%], b"));
  note(PSTR("q — калибровка потенциометра A0 (колёса качаются ±60°), p — состояние"));
}

void handleServerSteer(char* module, char* value, char* extra) {
  if (!module || !value || extra) {
    note(PSTR("Шаблон: w FL 12.5 | w FL zero | w FL calib"));
    return;
  }
  for (char* p = module; *p; ++p) *p = (char)toupper((unsigned char)*p);
  char command[24];
  if (strcasecmp_P(value, PSTR("zero")) == 0 || strcasecmp_P(value, PSTR("z")) == 0) {
    strcpy_P(command, PSTR("z"));
  } else if (strcasecmp_P(value, PSTR("calib")) == 0 || strcasecmp_P(value, PSTR("k")) == 0) {
    strcpy_P(command, PSTR("k"));
  } else {
    float degrees = 0.0f;
    if (!parseFloatToken(value, &degrees) || fabsf(degrees) > STEER_LIMIT_DEG) {
      noteModule(module, PSTR("w: угол вне ±32°"));
      return;
    }
    char angle[16];
    formatFixed(lroundf(degrees * 10.0f), 1, angle, sizeof(angle));
    snprintf_P(command, sizeof(command), PSTR("w %s"), angle);
  }
  if (strcmp_P(module, PSTR(MODULE_ID)) == 0) {
    handleModuleCommand(command);
    return;
  }
  if (strcmp_P(module, PSTR("ALL")) == 0) {
    broadcastCommand(command);
    return;
  }
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    if (strcmp(module, SLAVE_MODULE_IDS[i]) != 0) continue;
    if (!sendToSlave(i, command) && !sendToSlave(i, command)) {
      noteModule(SLAVE_MODULE_IDS[i], PSTR("команда «%s» не доставлена по I2C (адрес %d)"),
                 command, (int)SLAVE_ADDRESSES[i]);
    }
    return;
  }
  note(PSTR("неизвестный модуль %s (есть FL, FR, RL, RR, ALL)"), module);
}

void processSerialLine(char* line) {
  // Пробелы по краям и повторные пробелы убираем: слейвам уходит ровная строка.
  char normalized[48];
  uint8_t length = 0;
  bool space = false;
  for (const char* p = line; *p && length < sizeof(normalized) - 1; ++p) {
    const bool blank = *p == ' ' || *p == '\t';
    if (blank) {
      space = length > 0;
      continue;
    }
    if (space && length < sizeof(normalized) - 2) normalized[length++] = ' ';
    space = false;
    normalized[length++] = *p;
  }
  normalized[length] = '\0';
  if (length == 0) return;
  // «@TLM …» и «# …» — наш же вывод (эхо терминала), командой не бывает.
  if (normalized[0] == '@' || normalized[0] == '#') return;

  char work[48];
  memcpy(work, normalized, length + 1);
  char* save = nullptr;
  char* command = strtok_r(work, " ", &save);
  if (!command) return;

  if (strcasecmp_P(command, PSTR("w")) == 0) {
    char* module = strtok_r(nullptr, " ", &save);
    char* value = strtok_r(nullptr, " ", &save);
    char* extra = strtok_r(nullptr, " ", &save);
    handleServerSteer(module, value, extra);
  } else if (strcasecmp_P(command, PSTR("t")) == 0) {
    char* value = strtok_r(nullptr, " ", &save);
    char* extra = strtok_r(nullptr, " ", &save);
    long percent = 0;
    if (!value || extra || !parseLongToken(value, &percent) ||
        percent < -DRIVE_MAX_PERCENT || percent > DRIVE_MAX_PERCENT) {
      stopAllDrive();
      return;
    }
    setDriveCommand((int8_t)percent);
  } else if (strcasecmp_P(command, PSTR("b")) == 0) {
    stopAllDrive();
  } else if (strcasecmp_P(command, PSTR("p")) == 0) {
    printReport();
  } else if (strcasecmp_P(command, PSTR("?")) == 0 || strcasecmp_P(command, PSTR("help")) == 0) {
    printHelp();
  } else if (strcasecmp_P(command, PSTR("k")) == 0 || strcasecmp_P(command, PSTR("calib")) == 0 ||
             strcasecmp_P(command, PSTR("s")) == 0 || strcasecmp_P(command, PSTR("g")) == 0 ||
             strcasecmp_P(command, PSTR("h")) == 0 || strcasecmp_P(command, PSTR("z")) == 0 ||
             strcasecmp_P(command, PSTR("zero")) == 0 || strcasecmp_P(command, PSTR("d")) == 0 ||
             strcasecmp_P(command, PSTR("c")) == 0 || strcasecmp_P(command, PSTR("q")) == 0) {
    broadcastCommand(normalized);
  } else {
    note(PSTR("Неизв. команда: %s (список: ?)"), command);
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
        processSerialLine(serial_line);
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
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);  // зависшая шина I2C не останавливает loop()
#endif
  steeringSetup();
  last_telemetry_ms = millis();
  note(PSTR("--- MASTER: руль v%u, датчики: ноль D18, пот A0; ноль не задан — k или z ---"),
       (unsigned)FIRMWARE_PROTOCOL);
  potBootNote();
  printHelp();
}

void loop() {
  readSerialLines();
  steeringLoop();
  const uint32_t now = millis();
  if ((uint32_t)(now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now;
    pollSlaves();
    if (txFree() >= 4 * 150) buildTelemetry();
  }
  txPump();
}
