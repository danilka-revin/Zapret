#include <Wire.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

// Один Mega 2560 / Mega Mini — мастер четырёх рулевых модулей.
// FL управляется локально, FR/RL/RR — три платы-слейва по I2C.
// USB Serial протокол с сервером: "w FL 12.0" или "w FR zero".

const uint8_t SLAVE_ADDRESSES[] = {2, 3, 4};
const char* const SLAVE_MODULE_IDS[] = {"FR", "RL", "RR"};
const uint8_t NUM_SLAVES = 3;
const char* const MODULE_ID = "FL";

// Распиновка каждого контроллера (Arduino Mega 2560 / Mega Mini).
const uint8_t TACH_PIN = 2;          // BLDC SC, импульсный выход тахометра
const uint8_t BLDC_SPEED_PIN = 3;    // BLDC speed: PWM (предел команды 20%)
const uint8_t BLDC_DIR_PIN = 4;      // BLDC DIR
const uint8_t BLDC_BRAKE_PIN = 5;    // BLDC BRAKE
const uint8_t STEP_PUL_PIN = 6;      // TB6600 PUL+
const uint8_t STEP_DIR_PIN = 7;      // TB6600 DIR+
const uint8_t STEP_ENA_PIN = 8;      // TB6600 ENA+
// D18 / оптопара намеренно не объявляются и не читаются в этой версии.

// Подстройте под механическую передачу и микрошаг TB6600:
// steps/degree = шагов_двигателя_за_оборот * микрошаг * передаточное_число / 360.
const float STEPS_PER_DEGREE = 133.333333f;
const float STEER_LIMIT_DEG = 32.0f;
const uint16_t STEP_PERIOD_US = 1000; // 1 кГц, медленный безопасный тестовый ход
const uint8_t TACH_PULSES_PER_REV = 1; // поменяйте под ваш BLDC-тахометр
const uint32_t I2C_CLOCK_HZ = 100000UL;
const uint16_t TELEMETRY_PERIOD_MS = 250;
const uint16_t SLAVE_POLL_PERIOD_MS = 250;
const uint16_t RPM_SAMPLE_PERIOD_MS = 500;
const int8_t DRIVE_MAX_PERCENT = 20;           // безопасный программный предел PWM
const uint16_t DRIVE_COMMAND_TIMEOUT_MS = 650; // потеря heartbeat → тормоз
const float DRIVE_ALIGNMENT_TOLERANCE_DEG = 2.5f;

// Полярность входов зависит от конкретного BLDC/TB6600 и схемы подключения.
// Для схемы с активным HIGH это корректные значения; проверьте их до подключения мотора.
const uint8_t BLDC_BRAKE_ACTIVE_LEVEL = HIGH;
const uint8_t BLDC_DIR_FORWARD_LEVEL = LOW;
const uint8_t STEPPER_ENABLE_ACTIVE_LEVEL = HIGH;
const uint8_t STEP_DIR_POSITIVE_LEVEL = LOW;

volatile int32_t current_step = 0;
volatile int32_t target_step = 0;
volatile bool is_moving = false;
volatile bool pulse_state = false;
volatile uint16_t tick_count = 0;
volatile uint32_t tach_edges = 0;

bool software_zero_set = false;
bool drive_active = false;
int8_t drive_percent = 0;
uint32_t last_drive_command_ms = 0;
int32_t slave_current_steps[NUM_SLAVES] = {0, 0, 0};
bool slave_is_moving[NUM_SLAVES] = {false, false, false};
bool slave_zero_set[NUM_SLAVES] = {false, false, false};
int8_t slave_drive_percent[NUM_SLAVES] = {0, 0, 0};
uint32_t slave_last_seen_ms[NUM_SLAVES] = {0, 0, 0};
uint32_t last_rpm_edges = 0;
uint32_t rpm_x10 = 0;
uint32_t last_rpm_ms = 0;
uint32_t last_telemetry_ms = 0;
uint32_t last_slave_poll_ms = 0;

char serial_line[48];
uint8_t serial_length = 0;
char tx_buf[512];
uint16_t tx_length = 0;
uint16_t tx_position = 0;

void setupStepTimer();
void processSerialLine(char* line);
bool setSoftwareZero();
bool moveToSteeringAngle(float degrees);
void updateRpm();
void buildTelemetry();
void appendTelemetry(const char* module, int32_t current, int32_t target,
                     bool moving, bool zeroed, uint32_t module_rpm_x10,
                     int8_t module_drive_percent);
void pollSlaves();
bool parseSlaveFrame(char* line, int32_t* current, int32_t* target,
                     bool* moving, bool* zeroed, uint32_t* module_rpm_x10,
                     int8_t* module_drive_percent);
bool handleNodeCommand(uint8_t address, const char* command);
bool driveReady();
void setDriveCommand(int8_t percent);
void applyDriveCommand(int8_t percent);
void stopDriveLocal();
void stopAllDrive();
void updateDriveWatchdog();
void formatFixed(long value, uint8_t decimals, char* out, size_t out_size);
void formatTenths(long value, char* out, size_t out_size);

void tachometerISR() {
  ++tach_edges;
}

// Timer1 CTC, 20 kHz (50 мкс). Длительность высокого и низкого уровня
// делится пополам; счётчик шага меняется по спаду импульса.
void setupStepTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11); // 16 MHz / 8 / (99 + 1) = 20 kHz
  OCR1A = 99;
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  if (!is_moving) {
    tick_count = 0;
    if (pulse_state) {
      digitalWrite(STEP_PUL_PIN, LOW);
      pulse_state = false;
    }
    return;
  }

  const int32_t current = current_step;
  const int32_t target = target_step;
  if (current == target) {
    is_moving = false;
    tick_count = 0;
    return;
  }

  // Одна половина импульса = STEP_PERIOD_US / 2; тик таймера = 50 мкс.
  uint16_t half_period_ticks = (STEP_PERIOD_US + 99U) / 100U;
  if (half_period_ticks < 1) half_period_ticks = 1;
  if (++tick_count < half_period_ticks) return;
  tick_count = 0;

  pulse_state = !pulse_state;
  digitalWrite(STEP_PUL_PIN, pulse_state ? HIGH : LOW);
  if (!pulse_state) {
    if (target > current) ++current_step;
    else --current_step;
    if (current_step == target_step) is_moving = false;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachometerISR, RISING);

  // Сначала выставляем безопасные уровни на выходных регистрах, затем включаем
  // драйверы пинов — уменьшаем вероятность импульса тяги/шага при старте.
  digitalWrite(BLDC_SPEED_PIN, LOW);
  digitalWrite(BLDC_DIR_PIN, BLDC_DIR_FORWARD_LEVEL);
  digitalWrite(BLDC_BRAKE_PIN, BLDC_BRAKE_ACTIVE_LEVEL);
  pinMode(BLDC_SPEED_PIN, OUTPUT);
  pinMode(BLDC_DIR_PIN, OUTPUT);
  pinMode(BLDC_BRAKE_PIN, OUTPUT);
  analogWrite(BLDC_SPEED_PIN, 0);

  digitalWrite(STEP_PUL_PIN, LOW);
  digitalWrite(STEP_DIR_PIN, STEP_DIR_POSITIVE_LEVEL);
  digitalWrite(STEP_ENA_PIN, STEPPER_ENABLE_ACTIVE_LEVEL == HIGH ? LOW : HIGH);
  pinMode(STEP_PUL_PIN, OUTPUT);
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_ENA_PIN, OUTPUT);

  setupStepTimer();
  last_rpm_ms = millis();
  last_telemetry_ms = millis();
  last_slave_poll_ms = millis();
  last_drive_command_ms = millis();
}

void loop() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serial_length > 0) {
        serial_line[serial_length] = '\0';
        processSerialLine(serial_line);
        serial_length = 0;
      }
    } else if (serial_length < sizeof(serial_line) - 1) {
      serial_line[serial_length++] = ch;
    } else {
      // Слишком длинная строка целиком отбрасывается.
      serial_length = 0;
    }
  }

  updateDriveWatchdog();
  updateRpm();
  const uint32_t now = millis();
  if ((uint32_t)(now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS &&
      tx_position >= tx_length) {
    last_telemetry_ms = now;
    buildTelemetry();
  }
  while (tx_position < tx_length && Serial.availableForWrite() > 0) {
    Serial.write((uint8_t)tx_buf[tx_position++]);
  }
}

void processSerialLine(char* line) {
  char* save = nullptr;
  char* command = strtok_r(line, " \t", &save);
  if (!command) return;

  if (strcasecmp(command, "b") == 0) {
    stopAllDrive();
    return;
  }
  if (strcasecmp(command, "d") == 0) {
    char* value = strtok_r(nullptr, " \t", &save);
    char* extra = strtok_r(nullptr, " \t", &save);
    if (!value || extra) {
      stopAllDrive();
      return;
    }
    char* end = nullptr;
    const long percent = strtol(value, &end, 10);
    if (end == value || *end != '\0' || percent < -DRIVE_MAX_PERCENT ||
        percent > DRIVE_MAX_PERCENT) {
      stopAllDrive();
      return;
    }
    setDriveCommand((int8_t)percent);
    return;
  }
  if (strcasecmp(command, "w") != 0) return;

  char* module = strtok_r(nullptr, " \t", &save);
  char* value = strtok_r(nullptr, " \t", &save);
  char* extra = strtok_r(nullptr, " \t", &save);
  if (!module || !value || extra) return;
  for (char* p = module; *p; ++p) *p = (char)toupper((unsigned char)*p);
  if (strcmp(module, "FL") == 0) {
    if (strcasecmp(value, "zero") == 0) {
      setSoftwareZero();
      return;
    }
    char* end = nullptr;
    const double degrees = strtod(value, &end);
    if (end == value || *end != '\0' || !isfinite(degrees)) return;
    moveToSteeringAngle((float)degrees);
    return;
  }

  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    if (strcmp(module, SLAVE_MODULE_IDS[i]) != 0) continue;
    if (strcasecmp(value, "zero") == 0) {
      handleNodeCommand(SLAVE_ADDRESSES[i], "z");
      return;
    }
    char* end = nullptr;
    const double degrees = strtod(value, &end);
    if (end == value || *end != '\0' || !isfinite(degrees) || fabs(degrees) > STEER_LIMIT_DEG) return;
    char command_buffer[20];
    char angle_buffer[14];
    formatTenths(lround(degrees * 10.0), angle_buffer, sizeof(angle_buffer));
    snprintf(command_buffer, sizeof(command_buffer), "w %s", angle_buffer);
    handleNodeCommand(SLAVE_ADDRESSES[i], command_buffer);
    return;
  }
}

bool driveReady() {
  int32_t local_steps;
  bool local_moving;
  noInterrupts();
  local_steps = current_step;
  local_moving = is_moving;
  interrupts();
  if (!software_zero_set || local_moving) return false;

  const float reference_angle = (float)local_steps / STEPS_PER_DEGREE;
  const uint32_t now = millis();
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    if (!slave_last_seen_ms[i] || (uint32_t)(now - slave_last_seen_ms[i]) > 1000 ||
        !slave_zero_set[i] || slave_is_moving[i]) return false;
    const float angle = (float)slave_current_steps[i] / STEPS_PER_DEGREE;
    if (fabsf(angle - reference_angle) > DRIVE_ALIGNMENT_TOLERANCE_DEG) return false;
  }
  return true;
}

void applyDriveCommand(int8_t percent) {
  const uint8_t direction = percent >= 0
      ? BLDC_DIR_FORWARD_LEVEL
      : (BLDC_DIR_FORWARD_LEVEL == HIGH ? LOW : HIGH);
  const bool reversing = drive_active &&
      ((percent > 0) != (drive_percent > 0));
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

void stopAllDrive() {
  stopDriveLocal();
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    handleNodeCommand(SLAVE_ADDRESSES[i], "b");
  }
}

void setDriveCommand(int8_t percent) {
  if (percent == 0) {
    stopAllDrive();
    return;
  }
  if (abs((int)percent) > DRIVE_MAX_PERCENT || !driveReady()) {
    stopAllDrive();
    return;
  }
  char command[8];
  snprintf(command, sizeof(command), "d %d", (int)percent);
  bool all_nodes_acknowledged = true;
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    if (!handleNodeCommand(SLAVE_ADDRESSES[i], command)) {
      all_nodes_acknowledged = false;
    }
  }
  if (!all_nodes_acknowledged) {
    // Fail closed: no local BLDC motion if any slave did not acknowledge.
    // Reachable nodes receive brake now; an unreachable node has its own watchdog.
    stopAllDrive();
    return;
  }
  applyDriveCommand(percent);
}

void updateDriveWatchdog() {
  if (drive_active && (uint32_t)(millis() - last_drive_command_ms) > DRIVE_COMMAND_TIMEOUT_MS) {
    stopAllDrive();
  }
}

bool setSoftwareZero() {
  noInterrupts();
  const bool busy = is_moving || drive_active;
  if (!busy) {
    current_step = 0;
    target_step = 0;
    pulse_state = false;
    tick_count = 0;
  }
  interrupts();
  if (busy) return false;
  digitalWrite(STEP_PUL_PIN, LOW);
  software_zero_set = true;
  return true;
}

bool moveToSteeringAngle(float degrees) {
  if (!isfinite(degrees) || fabs(degrees) > STEER_LIMIT_DEG ||
      !software_zero_set || drive_active) return false;

  int32_t current;
  bool busy;
  noInterrupts();
  current = current_step;
  busy = is_moving;
  interrupts();
  if (busy) return false;

  const int32_t target = (int32_t)lroundf(degrees * STEPS_PER_DEGREE);
  if (target == current) return true;

  const bool positive = target > current;
  const uint8_t direction_level = positive
      ? STEP_DIR_POSITIVE_LEVEL
      : (STEP_DIR_POSITIVE_LEVEL == HIGH ? LOW : HIGH);
  digitalWrite(STEP_ENA_PIN, STEPPER_ENABLE_ACTIVE_LEVEL);
  digitalWrite(STEP_DIR_PIN, direction_level);
  delayMicroseconds(10); // DIR setup time before the first STEP edge
  noInterrupts();
  target_step = target;
  is_moving = true;
  tick_count = 0;
  interrupts();
  return true;
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

void formatFixed(long value, uint8_t decimals, char* out, size_t out_size) {
  const long scale = decimals == 1 ? 10L : 100L;
  const long abs_value = value < 0 ? -value : value;
  if (decimals == 1) {
    snprintf(out, out_size, "%s%ld.%01ld", value < 0 ? "-" : "",
             abs_value / scale, abs_value % scale);
  } else {
    snprintf(out, out_size, "%s%ld.%02ld", value < 0 ? "-" : "",
             abs_value / scale, abs_value % scale);
  }
}

void formatTenths(long value, char* out, size_t out_size) {
  formatFixed(value, 1, out, out_size);
}

void appendTelemetry(const char* module, int32_t current, int32_t target,
                     bool moving, bool zeroed, uint32_t module_rpm_x10,
                     int8_t module_drive_percent) {
  char deg_buffer[16];
  char target_buffer[16];
  char rpm_buffer[16];
  char line[128];
  formatFixed(lround((double)current / STEPS_PER_DEGREE * 100.0), 2,
              deg_buffer, sizeof(deg_buffer));
  formatFixed(lround((double)target / STEPS_PER_DEGREE * 100.0), 2,
              target_buffer, sizeof(target_buffer));
  formatFixed((long)module_rpm_x10, 1, rpm_buffer, sizeof(rpm_buffer));
  const int n = snprintf(line, sizeof(line),
      "@TLM mod=%s deg=%s tgt=%s moving=%d cal=%d cycle=0 calib=0 opto=0 rpm=%s drive=%d t=%lu\r\n",
      module, deg_buffer, target_buffer, moving ? 1 : 0, zeroed ? 1 : 0,
      rpm_buffer, (int)module_drive_percent, (unsigned long)millis());
  if (n <= 0 || n >= (int)sizeof(line) || tx_length + (uint16_t)n >= sizeof(tx_buf)) return;
  memcpy(tx_buf + tx_length, line, (size_t)n);
  tx_length += (uint16_t)n;
}

void buildTelemetry() {
  tx_length = 0;
  tx_position = 0;
  noInterrupts();
  const int32_t current = current_step;
  const int32_t target = target_step;
  const bool moving = is_moving;
  interrupts();
  appendTelemetry(MODULE_ID, current, target, moving, software_zero_set, rpm_x10, drive_percent);

  const uint32_t now = millis();
  if ((uint32_t)(now - last_slave_poll_ms) >= SLAVE_POLL_PERIOD_MS) {
    last_slave_poll_ms = now;
    pollSlaves();
  }
}

bool parseSlaveFrame(char* line, int32_t* current, int32_t* target,
                     bool* moving, bool* zeroed, uint32_t* module_rpm_x10,
                     int8_t* module_drive_percent) {
  long fields[6];
  char* p = line;
  for (uint8_t i = 0; i < 6; ++i) {
    char* end = nullptr;
    fields[i] = strtol(p, &end, 10);
    if (end == p) return false;
    p = end;
    if (i < 5) {
      if (*p != ',') return false;
      ++p;
    }
  }
  if (*p != '\0' && *p != '\n' && *p != '\r') return false;
  if ((fields[2] != 0 && fields[2] != 1) ||
      (fields[3] != 0 && fields[3] != 1) || fields[4] < 0 ||
      fields[5] < -DRIVE_MAX_PERCENT || fields[5] > DRIVE_MAX_PERCENT) return false;
  *current = (int32_t)fields[0];
  *target = (int32_t)fields[1];
  *moving = fields[2] == 1;
  *zeroed = fields[3] == 1;
  *module_rpm_x10 = (uint32_t)fields[4];
  *module_drive_percent = (int8_t)fields[5];
  return true;
}

void pollSlaves() {
  for (uint8_t i = 0; i < NUM_SLAVES; ++i) {
    const uint8_t received = Wire.requestFrom(SLAVE_ADDRESSES[i], (uint8_t)32);
    if (received == 0) continue;
    char buffer[40];
    uint8_t length = 0;
    while (Wire.available() && length < sizeof(buffer) - 1) {
      const char ch = (char)Wire.read();
      if (ch == '\n') break;
      if (ch != '\r') buffer[length++] = ch;
    }
    while (Wire.available()) Wire.read();
    buffer[length] = '\0';

    int32_t current, target;
    bool moving, zeroed;
    uint32_t module_rpm_x10;
    int8_t module_drive_percent;
    if (!parseSlaveFrame(buffer, &current, &target, &moving, &zeroed,
                         &module_rpm_x10, &module_drive_percent)) continue;
    slave_current_steps[i] = current;
    slave_is_moving[i] = moving;
    slave_zero_set[i] = zeroed;
    slave_drive_percent[i] = module_drive_percent;
    slave_last_seen_ms[i] = millis();
    appendTelemetry(SLAVE_MODULE_IDS[i], current, target, moving, zeroed,
                    module_rpm_x10, module_drive_percent);
  }
}

bool handleNodeCommand(uint8_t address, const char* command) {
  Wire.beginTransmission(address);
  const size_t length = strlen(command);
  const size_t written = Wire.write((const uint8_t*)command, length);
  const uint8_t result = Wire.endTransmission();
  return written == length && result == 0;
}
