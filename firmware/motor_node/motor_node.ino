#include <Wire.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

// Одна и та же прошивка для трёх слейвов.
// Перед загрузкой меняйте NODE_ADDRESS и MODULE_ID в конфигурации ниже:
// FR = 2, RL = 3, RR = 4. Мастер — отдельная прошивка motor_master.
#ifndef NODE_ADDRESS
#define NODE_ADDRESS 2
#endif
#ifndef MODULE_ID
#define MODULE_ID "FR"
#endif

// Распиновка каждого контроллера (Arduino Mega 2560 / Mega Mini).
const uint8_t TACH_PIN = 2;          // BLDC SC, импульсный выход тахометра
const uint8_t BLDC_SPEED_PIN = 3;    // BLDC speed: PWM (предел команды 20%)
const uint8_t BLDC_DIR_PIN = 4;      // BLDC DIR
const uint8_t BLDC_BRAKE_PIN = 5;    // BLDC BRAKE
const uint8_t STEP_PUL_PIN = 6;      // TB6600 PUL+
const uint8_t STEP_DIR_PIN = 7;      // TB6600 DIR+
const uint8_t STEP_ENA_PIN = 8;      // TB6600 ENA+
// D18 / оптопара намеренно не объявляются и не читаются в этой версии.

const float STEPS_PER_DEGREE = 133.333333f;
const float STEER_LIMIT_DEG = 32.0f;
const uint16_t STEP_PERIOD_US = 1000;
const uint8_t TACH_PULSES_PER_REV = 1;
const uint16_t RPM_SAMPLE_PERIOD_MS = 500;
const uint16_t TELEMETRY_PERIOD_MS = 50;
const int8_t DRIVE_MAX_PERCENT = 20;
const uint16_t DRIVE_COMMAND_TIMEOUT_MS = 650;

// Проверьте полярность по документации вашего контроллера/драйвера.
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
uint32_t last_rpm_edges = 0;
uint32_t rpm_x10 = 0;
uint32_t last_rpm_ms = 0;
uint32_t last_telemetry_ms = 0;

volatile bool rx_ready = false;
volatile uint8_t rx_length = 0;
volatile char rx_buf[32];
char telemetry[2][40];
volatile uint8_t telemetry_length[2] = {0, 0};
volatile uint8_t telemetry_front = 0;
uint8_t telemetry_back = 1;

void setupStepTimer();
void onCommandReceived(int count);
void onTelemetryRequested();
void processNodeCommand(char* command);
bool setSoftwareZero();
bool moveToSteeringAngle(float degrees);
void updateRpm();
void refreshTelemetry();
void applyDriveCommand(int8_t percent);
void stopDriveLocal();
void updateDriveWatchdog();
void formatFixed(long value, uint8_t decimals, char* out, size_t out_size);

void tachometerISR() {
  ++tach_edges;
}

void setupStepTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11); // CTC, 20 kHz, тик 50 мкс
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

void onCommandReceived(int count) {
  (void)count;
  uint8_t n = 0;
  while (Wire.available() && n < sizeof(rx_buf) - 1) {
    rx_buf[n++] = (char)Wire.read();
  }
  while (Wire.available()) Wire.read();
  rx_buf[n] = '\0';
  rx_length = n;
  rx_ready = true;
}

void onTelemetryRequested() {
  const uint8_t index = telemetry_front;
  const uint8_t length = telemetry_length[index];
  if (length > 0) Wire.write((const uint8_t*)telemetry[index], length);
}

void setup() {
  Wire.begin(NODE_ADDRESS);
  Wire.onReceive(onCommandReceived);
  Wire.onRequest(onTelemetryRequested);

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
  last_drive_command_ms = millis();
  refreshTelemetry();
}

void loop() {
  if (rx_ready) {
    char command[32];
    uint8_t length;
    noInterrupts();
    length = rx_length;
    if (length >= sizeof(command)) length = sizeof(command) - 1;
    for (uint8_t i = 0; i < length; ++i) command[i] = rx_buf[i];
    command[length] = '\0';
    rx_ready = false;
    interrupts();
    processNodeCommand(command);
  }

  updateDriveWatchdog();
  updateRpm();
  const uint32_t now = millis();
  if ((uint32_t)(now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now;
    refreshTelemetry();
  }
}

void processNodeCommand(char* command) {
  while (*command == ' ' || *command == '\t') ++command;
  if (strcasecmp(command, "b") == 0) {
    stopDriveLocal();
    return;
  }
  if (tolower((unsigned char)command[0]) == 'd' &&
      (command[1] == ' ' || command[1] == '\t')) {
    char* value = command + 2;
    while (*value == ' ' || *value == '\t') ++value;
    char* end = nullptr;
    const long percent = strtol(value, &end, 10);
    while (end && (*end == ' ' || *end == '\t')) ++end;
    if (end == value || !end || *end != '\0' || percent < -DRIVE_MAX_PERCENT ||
        percent > DRIVE_MAX_PERCENT) {
      stopDriveLocal();
      return;
    }
    if (percent == 0) {
      stopDriveLocal();
      return;
    }
    if (!software_zero_set || is_moving ||
        (drive_active && ((percent > 0) != (drive_percent > 0)))) {
      stopDriveLocal();
      return;
    }
    applyDriveCommand((int8_t)percent);
    return;
  }
  if (strcasecmp(command, "z") == 0 || strcasecmp(command, "zero") == 0) {
    setSoftwareZero();
    return;
  }

  if (tolower((unsigned char)command[0]) != 'w' ||
      (command[1] != ' ' && command[1] != '\t')) return;
  char* value = command + 2;
  while (*value == ' ' || *value == '\t') ++value;
  char* end = nullptr;
  const double degrees = strtod(value, &end);
  while (end && (*end == ' ' || *end == '\t')) ++end;
  if (end == value || !end || *end != '\0' || !isfinite(degrees)) return;
  moveToSteeringAngle((float)degrees);
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

void updateDriveWatchdog() {
  if (drive_active && (uint32_t)(millis() - last_drive_command_ms) > DRIVE_COMMAND_TIMEOUT_MS) {
    stopDriveLocal();
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
  delayMicroseconds(10);
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
  if (TACH_PULSES_PER_REV == 0) rpm_x10 = 0;
  else rpm_x10 = (uint32_t)(((uint64_t)delta * 600000ULL) /
                            ((uint64_t)elapsed * TACH_PULSES_PER_REV));
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

void refreshTelemetry() {
  int32_t current;
  int32_t target;
  bool moving;
  noInterrupts();
  current = current_step;
  target = target_step;
  moving = is_moving;
  interrupts();

  char line[40];
  const int n = snprintf(line, sizeof(line), "%ld,%ld,%d,%d,%lu,%d\n",
      (long)current, (long)target, moving ? 1 : 0,
      software_zero_set ? 1 : 0, (unsigned long)rpm_x10, (int)drive_percent);
  if (n <= 0 || n >= (int)sizeof(line)) return;

  const uint8_t back = telemetry_back;
  memcpy(telemetry[back], line, (size_t)n);
  telemetry_length[back] = (uint8_t)n;
  noInterrupts();
  telemetry_front = back;
  telemetry_back = back ^ 1;
  interrupts();
}
