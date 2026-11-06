/* Модуль датчиков робота: GY-87, RC522, адресная лента, 4 реле, замок, концевик.

Пины (Mega 2560), ровно как в разводке:

  GY-87 (I²C, 5 В):        SDA — 20, SCL — 21
  RC522 (SPI, 3.3 В!):     RST — 9, MISO — 50, MOSI — 51, SCK — 52, SS — 53
  Адресная лента:          DIN — 6, питание от внешнего 5 В, GND общий
  Реле 4 канала (5 В):     IN1 — 2, IN2 — 3, IN3 — 4, IN4 — 5
  Замок двери (серво):     сигнал — 7  (питание серво — отдельное, не с 5 В Mega)
  Концевик двери:          8 → GND, INPUT_PULLUP (замкнут = дверь закрыта)

Свет:
  реле 1 — передний свет (head)      реле 3 — левый поворотник (left)
  реле 2 — стоп-сигнал (stop)        реле 4 — правый поворотник (right)
  габариты, стоп и поворотники дублируются адресной лентой; яркость габаритов
  задаётся командой dim.

Связь с сервером — только USB Serial 115200, строки с CRC-16/Modbus.
Это не аутентификация и не аппаратный E-STOP: CRC ловит только порчу линии.

  @S fw=1 hdg=12345 pitch=-120 roll=35 gz=12 press=69964 temp=150 alt=301674
     door=0 lock=0 light=5 dim=80 rfid=0 cards=2 link=1 err=0 t=123456
     crc=ABCD                                         (5 Гц)
  @CARD uid=A1B2C3D4 ok=1 cards=2 t=1234 crc=ABCD     (событие карты)
  @EVT kind=door open=1 t=1234 crc=ABCD               (дверь/замок)
  @ACK id=3 cmd=lock ok=1 t=1234 crc=ABCD             (ответ на команду)

Команды (строки, \r\n): ping, stat, ?,
  light head|stop|left|right|dim on|off, light all off|on,
  dim 0…255, strip auto|off,
  lock open [1…10] , lock close,
  card list | card add A1B2C3D4 | card learn | card del A1B2C3D4 | card clear,
  mag cal start | mag cal stop | mag cal reset

Безопасность: после сброса реле выключены, замок закрыт. Замок открывается
только на время (1…10 с) и закрывается сам; без команды от сервера дольше
LINK_TIMEOUT_MS он закрывается принудительно. Карта открывает замок только
если она записана в EEPROM и дверь закрыта. Прошивка не заменяет физический
ключ и не является сертифицированной СКУД.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <stdarg.h>
#include <avr/wdt.h>

// ------------------------------- версия и пины ------------------------------
//: Версия протокола модуля датчиков. Меняется вместе с разбором на сервере.
const uint8_t SENSOR_PROTOCOL = 1;
//: Своё π: strict ISO C++ не отдаёт M_PI, а avr-gcc считает float быстрее double.
static const float PI_F = 3.14159265358979f;

static const uint8_t RC522_SS_PIN = 53;
static const uint8_t RC522_RST_PIN = 9;
static const uint8_t STRIP_PIN = 6;
static const uint8_t RELAY_PINS[4] = {2, 3, 4, 5};
//: У большинства модулей реле входы активны низким уровнем. Проверьте свой:
//: при HIGH на IN реле должно молчать. Если наоборот — поставьте false.
static const bool RELAY_ACTIVE_LOW = true;
static const uint8_t LOCK_SERVO_PIN = 7;
static const uint8_t DOOR_SWITCH_PIN = 8;
//: Замкнут на GND = дверь закрыта.
static const uint8_t DOOR_CLOSED_LEVEL = LOW;

//: Сколько светодиодов в ленте. Лента длиннее — увеличьте, короче — уменьшите.
static const uint16_t STRIP_LEDS = 30;
static const uint8_t STRIP_BRIGHTNESS = 96;   // общий предел яркости, 0…255

//: Углы серво замка: 0 — закрыто, 90 — открыто. Подберите под свой замок.
static const uint8_t LOCK_CLOSED_ANGLE = 10;
static const uint8_t LOCK_OPEN_ANGLE = 80;
static const uint16_t LOCK_DEFAULT_MS = 3000;
static const uint16_t LOCK_MAX_MS = 10000;
//: Если сервер молчит дольше — замок закрывается сам (свет остаётся).
static const uint32_t LINK_TIMEOUT_MS = 10000;
//: Период опроса барометра: 2 Гц для высоты по давлению достаточно.
static const uint32_t BARO_PERIOD_MS = 500;

// ------------------------------- адреса I²C ---------------------------------
static const uint8_t MPU_ADDR = 0x68;         // MPU6050 (AD0 = GND)
static const uint8_t MAG_ADDR = 0x1E;         // HMC5883L
static const uint8_t BARO_ADDR = 0x77;        // BMP180

// Регистры MPU6050
static const uint8_t MPU_REG_PWR = 0x6B;
static const uint8_t MPU_REG_CONFIG = 0x1A;
static const uint8_t MPU_REG_SMPLRT = 0x19;
static const uint8_t MPU_REG_GYRO_CFG = 0x1B;
static const uint8_t MPU_REG_ACC_CFG = 0x1C;
static const uint8_t MPU_REG_ACCEL = 0x3B;    // 14 байт: accel, temp, gyro
static const uint8_t MPU_REG_WHOAMI = 0x75;

// Регистры HMC5883L. Порядок данных в FIFO — X, Z, Y (не X, Y, Z!).
static const uint8_t MAG_REG_CFG_A = 0x00;
static const uint8_t MAG_REG_CFG_B = 0x01;
static const uint8_t MAG_REG_MODE = 0x02;
static const uint8_t MAG_REG_DATA = 0x03;
static const uint8_t MAG_REG_ID_A = 0x0A;

// Регистры BMP180
static const uint8_t BARO_REG_CAL = 0xAA;     // 22 байта калибровки
static const uint8_t BARO_REG_CTRL = 0xF4;
static const uint8_t BARO_REG_DATA = 0xF6;
static const uint8_t BARO_REG_ID = 0xD0;
static const uint8_t BARO_CMD_TEMP = 0x2E;
static const uint8_t BARO_CMD_PRESS = 0x34;

// ------------------------------- регистры RC522 -----------------------------
static const uint8_t RC_REG_COMMAND = 0x01;
static const uint8_t RC_REG_COM_IRQ = 0x04;
static const uint8_t RC_REG_DIV_IRQ = 0x05;
static const uint8_t RC_REG_ERROR = 0x06;
static const uint8_t RC_REG_FIFO_DATA = 0x09;
static const uint8_t RC_REG_FIFO_LEVEL = 0x0A;
static const uint8_t RC_REG_CONTROL = 0x0C;
static const uint8_t RC_REG_BIT_FRAMING = 0x0D;
static const uint8_t RC_REG_TX_CONTROL = 0x14;
static const uint8_t RC_REG_TX_ASK = 0x15;
static const uint8_t RC_REG_MODE = 0x11;
static const uint8_t RC_REG_T_MODE = 0x2A;
static const uint8_t RC_REG_T_PRESCALER = 0x2B;
static const uint8_t RC_REG_T_RELOAD_H = 0x2C;
static const uint8_t RC_REG_T_RELOAD_L = 0x2D;
static const uint8_t RC_REG_VERSION = 0x37;

static const uint8_t RC_CMD_IDLE = 0x00;
static const uint8_t RC_CMD_TRANSCEIVE = 0x0C;
static const uint8_t RC_CMD_SOFT_RESET = 0x0F;

static const uint8_t PICC_WUPA = 0x52;
static const uint8_t PICC_SEL_CL1 = 0x93;
static const uint8_t PICC_SAK_CASCADE = 0x04;   // SAK с этим битом — UID длиннее 4 байт
static const uint8_t PICC_HALT = 0x50;

// ---------------------------------- статусы ---------------------------------
//: Биты ошибок в поле err= телеметрии.
static const uint8_t ERR_MPU = 1 << 0;
static const uint8_t ERR_MAG = 1 << 1;
static const uint8_t ERR_BARO = 1 << 2;
static const uint8_t ERR_RFID = 1 << 3;
static const uint8_t ERR_EEPROM = 1 << 4;
static const uint8_t ERR_LOCK = 1 << 5;
static const uint8_t ERR_I2C = 1 << 6;

//: Биты поля light= (что включено сейчас, с учётом мигания поворотников).
static const uint8_t LIGHT_HEAD = 1 << 0;
static const uint8_t LIGHT_STOP = 1 << 1;
static const uint8_t LIGHT_LEFT = 1 << 2;
static const uint8_t LIGHT_RIGHT = 1 << 3;
static const uint8_t LIGHT_DIM = 1 << 4;

//: Последнее событие RFID (поле rfid=).
static const uint8_t RFID_NONE = 0;
static const uint8_t RFID_GRANTED = 1;
static const uint8_t RFID_DENIED = 2;
static const uint8_t RFID_ENROLLED = 3;
static const uint8_t RFID_DELETED = 4;

// ------------------------------- EEPROM -------------------------------------
// Раскладка: карточки доступа и калибровка компаса в разных областях, у каждой
// свой CRC. Один повреждённый блок не стирает другой.
static const uint16_t CARD_EEPROM_ADDR = 0;
static const uint8_t CARD_MAGIC0 = 'S';
static const uint8_t CARD_MAGIC1 = 'N';
static const uint8_t CARD_MAGIC2 = 'S';
static const uint8_t CARD_VERSION = 1;
static const uint8_t CARD_SLOTS = 12;         // 12 карт × 4 байта UID
static const uint16_t CARD_BLOCK_BYTES = 6 + CARD_SLOTS * 4 + 2;

static const uint16_t MAG_EEPROM_ADDR = 64;
static const uint8_t MAG_MAGIC0 = 'M';
static const uint8_t MAG_MAGIC1 = 'G';
static const uint8_t MAG_VERSION = 1;

// ------------------------------- буфер вывода -------------------------------
#define TX_BUFFER_SIZE 512
static char tx_buf[TX_BUFFER_SIZE];
static uint16_t tx_length = 0;
static uint16_t tx_position = 0;

// ------------------------------- состояние ----------------------------------
struct CardSlot {
  uint8_t uid[4];
};

static CardSlot cards[CARD_SLOTS];
static uint8_t card_count = 0;
static bool card_learn = false;               // следующая карта будет записана

static uint8_t light_state = 0;               // биты LIGHT_*
static uint8_t dim_level = 80;                // яркость габаритов 0…255
static bool strip_auto = true;

static bool lock_open = false;
static uint32_t lock_deadline = 0;
static uint8_t lock_angle = LOCK_CLOSED_ANGLE;

static bool door_closed = true;
static bool door_raw = true;
static uint32_t door_changed_ms = 0;

static float mag_offset_x = 0.0f;             // поправка жёсткого смещения
static float mag_offset_y = 0.0f;
static bool mag_calibrating = false;
static bool mag_cal_armed = false;   // границы калибровки уже заданы первым замером
static float mag_min_x = 0.0f, mag_max_x = 0.0f;
static float mag_min_y = 0.0f, mag_max_y = 0.0f;

static int16_t heading_cd = 0;                // сотые доли градуса 0…35999
static int16_t pitch_cd = 0;
static int16_t roll_cd = 0;
static int16_t gyro_z_dps10 = 0;
static int32_t baro_pressure_pa = 0;
static int16_t baro_temp_c10 = 0;
static int32_t baro_altitude_cm = 0;

static uint8_t rfid_state = RFID_NONE;
static uint8_t error_mask = 0;
static bool link_ok = false;
static uint32_t last_command_ms = 0;
static uint32_t last_sensor_ms = 0;
static uint32_t last_baro_ms = 0;
static uint32_t last_telemetry_ms = 0;
static uint32_t last_blink_ms = 0;
static bool blink_phase = false;
static uint32_t command_id = 0;
static uint32_t last_card_seen_ms = 0;
static uint8_t last_card_uid[4] = {0, 0, 0, 0};

static char line_buf[96];
static uint8_t line_length = 0;

// BMP180 calibration
static int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
static uint16_t ac4, ac5, ac6;
static int32_t b5 = 0;
static bool baro_ready = false;


static Servo lockServo;
static Adafruit_NeoPixel strip(STRIP_LEDS, STRIP_PIN, NEO_GRB + NEO_KHZ800);

// ============================== вывод в USB ==================================
void txAppend(const char* data, uint16_t length) {
  if (tx_position >= tx_length) {
    tx_position = 0;
    tx_length = 0;
  }
  if (length == 0 || (uint32_t)tx_length + length > sizeof(tx_buf)) return;
  memcpy(tx_buf + tx_length, data, length);
  tx_length += length;
}

void txPump() {
  while (tx_position < tx_length && Serial.availableForWrite() > 0) {
    Serial.write((uint8_t)tx_buf[tx_position++]);
  }
}

//: CRC-16/Modbus по точному ASCII-хвосту кадра до поля crc=.
uint16_t frameCrc16(const char* data, uint16_t size) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < size; ++i) {
    crc ^= (uint8_t)data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001U : crc >> 1;
    }
  }
  return crc;
}

//: Дописать " crc=XXXX\r\n" и поставить строку в очередь передачи.
bool appendCheckedFrame(char* line, uint16_t capacity, int length) {
  if (length <= 0 || length >= (int)capacity) return false;
  const uint16_t crc = frameCrc16(line, (uint16_t)length);
  const int tail = snprintf_P(line + length, capacity - (uint16_t)length,
                              PSTR(" crc=%04X\r\n"), (unsigned)crc);
  if (tail <= 0 || tail >= (int)(capacity - (uint16_t)length)) return false;
  txAppend(line, (uint16_t)(length + tail));
  return true;
}

//: Сообщение человеку: «# SENSORS: текст». Сервер пишет его в журнал.
void note(const char* format_P, ...) {
  char line[160];
  int n = snprintf_P(line, sizeof(line) - 2, PSTR("# SENSORS: "));
  if (n < 0) return;
  va_list args;
  va_start(args, format_P);
  const int m = vsnprintf_P(line + n, sizeof(line) - 2 - (size_t)n, format_P, args);
  va_end(args);
  if (m < 0) return;
  n += m;
  if (n > (int)sizeof(line) - 3) n = (int)sizeof(line) - 3;
  line[n++] = '\r';
  line[n++] = '\n';
  txAppend(line, (uint16_t)n);
}

// ================================ I²C ========================================
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) {
    if (!Wire.available()) return false;
    buffer[i] = (uint8_t)Wire.read();
  }
  return true;
}

// =============================== MPU6050 =====================================
bool mpuInit() {
  uint8_t who = 0;
  if (!i2cRead(MPU_ADDR, MPU_REG_WHOAMI, &who, 1) || who != 0x68) return false;
  i2cWrite(MPU_ADDR, MPU_REG_PWR, 0x00);        // выйти из сна
  delay(50);
  i2cWrite(MPU_ADDR, MPU_REG_SMPLRT, 0x09);     // 100 Гц при DLPF 44 Гц
  i2cWrite(MPU_ADDR, MPU_REG_CONFIG, 0x03);
  i2cWrite(MPU_ADDR, MPU_REG_GYRO_CFG, 0x00);   // ±250 °/с
  i2cWrite(MPU_ADDR, MPU_REG_ACC_CFG, 0x00);    // ±2 g
  return true;
}

//: Один замер: accel в g, gyro в °/с. 0 — успех.
bool mpuRead(float* ax, float* ay, float* az, int16_t* gz_dps10) {
  uint8_t raw[14];
  if (!i2cRead(MPU_ADDR, MPU_REG_ACCEL, raw, sizeof(raw))) return false;
  const int16_t axr = (int16_t)((raw[0] << 8) | raw[1]);
  const int16_t ayr = (int16_t)((raw[2] << 8) | raw[3]);
  const int16_t azr = (int16_t)((raw[4] << 8) | raw[5]);
  const int16_t gzr = (int16_t)((raw[12] << 8) | raw[13]);
  *ax = axr / 16384.0f;
  *ay = ayr / 16384.0f;
  *az = azr / 16384.0f;
  *gz_dps10 = (int16_t)lroundf(gzr / 131.0f * 10.0f);
  return true;
}

// =============================== HMC5883L ====================================
bool magInit() {
  uint8_t id[3] = {0, 0, 0};
  if (!i2cRead(MAG_ADDR, MAG_REG_ID_A, id, 3) || id[0] != 'H' || id[1] != '4' || id[2] != '3') {
    return false;
  }
  i2cWrite(MAG_ADDR, MAG_REG_CFG_A, 0x70);      // 8 усреднений, 15 Гц, обычный замер
  i2cWrite(MAG_ADDR, MAG_REG_CFG_B, 0x20);      // усиление 1.3 Га
  return i2cWrite(MAG_ADDR, MAG_REG_MODE, 0x00);  // непрерывный режим
}

//: Магнитное поле в «сырых» отсчётах. Порядок регистров: X, Z, Y.
bool magRead(float* mx, float* my, float* mz) {
  uint8_t raw[6];
  if (!i2cRead(MAG_ADDR, MAG_REG_DATA, raw, sizeof(raw))) return false;
  const int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
  const int16_t z = (int16_t)((raw[3] << 8) | raw[2]);
  const int16_t y = (int16_t)((raw[5] << 8) | raw[4]);
  *mx = (float)x;
  *my = (float)y;
  *mz = (float)z;
  return !(x == 0 && y == 0 && z == 0);
}

// =============================== BMP180 ======================================
static int16_t be16(const uint8_t* p) { return (int16_t)((p[0] << 8) | p[1]); }

bool baroInit() {
  uint8_t id = 0;
  if (!i2cRead(BARO_ADDR, BARO_REG_ID, &id, 1) || id != 0x55) return false;
  uint8_t cal[22];
  if (!i2cRead(BARO_ADDR, BARO_REG_CAL, cal, sizeof(cal))) return false;
  ac1 = be16(cal + 0);
  ac2 = be16(cal + 2);
  ac3 = be16(cal + 4);
  ac4 = (uint16_t)be16(cal + 6);
  ac5 = (uint16_t)be16(cal + 8);
  ac6 = (uint16_t)be16(cal + 10);
  b1 = be16(cal + 12);
  b2 = be16(cal + 14);
  mb = be16(cal + 16);
  mc = be16(cal + 18);
  md = be16(cal + 20);
  return true;
}

//: Давление в Па и температура в 0.1 °C. Алгоритм из даташита BMP180.
bool baroRead(int32_t* pressure_pa, int16_t* temp_c10) {
  uint8_t raw[3];
  if (!i2cWrite(BARO_ADDR, BARO_REG_CTRL, BARO_CMD_TEMP)) return false;
  delay(5);
  if (!i2cRead(BARO_ADDR, BARO_REG_DATA, raw, 2)) return false;
  const int32_t ut = (int32_t)((raw[0] << 8) | raw[1]);
  const int32_t x1 = ((ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
  const int32_t x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
  b5 = x1 + x2;
  const int32_t t = (b5 + 8) >> 4;

  if (!i2cWrite(BARO_ADDR, BARO_REG_CTRL, BARO_CMD_PRESS)) return false;
  delay(5);
  if (!i2cRead(BARO_ADDR, BARO_REG_DATA, raw, 3)) return false;
  const int32_t up = (int32_t)(((uint32_t)raw[0] << 16 | (uint32_t)raw[1] << 8 | raw[2]) >> 8);
  int32_t b6 = b5 - 4000;
  int32_t x1b = ((int32_t)b2 * ((b6 * b6) >> 12)) >> 11;
  int32_t x2b = ((int32_t)ac2 * b6) >> 11;
  int32_t x3b = x1b + x2b;
  // OSS = 0 (одно преобразование): B3 = (((AC1·4 + X3) << OSS) + 2) / 4.
  int32_t b3 = (((int32_t)ac1 * 4 + x3b) + 2) >> 2;
  x1b = ((int32_t)ac3 * b6) >> 13;
  x2b = ((int32_t)b1 * ((b6 * b6) >> 12)) >> 16;
  x3b = ((x1b + x2b) + 2) >> 2;
  const uint32_t b4 = ((uint32_t)ac4 * (uint32_t)(x3b + 32768)) >> 15;
  const uint32_t b7 = ((uint32_t)up - b3) * 50000UL;
  int32_t p = b7 < 0x80000000UL ? (int32_t)((b7 * 2) / b4) : (int32_t)((b7 / b4) * 2);
  x1b = (p >> 8) * (p >> 8);
  x1b = (x1b * 3038) >> 16;
  x2b = (-7357 * p) >> 16;
  p = p + ((x1b + x2b + 3791) >> 4);
  *pressure_pa = p;
  *temp_c10 = (int16_t)t;
  return true;
}

// =============================== RC522 =======================================
void rcWrite(uint8_t reg, uint8_t value) {
  digitalWrite(RC522_SS_PIN, LOW);
  SPI.transfer((uint8_t)((reg << 1) & 0x7E));
  SPI.transfer(value);
  digitalWrite(RC522_SS_PIN, HIGH);
}

uint8_t rcRead(uint8_t reg) {
  digitalWrite(RC522_SS_PIN, LOW);
  SPI.transfer((uint8_t)(((reg << 1) & 0x7E) | 0x80));
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(RC522_SS_PIN, HIGH);
  return value;
}

void rcSetBits(uint8_t reg, uint8_t mask) { rcWrite(reg, rcRead(reg) | mask); }

void rcAntennaOn() {
  if ((rcRead(RC_REG_TX_CONTROL) & 0x03) != 0x03) rcSetBits(RC_REG_TX_CONTROL, 0x03);
}

bool rcReset() {
  digitalWrite(RC522_RST_PIN, LOW);
  delay(5);
  digitalWrite(RC522_RST_PIN, HIGH);
  delay(5);
  rcWrite(RC_REG_COMMAND, RC_CMD_SOFT_RESET);
  delay(5);
  for (uint8_t i = 0; i < 20; ++i) {
    if (rcRead(RC_REG_COMMAND) == RC_CMD_IDLE) break;
    delay(2);
  }
  // Внутренний таймер RC522: 13.56 МГц / (2·169 + 1) ≈ 40 кГц, 1000 тиков ≈ 25 мс.
  // Столько карта и получает на ответ; раньше таймаут ставил бы предел в доли мс.
  rcWrite(RC_REG_T_MODE, 0x80);
  rcWrite(RC_REG_T_PRESCALER, 0xA9);
  rcWrite(RC_REG_T_RELOAD_H, 0x03);
  rcWrite(RC_REG_T_RELOAD_L, 0xE8);
  rcWrite(RC_REG_TX_ASK, 0x40);
  rcWrite(RC_REG_MODE, 0x3D);
  rcAntennaOn();
  return rcRead(RC_REG_VERSION) != 0x00 && rcRead(RC_REG_VERSION) != 0xFF;
}

//: Начать обмен с картой. last_bits = 0 — целые байты; with_crc — дописать CRC_A.
//: Функция только запускает передачу: ответ читается в rcPoll(), поэтому loop()
//: не стоит 25 мс и свет/замок продолжают работать.
void rcStart(const uint8_t* frame, uint8_t length, uint8_t last_bits, bool with_crc) {
  rcWrite(RC_REG_COMMAND, RC_CMD_IDLE);
  rcWrite(RC_REG_COM_IRQ, 0x7F);          // сбросить флаги прерываний
  rcWrite(RC_REG_FIFO_LEVEL, 0x80);       // очистить FIFO
  for (uint8_t i = 0; i < length; ++i) rcWrite(RC_REG_FIFO_DATA, frame[i]);
  // TxCRCEn — старший бит; младшие два включают антенну, их нельзя затирать.
  rcWrite(RC_REG_TX_CONTROL, (uint8_t)((rcRead(RC_REG_TX_CONTROL) & 0x03) |
                                       (with_crc ? 0x80 : 0x00)));
  rcWrite(RC_REG_COMMAND, RC_CMD_TRANSCEIVE);
  rcWrite(RC_REG_BIT_FRAMING, (uint8_t)(0x80 | (last_bits & 0x07)));
}

//: 0 — ещё ждём, 1 — ответ в buffer, -1 — таймаут/ошибка линии.
int8_t rcPoll(uint8_t* buffer, uint8_t capacity, uint8_t* length) {
  const uint8_t irq = rcRead(RC_REG_COM_IRQ);
  if ((irq & 0x30) == 0) return 0;        // ни RxIRq, ни TimerIRq
  rcWrite(RC_REG_BIT_FRAMING, 0x00);
  rcWrite(RC_REG_COMMAND, RC_CMD_IDLE);
  if ((irq & 0x20) == 0) return -1;       // TimerIRq: карта молчит
  if (rcRead(RC_REG_ERROR) & 0x13) return -1;   // BufferOvfl | ParityErr | ProtocolErr
  uint8_t level = rcRead(RC_REG_FIFO_LEVEL);
  if (level > capacity) level = capacity;
  for (uint8_t i = 0; i < level; ++i) buffer[i] = rcRead(RC_REG_FIFO_DATA);
  *length = level;
  return 1;
}

// =============================== карточки ====================================
static const uint32_t CARD_COOLDOWN_MS = 2000;

uint16_t cardCrc() {
  uint8_t raw[CARD_BLOCK_BYTES];
  for (uint16_t i = 0; i < CARD_BLOCK_BYTES; ++i) raw[i] = EEPROM.read(CARD_EEPROM_ADDR + i);
  return frameCrc16((const char*)raw, (uint16_t)(CARD_BLOCK_BYTES - 2));
}

void cardWriteBlock() {
  EEPROM.update(CARD_EEPROM_ADDR + 0, CARD_MAGIC0);
  EEPROM.update(CARD_EEPROM_ADDR + 1, CARD_MAGIC1);
  EEPROM.update(CARD_EEPROM_ADDR + 2, CARD_MAGIC2);
  EEPROM.update(CARD_EEPROM_ADDR + 3, CARD_VERSION);
  EEPROM.update(CARD_EEPROM_ADDR + 4, card_count);
  EEPROM.update(CARD_EEPROM_ADDR + 5, 0);           // резерв
  for (uint8_t slot = 0; slot < CARD_SLOTS; ++slot) {
    for (uint8_t byte = 0; byte < 4; ++byte) {
      const uint8_t value = slot < card_count ? cards[slot].uid[byte] : 0xFF;
      EEPROM.update(CARD_EEPROM_ADDR + 6 + slot * 4 + byte, value);
    }
  }
  uint8_t raw[CARD_BLOCK_BYTES];
  for (uint16_t i = 0; i < CARD_BLOCK_BYTES; ++i) raw[i] = EEPROM.read(CARD_EEPROM_ADDR + i);
  const uint16_t crc = frameCrc16((const char*)raw, (uint16_t)(CARD_BLOCK_BYTES - 2));
  EEPROM.update(CARD_EEPROM_ADDR + CARD_BLOCK_BYTES - 2, (uint8_t)crc);
  EEPROM.update(CARD_EEPROM_ADDR + CARD_BLOCK_BYTES - 1, (uint8_t)(crc >> 8));
}

void cardsLoad() {
  card_count = 0;
  if (EEPROM.read(CARD_EEPROM_ADDR + 0) != CARD_MAGIC0 ||
      EEPROM.read(CARD_EEPROM_ADDR + 1) != CARD_MAGIC1 ||
      EEPROM.read(CARD_EEPROM_ADDR + 2) != CARD_MAGIC2 ||
      EEPROM.read(CARD_EEPROM_ADDR + 3) != CARD_VERSION) {
    return;
  }
  const uint8_t stored = EEPROM.read(CARD_EEPROM_ADDR + 4);
  if (stored > CARD_SLOTS) return;
  uint8_t raw[CARD_BLOCK_BYTES];
  for (uint16_t i = 0; i < CARD_BLOCK_BYTES; ++i) raw[i] = EEPROM.read(CARD_EEPROM_ADDR + i);
  const uint16_t expected = frameCrc16((const char*)raw, (uint16_t)(CARD_BLOCK_BYTES - 2));
  const uint16_t actual = (uint16_t)(raw[CARD_BLOCK_BYTES - 2] | (raw[CARD_BLOCK_BYTES - 1] << 8));
  if (expected != actual) {
    error_mask |= ERR_EEPROM;
    return;
  }
  card_count = stored;
  for (uint8_t slot = 0; slot < card_count; ++slot) {
    for (uint8_t byte = 0; byte < 4; ++byte) {
      cards[slot].uid[byte] = raw[6 + slot * 4 + byte];
    }
  }
}

int cardFind(const uint8_t* uid) {
  for (uint8_t slot = 0; slot < card_count; ++slot) {
    if (memcmp(cards[slot].uid, uid, 4) == 0) return slot;
  }
  return -1;
}

bool cardAdd(const uint8_t* uid) {
  if (cardFind(uid) >= 0) return true;
  if (card_count >= CARD_SLOTS) return false;
  memcpy(cards[card_count].uid, uid, 4);
  ++card_count;
  cardWriteBlock();
  return cardFind(uid) >= 0;
}

bool cardRemove(const uint8_t* uid) {
  const int slot = cardFind(uid);
  if (slot < 0) return false;
  for (uint8_t i = (uint8_t)slot; i + 1 < card_count; ++i) cards[i] = cards[i + 1];
  --card_count;
  cardWriteBlock();
  return true;
}

// =============================== калибровка ==================================
void magSaveOffsets() {
  uint8_t raw[12];
  raw[0] = MAG_MAGIC0;
  raw[1] = MAG_MAGIC1;
  raw[2] = MAG_VERSION;
  raw[3] = 0;
  const int16_t ox = (int16_t)lroundf(mag_offset_x);
  const int16_t oy = (int16_t)lroundf(mag_offset_y);
  raw[4] = (uint8_t)ox;
  raw[5] = (uint8_t)(ox >> 8);
  raw[6] = (uint8_t)oy;
  raw[7] = (uint8_t)(oy >> 8);
  const uint16_t crc = frameCrc16((const char*)raw, 8);
  raw[8] = (uint8_t)crc;
  raw[9] = (uint8_t)(crc >> 8);
  for (uint8_t i = 0; i < 8; ++i) EEPROM.update(MAG_EEPROM_ADDR + i, raw[i]);
  EEPROM.update(MAG_EEPROM_ADDR + 8, raw[8]);
  EEPROM.update(MAG_EEPROM_ADDR + 9, raw[9]);
}

void magLoadOffsets() {
  if (EEPROM.read(MAG_EEPROM_ADDR) != MAG_MAGIC0 ||
      EEPROM.read(MAG_EEPROM_ADDR + 1) != MAG_MAGIC1 ||
      EEPROM.read(MAG_EEPROM_ADDR + 2) != MAG_VERSION) {
    return;
  }
  uint8_t raw[10];
  for (uint8_t i = 0; i < 10; ++i) raw[i] = EEPROM.read(MAG_EEPROM_ADDR + i);
  const uint16_t expected = frameCrc16((const char*)raw, 8);
  const uint16_t actual = (uint16_t)(raw[8] | (raw[9] << 8));
  if (expected != actual) {
    error_mask |= ERR_EEPROM;
    return;
  }
  mag_offset_x = (float)(int16_t)(raw[4] | (raw[5] << 8));
  mag_offset_y = (float)(int16_t)(raw[6] | (raw[7] << 8));
}

// =============================== свет и реле =================================
void relayWrite(uint8_t index, bool on) {
  const uint8_t pin = RELAY_PINS[index];
  const uint8_t active = on ? (RELAY_ACTIVE_LOW ? LOW : HIGH) : (RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(pin, active);
}

//: Сколько сейчас горит с учётом мигания поворотников.
uint8_t effectiveLights() {
  uint8_t mask = light_state;
  if ((mask & (LIGHT_LEFT | LIGHT_RIGHT)) && !blink_phase) {
    mask &= (uint8_t)~(LIGHT_LEFT | LIGHT_RIGHT);
  }
  return mask;
}

void applyRelays(uint8_t mask) {
  relayWrite(0, (mask & LIGHT_HEAD) != 0);
  relayWrite(1, (mask & LIGHT_STOP) != 0);
  relayWrite(2, (mask & LIGHT_LEFT) != 0);
  relayWrite(3, (mask & LIGHT_RIGHT) != 0);
}

//: Лента повторяет свет: стоп — красный, поворотники — янтарные полосы,
//: габариты — белый на заданной яркости. Иначе лента погашена.
void applyStrip(uint8_t mask) {
  if (!strip_auto) return;
  strip.clear();
  const uint8_t scale = (uint8_t)((uint16_t)STRIP_BRIGHTNESS * (uint16_t)dim_level / 255);
  if (mask & LIGHT_STOP) {
    const uint32_t red = strip.Color(scale, 0, 0);
    for (uint16_t i = 0; i < STRIP_LEDS; ++i) strip.setPixelColor(i, red);
  } else if (mask & (LIGHT_LEFT | LIGHT_RIGHT)) {
    const uint32_t amber = strip.Color(scale, scale / 2, 0);
    const uint16_t half = STRIP_LEDS / 2;
    for (uint16_t i = 0; i < STRIP_LEDS; ++i) {
      const bool left_side = i < half;
      if (((mask & LIGHT_LEFT) && left_side) || ((mask & LIGHT_RIGHT) && !left_side)) {
        strip.setPixelColor(i, amber);
      }
    }
  } else if (mask & LIGHT_DIM) {
    const uint32_t white = strip.Color(scale / 2, scale / 2, scale / 2);
    for (uint16_t i = 0; i < STRIP_LEDS; ++i) strip.setPixelColor(i, white);
  } else if (mask & LIGHT_HEAD) {
    const uint32_t white = strip.Color(scale, scale, scale);
    for (uint16_t i = 0; i < STRIP_LEDS; ++i) strip.setPixelColor(i, white);
  }
  strip.show();
}

void lightsRefresh() {
  const uint8_t mask = effectiveLights();
  applyRelays(mask);
  applyStrip(mask);
}

void setLight(uint8_t bit, bool on) {
  if (on) light_state |= bit;
  else light_state &= (uint8_t)~bit;
  lightsRefresh();
}

// ================================== замок ====================================
void lockApply(uint8_t angle) {
  lock_angle = angle;
  if (!lockServo.attached()) return;
  lockServo.write(angle);
}

void lockSet(bool open, uint16_t milliseconds) {
  lock_open = open;
  if (open) {
    lock_deadline = millis() + (milliseconds == 0 ? LOCK_DEFAULT_MS : milliseconds);
    lockApply(LOCK_OPEN_ANGLE);
  } else {
    lock_deadline = 0;
    lockApply(LOCK_CLOSED_ANGLE);
  }
}

void lockTick(uint32_t now) {
  if (lock_open && lock_deadline && (int32_t)(now - lock_deadline) >= 0) {
    lockSet(false, 0);
    char line[48];
    const int n = snprintf_P(line, sizeof(line), PSTR("@EVT kind=lock open=0 t=%lu"),
                             (unsigned long)now);
    appendCheckedFrame(line, sizeof(line), n);
  }
}

// ================================= концевик ==================================
void doorTick(uint32_t now) {
  const bool raw = digitalRead(DOOR_SWITCH_PIN) == DOOR_CLOSED_LEVEL;
  if (raw != door_raw) {
    door_raw = raw;
    door_changed_ms = now;
    return;
  }
  if ((now - door_changed_ms) < 40 || raw == door_closed) return;
  door_closed = raw;
  char line[48];
  const int n = snprintf_P(line, sizeof(line), PSTR("@EVT kind=door open=%d t=%lu"),
                           door_closed ? 0 : 1, (unsigned long)now);
  appendCheckedFrame(line, sizeof(line), n);
}

// ================================ телеметрия =================================
void emitTelemetry() {
  char line[192];
  const int n = snprintf_P(line, sizeof(line),
      PSTR("@S fw=%u hdg=%d pitch=%d roll=%d gz=%d press=%ld temp=%d alt=%ld "
           "door=%d lock=%d light=%u dim=%u rfid=%u cards=%u link=%d err=%u t=%lu"),
      (unsigned)SENSOR_PROTOCOL, (int)heading_cd, (int)pitch_cd, (int)roll_cd,
      (int)gyro_z_dps10, (long)baro_pressure_pa, (int)baro_temp_c10, (long)baro_altitude_cm,
      door_closed ? 0 : 1, lock_open ? 1 : 0,
      (unsigned)effectiveLights(), (unsigned)dim_level, (unsigned)rfid_state,
      (unsigned)card_count, link_ok ? 1 : 0, (unsigned)error_mask,
      (unsigned long)millis());
  appendCheckedFrame(line, sizeof(line), n);
}

void emitCard(uint8_t verdict, const uint8_t* uid) {
  char line[96];
  const int n = snprintf_P(line, sizeof(line),
      PSTR("@CARD uid=%02X%02X%02X%02X ok=%u cards=%u t=%lu"),
      uid[0], uid[1], uid[2], uid[3], (unsigned)verdict, (unsigned)card_count,
      (unsigned long)millis());
  appendCheckedFrame(line, sizeof(line), n);
}

void emitAck(const char* cmd, bool ok) {
  char line[96];
  const int n = snprintf_P(line, sizeof(line), PSTR("@ACK id=%lu cmd=%s ok=%d t=%lu"),
                           (unsigned long)command_id, cmd, ok ? 1 : 0,
                           (unsigned long)millis());
  appendCheckedFrame(line, sizeof(line), n);
}

// ================================ опрос датчиков =============================
void sensorsTick(uint32_t now) {
  if ((now - last_sensor_ms) < 50) return;
  last_sensor_ms = now;

  float ax = 0, ay = 0, az = 0;
  if (mpuRead(&ax, &ay, &az, &gyro_z_dps10)) {
    error_mask &= (uint8_t)~ERR_MPU;
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    roll_cd = (int16_t)lroundf(roll * 180.0f / PI_F * 100.0f);
    pitch_cd = (int16_t)lroundf(pitch * 180.0f / PI_F * 100.0f);
    float mx = 0, my = 0, mz = 0;
    if (magRead(&mx, &my, &mz)) {
      error_mask &= (uint8_t)~ERR_MAG;
      if (mag_calibrating) {
        // Первый замер задаёт границы: ноль не обязательно внутри диапазона
        // (жёсткое смещение уводит всё поле в сторону).
        if (!mag_cal_armed) {
          mag_min_x = mag_max_x = mx;
          mag_min_y = mag_max_y = my;
          mag_cal_armed = true;
        } else {
          if (mx < mag_min_x) mag_min_x = mx;
          if (mx > mag_max_x) mag_max_x = mx;
          if (my < mag_min_y) mag_min_y = my;
          if (my > mag_max_y) mag_max_y = my;
        }
      }
      const float cx = mx - mag_offset_x;
      const float cy = my - mag_offset_y;
      // Наклонная компенсация: без неё компас врёт при крене и тангаже.
      const float sr = sinf(roll), cr = cosf(roll);
      const float sp = sinf(pitch), cp = cosf(pitch);
      const float xh = cx * cp + mz * sp;
      const float yh = cx * sr * sp + cy * cr - mz * sr * cp;
      float heading = atan2f(-yh, xh) * 180.0f / PI_F;
      if (heading < 0) heading += 360.0f;
      if (heading >= 360.0f) heading -= 360.0f;
      // 360.00° = 36000 не влезает в int16: сначала считаем в int32, потом сужаем.
      int32_t centi = lroundf(heading * 100.0f);
      if (centi >= 36000) centi -= 36000;
      if (centi < 0) centi = 0;
      heading_cd = (int16_t)centi;
    } else {
      error_mask |= ERR_MAG;
    }
  } else {
    error_mask |= ERR_MPU;
  }

  // Барометр медленнее IMU: замер BMP180 стоит ~10 мс (два преобразования).
  if (baro_ready && (now - last_baro_ms) >= BARO_PERIOD_MS) {
    last_baro_ms = now;
    int32_t pressure = 0;
    int16_t temperature = 0;
    if (baroRead(&pressure, &temperature)) {
      error_mask &= (uint8_t)~ERR_BARO;
      baro_pressure_pa = pressure;
      baro_temp_c10 = temperature;
      // Международная барометрическая формула, опорное давление 101325 Па.
      const float ratio = (float)pressure / 101325.0f;
      if (ratio > 0.1f) {
        baro_altitude_cm = (int32_t)lroundf(44330.0f * (1.0f - powf(ratio, 0.1903f)) * 100.0f);
      }
    } else {
      error_mask |= ERR_BARO;
    }
  }
}

// ================================== RFID =====================================
// Опрос карты разбит на шаги: WUPA → антиколлизия → SELECT → HALT. Каждый шаг
// запускается в одном проходе loop(), ответ забирается в следующих, поэтому
// ожидание карты не задерживает мигание поворотников и телеметрию.
// WUPA, а не REQA: карта после прошлого цикла стоит в HALT и на REQA не отвечает,
// пока её не вынут из поля.
static const uint32_t RFID_POLL_MS = 100;   // как часто пробуем поле
static const uint32_t RFID_STEP_MS = 40;    // дольше этого ответа не ждём

static uint8_t rfid_phase = 0;              // 0 ждём слот, 1 WUPA, 2 anticoll, 3 SELECT, 4 HALT
static uint8_t rfid_uid[4] = {0, 0, 0, 0};
static uint8_t rfid_answer[8] = {0};
static uint8_t rfid_answer_length = 0;
static uint32_t rfid_started_ms = 0;
static uint32_t rfid_slot_ms = 0;

void rfidRestart(uint32_t now) {
  rfid_phase = 0;
  rfid_slot_ms = now + RFID_POLL_MS;
}

//: Карта прочитана целиком: пускаем или отказываем, и всё равно гасим её в HALT.
void rfidCardRead(const uint8_t* uid, uint32_t now) {
  const bool same = memcmp(uid, last_card_uid, 4) == 0;
  if (same && (now - last_card_seen_ms) < CARD_COOLDOWN_MS) return;
  memcpy(last_card_uid, uid, 4);
  last_card_seen_ms = now;
  error_mask &= (uint8_t)~ERR_RFID;

  if (card_learn) {
    card_learn = false;
    if (cardAdd(uid)) {
      rfid_state = RFID_ENROLLED;
      emitCard(1, uid);
      note(PSTR("карта %02X%02X%02X%02X записана"), uid[0], uid[1], uid[2], uid[3]);
    } else {
      rfid_state = RFID_DENIED;
      emitCard(0, uid);
      note(PSTR("карта не записана: список заполнен (%u)"), (unsigned)card_count);
    }
    return;
  }

  const bool known = cardFind(uid) >= 0;
  rfid_state = known ? RFID_GRANTED : RFID_DENIED;
  emitCard(known ? 1 : 0, uid);
  if (known && door_closed) {
    lockSet(true, LOCK_DEFAULT_MS);
    note(PSTR("карта %02X%02X%02X%02X: замок открыт на %u мс"),
         uid[0], uid[1], uid[2], uid[3], (unsigned)LOCK_DEFAULT_MS);
  } else if (!known) {
    note(PSTR("карта %02X%02X%02X%02X не записана: доступа нет"),
         uid[0], uid[1], uid[2], uid[3]);
  } else {
    note(PSTR("дверь открыта: замок не открываю"));
  }
}

void rfidTick(uint32_t now) {
  if (rfid_phase == 0) {
    if ((int32_t)(now - rfid_slot_ms) < 0) return;
    static const uint8_t wupa = PICC_WUPA;
    rcStart(&wupa, 1, 7, false);
    rfid_phase = 1;
    rfid_started_ms = now;
    return;
  }

  const int8_t result = rcPoll(rfid_answer, sizeof(rfid_answer), &rfid_answer_length);
  if (result == 0) {
    if ((now - rfid_started_ms) > RFID_STEP_MS) rfidRestart(now);   // RC522 не ответил
    return;
  }
  if (result < 0) {
    rfidRestart(now);          // карты нет или ответ битый — это не отказ датчика
    return;
  }

  switch (rfid_phase) {
    case 1: {                  // WUPA: ATQA — два байта
      if (rfid_answer_length != 2) { rfidRestart(now); return; }
      const uint8_t anticoll[2] = {PICC_SEL_CL1, 0x20};
      rcStart(anticoll, 2, 0, false);
      rfid_phase = 2;
      rfid_started_ms = now;
      return;
    }
    case 2: {                  // антиколлизия: UID + BCC
      if (rfid_answer_length != 5) { rfidRestart(now); return; }
      if ((uint8_t)(rfid_answer[0] ^ rfid_answer[1] ^ rfid_answer[2] ^ rfid_answer[3]) !=
          rfid_answer[4]) {
        rfidRestart(now);
        return;
      }
      memcpy(rfid_uid, rfid_answer, 4);
      uint8_t select[7];
      select[0] = PICC_SEL_CL1;
      select[1] = 0x70;        // NVB: 7 байт — UID + BCC + CRC_A
      memcpy(select + 2, rfid_uid, 4);
      select[6] = rfid_answer[4];
      rcStart(select, 7, 0, true);
      rfid_phase = 3;
      rfid_started_ms = now;
      return;
    }
    case 3: {                  // SELECT: SAK + CRC_A
      if (rfid_answer_length != 3 || (rfid_answer[0] & PICC_SAK_CASCADE) != 0) {
        rfidRestart(now);      // карты с 7-байтным UID не поддерживаются
        return;
      }
      const uint8_t halt[2] = {PICC_HALT, 0x00};
      rcStart(halt, 2, 0, true);
      rfid_phase = 4;
      rfid_started_ms = now;
      return;
    }
    default: {                 // HALT отправлен
      rfidCardRead(rfid_uid, now);
      rfidRestart(now);
      return;
    }
  }
}

// ================================ команды ====================================
bool parseOnOff(const char* value, bool* on) {
  if (strcmp_P(value, PSTR("on")) == 0 || strcmp(value, "1") == 0) {
    *on = true;
    return true;
  }
  if (strcmp_P(value, PSTR("off")) == 0 || strcmp(value, "0") == 0) {
    *on = false;
    return true;
  }
  return false;
}

bool parseUid(const char* text, uint8_t* uid) {
  if (strlen(text) != 8) return false;
  for (uint8_t i = 0; i < 4; ++i) {
    char pair[3] = {text[i * 2], text[i * 2 + 1], 0};
    char* end = nullptr;
    const long value = strtol(pair, &end, 16);
    if (!end || *end != 0 || value < 0 || value > 0xFF) return false;
    uid[i] = (uint8_t)value;
  }
  return true;
}

void printHelp() {
  note(PSTR("команды: ping, stat, ?, light head|stop|left|right|dim on|off, light all off|on"));
  note(PSTR("dim 0…255, strip auto|off, lock open [1…10]|close, card list|add|learn|del|clear"));
  note(PSTR("mag cal start|stop|reset. Пины: I2C 20/21, RC522 SS53 RST9, лента D6, реле D2-D5"));
}

void printCards() {
  note(PSTR("карт в памяти: %u из %u"), (unsigned)card_count, (unsigned)CARD_SLOTS);
  for (uint8_t slot = 0; slot < card_count; ++slot) {
    note(PSTR("  %u: %02X%02X%02X%02X"), (unsigned)(slot + 1), cards[slot].uid[0],
         cards[slot].uid[1], cards[slot].uid[2], cards[slot].uid[3]);
  }
}

//: Разбор строки без динамической памяти: strtok_r режет на месте.
void handleCommand(char* line) {
  char* save = nullptr;
  char* word = strtok_r(line, " \t", &save);
  if (!word) return;
  ++command_id;
  last_command_ms = millis();
  link_ok = true;

  if (strcmp_P(word, PSTR("ping")) == 0 || strcmp_P(word, PSTR("stat")) == 0) {
    emitTelemetry();
    emitAck(word, true);
    return;
  }
  if (strcmp_P(word, PSTR("?")) == 0 || strcmp_P(word, PSTR("help")) == 0) {
    printHelp();
    emitAck("help", true);
    return;
  }
  if (strcmp_P(word, PSTR("light")) == 0) {
    char* name = strtok_r(nullptr, " \t", &save);
    char* value = strtok_r(nullptr, " \t", &save);
    if (!name) { emitAck("light", false); return; }
    if (strcmp_P(name, PSTR("all")) == 0) {
      bool on = false;
      if (!value || !parseOnOff(value, &on)) { emitAck("light", false); return; }
      light_state = on ? (uint8_t)(LIGHT_HEAD | LIGHT_STOP | LIGHT_LEFT | LIGHT_RIGHT | LIGHT_DIM) : 0;
      lightsRefresh();
      emitTelemetry();
      emitAck("light", true);
      return;
    }
    uint8_t bit = 0;
    if (strcmp_P(name, PSTR("head")) == 0) bit = LIGHT_HEAD;
    else if (strcmp_P(name, PSTR("stop")) == 0) bit = LIGHT_STOP;
    else if (strcmp_P(name, PSTR("left")) == 0) bit = LIGHT_LEFT;
    else if (strcmp_P(name, PSTR("right")) == 0) bit = LIGHT_RIGHT;
    else if (strcmp_P(name, PSTR("dim")) == 0) bit = LIGHT_DIM;
    bool on = false;
    if (!bit || !value || !parseOnOff(value, &on)) { emitAck("light", false); return; }
    setLight(bit, on);
    emitTelemetry();
    emitAck("light", true);
    return;
  }
  if (strcmp_P(word, PSTR("dim")) == 0) {
    char* value = strtok_r(nullptr, " \t", &save);
    if (!value) { emitAck("dim", false); return; }
    const long level = strtol(value, nullptr, 10);
    if (level < 0 || level > 255) { emitAck("dim", false); return; }
    dim_level = (uint8_t)level;
    lightsRefresh();
    emitTelemetry();
    emitAck("dim", true);
    return;
  }
  if (strcmp_P(word, PSTR("strip")) == 0) {
    char* value = strtok_r(nullptr, " \t", &save);
    if (!value) { emitAck("strip", false); return; }
    if (strcmp_P(value, PSTR("auto")) == 0) strip_auto = true;
    else if (strcmp_P(value, PSTR("off")) == 0) {
      strip_auto = false;
      strip.clear();
      strip.show();
    } else { emitAck("strip", false); return; }
    lightsRefresh();
    emitAck("strip", true);
    return;
  }
  if (strcmp_P(word, PSTR("lock")) == 0) {
    char* action = strtok_r(nullptr, " \t", &save);
    if (!action) { emitAck("lock", false); return; }
    if (strcmp_P(action, PSTR("open")) == 0) {
      char* seconds = strtok_r(nullptr, " \t", &save);
      long value = seconds ? strtol(seconds, nullptr, 10) : 0;
      // Отрицательное время — ошибка команды, а не повод открыть замок.
      if (value < 0) { emitAck("lock", false); return; }
      if (value == 0) value = LOCK_DEFAULT_MS / 1000;
      if (value > (long)(LOCK_MAX_MS / 1000)) value = LOCK_MAX_MS / 1000;
      lockSet(true, (uint16_t)(value * 1000));
      note(PSTR("замок открыт на %ld с"), value);
    } else if (strcmp_P(action, PSTR("close")) == 0) {
      lockSet(false, 0);
    } else { emitAck("lock", false); return; }
    emitTelemetry();
    emitAck("lock", true);
    return;
  }
  if (strcmp_P(word, PSTR("card")) == 0) {
    char* action = strtok_r(nullptr, " \t", &save);
    if (!action) { emitAck("card", false); return; }
    if (strcmp_P(action, PSTR("list")) == 0) {
      printCards();
      emitAck("card", true);
      return;
    }
    if (strcmp_P(action, PSTR("learn")) == 0) {
      card_learn = true;
      note(PSTR("поднесите карту: она будет записана"));
      emitAck("card", true);
      return;
    }
    if (strcmp_P(action, PSTR("clear")) == 0) {
      card_count = 0;
      cardWriteBlock();
      rfid_state = RFID_DELETED;
      note(PSTR("список карт очищен"));
      emitTelemetry();
      emitAck("card", true);
      return;
    }
    char* value = strtok_r(nullptr, " \t", &save);
    uint8_t uid[4];
    if (!value || !parseUid(value, uid)) { emitAck("card", false); return; }
    if (strcmp_P(action, PSTR("add")) == 0) {
      const bool ok = cardAdd(uid);
      if (ok) rfid_state = RFID_ENROLLED;
      emitTelemetry();
      emitAck("card", ok);
      return;
    }
    if (strcmp_P(action, PSTR("del")) == 0) {
      const bool ok = cardRemove(uid);
      if (ok) rfid_state = RFID_DELETED;
      emitTelemetry();
      emitAck("card", ok);
      return;
    }
    emitAck("card", false);
    return;
  }
  if (strcmp_P(word, PSTR("mag")) == 0) {
    char* action = strtok_r(nullptr, " \t", &save);
    char* sub = strtok_r(nullptr, " \t", &save);
    if (action && sub && strcmp_P(action, PSTR("cal")) == 0) {
      if (strcmp_P(sub, PSTR("start")) == 0) {
        mag_calibrating = true;
        mag_cal_armed = false;
        note(PSTR("калибровка компаса: медленно поверните робота на 360°"));
        emitAck("mag", true);
        return;
      }
      if (strcmp_P(sub, PSTR("stop")) == 0) {
        if (!mag_calibrating) { emitAck("mag", false); return; }
        mag_calibrating = false;
        const float span_x = mag_max_x - mag_min_x;
        const float span_y = mag_max_y - mag_min_y;
        if (span_x < 50.0f && span_y < 50.0f) {
          note(PSTR("калибровка не удалась: поле почти не менялось"));
          emitAck("mag", false);
          return;
        }
        mag_offset_x = (mag_max_x + mag_min_x) / 2.0f;
        mag_offset_y = (mag_max_y + mag_min_y) / 2.0f;
        magSaveOffsets();
        note(PSTR("смещение компаса сохранено: x=%d y=%d"), (int)mag_offset_x, (int)mag_offset_y);
        emitAck("mag", true);
        return;
      }
      if (strcmp_P(sub, PSTR("reset")) == 0) {
        mag_offset_x = mag_offset_y = 0.0f;
        magSaveOffsets();
        note(PSTR("смещение компаса сброшено"));
        emitAck("mag", true);
        return;
      }
    }
    emitAck("mag", false);
    return;
  }
  note(PSTR("неизвестная команда: %s"), word);
  emitAck(word, false);
}

void readSerial() {
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\r') continue;
    if (ch == '\n') {
      line_buf[line_length] = 0;
      if (line_length) handleCommand(line_buf);
      line_length = 0;
      continue;
    }
    if (line_length < sizeof(line_buf) - 1) line_buf[line_length++] = (char)ch;
    else line_length = 0;             // переполнение: строку выбрасываем целиком
  }
}

// ================================== setup ====================================
void setup() {
  wdt_disable();
  Serial.begin(115200);
  // Реле и лента — в безопасное состояние до любой инициализации датчиков.
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(RELAY_PINS[i], OUTPUT);
    relayWrite(i, false);
  }
  pinMode(DOOR_SWITCH_PIN, INPUT_PULLUP);
  pinMode(RC522_RST_PIN, OUTPUT);
  digitalWrite(RC522_RST_PIN, HIGH);
  pinMode(RC522_SS_PIN, OUTPUT);
  digitalWrite(RC522_SS_PIN, HIGH);

  lockServo.attach(LOCK_SERVO_PIN);
  lockSet(false, 0);

  strip.begin();
  strip.setBrightness(STRIP_BRIGHTNESS);
  strip.clear();
  strip.show();

  cardsLoad();
  magLoadOffsets();

  Wire.begin();
  Wire.setClock(400000L);
  if (!mpuInit()) error_mask |= ERR_MPU | ERR_I2C;
  if (!magInit()) error_mask |= ERR_MAG | ERR_I2C;
  baro_ready = baroInit();
  if (!baro_ready) error_mask |= ERR_BARO | ERR_I2C;

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);   // 4 МГц: RC522 держит до 10 МГц
  if (!rcReset()) error_mask |= ERR_RFID;

  note(PSTR("модуль датчиков fw%u: карт %u, ошибки 0x%02X"), (unsigned)SENSOR_PROTOCOL,
       (unsigned)card_count, (unsigned)error_mask);
  note(PSTR("I2C 20/21 · RC522 SS53 RST9 · лента D6 · реле D2-D5 · замок D7 · концевик D8"));

  last_sensor_ms = 0;
  last_telemetry_ms = millis();
  wdt_enable(WDTO_2S);
}

// =================================== loop ====================================
void loop() {
  wdt_reset();
  const uint32_t now = millis();

  readSerial();
  sensorsTick(now);
  doorTick(now);
  lockTick(now);

  // Мигание поворотников 1.6 Гц: реле и лента переключаются вместе.
  if ((light_state & (LIGHT_LEFT | LIGHT_RIGHT)) && (now - last_blink_ms) >= 300) {
    last_blink_ms = now;
    blink_phase = !blink_phase;
    lightsRefresh();
  }

  // Обмен с картой неблокирующий: каждый проход loop() делает один маленький шаг.
  rfidTick(now);

  if (!link_ok || (now - last_command_ms) > LINK_TIMEOUT_MS) {
    if (link_ok) {
      link_ok = false;
      note(PSTR("нет команд дольше %u мс: замок закрыт"), (unsigned)LINK_TIMEOUT_MS);
      lockSet(false, 0);
    }
  }

  if ((now - last_telemetry_ms) >= 200) {
    last_telemetry_ms = now;
    emitTelemetry();
  }

  txPump();
}
