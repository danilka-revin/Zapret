// Хост-симуляция модуля датчиков: одна плата Mega, но всё «железо» на ней —
// модели: GY-87 (MPU6050 + HMC5883L + BMP180 на I²C), RC522 на SPI с картой в
// поле, адресная лента, 4 реле и серво замка.
//
// Сборка и запуск (это делает tests/test_sensor_module.py):
//   g++ -std=c++17 -I tests/firmware_sim/stubs tests/firmware_sim/sensor_sim.cpp -o sensor_sim
//   ./sensor_sim            — все сценарии (каждый в своём процессе, как после сброса)
//   ./sensor_sim lights     — один сценарий
//   ./sensor_sim dump       — кадры @S/@CARD/@ACK/@EVT для разбора сервером
//
// Модели повторяют даташиты ровно настолько, насколько это ловит ошибки прошивки:
// регистры и порядок байт MPU6050/HMC5883L/BMP180, состояния карты ISO14443A
// (IDLE → READY → ACTIVE → HALT) и биты прерываний RC522.
#include "arduino_sim.h"

#include <cstring>
#include <functional>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

sim::Board board;

namespace sensors {
#define SIM_BOARD board
#include "board_prelude.h"
#include "../../firmware/sensor_module/sensor_module.ino"
#undef SIM_BOARD
}  // namespace sensors
#undef TX_BUFFER_SIZE

static int failures = 0;

#define CHECK(cond, ...)                                                          \
  do {                                                                            \
    if (!(cond)) {                                                                \
      ++failures;                                                                 \
      std::fprintf(stderr, "  FAIL %s:%d: %s — ", __FILE__, __LINE__, #cond);     \
      std::fprintf(stderr, __VA_ARGS__);                                          \
      std::fprintf(stderr, "\n");                                                 \
    }                                                                             \
  } while (0)

// ============================= GY-87: I²C ===================================
// BMP180: калибровка и пример замера из даташита Bosch (UT = 27898, UP = 23843
// дают 15.0 °C и 69964 Па). Если прошивка считает иначе — ошибка в формуле.
struct Gy87Model {
  bool mpu_present = true;
  bool mag_present = true;
  bool baro_present = true;
  // MPU6050: ускорение в g и угловая скорость вокруг Z в °/с
  float ax = 0.0f, ay = 0.0f, az = 1.0f, gz_dps = 0.0f;
  // HMC5883L: «сырые» отсчёты магнитного поля
  int16_t mx = 300, my = 0, mz = 0;
  bool mag_flat = false;   // поле не меняется — калибровка должна это заметить

  uint8_t ptr_mpu = 0, ptr_mag = 0, ptr_baro = 0;
  uint8_t baro_ctrl = 0;

  static const int AC1 = 408, AC2 = -72, AC3 = -14383, B1 = 6190, B2 = 4,
                   MB = -32768, MC = -8711, MD = 2868;
  static const int AC4 = 32741, AC5 = 32757, AC6 = 23153;
  static const int UT = 27898, UP = 23843;

  bool handle(uint8_t address, const uint8_t* tx, uint8_t tx_length,
              uint8_t* rx, uint8_t rx_length) {
    uint8_t* pointer = nullptr;
    if (address == 0x68) pointer = &ptr_mpu;
    else if (address == 0x1E) pointer = &ptr_mag;
    else if (address == 0x77) pointer = &ptr_baro;
    else return false;
    if (tx_length == 0 && rx_length > 0) return fill(address, *pointer, rx, rx_length);
    if (tx_length == 0) return true;
    *pointer = tx[0];
    for (uint8_t i = 1; i < tx_length; ++i) writeReg(address, (uint8_t)(tx[0] + i - 1), tx[i]);
    if (rx_length > 0) return fill(address, *pointer, rx, rx_length);
    return true;
  }

  void writeReg(uint8_t address, uint8_t reg, uint8_t value) {
    if (address == 0x77 && reg == 0xF4) baro_ctrl = value;
  }

  bool fill(uint8_t address, uint8_t start, uint8_t* rx, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i) rx[i] = byteAt(address, (uint8_t)(start + i));
    return true;
  }

  uint8_t byteAt(uint8_t address, uint8_t reg) {
    if (address == 0x68) return mpuByte(reg);
    if (address == 0x1E) return magByte(reg);
    return baroByte(reg);
  }

  static uint8_t hi(int16_t value) { return (uint8_t)((uint16_t)value >> 8); }
  static uint8_t lo(int16_t value) { return (uint8_t)((uint16_t)value & 0xFF); }

  uint8_t mpuByte(uint8_t reg) {
    if (reg == 0x75) return mpu_present ? 0x68 : 0x00;
    if (!mpu_present) return 0x00;
    const int16_t axr = (int16_t)(ax * 16384.0f);
    const int16_t ayr = (int16_t)(ay * 16384.0f);
    const int16_t azr = (int16_t)(az * 16384.0f);
    const int16_t gzr = (int16_t)(gz_dps * 131.0f);
    switch (reg) {
      case 0x3B: return hi(axr);
      case 0x3C: return lo(axr);
      case 0x3D: return hi(ayr);
      case 0x3E: return lo(ayr);
      case 0x3F: return hi(azr);
      case 0x40: return lo(azr);
      case 0x41: return 0;            // температура
      case 0x42: return 0;
      case 0x43: return 0;            // gyro X
      case 0x44: return 0;
      case 0x45: return 0;            // gyro Y
      case 0x46: return 0;
      case 0x47: return hi(gzr);
      case 0x48: return lo(gzr);
      default: return 0;
    }
  }

  uint8_t magByte(uint8_t reg) {
    if (reg == 0x0A) return mag_present ? 'H' : 0x00;
    if (reg == 0x0B) return mag_present ? '4' : 0x00;
    if (reg == 0x0C) return mag_present ? '3' : 0x00;
    if (!mag_present) return 0x00;
    switch (reg) {
      case 0x03: return lo(mx);       // порядок HMC5883L: X, Z, Y
      case 0x04: return hi(mx);
      case 0x05: return lo(mz);
      case 0x06: return hi(mz);
      case 0x07: return lo(my);
      case 0x08: return hi(my);
      default: return 0x00;
    }
  }

  uint8_t baroByte(uint8_t reg) {
    if (reg == 0xD0) return baro_present ? 0x55 : 0x00;
    if (!baro_present) return 0x00;
    static const int16_t cal[11] = {AC1, AC2, AC3, (int16_t)AC4, (int16_t)AC5, (int16_t)AC6,
                                    B1, B2, MB, MC, MD};
    if (reg >= 0xAA && reg < 0xAA + 22) {
      const int index = (reg - 0xAA) / 2;
      const bool high = ((reg - 0xAA) % 2) == 0;
      return high ? hi(cal[index]) : lo(cal[index]);
    }
    if (reg == 0xF4) return baro_ctrl;
    if (reg >= 0xF6 && reg <= 0xF8) {
      if ((baro_ctrl & 0x3F) == 0x2E) {          // замер температуры: 2 байта
        return reg == 0xF6 ? hi((int16_t)UT) : lo((int16_t)UT);
      }
      // 3 байта с выравниванием влево: прошивка берёт старшие 16 бит (oss = 0).
      const uint8_t bytes[3] = {(uint8_t)((UP >> 8) & 0xFF), (uint8_t)(UP & 0xFF), 0};
      return bytes[reg - 0xF6];
    }
    return 0x00;
  }

  //: Поле повернулось на angle градусов (0 — север по оси X магнитометра).
  void setHeading(float degrees, float amplitude = 300.0f) {
    const float radians = degrees * 3.14159265f / 180.0f;
    mx = (int16_t)(amplitude * cosf(radians));
    my = (int16_t)(-amplitude * sinf(radians));
  }
};

// ============================ RC522: SPI ====================================
// Состояния карты по ISO14443A: WUPA отвечает только в IDLE и HALT, антиколлизия
// (0x93 0x20) — только в IDLE, SELECT (0x93 0x70) — только в READY, HALT — в
// ACTIVE. Прошивка обязана пройти всю цепочку, иначе карта «залипнет».
struct Rc522Model {
  bool present = true;         // MISO живой (0x00/0xFF = микросхемы нет)
  bool card_present = false;
  bool halt_ignored = false;   // карта не принимает HALT (проверка терпимости)
  uint8_t uid[4] = {0xA1, 0xB2, 0xC3, 0xD4};
  uint8_t sak = 0x08;          // SAK без бита каскада: UID 4 байта
  enum State { IDLE, READY, ACTIVE, HALT } state = IDLE;

  uint8_t command = 0;
  uint8_t com_irq = 0;
  uint8_t error = 0;
  uint8_t tx_control = 0x00;
  std::vector<uint8_t> fifo;
  uint8_t addressed = 0;
  uint8_t phase = 0;
  bool reading = false;

  long transceives = 0;
  long halts = 0;

  void insert(const uint8_t* new_uid, bool present_now = true) {
    memcpy(uid, new_uid, 4);
    card_present = present_now;
    state = IDLE;
  }
  void remove() {
    card_present = false;
    state = IDLE;
  }

  //: Любой фронт SS начинает новую транзакцию: первый байт — адрес регистра.
  void onPinChange(uint8_t pin, uint8_t level) {
    if (pin == 53) phase = 0;
    (void)level;
  }

  uint8_t transfer(uint8_t out) {
    // Прошивка выбирает микросхему сама: SS = D53 (как в разводке).
    if (board.level[53] != LOW) return 0x3F;   // MISO в высокоимпедансе
    if (!present) return 0x00;                 // микросхемы нет: линия прижата к нулю
    if (phase == 0) {
      reading = (out & 0x80) != 0;
      addressed = (uint8_t)((out >> 1) & 0x3F);
      phase = 1;
      return 0x00;
    }
    if (reading) return readReg(addressed);
    writeReg(addressed, out);
    return 0x00;
  }

  uint8_t readReg(uint8_t reg) {
    switch (reg) {
      case 0x37: return 0x92;                      // VersionReg: RC522
      case 0x01: return command;
      case 0x04: return com_irq;
      case 0x06: return error;
      case 0x0A: return (uint8_t)fifo.size();
      case 0x14: return tx_control;
      case 0x09: {
        if (fifo.empty()) return 0;
        const uint8_t value = fifo.front();
        fifo.erase(fifo.begin());
        return value;
      }
      default: return 0x00;
    }
  }

  void writeReg(uint8_t reg, uint8_t value) {
    switch (reg) {
      case 0x01:
        command = value;
        if (value == 0x0F) reset();
        else if (value == 0x0C) transceive();
        break;
      case 0x04: com_irq &= (uint8_t)~value; break;   // единица сбрасывает флаг
      case 0x09: if (fifo.size() < 64) fifo.push_back(value); break;
      case 0x0A: if (value & 0x80) fifo.clear(); break;
      case 0x06: error = value; break;
      case 0x14: tx_control = value; break;
      default: break;
    }
  }

  void reset() {
    command = 0;
    com_irq = 0;
    error = 0;
    tx_control = 0;
    fifo.clear();
    phase = 0;
  }

  void answer(const std::vector<uint8_t>& payload) {
    fifo = payload;
    com_irq |= 0x20;      // RxIRq
    command = 0x00;
  }

  void timeout() {
    fifo.clear();
    com_irq |= 0x10;      // TimerIRq
    command = 0x00;
  }

  void transceive() {
    ++transceives;
    const std::vector<uint8_t> request = fifo;
    fifo.clear();
    if (request.empty()) return timeout();
    const uint8_t opcode = request[0];
    if (opcode == 0x52 || opcode == 0x26) {          // WUPA / REQA
      if (!card_present) return timeout();
      const bool wakes = state == IDLE || state == HALT;
      if (opcode == 0x52 && !wakes) return timeout();
      if (opcode == 0x26 && state != IDLE) return timeout();
      state = IDLE;
      return answer({0x04, 0x00});
    }
    if (opcode == 0x93 && request.size() >= 2) {
      if (!card_present) return timeout();
      const uint8_t nvb = request[1];
      if (nvb == 0x20) {                             // антиколлизия
        if (state != IDLE) return timeout();
        state = READY;
        const uint8_t bcc = (uint8_t)(uid[0] ^ uid[1] ^ uid[2] ^ uid[3]);
        return answer({uid[0], uid[1], uid[2], uid[3], bcc});
      }
      if (nvb == 0x70) {                             // SELECT первого уровня
        if (state != READY) return timeout();
        if (memcmp(request.data() + 2, uid, 4) != 0) return timeout();
        state = ACTIVE;
        return answer({sak, 0x00, 0x00});            // SAK + CRC_A
      }
      return timeout();
    }
    if (opcode == 0x50) {                            // HALT
      ++halts;
      if (!card_present) return timeout();
      if (halt_ignored) return timeout();
      if (state != ACTIVE && state != READY) return timeout();
      state = HALT;
      return answer({});
    }
    return timeout();
  }
};

Gy87Model gy87;
Rc522Model rc522;

static void bind_hardware() {
  board.i2c_fn = [](uint8_t address, const uint8_t* tx, uint8_t tx_length,
                    uint8_t* rx, uint8_t rx_length) {
    return gy87.handle(address, tx, tx_length, rx, rx_length);
  };
  board.spi_fn = [](uint8_t out) { return rc522.transfer(out); };
  board.write_fn = [](uint8_t pin, uint8_t level) { rc522.onPinChange(pin, level); };
}


// ============================== обвязка =====================================
static void boot() {
  sim::boards = {&board};
  board.setup_fn = sensors::setup;
  board.loop_fn = sensors::loop;
  board.reset_stats();
  board.serial.output.clear();
  board.serial.input.clear();
  board.setup_fn();
  // 250 мс — первый кадр телеметрии; ничего не командуем, реле и замок в покое.
  sim::run_for_ms(250);
}

static void send(const std::string& line) { board.serial.send(line); }

static void run(double milliseconds) { sim::run_for_ms(milliseconds); }

static bool wait_until(const std::function<bool()>& predicate, double timeout_ms) {
  const int64_t end = sim::now_us + (int64_t)(timeout_ms * 1000.0);
  while (sim::now_us < end) {
    if (predicate()) return true;
    sim::run_for_ms(1);
  }
  return predicate();
}

//: Всё, что плата выдала после отметки mark (отметка сдвигается).
static std::string take_output(size_t& mark) {
  const std::string all = board.serial.output;
  const std::string fresh = mark < all.size() ? all.substr(mark) : std::string();
  mark = all.size();
  return fresh;
}

static bool contains(const std::string& text, const char* needle) {
  return text.find(needle) != std::string::npos;
}

//: Последний кадр с данным префиксом в свежем выводе.
static std::string lastFrame(const std::string& text, const char* prefix) {
  std::string found;
  size_t position = 0;
  while (true) {
    const size_t start = text.find(prefix, position);
    if (start == std::string::npos) break;
    const size_t end = text.find('\n', start);
    found = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
    position = start + 1;
  }
  return found;
}

//: Значение числового поля key=… в кадре (-1, если поля нет).
static long field(const std::string& frame, const char* key) {
  const std::string pattern = std::string(key) + "=";
  const size_t at = frame.find(pattern);
  if (at == std::string::npos) return -1;
  return std::strtol(frame.c_str() + at + pattern.size(), nullptr, 10);
}

//: Значение строкового поля key=… в кадре (пустая строка, если поля нет).
static std::string fieldText(const std::string& frame, const char* key) {
  const std::string pattern = std::string(key) + "=";
  const size_t at = frame.find(pattern);
  if (at == std::string::npos) return std::string();
  const size_t start = at + pattern.size();
  const size_t end = frame.find(' ', start);
  return frame.substr(start, end == std::string::npos ? std::string::npos : end - start);
}

//: Приложить карту заново: между чтениями карта должна выйти из поля (HALT) и
//: пройти паузу CARD_COOLDOWN_MS, иначе прошивка намеренно молчит.
static void presentCard(const uint8_t* uid) {
  rc522.remove();
  run(150);            // прошивка видит пустое поле и ждёт карту заново
  rc522.insert(uid);
}

//: Дождаться кадра @CARD после отметки mark.
static std::string readCard(size_t& mark, double timeout_ms = 600) {
  wait_until([&]() { return contains(board.serial.output.substr(mark), "@CARD "); }, timeout_ms);
  return lastFrame(board.serial.output.substr(mark), "@CARD ");
}

//: Кадр телеметрии, свежий после отметки mark.
static std::string telemetry(size_t& mark, double timeout_ms = 400.0) {
  std::string frame;
  wait_until([&]() {
    const std::string fresh = take_output(mark);
    const std::string candidate = lastFrame(fresh, "@S ");
    if (!candidate.empty()) frame = candidate;
    return !frame.empty();
  }, timeout_ms);
  return frame;
}

static int countLows(const uint8_t* pins, int n) {
  int lows = 0;
  for (int i = 0; i < n; ++i) {
    if (board.level[pins[i]] == LOW) ++lows;
  }
  return lows;
}

static const uint8_t RELAYS[4] = {2, 3, 4, 5};

// ============================== сценарии ====================================
static void scenario_boot_safe() {
  boot();
  // Реле модуля активны низким уровнем: «выключено» = HIGH на всех четырёх.
  CHECK(countLows(RELAYS, 4) == 0, "после сброса реле должны быть выключены");
  CHECK(board.servo_attached && board.servo_pin == 7, "серво замка на D7");
  CHECK(board.servo_value == sensors::LOCK_CLOSED_ANGLE,
        "после сброса замок закрыт, angle=%d", board.servo_value);
  if (sensors::STRIP_ENABLED) {
    CHECK(board.pixels.size() == sensors::STRIP_LEDS, "лента инициализирована");
    for (uint32_t pixel : board.pixels) CHECK(pixel == 0, "после сброса лента погашена");
  } else {
    // Ленты пока нет: прошивка не должна ни писать пиксели, ни оставлять D6
    // в воздухе — линия данных прижата к земле.
    CHECK(board.pixels.empty(), "без ленты пиксели не пишутся");
    CHECK(board.strip_shows == 0, "без ленты show() не вызывается");
    CHECK(board.level[6] == LOW, "без ленты D6 прижат к земле");
  }
  CHECK(sim::wdt_enabled != 0, "watchdog включён");
  size_t mark = 0;
  const std::string fresh = take_output(mark);
  CHECK(contains(fresh, "@S "), "после сброса идёт телеметрия");
  const std::string frame = lastFrame(fresh, "@S ");
  CHECK(field(frame, "cards") == 0, "карт в чистой EEPROM нет");
  CHECK(field(frame, "err") == 0, "все датчики ответили, err=%ld", field(frame, "err"));
  CHECK(field(frame, "lock") == 0, "замок закрыт");
  CHECK(field(frame, "fw") == sensors::SENSOR_PROTOCOL, "версия протокола в кадре");
  CHECK(field(frame, "strip") == (sensors::STRIP_ENABLED ? 1 : 0),
        "признак ленты в кадре: strip=%ld", field(frame, "strip"));
}

static void scenario_compass() {
  boot();
  run(600);            // барометр опрашивается 2 раза в секунду
  size_t mark = board.serial.output.size();
  std::string frame = telemetry(mark);
  CHECK(field(frame, "hdg") >= -5 && field(frame, "hdg") <= 5,
        "поле на север: hdg=%ld", field(frame, "hdg"));
  CHECK(field(frame, "press") == 69964, "барометр по даташиту: press=%ld",
        field(frame, "press"));
  CHECK(field(frame, "temp") == 150, "температура 15.0 °C: temp=%ld", field(frame, "temp"));

  gy87.setHeading(90.0f);
  run(300);
  frame = telemetry(mark);
  CHECK(field(frame, "hdg") >= 8950 && field(frame, "hdg") <= 9050,
        "поле на запад: hdg=%ld", field(frame, "hdg"));

  gy87.setHeading(180.0f);
  run(300);
  frame = telemetry(mark);
  CHECK(field(frame, "hdg") >= 17950 && field(frame, "hdg") <= 18050,
        "поле на юг: hdg=%ld", field(frame, "hdg"));
}

static void scenario_tilt_compensation() {
  boot();
  // Наклон 20° вокруг оси X: без компенсации компас «поедет» на десятки градусов.
  const float tilt = 20.0f * 3.14159265f / 180.0f;
  gy87.ax = 0.0f;
  gy87.ay = sinf(tilt);
  gy87.az = cosf(tilt);
  run(300);
  size_t mark = board.serial.output.size();
  const std::string frame = telemetry(mark);
  CHECK(field(frame, "roll") >= 1950 && field(frame, "roll") <= 2050,
        "крен 20°: roll=%ld", field(frame, "roll"));
  CHECK(field(frame, "pitch") >= -50 && field(frame, "pitch") <= 50,
        "тангаж 0: pitch=%ld", field(frame, "pitch"));
  CHECK(field(frame, "hdg") >= -200 && field(frame, "hdg") <= 200,
        "компас с компенсацией наклона: hdg=%ld", field(frame, "hdg"));
  gy87.ay = 0.0f;
  gy87.az = 1.0f;
}

static void scenario_lights() {
  boot();
  size_t mark = board.serial.output.size();
  send("light head on");
  CHECK(wait_until([]() { return board.level[2] == LOW; }, 100),
        "передний свет включает реле 1 (D2)");
  std::string fresh = take_output(mark);
  const std::string ack = lastFrame(fresh, "@ACK ");
  CHECK(field(ack, "ok") == 1, "ответ на команду: %s", ack.c_str());
  CHECK(field(lastFrame(fresh, "@S "), "light") == sensors::LIGHT_HEAD,
        "в телеметрии горит только передний свет");

  send("light stop on");
  CHECK(wait_until([]() { return board.level[3] == LOW; }, 100), "стоп включает реле 2 (D3)");
  send("light right on");
  CHECK(wait_until([]() { return board.level[5] == LOW; }, 400),
        "правый поворотник включает реле 4 (D5)");

  mark = board.serial.output.size();
  send("light all off");
  CHECK(wait_until([]() { return countLows(RELAYS, 4) == 0; }, 200), "всё выключено");
  fresh = take_output(mark);
  CHECK(field(lastFrame(fresh, "@S "), "light") == 0, "в телеметрии light=0");
}

static void scenario_turn_blink() {
  boot();
  send("light left on");
  int transitions = 0;
  uint8_t previous = board.level[4];
  const int64_t end = sim::now_us + 1500 * 1000;
  while (sim::now_us < end) {
    sim::run_for_ms(1);
    if (board.level[4] != previous) {
      previous = board.level[4];
      ++transitions;
    }
  }
  CHECK(transitions >= 3, "поворотник мигает, переходов за 1.5 с: %d", transitions);

  if (sensors::STRIP_ENABLED) {
    // Лента в фазе «горит»: янтарная половина слева, правая погашена.
    bool saw_left_amber = false;
    bool saw_right_dark = false;
    for (int i = 0; i < 40; ++i) {
      sim::run_for_ms(25);
      const uint32_t left = board.pixels.empty() ? 0 : board.pixels.front();
      const uint32_t right = board.pixels.empty() ? 0 : board.pixels.back();
      // Янтарный = красный заметно больше зелёного; сравнивать с синим нельзя,
      // это проверка цвета, а не яркости.
      if (left != 0 && ((left >> 16) & 0xFF) > (((left >> 8) & 0xFF) + 4)) saw_left_amber = true;
      if (right == 0) saw_right_dark = true;
    }
    CHECK(saw_left_amber, "левая половина ленты горит янтарным");
    CHECK(saw_right_dark, "правая половина ленты не горит");

    send("light left off");
    run(200);
    bool all_dark = true;
    for (uint32_t pixel : board.pixels) {
      if (pixel != 0) all_dark = false;
    }
    CHECK(all_dark, "после выключения поворотника лента гаснет");
  } else {
    // Без ленты мигание видно только на реле, а лента не трогается вовсе.
    CHECK(board.strip_shows == 0, "без ленты поворотник не пишет в пиксели");
    CHECK(board.level[6] == LOW, "без ленты D6 остаётся прижатым к земле");
    send("light left off");
    run(200);
    CHECK(countLows(RELAYS, 4) == 0, "поворотник выключен на реле");
  }
}

static void scenario_dim_strip() {
  boot();
  size_t mark = board.serial.output.size();
  send("dim 200");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@ACK "); }, 200),
        "команда dim принята");
  send("light dim on");
  run(300);
  if (!sensors::STRIP_ENABLED) {
    // Ленты пока нет: габариты остаются только битом в телеметрии, а в пиксели
    // ничего не пишется. Команда strip off принимается молча.
    mark = board.serial.output.size();
    const std::string frame = telemetry(mark);
    CHECK((field(frame, "light") & sensors::LIGHT_DIM) != 0,
          "габариты отмечены в телеметрии: light=%ld", field(frame, "light"));
    CHECK(field(frame, "dim") == 200, "яркость габаритов принята: dim=%ld", field(frame, "dim"));
    CHECK(field(frame, "strip") == 0, "лента в кадре не заявлена");
    CHECK(board.strip_shows == 0, "без ленты пиксели не пишутся");
    CHECK(board.pixels.empty(), "буфера ленты нет");
    send("strip off");
    CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@ACK "); }, 200),
          "strip off принимается и без ленты");
    run(100);
    CHECK(board.strip_shows == 0, "strip off не трогает пиксели без ленты");
    CHECK(board.level[6] == LOW, "D6 прижат к земле");
    send("light dim off");
    run(100);
    return;
  }
  // Подсветка пола — тёплый белый: красного больше, синего меньше, но ровно по
  // всей ленте и во всех 73 диодах, иначе под роботом будут пятна.
  bool warm = !board.pixels.empty() && board.pixels.size() == sensors::STRIP_LEDS;
  const uint32_t first = board.pixels.empty() ? 0 : board.pixels.front();
  for (uint32_t pixel : board.pixels) {
    const uint8_t red = (pixel >> 16) & 0xFF;
    const uint8_t green = (pixel >> 8) & 0xFF;
    const uint8_t blue = pixel & 0xFF;
    if (pixel != first) warm = false;                 // лента ровная
    if (red == 0 || red < green || green < blue) warm = false;  // тёплый оттенок
    if (blue * 3 < red * 2) warm = false;             // не «оранжевая» лампа
  }
  CHECK(warm, "подсветка пола — ровный тёплый белый на всех %u диодах",
        (unsigned)sensors::STRIP_LEDS);
  const uint8_t level = board.pixels.empty() ? 0 : (uint8_t)((board.pixels.front() >> 16) & 0xFF);
  CHECK(level > 0 && level < sensors::STRIP_BRIGHTNESS,
        "яркость габаритов ниже предела ленты: %u", (unsigned)level);

  send("strip off");
  run(100);
  bool dark = true;
  for (uint32_t pixel : board.pixels) {
    if (pixel != 0) dark = false;
  }
  CHECK(dark, "strip off гасит ленту, даже если габариты включены");
}

static void scenario_lock_servo() {
  boot();
  size_t mark = board.serial.output.size();
  send("lock open 1");
  CHECK(wait_until([]() { return board.servo_value == sensors::LOCK_OPEN_ANGLE; }, 200),
        "замок открывается серво, angle=%d", board.servo_value);
  CHECK(contains(board.serial.output.substr(mark), "@ACK "), "команда замка подтверждена");

  mark = board.serial.output.size();
  CHECK(wait_until([&]() {
          return contains(board.serial.output.substr(mark), "kind=lock open=0");
        }, 1500), "через секунду замок закрывается сам");
  CHECK(board.servo_value == sensors::LOCK_CLOSED_ANGLE, "серво вернулось в закрытое");

  // Больше 10 с открытым замок не останется: команда обрезается по максимуму.
  send("lock open 60");
  CHECK(wait_until([]() { return board.servo_value == sensors::LOCK_OPEN_ANGLE; }, 200),
        "замок открыт по команде");
  run(10500);
  CHECK(board.servo_value == sensors::LOCK_CLOSED_ANGLE,
        "даже lock open 60 закрывается не позже 10 с, angle=%d", board.servo_value);
}

static void scenario_no_door_switch() {
  // Концевик (D8) не подключён и убран из прошивки: ни поля door=, ни кадров
  // kind=door быть не должно, а свободный вход ничего не запирает.
  boot();
  board.level[8] = LOW;        // «дверь открыта» на старом входе
  run(300);
  size_t mark = 0;
  const std::string out = take_output(mark);
  CHECK(!contains(out, "door="), "в телеметрии нет поля door=");
  CHECK(!contains(out, "kind=door"), "кадров о двери нет");
  CHECK(sim::wdt_enabled != 0, "watchdog включён");
}

static void scenario_card_denied() {
  boot();
  const uint8_t stranger[4] = {0x11, 0x22, 0x33, 0x44};
  rc522.insert(stranger);
  size_t mark = board.serial.output.size();
  CHECK(wait_until([&]() {
          return contains(board.serial.output.substr(mark), "@CARD ");
        }, 400), "чужая карта даёт событие");
  const std::string card = lastFrame(board.serial.output.substr(mark), "@CARD ");
  CHECK(card.find("uid=11223344") != std::string::npos, "UID в событии: %s", card.c_str());
  CHECK(field(card, "ok") == 0, "чужая карта отклонена: %s", card.c_str());
  CHECK(board.servo_value == sensors::LOCK_CLOSED_ANGLE, "замок не открылся");
  run(300);
  const std::string frame = telemetry(mark);
  CHECK(field(frame, "rfid") == sensors::RFID_DENIED, "в телеметрии отказ: rfid=%ld",
        field(frame, "rfid"));
}

static void scenario_card_granted() {
  boot();
  size_t mark = board.serial.output.size();
  send("card add A1B2C3D4 admin");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@ACK "); }, 200),
        "карта добавлена администратором");
  const uint8_t own_card[4] = {0xA1, 0xB2, 0xC3, 0xD4};
  rc522.insert(own_card);
  mark = board.serial.output.size();
  std::string card = readCard(mark);
  CHECK(card.find("uid=A1B2C3D4") != std::string::npos, "UID своей карты: %s", card.c_str());
  CHECK(field(card, "ok") == 1, "своя карта принята: %s", card.c_str());
  CHECK(fieldText(card, "role") == "admin", "роль в событии: %s", card.c_str());
  CHECK(fieldText(card, "act") == "open", "первое прикладывание открывает: %s", card.c_str());
  CHECK(field(card, "cards") == 1, "счётчик карт: %s", card.c_str());
  CHECK(wait_until([]() { return board.servo_value == sensors::LOCK_OPEN_ANGLE; }, 200),
        "своя карта открывает отсек");

  // Отсек держится открытым: это не трёхсекундное открытие по таймеру.
  run(5000);
  CHECK(board.servo_value == sensors::LOCK_OPEN_ANGLE,
        "администратор держит отсек открытым, angle=%d", board.servo_value);

  // Та же карта в поле не спамит: пауза 2 с между чтениями.
  mark = board.serial.output.size();
  run(3000);
  CHECK(!contains(board.serial.output.substr(mark), "@CARD "),
        "карта, забытая на считывателе, не переключает отсек повторно");

  // Второе прикладывание закрывает отсек.
  presentCard(own_card);
  mark = board.serial.output.size();
  card = readCard(mark);
  CHECK(field(card, "ok") == 1, "второе прикладывание принято: %s", card.c_str());
  CHECK(fieldText(card, "act") == "close", "второе прикладывание закрывает: %s", card.c_str());
  CHECK(board.servo_value == sensors::LOCK_CLOSED_ANGLE, "отсек закрыт, angle=%d",
        board.servo_value);

  // Третье — снова открывает: переключение, а не одноразовое открытие.
  presentCard(own_card);
  mark = board.serial.output.size();
  card = readCard(mark);
  CHECK(fieldText(card, "act") == "open", "третье прикладывание открывает: %s", card.c_str());
  CHECK(board.servo_value == sensors::LOCK_OPEN_ANGLE, "отсек снова открыт");

  // Список карт с ролями: по кадру на карту и терминатор.
  mark = board.serial.output.size();
  send("card list");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@CARDINFO end=1"); },
                   300), "card list отвечает машинным списком");
  const std::string info = lastFrame(board.serial.output.substr(mark), "@CARDINFO idx=1");
  CHECK(info.find("uid=A1B2C3D4") != std::string::npos, "список показывает UID: %s", info.c_str());
  CHECK(fieldText(info, "role") == "admin", "список показывает роль: %s", info.c_str());
}

static void scenario_card_user_needs_cargo() {
  boot();
  send("card add A1B2C3D4 user");
  run(60);
  const uint8_t card_uid[4] = {0xA1, 0xB2, 0xC3, 0xD4};

  // Вне окна погрузки карта пользователя отсек не открывает.
  rc522.insert(card_uid);
  size_t mark = board.serial.output.size();
  std::string card = readCard(mark);
  CHECK(field(card, "ok") == 0, "пользователь без окна погрузки отклонён: %s", card.c_str());
  CHECK(fieldText(card, "why") == "cargo", "причина отказа — cargo: %s", card.c_str());
  CHECK(fieldText(card, "role") == "user", "роль пользователя видна: %s", card.c_str());
  CHECK(board.servo_value == sensors::LOCK_CLOSED_ANGLE, "отсек закрыт");

  // Окно погрузки открывает карте пользователя доступ.
  mark = board.serial.output.size();
  send("cargo load");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "kind=cargo state=load"); },
                   300), "окно погрузки объявлено событием");
  run(300);
  const std::string frame = telemetry(mark);
  CHECK(field(frame, "cargo") == sensors::CARGO_LOAD, "в телеметрии cargo=1: %s", frame.c_str());

  presentCard(card_uid);
  mark = board.serial.output.size();
  card = readCard(mark, 800);
  CHECK(field(card, "ok") == 1, "на погрузке пользователь пущен: %s", card.c_str());
  CHECK(fieldText(card, "act") == "open", "отсек открыт на погрузке: %s", card.c_str());
  CHECK(board.servo_value == sensors::LOCK_OPEN_ANGLE, "замок открыт");

  // Второе прикладывание закрывает отсек и на погрузке.
  presentCard(card_uid);
  mark = board.serial.output.size();
  card = readCard(mark, 800);
  CHECK(fieldText(card, "act") == "close", "второе прикладывание закрывает: %s", card.c_str());
  CHECK(board.servo_value == sensors::LOCK_CLOSED_ANGLE, "замок закрыт");

  // Выгрузка — тоже разрешённое окно.
  send("cargo unload");
  run(300);
  presentCard(card_uid);
  mark = board.serial.output.size();
  card = readCard(mark, 800);
  CHECK(field(card, "ok") == 1, "на выгрузке пользователь пущен: %s", card.c_str());
  CHECK(board.servo_value == sensors::LOCK_OPEN_ANGLE, "замок открыт на выгрузке");

  // Конец окна: карта пользователя снова не действует.
  mark = board.serial.output.size();
  send("cargo end");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "kind=cargo state=end"); },
                   300), "окно закрыто событием");
  send("lock close");
  presentCard(card_uid);
  mark = board.serial.output.size();
  card = readCard(mark, 800);
  CHECK(field(card, "ok") == 0, "после окна пользователь снова отклонён: %s", card.c_str());
  CHECK(board.servo_value == sensors::LOCK_CLOSED_ANGLE, "отсек закрыт");
}

static void scenario_card_roles() {
  boot();
  size_t mark = board.serial.output.size();
  send("card add A1B2C3D4 user");
  send("card add 11223344 admin");
  run(80);
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@CARDINFO end=1"); },
                   300), "список карт после добавления");
  const std::string admin = lastFrame(board.serial.output.substr(mark), "@CARDINFO idx=2");
  CHECK(admin.find("uid=11223344") != std::string::npos, "вторая карта в списке: %s", admin.c_str());
  CHECK(fieldText(admin, "role") == "admin", "роль второй карты: %s", admin.c_str());

  // Смена роли: пользователь становится администратором и действует без окна.
  mark = board.serial.output.size();
  send("card role A1B2C3D4 admin");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@ACK "); }, 300),
        "роль меняется командой");
  const std::string info = lastFrame(board.serial.output.substr(mark), "@CARDINFO idx=1");
  CHECK(fieldText(info, "role") == "admin", "роль в списке обновилась: %s", info.c_str());

  const uint8_t card_uid[4] = {0xA1, 0xB2, 0xC3, 0xD4};
  rc522.insert(card_uid);
  mark = board.serial.output.size();
  const std::string card = readCard(mark);
  CHECK(field(card, "ok") == 1, "бывший пользователь теперь открывает: %s", card.c_str());
  CHECK(fieldText(card, "role") == "admin", "роль в событии: %s", card.c_str());

  // Роли переживают сброс платы.
  boot();
  mark = board.serial.output.size();
  send("card list");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@CARDINFO end=1"); },
                   300), "список после сброса");
  const std::string restored = lastFrame(board.serial.output.substr(mark), "@CARDINFO idx=1");
  CHECK(fieldText(restored, "role") == "admin", "роль поднялась из EEPROM: %s", restored.c_str());
}

static void scenario_card_legacy_migration() {
  // Старая прошивка хранила UID без ролей: блок 56 байт, версия 1.
  uint8_t block[sensors::CARD_BLOCK_BYTES_LEGACY];
  std::memset(block, 0xFF, sizeof(block));
  block[0] = 'S';
  block[1] = 'N';
  block[2] = 'S';
  block[3] = 1;
  block[4] = 1;
  block[5] = 0;
  const uint8_t uid[4] = {0x11, 0x22, 0x33, 0x44};
  std::memcpy(block + 6, uid, 4);
  const uint16_t crc = sensors::frameCrc16((const char*)block,
                                           sensors::CARD_BLOCK_BYTES_LEGACY - 2);
  block[sensors::CARD_BLOCK_BYTES_LEGACY - 2] = (uint8_t)crc;
  block[sensors::CARD_BLOCK_BYTES_LEGACY - 1] = (uint8_t)(crc >> 8);
  for (uint16_t i = 0; i < sensors::CARD_BLOCK_BYTES_LEGACY; ++i) {
    board.eeprom[sensors::CARD_EEPROM_ADDR + i] = block[i];
  }

  boot();
  size_t mark = board.serial.output.size();
  send("card list");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@CARDINFO end=1"); },
                   300), "старый блок читается");
  const std::string info = lastFrame(board.serial.output.substr(mark), "@CARDINFO idx=1");
  CHECK(fieldText(info, "role") == "admin",
        "перенесённая карта становится администратором: %s", info.c_str());

  // И она действительно открывает отсек без окна погрузки.
  rc522.insert(uid);
  mark = board.serial.output.size();
  const std::string card = readCard(mark);
  CHECK(field(card, "ok") == 1, "перенесённая карта открывает отсек: %s", card.c_str());
}

static void scenario_card_learn_persists() {
  boot();
  send("card learn admin");
  run(20);
  const uint8_t fresh[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  rc522.insert(fresh);
  size_t mark = board.serial.output.size();
  CHECK(wait_until([&]() {
          return contains(board.serial.output.substr(mark), "@CARD ");
        }, 400), "режим обучения ловит карту");
  const std::string card = lastFrame(board.serial.output.substr(mark), "@CARD ");
  CHECK(field(card, "ok") == 1, "карта записана: %s", card.c_str());
  CHECK(fieldText(card, "role") == "admin", "карта записана администратором: %s", card.c_str());
  CHECK(field(card, "cards") == 1, "в памяти одна карта: %s", card.c_str());

  // Сброс платы: список должен подняться из EEPROM вместе с ролью.
  boot();
  mark = board.serial.output.size();
  const std::string frame = telemetry(mark);
  CHECK(field(frame, "cards") == 1, "после сброса карта на месте: cards=%ld",
        field(frame, "cards"));
  send("card list");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@CARDINFO end=1"); },
                   300), "список после сброса");
  const std::string info = lastFrame(board.serial.output.substr(mark), "@CARDINFO idx=1");
  CHECK(fieldText(info, "role") == "admin", "роль запомнена: %s", info.c_str());
  CHECK(field(frame, "err") == 0, "EEPROM читается без ошибок: err=%ld", field(frame, "err"));
}

static void scenario_eeprom_corrupt() {
  boot();
  send("card add A1B2C3D4");
  run(50);
  // Портим последний байт блока карт — CRC не сойдётся.
  board.eeprom[sensors::CARD_EEPROM_ADDR + sensors::CARD_BLOCK_BYTES - 1] ^= 0xFF;
  boot();
  size_t mark = board.serial.output.size();
  const std::string frame = telemetry(mark);
  CHECK(field(frame, "cards") == 0, "битый блок не даёт карт: cards=%ld", field(frame, "cards"));
  CHECK((field(frame, "err") & sensors::ERR_EEPROM) != 0,
        "битый блок помечен ошибкой EEPROM: err=%ld", field(frame, "err"));
}

static void scenario_link_timeout() {
  boot();
  size_t mark = board.serial.output.size();
  send("lock open 10");
  CHECK(wait_until([]() { return board.servo_value == sensors::LOCK_OPEN_ANGLE; }, 200),
        "замок открыт");
  // Сервер молчит: через LINK_TIMEOUT_MS замок обязан закрыться сам.
  const bool closed = wait_until([&]() {
    return board.servo_value == sensors::LOCK_CLOSED_ANGLE &&
           contains(board.serial.output.substr(mark), "нет команд дольше");
  }, (double)sensors::LINK_TIMEOUT_MS + 2000);
  CHECK(closed, "без команд замок закрывается сам, angle=%d", board.servo_value);
  run(400);            // следующий кадр телеметрии уже после потери связи
  const std::string frame = telemetry(mark, 500);
  CHECK(field(frame, "link") == 0, "в телеметрии link=0: %s", frame.c_str());

  // Свет при потере связи не гаснет: робота нельзя оставить без фар.
  send("ping");
  run(100);
  mark = board.serial.output.size();
  send("light head on");
  run(1000);
  run((double)sensors::LINK_TIMEOUT_MS + 500);
  CHECK(board.level[2] == LOW, "потеря связи не выключает передний свет");
}

static void scenario_garbage_safe() {
  boot();
  size_t mark = board.serial.output.size();
  send("!!!!");
  send("light");
  send("light head maybe");
  send("card add XYZ");
  send("lock open -3");
  send("dim 999");
  send(std::string(200, 'x'));
  run(300);
  const std::string fresh = take_output(mark);
  CHECK(contains(fresh, "@ACK "), "на мусор приходит отказ, а не молчание");
  const std::string ack = lastFrame(fresh, "@ACK ");
  CHECK(field(ack, "ok") == 0, "неизвестная команда отклонена: %s", ack.c_str());
  CHECK(contains(fresh, "@S "), "телеметрия продолжается после мусора");
  CHECK(countLows(RELAYS, 4) == 0, "мусор не включает реле");
  CHECK(board.servo_value == sensors::LOCK_CLOSED_ANGLE, "мусор не открывает замок");
}

static void scenario_sensor_faults() {
  gy87.mag_present = false;
  rc522.present = false;
  boot();
  size_t mark = board.serial.output.size();
  const std::string frame = telemetry(mark);
  const long err = field(frame, "err");
  CHECK((err & sensors::ERR_MAG) != 0, "нет магнитометра — флаг MAG: err=%ld", err);
  CHECK((err & sensors::ERR_RFID) != 0, "нет RC522 — флаг RFID: err=%ld", err);
  CHECK((err & sensors::ERR_MPU) == 0, "MPU6050 на месте: err=%ld", err);
  CHECK((err & sensors::ERR_BARO) == 0, "BMP180 на месте: err=%ld", err);
  // Отказы датчиков не мешают свету и замку.
  send("light head on");
  CHECK(wait_until([]() { return board.level[2] == LOW; }, 100),
        "свет работает при отказавших датчиках");
  gy87.mag_present = true;
  rc522.present = true;
}

static void scenario_mag_calibration() {
  boot();
  size_t mark = board.serial.output.size();
  send("mag cal start");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@ACK "); }, 200),
        "калибровка начата");
  // Медленный оборот с большим постоянным смещением: без калибровки компас врёт.
  const float bias = 400.0f;
  for (int i = 0; i <= 12; ++i) {
    const float degrees = i * 30.0f;
    const float radians = degrees * 3.14159265f / 180.0f;
    gy87.mx = (int16_t)(300.0f * cosf(radians) + bias);
    gy87.my = (int16_t)(-300.0f * sinf(radians) + bias);
    run(120);
  }
  mark = board.serial.output.size();
  send("mag cal stop");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@ACK "); }, 200),
        "калибровка завершена");
  gy87.mx = (int16_t)(300.0f + bias);
  gy87.my = (int16_t)bias;
  run(300);

  mark = board.serial.output.size();
  const std::string frame = telemetry(mark);
  CHECK(field(frame, "hdg") >= -300 && field(frame, "hdg") <= 300,
        "после калибровки смещение убрано: hdg=%ld offset=(%.1f, %.1f) mx=%d my=%d",
        field(frame, "hdg"), sensors::mag_offset_x, sensors::mag_offset_y,
        (int)gy87.mx, (int)gy87.my);

  // Поправка сохраняется в EEPROM: после сброса компас по-прежнему верный.
  boot();
  run(300);
  mark = board.serial.output.size();
  const std::string after = telemetry(mark);
  CHECK(field(after, "hdg") >= -300 && field(after, "hdg") <= 300,
        "поправка компаса поднялась из EEPROM: hdg=%ld", field(after, "hdg"));
}

static void scenario_dump() {
  boot();
  send("light head on");
  run(60);
  send("lock open 1");
  run(60);
  send("card add A1B2C3D4 admin");
  run(60);
  send("card add 55667788 user");
  run(60);
  send("cargo load");
  run(60);
  const uint8_t stranger[4] = {0x11, 0x22, 0x33, 0x44};
  rc522.insert(stranger);
  run(300);
  const uint8_t own_card[4] = {0xA1, 0xB2, 0xC3, 0xD4};
  rc522.insert(own_card);
  run(300);
  send("card list");
  run(100);
  std::printf("%s", board.serial.output.c_str());
}

// ============================ запуск сценариев ==============================
struct Scenario {
  const char* name;
  void (*run)();
};

//: 73 диода внизу рамы: адресация, эффекты и команды режима.
static void scenario_strip_effects() {
  boot();
  CHECK(sensors::STRIP_LEDS == 73, "в ленте 73 диода, а не %u",
        (unsigned)sensors::STRIP_LEDS);
  CHECK(sensors::STRIP_PIN == 6, "DIN ленты на D6, а не %u", (unsigned)sensors::STRIP_PIN);
  // Признак ленты и число диодов уезжают в телеметрию: экран не врёт о ленте.
  size_t mark = board.serial.output.size();
  run(400);
  std::string frame = take_output(mark);
  CHECK(field(lastFrame(frame, "@S "), "strip") == (sensors::STRIP_ENABLED ? 1 : 0),
        "признак ленты в кадре совпадает со сборкой");
  CHECK(field(lastFrame(frame, "@S "), "nleds") == 73,
        "в кадре nleds=%ld", field(lastFrame(frame, "@S "), "nleds"));

  if (!sensors::STRIP_ENABLED) {
    CHECK(board.strip_shows == 0, "без ленты эффекты не трогают пиксели");
    return;
  }

  // Режим auto (по умолчанию): стоп красит всю ленту в красный.
  send("strip auto");
  send("light stop on");
  run(150);
  CHECK(board.pixels.size() == 73, "auto: буфер ровно на 73 диода");
  bool all_red = true;
  for (uint32_t pixel : board.pixels) {
    const uint8_t red = (pixel >> 16) & 0xFF;
    const uint8_t green = (pixel >> 8) & 0xFF;
    if (red == 0 || green != 0) all_red = false;
  }
  CHECK(all_red, "auto: стоп — все 73 диода красные");
  send("light stop off");
  run(150);

  // Режим floor: подсветка пола. Стопы и поворотники остаются на реле —
  // лента смотрит вниз, и красный под роботом ничего не сообщает сзади идущим.
  send("strip floor");
  run(150);
  CHECK(board.pixels.size() == 73, "floor: буфер ровно на 73 диода");
  bool all_on = true;
  for (uint32_t pixel : board.pixels) if (pixel == 0) all_on = false;
  CHECK(all_on, "floor: светятся все 73 диода");
  const uint32_t floor_color = board.pixels.empty() ? 0 : board.pixels.front();
  send("light stop on");
  run(150);
  CHECK(board.pixels.empty() || board.pixels.front() == floor_color,
        "floor: стоп не перекрашивает подсветку пола (он на реле)");
  // Реле активны низким уровнем: «включено» — это LOW на пине.
  CHECK(board.level[3] == LOW, "floor: при этом реле стопа включилось");
  send("light stop off");
  send("strip auto");
  run(150);

  // Волна: бегущая голова идёт по кругу, хвост затухает.
  send("strip wave");
  run(120);
  auto brightest_index = [&]() -> int {
    int best = -1;
    uint8_t level = 0;
    for (uint16_t i = 0; i < board.pixels.size(); ++i) {
      const uint8_t red = (board.pixels[i] >> 16) & 0xFF;
      if (red > level) { level = red; best = (int)i; }
    }
    return best;
  };
  const int before = brightest_index();
  CHECK(before >= 0, "волна: есть светящийся диод");
  int moved = 0;
  for (int step = 0; step < 40 && moved < 3; ++step) {
    const int was = brightest_index();
    sim::run_for_ms(20);
    if (brightest_index() != was) ++moved;
  }
  CHECK(moved > 0, "волна бежит: голова смещается по ленте");

  // off: лента гаснет целиком, команда подтверждается.
  send("strip off");
  run(200);
  bool dark = true;
  for (uint32_t pixel : board.pixels) if (pixel != 0) dark = false;
  CHECK(dark, "strip off гасит все 73 диода");

  // Неизвестный режим — отказ, а не молчаливое переключение.
  mark = board.serial.output.size();
  send("strip rainbow");
  CHECK(wait_until([&]() { return contains(board.serial.output.substr(mark), "@ACK "); }, 200),
        "strip rainbow отвечает отказом");
  CHECK(contains(board.serial.output.substr(mark), "cmd=strip ok=0"), "неизвестный режим отклонён");
}

static const Scenario SCENARIOS[] = {
    {"boot_safe", scenario_boot_safe},
    {"compass", scenario_compass},
    {"tilt_compensation", scenario_tilt_compensation},
    {"lights", scenario_lights},
    {"turn_blink", scenario_turn_blink},
    {"dim_strip", scenario_dim_strip},
    {"strip_effects", scenario_strip_effects},
    {"lock_servo", scenario_lock_servo},
    {"no_door_switch", scenario_no_door_switch},
    {"card_denied", scenario_card_denied},
    {"card_granted", scenario_card_granted},
    {"card_learn_persists", scenario_card_learn_persists},
    {"card_user_needs_cargo", scenario_card_user_needs_cargo},
    {"card_roles", scenario_card_roles},
    {"card_legacy_migration", scenario_card_legacy_migration},
    {"eeprom_corrupt", scenario_eeprom_corrupt},
    {"link_timeout", scenario_link_timeout},
    {"garbage_safe", scenario_garbage_safe},
    {"sensor_faults", scenario_sensor_faults},
    {"mag_calibration", scenario_mag_calibration},
};

int main(int argc, char** argv) {
  bind_hardware();
  if (argc > 1 && std::strcmp(argv[1], "dump") == 0) {
    scenario_dump();
    return 0;
  }
  int failed = 0;
  int ran = 0;
  for (const Scenario& scenario : SCENARIOS) {
    if (argc > 1 && std::strcmp(argv[1], scenario.name) != 0) continue;
    ++ran;
    std::fflush(stdout);
    const pid_t pid = fork();
    if (pid == 0) {
      scenario.run();
      std::fflush(stdout);
      if (failures) std::fprintf(stderr, "%s", board.serial.output.c_str());
      _exit(failures ? 1 : 0);
    }
    int status = 0;
    waitpid(pid, &status, 0);
    const bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
    std::printf("%-24s %s\n", scenario.name, ok ? "ok" : "FAIL");
    if (!ok) ++failed;
  }
  if (!ran) {
    std::fprintf(stderr, "нет сценария %s\n", argc > 1 ? argv[1] : "");
    return 2;
  }
  std::printf("%d/%d сценариев прошли\n", ran - failed, ran);
  return failed ? 1 : 0;
}
