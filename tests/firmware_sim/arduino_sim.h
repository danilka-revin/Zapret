// Хост-симуляция Arduino Mega для прошивки firmware/motor_wheel (колесо руля).
//
// Каждая плата — свой экземпляр Board: выводы, USB Serial, модель шагового
// привода (TB6600 шагает по фронту PUL, если ENA = HIGH, направление — по DIR)
// и оптодатчика нуля на D18 (флажок с гистерезисом). Заглушки Wire/I2C ниже
// оставлены для совместимости заголовков, сама прошивка I²C не использует.
// Скетч компилируется внутри своего namespace (см. sim.cpp), а функции
// digitalWrite/Serial/Wire там подменяются привязанными к своей плате.
// Время общее: тик Timer1 — 50 мкс, прерывания всех плат вызываются по тику,
// loop() каждой платы — между тиками.
#pragma once

#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <strings.h>
#include <vector>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 3
#define DEFAULT 1
// На Mega 2560 аналоговые входы — это цифровые выводы 54…69.
#define A0 54
#define A1 55
#define A2 56
#define A3 57
#define A4 58
#define A5 59
#define A6 60
#define A7 61
#define A8 62
#define A9 63
#define A10 64
#define A11 65
#define A12 66
#define A13 67
#define A14 68
#define A15 69
#define PROGMEM
#define PSTR(s) (s)
#define F(s) (s)
#define snprintf_P snprintf
#define vsnprintf_P vsnprintf
#define strncpy_P strncpy
#define strcpy_P strcpy
#define strcmp_P strcmp
#define strcasecmp_P strcasecmp
#define memcpy_P memcpy
#define ISR(vector) void vector##_handler()

typedef uint8_t byte;
typedef bool boolean;

// Регистры Timer1: прошивка их только настраивает, ход задаёт симуляция.
inline uint8_t TCCR1A = 0, TCCR1B = 0, TIMSK1 = 0;
inline uint16_t OCR1A = 0, TCNT1 = 0;
constexpr int WGM12 = 3, CS11 = 1, OCIE1A = 1;

namespace sim {

inline int64_t now_us = 0;
inline int64_t next_tick_us = 50;
void advance(int64_t us);

struct Board;

class SimSerial {
 public:
  std::deque<uint8_t> input;
  std::string output;
  void begin(unsigned long) {}
  int available() { return (int)input.size(); }
  int read() {
    if (input.empty()) return -1;
    const int ch = input.front();
    input.pop_front();
    return ch;
  }
  int availableForWrite() { return 63; }
  size_t write(uint8_t ch) {
    output.push_back((char)ch);
    return 1;
  }
  void send(const std::string& line) {
    for (char ch : line) input.push_back((uint8_t)ch);
    input.push_back('\n');
  }
};

class SimWire;
inline SimWire* bus[128] = {nullptr};

// Авария шины: слейв (или помеха) удерживает SDA низким. Пока флаг поднят,
// передачи не проходят (как NACK/таймаут на железе), а восстановление
// (9 тактов SCL) снимает его — см. Board::digital_write и i2cRecoverBus().
inline bool i2c_bus_stuck = false;
inline int i2c_scl_edges = 0;
// Сколько плат зарегистрировано на каждом адресе: >1 — коллизия адресов,
// как у трёх плат, прошитых одним motor_node.ino (данные на шине рвутся).
inline int bus_claims[128] = {0};

// Детерминированный ГПСЧ для помех на шине: сценарии гоняются в своих
// процессах (fork), поэтому каждый стартует с одного и того же зерна.
inline uint64_t rng_state = 0x243F6A8885A308D3ULL;
inline uint32_t rng_next() {
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 7;
  rng_state ^= rng_state << 17;
  return (uint32_t)(rng_state >> 32);
}
inline bool roll(double probability) {
  if (probability <= 0.0) return false;
  if (probability >= 1.0) return true;
  return (rng_next() >> 8) * (1.0 / 16777216.0) < probability;
}

class SimWire {
 public:
  Board* board = nullptr;
  int address = -1;
  bool connected = true;                 // false — плата «отключена» от шины
  void (*on_receive)(int) = nullptr;
  void (*on_request)() = nullptr;
  std::vector<uint8_t> tx;               // мастер: передача; слейв: ответ на запрос
  std::deque<uint8_t> rx;                // принятые байты
  int tx_address = -1;
  long transmissions = 0;
  uint32_t clock_hz = 400000;            // скорость шины у МАСТЕРА (setClock)
  uint32_t timeout_us = 1000000;         // Wire.setWireTimeout: сколько ждать ответ
  // Маргинальный шлейф: плата отвечает только на 100 кГц. Так выглядит дальний
  // модуль (обычно RR) с длинным проводом и внутренними подтяжками: на 400 кГц
  // он NACK-ает, на 100 кГц — работает. Мастер обязан сам это заметить.
  bool slow_bus_only = false;
  // Сколько ближайших кадров придёт битыми (помеха/обрыв): мастер должен
  // повторить запрос, а не объявлять модуль потерянным с первого сбоя.
  uint8_t corrupt_frames = 0;
  // Плата отвечает на этой скорости и медленнее (slow_bus_only == max_ok_hz
  // 100000). Выше — NACK/таймаут, как у дальнего модуля на длинном шлейфе.
  uint32_t max_ok_hz = 400000;
  // Потери от скорости: вероятность провала транзакции на 400 кГц, на
  // медленной шине пропорционально меньше («фронты SDA не успевают»).
  double loss_rate = 0.0;
  // Пачки помех (моторы, ШИМ): транзакция с вероятностью burst_rate открывает
  // окно на burst_ms, в котором все обращения к плате падают. Так на роботе
  // выглядят наводки: связь пропадает пачками, а не по одному кадру.
  double burst_rate = 0.0;
  double burst_ms = 0.0;
  int64_t burst_until_us = 0;

  void begin() { unregister(); }
  void begin(uint8_t addr) {
    unregister();
    address = addr;
    bus[addr] = this;
    ++bus_claims[addr];
  }
  // Wire.end(): плата снимает свой адрес с шины (как twi_disable на AVR).
  void end() { unregister(); }
  void setClock(uint32_t hz) { clock_hz = hz; }
  // Таймаут Wire на AVR отсчитывается на ВСЮ передачу: пока мастер ждёт ответ,
  // loop() стоит. Симуляция обязана это время тратить, иначе блокировки шины в
  // прошивке не видны (опрос «зависшей» шины выглядит бесплатным).
  void setWireTimeout(uint32_t us, bool) { timeout_us = us ? us : 1000000; }
  void onReceive(void (*fn)(int)) { on_receive = fn; }
  void onRequest(void (*fn)()) { on_request = fn; }
  // Время передачи: старт + по 9 бит на байт (данные + ACK) + стоп.
  static int64_t wire_us(uint32_t hz, size_t bytes) {
    if (hz == 0) hz = 100000;
    return (int64_t)(((uint64_t)bytes + 2) * 9 * 1000000 / hz);
  }
  // Отказ адреса (NACK) на железе возвращается сразу — ждём только адресную фазу.
  void spend_nack() { sim::advance(wire_us(clock_hz, 1)); }
  // Зависшая шина: мастер крутится до таймаута (twi_timeout_us).
  void spend_timeout() { sim::advance((int64_t)timeout_us); }
  // Общая модель отказа транзакции к этой плате. hz — скорость шины МАСТЕРА:
  // маргинальный шлейф глушит именно её, а не тактование самой платы.
  bool bus_fail(uint32_t hz) {
    if (!connected) return true;
    if (slow_bus_only && hz > 100000) return true;
    if (hz > max_ok_hz) return true;
    if (sim::now_us < burst_until_us) return true;
    if (roll(burst_rate)) {
      burst_until_us = sim::now_us + (int64_t)(burst_ms * 1000.0);
      return true;
    }
    return roll(loss_rate * (double)hz / 400000.0);
  }
  void beginTransmission(uint8_t addr) {
    tx_address = addr;
    tx.clear();
  }
  size_t write(uint8_t value) {
    if (tx.size() >= 32) return 0;       // буфер Wire на AVR — 32 байта
    tx.push_back(value);
    return 1;
  }
  size_t write(const uint8_t* data, size_t length) {
    size_t written = 0;
    for (size_t i = 0; i < length; ++i) written += write(data[i]);
    return written;
  }
  uint8_t endTransmission() {
    ++transmissions;
    SimWire* slave = tx_address >= 0 && tx_address < 128 ? bus[tx_address] : nullptr;
    if (i2c_bus_stuck) {                 // шина удерживается: BUS ERROR после таймаута
      spend_timeout();
      return 4;
    }
    if (!slave || !slave->on_receive) {  // NACK адреса
      spend_nack();
      return 2;
    }
    if (slave->bus_fail(clock_hz)) {     // помеха/скорость/обрыв
      spend_nack();
      return 2;
    }
    slave->rx.clear();
    for (uint8_t value : tx) slave->rx.push_back(value);
    slave->on_receive((int)tx.size());
    sim::advance(wire_us(clock_hz, tx.size() + 1));
    return 0;
  }
  uint8_t requestFrom(uint8_t addr, uint8_t quantity) {
    rx.clear();
    SimWire* slave = addr < 128 ? bus[addr] : nullptr;
    if (i2c_bus_stuck) {                 // таймаут: шина не отпускает SDA
      spend_timeout();
      return 0;
    }
    if (!slave || !slave->on_request) {
      spend_nack();
      return 0;
    }
    if (slave->bus_fail(clock_hz)) {     // помеха/скорость/обрыв
      spend_nack();
      return 0;
    }
    slave->tx.clear();
    slave->on_request();
    for (uint8_t i = 0; i < quantity; ++i) {
      rx.push_back(i < slave->tx.size() ? slave->tx[i] : 0xFF);  // дальше — линия в 1
    }
    sim::advance(wire_us(clock_hz, (size_t)quantity + 1));
    // Помеха: кадр пришёл битым один раз.
    if (slave->corrupt_frames > 0 && !rx.empty()) {
      rx[0] = '#';
      --slave->corrupt_frames;
    }
    // На адресе несколько плат — SDA тянут все сразу, кадр приходит битым.
    if (bus_claims[addr] > 1 && !rx.empty()) rx[0] = '#';
    return quantity;
  }
  int available() { return (int)rx.size(); }
  int read() {
    if (rx.empty()) return -1;
    const int value = rx.front();
    rx.pop_front();
    return value;
  }

 private:
  void unregister() {
    if (address >= 0 && address < 128) {
      if (bus[address] == this) bus[address] = nullptr;
      if (bus_claims[address] > 0) --bus_claims[address];
    }
    address = -1;
  }
};

struct Board {
  const char* name = "";
  uint8_t level[80] = {0};
  uint8_t mode[80] = {0};
  int analog[80] = {0};
  SimSerial serial;
  SimWire wire;
  // EEPROM платы (ATmega2560 — 4 КиБ, здесь хватает 1 КиБ): адрес узла и т.п.
  uint8_t eeprom[1024] = {0};
  uint8_t eeprom_read(uint16_t address) { return eeprom[address & 1023]; }
  void eeprom_write(uint16_t address, uint8_t value) { eeprom[address & 1023] = value; }
  void (*setup_fn)() = nullptr;
  void (*loop_fn)() = nullptr;
  void (*isr_fn)() = nullptr;
  void (*tach_fn)() = nullptr;

  // Модель руля: phys — где колесо на самом деле, в шагах от «прямо».
  long phys = 0;
  long phys_min = 0, phys_max = 0;
  // Заклинивание колеса: phys замирает на stall_phys, хотя драйвер продолжает
  // получать импульсы. Прошивка считает СГЕНЕРИРОВАННЫЕ импульсы, а не
  // положение колеса, поэтому открытый контур по шагам считает, что доехали.
  bool stall_enabled = false;
  long stall_phys = 0;
  long steps_lost = 0;
  // Модель потенциометра на A0: движок жёстко связан с колесом (1:1),
  // показывает угол (phys + pot_bias_steps) / phys_per_degree.
  bool pot_present = true;
  double pot_center_raw = 512.0;
  double pot_counts_per_degree = 1023.0 / 270.0;
  double pot_bias_steps = 0.0;
  double phys_per_degree = 133.333333333333;
  bool opto_present = true;
  bool opto_stuck = false;
  long flag_lo = -200, flag_hi = 200;    // шаги, где датчик закрыт
  long hysteresis = 20;
  bool opto_state = false;
  // Разброс срабатывания: при каждом освобождении датчика следующий сдвиг края флажка
  // берётся из списка (по кругу). Сдвиг вступает, только когда колесо вне старого и
  // нового окна: как люфт между подходами, а не перескок флажка под колесом.
  std::vector<long> flag_shift_script;
  size_t flag_shift_index = 0;
  long flag_shift = 0;
  bool flag_shift_pending = false;
  long flag_shift_next = 0;
  // Проверки сигналов TB6600.
  int64_t last_dir_change_us = -1000000;
  int64_t pul_rise_us = 0;
  int64_t last_rise_us = -1;
  int64_t min_high_us = 1000000;
  int64_t min_period_us = 1000000;
  long dir_setup_violations = 0;
  long steps = 0;
  long steps_while_disabled = 0;

  void reset_stats() {
    phys_min = phys_max = phys;
    min_high_us = min_period_us = 1000000;
    last_rise_us = -1;
    steps = 0;
    max_loop_us = 0;
  }

  // Самый долгий проход loop(): пока мастер ждёт Wire, loop() стоит — не качает
  // USB (телеметрия, heartbeat тяги) и не обновляет watchdog тяги (650 мс).
  int64_t max_loop_us = 0;

  bool opto_active() {
    if (opto_stuck) return true;
    if (!opto_present) return false;
    const long lo = flag_lo + flag_shift;
    const long hi = flag_hi + flag_shift;
    const bool inside = phys >= lo && phys <= hi;
    const bool held = phys >= lo - hysteresis && phys <= hi + hysteresis;
    const bool was = opto_state;
    opto_state = was ? held : inside;
    if (was && !opto_state && !flag_shift_script.empty()) {
      flag_shift_next = flag_shift_script[flag_shift_index % flag_shift_script.size()];
      ++flag_shift_index;
      flag_shift_pending = true;
    }
    if (flag_shift_pending) {
      const long next_lo = flag_lo + flag_shift_next;
      const long next_hi = flag_hi + flag_shift_next;
      const long union_lo = (lo < next_lo ? lo : next_lo) - hysteresis;
      const long union_hi = (hi > next_hi ? hi : next_hi) + hysteresis;
      if (phys < union_lo || phys > union_hi) {
        flag_shift = flag_shift_next;
        flag_shift_pending = false;
      }
    }
    return opto_state;
  }

  void pin_mode(uint8_t pin, uint8_t value) {
    mode[pin] = value;
    if (value == INPUT_PULLUP) level[pin] = HIGH;
  }

  void digital_write(uint8_t pin, uint8_t value) {
    value = value ? HIGH : LOW;
    const uint8_t before = level[pin];
    level[pin] = value;
    // Восстановление зависшей шины: 9 тактов SCL освобождают SDA.
    if (pin == 21 && mode[21] == OUTPUT && before == LOW && value == HIGH && i2c_bus_stuck) {
      if (++i2c_scl_edges >= 9) {
        i2c_bus_stuck = false;
        i2c_scl_edges = 0;
        level[20] = HIGH;
      }
    }
    if (pin == 7 && before != value) last_dir_change_us = now_us;
    if (pin != 6 || before == value) return;
    if (value == HIGH) {
      // TB6600 делает шаг по фронту PUL.
      if (now_us - last_dir_change_us < 5) ++dir_setup_violations;
      if (last_rise_us >= 0 && now_us - last_rise_us < min_period_us) min_period_us = now_us - last_rise_us;
      last_rise_us = now_us;
      pul_rise_us = now_us;
      if (level[8] != HIGH) {
        ++steps_while_disabled;
        return;
      }
      const long delta = level[7] == LOW ? 1 : -1;
      if (stall_enabled && ((delta > 0 && phys >= stall_phys) ||
                            (delta < 0 && phys <= stall_phys))) {
        ++steps_lost;
      } else {
        phys += delta;
      }
      ++steps;
      if (phys < phys_min) phys_min = phys;
      if (phys > phys_max) phys_max = phys;
    } else if (now_us - pul_rise_us < min_high_us) {
      min_high_us = now_us - pul_rise_us;
    }
  }

  int digital_read(uint8_t pin) {
    if (pin == 18) return opto_active() ? LOW : HIGH;
    if (pin == 20) return i2c_bus_stuck ? LOW : level[pin];  // SDA удерживается
    return level[pin];
  }

  int analog_read(uint8_t pin) {
    if (pin == A0) {
      if (!pot_present) return 1023;   // обрыв движка — вход прижат к 5 В
      const double angle = ((double)phys + pot_bias_steps) / phys_per_degree;
      double raw = pot_center_raw + pot_counts_per_degree * angle;
      if (raw < 0.0) raw = 0.0;
      if (raw > 1023.0) raw = 1023.0;
      return (int)(raw + 0.5);
    }
    return analog[pin];
  }
};

inline std::vector<Board*> boards;

inline void tick_all() {
  for (Board* board : boards) {
    if (board->isr_fn) board->isr_fn();
  }
}

inline void advance(int64_t us) {
  const int64_t target = now_us + us;
  while (next_tick_us <= target) {
    now_us = next_tick_us;
    next_tick_us += 50;
    tick_all();
  }
  now_us = target;
}

// loop() каждой платы зовётся раз за тик, но в разной фазе: на чётных тиках
// сразу после прерывания, на нечётных — за 1 мкс до следующего. Так ловятся
// гонки вроде «DIR сменили, а фронт PUL пришёл раньше чем через 5 мкс».
inline int64_t loop_round = 0;

inline void run_for_ms(double ms) {
  const int64_t end = now_us + (int64_t)(ms * 1000.0);
  while (now_us < end) {
    now_us = next_tick_us;
    next_tick_us += 50;
    tick_all();
    if (++loop_round & 1) now_us = next_tick_us - 1;
    for (Board* board : boards) {
      if (!board->loop_fn) continue;
      const int64_t started = now_us;
      board->loop_fn();
      const int64_t spent = now_us - started;
      if (spent > board->max_loop_us) board->max_loop_us = spent;
    }
  }
}

}  // namespace sim

inline unsigned long millis() { return (unsigned long)(sim::now_us / 1000); }
inline unsigned long micros() { return (unsigned long)sim::now_us; }
inline void delayMicroseconds(unsigned int us) { sim::advance(us); }
inline void delay(unsigned long ms) { sim::advance((int64_t)ms * 1000); }
inline void noInterrupts() {}
inline void interrupts() {}
