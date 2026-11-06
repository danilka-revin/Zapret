// Заглушка Wire.h для хост-симуляции прошивок (tests/firmware_sim).
// Библиотека ничего не знает про устройства: каждый обмен уходит в модель платы
// (sim_board.i2c_fn), которую настраивает сценарий. Без include guard намеренно:
// класс объявляется заново в каждом namespace со своей платой.
#include "../arduino_sim.h"

class TwoWire {
 public:
  void begin() {}
  void begin(int) {}
  void setClock(uint32_t) {}
  // На старых ядрах Arduino AVR этих вызовов нет — прошивка их и не требует.
  void setWireTimeout(uint32_t = 25000, bool = false) {}
  bool getWireTimeoutFlag() { return false; }
  void clearWireTimeoutFlag() {}

  void beginTransmission(uint8_t address) {
    address_ = address;
    tx_length_ = 0;
  }

  size_t write(uint8_t value) {
    if (tx_length_ < sizeof(tx_)) tx_[tx_length_++] = value;
    return 1;
  }

  uint8_t endTransmission(uint8_t = 1) {
    rx_length_ = 0;
    rx_position_ = 0;
    ok_ = sim_board.i2c_transfer(address_, tx_, tx_length_, nullptr, 0);
    return ok_ ? 0 : 2;   // 0 — ACK, 2 — нет ответа (NACK по адресу)
  }

  uint8_t requestFrom(uint8_t address, uint8_t length) {
    if (length > sizeof(rx_)) length = sizeof(rx_);
    ok_ = sim_board.i2c_transfer(address_, tx_, 0, rx_, length);
    rx_length_ = ok_ ? length : 0;
    rx_position_ = 0;
    return rx_length_;
  }

  int available() { return (int)(rx_length_ - rx_position_); }

  int read() {
    if (rx_position_ >= rx_length_) return -1;
    return rx_[rx_position_++];
  }

 private:
  uint8_t address_ = 0;
  uint8_t tx_[8] = {0};
  uint8_t tx_length_ = 0;
  uint8_t rx_[32] = {0};
  uint8_t rx_length_ = 0;
  uint8_t rx_position_ = 0;
  bool ok_ = false;
};

static TwoWire Wire;
