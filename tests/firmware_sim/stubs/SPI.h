// Заглушка SPI.h для хост-симуляции: каждый байт уходит в модель платы, а выбор
// устройства делает сама прошивка через digitalWrite(SS). Модель RC522 смотрит
// на уровень SS и собирает из байтов обращения к регистрам.
// Include guard намеренно нет: у каждой платы свой объект SPI.
#include "../arduino_sim.h"

#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV16 0x01
#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define MSBFIRST 1

class SPIClass {
 public:
  void begin() {}
  void end() {}
  void setClockDivider(uint8_t) {}
  void setBitOrder(uint8_t) {}
  void setDataMode(uint8_t) {}
  void beginTransaction() {}
  void endTransaction() {}
  uint8_t transfer(uint8_t out) { return sim_board.spi_transfer(out); }
};

static SPIClass SPI;
