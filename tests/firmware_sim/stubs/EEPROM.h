// Заглушка EEPROM.h (обёртка Arduino над avr/eeprom.h): тот же байтовый массив
// платы, что и у eeprom_read_byte, поэтому обе записи видят одни данные.
// Include guard намеренно нет: у каждой платы своя EEPROM.
#include "../arduino_sim.h"

class EEPROMClass {
 public:
  uint8_t read(int address) { return sim_board.eeprom_read((uint16_t)address); }
  void write(int address, uint8_t value) { sim_board.eeprom_write((uint16_t)address, value); }
  void update(int address, uint8_t value) {
    if (read(address) != value) write(address, value);
  }
  int length() { return 4096; }
};

static EEPROMClass EEPROM;
