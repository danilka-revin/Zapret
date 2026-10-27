// Заглушка библиотеки Servo для хост-симуляции прошивки колеса.
//
// Настоящая библиотека держит 50 Гц аппаратно (на Mega — Timer5) и выдаёт
// импульс 1000…2000 мкс. В симуляции важен не таймер, а ЧИСЛО, которое
// прошивка отправляет на выход скорости: его и запоминаем в плате
// (Board::servo_us), чтобы проверки читали реальный импульс, а не догадывались
// по проценту.
//
// Подключается после board_prelude.h (внутри namespace платы), поэтому
// sim_board здесь уже виден. Include guard намеренно НЕТ: как и board_prelude.h,
// заголовок включается по разу в каждом namespace платы, и класс Servo должен
// быть свой в каждом — иначе четыре колеса делили бы один объект.

#include <stdint.h>

class Servo {
 public:
  // attach(pin, min, max) — как в настоящей библиотеке: края импульса в мкс.
  int8_t attach(int pin, int min = 1000, int max = 2000) {
    sim_board.servo_attached = true;
    sim_board.servo_pin = (uint8_t)pin;
    sim_board.servo_min_us = (int16_t)min;
    sim_board.servo_max_us = (int16_t)max;
    sim_board.servo_us = (uint16_t)((min + max) / 2);
    return 1;
  }
  void detach() { sim_board.servo_attached = false; }
  // writeMicroseconds — основной вызов прошивки; выход за края обрезается,
  // как в библиотеке.
  void writeMicroseconds(int us) {
    if (us < sim_board.servo_min_us) us = sim_board.servo_min_us;
    if (us > sim_board.servo_max_us) us = sim_board.servo_max_us;
    sim_board.servo_us = (uint16_t)us;
  }
  int readMicroseconds() { return sim_board.servo_us; }
  // write(угол) — для совместимости: 0…180° ↔ min…max.
  void write(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    const long span = (long)sim_board.servo_max_us - (long)sim_board.servo_min_us;
    writeMicroseconds((int)((long)sim_board.servo_min_us + span * angle / 180));
  }
  bool attached() { return sim_board.servo_attached; }
};
