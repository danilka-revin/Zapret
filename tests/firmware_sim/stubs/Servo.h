// Заглушка библиотеки Servo для хост-симуляции прошивки колеса.
//
// На железе Servo выдаёт импульс 50 Гц на любом цифровом выводе Mega (таймер
// и прерывание внутри библиотеки). Симуляции важен только результат: какой
// импульс в микросекундах сейчас стоит на выводе D3. Его и запоминает плата
// (sim::Board::servo_us), а проверяют сценарии в sim.cpp.
//
// Охранной обёртки (#pragma once) нет намеренно: скетч компилируется четыре раза,
// каждый в своём namespace (wheel_fl, wheel_fr, …), и класс Servo должен быть
// объявлен в каждом из них — как и board_prelude.h.

class Servo {
 public:
  // Servo::attach(pin) — вывод импульса. Минимальная/максимальная длительность
  // (вторая форма) прошивке не нужна: она пишет микросекунды явно.
  uint8_t attach(int pin) {
    sim_board.servo_pin = (uint8_t)pin;
    return 1;
  }
  void attach(int pin, int, int) { attach(pin); }
  void detach() { sim_board.servo_pin = 0xFF; }
  bool attached() const { return sim_board.servo_pin != 0xFF; }
  void writeMicroseconds(int us) { sim_board.set_servo_us(us); }
  void write(int value) { sim_board.set_servo_us(544 + (int)value * 8); }
  int readMicroseconds() const { return (int)sim_board.servo_us; }
  int read() const { return (int)(sim_board.servo_us - 544) / 8; }
};
