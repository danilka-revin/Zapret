// Заглушка watchdog AVR: в симуляции сброса по таймеру нет, но сценарий видит,
// что прошивка его включает и регулярно кормит (wdt_reset).
#pragma once
#include "../../arduino_sim.h"

#define WDTO_15MS 0
#define WDTO_250MS 3
#define WDTO_1S 6
#define WDTO_2S 7
#define WDTO_4S 8

// Состояние лежит в namespace sim (arduino_sim.h): скетч включается в свой
// namespace, а сценарий должен видеть те же счётчики, что и прошивка.

inline void wdt_enable(uint8_t) { sim::wdt_enabled = 1; }
inline void wdt_disable() { sim::wdt_enabled = 0; }
inline void wdt_reset() { ++sim::wdt_kicks; }
