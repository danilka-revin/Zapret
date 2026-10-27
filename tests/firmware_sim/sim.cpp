// Хост-симуляция прошивки колеса motor_wheel: четыре независимые платы
// (FL/FR/RL/RR), каждая со своим USB и прошивкой СВОЕГО колеса; связь — только UART.
//
// Сборка и запуск (это делает tests/test_firmware.py):
//   g++ -std=c++17 -I tests/firmware_sim/stubs tests/firmware_sim/sim.cpp -o sim
//   ./sim            — все сценарии (каждый в своём процессе, как после сброса)
//   ./sim moves      — один сценарий
//   ./sim tlm_dump   — вывести @TLM всех четырёх колёс (для разбора сервером)
//   ./sim pty /tmp/wheel — четыре колеса за псевдотерминалами /tmp/wheel-FL…
#include "arduino_sim.h"

#include <chrono>
#include <fcntl.h>
#include <functional>
#include <sys/wait.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

sim::Board board_fl, board_fr, board_rl, board_rr;

namespace wheel_fl {
#define SIM_BOARD board_fl
#define MODULE_ID "FL"
#include "board_prelude.h"
#include "../../firmware/motor_wheel/motor_wheel.ino"
#undef SIM_BOARD
}  // namespace wheel_fl
#undef MODULE_ID
#undef TX_BUFFER_SIZE

namespace wheel_fr {
#define SIM_BOARD board_fr
#define MODULE_ID "FR"
#include "board_prelude.h"
#include "../../firmware/motor_wheel/motor_wheel.ino"
#undef SIM_BOARD
}  // namespace wheel_fr
#undef MODULE_ID
#undef TX_BUFFER_SIZE

namespace wheel_rl {
#define SIM_BOARD board_rl
#define MODULE_ID "RL"
#include "board_prelude.h"
#include "../../firmware/motor_wheel/motor_wheel.ino"
#undef SIM_BOARD
}  // namespace wheel_rl
#undef MODULE_ID
#undef TX_BUFFER_SIZE

namespace wheel_rr {
#define SIM_BOARD board_rr
#define MODULE_ID "RR"
#include "board_prelude.h"
#include "../../firmware/motor_wheel/motor_wheel.ino"
#undef SIM_BOARD
}  // namespace wheel_rr
#undef MODULE_ID
#undef TX_BUFFER_SIZE

static int failures = 0;

#define CHECK(cond, ...)                                                   \
  do {                                                                     \
    if (!(cond)) {                                                         \
      ++failures;                                                          \
      std::fprintf(stderr, "  FAIL %s:%d: %s — ", __FILE__, __LINE__, #cond); \
      std::fprintf(stderr, __VA_ARGS__);                                   \
      std::fprintf(stderr, "\n");                                          \
    }                                                                      \
  } while (0)

static void bind_boards() {
  board_fl.name = "FL";
  board_fl.setup_fn = wheel_fl::setup;
  board_fl.loop_fn = wheel_fl::loop;
  board_fl.isr_fn = wheel_fl::TIMER1_COMPA_vect_handler;
  board_fr.name = "FR";
  board_fr.setup_fn = wheel_fr::setup;
  board_fr.loop_fn = wheel_fr::loop;
  board_fr.isr_fn = wheel_fr::TIMER1_COMPA_vect_handler;
  board_rl.name = "RL";
  board_rl.setup_fn = wheel_rl::setup;
  board_rl.loop_fn = wheel_rl::loop;
  board_rl.isr_fn = wheel_rl::TIMER1_COMPA_vect_handler;
  board_rr.name = "RR";
  board_rr.setup_fn = wheel_rr::setup;
  board_rr.loop_fn = wheel_rr::loop;
  board_rr.isr_fn = wheel_rr::TIMER1_COMPA_vect_handler;
}

static void boot(std::vector<sim::Board*> list) {
  sim::boards = list;
  // слейвы раньше мастера: шина готова к первому опросу
  for (auto it = list.rbegin(); it != list.rend(); ++it) {
    (*it)->reset_stats();
    (*it)->setup_fn();
  }
  sim::run_for_ms(5);
}

static bool wait_until(const std::function<bool()>& predicate, double timeout_ms) {
  const int64_t end = sim::now_us + (int64_t)(timeout_ms * 1000.0);
  while (sim::now_us < end) {
    if (predicate()) return true;
    sim::run_for_ms(1);
  }
  return predicate();
}

static bool has(const sim::Board& board, const char* text) {
  return board.serial.output.find(text) != std::string::npos;
}

static long count(const sim::Board& board, const char* text) {
  long n = 0;
  size_t pos = 0;
  while ((pos = board.serial.output.find(text, pos)) != std::string::npos) {
    ++n;
    pos += std::strlen(text);
  }
  return n;
}

// То же, но только в хвосте вывода (после отметки) — «сколько раз с момента…».
static long count_since(const sim::Board& board, const char* text, size_t from) {
  long n = 0;
  size_t pos = from;
  while ((pos = board.serial.output.find(text, pos)) != std::string::npos) {
    ++n;
    pos += std::strlen(text);
  }
  return n;
}

static void check_signals(const sim::Board& board) {
  CHECK(board.dir_setup_violations == 0, "%s: DIR сменился позже чем за 5 мкс до PUL (%ld)",
        board.name, board.dir_setup_violations);
  CHECK(board.steps_while_disabled == 0, "%s: шаги при ENA != HIGH (%ld)", board.name,
        board.steps_while_disabled);
  CHECK(board.min_high_us >= 50, "%s: импульс PUL короче 50 мкс (%lld)", board.name,
        (long long)board.min_high_us);
}

// ---------------------------------------------------------------- один модуль
#define FR wheel_fr

static bool fr_idle() { return FR::calib_state == FR::CAL_IDLE && !FR::motorBusy(); }

static void calibrate_fr(double timeout_ms = 30000) {
  board_fr.serial.send("k");
  sim::run_for_ms(2);
  CHECK(FR::calib_state != FR::CAL_IDLE || FR::calib_error != FR::CAL_OK || FR::zero_set,
        "калибровка не началась");
  CHECK(wait_until(fr_idle, timeout_ms), "калибровка не закончилась за %.0f мс", timeout_ms);
}

static bool fr_settled() {
  return FR::calib_state == FR::CAL_IDLE && FR::pot_cal_state == FR::POT_CAL_IDLE &&
         !FR::motorBusy() && !FR::feedback_pending;
}

static bool fr_pot_idle() {
  return FR::pot_cal_state == FR::POT_CAL_IDLE && !FR::motorBusy();
}

static void calibrate_pot_fr() {
  board_fr.serial.send("q");
  sim::run_for_ms(2);
  CHECK(FR::pot_cal_state != FR::POT_CAL_IDLE, "q не запустила калибровку потенциометра");
  CHECK(wait_until(fr_pot_idle, 30000), "калибровка потенциометра не закончилась");
  CHECK(FR::pot_calibrated && FR::pot_zero_valid, "потенциометр не откалиброван: cal=%d zero=%d",
        FR::pot_calibrated, FR::pot_zero_valid);
}

static void scenario_boot() {
  boot({&board_fr});
  CHECK(board_fr.level[8] == HIGH, "ENA должен быть HIGH сразу после включения");
  CHECK(board_fr.level[6] == LOW, "PUL в покое LOW");
  CHECK(board_fr.mode[18] == INPUT_PULLUP, "D18 — INPUT_PULLUP");
  CHECK(!FR::zero_set, "ноль после включения не задан");
  CHECK(has(board_fr, "--- КОЛЕСО FR:"), "приветствие колеса: %s", board_fr.serial.output.c_str());
  CHECK(has(board_fr, "потенциометр A0: 512"), "потенциометр A0 прочитан: %s",
        board_fr.serial.output.c_str());
  CHECK(board_fr.level[5] == HIGH && board_fr.analog[3] == 0, "BLDC: тормоз включён, PWM 0");
}

static void scenario_calib_left() {
  board_fr.phys = -4000;                 // колесо на −30°, флажок в [−200, 200]
  boot({&board_fr});
  const int64_t started = sim::now_us;
  calibrate_fr();
  const double seconds = (sim::now_us - started) / 1e6;
  CHECK(FR::zero_set && FR::home_source == FR::HOME_OPTO && FR::calib_error == FR::CAL_OK,
        "ноль по датчику: zero=%d home=%d err=%d", FR::zero_set, FR::home_source, FR::calib_error);
  CHECK(std::labs(board_fr.phys) <= 1, "колесо в середине флажка, phys=%ld", board_fr.phys);
  CHECK(FR::stepNow() == 0, "счётчик в нуле: %ld", (long)FR::stepNow());
  CHECK(has(board_fr, "Калибровка ЗАВЕРШЕНА!"), "нет сообщения о завершении");
  CHECK(has(board_fr, "ширина флажка 3.00°"), "ширина флажка 400 шагов = 3.00°: %s",
        board_fr.serial.output.c_str());
  CHECK(seconds < 60.0, "калибровка шла %.1f с", seconds);
  CHECK(board_fr.min_period_us == 100, "рабочий шаг 100 мкс (10 кГц), а min период %lld",
        (long long)board_fr.min_period_us);
  CHECK(!has(board_fr, "ищу влево"), "датчик слева от колеса находится поиском вправо");
  check_signals(board_fr);
}

static void scenario_calib_right() {
  board_fr.phys = 3000;                  // флажок левее: вправо 135° пусто, потом влево
  boot({&board_fr});
  calibrate_fr();
  CHECK(has(board_fr, "справа датчика нет, ищу влево на 250°"), "переход к поиску влево");
  CHECK(has(board_fr, "датчик найден слева"), "датчик найден поиском влево");
  CHECK(FR::zero_set && std::labs(board_fr.phys) <= 1, "ноль найден: phys=%ld", board_fr.phys);
  CHECK(std::labs(board_fr.phys_max - (3000 + 16667)) <= 2, "сначала прошёл вправо 125°: max=%ld", board_fr.phys_max);
  check_signals(board_fr);
}

static void scenario_calib_inside() {
  board_fr.phys = 50;                    // старт внутри флажка
  boot({&board_fr});
  calibrate_fr();
  CHECK(has(board_fr, "датчик уже закрыт"), "старт с закрытым датчиком");
  CHECK(FR::zero_set && std::labs(board_fr.phys) <= 1, "ноль найден: phys=%ld", board_fr.phys);
  check_signals(board_fr);
}

static void scenario_calib_far_left() {
  board_fr.phys = -13000;                // −97.5°: ещё в зоне поиска вправо
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::zero_set && std::labs(board_fr.phys) <= 1, "ноль найден: phys=%ld", board_fr.phys);
}

static void scenario_calib_out_of_range() {
  board_fr.phys = -19000;                // −142.5°: флажок за пределом ±135° от старта
  boot({&board_fr});
  calibrate_fr();
  CHECK(!FR::zero_set && FR::calib_error == FR::CAL_ERR_NOT_FOUND, "ошибка «не найден»: err=%d",
        FR::calib_error);
  CHECK(has(board_fr, "ОШИБКА калибровки: датчик не найден"), "текст ошибки");
  CHECK(std::labs(board_fr.phys - (-19000 - 16667)) <= 2, "вправо 125° и влево 250°: phys=%ld",
        board_fr.phys);
  CHECK(!FR::motorBusy(), "мотор остановлен");
}

static void scenario_calib_no_sensor() {
  board_fr.opto_present = false;
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::calib_error == FR::CAL_ERR_NOT_FOUND && !FR::zero_set, "без датчика — ошибка");
  CHECK(std::labs(board_fr.phys_max - 16667) <= 2 && std::labs(board_fr.phys_min + 16667) <= 2,
        "ход поиска +125°/−250° (±125° от старта): min=%ld max=%ld", board_fr.phys_min,
        board_fr.phys_max);
  sim::run_for_ms(200);
  CHECK(std::labs(board_fr.phys + 16667) <= 2 && !FR::motorBusy(), "после ошибки колесо стоит");
}

static void scenario_calib_stuck() {
  board_fr.opto_stuck = true;
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::calib_error == FR::CAL_ERR_STUCK && !FR::zero_set, "залипший датчик: err=%d",
        FR::calib_error);
  CHECK(std::labs(board_fr.phys - 4000) <= 1, "выход из флажка ограничен 30°: phys=%ld", board_fr.phys);
}

static void scenario_calib_abort() {
  board_fr.phys = -8000;
  boot({&board_fr});
  board_fr.serial.send("k");
  sim::run_for_ms(30);
  board_fr.serial.send("s");
  sim::run_for_ms(5);
  const long stopped_at = board_fr.phys;
  CHECK(FR::calib_state == FR::CAL_IDLE && FR::calib_error == FR::CAL_ERR_ABORTED,
        "s отменяет калибровку");
  CHECK(has(board_fr, "прервана командой s") && has(board_fr, "ПАУЗА."), "сообщения об отмене");
  sim::run_for_ms(300);
  CHECK(board_fr.phys == stopped_at, "после s колесо не едет: %ld → %ld", stopped_at, board_fr.phys);
  board_fr.serial.send("g");
  sim::run_for_ms(50);
  CHECK(board_fr.phys == stopped_at, "g не продолжает отменённую калибровку");
  board_fr.serial.send("k");
  sim::run_for_ms(2);
  CHECK(FR::calib_state != FR::CAL_IDLE, "после отмены калибровку можно запустить снова");
  CHECK(wait_until(fr_idle, 30000) && FR::zero_set, "повторная калибровка прошла");
}

static void scenario_calib_asymmetric() {
  board_fr.flag_lo = 100;                // флажок [0.75°, 6.75°], середина 500 шагов
  board_fr.flag_hi = 900;
  board_fr.hysteresis = 30;
  board_fr.phys = 6000;
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::zero_set && std::labs(board_fr.phys - 500) <= 1,
        "ноль — середина флажка (гистерезис не сдвигает): phys=%ld", board_fr.phys);
}

// Флажок на краю диапазона, поиск с обеих сторон. Ход руля не выходит за ±135°.
static void scenario_calib_full_range_plus() {
  board_fr.flag_lo = 15800;              // флажок при +118.5°, найдётся быстрым ходом вправо
  board_fr.flag_hi = 16200;
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::zero_set && has(board_fr, "датчик найден справа"), "найден справа");
  CHECK(std::labs(board_fr.phys - 16000) <= 1, "ноль в середине флажка: phys=%ld", board_fr.phys);
  CHECK(board_fr.phys_max <= 18000 + 2 && board_fr.phys_min >= -18000 - 2,
        "ход не вышел за ±135°: min=%ld max=%ld", board_fr.phys_min, board_fr.phys_max);
  check_signals(board_fr);
}

static void scenario_calib_full_range_minus() {
  board_fr.flag_lo = -16200;             // флажок при −121.5°: вправо 135° пусто, ищем влево 270°
  board_fr.flag_hi = -15800;
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::zero_set && has(board_fr, "датчик найден слева"), "найден слева");
  CHECK(std::labs(board_fr.phys - (-16000)) <= 1, "ноль в середине флажка: phys=%ld", board_fr.phys);
  CHECK(board_fr.phys_max <= 18000 + 2 && board_fr.phys_min >= -18000 - 2,
        "ход не вышел за ±135°: min=%ld max=%ld", board_fr.phys_min, board_fr.phys_max);
  check_signals(board_fr);
}

// Срабатывание датчика сдвигается на +60, 0, −60 шагов (до 0.45°) от подхода к подходу.
// Один подход дал бы ноль со сдвигом на 30 шагов; среднее по трём подходам каждой
// кромки сдвиги гасит: ноль попадает в середину флажка.
static void scenario_calib_passes_averaged() {
  board_fr.flag_shift_script = {60, 0, -60};
  board_fr.phys = 0;
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::zero_set && FR::calib_error == FR::CAL_OK, "калибровка прошла: err=%d", FR::calib_error);
  CHECK(count(board_fr, "кромка справа найдена") == 3 && count(board_fr, "кромка слева найдена") == 3,
        "по три подхода на каждую кромку: справа %ld, слева %ld",
        count(board_fr, "кромка справа найдена"), count(board_fr, "кромка слева найдена"));
  CHECK(std::labs(board_fr.phys) <= 2, "среднее гасит сдвиги подходов: phys=%ld", board_fr.phys);
  check_signals(board_fr);
}

// Кромки с разных подходов расходятся на 300 шагов (2.25°) — больше допуска 2°.
// Ноль не ставим: ошибка 6.
static void scenario_calib_spread_error() {
  board_fr.flag_shift_script = {150, 0, -150};
  board_fr.phys = 0;
  boot({&board_fr});
  calibrate_fr();
  CHECK(!FR::zero_set && FR::calib_error == FR::CAL_ERR_SPREAD, "ошибка разброса: err=%d zero=%d",
        FR::calib_error, FR::zero_set);
  CHECK(has(board_fr, "кромки расходятся между подходами"), "текст ошибки разброса");
  CHECK(!FR::motorBusy(), "мотор остановлен");
}

static void scenario_moves() {
  board_fr.phys = -2500;
  boot({&board_fr});
  calibrate_fr();
  board_fr.reset_stats();

  board_fr.serial.send("w 12.5");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 1667; }, 1000),
        "w 12.5 → 1667 шагов, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == 1667, "колесо на 12.5°: phys=%ld", board_fr.phys);

  // Предел абсолютной команды — ±100° (STEER_LIMIT_DEG): боковой ход (90°) и
  // запас до 100° колесу доступны, угол вне предела его не двигает, возврат в
  // рабочее положение — тоже.
  board_fr.serial.send("w 90");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 12000; }, 3000),
        "w 90 → 12000 шагов, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == 12000, "колесо на 90°: phys=%ld", board_fr.phys);

  // 100° — новый потолок: колесо обязано доехать, а не быть отклонённым.
  board_fr.serial.send("w 100");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 13333; }, 3000),
        "w 100 → 13333 шага, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == 13333, "колесо на 100°: phys=%ld", board_fr.phys);

  board_fr.serial.send("w 101");
  sim::run_for_ms(50);
  CHECK(board_fr.phys == 13333 && has(board_fr, "угол вне ±100°"), "w 101 отклонена");

  board_fr.serial.send("w -100");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == -13333; }, 5000),
        "w -100 → -13333 шага, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == -13333, "колесо на −100°: phys=%ld", board_fr.phys);

  board_fr.serial.send("w 12.5");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 1667; }, 5000),
        "возврат на 12.5° после ±100°, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == 1667, "колесо снова на 12.5°: phys=%ld", board_fr.phys);

  const int64_t started = sim::now_us;
  board_fr.serial.send("d 10 f");
  sim::run_for_ms(2);
  CHECK(wait_until([] { return !FR::motorBusy(); }, 1000), "d 10 f закончилась");
  const double ms = (sim::now_us - started) / 1000.0;
  CHECK(board_fr.phys == 3000, "d 10 f → +1333 шага: phys=%ld", board_fr.phys);
  CHECK(ms > 130.0 && ms < 140.0, "1333 шага по 100 мкс ≈ 133 мс, а заняло %.1f мс", ms);
  CHECK(has(board_fr, "ВПЕРЕД (F) на 10.00°") && has(board_fr, "Цель достигнута. Угол: 22.50°"),
        "сообщения как в проверенном скетче: %s", board_fr.serial.output.c_str());

  board_fr.serial.send("d 5 b");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 2333; }, 1000),
        "d 5 b → 2333, сейчас %ld", (long)FR::stepNow());
  board_fr.serial.send("d 10");
  board_fr.serial.send("d -5 f");
  board_fr.serial.send("d 5 x");
  sim::run_for_ms(50);
  CHECK(has(board_fr, "Шаблон: d [градусы] [f/b]"), "d без направления");
  CHECK(has(board_fr, "Ошибка: Угол > 0!"), "отрицательный угол, как в скетче");
  CHECK(has(board_fr, "Ошибка направления! f или b"), "неверное направление");
  CHECK(board_fr.phys == 2333, "ошибочные команды не двигают колесо");

  board_fr.serial.send("d 200 f");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "вне хода ±125°"), "предел хода отклоняет d 200: %s",
        board_fr.serial.output.c_str());
  CHECK(board_fr.phys == 2333, "предел хода не двигает колесо");

  board_fr.serial.send("h");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 0; }, 1000), "h → 0");
  CHECK(board_fr.phys == 0, "колесо прямо: phys=%ld", board_fr.phys);
  board_fr.serial.send("p");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "Текущий: 0.00° | Целевой: 0.00° [СТОИТ] · ноль по датчику"), "p: %s",
        board_fr.serial.output.c_str());
  check_signals(board_fr);
}

static void scenario_pot_calibration() {
  board_fr.phys = 2500;                  // колесо не в нуле: q сначала едет в 0
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::pot_zero_valid && !FR::pot_calibrated,
        "после k ноль потенциометра есть, наклон ещё нет");
  CHECK(has(board_fr, "потенциометр A0: 0° = 512"), "ноль потенциометра записан: %s",
        board_fr.serial.output.c_str());
  calibrate_pot_fr();
  CHECK(std::labs(board_fr.phys) <= 1, "q вернула колесо в 0: phys=%ld", board_fr.phys);
  CHECK(std::fabs(FR::pot_counts_per_degree - 1023.0 / 270.0) < 0.05,
        "наклон %.4f отсч/°", FR::pot_counts_per_degree);
  CHECK(FR::moduleStatus() & FR::STATUS_POT, "бит STATUS_POT в кадре состояния");
  CHECK(has(board_fr, "Потенциометр: 3.78 отсч/°, 0° = 512"),
        "наклон и ноль напечатаны: %s", board_fr.serial.output.c_str());
  CHECK(has(board_fr, "калибровка завершена, доводка включена"), "сообщение о завершении q");
  board_fr.serial.send("p");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "Потенциометр A0: 512 = 0.00° (наклон измерен)"), "p: %s",
        board_fr.serial.output.c_str());
}

static void scenario_pot_feedback() {
  boot({&board_fr});
  calibrate_fr();
  calibrate_pot_fr();
  board_fr.pot_bias_steps = 600;         // датчик показывает на 4.5° больше колеса
  board_fr.serial.send("w 10");
  sim::run_for_ms(2);
  CHECK(wait_until(fr_settled, 3000), "w 10 с доводкой не закончилась");
  CHECK(std::fabs(FR::potAngleDeg() - 10.0f) < 0.8f,
        "доводка вывела датчик на 10°: %.2f (phys=%ld)", FR::potAngleDeg(), board_fr.phys);
  CHECK(has(board_fr, "Потенциометр: доводка"), "сообщение о доводке: %s",
        board_fr.serial.output.c_str());
  CHECK(has(board_fr, "Потенциометр: остаток"), "сообщение об остатке");
  CHECK(std::labs(FR::stepNow() - 1333) <= 10, "счётчик привязан к датчику: %ld",
        (long)FR::stepNow());
  board_fr.serial.send("w 0");
  sim::run_for_ms(2);
  CHECK(wait_until(fr_settled, 3000), "w 0 с доводкой не закончилась");
  CHECK(std::fabs(FR::potAngleDeg()) < 0.8f, "доводка вернула на 0: %.2f", FR::potAngleDeg());
  CHECK(std::labs(FR::stepNow()) <= 10, "счётчик снова в нуле: %ld", (long)FR::stepNow());
}

static void scenario_pot_mode() {
  boot({&board_fr});
  calibrate_fr();
  calibrate_pot_fr();
  board_fr.serial.send("pm 0");
  sim::run_for_ms(5);
  CHECK(!FR::pot_enabled && !FR::potFeedbackAllowed(), "pm 0 disables feedback");
  CHECK((FR::moduleStatus() & 0x3000) == 0x2000, "mode off reported");
  board_fr.pot_bias_steps = 6000;
  const int16_t raw = FR::pot_raw;
  board_fr.serial.send("w 10");
  sim::run_for_ms(5);
  board_fr.serial.send("pm 1");
  sim::run_for_ms(5);
  CHECK(!FR::pot_enabled, "cannot switch during motion");
  CHECK(wait_until(fr_settled, 3000), "open loop move completes");
  CHECK(FR::stepNow() == FR::degreesToSteps(10), "open loop target reached");
  CHECK(FR::pot_raw == raw, "disabled ADC ignored");
  board_fr.serial.send("q");
  sim::run_for_ms(5);
  CHECK(FR::pot_cal_state == FR::POT_CAL_IDLE, "q rejected when disabled");
  board_fr.serial.send("pm 2");
  sim::run_for_ms(5);
  CHECK(!FR::pot_enabled, "invalid mode rejected");
  board_fr.serial.send("pm 1");
  sim::run_for_ms(5);
  CHECK(FR::pot_enabled && !FR::pot_calibrated, "enable requires fresh calibration");
  CHECK((FR::moduleStatus() & 0x3000) == 0x3000, "mode on reported");
}

static void scenario_pot_unplugged() {
  board_fr.pot_present = false;
  boot({&board_fr});
  CHECK(has(board_fr, "потенциометр A0: 1023 — на краю"), "boot без датчика: %s",
        board_fr.serial.output.c_str());
  calibrate_fr();
  CHECK(FR::zero_set && !FR::pot_zero_valid && !FR::pot_calibrated,
        "без потенциометра ноль руля есть, доводки нет");
  board_fr.serial.send("q");
  sim::run_for_ms(5);
  CHECK(FR::pot_cal_state == FR::POT_CAL_IDLE, "q без датчика отклонена");
  CHECK(has(board_fr, "на краю, проверьте провод"), "сообщение об обрыве");
  board_fr.serial.send("w 10");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 1333; }, 1000),
        "повороты без потенциометра работают, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == 1333, "phys=%ld", board_fr.phys);
}

static void scenario_swing() {
  boot({&board_fr});
  board_fr.serial.send("z");
  sim::run_for_ms(2);
  board_fr.reset_stats();
  board_fr.serial.send("c 20");
  sim::run_for_ms(1500);
  CHECK(board_fr.phys_min == 0 && board_fr.phys_max == 2667, "качание 0…20°: %ld…%ld",
        board_fr.phys_min, board_fr.phys_max);
  CHECK(FR::cycle_active, "цикл идёт");
  board_fr.serial.send("c c");
  sim::run_for_ms(5);
  const long at = board_fr.phys;
  sim::run_for_ms(300);
  CHECK(!FR::cycle_active && board_fr.phys == at, "c c останавливает качание");
  CHECK(has(board_fr, "ЦИКЛ качания на 20.00°") && has(board_fr, "ЦИКЛ ВЫКЛ. Угол:"), "сообщения цикла");
  board_fr.serial.send("c c");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "Цикл и так выключен."), "повторное c c");
}

static void scenario_pause_resume() {
  boot({&board_fr});
  board_fr.serial.send("z");
  board_fr.serial.send("d 90 f");        // 12000 шагов ≈ 1,2 с
  sim::run_for_ms(300);
  board_fr.serial.send("s");
  sim::run_for_ms(5);
  const long paused = board_fr.phys;
  CHECK(paused > 2000 && paused < 4000, "стоп на ходу: phys=%ld", paused);
  CHECK(FR::stepNow() == paused, "счётчик совпадает с колесом после s: %ld vs %ld",
        (long)FR::stepNow(), paused);
  board_fr.serial.send("g");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 12000; }, 2000),
        "g довёл до цели, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == 12000 && has(board_fr, "ПРОДОЛЖЕНИЕ..."), "phys=%ld", board_fr.phys);
  board_fr.serial.send("g");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "Уже в цели."), "g без прерванного хода");
}

static void scenario_zero_manual() {
  board_fr.phys = 700;
  boot({&board_fr});
  board_fr.serial.send("w 5");
  sim::run_for_ms(20);
  CHECK(has(board_fr, "w: ноль не задан"), "w без нуля отклонена");
  CHECK(board_fr.phys == 700, "без нуля абсолютная команда не двигает колесо");
  board_fr.serial.send("z");
  sim::run_for_ms(2);
  CHECK(FR::zero_set && FR::home_source == FR::HOME_MANUAL && FR::stepNow() == 0, "z — ручной ноль");
  CHECK((FR::moduleStatus() & FR::STATUS_HOME_OPTO) == 0, "ручной ноль не помечен как датчик");
  board_fr.serial.send("w -5");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == -667; }, 1000), "w -5 после z");
  CHECK(board_fr.phys == 700 - 667, "phys=%ld", board_fr.phys);
}

// Заводской выход — 0–5 В: t 8 при пределе 20 % = 8/20 полной шкалы = 2000 мВ,
// то есть 102/255 скважности (после RC-фильтра это 2 В на входе регулятора).
static const int DRIVE_DUTY_8 = 102;

static void scenario_drive() {
  boot({&board_fr});
  board_fr.serial.send("t 8");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 0 && board_fr.level[5] == HIGH, "без нуля тяга запрещена");
  board_fr.serial.send("z");
  board_fr.serial.send("t 8");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == DRIVE_DUTY_8 && board_fr.level[5] == LOW,
        "t 8 → 102/255 (2 В), тормоз снят");
  board_fr.serial.send("d 5 f");
  sim::run_for_ms(20);
  CHECK(board_fr.phys == 0 && has(board_fr, "руль заблокирован: включена тяга"), "при тяге руль не крутится");
  for (int i = 0; i < 5; ++i) {
    board_fr.serial.send("t 8");
    sim::run_for_ms(200);
  }
  CHECK(FR::drive_active && board_fr.analog[3] == DRIVE_DUTY_8,
        "heartbeat каждые 200 мс держит тягу");
  sim::run_for_ms(700);
  CHECK(!FR::drive_active && board_fr.analog[3] == 0 && board_fr.level[5] == HIGH,
        "watchdog 650 мс: PWM 0, тормоз");
  CHECK(has(board_fr, "нет команды дольше 650 мс"), "сообщение watchdog");
  board_fr.serial.send("t 8");
  board_fr.serial.send("t -8");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 0 && board_fr.level[5] == HIGH, "реверс без остановки — тормоз");
  board_fr.serial.send("t 25");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 0, "t 25 — за пределом, тормоз");
  board_fr.serial.send("t 8");
  sim::run_for_ms(5);
  board_fr.serial.send("s");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 0 && board_fr.level[5] == HIGH, "s выключает и тягу");
}

static void scenario_serial_garbage() {
  boot({&board_fl, &board_fr, &board_rl, &board_rr});
  for (const char* chunk : {"@TLM deg=0.00 tgt=0.00 mov", "\x07\xff#"}) {
    for (const char* p = chunk; *p; ++p) board_fl.serial.input.push_back((uint8_t)*p);
    sim::run_for_ms(1200);               // обрывок без \n висит дольше секунды
  }
  board_fl.serial.send("k");
  sim::run_for_ms(5);
  CHECK(wheel_fl::calib_state != wheel_fl::CAL_IDLE, "после мусора в порту команда k всё равно выполнена");
  board_fl.serial.send("@TLM deg=0.00 tgt=0.00 moving=0 cal=0");
  board_fl.serial.send("# FL: эхо сообщения");
  sim::run_for_ms(5);
  CHECK(!has(board_fl, "Неизв. команда: @TLM") && !has(board_fl, "Неизв. команда: #"),
        "эхо телеметрии и сообщений не считается командой");
  board_fl.serial.send("d 1");            // частичный ввод человека с паузами < 1 с
  sim::run_for_ms(5);
  CHECK(has(board_fl, "Шаблон: d [градусы] [f/b]"), "обычные ошибки по-прежнему видны");
}


// ---------------------------------------------------------------- вся система
#define M wheel_fl
static std::vector<sim::Board*> all_boards() { return {&board_fl, &board_fr, &board_rl, &board_rr}; }

static bool all_idle() {
  return M::calib_state == M::CAL_IDLE && !M::motorBusy() &&
         wheel_fr::calib_state == wheel_fr::CAL_IDLE && !wheel_fr::motorBusy() &&
         wheel_rl::calib_state == wheel_rl::CAL_IDLE && !wheel_rl::motorBusy() &&
         wheel_rr::calib_state == wheel_rr::CAL_IDLE && !wheel_rr::motorBusy();
}

static bool all_zero() { return M::zero_set && wheel_fr::zero_set && wheel_rl::zero_set && wheel_rr::zero_set; }

static bool all_pot_idle() {
  return M::pot_cal_state == M::POT_CAL_IDLE && wheel_fr::pot_cal_state == wheel_fr::POT_CAL_IDLE &&
         wheel_rl::pot_cal_state == wheel_rl::POT_CAL_IDLE && wheel_rr::pot_cal_state == wheel_rr::POT_CAL_IDLE &&
         !M::motorBusy() && !wheel_fr::motorBusy() && !wheel_rl::motorBusy() && !wheel_rr::motorBusy();
}

// Сервер шлёт «k» на каждый порт: каждое колесо калибруется само.
static void calibrate_all() {
  board_fl.phys = -3000;
  board_fr.phys = 2500;                  // справа от флажка: поиск влево
  board_rl.phys = 40;                    // внутри флажка
  board_rr.phys = -9000;
  board_rr.flag_lo = 300;                // у RR флажок смещён: ноль будет на 500 шагах
  board_rr.flag_hi = 700;
  boot(all_boards());
  sim::run_for_ms(300);
  for (sim::Board* board : all_boards()) board->serial.send("k");
  sim::run_for_ms(5);
  CHECK(wait_until(all_idle, 30000), "калибровка всех колёс закончилась");
  sim::run_for_ms(300);                  // телеметрия всех четырёх колёс
}

// Only telemetry, no USB-port/role discovery response.
static void scenario_identity() {
  boot(all_boards());
  for (sim::Board* board : all_boards()) board->serial.send("@ID");
  sim::run_for_ms(300);
  for (sim::Board* board : all_boards()) {
    CHECK(!has(*board, "@ID"), "no discovery answer");
    CHECK(has(*board, "@TLM deg="), "telemetry on selected port");
    CHECK(!has(*board, "mod="), "role is not reported by firmware");
  }
  board_fl.serial.send("w RR 90");
  sim::run_for_ms(200);
  CHECK(wheel_fl::stepNow() == 0, "legacy foreign command ignored");
}

static void scenario_system_calibrate_all() {
  calibrate_all();
  CHECK(all_zero(), "ноль задан у всех: FL=%d FR=%d RL=%d RR=%d", M::zero_set, wheel_fr::zero_set,
        wheel_rl::zero_set, wheel_rr::zero_set);
  CHECK(std::labs(board_fl.phys) <= 1 && std::labs(board_fr.phys) <= 1 && std::labs(board_rl.phys) <= 1 &&
        std::labs(board_rr.phys - 500) <= 1,
        "колёса на нулях датчиков: %ld %ld %ld %ld", board_fl.phys, board_fr.phys, board_rl.phys,
        board_rr.phys);
  for (sim::Board* board : all_boards()) {
    const std::string tag = std::string("@TLM deg=0.00 tgt=0.00 moving=0 cal=1 cycle=0 calib=0");
    CHECK(board->serial.output.find(tag) != std::string::npos,
          "телеметрия %s после калибровки: %s", board->name, board->serial.output.c_str());
    const std::string done = std::string("# ") + board->name + ": Калибровка ЗАВЕРШЕНА!";
    CHECK(board->serial.output.find(done) != std::string::npos,
          "%s сам сообщает о калибровке", board->name);
    CHECK(has(*board, "home=opto err=0") && has(*board, "fw=5"), "поля home/err/fw в @TLM %s",
          board->name);
    check_signals(*board);
  }
}

static void scenario_system_server_commands() {
  calibrate_all();
  // Адресная команда с именем чужого колеса не выполняется.
  board_fl.serial.send("w FR 12.5");
  sim::run_for_ms(300);
  CHECK(wheel_fr::stepNow() == 0 && wheel_fl::stepNow() == 0,
        "w FR 12.5, пришедшая на FL, никого не крутит");
  // Своя команда — крутит только своё колесо.
  board_fr.serial.send("w FR 12.5");
  CHECK(wait_until([] { return !wheel_fr::motorBusy() && wheel_fr::stepNow() == 1667; }, 3000),
        "w FR 12.5 на своём порту: FR=%ld", (long)wheel_fr::stepNow());
  CHECK(wheel_fl::stepNow() == 0 && wheel_rl::stepNow() == 0, "остальные не тронуты");
  sim::run_for_ms(300);                  // телеметрия FR: tgt=12.50 в кадре
  CHECK(has(board_fr, "@TLM deg=12.50 tgt=12.50 moving=0 cal=1"), "телеметрия FR на 12.5°");
  board_fl.serial.send("w FL -7");
  CHECK(wait_until([] { return !wheel_fl::motorBusy() && wheel_fl::stepNow() == -933; }, 3000),
        "w FL -7 — своё колесо: %ld", (long)wheel_fl::stepNow());
  board_rl.serial.send("w RL zero");
  sim::run_for_ms(50);
  CHECK(wheel_rl::home_source == wheel_rl::HOME_MANUAL, "w RL zero — ручной ноль RL");
  board_rr.serial.send("w RR calib");
  sim::run_for_ms(5);
  CHECK(wheel_rr::calib_state != wheel_rr::CAL_IDLE, "w RR calib запускает калибровку RR");
  CHECK(wait_until([] { return wheel_rr::calib_state == wheel_rr::CAL_IDLE; }, 30000) &&
        wheel_rr::home_source == wheel_rr::HOME_OPTO, "RR откалиброван заново");
  board_fr.serial.send("w XX 5");
  board_fl.serial.send("w FL 101");
  sim::run_for_ms(20);
  CHECK(has(board_fr, "неизвестный модуль XX"), "неизвестный модуль");
  CHECK(has(board_fl, "# FL: w: угол вне ±100°"), "предел угла у сервера");
  // Безымянная команда «w 90» работает на любом порту: так прошивают с руки.
  board_fr.serial.send("w 90");
  CHECK(wait_until([] { return !wheel_fr::motorBusy() && wheel_fr::stepNow() == 12000; }, 5000),
        "w 90 → 12000 шагов, сейчас %ld", (long)wheel_fr::stepNow());
  board_fr.serial.send("w FR 100");
  CHECK(wait_until([] { return !wheel_fr::motorBusy() && wheel_fr::stepNow() == 13333; }, 5000),
        "w FR 100 → 13333 шага, сейчас %ld", (long)wheel_fr::stepNow());
  board_fr.serial.send("w FR z");
  sim::run_for_ms(20);
  CHECK(wheel_fr::home_source == wheel_fr::HOME_MANUAL, "короткое w FR z — ручной ноль");
  board_fr.serial.send("w FR k");
  CHECK(wait_until([] { return wheel_fr::calib_state == wheel_fr::CAL_IDLE; }, 30000),
        "короткое w FR k — калибровка");
}

static void send_drive_heartbeat(int percent, int times) {
  char command[16];
  std::snprintf(command, sizeof(command), "t %d", percent);
  for (int i = 0; i < times; ++i) {
    for (sim::Board* board : all_boards()) board->serial.send(command);
    sim::run_for_ms(200);
  }
}

static bool all_pwm(int value) {
  return board_fl.analog[3] == value && board_fr.analog[3] == value && board_rl.analog[3] == value &&
         board_rr.analog[3] == value;
}

static void scenario_system_drive() {
  calibrate_all();
  send_drive_heartbeat(8, 3);
  CHECK(all_pwm(DRIVE_DUTY_8), "t 8 на все четыре порта: %d %d %d %d", board_fl.analog[3],
        board_fr.analog[3], board_rl.analog[3], board_rr.analog[3]);
  for (sim::Board* board : all_boards()) board->serial.send("b");
  sim::run_for_ms(10);
  CHECK(all_pwm(0) && board_fl.level[5] == HIGH && board_rr.level[5] == HIGH, "b — тормоз у всех");
  // Плата без нуля тягу не берёт: у каждой платы свой fail-safe.
  wheel_rl::zero_set = false;
  send_drive_heartbeat(8, 2);
  CHECK(board_rl.analog[3] == 0 && board_fl.analog[3] == DRIVE_DUTY_8 &&
            board_fr.analog[3] == DRIVE_DUTY_8,
        "RL без нуля не едет, остальные едут: %d %d %d", board_rl.analog[3], board_fl.analog[3],
        board_fr.analog[3]);
  wheel_rl::zero_set = true;
  send_drive_heartbeat(8, 1);
  CHECK(all_pwm(DRIVE_DUTY_8), "нули вернули — тяга снова у всех");
  sim::run_for_ms(800);
  CHECK(all_pwm(0), "без heartbeat все останавливаются по watchdog");
  CHECK(has(board_fl, "нет команды дольше 650 мс"), "watchdog объяснён в журнале FL");
}

static void scenario_system_broadcast() {
  calibrate_all();
  for (sim::Board* board : all_boards()) board->serial.send("d 10 f");
  sim::run_for_ms(5);                    // команды обработаны — все в движении
  CHECK(wait_until(all_idle, 2000), "d 10 f у всех закончилась");
  CHECK(wheel_fl::stepNow() == 1333 && wheel_fr::stepNow() == 1333 && wheel_rl::stepNow() == 1333 &&
            wheel_rr::stepNow() == 1333,
        "рассылка на все порты — все четыре колеса +10°");
  for (sim::Board* board : all_boards()) board->serial.send("d 90 b");
  sim::run_for_ms(200);
  for (sim::Board* board : all_boards()) board->serial.send("s");
  sim::run_for_ms(5);
  const long fl = board_fl.phys;
  const long rr = board_rr.phys;
  sim::run_for_ms(300);
  CHECK(board_fl.phys == fl && board_rr.phys == rr, "s останавливает все колёса");
  for (sim::Board* board : all_boards()) board->serial.send("h");
  sim::run_for_ms(5);
  CHECK(wait_until(all_idle, 3000), "h у всех закончилась");
  CHECK(wheel_fl::stepNow() == 0 && wheel_fr::stepNow() == 0 && wheel_rl::stepNow() == 0 &&
            wheel_rr::stepNow() == 0,
        "h — все в нуле");
  board_rl.serial.send("p");
  sim::run_for_ms(5);
  CHECK(has(board_rl, "# RL: Текущий: 0.00° | Целевой: 0.00° [СТОИТ]"), "p показывает своё колесо");
  CHECK(has(board_rl, "колесо RL, прошивка v5"), "p называет колесо и прошивку");
  for (sim::Board* board : all_boards()) board->serial.send("pm 0");
  sim::run_for_ms(600);
  CHECK(!M::pot_enabled && !wheel_fr::pot_enabled && !wheel_rl::pot_enabled && !wheel_rr::pot_enabled,
        "pm 0 доходит до всех четырёх плат");
  CHECK(has(board_fl, "pm=0"), "disabled mode is in telemetry");
  for (sim::Board* board : all_boards()) board->serial.send("pm 1");
  sim::run_for_ms(600);
  CHECK(M::pot_enabled && wheel_fr::pot_enabled && wheel_rl::pot_enabled && wheel_rr::pot_enabled,
        "pm 1 доходит до всех четырёх плат");
  CHECK(has(board_fl, "pm=1"), "enabled mode is in telemetry");
}

// Пресет бортового экрана — это четыре команды подряд: FL, FR, RL, RR с
// интервалом в десятки миллисекунд (экран шлёт их по очереди, не дожидаясь
// окончания поворота). Проверяем, что мастер раздаёт всю пачку и что последнее
// в ней колесо не теряется: жалоба «RR всегда ничего не исполняет» означала бы,
// что четвёртая команда до слейва не доходит.
// Доводка по потенциометру доворачивает колесо на единицы градусов (люфт, увод),
// а не возит его по всей шкале. Если датчик расходится с целью сильно — сбился
// его ноль, изношена дорожка или не сделана q, — верить ему нельзя: раньше
// доводка догоняла именно датчик и утаскивала колесо на десятки градусов от
// команды («повернулось на 4° вместо 35°»), а привязка счётчика шагов к датчику
// заставляла телеметрию отрапортовать успех при колесе совсем не там.
static void scenario_pot_distrust() {
  boot({&board_fr});
  calibrate_fr();
  calibrate_pot_fr();
  // Датчик «уехал» относительно колеса на 31°: так выглядит сбившийся ноль.
  board_fr.pot_bias_steps = 4133;
  board_fr.serial.send("w 35");
  sim::run_for_ms(2);
  CHECK(wait_until(fr_settled, 8000), "w 35 не закончилась");
  // Колесо осталось там, куда его привели шаги, а не там, где видит датчик.
  CHECK(board_fr.phys == 4667, "колесо на 35° (4667 шагов), а не %ld", board_fr.phys);
  CHECK(FR::stepNow() == 4667, "счётчик шагов не переписан датчиком: %ld",
        (long)FR::stepNow());
  CHECK(has(board_fr, "расхождение 31.08"), "расхождение названо оператору: %s",
        board_fr.serial.output.c_str());
  CHECK(has(board_fr, "доводка отключена"), "доводка честно отключена: %s",
        board_fr.serial.output.c_str());
  // В пределах доверия датчик по-прежнему доворачивает колесо (см. pot_feedback).
}

// Заклинивание колеса: драйвер продолжает получать импульсы, а колесо стоит.
// Прошивка считает СГЕНЕРИРОВАННЫЕ импульсы (current_step += step_dir), поэтому
// открытый контур по шагам уверен, что цель достигнута. Без откалиброванного
// потенциометра заметить это нечем — телеметрия рапортует 35° при колесе на 4°.
static void scenario_stalled_wheel_without_pot() {
  boot({&board_fr});
  calibrate_fr();
  board_fr.stall_enabled = true;
  board_fr.stall_phys = 533;                 // колесо клинит на 4°
  board_fr.serial.output.clear();
  board_fr.serial.send("w 35");
  sim::run_for_ms(2);
  CHECK(wait_until(fr_settled, 8000), "w 35 не закончилась");
  CHECK(board_fr.phys == 533, "колесо заклинило на 533 шагах, а не %ld", board_fr.phys);
  CHECK(board_fr.steps_lost > 4000, "потерянные импульсы посчитаны: %ld",
        board_fr.steps_lost);
  CHECK(FR::stepNow() == 4667, "счётчик шагов рапортует 35°: %ld", (long)FR::stepNow());
  CHECK(!FR::pot_calibrated, "без q доводка выключена — расхождение некому заметить");
}

// То же заклинивание, но потенциометр откалиброван: датчик видит настоящие 4°,
// расхождение с целью больше POT_TRUST_DEG, и оператору честно говорят, что
// колесо не там. Доводка при этом не пытается «догнать» цель через датчик.
static void scenario_stalled_wheel_with_pot() {
  boot({&board_fr});
  calibrate_fr();
  calibrate_pot_fr();
  board_fr.stall_enabled = true;
  board_fr.stall_phys = 533;
  board_fr.serial.output.clear();
  board_fr.serial.send("w 35");
  sim::run_for_ms(2);
  CHECK(wait_until(fr_settled, 8000), "w 35 не закончилась");
  CHECK(board_fr.phys == 533, "колесо на 533 шагах: %ld", board_fr.phys);
  CHECK(has(board_fr, "расхождение"), "заклинивание названо оператору: %s",
        board_fr.serial.output.c_str());
  CHECK(has(board_fr, "доводка отключена"), "доводка честно отключена: %s",
        board_fr.serial.output.c_str());
}

// Наклон потенциометра — свойство железа, а не сессии. Пока он жил только в
// RAM, доводка после каждой перезагрузки молча выключалась (pot_calibrated =
// false), и колесо снова ездило вслепую. Теперь q достаточно выполнить один раз.
static void scenario_pot_slope_survives_reboot() {
  boot({&board_fr});
  calibrate_fr();
  calibrate_pot_fr();
  const float slope = FR::pot_counts_per_degree;
  CHECK(slope > 0.0f, "наклон измерен");
  // Перезагрузка платы. setup() не обнуляет глобалы скетча (в реальном AVR их
  // сбрасывает старт процессора), поэтому RAM-состояние pot гасим руками —
  // иначе pot_calibrated доживёт с прошлой калибровки и сценарий пройдёт
  // впустую, ничего не проверив.
  FR::pot_calibrated = false;
  FR::pot_zero_valid = false;
  FR::pot_counts_per_degree = FR::POT_NOMINAL_COUNTS_PER_DEG;
  board_fr.serial.output.clear();
  FR::setup();
  sim::run_for_ms(10);
  CHECK(FR::pot_calibrated, "наклон восстановлен из EEPROM без повторной q");
  CHECK(FR::pot_counts_per_degree == slope, "наклон тот же: %f != %f",
        (double)FR::pot_counts_per_degree, (double)slope);
  CHECK(has(board_fr, "наклон из EEPROM"), "оператору сказано, что наклон из памяти: %s",
        board_fr.serial.output.c_str());
}


// Пресет бортового экрана — это четыре адресные команды подряд: FL, FR, RL, RR
// с интервалом в десятки миллисекунд, каждая на свой порт. Последнее в пачке
// колесо (RR) не должно теряться: жалоба «RR всегда ничего не исполняет»
// означала бы, что четвёртая команда до платы не доходит.
static void scenario_preset_burst() {
  calibrate_all();
  const long rr_zero = (long)wheel_rr::stepNow();
  board_fl.serial.send("w FL 90");
  sim::run_for_ms(15);
  board_fr.serial.send("w FR 90");
  sim::run_for_ms(15);
  board_rl.serial.send("w RL 90");
  sim::run_for_ms(15);
  board_rr.serial.send("w RR 90");
  CHECK(wait_until(all_idle, 20000), "все четыре колеса доехали и остановились");
  CHECK(wheel_fl::stepNow() == 12000, "FL: 90° — это 12000 шагов, а не %ld", (long)wheel_fl::stepNow());
  CHECK(wheel_fr::stepNow() == 12000, "FR: %ld шагов", (long)wheel_fr::stepNow());
  CHECK(wheel_rl::stepNow() == 12000, "RL: %ld шагов", (long)wheel_rl::stepNow());
  CHECK((long)wheel_rr::stepNow() == rr_zero + 12000,
        "RR — последний в пачке — доехал: %ld шагов от нуля %ld",
        (long)wheel_rr::stepNow(), rr_zero);
  for (sim::Board* board : all_boards()) {
    CHECK(has(*board, "tgt=90.00"),
          "цель 90° в телеметрии %s видна экрану: %s", board->name,
          board->serial.output.c_str());
    check_signals(*board);
  }
}

static void scenario_tlm_dump() {
  calibrate_all();
  for (sim::Board* board : all_boards()) board->serial.send("q");
  sim::run_for_ms(5);                    // команды обработаны — калибровка идёт
  CHECK(wait_until(all_pot_idle, 30000), "калибровка потенциометров всех колёс");
  board_fr.serial.send("w FR 12.5");
  sim::run_for_ms(700);
  for (sim::Board* board : all_boards()) {
    std::printf("PORT %s\n", board->name);
    std::fwrite(board->serial.output.data(), 1, board->serial.output.size(), stdout);
  }
}

// ./sim pty /tmp/wheel — четыре колеса в реальном времени за псевдотерминалами
// /tmp/wheel-FL … /tmp/wheel-RR: задайте вручную RC_WHEEL_FL/FR/RL/RR_PORT и видит настоящую логику прошивки — для ручной проверки без железа.
static int run_pty(const char* link_base) {
  struct Link {
    int fd;
    std::string path;
    size_t sent = 0;
  };
  std::vector<Link> links;
  for (sim::Board* board : all_boards()) {
    const int fd = posix_openpt(O_RDWR | O_NOCTTY);
    if (fd < 0 || grantpt(fd) != 0 || unlockpt(fd) != 0) {
      std::perror("posix_openpt");
      return 2;
    }
    // Сторона робота сразу в raw-режиме: без эха телеметрии обратно в «сервер».
    const int slave = open(ptsname(fd), O_RDWR | O_NOCTTY);
    if (slave >= 0) {
      termios mode;
      if (tcgetattr(slave, &mode) == 0) {
        cfmakeraw(&mode);
        tcsetattr(slave, TCSANOW, &mode);
      }
      close(slave);
    }
    Link link;
    link.fd = fd;
    link.path = std::string(link_base) + "-" + board->name;
    unlink(link.path.c_str());
    if (symlink(ptsname(fd), link.path.c_str()) != 0) {
      std::perror("symlink");
      return 2;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    links.push_back(link);
  }
  bind_boards();
  board_fl.phys = -3000;
  board_fr.phys = 2500;
  board_rl.phys = 40;
  board_rr.phys = -9000;
  boot(all_boards());
  std::printf("колёса: %s-FL … %s-RR → псевдотерминалы\n", link_base, link_base);
  std::fflush(stdout);
  const auto wall_start = std::chrono::steady_clock::now();
  const int64_t sim_start = sim::now_us;
  for (;;) {
    std::vector<sim::Board*> boards = all_boards();
    for (size_t i = 0; i < links.size(); ++i) {
      char buffer[256];
      const ssize_t got = read(links[i].fd, buffer, sizeof(buffer));
      for (ssize_t j = 0; j < got; ++j) boards[i]->serial.input.push_back((uint8_t)buffer[j]);
    }
    sim::run_for_ms(10);
    for (size_t i = 0; i < links.size(); ++i) {
      const std::string& out = boards[i]->serial.output;
      if (out.size() > links[i].sent) {
        const ssize_t written = write(links[i].fd, out.data() + links[i].sent, out.size() - links[i].sent);
        if (written > 0) links[i].sent += (size_t)written;
      }
    }
    std::this_thread::sleep_until(wall_start + std::chrono::microseconds(sim::now_us - sim_start));
  }
}

// Настройка тяги (команда m): режим выхода, тахометр, тормоз, реверс, предел,
// мёртвая зона в милливольтах и сохранение в EEPROM. Раньше всё это было
// зашито в исходник: колесо с аналоговым входом 0–5 В или без тахометра
// приходилось перепрошивать, а сервер открывает порт с DTR — плата стартует
// заново при каждом запуске экрана.
static void scenario_drive_config() {
  boot({&board_fr});
  board_fr.serial.send("z");
  sim::run_for_ms(2);
  CHECK(board_fr.tach_fn != nullptr, "по умолчанию тахометр включён");
  CHECK(FR::drive_mode == FR::DRIVE_OUT_VOLTAGE, "по умолчанию 0–5 В");

  // ШИМ остаётся доступен: процент — это скважность (t 10 → 26/255).
  board_fr.serial.send("m pwm");
  board_fr.serial.send("t 10");
  sim::run_for_ms(2);
  CHECK(FR::drive_mode == FR::DRIVE_OUT_PWM, "m pwm — режим ШИМ");
  CHECK(board_fr.analog[3] == 26, "t 10 в режиме ШИМ: %d/255", board_fr.analog[3]);
  board_fr.serial.send("b");
  sim::run_for_ms(2);

  board_fr.serial.send("m volt");
  board_fr.serial.send("m vmin 500");
  board_fr.serial.send("m vmax 4500");
  sim::run_for_ms(2);
  CHECK(FR::drive_mode == FR::DRIVE_OUT_VOLTAGE, "m volt — режим 0–5 В");
  board_fr.serial.send("t 10");
  sim::run_for_ms(2);
  // 500 + (4500 − 500)·10/20 = 2500 мВ → 128/255 скважности.
  CHECK(board_fr.analog[3] == 128, "t 10 в режиме 0–5 В: %d/255", board_fr.analog[3]);
  CHECK(std::labs((long)FR::driveOutputMv() - 2500L) <= 20,
        "выход %u мВ, ожидалось 2500", (unsigned)FR::driveOutputMv());
  board_fr.serial.send("b");
  sim::run_for_ms(2);

  // Тахометр не подключён: прерывание снимается, телеметрия говорит tach=0.
  board_fr.serial.send("m tach 0");
  sim::run_for_ms(300);
  CHECK(board_fr.tach_fn == nullptr, "m tach 0 снимает прерывание");
  CHECK(!FR::drive_tach_enabled, "тахометр выключен");
  CHECK(has(board_fr, "tach=0"), "в @TLM нет tach=0: %s", board_fr.serial.output.c_str());
  board_fr.serial.send("m tach 1");
  sim::run_for_ms(2);
  CHECK(board_fr.tach_fn != nullptr, "m tach 1 возвращает прерывание");

  // Тормозной вход не подключён: BRAKE не тянется в активный уровень.
  board_fr.serial.send("m brake 0");
  sim::run_for_ms(2);
  CHECK(board_fr.level[5] == LOW, "без тормоза BRAKE неактивен");
  board_fr.serial.send("t 5");
  sim::run_for_ms(2);
  CHECK(FR::drive_active && board_fr.analog[3] > 0 && board_fr.level[5] == LOW,
        "едем без тормозного входа: %d", board_fr.analog[3]);
  board_fr.serial.send("b");
  sim::run_for_ms(2);
  CHECK(board_fr.analog[3] == 0 && board_fr.level[5] == LOW, "стоп без тормоза — выход 0");
  board_fr.serial.send("m brake 1");
  sim::run_for_ms(2);
  CHECK(board_fr.level[5] == HIGH, "тормоз снова включён");

  // Реверс: DIR перевёрнут относительно штатного «вперёд».
  board_fr.serial.send("m dir 1");
  board_fr.serial.send("t 5");
  sim::run_for_ms(2);
  CHECK(board_fr.level[4] != FR::BLDC_DIR_FORWARD_LEVEL, "m dir 1 инвертирует DIR");
  board_fr.serial.send("b");
  sim::run_for_ms(2);

  // Предел: выше предела тяга не берётся, ровно предел — едет.
  board_fr.serial.send("m max 6");
  sim::run_for_ms(2);
  board_fr.serial.send("t 8");
  sim::run_for_ms(2);
  CHECK(!FR::drive_active && board_fr.analog[3] == 0, "t выше предела не выполняется");
  board_fr.serial.send("t 6");
  sim::run_for_ms(2);
  CHECK(FR::drive_active && board_fr.analog[3] > 0, "t по пределу едет");
  board_fr.serial.send("b");
  sim::run_for_ms(2);

  // Некорректные значения отклоняются, а не «примерно применяются».
  board_fr.serial.send("m max 0");
  board_fr.serial.send("m vmin 5000");
  sim::run_for_ms(2);
  CHECK(FR::drive_max_percent == 6 && FR::drive_min_mv == 500,
        "некорректные m max / m vmin отклонены: max=%u vmin=%u",
        (unsigned)FR::drive_max_percent, (unsigned)FR::drive_min_mv);

  // Настройка переживает перезапуск платы: сервер открывает порт с DTR.
  FR::drive_mode = FR::DRIVE_OUT_PWM;
  FR::drive_tach_enabled = false;
  FR::drive_brake_enabled = true;
  FR::drive_dir_reverse = false;
  FR::drive_max_percent = FR::DRIVE_MAX_PERCENT;
  FR::drive_min_mv = 0;
  FR::drive_max_mv = FR::DRIVE_MV_FULL;
  board_fr.serial.output.clear();
  FR::setup();
  sim::run_for_ms(5);
  CHECK(FR::drive_mode == FR::DRIVE_OUT_VOLTAGE && FR::drive_max_percent == 6 &&
            FR::drive_min_mv == 500 && FR::drive_max_mv == 4500 && FR::drive_dir_reverse &&
            FR::drive_brake_enabled,
        "настройка из EEPROM: mode=%u max=%u %u-%u dir=%d tach=%d brake=%d",
        (unsigned)FR::drive_mode, (unsigned)FR::drive_max_percent,
        (unsigned)FR::drive_min_mv, (unsigned)FR::drive_max_mv, FR::drive_dir_reverse,
        FR::drive_tach_enabled, FR::drive_brake_enabled);

  // Заводские значения: аналоговый выход, тахометр и тормоз, предел 20 %.
  board_fr.serial.send("m brake 1");
  board_fr.serial.send("m reset");
  sim::run_for_ms(2);
  CHECK(FR::drive_mode == FR::DRIVE_OUT_VOLTAGE && FR::drive_max_percent == FR::DRIVE_MAX_PERCENT &&
            FR::drive_min_mv == 0 && FR::drive_max_mv == FR::DRIVE_MV_FULL &&
            !FR::drive_dir_reverse && FR::drive_brake_enabled,
        "m reset — заводские значения");
}

// Тяга одного колеса: сервер шлёт «t FL 8» на каждый порт, а колесо берёт
// только своё имя. Раньше на «t FL 8» плата видела второй аргумент и гасила
// выход — то есть адресная тяга не работала бы вовсе.
static void scenario_drive_one_wheel() {
  calibrate_all();
  board_fl.serial.send("t FL 8");
  sim::run_for_ms(2);
  CHECK(wheel_fl::drive_active && board_fl.analog[3] == DRIVE_DUTY_8,
        "своё имя во «t FL 8» не мешает тяге: %d", board_fl.analog[3]);
  CHECK(!wheel_fr::drive_active && board_fr.analog[3] == 0,
        "чужое имя «t FL 8» на FR ничего не включает: %d", board_fr.analog[3]);
  CHECK(!wheel_rr::drive_active && board_rr.analog[3] == 0, "и на RR тоже");

  // Heartbeat идёт только на FL: остальные стоят, а через 650 мс FL встаёт сам.
  for (int i = 0; i < 3; ++i) {
    board_fl.serial.send("t FL 8");
    sim::run_for_ms(200);
  }
  CHECK(wheel_fl::drive_active, "heartbeat «t FL 8» держит только FL");
  sim::run_for_ms(700);
  CHECK(!wheel_fl::drive_active && board_fl.analog[3] == 0, "без heartbeat FL встал по watchdog");

  // Разные колёса едут в разные стороны — так и делается разворот на месте.
  board_fl.serial.send("t FL 6");
  board_fr.serial.send("t FR -6");
  sim::run_for_ms(2);
  CHECK(wheel_fl::drive_active && wheel_fr::drive_active,
        "FL и FR едут одновременно");
  CHECK(board_fl.level[4] != board_fr.level[4], "направления разные: FL=%d FR=%d",
        board_fl.level[4], board_fr.level[4]);
  CHECK(board_rl.analog[3] == 0 && board_rr.analog[3] == 0, "RL и RR стоят");
  board_fl.serial.send("t FL 0");
  sim::run_for_ms(2);
  CHECK(!wheel_fl::drive_active && board_fl.analog[3] == 0, "«t FL 0» — стоп только FL");
  CHECK(wheel_fr::drive_active, "FR продолжает ехать");
  for (sim::Board* board : all_boards()) board->serial.send("b");
  sim::run_for_ms(2);
  CHECK(!wheel_fr::drive_active && board_fr.analog[3] == 0, "b — тормоз всем");

  // Имя без числа и чужое неизвестное имя честно объяснены оператору.
  board_fl.serial.send("t FL");
  board_fl.serial.send("t FX 8");
  sim::run_for_ms(2);
  CHECK(has(board_fl, "Шаблон: t [FL]"), "подсказка по «t FL» без числа");
  CHECK(has(board_fl, "неизвестный модуль FX"), "неизвестное имя колеса объяснено");
}

struct Scenario {
  const char* name;
  void (*fn)();
};

static const Scenario SCENARIOS[] = {
    {"boot", scenario_boot},
    {"identity", scenario_identity},
    {"calib_left", scenario_calib_left},
    {"calib_right", scenario_calib_right},
    {"calib_inside", scenario_calib_inside},
    {"calib_far_left", scenario_calib_far_left},
    {"calib_out_of_range", scenario_calib_out_of_range},
    {"calib_no_sensor", scenario_calib_no_sensor},
    {"calib_stuck", scenario_calib_stuck},
    {"calib_abort", scenario_calib_abort},
    {"calib_asymmetric", scenario_calib_asymmetric},
    {"calib_full_range_plus", scenario_calib_full_range_plus},
    {"calib_full_range_minus", scenario_calib_full_range_minus},
    {"calib_passes_averaged", scenario_calib_passes_averaged},
    {"calib_spread_error", scenario_calib_spread_error},
    {"moves", scenario_moves},
    {"pot_calibration", scenario_pot_calibration},
    {"pot_feedback", scenario_pot_feedback},
    {"pot_unplugged", scenario_pot_unplugged},
    {"pot_mode", scenario_pot_mode},
    {"swing", scenario_swing},
    {"pause_resume", scenario_pause_resume},
    {"zero_manual", scenario_zero_manual},
    {"drive", scenario_drive},
    {"serial_garbage", scenario_serial_garbage},
    {"system_calibrate_all", scenario_system_calibrate_all},
    {"system_server_commands", scenario_system_server_commands},
    {"system_drive", scenario_system_drive},
    {"system_broadcast", scenario_system_broadcast},
    {"preset_burst", scenario_preset_burst},
    {"pot_distrust", scenario_pot_distrust},
    {"stalled_wheel_without_pot", scenario_stalled_wheel_without_pot},
    {"stalled_wheel_with_pot", scenario_stalled_wheel_with_pot},
    {"pot_slope_survives_reboot", scenario_pot_slope_survives_reboot},
    {"drive_config", scenario_drive_config},
    {"drive_one_wheel", scenario_drive_one_wheel},
};

static int run_one(const Scenario& scenario) {
  bind_boards();
  scenario.fn();
  if (failures) {
    std::fprintf(stderr, "%s: %d ошибок\n", scenario.name, failures);
    for (sim::Board* board : sim::boards) {
      std::fprintf(stderr, "--- вывод %s ---\n%s\n", board->name, board->serial.output.c_str());
    }
  }
  return failures ? 1 : 0;
}

int main(int argc, char** argv) {
  if (argc > 2 && std::strcmp(argv[1], "pty") == 0) return run_pty(argv[2]);
  if (argc > 1 && std::strcmp(argv[1], "tlm_dump") == 0) {
    bind_boards();
    scenario_tlm_dump();
    return failures ? 1 : 0;
  }
  int failed = 0;
  int ran = 0;
  for (const Scenario& scenario : SCENARIOS) {
    if (argc > 1 && std::strcmp(argv[1], scenario.name) != 0) continue;
    ++ran;
    std::fflush(stdout);
    const pid_t pid = fork();
    if (pid == 0) _exit(run_one(scenario));
    int status = 0;
    waitpid(pid, &status, 0);
    const bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
    std::printf("%-26s %s\n", scenario.name, ok ? "ok" : "FAIL");
    if (!ok) ++failed;
  }
  if (!ran) {
    std::fprintf(stderr, "нет сценария %s\n", argc > 1 ? argv[1] : "");
    return 2;
  }
  std::printf("%d/%d сценариев прошли\n", ran - failed, ran);
  return failed ? 1 : 0;
}
