// Хост-симуляция прошивок руля: мастер (FL) + три слейва (FR/RL/RR) на общей
// шине I2C, у каждого — модель TB6600 и оптодатчика нуля на D18.
//
// Сборка и запуск (это делает tests/test_firmware.py):
//   g++ -std=c++17 -I tests/firmware_sim/stubs tests/firmware_sim/sim.cpp -o sim
//   ./sim            — все сценарии (каждый в своём процессе, как после сброса)
//   ./sim moves      — один сценарий
//   ./sim tlm_dump   — вывести USB мастера после калибровки (для разбора сервером)
#include "arduino_sim.h"

#include <chrono>
#include <fcntl.h>
#include <functional>
#include <sys/wait.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

sim::Board board_fl, board_fr, board_rl, board_rr;

namespace master {
#define SIM_BOARD board_fl
#include "board_prelude.h"
#include "../../firmware/motor_master/motor_master.ino"
#undef SIM_BOARD
}  // namespace master
#undef MODULE_ID
#undef TX_BUFFER_SIZE

namespace node_fr {
#define SIM_BOARD board_fr
#define NODE_ADDRESS 2
#define MODULE_ID "FR"
#include "board_prelude.h"
#include "../../firmware/motor_node/motor_node.ino"
#undef SIM_BOARD
}  // namespace node_fr
#undef NODE_ADDRESS
#undef MODULE_ID
#undef TX_BUFFER_SIZE

namespace node_rl {
#define SIM_BOARD board_rl
#define NODE_ADDRESS 3
#define MODULE_ID "RL"
#include "board_prelude.h"
#include "../../firmware/motor_node/motor_node.ino"
#undef SIM_BOARD
}  // namespace node_rl
#undef NODE_ADDRESS
#undef MODULE_ID
#undef TX_BUFFER_SIZE

namespace node_rr {
#define SIM_BOARD board_rr
#define NODE_ADDRESS 4
#define MODULE_ID "RR"
#include "board_prelude.h"
#include "../../firmware/motor_node/motor_node.ino"
#undef SIM_BOARD
}  // namespace node_rr
#undef NODE_ADDRESS
#undef MODULE_ID
#undef TX_BUFFER_SIZE

// ------------------------------------------------------------------------
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

static void wire_boards() {
  board_fl.name = "FL";
  board_fl.setup_fn = master::setup;
  board_fl.loop_fn = master::loop;
  board_fl.isr_fn = master::TIMER1_COMPA_vect_handler;
  board_fr.name = "FR";
  board_fr.setup_fn = node_fr::setup;
  board_fr.loop_fn = node_fr::loop;
  board_fr.isr_fn = node_fr::TIMER1_COMPA_vect_handler;
  board_rl.name = "RL";
  board_rl.setup_fn = node_rl::setup;
  board_rl.loop_fn = node_rl::loop;
  board_rl.isr_fn = node_rl::TIMER1_COMPA_vect_handler;
  board_rr.name = "RR";
  board_rr.setup_fn = node_rr::setup;
  board_rr.loop_fn = node_rr::loop;
  board_rr.isr_fn = node_rr::TIMER1_COMPA_vect_handler;
  for (sim::Board* board : {&board_fl, &board_fr, &board_rl, &board_rr}) board->wire.board = board;
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
#define FR node_fr

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
  CHECK(has(board_fr, "--- SLAVE FR, I2C 2"), "приветствие слейва: %s", board_fr.serial.output.c_str());
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

  // Предел абсолютной команды — ±90° (STEER_LIMIT_DEG): боковой ход колесу
  // доступен, угол вне предела его не двигает, возврат в рабочее положение — тоже.
  board_fr.serial.send("w 90");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 12000; }, 3000),
        "w 90 → 12000 шагов, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == 12000, "колесо на 90°: phys=%ld", board_fr.phys);

  board_fr.serial.send("w 100");
  sim::run_for_ms(50);
  CHECK(board_fr.phys == 12000 && has(board_fr, "угол вне ±90°"), "w 100 отклонена");

  board_fr.serial.send("w -90");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == -12000; }, 5000),
        "w -90 → -12000 шагов, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == -12000, "колесо на −90°: phys=%ld", board_fr.phys);

  board_fr.serial.send("w 12.5");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 1667; }, 5000),
        "возврат на 12.5° после ±90°, сейчас %ld", (long)FR::stepNow());
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

static void scenario_drive() {
  boot({&board_fr});
  board_fr.serial.send("t 8");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 0 && board_fr.level[5] == HIGH, "без нуля тяга запрещена");
  board_fr.serial.send("z");
  board_fr.serial.send("t 8");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 20 && board_fr.level[5] == LOW, "t 8 → PWM 20/255, тормоз снят");
  board_fr.serial.send("d 5 f");
  sim::run_for_ms(20);
  CHECK(board_fr.phys == 0 && has(board_fr, "руль заблокирован: включена тяга"), "при тяге руль не крутится");
  for (int i = 0; i < 5; ++i) {
    board_fr.serial.send("t 8");
    sim::run_for_ms(200);
  }
  CHECK(FR::drive_active && board_fr.analog[3] == 20, "heartbeat каждые 200 мс держит тягу");
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
  for (const char* chunk : {"@TLM mod=RR deg=0.00 tgt=0.00 mov", "\x07\xff#"}) {
    for (const char* p = chunk; *p; ++p) board_fl.serial.input.push_back((uint8_t)*p);
    sim::run_for_ms(1200);               // обрывок без \n висит дольше секунды
  }
  board_fl.serial.send("k");
  sim::run_for_ms(5);
  CHECK(master::calib_state != master::CAL_IDLE, "после мусора в порту команда k всё равно выполнена");
  board_fl.serial.send("@TLM mod=FL deg=0.00 tgt=0.00 moving=0 cal=0");
  board_fl.serial.send("# FL: эхо сообщения");
  sim::run_for_ms(5);
  CHECK(!has(board_fl, "Неизв. команда: @TLM") && !has(board_fl, "Неизв. команда: #"),
        "эхо телеметрии и сообщений не считается командой");
  board_fl.serial.send("d 1");            // частичный ввод человека с паузами < 1 с
  sim::run_for_ms(5);
  CHECK(has(board_fl, "Шаблон: d [градусы] [f/b]"), "обычные ошибки по-прежнему видны");
}

static void scenario_frame_limits() {
  boot({&board_fr});
  FR::current_step = -123456;
  FR::target_step = 123456;
  FR::rpm_x10 = 4000000000UL;
  FR::drive_percent = -20;
  FR::refreshFrame();
  const uint8_t index = FR::frame_front;
  const std::string frame(FR::frame[index], FR::frame_length[index]);
  CHECK(frame.size() <= 32, "кадр I2C не длиннее 32 байт: %zu «%s»", frame.size(), frame.c_str());
  CHECK(frame.rfind("-99999,99999,", 0) == 0, "шаги ограничены ±99999: %s", frame.c_str());
  CHECK(frame.find(",99999,-20,") != std::string::npos, "об/мин и тяга на месте: %s", frame.c_str());
  int commas = 0;
  for (char ch : frame) commas += ch == ',' ? 1 : 0;
  CHECK(commas == 5, "кадр v3: шесть полей, счётчик отдельно: %s", frame.c_str());
}

// ---------------------------------------------------------------- вся система
#define M master
static std::vector<sim::Board*> all_boards() { return {&board_fl, &board_fr, &board_rl, &board_rr}; }

static bool all_idle() {
  return M::calib_state == M::CAL_IDLE && !M::motorBusy() &&
         node_fr::calib_state == node_fr::CAL_IDLE && !node_fr::motorBusy() &&
         node_rl::calib_state == node_rl::CAL_IDLE && !node_rl::motorBusy() &&
         node_rr::calib_state == node_rr::CAL_IDLE && !node_rr::motorBusy();
}

static bool all_zero() { return M::zero_set && node_fr::zero_set && node_rl::zero_set && node_rr::zero_set; }

static bool all_pot_idle() {
  return M::pot_cal_state == M::POT_CAL_IDLE && node_fr::pot_cal_state == node_fr::POT_CAL_IDLE &&
         node_rl::pot_cal_state == node_rl::POT_CAL_IDLE && node_rr::pot_cal_state == node_rr::POT_CAL_IDLE &&
         !M::motorBusy() && !node_fr::motorBusy() && !node_rl::motorBusy() && !node_rr::motorBusy();
}

static void calibrate_all() {
  board_fl.phys = -3000;
  board_fr.phys = 2500;                  // справа от флажка: поиск влево
  board_rl.phys = 40;                    // внутри флажка
  board_rr.phys = -9000;
  board_rr.flag_lo = 300;                // у RR флажок смещён: ноль будет на 500 шагах
  board_rr.flag_hi = 700;
  boot(all_boards());
  sim::run_for_ms(600);
  board_fl.serial.send("k");
  sim::run_for_ms(5);
  CHECK(wait_until(all_idle, 30000), "калибровка всех колёс закончилась");
  sim::run_for_ms(600);                  // мастер опросит слейвов
}

static void scenario_system_calibrate_all() {
  calibrate_all();
  CHECK(all_zero(), "ноль задан у всех: FL=%d FR=%d RL=%d RR=%d", M::zero_set, node_fr::zero_set,
        node_rl::zero_set, node_rr::zero_set);
  CHECK(std::labs(board_fl.phys) <= 1 && std::labs(board_fr.phys) <= 1 &&
        std::labs(board_rl.phys) <= 1 && std::labs(board_rr.phys - 500) <= 1,
        "колёса на нулях датчиков: %ld %ld %ld %ld", board_fl.phys, board_fr.phys, board_rl.phys,
        board_rr.phys);
  const std::string& out = board_fl.serial.output;
  for (const char* module : {"FL", "FR", "RL", "RR"}) {
    const std::string tag = std::string("@TLM mod=") + module + " deg=0.00 tgt=0.00 moving=0 cal=1 cycle=0 calib=0";
    CHECK(out.find(tag) != std::string::npos, "телеметрия %s после калибровки", module);
  }
  CHECK(has(board_fl, "home=opto err=0") && has(board_fl, "fw=3"), "поля home/err/fw в @TLM");
  CHECK(has(board_fl, "# FR: Калибровка ЗАВЕРШЕНА!") && has(board_fl, "# RR: Калибровка ЗАВЕРШЕНА!"),
        "мастер сообщает о калибровке слейвов");
  CHECK(has(board_fl, "# FL: Калибровка ЗАВЕРШЕНА!"), "и о своей");
  for (sim::Board* board : all_boards()) check_signals(*board);
}

static void scenario_system_server_commands() {
  calibrate_all();
  board_fl.serial.send("w FR 12.5");
  sim::run_for_ms(400);
  CHECK(board_fr.phys == 1667 && std::labs(board_fl.phys) <= 1 && std::labs(board_rl.phys) <= 1,
        "w FR 12.5 крутит только FR: FR=%ld FL=%ld RL=%ld", board_fr.phys, board_fl.phys, board_rl.phys);
  sim::run_for_ms(300);
  CHECK(has(board_fl, "@TLM mod=FR deg=12.50 tgt=12.50 moving=0 cal=1"), "телеметрия FR на 12.5°");
  board_fl.serial.send("w FL -7");
  sim::run_for_ms(300);
  CHECK(M::stepNow() == -933, "w FL -7 — локальный модуль: %ld", (long)M::stepNow());
  board_fl.serial.send("w RL zero");
  sim::run_for_ms(50);
  CHECK(node_rl::home_source == node_rl::HOME_MANUAL, "w RL zero — ручной ноль RL");
  board_fl.serial.send("w RR calib");
  sim::run_for_ms(5);
  CHECK(node_rr::calib_state != node_rr::CAL_IDLE, "w RR calib запускает калибровку RR");
  CHECK(wait_until([] { return node_rr::calib_state == node_rr::CAL_IDLE; }, 30000) &&
        node_rr::home_source == node_rr::HOME_OPTO, "RR откалиброван заново");
  board_fl.serial.send("w XX 5");
  board_fl.serial.send("w FL 100");
  sim::run_for_ms(20);
  CHECK(has(board_fl, "неизвестный модуль XX"), "неизвестный модуль");
  CHECK(has(board_fl, "# FL: w: угол вне ±90°"), "предел угла у сервера");
  // 90° доходят до слейва по I2C: боковой ход доступен не только локальному модулю.
  board_fl.serial.send("w FR 90");
  CHECK(wait_until([] { return !node_fr::motorBusy() && node_fr::stepNow() == 12000; }, 5000),
        "w FR 90 → слейв на 12000 шагов, сейчас %ld", (long)node_fr::stepNow());
  CHECK(board_fr.phys == 12000, "колесо FR на 90°: phys=%ld", board_fr.phys);
}

static void send_drive_heartbeat(int percent, int times) {
  char command[16];
  std::snprintf(command, sizeof(command), "t %d", percent);
  for (int i = 0; i < times; ++i) {
    board_fl.serial.send(command);
    sim::run_for_ms(200);
  }
}

static bool all_pwm(int value) {
  return board_fl.analog[3] == value && board_fr.analog[3] == value && board_rl.analog[3] == value &&
         board_rr.analog[3] == value;
}

static void scenario_system_drive() {
  calibrate_all();
  board_fl.serial.send("w RR 3.75");    // RR выравниваем по остальным (его ноль смещён флажком)
  sim::run_for_ms(300);
  board_fl.serial.send("w RR 0");
  sim::run_for_ms(300);
  send_drive_heartbeat(8, 3);
  CHECK(all_pwm(20), "t 8 — все четыре BLDC: %d %d %d %d", board_fl.analog[3], board_fr.analog[3],
        board_rl.analog[3], board_rr.analog[3]);
  board_fl.serial.send("b");
  sim::run_for_ms(10);
  CHECK(all_pwm(0) && board_fl.level[5] == HIGH && board_rr.level[5] == HIGH, "b — тормоз у всех");
  board_fl.serial.send("w FR 10");
  sim::run_for_ms(600);
  send_drive_heartbeat(8, 2);
  CHECK(all_pwm(0), "колёса не выровнены — тяга запрещена");
  board_fl.serial.send("w FR 0");
  sim::run_for_ms(600);
  send_drive_heartbeat(8, 1);
  CHECK(all_pwm(20), "выровняли — тяга снова разрешена");
  sim::run_for_ms(800);
  CHECK(all_pwm(0), "без heartbeat все останавливаются по watchdog");
}

static void scenario_system_slave_offline() {
  calibrate_all();
  board_rr.wire.connected = false;
  sim::run_for_ms(1500);
  CHECK(has(board_fl, "# RR: нет связи по I2C (адрес 4)"), "мастер сообщает о потере RR");
  // Пока RR не старше SLAVE_HOLD_MS, мастер держит его последние значения в
  // телеметрии: экран не мигает «то 4, то 2, то 1» от одиночной помехи.
  const size_t hold_mark = board_fl.serial.output.size();
  sim::run_for_ms(600);
  CHECK(board_fl.serial.output.substr(hold_mark).find("@TLM mod=RR") != std::string::npos,
        "RR удержан в телеметрии последние значения");
  // Старше SLAVE_HOLD_MS — строк быть не должно: сервер честно видит потерю.
  sim::run_for_ms(1600);
  const size_t mark = board_fl.serial.output.size();
  sim::run_for_ms(600);
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(tail.find("@TLM mod=RR") == std::string::npos, "строк RR больше нет: сервер видит «нет связи»");
  CHECK(tail.find("@TLM mod=FR") != std::string::npos, "остальные модули на связи");
  send_drive_heartbeat(8, 2);
  CHECK(all_pwm(0), "без RR тяга запрещена");
  board_fl.serial.send("d 10 f");
  sim::run_for_ms(20);
  CHECK(has(board_fl, "# RR: команда «d 10 f» не доставлена по I2C"), "недоставленная команда видна");
  board_rr.wire.connected = true;
  sim::run_for_ms(600);
  CHECK(count(board_fl, "# RR: на связи по I2C") >= 2, "RR вернулся на связь");
}

static void scenario_system_broadcast() {
  calibrate_all();
  board_fl.serial.send("d 10 f");
  sim::run_for_ms(5);
  CHECK(wait_until(all_idle, 2000), "d 10 f у всех закончилась");
  CHECK(M::stepNow() == 1333 && node_fr::stepNow() == 1333 && node_rl::stepNow() == 1333 &&
        node_rr::stepNow() == 1333, "одна команда — все четыре колеса +10°");
  board_fl.serial.send("d 90 b");
  sim::run_for_ms(200);
  board_fl.serial.send("s");
  sim::run_for_ms(5);
  const long fl = board_fl.phys;
  const long rr = board_rr.phys;
  sim::run_for_ms(300);
  CHECK(board_fl.phys == fl && board_rr.phys == rr, "s останавливает все колёса");
  board_fl.serial.send("h");
  sim::run_for_ms(5);
  CHECK(wait_until(all_idle, 3000), "h у всех закончилась");
  CHECK(M::stepNow() == 0 && node_fr::stepNow() == 0 && node_rl::stepNow() == 0 && node_rr::stepNow() == 0,
        "h — все в нуле");
  sim::run_for_ms(300);                  // мастер опросит слейвов
  board_fl.serial.send("p");
  sim::run_for_ms(5);
  CHECK(has(board_fl, "# RL: Текущий: 0.00° | Целевой: 0.00° [СТОИТ]"), "p показывает слейвов");
  board_fl.serial.send("pm 0");
  sim::run_for_ms(600);
  CHECK(!M::pot_enabled && !node_fr::pot_enabled && !node_rl::pot_enabled && !node_rr::pot_enabled,
        "pm 0 reaches all four boards");
  CHECK(has(board_fl, "pm=0"), "disabled mode is in telemetry");
  board_fl.serial.send("pm 1");
  sim::run_for_ms(600);
  CHECK(M::pot_enabled && node_fr::pot_enabled && node_rl::pot_enabled && node_rr::pot_enabled,
        "pm 1 reaches all four boards");
  CHECK(has(board_fl, "pm=1"), "enabled mode is in telemetry");
}

// Шина I2C: скан адресов, адрес платы в EEPROM и расклинка зависшей шины.
static void scenario_i2c_scan() {
  calibrate_all();
  const size_t mark = board_fl.serial.output.size();
  board_fl.serial.send("i2c");
  sim::run_for_ms(20);
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(tail.find("I2C 400 кГц: отвечают 2,3,4") != std::string::npos,
        "скан шины видит все три слейва: %s", tail.c_str());
  CHECK(tail.find("не ответил ни один слейв") == std::string::npos, "ложной тревоги нет");
}

static void scenario_i2c_missing_address() {
  // Плата RR не запущена: мастер обязан сказать, что ответили не все адреса,
  // а не молча показывать три колеса.
  boot({&board_fl, &board_fr, &board_rl});
  sim::run_for_ms(1500);
  CHECK(has(board_fl, "I2C: ответили не все адреса 2/3/4"),
        "диагностика неполной шины: %s", board_fl.serial.output.c_str());
}

static void scenario_node_address() {
  boot({&board_fr});
  CHECK(has(board_fr, "I2C 2 (по умолчанию)"), "адрес по умолчанию в приветствии");
  board_fr.serial.send("a");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "I2C адрес 2 (по умолчанию)"), "a показывает адрес и источник");
  board_fr.serial.send("a 7");
  sim::run_for_ms(5);
  CHECK(FR::node_address == 7 && FR::node_address_from_eeprom, "адрес сменён на 7");
  CHECK(sim::bus[7] == &board_fr.wire && sim::bus[2] == nullptr, "на шине только новый адрес");
  board_fr.serial.send("a 0");
  sim::run_for_ms(5);
  CHECK(FR::node_address == 7, "адрес 0 отвергнут");
  board_fr.serial.send("a 200");
  sim::run_for_ms(5);
  CHECK(FR::node_address == 7, "адрес 200 отвергнут");
  // Перезагрузка платы: адрес берётся из EEPROM, а не из NODE_ADDRESS.
  board_fr.serial.output.clear();
  FR::setup();
  sim::run_for_ms(10);
  CHECK(has(board_fr, "I2C 7 (EEPROM)"), "EEPROM пережил перезагрузку: %s",
        board_fr.serial.output.c_str());
  board_fr.serial.send("p");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "I2C адрес 7 (из EEPROM)"), "p печатает адрес");
}

static void scenario_i2c_duplicate_address() {
  // Две платы на одном адресе 2 — ровно то, что получается, если прошить все
  // три ноды одним файлом. Адрес ACK-ает, но кадры битые: мастер обязан
  // сказать об этом, а не просто «нет связи».
  boot(all_boards());
  sim::run_for_ms(600);
  board_rl.serial.send("a 2");
  sim::run_for_ms(5);
  CHECK(node_rl::node_address == 2 && sim::bus_claims[2] == 2, "две платы на адресе 2");
  const size_t mark = board_fl.serial.output.size();
  sim::run_for_ms(3000);
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(tail.find("адрес 2 отвечает, но кадр битый") != std::string::npos,
        "мастер распознал коллизию адресов: %s", tail.c_str());
  // RL держится в телеметрии до SLAVE_HOLD_MS (его последние значения ещё
  // свежие), а вот дальше — не числится: кадры-то битые.
  sim::run_for_ms(1000);
  const size_t after_hold = board_fl.serial.output.size();
  sim::run_for_ms(600);
  CHECK(board_fl.serial.output.substr(after_hold).find("@TLM mod=RL") == std::string::npos,
        "RL на связи не числится");
}

static void scenario_bus_recovery() {
  calibrate_all();
  CHECK(M::slave_online[0] && M::slave_online[1] && M::slave_online[2], "слейвы на связи");
  // Слейв удерживает SDA: все запросы мастера не проходят — ровно симптом
  // «виден только мастер».
  sim::i2c_bus_stuck = true;
  sim::i2c_scl_edges = 0;
  const size_t mark = board_fl.serial.output.size();
  sim::run_for_ms(3000);
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(tail.find("шина молчит") != std::string::npos, "мастер сообщил о расклинке: %s", tail.c_str());
  CHECK(!sim::i2c_bus_stuck, "9 тактов SCL освободили SDA");
  sim::run_for_ms(1500);
  CHECK(M::slave_online[0] && M::slave_online[1] && M::slave_online[2], "слейвы вернулись на шину");
}

// Счётчик кадров v3 — отдельное поле, а не биты 12–15 состояния. В v2 он
// делил их с режимом потенциометра (0x1000/0x2000): pm= мигал 0/1, и
// «Без потенциометра» в телеметрии выглядел то включённым, то выключенным.
static void scenario_i2c_frame_v3_status() {
  calibrate_all();
  board_fl.serial.send("pm 0");
  sim::run_for_ms(700);
  CHECK(!M::pot_enabled && !node_fr::pot_enabled && !node_rl::pot_enabled && !node_rr::pot_enabled,
        "pm 0 дошёл до всех четырёх плат");
  const size_t mark = board_fl.serial.output.size();
  sim::run_for_ms(2000);                 // восемь периодов телеметрии
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(tail.find("pm=0") != std::string::npos, "pm=0 в телеметрии: %s", tail.c_str());
  CHECK(tail.find("pm=1") == std::string::npos, "pm= не мигает: %s", tail.c_str());
  CHECK(tail.find("pm=-1") == std::string::npos, "режим всех плат подтверждён");
  CHECK(tail.find("fw=3") != std::string::npos, "fw=3 у новых кадров");
}

// Мастер понимает старый кадр v2 (5 полей), но не выдаёт его биты 0x1000/0x2000
// за режим потенциометра: там лежал счётчик кадров, восстановить режим нельзя.
static void scenario_i2c_frame_v2_compat() {
  boot({&board_fl});
  int32_t current = 0;
  int32_t target = 0;
  uint16_t status = 0;
  uint32_t rpm = 0;
  int8_t drive = 0;
  uint8_t seq = 0;
  uint8_t protocol = 0;
  char v2[] = "-133,133,8256,0,0";       // 8256 = 0x2040: V2 | POT_MODE_SUPPORTED
  CHECK(M::parseSlaveFrame(v2, &current, &target, &status, &rpm, &drive, &seq, &protocol),
        "старый кадр разбирается");
  CHECK(protocol == 2 && seq == 2, "fw=2: счётчик из битов 12–15, seq=%u", (unsigned)seq);
  CHECK((status & (M::STATUS_POT_MODE | M::STATUS_POT_MODE_SUPPORTED)) == 0,
        "биты режима из старого кадра не выдумываются");
  char v3[] = "-133,133,2040,0,0,7";
  CHECK(M::parseSlaveFrame(v3, &current, &target, &status, &rpm, &drive, &seq, &protocol),
        "новый кадр разбирается");
  CHECK(protocol == 3 && seq == 7 && (status & M::STATUS_POT_MODE_SUPPORTED),
        "fw=3: режим и счётчик не пересекаются");
  char bad[] = "-133,133,2040,0,0,16";
  CHECK(!M::parseSlaveFrame(bad, &current, &target, &status, &rpm, &drive, &seq, &protocol),
        "счётчик вне 0…15 отвергнут");
}

// Одиночная помеха на шине не должна выглядеть как потеря модуля: мастер
// повторяет битый кадр, а не убирает колесо из телеметрии на четверть секунды.
static void scenario_i2c_corrupt_frame_retry() {
  calibrate_all();
  board_rr.wire.corrupt_frames = 3;      // три битых кадра подряд
  const size_t mark = board_fl.serial.output.size();
  sim::run_for_ms(1500);
  CHECK(M::slave_online[2], "битые кадры повторены, RR на связи");
  CHECK(board_fl.serial.output.substr(mark).find("# RR: нет связи") == std::string::npos,
        "ложной потери связи в журнале нет");
}

// RR на 400 кГц не отвечает (длинный шлейф, только внутренние подтяжки), на
// 100 кГц работает. Раньше мастер уходил на 100 кГц, только если не ответил
// НИКТО: при «RR не виден, остальные иногда пропадают» он оставался на 400.
static void scenario_i2c_slow_bus_at_boot() {
  board_fl.phys = -3000;
  board_fr.phys = 2500;
  board_rl.phys = 40;
  board_rr.phys = -9000;
  board_rr.flag_lo = 300;
  board_rr.flag_hi = 700;
  board_rr.wire.slow_bus_only = true;
  boot(all_boards());
  sim::run_for_ms(2500);                 // скан при старте (+1 с) уходит на 100 кГц
  CHECK(M::i2c_clock_hz == 100000, "мастер сам перешёл на 100 кГц: %lu",
        (unsigned long)M::i2c_clock_hz);
  CHECK(has(board_fl, "работаю на 100 кГц") || has(board_fl, "остаюсь на 100 кГц"),
        "в журнале объяснён переход");
  board_fl.serial.send("k");
  sim::run_for_ms(5);
  CHECK(wait_until(all_idle, 30000), "калибровка всех колёс закончилась");
  sim::run_for_ms(600);
  CHECK(M::slave_online[2] && node_rr::zero_set, "RR на связи и откалиброван");
  CHECK(has(board_fl, "# RR: на связи по I2C"), "журнал: RR вернулся на шину");
}

// То же на ходу: RR работал, потом провод/подтяжки «поплыли» и на 400 кГц он
// пропал. Мастер обязан через три опроса попробовать 100 кГц и вернуть RR.
static void scenario_i2c_slow_bus_runtime() {
  calibrate_all();
  CHECK(M::i2c_clock_hz == 400000 && M::slave_online[2], "до сбоя RR на 400 кГц");
  board_rr.wire.slow_bus_only = true;
  sim::run_for_ms(5000);
  CHECK(M::i2c_clock_hz == 100000, "мастер перешёл на 100 кГц: %lu",
        (unsigned long)M::i2c_clock_hz);
  CHECK(has(board_fl, "остаюсь на 100 кГц"), "в журнале объяснён переход");
  CHECK(M::slave_online[2], "RR снова на связи");
  const size_t mark = board_fl.serial.output.size();
  sim::run_for_ms(600);
  CHECK(board_fl.serial.output.substr(mark).find("@TLM mod=RR") != std::string::npos,
        "RR снова в телеметрии");
}

// Телеметрия всех модулей без провалов: в каждом окне полсекунды должна быть
// хотя бы одна строка @TLM каждого модуля. Ровно это видит сервер: если строка
// пропала, через секунду модуль на экране «нет связи».
static void check_tlm_no_gaps(double total_ms) {
  const char* modules[4] = {"FL", "FR", "RL", "RR"};
  const int slices = (int)(total_ms / 500.0);
  for (int slice = 0; slice < slices; ++slice) {
    const size_t mark = board_fl.serial.output.size();
    sim::run_for_ms(500);
    const std::string window = board_fl.serial.output.substr(mark);
    for (const char* mod : modules) {
      CHECK(window.find(std::string("@TLM mod=") + mod) != std::string::npos,
            "без провалов @TLM: в окне %d нет %s", slice, mod);
    }
  }
}

// «То 4 модуля на связи, то 2, то 1, мастер всегда виден»: на 400 кГц кадры
// теряются через раз (маргинальный шлейф, только внутренние подтяжки), а
// счётчик пропусков «подряд» сбрасывается каждым удачным кадром — мастер так
// и сидел бы на 400 кГц, и модули мигали «нет связи» вечно. Теперь потери
// считаются окном последних опросов, мастер сам уходит с 400 кГц, а пока кадр
// не прочитан — держит модуль в телеметрии (SLAVE_HOLD_MS): экран не мигает.
static void scenario_i2c_flaky_bus() {
  board_fr.wire.loss_rate = 0.70;
  board_rl.wire.loss_rate = 0.72;
  board_rr.wire.loss_rate = 0.78;
  boot(all_boards());
  sim::run_for_ms(2000);                 // старт, первые опросы, скан шины
  const unsigned long started = millis();
  for (int i = 0; i < 24 && M::i2c_clock_hz >= 400000; ++i) sim::run_for_ms(250);
  CHECK(M::i2c_clock_hz < 400000,
        "мастер сам ушёл с 400 кГц при «плавающих» потерях: %lu кГц",
        (unsigned long)M::i2c_clock_hz);
  CHECK(millis() - started <= 6000, "переход по окну потерь занял меньше 6 с: %lu мс",
        (unsigned long)(millis() - started));
  CHECK(M::slave_online[0] && M::slave_online[1] && M::slave_online[2],
        "все три слейва на связи");
  check_tlm_no_gaps(10000);
}

// Пачки помех от моторов: шина «пропадает» на 1,2 с. Кадры не читаются, но
// мастер держит модуль в телеметрии (SLAVE_HOLD_MS) — экран не мигает «нет
// связи» от одиночной помехи; тяга при этом честно запрещается: slaveFresh
// смотрит на настоящий кадр, а не на удержание в телеметрии.
static void scenario_i2c_noise_bursts() {
  calibrate_all();
  board_rr.wire.burst_until_us = sim::now_us + 1200 * 1000;   // 1,2 с помех на RR
  const size_t mark = board_fl.serial.output.size();
  sim::run_for_ms(1100);                 // RR без кадров дольше SLAVE_STALE_MS
  board_fl.serial.send("t 8");
  sim::run_for_ms(20);
  CHECK(all_pwm(0), "тяга запрещена: RR не давал кадр дольше секунды");
  sim::run_for_ms(600);                  // окно помех закончилось, RR вернулся
  const std::string tail = board_fl.serial.output.substr(mark);
  int rr_lines = 0;
  for (size_t pos = tail.find("@TLM mod=RR"); pos != std::string::npos;
       pos = tail.find("@TLM mod=RR", pos + 1)) {
    ++rr_lines;
  }
  CHECK(rr_lines >= 4, "RR держится в телеметрии все 1,2 с помехи: %d строк", rr_lines);
  CHECK(tail.find("# RR: нет связи по I2C") != std::string::npos, "журнал пишет о потере");
  // Дальше — «моторные» помехи пачками на всех трёх модулях: телеметрия
  // обязана идти без провалов (удержание + повторы чтения).
  for (sim::Board* board : {&board_fr, &board_rl, &board_rr}) {
    board->wire.burst_rate = 0.02;
    board->wire.burst_ms = 1200;
  }
  check_tlm_no_gaps(10000);
}

// Дальний модуль не тянет даже 50 кГц — только самую медленную ступень.
// Мастер обязан пройти лестницу скоростей до конца, а не застрять на 100 кГц.
static void scenario_i2c_very_slow_bus() {
  board_rr.wire.max_ok_hz = 40000;       // держит 31,25 кГц и медленнее
  boot(all_boards());
  sim::run_for_ms(2500);
  CHECK(M::i2c_clock_hz == 31250, "мастер дошёл до 31,25 кГц: %lu",
        (unsigned long)M::i2c_clock_hz);
  CHECK(M::slave_online[0] && M::slave_online[1] && M::slave_online[2],
        "все три слейва на связи на медленной шине");
}

// Адрес платы мог остаться в EEPROM от прежней команды «a»: RR отвечает на 5,
// и мастер обязан не просто сказать «ответили не все», а назвать лишний адрес.
static void scenario_i2c_unexpected_address() {
  boot(all_boards());
  sim::run_for_ms(600);
  board_rr.serial.send("a 5");
  sim::run_for_ms(10);
  CHECK(sim::bus[5] == &board_rr.wire && sim::bus[4] == nullptr, "RR переехал на адрес 5");
  const size_t mark = board_fl.serial.output.size();
  board_fl.serial.send("i2c");
  sim::run_for_ms(50);
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(tail.find("лишний адрес 5") != std::string::npos, "лишний адрес назван: %s", tail.c_str());
  CHECK(tail.find("на её USB: a 4") != std::string::npos, "подсказан адрес RR: %s", tail.c_str());
  CHECK(tail.find("RR: НЕТ СВЯЗИ: адрес 4") != std::string::npos,
        "молчащий модуль назван по имени: %s", tail.c_str());
}

// Авария шины во время работы. Каждый запрос к Wire на удерживаемой SDA стоит
// полного таймаута (25 мс), а опрос трёх слейвов с повторами, пробы медленных
// ступеней и скан 1…119 адресов шли одним куском внутри loop(). На роботе это
// выглядело как «шина отвалилась»: @TLM замолкал на секунды, сервер гасил все
// колёса (STALE_S = 1 с), команды оператора висели в очереди, а watchdog тяги
// срабатывал не по отсутствию heartbeat, а потому что loop() не доходил до него.
static void scenario_bus_stall_keeps_the_loop_alive() {
  calibrate_all();
  send_drive_heartbeat(8, 2);
  CHECK(all_pwm(20), "тяга шла до аварии шины");
  board_fl.reset_stats();                  // самый долгий loop() меряем с аварии
  sim::i2c_bus_stuck = true;
  sim::i2c_scl_edges = 0;
  const size_t mark = board_fl.serial.output.size();
  sim::run_for_ms(3000);
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(board_fl.max_loop_us < 400000, "loop() не вставал дольше 400 мс: %lld мкс",
        (long long)board_fl.max_loop_us);
  // За 3 с при периоде 250 мс блоков @TLM должно быть около 12; сервер считает
  // модуль живым 1 с, поэтому меньше 6 — это уже «пропала связь» на экране.
  CHECK(count_since(board_fl, "@TLM mod=FL", mark) >= 6,
        "телеметрия во время аварии не замолкала: %ld блоков",
        count_since(board_fl, "@TLM mod=FL", mark));
  CHECK(!sim::i2c_bus_stuck, "шина расклинена");
  sim::run_for_ms(1500);
  CHECK(M::slave_online[0] && M::slave_online[1] && M::slave_online[2],
        "слейвы вернулись на шину после расклинки");
  send_drive_heartbeat(8, 2);
  CHECK(all_pwm(20), "после аварии тяга снова разрешена");
}

// «Крутится только одно колесо». ACK на шине означает лишь «байты до платы
// дошли»: слейв отказывается молча (ноль не задан, включена тяга, старая
// прошивка режет угол), а экран рапортовал «команда принята». Мастер обязан
// проверить выполнение по кадру и назвать причину.
static void scenario_slave_rejects_command() {
  calibrate_all();
  // RR потерял ноль (так выглядит и плата со старой прошивкой: угол не принят).
  node_rr::zero_set = false;
  node_rr::haltMotor();
  const size_t mark = board_fl.serial.output.size();
  board_fl.serial.send("w RR 90");
  board_fl.serial.send("w FR 90");
  sim::run_for_ms(2500);
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(node_fr::stepNow() == 12000, "FR принял 90°: %ld шагов", (long)node_fr::stepNow());
  CHECK(node_rr::stepNow() == 0, "RR не поехал: %ld шагов", (long)node_rr::stepNow());
  CHECK(tail.find("# RR: команда «w 90.0» не выполнена: ноль не задан") != std::string::npos,
        "мастер назвал причину отказа RR: %s", tail.c_str());
  CHECK(tail.find("# FR: команда «w 90.0» не выполнена") == std::string::npos,
        "про выполненную команду ложной тревоги нет: %s", tail.c_str());
  // Отказ по тяге: слейв отвечает «руль заблокирован», мастер это видит.
  board_fl.serial.send("w FR 0");
  sim::run_for_ms(1500);
  node_rr::zero_set = true;
  node_rr::drive_active = true;
  node_rr::drive_percent = 8;          // в кадре видна тяга — по ней и причина
  node_rr::last_drive_command_ms = millis();   // watchdog слейва её пока не снял
  const size_t busy_mark = board_fl.serial.output.size();
  board_fl.serial.send("w RR 45");
  sim::run_for_ms(2000);
  CHECK(board_fl.serial.output.substr(busy_mark).find("не выполнена: включена тяга") !=
            std::string::npos,
        "отказ из-за тяги объяснён: %s", board_fl.serial.output.c_str());
  node_rr::drive_active = false;
  // Команда принята — тишина в журнале: предупреждение не должно быть шумным.
  const size_t ok_mark = board_fl.serial.output.size();
  board_fl.serial.send("w RR 45");
  sim::run_for_ms(2000);
  CHECK(node_rr::stepNow() == 6000, "после устранения причины RR поехал: %ld", (long)node_rr::stepNow());
  CHECK(board_fl.serial.output.substr(ok_mark).find("не выполнена") == std::string::npos,
        "выполненная команда не сопровождается предупреждением");
}

// Пресет бортового экрана — это четыре команды подряд: FL, FR, RL, RR с
// интервалом в десятки миллисекунд (экран шлёт их по очереди, не дожидаясь
// окончания поворота). Проверяем, что мастер раздаёт всю пачку и что последнее
// в ней колесо не теряется: жалоба «RR всегда ничего не исполняет» означала бы,
// что четвёртая команда до слейва не доходит.
static void scenario_preset_burst() {
  calibrate_all();
  const long rr_zero = (long)node_rr::stepNow();
  const size_t mark = board_fl.serial.output.size();
  board_fl.serial.send("w FL 90");
  sim::run_for_ms(15);
  board_fl.serial.send("w FR 90");
  sim::run_for_ms(15);
  board_fl.serial.send("w RL 90");
  sim::run_for_ms(15);
  board_fl.serial.send("w RR 90");
  CHECK(wait_until(all_idle, 20000), "все четыре колеса доехали и остановились");
  CHECK(M::stepNow() == 12000, "FL: 90° — это 12000 шагов, а не %ld", (long)M::stepNow());
  CHECK(node_fr::stepNow() == 12000, "FR: %ld шагов", (long)node_fr::stepNow());
  CHECK(node_rl::stepNow() == 12000, "RL: %ld шагов", (long)node_rl::stepNow());
  CHECK((long)node_rr::stepNow() == rr_zero + 12000,
        "RR — последний в пачке — доехал: %ld шагов от нуля %ld",
        (long)node_rr::stepNow(), rr_zero);
  // Цели в кадрах слейвов (0 = FR, 1 = RL, 2 = RR) — те самые 90°, иначе экран
  // не увидел бы выполненной команды и держал бы колесо заблокированным.
  CHECK(M::slave_target[0] == 12000 && M::slave_target[1] == 12000 &&
            M::slave_target[2] == 12000,
        "цели слейвов в телеметрии: %ld %ld %ld", (long)M::slave_target[0],
        (long)M::slave_target[1], (long)M::slave_target[2]);
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(tail.find("не выполнена") == std::string::npos,
        "ни одно колесо пачки не отказало: %s", tail.c_str());
}

// Счётчик кадров 4-битный, а кадр слейв обновляет каждые 50 мс: опрос,
// пришедшийся ровно на 16 кадров позже (800 мс — так бывает вокруг диагностики
// шины), видит тот же номер. По одному номеру полностью живой модуль
// объявлялся потерянным — ещё один источник «то 4 колеса, то 1».
static void scenario_i2c_frame_freshness() {
  calibrate_all();
  CHECK(M::slave_online[0] && M::slave_online[2], "слейвы на связи");
  CHECK(M::slaveFrameChanged(0, M::slave_current[0], M::slave_target[0] + 133,
                             M::slave_status[0], M::slave_rpm_x10[0], M::slave_drive[0],
                             M::slave_seq[0]),
        "сменилась цель при том же счётчике — кадр живой");
  CHECK(M::slaveFrameChanged(2, M::slave_current[2], M::slave_target[2],
                             (uint16_t)(M::slave_status[2] ^ M::STATUS_MOVING),
                             M::slave_rpm_x10[2], M::slave_drive[2], M::slave_seq[2]),
        "сменилось состояние при том же счётчике — кадр живой");
  // Ничего не изменилось: так отвечает застывший слейв (loop() встал, а TWI
  // отдаёт старый буфер) — его по-прежнему не считаем живым.
  CHECK(!M::slaveFrameChanged(0, M::slave_current[0], M::slave_target[0],
                              M::slave_status[0], M::slave_rpm_x10[0], M::slave_drive[0],
                              M::slave_seq[0]),
        "застывший кадр живым не считается");
}

static void scenario_tlm_dump() {
  calibrate_all();
  board_fl.serial.send("q");
  sim::run_for_ms(5);
  CHECK(wait_until(all_pot_idle, 30000), "калибровка потенциометров всех колёс");
  sim::run_for_ms(600);                  // мастер опросит слейвов: pot=1 в кадрах
  board_fl.serial.send("w FR 12.5");
  sim::run_for_ms(700);
  std::fwrite(board_fl.serial.output.data(), 1, board_fl.serial.output.size(), stdout);
}

// ./sim pty /tmp/master-link — мастер и три слейва в реальном времени за
// псевдотерминалом: сервер робота (RC_SLAM_PORT=/tmp/master-link) видит настоящую
// логику прошивки — для ручной проверки экрана без железа.
static int run_pty(const char* link_path) {
  const int fd = posix_openpt(O_RDWR | O_NOCTTY);
  if (fd < 0 || grantpt(fd) != 0 || unlockpt(fd) != 0) {
    std::perror("posix_openpt");
    return 2;
  }
  // Сторона робота сразу в raw-режиме: без эха телеметрии обратно в «мастер».
  const int slave = open(ptsname(fd), O_RDWR | O_NOCTTY);
  if (slave >= 0) {
    termios mode;
    if (tcgetattr(slave, &mode) == 0) {
      cfmakeraw(&mode);
      tcsetattr(slave, TCSANOW, &mode);
    }
    close(slave);
  }
  unlink(link_path);
  if (symlink(ptsname(fd), link_path) != 0) {
    std::perror("symlink");
    return 2;
  }
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
  wire_boards();
  board_fl.phys = -3000;
  board_fr.phys = 2500;
  board_rl.phys = 40;
  board_rr.phys = -9000;
  boot(all_boards());
  std::printf("мастер на %s → %s\n", link_path, ptsname(fd));
  std::fflush(stdout);
  size_t sent = 0;
  const auto wall_start = std::chrono::steady_clock::now();
  const int64_t sim_start = sim::now_us;
  for (;;) {
    char buffer[256];
    const ssize_t got = read(fd, buffer, sizeof(buffer));
    for (ssize_t i = 0; i < got; ++i) board_fl.serial.input.push_back((uint8_t)buffer[i]);
    sim::run_for_ms(10);
    const std::string& out = board_fl.serial.output;
    if (out.size() > sent) {
      const ssize_t written = write(fd, out.data() + sent, out.size() - sent);
      if (written > 0) sent += (size_t)written;
    }
    std::this_thread::sleep_until(wall_start + std::chrono::microseconds(sim::now_us - sim_start));
  }
}

struct Scenario {
  const char* name;
  void (*fn)();
};

static const Scenario SCENARIOS[] = {
    {"boot", scenario_boot},
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
    {"frame_limits", scenario_frame_limits},
    {"serial_garbage", scenario_serial_garbage},
    {"system_calibrate_all", scenario_system_calibrate_all},
    {"system_server_commands", scenario_system_server_commands},
    {"system_drive", scenario_system_drive},
    {"system_slave_offline", scenario_system_slave_offline},
    {"system_broadcast", scenario_system_broadcast},
    {"i2c_scan", scenario_i2c_scan},
    {"i2c_missing_address", scenario_i2c_missing_address},
    {"node_address", scenario_node_address},
    {"bus_recovery", scenario_bus_recovery},
    {"i2c_duplicate_address", scenario_i2c_duplicate_address},
    {"i2c_frame_v3_status", scenario_i2c_frame_v3_status},
    {"i2c_frame_v2_compat", scenario_i2c_frame_v2_compat},
    {"i2c_corrupt_frame_retry", scenario_i2c_corrupt_frame_retry},
    {"i2c_slow_bus_at_boot", scenario_i2c_slow_bus_at_boot},
    {"i2c_slow_bus_runtime", scenario_i2c_slow_bus_runtime},
    {"i2c_flaky_bus", scenario_i2c_flaky_bus},
    {"i2c_noise_bursts", scenario_i2c_noise_bursts},
    {"i2c_very_slow_bus", scenario_i2c_very_slow_bus},
    {"i2c_unexpected_address", scenario_i2c_unexpected_address},
    {"bus_stall_loop_alive", scenario_bus_stall_keeps_the_loop_alive},
    {"slave_rejects_command", scenario_slave_rejects_command},
    {"i2c_frame_freshness", scenario_i2c_frame_freshness},
    {"preset_burst", scenario_preset_burst},
};

static int run_one(const Scenario& scenario) {
  wire_boards();
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
    wire_boards();
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
