# Аудит «полного нуля» — 2026-11-16

## Что проверяли
- `robot_control/full_zero.py` — автомат фаз, журнал, билеты движения
- `slam_gui/backend.py` → `MasterSource` — 4 платы, CRC, роли, журнал, тормоза
- `robot_control/kiosk_app.py` — `api/kiosk_full_zero`, билеты `ticket()`, `motion_gate`
- `slam_gui/full_zero.js` — модалка, токены, опрос 400 мс, причины/подсказки
- `tests/test_full_zero.py` (31 тест) — оптика, журнал, EEPROM, таймауты, стенд

## Как должен работать (коротко)
1. **Билет** `ticket()` + `motion_gate` резервируют все пути движения.
2. `begin_full_zero()` → новый `brakeId` на 4 платах (fw23), иначе `stopping`.
3. `fault` → `reset_fault` только если ни одно колесо не `homed`; журнал `data/full-zero.pending` пишется **до** `zr`.
4. `calibrate` → 4× опто-калибровки, затем `adjust` → точная подстройка ±20°.
5. `save` → `zq` (preflight) → journal → `zc` (commit) → 4×ACK status 2, проверка `stored==proposed`.
6. При обрыве — `pending` остаётся, тяга заблокирована до полного цикла с 4 платами.

## Странности, найденные и исправленные

### 1) Пульт (8080) не открывал мастер
- **Было:** `full_zero.js` говорил только с `api/kiosk_full_zero` (киоск 8090). На `/console` это 404, оператор видел «соединение потеряно».
- **Становилось странно:** пульт показывал «движение разрешено», хотя киоск уже в `full_zero_active`.
- **Исправлено:** `full_zero.js` → авто-выбор `api/control/full_zero` на `/console`, fallback на второй эндпоинт, проброс `X-CSRF-Token` для прокси `map_peer.CONTROL_ROUTES: full_zero`.

### 2) Закрытие мастера по клику мимо
- **Было:** клик по фону не закрывал, `hidden` без `display:flex` оставлял оверлей.
- **Исправлено:** клик по backdrop закрывает с `cancelDetached`, наблюдатель `MutationObserver` синхронизирует `hidden ↔ display`.

### 3) Кнопка мастера отсутствовала на пульте
- **Было:** только на киоске `/kiosk` (`robot.html#full-zero-open`). Дистанционный оператор не мог запустить калибровку.
- **Исправлено:** секция «Калибровка шасси» в `slam_gui/index.html#view-drive` с кнопкой, бейджем 4 модуля, подсказкой журнала.

### 4) Дизайн пульта мешал важным кнопкам
- **Было:** узкий `200px` сайдбар, мелкие метрики, плоская карта, телеоп без акцентов.
- **Исправлено:** `styles.css` v2 — переменные, градиенты, карточки 16px, sticky шапка, E-STOP красный pill, метрики с верхними акцентами, cockpit `1fr 320px`, модалки blur, тёмная тема.

### 5) Журнал — узкое место при сбое питания
- **Проверено:** `write_pending` делает `mkdir → fsync(parent)` для каждой новой директории, `replace → fsync(parent)`, `clear_pending → fsync(parent)`. При `OSError` на `fsync` журнал может остаться, но `ticket()[1]` остаётся `True` → тяга заблокирована — fail-closed. Тест `test_real_owner_fsync_failures_leave_in_memory_interlock` подтверждает.
- **Рекомендация:** на борту держать `data/` на ext4 с журналом, не на FAT.

### 6) Потенциальная гонка `motion_gate` + навигация
- **Проверено:** `navigation.manual_override(lambda: full_zero.handle(begin))` останавливает маршрут до резервации тормозов. `before_request` с `zero_epoch` ловит смену билета во время ожидания `motion_gate`. Дублирующая проверка `zero_locked` избыточна, но безвредна.

## Что остаётся «по дизайну» (не баг)
- Повторный `+0.1°` ждёт свежий `target/angle` + `status 1`; старое эхо не засчитывается.
- `RESET_TIMEOUT_S 8c`, `MOVE_TIMEOUT_S 10c`, `CAL_TIMEOUT_S 125c` — жёстко, чтобы не висеть.
- `E-STOP` в мастере — софтовый, нужен физический.

## Проверки
- `pytest tests/test_full_zero.py -q` — 31 passed
- Ручная проверка: открыть `/` → «Калибровка полного нуля» → `braking` → `ready` → `calibrating` → `adjust` → `save` → `saving` → `saved` (4×ACK), при обрыве — `recovery` + `pendingRecovery`.
