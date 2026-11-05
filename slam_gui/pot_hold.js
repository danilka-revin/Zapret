/* Активное удержание угла по потенциометру (команда прошивки ph 0|1).
 *
 * Один переключатель на два экрана: бортовой (.steer-calib-bar, robot.js) и
 * веб-пульт (.rc-steering-toolbar, steering.js). Состояние берётся ТОЛЬКО из
 * телеметрии potHold всех четырёх плат: оптимистичного «щёлкнули — стало
 * включено» здесь нет, иначе экран показывал бы удержание, которого плата не
 * подтвердила. Максимум — «ждём подтверждения» с тем же сроком, что у режима
 * потенциометра (5 с).
 *
 * Почему отдельная команда, а не переключатель режима pm: pm сбрасывает
 * калибровку датчика (q), поэтому частые переключения «доводка вкл/выкл»
 * каждый раз требовали бы новой калибровки.
 */
(function () {
  'use strict';

  const MODULES = ['FL', 'FR', 'RL', 'RR'];
  const CONFIRM_WAIT_MS = 5000;

  const errors = {
    firmware_update_required: 'Обновите прошивки всех четырёх плат: удержание появилось в fw24.',
    module_offline: 'Нужна связь со всеми четырьмя платами.',
    module_busy: 'Сначала остановите движение и дождитесь конца калибровки.',
    pot_mode_off: 'Сначала включите режим с потенциометром.',
    pot_not_calibrated: 'Колесо не откалибровано по потенциометру или нет сигнала A0: нужны ноль руля и q.',
    pot_angle_unavailable: 'Нет достоверного угла A0: проверьте датчик и калибровку.',
    pot_hold_far: 'Отклонение датчика от цели больше 15°: проверьте положение и причину расхождения.',
    keyboard_active: 'Выйдите из режима WASD и дождитесь остановки.',
    serial_unavailable: 'Нет связи с платами.',
    pin_required: 'Разблокируйте управление PIN-кодом.',
    estop: 'Включён E-STOP.',
    neutral_locked: 'Сначала верните и подтвердите тормоз.',
    control_not_supported: 'Удержание недоступно для этого источника управления.',
    invalid_action: 'Непонятная команда удержания.',
    payload: 'Непонятная команда удержания.',
    command_queue_full: 'Очередь команд переполнена — повторите.',
    full_zero_active: 'Идёт настройка полного нуля — дождитесь конца.',
    full_zero_eeprom_fault: 'Ошибка памяти полного нуля — удержание заблокировано.',
    navigation_stopping: 'Останавливается автономный маршрут — повторите.'
  };

  let host = null;
  let button = null;
  let status = null;
  let alertBox = null;
  let hint = null;
  let lastState = null;     // последний снимок телеметрии: нужен нажатию
  let lastAuthorized = false;
  let pending = false;      // запрос в пути
  let desired = null;       // 'on' | 'off' — чего ждём от телеметрии
  let deadline = 0;

  function make(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text != null) node.textContent = text;
    return node;
  }

  function build(parent) {
    host = make('section', 'calibration-card pot-hold');
    host.setAttribute('aria-labelledby', 'pot-hold-title');
    const heading = make('div', 'calibration-card__head');
    const icon = make('span', 'calibration-card__icon', '↺');
    icon.setAttribute('aria-hidden', 'true');
    const headingText = make('div');
    const eyebrow = make('span', 'calibration-card__eyebrow', 'УДЕРЖАНИЕ УГЛА');
    const title = make('h5', '', 'Активная доводка');
    title.setAttribute('id', 'pot-hold-title');
    headingText.appendChild(eyebrow);
    headingText.appendChild(title);
    heading.appendChild(icon);
    heading.appendChild(headingText);
    button = make('button', 'pot-hold__switch', 'Активная доводка — выкл');
    button.type = 'button';
    button.setAttribute('role', 'switch');
    button.setAttribute('aria-checked', 'false');
    hint = make('small', 'pot-hold__hint',
      'колесо само возвращается к заданному углу; после перезапуска платы — выключено; ' +
      'ручной сдвиг больше 15° отключает удержание');
    status = make('span', 'calibration-status pot-hold__status', 'Нет подтверждения плат');
    status.setAttribute('role', 'status');
    alertBox = make('span', 'calibration-alert pot-hold__alert', '');
    alertBox.setAttribute('role', 'alert');
    button.addEventListener('click', () => { void toggle(); });
    // Клавиатура: Enter и пробел работают как нажатие. preventDefault не даёт
    // браузеру продублировать его ещё и синтетическим click — иначе на одно
    // нажатие ушло бы две команды.
    button.addEventListener('keydown', (event) => {
      const key = event && event.key;
      if (key !== 'Enter' && key !== ' ' && key !== 'Spacebar') return;
      if (event.preventDefault) event.preventDefault();
      void toggle();
    });
    host.appendChild(heading);
    host.appendChild(hint);
    host.appendChild(button);
    host.appendChild(status);
    host.appendChild(alertBox);
    parent.appendChild(host);
  }

  function motorsOf(state) {
    return (state && Array.isArray(state.motors)) ? state.motors : [];
  }

  function facts(state, authorized) {
    const motors = motorsOf(state);
    const online = MODULES.map((id) => motors.find((m) => m && m.id === id)).filter(Boolean)
      .filter((m) => m.online);
    // Прошивка fw24+ присылает potHold булевым. Поле отсутствует — плата его
    // не знает: это «нужна прошивка», а не «выключено».
    const confirmed = online.length === MODULES.length &&
      online.every((m) => typeof m.potHold === 'boolean');
    const mode = !confirmed ? 'unknown'
      : online.every((m) => m.potHold === true) ? 'on'
        : online.every((m) => m.potHold === false) ? 'off' : 'mixed';
    // Без датчика (pm 0 или нет q) удержание невозможно — переключатель
    // недоступен, а не «тихо включён».
    const sensorProblems = online.filter((m) => m.pot !== true || m.potMode === false || m.homed !== true ||
      (m.firmware >= 26 && (m.fullZeroFault !== false ||
        typeof m.potAngle !== 'number' || !Number.isFinite(m.potAngle))));
    const sensorReady = online.length === MODULES.length && sensorProblems.length === 0;
    const zeroFault = online.filter((m) => m.fullZeroFault === true);
    const live = online.filter((m) => Number(m.liveId) > 0);
    // The fw26 measurement is diagnostic and only blocks an unsafe enable;
    // it never commands a correction or replaces the controller's step count.
    const far = online.filter((m) => m.firmware >= 26 && m.pot === true &&
      typeof m.potAngle === 'number' && Number.isFinite(m.potAngle) &&
      typeof m.targetAngle === 'number' && Number.isFinite(m.targetAngle) &&
      Math.abs(m.potAngle - m.targetAngle) > 15);
    const busy = online.some((m) => m.moving || m.calibrating || m.cycling || m.drivePercent);
    const estop = !!(state && state.estop && (state.estop.latched || state.estop.unknown));
    const neutral = !!(state && state.neutral && state.neutral.locked);
    return { motors, online, confirmed, mode, sensorReady, sensorProblems, zeroFault, live, far, busy, estop, neutral, authorized };
  }

  function label(info) {
    if (!info.confirmed) return 'Активная доводка — нет подтверждения';
    if (info.mode === 'on') return 'Активная доводка — вкл';
    if (info.mode === 'off') return 'Активная доводка — выкл';
    return 'Активная доводка — платы расходятся';
  }

  function statusText(info) {
    if (desired) return 'Ждём подтверждения плат…';
    if (!info.authorized) return 'Разблокируйте управление PIN-кодом.';
    if (info.estop) return 'E-STOP: удержание недоступно.';
    if (info.neutral) return 'Сначала верните и подтвердите тормоз.';
    if (!info.confirmed) {
      if (info.online.length < MODULES.length) return 'Нужна связь со всеми четырьмя платами.';
      return 'Нужна прошивка fw24: платы не сообщают состояние удержания.';
    }
    const sensorIds = info.sensorProblems.map(m => m.id).join(', ');
    const problem = info.zeroFault.length ? 'ошибка памяти нуля: ' + info.zeroFault.map(m => m.id).join(', ')
      : sensorIds ? 'датчик не готов: ' + sensorIds : '';
    if (info.mode === 'mixed') {
      return 'Платы расходятся' + (problem ? '; ' + problem : '') +
        '. Выключите удержание и проверьте причину.';
    }
    // Older firmware may retain ph=1 while A0 is unavailable. Show the
    // reported switch state, but never claim that correction is healthy.
    if (info.mode === 'on' && problem) {
      return 'Платы сообщают «вкл», но ' + problem + '. Выключите удержание и проверьте причину.';
    }
    // Пока удержание само подтягивает угол, «идёт движение» — это его работа,
    // а не причина запретить выключатель.
    if (info.mode === 'on') {
      return info.busy ? 'Подтверждено: подтягивает угол…'
        : 'Подтверждено: колесо само возвращается к заданному углу.';
    }
    if (info.busy) return 'Идёт движение или калибровка — сначала остановите.';
    if (info.zeroFault.length) return 'Ошибка памяти полного нуля: ' + info.zeroFault.map(m => m.id).join(', ') + '.';
    if (info.live.length) return 'Выйдите из WASD и дождитесь остановки: ' + info.live.map(m => m.id).join(', ') + '.';
    if (!info.sensorReady) {
      const ids = info.sensorProblems.map(m => m.id).join(', ');
      return 'Датчик потенциометра не готов' + (ids ? ': ' + ids : '') + '. Проверьте ноль, q, режим и сигнал A0.';
    }
    if (info.far.length) {
      return 'Отклонение от цели больше 15°: ' + info.far.map(m => m.id).join(', ') + '. Проверьте положение и датчик.';
    }
    return 'Подтверждено: удержание выключено; калибровка q не включает его автоматически.';
  }

  function disabledFor(info) {
    if (pending || !!desired) return true;          // ждём подтверждения плат
    if (!info.authorized) return true;
    if (info.estop || info.neutral) return true;
    if (!info.confirmed) return true;
    // Выключить можно всегда: пока удержание подтягивает угол, колесо занято, и
    // запрет «идёт движение» оставил бы оператора без выключателя. Расхождение
    // плат лечится тем же выключением.
    if (info.mode !== 'off') return false;
    if (info.busy || info.live.length || info.far.length) return true;
    if (!info.sensorReady || !info.motors.every((m) => m.online)) return true;
    return false;
  }

  window.RCPotHold = {
    render(parent, state, authorized) {
      if (!parent) return;
      if (!host) build(parent);
      else if (host.parentElement !== parent) parent.appendChild(host);
      lastState = state;
      lastAuthorized = !!authorized;
      const info = facts(state, !!authorized);
      // Подтверждение пришло — ожидание закончено.
      if (desired) {
        const target = desired === 'on';
        if ((target && info.mode === 'on') || (!target && info.mode === 'off')) {
          desired = null;
          alertBox.textContent = '';
        } else if (Date.now() > deadline) {
          const missing = MODULES.filter(id => {
            const motor = info.motors.find(m => m && m.id === id);
            return !motor || !motor.online || motor.potHold !== target;
          });
          desired = null;
          alertBox.textContent = 'Платы не подтвердили: ' + missing.join(', ') +
            '. Проверьте связь, сигнал A0, отклонение от цели и журнал плат.' +
            (info.mode === 'mixed' ? ' Часть плат включена — выключите удержание.' : '');
        }
      }
      button.disabled = disabledFor(info);
      if (host.getAttribute('data-mode') !== info.mode) host.setAttribute('data-mode', info.mode);
      if (button.getAttribute('data-mode') !== info.mode) button.setAttribute('data-mode', info.mode);
      if (info.mode === 'on' || info.mode === 'off') {
        const checked = info.mode === 'on' ? 'true' : 'false';
        if (button.getAttribute('role') !== 'switch') button.setAttribute('role', 'switch');
        if (button.getAttribute('aria-checked') !== checked) button.setAttribute('aria-checked', checked);
      } else {
        // A switch only supports true/false: unknown or conflicting boards are
        // an ordinary button, not an accessible claim that holding is off.
        button.removeAttribute('role');
        button.removeAttribute('aria-checked');
      }
      const nextLabel = label(info);
      if (button.getAttribute('aria-label') !== nextLabel) button.setAttribute('aria-label', nextLabel);
      if (button.textContent !== nextLabel) button.textContent = nextLabel;
      hint.hidden = false;
      const nextStatus = statusText(info);
      if (status.textContent !== nextStatus) status.textContent = nextStatus;
    },
    // Для проверок и для экранов, которым нужен только факт.
    state() {
      return { pending, desired };
    }
  };

  async function toggle() {
    if (!button || button.disabled) return;
    const info = facts(lastState, lastAuthorized);
    // Только когда все платы говорят «выключено», нажатие включает; любое
    // другое состояние (включено или расходятся) нажатие снимает.
    const action = info.mode === 'off' ? 'on' : 'off';
    if (action === 'on') {
      const agreed = typeof window.confirm === 'function'
        ? window.confirm('Активная доводка: колесо будет само возвращаться к заданному углу ' +
          'и сопротивляться ручному сдвигу до 15°. Включить?')
        : true;
      if (!agreed) return;
    }
    await send(action);
  }

  async function send(action) {
    pending = true;
    alertBox.textContent = '';
    button.disabled = true;
    try {
      const response = await fetch('api/kiosk_pot_hold', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
        body: JSON.stringify({ action })
      });
      const result = await response.json().catch(() => ({}));
      if (!response.ok || !result.ok) {
        const detail = Array.isArray(result.modules) && result.modules.length
          ? ' Колёса: ' + result.modules.join(', ') + '.' : '';
        throw new Error((errors[result.reason] || ('Команда отклонена: ' + (result.reason || response.status))) + detail);
      }
      // Оптимистичного переключения нет: состояние подтвердит только телеметрия.
      desired = action;
      deadline = Date.now() + CONFIRM_WAIT_MS;
      status.textContent = 'Ждём подтверждения плат…';
    } catch (error) {
      alertBox.textContent = (error && error.message) ? error.message : 'Ошибка связи';
    } finally {
      pending = false;
    }
  }
})();
