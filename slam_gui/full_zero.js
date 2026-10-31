/* The onboard owner holds the motion lease; the browser never talks to USB. */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const modal = $('full-zero-modal');
  const open = $('full-zero-open');
  if (!modal || !open) return;
  const ids = ['FL', 'FR', 'RL', 'RR'];
  const calibrate = $('full-zero-calibrate');
  const save = $('full-zero-save');
  const retry = $('full-zero-retry');
  const message = $('full-zero-message');
  const adjustButtons = Array.from(modal.querySelectorAll('[data-full-zero-adjust]'));
  let token = null;
  let phase = 'opening';
  let last = {};
  let busy = false;
  let timer = null;
  let previousFocus = null;
  let errorText = '';
  let generation = 0;

  function cancelDetached(session) {
    if (!session) return;
    void fetch('api/kiosk_full_zero', {method: 'POST', keepalive: true,
      headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'steering'},
      body: JSON.stringify({action: 'cancel', token: session})}).catch(() => {});
  }

  const reasons = {
    estop: 'Включён E-STOP. Проверьте робота и снимите блокировку по PIN.',
    neutral_locked: 'Сначала выключите нейтраль и подтвердите тормоз.',
    full_zero_active: 'Настройка уже выполняется в другом окне.',
    control_not_supported: 'Бортовой источник не поддерживает настройку полного нуля.',
    serial_unavailable: 'Нет связи с четырьмя платами колёс.',
    module_offline: 'Нет свежей связи с каждым из четырёх колёс.',
    firmware_update_required: 'На все четыре платы нужна прошивка fw23 или новее.',
    wheel_moving: 'Колёса ещё вращаются. Дождитесь остановки и проверьте SC.',
    module_busy: 'Плата сообщает о движении. Остановите тягу и руль.',
    sc_invalid: 'Нет достоверной телеметрии SC одного или нескольких колёс.',
    invalid_telemetry: 'Нет полного и достоверного состояния всех четырёх плат.',
    brake_unconfirmed: 'Не все платы подтвердили новую команду тормоза.',
    brake_changed: 'Тормоз изменился или плата перезапустилась. Настройка остановлена.',
    calibration_failed: 'Оптокалибровка завершилась ошибкой. Проверьте датчики и ход колёс.',
    calibration_unconfirmed: 'Не удалось подтвердить запуск и завершение калибровки всех колёс.',
    optical_zero_required: 'Нужна успешно завершённая оптокалибровка; ручной ноль не подходит.',
    steering_unconfirmed: 'Поворот не подтверждён телеметрией. Настройка остановлена.',
    offset_limit: 'Поправка от оптодатчика превышает ±20°. Проверьте положение флажка и механики.',
    eeprom_error: 'Плата сообщила об ошибке памяти или несовпадении роли. Проверьте плату.',
    full_zero_eeprom_fault: 'Ошибка памяти полного нуля на одной из плат: проверьте все платы.',
    preflight_unconfirmed: 'Не все платы подтвердили проверку. Новая запись в этом запросе не начиналась.',
    partial_save: 'Запись подтверждена не всеми платами. Движение заблокировано; повторите мастер со всеми четырьмя платами.',
    journal_unavailable: 'Не удалось надёжно записать журнал на бортовой компьютер. Движение может оставаться заблокированным.',
    operator_timeout: 'Связь с окном потеряна. Настройка остановлена.',
    navigation_active: 'Сначала остановите автономную навигацию.',
    navigation_stopping: 'Дождитесь завершения остановки навигации.',
    invalid_phase: 'Дождитесь завершения текущего действия.',
    session_expired: 'Сессия завершилась. Начните настройку заново.',
    cancelled: 'Настройка отменена. Сохранённые ранее значения не удалены.',
    stop: 'Настройка остановлена командой СТОП.',
    command_queue_full: 'Очередь команд занята. Повторите после остановки.',
    internal_error: 'Внутренняя ошибка: настройка остановлена, проверьте связь и тормоз.'
  };
  const messages = {
    opening: 'Останавливаем предыдущие команды и проверяем связь…',
    braking: 'Ожидаем свежего подтверждения тормоза от четырёх плат…',
    ready: 'Тормоз подтверждён. Убедитесь, что рядом никого нет, и запустите оптокалибровку.',
    calibrating: 'Идёт поиск флажков и подход к сохранённому нулю. Не приближайтесь к колёсам.',
    adjust: 'Подправьте каждое колесо по физической оси корпуса, затем сохраните все четыре.',
    adjusting: 'Поворачиваем колесо. Дождитесь подтверждения положения…',
    checking: 'Проверяем опорный ноль, остановку и допустимость поправок на каждой плате…',
    saving: 'Записываем память плат. Не выключайте питание и не отключайте USB.',
    stopping: 'Останавливаем рулевые модули и ждём подтверждения тормоза…',
    saved: 'Сохранено: все четыре платы подтвердили запись в EEPROM. После следующей оптокалибровки они вернутся в этот ноль.',
    idle: 'Настройка остановлена.',
    recovery: 'Запись могла выполниться частично. Движение заблокировано: подключите все платы и повторите полную настройку.',
    connection: 'Нет ответа бортового сервера. Не считайте запись или остановку подтверждённой.'
  };

  function render() {
    const data = last || {};
    const saved = phase === 'saved';
    calibrate.disabled = busy || phase !== 'ready';
    save.disabled = busy || phase !== 'adjust';
    adjustButtons.forEach((button) => { button.disabled = busy || phase !== 'adjust'; });
    retry.hidden = !(['idle', 'recovery', 'failed'].includes(phase) || (phase === 'connection' && !token));
    retry.disabled = busy;
    $('full-zero-cancel').textContent = saved || ['idle', 'recovery', 'failed'].includes(phase)
      ? 'Закрыть' : 'СТОП и отменить';
    const issue = errorText || (data.reason && reasons[data.reason]) || '';
    const recovery = data.pendingRecovery && phase !== 'saved'
      ? 'Внимание: предыдущее сохранение могло быть частичным, движение заблокировано. ' : '';
    message.textContent = recovery + (issue || messages[phase] || 'Проверяем состояние…');
    message.dataset.tone = saved && !issue ? 'success' : (issue || phase === 'recovery' || phase === 'connection' ? 'error' : 'normal');
    const seen = Array.isArray(data.calibratingSeen) ? data.calibratingSeen : [];
    const acknowledged = Array.isArray(data.acknowledged) ? data.acknowledged : [];
    ids.forEach((id) => {
      const motor = (data.motors || {})[id] || {};
      const angle = motor.online === true && typeof motor.angle === 'number' && Number.isFinite(motor.angle)
        ? (Math.abs(motor.angle) < 0.005 ? '0,00' : motor.angle.toFixed(2).replace('.', ',')) + '°' : '—';
      $('full-zero-angle-' + id).textContent = angle;
      let label = 'ожидание';
      if (motor.online === false) label = 'нет связи';
      else if (phase === 'calibrating') label = motor.calibrating ? 'поиск датчика' : (seen.includes(id) ? 'калибровка завершена' : 'ждём начала');
      else if (phase === 'adjusting' && motor.moving) label = 'поворачивается';
      else if (phase === 'checking') label = 'проверка';
      else if (phase === 'saving') label = acknowledged.includes(id) ? 'записано' : 'ожидание записи';
      else if (saved) label = 'запись подтверждена';
      else if (['adjust', 'adjusting'].includes(phase)) label = 'опора подтверждена';
      else if (motor.firmware && motor.firmware < 23) label = 'обновите прошивку';
      $('full-zero-state-' + id).textContent = label;
    });
  }

  async function send(action, extra) {
    const sentGeneration = generation;
    busy = true;
    errorText = '';
    render();
    const payload = Object.assign({action}, action === 'begin' ? {} : {token}, extra || {});
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 3000);
    try {
      const response = await fetch('api/kiosk_full_zero', {
        method: 'POST', cache: 'no-store', signal: controller.signal,
        headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'steering'},
        body: JSON.stringify(payload)
      });
      const data = await response.json().catch(() => ({}));
      if (sentGeneration !== generation) {
        if (action === 'begin' && data.token) cancelDetached(data.token);
        return null;
      }
      if (data.phase) { last = data; phase = data.phase; }
      else if (typeof data.pendingRecovery === 'boolean') last = {...last, pendingRecovery: data.pendingRecovery};
      if (action === 'begin' && data.token) token = data.token;
      if (!response.ok || !data.ok) {
        errorText = reasons[data.reason] || data.error || 'Команда не подтверждена. Проверьте связь.';
        if (action === 'begin' && !data.phase) phase = 'failed';
        if (data.phase === 'idle' || data.phase === 'recovery') token = null;
      } else if (action === 'begin') {
        token = data.token;
      }
      return data;
    } catch (_error) {
      if (sentGeneration === generation) {
        phase = 'connection';
        errorText = messages.connection;
      }
      return null;
    } finally {
      clearTimeout(timeout);
      if (sentGeneration === generation) { busy = false; render(); }
    }
  }

  function startPolling() {
    if (timer) clearInterval(timer);
    timer = setInterval(() => {
      if (!modal.hidden && token && !busy && phase !== 'saved') void send('status');
    }, 400);
  }

  async function begin() {
    const startedGeneration = ++generation;
    token = null; last = {}; phase = 'opening'; errorText = ''; busy = false; render();
    try {
      if (window.RCPrepareFullZero) await window.RCPrepareFullZero();
    } catch (_error) {
      if (startedGeneration === generation) {
        phase = 'failed'; errorText = 'Не удалось остановить предыдущее управление.'; render();
      }
      return;
    }
    if (startedGeneration !== generation || modal.hidden) return;
    await send('begin');
    if (startedGeneration === generation && token) startPolling();
  }

  function close() {
    const session = token;
    const needsStop = !['saved', 'idle', 'recovery'].includes(phase);
    ++generation;
    if (timer) clearInterval(timer);
    timer = null;
    modal.hidden = true;
    token = null;
    busy = false;
    if (needsStop) cancelDetached(session);  // never wait for an in-flight request
    if (previousFocus && typeof previousFocus.focus === 'function') previousFocus.focus();
  }

  open.addEventListener('click', () => {
    if (!modal.hidden) return;
    previousFocus = document.activeElement;
    modal.hidden = false;
    $('full-zero-close').focus();
    void begin();
  });
  $('full-zero-close').addEventListener('click', () => { void close(); });
  $('full-zero-cancel').addEventListener('click', () => { void close(); });
  retry.addEventListener('click', () => { if (!busy) void begin(); });
  calibrate.addEventListener('click', () => { if (!busy && phase === 'ready') void send('calibrate'); });
  save.addEventListener('click', () => { if (!busy && phase === 'adjust') void send('save'); });
  adjustButtons.forEach((button) => button.addEventListener('click', () => {
    if (!busy && phase === 'adjust') void send('adjust', {module: button.dataset.fullZeroAdjust, delta: Number(button.dataset.delta)});
  }));
  $('full-zero-estop').addEventListener('click', async () => {
    // Independent physical E-STOP remains necessary; this is a software latch.
    try {
      await fetch('api/estop', {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'drive'}, body: '{}'});
      errorText = reasons.estop;
    } catch (_error) { errorText = 'Нет подтверждения E-STOP. Используйте независимый аварийный выключатель.'; }
    render();
  });
  modal.addEventListener('keydown', (event) => {
    if (event.key === 'Escape') { event.preventDefault(); void close(); return; }
    if (event.key !== 'Tab') return;
    const focusable = Array.from(modal.querySelectorAll('button:not([disabled]):not([hidden])'));
    if (!focusable.length) return;
    const current = focusable.indexOf(document.activeElement);
    if (event.shiftKey && current <= 0) { event.preventDefault(); focusable[focusable.length - 1].focus(); }
    else if (!event.shiftKey && current === focusable.length - 1) { event.preventDefault(); focusable[0].focus(); }
  });
  window.addEventListener('pagehide', () => {
    if (!token || modal.hidden || phase === 'saved') return;
    cancelDetached(token);
  });
}());
