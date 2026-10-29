/* Удалённый пульт сохраняет прежние кнопки направления и percent-протокол.
 * Бортовой экран использует отдельные прямые Servo-ползунки; этот файл на нём
 * не загружается. USB остаётся у борта, fetch проходит через web_controls.js. */
(function () {
  'use strict';
  if (!window.RCRemoteControls || document.body.dataset.remoteView !== 'motors') return;

  const ids = ['ALL', 'FL', 'FR', 'RL', 'RR'];
  const active = new Map(); // id -> {direction, pointerId/key}
  const pending = new Map();
  const $ = id => document.getElementById(id);
  let state = null;
  let sending = false;
  let flush = Promise.resolve();
  let timer = null;
  let lastTick = 0;
  let note = '';
  let noteUntil = 0;
  const reasonText = {
    serial_unavailable: 'Нет связи с USB-портами.',
    module_offline: 'Нет свежей телеметрии колеса.',
    drive_firmware_required: 'Обновите прошивку тяги.',
    not_homed: 'Сначала задайте ноль рулевых колёс.',
    module_busy: 'Дождитесь остановки руля.',
    estop: 'E-STOP взведён.',
    command_expired: 'Команда устарела в сети.',
    navigation_stopping: 'Маршрут останавливается. Повторите после остановки.',
    command_queue_full: 'Очередь команд занята.',
    pin_required: 'Требуется авторизация.'
  };

  function allowed(id) {
    const drive = state && state.drive || {};
    if (!state || state.linkOk === false || state.estop || !drive.available) return false;
    return id === 'ALL' ? drive.canDrive === true
      : !!(drive.perModule && drive.perModule[id] && drive.perModule[id].canDrive);
  }

  function percent(id) {
    const slider = $('remote-drive-speed-' + id);
    const value = Math.round(Number(slider && slider.value) || 8);
    return Math.max(1, Math.min(Number(slider && slider.max) || 100, value));
  }

  function render() {
    const drive = state && state.drive || {};
    const max = Math.max(1, Math.min(100, Number(drive.maxPercent) || 100));
    ids.forEach(id => {
      const enabled = allowed(id);
      const slider = $('remote-drive-speed-' + id);
      const value = $('remote-drive-value-' + id);
      if (slider) {
        slider.max = String(max);
        if (Number(slider.value) > max) slider.value = String(max);
        slider.disabled = !enabled;
        if (value) value.textContent = String(percent(id));
      }
      const incompatible = id === 'ALL'
        ? [...active.keys()].some(key => key !== 'ALL') : active.has('ALL');
      document.querySelectorAll('[data-remote-drive="' + id + '"]').forEach(button => {
        button.disabled = !enabled || incompatible;
      });
      if (id !== 'ALL') {
        const output = $('remote-drive-state-' + id);
        if (output) {
          const wheel = drive.perModule && drive.perModule[id];
          const moving = active.get(id);
          output.textContent = moving ? (moving.direction > 0 ? 'Вперёд' : 'Назад') + ' · удерживайте'
            : enabled ? 'Готово' : reasonText[wheel && wheel.reason] || 'Нет свежей связи';
        }
      }
    });
    const output = $('remote-drive-status');
    if (output) {
      const moving = active.get('ALL');
      output.textContent = note && Date.now() < noteUntil ? note
        : moving ? (moving.direction > 0 ? 'Вперёд' : 'Назад') + ' · удерживайте'
        : allowed('ALL') ? 'Готово · удерживайте кнопку' : reasonText[drive.reason] || 'Нет свежей связи';
    }
  }

  function stopTimerIfIdle() {
    if (!active.size && timer) { clearInterval(timer); timer = null; }
  }

  function queueCommand(id, target) {
    if (id === 'ALL' && target === 0) pending.clear();
    pending.set(id, target);
    if (sending) return flush;
    sending = true;
    flush = (async () => {
      while (pending.size) {
        // Останов отправляется раньше оставшихся heartbeat.
        const stopped = [...pending].find(([, value]) => value === 0);
        const key = stopped ? stopped[0] : pending.keys().next().value;
        const value = pending.get(key);
        pending.delete(key);
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 550);
        try {
          const body = key === 'ALL' ? {percent: value} : {module: key, percent: value};
          const response = await fetch('api/kiosk_drive', {
            method: 'POST', cache: 'no-store', signal: controller.signal,
            headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'drive'},
            body: JSON.stringify(body)
          });
          const result = await response.json().catch(() => ({}));
          if (!response.ok || !result.ok) throw new Error(result.reason || 'command_failed');
        } catch (error) {
          note = reasonText[error.message] || 'Связь потеряна · watchdog остановит моторы.';
          noteUntil = Date.now() + 6000;
          if (value !== 0) {
            if (key === 'ALL') active.clear(); else active.delete(key);
            stopTimerIfIdle();
            // Ошибка не должна автоматически возобновить движение.
            pending.set(key, 0);
          }
        } finally {
          clearTimeout(timeout);
        }
      }
    })().finally(() => { sending = false; render(); });
    return flush;
  }

  function stopOne(id) {
    if (!active.has(id)) return;
    active.delete(id);
    stopTimerIfIdle();
    void queueCommand(id, 0);
    render();
  }

  function stopAll(force) {
    if (!force && !active.size && !timer) return;
    active.clear();
    stopTimerIfIdle();
    void queueCommand('ALL', 0);
    render();
  }

  function heartbeat() {
    if (Date.now() - lastTick > 600) {
      note = 'Управление прервано · нажмите кнопку заново.';
      noteUntil = Date.now() + 6000;
      stopAll(true);
      return;
    }
    lastTick = Date.now();
    active.forEach((hold, id) => { void queueCommand(id, hold.direction * percent(id)); });
  }

  function start(id, direction, trigger) {
    if (!allowed(id) || active.has(id)) return;
    if ((id === 'ALL' && active.size) || (id !== 'ALL' && active.has('ALL'))) return;
    active.set(id, {direction, ...trigger});
    lastTick = Date.now();
    if (!timer) timer = setInterval(heartbeat, 200);
    heartbeat();
    render();
  }

  document.querySelectorAll('[data-remote-drive]').forEach(button => {
    const id = button.dataset.remoteDrive;
    const direction = Number(button.dataset.direction);
    button.addEventListener('pointerdown', event => {
      if (event.button != null && event.button !== 0) return;
      event.preventDefault();
      start(id, direction, {pointerId: event.pointerId});
    });
    button.addEventListener('pointerup', () => stopOne(id));
    button.addEventListener('pointercancel', () => stopOne(id));
    button.addEventListener('keydown', event => {
      if ((event.key === ' ' || event.key === 'Enter') && !event.repeat) {
        event.preventDefault();
        start(id, direction, {key: event.key});
      }
    });
    button.addEventListener('keyup', event => {
      if (event.key === ' ' || event.key === 'Enter') { event.preventDefault(); stopOne(id); }
    });
  });
  document.querySelectorAll('[data-remote-stop]').forEach(button => {
    button.addEventListener('click', () => {
      const id = button.dataset.remoteStop;
      if (id === 'ALL') stopAll(true);
      else { if (!active.has(id)) void queueCommand(id, 0); stopOne(id); }
    });
  });
  ids.forEach(id => {
    const slider = $('remote-drive-speed-' + id);
    if (slider) slider.addEventListener('input', () => {
      const hold = active.get(id);
      if (hold) void queueCommand(id, hold.direction * percent(id));
      render();
    });
  });
  window.addEventListener('pointerup', event => {
    active.forEach((hold, id) => { if (hold.pointerId === event.pointerId) stopOne(id); });
  });
  window.addEventListener('blur', () => stopAll(false));
  window.addEventListener('pagehide', () => stopAll(false));
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopAll(false); });

  // При переключении панели или потере телеметрии robot.js погасит старый режим.
  window.RCRemoteLegacyStop = () => stopAll(false);
  window.RCRemoteLegacyState = data => {
    state = data;
    if ([...active.keys()].some(id => !allowed(id))) stopAll(true);
    render();
  };
  render();
}());
