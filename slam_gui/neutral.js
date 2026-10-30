/* Manual freewheel: a short browser lease, with a separate brake ACK on exit. */
(function () {
  'use strict';

  function create(options) {
    const button = document.getElementById('neutral-toggle');
    const status = document.getElementById('neutral-status');
    if (!button || !status) return null;
    let session = null, sequence = 0, generation = 0, timer = null;
    let pending = false, starting = false, localPhase = '', note = '', noteUntil = 0;
    const messages = {
      neutral_locked: 'Нейтраль уже занята или тормоз ещё не подтверждён.',
      neutral_firmware_required: 'Нужна прошивка v18 или новее на всех четырёх платах.',
      neutral_expired: 'Связь прервана. Верните тормоз и дождитесь подтверждения.',
      neutral_unconfirmed: 'Не все колёса подтвердили нейтраль. Верните тормоз.',
      neutral_session: 'Нейтраль прервана; проверьте тормоз.',
      brake_unconfirmed: 'Тормоз подтверждён не всеми платами. Проверьте связь и повторите.',
      module_offline: 'Нет свежей связи со всеми четырьмя платами.',
      serial_unavailable: 'Нет связи с платами колёс.',
      module_busy: 'Сначала остановите тягу, руление и все задания.',
      maneuver_busy: 'Сначала остановите манёвр.',
      navigation_active: 'Сначала остановите маршрут.',
      wheel_moving: 'Дождитесь остановки колёс.',
      sc_invalid: 'Нет корректных показаний SC на всех колёсах.',
      estop: 'Включён E-STOP.',
      command_expired: 'Команда задержалась; проверьте связь и часы.',
      command_failed: 'Команда не подтверждена; проверьте связь.'
    };
    function say(text) { note = text; noteUntil = Date.now() + 6000; render(); }
    function state() { return (options.getState() || {}).neutral || null; }
    function render() {
      const info = state(), phase = info && info.phase || 'unknown';
      const locked = !!session || !!(info && info.locked);
      button.textContent = locked ? 'Вернуть тормоз' : 'Снять тормоз';
      button.setAttribute('aria-pressed', String(phase === 'neutral' || localPhase === 'neutral'));
      button.disabled = starting || (!info && !session) || phase === 'restoring';
      const labels = {
        idle: 'Нейтраль выключена. Механическое удержание не подтверждается.',
        braking: 'Проверяем остановку и подтверждение тормоза всех колёс…',
        engaging: 'Проверяем снятие тормоза на четырёх платах…',
        neutral: 'НЕЙТРАЛЬ · колёса могут свободно катиться. Другие команды заблокированы.',
        restoring: 'Возвращаем тормоз · ждём подтверждение всех четырёх плат…',
        fault: (messages[info && info.reason] || 'Нейтраль прервана. Верните тормоз и проверьте связь.'),
        unknown: 'Нет состояния нейтрали от бортового сервера.'
      };
      status.dataset.locked = String(locked);
      status.textContent = note && Date.now() < noteUntil ? note :
        labels[localPhase && session ? localPhase : phase] || labels.unknown;
      const keyboard = document.getElementById('keyboard-enable');
      if (keyboard) keyboard.disabled = locked;
    }
    async function post(body, keepalive) {
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 500);
      try {
        const headers = {'Content-Type': 'application/json', 'X-Robot-Control': 'drive'};
        if (body.action !== 'restore') headers['X-Robot-Command-Deadline'] = String(Date.now() / 1000 + 0.5);
        const response = await fetch('api/kiosk_neutral', {method: 'POST', cache: 'no-store',
          keepalive: !!keepalive, headers, body: JSON.stringify(body), signal: controller.signal});
        const result = await response.json().catch(() => ({}));
        if (!response.ok || !result.ok) throw new Error(result.reason || 'command_failed');
        return result;
      } finally { clearTimeout(timeout); }
    }
    function stopTimer() {
      if (timer) clearInterval(timer);
      timer = null;
      session = null;
      localPhase = '';
      ++generation;
    }
    async function pulse(gen) {
      if (!session || pending || gen !== generation) return;
      pending = true;
      const token = session;
      try {
        const result = await post({action: 'heartbeat', session: token, sequence: ++sequence});
        if (gen !== generation) return;
        localPhase = result.phase;
        render();
      } catch (error) {
        if (gen !== generation) return;
        stopTimer();
        say(messages[error.message] || 'Связь потеряна. Верните тормоз и проверьте робота.');
      } finally { pending = false; }
    }
    async function begin() {
      const gen = ++generation;
      starting = true;
      render();
      try {
        const result = await post({action: 'begin'});
        if (gen !== generation || document.hidden) {
          void post({action: 'restore'}, true).catch(() => {});
          return;
        }
        session = result.session;
        sequence = 0;
        localPhase = result.phase;
        note = '';
        timer = setInterval(() => { void pulse(gen); }, 150);
        void pulse(gen);
      } catch (error) {
        if (gen === generation) say(messages[error.message] || 'Не удалось снять тормоз.');
      } finally { starting = false; render(); }
    }
    async function restore(keepalive) {
      stopTimer();
      starting = true;
      render();
      try {
        await post({action: 'restore'}, keepalive);
        note = '';
      } catch (error) {
        say(messages[error.message] || 'Тормоз не подтверждён; проверьте связь.');
      } finally { starting = false; render(); }
    }
    button.addEventListener('click', () => {
      if (session || (state() && state().locked)) void restore(false);
      else void begin();
    });
    window.addEventListener('pagehide', () => { if (session || starting) void restore(true); });
    document.addEventListener('visibilitychange', () => {
      if (document.hidden && (session || starting)) void restore(true);
    });
    render();
    return {
      isActive: () => !!session || starting,
      restore,
      stateChanged: () => {
        const info = state();
        if (info && info.phase === 'fault' && session) stopTimer();
        render();
      }
    };
  }
  window.RCNeutral = {create};
}());
