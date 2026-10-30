/* WASD is a separate hold-to-run mode. The onboard server owns the lease,
 * validates each sequence/angle and expires missing traction heartbeats. */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (root) root.RCKeyboard = api;
}(typeof window !== 'undefined' ? window : null, function () {
  'use strict';
  const IDS = ['FL', 'FR', 'RL', 'RR'];

  // WASD selects the wheel direction; arrows separately enable traction.
  // In crab, +90° with negative traction goes left without turning to -90°.
  // With no WASD key held the selected direction is straight ahead.
  function plan(keys, mode) {
    const has = (k) => keys.has(k);
    const up = has('ArrowUp'), down = has('ArrowDown');
    const w = has('KeyW'), s = has('KeyS'), a = has('KeyA'), d = has('KeyD');
    if ((w && s) || (a && d) || (up && down)) {
      return {angles: {FL: 0, FR: 0, RL: 0, RR: 0}, direction: 0};
    }
    const arrow = Number(up) - Number(down);
    const right = Number(d) - Number(a);
    const forward = w || s ? Number(w) - Number(s) : (right ? 0 : 1);
    if (mode === 'normal') {
      const angle = right * 25;
      const longitudinal = s ? -1 : 1;
      return {angles: {FL: angle, FR: angle, RL: -angle || 0, RR: -angle || 0},
        direction: arrow * longitudinal || 0};
    }
    const angle = (forward ? right * 45 * forward : 90) || 0;
    const selectedDirection = forward || right;
    return {angles: {FL: angle, FR: angle, RL: angle, RR: angle}, direction: arrow * selectedDirection || 0};
  }

  function create(options) {
    const $ = (id) => document.getElementById(id);
    const panel = $('keyboard-panel');
    if (!panel) return null;
    const button = $('keyboard-enable');
    const mode = $('keyboard-mode');
    const speed = $('keyboard-speed');
    const status = $('keyboard-status');
    const state = {enabled: false, enabling: false, token: null, sequence: 0,
      generation: 0, keys: new Set(), timer: null, running: null, planning: false,
      drivePending: false, pingPending: false, lastTick: 0, lastDrive: 0, lastPing: 0,
      chain: Promise.resolve()};
    const reasons = {
      keyboard_expired: 'Управление просрочено; включите WASD заново.',
      keyboard_session: 'WASD прервано другой командой; включите заново.',
      stale_command: 'Команда устарела; движение остановлено.',
      command_expired: 'Команда задержалась; включите WASD заново (проверьте часы).',
      module_offline: 'Нет свежей телеметрии четырёх колёс.',
      serial_unavailable: 'Нет связи с платами колёс.',
      not_homed: 'Сначала откалибруйте все рулевые колёса.',
      wheel_moving: 'Колёса ещё вращаются.',
      module_busy: 'Сначала остановите тягу и рулевые модули.',
      steering_changed: 'Углы руля не подтверждены; тяга снята.',
      brake_firmware_required: 'На всех платах нужна прошивка с подтверждением тормоза.',
      live_firmware_required: 'Для руления на ходу нужна fw20 или новее на всех четырёх платах.',
      live_unconfirmed: 'Не все платы подтвердили управление; тяга снята.',
      brake_unconfirmed: 'Тормоз не подтверждён всеми платами.',
      neutral_locked: 'Сначала верните тормоз после нейтрали.',
      sc_invalid: 'Нет корректных показаний SC.',
      sc_changed: 'Счётчик SC или плата изменились; тяга снята.',
      sc_stalled: 'Нет переходов SC на одном из колёс; тяга снята.',
      estop: 'Включён E-STOP.',
      manual_override: 'Другая команда остановила WASD.',
      navigation_stopping: 'Дождитесь остановки навигации.',
      command_failed: 'Нет подтверждения команды; тяга остановлена.'
    };
    function show(text) { if (status) status.textContent = text; }
    function updateButton() {
      if (button) {
        button.textContent = state.enabled || state.enabling ? 'Выключить WASD' : 'Включить WASD';
        button.setAttribute('aria-pressed', String(state.enabled));
      }
      if (status) status.dataset.active = String(state.enabled);
      window.RCKeyboardActive = state.enabled || state.enabling;
    }
    function valid(gen) { return state.enabled && gen === state.generation && !!state.token; }
    function speedValue() {
      return Math.max(1, Math.min(100, Math.round(Number(speed && speed.value) || 20)));
    }
    async function post(body, keepalive) {
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 550);
      try {
        const headers = {'Content-Type': 'application/json', 'X-Robot-Control': 'drive'};
        // A delayed BEGIN/STEER/ARM/DRIVE must not become a fresh command when
        // the browser is already gone. STOP/END always remain deliverable.
        if (body.action !== 'stop' && body.action !== 'end') {
          headers['X-Robot-Command-Deadline'] = String(Date.now() / 1000 + 0.5);
        }
        const response = await fetch('api/kiosk_keyboard', {
          method: 'POST', cache: 'no-store', signal: controller.signal, keepalive: !!keepalive,
          headers, body: JSON.stringify(body)
        });
        const result = await response.json().catch(() => ({}));
        if (!response.ok || !result.ok) {
          const error = new Error(result.reason || 'command_failed');
          error.retryable = result.retryable === true;
          throw error;
        }
        return result;
      } finally { clearTimeout(timeout); }
    }
    function send(action, values, keepalive) {
      const token = state.token;
      const sequence = ++state.sequence;
      return post(Object.assign({action, session: token, sequence}, values || {}), keepalive);
    }
    function retire(token, sequence, keepalive) {
      if (!token) return;
      void post({action: 'end', session: token, sequence}, keepalive).catch(() => {});
    }
    function disable(message, keepalive) {
      const token = state.token;
      const sequence = ++state.sequence;
      ++state.generation;
      state.enabled = false;
      state.enabling = false;
      state.token = null;
      state.keys.clear();
      state.running = null;
      state.planning = false;
      if (state.timer) clearInterval(state.timer);
      state.timer = null;
      updateButton();
      show(message || 'WASD выключено; тяга снята.');
      retire(token, sequence, keepalive);
    }
    function fail(error) {
      const reason = error && error.message || 'command_failed';
      disable(reasons[reason] || 'Нет подтверждения команды; тяга остановлена.');
    }
    function motorMap(data) {
      const result = {};
      (Array.isArray(data && data.motors) ? data.motors : []).forEach((m) => { if (m && IDS.includes(m.id)) result[m.id] = m; });
      return result;
    }
    function snapshotError(data) {
      if (!data || Date.now() - options.freshAt() > 850 || data.linkOk === false) return 'module_offline';
      if (data.estop) return 'estop';
      if (data.neutral && data.neutral.locked) return 'neutral_locked';
      if (!data.steeringControl || !data.steeringControl.available || !data.drive || !data.drive.available) return 'serial_unavailable';
      const motors = motorMap(data);
      for (const id of IDS) {
        const m = motors[id];
        if (!m || m.online !== true || typeof m.telemetryAgeS !== 'number' || m.telemetryAgeS >= 0.75) return 'module_offline';
        if (m.homed !== true) return 'not_homed';
        if (m.calibrating || m.cycling) return 'module_busy';
        if (typeof m.scRpm !== 'number' || !Number.isFinite(m.scRpm) || m.scRpm < 0 ||
            (m.firmware >= 19 && typeof m.scRecent !== 'boolean')) return 'sc_invalid';
        if (!Number.isInteger(m.firmware) || m.firmware < 20 ||
            !Number.isInteger(m.liveId) || m.liveId < 0 || m.liveId >= 2147483647 ||
            !Number.isInteger(m.liveSeq) || m.liveSeq < 0 || m.liveSeq >= 2147483647) return 'live_firmware_required';
        if (!Number.isInteger(m.brakeId) || m.brakeId < 0 || m.brakeId > 2147483647) return 'brake_firmware_required';
        if (typeof m.angle !== 'number' || !Number.isFinite(m.angle) || !Number.isInteger(m.drivePercent)) return 'module_offline';
      }
      return null;
    }
    function stopped(data) {
      const motors = motorMap(data);
      return !data.drive.active && IDS.every((id) => !motors[id].moving && motors[id].drivePercent === 0 &&
        motors[id].scRpm <= 1 && motors[id].scRecent !== true);
    }
    function arrived(data, angles) {
      const motors = motorMap(data);
      return IDS.every((id) => !motors[id].moving && Math.abs(motors[id].angle - angles[id]) <= 1.5);
    }
    function delay(ms) { return new Promise((resolve) => setTimeout(resolve, ms)); }
    async function waitFor(gen, predicate, timeoutMs) {
      const end = Date.now() + timeoutMs;
      while (valid(gen)) {
        const data = options.getState();
        const error = snapshotError(data);
        if (error) throw new Error(error);
        if (predicate(data)) return data;
        if (Date.now() > end) throw new Error('steering_changed');
        await delay(25);
      }
      return null;
    }
    async function sendWhenStopped(gen, action, values) {
      const end = Date.now() + 6000;
      while (valid(gen)) {
        try { return await send(action, values); }
        catch (error) {
          // The server may have a newer SC sample than the browser. Keep the
          // brakes on and retry only the explicit, safe coasting response.
          if (!error.retryable || error.message !== 'wheel_moving' || Date.now() > end) throw error;
          show('Колёса ещё вращаются · ждём остановки…');
          await delay(50);
        }
      }
      return null;
    }
    async function heartbeat(gen, current, permit) {
      if (!valid(gen) || state.drivePending) return;
      state.drivePending = true;
      state.lastDrive = Date.now();
      try {
        await send('live', {percent: current.direction * speedValue(), angles: current.angles, permit: permit || ''});
      } catch (error) {
        if (valid(gen)) fail(error);
      } finally {
        state.drivePending = false;
        // Coalesce fast key changes while one HTTP heartbeat is in flight.
        if (valid(gen) && state.running && state.running !== current) void heartbeat(gen, state.running, '');
      }
    }
    function tick() {
      if (!state.enabled) return;
      const now = Date.now(), gen = state.generation;
      if (state.running && now - state.lastTick > 550) {
        disable('Пауза браузера · отпустите клавиши и включите WASD заново.');
        return;
      }
      state.lastTick = now;
      const data = options.getState(), error = snapshotError(data);
      if (error) { fail(new Error(error)); return; }
      if (state.running) {
        // The onboard lease checks each board's sequence and steering progress;
        // being between the old and new target is expected during a live turn.
        if (!state.drivePending && now - state.lastDrive >= 180) void heartbeat(gen, state.running, '');
      } else if (!state.planning && !state.pingPending && now - state.lastPing >= 1000) {
        state.pingPending = true;
        state.lastPing = now;
        void send('ping').catch((error) => { if (valid(gen)) fail(error); })
          .finally(() => { state.pingPending = false; });
      }
    }
    async function enable() {
      if (state.enabled || state.enabling) { disable(); return; }
      const gen = ++state.generation;
      state.enabling = true;
      updateButton();
      show('Останавливаем другую тягу и открываем управление…');
      try {
        await options.stopManual();
        if (gen !== state.generation) return;
        const error = snapshotError(options.getState());
        if (error) throw new Error(error);
        const result = await post({action: 'begin'});
        if (gen !== state.generation) { retire(result.session, 1, false); return; }
        state.token = result.session;
        state.sequence = 0;
        state.enabled = true;
        state.enabling = false;
        if (options.onReady) options.onReady();
        state.lastTick = state.lastPing = Date.now();
        state.timer = setInterval(tick, 150);
        updateButton();
        show('WASD включено · удерживайте клавиши.');
      } catch (error) {
        if (gen === state.generation) fail(error);
      }
    }
    function replan() {
      if (!state.enabled) return;
      const desired = plan(state.keys, mode ? mode.value : 'crab');
      if (state.running && desired.direction && desired.direction === state.running.direction) {
        const changed = IDS.some((id) => desired.angles[id] !== state.running.angles[id]);
        if (changed) {
          state.running = desired;
          show('Рулим на ходу · удерживайте стрелку.');
          if (!state.drivePending) void heartbeat(state.generation, desired, '');
        }
        return;
      }
      const gen = ++state.generation;
      state.running = null;
      state.planning = true;
      show(desired ? 'Остановка перед изменением углов…' : 'Останавливаемся…');
      // Do not wait for an older request before issuing STOP. A higher sequence
      // makes every older request harmless even if the network reorders them.
      const stopping = send('stop').then((result) => ({result}), (error) => ({error}));
      state.chain = state.chain.catch(() => {}).then(async () => {
        const stopReply = await stopping;
        if (!valid(gen)) return;
        if (stopReply.error) throw stopReply.error;
        if (!desired) { state.planning = false; show('Тяга снята · WASD включено.'); return; }
        const brakeId = stopReply.result.brakeId;
        if (!Number.isInteger(brakeId) || brakeId <= 0) throw new Error('brake_firmware_required');
        let data = await waitFor(gen, (current) => stopped(current) &&
          IDS.every((id) => motorMap(current)[id].brakeId === brakeId), 6000);
        if (!data || !valid(gen)) return;
        const sentSteer = IDS.some((id) => Math.abs(motorMap(data)[id].angle - desired.angles[id]) > 1.0);
        if (sentSteer) await sendWhenStopped(gen, 'steer_all', {angles: desired.angles});
        if (!valid(gen)) return;
        const sentAt = Date.now();
        show('Ждём подтверждения углов всех колёс…');
        data = await waitFor(gen, (current) => (!sentSteer || options.freshAt() > sentAt) && arrived(current, desired.angles), 6000);
        if (!data || !valid(gen)) return;
        if (!desired.direction) {
          state.planning = false;
          show('Руль выставлен · без тяги.');
          return;
        }
        // An older heartbeat may still be awaiting its response. Do not spend
        // the short-lived start permit until it has finished or timed out.
        while (valid(gen) && state.drivePending) await delay(30);
        if (!valid(gen)) return;
        // A server-issued short-lived permit bounds delayed FIRST drive packets.
        const arm = await sendWhenStopped(gen, 'arm', {direction: desired.direction, angles: desired.angles});
        if (!valid(gen) || !arm) return;
        state.running = desired;
        state.planning = false;
        state.lastTick = Date.now();
        await heartbeat(gen, desired, arm.permit);
        if (valid(gen) && state.running) show('Движение · удерживайте клавиши.');
      }).catch((error) => { if (valid(gen)) fail(error); });
    }
    function keyCode(event) {
      if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ArrowUp', 'ArrowDown'].includes(event.code)) return event.code;
      if (event.key === 'ArrowUp' || event.key === 'ArrowDown') return event.key;
      const key = String(event.key || '').toUpperCase();
      return ['W', 'A', 'S', 'D'].includes(key) ? 'Key' + key : '';
    }
    function editable(target) {
      return !!(target && (target.isContentEditable || (target.closest && target.closest('input, textarea, select, [contenteditable="true"]'))));
    }
    window.addEventListener('keydown', (event) => {
      if (!state.enabled || document.hidden || event.ctrlKey || event.altKey || event.metaKey || editable(event.target)) return;
      const code = keyCode(event);
      if (!code) return;
      event.preventDefault();
      if (event.repeat || state.keys.has(code)) return;
      state.keys.add(code);
      replan();
    });
    window.addEventListener('keyup', (event) => {
      const code = keyCode(event);
      if (!state.enabled || !code || !state.keys.has(code)) return;
      event.preventDefault();
      state.keys.delete(code);
      replan();
    });
    if (button) button.addEventListener('click', () => { void enable(); });
    const stop = $('keyboard-stop');
    if (stop) stop.addEventListener('click', () => disable('СТОП · WASD выключено.'));
    if (mode) mode.addEventListener('change', () => {
      if (!state.enabled) return;
      state.keys.clear();
      replan();
    });
    if (speed) speed.addEventListener('input', () => {
      const output = $('keyboard-speed-value');
      if (output) output.textContent = String(speedValue());
    });
    window.addEventListener('blur', () => { if (state.enabled || state.enabling) disable('Потерян фокус · WASD выключено.'); });
    window.addEventListener('pagehide', () => { if (state.enabled || state.enabling) disable('Страница закрыта · WASD выключено.', true); });
    document.addEventListener('visibilitychange', () => { if (document.hidden && (state.enabled || state.enabling)) disable('Вкладка скрыта · WASD выключено.', true); });
    updateButton();
    return {disable, isActive: () => state.enabled || state.enabling,
      session: () => state.token,
      stateChanged: () => { if (state.enabled) { const error = snapshotError(options.getState()); if (error) fail(new Error(error)); } }};
  }
  return {plan, create};
}));
