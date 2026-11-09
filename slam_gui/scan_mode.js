/* Режим «Скан»: объезд помещения с камерами и живой картой.
 *
 * Здесь только то, чего нет на экране «Карта»: включаемое вручную управление
 * тягой и рулём, удержание кнопки вместо удержания клавиши, камеры и
 * сохранение/загрузка карты. Саму карту рисует map_view.js — он уже умеет
 * позу, сетку и тему, второй рисовальщик нам не нужен.
 *
 * Правила, которые важнее удобства:
 *   • управление включается кнопкой, а снимается кнопкой, Esc, `pagehide`,
 *     потерей фокуса и молчанием страницы (deadman на сервере);
 *   • отпущенная кнопка немедленно отправляет нулевую тягу, не дожидаясь
 *     следующего тика: «отпустил» должно значить «встал»;
 *   • пока управление выключено, кнопки движения заблокированы в разметке,
 *     а не только на сервере.
 */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const POLL_MS = 700;
  //: Как часто продлевать окно управления. Прошивка гасит колесо через 650 мс
  //: без heartbeat тяги, поэтому окно сервера продлеваем заметно чаще.
  const HEARTBEAT_MS = 1500;

  const state = {
    csrf: '',
    token: null,
    enabled: false,
    slamMode: 'mapping',
    maxPercent: 20,
    timer: null,
    heartbeatTimer: null,
    driveTimer: null,
    camTimers: [],
    busy: false,
    holding: null,          // {id, direction} пока кнопку держат
    keys: new Set(),
    keyboardGeneration: 0,
    keyboardQueue: Promise.resolve(),
    keyboardPlanKey: '',
    keyboardAppliedAngles: null,
    lastStatus: null,
    lidarBusy: false,
    robotBusy: false,
    sourceTimer: null,
    view3d: false,
  };

  const REASONS = {
    scan_disabled: 'Управление выключено — включите его кнопкой.',
    scan_timeout: 'Страница долго молчала, управление снято. Включите заново.',
    scan_token: 'Управление перехватила другая вкладка. Включите заново.',
    drive_limit: 'Это больше предела тяги режима «Скан».',
    estop: 'Взведён E-STOP.',
    brake_failed: 'Шасси не подтвердило остановку — управление не включено.',
    serial_unavailable: 'Нет связи с шасси.',
    module_offline: 'Нет телеметрии колёс.',
    not_homed: 'Сначала откалибруйте руль.',
    control_not_supported: 'Источник не поддерживает команду.',
    request_header_required: 'Сервер требует заголовок управления.',
  };

  function text(id, value) {
    const el = $(id);
    if (el) el.textContent = value;
  }

  function message(value, tone) {
    const el = $('scan-msg');
    if (!el) return;
    el.textContent = value;
    el.dataset.tone = tone || '';
  }

  function reasonText(reason, fallback) {
    return REASONS[reason] || fallback || String(reason || 'команда отклонена');
  }

  async function getJson(url) {
    const response = await fetch(url, {cache: 'no-store'});
    const body = await response.json().catch(() => null);
    if (!response.ok || !body || !body.ok) {
      throw new Error((body && (body.error || body.reason)) || ('HTTP ' + response.status));
    }
    return body;
  }

  async function postJson(url, payload) {
    const headers = {'Content-Type': 'application/json'};
    if (state.csrf) headers['X-CSRF-Token'] = state.csrf;
    if (state.token) headers['X-Scan-Token'] = state.token;
    const response = await fetch(url, {
      method: 'POST', cache: 'no-store', headers,
      body: JSON.stringify(payload || {}),
    });
    const body = await response.json().catch(() => null);
    if (!response.ok || !body || !body.ok) {
      const error = new Error((body && (body.error || body.reason)) || ('HTTP ' + response.status));
      error.reason = body && body.reason;
      error.status = response.status;
      throw error;
    }
    return body;
  }

  /* ------------------------------------------------------------ отрисовка */
  function renderState() {
    const chip = $('scan-state');
    if (chip) {
      chip.textContent = state.enabled ? 'включено' : 'выключено';
      chip.dataset.tone = state.enabled ? 'on' : 'off';
    }
    const enable = $('scan-enable');
    if (enable) enable.disabled = state.enabled || state.busy;
    const disable = $('scan-disable');
    if (disable) disable.disabled = !state.enabled;
    const hint = $('scan-hint');
    if (hint) {
      hint.textContent = state.enabled
        ? 'Управление включено. Держите кнопку — робот едет, отпустите — встаёт.'
        : 'Включите управление, чтобы ехать и снимать карту. Пока оно выключено, '
          + 'тяга и руль из этого режима не работают.';
    }
    const speed = $('scan-speed');
    if (speed) {
      speed.max = String(state.maxPercent);
      if (Number(speed.value) > state.maxPercent) speed.value = String(state.maxPercent);
      speed.disabled = !state.enabled;
      text('scan-speed-value', speed.value);
    }
    text('scan-max', String(state.maxPercent));
    ['scan-forward', 'scan-back-arrow', 'scan-left', 'scan-right'].forEach((id) => {
      const button = $(id);
      if (button) button.disabled = !state.enabled;
    });
    const keyMode = $('scan-key-mode');
    if (keyMode) keyMode.disabled = !state.enabled;
    const keyStatus = $('scan-key-status');
    if (keyStatus && !state.enabled) keyStatus.textContent = 'Включите управление выше, затем удерживайте WASD и ↑/↓.';
  }

  function renderStatus(status) {
    if (!status) return;
    state.lastStatus = status;
    if (typeof status.maxPercent === 'number') state.maxPercent = status.maxPercent;
    if (status.enabled === false && state.enabled) {
      // Сервер снял управление (deadman/E-STOP): честно показать и погасить.
      state.enabled = false;
      state.token = null;
      message(reasonText(status.reason, 'Управление снято.'), 'warn');
      stopDriving();
    }
    renderState();
    const link = $('scan-link');
    if (link) {
      const map = status.map || {};
      const live = Number(map.updatedAt) > 0;
      link.textContent = live ? 'связь: карта растёт' : 'связь: скана нет';
      link.dataset.tone = live ? 'on' : 'idle';
    }
    const saved = status.map && status.map.savedAt;
    if (saved) {
      const when = new Date(Number(saved) * 1000);
      const time = Number.isFinite(when.getTime())
        ? when.toLocaleTimeString('ru-RU', {hour: '2-digit', minute: '2-digit'}) : '—';
      text('map-file', 'сохранено: ' + time + ' · ' + ((status.map.paths || {}).pgm || '—'));
    }
  }

  async function refresh() {
    try {
      const body = await getJson('api/scan_mode');
      if (body && body.slamMode) renderSlamMode(body.slamMode);
      renderStatus(body);
    } catch (error) {
      const link = $('scan-link');
      if (link) { link.textContent = 'связь: нет'; link.dataset.tone = 'err'; }
    }
  }

  async function pollLidarView() {
    if (state.lidarBusy) return;
    state.lidarBusy = true;
    try {
      const response = await fetch('api/lidar', {cache: 'no-store'});
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const body = await response.json();
      const points = body && Array.isArray(body.scan) ? body.scan : [];
      if (window.RCMapView && typeof window.RCMapView.setScan === 'function') window.RCMapView.setScan(points);
      if (window.RCKinectView && typeof window.RCKinectView.setLidarScan === 'function') {
        window.RCKinectView.setLidarScan(points, body);
      }
    } catch (error) {
      if (window.RCKinectView && typeof window.RCKinectView.setLidarScan === 'function') {
        window.RCKinectView.setLidarScan([], {connected: false, age_s: null});
      }
    } finally { state.lidarBusy = false; }
  }

  async function pollRobotModel() {
    if (state.robotBusy || !window.RCRobotModel) return;
    state.robotBusy = true;
    try {
      const response = await fetch('api/state', {cache: 'no-store'});
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const body = await response.json();
      if (body && body.ok && body.data) window.RCRobotModel.update(body.data);
    } catch (error) {
      // RCRobotModel expires old angles on its own; a missed poll never refreshes them.
    } finally { state.robotBusy = false; }
  }

  function selectMapView(view) {
    const use3d = view === '3d';
    state.view3d = use3d;
    const map2d = $('scan-map-2d'), map3d = $('scan-map-3d');
    const button2d = $('scan-view-2d'), button3d = $('scan-view-3d');
    if (map2d) map2d.hidden = use3d;
    if (map3d) map3d.hidden = !use3d;
    if (button2d) { button2d.classList.toggle('is-active', !use3d); button2d.setAttribute('aria-selected', String(!use3d)); }
    if (button3d) { button3d.classList.toggle('is-active', use3d); button3d.setAttribute('aria-selected', String(use3d)); }
    if (window.RCKinectView && typeof window.RCKinectView.setMode === 'function') {
      window.RCKinectView.setMode(use3d ? 'combined' : 'lidar');
    }
    if (use3d) { void pollLidarView(); void pollRobotModel(); }
    if (!use3d && window.RCMapView) window.RCMapView.draw();
  }

  /* ------------------------------------------------------------ управление */
  function speed() {
    const slider = $('scan-speed');
    const value = Math.round(Number(slider && slider.value) || 8);
    return Math.max(1, Math.min(state.maxPercent, value));
  }

  async function sendDrive(percent) {
    if (!state.enabled && percent !== 0) return false;
    try {
      await postJson('api/scan_mode/drive', {percent: percent});
      return true;
    } catch (error) {
      if (error.reason === 'scan_disabled' || error.reason === 'scan_timeout'
          || error.reason === 'scan_token') {
        await dropControl(error.reason);
      }
      message(reasonText(error.reason, error.message), 'err');
      return false;
    }
  }

  async function sendSteer(module, angle) {
    if (!state.enabled) return null;
    try {
      return await postJson('api/scan_mode/steer', {module: module, angle: angle});
    } catch (error) {
      message(reasonText(error.reason, error.message), 'err');
      return null;
    }
  }

  function stopDriving() {
    state.holding = null;
    if (state.driveTimer) { clearInterval(state.driveTimer); state.driveTimer = null; }
  }

  function startHolding(id, direction) {
    if (!state.enabled || state.holding) return;
    state.holding = {id: id, direction: direction};
    void sendDrive(direction * speed());
    if (state.driveTimer) clearInterval(state.driveTimer);
    // Пока кнопку держат, тяга повторяется: сервер и прошивка ждут heartbeat,
    // а не одну команду «навсегда».
    state.driveTimer = setInterval(() => { void sendDrive(direction * speed()); }, 250);
  }

  function releaseHolding() {
    if (!state.holding) {
      // Кнопка «стоп» и Esc: тяга снимается в любом случае.
      return sendDrive(0);
    }
    stopDriving();
    return sendDrive(0);
  }

  const WHEEL_IDS = ['FL', 'FR', 'RL', 'RR'];
  function keyboardPlanKey(plan) {
    return String(plan.direction) + ':' + WHEEL_IDS.map(id => Number(plan.angles[id]).toFixed(2)).join(',');
  }
  function applyKeyboardPlan() {
    if (!state.enabled || !window.RCKeyboard || typeof window.RCKeyboard.plan !== 'function') return;
    if (state.holding && state.holding.id !== 'keyboard') {
      state.keys.clear();
      state.keyboardPlanKey = '';
      text('scan-key-status', 'Отпустите экранную кнопку движения, прежде чем переключать клавиши.');
      return;
    }
    const mode = $('scan-key-mode');
    const desired = window.RCKeyboard.plan(state.keys, mode ? mode.value : 'crab', null);
    const key = keyboardPlanKey(desired);
    if (key === state.keyboardPlanKey) return;
    state.keyboardPlanKey = key;
    const generation = ++state.keyboardGeneration;
    state.keyboardQueue = state.keyboardQueue.catch(() => {}).then(async () => {
      if (generation !== state.keyboardGeneration || !state.enabled) return;
      const previous = state.keyboardAppliedAngles;
      const steeringChanged = !previous || WHEEL_IDS.some(id => Math.abs(previous[id] - desired.angles[id]) > .01);
      const directionChanged = !state.holding || state.holding.direction !== desired.direction;
      if (steeringChanged) {
        await releaseHolding();
        if (generation !== state.keyboardGeneration || !state.enabled) return;
        const results = await Promise.all(WHEEL_IDS.map(id => sendSteer(id, desired.angles[id])));
        if (generation !== state.keyboardGeneration || !state.enabled) return;
        if (results.some(result => !result)) {
          state.keyboardPlanKey = '';
          text('scan-key-status', 'Не удалось выставить все четыре угла; тяга остановлена.');
          return;
        }
        if (!desired.direction) state.keyboardAppliedAngles = {...desired.angles};
      } else if (desired.direction === 0 && state.holding && state.holding.id === 'keyboard') {
        await releaseHolding();
      } else if (directionChanged && state.holding && state.holding.id === 'keyboard') {
        await releaseHolding();
      }
      if (desired.direction) {
        const arrived = await waitForKeyboardAngles(desired.angles, generation);
        if (generation !== state.keyboardGeneration || !state.enabled) return;
        if (!arrived) {
          state.keyboardPlanKey = '';
          text('scan-key-status', 'Колёса не подтвердили заданные углы; тяга не включена.');
          return;
        }
        state.keyboardAppliedAngles = {...desired.angles};
      }
      if (generation !== state.keyboardGeneration || !state.enabled) return;
      if (desired.direction && (!state.holding || state.holding.id !== 'keyboard')) {
        startHolding('keyboard', desired.direction);
      }
      const activeKeys = state.keys.size ? [...state.keys].map(code => code.replace('Key', '')).join(' ') : '—';
      text('scan-key-status', desired.direction
        ? 'WASD ' + activeKeys + ' · движение удерживается; отпустите клавиши для STOP.'
        : state.keys.size ? 'WASD ' + activeKeys + ' · углы выставлены, тяга остановлена.'
          : 'WASD отпущено · тяга остановлена.');
    });
  }

  async function waitForKeyboardAngles(angles, generation) {
    const until = Date.now() + 3500;
    while (Date.now() < until && generation === state.keyboardGeneration && state.enabled) {
      try {
        const response = await fetch('api/state', {cache: 'no-store'});
        if (response.ok) {
          const body = await response.json();
          if (body && body.ok && body.data && window.RCRobotModel) {
            window.RCRobotModel.update(body.data);
            const live = window.RCRobotModel.state();
            if (live.fresh && live.wheels.every(w => w.angle !== null && !w.fault &&
                Math.abs(w.angle - angles[w.id]) <= 1.5)) return true;
          }
        }
      } catch (error) { /* lost feedback means no traction */ }
      await new Promise(resolve => setTimeout(resolve, 120));
    }
    return false;
  }

  function scanKeyCode(event) {
    if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ArrowUp', 'ArrowDown'].includes(event.code)) return event.code;
    if (event.key === 'ArrowUp' || event.key === 'ArrowDown') return event.key;
    const key = String(event.key || '').toUpperCase();
    return ['W', 'A', 'S', 'D'].includes(key) ? 'Key' + key : '';
  }

  function clearKeyboardInput() {
    state.keys.clear();
    state.keyboardPlanKey = '';
    state.keyboardAppliedAngles = null;
    state.keyboardGeneration++;
    if (state.holding && state.holding.id === 'keyboard') void releaseHolding();
  }

  async function enable() {
    if (state.busy) return;
    state.busy = true;
    renderState();
    try {
      const body = await postJson('api/scan_mode/enable', {});
      state.token = body.token || null;
      state.enabled = true;
      state.keyboardAppliedAngles = null;
      state.keyboardPlanKey = '';
      message('Управление включено. Держите кнопку или используйте WASD.', 'ok');
      renderStatus(body);
    } catch (error) {
      state.enabled = false;
      state.token = null;
      message(reasonText(error.reason, error.message), 'err');
      await refresh();
    } finally {
      state.busy = false;
      renderState();
    }
  }

  async function dropControl(reason) {
    state.enabled = false;
    state.token = null;
    clearKeyboardInput();
    stopDriving();
    renderState();
    try {
      const body = await postJson('api/scan_mode/disable', {});
      renderStatus(body);
    } catch (error) {
      await refresh();
    }
    if (reason) message(reasonText(reason, 'Управление выключено.'), 'warn');
  }

  async function heartbeat() {
    if (!state.enabled) return;
    try {
      const body = await postJson('api/scan_mode/heartbeat', {token: state.token});
      if (body.enabled === false) {
        state.enabled = false;
        state.token = null;
        clearKeyboardInput();
        stopDriving();
        message('Управление снято сервером. Включите заново.', 'warn');
        renderState();
      }
    } catch (error) {
      if (error.reason === 'scan_token' || error.reason === 'scan_disabled') {
        await dropControl(error.reason);
      }
    }
  }

  /* ---------------------------------------------------------------- камеры */
  function startCameras() {
    for (const n of [1, 2, 3, 4]) {
      const canvas = $('cam' + n + '-canvas');
      if (canvas) pollCamera(n);
    }
  }

  function stopCameras() {
    state.camTimers.forEach((timer) => clearTimeout(timer));
    state.camTimers = [];
  }

  function pollCamera(n) {
    if (document.hidden) {
      state.camTimers[n] = setTimeout(() => pollCamera(n), 1200);
      return;
    }
    const canvas = $('cam' + n + '-canvas');
    if (!canvas) return;
    const image = new Image();
    const finish = (ok) => {
      if (ok && image.naturalWidth > 0) drawCamera(n, canvas, image);
      const chip = $('cam' + n + '-chip');
      if (chip) {
        chip.textContent = ok ? '● LIVE' : 'нет сигнала';
        chip.dataset.tone = ok ? 'on' : 'err';
      }
      const meta = $('cam' + n + '-meta');
      if (meta) {
        meta.textContent = ok ? image.naturalWidth + '×' + image.naturalHeight : 'переподключение…';
      }
      state.camTimers[n] = setTimeout(() => pollCamera(n), ok ? 120 : 1200);
    };
    const timeout = setTimeout(() => finish(false), 5000);
    image.onload = () => { clearTimeout(timeout); finish(true); };
    image.onerror = () => { clearTimeout(timeout); finish(false); };
    const url = 'api/camera/' + (n - 1) + '/frame?t=' + Date.now();
    image.src = window.RCSessionUrl ? window.RCSessionUrl(url) : url;
  }

  function drawCamera(n, canvas, image) {
    const rect = canvas.getBoundingClientRect ? canvas.getBoundingClientRect() : null;
    const width = Math.max(160, Math.round((rect && rect.width) || 320));
    const height = Math.max(90, Math.round((rect && rect.height) || 180));
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    const ctx = canvas.getContext && canvas.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#050708';
    ctx.fillRect(0, 0, width, height);
    const fit = Math.min(width / image.naturalWidth, height / image.naturalHeight);
    const w = image.naturalWidth * fit;
    const h = image.naturalHeight * fit;
    ctx.drawImage(image, (width - w) / 2, (height - h) / 2, w, h);
  }

  /* ------------------------------------------------------------ карта/файл */
  async function mapAction(url, label, confirmText) {
    if (confirmText && typeof confirm === 'function' && !confirm(confirmText)) return;
    try {
      const body = await postJson(url, {});
      message(label + ': готово', 'ok');
      if (window.RCMapView && typeof window.RCMapView.show === 'function') {
        window.RCMapView.show();
      }
      await refresh();
      return body;
    } catch (error) {
      message(label + ': ' + reasonText(error.reason, error.message), 'err');
      return null;
    }
  }

  /* ----------------------------------------------------------- обработчики */
  function holdButton(id, direction) {
    const button = $(id);
    if (!button) return;
    const press = (event) => {
      if (event && event.preventDefault) event.preventDefault();
      startHolding(id, direction);
    };
    const release = () => {
      if (state.holding && state.holding.id === id) releaseHolding();
    };
    button.addEventListener('pointerdown', press);
    button.addEventListener('pointerup', release);
    button.addEventListener('pointerleave', release);
    button.addEventListener('pointercancel', release);
    // Клавиатура и клик без pointer-событий: одно нажатие = короткий импульс.
    button.addEventListener('click', () => {
      if (!state.enabled) return;
      void sendDrive(direction * speed());
      setTimeout(() => { void sendDrive(0); }, 400);
    });
  }

  /* ---------------------------------------------------------- режим SLAM --- */
  function renderSlamMode(mode) {
    const next = mode === 'localize' ? 'localize' : 'mapping';
    state.slamMode = next;
    for (const button of document.querySelectorAll('[data-slam-mode]')) {
      const active = button.getAttribute('data-slam-mode') === next;
      button.classList.toggle('is-on', active);
      button.setAttribute('aria-pressed', String(active));
    }
    text('map-slam-label', next === 'localize' ? 'Только локализация' : 'Активный SLAM');
    text('map-slam-hint', next === 'localize'
      ? 'робот видит стены и сопоставляет их с картой, но не дорисовывает её'
      : 'скан дорисовывает стены и проходы — карта растёт');
    if (document.body && document.body.classList) {
      document.body.classList.toggle('map-mode-localize', next === 'localize');
    }
    // Строку «где робот на карте» рисует общий map_view.js по снимку карты:
    // здесь только перерисовываем её под новый режим.
    if (window.RCMapView && window.RCMapView.drawLocalization) {
      window.RCMapView.drawLocalization();
    }
  }

  async function setSlamMode(mode) {
    const next = mode === 'localize' ? 'localize' : 'mapping';
    try {
      const body = await postJson('api/kiosk_map/settings', {settings: {slam_mode: next}});
      renderSlamMode((body && body.settings && body.settings.slam_mode) || next);
      message(next === 'localize'
        ? 'Только локализация: карта больше не дорисовывается.'
        : 'Активный SLAM: скан снова дорисовывает карту.', 'ok');
    } catch (error) {
      message('Не удалось переключить режим карты: ' + (error.message || error), 'err');
    }
  }

  function bind() {
    const view2d = $('scan-view-2d'), view3d = $('scan-view-3d');
    if (view2d) view2d.addEventListener('click', () => selectMapView('2d'));
    if (view3d) view3d.addEventListener('click', () => selectMapView('3d'));
    selectMapView('2d');
    const keyMode = $('scan-key-mode');
    if (keyMode) keyMode.addEventListener('change', () => {
      state.keyboardPlanKey = '';
      applyKeyboardPlan();
    });
    const enableButton = $('scan-enable');
    if (enableButton) enableButton.addEventListener('click', () => { void enable(); });
    const disableButton = $('scan-disable');
    if (disableButton) disableButton.addEventListener('click', () => { void dropControl(''); });
    const stopButton = $('scan-stop');
    if (stopButton) stopButton.addEventListener('click', () => {
      releaseHolding();
      message('Стоп.', '');
    });

    holdButton('scan-forward', 1);
    holdButton('scan-back-arrow', -1);

    const left = $('scan-left');
    if (left) left.addEventListener('click', async () => {
      // Поворот — углами руля, как на основном экране: система знаков прошивки,
      // положительный угол — вправо.
      const angle = 25;
      const results = await Promise.all(['FL', 'FR', 'RL', 'RR'].map((module) => {
        return sendSteer(module, module === 'FL' || module === 'FR' ? -angle : angle);
      }));
      const ok = results.filter(Boolean).length;
      message(ok ? 'Колёса влево (' + ok + '/4).' : 'Руль не подтвердил поворот.', ok ? 'ok' : 'err');
    });
    const right = $('scan-right');
    if (right) right.addEventListener('click', async () => {
      const angle = 25;
      const results = await Promise.all(['FL', 'FR', 'RL', 'RR'].map((module) => {
        return sendSteer(module, module === 'FL' || module === 'FR' ? angle : -angle);
      }));
      const ok = results.filter(Boolean).length;
      message(ok ? 'Колёса вправо (' + ok + '/4).' : 'Руль не подтвердил поворот.', ok ? 'ok' : 'err');
    });

    const speed = $('scan-speed');
    if (speed) speed.addEventListener('input', () => {
      text('scan-speed-value', speed.value);
      if (state.holding) void sendDrive(state.holding.direction * speed());
    });

    const save = $('scan-map-save');
    if (save) save.addEventListener('click', () => { void mapAction('api/kiosk_map/save', 'Сохранение'); });
    const load = $('scan-map-load');
    if (load) load.addEventListener('click', () => {
      void mapAction('api/scan_mode/map/load', 'Загрузка карты',
                     'Загрузить сохранённую карту? Текущая карта будет заменена.');
    });
    const reset = $('scan-map-reset');
    if (reset) reset.addEventListener('click', () => {
      void mapAction('api/kiosk_map/reset', 'Новая карта',
                     'Начать новую карту? Текущая сетка будет очищена.');
    });

    // Режим SLAM: при объезде карта дорисовывается, при проезде — только
    // сопоставляется. Переключение карту не стирает.
    for (const button of document.querySelectorAll('[data-slam-mode]')) {
      button.addEventListener('click', () => {
        void setSlamMode(button.getAttribute('data-slam-mode'));
      });
    }

    document.addEventListener('keydown', (event) => {
      if (event.key === 'Escape') {
        if (state.enabled) void dropControl('');
        releaseHolding();
        return;
      }
      if (!state.enabled || event.ctrlKey || event.altKey || event.metaKey) return;
      const target = event.target;
      if (target && target.closest && target.closest('input, select, textarea, [contenteditable="true"], #kinect-3d-canvas')) return;
      const code = scanKeyCode(event);
      if (!code) return;
      event.preventDefault();
      if (event.repeat || state.keys.has(code)) return;
      state.keys.add(code);
      applyKeyboardPlan();
    });
    document.addEventListener('keyup', (event) => {
      const code = scanKeyCode(event);
      if (!code || !state.keys.has(code)) return;
      event.preventDefault();
      state.keys.delete(code);
      applyKeyboardPlan();
    });
    // Ушли со страницы или переключили вкладку — тяга снимается сразу.
    window.addEventListener('blur', () => {
      clearKeyboardInput();
      if (state.holding) releaseHolding();
    });
    window.addEventListener('pagehide', () => {
      clearKeyboardInput();
      stopDriving();
      stopCameras();
      if (state.enabled) {
        state.enabled = false;
        void postJson('api/scan_mode/disable', {}).catch(() => {});
      }
    });
    document.addEventListener('visibilitychange', () => {
      if (document.hidden && state.enabled) void dropControl('scan_timeout');
    });
  }

  function init() {
    bind();
    if (window.RCMapView && typeof window.RCMapView.show === 'function') {
      window.RCMapView.show();
    }
    startCameras();
    void refresh();
    void pollLidarView();
    state.timer = setInterval(() => { void refresh(); }, POLL_MS);
    state.heartbeatTimer = setInterval(() => { void heartbeat(); }, HEARTBEAT_MS);
    if (state.sourceTimer) clearInterval(state.sourceTimer);
    state.sourceTimer = setInterval(() => {
      void pollLidarView();
      if (state.view3d) void pollRobotModel();
    }, POLL_MS);
  }

  const api = {init, refresh, enable, disable: dropControl, getState: () => ({...state})};
  if (typeof window !== 'undefined') window.RCScanMode = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (typeof document !== 'undefined' && document.readyState === 'complete') init();
  else if (typeof document !== 'undefined' && document.addEventListener) {
    document.addEventListener('DOMContentLoaded', init);
  }
})();
