/* Вкладка «Автономность · трасса с конусами».
 *
 * Работает и на бортовом киоске (8090), и на пульте (8080, команды через
 * прокси с CSRF-токеном из /api/autonomy/status). Относительные URL + общий
 * slam_auth.js — страница не знает, где именно открыта.
 */
(function () {
  'use strict';
  if (window.RCAutonomy) return;

  const $ = (id) => document.getElementById(id);
  const MARKER = 'autonomy';
  const PHASES = {
    idle: 'ОЖИДАНИЕ', starting: 'ЗАПУСК', following: 'ЕДУ ПО ЦЕНТРУ',
    stop_wait: 'СТОП · ЖДУ 3 С', traffic_wait: 'ЖДУ ЗЕЛЁНЫЙ',
    bump_slow: 'КОЧКА · МЕДЛЕННО', bump_climb: 'КОЧКА · ДОБАВЛЯЮ ТЯГУ',
    bump_exit: 'ПОСЛЕ КОЧКИ · МЕДЛЕННО', lane_lost: 'КОРИДОР ПОТЕРЯН',
    done: 'ТРАССА ПРОЙДЕНА', cancelled: 'ОСТАНОВЛЕНО', error: 'ОШИБКА / STOP',
  };
  const CHECK_LABELS = {
    enabled: 'режим включён (RC_TRACK_ENABLE=1)',
    estopClear: 'E-STOP снят', visionOk: 'машинное зрение отвечает',
    visionRunning: 'поток разбора кадров идёт', camerasFresh: 'свежие кадры камер',
    conesVisible: 'конусы видны', corridorOk: 'коридор распознан',
    geometryCalibrated: 'геометрия камер откалибрована',
    driveReady: 'шасси готово к тяге', steerReady: 'адресное руление доступно',
    depthFresh: 'свежий кадр Kinect 360', imuFresh: 'свежий гироскоп / наклон',
    rivalsIdle: 'другие режимы тяги выключены',
    blindZone: 'ближняя зона осмотрена', standstill: 'робот стоит',
  };
  const OPTIONAL_CHECKS = new Set(['geometryCalibrated', 'depthFresh', 'imuFresh', 'blindZone']);
  const TRAFFIC_NAMES = {red: 'КРАСНЫЙ', yellow: 'ЖЁЛТЫЙ', green: 'ЗЕЛЁНЫЙ',
                         unknown: 'НЕ ОПРЕДЕЛЁН', none: 'НЕ ВИДЕН'};
  const state = {status: null, csrf: '', busy: false, initialized: false, timer: null, camTimer: null};

  function text(id, value) {
    const el = $(id);
    if (el) el.textContent = String(value == null ? '—' : value);
  }
  function tone(id, value) {
    const el = $(id);
    if (el) el.dataset.tone = value || 'idle';
  }
  function sessionUrl(url) {
    return typeof window.RCSessionUrl === 'function' ? window.RCSessionUrl(url) : url;
  }
  function note(message, isError) {
    text('aut-command-note', message);
    tone('aut-command-note', isError ? 'err' : 'ok');
  }

  async function api(path, method, payload, marker) {
    const headers = {'X-Robot-Control': marker || MARKER};
    if (method === 'POST') headers['Content-Type'] = 'application/json';
    if (state.csrf) headers['X-CSRF-Token'] = state.csrf;
    const init = {method: method || 'GET', cache: 'no-store', headers};
    if (method === 'POST') init.body = JSON.stringify(payload || {});
    const response = await fetch(sessionUrl(path), init);
    const body = await response.json().catch(() => null);
    if (!response.ok || !body || !body.ok) {
      throw new Error((body && (body.error || body.reason)) || ('HTTP ' + response.status));
    }
    return body;
  }

  /* ------------------------------------------------------------- готовность */
  function renderChecks(status) {
    const box = $('aut-checks');
    if (!box) return;
    box.textContent = '';
    const checks = status.checks || {};
    Object.keys(CHECK_LABELS).forEach((key) => {
      const ok = !!checks[key];
      const line = document.createElement('li');
      line.className = 'aut-check ' + (ok ? 'is-ok' : OPTIONAL_CHECKS.has(key) ? 'is-warn' : 'is-bad');
      const mark = document.createElement('span');
      mark.className = 'aut-check-mark';
      mark.textContent = ok ? '✓' : OPTIONAL_CHECKS.has(key) ? '•' : '✗';
      line.appendChild(mark);
      const label = document.createElement('span');
      label.textContent = CHECK_LABELS[key];
      line.appendChild(label);
      box.appendChild(line);
    });
    const blockers = Array.isArray(status.blockers) ? status.blockers : [];
    const warnings = Array.isArray(status.warnings) ? status.warnings : [];
    text('aut-blockers', blockers.length ? blockers.join('\n') : '');
    const warnBox = $('aut-warnings');
    if (warnBox) {
      warnBox.textContent = '';
      warnings.forEach((item) => {
        const line = document.createElement('li');
        line.textContent = item;
        warnBox.appendChild(line);
      });
    }
    text('aut-ready-state', blockers.length ? blockers.length + ' блокировок' : 'можно ехать');
    tone('aut-ready-state', blockers.length ? 'err' : 'go');
    text('aut-enable-state', status.enabled ?
      (status.simulation ? 'ВКЛЮЧЁН · БЕЗ ТЯГИ' : 'ВКЛЮЧЁН') : 'ВЫКЛЮЧЕН');
  }

  /* --------------------------------------------------------------- состояние */
  function renderLive(status) {
    const phase = status.phase || 'idle';
    const label = PHASES[phase] || String(phase).toUpperCase();
    text('aut-badge', label);
    tone('aut-badge', phase === 'following' || phase === 'done' ? 'go' :
      phase === 'stop_wait' || phase === 'traffic_wait' || phase === 'error' ? 'stop' :
      phase === 'bump_slow' || phase === 'bump_climb' || phase === 'bump_exit' ||
      phase === 'lane_lost' ? 'warn' : 'idle');
    text('aut-phase', label);
    text('aut-message', status.message || 'Автономность не запущена.');
    text('aut-distance', (Number(status.distanceM) || 0).toFixed(1));
    text('aut-percent', (Number(status.percent) || 0).toFixed(1));
    text('aut-steer', (Number(status.steerLeftDeg) || 0).toFixed(1));
    text('aut-boost', (Number(status.boostPercent) || 0).toFixed(1));
    const corridor = status.corridor || {};
    text('aut-corridor', corridor.ok ?
      (Number(corridor.centerM) || 0).toFixed(2) + ' / ' +
      (corridor.widthM == null ? '?' : Number(corridor.widthM).toFixed(2)) : '—');
    text('aut-cones', (status.cones || []).length);
    text('aut-corridor-meta', 'конусов: ' + (status.cones || []).length +
      ' · границы: ' + (corridor.leftCount == null ? '—' : corridor.leftCount) + '/' +
      (corridor.rightCount == null ? '—' : corridor.rightCount));
    const traffic = status.traffic || {};
    text('aut-traffic', TRAFFIC_NAMES[traffic.state || 'none'] || traffic.state || 'НЕ ВИДЕН');
    tone('aut-kpi-traffic', traffic.state === 'green' ? 'go' :
      traffic.visible && traffic.hold ? 'stop' : 'idle');
    const stops = (status.signs || []).filter((s) => s.class === 'stop_sign');
    text('aut-stop-sign', phase === 'stop_wait' ? 'СТОЮ 3 С' :
      stops.length ? (stops.some((s) => s.side === 'side') ? 'С КРАЮ' : 'ВПЕРЕДИ') : '—');
    const ground = status.ground || {};
    text('aut-bump', ground.nearestBumpM != null ? Number(ground.nearestBumpM).toFixed(2) :
      (phase.indexOf('bump') === 0 ? 'ПО ЗНАКУ' : '—'));
    const imu = status.imu || {};
    text('aut-imu', imu.fresh ? (Number(imu.pitch) || 0).toFixed(1) : 'нет');
    const progress = $('aut-progress');
    if (progress) {
      const distance = Number(status.distanceM) || 0;
      progress.max = Math.max(50, Math.ceil((distance + 5) / 10) * 10);
      progress.value = distance;
    }
    renderEvents('aut-events', status.events || []);
    const startBtn = $('aut-start');
    const stopBtn = $('aut-stop');
    const blind = $('aut-blind');
    // canStart from status() не учитывает разовую галочку осмотра: пуск
    // требует её в теле POST, поэтому кнопка тоже смотрит на чекбокс.
    const blindOk = !status.requireBlindCheck || !!(blind && blind.checked);
    if (startBtn) startBtn.disabled = state.busy || status.active ||
      !status.canStart || !blindOk;
    if (stopBtn) stopBtn.disabled = state.busy || !status.active;
    const empty = $('aut-canvas-empty');
    if (empty) empty.hidden = (status.cones || []).length > 0;
  }

  function renderEvents(id, events) {
    const box = $(id);
    if (!box) return;
    box.textContent = '';
    const tail = (events || []).slice(-14);
    tail.forEach((event) => {
      const line = document.createElement('li');
      line.dataset.tone = event.tone || 'info';
      line.textContent = (Number(event.atS) || 0).toFixed(1) + ' с · ' +
        (Number(event.distanceM) || 0).toFixed(1) + ' м · ' + String(event.message || '');
      box.appendChild(line);
    });
  }

  /* ------------------------------------------------------------------ схема */
  function renderCanvas(status) {
    const canvas = $('aut-canvas');
    if (!canvas || !canvas.getContext) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const rect = canvas.getBoundingClientRect ? canvas.getBoundingClientRect() : {width: 640, height: 480};
    const w = Math.max(1, Math.round(rect.width || canvas.width || 640));
    const h = Math.max(1, Math.round(rect.height || canvas.height || 480));
    const dpr = Math.min(2, Number(window.devicePixelRatio) || 1);
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const scale = Math.min(w / 4.2, h / 8.2);
    const originX = w / 2;
    const originY = h - 26;
    const px = (y) => originX - y * scale;
    const py = (x) => originY - x * scale;
    const light = document.documentElement.getAttribute('data-map-theme') === 'light';
    ctx.strokeStyle = light ? 'rgba(20,16,19,.12)' : 'rgba(255,255,255,.09)';
    ctx.lineWidth = 1;
    for (let metre = 1; metre <= 8; metre += 1) {
      ctx.beginPath();
      ctx.moveTo(0, py(metre));
      ctx.lineTo(w, py(metre));
      ctx.stroke();
    }
    [-2, -1, 0, 1, 2].forEach((metre) => {
      ctx.beginPath();
      ctx.moveTo(px(metre), 0);
      ctx.lineTo(px(metre), h);
      ctx.stroke();
    });
    // Центр коридора: от текущего смещения до точки прицеливания.
    const corridor = status.corridor || {};
    if (corridor.ok) {
      const look = Math.max(1, Number((status.config || {}).lookahead_m) || 2.5);
      const ahead = Number(corridor.centerAheadM == null ? corridor.centerM : corridor.centerAheadM) || 0;
      const here = Number(corridor.centerM) || 0;
      ctx.strokeStyle = '#61d68f';
      ctx.lineWidth = 3;
      ctx.setLineDash([7, 5]);
      ctx.beginPath();
      ctx.moveTo(px(here), py(0));
      ctx.lineTo(px(ahead), py(look));
      ctx.stroke();
      ctx.setLineDash([]);
    }
    (status.cones || []).forEach((cone) => {
      const x = Number(cone.x) || 0;
      const y = Number(cone.y) || 0;
      if (x < 0 || x > 8.2) return;
      ctx.fillStyle = '#ff7a29';
      ctx.beginPath();
      ctx.arc(px(y), py(x), 4.5, 0, Math.PI * 2);
      ctx.fill();
    });
    (status.signs || []).forEach((sign) => {
      const x = Number(sign.x) || 0;
      const y = Number(sign.y) || 0;
      if (x < 0 || x > 8.2) return;
      const cx = px(y), cy = py(x);
      if (sign.class === 'stop_sign') {
        ctx.fillStyle = '#ff3d71';
        ctx.fillRect(cx - 7, cy - 7, 14, 14);
      } else if (sign.class === 'speed_bump') {
        ctx.fillStyle = '#ffb020';
        ctx.fillRect(cx - 7, cy - 5, 14, 10);
      } else if (sign.class === 'traffic_light') {
        const signal = (status.traffic || {}).state || sign.signal || 'unknown';
        ctx.fillStyle = '#33363b';
        ctx.fillRect(cx - 6, cy - 12, 12, 24);
        ctx.fillStyle = signal === 'green' ? '#54d67f' : signal === 'yellow' ? '#ffc94a' :
          signal === 'red' ? '#ff5a5a' : '#888';
        ctx.beginPath();
        ctx.arc(cx, cy, 4, 0, Math.PI * 2);
        ctx.fill();
      }
    });
    // Робот: треугольник носом вверх.
    ctx.fillStyle = light ? '#222' : '#f5f2f4';
    ctx.beginPath();
    ctx.moveTo(originX, originY - 13);
    ctx.lineTo(originX - 11, originY + 11);
    ctx.lineTo(originX + 11, originY + 11);
    ctx.closePath();
    ctx.fill();
    ctx.fillStyle = light ? '#666' : '#a2949a';
    ctx.font = '11px ui-monospace, monospace';
    ctx.fillText('0 м', 8, originY - 4);
    ctx.fillText('8 м', 8, py(8) + 12);
  }

  /* ------------------------------------------------------------------ камеры */
  function updateCameras() {
    const status = state.status;
    if (!status || typeof document === 'undefined') return;
    const visionOk = !!(status.checks || {}).visionOk;
    for (let cam = 0; cam < 4; cam += 1) {
      const img = $('aut-cam-' + cam);
      if (!img) continue;
      const path = (visionOk ? 'api/vision/camera/' : 'api/camera/') + cam +
        '/frame?t=' + Date.now();
      img.src = sessionUrl(path);
    }
  }

  /* ------------------------------------------------------------------- обмен */
  async function refresh() {
    try {
      const status = await api('api/autonomy/status', 'GET');
      state.status = status;
      state.csrf = status.csrfToken || state.csrf;
      renderChecks(status);
      renderLive(status);
      renderCanvas(status);
    } catch (error) {
      text('aut-message', 'Нет связи с бортом: ' + error.message);
      tone('aut-badge', 'err');
    }
  }

  async function command(path, payload, message) {
    if (state.busy) return;
    state.busy = true;
    note(message || 'Отправляю команду…');
    try {
      await api(path, 'POST', payload || {});
      note('Команда принята.');
    } catch (error) {
      note('Команда отклонена: ' + error.message, true);
    } finally {
      state.busy = false;
      await refresh();
    }
  }

  async function emergencyStop() {
    note('E-STOP: взвожу аварийную остановку…');
    const paths = ['api/estop', 'api/control/estop'];
    for (const path of paths) {
      try {
        await api(path, 'POST', {}, 'drive');
        note('E-STOP взведён: тяга снята. Снять можно только с PIN на главном экране.');
        await refresh();
        return;
      } catch (error) { /* пробуем путь веб-пульта */ }
    }
    note('E-STOP не подтверждён бортом — нажмите физическую кнопку на роботе!', true);
  }

  function init() {
    if (state.initialized) return;
    state.initialized = true;
    const start = $('aut-start');
    const stop = $('aut-stop');
    const estop = $('aut-estop');
    const sim = $('aut-simulate');
    const blind = $('aut-blind');
    const theme = $('aut-theme');
    if (start) start.addEventListener('click', () => command(
      'api/autonomy/start', {blindZoneChecked: !!(blind && blind.checked)}, 'Запускаю автономность…'));
    if (stop) stop.addEventListener('click', () => command(
      'api/autonomy/stop', {}, 'Останавливаю автономность…'));
    if (estop) estop.addEventListener('click', emergencyStop);
    if (sim) sim.addEventListener('click', async () => {
      const select = $('aut-scenario');
      const scenario = select ? select.value : 'full';
      text('aut-sim-summary', 'Считаю сухой прогон…');
      try {
        const result = await api('api/autonomy/simulate', 'POST', {scenario});
        renderEvents('aut-sim-events', result.events || []);
        text('aut-sim-summary', result.phase === 'done'
          ? 'Готово: ' + result.distanceM + ' м за ' + result.elapsedS + ' с · СТОП ' +
            result.counters.stop + ' · кочек ' + result.counters.bump +
            ' · светофоров ' + result.counters.traffic +
            ' · добавок тяги ' + result.counters.powerBoosts
          : 'Прогон завершился: ' + (result.message || result.phase));
      } catch (error) {
        text('aut-sim-summary', 'Сухой прогон недоступен: ' + error.message);
      }
    });
    if (blind) blind.addEventListener('change', () => {
      if (state.status) renderLive(state.status);
    });
    if (theme) theme.addEventListener('click', () => {
      const light = document.documentElement.getAttribute('data-map-theme') === 'light';
      const next = light ? 'dark' : 'light';
      document.documentElement.setAttribute('data-map-theme', next);
      theme.textContent = light ? 'Светлая тема' : 'Тёмная тема';
      theme.setAttribute('aria-pressed', String(!light));
      try { localStorage.setItem('kiosk_map_theme', next); } catch (error) { /* приватный режим */ }
      if (state.status) renderCanvas(state.status);
    });
    // Кадры камер показываем только после первого статуса: по нему известно,
    // включено ли машинное зрение и какой URL разметки использовать.
    refresh().then(updateCameras);
    state.timer = setInterval(refresh, 1000);
    state.camTimer = setInterval(updateCameras, 1400);
  }

  window.RCAutonomy = {init, refresh, renderCanvas, renderLive, renderChecks,
                       command, emergencyStop, _state: state};
})();
