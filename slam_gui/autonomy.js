/* Вкладка «Автономность · движение вперёд».
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
    idle: 'ОЖИДАНИЕ', starting: 'ЗАПУСК', following: 'ЕДУ ВПЕРЁД',
    obstacle_approach: 'ПРЕПЯТСТВИЕ · МЕДЛЕННО',
    aligning: 'ПОДТВЕРЖДАЮ УГОЛ КОЛЁС', stop_approach: 'ПОДЪЕЗЖАЮ К СТОП',
    stop_braking: 'СТОП · ТОРМОЖУ', stop_wait: 'СТОП · ВЫДЕРЖКА', traffic_wait: 'ЖДУ ЗЕЛЁНЫЙ',
    bump_wait: 'ЖДУ ПОДТВЕРЖДЕНИЯ КОЧКИ', bump_slow: 'КОЧКА · МЕДЛЕННО', bump_climb: 'ПЕРЕЕЗЖАЮ КОЧКУ',
    bump_exit: 'ПОСЛЕ КОЧКИ · МЕДЛЕННО', vision_lost: 'НЕТ СВЕЖИХ КАДРОВ',
    lane_lost: 'НЕТ СВЕЖИХ КАДРОВ', blocked: 'ПРЕПЯТСТВИЕ / STOP', done: 'ФИНИШ', cancelled: 'ЗАЕЗД ПРЕРВАН', error: 'ОШИБКА / STOP',
  };
  const CHECK_LABELS = {
    enabled: 'режим включён (RC_TRACK_ENABLE=1)',
    estopClear: 'E-STOP снят', visionOk: 'машинное зрение отвечает',
    visionRunning: 'поток разбора кадров идёт', camerasFresh: 'свежие кадры обеих передних камер',
    lidarReady: 'свежий передний сектор физического лидара', lidarClear: 'лидар не видит препятствий на пути',
    geometryCalibrated: 'геометрия уточнена (необязательно)',
    cameraPortsDistinct: 'камеры назначены на разные порты',
    driveReady: 'шасси готово к тяге', steerReady: 'адресное руление доступно',
    chassisLink: 'связь с платами колёс (@TLM)', wheelsOnline: 'все 4 колеса онлайн',
    depthFresh: 'свежий навигационный кадр Kinect 360',
    imuFresh: 'свежий гироскоп / наклон',
    wheelFeedback: 'свежая телеметрия SC всех колёс', brakeFeedback: 'подтверждаемый STOP',
    powerLimits: 'согласованные пределы тяги',
    rivalsIdle: 'другие режимы тяги выключены',
    blindZone: 'ближняя зона осмотрена', standstill: 'робот стоит',
  };
  const SIMULATION_CHECKS = new Set(['cameraPortsDistinct', 'lidarReady', 'lidarClear',
    'driveReady', 'steerReady', 'chassisLink', 'wheelsOnline', 'wheelFeedback', 'brakeFeedback']);
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
      const optional = key === 'blindZone' || key === 'geometryCalibrated' || (key === 'imuFresh' && !(status.config || {}).require_imu) ||
        (status.simulation && (SIMULATION_CHECKS.has(key) ||
          (key === 'depthFresh' && !(status.config || {}).require_depth)));
      const line = document.createElement('li');
      line.className = 'aut-check ' + (ok ? 'is-ok' : optional ? 'is-warn' : 'is-bad');
      const mark = document.createElement('span');
      mark.className = 'aut-check-mark';
      mark.textContent = ok ? '✓' : optional ? '•' : '✗';
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
    text('aut-ready-state', blockers.length ? blockers.length + ' блокировок' : 'проверки пройдены');
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
      ['stop_wait', 'stop_braking', 'traffic_wait', 'error', 'blocked'].includes(phase) ? 'stop' :
      phase === 'bump_slow' || phase === 'bump_climb' || phase === 'bump_exit' ||
      ['vision_lost', 'lane_lost', 'obstacle_approach'].includes(phase) ? 'warn' : 'idle');
    text('aut-phase', label);
    text('aut-message', status.message || 'Автономность не запущена.');
    text('aut-distance', (Number(status.distanceM) || 0).toFixed(1));
    text('aut-percent', (Number(status.percent) || 0).toFixed(1));
    text('aut-steer', (Number(status.steerLeftDeg) || 0).toFixed(1));
    text('aut-boost', (Number(status.boostPercent) || 0).toFixed(1));
    const lidar = status.lidar || {};
    text('aut-lidar', lidar.blocked ? 'ПРЕПЯТСТВИЕ' : lidar.ready ? 'СЕКТОР В НОРМЕ' : 'НЕТ СЕКТОРА');
    tone('aut-kpi-lidar', lidar.blocked ? 'stop' : phase === 'obstacle_approach' ? 'warn' : lidar.ready ? 'go' : 'warn');
    text('aut-obstacle-distance', lidar.nearestAheadM == null ? '—' : Number(lidar.nearestAheadM).toFixed(2));
    text('aut-lidar-points', Number(lidar.points) || 0);
    text('aut-path-meta', 'прямо · передний сектор: ' + (lidar.ready ? 'есть' : 'нет') +
      ' · точек: ' + (Number(lidar.points) || 0));
    const traffic = status.traffic || {};
    text('aut-traffic', TRAFFIC_NAMES[traffic.state || 'none'] || traffic.state || 'НЕ ВИДЕН');
    tone('aut-kpi-traffic', traffic.state === 'green' ? 'go' :
      traffic.visible && traffic.hold ? 'stop' : 'idle');
    const stops = (status.signs || []).filter((s) => s.class === 'stop_sign');
    text('aut-stop-sign', phase === 'stop_wait' ? 'ВЫДЕРЖКА' : phase === 'stop_braking' ? 'ТОРМОЖУ' :
      stops.length ? (stops.some((s) => s.side === 'side') ? 'С КРАЮ' : 'ВПЕРЕДИ') : '—');
    const ground = status.ground || {};
    text('aut-bump', ground.nearestBumpM != null ? Number(ground.nearestBumpM).toFixed(2) :
      (phase === 'bump_wait' ? 'НЕ ПОДТВЕРЖДЕНА' : '—'));
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
    if (stopBtn) stopBtn.disabled = !status.active && !state.busy;
    const empty = $('aut-canvas-empty');
    if (empty) empty.hidden = !!lidar.ready;
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
    // Направление прямо и габарит пути торможения — без привязки к столбикам.
    const config = status.config || {};
    const lidar = status.lidar || {};
    const halfLength = (Number(config.robot_length_m) || 0.75) / 2;
    const halfWidth = (Number(config.robot_width_m) || 0.60) / 2 + (Number(config.clearance_m) || 0.12);
    const stopRange = (Number(config.stop_clearance_m) || 0.8) + halfLength + (Number(config.clearance_m) || 0.12);
    ctx.fillStyle = lidar.blocked ? 'rgba(255,90,90,.18)' : lidar.ready ? 'rgba(97,214,143,.15)' : 'rgba(150,150,150,.12)';
    ctx.fillRect(px(halfWidth), py(stopRange), 2 * halfWidth * scale, stopRange * scale);
    ctx.strokeStyle = lidar.blocked ? '#ff5a5a' : lidar.ready ? '#61d68f' : '#888';
    ctx.lineWidth = 3;
    ctx.setLineDash([7, 5]);
    ctx.beginPath();
    ctx.moveTo(px(0), py(0));
    ctx.lineTo(px(0), py(Math.min(7.5, Math.max(2.5, Number(config.lidar_slow_distance_m) || 2))));
    ctx.stroke();
    ctx.setLineDash([]);
    const nearest = lidar.nearestPoint;
    if (nearest && Number.isFinite(nearest.x) && Number.isFinite(nearest.y) && nearest.x >= 0 && nearest.x <= 8.2) {
      ctx.fillStyle = '#51cfe8';
      ctx.beginPath();
      ctx.arc(px(nearest.y), py(nearest.x), 6, 0, Math.PI * 2);
      ctx.fill();
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
      const frame = cam + '/frame?t=' + Date.now();
      // Vision readiness is global; this camera's annotation may be missing or stale.
      img.onerror = visionOk ? () => { img.onerror = null; img.src = sessionUrl('api/camera/' + frame); } : null;
      img.src = sessionUrl((visionOk ? 'api/vision/camera/' : 'api/camera/') + frame);
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
      state.status = null;
      const start = $('aut-start');
      if (start) start.disabled = true;
      text('aut-message', 'Нет связи с бортом: ' + error.message);
      tone('aut-badge', 'err');
    }
  }

  async function command(path, payload, message) {
    if (state.busy && path !== 'api/autonomy/stop') return;
    state.busy = true;
    if (state.status) renderLive(state.status);
    note(message || 'Отправляю команду…');
    try {
      await api(path, 'POST', payload || {});
      const blind = $('aut-blind');
      if (blind) blind.checked = false;
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
        text('aut-sim-summary', result.passed && result.expectedStop
          ? 'Проверка прошла: STOP перед препятствием после ' + result.distanceM + ' м.'
          : result.passed || (result.passed == null && result.phase === 'done')
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
    // При прямом открытии /autonomy нет другой страницы, запускающей VisionService.
    api('api/vision/state', 'GET').then(refresh, () => {});
    state.timer = setInterval(refresh, 1000);
    state.camTimer = setInterval(updateCameras, 1400);
  }

  window.RCAutonomy = {init, refresh, renderCanvas, renderLive, renderChecks,
                       command, emergencyStop, _state: state};
})();
