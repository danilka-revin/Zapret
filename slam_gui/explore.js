/* Автономное обследование: карта растёт сама, оператор только запускает и стопит.
 * Работает и на киоске (8090), и на пульте (8080): команды идут относительными
 * URL, CSRF-токен берётся из /api/exploration/status или /api/calibration/geometry.
 * Здесь же мастер калибровки: замеры с робота сохраняются в .env на борту.
 */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const PHASES = {
    idle: 'ожидание', starting: 'запуск', planning: 'поиск границы',
    driving: 'движение', mapping: 'обновление карты', waiting: 'ожидание сканов',
    done: 'завершено', cancelled: 'остановлено', error: 'ошибка',
  };
  let csrf = '';
  let busy = false;
  let timer = null;
  let initialized = false;
  let calibrationFields = [];

  function text(id, value) { const el = $(id); if (el) el.textContent = value; }
  function okTone(el, good) { if (!el) return; el.classList.toggle('ok', !!good); el.classList.toggle('bad', !good); }

  async function getJson(url) {
    const response = await fetch(url, { cache: 'no-store' });
    const body = await response.json().catch(() => null);
    if (!response.ok || !body || !body.ok) {
      throw new Error((body && (body.error || body.reason)) || ('HTTP ' + response.status));
    }
    return body;
  }

  async function postJson(url, payload, marker) {
    const headers = {
      'Content-Type': 'application/json',
      'X-Robot-Control': marker || 'exploration',
    };
    if (csrf) headers['X-CSRF-Token'] = csrf;
    const response = await fetch(url, {
      method: 'POST', cache: 'no-store', headers,
      body: JSON.stringify(payload || {}),
    });
    const body = await response.json().catch(() => null);
    if (!response.ok || !body || !body.ok) {
      throw new Error((body && (body.error || body.reason)) || ('HTTP ' + response.status));
    }
    return body;
  }

  function renderBlockers(status) {
    const box = $('explore-blockers');
    if (!box) return;
    const blockers = Array.isArray(status.blockers) ? status.blockers : [];
    box.textContent = '';
    if (!blockers.length) return;
    const title = document.createElement('b');
    title.textContent = 'Перед пуском:';
    box.appendChild(title);
    blockers.forEach((reason) => {
      const line = document.createElement('div');
      line.className = 'explore-blocker';
      line.textContent = '• ' + String(reason);
      box.appendChild(line);
    });
  }

  function renderNav(nav) {
    const box = $('explore-nav');
    if (!box || !nav || !nav.checks) return;
    box.textContent = '';
    const title = document.createElement('b');
    title.textContent = 'Готовность навигации: ' + (nav.ready ? 'можно ехать' :
      (nav.reason || 'есть блокирующие проверки'));
    box.appendChild(title);
    Object.keys(nav.checks).forEach((key) => {
      const line = document.createElement('div');
      line.className = 'explore-nav-check ' + (nav.checks[key] ? 'ok' : 'bad');
      line.textContent = (nav.checks[key] ? '✓ ' : '✗ ') +
        (NAV_LABELS[key] || key);
      box.appendChild(line);
    });
  }

  const NAV_LABELS = {
    enabled: 'включена навигация (RC_NAV_ENABLE=1)',
    speedCalibrated: 'измерена скорость: RC_NAV_MPS_PER_PERCENT',
    speedWithinLimit: 'скорость внутри RC_NAV_MAX_PERCENT',
    masterReady: 'готово шасси (RC_SLAM_SOURCE=master, canDrive)',
    driveStopped: 'тяга остановлена перед стартом',
    mapFresh: 'свежий скан лидара',
    mapReady: 'не менее двух сканов для локализации',
    mapTrusted: 'карта построена с физического лидара',
    mapIntegrity: 'карта с сопоставлением сканов и без decay',
    scanReady: 'полный скан без пропусков',
    poseReady: 'качество scan matching выше порога',
    mapDecayDisabled: 'забывание препятствий выключено (decay=0)',
    footprintSufficient: 'защитный радиус больше половины диагонали корпуса',
    estopClear: 'E-STOP снят',
  };

  function render(status) {
    if (status.csrfToken) csrf = status.csrfToken;
    renderNav(status.navigation);
    const phase = String(status.phase || 'idle');
    text('explore-status', PHASES[phase] || phase);
    const badge = $('explore-status');
    if (badge) badge.dataset.phase = phase;
    text('explore-message', status.message || '—');
    text('explore-coverage', Number.isFinite(status.coveragePct) ? status.coveragePct.toFixed(1) + '%' : '—');
    text('explore-steps', String(status.steps ?? 0) + ' / ' + String(status.maxSteps ?? '—'));
    text('explore-frontiers', String(status.frontiers ?? 0));
    text('explore-kinect', status.kinectFresh ? 'свежий' : 'нет кадра');
    okTone($('explore-kinect'), status.kinectFresh);
    text('explore-enabled', status.enabled && status.navigationEnabled ? 'включено' : 'выключено');
    okTone($('explore-enabled'), status.enabled && status.navigationEnabled);
    renderBlockers(status);
    const startBtn = $('explore-start');
    const stopBtn = $('explore-stop');
    const canStart = status.canStart !== false;
    if (startBtn) startBtn.disabled = busy || status.active || !canStart ||
      !status.enabled || !status.navigationEnabled;
    if (stopBtn) stopBtn.disabled = busy || !status.active;
  }

  function renderLayout(layout) {
    if (!layout || !Array.isArray(layout.cameras)) return;
    const cams = layout.cameras.slice(0, 4)
      .map((cam) => `${cam.role}: ${cam.yaw_deg}°`)
      .join(' · ');
    const units = Object.keys(layout.kinects || { front: {} });
    text('explore-cameras', (cams || '—') + ' · Kinect: ' + units.join(' + '));
    text('explore-calibration', layout.calibrated
      ? 'углы заданы в .env'
      : 'углы не откалиброваны — откройте «Калибровка геометрии»');
    okTone($('explore-calibration'), layout.calibrated);
    if (layout.duplicatePorts && layout.duplicatePorts.length) {
      text('explore-message', 'Две камеры используют один порт: ' + layout.duplicatePorts.join(', '));
    }
  }

  function renderLandmarks(items) {
    const box = $('explore-landmarks');
    if (!box) return;
    box.textContent = '';
    (items || []).slice(-3).reverse().forEach((entry) => {
      const line = document.createElement('div');
      line.className = 'explore-landmark';
      const pose = entry.pose || {};
      const caption = document.createElement('span');
      caption.textContent = `шаг ${entry.step}: ${Number(pose.x || 0).toFixed(2)}, ${Number(pose.y || 0).toFixed(2)} м`;
      line.appendChild(caption);
      (entry.frames || []).forEach((frame) => {
        const img = document.createElement('img');
        img.src = 'api/exploration/landmark/' + encodeURIComponent(frame.file);
        img.alt = frame.role || ('камера ' + frame.camera);
        img.loading = 'lazy';
        line.appendChild(img);
      });
      box.appendChild(line);
    });
    if (!box.childNodes.length) text('explore-landmarks', 'ориентиров пока нет');
  }

  // -- калибровка геометрии ---------------------------------------------- //
  function renderCalibration(payload) {
    if (payload.csrfToken) csrf = payload.csrfToken;
    if (payload.hint) text('explore-cal-hint', payload.hint);
    text('explore-cal-state', payload.calibrated ? 'замеры внесены' : 'требуется замер');
    okTone($('explore-cal-state'), payload.calibrated);
    const box = $('explore-cal-fields');
    if (!box || !Array.isArray(payload.groups)) return;
    box.textContent = '';
    calibrationFields = [];
    payload.groups.forEach((group) => {
      const wrap = document.createElement('fieldset');
      wrap.className = 'explore-cal-group';
      const legend = document.createElement('legend');
      legend.textContent = group.title + (group.port ? ' · ' + group.port : '');
      wrap.appendChild(legend);
      (group.fields || []).forEach((field) => {
        const label = document.createElement('label');
        label.className = 'explore-cal-field';
        const caption = document.createElement('span');
        caption.textContent = field.label + (field.unit ? ', ' + field.unit : '');
        const input = document.createElement('input');
        input.type = 'number';
        input.step = String(field.step || 0.01);
        input.min = String(field.min);
        input.max = String(field.max);
        input.value = (field.value === null || field.value === undefined) ? '' : String(field.value);
        if (field.measured) input.placeholder = 'измерить на площадке';
        input.dataset.key = field.key;
        label.appendChild(caption);
        label.appendChild(input);
        wrap.appendChild(label);
        calibrationFields.push({ key: field.key, input });
      });
      box.appendChild(wrap);
    });
  }

  async function loadCalibration() {
    try {
      renderCalibration(await getJson('api/calibration/geometry'));
    } catch (err) {
      text('explore-cal-message', 'Не удалось прочитать геометрию: ' + (err.message || err));
    }
  }

  async function saveCalibration(event) {
    if (event && event.preventDefault) event.preventDefault();
    const values = {};
    // Пустые поля (ещё не измеренные) не отправляем: сервер не должен записать
    // нулевую скорость вместо отсутствующего замера.
    calibrationFields.forEach((entry) => {
      const raw = String(entry.input.value || '').trim();
      if (raw !== '') values[entry.key] = raw;
    });
    if (!Object.keys(values).length) {
      text('explore-cal-message', 'Поля ещё не загружены.');
      return;
    }
    busy = true;
    try {
      const body = await postJson('api/calibration/geometry', { values }, 'calibration');
      text('explore-cal-message', 'Сохранено полей: ' + (body.savedKeys || []).length +
        (body.calibrated ? ' · геометрия откалибрована' : ' · заполните все углы'));
      await loadCalibration();
      try { renderLayout(await getJson('api/sensors/layout')); } catch (e) { /* не критично */ }
    } catch (err) {
      text('explore-cal-message', 'Сохранение отклонено: ' + (err.message || err));
    } finally {
      busy = false;
    }
  }

  async function refresh() {
    if (document.hidden) return;
    try {
      const status = await getJson('api/exploration/status');
      render(status);
    } catch (err) {
      text('explore-status', 'нет связи');
      text('explore-message', err.message || String(err));
    }
  }

  async function refreshExtras() {
    if (document.hidden) return;
    try { renderLayout(await getJson('api/sensors/layout')); } catch (e) { /* экран живёт и без геометрии */ }
    try {
      const result = await getJson('api/exploration/landmarks?limit=3');
      renderLandmarks(result.landmarks);
    } catch (e) { /* нет ориентиров — не ошибка */ }
  }

  async function start() {
    const check = $('explore-blind');
    if (check && !check.checked) {
      text('explore-message', 'Подтвердите осмотр ближней зоны вокруг робота.');
      return;
    }
    busy = true;
    try {
      const body = await postJson('api/exploration/start', { blindZoneChecked: true });
      if (body.status) render(body.status);
      text('explore-message', 'Обследование запущено. Следите за роботом и держите E-STOP доступным.');
    } catch (err) {
      text('explore-message', 'Запуск отклонён: ' + (err.message || err));
    } finally {
      busy = false;
      await refresh();
    }
  }

  async function stop() {
    busy = true;
    try {
      const body = await postJson('api/exploration/stop', {});
      if (body.status) render(body.status);
      text('explore-message', 'Обследование остановлено, тяга снята.');
    } catch (err) {
      text('explore-message', 'Не удалось остановить: ' + (err.message || err));
    } finally {
      busy = false;
      await refresh();
    }
  }

  function init() {
    if (initialized) return;
    initialized = true;
    const startBtn = $('explore-start');
    const stopBtn = $('explore-stop');
    if (startBtn) startBtn.addEventListener('click', start);
    if (stopBtn) stopBtn.addEventListener('click', stop);
    const form = $('explore-cal-form');
    if (form) form.addEventListener('submit', saveCalibration);
    const reload = $('explore-cal-reload');
    if (reload) reload.addEventListener('click', loadCalibration);
    refresh();
    refreshExtras();
    loadCalibration();
    timer = setInterval(() => { refresh(); }, 1500);
    document.addEventListener('visibilitychange', () => {
      if (!document.hidden) { refresh(); refreshExtras(); }
    });
  }

  window.RCExplore = { init, refresh, start, stop, render, renderLayout,
    renderCalibration, loadCalibration, saveCalibration,
    stopPolling() { if (timer) clearInterval(timer); timer = null; } };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
