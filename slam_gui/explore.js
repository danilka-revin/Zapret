/* Автономное обследование: карта растёт сама, оператор только запускает и стопит.
 * Работает и на киоске (8090), и на пульте (8080): команды идут относительными
 * URL, CSRF-токен берётся из /api/exploration/status, как у навигации.
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

  async function postJson(url, payload) {
    const headers = { 'Content-Type': 'application/json', 'X-Robot-Control': 'exploration' };
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

  function render(status) {
    if (status.csrfToken) csrf = status.csrfToken;
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
    const startBtn = $('explore-start');
    const stopBtn = $('explore-stop');
    if (startBtn) startBtn.disabled = busy || status.active || !status.enabled || !status.navigationEnabled;
    if (stopBtn) stopBtn.disabled = busy || !status.active;
  }

  function renderLayout(layout) {
    if (!layout || !Array.isArray(layout.cameras)) return;
    const cams = layout.cameras.slice(0, 4)
      .map((cam) => `${cam.role}: ${cam.yaw_deg}°`)
      .join(' · ');
    text('explore-cameras', cams || '—');
    text('explore-calibration', layout.calibrated
      ? 'углы заданы в .env'
      : 'углы не откалиброваны — заполните RC_CAMERA_* / RC_KINECT_* в .env');
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
    refresh();
    refreshExtras();
    timer = setInterval(() => { refresh(); }, 1500);
    document.addEventListener('visibilitychange', () => {
      if (!document.hidden) { refresh(); refreshExtras(); }
    });
  }

  window.RCExplore = { init, refresh, start, stop, render,
    stopPolling() { if (timer) clearInterval(timer); timer = null; } };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
