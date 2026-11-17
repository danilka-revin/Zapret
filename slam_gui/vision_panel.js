/* YOLO-разметка на панели четырёх камер: рамки, светофор и поле со знаками.
 *
 * Один модуль обслуживает и инженерный пульт (main.html), и бортовой киоск
 * (robot.html), и страницу карты (map.html — там рисуются только отметки).
 * Кадры с рамками отдаёт сервер (/api/vision/camera/N/frame), поэтому
 * разметка не расходится с цифровым зумом и панорамированием камер.
 */
(function () {
  'use strict';
  if (window.RCVision) return;

  const $ = (id) => document.getElementById(id);
  const SIGNALS = ['red', 'yellow', 'green'];
  const SIGNAL_NAMES = {red: 'КРАСНЫЙ', yellow: 'ЖЁЛТЫЙ', green: 'ЗЕЛЁНЫЙ', unknown: 'НЕ ОПРЕДЕЛЁН'};
  const CAMERA_NAMES = ['ПЕРЕД-Л', 'ЗАД-Л', 'ПЕРЕД-П', 'ЗАД-П'];
  const state = {snapshot: null, overlay: true, online: false, listeners: new Set(), timer: null};

  function sessionUrl(url) {
    return window.RCSessionUrl ? window.RCSessionUrl(url) : url;
  }

  function emit(snapshot) {
    for (const listener of state.listeners) {
      try { listener(snapshot); } catch (error) { console.warn('vision listener failed', error); }
    }
    try {
      window.dispatchEvent(new CustomEvent('rc-vision-state', {detail: snapshot}));
    } catch (error) { /* старый браузер без CustomEvent — панель всё равно работает */ }
  }

  function frameUrl(camId) {
    const annotated = state.overlay && state.online && state.snapshot &&
      state.snapshot.enabled && state.snapshot.ok;
    const path = annotated
      ? 'api/vision/camera/' + camId + '/frame'
      : 'api/camera/' + camId + '/frame';
    return sessionUrl(path + '?t=' + Date.now());
  }

  function setOverlay(value) {
    state.overlay = !!value;
    const box = $('vision-annot');
    if (box) box.checked = state.overlay;
  }

  /* ------------------------------------------------------------ отрисовка */
  function renderLights(snapshot) {
    const traffic = snapshot.trafficLight || {state: 'unknown', counts: {}};
    const counts = traffic.counts || {};
    const max = Math.max(1, ...SIGNALS.map((name) => Number(counts[name] || 0)));
    SIGNALS.forEach((name) => {
      const lamp = document.querySelector('.rcv-lamp[data-signal="' + name + '"]');
      if (!lamp) return;
      const value = Number(counts[name] || 0);
      lamp.classList.toggle('is-on', traffic.state === name);
      lamp.classList.toggle('is-seen', value > 0);
      lamp.style.setProperty('--rcv-strength', String(0.25 + 0.75 * value / max));
      lamp.title = SIGNAL_NAMES[name] + ' · пикселей: ' + value;
    });
    const label = $('vision-signal');
    if (label) {
      label.textContent = (traffic.label || SIGNAL_NAMES[traffic.state] || SIGNAL_NAMES.unknown) +
        (traffic.cameraId !== null && traffic.cameraId !== undefined
          ? ' · CAM ' + traffic.cameraId : '');
      label.dataset.state = traffic.state || 'unknown';
    }
  }

  function renderSigns(snapshot) {
    const host = $('vision-signs');
    if (!host) return;
    const counts = snapshot.counts || {};
    const classes = snapshot.classes || [];
    const found = classes.filter((item) => Number(counts[item.name] || 0) > 0);
    if (!found.length) {
      host.innerHTML = '<span class="rcv-none">знаков в кадре нет</span>';
      return;
    }
    host.innerHTML = found.map((item) =>
      '<span class="rcv-chip" style="--rcv-color:' + item.color + '">' +
      '<i></i><b>' + item.short + '</b> ' + Number(counts[item.name]) + '</span>').join('');
  }

  function renderMeta(snapshot) {
    const backend = $('vision-backend');
    if (backend) {
      const names = {yolo: 'YOLO', 'yolo+heuristic': 'YOLO + эвристика',
        heuristic: 'встроенный детектор', unavailable: 'недоступно'};
      backend.textContent = (names[snapshot.backend] || snapshot.backend) +
        (snapshot.enabled ? '' : ' · выключено');
      backend.dataset.state = snapshot.ok ? 'ok' : 'off';
    }
    const cameras = $('vision-cameras');
    if (cameras) {
      cameras.textContent = 'камеры: ' + [0, 1, 2, 3].map((id) => {
        const cam = snapshot.cameras && snapshot.cameras[String(id)];
        const n = cam && cam.detections ? cam.detections.length : 0;
        return CAMERA_NAMES[id] + ' ' + n;
      }).join(' · ');
    }
    const fps = $('vision-fps');
    if (fps) fps.textContent = snapshot.ok
      ? (String(snapshot.backend).indexOf('yolo') === 0 ? 'YOLO · ' : '') +
        snapshot.inferenceMs + ' мс/кадр'
      : (snapshot.reason || 'нет данных');
  }

  function drawField(snapshot) {
    const canvas = $('vision-field');
    if (!canvas || !canvas.getContext) return;
    const ctx = canvas.getContext('2d');
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const rect = canvas.getBoundingClientRect();
    const w = Math.max(180, Math.round(rect.width || canvas.width));
    const h = Math.max(140, Math.round(rect.height || canvas.height));
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#0c1016';
    ctx.fillRect(0, 0, w, h);

    const cx = w / 2, bottom = h - 14;
    const range = 4.0;                       // метров вперёд от робота
    const scale = Math.min((h - 30) / range, (w - 20) / (range * 1.4));
    ctx.strokeStyle = 'rgba(255,255,255,.08)';
    ctx.lineWidth = 1;
    for (let meter = 1; meter <= range; meter += 1) {
      ctx.beginPath();
      ctx.arc(cx, bottom, meter * scale, Math.PI, 2 * Math.PI);
      ctx.stroke();
      ctx.fillStyle = 'rgba(255,255,255,.28)';
      ctx.font = '9px ui-monospace, monospace';
      ctx.fillText(meter + ' м', cx + 4, bottom - meter * scale - 2);
    }
    ctx.beginPath();
    ctx.moveTo(cx, bottom); ctx.lineTo(cx, bottom - range * scale);
    ctx.strokeStyle = 'rgba(255,255,255,.12)';
    ctx.stroke();

    // Робот внизу по центру: ось X — вперёд, ось Y — влево (как у лидара).
    ctx.fillStyle = '#7dd3fc';
    ctx.fillRect(cx - 9, bottom - 5, 18, 9);
    ctx.fillStyle = 'rgba(255,255,255,.55)';
    ctx.font = '9px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('РОБОТ', cx, bottom + 11);

    const markers = (snapshot.field || []).slice(0, 24);
    markers.forEach((marker) => {
      const x = cx - Number(marker.y_m || 0) * scale;
      const y = bottom - Number(marker.x_m || 0) * scale;
      if (x < 4 || x > w - 4 || y < 4 || y > h - 4) return;
      const color = marker.class === 'traffic_light' && marker.signal &&
        {red: '#ff4d5e', yellow: '#ffd23f', green: '#3ddc84'}[marker.signal] || marker.color;
      ctx.beginPath();
      ctx.arc(x, y, 8, 0, Math.PI * 2);
      ctx.fillStyle = color || '#fff';
      ctx.fill();
      ctx.strokeStyle = 'rgba(255,255,255,.85)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.fillStyle = '#0c1016';
      ctx.font = '700 8px ui-monospace, monospace';
      ctx.textAlign = 'center';
      ctx.fillText(marker.short ? marker.short.slice(0, 3) : '•', x, y + 3);
    });
    if (!markers.length) {
      ctx.fillStyle = 'rgba(255,255,255,.45)';
      ctx.font = '10px Inter, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('поле: отметок нет', cx, 18);
    }
  }

  function render(snapshot) {
    renderLights(snapshot);
    renderSigns(snapshot);
    renderMeta(snapshot);
    drawField(snapshot);
  }

  /* --------------------------------------------------------------- опрос */
  async function poll() {
    if (document.hidden) return;
    try {
      const response = await fetch(sessionUrl('api/vision/state?t=' + Date.now()),
        {cache: 'no-store'});
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const snapshot = await response.json();
      if (!snapshot || typeof snapshot !== 'object') throw new Error('bad payload');
      state.snapshot = snapshot;
      state.online = !!snapshot.ok;
      render(snapshot);
      emit(snapshot);
    } catch (error) {
      state.online = false;
      const backend = $('vision-backend');
      if (backend) { backend.textContent = 'нет связи с распознаванием'; backend.dataset.state = 'off'; }
    }
  }

  function init() {
    const box = $('vision-annot');
    if (box) {
      box.checked = state.overlay;
      box.addEventListener('change', () => setOverlay(box.checked));
    }
    window.addEventListener('resize', () => { if (state.snapshot) drawField(state.snapshot); });
    poll();
    state.timer = setInterval(poll, 800);
  }

  window.RCVision = {
    frameUrl,
    setOverlay,
    getState: () => state.snapshot,
    onState(listener) {
      if (typeof listener !== 'function') return () => {};
      state.listeners.add(listener);
      return () => state.listeners.delete(listener);
    },
    refresh: poll,
  };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
