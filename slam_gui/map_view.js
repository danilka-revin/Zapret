/* Страница «Карта лидара» бортового экрана: живая карта, сброс, настройки.
 *
 * Карту строит сервер (robot_control/slam_map.py): он же впитывает сканы
 * лидара, считает позу и перезаписывает файл карты на рабочем столе.
 * Экран получает готовый кадр (PNG в base64) и рисует его как в ROS:
 *   • сетка занятости: белый — свободно, чёрный — препятствие, серый — не видели;
 *     для тёмной темы карта перекрашивается (свободно — почти чёрный,
 *     препятствие — приглушённый красный, «не видели» — тёмно-серый);
 *   • робот — треугольник с курсом, как модель в RViz;
 *   • поверх карты — живой скан лидара (текущие точки), как «LaserScan» в RViz.
 *     Ноды рисуются цветом, размером и прозрачностью, настроенными у лидара
 *     (окно «Настройки лидара» → «Отображение карты»: Flat Color или
 *     Min/Max Color, Size, Alpha), поэтому видны на карте в любом стиле;
 *   • сверху — статус MAPPING, как у карты в ROS.
 *
 * Настройки карты (сетка, дальность, чувствительность, забывание старых
 * препятствий, автосохранение) меняются здесь же и уходят на сервер:
 * POST api/kiosk_map/settings.
 */
(function () {
  'use strict';
  if (window.RCMapView) return;

  const POLL_MS = 700;
  const DEFAULTS = {
    resolution: 0.10,
    size_m: 30.0,
    max_range: 8.0,
    min_range: 0.30,
    hit: 0.65,
    miss: 0.30,
    decay: 0.0,
    matching: true,
    autosave: true,
    autosave_s: 60.0,
  };

  // Пресеты чувствительности: «мягко» — стены замечаются сразу, «жёстко» —
  // одиночные точки не мешают карте.
  const SENSITIVITY = {
    soft: { hit: 0.45, miss: 0.20 },
    normal: { hit: 0.65, miss: 0.30 },
    hard: { hit: 0.85, miss: 0.45 },
  };

  // Палитра карты. Выбор «тёмная / светлая» общий для страницы /map и карты
  // внутри бортового экрана и хранится в localStorage.
  // Ноды живого скана по умолчанию рисуются цветом, настроенным у лидара
  // (lidar_view.js: Flat Color или Min/Max Color) — см. scanStyle(). Цвета
  // ниже остаются запасными, когда модуль лидара не загружен (отдельная
  // страница /map без lidar_view.js): белые точки на тёмной карте и тёмные
  // на светлой, чтобы ноды были видны всегда.
  const THEME_KEY = 'kiosk_map_theme';
  const MAP_THEMES = {
    dark: {
      bg: '#08070a',
      grid: 'rgba(255, 255, 255, .07)',
      text: 'rgba(226, 214, 220, .78)',
      scan: 'rgba(244, 244, 246, .92)',
      robot: 'rgba(192, 80, 90, .95)',
      robotStroke: '#ffffff',
      // Значения сетки занятости с сервера: 254 — свободно, 0 — препятствие,
      // 205 — «не видели». Перекрашиваем PNG под тёмную тему.
      free: [18, 14, 20],
      occupied: [192, 80, 90],
      unknown: [42, 36, 46],
    },
    light: {
      bg: '#ffffff',
      grid: 'rgba(20, 10, 14, .10)',
      text: 'rgba(30, 20, 24, .72)',
      scan: 'rgba(30, 20, 24, .88)',
      robot: 'rgba(192, 80, 90, .95)',
      robotStroke: '#3a0b0f',
      free: [255, 255, 255],
      occupied: [23, 18, 21],
      unknown: [215, 208, 211],
    },
  };

  const $ = (id) => document.getElementById(id);
  const setText = (el, text) => { if (el) el.textContent = text; };

  const state = {
    snapshot: null,
    image: null,
    imageKey: '',
    rendered: null,
    renderedKey: '',
    theme: 'dark',
    scan: [],
    follow: false,
    zoom: 1,
    panX: 0,
    panY: 0,
    timer: null,
    scanTimer: null,
    busy: false,
    scanBusy: false,
    initialized: false,
    lastError: '',
  };

  function fmt(value, digits) {
    if (value === null || value === undefined || !Number.isFinite(+value)) return '—';
    return (+value).toFixed(digits == null ? 1 : digits);
  }

  function msg(text, tone) {
    const el = $('map-msg');
    if (!el) return;
    el.textContent = text || '';
    el.dataset.tone = tone || '';
  }

  /* ------------------------------------------------------------- картинка */
  function loadImage(pngBase64) {
    if (!pngBase64) return;
    if (pngBase64 === state.imageKey && state.image) return;
    state.imageKey = pngBase64;
    state.rendered = null;
    state.renderedKey = '';
    const image = new Image();
    image.onload = () => { state.image = image; draw(); };
    image.src = 'data:image/png;base64,' + pngBase64;
  }

  function palette() {
    return MAP_THEMES[state.theme] || MAP_THEMES.dark;
  }

  /* Перекрасить серую карту сервера под выбранную тему. В Node-прогоне без
   * настоящего canvas возвращаем исходную картинку — отрисовка не падает. */
  function themedMapImage() {
    const image = state.image;
    if (!image || !image.complete || !image.naturalWidth) return image;
    const key = state.imageKey + '|' + state.theme;
    if (state.rendered && state.renderedKey === key) return state.rendered;
    const pal = palette();
    try {
      const canvas = document.createElement('canvas');
      canvas.width = image.naturalWidth;
      canvas.height = image.naturalHeight;
      const ctx = canvas.getContext && canvas.getContext('2d');
      if (!ctx || typeof ctx.getImageData !== 'function' || typeof ctx.putImageData !== 'function') return image;
      ctx.drawImage(image, 0, 0);
      const frame = ctx.getImageData(0, 0, canvas.width, canvas.height);
      const pixels = frame && frame.data;
      if (!pixels || !pixels.length) return image;
      for (let i = 0; i < pixels.length; i += 4) {
        const value = (pixels[i] + pixels[i + 1] + pixels[i + 2]) / 3;
        const rgb = value >= 245 ? pal.free : (value <= 24 ? pal.occupied : pal.unknown);
        pixels[i] = rgb[0];
        pixels[i + 1] = rgb[1];
        pixels[i + 2] = rgb[2];
      }
      ctx.putImageData(frame, 0, 0);
      state.rendered = canvas;
      state.renderedKey = key;
      return canvas;
    } catch (err) {
      return image;
    }
  }

  function updateThemeButton() {
    const button = $('map-theme-toggle');
    if (!button) return;
    button.textContent = state.theme === 'dark' ? 'Светлая карта' : 'Тёмная карта';
    button.setAttribute('aria-pressed', String(state.theme === 'light'));
    button.setAttribute('title', state.theme === 'dark'
      ? 'Переключить на светлую карту' : 'Переключить на тёмную карту');
  }

  function applyTheme(theme, persist) {
    state.theme = theme === 'light' ? 'light' : 'dark';
    try { document.documentElement.setAttribute('data-map-theme', state.theme); } catch (err) {}
    // Цвет адресной строки мобильного браузера — под выбранную тему.
    const meta = document.querySelector && document.querySelector('meta[name="theme-color"]');
    if (meta) meta.setAttribute('content', state.theme === 'light' ? '#f3f1f2' : '#000000');
    if (persist) {
      try { localStorage.setItem(THEME_KEY, state.theme); } catch (err) {}
    }
    state.rendered = null;
    state.renderedKey = '';
    updateThemeButton();
    draw();
  }

  function loadTheme() {
    let theme = 'dark';
    try { theme = localStorage.getItem(THEME_KEY) === 'light' ? 'light' : 'dark'; } catch (err) {}
    applyTheme(theme, false);
  }

  /* -------------------------------------------------------------- отрисовка */
  // Мировые координаты карты: x — вперёд, y — влево (как у лидара).
  function worldToCanvas(mx, my, view) {
    return {
      x: view.x0 + (mx - view.originX) * view.scale,
      y: view.y0 - (my - view.originY) * view.scale,
    };
  }

  function computeView(ctx, width, height, snap) {
    const res = snap.resolution || DEFAULTS.resolution;
    const originX = snap.origin ? snap.origin.x : 0;
    const originY = snap.origin ? snap.origin.y : 0;
    const mapW = (snap.width || 0) * res;
    const mapH = (snap.height || 0) * res;
    const pose = snap.pose || { x: 0, y: 0, yaw: 0 };
    let scale;
    let cx;
    let cy;
    if (state.follow) {
      // Следим за роботом: вид centered на текущей позе.
      scale = Math.min(width, height) / 12 * state.zoom;   // 12 м поперёк экрана
      cx = width / 2;
      cy = height / 2;
    } else {
      // Вся карта целиком в кадре.
      scale = Math.min(width / Math.max(0.1, mapW), height / Math.max(0.1, mapH)) * 0.96 * state.zoom;
      cx = width / 2;
      cy = height / 2;
    }
    // Центр карты в мире: середина сетки.
    const centerX = originX + mapW / 2;
    const centerY = originY + mapH / 2;
    let x0;
    let y0;
    if (state.follow) {
      // Робот — в центре экрана: сдвигаем начало мира так, чтобы он туда встал.
      x0 = cx - (pose.x - originX) * scale;
      y0 = cy + (pose.y - originY) * scale;
    } else {
      x0 = cx - mapW * scale / 2;
      y0 = cy + mapH * scale / 2;
    }
    return { scale, x0: x0 + state.panX, y0: y0 + state.panY, originX, originY, mapW, mapH, centerX, centerY };
  }

  function drawRobot(ctx, view, pose, pal) {
    const p = worldToCanvas(pose.x, pose.y, view);
    const size = 14;
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-pose.yaw);            // экран: y вниз, поэтому yaw со знаком минус
    ctx.beginPath();
    ctx.moveTo(0, -size);
    ctx.lineTo(size * 0.62, size * 0.72);
    ctx.lineTo(0, size * 0.34);
    ctx.lineTo(-size * 0.62, size * 0.72);
    ctx.closePath();
    ctx.fillStyle = pal.robot;
    ctx.fill();
    ctx.strokeStyle = pal.robotStroke;
    ctx.lineWidth = 1.4;
    ctx.stroke();
    ctx.restore();
  }

  /* Ноды живого скана: цвет, прозрачность и размер точки берём из настроек
   * отображения лидара (lidar_view.js — то же окно «Настройки лидара»):
   * Flat Color — один цвет, Range — градиент Min/Max Color по дальности,
   * Size — размер точки в метрах, как в RViz. Тогда ноды на карте видны
   * ровно тем цветом, который настроен у лидара. Без модуля лидара
   * (отдельная страница /map) остаётся запасной цвет темы. */
  function lidarDisplay() {
    if (!window.RCLidarView || typeof window.RCLidarView.getSettings !== 'function') return null;
    try {
      const settings = window.RCLidarView.getSettings();
      return settings && typeof settings === 'object' ? settings : null;
    } catch (err) {
      return null;
    }
  }

  function hexToRgb(hex) {
    if (typeof hex !== 'string' || !/^#[0-9a-f]{6}$/i.test(hex)) return null;
    const n = parseInt(hex.slice(1), 16);
    return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
  }

  function scanStyle(settings) {
    if (!settings) return null;
    const alpha = Number.isFinite(+settings.alpha) ? Math.min(1, Math.max(0.1, +settings.alpha)) : 1;
    const sizeM = Number.isFinite(+settings.size) ? +settings.size : 0.08;
    if (settings.color === 'flat') {
      const rgb = hexToRgb(settings.flatColor);
      if (!rgb) return null;
      return {
        alpha,
        sizeM,
        colorOf: () => 'rgb(' + rgb[0] + ',' + rgb[1] + ',' + rgb[2] + ')',
      };
    }
    const a = hexToRgb(settings.minColor);
    const b = hexToRgb(settings.maxColor);
    if (!a || !b) return null;
    const lo = Number(settings.minValue);
    const hi = Number(settings.maxValue);
    const span = Number.isFinite(lo) && Number.isFinite(hi) && hi > lo ? hi - lo : 1;
    return {
      alpha,
      sizeM,
      colorOf: (r) => {
        const t = Math.min(1, Math.max(0, (r - lo) / span));
        const mix = (i) => Math.round(a[i] + (b[i] - a[i]) * t);
        return 'rgb(' + mix(0) + ',' + mix(1) + ',' + mix(2) + ')';
      },
    };
  }

  function drawScan(ctx, view, pose, width, height, pal) {
    if (!state.scan.length) return;
    const style = scanStyle(lidarDisplay());
    const dot = style
      ? Math.max(2, Math.min(10, style.sizeM * view.scale))   // Size (m) как в RViz
      : Math.max(2, Math.min(4, view.scale * 0.05));          // запасной размер темы
    if (style) ctx.globalAlpha = style.alpha;
    const cos = Math.cos(pose.yaw);
    const sin = Math.sin(pose.yaw);
    state.scan.forEach((point) => {
      if (!Number.isFinite(point.a) || !Number.isFinite(point.r) || point.r < 0) return;
      const lx = point.r * Math.cos(point.a);
      const ly = point.r * Math.sin(point.a);
      const mx = pose.x + lx * cos - ly * sin;
      const my = pose.y + lx * sin + ly * cos;
      const p = worldToCanvas(mx, my, view);
      if (p.x < -4 || p.y < -4 || p.x > width + 4 || p.y > height + 4) return;
      ctx.fillStyle = style ? style.colorOf(point.r) : pal.scan;
      ctx.fillRect(p.x - dot / 2, p.y - dot / 2, dot, dot);
    });
    if (style) ctx.globalAlpha = 1;
  }

  function draw() {
    const canvas = $('map-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext && canvas.getContext('2d');
    if (!ctx) return;
    const snap = state.snapshot;
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect ? canvas.getBoundingClientRect() : { width: 640, height: 640 };
    const width = Math.max(320, Math.round((rect.width || 640)));
    const height = Math.max(240, Math.round((rect.height || 640)));
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
    }
    const pal = palette();
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = pal.bg;
    ctx.fillRect(0, 0, width, height);
    if (!snap) {
      ctx.fillStyle = pal.text;
      ctx.font = '13px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('карта ещё не собрана', width / 2, height / 2);
      return;
    }
    const view = computeView(ctx, width, height, snap);
    // Сама сетка занятости (картинка с сервера), перекрашенная под тему.
    if (state.image && state.image.complete && state.image.naturalWidth) {
      const destW = view.mapW * view.scale;
      const destH = view.mapH * view.scale;
      const topLeft = worldToCanvas(view.originX, view.originY + view.mapH, view);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(themedMapImage(), topLeft.x, topLeft.y, destW, destH);
    }
    // Сетка метров — как в RViz.
    drawGrid(ctx, view, width, height, pal);
    drawScan(ctx, view, snap.pose || { x: 0, y: 0, yaw: 0 }, width, height, pal);
    drawRobot(ctx, view, snap.pose || { x: 0, y: 0, yaw: 0 }, pal);
    // Подпись: как в ROS — кадр, сетка, масштаб.
    ctx.fillStyle = pal.text;
    ctx.font = '11px ui-monospace, monospace';
    ctx.textAlign = 'left';
    const step = niceStep(view.scale);
    ctx.fillText('map · ' + step + ' м/клетка · ' + fmt(snap.stats && snap.stats.coverage * 100, 0) + '% изучено', 10, height - 10);
  }

  function niceStep(scale) {
    const steps = [0.1, 0.2, 0.5, 1, 2, 5, 10];
    for (let i = 0; i < steps.length; i += 1) {
      if (steps[i] * scale >= 26) return steps[i];
    }
    return steps[steps.length - 1];
  }

  function drawGrid(ctx, view, width, height, pal) {
    const step = niceStep(view.scale);
    const minX = view.originX + (0 - view.x0) / view.scale;
    const maxX = view.originX + (width - view.x0) / view.scale;
    const maxY = view.originY + (view.y0 - 0) / view.scale;
    const minY = view.originY + (view.y0 - height) / view.scale;
    ctx.strokeStyle = pal.grid;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(minX / step) * step; x <= maxX; x += step) {
      const sx = view.x0 + (x - view.originX) * view.scale;
      ctx.moveTo(sx, 0);
      ctx.lineTo(sx, height);
    }
    for (let y = Math.ceil(minY / step) * step; y <= maxY; y += step) {
      const sy = view.y0 - (y - view.originY) * view.scale;
      ctx.moveTo(0, sy);
      ctx.lineTo(width, sy);
    }
    ctx.stroke();
  }

  /* ------------------------------------------------------------- статистика */
  function renderStats(snap) {
    const stats = snap.stats || {};
    const pose = snap.pose || { x: 0, y: 0, yaw: 0 };
    setText($('map-size'), stats.sizeM ? fmt(stats.sizeM, 1) + ' × ' + fmt(stats.sizeM, 1) + ' м' : '—');
    setText($('map-res'), fmt(snap.resolution, 2) + ' м/клетка · ' + (stats.width || 0) + '×' + (stats.height || 0));
    setText($('map-cov'), fmt((stats.coverage || 0) * 100, 1) + ' %');
    setText($('map-occ'), String(stats.occupied === undefined ? '—' : stats.occupied));
    setText($('map-pose'), fmt(pose.x, 2) + ' / ' + fmt(pose.y, 2) + ' м · ' + fmt(pose.yaw * 180 / Math.PI, 0) + '°');
    setText($('map-scans'), String(snap.scans || 0) + ' · точек ' + (snap.points || 0));
    const paths = snap.paths || {};
    setText($('map-file'), 'файл: ' + (paths.pgm || '—'));
    const meta = $('map-meta');
    if (meta) {
      const when = snap.updatedAt ? new Date(snap.updatedAt * 1000) : null;
      meta.textContent = snap.scans
        ? (when ? 'обновлено ' + when.toLocaleTimeString('ru-RU', { hour12: false }) : 'обновляется')
        : 'ждём скан лидара';
    }
    const badge = $('map-badge');
    if (badge) {
      badge.textContent = snap.scans ? 'MAPPING' : 'IDLE';
      badge.dataset.tone = snap.scans ? 'live' : 'idle';
    }
    const empty = $('map-empty');
    if (empty) {
      const noScan = !snap.scans;
      empty.hidden = !noScan;
      if (noScan) setText(empty, 'скана лидара нет — карта не растёт');
    }
    const quality = $('map-quality');
    if (quality) {
      quality.textContent = snap.matched
        ? 'сопоставление сканов: ' + Math.round((snap.quality || 0) * 100) + ' %'
        : 'сопоставление сканов: —';
    }
  }

  /* -------------------------------------------------------------- настройки */
  function fillSettings(settings) {
    const s = settings || DEFAULTS;
    const resolution = $('map-set-resolution');
    if (resolution) resolution.value = String(s.resolution);
    const size = $('map-set-size');
    if (size) size.value = String(s.size_m);
    const range = $('map-set-range');
    if (range) range.value = String(s.max_range);
    const sensitivity = $('map-set-sensitivity');
    if (sensitivity) {
      const found = Object.keys(SENSITIVITY).find((key) =>
        Math.abs(SENSITIVITY[key].hit - s.hit) < 0.01 && Math.abs(SENSITIVITY[key].miss - s.miss) < 0.01);
      sensitivity.value = found || 'normal';
    }
    const decay = $('map-set-decay');
    if (decay) decay.value = String(s.decay);
    const matching = $('map-set-matching');
    if (matching) matching.checked = s.matching !== false;
    const autosave = $('map-set-autosave');
    if (autosave) autosave.checked = s.autosave !== false;
    const autosaveEvery = $('map-set-autosave-every');
    if (autosaveEvery) autosaveEvery.value = String(s.autosave_s);
  }

  function collectSettings() {
    const out = {};
    const resolution = $('map-set-resolution');
    if (resolution) out.resolution = Number(resolution.value);
    const size = $('map-set-size');
    if (size) out.size_m = Number(size.value);
    const range = $('map-set-range');
    if (range) out.max_range = Number(range.value);
    const sensitivity = $('map-set-sensitivity');
    if (sensitivity && SENSITIVITY[sensitivity.value]) {
      out.hit = SENSITIVITY[sensitivity.value].hit;
      out.miss = SENSITIVITY[sensitivity.value].miss;
    }
    const decay = $('map-set-decay');
    if (decay) out.decay = Number(decay.value);
    const matching = $('map-set-matching');
    if (matching) out.matching = matching.checked;
    const autosave = $('map-set-autosave');
    if (autosave) out.autosave = autosave.checked;
    const autosaveEvery = $('map-set-autosave-every');
    if (autosaveEvery) out.autosave_s = Number(autosaveEvery.value);
    return out;
  }

  /* ----------------------------------------------------------------- опрос */
  async function fetchMap() {
    if (state.busy) return;
    // Пока страница закрыта, карту спрашивать незачем: она всё равно
    // строится на сервере, а экран только показывает.
    const page = $('swipe-map-view');
    if (page && page.hidden) return;
    state.busy = true;
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 5000);
    try {
      const response = await fetch('api/kiosk_map', { cache: 'no-store', signal: controller.signal });
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const body = await response.json();
      if (!body || !body.ok) throw new Error('пустой ответ сервера');
      state.snapshot = body;
      state.lastError = '';
      loadImage(body.png);
      renderStats(body);
      draw();
    } catch (err) {
      state.lastError = String((err && err.message) || err);
      const meta = $('map-meta');
      if (meta) meta.textContent = 'Нет связи с бортовым сервером карты · повторяем подключение';
      const badge = $('map-badge');
      if (badge) { badge.textContent = 'OFFLINE'; badge.dataset.tone = 'err'; }
    } finally {
      clearTimeout(timeout);
      state.busy = false;
    }
  }

  /* Живой скан для отдельной страницы /map: на бортовом экране точки кладёт
   * robot.js, а здесь берём их из api/lidar — тогда ноды лидара видны на
   * карте и на отдельной странице, а не только на киоске. */
  async function fetchScan() {
    if (state.scanBusy) return;
    const page = $('swipe-map-view');
    if (page && page.hidden) return;
    state.scanBusy = true;
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 5000);
    try {
      const response = await fetch('api/lidar', { cache: 'no-store', signal: controller.signal });
      if (!response.ok) return;
      const body = await response.json();
      const points = body && Array.isArray(body.scan) ? body.scan : [];
      state.scan = points.filter((point) =>
        point && Number.isFinite(point.a) && Number.isFinite(point.r) && point.r >= 0);
      if (state.snapshot) draw();
    } catch (err) {
      // Тихо: без скана карта просто не дополняется точками.
    } finally {
      clearTimeout(timeout);
      state.scanBusy = false;
    }
  }

  async function post(url, payload) {
    const response = await fetch(url, {
      method: 'POST',
      cache: 'no-store',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload || {}),
    });
    const body = await response.json().catch(() => null);
    if (!response.ok || !body || !body.ok) {
      throw new Error((body && body.error) || ('HTTP ' + response.status));
    }
    return body;
  }

  async function resetMap() {
    if (state.busy) return;
    const button = $('map-reset');
    if (button) {
      button.disabled = true;
      button.textContent = 'Сбрасываю…';
    }
    try {
      const body = await post('api/kiosk_map/reset');
      msg('Карта очищена, файл обновлён: ' + ((body.paths && body.paths.pgm) || ''), 'ok');
      await fetchMap();
    } catch (err) {
      msg('Не удалось сбросить карту: ' + (err.message || err), 'err');
    } finally {
      if (button) {
        button.disabled = false;
        button.textContent = 'Сбросить карту';
      }
    }
  }

  async function saveMap() {
    if (state.busy) return;
    const button = $('map-save');
    if (button) {
      button.disabled = true;
      button.textContent = 'Сохраняю…';
    }
    try {
      const body = await post('api/kiosk_map/save');
      msg('Карта сохранена: ' + ((body.paths && body.paths.pgm) || ''), 'ok');
    } catch (err) {
      msg('Не удалось сохранить карту: ' + (err.message || err), 'err');
    } finally {
      if (button) {
        button.disabled = false;
        button.textContent = 'Сохранить карту';
      }
    }
  }

  async function applySettings() {
    if (state.busy) return;
    const button = $('map-settings-apply');
    if (button) {
      button.disabled = true;
      button.textContent = 'Применяю…';
    }
    try {
      const body = await post('api/kiosk_map/settings', { settings: collectSettings() });
      fillSettings(body.settings);
      msg(body.mapReset
        ? 'Настройки применены. Сетка изменилась — карта очищена, строим заново.'
        : 'Настройки применены', body.mapReset ? 'warn' : 'ok');
      await fetchMap();
    } catch (err) {
      msg('Не удалось применить настройки: ' + (err.message || err), 'err');
    } finally {
      if (button) {
        button.disabled = false;
        button.textContent = 'Применить';
      }
    }
  }

  function toggleSettings(force) {
    const panel = $('map-settings');
    const button = $('map-settings-toggle');
    if (!panel) return;
    const hidden = force === undefined ? !panel.hidden : !force;
    panel.hidden = hidden;
    if (button) button.setAttribute('aria-expanded', String(!hidden));
  }

  /* ------------------------------------------------------------------ пуск */
  function init() {
    if (state.initialized) return;
    state.initialized = true;
    const themeButton = $('map-theme-toggle');
    if (themeButton) {
      themeButton.addEventListener('click', () => {
        applyTheme(state.theme === 'dark' ? 'light' : 'dark', true);
      });
    }
    loadTheme();
    const reset = $('map-reset');
    if (reset) reset.addEventListener('click', () => { void resetMap(); });
    const save = $('map-save');
    if (save) save.addEventListener('click', () => { void saveMap(); });
    const apply = $('map-settings-apply');
    if (apply) apply.addEventListener('click', () => { void applySettings(); });
    const toggle = $('map-settings-toggle');
    if (toggle) toggle.addEventListener('click', () => toggleSettings());
    const follow = $('map-follow');
    if (follow) {
      follow.addEventListener('click', () => {
        state.follow = !state.follow;
        follow.classList.toggle('is-on', state.follow);
        follow.textContent = state.follow ? 'Следить за роботом' : 'Вся карта';
        draw();
      });
    }
    const canvas = $('map-canvas');
    if (canvas) {
      canvas.tabIndex = 0;
      const viewport = window.RCViewport ? window.RCViewport(canvas, (view) => {
        state.zoom = view.zoom; state.panX = view.x; state.panY = view.y; draw();
      }, { min: 0.4, max: 8 }) : null;
      const zoom = (factor) => {
        if (viewport) viewport.zoomTo(viewport.state.zoom * factor);
        else { state.zoom = Math.max(0.4, Math.min(8, state.zoom * factor)); draw(); }
      };
      for (const [id, factor] of [['map-zoom-in', 1.25], ['map-zoom-out', 0.8]]) {
        const button = $(id);
        if (button) button.addEventListener('click', () => zoom(factor));
      }
      const fit = $('map-fit');
      if (fit) fit.addEventListener('click', () => {
        state.follow = false;
        if (follow) { follow.classList.remove('is-on'); follow.textContent = 'Вся карта'; }
        if (viewport) viewport.reset();
        else { state.zoom = 1; state.panX = state.panY = 0; draw(); }
      });
      if (!viewport) canvas.addEventListener('wheel', (event) => {
        event.preventDefault(); zoom(event.deltaY > 0 ? 0.9 : 1.1);
      }, { passive: false });
    }
    window.addEventListener('resize', () => draw());
    fillSettings(DEFAULTS);
    void fetchMap();
    if (state.timer) clearInterval(state.timer);
    state.timer = setInterval(() => { void fetchMap(); }, POLL_MS);
    // Отдельная страница /map: скан лидара никто не кладёт (на киоске это
    // делает robot.js), поэтому спрашиваем сами — ноды видны на карте сразу.
    const isKiosk = document.body && document.body.classList
      && document.body.classList.contains('kiosk-body');
    if (!isKiosk) {
      void fetchScan();
      if (state.scanTimer) clearInterval(state.scanTimer);
      state.scanTimer = setInterval(() => { void fetchScan(); }, POLL_MS);
    }
  }

  window.RCMapView = {
    init,
    show() {
      init();
      draw();
      void fetchMap();
    },
    draw,
    setScan(points) {
      state.scan = Array.isArray(points) ? points : [];
      if (state.snapshot) draw();
    },
    getSnapshot() { return state.snapshot; },
  };
})();
