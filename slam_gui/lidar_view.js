/* Карта лидара в стиле RViz (LaserScan + Grid + Axes + RobotModel).
 *
 * Общий модуль для основного экрана (main.js) и бортового (robot.js):
 *   • метровая сетка и кольца дальности, шаг подбирается под масштаб;
 *   • оси как в RViz: X — вперёд (красный), Y — влево (зелёный);
 *   • модель шасси в метрах (колёсная база и колея из rus_core/kinematics.py);
 *   • точки: Flat Color или Range (AxisColor: Min/Max Color по Min/Max Value);
 *     размер точки — в метрах (Size, как в RViz), значит растёт вместе с зумом;
 *   • Decay Time — точки предыдущих сканов тают за указанное время;
 *   • зум: колесо мыши (к курсору), щипок на сенсорном экране, кнопки «+» «−»,
 *     перетаскивание — сдвиг, двойной щелчок — вид по умолчанию.
 *
 * Настройки отображения хранятся в localStorage этого экрана и применяются
 * сразу; параметры железа (порт, протокол) сюда не относятся — это lidar_settings.js.
 */
(function () {
  'use strict';
  if (window.RCLidarView) return;

  const STORAGE_KEY = 'rc_lidar_view_v1';
  const POS_KEY_PREFIX = 'rc_lidar_pos_v1:';  // зум и сдвиг карты (по id канвы)
  const R_DEFAULT = 10;        // м: радиус обзора по умолчанию (как прежняя дальность карты)
  const R_MIN = 0.5;           // м: максимальный зум
  const R_MAX = 40;            // м: максимальный отдалённый вид
  const PAD = 14;              // px: отступ от края канвы до сетки
  const ZOOM_STEP = 1.25;      // шаг кнопок «+» и «−»
  const WHEEL_SENS = 0.0015;   // чувствительность колеса мыши
  const GRID_MIN_PX = 24;      // шаг сетки не мельче этого на экране
  const STEPS = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20];   // «круглые» шаги, м
  const ARROW_M = 1.2;         // длина стрелок осей, м
  // Геометрия шасси (robot_control/rus_core/kinematics.py: wheelbase, track)
  const WHEELBASE_M = 0.55;
  const TRACK_M = 0.46;
  const WHEEL_LEN_M = 0.10;    // размер колеса вдоль оси, м (ориентир)
  const WHEEL_WID_M = 0.06;    // поперёк оси, м (ориентир)

  const DEFAULTS = {
    size: 0.08,           // Size (m): размер точки, м
    alpha: 1,             // Alpha
    color: 'range',       // Color Transformer: 'range' (AxisColor) | 'flat' (Flat Color)
    flatColor: '#9fe36b', // Flat Color
    minColor: '#ff5a36',  // Min Color: у препятствия
    maxColor: '#5fd3ff',  // Max Color: на Max Value
    minValue: 0,          // Min Value, м
    maxValue: 10,         // Max Value, м
    decay: 0,             // Decay Time, с (0 — только текущий скан)
    grid: true,           // Grid
    axes: true            // Axes
  };

  const HEX = /^#[0-9a-f]{6}$/i;
  const views = new Set();

  function num(value, lo, hi, fallback) {
    const n = Number(value);
    return Number.isFinite(n) ? Math.min(hi, Math.max(lo, n)) : fallback;
  }

  function sanitize(raw) {
    const s = Object.assign({}, DEFAULTS, raw && typeof raw === 'object' ? raw : {});
    const out = {
      size: num(s.size, 0.01, 0.5, DEFAULTS.size),
      alpha: num(s.alpha, 0.1, 1, DEFAULTS.alpha),
      color: s.color === 'flat' ? 'flat' : 'range',
      flatColor: HEX.test(s.flatColor) ? s.flatColor : DEFAULTS.flatColor,
      minColor: HEX.test(s.minColor) ? s.minColor : DEFAULTS.minColor,
      maxColor: HEX.test(s.maxColor) ? s.maxColor : DEFAULTS.maxColor,
      minValue: num(s.minValue, 0, 50, DEFAULTS.minValue),
      maxValue: num(s.maxValue, 0.5, 60, DEFAULTS.maxValue),
      decay: num(s.decay, 0, 30, DEFAULTS.decay),
      grid: s.grid !== false,
      axes: s.axes !== false
    };
    if (out.maxValue <= out.minValue) out.maxValue = out.minValue + 0.5;
    return out;
  }

  function load() {
    let raw = null;
    try { raw = JSON.parse(localStorage.getItem(STORAGE_KEY) || 'null'); } catch (e) { raw = null; }
    return sanitize(raw);
  }

  let settings = load();

  // Зум и сдвиг карты сохраняются, чтобы перерисовка или перезагрузка экрана
  // не возвращали вид к 10 м. Запись отложена: при перетаскивании не пишем на каждый кадр.
  function posKey(view) {
    return POS_KEY_PREFIX + (view.canvas && view.canvas.id ? view.canvas.id : 'lidar');
  }

  function loadPos(view) {
    let raw = null;
    try { raw = JSON.parse(localStorage.getItem(posKey(view)) || 'null'); } catch (e) { raw = null; }
    if (!raw || typeof raw !== 'object') return;
    view.radius = num(raw.radius, R_MIN, R_MAX, R_DEFAULT);
    view.px = Number.isFinite(Number(raw.px)) ? Number(raw.px) : 0;
    view.py = Number.isFinite(Number(raw.py)) ? Number(raw.py) : 0;
  }

  function savePosSoon(view) {
    if (view.saveTimer) clearTimeout(view.saveTimer);
    view.saveTimer = setTimeout(() => {
      view.saveTimer = null;
      try {
        localStorage.setItem(posKey(view), JSON.stringify({ radius: view.radius, px: view.px, py: view.py }));
      } catch (e) { /* режим без хранилища — вид просто не сохранится */ }
    }, 250);
  }

  function getSettings() { return Object.assign({}, settings); }

  function setSettings(patch) {
    settings = sanitize(Object.assign({}, settings, patch || {}));
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(settings)); } catch (e) { /* режим без хранилища */ }
    redrawAll();
    return getSettings();
  }

  /* ------------------------------------------------------------ вид */

  function makeView(canvas) {
    const view = {
      canvas,
      radius: R_DEFAULT,  // м: расстояние от центра до края канвы по меньшей стороне
      px: 0, py: 0,       // м: мировая точка в центре канвы (сдвиг при перетаскивании)
      pts: [],            // текущий скан
      frames: [],         // история для Decay Time: [{t, pts}]
      lastKey: '',
      pointers: new Map(),
      pinchDist: 0,
      saveTimer: null
    };
    loadPos(view);
    return view;
  }

  // Текущая дальность обзора (радиус, м) — экран показывает её рядом с картой.
  function notifyView(view) {
    const c = view.canvas;
    if (!c || typeof c.dispatchEvent !== 'function' || typeof CustomEvent !== 'function') return;
    try {
      c.dispatchEvent(new CustomEvent('rc-lidar-view', { detail: { radius: view.radius } }));
    } catch (e) { /* старые браузеры без CustomEvent — подпись просто не обновится */ }
  }

  function localPoint(view, event) {
    const rect = view.canvas.getBoundingClientRect ? view.canvas.getBoundingClientRect() : { left: 0, top: 0 };
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  // Размеры канвы и масштаб (px на метр). Берём запасные размеры, как и раньше:
  // clientWidth бывает 0 до раскладки, а на старых вьюхах нет aspect-ratio.
  function layout(view) {
    const c = view.canvas;
    const dpr = window.devicePixelRatio || 1;
    const parent = c.parentElement;
    const rect = c.getBoundingClientRect ? c.getBoundingClientRect() : null;
    const w = c.clientWidth || (rect && rect.width) || (parent && parent.clientWidth) || (c.width / dpr) || 280;
    const h = c.clientHeight || (rect && rect.height) || (parent && parent.clientHeight) || (c.height / dpr) || 280;
    if (!w || !h) return null;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) {
      c.width = Math.round(w * dpr);
      c.height = Math.round(h * dpr);
    }
    const k = Math.max(1, Math.min(w, h) / 2 - PAD) / view.radius;
    return { w, h, dpr, cx: w / 2, cy: h / 2, k };
  }

  // Экран → мир (x — вперёд, y — влево, робот в начале координат).
  function toWorld(view, L, x, y) {
    return { x: view.px + (L.cy - y) / L.k, y: view.py + (L.cx - x) / L.k };
  }

  // Зум к точке: мировая точка под курсором остаётся на месте (как в RViz).
  // factor > 1 — приблизить.
  function zoomAt(view, factor, point) {
    const L0 = layout(view);
    if (!L0 || !Number.isFinite(factor) || factor <= 0) return;
    const anchor = point || { x: L0.cx, y: L0.cy };
    const world = toWorld(view, L0, anchor.x, anchor.y);
    view.radius = Math.min(R_MAX, Math.max(R_MIN, view.radius / factor));
    const L1 = layout(view);
    if (!L1) return;
    view.px = world.x - (L1.cy - anchor.y) / L1.k;
    view.py = world.y - (L1.cx - anchor.x) / L1.k;
    redraw(view);
    savePosSoon(view);
  }

  function resetView(view) {
    view.radius = R_DEFAULT;
    view.px = 0;
    view.py = 0;
    redraw(view);
    savePosSoon(view);
  }

  function frameKey(pts, stamp) {
    if (stamp != null && Number.isFinite(Number(stamp))) return 's' + Number(stamp);
    if (!pts.length) return '';
    const mid = pts[pts.length >> 1];
    return [pts.length, pts[0].a, pts[0].r, mid.r, pts[pts.length - 1].r].join('|');
  }

  /* ------------------------------------------------------------ ввод */

  function bindInput(view) {
    const c = view.canvas;
    if (c.style) c.style.touchAction = 'none';

    c.addEventListener('wheel', (event) => {
      event.preventDefault();
      const dy = event.deltaMode === 1 ? event.deltaY * 33 : event.deltaY;
      zoomAt(view, Math.exp(-dy * WHEEL_SENS), localPoint(view, event));
    }, { passive: false });

    // Касания и нажатия не должны уходить на перелистывание экрана (swipe).
    ['touchstart', 'mousedown'].forEach((name) => {
      c.addEventListener(name, (event) => { event.stopPropagation(); });
    });

    c.addEventListener('pointerdown', (event) => {
      view.pointers.set(event.pointerId, localPoint(view, event));
      if (c.setPointerCapture) {
        try { c.setPointerCapture(event.pointerId); } catch (e) { /* нет захвата — не страшно */ }
      }
      if (view.pointers.size < 2) view.pinchDist = 0;
    });

    c.addEventListener('pointermove', (event) => {
      if (!view.pointers.has(event.pointerId)) return;
      const prev = view.pointers.get(event.pointerId);
      const cur = localPoint(view, event);
      view.pointers.set(event.pointerId, cur);
      const L = layout(view);
      if (!L) return;
      if (view.pointers.size >= 2) {
        // Щипок: расстояние между двумя пальцами меняет масштаб.
        const [a, b] = Array.from(view.pointers.values());
        const dist = Math.hypot(a.x - b.x, a.y - b.y);
        if (view.pinchDist > 0 && dist > 0) {
          zoomAt(view, dist / view.pinchDist, { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 });
        }
        view.pinchDist = dist;
        return;
      }
      // Перетаскивание: карта следует за курсором.
      view.px += (cur.y - prev.y) / L.k;
      view.py += (cur.x - prev.x) / L.k;
      redraw(view);
    });

    const release = (event) => {
      view.pointers.delete(event.pointerId);
      if (view.pointers.size < 2) view.pinchDist = 0;
      savePosSoon(view);
    };
    c.addEventListener('pointerup', release);
    c.addEventListener('pointercancel', release);
    c.addEventListener('dblclick', () => resetView(view));
  }

  // Кнопки масштаба рисуются поверх карты; в обычной разметке канвы лежат в .lidar-map.
  function mountControls(view) {
    const host = view.canvas.parentElement;
    if (!host || !host.appendChild || !document.createElement) return;
    const box = document.createElement('div');
    box.className = 'rc-lv-zoom';
    box.setAttribute('role', 'group');
    box.setAttribute('aria-label', 'Масштаб карты лидара');
    const buttons = [
      { label: '+', title: 'Приблизить (колесо мыши вверх, щипок)', act: () => zoomAt(view, ZOOM_STEP) },
      { label: '−', title: 'Отдалить (колесо мыши вниз, щипок)', act: () => zoomAt(view, 1 / ZOOM_STEP) },
      { label: '⊙', title: 'Вид по умолчанию: робот в центре, 10 м (двойной щелчок)', act: () => resetView(view) }
    ];
    buttons.forEach((spec) => {
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = spec.label;
      button.title = spec.title;
      button.setAttribute('aria-label', spec.title);
      button.addEventListener('click', (event) => {
        event.stopPropagation();
        spec.act();
      });
      box.appendChild(button);
    });
    host.appendChild(box);
  }

  function attach(canvas) {
    if (!canvas) return null;
    if (canvas.__rcLidarView) return canvas.__rcLidarView;
    const view = makeView(canvas);
    canvas.__rcLidarView = view;
    views.add(view);
    bindInput(view);
    mountControls(view);
    return view;
  }

  /* ------------------------------------------------------------ рисование */

  function hexToRgb(hex) {
    const n = parseInt(hex.slice(1), 16);
    return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
  }

  // Цвет точки по дальности (AxisColor): Min Color у Min Value → Max Color у Max Value.
  function rangeColorBuilder() {
    const a = hexToRgb(settings.minColor);
    const b = hexToRgb(settings.maxColor);
    const span = settings.maxValue - settings.minValue;
    return (r) => {
      const t = Math.min(1, Math.max(0, (r - settings.minValue) / span));
      const mix = (i) => Math.round(a[i] + (b[i] - a[i]) * t);
      return 'rgb(' + mix(0) + ',' + mix(1) + ',' + mix(2) + ')';
    };
  }

  function niceStep(k) {
    for (let i = 0; i < STEPS.length; i += 1) {
      if (STEPS[i] * k >= GRID_MIN_PX) return STEPS[i];
    }
    return STEPS[STEPS.length - 1];
  }

  function fmtM(m) {
    return (m < 1 ? m.toFixed(2) : String(Math.round(m))) + ' м';
  }

  function drawGrid(ctx, view, L, step) {
    const span = Math.max(L.w, L.h) / 2 / L.k + 1;
    const x0 = view.px - span;
    const x1 = view.px + span;
    const y0 = view.py - span;
    const y1 = view.py + span;
    const major = step < 5 ? step * 5 : step * 2;
    ctx.lineWidth = 1;
    // Линии постоянного x — горизонтальные на экране.
    for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) {
      const sy = L.cy - (x - view.px) * L.k;
      const isMajor = Math.abs(Math.round(x / major) * major - x) < step / 1000;
      ctx.strokeStyle = isMajor ? 'rgba(160,190,172,0.22)' : 'rgba(160,190,172,0.10)';
      ctx.beginPath();
      ctx.moveTo(0, sy);
      ctx.lineTo(L.w, sy);
      ctx.stroke();
    }
    // Линии постоянного y — вертикальные.
    for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) {
      const sx = L.cx - (y - view.py) * L.k;
      const isMajor = Math.abs(Math.round(y / major) * major - y) < step / 1000;
      ctx.strokeStyle = isMajor ? 'rgba(160,190,172,0.22)' : 'rgba(160,190,172,0.10)';
      ctx.beginPath();
      ctx.moveTo(sx, 0);
      ctx.lineTo(sx, L.h);
      ctx.stroke();
    }
  }

  // Кольца дальности от робота и подписи в метрах (как RViz Grid + distance rings).
  function drawRings(ctx, view, L, step) {
    const ox = L.cx + view.py * L.k;    // экран робота
    const oy = L.cy + view.px * L.k;
    const maxR = Math.hypot(L.w, L.h) / L.k + Math.hypot(view.px, view.py);
    ctx.strokeStyle = 'rgba(160,190,172,0.28)';
    ctx.fillStyle = 'rgba(150,170,160,0.9)';
    ctx.font = '11px system-ui, sans-serif';
    ctx.lineWidth = 1;
    for (let m = step; m <= maxR; m += step) {
      const rPx = m * L.k;
      ctx.beginPath();
      ctx.arc(ox, oy, rPx, 0, Math.PI * 2);
      ctx.stroke();
      const ly = oy - rPx;
      if (ly >= 0 && ly <= L.h) ctx.fillText(fmtM(m), ox + 4, ly - 3);
    }
  }

  function drawAxes(ctx, view, L) {
    const ox = L.cx + view.py * L.k;
    const oy = L.cy + view.px * L.k;
    const len = ARROW_M * L.k;
    ctx.lineWidth = 1.5;
    // X — вперёд (вверх на экране), красный.
    ctx.strokeStyle = '#e5484d';
    ctx.fillStyle = '#e5484d';
    ctx.beginPath();
    ctx.moveTo(ox, oy);
    ctx.lineTo(ox, oy - len);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(ox, oy - len - 6);
    ctx.lineTo(ox - 4, oy - len + 2);
    ctx.lineTo(ox + 4, oy - len + 2);
    ctx.closePath();
    ctx.fill();
    ctx.font = 'bold 11px system-ui, sans-serif';
    ctx.fillText('X', ox + 5, oy - len - 6);
    // Y — влево (на экране влево), зелёный.
    ctx.strokeStyle = '#46a758';
    ctx.fillStyle = '#46a758';
    ctx.beginPath();
    ctx.moveTo(ox, oy);
    ctx.lineTo(ox - len, oy);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(ox - len - 6, oy);
    ctx.lineTo(ox - len + 2, oy - 4);
    ctx.lineTo(ox - len + 2, oy + 4);
    ctx.closePath();
    ctx.fill();
    ctx.fillText('Y', ox - len - 4, oy - 7);
  }

  // Шасси в масштабе метра: корпус, четыре колеса и нос-стрелка (видна всегда).
  function drawRobot(ctx, view, L) {
    const ox = L.cx + view.py * L.k;
    const oy = L.cy + view.px * L.k;
    const hw = (TRACK_M / 2) * L.k;     // полуколея, px
    const hl = (WHEELBASE_M / 2) * L.k; // полубаза, px
    ctx.fillStyle = 'rgba(70,100,84,0.55)';
    ctx.strokeStyle = '#8fc3a2';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.rect(ox - hw, oy - hl, hw * 2, hl * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#3c4a43';
    const ww = WHEEL_WID_M * L.k;
    const wl = WHEEL_LEN_M * L.k;
    const wy = (TRACK_M / 2 + 0.02) * L.k;   // колёса чуть наружу от корпуса
    [-1, 1].forEach((front) => {
      [-1, 1].forEach((side) => {
        const cx = ox - side * wy;               // side: +1 — левый борт (Y влево)
        const cy = oy - front * hl;              // front: +1 — перед (X вперёд)
        ctx.beginPath();
        ctx.rect(cx - ww / 2, cy - wl / 2, ww, wl);
        ctx.fill();
      });
    });
    // Нос: треугольник над передним краем, всегда читается даже в масштабе 40 м.
    const nose = oy - hl - 2;
    ctx.fillStyle = '#e6efe9';
    ctx.beginPath();
    ctx.moveTo(ox, nose - 8);
    ctx.lineTo(ox - 5, nose);
    ctx.lineTo(ox + 5, nose);
    ctx.closePath();
    ctx.fill();
  }

  function drawPoints(ctx, view, L, frames, now) {
    const colorOf = settings.color === 'flat'
      ? () => settings.flatColor
      : rangeColorBuilder();
    const baseAlpha = settings.alpha;
    const size = Math.min(12, Math.max(2, settings.size * L.k));
    frames.forEach((frame) => {
      let fade = 1;
      if (settings.decay > 0) fade = Math.min(1, Math.max(0, 1 - (now - frame.t) / settings.decay));
      if (fade <= 0) return;
      ctx.globalAlpha = baseAlpha * fade;
      frame.pts.forEach((p) => {
        if (!(p.r >= 0)) return;
        const wx = p.r * Math.cos(p.a);
        const wy = p.r * Math.sin(p.a);
        const x = L.cx - (wy - view.py) * L.k;
        const y = L.cy - (wx - view.px) * L.k;
        if (x < -size || x > L.w + size || y < -size || y > L.h + size) return;
        ctx.fillStyle = colorOf(p.r);
        ctx.fillRect(x - size / 2, y - size / 2, size, size);
      });
    });
    ctx.globalAlpha = 1;
  }

  function drawInfo(ctx, view, L, step) {
    ctx.fillStyle = 'rgba(150,170,160,0.9)';
    ctx.font = '11px system-ui, sans-serif';
    const label = 'Fixed Frame: base_link · сетка ' + fmtM(step) + ' · радиус ' + fmtM(view.radius);
    ctx.fillText(label, 8, L.h - 8);
  }

  function redraw(view) {
    const canvas = view.canvas;
    const L = layout(view);
    if (!L) return;
    const ctx = canvas.getContext && canvas.getContext('2d');
    if (!ctx) return;
    const dpr = L.dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, L.w, L.h);

    const step = niceStep(L.k);
    if (settings.grid) drawGrid(ctx, view, L, step);
    drawRings(ctx, view, L, step);
    if (settings.axes) drawAxes(ctx, view, L);

    const now = Date.now() / 1000;
    const frames = settings.decay > 0
      ? view.frames
      : (view.pts.length ? [{ t: now, pts: view.pts }] : []);
    drawPoints(ctx, view, L, frames, now);
    drawRobot(ctx, view, L);
    drawInfo(ctx, view, L, step);
    if (view.lastRadius !== view.radius) {
      view.lastRadius = view.radius;
      notifyView(view);
    }
  }

  function redrawAll() {
    views.forEach((view) => redraw(view));
  }

  /* ------------------------------------------------------------ API */

  // Главный вызов экранов: новый скан на канву (stamp — scan_at из lidarStatus,
  // по нему отличаем новый скан от повтора для Decay Time).
  function render(canvas, pts, options) {
    const view = attach(canvas);
    if (!view) return;
    view.pts = Array.isArray(pts) ? pts : [];
    const now = Date.now() / 1000;
    if (settings.decay > 0) {
      const key = frameKey(view.pts, options && options.stamp);
      if (view.pts.length && key !== view.lastKey) {
        view.frames.push({ t: now, pts: view.pts });
        view.lastKey = key;
      }
      view.frames = view.frames.filter((frame) => now - frame.t <= settings.decay);
    } else if (view.frames.length) {
      view.frames = [];
      view.lastKey = '';
    }
    redraw(view);
  }

  // Сброс карты: забываем историю (Decay Time) и не рисуем старые точки.
  function clear(canvas) {
    const view = canvas && canvas.__rcLidarView;
    if (!view) return;
    view.pts = [];
    view.frames = [];
    view.lastKey = '';
  }

  function zoom(canvas, factor) {
    const view = attach(canvas);
    if (view) zoomAt(view, factor);
  }

  function reset(canvas) {
    const view = attach(canvas);
    if (view) resetView(view);
  }

  window.RCLidarView = {
    DEFAULTS: Object.assign({}, DEFAULTS),
    attach,
    render,
    clear,
    zoom,
    reset,
    redrawAll,
    getSettings,
    setSettings
  };
})();
