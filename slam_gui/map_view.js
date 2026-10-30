/* Страница «Карта лидара» и отдельное окно «Движение по маршруту».
 *
 * Габариты робота: длина 75 см (0.75 м), ширина 60 см (0.60 м), высота 30 см (0.30 м).
 * Лидар установлен спереди корпуса (+0.35 м по продольной оси +X).
 *
 * Карту строит сервер (robot_control/slam_map.py): он же впитывает сканы
 * лидара, считает позу и перезаписывает файл карты на рабочем столе.
 * Экран получает готовый кадр (PNG в base64) и рисует его как в ROS:
 *   • сетка занятости: белый — свободно, чёрный — препятствие, серый — не видели;
 *   • робот — корпус 75×60 см в масштабе карты с 4 колёсами и передним лидаром;
 *   • цель на карте ставится прямым кликом/тапом по карте или кнопкой «Цель на карте»,
 *     сразу отображается на карте и строит маршрут;
 *   • редактор нескольких точек и предпросмотр пути прямо на карте; /route — совместимый отдельный вид
 *     с плавной анимацией проезда корпуса 75×60×30 см, поворота колёс и работы
 *     переднего лидара.
 */
(function () {
  'use strict';
  if (window.RCMapView) return;

  const POLL_MS = 700;
  const ROBOT_LENGTH_M = 0.75;
  const ROBOT_WIDTH_M = 0.60;
  const ROBOT_HEIGHT_M = 0.30;
  const LIDAR_OFFSET_X_M = 0.35;
  const WHEELBASE_M = 0.50;    // расстояния между осями поворота, по умолчанию
  const TRACK_M = 0.38;
  const GOAL_STORAGE_KEY = 'rc_nav_goal_v1';

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

  const SENSITIVITY = {
    soft: { hit: 0.45, miss: 0.20 },
    normal: { hit: 0.65, miss: 0.30 },
    hard: { hit: 0.85, miss: 0.45 },
  };

  const THEME_KEY = 'kiosk_map_theme';
  const MAP_THEMES = {
    dark: {
      bg: '#08070a',
      grid: 'rgba(255, 255, 255, .07)',
      text: 'rgba(226, 214, 220, .78)',
      scan: 'rgba(244, 244, 246, .92)',
      robot: 'rgba(192, 80, 90, .92)',
      robotStroke: '#ffffff',
      free: [18, 14, 20],
      occupied: [192, 80, 90],
      unknown: [42, 36, 46],
    },
    light: {
      bg: '#ffffff',
      grid: 'rgba(20, 10, 14, .10)',
      text: 'rgba(30, 20, 24, .72)',
      scan: 'rgba(30, 20, 24, .88)',
      robot: 'rgba(192, 80, 90, .92)',
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
    robotBusy: false,
    robotTimer: null,
    navigationBusy: false,
    navigationTimer: null,
    navigationStatus: null,
    navigationPlan: null,
    routeGoals: [],
    routeTouched: false,
    navigationCsrfToken: '',
    goalPoint: null,
    lastUserMessage: '',
    lastUserTone: '',
    lastUserMessageAt: 0,
    selectingGoal: false,
    selectionPointer: null,
    lastGoalPointerUpAt: 0,
    routeWindowOpen: false,
    routeZoom: 1,
    routePanX: 0,
    routePanY: 0,
    previewActive: false,
    previewStartAt: 0,
    previewProgress: 0,
    previewRaf: 0,
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

  function fallbackSnapshot() {
    return state.snapshot || {
      resolution: DEFAULTS.resolution,
      width: 300,
      height: 300,
      origin: { x: -15.0, y: -15.0 },
      pose: { x: 0.0, y: 0.0, yaw: 0.0 },
      stats: { coverage: 0, sizeM: 30.0, width: 300, height: 300, occupied: 0 },
      scans: 0,
    };
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
    const meta = document.querySelector && document.querySelector('meta[name="theme-color"]');
    if (meta) meta.setAttribute('content', state.theme === 'light' ? '#f3f1f2' : '#000000');
    if (persist) {
      try { localStorage.setItem(THEME_KEY, state.theme); } catch (err) {}
    }
    state.rendered = null;
    state.renderedKey = '';
    updateThemeButton();
    draw();
    drawRouteWindow();
  }

  function loadTheme() {
    let theme = 'dark';
    try { theme = localStorage.getItem(THEME_KEY) === 'light' ? 'light' : 'dark'; } catch (err) {}
    applyTheme(theme, false);
  }

  /* -------------------------------------------------------------- отрисовка карты */
  function worldToCanvas(mx, my, view) {
    return {
      x: view.x0 + (mx - view.originX) * view.scale,
      y: view.y0 - (my - view.originY) * view.scale,
    };
  }

  function computeView(ctx, width, height, snap) {
    const s = snap || fallbackSnapshot();
    const res = s.resolution || DEFAULTS.resolution;
    const originX = s.origin ? s.origin.x : -15.0;
    const originY = s.origin ? s.origin.y : -15.0;
    const mapW = (s.width || 300) * res;
    const mapH = (s.height || 300) * res;
    const pose = s.pose || { x: 0, y: 0, yaw: 0 };
    let scale;
    const cx = width / 2;
    const cy = height / 2;
    if (state.follow) {
      scale = Math.min(width, height) / 12 * state.zoom;
    } else {
      scale = Math.min(width / Math.max(0.1, mapW), height / Math.max(0.1, mapH)) * 0.96 * state.zoom;
    }
    const centerX = originX + mapW / 2;
    const centerY = originY + mapH / 2;
    let x0;
    let y0;
    if (state.follow) {
      x0 = cx - (pose.x - originX) * scale;
      y0 = cy + (pose.y - originY) * scale;
    } else {
      x0 = cx - mapW * scale / 2;
      y0 = cy + mapH * scale / 2;
    }
    return { scale, x0: x0 + state.panX, y0: y0 + state.panY, originX, originY, mapW, mapH, centerX, centerY };
  }

  /* Рисуем робот 75×60 см (высота 30 см) с 4 колёсами и лидаром спереди (+35 см). */
  function drawRobotChassis(ctx, screenX, screenY, yaw, scale, pal, steerRad) {
    if (window.RCRobotModel) {
      const options = { light: state.theme === 'light' };
      if (state.previewActive) {
        // steerRad — в системе ROS: плюс означает поворот влево. Модель шасси
        // рисует углы в системе прошивки (плюс — вправо), поэтому знак
        // разворачиваем, иначе колёса на предпросмотре маршрута смотрят не туда.
        const hardwareSteerRad = -steerRad;
        options.state = { wheels: ['FL', 'FR', 'RL', 'RR'].map(id => ({ id, angle: null, radians: null, targetAngle: hardwareSteerRad * 180 / Math.PI, targetRadians: hardwareSteerRad, homed: true })), estop: false };
      }
      window.RCRobotModel.draw(ctx, screenX, screenY, yaw, scale, options);
      return;
    }
    const lenPx = Math.max(18, ROBOT_LENGTH_M * scale);
    const widPx = Math.max(14, ROBOT_WIDTH_M * scale);
    const halfL = lenPx / 2;
    const halfW = widPx / 2;
    const wheelAngle = Number.isFinite(steerRad) ? steerRad : 0;

    ctx.save();
    ctx.translate(screenX, screenY);
    ctx.rotate(-yaw);

    // Сектор обзора переднего лидара.
    const lidarX = halfL * (LIDAR_OFFSET_X_M / (ROBOT_LENGTH_M / 2));
    ctx.beginPath();
    ctx.moveTo(lidarX, 0);
    ctx.arc(lidarX, 0, Math.max(22, lenPx * 1.15), -0.48, 0.48);
    ctx.closePath();
    ctx.fillStyle = 'rgba(39, 193, 232, .12)';
    ctx.fill();

    // Четыре поворачиваемых колеса.
    const wxOff = halfL * (WHEELBASE_M / ROBOT_LENGTH_M);
    const wyOff = halfW * (TRACK_M / ROBOT_WIDTH_M);
    const wLen = Math.max(5, lenPx * 0.22);
    const wWid = Math.max(3, widPx * 0.14);
    for (const sx of [-1, 1]) {
      for (const sy of [-1, 1]) {
        ctx.save();
        ctx.translate(sx * wxOff, sy * wyOff);
        ctx.rotate(-wheelAngle);
        ctx.fillStyle = '#1b1d24';
        ctx.strokeStyle = '#96a2b0';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.rect(-wLen / 2, -wWid / 2, wLen, wWid);
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }
    }

    // Корпус 75 × 60 см со скошенным передом.
    const bevel = Math.max(3, halfW * 0.25);
    ctx.beginPath();
    ctx.moveTo(halfL, -halfW + bevel);
    ctx.lineTo(halfL, halfW - bevel);
    ctx.lineTo(halfL - bevel, halfW);
    ctx.lineTo(-halfL, halfW);
    ctx.lineTo(-halfL, -halfW);
    ctx.lineTo(halfL - bevel, -halfW);
    ctx.closePath();
    ctx.fillStyle = pal.robot;
    ctx.fill();
    ctx.strokeStyle = pal.robotStroke;
    ctx.lineWidth = 1.6;
    ctx.stroke();

    // Стрелка направления вперёд (+X).
    ctx.beginPath();
    ctx.moveTo(halfL * 0.55, 0);
    ctx.lineTo(-halfL * 0.25, -halfW * 0.36);
    ctx.lineTo(-halfL * 0.10, 0);
    ctx.lineTo(-halfL * 0.25, halfW * 0.36);
    ctx.closePath();
    ctx.fillStyle = 'rgba(255, 255, 255, .88)';
    ctx.fill();

    // Передний лидар (стоит спереди корпуса).
    const lidarRadius = Math.max(3.2, Math.min(7, widPx * 0.16));
    ctx.beginPath();
    ctx.arc(lidarX, 0, lidarRadius, 0, Math.PI * 2);
    ctx.fillStyle = '#27c1e8';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.3;
    ctx.stroke();

    ctx.restore();
  }

  function drawRobot(ctx, view, pose, pal) {
    const p = worldToCanvas(pose.x, pose.y, view);
    if (window.RCRobotModel) {
      window.RCRobotModel.draw(ctx, p.x, p.y, pose.yaw || 0, view.scale, { light: state.theme === 'light' });
    } else drawRobotChassis(ctx, p.x, p.y, pose.yaw || 0, view.scale, pal, 0);
  }

  function drawGoalPin(ctx, screen, goal, isPlanned) {
    const color = isPlanned ? '#27c1e8' : '#ffc46b';
    const fill = isPlanned ? 'rgba(39, 193, 232, .22)' : 'rgba(255, 196, 107, .24)';
    ctx.save();
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, 13, 0, Math.PI * 2);
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(screen.x, screen.y, 4, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();

    ctx.beginPath();
    ctx.moveTo(screen.x - 8, screen.y);
    ctx.lineTo(screen.x + 8, screen.y);
    ctx.moveTo(screen.x, screen.y - 8);
    ctx.lineTo(screen.x, screen.y + 8);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.6;
    ctx.stroke();

    ctx.font = 'bold 11px ui-monospace, monospace';
    ctx.textAlign = 'left';
    ctx.fillStyle = color;
    ctx.fillText(
      'ЦЕЛЬ (' + (+goal.x).toFixed(2) + '; ' + (+goal.y).toFixed(2) + ')',
      screen.x + 15,
      screen.y - 6
    );
    ctx.restore();
  }

  function drawNavigation(ctx, view) {
    const plan = state.navigationPlan;
    const points = plan && Array.isArray(plan.path) ? plan.path : [];
    state.routeGoals.forEach((point, index) => {
      const screen = worldToCanvas(point.x, point.y, view);
      ctx.save(); ctx.fillStyle = '#c0505a';
      ctx.beginPath(); ctx.arc(screen.x, screen.y, 12, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#fff'; ctx.font = 'bold 12px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(String(index + 1), screen.x, screen.y); ctx.restore();
    });
    if (state.previewActive && points.length > 1) {
      const motion = sampleRouteMotion(points, state.previewProgress, +(fallbackSnapshot().pose || {}).yaw || 0);
      const screen = worldToCanvas(motion.x, motion.y, view);
      drawRobotChassis(ctx, screen.x, screen.y, motion.yaw, view.scale, MAP_THEMES[state.theme], motion.steerRad);
      ctx.save(); ctx.fillStyle = MAP_THEMES[state.theme].text;
      ctx.font = '11px system-ui'; ctx.textAlign = 'center';
      ctx.fillText('предпросмотр', screen.x, screen.y - Math.max(24, view.scale * .5)); ctx.restore();
    }
    const activeGoal = (plan && plan.goal) || selectedGoal() || state.goalPoint;
    if (!points.length && !activeGoal) return;

    ctx.save();
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';

    if (points.length >= 2) {
      // Габаритный коридор движения робота (ширина 60 см).
      ctx.strokeStyle = 'rgba(192, 80, 90, .14)';
      ctx.lineWidth = Math.max(10, ROBOT_WIDTH_M * view.scale);
      ctx.beginPath();
      points.forEach((point, index) => {
        if (!point || !Number.isFinite(+point.x) || !Number.isFinite(+point.y)) return;
        const screen = worldToCanvas(+point.x, +point.y, view);
        if (index === 0) ctx.moveTo(screen.x, screen.y);
        else ctx.lineTo(screen.x, screen.y);
      });
      ctx.stroke();

      // Основная линия маршрута.
      ctx.strokeStyle = 'rgba(192, 80, 90, .96)';
      ctx.lineWidth = 4;
      ctx.shadowColor = 'rgba(0, 0, 0, .8)';
      ctx.shadowBlur = 5;
      ctx.beginPath();
      points.forEach((point, index) => {
        if (!point || !Number.isFinite(+point.x) || !Number.isFinite(+point.y)) return;
        const screen = worldToCanvas(+point.x, +point.y, view);
        if (index === 0) ctx.moveTo(screen.x, screen.y);
        else ctx.lineTo(screen.x, screen.y);
      });
      ctx.stroke();
      ctx.shadowBlur = 0;

      // Стрелки направления на сегментах.
      for (let i = 0; i < points.length - 1; i += 1) {
        const a = worldToCanvas(+points[i].x, +points[i].y, view);
        const b = worldToCanvas(+points[i + 1].x, +points[i + 1].y, view);
        const segPx = Math.hypot(b.x - a.x, b.y - a.y);
        if (segPx < 28) continue;
        const angle = Math.atan2(b.y - a.y, b.x - a.x);
        const mx = (a.x + b.x) / 2;
        const my = (a.y + b.y) / 2;
        ctx.save();
        ctx.translate(mx, my);
        ctx.rotate(angle);
        ctx.beginPath();
        ctx.moveTo(5, 0);
        ctx.lineTo(-4, -4);
        ctx.lineTo(-4, 4);
        ctx.closePath();
        ctx.fillStyle = '#ffffff';
        ctx.fill();
        ctx.restore();
      }

      // Промежуточные узлы маршрута.
      points.slice(1, -1).forEach((point) => {
        const screen = worldToCanvas(+point.x, +point.y, view);
        ctx.beginPath();
        ctx.arc(screen.x, screen.y, 4.5, 0, Math.PI * 2);
        ctx.fillStyle = '#27c1e8';
        ctx.fill();
        ctx.strokeStyle = '#07131a';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      });
    } else if (activeGoal && Number.isFinite(+activeGoal.x) && Number.isFinite(+activeGoal.y)) {
      // Предварительная линия от робота к выбранной цели до расчёта маршрута.
      const snap = fallbackSnapshot();
      const pose = snap.pose || { x: 0, y: 0 };
      const from = worldToCanvas(+pose.x || 0, +pose.y || 0, view);
      const to = worldToCanvas(+activeGoal.x, +activeGoal.y, view);
      if (typeof ctx.setLineDash === 'function') ctx.setLineDash([6, 5]);
      ctx.strokeStyle = 'rgba(255, 196, 107, .82)';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.stroke();
      if (typeof ctx.setLineDash === 'function') ctx.setLineDash([]);
    }

    const goal = activeGoal || (points.length ? points[points.length - 1] : null);
    if (goal && Number.isFinite(+goal.x) && Number.isFinite(+goal.y)) {
      const screen = worldToCanvas(+goal.x, +goal.y, view);
      drawGoalPin(ctx, screen, goal, points.length >= 2);
    }
    ctx.restore();
  }

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
      ? Math.max(2, Math.min(10, style.sizeM * view.scale))
      : Math.max(2, Math.min(4, view.scale * 0.05));
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
    if (!snap && !state.goalPoint && !selectedGoal()) {
      ctx.fillStyle = pal.text;
      ctx.font = '13px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('карта ещё не собрана', width / 2, height / 2);
      return;
    }
    const effectiveSnap = snap || fallbackSnapshot();
    const view = computeView(ctx, width, height, effectiveSnap);
    if (state.image && state.image.complete && state.image.naturalWidth) {
      const destW = view.mapW * view.scale;
      const destH = view.mapH * view.scale;
      const topLeft = worldToCanvas(view.originX, view.originY + view.mapH, view);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(themedMapImage(), topLeft.x, topLeft.y, destW, destH);
    }
    drawGrid(ctx, view, width, height, pal);
    drawNavigation(ctx, view);
    drawScan(ctx, view, effectiveSnap.pose || { x: 0, y: 0, yaw: 0 }, width, height, pal);
    drawRobot(ctx, view, effectiveSnap.pose || { x: 0, y: 0, yaw: 0 }, pal);
    if (window.RCRobotModel) window.RCRobotModel.drawStatus(ctx, width, height, { light: state.theme === 'light', bottom: 32 });
    ctx.fillStyle = pal.text;
    ctx.font = '11px ui-monospace, monospace';
    ctx.textAlign = 'left';
    const step = niceStep(view.scale);
    ctx.fillText(
      'map · ' + step + ' м/клетка · робот 75×60×30 см (лидар спереди) · ' +
      fmt(effectiveSnap.stats && effectiveSnap.stats.coverage * 100, 0) + '% изучено',
      10,
      height - 10
    );
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

  /* =========================================================================
   * ОТДЕЛЬНОЕ ОКНО ДВИЖЕНИЯ ПО МАРШРУТУ (#route-window / /route)
   * ========================================================================= */
  function isRouteWindowVisible() {
    const win = $('route-window');
    if (!win) return false;
    return !win.hidden || win.classList.contains('route-window--standalone');
  }

  function currentRoutePoints() {
    const plan = state.navigationPlan;
    if (plan && Array.isArray(plan.path) && plan.path.length >= 2) {
      return plan.path;
    }
    const snap = fallbackSnapshot();
    const pose = snap.pose || { x: 0, y: 0, yaw: 0 };
    const goal = selectedGoal() || state.goalPoint;
    if (goal && Number.isFinite(+goal.x) && Number.isFinite(+goal.y)) {
      return [
        { x: +pose.x || 0, y: +pose.y || 0 },
        { x: +goal.x, y: +goal.y },
      ];
    }
    return [{ x: +pose.x || 0, y: +pose.y || 0 }];
  }

  /* Состояние робота вдоль маршрута (для плавной анимации и реального хода). */
  function sampleRouteMotion(points, progress01, baseYaw) {
    if (!points || points.length === 0) {
      return { x: 0, y: 0, yaw: baseYaw || 0, steerRad: 0, segmentIndex: 0, totalM: 0 };
    }
    if (points.length === 1) {
      return { x: +points[0].x || 0, y: +points[0].y || 0, yaw: baseYaw || 0, steerRad: 0, segmentIndex: 0, totalM: 0 };
    }
    const segs = [];
    let totalM = 0;
    for (let i = 0; i < points.length - 1; i += 1) {
      const a = points[i];
      const b = points[i + 1];
      const len = Math.hypot((+b.x) - (+a.x), (+b.y) - (+a.y));
      const worldAng = len > 1e-6 ? Math.atan2((+b.y) - (+a.y), (+b.x) - (+a.x)) : (baseYaw || 0);
      let rel = ((worldAng - (baseYaw || 0) + Math.PI) % (Math.PI * 2)) - Math.PI;
      if (rel > Math.PI / 2) rel -= Math.PI;
      if (rel < -Math.PI / 2) rel += Math.PI;
      segs.push({ a, b, len, worldAng, steerRad: rel });
      totalM += len;
    }
    if (totalM <= 1e-6) {
      return { x: +points[0].x, y: +points[0].y, yaw: baseYaw || 0, steerRad: 0, segmentIndex: 0, totalM: 0 };
    }
    const targetDist = Math.max(0, Math.min(1, progress01)) * totalM;
    let acc = 0;
    for (let i = 0; i < segs.length; i += 1) {
      const seg = segs[i];
      if (acc + seg.len >= targetDist || i === segs.length - 1) {
        const localT = seg.len > 1e-6 ? Math.max(0, Math.min(1, (targetDist - acc) / seg.len)) : 1;
        // Плавный профиль скорости (ease-in-out) на каждом участке маршрута.
        const smoothT = localT * localT * (3 - 2 * localT);
        return {
          x: (+seg.a.x) + ((+seg.b.x) - (+seg.a.x)) * smoothT,
          y: (+seg.a.y) + ((+seg.b.y) - (+seg.a.y)) * smoothT,
          yaw: baseYaw || 0,
          steerRad: seg.steerRad,
          segmentIndex: i + 1,
          totalM,
        };
      }
      acc += seg.len;
    }
    const last = points[points.length - 1];
    return { x: +last.x, y: +last.y, yaw: baseYaw || 0, steerRad: 0, segmentIndex: segs.length, totalM };
  }

  /* Боковой профиль 75 × 30 см с передним лидаром в углу окна маршрута. */
  function drawSideProfileInset(ctx, width, height, nowSec) {
    const boxW = Math.min(230, Math.max(170, Math.round(width * 0.28)));
    const boxH = 88;
    const x0 = width - boxW - 14;
    const y0 = height - boxH - 26;
    if (x0 < 12 || y0 < 40) return;

    ctx.save();
    ctx.fillStyle = 'rgba(10, 15, 22, .88)';
    ctx.strokeStyle = 'rgba(39, 193, 232, .35)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.rect(x0, y0, boxW, boxH);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#9fb4c4';
    ctx.font = 'bold 10px ui-monospace, monospace';
    ctx.textAlign = 'left';
    ctx.fillText('ПРОФИЛЬ: 75×60×30 см · ЛИДАР СПЕРЕДИ', x0 + 8, y0 + 14);

    // Масштаб профиля: длина 75 см -> 108 px, высота 30 см -> 34 px.
    const bodyW = 108;
    const bodyH = 28;
    const bx = x0 + 28;
    const by = y0 + 36;
    ctx.fillStyle = 'rgba(192, 80, 90, .82)';
    ctx.strokeStyle = '#f5f2f4';
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.rect(bx, by, bodyW, bodyH);
    ctx.fill();
    ctx.stroke();

    // Колёса сбоку.
    for (const wx of [bx + 18, bx + bodyW - 18]) {
      ctx.beginPath();
      ctx.arc(wx, by + bodyH + 3, 9, 0, Math.PI * 2);
      ctx.fillStyle = '#1b1f26';
      ctx.fill();
      ctx.strokeStyle = '#8fa4b5';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // Башня лидара СПЕРЕДИ (правый передний край корпуса).
    const lx = bx + bodyW - 8;
    const ly = by - 6;
    ctx.fillStyle = '#27c1e8';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.rect(lx - 6, ly, 12, 6);
    ctx.fill();
    ctx.stroke();

    // Лучи переднего лидара.
    const sweep = Math.sin(nowSec * 4) * 0.18;
    ctx.beginPath();
    ctx.moveTo(lx + 6, ly + 3);
    ctx.lineTo(lx + 52, ly - 12 + sweep * 20);
    ctx.lineTo(lx + 52, ly + 18 + sweep * 20);
    ctx.closePath();
    ctx.fillStyle = 'rgba(39, 193, 232, .22)';
    ctx.fill();

    // Размерные подписи: 75 см и 30 см.
    ctx.fillStyle = '#d7e5f0';
    ctx.font = '10px ui-monospace, monospace';
    ctx.fillText('75 см (0.75 м)', bx + 16, by + bodyH + 18);
    ctx.fillText('30 см', bx - 24, by + 18);
    ctx.fillStyle = '#7ee4ff';
    ctx.fillText('лидар', lx - 10, ly - 3);
    ctx.restore();
  }

  function drawRouteWindow() {
    const canvas = $('route-motion-canvas');
    if (!canvas || !isRouteWindowVisible()) return;
    const ctx = canvas.getContext && canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect ? canvas.getBoundingClientRect() : { width: 720, height: 480 };
    const width = Math.max(320, Math.round(rect.width || 720));
    const height = Math.max(240, Math.round(rect.height || 480));
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
    }

    const snap = fallbackSnapshot();
    const basePose = snap.pose || { x: 0, y: 0, yaw: 0 };
    const points = currentRoutePoints();
    const status = state.navigationStatus || {};
    const phase = String(status.phase || 'idle');
    const executing = ['starting', 'executing', 'stopping'].includes(phase);
    const progress = executing && Number.isFinite(+status.progress)
      ? Math.max(0, Math.min(1, +status.progress))
      : (state.previewActive ? state.previewProgress : (phase === 'succeeded' ? 1 : 0));

    const motion = executing && status.currentPose
      ? {
          x: +status.currentPose.x || basePose.x || 0,
          y: +status.currentPose.y || basePose.y || 0,
          yaw: +status.currentPose.yaw || basePose.yaw || 0,
          steerRad: 0,
          segmentIndex: Number(status.waypoint || 0) + 1,
          totalM: (state.navigationPlan && +state.navigationPlan.distanceM) || 0,
        }
      : sampleRouteMotion(points, progress, +basePose.yaw || 0);

    // Автоматическое кадрирование вокруг робота и всего маршрута.
    let minX = (+basePose.x || 0) - 1.2;
    let maxX = (+basePose.x || 0) + 1.2;
    let minY = (+basePose.y || 0) - 1.2;
    let maxY = (+basePose.y || 0) + 1.2;
    points.forEach((pt) => {
      if (!pt || !Number.isFinite(+pt.x) || !Number.isFinite(+pt.y)) return;
      minX = Math.min(minX, +pt.x - 0.9);
      maxX = Math.max(maxX, +pt.x + 0.9);
      minY = Math.min(minY, +pt.y - 0.9);
      maxY = Math.max(maxY, +pt.y + 0.9);
    });
    const spanX = Math.max(2.4, maxX - minX);
    const spanY = Math.max(2.0, maxY - minY);
    const scale = Math.min((width - 64) / spanX, (height - 64) / spanY) * state.routeZoom;
    const midX = (minX + maxX) / 2;
    const midY = (minY + maxY) / 2;
    const view = {
      scale,
      originX: midX,
      originY: midY,
      x0: width / 2 + state.routePanX,
      y0: height / 2 + state.routePanY,
    };

    const toScreen = (wx, wy) => ({
      x: view.x0 + (wx - view.originX) * view.scale,
      y: view.y0 - (wy - view.originY) * view.scale,
    });

    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#070b10';
    ctx.fillRect(0, 0, width, height);

    // Метровая сетка.
    const step = niceStep(scale);
    const wMinX = view.originX - width / scale;
    const wMaxX = view.originX + width / scale;
    const wMinY = view.originY - height / scale;
    const wMaxY = view.originY + height / scale;
    ctx.strokeStyle = 'rgba(130, 175, 210, .10)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(wMinX / step) * step; x <= wMaxX; x += step) {
      const sx = toScreen(x, 0).x;
      ctx.moveTo(sx, 0);
      ctx.lineTo(sx, height);
    }
    for (let y = Math.ceil(wMinY / step) * step; y <= wMaxY; y += step) {
      const sy = toScreen(0, y).y;
      ctx.moveTo(0, sy);
      ctx.lineTo(width, sy);
    }
    ctx.stroke();

    // Оси координат от стартовой позы.
    const originScreen = toScreen(+basePose.x || 0, +basePose.y || 0);
    ctx.strokeStyle = 'rgba(229, 72, 77, .45)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(originScreen.x, originScreen.y);
    ctx.lineTo(originScreen.x + 0.8 * scale, originScreen.y);
    ctx.stroke();
    ctx.strokeStyle = 'rgba(70, 167, 88, .45)';
    ctx.beginPath();
    ctx.moveTo(originScreen.x, originScreen.y);
    ctx.lineTo(originScreen.x, originScreen.y - 0.8 * scale);
    ctx.stroke();

    const nowSec = Date.now() / 1000;

    // Коридор габарита робота (60 см ширина корпуса) и траектория маршрута.
    if (points.length >= 2) {
      ctx.save();
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';

      // Внешний защитный коридор (footprint).
      ctx.strokeStyle = 'rgba(39, 193, 232, .10)';
      ctx.lineWidth = Math.max(16, ROBOT_LENGTH_M * scale);
      ctx.beginPath();
      points.forEach((pt, idx) => {
        const s = toScreen(+pt.x, +pt.y);
        if (idx === 0) ctx.moveTo(s.x, s.y);
        else ctx.lineTo(s.x, s.y);
      });
      ctx.stroke();

      // Коридор ширины шасси 60 см.
      ctx.strokeStyle = 'rgba(83, 226, 161, .16)';
      ctx.lineWidth = Math.max(10, ROBOT_WIDTH_M * scale);
      ctx.beginPath();
      points.forEach((pt, idx) => {
        const s = toScreen(+pt.x, +pt.y);
        if (idx === 0) ctx.moveTo(s.x, s.y);
        else ctx.lineTo(s.x, s.y);
      });
      ctx.stroke();

      // Линия маршрута.
      ctx.strokeStyle = '#27c1e8';
      ctx.lineWidth = 4;
      ctx.beginPath();
      points.forEach((pt, idx) => {
        const s = toScreen(+pt.x, +pt.y);
        if (idx === 0) ctx.moveTo(s.x, s.y);
        else ctx.lineTo(s.x, s.y);
      });
      ctx.stroke();

      // Бегущие шевроны направления вдоль сегментов маршрута.
      const flowShift = (nowSec * 28) % 28;
      for (let i = 0; i < points.length - 1; i += 1) {
        const a = toScreen(+points[i].x, +points[i].y);
        const b = toScreen(+points[i + 1].x, +points[i + 1].y);
        const segLenPx = Math.hypot(b.x - a.x, b.y - a.y);
        if (segLenPx < 20) continue;
        const ang = Math.atan2(b.y - a.y, b.x - a.x);
        const ux = Math.cos(ang);
        const uy = Math.sin(ang);
        for (let d = flowShift; d < segLenPx - 10; d += 28) {
          const cx = a.x + ux * d;
          const cy = a.y + uy * d;
          ctx.save();
          ctx.translate(cx, cy);
          ctx.rotate(ang);
          ctx.beginPath();
          ctx.moveTo(-4, -4);
          ctx.lineTo(3, 0);
          ctx.lineTo(-4, 4);
          ctx.strokeStyle = 'rgba(255, 255, 255, .85)';
          ctx.lineWidth = 1.8;
          ctx.stroke();
          ctx.restore();
        }
        // Подпись длины и угла сегмента.
        const segM = Math.hypot((+points[i + 1].x) - (+points[i].x), (+points[i + 1].y) - (+points[i].y));
        const mx = (a.x + b.x) / 2;
        const my = (a.y + b.y) / 2;
        ctx.fillStyle = 'rgba(215, 236, 248, .86)';
        ctx.font = '11px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.fillText(segM.toFixed(2) + ' м', mx, my - 10);
      }

      // Точки маршрута (СТАРТ, Т1..Тn, ЦЕЛЬ).
      points.forEach((pt, idx) => {
        const s = toScreen(+pt.x, +pt.y);
        const isFirst = idx === 0;
        const isLast = idx === points.length - 1;
        ctx.beginPath();
        ctx.arc(s.x, s.y, isLast ? 9 : 6, 0, Math.PI * 2);
        ctx.fillStyle = isLast ? '#53e2a1' : (isFirst ? '#ffc46b' : '#27c1e8');
        ctx.fill();
        ctx.strokeStyle = '#071018';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.fillStyle = '#e8f3fa';
        ctx.font = 'bold 11px ui-monospace, monospace';
        ctx.textAlign = 'left';
        const tag = isFirst ? 'СТАРТ' : (isLast ? 'ЦЕЛЬ (' + (+pt.x).toFixed(2) + '; ' + (+pt.y).toFixed(2) + ')' : 'Т' + idx);
        ctx.fillText(tag, s.x + 12, s.y + 4);
      });
      ctx.restore();
    }

    // Точки живого скана лидара (от переднего лидара робота).
    if (state.scan.length) {
      const cos = Math.cos(motion.yaw);
      const sin = Math.sin(motion.yaw);
      ctx.fillStyle = 'rgba(255, 110, 118, .85)';
      state.scan.forEach((pt) => {
        if (!Number.isFinite(pt.a) || !Number.isFinite(pt.r) || pt.r <= 0) return;
        const lx = pt.r * Math.cos(pt.a);
        const ly = pt.r * Math.sin(pt.a);
        const wx = motion.x + lx * cos - ly * sin;
        const wy = motion.y + lx * sin + ly * cos;
        const sp = toScreen(wx, wy);
        ctx.fillRect(sp.x - 2, sp.y - 2, 4, 4);
      });
    }

    // Сам робот 75×60×30 см с передним лидаром на траектории.
    const robotScreen = toScreen(motion.x, motion.y);
    drawRobotChassis(
      ctx,
      robotScreen.x,
      robotScreen.y,
      motion.yaw,
      scale,
      { robot: 'rgba(192, 80, 90, .92)', robotStroke: '#ffffff' },
      motion.steerRad
    );

    // Врезка: вид сбоку 75×30 см и лидар спереди.
    drawSideProfileInset(ctx, width, height, nowSec);

    // Обновляем HUD и KPI окна маршрута.
    const steerDeg = Math.round((motion.steerRad || 0) * 180 / Math.PI);
    setText($('route-hud-steer'), 'Колёса: ' + (steerDeg > 0 ? '+' : '') + steerDeg + '°');
    setText($('route-kpi-pose'), fmt(motion.x, 2) + ' / ' + fmt(motion.y, 2) + ' м · ' + fmt(motion.yaw * 180 / Math.PI, 0) + '°');
    const pct = Math.round(progress * 100);
    setText($('route-kpi-pct'), pct + '%');
    const bar = $('route-win-progress-fill');
    if (bar) bar.style.width = pct + '%';
    const goal = (state.navigationPlan && state.navigationPlan.goal) || selectedGoal() || state.goalPoint;
    setText($('route-kpi-goal'), goal ? fmt(goal.x, 2) + ' / ' + fmt(goal.y, 2) + ' м' : '—');
    const distM = (state.navigationPlan && state.navigationPlan.distanceM) || motion.totalM;
    setText($('route-kpi-dist'), distM ? fmt(distM, 2) + ' м' : '—');
    const totalWp = Math.max(0, points.length - 1);
    setText($('route-kpi-wp'), (totalWp ? Math.min(totalWp, motion.segmentIndex || 1) : 0) + ' / ' + totalWp);
  }

  function startRoutePreview() {
    if (routeLocked()) return;
    if (state.previewActive) {
      state.previewActive = false;
      if (state.previewRaf) cancelAnimationFrame(state.previewRaf);
      navigationMessage('Предпросмотр остановлен. Привод не включался.', '', true);
      draw(); return;
    }
    const pts = state.navigationPlan && state.navigationPlan.path || [];
    if (pts.length < 2 || !navigationGoalMatches(state.navigationPlan)) {
      navigationMessage('Сначала постройте маршрут через выбранные точки.', 'warn', true);
      return;
    }
    state.previewActive = true;
    state.previewStartAt = Date.now();
    state.previewProgress = 0;
    const btn = $('route-win-preview');
    if (btn) btn.textContent = '⟳ Повторить предпросмотр';
    navigationMessage('Предпросмотр пути · привод выключен.', 'ok', true);
    const durationMs = Math.max(2600, Math.min(10000, pts.length * 1800));
    function stepFrame() {
      if (!state.previewActive) return;
      const elapsed = Date.now() - state.previewStartAt;
      state.previewProgress = Math.min(1, elapsed / durationMs);
      drawRouteWindow();
      draw();
      if (state.previewProgress < 1) {
        state.previewRaf = requestAnimationFrame(stepFrame);
      } else {
        state.previewActive = false;
        navigationMessage('Предпросмотр завершён. Это визуализация, а не гарантия безопасного проезда. Перед стартом проверьте зону.', 'ok', true);
        draw();
      }
    }
    if (state.previewRaf) cancelAnimationFrame(state.previewRaf);
    state.previewRaf = requestAnimationFrame(stepFrame);
    drawRouteWindow();
  }

  function openRouteWindow() {
    if ($('nav-point-list')) {
      const panel = $('nav-point-list').closest('.nav-panel');
      if (panel) panel.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
      return;
    }
    const win = $('route-window');
    if (!win) return;
    win.hidden = false;
    state.routeWindowOpen = true;
    syncGoalInputs();
    drawRouteWindow();
    if (currentRoutePoints().length >= 2 && !state.previewActive) {
      startRoutePreview();
    }
  }

  function closeRouteWindow() {
    const win = $('route-window');
    if (!win || win.classList.contains('route-window--standalone')) return;
    win.hidden = true;
    state.routeWindowOpen = false;
    state.previewActive = false;
    if (state.previewRaf) cancelAnimationFrame(state.previewRaf);
  }

  function routeWindowMessage(text, tone) {
    const el = $('route-win-msg');
    if (!el) return;
    el.textContent = text || '';
    el.dataset.tone = tone || '';
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
    const page = $('swipe-map-view');
    if (page && page.hidden && !isRouteWindowVisible()) return;
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
      drawRouteWindow();
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

  // Standalone map has no main/robot poller. Reuse their read-only telemetry API.
  async function fetchRobotState() {
    if (!window.RCRobotModel || state.robotBusy) return;
    state.robotBusy = true;
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 1800);
    try {
      const response = await fetch('api/state', { cache: 'no-store', signal: controller.signal });
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const body = await response.json();
      if (body && body.ok && body.data) window.RCRobotModel.update(body.data);
    } catch (err) {
      // Do not reset angles to zero or refresh the age on failed requests.
    } finally {
      clearTimeout(timeout); state.robotBusy = false;
      draw(); drawRouteWindow();
    }
  }

  async function fetchScan() {
    if (state.scanBusy) return;
    const page = $('swipe-map-view');
    if (page && page.hidden && !isRouteWindowVisible()) return;
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
      drawRouteWindow();
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

  function navigationMessage(text, tone, isUserAction) {
    if (isUserAction) {
      state.lastUserMessage = text || '';
      state.lastUserTone = tone || '';
      state.lastUserMessageAt = Date.now();
    }
    const el = $('nav-msg');
    if (el) {
      el.textContent = text || '';
      el.dataset.tone = tone || '';
    }
    routeWindowMessage(text, tone);
  }

  function navigationPhaseLabel(phase) {
    return ({
      idle: 'ожидание', planned: 'маршрут готов', starting: 'запуск',
      executing: 'едет', stopping: 'остановка', succeeded: 'достигнуто',
      cancelled: 'отменён', blocked: 'заблокирован', error: 'ошибка',
    })[phase] || phase || 'нет связи';
  }

  function setGoalInputs(x, y) {
    const xStr = Number(x).toFixed(2);
    const yStr = Number(y).toFixed(2);
    for (const id of ['nav-goal-x', 'route-win-goal-x']) {
      const el = $(id);
      if (el && el.value !== xStr) el.value = xStr;
    }
    for (const id of ['nav-goal-y', 'route-win-goal-y']) {
      const el = $(id);
      if (el && el.value !== yStr) el.value = yStr;
    }
    state.goalPoint = { x: Number(xStr), y: Number(yStr) };
    try {
      localStorage.setItem(GOAL_STORAGE_KEY, JSON.stringify(state.goalPoint));
    } catch (err) {}
  }

  function syncGoalInputs() {
    const goal = selectedGoal() || state.goalPoint;
    if (goal) setGoalInputs(goal.x, goal.y);
  }

  function selectedGoal() {
    for (const [xId, yId] of [['nav-goal-x', 'nav-goal-y'], ['route-win-goal-x', 'route-win-goal-y']]) {
      const xField = $(xId);
      const yField = $(yId);
      if (xField && yField && String(xField.value).trim() && String(yField.value).trim()) {
        const x = Number(xField.value);
        const y = Number(yField.value);
        if (Number.isFinite(x) && Number.isFinite(y)) return { x, y };
      }
    }
    return state.goalPoint;
  }

  function routeLocked() {
    return state.navigationBusy || !!(state.navigationStatus && ['starting', 'executing', 'stopping'].includes(state.navigationStatus.phase));
  }

  function routeEdited() {
    state.routeTouched = true;
    state.navigationPlan = null;
    state.previewActive = false;
    if (state.previewRaf) cancelAnimationFrame(state.previewRaf);
    const list = $('nav-point-list');
    if (list) {
      list.replaceChildren();
      state.routeGoals.forEach((point, index) => {
        const row = document.createElement('li');
        const label = document.createElement('span');
        label.textContent = (index + 1) + '.  ' + point.x.toFixed(2) + ' / ' + point.y.toFixed(2) + ' м';
        row.appendChild(label);
        [['↑', 'Выше', -1], ['↓', 'Ниже', 1], ['×', 'Удалить', 0]].forEach(([text, title, delta]) => {
          const button = document.createElement('button');
          button.type = 'button'; button.className = 'map-btn'; button.textContent = text;
          button.setAttribute('aria-label', title + ' точку ' + (index + 1));
          button.disabled = delta !== 0 && (index + delta < 0 || index + delta >= state.routeGoals.length);
          button.addEventListener('click', () => {
            if (routeLocked()) return;
            if (delta === 0) state.routeGoals.splice(index, 1);
            else [state.routeGoals[index], state.routeGoals[index + delta]] = [state.routeGoals[index + delta], state.routeGoals[index]];
            routeEdited();
          });
          row.appendChild(button);
        });
        list.appendChild(row);
      });
    }
    const last = state.routeGoals[state.routeGoals.length - 1];
    if (last) setGoalInputs(last.x, last.y);
    else {
      state.goalPoint = null;
      for (const id of ['nav-goal-x', 'nav-goal-y', 'route-win-goal-x', 'route-win-goal-y']) if ($(id)) $(id).value = '';
      try { localStorage.removeItem(GOAL_STORAGE_KEY); } catch (err) {}
    }
    for (const id of ['nav-start', 'route-win-start']) if ($(id)) $(id).disabled = true;
    setText($('nav-route-summary'), 'Точек: ' + state.routeGoals.length + ' · постройте маршрут');
    navigationMessage('Точки изменены. Постройте маршрут для проверки пути.', 'warn', true);
    draw();
  }

  function addRoutePoint(point) {
    if (routeLocked()) return;
    if (state.routeGoals.length >= 20) {
      navigationMessage('В маршруте может быть не больше 20 точек.', 'warn', true);
      return;
    }
    state.selectingGoal = false;
    document.body.classList.remove('nav-selecting');
    if ($('nav-select-goal')) {
      $('nav-select-goal').classList.remove('is-on');
      $('nav-select-goal').setAttribute('aria-pressed', 'false');
    }
    state.routeGoals.push({ x: +point.x.toFixed(2), y: +point.y.toFixed(2) });
    routeEdited();
  }

  function routePayload(goal) {
    return { goal: state.routeGoals.length ? state.routeGoals[state.routeGoals.length - 1] : goal,
      goals: state.routeGoals.length ? state.routeGoals.map(p => ({ ...p })) : [goal] };
  }

  function navigationGoalMatches(plan) {
    const selected = state.routeGoals.length ? state.routeGoals[state.routeGoals.length - 1] : selectedGoal();
    if (plan && plan.goalsRequested && JSON.stringify(plan.goalsRequested) !== JSON.stringify(routePayload(selected).goals)) return false;
    const requested = plan && (plan.goalRequested || plan.goal);
    return !!(selected && requested && Number.isFinite(+requested.x) && Number.isFinite(+requested.y)
      && Math.abs(selected.x - +requested.x) <= 0.01
      && Math.abs(selected.y - +requested.y) <= 0.01);
  }

  function renderNavigationStatus(status) {
    if (!status || typeof status !== 'object') return;
    state.navigationStatus = status;
    if (!state.routeTouched && status.goal && Number.isFinite(+status.goal.x) && Number.isFinite(+status.goal.y) && !selectedGoal()) {
      setGoalInputs(+status.goal.x, +status.goal.y);
    }
    if (Object.prototype.hasOwnProperty.call(status, 'plan')) {
      if (status.plan || !state.navigationPlan) {
        state.navigationPlan = status.plan && (navigationGoalMatches(status.plan) || routeLocked()) ? status.plan : null;
      }
      draw();
      drawRouteWindow();
    }
    if (status.csrfToken) state.navigationCsrfToken = status.csrfToken;
    const phase = String(status.phase || 'idle');
    for (const id of ['nav-status', 'route-win-status']) {
      const badge = $(id);
      if (badge) {
        badge.textContent = navigationPhaseLabel(phase);
        badge.dataset.phase = phase;
      }
    }
    const plan = state.navigationPlan;
    const active = ['starting', 'executing', 'stopping'].includes(phase);
    for (const id of ['nav-add-point', 'nav-clear-points', 'nav-preview']) {
      if ($(id)) $(id).disabled = active || state.navigationBusy;
    }
    const canStartNow = !!(status.canStart && plan && navigationGoalMatches(plan) && !state.navigationBusy);
    for (const id of ['nav-start', 'route-win-start']) {
      const start = $(id);
      if (start) start.disabled = !canStartNow;
    }
    for (const id of ['nav-cancel', 'route-win-cancel']) {
      const cancel = $(id);
      if (cancel) cancel.disabled = !active || state.navigationBusy;
    }
    for (const id of ['nav-plan', 'route-win-plan']) {
      const planButton = $(id);
      if (planButton) planButton.disabled = active || state.navigationBusy;
    }
    for (const id of ['nav-estop', 'route-win-estop']) {
      const estop = $(id);
      if (estop) estop.disabled = state.navigationBusy;
    }
    const summary = $('nav-route-summary');
    if (summary) {
      if (!plan || !Array.isArray(plan.path)) {
        const goal = selectedGoal();
        summary.textContent = goal
          ? 'цель (' + fmt(goal.x, 2) + '; ' + fmt(goal.y, 2) + ') м · нажмите «Построить маршрут»'
          : 'траектория не рассчитана';
      } else {
        const pieces = [fmt(plan.distanceM, 2) + ' м', String(plan.waypoints || 0) + ' участ.'];
        if (active && Number.isFinite(+status.progress)) pieces.push(Math.round(+status.progress * 100) + '%');
        summary.textContent = pieces.join(' · ') + ' · габарит ' + fmt(plan.footprintM, 2) + ' м (75×60×30 см)';
      }
    }
    const safety = $('nav-safety-note');
    if (safety) {
      safety.textContent = 'Габариты 75×60×30 см, лидар спереди. Поворота на месте нет: допустимы только прямые/крабовые отрезки ±28°. Неизвестные клетки запрещены. ' +
        (status.enabled ? 'Автотяга разрешена конфигурацией, но требует актуальной готовности и подтверждения.'
          : 'Автотяга выключена по умолчанию.');
    }
    if (state.selectingGoal) return;
    if (phase === 'idle' && state.lastUserMessage && Date.now() - state.lastUserMessageAt < 15000) {
      navigationMessage(state.lastUserMessage, state.lastUserTone, false);
      return;
    }
    let text = status.message || 'Состояние навигации обновлено.';
    const tone = ['blocked', 'error'].includes(phase) ? 'err'
      : (phase === 'planned' || status.reason ? 'warn' : (phase === 'succeeded' ? 'ok' : ''));
    if (status.reason && !status.canStart && !active) text += ' ' + status.reason;
    navigationMessage(text, tone, false);
  }

  async function refreshNavigation() {
    if (state.navigationBusy) return;
    const page = $('swipe-map-view');
    if (page && page.hidden && !isRouteWindowVisible()) return;
    try {
      const response = await fetch('api/navigation/status', { cache: 'no-store' });
      const body = await response.json().catch(() => null);
      if (!response.ok || !body || !body.ok) {
        throw new Error((body && body.error) || ('HTTP ' + response.status));
      }
      renderNavigationStatus(body);
    } catch (err) {
      for (const id of ['nav-status', 'route-win-status']) {
        const badge = $(id);
        if (badge) { badge.textContent = 'нет связи'; badge.dataset.phase = 'error'; }
      }
      for (const id of ['nav-start', 'route-win-start']) {
        const start = $(id);
        if (start) start.disabled = true;
      }
      navigationMessage('Не удалось получить состояние навигации: ' + (err.message || err), 'err', false);
    }
  }

  async function postNavigation(url, payload) {
    const headers = { 'Content-Type': 'application/json', 'X-Robot-Control': 'navigation' };
    if (state.navigationCsrfToken) headers['X-CSRF-Token'] = state.navigationCsrfToken;
    const response = await fetch(url, {
      method: 'POST', cache: 'no-store', headers,
      body: JSON.stringify(payload || {}),
    });
    const body = await response.json().catch(() => null);
    if (!response.ok || !body || !body.ok) {
      const error = new Error((body && body.error) || ('HTTP ' + response.status));
      error.payload = body;
      error.status = response.status;
      throw error;
    }
    return body;
  }

  function setGoalFromCanvas(event) {
    const canvas = $('map-canvas');
    if (!canvas || routeLocked()) return;
    const snap = fallbackSnapshot();
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(320, Math.round(rect.width || 640));
    const height = Math.max(240, Math.round(rect.height || 640));
    const view = computeView(null, width, height, snap);
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    const wx = view.originX + (x - view.x0) / view.scale;
    const wy = view.originY + (view.y0 - y) / view.scale;
    if ($('nav-point-list')) { addRoutePoint({ x: wx, y: wy }); return; }
    setGoalInputs(wx, wy);
    if (state.navigationPlan && !navigationGoalMatches(state.navigationPlan)) {
      state.navigationPlan = null;
    }
    state.selectingGoal = false;
    state.selectionPointer = null;
    document.body.classList.remove('nav-selecting');
    const pick = $('nav-select-goal');
    if (pick) { pick.classList.remove('is-on'); pick.setAttribute('aria-pressed', 'false'); }
    navigationMessage(
      'Цель отмечена: X ' + wx.toFixed(2) + ' м, Y ' + wy.toFixed(2) + ' м. Постройте маршрут.',
      'ok',
      true
    );
    for (const id of ['nav-start', 'route-win-start']) {
      const start = $(id);
      if (start) start.disabled = true;
    }
    if (state.navigationStatus) renderNavigationStatus(state.navigationStatus);
    draw();
    drawRouteWindow();
    if (!state.navigationBusy) {
      void planNavigation({ autoFromCanvas: true });
    }
  }

  async function planNavigation(options) {
    if (routeLocked()) return;
    const goal = state.routeGoals.length ? state.routeGoals[state.routeGoals.length - 1] : selectedGoal();
    if (!goal) {
      navigationMessage('Укажите координаты X и Y или нажмите на карту.', 'warn', true);
      return;
    }
    setGoalInputs(goal.x, goal.y);
    state.navigationBusy = true;
    state.navigationPlan = null;
    draw();
    drawRouteWindow();
    const buttons = [$('nav-plan'), $('route-win-plan')].filter(Boolean);
    let refreshAfter = false;
    buttons.forEach((button) => { button.disabled = true; button.textContent = 'Считаю маршрут…'; });
    try {
      const body = await post('api/navigation/plan', routePayload(goal));
      state.navigationPlan = body.plan || null;
      renderNavigationStatus(body.status || {});
      state.navigationPlan = body.plan || state.navigationPlan;
      draw();
      drawRouteWindow();
      navigationMessage('Маршрут рассчитан. Проверьте путь и порядок точек на карте перед запуском.', 'ok', true);
      if (options && options.openWindow) openRouteWindow();
      else if (isRouteWindowVisible()) startRoutePreview();
    } catch (err) {
      state.navigationPlan = null;
      state.navigationStatus = null;
      draw();
      drawRouteWindow();
      navigationMessage('Не удалось построить маршрут: ' + (err.message || err), 'err', true);
      refreshAfter = true;
    } finally {
      state.navigationBusy = false;
      buttons.forEach((button) => { button.disabled = false; button.textContent = 'Построить маршрут'; });
      if (state.navigationStatus) renderNavigationStatus(state.navigationStatus);
    }
    if (refreshAfter) await refreshNavigation();
  }

  async function startNavigation() {
    const goal = state.routeGoals.length ? state.routeGoals[state.routeGoals.length - 1] : selectedGoal();
    if (!goal || !navigationGoalMatches(state.navigationPlan)) {
      navigationMessage('Сначала постройте маршрут до выбранной цели.', 'warn', true);
      return;
    }
    const summary = state.navigationPlan
      ? fmt(state.navigationPlan.distanceM, 2) + ' м, ' + (state.navigationPlan.waypoints || 0) + ' участков'
      : 'маршрут';
    if (!window.confirm('Запустить автономное движение по проверенному маршруту (' + summary + ')? Убедитесь, что зона свободна и E-STOP доступен.')) return;
    state.previewActive = false;
    if (state.previewRaf) cancelAnimationFrame(state.previewRaf);
    draw();
    state.navigationBusy = true;
    for (const id of ['nav-start', 'route-win-start']) {
      const button = $(id);
      if (button) button.disabled = true;
    }
    let refreshAfter = false;
    try {
      const body = await postNavigation('api/navigation/start', routePayload(goal));
      renderNavigationStatus(body.status || {});
      navigationMessage((body.status && body.status.message) || 'Маршрут запущен.', 'warn', true);
      openRouteWindow();
    } catch (err) {
      const payload = err && err.payload;
      if (payload && payload.code === 'plan_changed' && payload.plan) {
        state.navigationPlan = payload.plan;
        if (payload.status) renderNavigationStatus(payload.status);
        else state.navigationStatus = null;
        draw();
        drawRouteWindow();
        navigationMessage(err.message || 'Карта изменилась: проверьте обновлённый маршрут.', 'warn', true);
        refreshAfter = !payload.status;
      } else {
        state.navigationStatus = null;
        navigationMessage('Запуск отклонён: ' + (err.message || err), 'err', true);
        refreshAfter = true;
      }
    } finally {
      state.navigationBusy = false;
      if (state.navigationStatus) renderNavigationStatus(state.navigationStatus);
    }
    if (refreshAfter) await refreshNavigation();
  }

  async function cancelNavigation() {
    state.navigationBusy = true;
    for (const id of ['nav-cancel', 'route-win-cancel']) {
      const button = $(id);
      if (button) button.disabled = true;
    }
    try {
      const body = await postNavigation('api/navigation/cancel', {});
      const status = body.status || body;
      renderNavigationStatus(status);
      navigationMessage(status.message || 'Команда остановки отправлена.', 'warn', true);
    } catch (err) {
      navigationMessage('Не удалось передать остановку: ' + (err.message || err), 'err', true);
    } finally {
      state.navigationBusy = false;
      await refreshNavigation();
    }
  }

  async function assertNavigationEstop() {
    state.navigationBusy = true;
    for (const id of ['nav-estop', 'route-win-estop']) {
      const button = $(id);
      if (button) button.disabled = true;
    }
    try {
      const body = await postNavigation('api/navigation/estop', {});
      if (body.status) renderNavigationStatus(body.status);
      navigationMessage(body.driveStopped
        ? 'E-STOP взведён на борту; тяга остановлена.'
        : 'E-STOP защёлкнут, но борт не подтвердил останов тяги — используйте физический выключатель.', 'err', true);
    } catch (err) {
      navigationMessage('E-STOP не подтверждён бортом: ' + (err.message || err) + '. Используйте физический выключатель.', 'err', true);
    } finally {
      state.navigationBusy = false;
      for (const id of ['nav-estop', 'route-win-estop']) {
        const button = $(id);
        if (button) button.disabled = false;
      }
      await refreshNavigation();
    }
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
      state.navigationPlan = null;
      draw();
      drawRouteWindow();
      void refreshNavigation();
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
      state.navigationPlan = null;
      draw();
      drawRouteWindow();
      void refreshNavigation();
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

    try {
      const savedGoal = JSON.parse(localStorage.getItem(GOAL_STORAGE_KEY) || 'null');
      if (savedGoal && Number.isFinite(+savedGoal.x) && Number.isFinite(+savedGoal.y)) {
        setGoalInputs(+savedGoal.x, +savedGoal.y);
      }
    } catch (err) {}

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

    for (const id of ['nav-plan', 'route-win-plan']) {
      const planButton = $(id);
      if (planButton) planButton.addEventListener('click', () => { void planNavigation(); });
    }
    for (const id of ['nav-start', 'route-win-start']) {
      const startButton = $(id);
      if (startButton) startButton.addEventListener('click', () => { void startNavigation(); });
    }
    for (const id of ['nav-cancel', 'route-win-cancel']) {
      const cancelButton = $(id);
      if (cancelButton) cancelButton.addEventListener('click', () => { void cancelNavigation(); });
    }
    for (const id of ['nav-estop', 'route-win-estop']) {
      const estopButton = $(id);
      if (estopButton) estopButton.addEventListener('click', () => { void assertNavigationEstop(); });
    }
    for (const id of ['nav-open-route-window', 'map-open-route-win']) {
      const openWinBtn = $(id);
      if (openWinBtn) openWinBtn.addEventListener('click', () => openRouteWindow());
    }
    const closeWinBtn = $('route-win-close');
    if (closeWinBtn) closeWinBtn.addEventListener('click', () => closeRouteWindow());
    const winBackdrop = $('route-win-backdrop');
    if (winBackdrop) winBackdrop.addEventListener('click', () => closeRouteWindow());
    if ($('nav-add-point')) $('nav-add-point').addEventListener('click', () => {
      const x = $('nav-goal-x'), y = $('nav-goal-y');
      if (!x.value.trim() || !y.value.trim() || !Number.isFinite(+x.value) || !Number.isFinite(+y.value)) {
        navigationMessage('Введите обе координаты X и Y.', 'warn', true); return;
      }
      addRoutePoint({ x: +x.value, y: +y.value });
    });
    if ($('nav-clear-points')) $('nav-clear-points').addEventListener('click', () => {
      if (routeLocked()) return;
      state.routeGoals = []; state.goalPoint = null;
      for (const id of ['nav-goal-x', 'nav-goal-y', 'route-win-goal-x', 'route-win-goal-y']) if ($(id)) $(id).value = '';
      routeEdited();
    });
    if ($('nav-preview')) $('nav-preview').addEventListener('click', startRoutePreview);
    const previewBtn = $('route-win-preview');
    if (previewBtn) previewBtn.addEventListener('click', () => startRoutePreview());

    const pickButton = $('nav-select-goal');
    if (pickButton) pickButton.addEventListener('click', () => {
      state.selectingGoal = !state.selectingGoal;
      state.selectionPointer = null;
      pickButton.classList.toggle('is-on', state.selectingGoal);
      pickButton.setAttribute('aria-pressed', String(state.selectingGoal));
      document.body.classList.toggle('nav-selecting', state.selectingGoal);
      navigationMessage(state.selectingGoal
        ? 'Нажмите на свободную клетку карты, чтобы поставить цель.'
        : 'Выбор цели отменён.', state.selectingGoal ? 'warn' : '', true);
    });

    for (const [xId, yId] of [['nav-goal-x', 'nav-goal-y'], ['route-win-goal-x', 'route-win-goal-y']]) {
      const xIn = $(xId);
      const yIn = $(yId);
      const onGoalInput = () => {
        if (xIn && yIn && String(xIn.value).trim() && String(yIn.value).trim()) {
          const xv = Number(xIn.value);
          const yv = Number(yIn.value);
          if (Number.isFinite(xv) && Number.isFinite(yv)) {
            state.goalPoint = { x: xv, y: yv };
            for (const otherX of ['nav-goal-x', 'route-win-goal-x']) {
              const el = $(otherX);
              if (el && el !== xIn) el.value = xIn.value;
            }
            for (const otherY of ['nav-goal-y', 'route-win-goal-y']) {
              const el = $(otherY);
              if (el && el !== yIn) el.value = yIn.value;
            }
          }
        }
        if (state.navigationStatus) renderNavigationStatus(state.navigationStatus);
        draw();
        drawRouteWindow();
      };
      if (xIn) xIn.addEventListener('input', onGoalInput);
      if (yIn) yIn.addEventListener('input', onGoalInput);
    }

    const canvas = $('map-canvas');
    if (canvas) {
      canvas.tabIndex = 0;
      const viewport = window.RCViewport ? window.RCViewport(canvas, (view) => {
        state.zoom = view.zoom; state.panX = view.x; state.panY = view.y; draw();
      }, { min: 0.4, max: 8, dragThreshold: 6, canPan: () => !state.selectingGoal }) : null;

      canvas.addEventListener('pointerdown', (event) => {
        if (event.pointerType === 'mouse' && event.button !== 0) return;
        state.selectionPointer = { x: event.clientX, y: event.clientY, moved: false };
      });
      canvas.addEventListener('pointermove', (event) => {
        if (state.selectionPointer && state.selectionPointer.moved !== true
            && Math.hypot(event.clientX - state.selectionPointer.x,
                          event.clientY - state.selectionPointer.y) > 8) {
          state.selectionPointer.moved = true;
        }
      });
      canvas.addEventListener('pointerup', (event) => {
        if (event.pointerType === 'mouse' && event.button !== 0) return;
        const ptr = state.selectionPointer;
        state.selectionPointer = null;
        if (!ptr || ptr.moved) return;
        state.lastGoalPointerUpAt = Date.now();
        setGoalFromCanvas(event);
      });
      canvas.addEventListener('click', (event) => {
        if (Date.now() - state.lastGoalPointerUpAt < 350) return;
        if (state.selectionPointer && state.selectionPointer.moved) {
          state.selectionPointer = null;
          return;
        }
        state.selectionPointer = null;
        setGoalFromCanvas(event);
      });

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

    const routeCanvas = $('route-motion-canvas');
    if (routeCanvas && window.RCViewport) {
      window.RCViewport(routeCanvas, (view) => {
        state.routeZoom = view.zoom;
        state.routePanX = view.x;
        state.routePanY = view.y;
        drawRouteWindow();
      }, { min: 0.5, max: 6, dragThreshold: 4 });
    }

    window.addEventListener('resize', () => { draw(); drawRouteWindow(); });
    if (window.RCRobotModel) {
      window.RCRobotModel.bindColors('map-wheel-target-color', 'map-wheel-actual-color', 'map-wheel-colors-reset');
      window.RCRobotModel.subscribe(() => { draw(); drawRouteWindow(); });
    }
    fillSettings(DEFAULTS);
    void fetchMap();
    void refreshNavigation();
    if (state.timer) clearInterval(state.timer);
    state.timer = setInterval(() => { void fetchMap(); }, POLL_MS);
    if (state.navigationTimer) clearInterval(state.navigationTimer);
    state.navigationTimer = setInterval(() => { void refreshNavigation(); }, POLL_MS);

    const isKiosk = document.body && document.body.classList
      && document.body.classList.contains('kiosk-body');
    if (!isKiosk) {
      void fetchRobotState();
      if (state.robotTimer) clearInterval(state.robotTimer);
      state.robotTimer = setInterval(() => { void fetchRobotState(); }, POLL_MS);
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
      drawRouteWindow();
      void fetchMap();
      void refreshNavigation();
    },
    draw,
    drawRouteWindow,
    openRouteWindow,
    closeRouteWindow,
    startRoutePreview,
    setScan(points) {
      state.scan = Array.isArray(points) ? points : [];
      if (state.snapshot) draw();
      drawRouteWindow();
    },
    getSnapshot() { return state.snapshot; },
  };
})();
