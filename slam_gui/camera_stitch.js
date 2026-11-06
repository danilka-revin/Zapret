/* Browser-side live composition of two existing robot camera feeds. */
(function () {
  'use strict';

  const STORAGE_KEY = 'robot-camera-stitch-v1';
  const IDS = [0, 1, 2, 3];
  const NAMES = ['FL', 'RL', 'FR', 'RR'];
  const DEFAULTS = {
    left: { camera: 0, mirror: false, crop: 0, zoom: 100, x: 0, y: 0, roll: 0 },
    right: { camera: 2, mirror: false, crop: 0, zoom: 100, x: 0, y: 0, roll: 0 },
    grid: true,
  };
  const RANGES = {
    crop: [0, 45], zoom: [80, 220], x: [-50, 50], y: [-40, 40], roll: [-15, 15],
  };
  const $ = (id) => document.getElementById(id);
  let config = readConfig();
  let canvas, ctx, panel, toggle, status, seam;
  let active = false;
  let initialized = false;
  let unsubscribe = null;

  function copyDefaults() {
    return JSON.parse(JSON.stringify(DEFAULTS));
  }

  function readConfig() {
    const result = copyDefaults();
    try {
      const saved = JSON.parse(window.localStorage.getItem(STORAGE_KEY) || 'null');
      if (!saved || typeof saved !== 'object') return result;
      for (const side of ['left', 'right']) {
        const item = saved[side];
        if (!item || typeof item !== 'object') continue;
        if (IDS.includes(Number(item.camera))) result[side].camera = Number(item.camera);
        result[side].mirror = !!item.mirror;
        for (const key of Object.keys(RANGES)) {
          const value = Number(item[key]);
          if (Number.isFinite(value)) result[side][key] = Math.max(RANGES[key][0], Math.min(RANGES[key][1], value));
        }
      }
      if (result.left.camera === result.right.camera) result.right.camera = result.left.camera === 2 ? 0 : 2;
      result.grid = saved.grid !== false;
    } catch (_) { /* storage may be unavailable or contain malformed JSON */ }
    return result;
  }

  function persist() {
    try { window.localStorage.setItem(STORAGE_KEY, JSON.stringify(config)); }
    catch (_) { /* private browsing/quota: controls remain usable for this page */ }
  }

  function controlId(side, key) { return 'cam-stitch-' + side + '-' + key; }

  function format(key, value) {
    if (key === 'zoom') return Math.round(value) + '%';
    if (key === 'roll') return Number(value).toFixed(1).replace(/\.0$/, '') + '°';
    return Math.round(value) + '%';
  }

  function syncControls() {
    for (const side of ['left', 'right']) {
      for (const key of ['camera', 'mirror', ...Object.keys(RANGES)]) {
        const input = $(controlId(side, key));
        if (!input) continue;
        if (input.type === 'checkbox') input.checked = !!config[side][key];
        else input.value = String(config[side][key]);
        const output = $(controlId(side, key + '-value'));
        if (output) output.textContent = format(key, config[side][key]);
      }
    }
    const grid = $('cam-stitch-grid');
    if (grid) grid.checked = config.grid;
    if (seam) seam.hidden = !config.grid;
  }

  function drawSide(side, bounds, dpr) {
    const paneWidth = bounds.width / 2;
    const frame = window.RCCameraView && window.RCCameraView.getFrame(config[side].camera);
    const cameraIndex = config[side].camera;
    const centerX = (side === 'left' ? paneWidth / 2 : paneWidth * 1.5) + (config[side].x / 100) * paneWidth;
    const centerY = bounds.height / 2 + (config[side].y / 100) * bounds.height;
    ctx.save();
    ctx.beginPath();
    ctx.rect(side === 'left' ? 0 : paneWidth, 0, paneWidth, bounds.height);
    ctx.clip();
    if (frame && frame.naturalWidth > 0 && frame.naturalHeight > 0) {
      const cropFraction = config[side].crop / 100;
      // Trim only the edge facing the join; mirroring swaps the source edge.
      const cropRight = (side === 'left') !== config[side].mirror;
      const sourceWidth = frame.naturalWidth * (1 - cropFraction);
      const sourceX = cropRight ? frame.naturalWidth - sourceWidth : 0;
      const scale = Math.max(paneWidth / sourceWidth, bounds.height / frame.naturalHeight) * config[side].zoom / 100;
      const drawWidth = sourceWidth * scale;
      const drawHeight = frame.naturalHeight * scale;
      ctx.translate(centerX, centerY);
      ctx.rotate(config[side].roll * Math.PI / 180);
      ctx.scale(config[side].mirror ? -1 : 1, 1);
      ctx.drawImage(frame, sourceX, 0, sourceWidth, frame.naturalHeight,
        -drawWidth / 2, -drawHeight / 2, drawWidth, drawHeight);
    } else {
      ctx.fillStyle = '#c2c2ca';
      ctx.font = '600 14px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('Нет кадра · камера ' + NAMES[cameraIndex], centerX, centerY);
    }
    ctx.restore();

    if (config.grid) {
      ctx.save();
      ctx.strokeStyle = 'rgba(255,255,255,.36)';
      ctx.lineWidth = 1 / dpr;
      ctx.setLineDash([5 / dpr, 5 / dpr]);
      ctx.beginPath();
      const left = side === 'left' ? 0 : paneWidth;
      ctx.moveTo(left + paneWidth / 2, 0);
      ctx.lineTo(left + paneWidth / 2, bounds.height);
      ctx.moveTo(left, bounds.height / 2);
      ctx.lineTo(left + paneWidth, bounds.height / 2);
      ctx.stroke();
      ctx.restore();
      ctx.save();
      ctx.fillStyle = 'rgba(0,0,0,.55)';
      ctx.fillRect(side === 'left' ? 8 : paneWidth + 8, 8, 54, 24);
      ctx.fillStyle = '#fff';
      ctx.font = '700 12px system-ui, sans-serif';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText(NAMES[cameraIndex], side === 'left' ? 16 : paneWidth + 16, 20);
      ctx.restore();
    }
  }

  function render() {
    if (!canvas || !ctx) return;
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const pixelWidth = Math.round(rect.width * dpr);
    const pixelHeight = Math.round(rect.height * dpr);
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#050506';
    ctx.fillRect(0, 0, rect.width, rect.height);
    drawSide('left', rect, dpr);
    drawSide('right', rect, dpr);
    const live = ['left', 'right'].map((side) => {
      const id = config[side].camera;
      return NAMES[id] + ' ' + (window.RCCameraView && window.RCCameraView.getFrame(id) ? 'LIVE' : 'нет сигнала');
    });
    if (status) status.textContent = live.join(' · ');
  }

  function bindControls() {
    for (const side of ['left', 'right']) {
      for (const key of ['camera', 'mirror', ...Object.keys(RANGES)]) {
        const input = $(controlId(side, key));
        if (!input) continue;
        input.addEventListener('input', () => updateControl(side, key, input));
        input.addEventListener('change', () => updateControl(side, key, input));
      }
    }
    const grid = $('cam-stitch-grid');
    if (grid) grid.addEventListener('change', () => {
      config.grid = grid.checked;
      if (seam) seam.hidden = !config.grid;
      persist();
      render();
    });
    if (toggle) toggle.addEventListener('click', () => setOpen(panel.hidden));
    const reset = $('cam-stitch-reset');
    if (reset) reset.addEventListener('click', () => {
      config = copyDefaults();
      syncControls();
      persist();
      render();
    });
  }

  function updateControl(side, key, input) {
    const oldCamera = config[side].camera;
    if (key === 'camera') {
      const next = Number(input.value);
      if (!IDS.includes(next)) return;
      const other = side === 'left' ? 'right' : 'left';
      if (next === config[other].camera) {
        config[other].camera = oldCamera;
        const otherControl = $(controlId(other, 'camera'));
        if (otherControl) otherControl.value = String(oldCamera);
      }
      config[side].camera = next;
    } else if (key === 'mirror') config[side].mirror = input.checked;
    else {
      const value = Number(input.value);
      if (!Number.isFinite(value)) return;
      config[side][key] = Math.max(RANGES[key][0], Math.min(RANGES[key][1], value));
      const output = $(controlId(side, key + '-value'));
      if (output) output.textContent = format(key, config[side][key]);
    }
    persist();
    render();
  }

  function setOpen(open) {
    if (!panel || !toggle) return;
    panel.hidden = !open;
    toggle.setAttribute('aria-pressed', String(open));
    toggle.textContent = open ? 'Обычный обзор' : 'Сшить 2 камеры';
    const view = $('swipe-cameras-view');
    if (view) view.classList.toggle('camera-stitch-open', open);
    if (open) requestAnimationFrame(render);
  }

  function init() {
    if (initialized) return;
    initialized = true;
    canvas = $('cam-stitch-canvas');
    panel = $('cam-stitch-panel');
    toggle = $('cam-stitch-toggle');
    status = $('cam-stitch-status');
    seam = document.querySelector('.cam-stitch-seam');
    if (!canvas || !panel || !toggle) return;
    ctx = canvas.getContext('2d');
    if (!ctx) return;
    syncControls();
    bindControls();
    if (window.RCCameraView && window.RCCameraView.subscribe) {
      unsubscribe = window.RCCameraView.subscribe(() => render());
    }
    window.addEventListener('resize', render);
    if (typeof ResizeObserver !== 'undefined') new ResizeObserver(render).observe(canvas);
    render();
  }

  window.RCCameraStitch = {
    setActive(value) {
      init();
      active = !!value;
      if (active) requestAnimationFrame(render);
      else if (unsubscribe) { unsubscribe(); unsubscribe = null; }
      if (active && !unsubscribe && window.RCCameraView && window.RCCameraView.subscribe) {
        unsubscribe = window.RCCameraView.subscribe(() => render());
      }
    },
    setOpen,
    render,
  };

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init, { once: true });
  else init();
})();
