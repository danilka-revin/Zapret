/* Live single-frame Kinect depth preview. This is not a reconstructed map. */
(function (root) {
  'use strict';
  function decodeFrame(data) {
    if (!data || !data.available || !data.oneFrameOnly || !data.notForNavigation ||
        !Number.isInteger(data.width) || data.width < 1 || data.width > 160 ||
        !Number.isInteger(data.height) || data.height < 1 || data.height > 120 ||
        !Number.isInteger(data.stride) || data.stride < 1 || data.stride > 640 ||
        ![data.fx, data.fy, data.cx, data.cy].every(Number.isFinite) || data.fx <= 0 || data.fy <= 0 ||
        typeof data.data !== 'string') return [];
    let raw;
    try { raw = root.atob(data.data); } catch (_) { return []; }
    if (raw.length !== data.width * data.height * 2) return [];
    const result = [];
    for (let y = 0; y < data.height; y++) {
      for (let x = 0; x < data.width; x++) {
        const i = (y * data.width + x) * 2;
        const mm = raw.charCodeAt(i) | (raw.charCodeAt(i + 1) << 8);
        if (!mm) continue;
        const z = mm / 1000;
        result.push([(x * data.stride - data.cx) * z / data.fx,
          -(y * data.stride - data.cy) * z / data.fy, z]);
      }
    }
    return result;
  }
  if (typeof module !== 'undefined' && module.exports) module.exports = { decodeFrame };
  if (!root.document) return;

  const $ = (id) => root.document.getElementById(id);
  let active = false, points = [], yaw = 0.25, pitch = -0.15, zoom = 1;
  let deadline = 0, running = false, generation = 0, lastFrameId = null, lastDeadline = 0;
  let timer = null;
  const pointers = new Map();
  const clamp = (v, min, max) => Math.max(min, Math.min(max, v));

  function setMessage(message) {
    const label = $('kinect-3d-status');
    if (label) label.textContent = message;
  }
  function draw() {
    const canvas = $('kinect-3d-canvas');
    if (!canvas || !active) return;
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const ratio = Math.min(root.devicePixelRatio || 1, 2);
    const width = Math.round(rect.width * ratio), height = Math.round(rect.height * ratio);
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.clearRect(0, 0, width, height);
    const now = performance.now();
    if (now >= deadline) points = [];
    $('kinect-3d-empty').hidden = points.length > 0;
    if (!points.length) return;
    // Orbit around a point two metres in front of the camera.
    const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
    const focal = Math.min(width, height) * 1.05 * zoom;
    const size = Math.max(1.5 * ratio, Math.min(3 * ratio, width / 220));
    for (const point of points) {
      const x = point[0], y = point[1], z = point[2] - 2;
      const rx = x * cy + z * sy, rz = -x * sy + z * cy;
      const ry = y * cp - rz * sp, depth = y * sp + rz * cp + 6;
      if (depth <= 0.1) continue;
      const sx = width / 2 + rx * focal / depth, screenY = height / 2 - ry * focal / depth;
      if (sx < 0 || sx >= width || screenY < 0 || screenY >= height) continue;
      const hue = Math.round(195 - clamp(point[2], 0.45, 4) * 22);
      ctx.fillStyle = 'hsl(' + hue + ', 85%, 65%)';
      ctx.fillRect(sx, screenY, size, size);
    }
  }
  function expire(message) {
    points = []; deadline = 0;
    setMessage(message);
    draw();
  }
  async function poll() {
    if (!active || root.document.hidden || running) return;
    running = true;
    const current = generation, started = performance.now();
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 2500);
    try {
      const response = await fetch('/api/kinect/preview', {cache: 'no-store', signal: controller.signal});
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const data = await response.json();
      if (!active || generation !== current) return;
      const now = performance.now();
      // Subtract the entire request duration so a slow response cannot extend freshness.
      const remaining = (Number(data.maxAgeS) - Number(data.ageS)) * 1000 - (now - started);
      if (!data.available || !Number.isFinite(remaining) || remaining <= 0 ||
          !Number.isSafeInteger(data.frameId) || data.frameId < 1) {
        expire(data.source && data.source !== 'depth' ? 'Нужен сырой кадр глубины Kinect' : 'Нет свежего кадра глубины Kinect');
        return;
      }
      let nextDeadline = now + Math.min(remaining, 1500);
      if (data.frameId === lastFrameId) nextDeadline = Math.min(nextDeadline, lastDeadline);
      else { lastFrameId = data.frameId; }
      lastDeadline = nextDeadline;
      if (nextDeadline <= now) { expire('Кадр Kinect устарел'); return; }
      const decoded = decodeFrame(data);
      if (!decoded.length) { expire('Нет точек в кадре глубины'); return; }
      points = decoded; deadline = nextDeadline;
      const label = data.captureSource === 'demo' ? 'ДЕМО · синтетический кадр' : 'Свежий кадр';
      setMessage(label + ' · ' + decoded.length.toLocaleString('ru-RU') + ' точек');
      draw();
    } catch (_) {
      if (active && generation === current) expire('Нет связи с Kinect или бортовым сервером');
    } finally {
      clearTimeout(timeout);
      running = false;
    }
  }
  function setActive(value) {
    active = value; generation++;
    $('sensor-lidar').setAttribute('aria-pressed', String(!active));
    $('sensor-kinect').setAttribute('aria-pressed', String(active));
    $('sensor-lidar-view').hidden = active;
    $('sensor-kinect-view').hidden = !active;
    $('lidar-meta').hidden = active;
    pointers.clear();
    if (active) { expire('Ожидание кадра Kinect'); lastFrameId = null; lastDeadline = 0; void poll(); }
    else { points = []; deadline = 0; }
  }
  function bind() {
    const canvas = $('kinect-3d-canvas');
    if (!canvas || !$('sensor-lidar') || !$('sensor-kinect')) return;
    $('sensor-lidar').addEventListener('click', () => setActive(false));
    $('sensor-kinect').addEventListener('click', () => setActive(true));
    $('kinect-zoom-in').addEventListener('click', () => { zoom = clamp(zoom * 1.25, .4, 5); draw(); });
    $('kinect-zoom-out').addEventListener('click', () => { zoom = clamp(zoom / 1.25, .4, 5); draw(); });
    $('kinect-reset-view').addEventListener('click', () => { yaw = .25; pitch = -.15; zoom = 1; draw(); });
    canvas.addEventListener('wheel', (e) => { e.preventDefault(); zoom = clamp(zoom * Math.exp(-e.deltaY * .001), .4, 5); draw(); }, {passive: false});
    canvas.addEventListener('keydown', (e) => {
      if (e.key === 'ArrowLeft') yaw -= .15;
      else if (e.key === 'ArrowRight') yaw += .15;
      else if (e.key === 'ArrowUp') pitch = clamp(pitch - .15, -1.45, 1.45);
      else if (e.key === 'ArrowDown') pitch = clamp(pitch + .15, -1.45, 1.45);
      else return;
      e.preventDefault(); draw();
    });
    canvas.addEventListener('pointerdown', (e) => {
      e.preventDefault(); canvas.setPointerCapture(e.pointerId);
      pointers.set(e.pointerId, {x: e.clientX, y: e.clientY});
    });
    canvas.addEventListener('pointermove', (e) => {
      if (!pointers.has(e.pointerId)) return;
      e.preventDefault();
      const before = pointers.get(e.pointerId), others = [...pointers.entries()].filter(([id]) => id !== e.pointerId);
      if (others.length) {
        const other = others[0][1];
        const oldDist = Math.hypot(before.x - other.x, before.y - other.y);
        const newDist = Math.hypot(e.clientX - other.x, e.clientY - other.y);
        if (oldDist > 5) zoom = clamp(zoom * newDist / oldDist, .4, 5);
      } else {
        yaw += (e.clientX - before.x) * .01;
        pitch = clamp(pitch + (e.clientY - before.y) * .01, -1.45, 1.45);
      }
      pointers.set(e.pointerId, {x: e.clientX, y: e.clientY}); draw();
    });
    const release = (e) => pointers.delete(e.pointerId);
    canvas.addEventListener('pointerup', release);
    canvas.addEventListener('pointercancel', release);
    canvas.addEventListener('lostpointercapture', release);
    root.addEventListener('resize', draw);
    root.document.addEventListener('visibilitychange', () => {
      if (root.document.hidden) { generation++; expire('Просмотр приостановлен'); }
      else if (active) void poll();
    });
    timer = setInterval(() => {
      if (!active) return;
      if (points.length && performance.now() >= deadline) expire('Кадр Kinect устарел');
      void poll();
    }, 500);
  }
  root.RCKinectView = {active: () => active};
  if (root.document.readyState === 'loading') root.document.addEventListener('DOMContentLoaded', bind);
  else bind();
})(typeof window !== 'undefined' ? window : globalThis);
