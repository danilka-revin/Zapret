/* Live 3D sensor view: Kinect depth and a current LiDAR scan, never accumulated or saved. */
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
  const LIDAR_HEIGHT_M = 0.55;
  const KINECT_FORWARD_OF_LIDAR_M = 0.13;
  const DEFAULT_LIDAR_OFFSET_M = 0.28;
  function transformLidar(points, lidarOffsetM = DEFAULT_LIDAR_OFFSET_M, heightM = LIDAR_HEIGHT_M) {
    if (!Array.isArray(points)) return [];
    const offset = Number.isFinite(Number(lidarOffsetM)) ? Number(lidarOffsetM) : DEFAULT_LIDAR_OFFSET_M;
    const height = Number.isFinite(Number(heightM)) ? Number(heightM) : LIDAR_HEIGHT_M;
    return points.filter(p => p && Number.isFinite(p.a) && Number.isFinite(p.r) && p.r >= 0)
      .map(p => [-p.r * Math.sin(p.a), height, offset + p.r * Math.cos(p.a), 'lidar']);
  }
  function transformKinect(points, mount = {}, lidarOffsetM = DEFAULT_LIDAR_OFFSET_M) {
    if (!Array.isArray(points)) return [];
    const offset = Number.isFinite(Number(lidarOffsetM)) ? Number(lidarOffsetM) : DEFAULT_LIDAR_OFFSET_M;
    const zMount = Number.isFinite(Number(mount.z_m)) ? Number(mount.z_m) : 0.45;
    const yaw = (Number.isFinite(Number(mount.yaw_deg)) ? Number(mount.yaw_deg) : 0) * Math.PI / 180;
    const pitch = (Number.isFinite(Number(mount.pitch_deg)) ? Number(mount.pitch_deg) : 0) * Math.PI / 180;
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    const cosP = Math.cos(pitch), sinP = Math.sin(pitch);
    const sensorX = offset + KINECT_FORWARD_OF_LIDAR_M;
    return points.filter(p => p && p.length >= 3 && p.slice(0, 3).every(Number.isFinite)).map(p => {
      const xRight = p[0], yUp = p[1], zForward = p[2];
      const pitchedUp = yUp * cosP - zForward * sinP;
      const pitchedForward = yUp * sinP + zForward * cosP;
      const robotForward = pitchedForward * cosY + xRight * sinY;
      const robotLeft = pitchedForward * sinY - xRight * cosY;
      return [-robotLeft, zMount + pitchedUp, sensorX + robotForward, 'kinect'];
    });
  }
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = { decodeFrame, transformLidar, transformKinect };
  }
  if (!root.document) return;

  const $ = (id) => root.document.getElementById(id);
  let mode = 'lidar', kinectPoints = [], lidarCloud = [], yaw = 0.25, pitch = -0.15, zoom = 1;
  let deadline = 0, lidarDeadline = 0, lidarOffsetM = DEFAULT_LIDAR_OFFSET_M;
  let kinectMount = {z_m: 0.45, yaw_deg: 0, pitch_deg: 0};
  let running = false, generation = 0, lastFrameId = null, lastDeadline = 0;
  let timer = null;
  const pointers = new Map();
  const clamp = (v, min, max) => Math.max(min, Math.min(max, v));

  function setMessage(message) {
    const label = $('kinect-3d-status');
    if (label) label.textContent = message;
  }
  function pointCountText(n) {
    const tens = n % 100, ones = n % 10;
    const noun = ones === 1 && tens !== 11 ? 'точка'
      : ones >= 2 && ones <= 4 && (tens < 12 || tens > 14) ? 'точки' : 'точек';
    return n.toLocaleString('ru-RU') + ' ' + noun;
  }
  function updateModeMessage(now) {
    if (mode !== 'combined') return;
    const kinectFresh = now < deadline && kinectPoints.length > 0;
    const lidarFresh = now < lidarDeadline && lidarCloud.length > 0;
    const kText = kinectFresh ? pointCountText(kinectPoints.length) : 'нет кадра';
    const lText = lidarFresh ? pointCountText(lidarCloud.length) : 'нет скана';
    setMessage('Лидар: ' + lText + ' · Kinect: ' + kText + ' · live, без сохранения');
    const note = $('kinect-3d-mode-note');
    const kinectZ = Number.isFinite(Number(kinectMount.z_m)) ? Number(kinectMount.z_m) : 0.45;
    if (note) note.textContent = 'Лидар z=550 мм (оранжевый) · Kinect +130 мм вперёд (сине-зелёный), z=' +
      Math.round(kinectZ * 1000) + ' мм';
  }
  function draw() {
    const canvas = $('kinect-3d-canvas');
    if (!canvas || mode === 'lidar') return;
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const ratio = Math.min(root.devicePixelRatio || 1, 2);
    const width = Math.round(rect.width * ratio), height = Math.round(rect.height * ratio);
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.clearRect(0, 0, width, height);
    const now = performance.now();
    const liveKinect = now < deadline ? kinectPoints : [];
    const liveLidar = now < lidarDeadline ? lidarCloud : [];
    const displayPoints = mode === 'combined'
      ? transformKinect(liveKinect, kinectMount, lidarOffsetM).concat(liveLidar)
      : liveKinect;
    const empty = $('kinect-3d-empty');
    if (empty) {
      empty.hidden = displayPoints.length > 0;
      empty.textContent = mode === 'combined'
        ? 'Ожидание свежего скана лидара и кадра Kinect'
        : 'Нет свежего кадра глубины Kinect';
    }
    updateModeMessage(now);
    if (!displayPoints.length) return;
    // Orbit around a point two metres in front of the sensor/robot frame.
    const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
    const focal = Math.min(width, height) * 1.05 * zoom;
    const size = Math.max(1.5 * ratio, Math.min(3 * ratio, width / 220));
    const project = (point) => {
      const z = point[2] - 2;
      const rx = point[0] * cy + z * sy, rz = -point[0] * sy + z * cy;
      const ry = point[1] * cp - rz * sp, depth = point[1] * sp + rz * cp + 6;
      if (depth <= 0.1) return null;
      return {x: width / 2 + rx * focal / depth, y: height / 2 - ry * focal / depth};
    };
    if (mode === 'combined') {
      ctx.strokeStyle = 'rgba(145, 170, 190, 0.20)';
      ctx.lineWidth = ratio;
      const gridLine = (a, b) => {
        const p1 = project(a), p2 = project(b);
        if (!p1 || !p2) return;
        ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
      };
      for (let forward = 0; forward <= 6; forward += 1) gridLine([-3, 0, forward], [3, 0, forward]);
      for (let left = -3; left <= 3; left += 1) gridLine([-left, 0, 0], [-left, 0, 6]);
    }
    for (const point of displayPoints) {
      const screen = project(point);
      if (!screen || screen.x < 0 || screen.x >= width || screen.y < 0 || screen.y >= height) continue;
      if (point[3] === 'lidar') ctx.fillStyle = '#ffb34d';
      else {
        const hue = Math.round(195 - clamp(point[2], 0.45, 4) * 22);
        ctx.fillStyle = 'hsl(' + hue + ', 85%, 65%)';
      }
      ctx.fillRect(screen.x, screen.y, size, size);
    }
  }
  function expire(message) {
    kinectPoints = []; deadline = 0;
    if (mode !== 'combined') setMessage(message);
    draw();
  }
  async function poll() {
    if (mode === 'lidar' || root.document.hidden || running) return;
    running = true;
    const current = generation, started = performance.now();
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 2500);
    try {
      const response = await fetch('/api/kinect/preview', {cache: 'no-store', signal: controller.signal});
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const data = await response.json();
      if (mode === 'lidar' || generation !== current) return;
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
      kinectPoints = decoded;
      if (data.mount && typeof data.mount === 'object') kinectMount = data.mount;
      deadline = nextDeadline;
      if (mode !== 'combined') {
        const label = data.captureSource === 'demo' ? 'ДЕМО · синтетический кадр' : 'Свежий кадр';
        setMessage(label + ' · ' + pointCountText(decoded.length));
      }
      draw();
    } catch (_) {
      if (mode !== 'lidar' && generation === current) expire('Нет связи с Kinect или бортовым сервером');
    } finally {
      clearTimeout(timeout);
      running = false;
    }
  }
  function setLidarScan(points, status) {
    const ageS = Number(status && status.age_s);
    const fresh = !!(status && status.connected === true && status.demo !== true &&
      Number.isFinite(ageS) && ageS >= 0 && ageS < 1.5 && Array.isArray(points) && points.length);
    lidarOffsetM = Number.isFinite(Number(status && status.lidar_offset_m))
      ? Number(status.lidar_offset_m) : DEFAULT_LIDAR_OFFSET_M;
    lidarCloud = fresh ? transformLidar(points, lidarOffsetM) : [];
    lidarDeadline = fresh ? performance.now() + Math.max(0, (1.5 - ageS) * 1000) : 0;
    if (mode === 'combined') draw();
  }
  function setMode(next) {
    if (!['lidar', 'kinect', 'combined'].includes(next)) return;
    mode = next; generation++;
    const active = mode !== 'lidar';
    $('sensor-lidar').setAttribute('aria-pressed', String(mode === 'lidar'));
    $('sensor-kinect').setAttribute('aria-pressed', String(mode === 'kinect'));
    $('sensor-combined').setAttribute('aria-pressed', String(mode === 'combined'));
    $('sensor-lidar-view').hidden = active;
    $('sensor-kinect-view').hidden = !active;
    $('lidar-meta').hidden = active;
    const note = $('kinect-3d-mode-note');
    if (note && mode !== 'combined') note.textContent = 'Один кадр · не для навигации';
    pointers.clear();
    if (active) {
      expire(mode === 'combined' ? 'Ожидание свежего скана и кадра' : 'Ожидание кадра Kinect');
      lastFrameId = null; lastDeadline = 0; void poll();
    } else {
      kinectPoints = []; deadline = 0; draw();
    }
  }
  function bind() {
    const canvas = $('kinect-3d-canvas');
    if (!canvas || !$('sensor-lidar') || !$('sensor-kinect') || !$('sensor-combined')) return;
    $('sensor-lidar').addEventListener('click', () => setMode('lidar'));
    $('sensor-kinect').addEventListener('click', () => setMode('kinect'));
    $('sensor-combined').addEventListener('click', () => setMode('combined'));
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
      else if (mode !== 'lidar') void poll();
    });
    timer = setInterval(() => {
      if (mode === 'lidar') return;
      if (kinectPoints.length && performance.now() >= deadline) expire('Кадр Kinect устарел');
      draw();
      void poll();
    }, 500);
  }
  root.RCKinectView = {active: () => mode !== 'lidar', mode: () => mode, setLidarScan};
  if (root.document.readyState === 'loading') root.document.addEventListener('DOMContentLoaded', bind);
  else bind();
})(typeof window !== 'undefined' ? window : globalThis);
