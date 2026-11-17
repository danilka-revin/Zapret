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
  function orbitCoordinates(point, yaw, pitch, center = [0, 0, 0]) {
    const x = point[0] - center[0], y = point[1] - center[1], z = point[2] - center[2];
    const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
    const rx = x * cy + z * sy, rz = -x * sy + z * cy;
    return [rx, y * cp - rz * sp, y * sp + rz * cp];
  }
  const SIGN_FACE_COLORS = {
    stop_sign: '#c81e3a', pedestrian_crossing: '#1565c0', speed_bump: '#1565c0',
    traffic_light: '#111827', cone: '#f97316'
  };
  const SIGNAL_COLORS = {red: '#ff4d5e', yellow: '#ffd23f', green: '#3ddc84'};
  const SIGN_ASCII = {stop_sign: 'STOP', pedestrian_crossing: 'PED', speed_bump: 'BUMP',
    traffic_light: 'LIGHT', cone: 'CONE'};
  const SIGN_SHAPES = {stop_sign: 'octagon', pedestrian_crossing: 'square',
    speed_bump: 'square', traffic_light: 'panel', cone: 'cone'};
  function finiteNumber(value, fallback) {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  }
  function visionSignGeometry(marker) {
    const data = marker || {};
    const x = finiteNumber(data.x_m, 0), y = finiteNumber(data.y_m, 0);
    const z = finiteNumber(data.z_m, finiteNumber(data.centerZ_m, 0.9));
    // Карта: X вперёд, Y влево. 3D-вид: X вправо, Y вверх, Z вперёд.
    const center = [-y, z, x];
    const headingDeg = finiteNumber(data.headingDeg,
      finiteNumber(data.bearingDeg, Math.atan2(y, x) * 180 / Math.PI));
    const h = headingDeg * Math.PI / 180;
    const dir = [-Math.sin(h), 0, Math.cos(h)];       // от робота к знаку
    const normal = [-dir[0], 0, -dir[2]];             // лицевая сторона — к роботу
    const right = [normal[2], 0, -normal[0]];
    return {
      center, right, up: [0, 1, 0], normal,
      width: Math.max(.05, finiteNumber(data.width_m, .30)),
      height: Math.max(.05, finiteNumber(data.height_m, .30)),
      thickness: Math.max(.005, finiteNumber(data.thickness_m, .02)),
      shape: String(data.shape || SIGN_SHAPES[data.class || data.className] || 'square'),
      className: String(data.class || data.className || ''),
      color: String(data.color || '#94a3b8'),
      short: String(data.short || ''), label: String(data.label || ''),
      signal: String(data.signal || 'unknown'),
      sizeScale: finiteNumber(data.sizeScale, 1),
      scale1to1: data.scale1to1 !== false,
      positionSource: String(data.positionSource || 'camera-estimate')
    };
  }
  function signLocalShape(geom) {
    const hw = geom.width / 2, hh = geom.height / 2;
    if (geom.shape === 'octagon') {
      const cut = Math.min(hw, hh) * .30;
      return [[-hw + cut, -hh], [hw - cut, -hh], [hw, -hh + cut], [hw, hh - cut],
        [hw - cut, hh], [-hw + cut, hh], [-hw, hh - cut], [-hw, -hh + cut]];
    }
    if (geom.shape === 'triangle' || geom.shape === 'cone') {
      return [[-hw, -hh], [hw, -hh], [0, hh]];
    }
    return [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]];
  }
  function signPoint(geom, u, v, offset) {
    return [geom.center[0] + geom.right[0] * u + geom.normal[0] * offset,
      geom.center[1] + geom.up[1] * v,
      geom.center[2] + geom.right[2] * u + geom.normal[2] * offset];
  }
  function visionSignFaces(marker) {
    const geom = visionSignGeometry(marker);
    const shape = signLocalShape(geom);
    const front = shape.map(([u, v]) => signPoint(geom, u, v, geom.thickness / 2));
    const back = shape.map(([u, v]) => signPoint(geom, u, v, -geom.thickness / 2));
    const fill = SIGN_FACE_COLORS[geom.className] || geom.color;
    const faces = [
      {vertices: front, color: fill, kind: 'front'},
      {vertices: back, color: fill, kind: 'back'}
    ];
    for (let i = 0; i < shape.length; i++) {
      const j = (i + 1) % shape.length;
      faces.push({vertices: [front[i], front[j], back[j], back[i]],
        color: '#1e293b', kind: 'side'});
    }
    return {geom, shape, faces};
  }
  function robotParts(live, geometry = {}, lidarOffsetM = DEFAULT_LIDAR_OFFSET_M) {
    const wheelbase = Number.isFinite(Number(geometry.wheelbase)) ? Number(geometry.wheelbase) : 0.50;
    const track = Number.isFinite(Number(geometry.track)) ? Number(geometry.track) : 0.38;
    const wheels = new Map((live && Array.isArray(live.wheels) ? live.wheels : []).map(w => [w.id, w]));
    const kinectForward = lidarOffsetM + KINECT_FORWARD_OF_LIDAR_M;
    const bevel = (width, length, cut) => [
      [-width / 2 + cut, -length / 2], [width / 2 - cut, -length / 2],
      [width / 2, -length / 2 + cut], [width / 2, length / 2 - cut],
      [width / 2 - cut, length / 2], [-width / 2 + cut, length / 2],
      [-width / 2, length / 2 - cut], [-width / 2, -length / 2 + cut]
    ];
    const parts = [
      // Low, chamfered dark shell and almost flush lid match the robot's angular case.
      {kind: 'shell', center: [0, .325, 0], yMin: .16, yMax: .49, footprint: bevel(.60, .75, .105)},
      {kind: 'roof', center: [0, .497, 0], yMin: .49, yMax: .503, footprint: bevel(.575, .725, .095)},
      // Kinect bar is mounted on the front face; its optical center is at the measured mount.
      {kind: 'kinectBezel', center: [0, .44, kinectForward - .025], size: [.47, .145, .028], yaw: 0},
      {kind: 'kinectGlass', center: [0, .445, kinectForward - .010], size: [.405, .078, .012], yaw: 0},
      ...[-.135, -.045, .055, .145].map((x, i) => ({
        kind: i === 2 ? 'kinectLed' : 'kinectLens', center: [x, .45, kinectForward], size: [.026, .027, .009], yaw: 0
      })),
      // Side camera bezels and amber marker lights are signature details of the shell.
      ...[-1, 1].flatMap(side => [
        {kind: 'sideBezel', center: [side * .301, .425, .075], size: [.022, .085, .16], yaw: 0},
        {kind: 'sideLens', center: [side * .315, .426, .075], size: [.009, .042, .105], yaw: 0},
        {kind: 'amber', center: [side * .304, .285, .075], size: [.018, .04, .12], yaw: 0}
      ]),
      // Four short standoffs and the compact top-mounted lidar puck.
      ...[-1, 1].flatMap(x => [-1, 1].map(z => ({kind: 'lidarPost',
        center: [x * .042, .523, lidarOffsetM + z * .042], size: [.012, .045, .012], yaw: 0}))),
      {kind: 'lidar', center: [0, LIDAR_HEIGHT_M, lidarOffsetM], size: [.135, .07, .135], yaw: 0},
      // Small silver grab handles on the top deck.
      ...[-1, 1].flatMap(side => [
        {kind: 'handleFoot', center: [side * .225, .514, .225], size: [.065, .018, .04], yaw: 0},
        {kind: 'handle', center: [side * .225, .538, .225], size: [.012, .038, .012], yaw: 0}
      ])
    ];
    for (const id of ['FL', 'FR', 'RL', 'RR']) {
      const front = id[0] === 'F', left = id[1] === 'L';
      const wheel = wheels.get(id);
      const angle = live && live.fresh && wheel && Number.isFinite(wheel.angle) ? wheel.angle * Math.PI / 180 : 0;
      parts.push({kind: 'wheel', id, angle: live && live.fresh && wheel && Number.isFinite(wheel.angle) ? wheel.angle : null,
        center: [left ? -track / 2 : track / 2, .145, front ? wheelbase / 2 : -wheelbase / 2],
        radius: .14, width: .085, yaw: angle});
    }
    return parts;
  }
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = { decodeFrame, transformLidar, transformKinect, robotParts, orbitCoordinates,
      visionSignGeometry, signLocalShape, signPoint, visionSignFaces };
  }
  if (!root.document) return;

  const $ = (id) => root.document.getElementById(id);
  let mode = 'lidar', kinectPoints = [], lidarCloud = [], yaw = 0.25, pitch = -0.15, zoom = 1;
  let signMarkers = [], signDeadline = 0;
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
    const signsFresh = now < signDeadline && signMarkers.length > 0;
    const scaled = signsFresh && signMarkers.find(marker => marker.scale1to1 === false);
    const scaleNote = scaled
      ? ' (масштаб ×' + Number(scaled.sizeScale || 1).toFixed(2) + ')'
      : ' (1:1)';
    const signNote = signsFresh ? ' · знаков: ' + signMarkers.length + scaleNote : '';
    if (note) note.textContent = 'Модель робота · фактические углы колёс (при свежей телеметрии) · Лидар z=550 мм · Kinect +130 мм вперёд, z=' +
      Math.round(kinectZ * 1000) + ' мм' + signNote;
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
      empty.hidden = displayPoints.length > 0 || mode === 'combined';
      empty.textContent = mode === 'combined'
        ? 'Ожидание свежего скана лидара и кадра Kinect'
        : 'Нет свежего кадра глубины Kinect';
    }
    updateModeMessage(now);
    if (!displayPoints.length && mode !== 'combined') return;
    // Orbit around the robot's ground-plane origin, shared with the LiDAR map.
    const focal = Math.min(width, height) * 1.05 * zoom;
    const size = Math.max(1.5 * ratio, Math.min(3 * ratio, width / 220));
    const project = (point) => {
      const [rx, ry, viewDepth] = orbitCoordinates(point, yaw, pitch);
      const depth = viewDepth + 6;
      if (depth <= 0.1) return null;
      return {x: width / 2 + rx * focal / depth, y: height / 2 - ry * focal / depth, depth};
    };
    const drawBox = (box, palette) => {
      const [cx, cy0, cz] = box.center, [sx, sy0, sz] = box.size;
      const c = Math.cos(box.yaw || 0), s = Math.sin(box.yaw || 0);
      const local = [[-1,-1,-1],[1,-1,-1],[1,-1,1],[-1,-1,1],[-1,1,-1],[1,1,-1],[1,1,1],[-1,1,1]];
      const vertices = local.map(([x,y,z]) => {
        const lx = x * sx / 2, lz = z * sz / 2;
        return [cx + lx * c + lz * s, cy0 + y * sy0 / 2, cz - lx * s + lz * c];
      });
      const faces = [
        {ids:[4,7,6,5], color:palette.top}, {ids:[3,2,6,7], color:palette.front},
        {ids:[0,4,5,1], color:palette.back}, {ids:[0,3,7,4], color:palette.left},
        {ids:[1,5,6,2], color:palette.right}, {ids:[0,1,2,3], color:palette.bottom}
      ].map(face => ({...face, points: face.ids.map(i => project(vertices[i]))}));
      faces.filter(face => face.points.every(Boolean)).sort((a, b) =>
        b.points.reduce((sum, p) => sum + p.depth, 0) - a.points.reduce((sum, p) => sum + p.depth, 0))
        .forEach(face => {
          ctx.beginPath(); ctx.moveTo(face.points[0].x, face.points[0].y);
          face.points.slice(1).forEach(p => ctx.lineTo(p.x, p.y)); ctx.closePath();
          ctx.fillStyle = face.color; ctx.fill(); ctx.strokeStyle = 'rgba(9, 13, 18, .9)'; ctx.lineWidth = Math.max(1, ratio); ctx.stroke();
        });
    };
    const drawFaces = (faces, edge = 'rgba(8, 10, 13, .95)') => {
      faces.map(face => ({...face, points: face.vertices.map(project)}))
        .filter(face => face.points.length >= 3 && face.points.every(Boolean))
        .sort((a, b) => b.points.reduce((sum, p) => sum + p.depth, 0) -
          a.points.reduce((sum, p) => sum + p.depth, 0))
        .forEach(face => {
          ctx.beginPath(); ctx.moveTo(face.points[0].x, face.points[0].y);
          face.points.slice(1).forEach(point => ctx.lineTo(point.x, point.y)); ctx.closePath();
          ctx.fillStyle = face.color; ctx.fill(); ctx.strokeStyle = edge;
          ctx.lineWidth = Math.max(1, ratio); ctx.stroke();
        });
    };
    /* Знак в масштабе 1:1: тонкая объёмная табличка, стойка до пола и symbol
     * на ближайшей к наблюдателю стороне. Все размеры — метры из CLASS_INFO. */
    const drawVisionSign = (marker) => {
      const built = visionSignFaces(marker), geom = built.geom;
      const ground = [geom.center[0], 0, geom.center[2]];
      const bottom = [geom.center[0], Math.max(0, geom.center[1] - geom.height / 2), geom.center[2]];
      const pg = project(ground), pb = project(bottom);
      if (pg && pb) {
        ctx.beginPath(); ctx.moveTo(pg.x, pg.y); ctx.lineTo(pb.x, pb.y);
        ctx.strokeStyle = 'rgba(148, 163, 184, .55)';
        ctx.lineWidth = Math.max(1, ratio); ctx.stroke();
        ctx.beginPath(); ctx.arc(pg.x, pg.y, 3 * ratio, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(148, 163, 184, .4)'; ctx.fill();
      }
      drawFaces(built.faces, 'rgba(8, 10, 13, .9)');
      const frontCenter = project(signPoint(geom, 0, 0, geom.thickness / 2 + .002));
      const backCenter = project(signPoint(geom, 0, 0, -geom.thickness / 2 - .002));
      const nearFront = !!(frontCenter && backCenter && frontCenter.depth < backCenter.depth);
      const near = nearFront ? frontCenter : (backCenter || frontCenter);
      if (!near) return;
      const offset = nearFront ? geom.thickness / 2 + .004 : -geom.thickness / 2 - .004;
      const edge = project(signPoint(geom, geom.width / 2, 0, offset));
      const pixelWidth = edge ? Math.max(6, Math.abs(edge.x - near.x) * 2) : 24;
      const fontPx = clamp(pixelWidth * .26, 8, 18) * ratio;
      if (geom.shape === 'octagon') {
        const inner = signLocalShape(geom).map(([u, v]) => [u * .78, v * .78]);
        const points = inner.map(([u, v]) => project(signPoint(geom, u, v, offset + .001)));
        if (points.every(Boolean)) {
          ctx.beginPath(); ctx.moveTo(points[0].x, points[0].y);
          points.slice(1).forEach(p => ctx.lineTo(p.x, p.y)); ctx.closePath();
          ctx.strokeStyle = '#fff'; ctx.lineWidth = Math.max(1.5, ratio * 1.5); ctx.stroke();
        }
      }
      if (geom.shape === 'square' &&
          (geom.className === 'pedestrian_crossing' || geom.className === 'speed_bump')) {
        const tri = [[-geom.width * .31, -geom.height * .29],
          [geom.width * .31, -geom.height * .29], [0, geom.height * .31]];
        const points = tri.map(([u, v]) => project(signPoint(geom, u, v, offset + .001)));
        if (points.every(Boolean)) {
          ctx.beginPath(); ctx.moveTo(points[0].x, points[0].y);
          points.slice(1).forEach(p => ctx.lineTo(p.x, p.y)); ctx.closePath();
          ctx.fillStyle = '#fff'; ctx.fill();
        }
      }
      if (geom.shape === 'panel') {
        const names = ['red', 'yellow', 'green'];
        const levels = [geom.height * .28, 0, -geom.height * .28];
        const radius = Math.min(geom.width, geom.height) * .17;
        names.forEach((name, index) => {
          const center = project(signPoint(geom, 0, levels[index], offset + .002));
          const rim = project(signPoint(geom, radius, levels[index], offset + .002));
          if (!center || !rim) return;
          const r = Math.max(2 * ratio, Math.abs(rim.x - center.x));
          ctx.beginPath(); ctx.arc(center.x, center.y, r, 0, Math.PI * 2);
          ctx.fillStyle = geom.signal === name ? SIGNAL_COLORS[name] : 'rgba(255, 255, 255, .16)';
          ctx.fill(); ctx.strokeStyle = 'rgba(0, 0, 0, .7)';
          ctx.lineWidth = Math.max(1, ratio); ctx.stroke();
        });
      } else {
        ctx.fillStyle = '#fff';
        ctx.font = '700 ' + fontPx + 'px ui-monospace, monospace';
        ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(SIGN_ASCII[geom.className] || geom.short || '•', near.x, near.y);
      }
    };
    const drawHull = (part, palette) => {
      const low = part.footprint.map(([x, z]) => [x, part.yMin, z]);
      const high = part.footprint.map(([x, z]) => [x, part.yMax, z]);
      const faces = [{vertices: high, color: palette.top}, {vertices: low, color: palette.bottom}];
      for (let i = 0; i < low.length; i++) {
        const next = (i + 1) % low.length;
        const shade = i === 2 || i === 6 ? palette.sideLight : palette.side;
        faces.push({vertices: [low[i], low[next], high[next], high[i]], color: shade});
      }
      drawFaces(faces, palette.edge);
    };
    const drawWheel = (part, palette) => {
      const [cx, cy0, cz] = part.center, c = Math.cos(part.yaw), s = Math.sin(part.yaw);
      const segments = 16;
      const point = (axle, theta, radius = part.radius) => {
        const y = radius * Math.cos(theta), z = radius * Math.sin(theta);
        return [cx + axle * c + z * s, cy0 + y, cz - axle * s + z * c];
      };
      const left = [], right = [];
      for (let i = 0; i < segments; i++) {
        const theta = i * Math.PI * 2 / segments;
        left.push(point(-part.width / 2, theta)); right.push(point(part.width / 2, theta));
      }
      const faces = [
        {vertices: left.slice().reverse(), color: palette.side},
        {vertices: right, color: palette.side}
      ];
      for (let i = 0; i < segments; i++) {
        const next = (i + 1) % segments;
        faces.push({vertices: [left[i], right[i], right[next], left[next]],
          color: i % 4 === 0 ? palette.treadLight : palette.tread});
      }
      drawFaces(faces, palette.edge);
      // Sidewall accent ring and hub make the wheel plane legible as it steers.
      const nearAxle = [-part.width / 2 - .001, part.width / 2 + .001]
        .sort((a, b) => project(point(a, 0, 0)).depth - project(point(b, 0, 0)).depth)[0];
      for (const axle of [nearAxle]) {
        const ring = []
        for (let i = 0; i <= segments; i++) ring.push(project(point(axle, i * Math.PI * 2 / segments, part.radius * .78)));
        if (ring.every(Boolean)) {
          ctx.beginPath(); ctx.moveTo(ring[0].x, ring[0].y);
          ring.slice(1).forEach(p => ctx.lineTo(p.x, p.y)); ctx.closePath();
          ctx.strokeStyle = palette.rim; ctx.lineWidth = Math.max(1.5 * ratio, 2); ctx.stroke();
          const hub = project(point(axle, 0, 0));
          if (hub) { ctx.fillStyle = palette.hub; ctx.fillRect(hub.x - 2 * ratio, hub.y - 2 * ratio, 4 * ratio, 4 * ratio); }
        }
      }
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
    // Дальние знаки — ДО модели робота, близкие к наблюдателю — ПОСЛЕ:
    // простоеDepth-сортировка относительно центра робота без общего z-буфера.
    const signLayer = mode === 'combined' && now < signDeadline
      ? signMarkers.map(marker => {
          const screen = project(visionSignGeometry(marker).center);
          return screen ? {marker, depth: screen.depth} : null;
        }).filter(Boolean).sort((a, b) => b.depth - a.depth)
      : [];
    const robotDepth = project([0, 0, 0]);
    const drawSignLayer = (near) => signLayer
      .filter(item => robotDepth && item.depth < robotDepth.depth === near)
      .forEach(item => drawVisionSign(item.marker));
    if (signLayer.length) drawSignLayer(false);
    if (mode === 'combined') {
      const live = root.RCRobotModel && typeof root.RCRobotModel.state === 'function'
        ? root.RCRobotModel.state() : null;
      const geometry = root.RCRobotModel && typeof root.RCRobotModel.geometry === 'function'
        ? root.RCRobotModel.geometry() : {wheelbase: 0.50, track: 0.38};
      const palettes = {
        shell: {top:'#171b1f', side:'#090b0d', sideLight:'#111519', bottom:'#050607', edge:'#30363b'},
        roof: {top:'#22272b', side:'#171b1f', sideLight:'#1b2024', bottom:'#111416', edge:'#353b40'},
        wheel: {side:'#15181b', tread:'#202428', treadLight:'#303438', rim:'#b04d35', hub:'#596168', edge:'#070809'},
        wheelUnknown: {side:'#34383b', tread:'#45494c', treadLight:'#55595c', rim:'#777b7e', hub:'#888d90', edge:'#161819'},
        pink: {top:'#ff398c', front:'#e91d72', back:'#b91256', left:'#d61b68', right:'#f52b7f', bottom:'#8e1248'},
        glass: {top:'#20252a', front:'#090d11', back:'#07090b', left:'#151a1e', right:'#1b2024', bottom:'#050607'},
        lens: {top:'#69747b', front:'#252b30', back:'#171c20', left:'#414a50', right:'#59636a', bottom:'#101316'},
        led: {top:'#9aff7c', front:'#38d65d', back:'#158c3d', left:'#23b94b', right:'#55e66a', bottom:'#0b5329'},
        amber: {top:'#ffc14e', front:'#e88b1b', back:'#a94f0a', left:'#c86a13', right:'#f3a226', bottom:'#773807'},
        lidar: {top:'#2a3035', front:'#14181b', back:'#090b0d', left:'#1b2024', right:'#252b30', bottom:'#090b0d'},
        post: {top:'#aeb6bb', front:'#6c757b', back:'#444c51', left:'#596167', right:'#858e94', bottom:'#343a3f'},
        handle: {top:'#d4dadd', front:'#8e989e', back:'#596168', left:'#747e84', right:'#b1babf', bottom:'#51595e'}
      };
      const parts = robotParts(live, geometry, lidarOffsetM).map(part => ({part, depth: project(part.center).depth}));
      parts.sort((a, b) => b.depth - a.depth).forEach(({part}) => {
        if (part.kind === 'wheel') drawWheel(part, part.angle === null ? palettes.wheelUnknown : palettes.wheel);
        else if (part.kind === 'shell' || part.kind === 'roof') drawHull(part, palettes[part.kind]);
        else {
          const palette = part.kind === 'kinectBezel' || part.kind === 'sideBezel' ? palettes.pink
            : part.kind === 'kinectGlass' || part.kind === 'sideLens' ? palettes.glass
            : part.kind === 'kinectLens' ? palettes.lens
            : part.kind === 'kinectLed' ? palettes.led
            : part.kind === 'amber' ? palettes.amber
            : part.kind === 'lidarPost' ? palettes.post
            : part.kind === 'handle' || part.kind === 'handleFoot' ? palettes.handle
            : palettes.lidar;
          drawBox(part, palette);
        }
      });
      if (signLayer.length) drawSignLayer(true);
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
  function setVisionMarkers(markers) {
    signMarkers = Array.isArray(markers)
      ? markers.filter(marker => marker && Number.isFinite(Number(marker.x_m)) &&
          Number.isFinite(Number(marker.y_m))).slice(0, 32)
      : [];
    signDeadline = signMarkers.length ? performance.now() + 1500 : 0;
    if (mode !== 'lidar') draw();
  }
  function bindVision() {
    // vision_panel.js уже опрашивает /api/vision/state; карта и 3D-обзор
    // просто подписываются на его событие, не открывая второй polling.
    root.addEventListener('rc-vision-state', (event) => {
      const payload = event && event.detail;
      setVisionMarkers(payload && payload.field);
    });
    const vision = root.RCVision;
    if (vision && typeof vision.onState === 'function') {
      vision.onState((payload) => setVisionMarkers(payload && payload.field));
    }
  }
  function setMode(next) {
    if (!['lidar', 'kinect', 'combined'].includes(next)) return;
    mode = next; generation++;
    const active = mode !== 'lidar';
    const lidarButton = $('sensor-lidar'), kinectButton = $('sensor-kinect'), combinedButton = $('sensor-combined');
    if (lidarButton) lidarButton.setAttribute('aria-pressed', String(mode === 'lidar'));
    if (kinectButton) kinectButton.setAttribute('aria-pressed', String(mode === 'kinect'));
    if (combinedButton) combinedButton.setAttribute('aria-pressed', String(mode === 'combined'));
    const lidarView = $('sensor-lidar-view'), kinectView = $('sensor-kinect-view'), lidarMeta = $('lidar-meta');
    if (lidarView) lidarView.hidden = active;
    if (kinectView) kinectView.hidden = !active;
    if (lidarMeta) lidarMeta.hidden = active;
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
    bindVision();
    const canvas = $('kinect-3d-canvas');
    if (!canvas) return;
    const lidarButton = $('sensor-lidar'), kinectButton = $('sensor-kinect'), combinedButton = $('sensor-combined');
    if (lidarButton) lidarButton.addEventListener('click', () => setMode('lidar'));
    if (kinectButton) kinectButton.addEventListener('click', () => setMode('kinect'));
    if (combinedButton) combinedButton.addEventListener('click', () => setMode('combined'));
    const zoomIn = $('kinect-zoom-in'), zoomOut = $('kinect-zoom-out'), reset = $('kinect-reset-view');
    if (zoomIn) zoomIn.addEventListener('click', () => { zoom = clamp(zoom * 1.25, .4, 5); draw(); });
    if (zoomOut) zoomOut.addEventListener('click', () => { zoom = clamp(zoom / 1.25, .4, 5); draw(); });
    if (reset) reset.addEventListener('click', () => { yaw = .25; pitch = -.15; zoom = 1; draw(); });
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
  root.RCKinectView = {active: () => mode !== 'lidar', mode: () => mode, setMode, draw,
    setLidarScan, setVisionMarkers};
  if (root.document.readyState === 'loading') root.document.addEventListener('DOMContentLoaded', bind);
  else bind();
})(typeof window !== 'undefined' ? window : globalThis);
