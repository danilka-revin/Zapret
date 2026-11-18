const assert = require('node:assert/strict');
const {decodeFrame, transformLidar, transformKinect, robotParts, orbitCoordinates,
  kinectFrustum, visionSignGeometry, signLocalShape, signPoint,
  visionSignFaces} = require('../../slam_gui/kinect_view.js');
const payload = {available: true, oneFrameOnly: true, notForNavigation: true,
  width: 2, height: 2, stride: 2, fx: 2, fy: 2, cx: 1, cy: 1,
  data: Buffer.from([0xe8, 3, 0, 0, 0xd0, 7, 0xb8, 0x0b]).toString('base64')};
assert.deepEqual(decodeFrame(payload), [[-.5, .5, 1], [-1, -1, 2], [1.5, -1.5, 3]]);
assert.deepEqual(decodeFrame({...payload, data: '!!!!'}), []);
assert.deepEqual(decodeFrame({...payload, notForNavigation: false}), []);
assert.deepEqual(decodeFrame({...payload, width: 161}), []);
const lidarPoint = transformLidar([{a: 0, r: 1}], .28)[0];
assert.ok(Math.abs(lidarPoint[0]) < 1e-9);
assert.equal(lidarPoint[1], .55);
assert.ok(Math.abs(lidarPoint[2] - 1.28) < 1e-9);
assert.equal(lidarPoint[3], 'lidar');
const kinectPoint = transformKinect([[.2, 0, 1]], {z_m: .45, yaw_deg: 0, pitch_deg: 0}, .28)[0];
assert.ok(Math.abs(kinectPoint[0] - .2) < 1e-9);
  assert.equal(kinectPoint[1], .45);
assert.ok(Math.abs(kinectPoint[2] - 1.41) < 1e-9);
assert.equal(kinectPoint[3], 'kinect');
assert.deepEqual(orbitCoordinates([0, 0, 0], .4, -.2), [0, 0, 0], 'robot origin is the orbit pivot');
assert.deepEqual(orbitCoordinates([0, 0, 2], 0, 0), [0, 0, 2], 'no fixed forward offset moves the pivot');
// Зона видимости: усечённая пирамида, дальние углы конечны и расходятся
// ровно на заданные углы/дальность из калибровки.
const frustum = kinectFrustum({z_m: .45, yaw_deg: 0, pitch_deg: 0},
  {hfov_deg: 57, vfov_deg: 43, min_range_m: .45, max_range_m: 4}, .28);
assert.equal(frustum.faces.length, 5, '4 боковые грани + дальняя плоскость');
assert.ok(frustum.rings.length >= 3, 'метки дальности 1/2/3/4 м');
assert.ok(frustum.farRect.every(p => p.every(Number.isFinite)), 'нет NaN в 3D-геометрии');
assert.ok(Math.abs(frustum.far - 4) < 1e-9);
assert.ok(Math.abs(frustum.origin[2] - .41) < 1e-9, 'Kinect на 130 мм впереди лидара');
const farWidth = Math.hypot(frustum.farRect[1][0] - frustum.farRect[0][0],
  frustum.farRect[1][2] - frustum.farRect[0][2]);
assert.ok(Math.abs(farWidth - 2 * 4 * Math.tan(57 * Math.PI / 360)) < 1e-9);
// 3D-знак 1:1: карта (X вперёд, Y влево) → вид (X вправо, Y вверх, Z вперёд).
const signMarker = {x_m: 1.5, y_m: .5, z_m: .9, width_m: .30, height_m: .30,
  thickness_m: .02, shape: 'octagon', class: 'stop_sign', headingDeg: 18.4,
  scale1to1: true, sizeScale: 1, signal: 'red', positionSource: 'camera-estimate'};
const signGeom = visionSignGeometry(signMarker);
assert.deepEqual(signGeom.center, [-.5, .9, 1.5]);
assert.equal(signGeom.width, .30);
assert.equal(signGeom.height, .30);
assert.equal(signGeom.thickness, .02);
assert.equal(signLocalShape(signGeom).length, 8, 'STOP — восьмиугольник');
assert.equal(visionSignFaces(signMarker).faces.length, 10, '8 рёбер + 2 стороны');
const left = signPoint(signGeom, -signGeom.width / 2, 0, 0);
const right = signPoint(signGeom, signGeom.width / 2, 0, 0);
assert.ok(Math.abs(Math.hypot(right[0] - left[0], right[2] - left[2]) - .30) < 1e-9,
  'ширина таблички в 3D равна ровно 0,30 м');
const articulated = robotParts({fresh: true, wheels: [
  {id: 'FL', angle: 30}, {id: 'FR', angle: -15}, {id: 'RL', angle: 5}, {id: 'RR', angle: null}
]}, {wheelbase: .6, track: .4}, .3).filter(part => part.kind === 'wheel');
assert.equal(articulated.length, 4);
assert.ok(Math.abs(articulated.find(w => w.id === 'FL').yaw - Math.PI / 6) < 1e-9);
assert.equal(articulated.find(w => w.id === 'FR').angle, -15);
assert.deepEqual(articulated.find(w => w.id === 'FL').center, [-.2, .145, .3]);
const fullRobot = robotParts(null, {wheelbase: .5, track: .4}, .3);
const shell = fullRobot.find(part => part.kind === 'shell');
assert.equal(Math.max(...shell.footprint.map(point => point[0])), .3);
assert.equal(Math.min(...shell.footprint.map(point => point[0])), -.3);
assert.equal(Math.max(...shell.footprint.map(point => point[1])), .375);
assert.equal(fullRobot.find(part => part.kind === 'lidar').center[1], .55);
assert.equal(fullRobot.find(part => part.kind === 'lidar').center[2], .3);
assert.equal(fullRobot.find(part => part.kind === 'kinectLens').center[1], .45);
assert.equal(fullRobot.find(part => part.kind === 'kinectLens').center[2], .43);
assert.equal(articulated.find(w => w.id === 'RR').angle, null);
const staleRobot = robotParts({fresh: false, wheels: [{id: 'FL', angle: 30}]})
  .find(part => part.kind === 'wheel' && part.id === 'FL');
assert.equal(staleRobot.angle, null);
assert.equal(staleRobot.yaw, 0);
console.log('Kinect/LiDAR transforms and articulated robot geometry: OK');

// Exercise the actual home-panel event handlers without a browser or hardware.
const vm = require('node:vm');
const fs = require('node:fs');
const source = fs.readFileSync(require('node:path').join(__dirname, '../../slam_gui/kinect_view.js'), 'utf8');
let now = 100, interval, fetches = 0, lastUrl = '', lastInit = null, rectangles = [], polygons = 0;
let lidarRedraws = 0;
const docKeydown = [];
class ClassList {
  constructor() { this.set = new Set(); }
  toggle(name, force) {
    const on = force === undefined ? !this.set.has(name) : !!force;
    if (on) this.set.add(name); else this.set.delete(name);
    return on;
  }
  contains(name) { return this.set.has(name); }
}
class Element {
  constructor() {
    this.listeners = {}; this.hidden = false; this.attrs = {}; this.textContent = '';
    this.title = ''; this.classList = new ClassList(); this.label = null;
  }
  addEventListener(name, fn) { this.listeners[name] = fn; }
  setAttribute(name, value) { this.attrs[name] = value; }
  querySelector(sel) { return sel === '.sensor-fs-label' ? this.label : null; }
  focus() {}
  emit(name, event = {}) {
    if (this.listeners[name]) {
      this.listeners[name]({preventDefault() {}, stopPropagation() {}, ...event});
    }
  }
  getBoundingClientRect() {
    const fs = elements['sensor-overview-panel'] &&
      elements['sensor-overview-panel'].classList.contains('is-fullscreen');
    return fs ? {width: 1920, height: 1080} : {width: 600, height: 400};
  }
  setPointerCapture() {}
  getContext() { return {clearRect() { rectangles = []; polygons = 0; }, beginPath() {}, closePath() { polygons++; }, fill() {}, moveTo() {}, lineTo() {}, stroke() {},
    arc() {}, fillText() {}, fillRect(x, y, w, h) { rectangles.push([x, y, w, h]); },
    set fillStyle(v) {}, set strokeStyle(v) {}, set lineWidth(v) {}, set font(v) {},
    set textAlign(v) {}, set textBaseline(v) {}}; }
}
const ids = ['sensor-overview-panel', 'sensor-lidar', 'sensor-kinect', 'sensor-combined',
  'sensor-lidar-view', 'sensor-kinect-view', 'lidar-meta', 'sensor-fullscreen-toggle',
  'lidar-fullscreen-btn', 'kinect-fullscreen-btn',
  'kinect-3d-status', 'kinect-3d-mode-note', 'kinect-3d-empty', 'kinect-3d-canvas', 'kinect-zoom-in', 'kinect-zoom-out',
  'kinect-reset-view', 'kinect-fov-toggle'];
const elements = Object.fromEntries(ids.map(id => [id, new Element()]));
elements['sensor-fullscreen-toggle'].label = {textContent: 'На полный экран'};
const body = {classList: new ClassList()};
const window = {
  location: {search: '?m=guest%2Btoken'},
  document: {
    readyState: 'complete', hidden: false, body,
    getElementById: id => elements[id],
    addEventListener(name, fn) { if (name === 'keydown') docKeydown.push(fn); },
  },
  RCLidarView: {redrawAll() { lidarRedraws++; }},
  atob, devicePixelRatio: 1, addEventListener() {},
};
const context = {window, URLSearchParams, performance: {now: () => now}, AbortController, clearTimeout, setTimeout,
  setInterval: fn => { interval = fn; return 1; },
  fetch: async (url, init) => {
    fetches++; lastUrl = url; lastInit = init;
    return {ok: true, json: async () => ({...payload, frameId: 1, ageS: .1, maxAgeS: 1.5})};
  },
  console};
vm.runInNewContext(source, context);
(async () => {
  assert.equal(window.RCKinectView.active(), false);
  elements['sensor-kinect'].emit('click');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(window.RCKinectView.active(), true);
  assert.equal(elements['sensor-lidar-view'].hidden, true);
  assert.equal(elements['sensor-kinect-view'].hidden, false);
  assert.equal(elements['kinect-3d-empty'].hidden, true);
  assert.equal(rectangles.length, 3);
  let before = JSON.stringify(rectangles);
  elements['kinect-3d-canvas'].emit('pointerdown', {pointerId: 1, clientX: 10, clientY: 10});
  elements['kinect-3d-canvas'].emit('pointermove', {pointerId: 1, clientX: 60, clientY: 30});
  assert.notEqual(JSON.stringify(rectangles), before, 'drag rotates cloud');
  before = JSON.stringify(rectangles);
  elements['kinect-3d-canvas'].emit('keydown', {key: 'ArrowRight'});
  assert.notEqual(JSON.stringify(rectangles), before, 'arrow key rotates cloud');
  before = JSON.stringify(rectangles);
  elements['kinect-zoom-in'].emit('click');
  assert.notEqual(JSON.stringify(rectangles), before, 'button zooms cloud');
  now = 1700;
  interval();
  assert.equal(elements['kinect-3d-empty'].hidden, false, 'expired frame disappears');
  elements['sensor-lidar'].emit('click');
  assert.equal(elements['sensor-lidar-view'].hidden, false);
  const requests = fetches;
  interval();
  assert.equal(fetches, requests, 'no Kinect polling in lidar mode');
  window.RCKinectView.setLidarScan([{a: 0, r: 2}, {a: Math.PI / 2, r: 1}],
    {connected: true, age_s: .1, lidar_offset_m: .28});
  elements['sensor-combined'].emit('click');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(window.RCKinectView.mode(), 'combined');
  assert.equal(elements['sensor-combined'].attrs['aria-pressed'], 'true');
  assert.equal(elements['kinect-3d-empty'].hidden, true);
  assert.match(elements['kinect-3d-status'].textContent, /Лидар: 2 точки · Kinect:/);
  assert.match(elements['kinect-3d-mode-note'].textContent, /Модель робота · фактические углы колёс \(при свежей телеметрии\) · Лидар z=550 мм · Kinect \+130 мм вперёд, z=450 мм/);
  assert.match(elements['kinect-3d-mode-note'].textContent, /зона Kinect 57°×43°, до 4\.0 м/);
  assert.equal(elements['kinect-fov-toggle'].hidden, false, 'в общем обзоре доступен переключатель зоны');
  assert.equal(elements['kinect-fov-toggle'].attrs['aria-pressed'], 'true');
  assert.ok(rectangles.length > 0, 'combined view renders the live sensor cloud');
  assert.ok(polygons >= 30, 'combined view renders the 3D robot chassis, sensors, and wheels');
  window.RCKinectView.setKinectFov(false);
  const withoutFov = polygons;
  assert.equal(elements['kinect-fov-toggle'].attrs['aria-pressed'], 'false');
  assert.match(elements['kinect-3d-mode-note'].textContent, /зона Kinect выключена/);
  window.RCKinectView.setKinectFov(true);
  assert.ok(polygons > withoutFov, 'пирамида видимости добавляет грани в 3D-обзор');
  const beforeSigns = polygons;
  window.RCKinectView.setVisionMarkers([signMarker]);
  assert.ok(polygons > beforeSigns, '3D-знак добавляет объёмные грани в общий обзор');
  assert.match(elements['kinect-3d-mode-note'].textContent, /знаков: 1 \(1:1\)/);
  window.RCKinectView.setVisionMarkers([{...signMarker, scale1to1: false, sizeScale: 1.2,
    width_m: .36, height_m: .36}]);
  assert.match(elements['kinect-3d-mode-note'].textContent, /знаков: 1 \(масштаб ×1\.20\)/);
  assert.equal(visionSignGeometry({...signMarker, width_m: .36}).width, .36,
    'RC_VISION_SIGN_SCALE меняет физический размер в 3D');

  // Signed guest monitor query parameter is preserved on Kinect preview polls.
  assert.equal(lastUrl, '/api/kinect/preview?m=guest%2Btoken');
  assert.equal(lastInit.credentials, 'same-origin');

  // Fullscreen works for combined 3D, Kinect 3D, and 2D LiDAR.
  assert.equal(window.RCKinectView.isFullscreen(), false);
  elements['kinect-fullscreen-btn'].emit('click');
  assert.equal(window.RCKinectView.isFullscreen(), true);
  assert.equal(elements['sensor-overview-panel'].classList.contains('is-fullscreen'), true);
  assert.equal(body.classList.contains('sensor-fullscreen'), true);
  assert.equal(elements['sensor-fullscreen-toggle'].label.textContent, 'Свернуть');
  assert.equal(elements['sensor-fullscreen-toggle'].attrs['aria-pressed'], 'true');
  assert.equal(elements['lidar-fullscreen-btn'].attrs['aria-pressed'], 'true');
  assert.equal(elements['kinect-fullscreen-btn'].attrs['aria-pressed'], 'true');
  assert.equal(elements['kinect-3d-canvas'].width, 1920, '3D canvas resizes to fullscreen');
  assert.equal(elements['kinect-3d-canvas'].height, 1080);

  // Switch to Kinect 3D while staying in fullscreen, then collapse via header button.
  elements['sensor-kinect'].emit('click');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(window.RCKinectView.mode(), 'kinect');
  assert.equal(window.RCKinectView.isFullscreen(), true);
  elements['sensor-fullscreen-toggle'].emit('click');
  assert.equal(window.RCKinectView.isFullscreen(), false);
  assert.equal(elements['sensor-fullscreen-toggle'].label.textContent, 'На полный экран');
  assert.equal(elements['kinect-3d-canvas'].width, 600, '3D canvas returns to panel size');

  // Expand 2D LiDAR via its corner button and exit with Escape.
  elements['sensor-lidar'].emit('click');
  const redrawsBefore = lidarRedraws;
  elements['lidar-fullscreen-btn'].emit('click');
  assert.equal(window.RCKinectView.mode(), 'lidar');
  assert.equal(window.RCKinectView.isFullscreen(), true);
  assert.ok(lidarRedraws > redrawsBefore, '2D LiDAR redraws immediately on fullscreen');
  let escStopped = false;
  docKeydown.forEach(fn => fn({
    key: 'Escape', preventDefault() {},
    stopImmediatePropagation() { escStopped = true; },
  }));
  assert.equal(escStopped, true);
  assert.equal(window.RCKinectView.isFullscreen(), false);
  assert.equal(body.classList.contains('sensor-fullscreen'), false);
  console.log('Kinect/LiDAR combined 3D interactions, signs, and fullscreen: OK');
})().catch(err => { console.error(err); process.exitCode = 1; });
