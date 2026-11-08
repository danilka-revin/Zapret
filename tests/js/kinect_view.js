const assert = require('node:assert/strict');
const {decodeFrame, transformLidar, transformKinect, robotParts} = require('../../slam_gui/kinect_view.js');
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
let now = 100, interval, fetches = 0, rectangles = [], polygons = 0;
class Element {
  constructor() { this.listeners = {}; this.hidden = false; this.attrs = {}; this.textContent = ''; }
  addEventListener(name, fn) { this.listeners[name] = fn; }
  setAttribute(name, value) { this.attrs[name] = value; }
  emit(name, event = {}) { if (this.listeners[name]) this.listeners[name]({preventDefault() {}, ...event}); }
  getBoundingClientRect() { return {width: 600, height: 400}; }
  setPointerCapture() {}
  getContext() { return {clearRect() { rectangles = []; polygons = 0; }, beginPath() {}, closePath() { polygons++; }, fill() {}, moveTo() {}, lineTo() {}, stroke() {},
    fillRect(x, y, w, h) { rectangles.push([x, y, w, h]); }, set fillStyle(v) {}, set strokeStyle(v) {}, set lineWidth(v) {}}; }
}
const ids = ['sensor-lidar', 'sensor-kinect', 'sensor-combined', 'sensor-lidar-view', 'sensor-kinect-view', 'lidar-meta',
  'kinect-3d-status', 'kinect-3d-mode-note', 'kinect-3d-empty', 'kinect-3d-canvas', 'kinect-zoom-in', 'kinect-zoom-out', 'kinect-reset-view'];
const elements = Object.fromEntries(ids.map(id => [id, new Element()]));
const window = {
  document: {readyState: 'complete', hidden: false, getElementById: id => elements[id], addEventListener() {}},
  atob, devicePixelRatio: 1, addEventListener() {},
};
const context = {window, performance: {now: () => now}, AbortController, clearTimeout, setTimeout,
  setInterval: fn => { interval = fn; return 1; },
  fetch: async () => { fetches++; return {ok: true, json: async () => ({...payload, frameId: 1, ageS: .1, maxAgeS: 1.5})}; },
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
  assert.ok(rectangles.length > 0, 'combined view renders the live sensor cloud');
  assert.ok(polygons >= 30, 'combined view renders the 3D robot chassis, sensors, and wheels');
  console.log('Kinect/LiDAR combined 3D interactions: OK');
})().catch(err => { console.error(err); process.exitCode = 1; });
