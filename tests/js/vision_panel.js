'use strict';
/* Панель YOLO без браузера: проверяем подмену URL кадров, лампы светофора,
 * список знаков и отрисовку поля. Запускается из tests/test_vision.py через node. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const lamps = {};
for (const name of ['red', 'yellow', 'green']) {
  const classes = new Set();
  lamps[name] = {
    classList: {
      toggle(cls, value) { if (value) classes.add(cls); else classes.delete(cls); },
      contains: (cls) => classes.has(cls),
    },
    style: {setProperty(key, value) { this[key] = value; }},
    title: '',
  };
}
const ctx2dCalls = [];
const ctx2d = new Proxy({}, {
  get(_target, prop) {
    if (prop === 'canvas') return {};
    return (...args) => { ctx2dCalls.push([prop, ...args]); };
  },
  set() { return true; },
});
function element(id) {
  return {
    id, textContent: '', innerHTML: '', title: '', dataset: {}, checked: true,
    style: {setProperty() {}},
    classList: {toggle() {}},
    width: 260, height: 150,
    getContext: () => ctx2d,
    getBoundingClientRect: () => ({width: 260, height: 150}),
    addEventListener(type, handler) { this['on' + type] = handler; },
  };
}
const elements = {};
const ids = ['vision-annot', 'vision-backend', 'vision-signal', 'vision-cameras',
  'vision-fps', 'vision-signs', 'vision-field'];
ids.forEach((id) => { elements[id] = element(id); });

const dispatched = [];
const eventListeners = {};
const snapshot = {
  ok: true, enabled: true, backend: 'yolo+heuristic', inferenceMs: 12.5,
  reason: 'модель yolo-stop.pt + встроенный детектор',
  counts: {stop_sign: 2, pedestrian_crossing: 0, speed_bump: 1, traffic_light: 1, cone: 2},
  classes: [
    {name: 'stop_sign', short: 'STOP', color: '#ff3d71'},
    {name: 'speed_bump', short: 'НЕРОВН', color: '#ffb020'},
  ],
  cameras: {0: {active: true, captureAgeS: 0.1, detections: [{}, {}]}, 1: {detections: []},
    2: {detections: [{}]}, 3: {detections: []}},
  trafficLight: {state: 'red', label: 'Красный — стоим', cameraId: 0,
    counts: {red: 900, yellow: 120, green: 20}, action: 'stop'},
  field: [
    {class: 'stop_sign', short: 'STOP', color: '#ff3d71', x_m: 1.4, y_m: 0.3},
    {class: 'traffic_light', short: 'СВЕТОФОР', color: '#a78bfa',
      signal: 'red', x_m: 2.2, y_m: -0.4},
  ],
};

const ctx = {
  Date, Math, Number, String, Array, Object, Set, JSON, console,
  setInterval: () => 1, clearInterval: () => {},
  setTimeout: () => 1, clearTimeout: () => {},
  devicePixelRatio: 1,
  CustomEvent: class { constructor(type, options) { this.type = type; this.detail = options.detail; } },
  document: {
    readyState: 'loading', hidden: false,
    getElementById: (id) => elements[id] || null,
    querySelector: (selector) => {
      const match = /data-signal="(red|yellow|green)"/.exec(selector);
      return match ? lamps[match[1]] : null;
    },
    addEventListener(type, handler) {
      if (type === 'DOMContentLoaded') this.domReady = handler;
    },
    dispatchDomReady() { if (this.domReady) this.domReady(); },
  },
  fetch: () => Promise.resolve({ok: true, json: () => Promise.resolve(snapshot)}),
};
ctx.window = ctx;
ctx.window.addEventListener = (type, handler) => {
  (eventListeners[type] = eventListeners[type] || []).push(handler);
};
ctx.window.dispatchEvent = (event) => {
  dispatched.push(event);
  (eventListeners[event.type] || []).forEach((handler) => handler(event));
};

const bootHandlers = [];
ctx.document.addEventListener = (type, handler) => {
  if (type === 'DOMContentLoaded') bootHandlers.push(handler);
};

vm.runInNewContext(
  fs.readFileSync(path.join(__dirname, '../../slam_gui/vision_panel.js'), 'utf8'), ctx);
let mapPayload = null;
ctx.window.addEventListener('rc-vision-state', (event) => { mapPayload = event.detail; });
bootHandlers.forEach((handler) => handler());

(async () => {
  await new Promise((resolve) => setImmediate(resolve));
  const vision = ctx.RCVision;
  assert(vision, 'RCVision должен регистрироваться');
  assert.equal(vision.getState().backend, 'yolo+heuristic');
  assert.match(vision.frameUrl(0), /api\/vision\/camera\/0\/frame\?t=/);
  assert.match(vision.getState().field.length ? 'field' : '', /field/);

  vision.setOverlay(false);
  assert.match(vision.frameUrl(3), /api\/camera\/3\/frame\?t=/);
  vision.setOverlay(true);
  assert.match(vision.frameUrl(0), /api\/vision\/camera\/0\/frame\?t=/);
  assert.match(vision.frameUrl(3), /api\/camera\/3\/frame\?t=/, 'no annotation yet: show the raw camera');

  assert.equal(elements['vision-signal'].textContent, 'Красный — стоим · CAM 0');
  assert.equal(elements['vision-signal'].dataset.state, 'red');
  assert(lamps.red.classList.contains('is-on'), 'красная лампа горит');
  assert(!lamps.green.classList.contains('is-on'), 'зелёная лампа не активна');
  assert(lamps.yellow.classList.contains('is-seen'), 'жёлтые диоды видны, но не сигнал');
  assert.match(elements['vision-signs'].innerHTML, /STOP/);
  assert.match(elements['vision-signs'].innerHTML, /НЕРОВН/);
  assert.match(elements['vision-backend'].textContent, /YOLO/);
  assert.match(elements['vision-cameras'].textContent, /ПЕРЕД-Л 2/);
  assert(ctx2dCalls.some((call) => call[0] === 'arc'), 'поле должно рисовать отметки');
  assert(dispatched.some((event) => event.type === 'rc-vision-state'),
    'событие rc-vision-state должно доходить до карты');
  assert.equal(mapPayload.field.length, 2, 'карта получает отметки знаков');

  const fail = () => {};
  const image = {onerror: fail};
  vision.loadFrame(image, 0);
  assert.match(image.src, /api\/vision\/camera\/0\/frame/);
  image.onerror(); // annotated route returned 404 after the state snapshot
  assert.match(image.src, /api\/camera\/0\/frame/);
  assert.equal(image.onerror, fail, 'a raw failure goes to the original poller');
  // Closed-course timeout remains UNKNOWN and never lights the green lamp.
  snapshot.trafficLight = {state: 'unknown', label: 'Сигнал не определён', action: 'wait', counts: {},
    closedCourse: {enabled: true, phase: 'waiting', remainingS: 3.0, recommendation: 'wait'}};
  await vision.refresh();
  assert.match(elements['vision-signal'].textContent, /ПОЛИГОН.*3.0 с/);
  snapshot.trafficLight.closedCourse = {enabled: true, phase: 'timeout_advisory',
    recommendation: 'proceed_closed_course', remainingS: null};
  await vision.refresh();
  assert.match(elements['vision-signal'].textContent, /рекомендация проезда.*не зелёный/);
  assert.equal(elements['vision-signal'].dataset.state, 'unknown');
  assert(!lamps.green.classList.contains('is-on'));
  assert.equal(vision.getState().trafficLight.action, 'wait');
  snapshot.trafficLight.state = 'red';
  snapshot.trafficLight.label = 'Красный — стоим';
  await vision.refresh();
  assert(!elements['vision-signal'].textContent.includes('рекомендация проезда'), 'advice cannot override a known colour');
  snapshot.trafficLight.state = 'unknown';
  snapshot.trafficLight.label = 'Сигнал не определён';
  await vision.refresh();
  ctx.fetch = () => Promise.reject(new Error('camera API offline'));
  await vision.refresh();
  assert.equal(vision.getState().trafficLight.action, 'wait');
  assert.equal(vision.getState().trafficLight.closedCourse.recommendation, 'wait');
  assert(!elements['vision-signal'].textContent.includes('рекомендация проезда'));
  assert(!lamps.green.classList.contains('is-on'));
  assert.match(elements['vision-signal'].textContent, /Нет свежих данных/);
  assert.equal(mapPayload.field.length, 0, 'offline does not preserve old field/advice events');
  ctx.fetch = () => Promise.resolve({ok: true, json: () => Promise.resolve(snapshot)});

  // Знаки — только индикация: выключенный ограничитель ничего не обещает даже
  // при подтверждённом красном, а включённый вручную честно об этом пишет.
  snapshot.trafficLight = {state: 'red', label: 'Красный — стоим', cameraId: 0, action: 'stop',
    counts: {red: 900}, guard: {enabled: false, blocked: false, bump: false, speedCapPercent: 100,
      controlsMotors: false, reason: 'ограничения по знакам выключены (RC_GUARD_ENABLED=0)'}};
  await vision.refresh();
  assert.match(elements['vision-signal'].textContent, /Красный — стоим · CAM 0/);
  assert.equal(elements['vision-signal'].dataset.state, 'red');
  assert.doesNotMatch(elements['vision-signal'].textContent, /ТЯГА ЗАБЛОКИРОВАНА|НЕРОВНОСТЬ/,
    'выключенный ограничитель не обещает ограничений');
  // Ограничитель включили вручную — панель снова показывает его состояние.
  snapshot.trafficLight.guard = {enabled: true, blocked: true, blockedS: 3.5, bump: false,
    speedCapPercent: 100, controlsMotors: false};
  await vision.refresh();
  assert.match(elements['vision-signal'].textContent, /ТЯГА ЗАБЛОКИРОВАНА \(RC_GUARD_ENABLED=1\)/);
  snapshot.trafficLight.guard = {enabled: true, blocked: false, bump: true, speedCapPercent: 80,
    slowdown: 0.2, controlsMotors: false};
  await vision.refresh();
  assert.match(elements['vision-signal'].textContent, /НЕРОВНОСТЬ: потолок 80 %/);
  snapshot.trafficLight.guard = {enabled: true, blocked: false, bump: false, speedCapPercent: 100};
  await vision.refresh();
  assert.doesNotMatch(elements['vision-signal'].textContent, /НЕРОВНОСТЬ|ТЯГА/);

  // Выключенный сервис не подменяет кадры: панель остаётся, но рамок нет.
  snapshot.ok = false;
  await vision.refresh();
  await new Promise((resolve) => setImmediate(resolve));
  assert.match(vision.frameUrl(0), /api\/camera\/0\/frame\?t=/);
  assert.equal(elements['vision-signal'].dataset.state, 'unknown');
  assert(!lamps.green.classList.contains('is-on'));
  console.log('vision panel: OK');
})().catch((error) => { console.error(error); process.exitCode = 1; });
