/* DOM-логика панели «Автообследование» и мастера калибровки без браузера.
 * Тот же приём, что в camera_view.js: минимальный document/window в vm,
 * детерминированный fetch. Проверяет реальный slam_gui/explore.js, а не копию.
 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

const elements = new Map();
const fetches = [];
const documentEvents = {};

class Element {
  constructor(id) {
    this.id = id || '';
    this.tagName = (id || '').toUpperCase();
    this.events = {};
    this.children = [];
    this.dataset = {};
    this.attrs = {};
    this.style = {};
    this.classes = new Set();
    this._text = '';
    this.disabled = false;
    this.checked = false;
    this.value = '';
    this.hidden = false;
    this.classList = {
      contains: (c) => this.classes.has(c),
      toggle: (c, on) => { if (on) this.classes.add(c); else this.classes.delete(c); },
      add: (c) => this.classes.add(c),
      remove: (c) => this.classes.delete(c),
    };
  }
  addEventListener(name, fn) { (this.events[name] ||= []).push(fn); }
  fire(name, event = {}) {
    (this.events[name] || []).forEach((fn) => fn({
      target: this, preventDefault() {}, stopPropagation() {}, ...event,
    }));
  }
  appendChild(child) { this.children.push(child); return child; }
  get childNodes() { return this.children; }
  setAttribute(k, v) { this.attrs[k] = v; }
  get textContent() {
    return this._text + this.children.map((c) => c.textContent).join('');
  }
  set textContent(value) { this._text = String(value); this.children = []; }
}
function get(id) {
  if (!elements.has(id)) elements.set(id, new Element(id));
  return elements.get(id);
}

const status = {
  ok: true, phase: 'idle', message: 'Обследование не запущено',
  coveragePct: 12.5, steps: 2, maxSteps: 40, frontiers: 3,
  kinectFresh: false, enabled: false, navigationEnabled: false, active: false,
  canStart: false,
  blockers: ['Автообследование выключено', 'Нет свежего кадра Kinect 360'],
  warnings: ['Геометрия не откалибрована'], csrfToken: 'csrf-123',
};
const layout = {
  ok: true, calibrated: false, cameraCount: 4, duplicatePorts: [],
  cameras: [
    { role: 'front_left', yaw_deg: -25 }, { role: 'rear_left', yaw_deg: -135 },
    { role: 'front_right', yaw_deg: 25 }, { role: 'rear_right', yaw_deg: 135 },
  ],
};
const calibration = {
  ok: true, calibrated: false, csrfToken: 'csrf-123',
  hint: 'Замерьте углы', fieldCount: 2,
  groups: [
    { id: 'kinect', title: 'Kinect 360', fields: [
      { key: 'RC_KINECT_HFOV_DEG', label: 'Угол', unit: '°', value: 57, min: 10, max: 170, step: 0.5 },
      { key: 'RC_KINECT_X_M', label: 'X', unit: 'м', value: 0.35, min: -2, max: 2, step: 0.01 },
    ] },
    { id: 'front_left', title: 'Передняя левая', port: '/dev/video0', fields: [
      { key: 'RC_CAMERA_FL_YAW_DEG', label: 'Угол', unit: '°', value: -25, min: -360, max: 360, step: 0.5 },
    ] },
  ],
};
const replies = {
  'api/exploration/status': status,
  'api/sensors/layout': layout,
  'api/exploration/landmarks?limit=3': { ok: true, landmarks: [] },
  'api/calibration/geometry': calibration,
  'api/exploration/start': { ok: true, status },
  'api/exploration/stop': { ok: true, status },
};

function makeResponse(payload) {
  return Promise.resolve({ ok: true, status: 200, json: () => Promise.resolve(payload) });
}
function fetchStub(url, options = {}) {
  fetches.push({ url, options });
  const clean = String(url).split('?')[0] === 'api/exploration/landmarks'
    ? 'api/exploration/landmarks?limit=3' : url;
  if (options.method === 'POST' && url === 'api/calibration/geometry') {
    return makeResponse({ ok: true, savedKeys: Object.keys(JSON.parse(options.body).values), calibrated: true });
  }
  return makeResponse(replies[clean] || { ok: true });
}

const context = {
  console, Math, Date, JSON, Object, Array, Number, String, Promise, Error,
  encodeURIComponent, setImmediate,
  fetch: fetchStub,
  setInterval: () => 7,
  clearInterval: () => {},
  document: {
    hidden: false, readyState: 'loading',
    getElementById: get,
    createElement: (tag) => new Element(tag),
    addEventListener: (name, fn) => { (documentEvents[name] ||= []).push(fn); },
  },
};
context.window = context;
vm.createContext(context);
vm.runInContext(
  fs.readFileSync(path.join(__dirname, '../../slam_gui/explore.js'), 'utf8'), context);
const ui = context.RCExplore;
assert.ok(ui && ui.render && ui.saveCalibration, 'explore.js экспортирует RCExplore');

async function settle() {
  for (let i = 0; i < 6; i += 1) await new Promise((resolve) => setImmediate(resolve));
}

(async () => {
  // init навешивает обработчики и делает все первичные запросы.
  (documentEvents.DOMContentLoaded || []).forEach((fn) => fn());
  await settle();
  const urls = fetches.map((entry) => entry.url);
  for (const expected of ['api/exploration/status', 'api/sensors/layout',
    'api/exploration/landmarks?limit=3', 'api/calibration/geometry']) {
    assert.ok(urls.includes(expected), 'первичный запрос ' + expected);
  }
  assert.equal(get('explore-start').disabled, true, 'пуск заблокирован чек-листом');
  assert.equal(get('explore-status').dataset.phase, 'idle');
  assert.match(get('explore-coverage').textContent, /12\.5%/);
  assert.equal(get('explore-blockers').children.length, 3, 'заголовок + 2 причины');
  assert.match(get('explore-calibration').textContent, /Калибровка/);
  assert.equal(get('explore-cal-state').textContent, 'требуется замер');
  assert.equal(get('explore-cal-fields').children.length, 2, 'две группы полей');
  const inputs = get('explore-cal-fields').children.flatMap((group) =>
    group.children
      .filter((child) => child.className === 'explore-cal-field')
      .flatMap((label) => label.children.filter((child) => child.type === 'number')));
  assert.equal(inputs.length, 3);
  assert.equal(inputs[0].dataset.key, 'RC_KINECT_HFOV_DEG');
  assert.equal(inputs[0].value, '57');

  // Сохранение уходит с маркером calibration и CSRF-токеном из статуса.
  const before = fetches.length;
  await ui.saveCalibration({ preventDefault() {} });
  const post = fetches.slice(before).find((entry) => entry.options.method === 'POST');
  assert.ok(post, 'POST сохранения геометрии');
  assert.equal(post.options.headers['X-Robot-Control'], 'calibration');
  assert.equal(post.options.headers['X-CSRF-Token'], 'csrf-123');
  assert.equal(Object.keys(JSON.parse(post.options.body).values).length, 3);
  assert.match(get('explore-cal-message').textContent, /Сохранено полей: 3/);

  // Пуск без подтверждения слепой зоны не отправляет команду.
  const beforeStart = fetches.length;
  await ui.start();
  assert.equal(fetches.length, beforeStart);
  assert.match(get('explore-message').textContent, /ближней зоны/i);
  get('explore-blind').checked = true;
  await ui.start();
  const startPost = fetches.slice(beforeStart).find((entry) => entry.url === 'api/exploration/start');
  assert.ok(startPost && startPost.options.headers['X-Robot-Control'] === 'exploration');
  assert.equal(JSON.parse(startPost.options.body).blindZoneChecked, true);

  await ui.stop();
  assert.ok(fetches.some((entry) => entry.url === 'api/exploration/stop'));
  ui.stopPolling();
  console.log('explore.js DOM: ok');
})().catch((err) => { console.error(err); process.exit(1); });
