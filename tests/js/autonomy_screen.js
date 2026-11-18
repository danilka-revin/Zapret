/* Экран /autonomy без браузера: настоящий autonomy.js + подменённый fetch.
 * Запуск: node tests/js/autonomy_screen.js <payload.json>
 */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const ROOT = path.resolve(__dirname, '..', '..');
const payload = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const html = fs.readFileSync(path.join(ROOT, 'slam_gui', 'autonomy.html'), 'utf8');

class ClassList {
  constructor() { this.set = new Set(); }
  add(...names) { names.forEach((n) => this.set.add(n)); }
  remove(...names) { names.forEach((n) => this.set.delete(n)); }
  contains(name) { return this.set.has(name); }
  toggle(name, force) { const on = force === undefined ? !this.set.has(name) : !!force; if (on) this.set.add(name); else this.set.delete(name); return on; }
}
function makeCtx() {
  return new Proxy({ calls: [] }, {
    get(obj, prop) { if (prop in obj) return obj[prop]; return (...args) => { obj.calls.push([prop, ...args]); }; },
    set(obj, prop, value) { obj[prop] = value; return true; }
  });
}
class El {
  constructor(id, tag) {
    this.id = id || ''; this.tagName = (tag || 'div').toUpperCase();
    this.textContent = ''; this.hidden = false; this.value = ''; this.checked = false; this.disabled = false;
    this.src = ''; this.attrs = {}; this.dataset = {}; this.classList = new ClassList(); this.children = [];
    this.clientWidth = 900; this.clientHeight = 600; this.width = 900; this.height = 600;
    this.events = {}; this.style = {};
  }
  get textContent() { return this._textContent + (this.children || []).map((c) => c.textContent).join(''); }
  set textContent(value) { this._textContent = String(value); this.children = []; }
  addEventListener(name, listener) { (this.events[name] ||= []).push(listener); }
  removeEventListener() {}
  dispatch(name, event) { (this.events[name] || []).forEach((listener) => listener(Object.assign({ target: this, preventDefault() {} }, event))); }
  click() { if (!this.disabled) this.dispatch('click'); }
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  replaceChildren(...children) { this.children = children; children.forEach((child) => { child.parentElement = this; }); }
  setAttribute(name, value) { this.attrs[name] = String(value); if (name.startsWith('data-')) this.dataset[name.slice(5)] = String(value); }
  getAttribute(name) { return this.attrs[name] === undefined ? null : this.attrs[name]; }
  getContext() { if (!this._ctx) this._ctx = makeCtx(); return this._ctx; }
  getBoundingClientRect() { return { left: 0, top: 0, width: 900, height: 600 }; }
  querySelector() { return null; }
  querySelectorAll() { return []; }
  focus() {}
}
const elements = new Map();
for (const match of html.matchAll(/id="([^"]+)"/g)) {
  const id = match[1];
  const tag = /canvas/.test(id) ? 'canvas' : (/select/.test(id) ? 'select' : (/input|checkbox/.test(id) ? 'input' : 'div'));
  elements.set(id, new El(id, tag));
}
function byId(id) { if (!elements.has(id)) elements.set(id, new El(id)); return elements.get(id); }
const documentElement = new El('html'); documentElement.setAttribute('data-map-theme', 'dark');
global.document = {
  readyState: 'complete', documentElement, body: new El('body'), title: '',
  getElementById: byId, createElement: (tag) => new El('', tag), createTextNode: (text) => ({ textContent: text }),
  querySelector: () => null, querySelectorAll: () => [], addEventListener: () => {}, removeEventListener: () => {}
};
global.window = global; global.self = global;
global.localStorage = { store: {}, getItem(k) { return this.store[k] || null; }, setItem(k, v) { this.store[k] = String(v); } };
global.Image = class { constructor() { this.complete = false; } set src(v) { this._src = v; } get src() { return this._src; } };
global.devicePixelRatio = 1;
global.performance = { now: () => 0 };
global.requestAnimationFrame = () => 0;
global.cancelAnimationFrame = () => {};
global.addEventListener = () => {}; global.removeEventListener = () => {};
global.setInterval = () => 0; // опрос статуса/камер в тесте не нужен
global.clearInterval = () => {};

let started = false;
let stopped = false;
function statusBody() {
  return Object.assign({}, payload.status, {
    active: started && !stopped,
    phase: stopped ? 'cancelled' : started ? 'following' : 'idle',
    canStart: payload.status.canStart && !started && !stopped,
    csrfToken: 'test-token'
  });
}
const calls = [];
global.fetch = async (url, init) => {
  const path = String(url);
  calls.push({ url: path, init });
  let body;
  if (/api\/autonomy\/status$/.test(path)) body = statusBody();
  else if (/api\/autonomy\/start$/.test(path)) { started = true; body = { ok: true, status: statusBody() }; }
  else if (/api\/autonomy\/stop$/.test(path)) { stopped = true; body = { ok: true, status: statusBody() }; }
  else if (/api\/autonomy\/simulate$/.test(path)) body = payload.simulate;
  else if (/api\/(control\/)?estop$/.test(path)) body = { ok: true, driveStopped: true };
  else body = { ok: false, error: 'unknown route' };
  return { ok: body.ok !== false, status: body.ok === false ? 400 : 200, json: async () => body };
};

async function flush(times = 8) {
  for (let i = 0; i < times; i += 1) await new Promise((resolve) => setImmediate(resolve));
}

(async () => {
  vm.runInThisContext(fs.readFileSync(path.join(ROOT, 'slam_gui', 'autonomy.js'), 'utf8'), { filename: 'autonomy.js' });
  await global.RCAutonomy.init();
  await flush();
  const start = byId('aut-start');
  const stop = byId('aut-stop');
  const blind = byId('aut-blind');
  const checkLines = byId('aut-checks').children;
  const initial = {
    badChecks: checkLines.filter((line) => line.className.endsWith('is-bad')).map((line) => line.textContent),
    geometryClass: checkLines.find((line) => /геометрия/.test(line.textContent)).className,
    canvasEmptyHidden: byId('aut-canvas-empty').hidden,
    obstacleDistance: byId('aut-obstacle-distance').textContent,
    lidarPoints: byId('aut-lidar-points').textContent,
    pathMeta: byId('aut-path-meta').textContent,
  };
  const beforeBlind = start.disabled;
  blind.checked = true;
  blind.dispatch('change');
  const afterBlind = start.disabled;

  start.click(); await flush();
  stop.click(); await flush();
  byId('aut-scenario').value = payload.simulate.scenario || 'bump';
  byId('aut-simulate').click(); await flush();
  byId('aut-estop').click(); await flush();

  global.RCAutonomy.renderLive(Object.assign({}, global.RCAutonomy._state.status, { phase: 'obstacle_approach' }));
  const approachPhase = byId('aut-phase').textContent;
  global.RCAutonomy.renderLive(global.RCAutonomy._state.status);
  const find = (needle) => calls.filter((call) => call.url.indexOf(needle) !== -1);
  const cam = byId('aut-cam-0'), cameraSrc = cam.src;
  cam.onerror();  // no fresh annotation for this input
  const result = {
    calls,
    beforeBlind, afterBlind, initial, approachPhase, simSummary: byId('aut-sim-summary').textContent,
    checks: byId('aut-checks').children.length,
    warnings: byId('aut-warnings').children.length,
    events: byId('aut-events').children.length,
    simEvents: byId('aut-sim-events').children.length,
    phase: byId('aut-phase').textContent,
    distance: byId('aut-distance').textContent,
    badge: byId('aut-badge').textContent,
    csrf: global.RCAutonomy._state.csrf,
    cameraSrc, fallbackSrc: cam.src,
    canvasCalls: byId('aut-canvas').getContext().calls.length,
    startCalls: find('/start'),
    stopCalls: find('/stop'),
    simulateCalls: find('/simulate'),
    estopCalls: find('estop')
  };
  process.stdout.write(JSON.stringify(result));
})().catch((error) => { console.error(error && error.stack || error); process.exit(1); });
