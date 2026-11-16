/* Экран /delivery без браузера: настоящий delivery.js + подменённый fetch.
 * Запуск: node tests/js/delivery_screen.js <payload.json>
 */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const ROOT = path.resolve(__dirname, '..', '..');
const payload = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const html = fs.readFileSync(path.join(ROOT, 'slam_gui', 'delivery.html'), 'utf8');

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
    this.attrs = {}; this.dataset = {}; this.classList = new ClassList(); this.children = [];
    this.clientWidth = 900; this.clientHeight = 600; this.width = 900; this.height = 600;
    this.events = {}; this.style = {};
  }
  addEventListener(name, listener) { (this.events[name] ||= []).push(listener); }
  removeEventListener() {}
  dispatch(name, event) { (this.events[name] || []).forEach((listener) => listener(Object.assign({ target: this, preventDefault() {} }, event))); }
  click() { this.dispatch('click'); }
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
  elements.set(id, new El(id, /canvas/.test(id) ? 'canvas' : (/select/.test(id) ? 'select' : (/checkbox/.test(id) ? 'input' : 'div'))));
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
const calls = [];
global.fetch = async (url, init) => {
  calls.push({ url, init });
  let body;
  if (url === '/api/delivery/map') body = payload.map;
  else if (url === '/api/delivery/status') body = payload.status;
  else if (url === '/api/delivery/plan') body = payload.plan;
  else if (url === '/api/delivery/simulate') body = payload.simulate;
  else if (url === '/api/delivery/setup') body = payload.setup || payload.plan;
  else if (url === '/api/delivery/reset') body = payload.reset || { ok: true, status: payload.status };
  else body = { ok: false, error: 'unknown route' };
  return { ok: body.ok !== false, status: body.ok === false ? 400 : 200, json: async () => body };
};

(async () => {
  vm.runInThisContext(fs.readFileSync(path.join(ROOT, 'slam_gui', 'delivery.js'), 'utf8'), { filename: 'delivery.js' });
  await global.RCDelivery.init();
  const goal = byId('delivery-goal'); goal.value = 'ntc_delivery'; goal.dispatch('change');
  const ack = byId('delivery-ack'); ack.checked = true; ack.dispatch('change');
  byId('delivery-x').value = '22'; byId('delivery-y').value = '140'; byId('delivery-yaw').value = '80';
  byId('delivery-x').dispatch('change');
  byId('delivery-plan').click();
  await new Promise((resolve) => setImmediate(resolve));
  byId('delivery-simulate').click();
  await new Promise((resolve) => setImmediate(resolve));
  const result = {
    calls,
    goalOptions: goal.children.length,
    maneuvers: byId('delivery-maneuvers').children.length,
    events: byId('delivery-events').children.length,
    simulateDisabled: byId('delivery-simulate').disabled,
    start: global.RCDelivery.state.start,
    planDistance: global.RCDelivery.state.plan && global.RCDelivery.state.plan.totalDistanceUnits,
    simulationOk: global.RCDelivery.state.simulation && global.RCDelivery.state.simulation.ok,
    safetyText: byId('delivery-safety').textContent
  };
  process.stdout.write(JSON.stringify(result));
})().catch((error) => { console.error(error && error.stack || error); process.exit(1); });
