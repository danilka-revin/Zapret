/* Отдельный экран навигации (/navigation) — без браузера.
 *
 * Поднимается минимальный DOM той же разметки, что в slam_gui/navigation.html,
 * и выполняются настоящие slam_gui/map_view.js и slam_gui/navigation_screen.js.
 * `fetch` отдаёт заготовленные ответы API карты и навигации.
 *
 * Что закрепляется:
 *   • шаги 1–3 существуют и подсвечивают, где оператор остановился;
 *   • список готовности берёт те же diagnostics.checks, что решают canStart,
 *     и пишет, что именно мешает;
 *   • клик по карте ставит ОДНУ цель (на /map панели навигации больше нет);
 *   • «Здесь стоит робот» берёт позу из снимка карты;
 *   • «Поехать» активна только при canStart + построенном маршруте.
 *
 * Запуск: node tests/js/navigation_screen.js <payload.json>
 */

'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..', '..');
const payloadFile = process.argv[2];
if (!payloadFile) {
  console.error('нужен файл с ответом API');
  process.exit(2);
}
const payload = JSON.parse(fs.readFileSync(payloadFile, 'utf8'));

/* ------------------------------------------------------------------ DOM */
class ClassList {
  constructor() { this.set = new Set(); }
  add(...names) { names.forEach((n) => this.set.add(n)); }
  remove(...names) { names.forEach((n) => this.set.delete(n)); }
  contains(name) { return this.set.has(name); }
  toggle(name, force) {
    const on = force === undefined ? !this.set.has(name) : !!force;
    if (on) this.set.add(name); else this.set.delete(name);
    return on;
  }
}

function makeCtx(el) {
  const target = {
    canvas: el,
    calls: [],
    fillRect: (...args) => { target.calls.push(['fillRect', ...args]); },
    clearRect: () => {},
    drawImage: (...args) => { target.calls.push(['drawImage', ...args]); },
    measureText: () => ({ width: 10 }),
    fillText: (...args) => { target.calls.push(['fillText', ...args]); },
  };
  return new Proxy(target, {
    get(obj, prop) {
      if (prop in obj) return obj[prop];
      return () => {};
    },
    set(obj, prop, value) { obj[prop] = value; return true; },
  });
}

class El {
  constructor(id, tag, attrs) {
    this.id = id || '';
    this.tagName = (tag || 'div').toUpperCase();
    this.textContent = '';
    this.innerHTML = '';
    this.hidden = false;
    this.title = '';
    this.value = '';
    this.checked = false;
    this.disabled = false;
    this.attrs = Object.assign({}, attrs);
    this.style = { setProperty() {}, removeProperty() {}, cssText: '' };
    this.dataset = Object.assign({}, (attrs && attrs.dataset) || {});
    this.classList = new ClassList();
    this.children = [];
    this.parentElement = null;
    this.clientWidth = 900;
    this.clientHeight = 640;
    this.width = 900;
    this.height = 640;
    this.complete = true;
    this.naturalWidth = 300;
    this.events = {};
  }
  addEventListener(name, listener) { (this.events[name] ||= []).push(listener); }
  removeEventListener() {}
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  replaceChildren(...children) {
    this.children = children;
    children.forEach((child) => { child.parentElement = this; });
  }
  querySelector() { return null; }
  querySelectorAll() { return []; }
  getContext() { if (!this._ctx) this._ctx = makeCtx(this); return this._ctx; }
  getBoundingClientRect() { return { width: 900, height: 640, top: 0, left: 0 }; }
  setAttribute(name, value) {
    this.attrs[name] = value;
    if (name.startsWith('data-')) this.dataset[name.slice(5)] = value;
    else this[name] = value;
  }
  getAttribute(name) {
    if (name && name.startsWith('data-')) {
      const value = this.dataset[name.slice(5)];
      return value === undefined ? null : value;
    }
    if (name in this.attrs) return this.attrs[name];
    return this[name] === undefined ? null : this[name];
  }
  focus() {}
  click() { (this.events.click || []).forEach((l) => l({ target: this, preventDefault() {} })); }
  closest() { return null; }
  /* className и classList в DOM синхронны — здесь тоже. */
  set className(value) {
    this._className = String(value || '');
    this.classList.set = new Set(this._className.split(/\s+/).filter(Boolean));
  }
  get className() { return this._className || ''; }
}

/* Весь текст узла, как его прочитает человек (вложенные элементы тоже). */
function textOf(el) {
  if (!el) return '';
  const own = el.textContent || '';
  const children = (el.children || []).map(textOf).join('');
  return own + children;
}

const elements = new Map();
function byId(id) {
  if (!elements.has(id)) {
    elements.set(id, new El(id, /canvas/.test(id) ? 'canvas' : 'div'));
  }
  return elements.get(id);
}

/* Разметка страницы: только то, что реально есть в navigation.html. */
const PAGE_IDS = [
  'map-canvas', 'map-badge', 'map-empty', 'map-zoom-in', 'map-zoom-out', 'map-fit',
  'map-follow', 'map-theme-toggle',
  'navscreen-mapinfo', 'navscreen-posemeta', 'navscreen-goal-note', 'navscreen-check-list',
  'navscreen-checks-state', 'navscreen-blockers', 'navscreen-reason', 'navscreen-route',
  'navscreen-back', 'navscreen-scan',
  'nav-goal-x', 'nav-goal-y', 'nav-select-goal', 'nav-use-robot', 'nav-plan',
  'nav-start', 'nav-resume', 'nav-cancel', 'nav-estop', 'nav-status', 'nav-msg',
  'nav-route-summary', 'nav-safety-note',
];
PAGE_IDS.forEach((id) => byId(id));

const steps = {};
['goal', 'plan', 'drive'].forEach((name) => {
  steps[name] = new El('', 'section');
  steps[name].setAttribute('data-navstep', name);
  steps[name].setAttribute('data-state', 'todo');
});

global.document = {
  readyState: 'complete',
  body: new El('body'),
  documentElement: new El('html'),
  title: '',
  getElementById: byId,
  createElement: (tag) => new El('', tag),
  createDocumentFragment: () => new El('', 'fragment'),
  createTextNode: (text) => ({ textContent: text }),
  querySelector: (selector) => {
    const match = /\[data-navstep="([^"]+)"\]/.exec(String(selector));
    return match ? steps[match[1]] : null;
  },
  querySelectorAll: () => [],
  addEventListener: () => {},
  removeEventListener: () => {},
  hidden: false,
};

class FakeImage {
  constructor() { this.complete = false; this.naturalWidth = 0; this.onload = null; }
  set src(value) { this._src = value; this.complete = true; this.naturalWidth = 300; if (this.onload) this.onload(); }
  get src() { return this._src; }
}

global.window = global;
global.self = global;
global.Image = FakeImage;
global.devicePixelRatio = 1;
const confirmations = [];
global.confirm = (message) => { confirmations.push(String(message)); return true; };
global.addEventListener = () => {};
global.removeEventListener = () => {};
global.dispatchEvent = () => true;
global.requestAnimationFrame = () => 0;
global.cancelAnimationFrame = () => {};
global.matchMedia = () => ({ matches: false, addEventListener() {}, removeEventListener() {} });
global.MutationObserver = class { observe() {} disconnect() {} };
global.localStorage = {
  store: {},
  getItem(key) { return Object.prototype.hasOwnProperty.call(this.store, key) ? this.store[key] : null; },
  setItem(key, value) { this.store[key] = String(value); },
  removeItem(key) { delete this.store[key]; },
};
global.location = { search: '' };

/* --------------------------------------------------------------- ответы API */
const calls = [];
const renderErrors = [];
console.error = (...args) => { renderErrors.push(args.map((a) => String(a)).join(' ')); };

function reply(body, ok) {
  return { ok: ok !== false, status: ok === false ? 500 : 200, json: async () => body };
}

const DEFAULT_STATUS = {
  ok: true, phase: 'idle', message: 'Ожидание цели', canStart: false, canResume: false,
  enabled: false, reason: 'Автономная тяга выключена.', plan: null,
  diagnostics: { checks: { enabled: false, speedCalibrated: true, mapReady: true } },
};

global.fetch = async (url, options) => {
  const text = String(url);
  const method = (options && options.method) || 'GET';
  const body = options && options.body ? JSON.parse(options.body) : null;
  calls.push({ url: text, method, body, headers: options && options.headers });
  if (/api\/navigation\/status/.test(text)) return reply(payload.navigationStatus || DEFAULT_STATUS);
  if (/api\/navigation\/plan/.test(text)) return reply(payload.navigationPlanReply || { ok: false, error: 'нет заготовки' });
  if (/api\/navigation\/start/.test(text)) return reply(payload.navigationStartReply || { ok: true, status: { phase: 'starting' } });
  if (/api\/navigation\/estop/.test(text)) return reply({ ok: true });
  return reply(payload.snapshot || { ok: false });
};

const originalSetTimeout = global.setTimeout;
global.setTimeout = (cb) => originalSetTimeout(cb, 0);
global.setInterval = () => 0;
global.clearInterval = () => {};

/* ------------------------------------------------------------------ пуск */
for (const file of ['robot_model.js', 'map_view.js', 'navigation_screen.js']) {
  const full = path.join(ROOT, 'slam_gui', file);
  vm.runInThisContext(fs.readFileSync(full, 'utf8'), { filename: full });
}

const sleep = (ms) => new Promise((resolve) => originalSetTimeout(resolve, ms));

(async function run() {
  global.RCMapView.init();
  await sleep(30);
  global.RCNavigationScreen.init();
  await sleep(30);

  const state = () => ({
    goal: steps.goal.getAttribute('data-state'),
    plan: steps.plan.getAttribute('data-state'),
    drive: steps.drive.getAttribute('data-state'),
  });

  const initial = {
    steps: state(),
    reason: byId('navscreen-reason').textContent,
    checks: byId('navscreen-check-list').children.map((row) => ({
      ok: row.classList.contains('is-ok'),
      text: textOf(row),
    })),
    checksState: byId('navscreen-checks-state').textContent,
    blockers: byId('navscreen-blockers').textContent,
    startDisabled: byId('nav-start').disabled,
    mapinfo: byId('navscreen-mapinfo').textContent,
    posemeta: byId('navscreen-posemeta').textContent,
  };

  // Клик по карте: точка (1 м; 0) при карте 30×30 м и холсте 900×640.
  const canvas = byId('map-canvas');
  const rect = canvas.getBoundingClientRect();
  const before = calls.filter((call) => /navigation\/plan/.test(call.url)).length;
  (canvas.events.click || []).forEach((listener) => listener({ clientX: 469.5, clientY: 320 }));
  await sleep(30);
  const afterClick = {
    goalX: byId('nav-goal-x').value,
    goalY: byId('nav-goal-y').value,
    note: byId('navscreen-goal-note').textContent,
    steps: state(),
    reason: byId('navscreen-reason').textContent,
    planCalls: calls.filter((call) => /navigation\/plan/.test(call.url)).length - before,
    summary: byId('nav-route-summary').textContent,
    startDisabled: byId('nav-start').disabled,
  };

  // Шаг 2: явные координаты -> «Построить маршрут» -> «Поехать» открывается
  // только по canStart от сервера.
  for (const [id, value] of [['nav-goal-x', '2'], ['nav-goal-y', '0']]) {
    const input = byId(id);
    input.value = value;
    (input.events.input || []).forEach((listener) => listener({ target: input }));
  }
  byId('nav-plan').click();
  await sleep(30);
  const afterPlan = {
    steps: state(),
    summary: byId('nav-route-summary').textContent,
    startDisabled: byId('nav-start').disabled,
    reason: byId('navscreen-reason').textContent,
    planBody: (calls.filter((call) => /navigation\/plan/.test(call.url)).pop() || {}).body,
  };

  // «Здесь стоит робот»: цель из позы снимка карты.
  byId('nav-goal-x').value = '';
  byId('nav-goal-y').value = '';
  byId('nav-use-robot').click();
  await sleep(10);
  const afterRobot = {
    goalX: byId('nav-goal-x').value,
    goalY: byId('nav-goal-y').value,
    note: byId('navscreen-goal-note').textContent,
  };

  // E-STOP остаётся доступен оператору всегда.
  byId('nav-estop').click();
  await sleep(10);
  const estop = {
    calls: calls.filter((call) => /navigation\/estop/.test(call.url)).length,
    message: byId('nav-msg').textContent,
  };

  console.log(JSON.stringify({ initial, afterClick, afterPlan, afterRobot, estop,
                               confirmations, renderErrors }));
  process.exit(0);
})().catch((error) => {
  console.error(error && error.stack ? error.stack : String(error));
  process.exit(1);
});
