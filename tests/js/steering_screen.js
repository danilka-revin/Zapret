/* Проверка руления на бортовом экране — без браузера.
 *
 * Экран робота (slam_gui/robot.js) написан под браузер и ничего не
 * экспортирует, поэтому здесь поднимается минимальный DOM (тот же приём, что в
 * lidar_screen.js), скрипт выполняется как в странице, а результат читается из
 * тех же элементов и запросов, которые видит оператор:
 *   • какие пресеты нарисованы и какие из них нажимаемы;
 *   • ход ползунков (min/max) — он берётся из ответа сервера, а не зашит;
 *   • какие команды ушли в api/kiosk_steer и api/kiosk_drive после нажатий.
 *
 * Запуск (из tests/test_steering_screen.py или вручную):
 *   node tests/js/steering_screen.js <файл-payload.json>
 * На stdout — JSON с тем, что увидели.
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

class El {
  constructor(id, tag) {
    this.id = id || '';
    this.tagName = (tag || 'div').toUpperCase();
    this.textContent = '';
    this.hidden = false;
    this.title = '';
    this.value = '0';
    this.min = '';
    this.max = '';
    this.disabled = false;
    this.style = { setProperty() {}, removeProperty() {}, cssText: '' };
    this.dataset = {};
    this.classList = new ClassList();
    this.children = [];
    this.parentElement = null;
    this.clientWidth = 320;
    this.clientHeight = 320;
    this.width = 320;
    this.height = 320;
    this.dots = [];
    this._innerHTML = '';
    this.events = {};
  }
  get innerHTML() { return this._innerHTML; }
  set innerHTML(value) {
    this._innerHTML = value;
    if (value === '') this.children = [];
  }
  addEventListener(name, listener) { (this.events[name] ||= []).push(listener); }
  removeEventListener() {}
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  /* Экран ищет карточки колёс, кнопки пресетов и колёса схемы по атрибутам.
   * Настоящего разбора HTML здесь нет, поэтому селектор по атрибуту отдаёт
   * свой постоянный элемент-заменитель: обработчики работают как в браузере. */
  querySelector(selector) {
    if (selector === '.kiosk') return byId('kiosk-root');
    this._bySelector ||= new Map();
    if (!this._bySelector.has(selector)) this._bySelector.set(selector, new El('', 'div'));
    return this._bySelector.get(selector);
  }
  querySelectorAll(selector) {
    if (/data-steer-preset/.test(selector)) {
      return byId('steer-preset-grid').children.filter((child) => child.dataset && child.dataset.steerPreset);
    }
    return [];
  }
  getContext() {
    if (!this._ctx) {
      const el = this;
      this._ctx = new Proxy({ canvas: el, measureText: () => ({ width: 10 }) }, {
        get(obj, prop) { return prop in obj ? obj[prop] : () => {}; },
        set(obj, prop, value) { obj[prop] = value; return true; },
      });
    }
    return this._ctx;
  }
  getBoundingClientRect() { return { width: 320, height: 320, top: 0, left: 0 }; }
  setAttribute(name, value) { this[name] = value; }
  getAttribute(name) { return this[name] === undefined ? null : this[name]; }
  focus() { global.document.activeElement = this; }
  click() { this.fire('click'); }
  fire(name, extra) {
    /* В браузере касание поля ввода отдаёт ему фокус. Экран по activeElement
     * отличает «оператор держит ползунок» от «показываю телеметрию», поэтому
     * без этого ползунок каждый кадр возвращался к углу из ответа сервера. */
    if (this.tagName === 'INPUT' && (name === 'pointerdown' || name === 'input' || name === 'change')) {
      global.document.activeElement = this;
    }
    const event = Object.assign({ type: name, target: this, preventDefault() {}, stopPropagation() {} }, extra);
    (this.events[name] || []).forEach((listener) => listener(event));
    return event;
  }
  closest() { return null; }
  play() { return Promise.resolve(); }
}

const elements = new Map();
function byId(id) {
  if (!elements.has(id)) elements.set(id, new El(id, /canvas/.test(id) ? 'canvas' : 'div'));
  return elements.get(id);
}

const renderErrors = [];
console.warn = (...args) => { renderErrors.push(args.map(String).join(' ')); };
console.error = (...args) => { renderErrors.push(args.map(String).join(' ')); };

global.document = {
  readyState: 'complete',
  body: new El('body'),
  documentElement: new El('html'),
  title: '',
  getElementById: byId,
  createElement: (tag) => new El('', tag),
  createDocumentFragment: () => new El('', 'fragment'),
  createTextNode: (text) => ({ textContent: text }),
  querySelector: (selector) => (selector === '.kiosk' ? byId('kiosk-root') : new El('', 'div')),
  querySelectorAll: () => [],
  addEventListener: () => {},
  removeEventListener: () => {},
  hidden: false,
  activeElement: null,
};

const rafQueue = [];
const timeouts = [];
const windowListeners = new Map();
global.window = global;
global.self = global;
global.addEventListener = (name, fn) => {
  if (!windowListeners.has(name)) windowListeners.set(name, []);
  windowListeners.get(name).push(fn);
};
global.removeEventListener = () => {};
global.dispatchEvent = (event) => {
  (windowListeners.get(event && event.type) || []).forEach((fn) => {
    try { fn(event); } catch (err) { /* как в браузере: сбой обработчика не роняет страницу */ }
  });
  return true;
};
global.location = { search: '', hash: '', href: 'http://127.0.0.1:8090/kiosk', protocol: 'http:' };
global.localStorage = {
  store: {},
  getItem(key) { return Object.prototype.hasOwnProperty.call(this.store, key) ? this.store[key] : null; },
  setItem(key, value) { this.store[key] = String(value); },
  removeItem(key) { delete this.store[key]; },
};
Object.defineProperty(global, 'navigator', {
  value: { userAgent: 'node-harness', vibrate: () => true, language: 'ru' },
  configurable: true, writable: true,
});
global.getComputedStyle = () => ({ fontFamily: 'sans-serif', getPropertyValue: () => '' });
global.requestAnimationFrame = (cb) => { rafQueue.push(cb); return rafQueue.length; };
global.cancelAnimationFrame = () => {};
global.matchMedia = () => ({ matches: false, addEventListener() {}, removeEventListener() {} });
global.MutationObserver = class {
  observe() {} disconnect() {} takeRecords() { return []; }
};
global.URLSearchParams = URLSearchParams;
global.AbortController = class {
  constructor() { this.signal = { aborted: false }; }
  abort() { this.signal.aborted = true; }
};
global.confirm = () => payload.confirm !== false;

/* ------------------------------------------------------- сеть (как в экране) */
const sent = [];      // команды руления: {module, angle}
const drives = [];    // команды тяги: {percent}
const emptyJournal = { ok: true, data: { entries: [], count: 0, shown: 0, dropped: 0, limit: 400,
                                         levels: {}, uptimeSec: 0, source: null, demo: false, journal: 'robot' } };

function jsonResponse(body, status) {
  const code = status === undefined ? 200 : status;
  return { ok: code >= 200 && code < 300, status: code, json: async () => body };
}

/* Журнал запросов: когда проверка падает, по нему видно, что экран вообще
 * спрашивал у сервера и отправлял ли команду. */
const netlog = [];

/* Телеметрия по кадрам: в жизни колесо сначала едет, потом останавливается —
 * и не всегда там, куда его послали. ``motorFrames`` позволяет показать экрану
 * именно эту последовательность, а не один застывший снимок. */
let frameIndex = 0;
function nextState() {
  const frames = payload.motorFrames;
  if (!Array.isArray(frames) || frames.length === 0) return payload;
  const motors = frames[Math.min(frameIndex, frames.length - 1)];
  frameIndex += 1;
  return Object.assign({}, payload, {
    data: Object.assign({}, payload.data, { motors }),
  });
}

global.fetch = async (url, init) => {
  const text = String(url);
  netlog.push(text);
  const body = init && init.body ? JSON.parse(init.body) : {};
  if (/kiosk_steer/.test(text)) {
    const module = String(body.module || '');
    sent.push({ module, angle: body.angle, header: (init.headers || {})['X-Robot-Control'] || null });
    const limit = Number((payload.data.steeringControl || {}).limitDeg || 90);
    const failOn = payload.steerFailOn || null;
    if (failOn && failOn.toUpperCase() === module.toUpperCase()) {
      return jsonResponse({ ok: false, reason: payload.steerFailReason || 'module_busy' }, 409);
    }
    if (Math.abs(Number(body.angle)) > limit) {
      return jsonResponse({ ok: false, reason: 'angle_limit', limit }, 400);
    }
    return jsonResponse({ ok: true, queued: true, module: module.toUpperCase(), angle: Number(body.angle) }, 200);
  }
  if (/kiosk_drive|api\/drive/.test(text)) {
    drives.push({ percent: body.percent });
    if (payload.driveRefuse) return jsonResponse({ ok: false, reason: payload.driveRefuse }, 409);
    return jsonResponse({ ok: true, percent: body.percent, maxPercent: 20 }, 200);
  }
  if (/kiosk_state/.test(text)) return jsonResponse(nextState());
  if (/kiosk_logs/.test(text)) return jsonResponse(payload.logs || emptyJournal);
  if (/camera/.test(text)) return jsonResponse({ ok: true, cameras: [] });
  return jsonResponse({ ok: true });
};

const originalSetTimeout = global.setTimeout;
global.setTimeout = (cb, ms) => { timeouts.push(cb); return timeouts.length; };
global.clearTimeout = () => {};

/* Интервалы экрана (опрос состояния каждые 250 мс, автоблокировка) в заглушке
 * пропадали: экран получал телеметрию ровно один раз и дальше жил застывшим
 * снимком. Без этого нельзя показать, что колесо доехало или остановилось
 * раньше цели, — а именно по телеметрии экран решает, снята ли команда. */
const intervals = [];
global.setInterval = (cb) => { intervals.push(cb); return intervals.length; };
global.clearInterval = () => {};

/* Виртуальные часы: часть поведения экрана зависит от времени (сколько ждём
 * колесо, сколько держим сообщение). Включаются только там, где это нужно —
 * остальные проверки идут на настоящем времени, как раньше. */
const virtualClock = payload.virtualClock === true;
const CLOCK_STEP_MS = 250;               // как POLL_MS: один проход = один опрос
let virtualNow = 1000000;
if (virtualClock) {
  const RealDate = Date;
  global.Date = class extends RealDate {
    static now() { return virtualNow; }
  };
}

/* ------------------------------------------------------------- запуск */
/* Начальные значения полей берём из robot.html: в браузере ползунок тяги
 * стартует с 8 %, а ход руля — из min/max разметки. Без этого заглушка
 * подставляла всем полям '0' и экран «ехал» с нулевой тягой. */
function seedFromMarkup() {
  const html = fs.readFileSync(path.join(ROOT, 'slam_gui', 'robot.html'), 'utf8');
  const tagRe = /<(input|button|select|output|textarea)\b[^>]*>/g;
  let match;
  while ((match = tagRe.exec(html)) !== null) {
    const tag = match[0];
    const id = /\bid="([^"]+)"/.exec(tag);
    if (!id) continue;
    const el = byId(id[1]);
    el.tagName = match[1].toUpperCase();
    const value = /\bvalue="([^"]*)"/.exec(tag);
    if (value) el.value = value[1];
    const min = /\bmin="([^"]*)"/.exec(tag);
    if (min) el.min = min[1];
    const max = /\bmax="([^"]*)"/.exec(tag);
    if (max) el.max = max[1];
    if (/\bdisabled\b/.test(tag)) el.disabled = true;
    if (/\bhidden\b/.test(tag)) el.hidden = true;
  }
}
seedFromMarkup();

function load(name) {
  const file = path.join(ROOT, 'slam_gui', name);
  vm.runInThisContext(fs.readFileSync(file, 'utf8'), { filename: file });
}
load('robot_model.js');
load('steer_presets.js');      // как в robot.html: общий модуль грузится до экрана
if (payload.remoteView) {
  global.RCRemoteControls = true;
  byId('ctrl-overlay').hidden = true;
  document.body.dataset.remoteView = payload.remoteView;
}
load('robot.js');

const sleep = (ms) => new Promise((resolve) => originalSetTimeout(resolve, ms));

async function flush(rounds) {
  for (let i = 0; i < (rounds || 6); i += 1) {
    await sleep(2);
    if (virtualClock) virtualNow += CLOCK_STEP_MS;
    while (rafQueue.length) {
      const cb = rafQueue.shift();
      try { cb(Date.now()); } catch (err) { renderErrors.push(String(err && err.message || err)); }
    }
    // Интервалы не снимаются: каждый проход — новый тик опроса состояния.
    for (const cb of intervals.slice()) {
      try { cb(); } catch (err) { renderErrors.push(String(err && err.message || err)); }
    }
    while (timeouts.length) {
      const cb = timeouts.shift();
      try { cb(); } catch (err) { renderErrors.push(String(err && err.message || err)); }
    }
  }
}

function presetButtons() {
  return byId('steer-preset-grid').children
    .filter((child) => child.dataset && child.dataset.steerPreset)
    .map((child) => ({
      id: child.dataset.steerPreset,
      title: (child.children[1] || {}).textContent || '',
      hint: (child.children[2] || {}).textContent || '',
      disabled: !!child.disabled,
      unavailable: child.classList.contains('is-unavailable'),
      wheels: ((child.children[0] || {}).children || []).length,
    }));
}

function findPreset(id) {
  return byId('steer-preset-grid').children.find((child) => child.dataset && child.dataset.steerPreset === id);
}

(async function run() {
  await flush(6);

  const actions = payload.actions || [];
  for (const action of actions) {
    if (action.type === 'preset') {
      const button = findPreset(action.id);
      if (!button) { renderErrors.push('пресет не найден: ' + action.id); continue; }
      button.fire('click');
      await flush(action.rounds || 4);
    } else if (action.type === 'master') {
      const slider = byId('steer-master-slider');
      slider.fire('pointerdown', { pointerId: 1 });
      slider.value = String(action.angle);
      slider.fire('input');
      await flush(1);
      byId('steer-apply-all').fire('click');
      await flush(action.rounds || 4);
    } else if (action.type === 'wheel') {
      const slider = byId('steer-slider-' + action.id);
      slider.fire('pointerdown', { pointerId: 1 });
      slider.value = String(action.angle);
      slider.fire('input');
      await flush(1);
      if (action.send !== false) slider.fire('change');
      await flush(action.rounds || 4);
    } else if (action.type === 'drive') {
      const button = byId(action.direction < 0 ? 'drive-back' : 'drive-forward');
      button.fire('pointerdown', { pointerId: 1 });
      await flush(1);
      global.dispatchEvent({ type: 'pointerup' });
      await flush(action.rounds || 3);
    } else if (action.type === 'stop') {
      byId('drive-stop').fire('click');
      await flush(3);
    }
  }

  const result = {
    batteryVisible: !byId('swipe-battery-view').hidden,
    motorsVisible: !byId('swipe-motors-view').hidden,
    presets: presetButtons(),
    presetCount: presetButtons().length,
    sharedLimit: global.RCSteerPresets ? global.RCSteerPresets.limit() : null,
    masterSlider: { min: byId('steer-master-slider').min, max: byId('steer-master-slider').max,
                    value: byId('steer-master-slider').value },
    wheelSliders: ['FL', 'FR', 'RL', 'RR'].map((id) => ({
      id, min: byId('steer-slider-' + id).min, max: byId('steer-slider-' + id).max,
    })),
    wheels: ['FL', 'FR', 'RL', 'RR'].map((id) => ({
      id,
      state: byId('steer-state-' + id).textContent,
      sliderDisabled: !!byId('steer-slider-' + id).disabled,
    })),
    status: byId('steer-preset-status').textContent,
    masterMessages: byId('steer-messages').innerHTML,
    sourceStatus: byId('steer-source-status').textContent,
    dataChip: byId('data-chip').textContent,
    driveStatus: byId('drive-status').textContent,
    heroAngle: byId('steer-hero-angle').textContent,
    heroMode: byId('steer-hero-mode').textContent,
    masterValue: byId('steer-master-value').textContent,
    sent,
    drives,
    netlog,
    renderErrors,
  };
  process.stdout.write(JSON.stringify(result, null, 2) + '\n');
})().catch((error) => {
  process.stdout.write(JSON.stringify({ fatal: String(error && error.stack || error), renderErrors }) + '\n');
  process.exit(1);
});
