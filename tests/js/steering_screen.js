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
  insertAdjacentElement(position, element) { this.adjacent = element; }
  dispatchEvent(event) { this.fire(event.type); return true; }
  setPointerCapture(id) { this.capture = id; }
  hasPointerCapture(id) { return this.capture === id; }
  releasePointerCapture() { this.capture = null; }
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
  createElementNS: (_ns, tag) => new El('', tag),
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
const drives = [];    // команды тяги: {value} или прежний {percent}
const driveDeadlines = [];  // сроки годности команд: STOP без срока
const emptyJournal = { ok: true, data: { entries: [], count: 0, shown: 0, dropped: 0, limit: 400,
                                         levels: {}, uptimeSec: 0, source: null, demo: false, journal: 'robot' } };

function jsonResponse(body, status) {
  const code = status === undefined ? 200 : status;
  return { ok: code >= 200 && code < 300, status: code, json: async () => body };
}

/* Журнал запросов: когда проверка падает, по нему видно, что экран вообще
 * спрашивал у сервера и отправлял ли команду. */
const netlog = [];

/* Запросы настройки привода: их быть не должно вовсе. Журнал нужен, чтобы
 * проверка «экран ничего не настраивает» опиралась на факты, а не на разметку. */
const driveCfgSent = [];

/* Манёвры («метр вперёд», «90° на месте»): экран сначала сохраняет числа
 * пресета, потом просит старт — журнал показывает обе команды по порядку. */
const maneuversSent = [];

/* Телеметрия по кадрам: в жизни колесо сначала едет, потом останавливается —
 * и не всегда там, куда его послали. ``motorFrames`` позволяет показать экрану
 * именно эту последовательность, а не один застывший снимок. */
let frameIndex = 0;
let kioskStateDown = false;
let heldDriveResolve = null;
let heldOneDrive = false;
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
  if (/drive_config/.test(text)) {
    driveCfgSent.push({
      action: body.action,
      value: body.value === undefined ? null : body.value,
      module: body.module === undefined ? null : body.module,
      header: (init.headers || {})['X-Robot-Control'] || null,
    });
    if (payload.driveCfgRefuse) return jsonResponse({ ok: false, reason: payload.driveCfgRefuse }, 409);
    return jsonResponse({ ok: true, queued: true, action: body.action }, 200);
  }
  if (/kiosk_maneuver_config/.test(text)) {
    maneuversSent.push({url: text, action: 'config', name: body.name,
                        values: body.values === undefined ? null : body.values});
    if (payload.maneuverConfigRefuse) {
      return jsonResponse({ ok: false, reason: payload.maneuverConfigRefuse }, 400);
    }
    return jsonResponse({ ok: true }, 200);
  }
  if (/kiosk_maneuver/.test(text)) {
    maneuversSent.push({url: text, action: body.action, name: body.name || null});
    if (payload.maneuverRefuse) {
      return jsonResponse({ ok: false, reason: payload.maneuverRefuse }, 409);
    }
    return jsonResponse({ ok: true, maneuver: { state: 'steering' } }, 200);
  }
  if (/kiosk_drive|api\/drive/.test(text)) {
    const sent = {module: body.module === undefined ? null : body.module};
    if (Object.prototype.hasOwnProperty.call(body, 'value')) sent.value = body.value;
    else sent.percent = body.percent;
    if (Object.prototype.hasOwnProperty.call(body, 'direction')) sent.direction = body.direction;
    drives.push(sent);
    driveDeadlines.push((init.headers || {})['X-Robot-Command-Deadline'] || null);
    if (payload.deferFirstDrive && !heldOneDrive && Number(body.value) > 0) {
      heldOneDrive = true;
      await new Promise((resolve) => { heldDriveResolve = resolve; });
    }
    if (payload.driveRefuse) return jsonResponse({ ok: false, reason: payload.driveRefuse }, 409);
    return jsonResponse({ ok: true, value: body.value, percent: body.percent }, 200);
  }
  if (/kiosk_state/.test(text)) {
    if (kioskStateDown) throw new Error('simulated connection loss');
    return jsonResponse(nextState());
  }
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
let intervalSeq = 0;
global.setInterval = (cb) => {
  intervalSeq += 1;
  intervals.push({ id: intervalSeq, cb });
  return intervalSeq;
};
/* clearInterval снимает таймер по-настоящему: экран останавливает опрос, когда
 * дело сделано (например, прошивка закончилась), и дальше тикать не должен. */
global.clearInterval = (id) => {
  const index = intervals.findIndex((entry) => entry.id === id);
  if (index >= 0) intervals.splice(index, 1);
};

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
if (payload.dials) load('steer_dial.js');
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
    for (const entry of intervals.slice()) {
      try { entry.cb(); } catch (err) { renderErrors.push(String(err && err.message || err)); }
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
    if (action.type === 'dial') {
      const slider = byId('steer-slider-' + (action.module || 'FL'));
      const dial = global.RCSteerDial.attach(slider).element;
      const pos = angle => ({pointerId:1, button:0, clientX:160+120*Math.sin(angle*Math.PI/180), clientY:160-120*Math.cos(angle*Math.PI/180)});
      dial.fire('pointerdown', pos(action.angle));
      if (action.move != null) dial.fire('pointermove', pos(action.move));
      if (action.disable) { slider.disabled = true; global.RCSteerDial.sync(slider, null); }
      dial.fire(action.cancel ? 'pointercancel' : 'pointerup', pos(action.move == null ? action.angle : action.move));
      await flush(action.rounds || 4);
    } else if (action.type === 'preset') {
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
    } else if (action.type === 'speed' || action.type === 'drive' || action.type === 'wheel-drive') {
      const id = action.type === 'wheel-drive' || action.id ? String(action.id || 'FL') : 'ALL';
      const slider = byId(id === 'ALL' ? 'drive-speed' : 'drive-module-' + id);
      slider.value = String(action.value === undefined ? 8 : action.value);
      slider.fire('input');
      await flush(action.rounds || 2);
      if (action.type !== 'speed') {
        const direction = action.direction === 'backward' ? 'back' : 'forward';
        const pointerId = action.pointerId || (id === 'ALL' ? 1 : ['FL', 'FR', 'RL', 'RR'].indexOf(id) + 2);
        const button = byId('direct-drive-' + direction + '-' + id);
        // Короткое касание/клик: браузер выдаёт click после отпускания.
        button.fire('pointerdown', {button: 0, pointerId});
        button.fire('pointerup', {pointerId});
        if (!button.disabled) button.fire('click', {detail: 1});
        await flush(action.rounds || 2);
        if (action.release === true) {
          button.fire('pointerup', {pointerId});
          await flush(2);
        }
      }
    } else if (action.type === 'press') {
      const id = String(action.id || 'ALL');
      const direction = action.direction === 'backward' ? 'back' : 'forward';
      const pointerId = action.pointerId || (id === 'ALL' ? 1 : ['FL', 'FR', 'RL', 'RR'].indexOf(id) + 2);
      const button = byId('direct-drive-' + direction + '-' + id);
      button.fire('pointerdown', {button: 0, pointerId});
      button.fire('pointerup', {pointerId});
      if (!button.disabled) button.fire('click', {detail: 1});
      await flush(action.rounds || 3);
    } else if (action.type === 'release') {
      const id = String(action.id || 'ALL');
      const direction = action.direction === 'backward' ? 'back' : 'forward';
      const pointerId = action.pointerId || (id === 'ALL' ? 1 : ['FL', 'FR', 'RL', 'RR'].indexOf(id) + 2);
      byId('direct-drive-' + direction + '-' + id).fire(action.cancel ? 'pointercancel' : 'pointerup', {pointerId});
      await flush(action.rounds || 3);
    } else if (action.type === 'key') {
      const id = String(action.id || 'ALL');
      const direction = action.direction === 'backward' ? 'back' : 'forward';
      const button = byId('direct-drive-' + direction + '-' + id);
      const event = action.event || 'keydown';
      const key = action.key || 'Enter';
      button.fire(event, {key});
      // Нативные кнопки посылают click с Enter на keydown и с пробела на keyup.
      if (!button.disabled && ((key === 'Enter' && event === 'keydown') || (key === ' ' && event === 'keyup'))) {
        button.fire('click', {detail: 0});
      }
      await flush(action.rounds || 3);
    } else if (action.type === 'wait') {
      await flush(action.rounds || 10);
    } else if (action.type === 'blur') {
      global.dispatchEvent({type: 'blur'});
      await flush(action.rounds || 3);
    } else if (action.type === 'finish-drive') {
      if (heldDriveResolve) heldDriveResolve();
      heldDriveResolve = null;
      await flush(action.rounds || 4);
    } else if (action.type === 'server-offline' || action.type === 'server-online') {
      kioskStateDown = action.type === 'server-offline';
      await flush(action.rounds || 16);
    } else if (action.type === 'servo-offline') {
      const state = payload.data.drive.servo;
      state.perModule[action.id].canDrive = false;
      state.perModule[action.id].reason = 'module_offline';
      state.canDrive = false;
      state.reason = 'module_offline';
      await flush(action.rounds || 4);
    } else if (action.type === 'servo-online') {
      const state = payload.data.drive.servo;
      state.perModule[action.id].canDrive = true;
      state.perModule[action.id].reason = null;
      state.canDrive = Object.values(state.perModule).every(item => item.canDrive);
      state.reason = null;
      await flush(action.rounds || 4);
    } else if (action.type === 'wheel-stop') {
      byId('drive-module-stop-' + String(action.id || 'FL')).fire('click');
      await flush(3);
    } else if (action.type === 'stop') {
      byId('drive-stop').fire('click');
      await flush(3);
    } else if (action.type === 'maneuver') {
      const id = String(action.id || 'forward');
      if (action.target !== undefined) {
        const targetEl = byId('maneuver-target-' + id);
        targetEl.value = String(action.target);
        targetEl.fire('input');
      }
      if (action.percent !== undefined) {
        const percentEl = byId('maneuver-percent-' + id);
        percentEl.value = String(action.percent);
        percentEl.fire('input');
      }
      byId('maneuver-' + id).fire('click');
      await flush(action.rounds || 8);
    } else if (action.type === 'maneuver-stop') {
      byId('maneuver-stop').fire('click');
      await flush(action.rounds || 6);
    }
  }

  const result = {
    dialStates: ['FL', 'FR', 'RL', 'RR'].map(id => {
      const dial = global.RCSteerDial && global.RCSteerDial.attach(byId('steer-slider-' + id));
      return dial ? {id, disabled:dial.element.getAttribute('aria-disabled'), value:dial.element.getAttribute('aria-valuenow')} : null;
    }),
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
    driveReported: byId('drive-servo-reported').textContent,
    driveRpmReported: byId('drive-rpm-reported').textContent,
    driveSpeedMax: byId('drive-speed').max,
    driveSpeedMin: byId('drive-speed').min,
    driveSpeedValue: byId('drive-speed').value,
    driveSpeedDisabled: !!byId('drive-speed').disabled,
    driveBackDisabled: !!byId('direct-drive-back-ALL').disabled,
    driveForwardDisabled: !!byId('direct-drive-forward-ALL').disabled,
    driveBackPressed: byId('direct-drive-back-ALL').getAttribute('aria-pressed'),
    driveForwardPressed: byId('direct-drive-forward-ALL').getAttribute('aria-pressed'),
    rpmText: ['FL', 'FR', 'RL', 'RR'].map((id) => byId('rpm-' + id).textContent),
    wheelDrive: ['FL', 'FR', 'RL', 'RR'].map((id) => ({
      id,
      state: byId('drive-module-state-' + id).textContent,
      output: byId('drive-module-output-' + id).textContent,
      rpm: byId('drive-module-rpm-' + id).textContent,
      sliderMax: byId('drive-module-' + id).max,
      sliderMin: byId('drive-module-' + id).min,
      sliderValue: byId('drive-module-' + id).value,
      sliderDisabled: !!byId('drive-module-' + id).disabled,
      backDisabled: !!byId('direct-drive-back-' + id).disabled,
      forwardDisabled: !!byId('direct-drive-forward-' + id).disabled,
      backPressed: byId('direct-drive-back-' + id).getAttribute('aria-pressed'),
      forwardPressed: byId('direct-drive-forward-' + id).getAttribute('aria-pressed'),
      stopDisabled: !!byId('drive-module-stop-' + id).disabled,
    })),
    maneuvers: {
      buttons: ['forward', 'backward', 'left', 'right', 'spin_left', 'spin_right'].map((id) => ({
        id,
        disabled: !!byId('maneuver-' + id).disabled,
        label: byId('maneuver-' + id).textContent,
        target: byId('maneuver-target-' + id).value,
        percent: byId('maneuver-percent-' + id).value,
      })),
      stopDisabled: !!byId('maneuver-stop').disabled,
      status: byId('maneuver-status').textContent,
      active: byId('maneuver-status').dataset.active,
    },
    maneuversSent,
    heroAngle: byId('steer-hero-angle').textContent,
    heroMode: byId('steer-hero-mode').textContent,
    masterValue: byId('steer-master-value').textContent,
    sent,
    drives,
    driveDeadlines,
    driveCfgSent,
    netlog,
    renderErrors,
  };
  process.stdout.write(JSON.stringify(result, null, 2) + '\n');
})().catch((error) => {
  process.stdout.write(JSON.stringify({ fatal: String(error && error.stack || error), renderErrors }) + '\n');
  process.exit(1);
});
