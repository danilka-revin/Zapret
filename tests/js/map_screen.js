/* Проверка страницы «Карта лидара» бортового экрана — без браузера.
 *
 * Скрипт страницы (slam_gui/map_view.js) написан под браузер, поэтому здесь
 * поднимается минимальный DOM, скрипт выполняется как в странице, а `fetch`
 * отдаёт заготовленные ответы API карты. Результат читается из тех же
 * элементов, которые видит оператор: статистика карты, плашка MAPPING/IDLE,
 * путь к файлу карты и сообщения о действиях (сброс, настройки, сохранение).
 *
 * Запуск:
 *   node tests/js/map_screen.js <файл-payload.json>
 * На stdout — JSON: { stats, badge, file, msg, calls, renderErrors }.
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

/* Контекст канвы: всё, что рисует карта, просто запоминаем. */
function makeCtx(el) {
  const target = {
    canvas: el,
    calls: [],
    fillRect: (...args) => { target.calls.push(['fillRect', ...args]); },
    clearRect: () => {},
    drawImage: (...args) => { target.calls.push(['drawImage', ...args]); },
    measureText: () => ({ width: 10 }),
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
  constructor(id, tag) {
    this.id = id || '';
    this.tagName = (tag || 'div').toUpperCase();
    this.textContent = '';
    this.innerHTML = '';
    this.hidden = false;
    this.title = '';
    this.value = '0';
    this.checked = false;
    this.disabled = false;
    this.style = { setProperty() {}, removeProperty() {}, cssText: '' };
    this.dataset = {};
    this.classList = new ClassList();
    this.children = [];
    this.parentElement = null;
    this.clientWidth = 800;
    this.clientHeight = 600;
    this.width = 800;
    this.height = 600;
    this.complete = true;
    this.naturalWidth = 300;
    this.events = {};
  }
  addEventListener(name, listener) { (this.events[name] ||= []).push(listener); }
  removeEventListener() {}
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  querySelector() { return null; }
  querySelectorAll() { return []; }
  getContext() { if (!this._ctx) this._ctx = makeCtx(this); return this._ctx; }
  getBoundingClientRect() { return { width: 800, height: 600, top: 0, left: 0 }; }
  setAttribute(name, value) { this[name] = value; }
  getAttribute(name) { return this[name] === undefined ? null : this[name]; }
  focus() {}
  click() { (this.events.click || []).forEach((listener) => listener({ target: this, preventDefault() {} })); }
  closest() { return null; }
}

const elements = new Map();
function byId(id) {
  if (!elements.has(id)) {
    elements.set(id, new El(id, /canvas/.test(id) ? 'canvas' : 'div'));
  }
  return elements.get(id);
}

global.document = {
  readyState: 'complete',
  body: new El('body'),
  documentElement: new El('html'),
  title: '',
  getElementById: byId,
  createElement: (tag) => new El('', tag),
  createDocumentFragment: () => new El('', 'fragment'),
  createTextNode: (text) => ({ textContent: text }),
  querySelector: () => null,
  querySelectorAll: () => [],
  addEventListener: () => {},
  removeEventListener: () => {},
  hidden: false,
};

/* Картинка карты приходит как data-URL: в Node нет Image, поэтому заменяем. */
class FakeImage {
  constructor() { this.complete = false; this.naturalWidth = 0; this.onload = null; }
  set src(value) {
    this._src = value;
    this.complete = true;
    this.naturalWidth = 300;
    if (this.onload) this.onload();
  }
  get src() { return this._src; }
}

global.window = global;
global.self = global;
global.Image = FakeImage;
global.devicePixelRatio = 1;
global.addEventListener = () => {};
global.removeEventListener = () => {};
global.dispatchEvent = () => true;
global.requestAnimationFrame = () => 0;
global.cancelAnimationFrame = () => {};
global.matchMedia = () => ({ matches: false, addEventListener() {}, removeEventListener() {} });
global.MutationObserver = class {
  constructor(callback) { this.callback = callback; }
  observe() {}
  disconnect() {}
};

/* --------------------------------------------------------------- ответы API */
const calls = [];
const renderErrors = [];
const originalError = console.error.bind(console);
console.error = (...args) => { renderErrors.push(args.map((a) => String(a)).join(' ')); };

function reply(body, ok) {
  return {
    ok: ok !== false,
    status: ok === false ? 500 : 200,
    json: async () => body,
  };
}

global.fetch = async (url, options) => {
  const text = String(url);
  const method = (options && options.method) || 'GET';
  const body = options && options.body ? JSON.parse(options.body) : null;
  calls.push({ url: text, method, body });
  if (/kiosk_map\/reset/.test(text)) {
    return reply(payload.reset || { ok: true, paths: payload.paths });
  }
  if (/kiosk_map\/save/.test(text)) {
    return reply(payload.save || { ok: true, paths: payload.paths });
  }
  if (/kiosk_map\/settings/.test(text)) {
    return reply(payload.settingsReply || { ok: true, settings: payload.settings, mapReset: false });
  }
  return reply(payload.snapshot || payload);
};

const originalSetTimeout = global.setTimeout;
global.setTimeout = (cb, ms) => { return originalSetTimeout(cb, 0); };
global.setInterval = () => 0;
global.clearInterval = () => {};

/* ------------------------------------------------------------------ пуск */
const file = path.join(ROOT, 'slam_gui', 'map_view.js');
vm.runInThisContext(fs.readFileSync(file, 'utf8'), { filename: file });

const sleep = (ms) => new Promise((resolve) => originalSetTimeout(resolve, ms));

(async function run() {
  // Страница открыта: карта должна приехать с сервера и заполнить статистику.
  byId('swipe-map-view').hidden = false;
  global.RCMapView.init();
  await sleep(20);

  const snapshot = payload.snapshot || payload;
  const stats = {
    size: byId('map-size').textContent,
    resolution: byId('map-res').textContent,
    coverage: byId('map-cov').textContent,
    occupied: byId('map-occ').textContent,
    pose: byId('map-pose').textContent,
    scans: byId('map-scans').textContent,
    file: byId('map-file').textContent,
    quality: byId('map-quality').textContent,
    meta: byId('map-meta').textContent,
  };

  // Сброс карты.
  byId('map-reset').click();
  await sleep(20);
  const afterReset = { msg: byId('map-msg').textContent, tone: byId('map-msg').dataset.tone };

  // Сохранение карты на рабочий стол.
  byId('map-save').click();
  await sleep(20);
  const afterSave = { msg: byId('map-msg').textContent, tone: byId('map-msg').dataset.tone };

  // Настройки: чувствительность и дальность уходят на сервер.
  byId('map-set-range').value = '6';
  byId('map-set-sensitivity').value = 'hard';
  byId('map-settings-apply').click();
  await sleep(20);
  const afterApply = { msg: byId('map-msg').textContent, tone: byId('map-msg').dataset.tone };

  // Живой скан: точки лидара рисуются поверх карты.
  global.RCMapView.setScan([{ a: 0, r: 2 }, { a: 1.57, r: 3 }]);

  // Тема карты: по умолчанию тёмная, кнопка переключает на светлую.
  const themeAttr = () => (global.document.documentElement.getAttribute('data-map-theme') || '');
  const themeBefore = { attr: themeAttr(), label: byId('map-theme-toggle').textContent,
                        pressed: byId('map-theme-toggle').getAttribute('aria-pressed') };
  byId('map-theme-toggle').click();
  const themeAfter = { attr: themeAttr(), label: byId('map-theme-toggle').textContent,
                       pressed: byId('map-theme-toggle').getAttribute('aria-pressed') };

  console.log(JSON.stringify({
    stats,
    themeBefore,
    themeAfter,
    badge: byId('map-badge').textContent,
    badgeTone: byId('map-badge').dataset.tone,
    emptyHidden: byId('map-empty').hidden,
    afterReset,
    afterSave,
    afterApply,
    calls,
    renderErrors,
  }, null, 2));
  console.error = originalError;
})().catch((err) => {
  originalError(err && err.stack ? err.stack : String(err));
  process.exit(1);
});
