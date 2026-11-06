/* Проверка выбора портов на странице «Устройства» — без браузера.
 *
 * Страница (slam_gui/devices.js) написана под браузер, поэтому здесь поднят
 * минимальный DOM, скрипт выполняется как в странице, а `fetch` отдаёт
 * заготовленные ответы /api/devices и /api/devices/ports. Результат читается
 * из тех же <select>, которые видит оператор: значения, подписи, группы и
 * повторяющиеся порты.
 *
 * Запуск:
 *   node tests/js/devices_ports.js <файл-payload.json>
 * На stdout — JSON: { serial: {...}, video: [...], usb: [...], cards: {...},
 *                      wheels: {...}, found: {...}, calls: [...] }.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..', '..');
const payloadFile = process.argv[2];
if (!payloadFile) {
  console.error('нужен файл с ответами API');
  process.exit(2);
}
const payload = JSON.parse(fs.readFileSync(payloadFile, 'utf8'));
const realSetTimeout = setTimeout; // до подмены: отложенные задачи страницы не нужны

/* ------------------------------------------------------------------ DOM */
class El {
  constructor(id, tag) {
    this._id = id || '';
    this.tagName = (tag || 'div').toUpperCase();
    this.textContent = '';
    this.hidden = false;
    this.disabled = false;
    this.title = '';
    this.label = '';
    this.value = '';
    this.type = '';
    this.className = '';
    this.htmlFor = '';
    this.placeholder = '';
    this.maxLength = 0;
    this.spellcheck = true;
    this.autocomplete = '';
    this.style = { setProperty() {}, removeProperty() {}, cssText: '' };
    this.dataset = {};
    this.attributes = {};
    this.children = [];
    this.events = {};
    this.parentElement = null;
  }
  // Страница создаёт элементы через createElement и лишь потом задаёт id:
  // регистрируем их, иначе getElementById вернёт пустую заглушку.
  get id() { return this._id; }
  set id(value) {
    this._id = String(value || '');
    if (this._id) elements.set(this._id, this);
  }
  addEventListener(name, listener) { (this.events[name] ||= []).push(listener); }
  removeEventListener() {}
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  append(...children) { children.forEach((child) => this.appendChild(child)); }
  replaceChildren(...children) {
    this.children = [];
    children.forEach((child) => this.appendChild(child));
  }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  getAttribute(name) {
    if (name in this.attributes) return this.attributes[name];
    return null;
  }
  removeAttribute(name) { delete this.attributes[name]; }
  get options() {
    const out = [];
    const walk = (node) => {
      for (const child of node.children || []) {
        if (child.tagName === 'OPTGROUP') walk(child);
        else if (child.tagName === 'OPTION') out.push(child);
      }
    };
    walk(this);
    return out;
  }
  focus() {}
  click() { (this.events.click || []).forEach((listener) => listener({ target: this, preventDefault() {} })); }
  querySelector() { return null; }
  querySelectorAll() { return []; }
  getBoundingClientRect() { return {left: 0, right: 0, top: 0, bottom: 0, width: 0, height: 40}; }
}

const elements = new Map();
function byId(id) {
  if (!elements.has(id)) elements.set(id, new El(id, 'div'));
  return elements.get(id);
}

global.document = {
  readyState: 'complete',
  hidden: false,
  activeElement: null,
  body: new El('body'),
  documentElement: new El('html'),
  getElementById: byId,
  createElement: (tag) => new El('', tag),
  querySelector: () => null,
  querySelectorAll: () => [],
  addEventListener: () => {},
  removeEventListener: () => {},
};

global.window = global;
global.self = global;
global.addEventListener = () => {};
global.removeEventListener = () => {};
global.confirm = () => true;
global.AbortController = class {
  constructor() { this.signal = {}; }
  abort() {}
};
global.setInterval = () => 0;
global.clearInterval = () => {};
const timeouts = [];
global.setTimeout = (fn) => { timeouts.push(fn); return timeouts.length; };
global.clearTimeout = () => {};

/* --------------------------------------------------------------- ответы API */
const calls = [];
function reply(body, status) {
  const code = status || 200;
  return {ok: code < 400, status: code, json: async () => body};
}

global.fetch = async (url, options) => {
  const text = String(url);
  const method = (options && options.method) || 'GET';
  calls.push({url: text, method});
  if (/\/api\/control\/session$/.test(text)) {
    return {ok: false, status: 404, json: async () => ({ok: false})};
  }
  if (/\/api\/devices\/ports/.test(text)) return reply(payload.ports || {ok: true, roles: {}});
  if (/\/api\/devices/.test(text)) return reply(payload.snapshot || {ok: true, devices: []});
  if (/\/api\/sensor/.test(text)) return reply({ok: true});
  return reply({ok: true});
};

/* ------------------------------------------------------------------ пуск */
const renderErrors = [];
console.error = (...args) => { renderErrors.push(args.map((a) => String(a)).join(' ')); };

const file = path.join(ROOT, 'slam_gui', 'devices.js');
try {
  vm.runInThisContext(fs.readFileSync(file, 'utf8'), {filename: file});
} catch (err) {
  console.log(JSON.stringify({boot: String((err && err.stack) || err)}));
  process.exit(0);
}

const sleep = (ms) => new Promise((resolve) => realSetTimeout(resolve, ms));

function picker(id) {
  const node = byId(id);
  const options = node.options || [];
  return {
    count: options.length,
    values: options.map((option) => option.value).filter((value) => value !== ''),
    labels: options.map((option) => option.textContent).filter(Boolean),
    groups: (node.children || []).filter((child) => child.tagName === 'OPTGROUP')
      .map((group) => ({label: group.label,
                        values: (group.children || []).map((option) => option.value)})),
    missing: options.filter((option) => option.dataset.missing === '1')
      .map((option) => option.value),
    hint: options.length ? options[0].title || '' : '',
  };
}

function foundRows() {
  return (byId('ports-found-list').children || []).map((row) => ({
    port: (row.children[0] || {}).textContent || '',
    name: (row.children[1] || {}).textContent || '',
    kind: (row.children[2] || {}).textContent || '',
    // Владелец и «не подключено» — отдельные ячейки, порядок зависит от порта.
    owner: (row.children.find((cell) => /назначен:/.test(cell.textContent || '')) || {})
      .textContent || '',
    missing: (row.children.find((cell) => /не подключено/.test(cell.textContent || '')) || {})
      .textContent || '',
  }));
}

(async function run() {
  await sleep(30);

  const result = {
    found: {
      text: byId('ports-found').textContent,
      tone: byId('ports-found').dataset.tone || '',
      rows: foundRows(),
    },
    camera: picker('pick-camera_FL'),
    lidar: picker('pick-lidar'),
    kinect: picker('pick-kinect'),
    printer: picker('pick-printer'),
    wheels: ['1', '2', '3', '4'].map((index) => picker('wheel-pick-' + index)),
    inputs: ['hint-camera_FL', 'hint-lidar', 'wheel-slot-1'].map((id) => byId(id).value),
    status: byId('server-status').textContent,
    alert: byId('device-alert').textContent,
    calls: calls.slice(),
    errors: renderErrors.slice(),
  };

  /* Действие оператора: выбрал порт в списке — он попал в поле для .env. */
  const pick = byId('pick-lidar');
  const wanted = (payload.pickValue !== undefined)
    ? String(payload.pickValue) : (result.lidar.values[0] || '');
  pick.value = wanted;
  (pick.events.change || []).forEach((listener) => listener({target: pick}));
  await sleep(10);
  result.afterPick = {value: wanted, input: byId('hint-lidar').value};

  console.log(JSON.stringify(result));
  process.exit(0);
})();
