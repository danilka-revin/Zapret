/* Проверка страницы «Скан» без браузера: реальный slam_gui/scan_mode.js
 * выполняется в минимальном DOM, fetch отвечает заготовками.
 *
 * Проверяем то, что действительно может сломаться:
 *   • при выключенном управлении кнопки движения заблокированы и ничего не отправляют;
 *   • «Включить» включает, «Выключить» снимает тягу (нулевая команда уходит);
 *   • удержание кнопки повторяет тягу, отпускание гасит её нулём;
 *   • Esc и потеря фокуса снимают тягу;
 *   • молчание сервера (enabled=false) честно гасит управление на экране.
 *
 * Запуск: node tests/js/scan_screen.js
 */
'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const ROOT = path.resolve(__dirname, '..', '..');
const elements = new Map();
const calls = [];
const documentEvents = {};

class Element {
  constructor(id) {
    this.id = id || '';
    this.tagName = 'DIV';
    this.events = {};
    this.dataset = {};
    this.attrs = {};
    this.style = {};
    this.children = [];
    this.disabled = false;
    this.hidden = false;
    this.value = '8';
    this.max = '20';
    this.textContent = '';
    this.classList = {contains: () => false, add() {}, remove() {}, toggle() {}};
  }
  addEventListener(name, fn) { (this.events[name] ||= []).push(fn); }
  fire(name, event = {}) {
    (this.events[name] || []).forEach((fn) => fn({
      target: this, preventDefault() {}, stopPropagation() {}, key: event.key, ...event,
    }));
  }
  getContext() { return {setTransform() {}, fillRect() {}, drawImage() {}, fillStyle: ''}; }
  getBoundingClientRect() { return {width: 320, height: 180, top: 0, left: 0}; }
  setAttribute(k, v) { this.attrs[k] = v; this[k] = v; }
  getAttribute(k) { return this.attrs[k]; }
  appendChild(child) { this.children.push(child); return child; }
}

function byId(id) {
  if (!elements.has(id)) elements.set(id, new Element(id));
  return elements.get(id);
}

global.document = {
  readyState: 'complete',
  hidden: false,
  body: new Element('body'),
  documentElement: new Element('html'),
  title: '',
  getElementById: byId,
  createElement: () => new Element(''),
  querySelector: () => null,
  querySelectorAll: () => [],
  addEventListener: (name, fn) => { (documentEvents[name] ||= []).push(fn); },
  removeEventListener() {},
};
global.window = global;
global.self = global;
global.devicePixelRatio = 1;
const windowEvents = {};
global.addEventListener = (name, fn) => { (windowEvents[name] ||= []).push(fn); };
global.removeEventListener = () => {};
global.localStorage = {getItem: () => null, setItem() {}};
/* Кадры камер грузятся через Image, а не fetch: их адреса собираем отдельно. */
const imageRequests = [];
global.Image = class {
  constructor() { this.naturalWidth = 0; this.naturalHeight = 0; }
  set src(value) {
    this._src = value;
    imageRequests.push(String(value));
    // Камеры в этом прогоне отвечают кадром сразу.
    this.naturalWidth = 320;
    this.naturalHeight = 180;
    if (this.onload) this.onload();
  }
  get src() { return this._src; }
};
global.confirm = () => true;

/* ------------------------------------------------------------------- API */
let mode = {
  ok: true, enabled: false, token: null, maxPercent: 20, deadmanS: 5,
  reason: '', map: {updatedAt: 1, savedAt: 1, paths: {pgm: '/tmp/map.pgm'}},
};

function reply(body, status) {
  const code = status || 200;
  return {ok: code < 400, status: code, json: async () => body};
}

global.fetch = async (url, options) => {
  const text = String(url);
  const method = (options && options.method) || 'GET';
  const body = options && options.body ? JSON.parse(options.body) : null;
  calls.push({url: text, method, body, headers: (options && options.headers) || {}});

  if (/api\/scan_mode$/.test(text)) return reply({...mode});
  if (/api\/scan_mode\/enable$/.test(text)) {
    mode = {...mode, enabled: true, token: 'tok-1'};
    return reply({...mode, token: 'tok-1'});
  }
  if (/api\/scan_mode\/disable$/.test(text)) {
    mode = {...mode, enabled: false, token: null, reason: 'disabled'};
    return reply({...mode, braked: true});
  }
  if (/api\/scan_mode\/heartbeat$/.test(text)) return reply({...mode});
  if (/api\/scan_mode\/drive$/.test(text)) return reply({ok: true, percent: body.percent});
  if (/api\/scan_mode\/steer$/.test(text)) return reply({ok: true});
  if (/api\/camera\/\d\/frame/.test(text)) return reply({ok: true});
  if (/kiosk_map\/save|kiosk_map\/reset|scan_mode\/map\/load/.test(text)) return reply({ok: true});
  return reply({ok: true, scans: 3, stats: {}, pose: {x: 0, y: 0, yaw: 0}, png: ''});
};

/* Подменяем таймеры: тест не должен ждать реальные интервалы. */
const intervals = [];
global.setInterval = (fn) => { intervals.push(fn); return intervals.length; };
global.clearInterval = () => {};
const timeouts = [];
global.setTimeout = (fn) => { timeouts.push(fn); return 0; };
global.clearTimeout = () => {};

/* map_view.js в этом прогоне не нужен: рисуем только «показать»-заглушку. */
global.RCMapView = {show() {}, draw() {}};
global.RCSessionUrl = (url) => url;

const file = path.join(ROOT, 'slam_gui', 'scan_mode.js');
vm.runInThisContext(fs.readFileSync(file, 'utf8'), {filename: file});

function lastPost(pattern) {
  return calls.filter((c) => c.method === 'POST' && pattern.test(c.url)).pop();
}

const flush = () => new Promise((resolve) => setImmediate(resolve));

(async () => {
  // 1. Исходное состояние: управление выключено, кнопки движения недоступны.
  await flush();
  await flush();
  assert.equal(byId('scan-state').textContent, 'выключено', 'экран: выключено');
  assert.equal(byId('scan-forward').disabled, true, 'вперёд заблокировано');
  assert.equal(byId('scan-back-arrow').disabled, true, 'назад заблокировано');
  assert.equal(byId('scan-enable').disabled, false, 'включить доступно');

  // 2. Нажатие «вперёд» при выключенном управлении ничего не отправляет.
  const before = calls.filter((c) => /scan_mode\/drive/.test(c.url)).length;
  byId('scan-forward').fire('pointerdown');
  await flush();
  assert.equal(calls.filter((c) => /scan_mode\/drive/.test(c.url)).length, before,
               'без управления тяга не уходит');

  // 3. Включаем управление.
  byId('scan-enable').fire('click');
  await flush();
  await flush();
  assert.equal(byId('scan-state').textContent, 'включено', 'экран: включено');
  assert.equal(byId('scan-forward').disabled, false, 'вперёд разблокировано');
  assert.ok(lastPost(/scan_mode\/enable/), 'запрос включения ушёл');

  // 4. Удержание кнопки: тяга уходит сразу и повторяется.
  byId('scan-forward').fire('pointerdown');
  await flush();
  const drive = lastPost(/scan_mode\/drive/);
  assert.ok(drive, 'тяга ушла на сервер');
  assert.equal(drive.body.percent, 8, 'тяга = ползунок');
  assert.equal(drive.headers['X-Scan-Token'], 'tok-1', 'команда несёт токен');

  // 5. Отпускание: нулевая тяга.
  byId('scan-forward').fire('pointerup');
  await flush();
  assert.equal(lastPost(/scan_mode\/drive/).body.percent, 0, 'отпустил — тяга 0');

  // 6. Esc снимает управление целиком (и тормозит).
  documentEvents.keydown.forEach((fn) => fn({key: 'Escape'}));
  await flush();
  await flush();
  assert.ok(lastPost(/scan_mode\/disable/), 'Esc выключил управление');
  assert.equal(byId('scan-state').textContent, 'выключено', 'экран снова выключено');

  // 7. Пока выключено, клавиатура не двигает робота.
  const afterEsc = calls.filter((c) => /scan_mode\/drive/.test(c.url) && c.body.percent !== 0).length;
  documentEvents.keydown.forEach((fn) => fn({key: 'w'}));
  await flush();
  assert.equal(calls.filter((c) => /scan_mode\/drive/.test(c.url) && c.body.percent !== 0).length,
               afterEsc, 'WASD не работает при выключенном управлении');

  // 8. «Стоп» отправляет нулевую тягу всегда.
  byId('scan-stop').fire('click');
  await flush();
  assert.equal(lastPost(/scan_mode\/drive/).body.percent, 0, 'стоп — всегда 0');

  // 9. Сохранение карты идёт на существующий маршрут карты.
  byId('scan-map-save').fire('click');
  await flush();
  await flush();
  assert.ok(lastPost(/kiosk_map\/save/), 'сохранение карты ушло');

  // 10. Камеры опрашивают сервер по относительному адресу. Кадр запрашивается
  //     из setTimeout, поэтому сначала «проигрываем» отложенные вызовы.
  timeouts.splice(0, timeouts.length).forEach((fn) => fn());
  await flush();
  assert.ok(imageRequests.length >= 4, 'камеры опрашиваются: ' + imageRequests.length);
  for (let n = 0; n < 4; n += 1) {
    assert.ok(imageRequests.some((url) => url.indexOf('api/camera/' + n + '/frame') === 0),
              'кадр камеры ' + n + ' запрошен');
  }
  assert.ok(imageRequests.every((url) => !/^(https?:|\/\/)/.test(url)),
            'камеры запрашиваются относительным адресом');

  // 11. Молчание сервера (enabled=false в статусе) гасит управление на экране.
  mode = {...mode, enabled: false, reason: 'deadman'};
  intervals.forEach((fn) => fn());
  await flush();
  await flush();
  assert.equal(byId('scan-state').textContent, 'выключено', 'сервер снял управление — экран это показал');

  console.log('scan screen: OK');
  process.exit(0);
})().catch((error) => {
  console.error(error && error.stack || error);
  process.exit(1);
});
