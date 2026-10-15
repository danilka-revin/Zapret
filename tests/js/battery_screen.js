'use strict';
const assert = require('assert');
const fs = require('fs');
const vm = require('vm');

class El {
  constructor() {
    this.children = []; this.textContent = ''; this.hidden = false; this.attrs = {};
    this.classes = new Set();
    this.classList = { toggle: (name, on) => on ? this.classes.add(name) : this.classes.delete(name) };
  }
  append(...items) { this.children.push(...items); }
  replaceChildren(...items) { this.children = items; }
  setAttribute(key, value) { this.attrs[key] = value; }
  removeAttribute(key) { delete this.attrs[key]; }
}
const elements = new Map();
const el = (id) => { if (!elements.has(id)) elements.set(id, new El()); return elements.get(id); };
const page = el('swipe-battery-view');
let observer;
let nextPoll;
let fail = false;
let sample = {
  updatedAt: Date.now() / 1000, soc: 50, volts: 51.2, amps: -12.5, watts: -640,
  remainingAh: 25, fullAh: 50, cycles: 123, temperatures: [25, 26],
  chargeOn: true, dischargeOn: true, faults: [],
  minMv: 3200, maxMv: 3231, deltaMv: 31, averageMv: 3215.5,
  cellCount: 32, receivedCount: 32, complete: true,
  cells: Array.from({ length: 32 }, (_, i) => ({ number: i + 1, mv: 3200 + i, deviationMv: i - 15.5, balancing: i === 31 }))
};
let status = { connected: true, complete: true, configured: true, reason: 'Все банки прочитаны', data: sample };
const context = {
  document: { getElementById: el, createElement: () => new El() },
  MutationObserver: class { constructor(fn) { observer = fn; } observe() {} },
  Date, AbortController,
  setTimeout: (fn, ms) => { if (ms === 1000) nextPoll = fn; return 1; }, clearTimeout() {},
  fetch: async (url) => {
    assert.equal(url, 'api/battery');
    if (fail) throw new Error('offline');
    return { ok: true, json: async () => ({ ok: true, battery: status }) };
  }
};
const flush = () => new Promise((resolve) => setImmediate(resolve));
(async () => {
  context.window = context;
  vm.runInNewContext(fs.readFileSync('slam_gui/battery.js', 'utf8'), context);
  await flush();
  assert.equal(el('bms-cells').children.length, 32);
  const last = el('bms-cells').children[31].children;
  assert.equal(last[0].textContent, '32');
  assert.equal(last[1].textContent, '3.231 В · 3231 мВ');
  assert.equal(last[3].textContent, 'Активна');
  assert.equal(last[4].textContent, 'МАКС');
  assert.equal(el('bms-report').attrs['aria-disabled'], 'false');
  assert.equal(el('swipe-live').textContent, '● BMS');
  fail = true;
  await nextPoll();
  assert.equal(el('bms-cells').children.length, 32); // retained, but clearly stale
  assert(page.classes.has('bms-stale'));
  assert.equal(el('bms-report').attrs['aria-disabled'], 'true');
  assert.equal(el('bms-report').attrs.href, undefined);
  assert.match(el('bms-status').textContent, /Нет связи/);
  assert.equal(el('swipe-live').textContent, 'НЕТ ДАННЫХ BMS');
  fail = false;
  sample.cells[31].mv = null;
  sample.receivedCount = 31; sample.complete = false;
  status.complete = false; status.reason = 'Неполный отчёт';
  await nextPoll();
  assert.equal(el('bms-cells').children[31].children[4].textContent, 'НЕТ ДАННЫХ');
  assert.equal(el('swipe-live').textContent, 'НЕ ВСЕ БАНКИ');
  page.hidden = true;
  el('swipe-live').textContent = '● LIVE';
  observer();
  assert.equal(el('swipe-live').textContent, '● LIVE'); // do not overwrite camera status
  status = { connected: false, configured: false, complete: false, reason: 'BMS не настроена', data: null };
  await nextPoll();
  assert.equal(el('bms-cells').children.length, 1);
  assert.equal(el('bms-cells').children[0].children[0].textContent, 'Нет данных от BMS');
  assert.equal(el('bms-summary').children.length, 0);
  console.log('Battery page: all banks, missing banks, stale state and CSV controls OK');
})().catch((error) => { console.error(error); process.exitCode = 1; });
