'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

class Element {
  constructor(dataset = {}) { this.dataset = dataset; this.handlers = {}; this.disabled = false;
    this.value = '8'; this.max = '100'; this.textContent = ''; }
  addEventListener(name, callback) { (this.handlers[name] ||= []).push(callback); }
  fire(name, event = {}) {
    const e = Object.assign({button: 0, pointerId: 1, preventDefault() {}}, event);
    for (const fn of this.handlers[name] || []) fn(e);
  }
}
const ids = ['ALL', 'FL', 'FR', 'RL', 'RR'];
const elements = new Map();
const buttons = [], stops = [];
for (const id of ids) {
  for (const part of ['speed', 'value', 'state']) elements.set('remote-drive-' + part + '-' + id, new Element());
  for (const direction of [-1, 1]) buttons.push(new Element({remoteDrive: id, direction: String(direction)}));
  stops.push(new Element({remoteStop: id}));
}
elements.set('remote-drive-status', new Element());
const events = {};
const documentEvents = {};
const document = {
  body: {dataset: {remoteView: 'motors'}}, hidden: false,
  getElementById(id) { return elements.get(id) || null; },
  querySelectorAll(selector) {
    const match = selector.match(/^\[data-remote-drive="(\w+)"\]$/);
    if (match) return buttons.filter(b => b.dataset.remoteDrive === match[1]);
    if (selector === '[data-remote-drive]') return buttons;
    if (selector === '[data-remote-stop]') return stops;
    return [];
  },
  addEventListener(name, callback) { (documentEvents[name] ||= []).push(callback); }
};
const calls = [];
let holdNext = false, resolveHeld = null;
async function fetch(url, init) {
  const body = JSON.parse(init.body);
  calls.push(body);
  if (holdNext && body.percent) {
    holdNext = false;
    await new Promise(resolve => { resolveHeld = resolve; });
  }
  return {ok: true, json: async () => ({ok: true})};
}
let nextTimer = 1;
const intervals = new Map();
const window = {RCRemoteControls: true, addEventListener(name, callback) { (events[name] ||= []).push(callback); }};
const context = {window, document, fetch, AbortController, Date, Promise, Map, Object,
  setTimeout, clearTimeout,
  setInterval(fn) { const id = nextTimer++; intervals.set(id, fn); return id; },
  clearInterval(id) { intervals.delete(id); }};
vm.createContext(context);
vm.runInContext(fs.readFileSync(path.resolve(__dirname, '../../slam_gui/web_drive.js'), 'utf8'), context);
const settle = () => new Promise(resolve => setImmediate(resolve));
const button = (id, direction) => buttons.find(b => b.dataset.remoteDrive === id && +b.dataset.direction === direction);
const ready = () => ({linkOk: true, estop: null, drive: {available: true, canDrive: true, maxPercent: 100,
  perModule: Object.fromEntries(ids.slice(1).map(id => [id, {canDrive: true}]))}});

(async () => {
  // Старый сервер/прошивка без drive.servo всё равно может управляться удалённо.
  window.RCRemoteLegacyState(ready());
  assert(!button('ALL', 1).disabled);
  button('ALL', 1).fire('pointerdown', {pointerId: 1});
  await settle();
  assert.deepEqual(calls.at(-1), {percent: 8});
  button('ALL', 1).fire('pointerup');
  await settle();
  assert.deepEqual(calls.at(-1), {percent: 0});

  button('ALL', -1).fire('pointerdown', {pointerId: 2});
  await settle();
  assert.deepEqual(calls.at(-1), {percent: -8});
  button('ALL', -1).fire('pointerup');
  await settle();
  assert.deepEqual(calls.at(-1), {percent: 0});

  elements.get('remote-drive-speed-FR').value = '20';
  button('FL', 1).fire('pointerdown', {pointerId: 3});
  button('FR', -1).fire('pointerdown', {pointerId: 4});
  await settle(); await settle();
  assert(calls.some(call => call.module === 'FL' && call.percent === 8));
  assert(calls.some(call => call.module === 'FR' && call.percent === -20));
  button('FL', 1).fire('pointerup');
  await settle();
  assert.deepEqual(calls.at(-1), {module: 'FL', percent: 0});
  for (const fn of intervals.values()) fn();
  await settle();
  assert.deepEqual(calls.at(-1), {module: 'FR', percent: -20});
  button('FR', -1).fire('pointerup');
  await settle();
  assert.deepEqual(calls.at(-1), {module: 'FR', percent: 0});

  button('ALL', 1).fire('pointerdown', {pointerId: 5});
  await settle();
  const lost = ready(); lost.linkOk = false;
  window.RCRemoteLegacyState(lost);
  await settle();
  assert.deepEqual(calls.at(-1), {percent: 0});
  const length = calls.length;
  window.RCRemoteLegacyState(ready());
  for (const fn of intervals.values()) fn();
  await settle();
  assert.equal(calls.length, length); // восстановление связи не возобновляет тягу

  // Нажатие STOP при незавершённом запросе вытесняет ожидающие heartbeat.
  holdNext = true;
  button('ALL', 1).fire('pointerdown', {pointerId: 6});
  await settle();
  for (const fn of intervals.values()) fn();
  window.RCRemoteLegacyStop();
  resolveHeld();
  await settle(); await settle();
  assert.deepEqual(calls.at(-1), {percent: 0});
  assert(!intervals.size);
  assert(calls.every(body => !('value' in body)));
  console.log('remote legacy drive passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
