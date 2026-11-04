/* Browser-free contract test for the shared potentiometer card. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

class Element {
  constructor(tag) {
    this.tagName = tag.toUpperCase();
    this.children = [];
    this.attributes = {};
    this.listeners = {};
    this.textContent = '';
    this.disabled = false;
    this.value = '';
  }
  setAttribute(key, value) { this.attributes[key] = String(value); }
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  set innerHTML(value) {
    this.markup = value;
    this.select = new Element('select');
    this.button = new Element('button');
    this.status = new Element('span');
    this.alert = new Element('span');
  }
  querySelector(selector) {
    return {'select': this.select, 'button': this.button,
      '[role="status"]': this.status, '[role="alert"]': this.alert}[selector] || null;
  }
  addEventListener(name, callback) { this.listeners[name] = callback; }
  fire(name) { if (this.listeners[name]) this.listeners[name]({target: this}); }
}

global.window = global;
global.document = {createElement: tag => new Element(tag)};
let now = 1000000;
Date.now = () => now;
let agree = true;
global.confirm = () => agree;
const requests = [];
let nextResponse = {ok: true};
global.fetch = async (url, options) => {
  requests.push({url: String(url), method: options.method,
    header: options.headers['X-Robot-Control'], action: JSON.parse(options.body).action});
  const result = nextResponse;
  nextResponse = {ok: true};
  return {ok: result.ok, status: result.ok ? 200 : 409, json: async () => result};
};
vm.runInThisContext(fs.readFileSync(path.join(__dirname, '../../slam_gui/pot_mode.js'), 'utf8'));

const ids = ['FL', 'FR', 'RL', 'RR'];
const motors = (overrides = {}) => ids.map(id => ({id, online: true, potMode: true,
  pot: true, homed: true, moving: false, calibrating: false, cycling: false,
  drivePercent: 0, ...overrides}));
const state = (overrides = {}) => ({motors: motors(), estop: {latched: false, unknown: false},
  neutral: {locked: false}, ...overrides});
const parent = new Element('div');
const render = (data, authorized = true) => window.RCPotMode.render(parent, data, authorized);
const flush = () => new Promise(resolve => setImmediate(resolve));

(async () => {
  render(state());
  const host = parent.children[0];
  const {select, button, status, alert} = host;
  assert.equal(host.className, 'calibration-card pot-mode');
  assert.match(host.markup, /aria-label="Режим потенциометра"/);
  assert.equal(select.value, 'on');
  assert.equal(select.disabled, false);
  assert.equal(button.disabled, false);
  assert.match(status.textContent, /датчики откалиброваны/);
  assert.doesNotMatch(status.textContent, /доводка включена/);

  const blocked = [
    [state({motors: motors({online: false})}), true, /связь со всеми/],
    [state({motors: motors({potMode: null})}), true, /прошивка/],
    [state({motors: motors({moving: true})}), true, /движение/],
    [state({estop: {latched: true}}), true, /E-STOP/],
    [state({estop: {unknown: true}}), true, /E-STOP/],
    [state({neutral: {locked: true}}), true, /тормоз/],
    [state(), false, /PIN/],
  ];
  for (const [data, authorized, reason] of blocked) {
    render(data, authorized);
    assert.equal(select.disabled, true);
    assert.equal(button.disabled, true);
    assert.match(status.textContent, reason);
  }
  render(state({motors: motors({homed: false})}));
  assert.equal(select.disabled, false);
  assert.equal(button.disabled, true);
  assert.match(status.textContent, /ноль всех колёс/);
  render(state({motors: motors({pot: false})}));
  assert.match(status.textContent, /нужна калибровка датчиков/);
  const mixed = motors(); mixed[0].potMode = false;
  render(state({motors: mixed}));
  assert.equal(select.value, 'unknown');
  assert.match(status.textContent, /режимы плат различаются/i);

  // A command response only says it was queued. Old or partly changed board
  // telemetry must never make the selected mode look confirmed.
  render(state());
  select.value = 'off'; select.fire('change'); await flush();
  assert.deepEqual(requests[0], {url: 'api/kiosk_pot_mode', method: 'POST', header: 'steering', action: 'off'});
  render(state());
  assert.equal(select.value, 'on');
  assert.equal(select.disabled, true);
  assert.match(status.textContent, /Ожидание подтверждения/);
  render(state({motors: mixed}));
  assert.equal(select.value, 'unknown');
  assert.match(status.textContent, /Ожидание подтверждения/);
  render(state({motors: motors({potMode: false})}));
  assert.equal(select.value, 'off');
  assert.equal(select.disabled, false);
  assert.equal(button.disabled, true);
  assert.match(status.textContent, /Подтверждено: без потенциометра/);

  // Calibration is possible only with homed wheels and an explicit warning.
  render(state());
  agree = false; button.fire('click'); await flush();
  assert.equal(requests.length, 1);
  agree = true; button.fire('click'); await flush();
  assert.equal(requests.length, 2);
  assert.equal(requests[1].action, 'calibrate');

  nextResponse = {ok: false, reason: 'module_busy'};
  select.value = 'off'; select.fire('change'); await flush();
  assert.match(alert.textContent, /остановите тягу/);
  render(state());
  select.value = 'off'; select.fire('change'); await flush();
  now += 6000; render(state());
  assert.equal(select.value, 'on');
  assert.match(alert.textContent, /не подтвердили режим/);
  assert.equal(parent.children.length, 1);
  console.log('potentiometer modes, blocking, confirmation and telemetry passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
