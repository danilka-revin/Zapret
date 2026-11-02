/* Dependency-free state-machine regression; real browser/PTy tests complement it. */
const assert = require('node:assert/strict');

class EventTarget {
  constructor(id, editable = false) {
    this.id = id; this.editable = editable; this.handlers = {}; this.attrs = {}; this.dataset = {};
    this.value = ''; this.textContent = '';
  }
  addEventListener(type, callback) { (this.handlers[type] ||= []).push(callback); }
  emit(type, event = {}) { for (const callback of this.handlers[type] || []) callback(event); }
  setAttribute(name, value) { this.attrs[name] = value; }
  getAttribute(name) { return this.attrs[name]; }
  closest(selector) {
    if (selector === '#keyboard-panel') return this === panel || this.inPanel ? panel : null;
    return this.editable ? this : null;
  }
  focus() { document.activeElement = this; }
  blur() { if (document.activeElement === this) document.activeElement = document.body; this.emit('blur', {}); }
}
const panel = new EventTarget('keyboard-panel');
const button = new EventTarget('keyboard-enable');
const mode = new EventTarget('keyboard-mode', true); mode.value = 'crab'; mode.inPanel = true;
const slider = new EventTarget('keyboard-speed', true); slider.value = '20'; slider.inPanel = true;
const status = new EventTarget('keyboard-status');
const stopButton = new EventTarget('keyboard-stop');
const output = new EventTarget('keyboard-speed-value');
const elements = Object.fromEntries([panel, button, mode, slider, status, stopButton, output].map(el => [el.id, el]));
global.document = {hidden: false, body: new EventTarget('body'), activeElement: button,
  getElementById: id => elements[id], addEventListener() {}};
global.window = new EventTarget('window');
const {create} = require('../../slam_gui/keyboard.js');
const ids = ['FL', 'FR', 'RL', 'RR'];
const motors = ids.map(id => ({id, online: true, homed: true, telemetryAgeS: 0,
  calibrating: false, cycling: false, moving: false, angle: 0, drivePercent: 0,
  scRpm: 0, scRecent: false, firmware: 23, brakeId: 0, liveId: 0, liveSeq: 0}));
const telemetry = {linkOk: true, steeringControl: {available: true}, drive: {available: true, active: false}, motors};
const calls = [];
let brakeId = 0, session = 0, incompleteFirstAck = true, holdStop = null, holdArm = null, holdBegin = null;
function deferred() {
  let resolve;
  const promise = new Promise(done => { resolve = done; });
  return {promise, resolve};
}
function brake() {
  telemetry.drive.active = false;
  ++brakeId;
  for (const motor of motors) Object.assign(motor, {moving: false, drivePercent: 0,
    scRpm: 0, scRecent: false, brakeId, liveId: 0, liveSeq: 0});
}
global.fetch = async (_url, init) => {
  const body = JSON.parse(init.body);
  calls.push(body);
  let result = {ok: true};
  if (body.action === 'begin') {
    brake();
    if (incompleteFirstAck) motors[3].brakeId = 0;
    result = {ok: true, session: 'session-' + (++session), brakeId};
    if (holdBegin) { const gate = holdBegin; holdBegin = null; await gate.promise; }
  } else if (body.action === 'stop' || body.action === 'end') {
    if (body.action === 'stop' && holdStop) { const gate = holdStop; holdStop = null; await gate.promise; }
    brake(); result.brakeId = brakeId;
  } else if (body.action === 'steer_all') {
    for (const motor of motors) motor.angle = body.angles[motor.id];
  } else if (body.action === 'arm') {
    if (holdArm) { const gate = holdArm; holdArm = null; await gate.promise; }
    result.permit = 'permit';
  } else if (body.action === 'live') {
    telemetry.drive.active = true;
    for (const motor of motors) Object.assign(motor, {drivePercent: body.percent,
      liveId: brakeId, liveSeq: body.sequence, angle: body.angles[motor.id]});
  }
  return {ok: true, json: async () => result};
};
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
async function until(condition, label, timeout = 2500) {
  const end = Date.now() + timeout;
  while (Date.now() < end) { if (condition()) return; await sleep(10); }
  throw new Error(label + ': ' + JSON.stringify({calls: calls.slice(-15), status: status.textContent}));
}
function key(type, code, target = document.activeElement) {
  const event = {code, key: code.startsWith('Key') ? code.slice(3).toLowerCase() : code,
    target, repeat: false, preventDefault() {}, stopPropagation() { this.stopped = true; }};
  if (type === 'keydown' && target.inPanel) {
    target.emit(type, event);
    panel.emit(type, event);
  }
  if (!event.stopped) window.emit(type, event);
}
const down = code => key('keydown', code);
const up = code => key('keyup', code);
const count = action => calls.filter(call => call.action === action).length;

(async () => {
  const keyboard = create({getState: () => telemetry, freshAt: () => Date.now(), stopManual: async () => {}});
  button.emit('click');
  await until(() => button.getAttribute('aria-pressed') === 'true', 'enable');
  down('ArrowUp');
  await sleep(100);
  assert.equal(count('arm'), 0, 'must wait for all four matching brake ACKs');
  assert.equal(count('live'), 0);
  motors[3].brakeId = brakeId;
  incompleteFirstAck = false;
  await until(() => telemetry.drive.active, 'first start');

  // Many input changes while one STOP is delayed share precisely that STOP.
  const gate = deferred(); holdStop = gate;
  const before = count('stop');
  up('ArrowUp');
  await until(() => count('stop') === before + 1, 'STOP request');
  down('ArrowUp'); up('ArrowUp'); down('ArrowUp');
  await sleep(100);
  assert.equal(count('stop'), before + 1, 'overlapping STOPs');
  gate.resolve();
  await until(() => count('arm') >= 2 && telemetry.drive.active, 'restart');
  await sleep(150);
  assert.equal(button.getAttribute('aria-pressed'), 'true');

  // Release before a pending ARM reply: a fresh STOP revokes it; no live may
  // be sent when the delayed reply eventually comes back.
  up('ArrowUp');
  await until(() => !telemetry.drive.active, 'release');
  const armGate = deferred(); holdArm = armGate;
  const beforeArm = count('arm'), beforeLive = count('live');
  down('ArrowUp');
  await until(() => count('arm') === beforeArm + 1, 'delayed ARM');
  up('ArrowUp');
  await until(() => count('stop') >= before + 3, 'STOP while ARM pending');
  armGate.resolve();
  await sleep(200);
  assert.equal(count('live'), beforeLive, 'late ARM resumed motion');
  assert.equal(telemetry.drive.active, false);

  // Editable arrows never drive; Esc exits only WASD settings.
  slider.focus();
  const beforeEdit = count('live');
  down('ArrowUp'); up('ArrowUp');
  await sleep(80);
  assert.equal(count('live'), beforeEdit);
  key('keydown', 'Escape');
  assert.notEqual(document.activeElement, slider);
  down('ArrowUp');
  await until(() => telemetry.drive.active, 'drive after leaving slider');
  up('ArrowUp');
  await until(() => !telemetry.drive.active, 'final release');
  await keyboard.disable();

  // A caller reserving the hardware must be able to drain a BEGIN that was
  // cancelled while its reply was still in flight, including its late END.
  const beginGate = deferred(); holdBegin = beginGate;
  const beginBefore = count('begin'), endBefore = count('end');
  button.emit('click');
  await until(() => count('begin') === beginBefore + 1, 'in-flight BEGIN');
  let drained = false;
  const drain = keyboard.disable().then(() => { drained = true; });
  await sleep(80);
  assert.equal(drained, false, 'disable returned while a BEGIN was still in flight');
  beginGate.resolve();
  await drain;
  assert.equal(count('end'), endBefore + 1, 'late BEGIN was not retired before drain');
  console.log('keyboard runtime races OK');
})().catch(error => { console.error(error); process.exit(1); });
