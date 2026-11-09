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
const spinInfo = new EventTarget('keyboard-spin-geometry');
const elements = Object.fromEntries([panel, button, mode, slider, status, stopButton, output, spinInfo].map(el => [el.id, el]));
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
const pivot = Math.atan2(500, 380) * 180 / Math.PI;
const spin = {angleDeg: pivot, wheelbaseMm: 500, trackMm: 380,
  angles: {FL: pivot, FR: -pivot, RL: -pivot, RR: pivot}};
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
    result = {ok: true, session: 'session-' + (++session), brakeId, spin};
    if (holdBegin) { const gate = holdBegin; holdBegin = null; await gate.promise; }
  } else if (body.action === 'stop' || body.action === 'end') {
    if (body.action === 'stop' && holdStop) { const gate = holdStop; holdStop = null; await gate.promise; }
    brake(); result.brakeId = brakeId;
  } else if (body.action === 'steer_all') {
    for (const motor of motors) motor.angle = body.angles[motor.id] + (body.pattern === 'spin' ? 0.6 : 0);
  } else if (body.action === 'arm') {
    if (holdArm) { const gate = holdArm; holdArm = null; await gate.promise; }
    result.permit = 'permit';
  } else if (body.action === 'live') {
    telemetry.drive.active = true;
    for (const motor of motors) Object.assign(motor, {
      drivePercent: body.pattern === 'spin' && ['FR', 'RR'].includes(motor.id) ? -body.percent : body.percent,
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

  // The horizontal arrows use server geometry and require their own permit.
  assert.match(spinInfo.textContent, /52\.77°/);
  down('ArrowLeft');
  await until(() => telemetry.drive.active && motors[0].drivePercent === -20, 'left pivot');
  assert.deepEqual(motors.map(m => m.drivePercent), [-20, 20, -20, 20]);
  assert.ok(calls.some(c => c.action === 'arm' && c.pattern === 'spin' && c.direction === -1));
  assert.ok(calls.some(c => c.action === 'live' && c.pattern === 'spin' && c.percent === -20));
  up('ArrowLeft');
  await until(() => !telemetry.drive.active, 'left released');

  // Releasing before a spin permit returns cannot start an old rotation.
  const spinGate = deferred(); holdArm = spinGate;
  const oldArms = count('arm'), oldLives = count('live');
  down('ArrowRight');
  await until(() => count('arm') === oldArms + 1, 'delayed pivot ARM');
  up('ArrowRight');
  await until(() => calls.some(c => c.action === 'stop' && c.sequence > calls.findLast(x => x.action === 'arm').sequence), 'pivot STOP');
  spinGate.resolve();
  await sleep(150);
  assert.equal(count('live'), oldLives, 'late pivot permit resumed motion');
  assert.equal(telemetry.drive.active, false);

  // Incompatible arrows brake and require all of them to be released.
  down('ArrowUp');
  await until(() => telemetry.drive.active, 'forward before conflict');
  down('ArrowLeft');
  await until(() => !telemetry.drive.active, 'arrow conflict');
  const conflictLives = count('live');
  up('ArrowUp');
  await sleep(200);
  assert.equal(count('live'), conflictLives, 'still-held left arrow started after conflict');
  up('ArrowLeft');
  down('ArrowRight');
  await until(() => telemetry.drive.active && motors[0].drivePercent === 20 && motors[1].drivePercent === -20, 'right pivot');
  up('ArrowRight');
  await until(() => !telemetry.drive.active, 'right released');
  await keyboard.disable();

  // Ctrl+R toggles the mode exactly like the on-screen button and is
  // swallowed so the page never reloads mid-drive.
  const beginsBeforeToggle = count('begin');
  const hotkey = repeat => ({code: 'KeyR', key: 'r', ctrlKey: true, target: document.body,
    repeat: !!repeat, prevented: false, preventDefault() { this.prevented = true; }});
  const swallowed = hotkey();
  window.emit('keydown', swallowed);
  assert.equal(swallowed.prevented, true, 'Ctrl+R must be swallowed, not reload');
  await until(() => count('begin') === beginsBeforeToggle + 1, 'Ctrl+R enable');
  await until(() => button.getAttribute('aria-pressed') === 'true', 'Ctrl+R enabled');
  const held = hotkey(true);
  window.emit('keydown', held);
  assert.equal(held.prevented, true, 'repeated Ctrl+R is swallowed');
  assert.equal(count('begin'), beginsBeforeToggle + 1, 'held Ctrl+R flapped the mode');
  const toggledOff = hotkey();
  window.emit('keydown', toggledOff);
  await until(() => button.getAttribute('aria-pressed') === 'false', 'Ctrl+R disable');

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
