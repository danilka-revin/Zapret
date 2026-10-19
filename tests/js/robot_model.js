'use strict';
const assert = require('node:assert/strict');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const stored = {};
const events = {};
const localStorage = { getItem: key => stored[key] || null, setItem: (key, value) => { stored[key] = value; } };
const sandbox = { window: { addEventListener: (name, callback) => { events[name] = callback; } }, localStorage, Date, Math, Number, String, Array };
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../../slam_gui/robot_model.js'), 'utf8'), sandbox);
const model = sandbox.window.RCRobotModel;
const angles = [24, -18, 7.5, -3];
const payload = {
  linkOk: true, source: 'master', speedMps: .25, drive: { active: true },
  motors: ['RR', 'FL', 'RL', 'FR'].map(id => ({
    id, angle: angles[['FL', 'FR', 'RL', 'RR'].indexOf(id)], targetAngle: 80,
    online: true, homed: true
  }))
};
model.update(payload, 10000);
const live = model.state(10001);
assert.deepEqual(Array.from(live.wheels, w => w.angle), angles);
assert.equal(live.driving, true);
assert.equal(live.speed, .25);

const operations = [];
const ctx = new Proxy({}, {
  get: (_, key) => (...args) => { operations.push([key, ...args]); },
  set: (_, key, value) => { operations.push(['set', key, value]); return true; }
});
model.draw(ctx, 100, 200, Math.PI / 3, 100, { state: live });
const rotations = operations.filter(op => op[0] === 'rotate').map(op => op[1]);
assert.equal(rotations[0], -Math.PI / 3);
assert.equal(rotations.length, 9);
// Прошивка: +deg — вправо (по часовой стрелке сверху), canvas rotate(+a)
// тоже по часовой. Фактические и целевые углы рисуются с тем же знаком.
angles.forEach((angle, i) => {
  assert.ok(Math.abs(rotations[2 + i * 2] - angle * Math.PI / 180) < 1e-10);
  assert.ok(Math.abs(rotations[1 + i * 2] - 80 * Math.PI / 180) < 1e-10);
});
assert.ok(operations.some(op => op[0] === 'set' && op[1] === 'strokeStyle' && op[2] === '#36c275'));
assert.ok(operations.some(op => op[0] === 'set' && op[1] === 'fillStyle' && op[2] === '#e35d6a'));
const locations = operations.filter(op => op[0] === 'translate').slice(1);
assert.deepEqual(locations, [
  ['translate', 27.500000000000004, -25], ['translate', 27.500000000000004, 25],
  ['translate', -27.500000000000004, -25], ['translate', -27.500000000000004, 25]
]);

const stale = model.state(10000 + model.STALE_MS + 1);
assert.equal(stale.fresh, false);
assert.equal(stale.driving, false);
assert.equal(stale.speed, null);
assert.ok(stale.wheels.every(w => w.angle === null && w.targetAngle === null));
assert.match(model.summary(stale).rows[0], /FL —/);

payload.motors.find(w => w.id === 'FL').angle = null;
payload.motors.find(w => w.id === 'FR').online = false;
payload.motors.find(w => w.id === 'RL').angle = 'invalid';
payload.motors.find(w => w.id === 'RR').angle = Infinity;
model.update(payload, 20000);
assert.ok(model.state(20001).wheels.every(w => w.angle === null));
payload.linkOk = false;
model.update(payload, 20000);
assert.equal(model.state(20001).fresh, false);

payload.linkOk = true;
payload.estop = { latched: true };
payload.source = 'demo';
payload.motors = [{ id: 'FL', angle: 12, online: true, homed: false, calibrating: true }];
model.update(payload, 30000);
const stopped = model.state(30001);
assert.equal(stopped.driving, false);
assert.equal(stopped.wheels[0].calibrating, true);
assert.equal(stopped.wheels[0].homed, false);
assert.match(model.summary(stopped).status, /ДЕМО · E-STOP/);
assert.match(model.summary(stopped).rows[0], /калибр/);
// Missing target never falls back to actual; missing actual can retain a valid target.
payload.motors = [
  { id: 'FL', angle: 12, online: true, homed: true },
  { id: 'FR', angle: null, targetAngle: -23, online: true, homed: true }
];
model.update(payload, 40000);
const partial = model.state(40001);
assert.equal(partial.wheels[0].targetAngle, null);
assert.equal(partial.wheels[1].angle, null);
assert.equal(partial.wheels[1].targetAngle, -23);
assert.equal(partial.wheels[1].targetRadians, -23 * Math.PI / 180);

let changed = 0;
model.subscribe(() => changed++);
model.setColors({ target: '#123ABC', actual: '#abcdef' });
assert.equal(model.getColors().target, '#123abc');
assert.equal(model.getColors().actual, '#abcdef');
assert.equal(changed, 1);
const saved = JSON.parse(stored.rc_robot_wheel_colors_v1);
assert.equal(saved.target, '#123abc');
const colorsCopy = model.getColors(); colorsCopy.target = 'bad';
assert.equal(model.getColors().target, '#123abc');
model.setColors({ actual: 'invalid' });
assert.equal(model.getColors().actual, model.DEFAULT_COLORS.actual);
assert.equal(model.getColors().target, '#123abc');

// Reload uses persisted settings; storage events synchronize other tabs.
const reloaded = { window: {}, localStorage };
vm.createContext(reloaded);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../../slam_gui/robot_model.js'), 'utf8'), reloaded);
assert.equal(reloaded.window.RCRobotModel.getColors().target, '#123abc');
stored.rc_robot_wheel_colors_v1 = JSON.stringify({ target: '#111111', actual: '#222222' });
events.storage({ key: 'rc_robot_wheel_colors_v1' });
assert.equal(model.getColors().actual, '#222222');
stored.rc_robot_wheel_colors_v1 = '{broken';
events.storage({ key: 'rc_robot_wheel_colors_v1' });
assert.equal(model.getColors().actual, model.DEFAULT_COLORS.actual);

// Both settings panels use these bindings, which update immediately and reset together.
const controls = Object.fromEntries(['target', 'actual', 'reset'].map(id => [id, {
  value: '', events: {}, addEventListener(name, callback) { this.events[name] = callback; }
}]));
sandbox.document = { getElementById: id => controls[id] };
model.bindColors('target', 'actual', 'reset');
assert.equal(controls.target.value, model.DEFAULT_COLORS.target);
controls.target.value = '#ff8800'; controls.target.events.input();
assert.equal(model.getColors().target, '#ff8800');
model.setColors({ actual: '#335577' });
assert.equal(controls.actual.value, '#335577');
controls.reset.events.click();
assert.equal(model.getColors().target, model.DEFAULT_COLORS.target);
assert.equal(model.getColors().actual, model.DEFAULT_COLORS.actual);
delete sandbox.localStorage;
assert.doesNotThrow(() => model.setColors({ actual: '#123456' }));
console.log('Robot actual/target angles, transforms and color settings: OK');
