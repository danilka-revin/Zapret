'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const context = {window: {}};
vm.createContext(context);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../../slam_gui/wheel_angle.js'), 'utf8'), context);
const read = context.window.RCWheelAngle.read;
const valid = {online: true, homed: true, firmware: 26, pot: true, potMode: true,
  calibrating: false, fullZeroFault: false, angle: 2, potAngle: -8.25};
assert.equal(read(valid).value, -8.25);
assert.equal(read(valid).source, 'pot');
assert.equal(read(valid).label, 'потенциометр A0');
for (const patch of [{pot: false}, {potMode: false}, {homed: false}, {calibrating: true},
  {firmware: 25}, {fullZeroFault: true}, {fullZeroFault: undefined}, {potAngle: null}, {potAngle: NaN}, {potAngle: Infinity}, {potAngle: 181},
  {potAngle: '8'}, {potAngle: true}]) {
  const reading = read({...valid, ...patch});
  assert.equal(reading.value, 2, JSON.stringify(patch));
  assert.equal(reading.source, 'steps');
  assert.match(reading.label, /по шагам/);
}
assert.equal(read({...valid, online: false}).value, null);
assert.equal(read(valid, false).value, null);
assert.equal(read({...valid, pot: false, angle: null}).value, null);
console.log('A0, step fallback, invalid and stale readings passed');
