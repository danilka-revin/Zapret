'use strict';
/* Пресеты руления: знаки, пределы и честная недоступность разворота. */
const assert = require('node:assert/strict');
const presets = require('../../slam_gui/steer_presets.js');

const ids = ['FL', 'FR', 'RL', 'RR'];

assert.deepEqual(Array.from(presets.IDS), ids);
assert.equal(presets.LIMIT_DEG, 32);
assert.equal(presets.PRESETS.length, 4);

// Прошивка: плюс — вправо, минус — влево.
const straight = presets.anglesFor('straight');
assert.deepEqual(Object.keys(straight).sort(), ids.slice().sort());
ids.forEach((id) => assert.equal(straight[id], 0));

const left = presets.anglesFor('crab-left');
ids.forEach((id) => assert.equal(left[id], -presets.CRAB_DEG));
const right = presets.anglesFor('crab-right');
ids.forEach((id) => assert.equal(right[id], presets.CRAB_DEG));

// Разворот на месте одной тягой невозможен: пресет не должен делать вид, что он есть.
const pivot = presets.byId.pivot;
assert.equal(pivot.available, false);
assert.equal(presets.anglesFor('pivot'), null);
assert.match(pivot.reason, /одна тяга/);
assert.match(pivot.reason, /борт/);

assert.equal(presets.clampAngle(99), 32);
assert.equal(presets.clampAngle(-99), -32);
assert.equal(presets.clampAngle('12.6'), 13);
assert.equal(presets.clampAngle('abc'), null);
assert.equal(presets.uniformAngle({ FL: 5, FR: 5, RL: 5, RR: 6 }), 5);
assert.equal(presets.uniformAngle({ FL: 5, FR: 5, RL: 5, RR: 8 }), null);
assert.equal(presets.uniformAngle(null), null);
assert.match(presets.describe(-28), /влево 28/);
assert.match(presets.describe(28), /вправо 28/);
assert.match(presets.describe(0), /прямо/);
assert.match(presets.modeFor(-28), /Краб влево/);
assert.match(presets.modeFor(0), /Прямо/);

console.log('Steering presets signs, limits and pivot availability: OK');
