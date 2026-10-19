'use strict';
/* Пресеты руления: знаки, пределы, боковой ход и честная недоступность разворота. */
const assert = require('node:assert/strict');
const presets = require('../../slam_gui/steer_presets.js');

const ids = ['FL', 'FR', 'RL', 'RR'];

assert.deepEqual(Array.from(presets.IDS), ids);
assert.equal(presets.LIMIT_DEG, 90);
assert.equal(presets.limit(), 90);
assert.equal(presets.limitText(), '±90°');
// Пресетов больше, чем было: прямо, подруливание, краб и чистый боковой ход
// в обе стороны + честно недоступный разворот.
assert.equal(presets.PRESETS.length, 8);
assert.deepEqual(presets.PRESETS.map((p) => p.id), [
  'straight', 'veer-left', 'veer-right', 'crab-left', 'crab-right',
  'side-left', 'side-right', 'pivot'
]);

// Прошивка: плюс — вправо, минус — влево.
const straight = presets.anglesFor('straight');
assert.deepEqual(Object.keys(straight).sort(), ids.slice().sort());
ids.forEach((id) => assert.equal(straight[id], 0));

const veerLeft = presets.anglesFor('veer-left');
ids.forEach((id) => assert.equal(veerLeft[id], -presets.VEER_DEG));
const veerRight = presets.anglesFor('veer-right');
ids.forEach((id) => assert.equal(veerRight[id], presets.VEER_DEG));

const left = presets.anglesFor('crab-left');
ids.forEach((id) => assert.equal(left[id], -presets.CRAB_DEG));
const right = presets.anglesFor('crab-right');
ids.forEach((id) => assert.equal(right[id], presets.CRAB_DEG));

// Боковой ход: колёса перпендикулярно корпусу, знак — как у прошивки.
const sideLeft = presets.anglesFor('side-left');
ids.forEach((id) => assert.equal(sideLeft[id], -90));
const sideRight = presets.anglesFor('side-right');
ids.forEach((id) => assert.equal(sideRight[id], 90));

// Разворот на месте одной тягой невозможен: пресет не должен делать вид, что он есть.
const pivot = presets.byId.pivot;
assert.equal(pivot.available, false);
assert.equal(presets.anglesFor('pivot'), null);
assert.match(pivot.reason, /одна тяга/);
assert.match(pivot.reason, /борт/);

assert.equal(presets.clampAngle(150), 90);
assert.equal(presets.clampAngle(-150), -90);
assert.equal(presets.clampAngle(45), 45);
assert.equal(presets.clampAngle('12.6'), 13);
assert.equal(presets.clampAngle('abc'), null);
assert.equal(presets.uniformAngle({ FL: 5, FR: 5, RL: 5, RR: 6 }), 5);
assert.equal(presets.uniformAngle({ FL: 5, FR: 5, RL: 5, RR: 8 }), null);
assert.equal(presets.uniformAngle(null), null);
assert.match(presets.describe(-45), /влево 45/);
assert.match(presets.describe(45), /вправо 45/);
assert.match(presets.describe(-90), /боком влево 90/);
assert.match(presets.describe(0), /прямо/);
assert.match(presets.modeFor(-45), /Краб влево/);
assert.match(presets.modeFor(45), /Краб вправо/);
assert.match(presets.modeFor(-90), /Боком влево/);
assert.match(presets.modeFor(90), /Боком вправо/);
assert.match(presets.modeFor(0), /Прямо/);

// Предел приходит с сервера: старая прошивка режет ход — пресеты на 90° честно
// помечаются недоступными, а не уезжают молча на 32°.
assert.equal(presets.setLimit(32), 32);
assert.equal(presets.limitText(), '±32°');
assert.equal(presets.clampAngle(90), 32);
const limited = presets.get('side-right');
assert.equal(limited.available, false);
assert.match(limited.reason, /ограничен ±32°/);
assert.equal(presets.anglesFor('side-right'), null);
assert.equal(presets.get('crab-left').available, false);   // 45° тоже вне ±32°
assert.equal(presets.list().filter((p) => p.available).length, 3);
assert.equal(presets.get('straight').available, true);
assert.equal(presets.get('veer-right').available, true);

// Больше механического упора (±125°) предел выставить нельзя.
assert.equal(presets.setLimit(400), presets.TRAVEL_LIMIT_DEG);
assert.equal(presets.setLimit('abc'), presets.TRAVEL_LIMIT_DEG);
assert.equal(presets.setLimit(90), 90);
assert.equal(presets.list().filter((p) => p.available).length, 7);

// Ползунок экрана приводится к пределу: min/max и значение внутри хода.
const fakeRange = { min: '-32', max: '32', value: '40' };
assert.equal(presets.syncRange(fakeRange), 40);
assert.equal(fakeRange.min, '-90');
assert.equal(fakeRange.max, '90');
assert.equal(fakeRange.value, '40');
const outOfRange = { min: '-90', max: '90', value: '130' };
presets.syncRange(outOfRange);
assert.equal(outOfRange.value, '90');
assert.equal(presets.syncRange(null), null);

console.log('Steering presets signs, limits, side step and pivot availability: OK');
