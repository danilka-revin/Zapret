const assert = require('node:assert/strict');
const {plan} = require('../../slam_gui/keyboard.js');
const keys = (text, arrow = '') => new Set([...text].map(x => 'Key' + x).concat(arrow ? ['Arrow' + arrow] : []));
const angles = value => ({FL: value, FR: value, RL: value, RR: value});
// The same wheel axis supports the opposite physical direction by reversing
// the motors. Arrows select traction independently of steering keys.
for (const [key, angle, sign] of [
  ['', 0, 1], ['W', 0, 1], ['S', 0, -1], ['A', 90, -1], ['D', 90, 1],
  ['WD', 45, 1], ['WA', -45, 1], ['SD', -45, -1], ['SA', 45, -1]
]) {
  assert.deepEqual(plan(keys(key, 'Up'), 'crab'), {angles: angles(angle), direction: sign}, key + ' up');
  assert.deepEqual(plan(keys(key, 'Down'), 'crab'), {angles: angles(angle), direction: -sign}, key + ' down');
  assert.deepEqual(plan(keys(key), 'crab'), {angles: angles(angle), direction: 0}, key + ' no traction');
}
for (const conflict of ['WS', 'AD', 'WAD', 'WSA']) {
  assert.deepEqual(plan(keys(conflict, 'Up'), 'crab'), {angles: angles(0), direction: 0}, conflict);
}
assert.deepEqual(plan(new Set(['ArrowUp', 'ArrowDown', 'KeyD']), 'normal'), {angles: angles(0), direction: 0});
assert.deepEqual(plan(keys('D', 'Up'), 'normal'),
  {angles: {FL: 25, FR: 25, RL: -25, RR: -25}, direction: 1});
assert.deepEqual(plan(keys('SA', 'Up'), 'normal'),
  {angles: {FL: -25, FR: -25, RL: 25, RR: 25}, direction: -1});
assert.deepEqual(plan(keys('D'), 'normal'),
  {angles: {FL: 25, FR: 25, RL: -25, RR: -25}, direction: 0});
assert.deepEqual(plan(keys('', 'Up'), 'normal'), {angles: angles(0), direction: 1});
console.log('keyboard plans OK');
