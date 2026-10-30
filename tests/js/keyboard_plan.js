const assert = require('node:assert/strict');
const {plan} = require('../../slam_gui/keyboard.js');
const keys = text => new Set([...text].map(x => 'Key' + x));
const angles = value => ({FL: value, FR: value, RL: value, RR: value});
for (const [key, angle, direction] of [
  ['W', 0, 1], ['S', 0, -1], ['A', -90, 1], ['D', 90, 1],
  ['WD', 45, 1], ['WA', -45, 1], ['SD', -45, -1], ['SA', 45, -1]
]) {
  const got = plan(keys(key), 'crab');
  assert.equal(got.direction, direction, key);
  for (const value of Object.values(got.angles)) assert.equal(Math.abs(value - angle), 0, key);
}
assert.equal(plan(keys(''), 'crab'), null);
assert.equal(plan(keys('WS'), 'crab'), null);
assert.equal(plan(keys('AD'), 'crab'), null);
assert.deepEqual(plan(keys('WD'), 'normal'),
  {angles: {FL: 25, FR: 25, RL: 0, RR: 0}, direction: 1});
assert.deepEqual(plan(keys('SA'), 'normal'),
  {angles: {FL: -25, FR: -25, RL: 0, RR: 0}, direction: -1});
assert.deepEqual(plan(keys('D'), 'normal'),
  {angles: {FL: 25, FR: 25, RL: 0, RR: 0}, direction: 0});
console.log('keyboard plans OK');
