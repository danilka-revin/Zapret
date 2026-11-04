'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const context = { window: {} };
vm.createContext(context);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../../slam_gui/wheel_ports.js'), 'utf8'), context);
const view = context.window.RCWheelPorts;
const el = { textContent: '', title: '' };
const motor = { online: true, port: '/dev/ttyACM4', portPath: '/dev/serial/by-path/a-very-long-name' };
view.render(el, motor, true);
assert.equal(el.textContent, '/dev/ttyACM4');
assert.match(el.title, /\/dev\/serial\/by-path\/a-very-long-name/);
view.render(el, motor, false);
assert.equal(el.textContent, '—');
assert.match(el.title, /Нет свежего подтверждения/);
view.render(el, { ...motor, online: false }, true);
assert.equal(el.textContent, '—');
view.render(el, { ...motor, port: '' }, true);
assert.equal(el.textContent, '—');
view.render(el, { ...motor, port: '<img src=x onerror=alert(1)>' }, true);
assert.equal(el.textContent, '<img src=x onerror=alert(1)>'); // assigned as text, never HTML

const data = { masterPort: { auto: true, ready: false, modules: { FL: '/dev/a', RR: '/dev/b' },
  candidates: ['/dev/a', '/dev/b', '/dev/unknown'], legacy: ['/dev/legacy'],
  unclassified: ['/dev/unknown'], excluded: ['/dev/excluded'], error: 'порт занят' } };
const status = view.status(data);
for (const part of ['2/4', 'FR, RL', 'порт занят', '/dev/legacy', '/dev/unknown', '/dev/excluded', 'заблокированы']) {
  assert(status.includes(part), status);
}
assert.equal(view.status({masterPort: {auto: false}}), '');
assert.equal(view.status({}), '');
const empty = view.status({ masterPort: { auto: true, ready: false } });
assert.match(empty, /USB-порты не найдены/);
assert.match(empty, /FL, FR, RL, RR/);
console.log('wheel ports UI passed');
