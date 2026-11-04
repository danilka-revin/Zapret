'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const timers = new Map();
const images = [];
const frames = [];
let nextTimer = 0;
const ctx = {
  Date,
  setTimeout(fn, delay) { const id = ++nextTimer; timers.set(id, {fn, delay}); return id; },
  clearTimeout(id) { timers.delete(id); },
  Image: class {
    constructor() { this.naturalWidth = 640; images.push(this); }
    set src(url) { this.url = url; }
    get src() { return this.url; }
  },
};
ctx.window = ctx;
vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../../slam_gui/camera_frames.js'), 'utf8'), ctx);
const poller = ctx.RCCameraFrames.create((id, frame) => frames.push({id, frame}));
function waiting() { return images.filter(image => image.onload || image.onerror); }
function runTimer(delay) {
  const entry = [...timers].find(([, timer]) => timer.delay === delay);
  assert(entry, 'missing timer ' + delay);
  timers.delete(entry[0]);
  entry[1].fn();
}

poller.start();
poller.start();
assert.equal(images.length, 2, 'only two requests may occupy connections');
assert.match(images[0].src, /api\/camera\/0\/frame\?t=/);
assert.match(images[1].src, /api\/camera\/1\/frame\?t=/);
images[0].onload();
assert.equal(frames.at(-1).id, 0);
assert.equal(frames.at(-1).frame, images[0]);
assert.match(images[2].src, /api\/camera\/2\/frame/);
images[1].onerror();
assert.equal(frames.at(-1).id, 1);
assert.equal(frames.at(-1).frame, null, 'an absent camera is not live');
assert.match(images[3].src, /api\/camera\/3\/frame/);
assert.equal(waiting().length, 2);
images[3].onload();
runTimer(250); // camera 0 becomes ready again, despite camera 1 being absent
assert.match(images[4].src, /api\/camera\/0\/frame/);
runTimer(1000); // camera 1 waits in the queue; there are already two requests
assert.equal(images.length, 5);
assert.equal(waiting().length, 2);
runTimer(1500); // camera 2 hangs: abort and let the missing camera retry
assert.equal(images[2].src, '');
assert.equal(frames.at(-1).id, 2);
assert.equal(frames.at(-1).frame, null);
assert.match(images[5].src, /api\/camera\/1\/frame/);
assert.equal(waiting().length, 2);
const late = images[5].onload;
poller.stop();
assert.equal(waiting().length, 0);
assert.equal(timers.size, 0, 'stopping aborts pending requests and timers');
const count = frames.length;
late();
assert.equal(frames.length, count, 'late callbacks cannot revive a stopped poller');
assert.equal(timers.size, 0);
poller.start();
assert.match(images[6].src, /api\/camera\/0\/frame/);
assert.match(images[7].src, /api\/camera\/1\/frame/);
poller.stop();
assert.equal(timers.size, 0);
console.log('bounded camera frame polling: OK');
