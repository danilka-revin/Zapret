/* Browser-side two-camera compositor and calibration controls, without hardware. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const elements = new Map();
const timers = new Map();
const images = [];
const storage = new Map();
const drawCalls = [];
let timerId = 0;
class Element {
  constructor(id) {
    this.id = id; this.events = {}; this.hidden = false; this.textContent = '';
    this.classes = new Set(); this.attrs = {}; this.style = {}; this.checked = false;
    this.value = id.includes('-camera') ? (id.includes('right') ? '2' : '0') : '0';
    this.type = id.includes('-mirror') || id === 'cam-stitch-grid' ? 'checkbox'
      : (id.includes('-camera') ? 'select-one' : 'range');
    this.classList = {
      contains: (c) => this.classes.has(c),
      toggle: (c, on) => { if (on) this.classes.add(c); else this.classes.delete(c); },
    };
  }
  addEventListener(name, fn) { (this.events[name] ||= []).push(fn); }
  fire(name, event = {}) {
    (this.events[name] || []).forEach(fn => fn({ target: this, preventDefault() {}, stopPropagation() {}, ...event }));
  }
  getBoundingClientRect() { return { left:0, top:0, width:800, height:450 }; }
  closest() { return null; }
  setAttribute(k, v) { this.attrs[k] = v; }
  getContext() {
    return {
      setTransform() {}, fillRect() {}, drawImage: (...args) => { this.lastDraw = args; drawCalls.push(args); },
      save() {}, restore() {}, beginPath() {}, rect() {}, clip() {}, translate() {}, rotate() {}, scale() {},
      setLineDash() {}, moveTo() {}, lineTo() {}, stroke() {}, fillText() {},
    };
  }
}
function get(id) { if (!elements.has(id)) elements.set(id, new Element(id)); return elements.get(id); }
get('cam-stitch-panel').hidden = true;
const context = {
  console, Map, Math, Date, JSON, Number, String, Set,
  document: {
    hidden: false, readyState:'complete', getElementById:get,
    querySelector(selector) { return selector === '.cam-stitch-seam' ? get('cam-stitch-seam') : null; },
    addEventListener() {},
  },
  localStorage: {
    getItem(key) { return storage.get(key) || null; },
    setItem(key, value) { storage.set(key, value); },
  },
  addEventListener() {}, devicePixelRatio:1,
  requestAnimationFrame(fn) { fn(); return 1; },
  setTimeout(fn, delay) { const id = ++timerId; timers.set(id, {fn,delay}); return id; },
  clearTimeout(id) { timers.delete(id); },
  Image: class {
    constructor() { this.naturalWidth = 1920; this.naturalHeight = 1080; images.push(this); }
    set src(value) { this.url = value; }
    get src() { return this.url; }
  },
};
context.window = context;
vm.createContext(context);
for (const filename of ['viewport.js','camera_view.js','camera_stitch.js']) {
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../../slam_gui',filename),'utf8'),context);
}
const camera = context.RCCameraView;
const stitch = context.RCCameraStitch;
camera.setActive(true);
assert.equal(images.length,4,'stitch uses the four existing camera requests');
images[0].onload(); images[2].onload();
assert.equal(camera.getFrame(0), images[0]);
assert.equal(camera.getFrame(2), images[2]);
assert(drawCalls.some(args => args[0] === images[0]), 'left camera frame is composited');
assert(drawCalls.some(args => args[0] === images[2]), 'right camera frame is composited');
assert.match(get('cam-stitch-status').textContent, /FL LIVE · FR LIVE/);

// Same-camera selections are automatically disambiguated without losing the old pair.
get('cam-stitch-left-camera').value = '2';
get('cam-stitch-left-camera').fire('input');
assert.equal(get('cam-stitch-right-camera').value, '0');
assert.equal(storage.has('robot-camera-stitch-v1'), true, 'calibration auto-saves locally');

get('cam-stitch-left-mirror').checked = true;
get('cam-stitch-left-mirror').fire('input');
get('cam-stitch-right-crop').value = '14';
get('cam-stitch-right-crop').fire('input');
assert.equal(get('cam-stitch-right-crop-value').textContent, '14%');
get('cam-stitch-left-roll').value = '-3.5';
get('cam-stitch-left-roll').fire('input');
assert.equal(get('cam-stitch-left-roll-value').textContent, '-3.5°');

get('cam-stitch-toggle').fire('click');
assert.equal(get('cam-stitch-panel').hidden, false);
assert.equal(get('cam-stitch-toggle').attrs['aria-pressed'], 'true');
assert.equal(get('swipe-cameras-view').classList.contains('camera-stitch-open'), true);
get('cam-stitch-grid').checked = false;
get('cam-stitch-grid').fire('change');
assert.equal(get('cam-stitch-seam').hidden, true);

get('cam-stitch-reset').fire('click');
assert.equal(get('cam-stitch-left-camera').value, '0');
assert.equal(get('cam-stitch-right-camera').value, '2');
assert.equal(get('cam-stitch-left-mirror').checked, false);
assert.equal(get('cam-stitch-right-crop-value').textContent, '0%');
assert.equal(get('cam-stitch-grid').checked, true);
stitch.setActive(false);
assert.equal(typeof stitch.render, 'function');
camera.setActive(false);
assert.equal(timers.size, 0, 'compositor does not leave camera polling alive');
console.log('two-camera composite, pair selection, mirror, crop, alignment and persistence: OK');
