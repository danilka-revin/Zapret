/* Deterministic camera lifecycle and pointer gesture regression checks. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const elements = new Map();
const timers = new Map();
const images = [];
let timerId = 0;
class Element {
  constructor(id) {
    this.id = id; this.events = {}; this.hidden = false; this.textContent = '';
    this.classes = new Set(); this.attrs = {}; this.style = {};
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
  closest(selector) { return selector === '.cam-card' ? get(this.id.slice(0,4) + '-card') : null; }
  setPointerCapture() {}
  setAttribute(k, v) { this.attrs[k] = v; }
  getContext() {
    return { setTransform() {}, fillRect() {}, drawImage: (...args) => { this.lastDraw = args; } };
  }
}
function get(id) { if (!elements.has(id)) elements.set(id, new Element(id)); return elements.get(id); }
const context = {
  console, Map, Math, Date,
  document: { hidden: false, getElementById:get, addEventListener() {} },
  addEventListener() {}, devicePixelRatio:1,
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
for (const filename of ['viewport.js','camera_view.js']) {
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../../slam_gui',filename),'utf8'),context);
}
const camera = context.RCCameraView;
camera.setActive(true);
assert.equal(images.length,2);
assert.match(images[0].src,/camera\/0\/frame/);
assert.match(images[1].src,/camera\/1\/frame/);
images[0].onerror(); images[1].onload();
assert.equal(get('cam1-empty').hidden,false);
assert.equal(get('cam2-empty').hidden,true);
assert.match(get('cam2-chip').textContent,/LIVE/);
assert.equal(get('cam2-canvas').lastDraw.length,5);
get('cam2-in').fire('click');
assert.equal(get('cam2-zoom').textContent,'1.3×');
for (let i=0;i<30;i++) get('cam2-in').fire('click');
assert.equal(get('cam2-zoom').textContent,'8.0×');
get('cam2-reset').fire('click');
assert.equal(get('cam2-zoom').textContent,'1.0×');
get('cam2-view').fire('wheel',{deltaY:-300,clientX:400,clientY:225});
assert.notEqual(get('cam2-zoom').textContent,'1.0×');
get('cam2-expand').fire('click');
assert.equal(get('cam2-card').classList.contains('is-expanded'),true);
assert.equal(get('cam2-expand').attrs['aria-pressed'],'true');
get('cam2-expand').fire('click');
assert.equal(get('swipe-cameras-view').classList.contains('camera-expanded'),false);
camera.setActive(false);
assert.equal(timers.size,0,'all polls and timeouts must stop');
camera.setActive(true);
const lateCallback = images.at(-1).onload;
camera.setActive(false);
lateCallback();
assert.equal(timers.size,0,'a late frame cannot restart a closed screen');
camera.setActive(true);
const timeout = [...timers.values()].find(t => t.delay === 5000);
timeout.fn();
assert.equal(get('cam1-empty').hidden,false);
assert([...timers.values()].some(t => t.delay === 1000),'a timed out camera retries');
camera.setActive(false);
// The viewport uses the same semantics for map, touch pinch, wheel and buttons.
const canvas = new Element('test-viewport');
let latest;
const viewport = context.RCViewport(canvas, s => latest = {...s}, {min:0.4,max:8});
viewport.zoomTo(2); assert.equal(latest.zoom,2);
canvas.fire('pointerdown',{pointerId:1,clientX:100,clientY:100});
canvas.fire('pointermove',{pointerId:1,clientX:130,clientY:110});
assert.equal(latest.x,30); assert.equal(latest.y,10);
canvas.fire('pointerdown',{pointerId:2,clientX:230,clientY:110});
canvas.fire('pointermove',{pointerId:2,clientX:330,clientY:110});
assert.equal(latest.zoom,4,'two fingers moving apart must zoom in');
canvas.fire('pointercancel',{pointerId:1});
canvas.fire('pointerup',{pointerId:2});
const previous = {...latest};
canvas.fire('pointermove',{pointerId:1,clientX:500,clientY:110});
assert.deepEqual(latest,previous,'cancelled fingers no longer move the view');
viewport.zoomTo(100); assert.equal(latest.zoom,8);
viewport.zoomTo(0); assert.equal(latest.zoom,0.4);
viewport.reset(); assert.deepEqual(latest,{zoom:1,x:0,y:0});
console.log('camera lifecycle, rear-only feed, zoom/pan/pinch and cancellation: OK');
