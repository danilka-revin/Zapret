'use strict';
const assert = require('assert');
const fs = require('fs');
const vm = require('vm');
class El {
  constructor(tag) { this.tagName=tag; this.children=[]; this.attrs={}; this.listeners={}; this.hidden=false;
    this.value='0'; this.min='-90'; this.max='90'; this.disabled=false; this.captures=new Set(); }
  appendChild(child) { this.children.push(child); return child; }
  setAttribute(k,v) { this.attrs[k]=String(v); }
  getAttribute(k) { return this.attrs[k] ?? null; }
  insertAdjacentElement(position,el) { this.adjacent=el; }
  addEventListener(type,fn) { (this.listeners[type] ||= []).push(fn); }
  dispatchEvent(event) { for(const fn of this.listeners[event.type]||[]) fn(event); return true; }
  fire(type,props={}) { this.dispatchEvent({type,preventDefault(){},stopPropagation(){},...props}); }
  focus() { document.activeElement=this; }
  getBoundingClientRect() { return {left:10,top:20,width:200,height:200}; }
  setPointerCapture(id) { this.captures.add(id); }
  hasPointerCapture(id) { return this.captures.has(id); }
  releasePointerCapture(id) { this.captures.delete(id); this.fire('lostpointercapture',{pointerId:id}); }
}
const document = {hidden:false, listeners:{}, createElement:tag=>new El(tag), createElementNS:(_ns,tag)=>new El(tag),
  addEventListener(type,fn){(this.listeners[type] ||= []).push(fn);}};
const window = new El('window');
const context = {window, document, Event:class {constructor(type,options){this.type=type;Object.assign(this,options);}}};
vm.runInNewContext(fs.readFileSync('slam_gui/steer_dial.js','utf8'),context);
const input = new El('input');
input.setAttribute('aria-label','FL · целевой угол');
let inputs=0, changes=0;
input.addEventListener('input',()=>inputs++);
input.addEventListener('change',()=>changes++);
const control = window.RCSteerDial.attach(input), dial = control.element;
const find = (el,cls)=>el.attrs.class===cls ? el : el.children.map(child=>find(child,cls)).find(Boolean);
const actual = find(input.adjacent,'steer-dial-actual');
const target = find(input.adjacent,'steer-dial-target');
const knob = find(input.adjacent,'steer-dial-knob');
function position(angle) { const r=angle*Math.PI/180; return {pointerId:1,button:0,clientX:110+80*Math.sin(r),clientY:120-80*Math.cos(r)}; }
function pointer(type,angle,extra={}) { dial.fire(type,{...position(angle),...extra}); }
function reset(value=0) { input.value=String(value); input.disabled=false; control.sync(null); inputs=0; changes=0; }
assert.equal(window.RCSteerDial.attach(input),control);
assert(input.hidden);
assert.equal(dial.getAttribute('role'),'slider');
assert.equal(dial.getAttribute('aria-label'),'FL · целевой угол');
control.sync(-25);
assert.equal(actual.getAttribute('transform'),'rotate(-25 100 100)');
assert.equal(target.getAttribute('transform'),'rotate(0 100 100)');
assert.equal(changes,0,'telemetry must never commit');
control.sync(null); assert.equal(actual.getAttribute('visibility'),'hidden');
assert.match(dial.getAttribute('aria-valuetext'),/нет данных/);
// Pointer previews immediately, one existing change event at release only.
reset(); pointer('pointerdown',20); pointer('pointermove',40); pointer('pointermove',65);
assert.equal(input.value,'65'); assert.equal(changes,0); assert.equal(inputs,3);
pointer('pointerup',65); assert.equal(changes,1); assert.equal(dial.captures.size,0);
assert.equal(target.getAttribute('transform'),'rotate(65 100 100)');
// Signed direction: forward is top, right is positive; left is negative.
for (const angle of [-90,0,90]) {
  reset(); pointer('pointerdown',angle); pointer('pointerup',angle);
  assert.equal(Number(input.value),angle); assert.equal(changes,1);
  if(angle===90) assert(Number(knob.getAttribute('cx'))>179);
  if(angle===-90) assert(Number(knob.getAttribute('cx'))<21);
  if(angle===0) assert(Number(knob.getAttribute('cy'))<21);
}
// Never wrap directly from the right stop to the left at the rear of the circle.
reset(); pointer('pointerdown',120); pointer('pointermove',179); pointer('pointermove',-179); pointer('pointerup',-179);
assert.equal(input.value,'90'); assert.equal(changes,1);
reset(); pointer('pointerdown',-120); pointer('pointermove',-179); pointer('pointermove',179); pointer('pointerup',179);
assert.equal(input.value,'-90');
// Ignore right clicks, second contacts and center clicks.
reset(); pointer('pointerdown',30,{button:2}); pointer('pointerup',30); assert.equal(changes,0);
pointer('pointerdown',30,{isPrimary:false}); assert.equal(inputs,0);
dial.fire('pointerdown',{pointerId:1,clientX:110,clientY:120,button:0}); assert.equal(inputs,0);
pointer('pointerdown',20); pointer('pointermove',-60,{pointerId:2}); pointer('pointerup',-60,{pointerId:2});
assert.equal(input.value,'20'); assert.equal(changes,0); pointer('pointerup',20); assert.equal(changes,1);
// Cancellation, lost capture, focus/page loss and disabling do not turn a wheel.
for (const reason of ['pointercancel','lostpointercapture','blur','window-blur','pagehide','hidden','disabled','escape']) {
  reset(10); pointer('pointerdown',45); assert.equal(input.value,'45');
  if(reason==='window-blur') window.fire('blur');
  else if(reason==='pagehide') window.fire('pagehide');
  else if(reason==='hidden'){ document.hidden=true; document.listeners.visibilitychange.forEach(fn=>fn()); document.hidden=false; }
  else if(reason==='disabled'){input.disabled=true;control.sync(12);}
  else if(reason==='escape') dial.fire('keydown',{key:'Escape'});
  else dial.fire(reason,{pointerId:1});
  pointer('pointerup',45);
  assert.equal(changes,0,reason); assert.equal(input.value,'10',reason);
}
input.disabled=true; control.sync(null); pointer('pointerdown',80); pointer('pointerup',80);
assert.equal(changes,0); assert.equal(dial.tabIndex,-1); assert.equal(dial.getAttribute('aria-disabled'),'true');
// Keyboard: repeats preview, release commits once. Escape rolls back.
reset(); for(let i=0;i<3;i++) dial.fire('keydown',{key:'ArrowRight'});
assert.equal(input.value,'3'); assert.equal(changes,0);
dial.fire('keyup',{key:'ArrowRight'}); assert.equal(changes,1);
dial.fire('keydown',{key:'PageDown'}); dial.fire('keyup',{key:'PageDown'}); assert.equal(input.value,'-2');
dial.fire('keydown',{key:'Home'}); dial.fire('keyup',{key:'Home'}); assert.equal(input.value,'-90');
dial.fire('keydown',{key:'End'}); dial.fire('keyup',{key:'End'}); assert.equal(input.value,'90');
reset(15); dial.fire('keydown',{key:'ArrowLeft'}); dial.fire('keydown',{key:'Escape'}); dial.fire('keyup',{key:'ArrowLeft'});
assert.equal(input.value,'15'); assert.equal(changes,0);
// The server remains the limit source. No baked-in 90 degree stop.
reset(); input.min='-100'; input.max='100'; control.sync(0); pointer('pointerdown',100); pointer('pointerup',100);
assert.equal(input.value,'100'); assert.equal(dial.getAttribute('aria-valuemax'),'100');
input.min='-45'; input.max='45'; control.sync(0); reset(); pointer('pointerdown',80); pointer('pointerup',80); assert.equal(input.value,'45');
console.log('Shared steering dial: geometry, live/current separation, bounds, gestures, keyboard and cancellation OK');
