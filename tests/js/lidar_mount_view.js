'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync('slam_gui/lidar_view.js', 'utf8');
let marks = [], rings = [], model;
const context = { console, localStorage: { getItem: () => null, setItem() {} },
  setTimeout, clearTimeout, devicePixelRatio: 1,
  document: {createElement: () => ({setAttribute(){}, appendChild(){}, addEventListener(){}})},
  RCRobotModel: {subscribe(){}, draw(ctx, x, y, yaw, scale, options){ model = {x,y,yaw,scale,options}; }, drawStatus(){}} };
context.window = context;
vm.createContext(context);
vm.runInContext(source, context);
const ctx = {setTransform(){}, clearRect(){marks=[]; rings=[]}, beginPath(){}, moveTo(){}, lineTo(){}, stroke(){}, fill(){}, arc(x,y,r){rings.push({x,y,r})}, rect(){}, closePath(){}, fillText(){}, fillRect(x,y,w,h){marks.push({x,y,w,h})}};
const canvas = {clientWidth:400, clientHeight:400, width:400, height:400, style:{}, parentElement:null,
  addEventListener(){}, getBoundingClientRect(){return {left:0,top:0,width:400,height:400}}, getContext(){return ctx}};
context.RCLidarView.setSettings({grid:false,axes:false,decay:0});
context.RCLidarView.render(canvas, [{a:0,r:2},{a:Math.PI,r:2}], {lidarOffsetM:.28});
const k = (200-14)/10;
assert.equal(marks.length, 2);
assert.ok(rings.length && Math.abs(rings[0].y - (200 - .28 * k)) < .001);
assert.ok(Math.abs((marks[0].y + marks[0].h/2) - (200 - (2+.28)*k)) < .001);
assert.ok(Math.abs((marks[1].y + marks[1].h/2) - (200 - (-2+.28)*k)) < .001);
assert.equal(model.options.lidarOffsetM,.28);
assert.equal(model.options.minScale,0);
context.RCLidarView.render(canvas, [{a:0,r:2}], {lidarOffsetM:.5});
assert.ok(Math.abs((marks[0].y + marks[0].h/2) - (200 - 2.5*k)) < .001);
assert.equal(model.options.lidarOffsetM,.5);
