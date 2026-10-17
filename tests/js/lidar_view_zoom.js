/* Проверка зума карты лидара (slam_gui/lidar_view.js) без браузера.
 *
 * Запуск (из tests/test_lidar_view_zoom.py): node tests/js/lidar_view_zoom.js
 * На stdout — JSON с результатами: точка под курсором не уезжает при зуме колесом,
 * кнопки «+»/«−» меняют радиус, пределы зума соблюдаются, сброс возвращает 10 м.
 */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..', '..');
const src = fs.readFileSync(path.join(ROOT, 'slam_gui', 'lidar_view.js'), 'utf8');

const store = {};
const sandbox = {
  console,
  localStorage: {
    getItem: (k) => (k in store ? store[k] : null),
    setItem: (k, v) => { store[k] = String(v); },
  },
  devicePixelRatio: 1,
  document: { createElement: () => ({ setAttribute() {}, appendChild() {}, addEventListener() {} }) },
};
sandbox.window = sandbox;
vm.createContext(sandbox);
vm.runInContext(src, sandbox);
const RV = sandbox.RCLidarView;

const listeners = {};
const canvas = {
  clientWidth: 400, clientHeight: 400, width: 400, height: 400,
  style: {}, parentElement: null,
  addEventListener(name, fn) { (listeners[name] = listeners[name] || []).push(fn); },
  getBoundingClientRect() { return { left: 0, top: 0, width: 400, height: 400 }; },
  getContext() { return { setTransform() {}, clearRect() {}, beginPath() {}, moveTo() {}, lineTo() {},
    stroke() {}, fill() {}, arc() {}, rect() {}, closePath() {}, fillRect() {}, fillText() {} }; },
};
RV.attach(canvas);

const fire = (name, ev) => (listeners[name] || []).forEach((fn) => fn(Object.assign({ preventDefault() {}, stopPropagation() {} }, ev)));

// Внутренний масштаб мы не экспортируем, поэтому восстанавливаем его из вида:
// мировая точка под курсором по формуле канвы (центр 200,200; радиус 10 м).
function scaleFor(radius) { return (200 - 14) / radius; }

const results = {};
const view = canvas.__rcLidarView;

// 1. Колесо к курсору: мировая точка под курсором остаётся на месте.
const cursor = { clientX: 300, clientY: 120 };
const k0 = scaleFor(view.radius);
const worldBefore = { x: view.px + (200 - cursor.clientY) / k0, y: view.py + (200 - cursor.clientX) / k0 };
fire('wheel', Object.assign({ deltaY: -120, deltaMode: 0 }, cursor));
const k1 = scaleFor(view.radius);
const worldAfter = { x: view.px + (200 - cursor.clientY) / k1, y: view.py + (200 - cursor.clientX) / k1 };
results.wheelRadius = view.radius;
results.anchorDrift = Math.hypot(worldAfter.x - worldBefore.x, worldAfter.y - worldBefore.y);

// 2. Кнопки и пределы.
RV.reset(canvas);
RV.zoom(canvas, 1.25);
results.zoomInRadius = view.radius;
RV.zoom(canvas, 1 / 1.25);
results.zoomBackRadius = view.radius;
for (let i = 0; i < 80; i += 1) RV.zoom(canvas, 1.25);
results.minRadius = view.radius;
for (let i = 0; i < 80; i += 1) RV.zoom(canvas, 1 / 1.25);
results.maxRadius = view.radius;

// 3. Сброс вида.
RV.reset(canvas);
results.resetRadius = view.radius;
results.resetPan = [view.px, view.py];

// 4. Настройки: санитизация (мусор не ломает, max > min).
const saved = RV.setSettings({ size: 99, minValue: 5, maxValue: 3, color: 'nope', minColor: 'red' });
results.settings = { size: saved.size, minValue: saved.minValue, maxValue: saved.maxValue,
  color: saved.color, minColor: saved.minColor };
RV.setSettings(RV.DEFAULTS);
results.settingsReset = RV.getSettings().color;

console.log(JSON.stringify(results));
