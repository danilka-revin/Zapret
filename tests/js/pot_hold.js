/* Переключатель «Активная доводка» (slam_gui/pot_hold.js) — без браузера.
 *
 * Компонент не экспортирует DOM-хелперы, поэтому здесь поднимается минимальный
 * DOM (тот же приём, что в tests/js/steering_screen.js), скрипт выполняется как
 * в странице, а проверяется то, что видит оператор и что уходит на сервер:
 *   • подпись, aria-checked и доступность переключателя;
 *   • блокировки: нет связи, старая прошивка, нет калибровки, движение, E-STOP,
 *     нейтраль, нет PIN;
 *   • «ждём подтверждения плат» после отправки и отсутствие оптимистичного
 *     переключения: состояние меняет только телеметрия;
 *   • при disabled клик не отправляет POST.
 *
 * Запуск (из tests/test_pot_hold.py или вручную):
 *   node tests/js/pot_hold.js <файл-payload.json>
 * На stdout — JSON с тем, что увидели.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..', '..');
const payloadFile = process.argv[2];
if (!payloadFile) {
  console.error('нужен файл со сценарием');
  process.exit(2);
}
const payload = JSON.parse(fs.readFileSync(payloadFile, 'utf8'));

/* --------------------------------------------------------------- мини-DOM */
class El {
  constructor(tag) {
    this.tagName = (tag || 'div').toUpperCase();
    this.children = [];
    this.attributes = {};
    this.textContent = '';
    this.className = '';
    this.disabled = false;
    this.hidden = false;
    this.listeners = {};
    this.parentElement = null;
    this.type = '';
  }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  getAttribute(name) { return Object.prototype.hasOwnProperty.call(this.attributes, name) ? this.attributes[name] : null; }
  removeAttribute(name) { delete this.attributes[name]; }
  appendChild(child) {
    if (child.parentElement) {
      const index = child.parentElement.children.indexOf(child);
      if (index >= 0) child.parentElement.children.splice(index, 1);
    }
    this.children.push(child);
    child.parentElement = this;
    return child;
  }
  addEventListener(name, listener) { (this.listeners[name] = this.listeners[name] || []).push(listener); }
  removeEventListener() {}
  fire(name, extra) {
    const event = Object.assign({ type: name, target: this, preventDefault() {}, stopPropagation() {} }, extra || {});
    (this.listeners[name] || []).forEach((listener) => listener(event));
    return event;
  }
  /* Кнопка с disabled не получает click в браузере. Обе ветки проверяются
   * отдельно: и «браузер не нажал», и «событие всё-таки пришло». */
  click() {
    if (this.tagName === 'BUTTON' && this.disabled) return;
    this.fire('click');
  }
}

global.document = {
  createElement: (tag) => new El(tag),
  createDocumentFragment: () => new El('fragment'),
  addEventListener: () => {},
  removeEventListener: () => {},
  body: new El('body'),
};

let virtualNow = 1000000;
const RealDate = Date;
global.Date = class extends RealDate {
  static now() { return virtualNow; }
  constructor(...args) { if (args.length === 0) super(virtualNow); else super(...args); }
};

const requests = [];
global.fetch = async (url, init) => {
  const body = init && init.body ? JSON.parse(init.body) : {};
  requests.push({
    url: String(url),
    method: (init && init.method) || 'GET',
    header: (init && init.headers && init.headers['X-Robot-Control']) || null,
    action: body.action === undefined ? null : body.action,
  });
  const step = payload.responses && payload.responses[requests.length - 1];
  if (step && step.networkError) throw new Error('network');
  const status = step && step.status ? step.status : 200;
  const json = step && step.json !== undefined ? step.json : { ok: true, queued: true };
  return { ok: status >= 200 && status < 300, status, json: async () => json };
};

global.window = global;
global.self = global;
global.confirm = () => (payload.confirm === false ? false : true);

function load(name) {
  const file = path.join(ROOT, 'slam_gui', name);
  vm.runInThisContext(fs.readFileSync(file, 'utf8'), { filename: file });
}
load('pot_hold.js');

/* ------------------------------------------------------------- сценарий */
function motor(id, overrides) {
  const base = {
    id, online: true, homed: true, angle: 0, targetAngle: 0, moving: false,
    calibrating: false, cycling: false, drivePercent: 0, pot: true, potMode: true,
    potHold: false, firmware: 24,
  };
  return Object.assign(base, overrides || {});
}

function state(overrides) {
  const motors = overrides && overrides.motors ? overrides.motors : ['FL', 'FR', 'RL', 'RR'].map((id) => motor(id));
  return Object.assign({ motors, estop: { latched: false, unknown: false },
                         neutral: { locked: false } }, overrides || {});
}

function read(out) {
  return {
    text: out.button.textContent,
    role: out.button.getAttribute('role'),
    ariaChecked: out.button.getAttribute('aria-checked'),
    ariaLabel: out.button.getAttribute('aria-label'),
    tag: out.button.tagName,
    type: out.button.type,
    disabled: !!out.button.disabled,
    status: out.status.textContent,
    alert: out.alert.textContent,
    hint: out.hint.textContent,
    requests: requests.length,
  };
}

const sleep = (ms) => new Promise((resolve) => setImmediate(resolve));

(async function run() {
  const parent = new El('div');
  const seen = [];
  window.RCPotHold.render(parent, state({}), true);
  const host = parent.children[0];
  const byClass = name => host.children.find(child => child.className.split(/\s+/).includes(name));
  const parts = {
    button: byClass('pot-hold__switch'),
    hint: byClass('pot-hold__hint'),
    status: byClass('pot-hold__status'),
    alert: byClass('pot-hold__alert'),
  };
  seen.push(Object.assign({ step: 'render' }, read(parts)));

  for (const step of payload.steps || []) {
    if (step.advanceMs) virtualNow += step.advanceMs;
    const body = step.motors ? step.state || {} : (step.state || {});
    window.RCPotHold.render(parent, state(Object.assign({}, body,
      step.motors ? { motors: step.motors } : {})), step.authorized !== undefined ? step.authorized : true);
    if (step.click) {
      parts.button.fire('click');
      await sleep();
    }
    if (step.key) {
      // Клавиатура: Enter и пробел обрабатывает сам компонент, поэтому здесь
      // отправляется только keydown — синтетического click нет.
      parts.button.fire('keydown', { key: step.key });
      await sleep();
    }
    seen.push(Object.assign({ step: step.name || ('шаг ' + seen.length) }, read(parts)));
  }

  process.stdout.write(JSON.stringify({ seen, requests, parentChildren: parent.children.length }, null, 2) + '\n');
})().catch((error) => {
  process.stdout.write(JSON.stringify({ fatal: String((error && error.stack) || error) }) + '\n');
  process.exit(1);
});
