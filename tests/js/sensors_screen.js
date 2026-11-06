/* Проверка экрана «Модуль датчиков» бортового экрана — без браузера.
 *
 * Скрипт страницы (slam_gui/sensors.js) написан под браузер, поэтому здесь
 * поднимается минимальный DOM, скрипт выполняется как в странице, а `fetch`
 * отдаёт заготовленные ответы API модуля датчиков. Результат читается из тех
 * же элементов, которые видит оператор: полоса состояния, компас, барометр,
 * клавиши света, замок отсека, карты, журнал платы.
 *
 * Запуск:
 *   node tests/js/sensors_screen.js <файл-payload.json>
 * На stdout — JSON: { status, compass, baro, lights, strip, lock, cards,
 *                      log, msg, calls, errors }.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..', '..');
const payloadFile = process.argv[2];
if (!payloadFile) {
  console.error('нужен файл с ответом API');
  process.exit(2);
}
const payload = JSON.parse(fs.readFileSync(payloadFile, 'utf8'));

/* ------------------------------------------------------------------ DOM */
class El {
  constructor(id, tag) {
    this.id = id || '';
    this.tagName = (tag || 'div').toUpperCase();
    this.textContent = '';
    this.hidden = false;
    this.disabled = false;
    this.title = '';
    this.value = '';
    this.style = { setProperty() {}, removeProperty() {}, cssText: '' };
    this.dataset = {};
    this.attributes = {};
    this.children = [];
    this.events = {};
    this.parentElement = null;
  }
  addEventListener(name, listener) { (this.events[name] ||= []).push(listener); }
  removeEventListener() {}
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  replaceChildren(...children) { this.children = children; }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  getAttribute(name) {
    if (name in this.attributes) return this.attributes[name];
    return null;
  }
  focus() {}
  click() { (this.events.click || []).forEach((listener) => listener({ target: this, preventDefault() {} })); }
}

/* createElement кладёт элементы в общий список: по нему тест находит кнопки
 * внутри строк списка карт (иначе до них не добраться без всплытия событий). */
const created = [];

const elements = new Map();
function byId(id) {
  if (!elements.has(id)) elements.set(id, new El(id, 'div'));
  return elements.get(id);
}

/* Клавиши света: HTML-страница ищет их через [data-light]. */
const LIGHTS = ['head', 'stop', 'left', 'right', 'dim'];
const lightButtons = LIGHTS.map((name) => {
  const button = new El('light-' + name, 'button');
  button.setAttribute('data-light', name);
  button.setAttribute('aria-pressed', 'false');
  return button;
});

global.document = {
  readyState: 'complete',
  body: new El('body'),
  documentElement: new El('html'),
  getElementById: byId,
  createElement: (tag) => {
    const el = new El('', tag);
    created.push(el);
    return el;
  },
  querySelector: () => null,
  querySelectorAll: (selector) => (selector === '[data-light]' ? lightButtons : []),
  addEventListener: () => {},
  removeEventListener: () => {},
};

global.window = global;
global.self = global;
global.addEventListener = () => {};
global.removeEventListener = () => {};
global.AbortController = class {
  constructor() { this.signal = {}; }
  abort() {}
};
global.setInterval = () => 0;
global.clearInterval = () => {};

/* --------------------------------------------------------------- ответы API */
const calls = [];
function reply(body, status) {
  const code = status || 200;
  return { ok: code < 400, status: code, json: async () => body };
}

function failure(url) {
  const rules = payload.fail || [];
  for (const rule of rules) {
    if (new RegExp(rule.match).test(url)) return reply(rule.body || { ok: false, error: 'сбой' }, rule.status || 500);
  }
  return null;
}

/* Состояние платы до и после первой команды: так видно, что экран показывает
 * ответ сервера, а не свою догадку. */
const before = (payload.snapshot && payload.snapshot.data) || payload.snapshot || {};
const after = payload.after || before;
let posted = false;

global.fetch = async (url, options) => {
  const text = String(url);
  const method = (options && options.method) || 'GET';
  const body = options && options.body ? JSON.parse(options.body) : null;
  calls.push({ url: text, method, body });
  const failed = failure(text);
  if (failed) return failed;
  if (method === 'POST') posted = true;
  const current = posted ? after : before;
  if (method === 'POST') {
    if (/mag$/.test(text)) return reply(payload.magReply || { ok: true, data: current });
    if (/cards$/.test(text)) return reply(payload.cardsReply || { ok: true, data: current });
    if (/cargo$/.test(text)) return reply(payload.cargoReply || { ok: true, data: current });
    if (/lock$/.test(text)) return reply(payload.lockReply || { ok: true, data: current });
    if (/lights$/.test(text)) return reply(payload.lightsReply || { ok: true, data: current });
  }
  return reply({ ok: true, data: current });
};

/* ------------------------------------------------------------------ пуск */
const renderErrors = [];
console.error = (...args) => { renderErrors.push(args.map((a) => String(a)).join(' ')); };

const file = path.join(ROOT, 'slam_gui', 'sensors.js');
try {
  vm.runInThisContext(fs.readFileSync(file, 'utf8'), { filename: file });
} catch (err) {
  console.log(JSON.stringify({ boot: String((err && err.stack) || err) }));
  process.exit(0);
}

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

function lightsState() {
  const out = {};
  lightButtons.forEach((button) => {
    out[button.getAttribute('data-light')] = {
      pressed: button.getAttribute('aria-pressed'),
      disabled: button.disabled,
      title: button.title,
    };
  });
  return out;
}

function status() {
  return {
    tone: byId('sm-status').dataset.tone || '',
    title: byId('sm-state').textContent,
    sub: byId('sm-sub').textContent,
    port: byId('sm-port').textContent,
    age: byId('sm-age').textContent,
    errors: byId('sm-errors').textContent,
  };
}

function byAttr(name, value) {
  return created.filter((el) => el.getAttribute(name) === String(value));
}

function cardRows() {
  return (byId('sm-card-list-box').children || []).map((row) => ({
    uid: row.getAttribute('data-uid') || '',
    role: row.getAttribute('data-role') || '',
    text: row.children.map((child) => child.textContent).join(' | '),
  }));
}

function logItems() {
  return (byId('sm-log').children || []).map((item) => item.textContent);
}

(async function run() {
  await sleep(20);

  const result = {
    status: status(),
    compass: {
      heading: byId('sm-heading').textContent,
      cardinal: byId('sm-cardinal').textContent,
      needle: byId('sm-needle').style.transform || '',
      tilt: byId('sm-tilt').textContent,
    },
    baro: {
      pressure: byId('sm-pressure').textContent,
      temp: byId('sm-temp').textContent,
      alt: byId('sm-alt').textContent,
    },
    lights: lightsState(),
    strip: {
      noteHidden: byId('sm-strip-note').hidden,
      dim: byId('sm-dim').value,
      dimLabel: byId('sm-dim-value').textContent,
    },
    /* Концевика отсека нет: на экране только замок. */
    lock: { text: byId('sm-lock').textContent,
            state: byId('sm-lock').dataset.state || '' },
    window: {
      label: byId('sm-window').textContent,
      state: byId('sm-window-state').textContent,
      tone: byId('sm-window').dataset.tone || '',
    },
    cards: { count: byId('sm-cards-count').textContent, rfid: byId('sm-rfid').textContent,
             rfidTone: byId('sm-rfid').dataset.tone || '',
             rows: cardRows(), role: byId('sm-card-role').value,
             empty: (byId('sm-card-list-box').children || []).map((c) => c.textContent) },
    log: logItems(),
    msg: { text: byId('sm-msg').textContent, tone: byId('sm-msg').dataset.tone || '' },
    calls: calls.slice(),
    errors: renderErrors.slice(),
  };

  /* Действия оператора. */
  if (payload.actions) {
    const acts = payload.actions;
    if (acts.toggle) {
      const button = lightButtons.find((b) => b.getAttribute('data-light') === acts.toggle);
      if (button) button.click();
      await sleep(20);
      result.afterToggle = lightsState();
      result.afterToggleMsg = { text: byId('sm-msg').textContent, tone: byId('sm-msg').dataset.tone || '' };
    }
    if (acts.allOff) {
      byId('sm-lights-off').click();
      await sleep(20);
    }
    if (acts.dim !== undefined) {
      byId('sm-dim').value = String(acts.dim);
      (byId('sm-dim').events.change || []).forEach((listener) => listener({ target: byId('sm-dim') }));
      await sleep(20);
    }
    if (acts.pin !== undefined) byId('sm-pin').value = String(acts.pin);
    if (acts.lockOpen) {
      byId('sm-lock-open').click();
      await sleep(20);
      result.afterLock = { text: byId('sm-msg').textContent, tone: byId('sm-msg').dataset.tone || '' };
    }
    if (acts.lockClose) {
      byId('sm-lock-close').click();
      await sleep(20);
    }
    if (acts.uid !== undefined) byId('sm-card-uid').value = String(acts.uid);
    if (acts.cardRole !== undefined) byId('sm-card-role').value = String(acts.cardRole);
    if (acts.cardAdd) {
      byId('sm-card-add').click();
      await sleep(20);
      result.afterCardAdd = { text: byId('sm-msg').textContent, tone: byId('sm-msg').dataset.tone || '' };
    }
    if (acts.cardLearn) {
      byId('sm-card-learn').click();
      await sleep(20);
    }
    if (acts.cardList) {
      byId('sm-card-list').click();
      await sleep(20);
    }
    if (acts.cardClear) {
      byId('sm-card-clear').click();
      await sleep(20);
    }
    if (acts.roleChange) {
      const button = byAttr('data-card-action', 'role')
        .find((el) => el.getAttribute('data-uid') === String(acts.roleChange));
      (byId('sm-card-list-box').events.click || [])
        .forEach((listener) => listener({ target: button }));
      await sleep(20);
      result.afterRoleChange = { text: byId('sm-msg').textContent,
                                 tone: byId('sm-msg').dataset.tone || '' };
    }
    if (acts.cardDel) {
      const button = byAttr('data-card-action', 'del')
        .find((el) => el.getAttribute('data-uid') === String(acts.cardDel));
      (byId('sm-card-list-box').events.click || [])
        .forEach((listener) => listener({ target: button }));
      await sleep(20);
    }
    if (acts.cargo) {
      const id = acts.cargo === 'end' ? 'sm-cargo-end' : 'sm-cargo-' + acts.cargo;
      byId(id).click();
      await sleep(20);
      result.afterCargo = { text: byId('sm-msg').textContent,
                            tone: byId('sm-msg').dataset.tone || '' };
    }
    if (acts.mag) {
      byId('sm-cal-' + acts.mag).click();
      await sleep(20);
    }
    result.callsAfter = calls.slice();
    result.logAfter = logItems();
  }

  console.log(JSON.stringify(result));
  process.exit(0);
})();
