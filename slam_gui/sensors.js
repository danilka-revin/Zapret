/* Экран «Модуль датчиков» бортового экрана: замок, карты с ролями, окно погрузки.
 *
 * Плата живёт своей жизнью: сервер только показывает её состояние и передаёт
 * команды. Поэтому экран никогда не рисует «всё хорошо» из воздуха — нет кадра,
 * значит нет данных; нет ленты — значит габариты честно помечены как отключённые.
 * Концевика отсека нет, поэтому карточка показывает только замок.
 */
(function () {
  'use strict';

  const POLL_MS = 500;
  const CARDINALS = ['С', 'СВ', 'В', 'ЮВ', 'Ю', 'ЮЗ', 'З', 'СЗ'];
  const LIGHT_TITLES = {
    head: 'передний свет', stop: 'стопы', left: 'левый поворотник',
    right: 'правый поворотник', dim: 'габариты',
  };
  const ROLE_TITLES = {admin: 'администратор', user: 'пользователь'};
  const WINDOW_TITLES = {none: 'нет окна', load: 'погрузка', unload: 'выгрузка'};

  const state = {data: null, busy: false, timer: 0, lastCardSignature: ''};

  function $(id) { return document.getElementById(id); }

  function setText(el, text) {
    if (el && el.textContent !== String(text)) el.textContent = String(text);
  }

  function hasNumber(value) {
    return value !== null && value !== undefined && value !== '' && Number.isFinite(Number(value));
  }

  function fmt(value, digits) {
    if (!hasNumber(value)) return '—';
    return Number(value).toFixed(digits);
  }

  function cardinal(degrees) {
    if (!hasNumber(degrees)) return '';
    const index = Math.round(((Number(degrees) % 360) + 360) % 360 / 45) % 8;
    return CARDINALS[index];
  }

  function roleTitle(role) { return ROLE_TITLES[role] || 'нет'; }

  function message(text, tone) {
    const el = $('sm-msg');
    if (!el) return;
    el.textContent = text || '';
    el.dataset.tone = tone || '';
  }

  function pin() {
    const field = $('sm-pin');
    return field && typeof field.value === 'string' ? field.value.trim() : '';
  }

  function roleField() {
    const field = $('sm-card-role');
    const value = field && typeof field.value === 'string' ? field.value.trim() : '';
    return value === 'admin' ? 'admin' : 'user';
  }

  /* ------------------------------------------------------------ отрисовка */
  function renderStatus(data) {
    const badge = $('sm-status');
    const configured = data.configured;
    const ok = data.ok === true;
    const connected = data.connected === true || ok;
    let tone = 'err';
    let title = 'нет связи';
    let sub = 'Плата молчит: проверьте кабель и RC_SENSOR_PORT';
    if (configured === false) {
      title = 'не подключён';
      sub = 'Модуль датчиков не настроен: задайте RC_SENSOR_PORT в .env';
    } else if (ok) {
      tone = 'live';
      title = 'на связи';
      sub = 'Плата отвечает и присылает кадры';
    } else if (connected) {
      tone = 'warn';
      title = 'команда подтверждена';
      sub = 'Плата ответила по USB; свежая телеметрия датчиков пока не получена';
      if (data.error) sub += ' · ' + String(data.error);
    } else if (data.stale) {
      tone = 'warn';
      title = 'данные устарели';
      sub = 'Последний кадр старше двух секунд — команды могут не дойти';
    } else if (data.error) {
      // Сервер не ответил вовсе (configured неизвестен) — это не «ошибка порта».
      title = configured === true ? 'ошибка порта' : 'нет связи';
      sub = String(data.error);
    }
    if (badge) badge.dataset.tone = tone;
    setText($('sm-state'), title);
    setText($('sm-sub'), sub);
    setText($('sm-port'), data.port || '—');
    const age = Number(data.ageMs);
    setText($('sm-age'), Number.isFinite(age) ? (age / 1000).toFixed(1) + ' с назад' : '—');
    const errors = Array.isArray(data.errors) ? data.errors : [];
    setText($('sm-errors'), errors.length ? errors.join(', ') : (ok ? 'нет' : '—'));
  }

  // Концевика отсека пока нет (D8 не подключён): показываем только замок.
  function renderCompartment(data) {
    setText($('sm-lock'), data.lockOpen === true ? 'открыт'
      : data.lockOpen === false ? 'закрыт' : '—');
    const lockEl = $('sm-lock');
    if (lockEl) lockEl.dataset.state = data.lockOpen ? 'open' : 'closed';
  }

  function renderWindow(data) {
    const kind = data.windowKind || 'none';
    setText($('sm-window'), 'окно: ' + (kind === 'none' ? 'нет' : WINDOW_TITLES[kind] || kind));
    setText($('sm-window-state'), WINDOW_TITLES[kind] || 'нет окна');
    const box = $('sm-window');
    if (box) box.dataset.tone = kind === 'none' ? '' : 'ok';
  }

  function renderCompass(data) {
    const heading = data.heading;
    setText($('sm-heading'), hasNumber(heading) ? fmt(heading, 0) + '°' : '—');
    setText($('sm-cardinal'), cardinal(heading));
    const needle = $('sm-needle');
    if (needle && hasNumber(heading)) {
      needle.style.transform = 'rotate(' + Number(heading).toFixed(1) + 'deg)';
      needle.dataset.known = '1';
    } else if (needle) {
      needle.style.transform = 'rotate(0deg)';
      needle.dataset.known = '0';
    }
    const parts = [];
    parts.push(hasNumber(data.roll) ? 'крен ' + fmt(data.roll, 1) + '°' : 'крен —');
    parts.push(hasNumber(data.pitch) ? 'тангаж ' + fmt(data.pitch, 1) + '°' : 'тангаж —');
    parts.push(hasNumber(data.gyroZ) ? fmt(data.gyroZ, 1) + ' °/с' : '— °/с');
    setText($('sm-tilt'), parts.join(' · '));
  }

  function renderBarometer(data) {
    const hpa = Number(data.pressureHpa);
    if (Number.isFinite(hpa) && hpa > 0) {
      setText($('sm-pressure'), fmt(hpa, 1) + ' (' + fmt(hpa * 0.75006, 0) + ' мм рт. ст.)');
    } else {
      setText($('sm-pressure'), '—');
    }
    setText($('sm-temp'), hasNumber(data.temperatureC) ? fmt(data.temperatureC, 1) : '—');
    setText($('sm-alt'), hasNumber(data.altitudeM) ? fmt(data.altitudeM, 0) : '—');
  }

  function renderLights(data) {
    const lights = data.lights || {};
    const buttons = document.querySelectorAll ? document.querySelectorAll('[data-light]') : [];
    Array.prototype.forEach.call(buttons, (button) => {
      const name = button.getAttribute('data-light');
      const on = lights[name] === true;
      button.setAttribute('aria-pressed', on ? 'true' : 'false');
      button.dataset.state = on ? 'on' : 'off';
      // Габариты живут на ленте: без неё кнопку не обещаем.
      button.disabled = name === 'dim' && data.stripEnabled === false;
      button.title = button.disabled ? 'лента не подключена' : (LIGHT_TITLES[name] || name);
    });
    const stripNote = $('sm-strip-note');
    if (stripNote) stripNote.hidden = data.stripEnabled !== false;
    // Плата сама сообщает число диодов (nleds). Если оно не 73 — так и пишем:
    // лента могла быть обрезана или удлинена при монтаже.
    const leds = Number(data.stripLeds);
    const ledsNote = $('sm-strip-leds');
    if (ledsNote) {
      ledsNote.textContent = Number.isFinite(leds) && leds > 0
        ? 'плата сообщает: ' + leds + ' диодов'
          + (leds === 73 ? '' : ' — ожидалось 73, проверьте монтаж')
        : 'плата сообщает: старый прошивный протокол без счётчика';
    }
    const modeButtons = document.querySelectorAll ? document.querySelectorAll('[data-strip-mode]') : [];
    Array.prototype.forEach.call(modeButtons, (button) => {
      button.disabled = data.stripEnabled === false;
      button.title = button.disabled ? 'лента не подключена' : 'режим ленты на всё днище';
    });
    const slider = $('sm-dim');
    if (slider && hasNumber(data.dim)) {
      if (String(slider.value) !== String(data.dim)) slider.value = String(data.dim);
      setText($('sm-dim-value'), data.dim);
    }
  }

  /* Карточки карт: список приходит от платы (кадры @CARDINFO). */
  function cardSignature(list) {
    return list.map((item) => item.uid + ':' + item.role).join(',');
  }

  function renderCards(data) {
    const list = Array.isArray(data.cardList) ? data.cardList : [];
    setText($('sm-cards-count'), hasNumber(data.cards) ? data.cards : '—');
    const rfid = $('sm-rfid');
    const last = data.lastCard;
    if (rfid) {
      let text = 'карта: —';
      if (last) {
        const done = last.action === 'open' ? 'отсек открыт'
          : last.action === 'close' ? 'отсек закрыт'
            : last.action === 'enroll' ? 'карта записана'
              : 'отказ' + (last.reason && last.reason !== 'none'
                ? ': ' + (last.reasonTitle || last.reason) : '');
        text = 'карта ' + last.uid + ' (' + roleTitle(last.role) + ') — ' + done;
      }
      rfid.textContent = text;
      rfid.dataset.tone = last ? (last.granted ? 'ok' : 'err') : '';
    }

    const box = $('sm-card-list-box');
    if (!box) return;
    const signature = cardSignature(list);
    if (signature === state.lastCardSignature) return;
    state.lastCardSignature = signature;

    const rows = list.map((item) => {
      const row = document.createElement('li');
      row.className = 'card-row';
      row.setAttribute('data-uid', item.uid);
      row.setAttribute('data-role', item.role);
      const uid = document.createElement('span');
      uid.className = 'card-uid';
      uid.textContent = item.uid;
      const role = document.createElement('span');
      role.className = 'card-role';
      role.textContent = roleTitle(item.role);
      const flip = document.createElement('button');
      flip.type = 'button';
      flip.className = 'btn btn--tiny';
      flip.setAttribute('data-card-action', 'role');
      flip.setAttribute('data-uid', item.uid);
      flip.setAttribute('data-role', item.role === 'admin' ? 'user' : 'admin');
      flip.textContent = item.role === 'admin' ? 'сделать пользователем' : 'сделать администратором';
      const del = document.createElement('button');
      del.type = 'button';
      del.className = 'btn btn--tiny btn--danger';
      del.setAttribute('data-card-action', 'del');
      del.setAttribute('data-uid', item.uid);
      del.textContent = 'удалить';
      row.appendChild(uid);
      row.appendChild(role);
      row.appendChild(flip);
      row.appendChild(del);
      return row;
    });
    if (!rows.length) {
      const empty = document.createElement('li');
      empty.className = 'card-empty';
      empty.textContent = data.cardListKnown ? 'карт нет' : 'список не загружен';
      box.replaceChildren(empty);
      return;
    }
    box.replaceChildren.apply(box, rows);
  }

  function renderLog(data) {
    const list = $('sm-log');
    if (!list) return;
    const notes = (Array.isArray(data.events) ? data.events : [])
      .filter((event) => event && event.kind === 'note' && event.text)
      .slice(-6)
      .reverse();
    const items = notes.map((event) => {
      const li = document.createElement('li');
      li.textContent = event.text;
      return li;
    });
    if (items.length) list.replaceChildren.apply(list, items);
    else {
      const li = document.createElement('li');
      li.textContent = 'сообщений нет';
      list.replaceChildren(li);
    }
  }

  function render(data) {
    state.data = data || {};
    const safe = state.data;
    renderStatus(safe);
    renderCompartment(safe);
    renderWindow(safe);
    renderCards(safe);
    renderCompass(safe);
    renderBarometer(safe);
    renderLights(safe);
    renderLog(safe);
  }

  /* -------------------------------------------------------------- запросы */
  async function load() {
    if (state.busy) return;
    state.busy = true;
    const controller = typeof AbortController === 'function' ? new AbortController() : null;
    const timeout = controller ? setTimeout(() => controller.abort(), 3000) : 0;
    try {
      const response = await fetch('api/sensor_module', {
        cache: 'no-store', signal: controller ? controller.signal : undefined,
      });
      const body = await response.json();
      if (!body || body.ok !== true) throw new Error('сервер не отдал состояние');
      render(body.data || {});
    } catch (err) {
      const previous = state.data ? state.data.configured : null;
      render({ configured: previous, ok: false,
               error: 'нет ответа сервера: ' + String((err && err.message) || err) });
    } finally {
      if (timeout) clearTimeout(timeout);
      state.busy = false;
    }
  }

  async function post(url, body) {
    const response = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    let payload = {};
    try {
      payload = await response.json();
    } catch (err) {
      payload = {};
    }
    if (!response.ok || payload.ok === false) {
      throw new Error(payload.error || ('HTTP ' + response.status));
    }
    if (payload.data) render(payload.data);
    return payload;
  }

  async function act(label, url, body) {
    try {
      await post(url, body);
      message(label + ': команда отправлена', 'ok');
    } catch (err) {
      message(label + ': ' + String((err && err.message) || err), 'err');
    }
    await load();
  }

  function toggleLight(button) {
    if (!button || button.disabled) return;
    const name = button.getAttribute('data-light');
    const on = button.getAttribute('aria-pressed') !== 'true';
    act('свет — ' + (LIGHT_TITLES[name] || name), 'api/sensor_module/lights',
        { light: name, on });
  }

  function requirePin(what) {
    if (pin()) return true;
    // Без PIN сервер всё равно откажет: не тратим запрос впустую.
    message(what + ' требует PIN кейса', 'err');
    return false;
  }

  function cardCommand(label, body) {
    if (!requirePin('Изменение списка карт')) return;
    body.pin = pin();
    act('карты: ' + label, 'api/sensor_module/cards', body);
  }

  /* ------------------------------------------------------------ обработчики */
  function bind() {
    const lights = document.querySelectorAll ? document.querySelectorAll('[data-light]') : [];
    Array.prototype.forEach.call(lights, (button) => {
      button.addEventListener('click', () => toggleLight(button));
    });

    const off = $('sm-lights-off');
    if (off) off.addEventListener('click', () => act('свет', 'api/sensor_module/lights',
                                                     { all: true, on: false }));

    const stripModes = document.querySelectorAll ? document.querySelectorAll('[data-strip-mode]') : [];
    Array.prototype.forEach.call(stripModes, (button) => {
      button.addEventListener('click', () => {
        if (button.disabled) return;
        act('режим ленты — ' + button.textContent, 'api/sensor_module/strip',
            { mode: button.getAttribute('data-strip-mode') });
      });
    });

    const dim = $('sm-dim');
    if (dim) {
      dim.addEventListener('change', () => {
        const level = Number(dim.value);
        setText($('sm-dim-value'), level);
        act('яркость габаритов', 'api/sensor_module/lights', { dim: level });
      });
    }

    // Отсек: открытие (держится до карты или «Закрыть») и закрытие.
    const open = $('sm-lock-open');
    if (open) {
      open.addEventListener('click', () => {
        if (!requirePin('Открытие отсека')) return;
        act('отсек', 'api/sensor_module/lock', { hold: true, pin: pin() });
      });
    }
    const close = $('sm-lock-close');
    if (close) close.addEventListener('click', () => act('отсек', 'api/sensor_module/lock',
                                                         { open: false }));

    // Карты: добавить с ролью, записать с платы с ролью, обновить, очистить.
    const add = $('sm-card-add');
    if (add) {
      add.addEventListener('click', () => {
        const field = $('sm-card-uid');
        const uid = field && field.value ? field.value.trim().toUpperCase() : '';
        if (uid.length !== 8) {
          message('UID карты — 8 hex-знаков, например A1B2C3D4', 'err');
          return;
        }
        cardCommand('добавить ' + uid, { action: 'add', uid: uid, role: roleField() });
      });
    }
    const learn = $('sm-card-learn');
    if (learn) {
      learn.addEventListener('click', () => cardCommand('записать с платы',
                                                        { action: 'learn', role: roleField() }));
    }
    const list = $('sm-card-list');
    if (list) {
      list.addEventListener('click', () => act('карты: список', 'api/sensor_module/cards',
                                               { action: 'list' }));
    }
    const clear = $('sm-card-clear');
    if (clear) {
      clear.addEventListener('click', () => cardCommand('очистить', { action: 'clear' }));
    }

    // Кнопки в строках списка: сменить роль или удалить карту.
    const box = $('sm-card-list-box');
    if (box && box.addEventListener) {
      box.addEventListener('click', (event) => {
        const target = event && event.target;
        if (!target || !target.getAttribute) return;
        const action = target.getAttribute('data-card-action');
        const uid = target.getAttribute('data-uid');
        if (!action || !uid) return;
        if (action === 'role') {
          cardCommand('роль ' + uid, { action: 'role', uid: uid,
                                       role: target.getAttribute('data-role') || 'user' });
        } else if (action === 'del') {
          cardCommand('удалить ' + uid, { action: 'del', uid: uid });
        }
      });
    }

    // Окно погрузки: открытие требует PIN, закрытие — нет.
    const load = $('sm-cargo-load');
    if (load) {
      load.addEventListener('click', () => {
        if (!requirePin('Окно погрузки')) return;
        act('окно погрузки', 'api/sensor_module/cargo', { kind: 'load', pin: pin() });
      });
    }
    const unload = $('sm-cargo-unload');
    if (unload) {
      unload.addEventListener('click', () => {
        if (!requirePin('Окно выгрузки')) return;
        act('окно выгрузки', 'api/sensor_module/cargo', { kind: 'unload', pin: pin() });
      });
    }
    const end = $('sm-cargo-end');
    if (end) end.addEventListener('click', () => act('окно погрузки', 'api/sensor_module/cargo',
                                                     { kind: 'end' }));

    const mag = {'sm-cal-start': 'start', 'sm-cal-stop': 'stop', 'sm-cal-reset': 'reset'};
    Object.keys(mag).forEach((id) => {
      const button = $(id);
      if (!button) return;
      button.addEventListener('click', () => act('калибровка компаса', 'api/sensor_module/mag',
                                                 { action: mag[id] }));
    });
  }

  function init() {
    bind();
    load();
    if (typeof setInterval === 'function') state.timer = setInterval(load, POLL_MS);
  }

  const api = {init, load, render};
  if (typeof window !== 'undefined') window.RCSensors = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;

  if (typeof document !== 'undefined' && document.readyState === 'complete') init();
  else if (typeof document !== 'undefined' && document.addEventListener) {
    document.addEventListener('DOMContentLoaded', init);
  }
})();
