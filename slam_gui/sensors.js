/* Экран «Модуль датчиков» бортового экрана: свет, замок двери, карты, компас.
 *
 * Плата живёт своей жизнью: сервер только показывает её состояние и передаёт
 * команды. Поэтому экран никогда не рисует «всё хорошо» из воздуха — нет кадра,
 * значит нет данных; нет ленты — значит габариты честно помечены как отключённые.
 */
(function () {
  'use strict';

  const POLL_MS = 500;
  const CARDINALS = ['С', 'СВ', 'В', 'ЮВ', 'Ю', 'ЮЗ', 'З', 'СЗ'];
  const LIGHT_TITLES = {
    head: 'передний свет', stop: 'стопы', left: 'левый поворотник',
    right: 'правый поворотник', dim: 'габариты',
  };

  const state = {
    data: null,
    busy: false,
    timer: 0,
  };

  function $(id) { return document.getElementById(id); }

  function setText(el, text) {
    if (el && el.textContent !== String(text)) el.textContent = String(text);
  }

  function fmt(value, digits) {
    if (value === null || value === undefined || !Number.isFinite(Number(value))) return '—';
    return Number(value).toFixed(digits);
  }

  function cardinal(degrees) {
    if (!Number.isFinite(Number(degrees))) return '';
    const index = Math.round(((Number(degrees) % 360) + 360) % 360 / 45) % 8;
    return CARDINALS[index];
  }

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

  /* ------------------------------------------------------------ отрисовка */
  function renderStatus(data) {
    const badge = $('sm-status');
    const configured = data.configured;
    const ok = data.ok === true;
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

  function renderCompass(data) {
    const heading = data.heading;
    setText($('sm-heading'), Number.isFinite(Number(heading)) ? fmt(heading, 0) + '°' : '—');
    setText($('sm-cardinal'), cardinal(heading));
    const needle = $('sm-needle');
    if (needle && Number.isFinite(Number(heading))) {
      needle.style.transform = 'rotate(' + Number(heading).toFixed(1) + 'deg)';
      needle.dataset.known = '1';
    } else if (needle) {
      needle.style.transform = 'rotate(0deg)';
      needle.dataset.known = '0';
    }
    const parts = [];
    parts.push(Number.isFinite(Number(data.roll)) ? 'крен ' + fmt(data.roll, 1) + '°' : 'крен —');
    parts.push(Number.isFinite(Number(data.pitch)) ? 'тангаж ' + fmt(data.pitch, 1) + '°' : 'тангаж —');
    parts.push(Number.isFinite(Number(data.gyroZ)) ? fmt(data.gyroZ, 1) + ' °/с' : '— °/с');
    setText($('sm-tilt'), parts.join(' · '));
  }

  function renderBarometer(data) {
    const hpa = Number(data.pressureHpa);
    if (Number.isFinite(hpa) && hpa > 0) {
      setText($('sm-pressure'), fmt(hpa, 1) + ' (' + fmt(hpa * 0.75006, 0) + ' мм рт. ст.)');
    } else {
      setText($('sm-pressure'), '—');
    }
    setText($('sm-temp'), Number.isFinite(Number(data.temperatureC)) ? fmt(data.temperatureC, 1) : '—');
    setText($('sm-alt'), Number.isFinite(Number(data.altitudeM)) ? fmt(data.altitudeM, 0) : '—');
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
    const slider = $('sm-dim');
    if (slider && Number.isFinite(Number(data.dim))) {
      if (String(slider.value) !== String(data.dim)) slider.value = String(data.dim);
      setText($('sm-dim-value'), data.dim);
    }
  }

  function renderDoor(data) {
    setText($('sm-door'), data.doorOpen === true ? 'открыта' : data.doorOpen === false ? 'закрыта' : '—');
    setText($('sm-lock'), data.lockOpen === true ? 'открыт' : data.lockOpen === false ? 'закрыт' : '—');
    const lockEl = $('sm-lock');
    if (lockEl) lockEl.dataset.state = data.lockOpen ? 'open' : 'closed';
  }

  function renderCards(data) {
    setText($('sm-cards-count'), Number.isFinite(Number(data.cards)) ? data.cards : '—');
    const rfid = $('sm-rfid');
    const last = data.lastCard;
    if (rfid) {
      rfid.textContent = last
        ? 'карта ' + last.uid + (last.granted ? ' — пущена' : ' — отказ')
        : 'карта: —';
      rfid.dataset.tone = last ? (last.granted ? 'ok' : 'err') : '';
    }
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
    renderCompass(safe);
    renderBarometer(safe);
    renderLights(safe);
    renderDoor(safe);
    renderCards(safe);
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
      const reason = payload.error || ('HTTP ' + response.status);
      throw new Error(reason);
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

  /* ------------------------------------------------------------ обработчики */
  function bind() {
    const buttons = document.querySelectorAll ? document.querySelectorAll('[data-light]') : [];
    Array.prototype.forEach.call(buttons, (button) => {
      button.addEventListener('click', () => toggleLight(button));
    });

    const off = $('sm-lights-off');
    if (off) off.addEventListener('click', () => act('свет', 'api/sensor_module/lights',
                                                     { all: true, on: false }));

    const dim = $('sm-dim');
    if (dim) {
      dim.addEventListener('change', () => {
        const level = Number(dim.value);
        setText($('sm-dim-value'), level);
        act('яркость габаритов', 'api/sensor_module/lights', { dim: level });
      });
    }

    const open = $('sm-lock-open');
    if (open) {
      open.addEventListener('click', () => {
        if (!pin()) {
          // Без PIN замок не дёргаем даже запросом: сервер его всё равно отклонит.
          message('Открытие двери требует PIN кейса', 'err');
          return;
        }
        act('замок', 'api/sensor_module/lock', { open: true, seconds: 3, pin: pin() });
      });
    }
    const close = $('sm-lock-close');
    if (close) close.addEventListener('click', () => act('замок', 'api/sensor_module/lock',
                                                         { open: false }));

    const cards = {
      'sm-card-add': { action: 'add', uid: true },
      'sm-card-del': { action: 'del', uid: true },
      'sm-card-learn': { action: 'learn' },
      'sm-card-list': { action: 'list' },
      'sm-card-clear': { action: 'clear' },
    };
    Object.keys(cards).forEach((id) => {
      const button = $(id);
      if (!button) return;
      const spec = cards[id];
      button.addEventListener('click', () => {
        const body = { action: spec.action };
        if (spec.uid) {
          const field = $('sm-card-uid');
          const uid = field && field.value ? field.value.trim().toUpperCase() : '';
          if (uid.length !== 8) {
            message('UID карты — 8 hex-знаков, например A1B2C3D4', 'err');
            return;
          }
          body.uid = uid;
        }
        if (spec.action !== 'list') {
          if (!pin()) {
            message('Изменение списка карт требует PIN кейса', 'err');
            return;
          }
          body.pin = pin();
        }
        act('карты: ' + spec.action, 'api/sensor_module/cards', body);
      });
    });

    const mag = {
      'sm-cal-start': 'start',
      'sm-cal-stop': 'stop',
      'sm-cal-reset': 'reset',
    };
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

  const api = { init, load, render };
  if (typeof window !== 'undefined') window.RCSensors = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;

  if (typeof document !== 'undefined' && document.readyState === 'complete') init();
  else if (typeof document !== 'undefined' && document.addEventListener) {
    document.addEventListener('DOMContentLoaded', init);
  }
})();
