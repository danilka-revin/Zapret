/* Стартовый экран бортового дисплея: ввод PIN и выбор режима.
 *
 * PIN проверяет сервер (POST api/kiosk_mode): страница не знает кодов и не
 * решает сама, какой экран открыть — она лишь переходит по адресу из ответа.
 * Так режим нельзя «включить» через адресную строку, а сервер на каждом
 * запросе видит его в подписанной cookie.
 */
(function () {
  'use strict';

  const MAX_DIGITS = 8;
  const MIN_DIGITS = 4;

  const $ = (id) => document.getElementById(id);
  const card = $('pin-card');
  const dots = $('pin-dots');
  const count = $('pin-count');
  const message = $('pin-message');
  const attempts = $('pin-attempts');
  const keypad = $('keypad');

  let pin = '';
  let busy = false;
  let locked = false;

  // ------------------------------------ часы ---------------------------------
  function tick() {
    const now = new Date();
    const clock = $('start-clock');
    const date = $('start-date');
    if (clock) {
      clock.textContent = now.toLocaleTimeString('ru-RU', { hour: '2-digit', minute: '2-digit' });
    }
    if (date) {
      date.textContent = now.toLocaleDateString('ru-RU', { day: '2-digit', month: 'long', weekday: 'short' });
    }
  }
  tick();
  setInterval(tick, 15000);

  // ------------------------------ состояние PIN ------------------------------
  function renderDots() {
    if (!dots) return;
    const shown = Math.max(pin.length, MIN_DIGITS);
    while (dots.children.length < shown) {
      const dot = document.createElement('span');
      dot.className = 'pin-dot';
      dots.appendChild(dot);
    }
    while (dots.children.length > shown) dots.removeChild(dots.lastElementChild);
    Array.from(dots.children).forEach((dot, index) => {
      dot.setAttribute('data-on', index < pin.length ? '1' : '0');
    });
    if (count) count.textContent = pin.length ? pin.length + ' цифр' : '0 цифр';
    updateSubmit();
  }

  function updateSubmit() {
    const submit = keypad && keypad.querySelector('[data-action="submit"]');
    if (submit) submit.disabled = locked || busy || pin.length < MIN_DIGITS;
  }

  function say(text, tone) {
    if (!message) return;
    message.textContent = text;
    if (tone) message.setAttribute('data-tone', tone);
    else message.removeAttribute('data-tone');
  }

  function shake() {
    if (!card) return;
    card.setAttribute('data-shake', '1');
    setTimeout(() => card.removeAttribute('data-shake'), 420);
  }

  function addDigit(digit) {
    if (busy || locked) return;
    if (pin.length >= MAX_DIGITS) {
      say('Не больше 8 цифр. Нажмите «Открыть».', 'err');
      return;
    }
    pin += digit;
    renderDots();
    say('Введите код целиком и нажмите «Открыть».');
  }

  function erase() {
    if (busy) return;
    pin = pin.slice(0, -1);
    renderDots();
  }

  function clearPin() {
    if (busy) return;
    pin = '';
    renderDots();
    say('Введите код с клавиатуры, цифрами на экране или обычной клавиатурой.');
  }

  function lockFor(remainingMs, attemptsLeft) {
    locked = true;
    const until = Date.now() + Math.max(0, remainingMs || 0);
    const submit = keypad && keypad.querySelector('[data-action="submit"]');
    function step() {
      const left = Math.ceil((until - Date.now()) / 1000);
      if (left > 0) {
        say('Слишком много попыток: подождите ' + left + ' с.', 'err');
        if (attempts) attempts.textContent = 'ввод заблокирован';
        setTimeout(step, 250);
        return;
      }
      locked = false;
      if (attempts) attempts.textContent = (attemptsLeft || 5) + ' попыток';
      say('Ввод снова доступен.');
      if (submit) submit.disabled = false;
      updateSubmit();
    }
    step();
    updateSubmit();
  }

  // ------------------------------ проверка на сервере -----------------------
  async function submit() {
    if (busy || locked) return;
    if (pin.length < MIN_DIGITS) {
      say('Нужен код из 4…8 цифр.', 'err');
      clearPin();
      return;
    }
    busy = true;
    updateSubmit();
    say('Проверяю код…');
    try {
      const response = await fetch('api/kiosk_mode', {
        method: 'POST',
        cache: 'no-store',
        credentials: 'same-origin',
        headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'kiosk-mode' },
        body: JSON.stringify({ pin }),
      });
      const body = await response.json().catch(() => ({}));
      if (response.ok && body.ok && body.page) {
        say(body.mode === 'monitor' ? 'Бортовой монитор: открываю…' : 'Бортовой экран: открываю…', 'ok');
        window.location.assign(body.page);
        return;
      }
      pin = '';
      renderDots();
      shake();
      if (body.reason === 'blocked') {
        lockFor(body.remainingMs || 30000, 0);
      } else if (body.reason === 'wrong') {
        const left = typeof body.attemptsLeft === 'number' ? body.attemptsLeft : null;
        say('Неверный PIN. ' + (left === null ? '' : 'Осталось попыток: ' + left + '.'), 'err');
        if (attempts) attempts.textContent = left === null ? 'неверный PIN' : left + ' попыток';
      } else if (body.reason === 'format') {
        say('PIN — 4…8 цифр.', 'err');
      } else {
        say('Код не принят: ' + (body.error || 'проверьте PIN') + '.', 'err');
      }
    } catch (error) {
      pin = '';
      renderDots();
      say('Нет связи с бортовым сервером. Повторите ввод.', 'err');
    } finally {
      busy = false;
      updateSubmit();
    }
  }

  // ---------------------------------- ввод ----------------------------------
  if (keypad) {
    keypad.addEventListener('click', (event) => {
      const button = event.target.closest('button');
      if (!button || button.disabled) return;
      if (button.dataset.digit) addDigit(button.dataset.digit);
      else if (button.dataset.action === 'erase') erase();
      else if (button.dataset.action === 'submit') submit();
    });
  }

  document.addEventListener('keydown', (event) => {
    if (event.ctrlKey || event.altKey || event.metaKey) return;
    if (/^[0-9]$/.test(event.key)) {
      addDigit(event.key);
      event.preventDefault();
    } else if (event.key === 'Backspace') {
      erase();
      event.preventDefault();
    } else if (event.key === 'Enter') {
      submit();
      event.preventDefault();
    } else if (event.key === 'Escape') {
      clearPin();
      event.preventDefault();
    }
  });

  // --------------------------- связь с бортовым сервером --------------------
  async function checkLink() {
    const dot = $('start-link-dot');
    const text = $('start-link-text');
    try {
      const response = await fetch('api/kiosk_mode', { cache: 'no-store', credentials: 'same-origin' });
      const body = await response.json();
      if (!response.ok || !body.ok) throw new Error('bad reply');
      if (dot) dot.setAttribute('data-tone', 'ok');
      if (text) {
        text.textContent = body.mode === 'monitor'
          ? 'Борт на связи · предыдущий режим: бортовой монитор'
          : body.mode === 'main'
            ? 'Борт на связи · предыдущий режим: бортовой экран'
            : 'Борт на связи · режим ещё не выбран';
      }
    } catch (error) {
      if (dot) dot.setAttribute('data-tone', 'warn');
      if (text) text.textContent = 'Бортовой сервер не отвечает — PIN проверить нельзя.';
    }
  }
  checkLink();
  setInterval(checkLink, 5000);

  renderDots();
})();
