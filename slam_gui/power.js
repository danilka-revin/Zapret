/* Кнопка «Остановить всё»: гасит пульт (8080), бортовой экран (8090),
   супервизор лидара ROS 2 и окно киоска — то же, что ./scripts/stop.sh.

   Почему окно с подтверждением, а не один клик: действие обрывает связь с
   роботом, и нажать его случайно (пальцем по дисплею борта) нельзя. Первый
   клик взводит кнопку, второй — останавливает. */

(function () {
  'use strict';

  if (window.RCStop) return;
  const $ = (id) => document.getElementById(id);
  const STAMP_KEY = 'rc_stop_last';
  let info = null;          // ответ GET api/system/stop
  let armed = false;        // первый клик взводит, второй останавливает
  let opened = false;

  async function getJson(url, init, timeout = 15000) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(url, { cache: 'no-store', ...init, signal: controller.signal });
      const body = await response.json().catch(() => null);
      return { response, body };
    } finally {
      clearTimeout(timer);
    }
  }

  function mount() {
    if ($('rc-stop-launch')) return;
    const button = document.createElement('button');
    button.id = 'rc-stop-launch';
    button.className = 'rc-stop-launch';
    button.type = 'button';
    button.textContent = 'Остановить всё';
    button.title = 'Остановить пульт, бортовой экран, лидар и окно киоска';

    const overlay = document.createElement('div');
    overlay.id = 'rc-stop-overlay';
    overlay.className = 'rc-stop-overlay';
    overlay.hidden = true;
    overlay.innerHTML = `
      <section class="rc-stop-dialog" role="dialog" aria-modal="true" aria-labelledby="rc-stop-title">
        <header class="rc-stop-head"><div><h2 id="rc-stop-title">Остановить робота</h2>
          <p>Останавливается всё, что поднял ярлык «Запустить робот»: пульт оператора (8080),
             бортовой экран на дисплее робота (8090), супервизор лидара ROS 2 (он освобождает USB-порт)
             и окно киоск-браузера. Автозапуск при этом не выключается.</p></div>
          <button type="button" class="rc-stop-close" aria-label="Закрыть">×</button></header>
        <div class="rc-stop-facts">
          <div><span>Что остановится</span><b id="rc-stop-targets">—</b></div>
          <div><span>Как вернуть</span><b id="rc-stop-restart">ярлык «Запустить робот»</b></div>
          <div><span>Автозапуск</span><b id="rc-stop-autostart">не меняется</b></div>
        </div>
        <p class="rc-stop-state" id="rc-stop-state" role="status">Готово к остановке.</p>
        <pre id="rc-stop-log" class="rc-stop-log" hidden></pre>
        <footer class="rc-stop-actions">
          <button type="button" class="rc-stop-btn danger" id="rc-stop-run">Остановить всё</button>
          <button type="button" class="rc-stop-btn" id="rc-stop-dismiss">Отмена</button>
        </footer>
      </section>`;

    const header = document.querySelector('.top-status, .head-actions, .top-right');
    if (header) header.append(button);
    else { button.classList.add('rc-stop-floating'); document.body.append(button); }
    document.body.append(overlay);

    button.addEventListener('click', () => void open());
    overlay.querySelector('.rc-stop-close').addEventListener('click', close);
    $('rc-stop-dismiss').addEventListener('click', close);
    overlay.addEventListener('click', (event) => { if (event.target === overlay) close(); });
    $('rc-stop-run').addEventListener('click', () => void run());
    window.addEventListener('keydown', (event) => {
      if (event.key === 'Escape' && opened) close();
    });

    // Доступность остановки зависит от входа оператора на пульте (на борту
    // входа нет: там запрос помечается заголовком, как команды движения).
    void check();
  }

  async function check() {
    const button = $('rc-stop-launch');
    try {
      const { response, body } = await getJson('api/system/stop');
      if (response.status === 404 || response.status === 405) {
        // Сервер без этой кнопки: не показываем то, что не работает.
        if (button) button.remove();
        return;
      }
      if (!response.ok || !body || !body.ok) {
        info = null;
        if (button) {
          button.dataset.state = 'blocked';
          button.title = (body && body.error) || 'Остановка недоступна';
        }
        return;
      }
      info = body;
      if (button) button.dataset.state = '';
    } catch (error) {
      info = null;
      if (button) button.title = 'Сервер не отвечает';
    }
  }

  async function open() {
    opened = true;
    armed = false;
    const overlay = $('rc-stop-overlay');
    if (overlay) overlay.hidden = false;
    setState('Проверяю, доступна ли остановка…');
    if (!info) await check();
    if (!info) {
      setState('Остановка недоступна: нужен вход оператора в пульт (8080).', 'error');
      const run = $('rc-stop-run');
      if (run) run.disabled = true;
      return;
    }
    const run = $('rc-stop-run');
    if (run) { run.disabled = false; run.textContent = 'Остановить всё'; }
    const targets = (Array.isArray(info.targets) && info.targets.length)
      ? info.targets.join(', ')
      : 'пульт 8080, бортовой экран 8090, лидар ROS 2';
    setText('rc-stop-targets', targets);
    setText('rc-stop-restart', info.restart || 'ярлык «Запустить робот»');
    setText('rc-stop-autostart', info.autostart || 'не меняется');
    setState('Готово к остановке.');
  }

  function close() {
    opened = false;
    const overlay = $('rc-stop-overlay');
    if (overlay) overlay.hidden = true;
  }

  function setText(id, text) {
    const node = $(id);
    if (node) node.textContent = text;
  }

  function setState(text, tone) {
    const state = $('rc-stop-state');
    if (!state) return;
    state.textContent = text;
    state.dataset.tone = tone || '';
  }

  async function run() {
    if (!info) return;
    const button = $('rc-stop-run');
    if (!armed) {
      // Второй клик по той же кнопке: защита от случайного нажатия пальцем.
      armed = true;
      if (button) button.textContent = 'Подтвердить остановку';
      setState('Ещё раз — и серверы выключатся. Связь с роботом пропадёт.', 'warn');
      return;
    }
    if (button) button.disabled = true;
    setState('Останавливаю: пульт, экран, лидар и окно киоска…', 'warn');
    const headers = { 'Content-Type': 'application/json', 'X-Robot-Control': 'stop' };
    if (info.csrfToken) headers['X-CSRF-Token'] = info.csrfToken;
    try {
      const { response, body } = await getJson('api/system/stop', {
        method: 'POST', headers, body: '{}',
      }, 20000);
      if (!response.ok || !body || !body.ok) {
        if (button) button.disabled = false;
        armed = false;
        if (button) button.textContent = 'Остановить всё';
        setState('Не удалось остановить: ' + ((body && body.error) || ('HTTP ' + response.status)), 'error');
        return;
      }
      showStopped(body);
    } catch (error) {
      // Сервер мог успеть погаснуть до ответа — это тоже успех остановки.
      showStopped({});
    }
  }

  function showStopped(body) {
    const stamp = new Date().toLocaleString('ru-RU', { hour12: false });
    try {
      localStorage.setItem(STAMP_KEY, JSON.stringify({ at: stamp, log: body.log || 'var/stop.log' }));
    } catch (error) { /* приватный режим браузера — не мешает остановке */ }

    setText('rc-stop-title', 'Робот остановлен');
    const restart = body.restart || (info && info.restart) || 'ярлык «Запустить робот»';
    setState('Пульт и бортовой экран остановлены, лидар и окно киоска закрыты. '
      + 'Вернуть: ' + restart + '.', 'ok');
    const run = $('rc-stop-run');
    if (run) { run.disabled = true; run.textContent = 'Остановлено'; }
    const log = $('rc-stop-log');
    if (log) {
      log.hidden = false;
      log.textContent = 'журнал остановки: ' + (body.log || 'var/stop.log') + '\nвремя: ' + stamp;
    }
    const button = $('rc-stop-launch');
    if (button) {
      button.disabled = true;
      button.dataset.state = 'stopped';
      button.textContent = 'Остановлено';
    }
    // Флаг для остальных скриптов экрана: сервер выключен намеренно, значит
    // «связь потеряна» — это ожидаемое состояние, а не авария.
    window.RCStopped = true;
  }

  function start() {
    mount();
    window.RCStop = { open: () => void open(), stopped: () => !!window.RCStopped };
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
