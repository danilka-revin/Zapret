/* Панель «Лидар»: настройки порта/протокола/скорости/мотора без правки .env
   руками. Кнопка ставится в шапку пульта (8080) и бортового экрана (8090).
   Сохранение: пульт — вход оператора + CSRF, борт — PIN руления. */
(function () {
  'use strict';
  if (window.RCLidarSettings) return;
  const $ = (id) => document.getElementById(id);
  let csrfToken = '';
  let requiresPin = false;
  let opened = false;

  async function getJson(url, init) {
    const r = await fetch(url, { cache: 'no-store', ...(init || {}) });
    const body = await r.json().catch(() => null);
    return { r, body };
  }

  function esc(v) {
    return String(v == null ? '' : v).replace(/[&<>"']/g, (c) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
    }[c]));
  }

  function mount() {
    if ($('rc-lidar-launch')) return;
    const button = document.createElement('button');
    button.id = 'rc-lidar-launch';
    button.className = 'rc-lidar-launch';
    button.type = 'button';
    button.textContent = 'Лидар';
    button.title = 'Настройки лидара 360°: порт, протокол, скорость, мотор';

    const overlay = document.createElement('div');
    overlay.id = 'rc-lidar-overlay';
    overlay.className = 'rc-lidar-overlay';
    overlay.hidden = true;
    overlay.innerHTML = `
      <section class="rc-lidar-dialog" role="dialog" aria-modal="true" aria-labelledby="rc-lidar-title">
        <header class="rc-lidar-head"><div><h2 id="rc-lidar-title">Настройки лидара 360°</h2>
          <p>Сохраняются в .env и применяются сразу, без перезапуска. Бортовой экран подхватит их сам, а скан — у процесса, который держит порт.</p></div>
          <button type="button" class="rc-lidar-close" aria-label="Закрыть">×</button></header>
        <div class="rc-lidar-status" id="rc-lidar-status">Загружаю статус…</div>
        <form id="rc-lidar-form" class="rc-lidar-form">
          <label>Порт (пусто — автопоиск)
            <input id="rc-lidar-port" list="rc-lidar-ports" placeholder="/dev/serial/by-id/… или /dev/ttyUSB0" autocomplete="off" />
            <datalist id="rc-lidar-ports"></datalist>
          </label>
          <label>Протокол
            <select id="rc-lidar-protocol">
              <option value="auto">auto — подобрать самому</option>
              <option value="rplidar">rplidar — RPLIDAR A1/A2/A3/S1/C1</option>
              <option value="ld06">ld06 — LD06/LD19</option>
              <option value="lds01">lds01 — LDS-01/LDS-02 (TurtleBot3)</option>
            </select>
          </label>
          <label>Скорость, бод
            <select id="rc-lidar-baud">
              <option value="">авто — подобрать по модели</option>
              <option value="115200">115200 (A1/A2M8)</option>
              <option value="230400">230400 (LD06/LD19, LDS-01)</option>
              <option value="256000">256000 (A2M7/A2M12/A3/S1)</option>
              <option value="460800">460800 (C1)</option>
            </select>
          </label>
          <label>Мотор RPLIDAR (линия переходника)
            <select id="rc-lidar-motor">
              <option value="auto">auto — перебрать варианты</option>
              <option value="none">none — линии не трогать</option>
              <option value="dtr-low">dtr-low</option><option value="dtr-high">dtr-high</option>
              <option value="rts-low">rts-low</option><option value="rts-high">rts-high</option>
            </select>
          </label>
          <label class="rc-lidar-check"><input type="checkbox" id="rc-lidar-autoprobe" /> Перебирать все USB-порты (автопоиск)</label>
          <label id="rc-lidar-pin-wrap" hidden>PIN руления (нужен на бортовом экране)
            <input id="rc-lidar-pin" type="password" inputmode="numeric" autocomplete="off" placeholder="PIN" />
          </label>
          <p class="rc-lidar-msg" id="rc-lidar-msg" role="status"></p>
          <footer class="rc-lidar-actions">
            <button type="button" class="rc-lidar-btn" id="rc-lidar-rescan">Пересканировать</button>
            <button type="submit" class="rc-lidar-btn primary" id="rc-lidar-save">Сохранить и применить</button>
            <button type="button" class="rc-lidar-btn" id="rc-lidar-dismiss">Закрыть</button>
          </footer>
        </form>
      </section>`;
    const header = document.querySelector('.top-status, .head-actions, .top-right');
    if (header) header.append(button); else { button.classList.add('rc-lidar-floating'); document.body.append(button); }
    document.body.append(overlay);
    button.addEventListener('click', () => { opened = true; overlay.hidden = false; void load(); });
    overlay.querySelector('.rc-lidar-close').addEventListener('click', close);
    $('rc-lidar-dismiss').addEventListener('click', close);
    overlay.addEventListener('click', (e) => { if (e.target === overlay) close(); });
    window.addEventListener('keydown', (e) => { if (e.key === 'Escape' && opened) close(); });
    $('rc-lidar-form').addEventListener('submit', (e) => { e.preventDefault(); void save(); });
    $('rc-lidar-rescan').addEventListener('click', () => void rescan());
  }

  function close() { opened = false; const o = $('rc-lidar-overlay'); if (o) o.hidden = true; }
  function msg(text, bad) { const el = $('rc-lidar-msg'); if (el) { el.textContent = text || ''; el.dataset.bad = bad ? '1' : ''; } }

  function renderStatus(st) {
    const el = $('rc-lidar-status');
    if (!el || !st) return;
    const det = st.detected;
    const found = det && det.port ? `${det.port} · ${det.protocol || ''} · ${det.baud || ''} бод` : 'не найден';
    const usb = st.usb_diagnostics && st.usb_diagnostics.summary ? st.usb_diagnostics.summary : '';
    el.innerHTML = `<b>${st.connected ? 'Подключён' : 'Нет скана'}</b> · ${esc(st.source || '')} · точек: ${esc(st.points_count || 0)}<br>` +
      `<span>устройство: ${esc(found)} · причина: ${esc(st.reason || '')}${st.hint ? ' — ' + esc(st.hint) : ''}${usb ? '<br>' + esc(usb) : ''}</span>`;
    const b = $('rc-lidar-launch');
    if (b) b.dataset.state = st.connected ? 'ok' : 'bad';
  }

  async function load() {
    msg('');
    try {
      const { r, body } = await getJson('api/lidar/settings');
      if (!r.ok || !body || !body.ok) {
        msg((body && body.error) || 'Настройки недоступны: на веб-пульте нужен вход оператора.', true);
        if (body && body.status) renderStatus(body.status);
        return;
      }
      csrfToken = body.csrfToken || csrfToken;
      requiresPin = !!body.requiresPin;
      $('rc-lidar-pin-wrap').hidden = !requiresPin;
      const s = body.settings || {};
      $('rc-lidar-port').value = s.port || '';
      $('rc-lidar-protocol').value = s.protocol || 'auto';
      $('rc-lidar-baud').value = s.baud || '';
      $('rc-lidar-motor').value = s.motor || 'auto';
      $('rc-lidar-autoprobe').checked = s.autoprobe !== false;
      const list = $('rc-lidar-ports');
      list.innerHTML = ((body.options && body.options.ports) || []).map((p) => `<option value="${esc(p)}">`).join('');
      renderStatus(body.status);
    } catch (e) { msg('Сервер не отвечает — настройки лидара не загрузились.', true); }
  }

  function payload() {
    const body = {
      port: $('rc-lidar-port').value.trim(), protocol: $('rc-lidar-protocol').value,
      baud: $('rc-lidar-baud').value, motor: $('rc-lidar-motor').value,
      autoprobe: $('rc-lidar-autoprobe').checked
    };
    if (requiresPin) body.pin = $('rc-lidar-pin').value.trim();
    return body;
  }

  async function post(url, body) {
    const headers = { 'Content-Type': 'application/json' };
    if (csrfToken) headers['X-CSRF-Token'] = csrfToken;
    return getJson(url, { method: 'POST', headers, body: JSON.stringify(body || {}) });
  }

  async function save() {
    msg('Сохраняю в .env и перезапускаю поиск лидара…');
    try {
      const { r, body } = await post('api/lidar/settings', payload());
      if (!r.ok || !body || !body.ok) { msg((body && body.error) || 'Не удалось сохранить настройки.', true); return; }
      if (body.csrfToken) csrfToken = body.csrfToken;
      renderStatus(body.status);
      msg('Сохранено в .env и применено. Скан появится, как только порт ответит; бортовой экран подхватит настройки сам.');
    } catch (e) { msg('Сервер не отвечает при сохранении.', true); }
  }

  async function rescan() {
    msg('Пересканирую порты…');
    try {
      const body0 = requiresPin ? { pin: $('rc-lidar-pin').value.trim() } : {};
      const { r, body } = await post('api/lidar/rescan', body0);
      if (!r.ok || !body || !body.ok) { msg((body && body.error) || 'Не удалось пересканировать.', true); return; }
      renderStatus(body.status);
      msg('Поиск запущен заново с текущими настройками.');
    } catch (e) { msg('Сервер не отвечает при пересканировании.', true); }
  }

  function start() { mount(); window.RCLidarSettings = { open: () => { opened = true; $('rc-lidar-overlay').hidden = false; void load(); } }; }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start); else start();
})();
