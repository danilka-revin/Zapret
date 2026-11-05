/* The onboard owner supplies every status. Hints are not permissions or proof. */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const cards = new Map();
  const ids = {camera_FL: 0, camera_FR: 2, camera_RL: 1, camera_RR: 3};
  let busy = false, online = false, started = false;
  let lastOptions = {};

  // On 8080 mutations need an authenticated operator and a CSRF token. The
  // local 8090 kiosk deliberately has no login and does not expose this route.
  const session = fetch('/api/control/session', {cache: 'no-store'}).then(async response => {
    if (response.status === 404) {
      document.body.dataset.surface = 'kiosk';
      document.querySelectorAll('.remote-only').forEach(el => { el.hidden = true; });
      return '';
    }
    if (!response.ok) throw new Error('Для изменений нужен вход оператора.');
    const data = await response.json();
    if (!data.ok || !data.csrfToken) throw new Error('Не удалось получить защитный токен.');
    document.body.dataset.surface = 'remote';
    return data.csrfToken;
  });
  session.catch(() => {});

  async function post(path, body) {
    const token = await session;
    const headers = {'Content-Type': 'application/json', 'X-Robot-Control': 'devices'};
    if (token) headers['X-CSRF-Token'] = token;
    const response = await fetch(path, {method: 'POST', headers, cache: 'no-store', body: JSON.stringify(body)});
    const data = await response.json().catch(() => null);
    if (!response.ok || !data || !data.ok) {
      throw new Error((data && (data.error || data.reason)) || 'Не удалось выполнить запрос (HTTP ' + response.status + ').');
    }
    return data;
  }

  function element(tag, cls, text) {
    const node = document.createElement(tag);
    if (cls) node.className = cls;
    if (text != null) node.textContent = text;
    return node;
  }
  function fact(parent, label, cls) {
    const wrap = element('div');
    wrap.appendChild(element('span', 'fact-label', label));
    const value = element('div', 'fact-value' + (cls ? ' ' + cls : ''), '—');
    wrap.appendChild(value); parent.appendChild(wrap);
    return value;
  }
  function kind(role) {
    return role.startsWith('wheel_') ? 'КОЛЕСО' : role.startsWith('camera_') ? 'КАМЕРА' :
      ({lidar: 'СКАНЕР', kinect: 'ГЛУБИНА', printer: 'ПЕРИФЕРИЯ', bms: 'АККУМУЛЯТОР'}[role] || 'USB');
  }
  function cardCode(role) {
    return role.includes('_') ? role.split('_')[1] : ({lidar: 'LID', kinect: '3D', printer: 'PRN', bms: 'BMS'}[role] || 'USB');
  }
  function optionsFor(role, all) {
    if (role.startsWith('camera_')) return all.video || [];
    if (role === 'kinect') return all.usb || [];
    if (role === 'printer') return [...(all.printer || []), ...(all.serial || []), ...(all.usb || [])];
    return all.serial || [];
  }
  function updateOptions(card, all) {
    const options = optionsFor(card.role, all);
    const key = JSON.stringify(options);
    if (card.optionsKey === key) return;
    card.optionsKey = key;
    card.datalist.replaceChildren();
    const seen = new Set();
    for (const option of options) {
      if (!option || !option.port || seen.has(option.port)) continue;
      seen.add(option.port);
      const node = document.createElement('option');
      node.value = option.port;
      node.label = [option.name, option.vidPid, option.topology || option.connection,
        option.index !== '' && option.index != null ? 'индекс ' + option.index : ''].filter(Boolean).join(' · ');
      card.datalist.appendChild(node);
      for (const alias of option.aliases || []) {
        if (seen.has(alias)) continue;
        seen.add(alias);
        const alt = document.createElement('option');
        alt.value = alias; alt.label = node.label; card.datalist.appendChild(alt);
      }
    }
  }

  function buildCard(item) {
    const role = item.id;
    const card = {role};
    const root = card.root = element('article', 'card');
    root.dataset.role = role;
    const head = element('header', 'card-head');
    const titles = element('div');
    titles.appendChild(element('div', 'card-kind', kind(role)));
    titles.appendChild(element('h3', '', item.name));
    head.append(titles, element('div', 'card-code', cardCode(role)));
    root.appendChild(head);
    const body = element('div', 'card-body'); root.appendChild(body);
    card.status = element('div', 'card-status');
    card.status.append(element('span', 'dot'), card.statusText = element('span', '', 'Ожидание'));
    body.appendChild(card.status);
    const facts = element('div', 'facts'); body.appendChild(facts);
    card.port = fact(facts, 'Текущий порт', 'port');
    card.usb = fact(facts, 'Подключение');
    card.check = fact(facts, 'Проверка');
    card.detail = element('p', 'detail'); body.appendChild(card.detail);
    if (role.startsWith('camera_')) {
      card.preview = element('div', 'preview', 'Нет свежего изображения');
      card.preview.setAttribute('aria-label', 'Изображение камеры для проверки расположения');
      body.appendChild(card.preview);
      card.confirm = element('button', 'confirm', 'Подтвердить расположение по изображению');
      card.confirm.type = 'button'; card.confirm.hidden = true;
      card.confirm.addEventListener('click', async () => {
        if (!window.confirm('Вы видите именно эту камеру? Проверьте расположение объектива на роботе.')) return;
        card.confirm.disabled = true;
        try {
          await post('/api/devices/camera/confirm', {role});
          card.message.textContent = 'Расположение подтверждено оператором.';
          await refresh(true);
        } catch (err) { card.message.textContent = err.message || 'Не удалось подтвердить камеру.'; }
        finally { card.confirm.disabled = false; }
      });
      body.appendChild(card.confirm);
    }
    if (role === 'bms') {
      card.link = element('a', 'card-link', 'Настроить BMS на бортовом экране →');
      card.link.href = '/kiosk?open=bms';
      session.then(token => {
        if (token) { card.link.href = '/battery'; card.link.textContent = 'АКБ · просмотр (настройка на борту) →'; }
      }).catch(() => { card.link.hidden = true; });
      body.appendChild(card.link);
    }
    card.remembered = element('p', 'remembered'); body.appendChild(card.remembered);
    const form = card.form = element('form', 'hint-form');
    const label = element('label', 'hint-label', 'Подсказать порт');
    const inputId = 'hint-' + role;
    label.htmlFor = inputId; form.appendChild(label);
    const row = element('div', 'hint-row');
    card.input = element('input');
    card.input.id = inputId; card.input.type = 'text'; card.input.autocomplete = 'off';
    card.input.spellcheck = false; card.input.maxLength = 256;
    card.input.placeholder = role.startsWith('camera_') ? '/dev/video…' : role === 'kinect' ? 'usb:1-2…' : '/dev/ttyUSB…';
    const listId = 'options-' + role;
    card.input.setAttribute('list', listId);
    card.input.addEventListener('input', () => { card.dirty = true; });
    card.datalist = element('datalist'); card.datalist.id = listId;
    card.save = element('button', 'save', 'Сохранить'); card.save.type = 'submit';
    card.clear = element('button', 'clear', 'Сброс'); card.clear.type = 'button';
    row.append(card.input, card.save, card.clear); form.append(row, card.datalist);
    card.hint = element('p', 'hint-caption'); form.appendChild(card.hint);
    card.message = element('p', 'hint-message'); card.message.setAttribute('role', 'status'); form.appendChild(card.message);
    if (role === 'lidar') {
      const instructions = element('p', 'detail', 'Молчащий LDS‑01/LDS‑02: сначала физически проследите кабель до USB‑UART и укажите его порт выше. Запуск доступен только на бортовом экране и может включить вращение лидара. USB-автопоиск экспериментальный: не эксплуатируйте робота до устранения известных гонок USB.');
      card.lidarStart = element('button', 'confirm', 'Проверить и запустить LDS‑01');
      card.lidarStart.type = 'button';
      card.lidarStart.hidden = true; // shown only after the local kiosk is identified
      session.then(token => { if (!token) card.lidarStart.hidden = false; }).catch(() => {});
      card.lidarStart.addEventListener('click', async () => {
        const port = card.input.value.trim();
        card.lidarStart.disabled = true;
        try {
          card.message.textContent = 'Проверяем текущий USB-порт…';
          const prepared = await post('/api/devices/lidar/prepare', {port});
          const confirmed = window.confirm('Проверьте кабель ФИЗИЧЕСКИ: ' + prepared.port + ' → ' + prepared.device +
            '. Это именно LDS‑01/LDS‑02 (HLS‑LFCD2) на 230400 бод, а не колесо, BMS или принтер? ' +
            'После подтверждения будет отправлен один байт запуска и лидар может начать вращаться.');
          if (!confirmed) { card.message.textContent = 'Запуск отменён; команду не отправляли.'; return; }
          card.message.textContent = 'Запускаем и ждём проверенные кадры LDS‑01…';
          const result = await post('/api/devices/lidar/start', {token: prepared.token, confirmed: true});
          card.message.textContent = result.message + (result.rescanError ? ' Перезапуск драйвера: ' + result.rescanError : '');
          await refresh(true);
        } catch (err) { card.message.textContent = err.message || 'Не удалось проверить LDS‑01.'; }
        finally { card.lidarStart.disabled = false; }
      });
      card.lidarInstructions = instructions;
    }
    async function save(port) {
      card.save.disabled = card.clear.disabled = true;
      card.message.textContent = 'Сохраняем…';
      try {
        await post('/api/devices/hint', {role, port});
        card.dirty = false;
        card.input.value = port;
        card.message.textContent = role.startsWith('camera_')
          ? (port ? 'Дождитесь изображения и подтвердите расположение.' : 'Камера отключена до выбора нового порта.')
          : role === 'bms' ? 'Подсказка сохранена. Порт BMS выбирается вручную на борту.'
          : 'Сохранено как подсказка, не как подтверждение устройства.';
        await refresh(true);
      } catch (err) { card.message.textContent = err.message || 'Не удалось сохранить подсказку.'; }
      finally { card.save.disabled = card.clear.disabled = false; }
    }
    form.addEventListener('submit', event => { event.preventDefault(); void save(card.input.value.trim()); });
    card.clear.addEventListener('click', () => {
      if (role.startsWith('camera_') && !window.confirm('Очистить привязку? Камера остановится до выбора нового порта.')) return;
      void save('');
    });
    body.appendChild(form);
    if (card.lidarStart) body.append(card.lidarInstructions, card.lidarStart);
    const container = role.startsWith('wheel_') ? $('wheel-cards') : role.startsWith('camera_') ? $('camera-cards') : $('other-cards');
    container.appendChild(root);
    cards.set(role, card);
    return card;
  }
  function age(value) {
    if (typeof value !== 'number' || !Number.isFinite(value) || value < 0) return '';
    return ' · ' + (value < 10 ? value.toFixed(1) : Math.round(value)) + ' с назад';
  }
  function date(value) {
    if (typeof value !== 'number' || !Number.isFinite(value)) return '';
    const d = new Date(value * 1000);
    return Number.isNaN(d.getTime()) ? '' : d.toLocaleString('ru-RU');
  }
  function describeHint(item) {
    const hint = item.hint;
    if (!hint) return item.id.startsWith('wheel_') ? 'Необязательно: влияет только на порядок поиска.' :
      item.id === 'bms' ? 'Подсказка не включает чтение BMS; порт задаётся вручную.' :
      'Подсказка будет перепроверена при каждом подключении.';
    if (hint.disabled) return 'Привязка очищена; камера отключена до нового выбора.';
    const match = item.rememberedMatch || {};
    const reason = {serial: 'совпал текущий USB-серийный номер', socket: 'совпало гнездо и модель, не личность',
      ambiguous: 'неоднозначное совпадение', changed: 'устройство не совпало или отсутствует',
      unverified: 'ещё нет проверенной USB-идентификации'}[match.confidence] || 'ожидает проверки';
    return 'Подсказка: ' + (hint.port || '—') + ' · ' + reason + (match.port ? ' → ' + match.port : '') + '.';
  }
  function renderCard(item) {
    const card = cards.get(item.id) || buildCard(item);
    card.root.dataset.state = item.status || 'missing';
    card.statusText.textContent = (item.label || 'Нет подтверждения') + age(item.ageS);
    card.port.textContent = item.port || ('Не подтверждён' + (item.reportedPort ? ' · ROS сообщает ' + item.reportedPort : '') +
      (item.configuredPort ? ' · настроен ' + item.configuredPort : ''));
    const usb = item.usb || item.usbCandidate;
    card.usb.textContent = usb ? [item.usbCandidate && !item.usb ? 'Возможный USB' : '', usb.product,
      usb.vidPid, usb.topology && 'USB ' + usb.topology, usb.connection].filter(Boolean).join(' · ') : 'Нет данных о USB-пути';
    card.check.textContent = item.check || 'Ожидание данных драйвера';
    card.detail.textContent = item.detail || '';
    card.hint.textContent = describeHint(item);
    const previous = item.remembered;
    card.remembered.textContent = previous ? 'Ранее: ' + (previous.port || '—') +
      (date(previous.verifiedAt) ? ' · ' + date(previous.verifiedAt) : '') +
      (previous.check ? ' · ' + previous.check : '') + ' (требует повторной проверки)' : 'Ранее наблюдавшийся порт не сохранён.';
    if (!card.dirty && document.activeElement !== card.input) card.input.value = item.hint && item.hint.port || '';
    if (card.confirm) card.confirm.hidden = !item.canConfirm;
    updateOptions(card, lastOptions);
  }

  const previews = window.RCCameraFrames ? window.RCCameraFrames.create((id, image) => {
    const role = Object.keys(ids).find(key => ids[key] === id);
    const card = cards.get(role);
    if (!card || !card.preview) return;
    if (image) { image.alt = 'Свежий кадр: проверьте расположение камеры'; card.preview.replaceChildren(image); }
    else { card.preview.textContent = 'Нет свежего изображения'; }
  }) : null;

  function markOffline(message) {
    online = false;
    $('server-dot').className = 'dot bad'; $('server-status').textContent = message;
    $('last-update').textContent = 'Текущее состояние неизвестно';
    $('discovery-status').textContent = 'Нет связи с бортом: готовность колёс неизвестна.';
    $('count-verified').textContent = $('count-warning').textContent = $('count-missing').textContent = '—';
    for (const card of cards.values()) {
      card.root.dataset.state = 'missing'; card.statusText.textContent = 'Состояние неизвестно';
      card.port.textContent = 'Неизвестно'; card.usb.textContent = 'Нет свежих данных';
      card.check.textContent = 'Нет связи с бортовым сервером';
      if (card.confirm) card.confirm.hidden = true;
    }
    if (previews) previews.stop();
  }
  async function refresh(force) {
    if (busy) return;
    busy = true; $('devices-refresh').disabled = true;
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 5000);
    try {
      const response = await fetch('/api/devices' + (force ? '?refresh=1' : ''), {cache: 'no-store', signal: controller.signal});
      if (!response.ok) throw new Error('Бортовой сервер недоступен (HTTP ' + response.status + ').');
      const data = await response.json();
      if (!data.ok || !Array.isArray(data.devices) || data.devices.length !== 12) throw new Error('Неполный ответ бортового сервера.');
      online = true; lastOptions = data.options || {};
      const verified = data.devices.filter(d => d.status === 'verified').length;
      const warning = data.devices.filter(d => d.status === 'warning' || d.status === 'conflict').length;
      $('count-verified').textContent = verified; $('count-warning').textContent = warning;
      $('count-missing').textContent = data.devices.length - verified - warning;
      $('server-dot').className = 'dot good'; $('server-status').textContent = 'Связь с бортовым сервером';
      $('last-update').textContent = 'Обновлено: ' + (date(data.generatedAt) || 'только что');
      const discovery = data.wheelDiscovery || {};
      const unknown = Array.isArray(discovery.unclassified) ? discovery.unclassified : [];
      const excluded = Array.isArray(discovery.excluded) ? discovery.excluded : [];
      $('discovery-status').textContent = 'Проверка USB колёс: ' + (discovery.ready ? 'завершена' : 'не завершена / требуется внимание') +
        (unknown.length ? ' · не опознаны: ' + unknown.join(', ') : '') +
        (excluded.length ? ' · исключены: ' + excluded.join(', ') : '') +
        (discovery.error ? ' · ' + discovery.error : '') + '. Это не разрешение на движение.';
      const warnings = [];
      if (!data.usbAvailable) warnings.push('USB-топология недоступна. Подключения и подсказки нельзя надёжно сверить.');
      if (data.storageError) warnings.push(data.storageError);
      if (Array.isArray(data.producerErrors)) warnings.push(...data.producerErrors);
      $('device-alert').hidden = !warnings.length;
      $('device-alert').textContent = warnings.join(' · ');
      for (const item of data.devices) renderCard(item);
      if (previews && !document.hidden && !started) { previews.start(); started = true; }
    } catch (err) {
      markOffline(err.name === 'AbortError' ? 'Бортовой сервер не ответил вовремя.' : (err.message || 'Нет связи с бортом.'));
      started = false;
    } finally { clearTimeout(timeout); busy = false; $('devices-refresh').disabled = false; }
  }
  $('devices-refresh').addEventListener('click', () => { void refresh(true); });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { if (previews) previews.stop(); started = false; }
    else { void refresh(true); if (online && previews && !started) { previews.start(); started = true; } }
  });
  window.addEventListener('pagehide', () => { if (previews) previews.stop(); started = false; });
  void refresh(true);
  setInterval(() => { if (!document.hidden) void refresh(false); }, 2500);
})();
