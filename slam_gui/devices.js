/* Страница устройств: ручное назначение портов + состояние драйверов.
   Автопоиска портов нет: программа не решает за оператора, где какое
   устройство. Подтверждение по-прежнему берётся только у драйверов. */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const cards = new Map();
  const ids = {camera_FL: 0, camera_FR: 2, camera_RL: 1, camera_RR: 3};
  const ROLE_ORDER = ['camera_FL', 'camera_FR', 'camera_RL', 'camera_RR',
    'lidar', 'kinect', 'printer', 'bms'];
  let busy = false, online = false, started = false;
  let portsData = null;
  let portsRequest = null;
  // Списки портов приходят из двух мест: /api/devices (USB-топология) и
  // /api/devices/ports (реальные /dev). Показываем объединение, без повторов.
  let snapshotOptions = null, portsOptions = null;

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
  function combinedOptions(kind) {
    const out = [], seen = new Set();
    for (const source of [portsOptions, snapshotOptions]) {
      for (const option of (source && source[kind]) || []) {
        if (!option || !option.port || seen.has(option.port)) continue;
        seen.add(option.port); out.push(option);
      }
    }
    return out;
  }
  // Списки портов показывают ВСЁ, что подключено, а не только подходящий тип:
  // оператор видит полную картину USB, а подходящий тип идёт первым.
  const KIND_TITLES = {serial: 'последовательный', video: 'видео', usb: 'USB',
    printer: 'принтер (USB)'};
  const KIND_ORDER = ['serial', 'video', 'usb', 'printer'];
  function kindFor(role) {
    if (role.startsWith('camera_')) return 'video';
    if (role === 'kinect') return 'usb';
    return 'serial';
  }
  function allOptions() {
    const out = [], seen = new Set();
    for (const kind of KIND_ORDER) {
      for (const option of combinedOptions(kind)) {
        const key = String(option.port).toLowerCase();
        if (seen.has(key)) continue;
        seen.add(key);
        out.push(Object.assign({}, option, {kind, kindTitle: KIND_TITLES[kind]}));
      }
    }
    return out;
  }
  function optionsFor(role) {
    const preferred = kindFor(role);
    return allOptions().sort((a, b) => {
      if ((a.kind === preferred) !== (b.kind === preferred)) return a.kind === preferred ? -1 : 1;
      if ((a.exists !== false) !== (b.exists !== false)) return a.exists !== false ? -1 : 1;
      return String(a.port).localeCompare(String(b.port));
    });
  }
  function optionLabel(option) {
    return [option.name, option.kindTitle, option.kinect ? 'Kinect' : '',
      option.exists === false ? 'не подключено' : 'подключено'].filter(Boolean).join(' · ');
  }
  function renderFound() {
    const node = $('ports-found');
    if (!node) return;
    const all = allOptions();
    if (!all.length) {
      node.textContent = 'Подключённых портов не найдено. Проверьте USB-кабели, питание и права ' +
        'dialout; список обновляется каждые 2,5 с и при «Обновить».';
      node.dataset.tone = 'warn';
      return;
    }
    const parts = KIND_ORDER.map(kind => [kind, all.filter(o => o.kind === kind).length])
      .filter(([, count]) => count > 0)
      .map(([kind, count]) => KIND_TITLES[kind] + ' ' + count);
    node.textContent = 'Подключено портов: ' + all.length + ' · ' + parts.join(' · ') +
      '. В каждом списке показаны все порты, подходящий тип — первым.';
    node.dataset.tone = 'good';
  }
  function placeholderFor(role) {
    if (role.startsWith('camera_')) return '/dev/v4l/by-id/…';
    if (role === 'kinect') return 'usb:1-2';
    return '/dev/serial/by-id/…';
  }
  function updateOptions(card) {
    if (!card.datalist) return; // у колёс общий блок портов, своего списка нет
    const options = optionsFor(card.role);
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
      node.label = optionLabel(option);
      card.datalist.appendChild(node);
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
    card.assigned = fact(facts, 'Назначенный порт', 'assigned');
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
      card.link = element('a', 'card-link', 'Подробнее об АКБ на бортовом экране →');
      card.link.href = '/kiosk?open=bms';
      session.then(token => {
        if (token) { card.link.href = '/battery'; card.link.textContent = 'АКБ · просмотр (настройка порта — здесь) →'; }
      }).catch(() => { card.link.hidden = true; });
      body.appendChild(card.link);
    }
    card.remembered = element('p', 'remembered'); body.appendChild(card.remembered);
    if (!role.startsWith('wheel_')) {
      const form = card.form = element('form', 'hint-form');
      const label = element('label', 'hint-label', 'Назначить порт');
      const inputId = 'hint-' + role;
      label.htmlFor = inputId; form.appendChild(label);
      const row = element('div', 'hint-row');
      card.input = element('input');
      card.input.id = inputId; card.input.type = 'text'; card.input.autocomplete = 'off';
      card.input.spellcheck = false; card.input.maxLength = 256;
      card.input.placeholder = placeholderFor(role);
      const listId = 'options-' + role;
      card.input.setAttribute('list', listId);
      card.input.addEventListener('input', () => { card.dirty = true; });
      card.datalist = element('datalist'); card.datalist.id = listId;
      card.save = element('button', 'save', 'Сохранить'); card.save.type = 'submit';
      card.clear = element('button', 'clear', 'Сброс'); card.clear.type = 'button';
      row.append(card.input, card.save, card.clear); form.append(row, card.datalist);
      card.message = element('p', 'hint-message'); card.message.setAttribute('role', 'status');
      form.appendChild(card.message);
      async function save(port) {
        card.save.disabled = card.clear.disabled = true;
        card.message.textContent = 'Сохраняем…';
        try {
          const result = await post('/api/devices/ports', {role, port});
          card.dirty = false;
          card.input.value = port;
          card.message.textContent = result.message ||
            (port ? 'Порт назначен.' : 'Назначение очищено.');
          if (role.startsWith('camera_')) {
            card.message.textContent += port
              ? ' Дождитесь изображения и подтвердите расположение.'
              : ' Камера выключена до выбора нового порта.';
          }
          if (result.ports) renderPorts(result.ports);
          await refresh(true);
        } catch (err) { card.message.textContent = err.message || 'Не удалось сохранить порт.'; }
        finally { card.save.disabled = card.clear.disabled = false; }
      }
      form.addEventListener('submit', event => { event.preventDefault(); void save(card.input.value.trim()); });
      card.clear.addEventListener('click', () => {
        if (role.startsWith('camera_') && !window.confirm('Очистить назначение? Камера остановится до выбора нового порта.')) return;
        void save('');
      });
      body.appendChild(form);
      if (role === 'printer') {
        card.printerNote = element('p', 'detail',
          'Сохранение подтверждает, что кабель физически прослежен до принтера. Программа не отправляет пробную печать.');
        body.appendChild(card.printerNote);
      }
    } else {
      card.wheelNote = element('p', 'remembered',
        'Порт задаётся один раз в блоке «Колёсные модули · 4 порта»: плата сама сообщает роль.');
      body.appendChild(card.wheelNote);
    }
    if (role === 'lidar') {
      const instructions = element('p', 'detail', 'Молчащий LDS‑01/LDS‑02: назначьте его порт выше и проследите кабель физически. Запуск доступен только на бортовом экране и может включить вращение лидара.');
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
      body.appendChild(instructions);
      body.appendChild(card.lidarStart);
    }
    const container = role.startsWith('wheel_') ? $('wheel-cards') : role.startsWith('camera_') ? $('camera-cards') : $('other-cards');
    container.appendChild(root);
    cards.set(role, card);
    return card;
  }

  /* ------------------------- блок назначения портов ------------------------ */
  function buildWheelSlots() {
    const holder = $('wheel-slots');
    if (!holder || holder.children.length) return;
    for (let index = 1; index <= 4; index += 1) {
      const label = element('label', 'wheel-slot');
      label.htmlFor = 'wheel-slot-' + index;
      label.appendChild(element('span', 'wheel-slot-title', 'Порт ' + index));
      const input = element('input');
      input.id = 'wheel-slot-' + index;
      input.type = 'text';
      input.autocomplete = 'off';
      input.spellcheck = false;
      input.maxLength = 256;
      input.placeholder = '/dev/serial/by-id/…';
      input.setAttribute('list', 'wheel-slots-options');
      input.addEventListener('input', () => { input.dataset.dirty = '1'; });
      label.appendChild(input);
      holder.appendChild(label);
    }
  }

  function wheelSlotValues() {
    const out = [];
    for (let index = 1; index <= 4; index += 1) {
      const input = $('wheel-slot-' + index);
      out.push(input ? input.value.trim() : '');
    }
    return out;
  }

  // Список портов для колёс не зависит от ответа /api/devices/ports: на пульте
  // 8080 он может быть недоступен, а порты из /api/devices уже известны.
  function renderWheelOptions() {
    const list = $('wheel-slots-options');
    if (list) {
      list.replaceChildren();
      for (const option of optionsFor('wheels')) {
        if (!option || !option.port) continue;
        const node = document.createElement('option');
        node.value = option.port;
        node.label = optionLabel(option);
        list.appendChild(node);
      }
    }
    renderFound();
  }

  function renderWheelSlots(data) {
    const wheels = (data && data.wheels) || {};
    const slots = Array.isArray(wheels.slots) ? wheels.slots : ['', '', '', ''];
    buildWheelSlots();
    for (let index = 1; index <= 4; index += 1) {
      const input = $('wheel-slot-' + index);
      if (!input) continue;
      if (document.activeElement !== input && input.dataset.dirty !== '1') {
        input.value = slots[index - 1] || '';
      }
    }
    renderWheelOptions();
    const assigned = slots.filter(Boolean).length;
    const state = $('wheel-slots-state');
    if (state) {
      state.textContent = assigned ? assigned + ' / 4 назначено' : 'не назначены';
      state.dataset.tone = assigned === 4 ? 'good' : assigned ? 'warn' : 'dim';
    }
    const map = $('wheel-slots-map');
    if (map) {
      const resolved = wheels.resolved || {};
      const parts = ['FL', 'FR', 'RL', 'RR'].map(id =>
        id + ' · ' + (resolved[id] || 'не опознано'));
      map.textContent = 'Опознание по телеметрии: ' + parts.join(' · ') +
        (wheels.error ? ' · ошибка: ' + wheels.error : '') +
        (wheels.ready ? ' · все четыре на связи' : ' · движение заблокировано до полного опознания');
    }
  }

  function renderPortsIndex(data) {
    const list = $('ports-index');
    if (!list || !data || !data.roles) return;
    list.replaceChildren();
    for (const role of ROLE_ORDER) {
      const item = data.roles[role];
      if (!item) continue;
      const row = element('li', 'ports-index-row');
      row.dataset.portRole = role; // не data-role: он принадлежит карточкам
      row.appendChild(element('span', 'ports-index-title', item.title));
      row.appendChild(element('span', 'ports-index-value' + (item.port ? '' : ' empty'),
        item.port || 'не назначен'));
      row.appendChild(element('span', 'ports-index-key', item.envKey || ''));
      list.appendChild(row);
    }
  }

  function renderPorts(data) {
    if (!data || !data.ok) return;
    portsData = data;
    portsOptions = data.options || null;
    renderWheelSlots(data);
    renderPortsIndex(data);
    for (const card of cards.values()) {
      if (data.roles && card.assigned) {
        const role = data.roles[card.role];
        const assigned = role && role.port ? role.port : '';
        card.assigned.textContent = assigned || (card.role.startsWith('camera_')
          ? 'не назначен — камера выключена' : 'не назначен');
        card.assigned.dataset.tone = assigned ? 'good' : 'dim';
        if (card.input && !card.dirty && document.activeElement !== card.input) {
          card.input.value = assigned;
        }
        updateOptions(card);
      }
    }
  }

  async function fetchPorts(force) {
    if (portsRequest && !force) return portsRequest;
    portsRequest = fetch('/api/devices/ports', {cache: 'no-store'})
      .then(response => response.ok ? response.json() : null)
      .then(data => { renderPorts(data); return data; })
      .catch(() => null)
      .finally(() => { portsRequest = null; });
    return portsRequest;
  }

  function bindPortsPanel() {
    const save = $('wheel-slots-save');
    const clear = $('wheel-slots-clear');
    const message = $('wheel-slots-message');
    async function commit(ports) {
      if (!save) return;
      save.disabled = true;
      if (clear) clear.disabled = true;
      message.textContent = 'Сохраняем порты колёс…';
      try {
        const result = await post('/api/devices/ports', {role: 'wheels', ports});
        message.textContent = result.message || 'Порты колёс сохранены.';
        for (let index = 1; index <= 4; index += 1) {
          const input = $('wheel-slot-' + index);
          if (input) delete input.dataset.dirty;
        }
        if (result.ports) renderPorts(result.ports);
        await refresh(true);
      } catch (err) {
        message.textContent = err.message || 'Не удалось сохранить порты колёс.';
      } finally {
        save.disabled = false;
        if (clear) clear.disabled = false;
      }
    }
    if (save) save.addEventListener('click', () => { void commit(wheelSlotValues()); });
    if (clear) clear.addEventListener('click', () => {
      if (!window.confirm('Очистить все четыре порта колёс? Движение будет заблокировано.')) return;
      for (let index = 1; index <= 4; index += 1) {
        const input = $('wheel-slot-' + index);
        if (input) { input.value = ''; delete input.dataset.dirty; }
      }
      void commit(['', '', '', '']);
    });
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
    $('discovery-status').textContent = 'Нет связи с бортом: состояние устройств неизвестно.';
    $('count-verified').textContent = $('count-warning').textContent = $('count-missing').textContent = '—';
    for (const card of cards.values()) {
      card.root.dataset.state = 'missing'; card.statusText.textContent = 'Состояние неизвестно';
      card.port.textContent = 'Неизвестно'; card.usb.textContent = 'Нет свежих данных';
      card.check.textContent = 'Нет связи с бортовым сервером';
      if (card.confirm) card.confirm.hidden = true;
    }
    const map = $('wheel-slots-map');
    if (map) map.textContent = 'Опознание колёс: нет связи с бортовым сервером.';
    if (previews) previews.stop();
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
    if (card.assigned) {
      const assigned = item.assignedPort || '';
      card.assigned.textContent = assigned || (card.role.startsWith('wheel_')
        ? 'задаётся списком выше'
        : card.role.startsWith('camera_') ? 'не назначен — камера выключена' : 'не назначен');
      card.assigned.dataset.tone = assigned ? 'good' : 'dim';
    }
    const previous = item.remembered;
    card.remembered.textContent = previous ? 'Ранее: ' + (previous.port || '—') +
      (date(previous.verifiedAt) ? ' · ' + date(previous.verifiedAt) : '') +
      (previous.check ? ' · ' + previous.check : '') + ' (требует повторной проверки)' : '';
    if (card.input && !card.dirty && document.activeElement !== card.input && portsData) {
      const role = portsData.roles && portsData.roles[item.id];
      if (role) card.input.value = role.port || '';
    }
    if (card.confirm) card.confirm.hidden = !item.canConfirm;
    updateOptions(card);
  }

  async function refresh(force) {
    if (busy) return;
    busy = true; $('devices-refresh').disabled = true;
    if (force) void fetchPorts(true);
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 5000);
    try {
      const response = await fetch('/api/devices' + (force ? '?refresh=1' : ''), {cache: 'no-store', signal: controller.signal});
      if (!response.ok) throw new Error('Бортовой сервер недоступен (HTTP ' + response.status + ').');
      const data = await response.json();
      if (!data.ok || !Array.isArray(data.devices) || data.devices.length !== 12) throw new Error('Неполный ответ бортового сервера.');
      online = true;
      const verified = data.devices.filter(d => d.status === 'verified').length;
      const warning = data.devices.filter(d => d.status === 'warning' || d.status === 'conflict').length;
      $('count-verified').textContent = verified; $('count-warning').textContent = warning;
      $('count-missing').textContent = data.devices.length - verified - warning;
      $('server-dot').className = 'dot good'; $('server-status').textContent = 'Связь с бортовым сервером';
      $('last-update').textContent = 'Обновлено: ' + (date(data.generatedAt) || 'только что');
      const discovery = data.wheelDiscovery || {};
      const unknown = Array.isArray(discovery.unclassified) ? discovery.unclassified : [];
      const excluded = Array.isArray(discovery.excluded) ? discovery.excluded : [];
      const manualPorts = Array.isArray(discovery.manualPorts) ? discovery.manualPorts : [];
      $('discovery-status').textContent = 'Автопоиск отключён · назначено портов колёс: ' +
        manualPorts.length + '/4 · опознано телеметрией: ' + (discovery.candidates || []).length + '/4' +
        (unknown.length ? ' · не опознаны: ' + unknown.join(', ') : '') +
        (excluded.length ? ' · исключены: ' + excluded.join(', ') : '') +
        (discovery.error ? ' · ' + discovery.error : '') + '. Это не разрешение на движение.';
      const warnings = [];
      if (!data.usbAvailable) warnings.push('USB-топология недоступна. Подключения нельзя надёжно сверить.');
      if (data.storageError) warnings.push(data.storageError);
      if (Array.isArray(data.producerErrors)) warnings.push(...data.producerErrors);
      $('device-alert').hidden = !warnings.length;
      $('device-alert').textContent = warnings.join(' · ');
      snapshotOptions = data.options || null;
      for (const item of data.devices) renderCard(item);
      if (portsData) renderWheelSlots(portsData);
      renderWheelOptions(); // список портов и счётчик — из всех источников
      if (previews && !document.hidden && !started) { previews.start(); started = true; }
    } catch (err) {
      markOffline(err.name === 'AbortError' ? 'Бортовой сервер не ответил вовремя.' : (err.message || 'Нет связи с бортом.'));
      started = false;
    } finally { clearTimeout(timeout); busy = false; $('devices-refresh').disabled = false; }
  }

  buildWheelSlots();
  bindPortsPanel();
  $('devices-refresh').addEventListener('click', () => { void refresh(true); });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { if (previews) previews.stop(); started = false; }
    else { void refresh(true); if (online && previews && !started) { previews.start(); started = true; } }
  });
  window.addEventListener('pagehide', () => { if (previews) previews.stop(); started = false; });
  void fetchPorts(true);
  void refresh(true);
  setInterval(() => { if (!document.hidden) void refresh(false); }, 2500);
})();
