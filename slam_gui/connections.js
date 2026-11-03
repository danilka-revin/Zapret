(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const sections = ['usb', 'serial', 'network', 'bluetooth', 'peripherals', 'peers'];
  let busy = false;
  let last = null;
  let timer = null;

  function node(tag, text, className) {
    const el = document.createElement(tag);
    if (text !== undefined) el.textContent = String(text);
    if (className) el.className = className;
    return el;
  }
  function card(name, state, tone, fields) {
    const el = node('article', undefined, 'device');
    const head = node('div', undefined, 'device-head');
    head.appendChild(node('h3', name || 'Без имени'));
    head.appendChild(node('span', state, 'badge ' + (tone || '')));
    el.appendChild(head);
    const dl = node('dl');
    for (const [key, value] of fields) {
      if (value === undefined || value === null || value === '') continue;
      dl.appendChild(node('dt', key)); dl.appendChild(node('dd', value));
    }
    el.appendChild(dl);
    return el;
  }
  const netState = {up:'Активен', down:'Выключен', dormant:'Ожидание', unknown:'Неизвестно', lowerlayerdown:'Нет канала'};
  function device(type, d) {
    if (type === 'usb') return card(d.name, d.authorized ? 'Подключено' : 'Заблокировано ОС', d.authorized ? '' : 'warn',
      [['Порт USB', d.path], ['Производитель', d.manufacturer], ['VID:PID', d.vidPid], ['Серийный №', d.serial], ['Скорость', d.speed ? d.speed + ' Мбит/с' : '']]);
    if (type === 'serial') return card(d.path, d.present ? 'Присутствует' : 'Отсутствует', d.present ? '' : 'warn',
      [['Назначение', (d.roles || []).join(', ') || 'Не назначен'], ['Проверка', 'Без открытия порта']]);
    if (type === 'network') return card(d.name, netState[d.state] || d.state, d.state === 'up' ? '' : 'off',
      [['Тип', d.kind], ['MAC', d.mac], ['IP', (d.addresses || []).join(' · ') || 'Нет данных / не назначен'],
       ['Канал', d.carrier === '1' ? 'Есть' : d.carrier === '0' ? 'Нет' : 'Неизвестно'],
       ['Скорость', d.speed && d.speed !== '-1' ? d.speed + ' Мбит/с' : ''],
       ['Профиль NetworkManager', d.profile], ['Состояние NM', d.connectionState]]);
    if (type === 'peers') return card(d.address, 'Запись кэша', 'off',
      [['Интерфейс', d.interface], ['MAC', d.mac], ['Состояние ARP/NDP', (Array.isArray(d.state) ? d.state.join(', ') : d.state) || 'Неизвестно']]);
    if (type === 'bluetooth') return card(d.name || d.address, d.connected === true ? 'Подключено' : d.connected === false ? 'Не подключено' : 'Связь неизвестна', d.connected === true ? '' : 'off',
      [['Адрес', d.address], ['Сопряжено', d.paired === true ? 'Да' : d.paired === false ? 'Нет' : 'Неизвестно']]);
    return card(d.name || d.path, 'В ОС', '', [['Тип', d.kind], ['Устройство', d.path]]);
  }
  // ---------------------------------------------------- бортовой компьютер --
  const stateNames = {ok: 'норма', warn: 'внимание', critical: 'критично'};
  function fact(label, value, sub, tone) {
    const el = node('div', undefined, 'fact');
    el.appendChild(node('span', label, 'fact-label'));
    el.appendChild(node('b', value, 'fact-value' + (tone ? ' ' + tone : '')));
    if (sub) el.appendChild(node('span', sub, 'fact-sub'));
    return el;
  }
  function fmtUptime(sec) {
    if (!Number.isFinite(+sec) || +sec < 0) return '';
    const total = Math.floor(+sec), days = Math.floor(total / 86400);
    const hours = Math.floor((total % 86400) / 3600), minutes = Math.floor((total % 3600) / 60);
    if (days) return days + ' сут ' + hours + ' ч';
    return hours ? hours + ' ч ' + minutes + ' мин' : minutes + ' мин';
  }
  const isNum = v => v !== null && v !== undefined && Number.isFinite(+v);
  function renderSystem(sys, error) {
    const facts = $('system-facts');
    const zones = $('system-zones');
    const warning = $('system-warning');
    facts.replaceChildren(); zones.replaceChildren();
    const status = $('system-status');
    if (!sys) {
      status.textContent = '—';
      facts.appendChild(node('p', error || 'Борт ещё не прислал состояние компьютера'));
      warning.hidden = true;
      return;
    }
    const temp = sys.temperature || {}, cpu = sys.cpu || {}, mem = sys.memory || {}, disk = sys.disk || {};
    const known = temp.available || cpu.available || mem.available || disk.available;
    status.textContent = sys.status ? (stateNames[sys.status] || sys.status) : '—';
    if (!known) facts.appendChild(node('p', 'Сведения о компьютере недоступны'));

    const limits = sys.thresholds || {};
    const warnC = isNum(limits.warnTempC) ? +limits.warnTempC : 75;
    const critC = isNum(limits.critTempC) ? +limits.critTempC : 85;
    const tempTone = !isNum(temp.maxC) ? 'none'
      : temp.maxC >= critC ? 'crit' : temp.maxC >= warnC ? 'warn' : '';
    facts.appendChild(fact('Температура', isNum(temp.maxC) ? (+temp.maxC).toFixed(1) + ' °C' : 'нет датчика',
      temp.available ? (temp.hottest || '') + (isNum(temp.throttleCount) && temp.throttleCount > 0
        ? ' · троттлинг ' + temp.throttleCount : '') : (temp.reason || 'датчик не найден'), tempTone));

    const cpuSub = [];
    if (isNum(cpu.cores)) cpuSub.push(cpu.cores + ' ядер');
    if (isNum(cpu.load1)) cpuSub.push('средняя ' + (+cpu.load1).toFixed(2));
    if (isNum(cpu.frequencyMHz)) cpuSub.push(cpu.frequencyMHz + ' МГц');
    facts.appendChild(fact('Процессор', isNum(cpu.usagePercent) ? Math.round(cpu.usagePercent) + ' %' :
      (isNum(cpu.load1) ? (+cpu.load1).toFixed(2) + ' load' : '—'),
      cpu.model ? cpu.model + (cpuSub.length ? ' · ' + cpuSub.join(' · ') : '') : cpuSub.join(' · '),
      isNum(cpu.usagePercent) && cpu.usagePercent >= 90 ? 'warn' : ''));

    const memTone = !isNum(mem.usedPercent) ? 'none' : mem.usedPercent >= (limits.critMemoryPercent || 97)
      ? 'crit' : mem.usedPercent >= (limits.warnMemoryPercent || 90) ? 'warn' : '';
    facts.appendChild(fact('Память', isNum(mem.usedPercent) ? Math.round(mem.usedPercent) + ' %' : '—',
      isNum(mem.totalMB) ? (mem.usedMB + ' из ' + mem.totalMB + ' МБ')
        + (mem.swapTotalMB ? ' · подкачка ' + Math.round(mem.swapUsedPercent || 0) + ' %' : '')
        : (mem.reason || ''), memTone));

    const diskTone = !isNum(disk.usedPercent) ? 'none' : disk.usedPercent >= (limits.critDiskPercent || 97)
      ? 'crit' : disk.usedPercent >= (limits.warnDiskPercent || 90) ? 'warn' : '';
    facts.appendChild(fact('Диск', isNum(disk.usedPercent) ? Math.round(disk.usedPercent) + ' %' : '—',
      isNum(disk.totalGB) ? ('свободно ' + disk.freeGB + ' из ' + disk.totalGB + ' ГБ') : (disk.reason || ''), diskTone));

    const up = fmtUptime(sys.uptimeS);
    facts.appendChild(fact('Включён', up || '—', sys.stale ? 'данные устарели' : 'с момента загрузки', ''));

    for (const zone of temp.zones || []) {
      zones.appendChild(node('span', zone.name + ' ' + (+zone.celsius).toFixed(1) + ' °C',
        'zone' + (zone.celsius >= warnC ? ' hot' : '')));
    }
    const messages = (sys.warnings || []).slice();
    if (error) messages.push(error);
    warning.textContent = messages.join('\n');
    warning.hidden = !warning.textContent;
  }

  function render(data) {
    $('connections-host').textContent = 'Компьютер: ' + (data.hostname || '—');
    renderSystem(data.system, data.systemError);
    $('connections-updated').textContent = 'Обновление: ' + new Date(data.updatedAt * 1000).toLocaleTimeString('ru-RU');
    for (const type of sections) {
      const section = (data.sections || {})[type];
      const list = $(type + '-items');
      list.replaceChildren();
      const warning = $(type + '-warning');
      warning.textContent = section ? (section.warnings || []).join('\n') : 'Раздел не получен с борта';
      warning.hidden = !warning.textContent;
      let items = section ? section.items : [];
      if (type === 'bluetooth') {
        const bt = items || {};
        for (const adapter of bt.adapters || []) list.appendChild(card(adapter.name, 'Адаптер', '', [['Адрес', adapter.address]]));
        items = bt.devices || [];
      }
      const available = section && section.status !== 'unavailable';
      $(type + '-count').textContent = available ? String(items.length) : '—';
      for (const d of items) list.appendChild(device(type, d));
      if (!list.children.length) list.appendChild(node('p', !available ? 'Сведения недоступны' :
        section.status === 'partial' ? 'Нет доступных сведений об устройствах' : 'Устройства не обнаружены'));
    }
  }
  async function poll() {
    if (busy || document.hidden) return;
    busy = true;
    $('connections-refresh').disabled = true;
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 3500);
    try {
      const response = await fetch('/api/connections', {cache:'no-store', signal:controller.signal});
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const result = await response.json();
      if (!result.ok) throw new Error('Нет данных');
      if (result.data) { render(result.data); last = result.data; }
      const stale = !!result.error || !!(last && Date.now() / 1000 - last.updatedAt > 20);
      document.body.classList.toggle('is-stale', stale);
      $('connections-status').textContent = result.error || (stale ? 'Данные устарели' :
        !last ? 'Борт собирает список устройств…' : result.refreshing ? 'Обновление списка…' : 'Данные получены с борта');
    } catch (error) {
      document.body.classList.add('is-stale');
      $('connections-status').textContent = 'Нет связи с бортом' + (last ? ' · показаны последние данные' : ' · данные недоступны');
    } finally {
      clearTimeout(timeout);
      busy = false;
      $('connections-refresh').disabled = false;
      clearTimeout(timer);
      timer = setTimeout(poll, last ? 5000 : 1500);
    }
  }
  $('connections-refresh').addEventListener('click', poll);
  document.addEventListener('visibilitychange', () => { if (!document.hidden) void poll(); });
  void poll();
})();
