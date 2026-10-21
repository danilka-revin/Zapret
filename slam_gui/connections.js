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
  function render(data) {
    $('connections-host').textContent = 'Компьютер: ' + (data.hostname || '—');
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
