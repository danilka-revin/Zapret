/* Manual routing is persisted, never switched underneath moving wheels. */
(function () {
  'use strict';
  const panel = document.getElementById('wheel-port-settings');
  if (!panel) return;
  const fields = document.getElementById('wheel-port-fields');
  const message = document.getElementById('wheel-ports-msg');
  const save = document.getElementById('wheel-ports-save');
  const refresh = document.getElementById('wheel-ports-refresh');
  const titles = {FL: 'Переднее левое', FR: 'Переднее правое', RL: 'Заднее левое', RR: 'Заднее правое'};
  let loaded = false;
  let busy = false;
  async function load() {
    if (busy) return;
    busy = true;
    refresh.disabled = save.disabled = true;
    try {
      const response = await fetch('api/wheels/settings', {cache: 'no-store'});
      const data = await response.json();
      if (!response.ok || !data.ok) throw new Error(data.error || 'Не удалось получить порты');
      fields.replaceChildren();
      for (const [module, title] of Object.entries(titles)) {
        const label = document.createElement('label');
        label.className = 'cam-settings-field';
        const name = document.createElement('span');
        name.textContent = module + ' · ' + title;
        const select = document.createElement('select');
        select.dataset.module = module;
        select.setAttribute('aria-label', 'USB-порт: ' + title);
        select.add(new Option('Авто · по имени платы', ''));
        for (const port of data.ports || []) {
          const option = new Option(port.path + (port.reserved ? ' · занят/исключён' : !port.present ? ' · не подключён' : ''), port.path);
          option.disabled = port.reserved;
          select.add(option);
        }
        select.value = data.settings[module] || '';
        select.disabled = !data.canEdit;
        label.append(name, select);
        fields.append(label);
      }
      save.disabled = !data.canEdit;
      message.textContent = data.canEdit
        ? 'Выбор сохраняется в .env. Для применения остановите робот и перезапустите программу. Без телеметрии @TLM колесо останется не на связи.'
        : 'Менять порты нужно на бортовом экране (8090) → «Тяга и руление».';
      loaded = true;
    } catch (error) {
      message.textContent = 'Ошибка: ' + error.message;
    } finally {
      busy = false;
      refresh.disabled = false;
    }
  }
  panel.addEventListener('toggle', () => { if (panel.open && !loaded) load(); });
  refresh.addEventListener('click', load);
  save.addEventListener('click', async () => {
    if (busy) return;
    const settings = {};
    fields.querySelectorAll('select').forEach(select => { settings[select.dataset.module] = select.value; });
    busy = true;
    save.disabled = refresh.disabled = true;
    try {
      const response = await fetch('api/wheels/settings', {
        method: 'POST', headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'wheel-settings'},
        body: JSON.stringify(settings)
      });
      const data = await response.json();
      if (!response.ok || !data.ok) throw new Error(data.error || 'Не удалось сохранить');
      message.textContent = 'Сохранено. Остановите робот и перезапустите программу: до перезапуска действуют прежние назначения портов.';
    } catch (error) {
      message.textContent = 'Ошибка: ' + error.message;
    } finally {
      busy = false;
      save.disabled = refresh.disabled = false;
    }
  });
}());
