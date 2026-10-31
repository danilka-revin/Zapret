/* Live USB discovery status. Wheel roles come from fw22 CRC telemetry. */
(function () {
  'use strict';
  const panel = document.getElementById('wheel-port-settings');
  if (!panel) return;
  const fields = document.getElementById('wheel-port-fields');
  const message = document.getElementById('wheel-ports-msg');
  const refresh = document.getElementById('wheel-ports-refresh');
  const titles = {FL: 'Переднее левое', FR: 'Переднее правое', RL: 'Заднее левое', RR: 'Заднее правое'};
  let busy = false;

  async function load() {
    if (busy) return;
    busy = true;
    refresh.disabled = true;
    try {
      const response = await fetch('api/wheels/settings', {cache: 'no-store'});
      const data = await response.json();
      if (!response.ok || !data.ok) throw new Error(data.error || 'Не удалось получить состояние USB');
      fields.replaceChildren();
      for (const [module, title] of Object.entries(titles)) {
        const row = document.createElement('div');
        row.className = 'cam-settings-field';
        const name = document.createElement('span');
        name.textContent = module + ' · ' + title;
        const value = document.createElement('strong');
        value.textContent = (data.settings && data.settings[module]) || 'Не обнаружено';
        row.append(name, value);
        fields.append(row);
      }
      if (data.error) message.textContent = 'Ошибка: ' + data.error;
      else if (data.legacy && data.legacy.length) {
        message.textContent = 'Старая прошивка без защищённого имени: ' + data.legacy.join(', ') + '. Обновите до fw22; движение заблокировано.';
      } else {
        message.textContent = data.ready
          ? 'USB-порты определены автоматически. Проверьте физическое расположение плат и колёс.'
          : 'Проверяю USB-устройства. Если колесо не найдено, проверьте питание, кабель и прошивку fw22.';
      }
    } catch (error) {
      message.textContent = 'Ошибка: ' + error.message;
    } finally {
      busy = false;
      refresh.disabled = false;
    }
  }
  panel.addEventListener('toggle', () => { if (panel.open) void load(); });
  refresh.addEventListener('click', load);
  setInterval(() => { if (panel.open) void load(); }, 3000);
}());
