/* Battery receipt controls. The onboard owner creates all data and bytes;
   the remote page uses web_controls.js's authenticated same-origin proxy. */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const panel = $('receipt-panel');
  const page = $('swipe-battery-view');
  if (!panel || !page) return;
  const print = $('receipt-print'), test = $('receipt-test'), cancel = $('receipt-cancel');
  const settings = $('receipt-settings'), confirm = $('receipt-confirm');
  const port = $('receipt-port'), preview = $('receipt-preview');
  const levels = [...panel.querySelectorAll('input[name="receipt-level"]')];
  let printer = null, loadedSettings = false, formDirty = false, pending = false, pollTimer = null;
  let previewSequence = 0, lastPreviewAt = 0, reportMessage = '', reportTone = '', errorUntil = 0;
  const field = {
    baudRate: $('receipt-baud'), framing: $('receipt-framing'), width: $('receipt-width'),
    profile: $('receipt-profile'), language: $('receipt-language'), charset: $('receipt-charset'),
    lineDelayMs: $('receipt-delay')
  };
  const level = () => (levels.find((item) => item.checked) || {value: 'medium'}).value;
  const freshBattery = () => {
    const b = window._kioskBms;
    const received = window._kioskBmsReceivedAt;
    const elapsed = (Date.now() - received) / 1000;
    // The server supplies ageS; only time elapsed on this browser is added.
    // Comparing its clock to the robot's wall clock would block remote users
    // whose phone or tablet has a different time. The server rechecks anyway.
    return Boolean(b && b.connected && b.data && typeof b.ageS === 'number' &&
      Number.isFinite(b.ageS) && b.ageS >= 0 && Number.isFinite(elapsed) && elapsed >= 0 && b.ageS + elapsed <= 5);
  };
  const message = (text, tone) => {
    reportMessage = text || '';
    reportTone = tone || '';
    if (tone === 'error') errorUntil = Date.now() + 10000;
    else if (tone === 'ok') errorUntil = 0;
    $('receipt-message').textContent = reportMessage;
    $('receipt-message').dataset.tone = reportTone;
  };
  const requestId = () => {
    if (window.crypto && window.crypto.randomUUID) return window.crypto.randomUUID();
    return Date.now().toString(36) + '-' + Math.random().toString(36).slice(2) + '-' + Math.random().toString(36).slice(2);
  };
  async function request(url, options) {
    const response = await fetch(url, Object.assign({cache: 'no-store'}, options || {}));
    let data;
    try { data = await response.json(); } catch (_) { throw new Error('Сервер не вернул состояние принтера'); }
    if (!response.ok || !data.ok) throw new Error(data.error || data.reason || ('HTTP ' + response.status));
    return data;
  }
  const post = (route, body) => request(route, {method: 'POST', headers: {
    'Content-Type': 'application/json', 'X-Robot-Control': 'printer'
  }, body: JSON.stringify(body)});

  function ports(items, preferred) {
    // Inventory normally lists a stable by-id alias while status.port is the
    // resolved ttyUSB path. Match the inventory's real device before falling
    // back to a missing saved path, otherwise a live printer looks disconnected.
    const match = Array.isArray(items) && items.find((entry) => entry &&
      (entry.port === preferred || entry.device === preferred));
    const old = formDirty ? port.value : (match ? match.port : preferred);
    const seen = new Set();
    const list = document.createDocumentFragment();
    const empty = document.createElement('option'); empty.value = ''; empty.textContent = 'Не выбран'; list.append(empty);
    (Array.isArray(items) ? items : []).forEach((entry) => {
      if (!entry || typeof entry.port !== 'string' || seen.has(entry.port)) return;
      seen.add(entry.port);
      const option = document.createElement('option');
      option.value = entry.port;
      option.textContent = entry.port + (entry.description ? ' · ' + entry.description : '') + (entry.note ? ' — ' + entry.note : '');
      const usb = /^\/dev\/(tty(?:USB|ACM)\d+|serial\/by-(?:id|path)\/[A-Za-z0-9_.:+-]+)$/.test(entry.port);
      option.disabled = !usb || ['bms', 'master', 'lidar', 'chassis', 'arduino', 'busy'].includes(entry.role);
      list.append(option);
    });
    if (old && !seen.has(old)) {
      const missing = document.createElement('option'); missing.value = old; missing.textContent = old + ' — сохранён, но сейчас не найден';
      list.append(missing);
    }
    port.replaceChildren(list);
    port.value = old || '';
    if (port.selectedIndex < 0) port.value = '';
  }

  function updateForm(data, force) {
    if (!data || !data.settings) return;
    if (!loadedSettings || force) {
      for (const [key, el] of Object.entries(field)) el.value = String(data.settings[key]);
      loadedSettings = true;
      formDirty = false;
    }
    // When a unique adapter was renumbered, use its current live path, not an old tty number.
    const preferred = data.port || data.settings.port || '';
    if (data.ports) ports(data.ports, preferred);
  }

  function render() {
    const busy = pending || Boolean(printer && printer.printing);
    const ready = Boolean(printer && printer.ready && !busy);
    const battery = freshBattery();
    print.disabled = !ready || !battery;
    test.disabled = !ready;
    cancel.hidden = !Boolean(printer && printer.printing);
    $('receipt-save').disabled = busy;
    $('receipt-refresh').disabled = busy;
    const requirements = $('receipt-requirements');
    const why = !printer || busy ? '' : !printer.configured
      ? 'Чтобы печатать, настройте и физически подтвердите порт принтера.'
      : !printer.ready ? printer.detail
      : !battery ? 'Для отчёта нужна свежая связь с BMS; пробная страница доступна без неё.' : '';
    requirements.textContent = why;
    requirements.hidden = !why;
    print.title = why;
    const badge = $('receipt-badge');
    if (busy) { badge.textContent = 'ИДЁТ ОТПРАВКА'; badge.dataset.tone = 'busy'; }
    else if (printer && printer.ready) { badge.textContent = 'USB-АДАПТЕР ВЫБРАН'; badge.dataset.tone = 'ok'; }
    else { badge.textContent = printer && printer.configured ? 'НУЖНА ПРОВЕРКА' : 'НЕ НАСТРОЕН'; badge.dataset.tone = ''; }
    if (printer && printer.printing && printer.job) {
      const j = printer.job;
      message((j.message || 'Печать…') + ' ' + (j.sentLines || 0) + ' / ' + (j.totalLines || 0) + ' строк.', '');
    } else if (printer && printer.job && Date.now() >= errorUntil && ['sent', 'error', 'cancelled'].includes(printer.job.state)) {
      message(printer.job.message, printer.job.state === 'sent' ? 'ok' : 'error');
    } else if (!reportMessage || reportTone === 'status') {
      const detail = printer ? printer.detail : 'Нет связи с бортовым сервером.';
      message(detail + (printer && printer.ready && !battery ? ' Для отчёта дождитесь свежих данных BMS.' : ''), 'status');
    }
  }

  async function poll(forceForm) {
    try {
      const data = await request('api/printer');
      printer = data;
      updateForm(data, Boolean(forceForm));
      render();
    } catch (_) {
      printer = null;
      message('Нет связи с бортовым сервером. Если печать уже запущена, её результат пока неизвестен.', 'error');
      render();
    }
    clearTimeout(pollTimer);
    pollTimer = setTimeout(() => { if (!page.hidden || (printer && printer.printing)) void poll(false); }, 3000);
  }

  async function loadPreview() {
    const token = ++previewSequence;
    try {
      const data = await request('api/printer/preview?level=' + encodeURIComponent(level()));
      if (token !== previewSequence) return;
      $('receipt-preview-text').textContent = data.lines.join('\n');
      $('receipt-preview-meta').textContent = data.lineCount + ' строк · ' + data.width + ' символов';
      lastPreviewAt = Date.now();
    } catch (error) {
      if (token !== previewSequence) return;
      $('receipt-preview-text').textContent = error.message;
      $('receipt-preview-meta').textContent = 'Нет свежего снимка';
    }
  }

  async function submitJob(isTest) {
    if (pending || (printer && printer.printing)) return;
    pending = true; message('Отправляем запрос на борт…', ''); render();
    try {
      const body = isTest ? {requestId: requestId()} : {requestId: requestId(), level: level()};
      const result = await post(isTest ? 'api/printer/test' : 'api/printer/print', body);
      message(result.job.message, '');
      await poll(false);
    } catch (error) {
      message(error.message + ' Если связь пропала после отправки, сначала проверьте принтер и статус задания.', 'error');
      await poll(false);
    } finally { pending = false; render(); }
  }

  $('receipt-preview-toggle').addEventListener('click', () => {
    preview.hidden = !preview.hidden;
    $('receipt-preview-toggle').setAttribute('aria-expanded', String(!preview.hidden));
    if (!preview.hidden) void loadPreview();
  });
  levels.forEach((radio) => radio.addEventListener('change', () => { if (!preview.hidden) void loadPreview(); }));
  print.addEventListener('click', () => { void submitJob(false); });
  test.addEventListener('click', () => { void submitJob(true); });
  cancel.addEventListener('click', async () => {
    try { const data = await post('api/printer/cancel', {}); message(data.message, ''); await poll(false); }
    catch (error) { message(error.message, 'error'); }
  });
  Object.values(field).concat([port, confirm]).forEach((el) => el.addEventListener('change', () => { formDirty = true; }));
  $('receipt-refresh').addEventListener('click', () => { void poll(false); });
  $('receipt-save').addEventListener('click', async () => {
    if (pending) return;
    pending = true; render();
    const body = {port: port.value, baudRate: Number(field.baudRate.value), framing: field.framing.value,
      width: Number(field.width.value), profile: field.profile.value, language: field.language.value,
      charset: field.charset.value, lineDelayMs: Number(field.lineDelayMs.value), confirmed: confirm.checked};
    try {
      const data = await post('api/printer/config', body);
      printer = data; confirm.checked = false; formDirty = false;
      message(body.port ? 'Настройки сохранены. Совпадение адаптера не подтверждает модель или печать.' : 'Принтер отключён.', 'ok');
      if (!preview.hidden) await loadPreview();
      await poll(true);
    } catch (error) { message(error.message, 'error'); }
    finally { pending = false; render(); }
  });
  new MutationObserver(() => { if (!page.hidden) void poll(false); }).observe(page, {attributes:true, attributeFilter:['hidden']});
  setInterval(() => {
    if (page.hidden && !(printer && printer.printing)) return;
    render();
    if (!preview.hidden && freshBattery() && Date.now() - lastPreviewAt > 5000) void loadPreview();
  }, 1000);
  if (!page.hidden) void poll(false);
}());
