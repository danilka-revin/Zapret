/* Read-only fourth kiosk page. Every bank is rendered; no fixed cell limit. */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const page = $('swipe-battery-view');
  if (!page) return;
  let snapshot = null;
  let receivedAt = 0;
  let transportOk = false;
  const text = (id, value) => { const el = $(id); if (el) el.textContent = value; };
  const fmt = (value, decimals) => value == null || !Number.isFinite(Number(value)) ? '—' : Number(value).toFixed(decimals);

  function render() {
    window._kioskBms = transportOk ? snapshot : (snapshot ? { ...snapshot, connected: false } : null);
    const data = snapshot && snapshot.data;
    const fresh = transportOk && Date.now() - receivedAt < 5000 && snapshot && snapshot.connected;
    const available = Boolean(fresh && data);
    page.classList.toggle('bms-stale', !available);
    const reason = !transportOk ? 'Нет связи с сервером BMS' : snapshot.reason;
    text('bms-status', available ? reason : reason + ' · показания неактуальны');
    text('bms-updated', data
      ? 'Измерено: ' + new Date(data.updatedAt * 1000).toLocaleString('ru-RU') + (available ? '' : ' · последние известные данные')
      : 'Нет измерений. Настройте RC_BMS_PORT и подключите JBD BMS.');
    const report = $('bms-report');
    report.setAttribute('aria-disabled', available ? 'false' : 'true');
    if (available) report.setAttribute('href', 'api/battery/report.csv');
    else report.removeAttribute('href');
    const live = $('swipe-live');
    if (!page.hidden && live) live.textContent = available ? (snapshot.complete ? '● BMS' : 'НЕ ВСЕ БАНКИ') : 'НЕТ ДАННЫХ BMS';
    const summary = $('bms-summary');
    summary.replaceChildren();
    if (data) {
      const metrics = [
        ['Заряд', fmt(data.soc, 0) + ' %'], ['Напряжение', fmt(data.volts, 2) + ' В'],
        ['Ток (+ заряд / − разряд)', fmt(data.amps, 2) + ' А'], ['Мощность', fmt(data.watts, 1) + ' Вт'],
        ['Ёмкость: остаток / полная', fmt(data.remainingAh, 2) + ' / ' + fmt(data.fullAh, 2) + ' Ач'],
        ['Циклы', fmt(data.cycles, 0)],
        ['Температуры датчиков', data.temperatures.length ? data.temperatures.map((v, i) => 'T' + (i + 1) + ': ' + fmt(v, 1) + ' °C').join(' · ') : 'Нет датчиков'],
        ['Ключи BMS', 'Заряд: ' + (data.chargeOn ? 'вкл' : 'выкл') + ' · разряд: ' + (data.dischargeOn ? 'вкл' : 'выкл')]
      ];
      metrics.forEach(([label, value]) => {
        const item = document.createElement('div');
        const caption = document.createElement('span'); caption.textContent = label;
        const number = document.createElement('b'); number.textContent = value;
        item.append(caption, number); summary.append(item);
      });
    }
    text('bms-faults', data ? (data.faults.length ? 'Защиты: ' + data.faults.join(' · ') : 'Активных защит нет') : 'Защиты: нет данных');
    $('bms-faults').classList.toggle('bms-warning', Boolean(data && data.faults.length));
    text('bms-spread', data ? 'Мин: ' + data.minMv + ' мВ · макс: ' + data.maxMv + ' мВ · разброс: ' + data.deltaMv + ' мВ · среднее: ' + fmt(data.averageMv, 1) + ' мВ' : 'Разброс: —');
    text('bms-count', data ? 'Получено банок: ' + data.receivedCount + ' / ' + data.cellCount + (data.complete ? ' · полный отчёт' : ' · ВНИМАНИЕ: число банок не совпадает, статистика только по полученным') : 'Банки: нет данных');
    const body = $('bms-cells');
    body.replaceChildren();
    if (!data || !data.cells.length) {
      const row = document.createElement('tr'); const cell = document.createElement('td');
      cell.colSpan = 5; cell.textContent = 'Нет данных от BMS'; row.append(cell); body.append(row);
      return;
    }
    data.cells.forEach((bank) => {
      const row = document.createElement('tr');
      const tags = [];
      if (bank.mv == null) tags.push('НЕТ ДАННЫХ');
      else {
        if (bank.mv === data.minMv) tags.push('МИН');
        if (bank.mv === data.maxMv) tags.push('МАКС');
      }
      if (bank.number > data.cellCount) tags.push('СВЕРХ ЗАЯВЛЕННОГО ЧИСЛА');
      if (tags.length) row.className = 'bms-extreme';
      [String(bank.number).padStart(2, '0'), bank.mv == null ? '—' : fmt(bank.mv / 1000, 3) + ' В · ' + bank.mv + ' мВ',
        fmt(bank.deviationMv, 1) + ' мВ', bank.balancing ? 'Активна' : 'Нет', tags.join(' · ') || '—'
      ].forEach((value) => { const cell = document.createElement('td'); cell.textContent = value; row.append(cell); });
      body.append(row);
    });
  }

  async function poll() {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 2500);
    try {
      const response = await fetch('api/battery', { cache: 'no-store', signal: controller.signal });
      if (!response.ok) throw new Error('http ' + response.status);
      const result = await response.json();
      if (!result.ok || !result.battery) throw new Error('invalid battery response');
      snapshot = result.battery;
      receivedAt = Date.now();
      transportOk = true;
    } catch (error) {
      transportOk = false;
    } finally {
      clearTimeout(timer);
      render();
      setTimeout(poll, 1000);
    }
  }
  // Switching views must update the LIVE marker even between polls.
  new MutationObserver(render).observe(page, { attributes: true, attributeFilter: ['hidden'] });
  poll();
})();
