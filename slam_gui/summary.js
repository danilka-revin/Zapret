/* ============================================================================
 * summary.js — вкладка «Сводка» основного экрана (main.html, порт 8080).
 *
 * Данные — только с робота: GET api/stats (robot_control/stats.py), журнал
 * аудита замка — api/audit, журнал событий процесса — api/kiosk_logs.
 * Если сервер не отвечает, экран пишет об этом и ничего не подставляет вместо
 * цифр: значения остаются «—».
 *
 * Вкладка открывается адресом #summary (пункт меню «Сводка»). Пока она открыта,
 * экран пульта (шасси, АКБ, камеры, лидар) прячется, main.js продолжает опрос.
 * Все строки из телеметрии выводятся через textContent — без innerHTML.
 * ========================================================================== */
(function () {
  'use strict';

  const POLL_MS = 2000;
  const HIDE_SELECTORS = [
    '.main > .top', '.main > .alerts', '.main > .kpis', '.main > .stage', '.main > .specs',
  ];
  const LOG_LIMIT = 80;
  const AUDIT_LIMIT = 12;

  const $ = (id) => document.getElementById(id);
  let csrfToken = '';
  let lastData = null;
  let busy = false;

  /* ------------------------------------------------------------ формат */
  function num(v, digits) {
    return (typeof v === 'number' && isFinite(v)) ? v.toFixed(digits) : '—';
  }
  function durHms(sec) {
    if (typeof sec !== 'number' || !isFinite(sec)) return '—';
    const s = Math.max(0, Math.round(sec));
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), r = s % 60;
    return [h, m, r].map((x) => String(x).padStart(2, '0')).join(':');
  }
  function dateTime(ts) {
    if (typeof ts !== 'number' || !isFinite(ts)) return '—';
    return new Date(ts * 1000).toLocaleString('ru-RU', { hour12: false });
  }
  function timeOnly(ts) {
    if (typeof ts !== 'number' || !isFinite(ts)) return '—';
    return new Date(ts * 1000).toLocaleTimeString('ru-RU', { hour12: false });
  }
  function setText(id, text) {
    const el = $(id);
    if (el) el.textContent = text;
  }
  function setState(text, tone) {
    const el = $('sum-state');
    if (!el) return;
    el.textContent = text;
    el.dataset.tone = tone || '';
  }
  function cell(text, cls) {
    const td = document.createElement('td');
    td.textContent = text;
    if (cls) td.className = cls;
    return td;
  }
  function emptyRow(tbody, colspan, text) {
    tbody.replaceChildren();
    const tr = document.createElement('tr');
    const td = document.createElement('td');
    td.colSpan = colspan;
    td.className = 'summary-empty';
    td.textContent = text;
    tr.appendChild(td);
    tbody.appendChild(tr);
  }
  function listItem(list, text, cls) {
    const li = document.createElement('li');
    if (cls) li.className = cls;
    li.textContent = text;
    list.appendChild(li);
  }
  function resultText(result) {
    if (result === 'ok') return 'доставлен';
    if (result === 'aborted') return 'прерван';
    return result || '—';
  }

  /* ------------------------------------------------------------ вид */
  function isOpen() {
    return window.location.hash === '#summary';
  }

  function applyView() {
    const view = $('summary');
    if (!view) return;
    const on = isOpen();
    view.hidden = !on;
    document.querySelectorAll(HIDE_SELECTORS.join(',')).forEach((el) => { el.hidden = on; });
    document.body.classList.toggle('is-summary', on);
    if (on) refreshAll();
  }

  /* ------------------------------------------------------------ данные */
  async function getJson(url) {
    const r = await fetch(url, { cache: 'no-store' });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    return r.json();
  }

  function renderSummary(s) {
    const t = s.totals || {};
    const live = s.live || {};
    setText('sum-since', dateTime(s.since));

    setText('sum-link', live.linkOk ? 'есть' : 'нет связи');
    $('sum-link') && $('sum-link').classList.toggle('bad', !live.linkOk);
    setText('sum-trip-now', live.currentTrip
      ? '№' + live.currentTrip.id + ' · ' + num(live.currentTrip.distanceM / 1000, 3) + ' км'
      : 'стоянка');
    setText('sum-sample', timeOnly(live.lastSampleAt));

    setText('k-trips', String(t.trips || 0));
    setText('k-trips-sub', 'прервано: ' + (t.tripsAborted || 0));
    setText('k-dist', num(s.distanceKm, 2));
    setText('k-energy', num(t.energyUsedWh, 0));
    setText('k-whkm', s.whPerKm === null || s.whPerKm === undefined ? '—' : num(s.whPerKm, 1));
    setText('k-moving', durHms(t.movingSec));
    setText('k-avg-sub', 'ср. скорость ' + num(s.avgSpeedKmh, 1) + ' км/ч');
    setText('k-maxv', num(t.maxSpeedMps, 2));
    setText('k-charged', num(t.energyChargedWh, 0));
    setText('k-cycles', String(t.chargeCycles || 0));
    setText('k-faults', String(t.faults || 0));
    setText('k-lock', String((t.lockOpen || 0)) + ' / ' + String(t.lockDenied || 0));
    setText('k-lock-sub', 'открытий / отказов · смен PIN: ' + (t.pinChanged || 0));

    renderTrips(s.recentTrips || []);
    renderChart(s.recentTrips || []);
    renderUnavailable(s.unavailable || []);
  }

  function renderTrips(trips) {
    const tbody = $('sum-trips');
    if (!tbody) return;
    if (!trips.length) {
      emptyRow(tbody, 8, 'рейсов ещё нет — рейс фиксируется по первому движению робота');
      return;
    }
    tbody.replaceChildren();
    trips.forEach((tr) => {
      const row = document.createElement('tr');
      row.appendChild(cell('№' + tr.id));
      row.appendChild(cell(dateTime(tr.startedAt)));
      row.appendChild(cell(tr.route || '—'));
      row.appendChild(cell(num((tr.distanceM || 0) / 1000, 3)));
      row.appendChild(cell(num(tr.energyUsedWh, 1)));
      row.appendChild(cell(durHms(tr.movingSec)));
      row.appendChild(cell(num(tr.avgSpeedMps, 2)));
      row.appendChild(cell(resultText(tr.result), tr.result === 'ok' ? 'ok' : 'bad'));
      tbody.appendChild(row);
    });
  }

  function renderChart(trips) {
    const canvas = $('sum-chart');
    const empty = $('sum-chart-empty');
    if (!canvas) return;
    const values = trips.slice().reverse().map((t) => Number(t.energyUsedWh) || 0);
    if (empty) empty.hidden = values.length > 0;
    const ratio = window.devicePixelRatio || 1;
    const cssW = canvas.clientWidth || 480;
    const cssH = canvas.clientHeight || 180;
    canvas.width = Math.round(cssW * ratio);
    canvas.height = Math.round(cssH * ratio);
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, cssW, cssH);
    if (!values.length) return;
    const max = Math.max(1, ...values);
    const pad = 22;
    const bw = (cssW - pad * 2) / Math.max(values.length, 1);
    const styles = getComputedStyle(canvas);
    const accent = styles.getPropertyValue('--accent').trim() || '#ffffff';
    const muted = styles.getPropertyValue('--muted').trim() || '#9a9aa0';
    ctx.fillStyle = muted;
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillText('макс. ' + max.toFixed(0) + ' Вт·ч', pad, 12);
    values.forEach((v, i) => {
      const h = (cssH - pad * 2) * (v / max);
      ctx.fillStyle = accent;
      ctx.globalAlpha = 0.85;
      ctx.fillRect(pad + i * bw + bw * 0.15, cssH - pad - h, bw * 0.7, h);
      ctx.globalAlpha = 1;
    });
    ctx.fillStyle = muted;
    ctx.fillText('0', 4, cssH - pad);
  }

  function renderUnavailable(items) {
    const list = $('sum-unavailable');
    if (!list) return;
    list.replaceChildren();
    items.forEach((item) => listItem(list, item));
  }

  async function refreshAudit() {
    const list = $('sum-audit');
    if (!list) return;
    try {
      const body = await getJson('api/audit?limit=' + AUDIT_LIMIT);
      list.replaceChildren();
      const items = (body && body.audit) || [];
      setText('sum-audit-meta', body && body.integrity && body.integrity.ok === false
        ? 'журнал повреждён' : 'последние ' + items.length);
      if (!items.length) listItem(list, 'записей доступа ещё нет');
      items.forEach((e) => {
        const verb = e.action === 'unlock' ? 'открытие' : (e.action === 'pin_change' ? 'смена PIN' : e.action);
        listItem(list, timeOnly(e.ts) + ' · ' + verb + ' · ' + (e.ok ? 'разрешено' : 'отказ')
          + (e.detail ? ' — ' + e.detail : ''), e.ok ? 'ok' : 'bad');
      });
    } catch (err) {
      list.replaceChildren();
      listItem(list, 'журнал доступа недоступен: ' + err.message, 'bad');
    }
  }

  async function refreshLog() {
    const list = $('sum-log');
    if (!list) return;
    try {
      const body = await getJson('api/kiosk_logs?limit=' + LOG_LIMIT);
      const data = (body && body.data) || {};
      const entries = data.entries || [];
      list.replaceChildren();
      setText('sum-log-meta', 'записей: ' + (data.count || entries.length));
      if (!entries.length) listItem(list, 'событий процесса пока нет');
      entries.slice().reverse().forEach((e) => {
        const src = e.source ? e.source + ': ' : '';
        listItem(list, (e.time || timeOnly(e.ts)) + ' · ' + src + (e.msg || ''), e.level || '');
      });
    } catch (err) {
      list.replaceChildren();
      listItem(list, 'журнал недоступен: ' + err.message, 'bad');
    }
  }

  async function refreshAll() {
    if (busy) return;
    busy = true;
    try {
      const body = await getJson('api/stats');
      if (!body || !body.ok) throw new Error((body && body.error) || 'нет ответа');
      csrfToken = body.csrfToken || csrfToken;
      lastData = body.data;
      renderSummary(body.data);
      const live = body.data.live || {};
      setState(live.linkOk
        ? 'связь с роботом есть — сводка пополняется в реальном времени'
        : 'связи с роботом нет: новые рейсы и пробег не учитываются', live.linkOk ? 'ok' : 'warn');
    } catch (err) {
      setState('сводка недоступна: ' + err.message, 'bad');
    } finally {
      busy = false;
    }
    refreshAudit();
    refreshLog();
  }

  /* ------------------------------------------------------------ экспорт */
  function downloadBlob(name, blob) {
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    setTimeout(() => { URL.revokeObjectURL(url); a.remove(); }, 0);
  }

  async function downloadReport(kind) {
    try {
      const r = await fetch('api/stats/report.' + kind, { cache: 'no-store' });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      downloadBlob('rus_slam_stats.' + kind, await r.blob());
      setState('отчёт ' + kind.toUpperCase() + ' сохранён', 'ok');
    } catch (err) {
      setState('отчёт не выгружен: ' + err.message, 'bad');
    }
  }

  function esc(v) {
    return String(v === null || v === undefined ? '—' : v)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  function printReport() {
    const s = lastData;
    if (!s) { setState('сначала дождитесь данных сводки', 'warn'); return; }
    const t = s.totals || {};
    const rows = (s.recentTrips || []).map((tr) => '<tr><td>' + esc('№' + tr.id) + '</td><td>'
      + esc(dateTime(tr.startedAt)) + '</td><td>' + esc(tr.route) + '</td><td>'
      + esc(num((tr.distanceM || 0) / 1000, 3)) + '</td><td>' + esc(num(tr.energyUsedWh, 1)) + '</td><td>'
      + esc(durHms(tr.movingSec)) + '</td><td>' + esc(resultText(tr.result)) + '</td></tr>').join('');
    const na = (s.unavailable || []).map((x) => '<li>' + esc(x) + '</li>').join('');
    const html = '<!DOCTYPE html><html lang="ru"><head><meta charset="utf-8">'
      + '<title>RUS SLAM — сводка работы</title><style>'
      + 'body{font:13px/1.45 Arial,Helvetica,sans-serif;color:#17211d;margin:24px}'
      + 'h1{font-size:19px;margin:0 0 4px}h2{font-size:14px;margin:18px 0 6px}'
      + 'table{border-collapse:collapse;width:100%}th,td{border:1px solid #cfd8d3;padding:4px 6px;font-size:11px;text-align:left}'
      + 'th{background:#eef3f0}.muted{color:#6a7570;font-size:11px}</style></head><body>'
      + '<h1>RUS SLAM — сводка работы робота</h1>'
      + '<div class="muted">НТЦ АО «АВТОВАЗ» · сформировано ' + esc(new Date().toLocaleString('ru-RU', { hour12: false }))
      + ' · с ' + esc(dateTime(s.since)) + ' · данные только с робота</div>'
      + '<h2>Показатели</h2><table>'
      + '<tr><th>Рейсов</th><td>' + esc(t.trips || 0) + ' (прервано ' + esc(t.tripsAborted || 0) + ')</td></tr>'
      + '<tr><th>Пробег, км</th><td>' + esc(num(s.distanceKm, 3)) + '</td></tr>'
      + '<tr><th>Энергия на разряд, Вт·ч</th><td>' + esc(num(t.energyUsedWh, 1)) + '</td></tr>'
      + '<tr><th>Энергия на заряд, Вт·ч</th><td>' + esc(num(t.energyChargedWh, 1)) + '</td></tr>'
      + '<tr><th>Расход, Вт·ч/км</th><td>' + esc(s.whPerKm === null || s.whPerKm === undefined ? '—' : num(s.whPerKm, 1)) + '</td></tr>'
      + '<tr><th>Время в движении</th><td>' + esc(durHms(t.movingSec)) + '</td></tr>'
      + '<tr><th>Макс. скорость, м/с</th><td>' + esc(num(t.maxSpeedMps, 2)) + '</td></tr>'
      + '<tr><th>Циклы заряда</th><td>' + esc(t.chargeCycles || 0) + '</td></tr>'
      + '<tr><th>Неисправности модулей</th><td>' + esc(t.faults || 0) + '</td></tr>'
      + '<tr><th>Замок: открытий / отказов / смен PIN</th><td>' + esc(t.lockOpen || 0) + ' / ' + esc(t.lockDenied || 0)
      + ' / ' + esc(t.pinChanged || 0) + '</td></tr></table>'
      + '<h2>Последние рейсы</h2>' + (rows
        ? '<table><tr><th>№</th><th>Начало</th><th>Маршрут</th><th>км</th><th>Вт·ч</th><th>Движение</th><th>Итог</th></tr>' + rows + '</table>'
        : '<div class="muted">Рейсы ещё не зафиксированы.</div>')
      + '<h2>Нет источника в телеметрии</h2><ul>' + na + '</ul>'
      + '<p style="margin-top:28px">Оператор смены: ____________________ &nbsp; Подпись: ____________________</p>'
      + '</body></html>';
    const w = window.open('', '_blank');
    if (!w) { setState('браузер заблокировал окно печати — разрешите всплывающие окна', 'warn'); return; }
    w.document.open();
    w.document.write(html);
    w.document.close();
    w.focus();
    setTimeout(() => { try { w.print(); } catch (e) { /* окно закрыто пользователем */ } }, 150);
  }

  async function resetSummary() {
    if (!window.confirm('Сбросить сводку? Рейсы, пробег, энергия и счётчики обнулятся. Журнал доступа к замку сохранится.')) return;
    if (!csrfToken) await refreshAll();
    try {
      const r = await fetch('api/stats/reset', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-CSRF-Token': csrfToken },
        body: '{}',
      });
      const body = await r.json().catch(() => ({}));
      if (!r.ok || !body.ok) throw new Error(body.error || 'HTTP ' + r.status);
      setState('сводка сброшена', 'warn');
      await refreshAll();
    } catch (err) {
      setState('сброс не выполнен: ' + err.message, 'bad');
    }
  }

  /* ------------------------------------------------------------ запуск */
  function boot() {
    if (!$('summary')) return;
    $('sum-csv') && $('sum-csv').addEventListener('click', () => downloadReport('csv'));
    $('sum-json') && $('sum-json').addEventListener('click', () => downloadReport('json'));
    $('sum-print') && $('sum-print').addEventListener('click', printReport);
    $('sum-reset') && $('sum-reset').addEventListener('click', resetSummary);
    window.addEventListener('hashchange', applyView);
    window.addEventListener('resize', () => { if (isOpen() && lastData) renderChart(lastData.recentTrips || []); });
    setInterval(() => { if (isOpen()) refreshAll(); }, POLL_MS);
    applyView();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();

  // для проверок в браузере и тестов
  window.RS_SUMMARY = { renderSummary, applyView, refreshAll, isOpen };
})();
