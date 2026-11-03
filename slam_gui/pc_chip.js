/* Индикатор бортового компьютера для бортового экрана.
 *
 * Показывает температуру процессора и занятую память в одном тихом чипе
 * топбара. Данные приходят с борта: GET /api/system читает /proc и /sys и
 * отдаёт кэш. Датчика может не быть — тогда честно «ПК —», а не ноль.
 * Модуль ничего не пишет и не включает: только текст, класс и подсказка.
 */
(function () {
  'use strict';

  const POLL_MS = 5000;
  const STATE_NAMES = { ok: 'норма', warn: 'внимание', critical: 'критично' };

  const isNum = (value) => value !== null && value !== undefined && Number.isFinite(+value);

  function fmt(value, digits) {
    return isNum(value) ? (+value).toFixed(digits == null ? 1 : digits) : '—';
  }

  function uptimeText(seconds) {
    if (!isNum(seconds) || +seconds < 0) return '';
    const total = Math.floor(+seconds);
    const days = Math.floor(total / 86400);
    const hours = Math.floor((total % 86400) / 3600);
    if (days) return days + ' сут ' + hours + ' ч';
    const minutes = Math.floor((total % 3600) / 60);
    return hours ? hours + ' ч ' + minutes + ' мин' : minutes + ' мин';
  }

  /** Короткая надпись чипа и тон: ok / warn / err / none. */
  function short(sys) {
    if (!sys) return { text: 'ПК —', tone: 'none' };
    const temp = sys.temperature || {};
    const memory = sys.memory || {};
    const parts = [];
    if (isNum(temp.maxC)) parts.push('ЦП ' + Math.round(+temp.maxC) + ' °C');
    if (isNum(memory.usedPercent)) parts.push('ОЗУ ' + Math.round(+memory.usedPercent) + ' %');
    if (!parts.length) parts.push('ПК —');
    const tone = sys.status === 'critical' ? 'err'
      : sys.status === 'warn' ? 'warn'
        : sys.stale ? 'none' : 'ok';
    return { text: parts.join(' · '), tone: tone };
  }

  /** Подробности в подсказке: статус, цифры, пороги и причины «нет данных». */
  function details(sys, error) {
    if (!sys) return error || 'Состояние компьютера ещё не собрано';
    const temp = sys.temperature || {};
    const memory = sys.memory || {};
    const cpu = sys.cpu || {};
    const disk = sys.disk || {};
    const lines = ['Бортовой компьютер: ' + (STATE_NAMES[sys.status] || '—')];
    if (isNum(temp.maxC)) {
      lines.push('Температура ' + fmt(temp.maxC, 1) + ' °C'
        + (temp.hottest ? ' (' + temp.hottest + ')' : ''));
    }
    else if (temp.reason) lines.push('Температура: ' + temp.reason);
    if (isNum(cpu.usagePercent)) lines.push('Процессор ' + fmt(cpu.usagePercent, 0) + ' %');
    else if (isNum(cpu.load1)) lines.push('Средняя загрузка ' + fmt(cpu.load1, 2));
    if (isNum(memory.usedPercent)) {
      lines.push('Память ' + fmt(memory.usedPercent, 0) + ' %'
        + (isNum(memory.totalMB) ? ' из ' + memory.totalMB + ' МБ' : ''));
    }
    if (isNum(disk.usedPercent)) {
      lines.push('Диск ' + fmt(disk.usedPercent, 0) + ' %'
        + (isNum(disk.totalGB) ? ' из ' + fmt(disk.totalGB, 1) + ' ГБ' : ''));
    }
    const uptime = uptimeText(sys.uptimeS);
    if (uptime) lines.push('Включён ' + uptime);
    if (sys.stale) lines.push('Данные устарели');
    for (const warning of sys.warnings || []) lines.push('• ' + warning);
    if (error) lines.push(error);
    return lines.join('\n');
  }

  /** Отрисовка в существующий элемент. Возвращает его же — удобно для тестов. */
  function render(chip, sys, error) {
    if (!chip) return chip;
    const shown = short(sys);
    chip.textContent = shown.text;
    chip.className = 'chip pc-chip ' + shown.tone;
    chip.title = details(sys, error);
    return chip;
  }

  function poll(chip, url) {
    if (!chip || typeof fetch !== 'function') return Promise.resolve(null);
    const options = { cache: 'no-store' };
    const controller = typeof AbortController === 'function' ? new AbortController() : null;
    if (controller) options.signal = controller.signal;
    const timeout = controller ? setTimeout(function () { controller.abort(); }, 3000) : null;
    return fetch(url || 'api/system', options)
      .then(function (response) {
        if (!response.ok) throw new Error('http ' + response.status);
        return response.json();
      })
      .then(function (payload) {
        if (!payload || !payload.ok) throw new Error('нет данных');
        return render(chip, payload.data, payload.error);
      })
      .catch(function () {
        // Связь с бортом показывает data-chip; здесь только гасим индикатор,
        // чтобы старые цифры не выглядели свежими.
        if (chip && chip.classList && chip.classList.add) chip.classList.add('none');
        return null;
      })
      .then(function (result) {
        if (timeout) clearTimeout(timeout);
        return result;
      });
  }

  let timer = null;
  let pending = false;

  function tick(chip, url) {
    if (pending || document.hidden) return;
    pending = true;
    void poll(chip, url).then(function () { pending = false; });
  }

  /** Опрос каждые 5 секунд; без паузы при скрытой вкладке не копим запросы. */
  function start(chip, url) {
    if (!chip || timer) return;
    tick(chip, url);
    timer = setInterval(function () { tick(chip, url); }, POLL_MS);
    document.addEventListener('visibilitychange', function () {
      if (!document.hidden) tick(chip, url);
    });
  }

  window.RCPcChip = {
    POLL_MS: POLL_MS,
    short: short,
    details: details,
    render: render,
    poll: poll,
    start: start,
  };
})();
