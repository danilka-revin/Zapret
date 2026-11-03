'use strict';
// Индикатор бортового компьютера на экране киоска: только текст и подсказка,
// никаких запросов кроме GET /api/pc_state, честное «ПК —» без датчиков.
const assert = require('assert');
const fs = require('fs');
const vm = require('vm');

class El {
  constructor() { this.textContent = ''; this.className = ''; this.title = ''; this.hidden = false;
    this.classes = new Set();
    this.classList = { add: c => this.classes.add(c), remove: c => this.classes.delete(c),
      contains: c => this.classes.has(c) }; }
}

const chip = new El();
let payload = null, fail = false, calls = [];
const document = { hidden: false, addEventListener: () => {} };
const context = {
  document, Date, AbortController,
  setTimeout: () => 1, clearTimeout: () => {}, setInterval: () => 1,
  fetch: async (url, init) => {
    calls.push([url, init]);
    if (fail) throw new Error('offline');
    return { ok: true, json: async () => payload };
  },
};
context.window = context;

const flush = async () => { await new Promise(resolve => setImmediate(resolve)); };

const READING = {
  ok: true, refreshing: false, error: null,
  data: {
    hostname: 'robot-pc', status: 'warn', stale: false, uptimeS: 7200,
    warnings: ['Процессор 80.4 °C — выше 75 °C, проверьте охлаждение'],
    cpu: { available: true, usagePercent: 37.5, load1: 0.42, cores: 4 },
    memory: { available: true, totalMB: 3900, usedPercent: 46.2 },
    disk: { available: true, totalGB: 29.7, usedPercent: 31 },
    temperature: { available: true, maxC: 80.4, hottest: 'cpu-thermal', zones: [] },
  },
};

(async () => {
  vm.runInNewContext(fs.readFileSync('slam_gui/pc_chip.js', 'utf8'), context);
  const Chip = context.window.RCPcChip;
  assert(Chip, 'модуль должен публиковать window.RCPcChip');

  // Пусто: без данных экран не выдумывает нули.
  const empty = Chip.short(null);
  assert.strictEqual(empty.text, 'ПК —');
  assert.strictEqual(empty.tone, 'none');
  Chip.render(chip, null, 'нет данных');
  assert.strictEqual(chip.textContent, 'ПК —');
  assert.match(chip.className, /\bnone\b/);
  assert.match(chip.title, /нет данных/);

  // Обычное чтение: температура и память, тон «внимание».
  Chip.render(chip, READING.data, null);
  assert.strictEqual(chip.textContent, 'ЦП 80 °C · ОЗУ 46 %');
  assert.strictEqual(chip.className, 'chip pc-chip warn');
  assert.match(chip.title, /Бортовой компьютер: внимание/);
  assert.match(chip.title, /Температура 80\.4 °C \(cpu-thermal\)/);
  assert.match(chip.title, /Процессор 38 %/);
  assert.match(chip.title, /Память 46 % из 3900 МБ/);
  assert.match(chip.title, /Диск 31 % из 29\.7 ГБ/);
  assert.match(chip.title, /Включён 2 ч 0 мин/);
  assert.match(chip.title, /проверьте охлаждение/);

  // Критично и норма — только тон и подпись.
  Chip.render(chip, Object.assign({}, READING.data, { status: 'critical' }), null);
  assert.strictEqual(chip.className, 'chip pc-chip err');
  assert.match(chip.title, /критично/);
  Chip.render(chip, Object.assign({}, READING.data, { status: 'ok' }), null);
  assert.strictEqual(chip.className, 'chip pc-chip ok');

  // Нет датчика температуры — показываем память и объясняем причину.
  const noTemp = Object.assign({}, READING.data, {
    status: 'ok',
    temperature: { available: false, maxC: null, reason: 'датчики температуры не найдены' },
  });
  Chip.render(chip, noTemp, null);
  assert.strictEqual(chip.textContent, 'ОЗУ 46 %');
  assert.match(chip.title, /Температура: датчики температуры не найдены/);

  // Устаревший снимок не выглядит свежим.
  Chip.render(chip, Object.assign({}, READING.data, { status: 'ok', stale: true }), null);
  assert.strictEqual(chip.className, 'chip pc-chip none');
  assert.match(chip.title, /Данные устарели/);

  // Опрос: ровно один GET /api/pc_state без тела и без кэша.
  payload = READING;
  chip.className = '';
  await Chip.poll(chip, 'api/pc_state');
  await flush();
  assert.strictEqual(chip.textContent, 'ЦП 80 °C · ОЗУ 46 %');
  assert.strictEqual(calls.length, 1);
  assert.strictEqual(calls[0][0], 'api/pc_state');
  assert.strictEqual(calls[0][1].cache, 'no-store');
  assert(!calls[0][1].method && !calls[0][1].body);
  assert(!/\bPOST\b|\bPUT\b/.test(JSON.stringify(calls[0][1])));

  // Сервер не ответил: индикатор гаснет, прошлые цифры не остаются «живыми».
  fail = true;
  await Chip.poll(chip);
  await flush();
  assert(chip.classes.has('none'));

  // start() подписывается на видимость вкладки и не дублирует таймер.
  fail = false; payload = READING; calls = [];
  Chip.start(chip, 'api/pc_state');
  Chip.start(chip, 'api/pc_state');
  await flush();
  assert.strictEqual(calls.length, 1, 'двойной start не должен слать два запроса');

  console.log('PC chip: temperature/memory text, thresholds, honest «нет данных», read-only GET OK');
})().catch(error => { console.error(error); process.exitCode = 1; });
