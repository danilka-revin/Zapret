/* Guest monitor, without hardware. Observe the displayed values and actual requests. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm');
const els = Object.fromEntries(['soc', 'soc-bar', 'volts', 'amps', 'temp', 'soc-note', 'batt-note',
  'speed', 'chassis-meta'].map(k => ['mon-' + k, {textContent: '', style: {}}]));
const timers = new Map(), intervals = new Map(), requests = [];
let id = 0, battery, speed = 1.25, stateOnline = true;
const sample = {soc: 72, volts: 50.6, amps: -1.4, temperatures: [22, 25.2], directionText: 'Разряд'};
const ctx = {
  location: {search: '?m=guest%2Btoken'}, URLSearchParams, AbortController,
  document: {getElementById: k => els[k], addEventListener() {}}, addEventListener() {},
  setTimeout(fn, ms) { timers.set(++id, {fn, ms}); return id; }, clearTimeout(k) { timers.delete(k); },
  setInterval(fn, ms) { intervals.set(ms, fn); },
  async fetch(url, init) {
    requests.push({url, init});
    let data;
    if (url.startsWith('api/battery?')) {
      if (battery === undefined) return new Promise((_, reject) => init.signal.addEventListener('abort', reject));
      if (battery === 'error') throw new Error('offline');
      data = {ok: true, battery};
    } else if (url.startsWith('api/kiosk_state?')) {
      if (!stateOnline) throw new Error('offline');
      // Must not be mistaken for a live JBD sample.
      data = {ok: true, data: {soc: 99, wheelSpeedMps: speed, motors: [{online: true, id: 'FL'}]}};
    } else data = {ok: true};
    return {ok: true, json: async () => data};
  },
};
ctx.window = ctx;
const text = key => els['mon-' + key].textContent;
const settle = () => new Promise(r => setImmediate(r));
async function runTimer(ms) {
  const entry = [...timers].find(([, task]) => task.ms === ms);
  assert(entry, 'missing timer ' + ms);
  timers.delete(entry[0]); entry[1].fn(); await settle();
}
(async () => {
  vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../../slam_gui/monitor.js'), 'utf8'), ctx);
  await settle();
  assert.equal(text('speed'), '1.25 м/с', 'a pending BMS request does not delay chassis telemetry');
  assert.match(text('chassis-meta'), /1 из 4/);
  await runTimer(2500); // request never returned
  assert.equal(text('soc'), '—');
  assert.match(text('soc-note'), /Нет связи/);
  battery = {connected: true, data: sample};
  stateOnline = false; await intervals.get(700)(); await runTimer(1000);
  assert.equal(text('speed'), '— м/с');
  assert.equal(text('soc'), '72'); assert.equal(els['mon-soc-bar'].style.width, '72%');
  assert.equal(text('volts'), '50.6 В'); assert.equal(text('amps'), '-1.4 А');
  assert.equal(text('temp'), '25 °C'); assert.equal(text('soc-note'), 'Разряд');
  battery = {connected: true, data: {...sample, soc: 0}}; await runTimer(1000);
  assert.equal(text('soc'), '0'); assert.match(text('batt-note'), /^0%/);
  stateOnline = true; speed = .5; await intervals.get(700)();
  assert.equal(text('soc'), '0'); assert.equal(text('speed'), '0.50 м/с');
  for (battery of [{connected: true, data: {...sample, soc: null}}, {connected: true, data: null},
                   {connected: false, data: sample, reason: 'Данные устарели'},
                   {connected: false, data: null}, 'error']) {
    await runTimer(1000);
    assert.equal(text('soc'), '—'); assert.equal(els['mon-soc-bar'].style.width, '0%');
    assert(!text('batt-note').includes('%'));
    if (!battery.connected) assert.equal(text('volts'), '— В');
    assert.equal(text('speed'), '0.50 м/с');
  }
  battery = {connected: true, data: sample}; await runTimer(1000);
  assert.equal(text('soc'), '72', 'reconnects without reloading the monitor');
  assert(requests.filter(r => r.url.startsWith('api/battery')).every(r =>
    r.url === 'api/battery?m=guest%2Btoken' && r.init.credentials === 'same-origin' && !r.init.method));
  console.log('guest BMS: OK');
})().catch(e => { console.error(e); process.exitCode = 1; });
