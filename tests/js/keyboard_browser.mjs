/* Real browser -> kiosk -> four PTY firmware boards. Physics is simulated. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const origin = process.argv[2];
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 1200, height: 900}});
const errors = [], actions = [];
let fastPolls = 0;
page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if (request.url().includes('/api/kiosk_wheel_state')) ++fastPolls;
  if (request.url().includes('/api/kiosk_keyboard') && request.postData()) {
    try { actions.push(JSON.parse(request.postData())); } catch (_) {}
  }
});
const poll = async (predicate, timeout = 12000) => {
  const end = Date.now() + timeout;
  let data;
  while (Date.now() < end) {
    data = (await (await fetch(origin + '/api/kiosk_state')).json()).data;
    if (predicate(data)) return data;
    await page.waitForTimeout(35);
  }
  throw new Error('telemetry timeout: ' + JSON.stringify({motors: data && data.motors,
    status: await page.locator('#keyboard-status').textContent(), actions: actions.slice(-12)}));
};
const all = angle => ({FL: angle, FR: angle, RL: angle, RR: angle});
const waitWheels = (angles, sign) => poll(data => Object.entries(angles).every(([id, angle]) => {
  const m = data.motors.find(item => item.id === id);
  return m && m.online && !m.moving && Math.abs(m.angle - angle) <= 1.5 &&
    (sign === 0 ? m.drivePercent === 0 : Math.sign(m.drivePercent) === sign);
}));
const stopped = () => poll(data => data.motors.length === 4 &&
  data.motors.every(m => m.online && m.drivePercent === 0), 6000);
const enable = async () => {
  await page.locator('#keyboard-enable').click();
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'true', null, {timeout: 10000});
  await page.locator('#keyboard-enable').focus();
};
try {
  await page.goto(origin + '/main.html', {waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => typeof window._showSteeringPage === 'function');
  await page.evaluate(() => window._showSteeringPage());
  await poll(data => data.motors.length === 4 && data.motors.every(m => m.online && m.homed && m.firmware >= 20));
  await enable();
  assert.equal(await page.locator('#keyboard-speed').inputValue(), '20');

  // WASD without an arrow only positions steering. Arrow is hold-to-run.
  await page.keyboard.down('w');
  await waitWheels(all(0), 0);
  await page.keyboard.down('ArrowUp');
  await waitWheels(all(0), 1);
  await page.keyboard.up('ArrowUp');
  await stopped();
  await page.keyboard.up('w');
  await waitWheels(all(0), 0);

  // Four-wheel normal steering while driving: D turns fronts right, rears
  // left, and releasing D centres all four without a STOP or loss of thrust.
  await page.locator('#keyboard-mode').selectOption('normal');
  await page.locator('#keyboard-enable').focus();
  await page.keyboard.down('ArrowUp');
  await waitWheels(all(0), 1);
  const start = actions.length;
  await page.keyboard.down('d');
  await poll(data => data.motors.some(m => m.moving) && data.motors.every(m => m.drivePercent === 20));
  await waitWheels({FL: 25, FR: 25, RL: -25, RR: -25}, 1);
  await page.keyboard.up('d');
  await poll(data => data.motors.some(m => m.moving) && data.motors.every(m => m.drivePercent === 20));
  await waitWheels(all(0), 1);
  const turns = actions.slice(start);
  assert.ok(turns.some(a => a.action === 'live' && a.angles.FL === 25 && a.angles.RL === -25), JSON.stringify(turns));
  assert.ok(turns.some(a => a.action === 'live' && a.angles.FL === 0 && a.angles.RL === 0), JSON.stringify(turns));
  assert.ok(!turns.some(a => a.action === 'stop'), 'turn or self-centre stopped traction');
  // The key can also be released before the stepper reaches its target.
  const quickStart = actions.length;
  await page.keyboard.down('d');
  await poll(data => data.motors.some(m => m.moving) && data.motors.every(m => m.drivePercent === 20));
  await page.keyboard.up('d');
  await waitWheels(all(0), 1);
  assert.ok(!actions.slice(quickStart).some(a => a.action === 'stop'), 'quick release braked');
  await page.keyboard.up('ArrowUp');
  await stopped();

  // While stopped, releasing D must also cancel an unfinished turn and centre.
  await page.keyboard.down('d');
  await page.waitForTimeout(120);
  await page.keyboard.up('d');
  await waitWheels(all(0), 0);

  // Crab left and right use the SAME +90° axis and opposite motor directions.
  await page.locator('#keyboard-mode').selectOption('crab');
  await page.locator('#keyboard-enable').focus();
  await page.keyboard.down('d');
  await waitWheels(all(90), 0);
  await page.keyboard.down('ArrowUp');
  await waitWheels(all(90), 1);
  // Reversing the crab side while the arrow is held must brake first, then
  // reuse +90° with negative traction; no -90° command is ever sent.
  const crabStart = actions.length;
  await page.keyboard.up('d');
  await page.keyboard.down('a');
  await waitWheels(all(90), -1);
  const crabActions = actions.slice(crabStart);
  const brakeAt = crabActions.findIndex(a => a.action === 'stop');
  const reverseAt = crabActions.findIndex(a => a.action === 'live' && a.percent < 0 && a.angles.FL === 90);
  assert.ok(brakeAt >= 0 && reverseAt > brakeAt, JSON.stringify(crabActions));
  assert.ok(!crabActions.some(a => a.angles && Object.values(a.angles).includes(-90)), 'crab rotated to -90°');
  await page.keyboard.up('ArrowUp');
  await page.keyboard.up('a');
  await stopped();
  await waitWheels(all(0), 0);

  // Releasing during a turn must cancel old asynchronous plans/heartbeats.
  await page.keyboard.down('d');
  await page.waitForTimeout(80);
  await page.keyboard.up('d');
  await page.waitForTimeout(1800);
  await waitWheels(all(0), 0);
  await page.keyboard.down('ArrowUp');
  await waitWheels(all(0), 1);
  await page.evaluate(() => window.dispatchEvent(new Event('blur')));
  await stopped();
  assert.equal(await page.locator('#keyboard-enable').getAttribute('aria-pressed'), 'false');
  await page.waitForTimeout(800);
  await stopped();
  await page.keyboard.up('ArrowUp');

  // Loss of only the fast telemetry, or of the ordinary state, stops and
  // revokes the lease even while the arrow remains held.
  for (const endpoint of ['kiosk_wheel_state', 'kiosk_state']) {
    await enable();
    await page.keyboard.down('ArrowUp');
    await waitWheels(all(0), 1);
    assert.ok(fastPolls > 0);
    await page.route('**/api/' + endpoint, route => route.abort());
    await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'false', null, {timeout: 4000});
    await page.unroute('**/api/' + endpoint);
    await stopped();
    await page.waitForTimeout(800);
    await stopped();
    await page.keyboard.up('ArrowUp');
  }
  assert.deepEqual(errors, []);
  console.log('WASD browser + kiosk + four simulated USB boards: OK');
} finally { await browser.close(); }
