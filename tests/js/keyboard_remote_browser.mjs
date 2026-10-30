/* Authenticated browser -> remote proxy -> onboard -> four simulated wheels. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const [origin, kiosk] = process.argv.slice(2);
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 1200, height: 900}});
const errors = [], actions = [];
let pending = 0, maxPending = 0, fastCount = 0;
const isFast = req => req.url().includes('/api/control/wheel_state');
page.on('pageerror', e => errors.push(String(e)));
page.on('request', req => {
  if (isFast(req)) { ++fastCount; maxPending = Math.max(maxPending, ++pending); }
  if (req.url().includes('/api/control/keyboard') && req.postData()) {
    try { actions.push(JSON.parse(req.postData())); } catch (_) {}
  }
});
page.on('requestfinished', req => { if (isFast(req)) --pending; });
page.on('requestfailed', req => { if (isFast(req)) --pending; });
async function wait(predicate, timeout = 10000) {
  const deadline = Date.now() + timeout;
  while (Date.now() < deadline) {
    const data = (await (await fetch(kiosk + '/api/kiosk_state')).json()).data;
    if (predicate(data)) return data;
    await page.waitForTimeout(50);
  }
  throw new Error('remote WASD timeout: ' + JSON.stringify({actions: actions.slice(-12),
    status: await page.locator('#keyboard-status').textContent()}));
}
const moving = () => wait(data => data.motors.every(m => m.online && m.drivePercent === 20));
const stopped = () => wait(data => data.motors.every(m => m.online && m.drivePercent === 0));
try {
  await page.goto(origin + '/login');
  await page.locator('input[name="username"]').fill('operator');
  await page.locator('input[name="password"]').fill('correct-horse-battery');
  await page.locator('form button[type="submit"]').click();
  await page.goto(origin + '/controls');
  await wait(data => data.motors.every(m => m.online && m.homed && m.firmware >= 20));
  await page.locator('#keyboard-enable').click();
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'true');
  await page.locator('#keyboard-mode').selectOption('normal');
  await page.locator('#keyboard-enable').focus();
  await page.keyboard.down('ArrowUp');
  await moving();
  const beforeTurn = actions.length;
  await page.keyboard.down('d');
  await wait(data => data.motors.every(m => m.drivePercent === 20 && !m.moving &&
    Math.abs(m.angle - (m.id.startsWith('F') ? 25 : -25)) < 1.5));
  await page.keyboard.up('d');
  await wait(data => data.motors.every(m => m.drivePercent === 20 && !m.moving && Math.abs(m.angle) < 1.5));
  assert.ok(!actions.slice(beforeTurn).some(a => a.action === 'stop'), 'remote turn braked unexpectedly');
  await page.waitForTimeout(400);
  assert.ok(fastCount >= 3, 'remote fast wheel-state proxy was not used');
  assert.equal(maxPending, 1, 'wheel polls must never overlap');
  assert.ok(actions.some(a => a.action === 'arm') && actions.some(a => a.action === 'live'));

  // Loss of only the fast route is enough to stop; no delayed reply may resume.
  await page.route('**/api/control/wheel_state', route => route.abort());
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'false', null, {timeout: 4000});
  await page.unroute('**/api/control/wheel_state');
  await stopped();
  await page.waitForTimeout(900);
  await stopped();
  await page.keyboard.up('ArrowUp');
  assert.deepEqual(errors, []);
  console.log('remote WASD + fast telemetry + four simulated boards: OK');
} finally { await browser.close(); }
