/* Real browser -> kiosk (or authenticated web proxy) -> four PTY boards. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const [origin, kiosk, mode] = process.argv.slice(2);
const remote = mode === 'remote';
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 1200, height: 900}});
const errors = [], heartbeats = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if ((request.url().includes('/api/kiosk_neutral') || request.url().includes('/api/control/neutral')) && request.postData()) {
    try { heartbeats.push(JSON.parse(request.postData())); } catch (_) {}
  }
});
const wait = async (predicate, timeout = 7000) => {
  const end = Date.now() + timeout;
  let data;
  while (Date.now() < end) {
    data = (await (await fetch(kiosk + '/api/kiosk_state')).json()).data;
    if (predicate(data)) return data;
    await page.waitForTimeout(80);
  }
  throw new Error('neutral timeout: ' + JSON.stringify({neutral: data && data.neutral,
    motors: data && data.motors, ui: await page.locator('#neutral-status').textContent(),
    beats: heartbeats.slice(-8)}));
};
const request = async (path, body, marker = 'drive') => {
  const response = await fetch(kiosk + path, {method: 'POST',
    headers: {'Content-Type': 'application/json', 'X-Robot-Control': marker,
      'X-Robot-Command-Deadline': String(Date.now() / 1000 + 0.5)}, body: JSON.stringify(body)});
  return {status: response.status, body: await response.json()};
};
const active = () => wait(data => data.neutral.active && data.motors.every(m => m.neutralId > 0));
const braked = () => wait(data => data.neutral.phase === 'idle' && data.motors.every(m =>
  m.neutralId === 0 && m.neutralLocked === false && m.drivePercent === 0));

try {
  if (remote) {
    await page.goto(origin + '/login');
    await page.locator('input[name="username"]').fill('operator');
    await page.locator('input[name="password"]').fill('correct-horse-battery');
    await page.locator('form button[type="submit"]').click();
    await page.goto(origin + '/controls');
  } else {
    await page.goto(origin + '/main.html');
    await page.waitForFunction(() => typeof window._showSteeringPage === 'function');
    await page.evaluate(() => window._showSteeringPage());
  }
  await wait(data => data.neutral.phase === 'idle' && data.motors.every(m => m.online && m.firmware >= 20));
  await page.locator('#neutral-toggle').click();
  const started = await active();
  const token = heartbeats.find(item => item.action === 'heartbeat')?.session;
  assert.ok(token);
  assert.equal(new Set(started.motors.map(m => m.neutralId)).size, 1);
  await page.waitForTimeout(1200);
  await active();
  assert.match(await page.locator('#neutral-status').textContent(), /НЕЙТРАЛЬ/);
  assert.equal(await page.locator('#keyboard-enable').isDisabled(), true);
  for (const [path, body, marker] of [
    ['/api/kiosk_drive', {value: 40}, 'drive'],
    ['/api/kiosk_steer', {module: 'FL', angle: 20}, 'steering'],
    ['/api/kiosk_keyboard', {action: 'begin'}, 'drive'],
    ['/api/kiosk_maneuver', {action: 'start', name: 'forward'}, 'steering']
  ]) {
    const result = await request(path, body, marker);
    assert.equal(result.status, 409, JSON.stringify(result));
    assert.equal(result.body.reason, 'neutral_locked');
  }
  await page.locator('#neutral-toggle').click();
  await braked();
  const stale = await request('/api/kiosk_neutral', {action: 'heartbeat', session: token, sequence: 99999});
  assert.equal(stale.status, 409);
  await page.waitForTimeout(750);
  await braked();

  // An ordinary STOP revokes neutral for every wheel, including in-flight pings.
  await page.locator('#neutral-toggle').click();
  await active();
  const stoppedToken = heartbeats.filter(item => item.action === 'heartbeat').at(-1).session;
  assert.equal((await request('/api/kiosk_drive', {percent: 0})).status, 200);
  await wait(data => data.neutral.phase === 'fault' && data.motors.every(m => m.neutralId === 0));
  assert.equal((await request('/api/kiosk_neutral', {action: 'heartbeat', session: stoppedToken, sequence: 999999})).status, 409);
  await page.locator('#neutral-toggle').click();
  await braked();

  // Lost browser -> server lease expires, boards brake and old packets stay dead.
  await page.locator('#neutral-toggle').click();
  await active();
  const lostToken = heartbeats.filter(item => item.action === 'heartbeat').at(-1).session;
  const pattern = remote ? '**/api/control/neutral' : '**/api/kiosk_neutral';
  await page.route(pattern, route => {
    const body = route.request().postDataJSON();
    return body.action === 'heartbeat' ? route.abort() : route.continue();
  });
  const failed = await wait(data => data.neutral.phase === 'fault' && data.motors.every(m => m.neutralId === 0), 5000);
  assert.ok(failed.motors.every(m => m.neutralLocked === true), 'firmware must remain locked after a lost lease');
  const rejected = await request('/api/kiosk_drive', {value: 40});
  assert.equal(rejected.body.reason, 'neutral_locked');
  const late = await request('/api/kiosk_neutral', {action: 'heartbeat', session: lostToken, sequence: 999999});
  assert.equal(late.status, 409);
  await page.unroute(pattern);
  await page.locator('#neutral-toggle').click();
  await braked();
  assert.deepEqual(errors, []);
  console.log('neutral browser + four simulated USB boards: OK (' + mode + ')');
} finally { await browser.close(); }
