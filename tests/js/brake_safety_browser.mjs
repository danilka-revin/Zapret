/* Real browser -> kiosk (or authenticated proxy) -> four simulated USB boards. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const [origin, kiosk, mode] = process.argv.slice(2);
const remote = mode === 'remote';
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 1200, height: 900}});
const errors = [], releaseRequests = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if ((request.url().includes('/api/kiosk_neutral') || request.url().includes('/api/control/neutral')) && request.postData()) {
    try { releaseRequests.push(JSON.parse(request.postData())); } catch (_) {}
  }
});
const wait = async (predicate, timeout = 7000) => {
  const end = Date.now() + timeout;
  let data;
  while (Date.now() < end) {
    data = (await (await fetch(kiosk + '/api/kiosk_state')).json()).data;
    if (predicate(data)) return data;
    await page.waitForTimeout(40);
  }
  throw new Error('wheel state timeout: ' + JSON.stringify({neutral: data && data.neutral, motors: data && data.motors}));
};
const post = (path, body, marker = 'drive') => page.evaluate(async ([path, body, marker]) => {
  // The shared UI maps kiosk paths onto the authenticated proxy when remote.
  const response = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json',
    'X-Robot-Control': marker, 'X-Robot-Command-Deadline': String(Date.now() / 1000 + 0.5)},
    body: JSON.stringify(body)});
  return {status: response.status, body: await response.json()};
}, [path, body, marker]);

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
  await wait(data => data.neutral.phase === 'idle' && data.motors.every(m => m.online && m.firmware === 26));
  assert.equal(await page.locator('#neutral-panel, #neutral-toggle').count(), 0);
  assert.equal(await page.getByRole('button', {name: 'Снять тормоз'}).count(), 0);
  assert(await page.locator('#brake-recovery').isHidden());
  await page.waitForTimeout(300);
  assert.deepEqual(releaseRequests, [], 'page must not issue a neutral request');
  for (const body of [{action: 'begin'}, {action: 'heartbeat', session: 'stale', sequence: 999}]) {
    const rejected = await post('/api/kiosk_neutral', body);
    assert.equal(rejected.status, 410);
    assert.equal(rejected.body.reason, 'neutral_disabled');
  }
  const stationary = await wait(data => data.motors.every(m => m.neutralId === 0 && m.neutralLocked === false));
  assert.equal(stationary.neutral.locked, false);

  // Normal Servo.write traction and STOP still work through both surfaces.
  for (const value of [2, 150]) {
    const drive = await post('/api/kiosk_drive', {value, direction: 'forward'});
    assert.equal(drive.status, 200, JSON.stringify(drive));
    await wait(data => data.motors.every(m => m.driveServoValue === value), 3000);
    const stop = await post('/api/kiosk_drive', {value: 0});
    assert.equal(stop.status, 200, JSON.stringify(stop));
    await wait(data => data.motors.every(m => m.drivePercent === 0 && m.driveServoValue === 0));
  }
  const estop = await post('/api/estop', {});
  assert.equal(estop.status, 200);
  const blocked = await post('/api/kiosk_drive', {value: 40});
  assert.equal(blocked.status, 409);
  assert.equal(blocked.body.reason, 'estop');
  assert.equal((await post('/api/kiosk_drive', {value: 0})).status, 200);
  assert.deepEqual(errors, []);
  console.log('brake safety browser + four simulated USB boards: OK (' + mode + ')');
} finally { await browser.close(); }
