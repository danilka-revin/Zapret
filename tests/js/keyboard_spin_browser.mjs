/* Real browser -> (authenticated proxy) -> onboard -> 4 PTY firmware boards.
 * SC pulses are simulated; this is not a test of physical mechanics. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const [origin, kiosk, mode, trackText] = process.argv.slice(2);
const remote = mode === 'remote', track = Number(trackText);
const angle = Math.atan2(500, track) * 180 / Math.PI;
const angles = {FL: angle, FR: -angle, RL: -angle, RR: angle};
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 1200, height: 900}});
page.setDefaultTimeout(10000);
const actions = [], errors = [];
let stage = 'startup';
const endpoint = remote ? '/api/control/keyboard' : '/api/kiosk_keyboard';
page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if (request.url().includes(endpoint) && request.postData()) {
    try { actions.push(JSON.parse(request.postData())); } catch (_) {}
  }
});
const sleep = ms => page.waitForTimeout(ms);
async function wait(predicate, timeout = 10000) {
  const end = Date.now() + timeout;
  let data;
  while (Date.now() < end) {
    data = (await (await fetch(kiosk + '/api/kiosk_state', {signal: AbortSignal.timeout(4000)})).json()).data;
    if (predicate(data)) return data;
    await sleep(35);
  }
  throw new Error(stage + ' timeout: ' + JSON.stringify({status: await page.locator('#keyboard-status').textContent(),
    motors: data && data.motors, drive: data && data.drive, actions: actions.slice(-16)}));
}
const stopped = () => wait(data => data.motors.length === 4 && data.motors.every(m => m.online && m.drivePercent === 0));
const settled = () => wait(data => data.motors.length === 4 && data.motors.every(m => m.online &&
  m.drivePercent === 0 && !m.moving && m.scRpm <= 1 && m.scRecent === false));
const atAngles = wanted => wait(data => data.motors.every(m => m.online && !m.moving &&
  Math.abs(m.angle - wanted[m.id]) <= 1.0));
const zeroAngles = {FL: 0, FR: 0, RL: 0, RR: 0};
const pivot = direction => wait(data => data.motors.length === 4 && data.motors.every(m =>
  m.online && !m.moving && Math.abs(m.angle - angles[m.id]) <= 1.0 &&
  m.drivePercent === (['FL', 'RL'].includes(m.id) ? direction * 20 : -direction * 20)));
async function enable() {
  await page.locator('#keyboard-enable').click();
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'true');
  await page.locator('#keyboard-enable').focus();
}
let failure;
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
  await wait(data => data.motors.length === 4 && data.motors.every(m => m.online && m.homed && m.firmware >= 20));
  await enable();
  assert.match(await page.locator('#keyboard-spin-geometry').textContent(), new RegExp(angle.toFixed(2) + '°'));

  stage = 'release during initial steering';
  const initial = actions.length;
  await page.keyboard.down('ArrowLeft');
  await wait(data => data.motors.some(m => m.moving));
  await page.keyboard.up('ArrowLeft');
  await settled();
  await atAngles(zeroAngles);
  await sleep(250);
  assert.ok(!actions.slice(initial).some(a => a.action === 'live' && a.pattern === 'spin'),
    'released turn started after steering');

  stage = 'left hold';
  const leftStart = actions.length;
  await page.keyboard.down('ArrowLeft');
  const left = await pivot(-1);
  assert.deepEqual(left.drive.commandedModules, {FL: -20, FR: 20, RL: -20, RR: 20});
  // More than the 3-second SC-silence threshold: a healthy held turn must
  // continue, rather than acting like a fixed-angle/timed manoeuvre.
  await sleep(3500);
  await pivot(-1);
  const live = actions.slice(leftStart).filter(a => a.action === 'live');
  assert.ok(live.length >= 10 && live.every(a => a.pattern === 'spin' && a.percent === -20));
  for (const m of left.motors) assert.ok(Math.abs(m.angle - angles[m.id]) <= 1.0);
  assert.ok(left.motors.some(m => Math.abs(m.angle - angles[m.id]) > 0.15),
    'firmware simulator did not reproduce a potentiometer residual');
  await page.keyboard.up('ArrowLeft');
  await stopped();
  await settled();
  await atAngles(zeroAngles);

  stage = 'right hold and arrow conflict';
  await page.keyboard.down('ArrowRight');
  await pivot(1);
  await page.keyboard.down('ArrowLeft');
  await stopped();
  await page.keyboard.up('ArrowRight');
  const conflictEnd = actions.length;
  await sleep(800);
  await stopped();
  assert.ok(!actions.slice(conflictEnd).some(a => a.action === 'live'),
    'a still-held arrow resumed after conflict');
  await page.keyboard.up('ArrowLeft');
  await settled();
  await atAngles(zeroAngles);

  stage = 'delayed permit';
  let sawArm;
  const armSeen = new Promise(resolve => { sawArm = resolve; });
  let delayArm = true;
  const route = '**' + endpoint;
  await page.route(route, async r => {
    const body = r.request().postDataJSON();
    if (body.action === 'arm' && body.pattern === 'spin' && delayArm) {
      delayArm = false; sawArm(); await new Promise(resolve => setTimeout(resolve, 300));
    }
    await r.continue();
  });
  const delayed = actions.length;
  await page.keyboard.down('ArrowRight');
  await Promise.race([armSeen, sleep(8000).then(() => { throw new Error('spin ARM not sent'); })]);
  await page.keyboard.up('ArrowRight');
  await sleep(650);
  await page.unroute(route);
  await settled();
  await atAngles(zeroAngles);
  assert.ok(!actions.slice(delayed).some(a => a.action === 'live' && a.pattern === 'spin'),
    'late spin permit restarted rotation');

  stage = 'delayed first live packet';
  let sawLive;
  const liveSeen = new Promise(resolve => { sawLive = resolve; });
  let delayLive = true;
  await page.route(route, async r => {
    const body = r.request().postDataJSON();
    if (body.action === 'live' && body.pattern === 'spin' && delayLive) {
      delayLive = false; sawLive(); await new Promise(resolve => setTimeout(resolve, 300));
    }
    await r.continue();
  });
  const delayedResponse = page.waitForResponse(response => response.url().includes(endpoint) &&
    response.request().postDataJSON()?.action === 'live');
  await page.keyboard.down('ArrowLeft');
  await Promise.race([liveSeen, sleep(8000).then(() => { throw new Error('spin live not sent'); })]);
  await page.keyboard.up('ArrowLeft');
  const late = await delayedResponse;
  assert.equal(late.status(), 409);
  assert.equal((await late.json()).reason, 'stale_command');
  await page.unroute(route);
  await settled();
  await atAngles(zeroAngles);
  await sleep(400);
  await stopped();

  stage = 'focus loss';
  await page.keyboard.down('ArrowLeft');
  await pivot(-1);
  await page.evaluate(() => window.dispatchEvent(new Event('blur')));
  await stopped();
  assert.equal(await page.locator('#keyboard-enable').getAttribute('aria-pressed'), 'false');
  await sleep(800);
  await stopped();
  await page.keyboard.up('ArrowLeft');
  await settled();

  stage = 'fast telemetry loss';
  await enable();
  await page.keyboard.down('ArrowRight');
  await pivot(1);
  const fast = remote ? '**/api/control/wheel_state' : '**/api/kiosk_wheel_state';
  await page.route(fast, r => r.abort());
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'false', null, {timeout: 5000});
  await page.unroute(fast);
  await stopped();
  await sleep(800);
  await stopped();
  await page.keyboard.up('ArrowRight');
  await settled();

  stage = 'keyboard command loss and firmware watchdog';
  await enable();
  await page.keyboard.down('ArrowLeft');
  await pivot(-1);
  // Drop *all* keyboard commands, including END, so only the board watchdog
  // can stop the four wheels. Restoring the network must not resume them.
  await page.route(route, r => r.abort());
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'false', null, {timeout: 5000});
  await stopped();
  await page.unroute(route);
  await sleep(900);
  await stopped();
  await page.keyboard.up('ArrowLeft');
  await settled();

  stage = 'emergency stop';
  await enable();
  await page.keyboard.down('ArrowLeft');
  await pivot(-1);
  const response = await page.evaluate(async () => {
    const r = await fetch('api/estop', {method: 'POST', headers: {'Content-Type': 'application/json',
      'X-Robot-Control': 'drive'}, body: '{}'});
    return {status: r.status, body: await r.json()};
  });
  assert.equal(response.status, 200, JSON.stringify(response));
  await stopped();
  await sleep(800);
  await stopped();
  await page.keyboard.up('ArrowLeft');
  assert.deepEqual(errors, []);
  console.log('keyboard pivot + four simulated USB boards: OK (' + mode + ', track ' + track + ')');
} catch (error) { failure = error; }
finally { await Promise.race([browser.close().catch(() => {}), new Promise(resolve => setTimeout(resolve, 1500))]); }
if (failure) { console.error(stage, failure); process.exit(1); }
process.exit(0);
