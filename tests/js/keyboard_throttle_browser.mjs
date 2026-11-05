/* Hold-to-run arrow regression: rapid taps, reordered STOP and UI focus. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const origin = process.argv[2];
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 1200, height: 900}});
page.setDefaultTimeout(8000);
const actions = [], errors = [];
let interceptStop = false, stops = 0, releaseIntercepted;
let interceptLive = false, delayed = false, sawHeartbeat;
// Install the route before requests begin so the very first STOP is always
// intercepted even when Chromium has an active telemetry stream.
await page.route('**/api/kiosk_keyboard', async route => {
  const body = route.request().postDataJSON();
  if (interceptStop && body.action === 'stop' && ++stops === 1) {
    releaseIntercepted();
    await new Promise(resolve => setTimeout(resolve, 450));
  } else if (interceptLive && body.action === 'live' && !delayed) {
    delayed = true;
    sawHeartbeat();
    await new Promise(resolve => setTimeout(resolve, 250));
  }
  await route.continue().catch(() => {});
});
async function expectRequest(promise, label) {
  let timer;
  try {
    return await Promise.race([promise, new Promise((_, reject) => {
      timer = setTimeout(() => reject(new Error('waiting for ' + label + ': ' + JSON.stringify(actions.slice(-15)))), 7000);
    })]);
  } finally { clearTimeout(timer); }
}

page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if (request.url().includes('/api/kiosk_keyboard') && request.postData()) {
    try { actions.push(JSON.parse(request.postData())); } catch (_) {}
  }
});
async function poll(predicate, timeout = 7000) {
  const deadline = Date.now() + timeout;
  let data;
  while (Date.now() < deadline) {
    data = (await (await fetch(origin + '/api/kiosk_state', {signal: AbortSignal.timeout(4000)})).json()).data;
    if (predicate(data)) return data;
    await page.waitForTimeout(30);
  }
  throw new Error('throttle timeout: ' + JSON.stringify({
    status: await page.locator('#keyboard-status').textContent(),
    actions: actions.slice(-20), motors: data && data.motors,
    focused: await page.evaluate(() => document.activeElement && document.activeElement.id)}));
}
const moving = () => poll(data => data.motors.length === 4 && data.motors.every(m => m.online && m.drivePercent < 0));
const stopped = () => poll(data => data.motors.length === 4 && data.motors.every(m => m.online && m.drivePercent === 0));
let failure;
try {
  await page.goto(origin + '/main.html', {waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => typeof window._showSteeringPage === 'function');
  await page.evaluate(() => window._showSteeringPage());
  await poll(data => data.motors.length === 4 && data.motors.every(m => m.online && m.homed));
  await page.locator('#keyboard-enable').click();
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'true');
  await page.keyboard.down('ArrowUp');
  await moving();

  // The first STOP is delayed, while new press/release events arrive. A later
  // stale STOP must not silently brake a subsequently started live session.
  const firstStop = new Promise(resolve => { releaseIntercepted = resolve; });
  interceptStop = true;
  await page.keyboard.up('ArrowUp');
  await expectRequest(firstStop, 'first STOP');
  await page.keyboard.down('ArrowUp');
  await page.waitForTimeout(40);
  await page.keyboard.up('ArrowUp');
  await page.waitForTimeout(40);
  await page.keyboard.down('ArrowUp');
  await moving();
  await page.waitForTimeout(900); // delayed STOP must have completed
  assert.equal(await page.locator('#keyboard-enable').getAttribute('aria-pressed'), 'true',
    'WASD unexpectedly disabled after reordered stops: ' + await page.locator('#keyboard-status').textContent());
  await moving();
  assert.equal(stops, 1, 'a single in-flight STOP should coalesce rapid re-planning');
  interceptStop = false;
  await page.keyboard.up('ArrowUp');
  await stopped();
  await poll(data => data.motors.every(m => !m.scRecent && m.scRpm <= 1));

  // A real pointer selection of the slider must not leave its focus eating
  // subsequent drive arrows. Text fields elsewhere must still receive arrows.
  const slider = page.locator('#keyboard-speed');
  const box = await slider.boundingBox();
  await slider.click({position: {x: Math.floor(box.width * .2), y: Math.floor(box.height / 2)}});
  const speed = Number(await slider.inputValue());
  const pressAt = Date.now();
  await page.keyboard.down('ArrowUp');
  await moving();
  const startLatency = Date.now() - pressAt;
  assert.ok(startLatency < 1500, 'stopped simulated wheels reacted too slowly: ' + startLatency + 'ms');
  assert.equal(Number(await slider.inputValue()), speed, 'drive arrow changed the speed slider');

  // Input changes while a heartbeat is in flight must converge to the most
  // recent speed immediately after that response, without braking/re-arming.
  const delayedHeartbeat = new Promise(resolve => { sawHeartbeat = resolve; });
  interceptLive = true;
  await expectRequest(delayedHeartbeat, 'speed-update heartbeat');
  const changes = actions.length;
  await slider.click({position: {x: Math.floor(box.width * .4), y: Math.floor(box.height / 2)}});
  await slider.click({position: {x: Math.floor(box.width * .7), y: Math.floor(box.height / 2)}});
  const finalSpeed = Number(await slider.inputValue());
  await poll(data => data.motors.every(m => m.online && m.drivePercent === -finalSpeed));
  assert.ok(actions.slice(changes).some(a => a.action === 'live' && a.percent === -finalSpeed));
  assert.ok(!actions.slice(changes).some(a => a.action === 'stop' || a.action === 'arm'), 'speed adjustment restarted traction');
  interceptLive = false;
  await page.keyboard.up('ArrowUp');
  await stopped();
  assert.deepEqual(errors, []);
  console.log('WASD arrow throttle responsiveness: OK (simulated start ' + startLatency + 'ms)');
} catch (error) {
  failure = error;
} finally {
  await Promise.race([browser.close().catch(() => {}), new Promise(resolve => setTimeout(resolve, 1500))]);
}
if (failure) { console.error(failure); process.exit(1); }
process.exit(0);
