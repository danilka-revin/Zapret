/* Keyboard focus, delayed requests and hold-to-run safety through real fw23 PTYs. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const origin = process.argv[2];
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 1200, height: 900}});
page.setDefaultTimeout(8000);
const actions = [], responses = [], errors = [];
let routeMode = 'normal', resolveIntercept = null, firstDelayed = false;
let beginCount = 0, endCount = 0;
// Avoid modifying Chromium's routing while keyboard and telemetry traffic is
// active: that can let the target BEGIN/ARM/STOP bypass interception entirely.
await page.route('**/api/kiosk_keyboard', async route => {
  const action = route.request().postDataJSON().action;
  if (routeMode === 'begin' && action === 'begin') {
    resolveIntercept();
    await new Promise(resolve => setTimeout(resolve, 300));
  } else if (routeMode === 'lifecycle') {
    if (action === 'begin' && ++beginCount === 1) {
      resolveIntercept();
      await new Promise(resolve => setTimeout(resolve, 300));
    }
    if (action === 'end' && ++endCount === 1) await new Promise(resolve => setTimeout(resolve, 250));
  } else if (routeMode === action && !firstDelayed) {
    firstDelayed = true;
    resolveIntercept();
    await new Promise(resolve => setTimeout(resolve, 300));
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
    try { actions.push({...JSON.parse(request.postData()), at: Date.now()}); } catch (_) {}
  }
});
page.on('response', response => {
  const request = response.request();
  if (request.url().includes('/api/kiosk_keyboard') && request.postData()) {
    try { responses.push({...JSON.parse(request.postData()), at: Date.now(), status: response.status()}); } catch (_) {}
  }
});
async function poll(predicate, timeout = 8000) {
  const deadline = Date.now() + timeout;
  let data;
  while (Date.now() < deadline) {
    data = (await (await fetch(origin + '/api/kiosk_state', {signal: AbortSignal.timeout(4000)})).json()).data;
    if (predicate(data)) return data;
    await page.waitForTimeout(30);
  }
  throw new Error('keyboard safety timeout: ' + JSON.stringify({
    status: await page.locator('#keyboard-status').textContent(),
    actions: actions.slice(-20), motors: data && data.motors,
    focused: await page.evaluate(() => document.activeElement && document.activeElement.id)}));
}
const sign = value => poll(data => data.motors.length === 4 &&
  data.motors.every(m => m.online && (value ? Math.sign(m.drivePercent) === value : m.drivePercent === 0)));
const settled = () => poll(data => data.motors.length === 4 && data.motors.every(m =>
  m.online && m.drivePercent === 0 && !m.moving && !m.scRecent && m.scRpm <= 1));
const enabled = () => page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'true');
const disabled = () => page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'false');
const focusButton = () => page.locator('#keyboard-enable').focus();
const assertNoLive = (since, message) => assert.ok(!actions.slice(since).some(a => a.action === 'live'),
  message + ': ' + JSON.stringify(actions.slice(since)));
let failure, stage = 'setup';
try {
  await page.goto(origin + '/main.html', {waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => typeof window._showSteeringPage === 'function');
  await page.evaluate(() => window._showSteeringPage());
  await poll(data => data.motors.length === 4 && data.motors.every(m => m.online && m.homed));

  stage = 'quick BEGIN';
  // A key tapped while BEGIN is delayed must NOT start after it is released.
  const beginIntercepted = new Promise(resolve => { resolveIntercept = resolve; });
  routeMode = 'begin';
  const quick = actions.length;
  await page.locator('#keyboard-enable').click();
  await expectRequest(beginIntercepted, 'delayed BEGIN');
  await page.keyboard.down('ArrowUp');
  await page.keyboard.up('ArrowUp');
  await enabled();
  routeMode = 'normal';
  await page.waitForTimeout(400);
  await sign(0);
  assertNoLive(quick, 'released key started after enabling');

  // A held key DOES start as soon as the safe BEGIN/ACK/permit chain finishes.
  await page.keyboard.down('ArrowUp');
  await sign(-1);
  await page.keyboard.up('ArrowUp');
  await settled();

  stage = 'mode focus';
  // Mouse-selected mode must return focus to the control surface. In contrast,
  // keyboard selection (even after a dismissed pointer menu) must keep focus.
  const mode = page.locator('#keyboard-mode');
  await mode.click();
  await mode.selectOption('normal');
  assert.notEqual(await page.evaluate(() => document.activeElement.id), 'keyboard-mode');
  const afterMouseMode = actions.length;
  await page.keyboard.down('ArrowUp');
  await sign(-1);
  assert.ok(actions.slice(afterMouseMode).some(a => a.action === 'live'));
  await page.keyboard.up('ArrowUp');
  await settled();
  await mode.focus();
  await mode.dispatchEvent('pointerdown', {pointerId: 9, pointerType: 'mouse'});
  const keyboardMode = actions.length;
  await page.keyboard.press('ArrowUp');
  await page.waitForTimeout(150);
  assert.equal(await page.evaluate(() => document.activeElement.id), 'keyboard-mode', 'keyboard select lost focus');
  await page.keyboard.press('ArrowDown');
  await page.waitForTimeout(250);
  assertNoLive(keyboardMode, 'editing select generated traction');
  await sign(0);
  await page.keyboard.press('Escape');
  assert.notEqual(await page.evaluate(() => document.activeElement.id), 'keyboard-mode',
    JSON.stringify({status: await page.locator('#keyboard-status').textContent(), active: await page.locator('#keyboard-enable').getAttribute('aria-pressed'), actions: actions.slice(-8)}));

  stage = 'slider focus';
  // Slider arrows edit the slider when keyboard-focused; Esc restores driving.
  const slider = page.locator('#keyboard-speed');
  await slider.focus();
  const sliderAt = Number(await slider.inputValue());
  const keyboardSlider = actions.length;
  await page.keyboard.press('ArrowUp');
  assert.equal(Number(await slider.inputValue()), sliderAt + 1);
  await page.waitForTimeout(200);
  assertNoLive(keyboardSlider, 'editing slider generated traction');
  await page.keyboard.press('Escape');
  await page.keyboard.down('ArrowUp');
  await sign(-1);
  await page.keyboard.up('ArrowUp');
  await settled();

  stage = 'slider pointer';
  // End a real pointer drag outside the range: the slider must not retain focus.
  const box = await slider.boundingBox();
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width + 80, box.y + box.height / 2 + 50);
  await page.mouse.up();
  assert.notEqual(await page.evaluate(() => document.activeElement.id), 'keyboard-speed');

  stage = 'external input';
  // External editable fields must never dispatch traction or be blurred by Esc.
  await page.evaluate(() => {
    const input = document.createElement('input');
    input.id = 'keyboard-test-external'; input.type = 'text'; input.value = 'hello';
    document.body.appendChild(input); input.focus();
  });
  const external = actions.length;
  await page.keyboard.press('ArrowUp');
  await page.keyboard.press('w');
  await page.waitForTimeout(200);
  assertNoLive(external, 'editing another field generated traction');
  assert.equal(await page.evaluate(() => document.activeElement.id), 'keyboard-test-external');
  await sign(0);
  await page.evaluate(() => document.getElementById('keyboard-test-external').remove());
  await focusButton();

  stage = 'late ARM and live';
  // An ARM delivered after release must not start, even when the reply arrives.
  for (const delayedAction of ['arm', 'live']) {
    const delayed = new Promise(resolve => { resolveIntercept = resolve; });
    firstDelayed = false;
    routeMode = delayedAction;
    const at = actions.length;
    await page.keyboard.down('ArrowUp');
    await expectRequest(delayed, delayedAction);
    await page.keyboard.up('ArrowUp');
    await page.waitForTimeout(650);
    routeMode = 'normal';
    await settled();
    assert.equal(await page.locator('#keyboard-enable').getAttribute('aria-pressed'), 'true',
      'late ' + delayedAction + ' disabled WASD: ' + await page.locator('#keyboard-status').textContent());
    const segment = actions.slice(at);
    const stopIndex = segment.findIndex(a => a.action === 'stop');
    assert.ok(stopIndex >= 0, 'release did not STOP: ' + JSON.stringify(segment));
    if (delayedAction === 'arm') assertNoLive(at, 'late arm caused traction after release');
    assert.ok(!segment.slice(stopIndex + 1).some(a => a.action === 'live'),
      'traction request sent after release: ' + JSON.stringify(segment));
  }

  stage = 'reverse';
  // Conflicting arrows brake; releasing Up leaves Down held and reverses only
  // after the tagged STOP, four ACKs, stopped SC and a new permit.
  await page.keyboard.down('ArrowUp');
  await sign(-1);
  const reverse = actions.length;
  await page.keyboard.down('ArrowDown');
  await sign(0);
  await page.keyboard.up('ArrowUp');
  await sign(1);
  const reversal = actions.slice(reverse);
  const stop = reversal.findIndex(a => a.action === 'stop');
  const arm = reversal.findIndex(a => a.action === 'arm' && a.direction === 1);
  const live = reversal.findIndex(a => a.action === 'live' && a.percent > 0);
  assert.ok(stop >= 0 && arm > stop && live > arm, JSON.stringify(reversal));
  await page.keyboard.up('ArrowDown');
  await settled();

  stage = 'pending steering';
  // Holding gas while the selected steering is being prepared should reuse
  // the pending plan rather than STOP and restart the same turn.
  await mode.selectOption('crab');
  await focusButton();
  const steering = actions.length;
  await page.keyboard.down('d');
  await poll(data => data.motors.some(m => m.moving));
  await page.keyboard.down('ArrowUp');
  await poll(data => data.motors.every(m => m.online && m.drivePercent < 0 && !m.moving && Math.abs(m.angle - 90) <= 1.5));
  const prepared = actions.slice(steering);
  const steerIndex = prepared.findIndex(a => a.action === 'steer_all');
  const driveIndex = prepared.findIndex(a => a.action === 'live');
  assert.ok(steerIndex >= 0 && driveIndex > steerIndex, JSON.stringify(prepared));
  assert.ok(!prepared.slice(steerIndex + 1, driveIndex).some(a => a.action === 'stop'),
    'gas restarted the same steering plan: ' + JSON.stringify(prepared));
  await page.keyboard.up('ArrowUp');
  await page.keyboard.up('d');
  await settled();
  // STOP can complete before the newly requested return-to-centre begins.
  // Wait for its target as well, or the next D simply selects the existing 90°.
  await poll(data => data.motors.every(m => !m.moving && Math.abs(m.angle) <= 1.5));

  stage = 'cancel steering';
  // If gas is released during steering preparation, the reused plan is
  // cancelled and may never arm or start once the steering eventually settles.
  const cancelSteering = actions.length;
  await page.keyboard.down('d');
  await poll(data => data.motors.some(m => m.moving));
  await page.keyboard.down('ArrowUp');
  await page.waitForTimeout(30);
  await page.keyboard.up('ArrowUp');
  await page.keyboard.up('d');
  await settled();
  await page.waitForTimeout(400);
  assertNoLive(cancelSteering, 'released gas started after steering preparation');
  await sign(0);

  stage = 'brake expiry';
  // Firmware accepts a brake ID for at most 6 seconds. After the stricter
  // browser cache window expires, a fresh STOP must precede the next arm.
  await page.waitForTimeout(4200);
  const oldBrake = actions.length;
  await page.keyboard.down('ArrowUp');
  await sign(-1);
  const refreshed = actions.slice(oldBrake);
  assert.ok(refreshed.findIndex(a => a.action === 'stop') >= 0 &&
    refreshed.findIndex(a => a.action === 'arm') > refreshed.findIndex(a => a.action === 'stop'), JSON.stringify(refreshed));
  await page.keyboard.up('ArrowUp');
  await settled();
  await page.locator('#keyboard-enable').click();
  await disabled();

  stage = 'session lifecycle';
  // Disable/re-enable during an in-flight BEGIN: old END must finish before
  // the new BEGIN, and a key still held during opening should take effect.
  const firstBegin = new Promise(resolve => { resolveIntercept = resolve; });
  beginCount = endCount = 0;
  routeMode = 'lifecycle';
  const lifecycle = actions.length;
  const replyStart = responses.length;
  await page.locator('#keyboard-enable').click();
  await expectRequest(firstBegin, 'old BEGIN');
  await page.locator('#keyboard-enable').click();
  await page.locator('#keyboard-enable').click();
  await page.keyboard.down('ArrowUp');
  await enabled();
  await sign(-1);
  const life = actions.slice(lifecycle);
  const begins = life.filter(a => a.action === 'begin');
  const oldEnd = life.find(a => a.action === 'end');
  const endReply = responses.slice(replyStart).find(a => a.action === 'end');
  assert.equal(begins.length, 2, JSON.stringify(life));
  assert.ok(oldEnd && endReply && endReply.status === 200 && endReply.at <= begins[1].at,
    'new BEGIN overtook old END: ' + JSON.stringify({life, endReply}));
  routeMode = 'normal';
  await page.keyboard.up('ArrowUp');
  await settled();
  assert.deepEqual(errors, []);
  console.log('WASD keyboard safety + focus + delayed requests: OK');
} catch (error) {
  console.error('scenario stage:', stage, 'recent requests:', JSON.stringify(actions.slice(-15)));
  failure = error;
} finally {
  await Promise.race([browser.close().catch(() => {}), new Promise(resolve => setTimeout(resolve, 1500))]);
}
if (failure) { console.error(failure); process.exit(1); }
process.exit(0);
