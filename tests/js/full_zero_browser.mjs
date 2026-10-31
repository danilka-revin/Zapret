/* Optional real Chromium + simulated fw23 boards; run by pytest when
 * ZAPRET_BROWSER_NODE_MODULES points to playwright-core and @sparticuz/chromium.
 * The HTTP owner and USB PTYs are started and stopped by the test fixture. */
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {join} from 'node:path';
const modules = process.env.ZAPRET_BROWSER_NODE_MODULES;
if (!modules || !process.argv[2]) throw new Error('browser node_modules and URL required');
const require = createRequire(import.meta.url);
const {chromium} = require(join(modules, 'playwright-core'));
const chrome = (await import(join(modules, '@sparticuz/chromium/build/index.js'))).default;
const {inflate} = await import(join(modules, '@sparticuz/chromium/build/lambdafs.js'));
await inflate(join(modules, '@sparticuz/chromium/bin/al2023.tar.br'));
process.env.LD_LIBRARY_PATH = '/tmp/al2023/lib' + (process.env.LD_LIBRARY_PATH ? ':' + process.env.LD_LIBRARY_PATH : '');
const browser = await chromium.launch({executablePath: await chrome.executablePath(), args: chrome.args});
const page = await browser.newPage({viewport: {width: 1280, height: 800}});
const errors = [];
const actions = [];
const remote = process.argv[3] === 'remote';
page.on('pageerror', error => errors.push(String(error)));
page.on('request', req => {
  if ((req.url().includes('/api/kiosk_full_zero') || req.url().includes('/api/control/full_zero')) && req.method() === 'POST') {
    try { actions.push(req.postDataJSON()); } catch (_) { /* ignore */ }
  }
});
try {
  if (remote) {
    await page.goto(process.argv[2] + '/login', {waitUntil: 'domcontentloaded'});
    await page.locator('input[name="username"]').fill('operator');
    await page.locator('input[name="password"]').fill('correct-horse-battery');
    await Promise.all([page.waitForURL(url => !url.pathname.endsWith('/login')),
                       page.locator('button[type="submit"]').click()]);
    await page.goto(process.argv[2] + '/controls', {waitUntil: 'domcontentloaded'});
  } else {
    await page.goto(process.argv[2] + '/kiosk', {waitUntil: 'domcontentloaded'});
  }
  await page.waitForFunction(() => typeof window._openSwipe === 'function');
  await page.evaluate(() => window._openSwipe('motors'));
  await page.locator('#full-zero-open').click();
  await page.waitForSelector('#full-zero-modal:not([hidden])');
  await page.waitForFunction(() => !document.querySelector('#full-zero-calibrate').disabled, null, {timeout: 12000});
  assert.match(await page.locator('#full-zero-message').innerText(), /Тормоз подтверждён/);
  await page.setViewportSize({width: 390, height: 844});
  for (const selector of ['.full-zero-card', '#full-zero-cancel', '#full-zero-estop', '#full-zero-save']) {
    const box = await page.locator(selector).boundingBox();
    assert(box && box.x >= 0 && box.x + box.width <= 390 && box.y >= 0 && box.y + box.height <= 844,
           selector + ' outside mobile viewport');
  }
  await page.locator('[data-full-zero-wheel="RR"]').scrollIntoViewIfNeeded();
  assert.equal(await page.locator('[data-full-zero-wheel="RR"]').isVisible(), true);
  await page.setViewportSize({width: 1280, height: 800});
  assert.equal(await page.locator('#full-zero-save').isDisabled(), true);
  for (const id of ['FL', 'FR', 'RL', 'RR']) {
    assert.equal(await page.locator(`[data-full-zero-adjust="${id}"][data-delta="0.1"]`).isDisabled(), true);
  }
  await page.locator('#full-zero-calibrate').click();
  await page.waitForFunction(() => !document.querySelector('#full-zero-save').disabled, null, {timeout: 40000});
  for (const [id, delta] of [['FL', '0.1'], ['FR', '-0.1'], ['RL', '1'], ['RR', '-1']]) {
    const button = page.locator(`[data-full-zero-adjust="${id}"][data-delta="${delta}"]`);
    await button.click();
    await page.waitForFunction(() => !document.querySelector('#full-zero-save').disabled, null, {timeout: 10000});
  }
  assert(actions.some(body => body.action === 'adjust' && body.module === 'FL' && body.delta === 0.1));
  assert(actions.some(body => body.action === 'adjust' && body.module === 'FR' && body.delta === -0.1));
  await page.locator('#full-zero-save').click();
  await page.waitForFunction(() => document.querySelector('#full-zero-message').textContent.includes('все четыре платы подтвердили'), null, {timeout: 12000});
  for (const id of ['FL', 'FR', 'RL', 'RR']) {
    assert.match(await page.locator('#full-zero-state-' + id).innerText(), /запись подтверждена/);
  }
  assert.equal(await page.locator('#full-zero-save').isDisabled(), true);
  assert.deepEqual(errors, []);
  await page.locator('#full-zero-close').click();
  assert.equal(await page.locator('#full-zero-modal').isHidden(), true);
  // Start another physical calibration and STOP it while it is running.
  await page.locator('#full-zero-open').click();
  await page.waitForFunction(() => !document.querySelector('#full-zero-calibrate').disabled, null, {timeout: 12000});
  await page.locator('#full-zero-calibrate').click();
  // Do not wait for the response: cancellation must win even when the
  // calibrate HTTP request is delayed or still in flight through the proxy.
  await page.waitForTimeout(100);
  await page.locator('#full-zero-cancel').click();
  assert.equal(await page.locator('#full-zero-modal').isHidden(), true);
  await page.waitForTimeout(500);
  assert(actions.some(body => body.action === 'cancel'));

  // Exercise error presentation and late-response handling independently of
  // the real simulator: these HTTP replies deliberately model a partial save.
  let begins = 0, calibrations = 0, mockPhase = 'ready';
  const mockMotors = Object.fromEntries(['FL', 'FR', 'RL', 'RR'].map(id =>
    [id, {online: true, firmware: 23, angle: 0, targetAngle: 0}]));
  const result = (phase, extra = {}) => ({ok: true, phase, motors: mockMotors,
    calibratingSeen: [], acknowledged: [], pendingRecovery: true, ...extra});
  await page.route(remote ? '**/api/control/full_zero' : '**/api/kiosk_full_zero', async route => {
    const request = route.request().postDataJSON();
    let data, status = 200;
    if (request.action === 'begin') {
      begins += 1;
      if (begins === 1) { status = 409; data = {ok: false, reason: 'module_offline', pendingRecovery: true}; }
      else { mockPhase = 'ready'; data = result(mockPhase, {token: 'simulated-session'}); }
    } else if (request.action === 'calibrate') {
      calibrations += 1;
      if (calibrations === 1) { mockPhase = 'adjust'; data = result(mockPhase); }
      else { await new Promise(resolve => setTimeout(resolve, 700)); data = result('calibrating'); }
    } else if (request.action === 'save') {
      mockPhase = 'stopping'; status = 409;
      data = result(mockPhase, {ok: false, reason: 'partial_save', acknowledged: ['FL', 'FR']});
    } else if (request.action === 'status') {
      if (mockPhase === 'stopping') mockPhase = 'recovery';
      data = result(mockPhase, {reason: mockPhase === 'recovery' ? 'partial_save' : null});
    } else { data = result('idle'); }
    try { await route.fulfill({status, contentType: 'application/json', body: JSON.stringify(data)}); }
    catch (_) { /* browser may cancel the intentionally delayed reply */ }
  });
  await page.locator('#full-zero-open').click();
  await page.waitForFunction(() => !document.querySelector('#full-zero-retry').hidden, null, {timeout: 8000});
  assert.match(await page.locator('#full-zero-message').innerText(), /частичным.*движение заблокировано/);
  await page.locator('#full-zero-retry').click();
  await page.waitForFunction(() => !document.querySelector('#full-zero-calibrate').disabled, null, {timeout: 8000});
  await page.locator('#full-zero-calibrate').click();
  await page.waitForFunction(() => !document.querySelector('#full-zero-save').disabled, null, {timeout: 8000});
  await page.locator('#full-zero-save').click();
  await page.waitForFunction(() => document.querySelector('#full-zero-message').textContent.includes('движение заблокировано'), null, {timeout: 8000});
  assert.equal(await page.locator('#full-zero-save').isDisabled(), true);
  await page.waitForFunction(() => !document.querySelector('#full-zero-retry').hidden, null, {timeout: 8000});
  await page.locator('#full-zero-cancel').click();

  await page.locator('#full-zero-open').click();
  await page.waitForFunction(() => !document.querySelector('#full-zero-calibrate').disabled, null, {timeout: 8000});
  await page.locator('#full-zero-calibrate').click();
  await page.locator('#full-zero-cancel').click();
  await page.waitForTimeout(900);
  assert.equal(await page.locator('#full-zero-modal').isHidden(), true);
  assert.deepEqual(errors, []);
  console.log('Chromium: calibration, 0.1° adjustments, four EEPROM ACKs, partial save and delayed STOP passed');
} catch (error) {
  console.error(error);
  process.exitCode = 1;
} finally {
  // Chromium/Playwright can retain a transport during in-flight intercepted
  // requests. Bound test teardown; exiting the Node parent terminates its child.
  try { await Promise.race([browser.close(), new Promise(resolve => setTimeout(resolve, 5000))]); }
  catch (error) { console.error(error); process.exitCode = 1; }
}
process.exit(process.exitCode || 0);
