/* Chromium -> onboard (or authenticated web proxy) -> four simulated PTYs.
 * External movement and disconnection are simulator control files only. */
import assert from 'node:assert/strict';
import {writeFile, rename} from 'node:fs/promises';
import {join} from 'node:path';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const [origin, kiosk, directory, mode] = process.argv.slice(2);
const remote = mode === 'remote';
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 1200, height: 900}});
page.setDefaultTimeout(12000);
const errors = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('dialog', dialog => dialog.accept());
async function external(id, degrees, connected = true) {
  const tmp = join(directory, id + '.tmp');
  await writeFile(tmp, `${degrees} ${connected ? 1 : 0}`);
  await rename(tmp, join(directory, id));
}
async function waitState(predicate, timeout = 10000) {
  const end = Date.now() + timeout;
  let data;
  while (Date.now() < end) {
    data = (await (await fetch(kiosk + '/api/kiosk_state', {signal: AbortSignal.timeout(4000)})).json()).data;
    if (predicate(data)) return data;
    await page.waitForTimeout(50);
  }
  throw new Error('state timeout: ' + JSON.stringify(data?.motors));
}
function motor(data, id) { return data.motors.find(m => m.id === id); }
try {
  if (remote) {
    await page.goto(origin + '/login');
    await page.locator('input[name="username"]').fill('operator');
    await page.locator('input[name="password"]').fill('correct-horse-battery');
    await Promise.all([page.waitForURL(url => url.pathname !== '/login'),
      page.locator('button[type="submit"]').click()]);
  }
  await page.goto(origin + (remote ? '/controls?open=calibration' : '/kiosk?open=calibration'),
    {waitUntil: 'domcontentloaded'});
  await page.waitForSelector('.pot-hold__switch');
  await page.locator('#steer-fine').evaluate(el => { el.open = true; });
  await page.waitForFunction(() => !document.querySelector('.pot-hold__switch').disabled &&
    document.querySelector('#steer-angle-source-FR').textContent === 'потенциометр A0');
  assert.equal(await page.locator('.pot-hold__switch').getAttribute('aria-checked'), 'false');
  await page.waitForFunction(() => document.querySelector('#angle-source-FR').textContent.includes('A0'));
  assert.match(await page.locator('#angle-source-FR').textContent(), /A0/);

  // Wheel physically moves without generated steps; even reverse-wired A0 has
  // the correct sign. Readouts update while the controller step count stays 0.
  await external('FR', 7);
  const displaced = await waitState(data => motor(data, 'FR').potAngle > 5);
  assert(Math.abs(motor(displaced, 'FR').angle) < .1);
  await page.waitForFunction(() => parseFloat(document.querySelector('#steer-current-FR').textContent) > 5);
  assert(parseFloat(await page.locator('#angle-FR').textContent()) > 5);
  assert.equal(await page.locator('#steer-angle-source-FR').textContent(), 'потенциометр A0');

  await external('FR', 20);
  await page.waitForFunction(() => document.querySelector('.pot-hold__switch').disabled &&
    document.querySelector('.pot-hold__status').textContent.includes('FR') &&
    document.querySelector('.pot-hold__status').textContent.includes('15°'));
  assert((await waitState(data => motor(data, 'FR').potAngle > 16)).motors.every(m => m.potHold === false));
  await external('FR', 0);
  await page.waitForFunction(() => !document.querySelector('.pot-hold__switch').disabled);
  await page.locator('.pot-hold__switch').click();
  await waitState(data => data.motors.every(m => m.potHold === true));
  await page.waitForFunction(() => document.querySelector('.pot-hold__switch').getAttribute('aria-checked') === 'true');
  await external('FR', 6);
  await waitState(data => motor(data, 'FR').moving);
  await waitState(data => !motor(data, 'FR').moving && Math.abs(motor(data, 'FR').potAngle) < 2.5);
  assert((await waitState(data => data.motors.every(m => m.potHold === true))).motors.length === 4);

  // Lose A0 while a second autonomous correction is actually moving. fw26
  // cancels that pass and switches FR off, while other boards stay reported
  // on; the UI must expose the partial state and still allow a global off.
  await external('FR', 12);
  await waitState(data => motor(data, 'FR').moving);
  await external('FR', 12, false);
  const disconnected = await waitState(data => motor(data, 'FR').pot === false &&
    motor(data, 'FR').potHold === false && !motor(data, 'FR').moving &&
    data.motors.filter(m => m.id !== 'FR').every(m => m.potHold === true));
  await page.waitForFunction(() => document.querySelector('.pot-hold__switch').textContent.includes('платы расходятся') &&
    !document.querySelector('.pot-hold__switch').disabled &&
    document.querySelector('.pot-hold__status').textContent.includes('FR'));
  assert.equal(await page.locator('.pot-hold__switch').getAttribute('aria-checked'), null);
  await page.locator('.pot-hold__switch').click();
  await waitState(data => data.motors.every(m => m.potHold === false));
  await page.waitForFunction(() => document.querySelector('.pot-hold__switch').getAttribute('aria-checked') === 'false');
  assert.equal(motor(disconnected, 'FR').potAngle, null);
  assert.equal(motor(disconnected, 'FR').angleSource, 'steps');
  await page.waitForFunction(() => document.querySelector('#steer-angle-source-FR').textContent.includes('датчик не готов') &&
    document.querySelector('.pot-hold__switch').disabled && document.querySelector('.pot-hold__status').textContent.includes('FR'));
  await page.waitForFunction(() => document.querySelector('#angle-source-FR').textContent.includes('шаги'));
  assert.match(await page.locator('#angle-source-FR').textContent(), /шаги/);
  const stoppedAngle = motor(disconnected, 'FR').angle;
  await external('FR', 12, true);
  await waitState(data => motor(data, 'FR').potAngle !== null && data.motors.every(m => !m.potHold));
  await page.waitForTimeout(500);
  const reconnected = await waitState(data => motor(data, 'FR').potAngle !== null);
  assert(Math.abs(motor(reconnected, 'FR').angle - stoppedAngle) < .1, 'reconnection must not restart the correction');
  assert(reconnected.motors.every(m => !m.potHold));
  await external('FR', 12, false);
  await waitState(data => motor(data, 'FR').pot === false);

  // A failed poll cannot leave the last A0 or step reading looking current.
  await page.route(/\/api\/(kiosk_state|control\/state)(\?.*)?$/, route => route.abort('failed'));
  await page.waitForFunction(() => document.querySelector('#steer-current-FR').textContent === '—' &&
    document.querySelector('#steer-angle-source-FR').textContent.includes('нет свежих данных'));
  assert.equal(await page.locator('.pot-hold__switch').getAttribute('aria-checked'), null);
  assert.deepEqual(errors, []);
  console.log('four simulated potentiometers and confirmed hold: OK (' + mode + ')');
} finally {
  await browser.close();
}
