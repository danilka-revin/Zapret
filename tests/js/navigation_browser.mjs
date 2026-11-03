/* Real Chromium -> onboard navigation HTTP -> simulated lidar, map and wheels.
 * This is not a physical test of braking, steering, localization, or sensors. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';

const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const origin = process.argv[2];
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 1400, height: 1100}});
const errors = [], starts = [], confirmations = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if (request.url().endsWith('/api/navigation/start')) {
    starts.push({body: JSON.parse(request.postData()), headers: request.headers()});
  }
});
const waitStatus = async (phase, timeout = 15000) => {
  const deadline = Date.now() + timeout;
  let status;
  while (Date.now() < deadline) {
    status = await (await fetch(origin + '/api/navigation/status')).json();
    if (status.phase === phase) return status;
    if (['blocked', 'error', 'cancelled'].includes(status.phase)) throw Error(JSON.stringify(status));
    await page.waitForTimeout(100);
  }
  throw Error('navigation timeout: ' + JSON.stringify(status));
};
let exitCode = 1;
try {
  await page.goto(origin + '/map', {waitUntil: 'domcontentloaded'});
  await page.locator('#nav-goal-x').fill('2.5');
  await page.locator('#nav-goal-y').fill('0');
  await page.locator('#nav-add-point').click();
  await page.locator('#nav-plan').click();
  await page.waitForFunction(() => !document.getElementById('nav-start').disabled,
    null, {timeout: 12000});
  assert.match(await page.locator('#nav-route-summary').textContent(), /только известный участок/i);

  page.once('dialog', async dialog => { confirmations.push(dialog.message()); await dialog.dismiss(); });
  await page.locator('#nav-start').click();
  assert.match(confirmations[0], /слепая зона/i);
  assert.match(confirmations[0], /путь неизвестен/i);
  assert.equal(starts.length, 0);

  page.once('dialog', async dialog => { confirmations.push(dialog.message()); await dialog.accept(); });
  await page.locator('#nav-start').click();
  const status = await waitStatus('succeeded');
  assert.equal(starts.length, 1);
  assert.equal(starts[0].body.blindZoneChecked, true);
  assert.equal(starts[0].headers['x-robot-control'], 'navigation');
  assert.equal(status.goal.x, 2.5);
  assert.deepEqual(errors, []);
  console.log('real browser, partial route, explicit blind-zone confirmation, simulated goal: OK');
  exitCode = 0;
} catch (error) {
  console.error(error);
} finally {
  try {
    await Promise.race([browser.close(), new Promise(resolve => setTimeout(resolve, 2000))]);
  } catch (error) {
    console.error('Browser cleanup failed:', error);
    exitCode = 1;
  }
  // Chromium can leave handles open in CI, but never turn a failed test green.
  process.exit(exitCode);
}
