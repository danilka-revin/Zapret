/* Optional full-stack geometry check: Chromium -> kiosk HTTP -> real backend
 * and an isolated configuration file. The connected USB boards are simulated;
 * this test never starts a motor. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';

const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const origin = process.argv[2];
const browser = await chromium.launch({
  executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'],
  headless: true,
});
const page = await browser.newPage({viewport: {width: 1200, height: 900}});
const errors = [];
const geometryPosts = [];
const maneuverPosts = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if (request.method() !== 'POST') return;
  if (request.url().endsWith('/api/kiosk_maneuver_config')) geometryPosts.push(request.postDataJSON());
  if (request.url().endsWith('/api/kiosk_maneuver')) maneuverPosts.push(request.postDataJSON());
});

try {
  await page.goto(origin + '/main.html', {waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => typeof window._showSteeringPage === 'function');
  await page.evaluate(() => window._showSteeringPage());
  await page.locator('#drive-maneuvers > summary').click();
  await page.waitForFunction(() =>
    document.getElementById('geometry-wheelbaseMm').value === '500' &&
    document.getElementById('geometry-trackMm').value === '450' &&
    document.getElementById('geometry-wheelDiameterMm').value === '207.5', null, {timeout: 10000});
  assert.deepEqual(geometryPosts, []); // The old saved value wasn't migrated on load.

  await page.locator('#geometry-trackMm').fill('380');
  await page.waitForTimeout(800); // Fresh telemetry must not overwrite the unsaved edit.
  assert.equal(await page.locator('#geometry-trackMm').inputValue(), '380');
  await page.locator('#maneuver-spin_left').click();
  await page.waitForFunction(() => document.getElementById('maneuver-status').textContent.includes('Сначала сохраните'));
  assert.deepEqual(geometryPosts, []); // No config write before explicit save.
  assert.deepEqual(maneuverPosts, []); // The dirty form cannot start a maneuver.
  const before = await (await fetch(origin + '/api/kiosk_state', {cache: 'no-store'})).json();
  assert.equal(before.data.maneuverConfig.chassis.trackMm, 450);
  assert.equal(before.data.drive.maneuver.active, false);

  const response = page.waitForResponse(r => r.url().endsWith('/api/kiosk_maneuver_config') && r.request().method() === 'POST');
  await page.locator('#geometry-save').click();
  assert.equal((await response).status(), 200);
  await page.waitForFunction(() => document.getElementById('geometry-status').textContent.includes('сохранены'));
  assert.deepEqual(geometryPosts, [{name: 'chassis', values: {
    wheelDiameterMm: 207.5, wheelbaseMm: 500, trackMm: 380,
  }}]);

  await page.reload({waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() =>
    document.getElementById('geometry-trackMm').value === '380' &&
    document.getElementById('geometry-wheelDiameterMm').value === '207.5', null, {timeout: 10000});
  assert.deepEqual(errors, []);
  console.log('Chromium geometry: preserved until saved, persisted after reload: OK');
} finally {
  await browser.close();
}
