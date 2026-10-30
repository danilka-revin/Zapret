import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const origin = process.argv[2];
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage();
const poll = async (predicate, timeout = 10000) => {
  const deadline = Date.now() + timeout;
  while (Date.now() < deadline) {
    const data = (await (await fetch(origin + '/api/kiosk_state')).json()).data;
    if (predicate(data)) return;
    await page.waitForTimeout(100);
  }
  throw new Error('wheel telemetry timeout: ' + await page.locator('#keyboard-status').textContent());
};
try {
  await page.goto(origin + '/main.html');
  await page.evaluate(() => window._showSteeringPage());
  await poll(data => data.motors.length === 4 && data.motors.every(m => m.online && m.homed));
  await page.locator('#keyboard-enable').click();
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'true');
  await page.locator('#keyboard-enable').focus();
  await page.keyboard.down('ArrowUp');
  await poll(data => data.motors.every(m => m.drivePercent > 0));
  // No simulator pulses: the server, not only the browser, must stop all wheels.
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'false', null, {timeout: 6500});
  await poll(data => data.motors.every(m => m.drivePercent === 0), 3000);
  assert.match(await page.locator('#keyboard-status').textContent(), /SC/);
  await page.waitForTimeout(800);
  await poll(data => data.motors.every(m => m.drivePercent === 0), 3000);
  console.log('WASD missing SC: stopped');
} finally { await browser.close(); }
