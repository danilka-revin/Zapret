/* Optional full-stack check: real Chromium -> kiosk API -> four PTY firmware
 * simulators. Set RC_TEST_PLAYWRIGHT_CORE to playwright-core's entry point and
 * RC_TEST_CHROMIUM to the Chromium executable; tests/test_firmware.py starts
 * the temporary HTTP server and the simulated USB ports. No real motors. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';

const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const origin = process.argv[2];
const browser = await chromium.launch({
  executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'],
  headless: true,
});
const context = await browser.newContext({viewport: {width: 1200, height: 900}, hasTouch: true});
const page = await context.newPage();
const errors = [];
page.on('pageerror', error => errors.push(String(error)));

// waitForFunction treats a Promise object as truthy on some Playwright
// versions. Poll resolved JSON in Node so telemetry assertions really wait.
const telemetry = async (predicate) => {
  const deadline = Date.now() + 8000;
  let state;
  while (Date.now() < deadline) {
    state = await (await fetch(origin + '/api/kiosk_state', {cache: 'no-store'})).json();
    if (state.ok && predicate(state.data.motors || [])) return;
    await page.waitForTimeout(150);
  }
  throw new Error('telemetry timeout: ' + JSON.stringify(state && state.data && state.data.motors));
};
const valuesAre = async expected => telemetry(motors => Object.entries(expected).every(([id, value]) => {
  const motor = motors.find(item => item.id === id);
  return motor && motor.online && motor.driveServoValue === value;
}));
const allValuesAre = value => valuesAre(Object.fromEntries(['FL', 'FR', 'RL', 'RR'].map(id => [id, value])));
const scRpmAre = async expected => telemetry(motors => Object.entries(expected).every(([id, rpm]) => {
  const motor = motors.find(item => item.id === id);
  return motor && motor.online && motor.rpm === null && motor.tach === false &&
    typeof motor.scRpm === 'number' && Math.abs(motor.scRpm - rpm) < 0.6;
}));

const setSpeed = async (value, selector = '#drive-speed') => {
  const slider = page.locator(selector);
  await slider.scrollIntoViewIfNeeded();
  await slider.focus();
  // Dragging directly 150→2 must not pass through 0: reaching zero
  // intentionally disarms the latched direction and requires a new click.
  await slider.evaluate((element, next) => {
    element.value = String(next);
    element.dispatchEvent(new Event('input', {bubbles: true}));
  }, value);
  assert.equal(await slider.inputValue(), String(value));
};

const touch = async selector => {
  const button = page.locator(selector);
  await button.scrollIntoViewIfNeeded();
  const box = await button.boundingBox();
  assert.ok(box, `button not visible: ${selector}`);
  await page.touchscreen.tap(box.x + box.width / 2, box.y + box.height / 2);
};

try {
  await page.goto(origin + '/main.html', {waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => typeof window._showSteeringPage === 'function');
  await page.evaluate(() => window._showSteeringPage());
  await page.waitForFunction(() => !document.getElementById('drive-speed').disabled, null, {timeout: 10000});
  // Independent test pulses on D19 must appear in all onboard readouts without
  // starting the motors or entering the legacy feedback fields.
  await scRpmAre({FL: 60, FR: 120, RL: 30, RR: 0});
  await allValuesAre(0);
  await page.waitForFunction(() => {
    const near = (id, expected) => Math.abs(Number(document.getElementById(id).textContent) - expected) < 0.6;
    return near('rpm-FL', 60) && near('rpm-FR', 120) && near('rpm-RL', 30) && near('rpm-RR', 0) &&
      near('drive-module-rpm-FL', 60) && near('drive-module-rpm-FR', 120) &&
      near('drive-module-rpm-RL', 30) && near('drive-module-rpm-RR', 0) &&
      document.getElementById('drive-rpm-reported').textContent.includes('RR: 0.0');
  }, null, {timeout: 8000});

  await setSpeed(2);
  await touch('#direct-drive-forward-ALL');
  await allValuesAre(2);
  // A normal tap has already ended. More than two watchdog periods must pass
  // with the latest value still arriving, rather than stopping on pointerup.
  await page.waitForTimeout(1600);
  await allValuesAre(2);
  assert.equal(await page.locator('#direct-drive-forward-ALL').getAttribute('aria-pressed'), 'true');
  assert.equal(await page.locator('#direct-drive-back-ALL').isDisabled(), true);

  await setSpeed(150);
  await allValuesAre(150);
  await scRpmAre({FL: 60, FR: 120, RL: 30, RR: 0});
  await page.waitForFunction(() => document.getElementById('drive-servo-reported').textContent.includes('RR: 150'));
  await setSpeed(2);
  await allValuesAre(2);
  await page.waitForTimeout(800);
  await allValuesAre(2);

  await touch('#drive-stop');
  await allValuesAre(0);
  await page.waitForTimeout(800);
  await allValuesAre(0);
  assert.equal(await page.locator('#drive-speed').inputValue(), '2');
  assert.equal(await page.locator('#direct-drive-forward-ALL').getAttribute('aria-pressed'), 'false');

  // The individual wheel card also latches and updates independently.
  await page.locator('#steer-fine > summary').click();
  await setSpeed(2, '#drive-module-FL');
  await touch('#direct-drive-forward-FL');
  await valuesAre({FL: 2, FR: 0, RL: 0, RR: 0});
  const forwardFL = page.locator('#direct-drive-forward-FL');
  assert.equal(await forwardFL.getAttribute('aria-pressed'), 'true');
  assert.notEqual(await forwardFL.evaluate(element => getComputedStyle(element).boxShadow), 'none');
  assert.equal(await page.locator('#direct-drive-back-FL').isDisabled(), true);
  await page.waitForTimeout(800);
  await valuesAre({FL: 2, FR: 0, RL: 0, RR: 0});
  await setSpeed(150, '#drive-module-FL');
  await valuesAre({FL: 150, FR: 0, RL: 0, RR: 0});
  await touch('#drive-module-stop-FL');
  await allValuesAre(0);

  // A native Enter activation starts the other direction; losing the visible
  // page is still fail-closed and does not resume after focus returns.
  const back = page.locator('#direct-drive-back-ALL');
  await back.focus();
  await page.keyboard.press('Enter');
  await allValuesAre(2);
  // Headless Chromium may not generate blur when switching tabs.
  await page.evaluate(() => window.dispatchEvent(new Event('blur')));
  await page.waitForTimeout(900);
  await allValuesAre(0);
  await page.waitForTimeout(800);
  await allValuesAre(0);
  assert.deepEqual(errors, []);
  console.log('Chromium touch/keyboard + kiosk API + four simulated USB boards: OK');
} finally {
  await browser.close();
}
