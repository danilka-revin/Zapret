/* Full-stack simulation: real browser -> kiosk -> four PTY firmware boards.
 * The simulator's SC pulses follow a fake motor model, not physical motors. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const origin = process.argv[2];
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const context = await browser.newContext({viewport: {width: 1200, height: 900}});
const page = await context.newPage();
const errors = [], actions = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if (request.url().includes('/api/kiosk_keyboard') && request.postData()) {
    try { actions.push(JSON.parse(request.postData())); } catch (_) {}
  }
});
const poll = async (predicate, timeout = 12000) => {
  const end = Date.now() + timeout;
  let data;
  while (Date.now() < end) {
    data = (await (await fetch(origin + '/api/kiosk_state')).json()).data;
    if (predicate(data)) return data;
    await page.waitForTimeout(100);
  }
  throw new Error('telemetry timeout: ' + JSON.stringify({motors: data && data.motors,
    status: await page.locator('#keyboard-status').textContent(), actions: actions.slice(-12)}));
};
const waitWheels = async (angles, sign) => poll(data => Object.entries(angles).every(([id, angle]) => {
  const m = data.motors.find(item => item.id === id);
  return m && m.online && !m.moving && Math.abs(m.angle - angle) <= 1.5 &&
    (sign === 0 ? m.drivePercent === 0 : Math.sign(m.drivePercent) === sign);
}));
const all = angle => ({FL: angle, FR: angle, RL: angle, RR: angle});
const stopped = async () => poll(data => data.motors.length === 4 &&
  data.motors.every(m => m.online && m.drivePercent === 0), 6000);
const enable = async () => {
  await page.locator('#keyboard-enable').click();
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'true', null, {timeout: 10000});
  await page.locator('#keyboard-enable').focus();
};

try {
  await page.goto(origin + '/main.html', {waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => typeof window._showSteeringPage === 'function');
  await page.evaluate(() => window._showSteeringPage());
  await poll(data => data.motors.length === 4 && data.motors.every(m => m.online && m.homed && m.firmware === 17), 10000);
  await enable();

  // Краб: D действительно едет вправо боком на +90°.
  await page.keyboard.down('d');
  await waitWheels(all(90), 1);
  await page.waitForTimeout(500);
  await page.keyboard.up('d');
  await stopped();
  await page.waitForTimeout(800);
  await stopped();

  // Обычное руление: передние колёса поворачиваются, задние прямо.
  await page.locator('#keyboard-mode').selectOption('normal');
  await page.locator('#keyboard-enable').focus();
  await page.keyboard.down('w');
  await waitWheels(all(0), 1);
  const actionStart = actions.length;
  await page.keyboard.down('d');
  await waitWheels({FL: 25, FR: 25, RL: 0, RR: 0}, 1);
  const changed = actions.slice(actionStart);
  const stopIndex = changed.findIndex(a => a.action === 'stop');
  const steerIndex = changed.findIndex(a => a.action === 'steer');
  const armIndex = changed.findIndex(a => a.action === 'arm');
  const driveIndex = changed.findIndex(a => a.action === 'drive');
  assert.ok(stopIndex >= 0 && stopIndex < steerIndex && steerIndex < armIndex && armIndex < driveIndex,
    JSON.stringify(changed));
  await page.keyboard.up('d');
  await waitWheels(all(0), 1);  // W ещё удерживается, после подтверждения руля продолжаем
  await page.keyboard.up('w');
  await stopped();

  // A без W/S только рулит, моторы не запускаются.
  await page.keyboard.down('a');
  await waitWheels({FL: -25, FR: -25, RL: 0, RR: 0}, 0);
  await page.keyboard.up('a');
  await stopped();

  // Отпускание во время поворота инвалидирует все старые асинхронные задачи.
  await page.locator('#keyboard-mode').selectOption('crab');
  await page.locator('#keyboard-enable').focus();
  await page.keyboard.down('d');
  await page.waitForTimeout(100);
  await page.keyboard.up('d');
  await page.waitForTimeout(2200);
  await stopped();

  // Отрицательное направление и потеря фокуса: ни одного самопроизвольного рестарта.
  await page.keyboard.down('s');
  await waitWheels(all(0), -1);
  // Headless Chromium не всегда меняет focus при открытии второй вкладки;
  // явно доставляем такое же событие, как при потере фокуса окна.
  await page.evaluate(() => window.dispatchEvent(new Event('blur')));
  await page.waitForTimeout(900);
  await stopped();
  assert.equal(await page.locator('#keyboard-enable').getAttribute('aria-pressed'), 'false');
  await page.waitForTimeout(800);
  await stopped();
  await page.keyboard.up('s');

  // Потеря телеметрии при удержании: отключить режим и не возобновлять его.
  await enable();
  await page.keyboard.down('w');
  await waitWheels(all(0), 1);
  await page.route('**/api/kiosk_state', route => route.abort());
  await page.waitForFunction(() => document.getElementById('keyboard-enable').getAttribute('aria-pressed') === 'false', null, {timeout: 4000});
  await page.unroute('**/api/kiosk_state');
  await stopped();
  await page.waitForTimeout(900);
  await stopped();
  await page.keyboard.up('w');
  assert.deepEqual(errors, []);
  console.log('WASD browser + kiosk + four simulated USB boards: OK');
} finally {
  await browser.close();
}
