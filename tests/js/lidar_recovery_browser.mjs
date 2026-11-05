/* Onboard UI -> Flask -> one silent simulated LDS PTY. No real wiring is tested. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const [origin, port] = process.argv.slice(2);
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
const page = await browser.newPage({viewport: {width: 390, height: 900}});
page.setDefaultTimeout(10000);
const requests = [], errors = [];
let confirm = false;
page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if (request.url().includes('/api/devices/lidar/')) requests.push(request.url());
});
page.on('dialog', dialog => confirm ? dialog.accept() : dialog.dismiss());
try {
  await page.goto(origin + '/devices', {waitUntil: 'domcontentloaded'});
  const card = page.locator('[data-role="lidar"]');
  await card.waitFor();
  const button = card.getByRole('button', {name: 'Проверить и запустить LDS‑01'});
  await button.waitFor({state: 'visible'});
  await card.locator('input').fill(port);
  await button.click();
  await card.locator('[role="status"]').filter({hasText: 'Запуск отменён'}).waitFor();
  assert.equal(requests.filter(url => url.endsWith('/start')).length, 0);
  assert.equal(requests.filter(url => url.endsWith('/prepare')).length, 1);

  confirm = true;
  await button.click();
  await card.locator('[role="status"]').filter({hasText: 'Кадры LDS-01 с контрольной суммой получены'}).waitFor();
  assert.equal(requests.filter(url => url.endsWith('/start')).length, 1);
  assert.equal(requests.filter(url => url.endsWith('/prepare')).length, 2);

  // A failed ROS driver is a symptom, not a guessed diagnosis. Check the
  // actual settings modal displays the supervisor's detail and escapes it.
  const status = {connected: false, reason: 'Нет свежего скана', ros: {connected: false,
    supervisor: {state: 'failed', driver: 'hls_lfcd_lds_driver',
      reason: 'Драйвер не найден <img src=x>', hint: 'Проверьте пакет ROS'}}};
  await page.route('**/api/lidar/settings', route => route.fulfill({status: 200,
    contentType: 'application/json', body: JSON.stringify({ok: true, settings: {}, options: {ports: []}, status})}));
  await page.route('**/api/lidar', route => route.fulfill({status: 200,
    contentType: 'application/json', body: JSON.stringify({ok: true, ...status})}));
  await page.goto(origin + '/main.html', {waitUntil: 'domcontentloaded'});
  await page.locator('#rc-lidar-launch').click();
  const detail = page.locator('#rc-lidar-status');
  await detail.filter({hasText: 'Проверьте пакет ROS'}).waitFor();
  const text = await detail.textContent();
  assert.ok(text.includes('failed') && text.includes('Драйвер не найден <img src=x>'), text);
  assert.equal(await detail.locator('img').count(), 0, 'supervisor text must be escaped');
  assert.deepEqual(errors, []);
  console.log('LDS recovery UI + simulated UART: OK');
} finally { await browser.close(); }
