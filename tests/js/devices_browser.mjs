/* Synthetic camera/USB integration: no physical camera is exercised. */
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {join} from 'node:path';
const modules = process.env.ZAPRET_BROWSER_NODE_MODULES;
const require = createRequire(import.meta.url);
const {chromium} = require(join(modules, 'playwright-core'));
const chrome = (await import(join(modules, '@sparticuz/chromium/build/index.js'))).default;
const {inflate} = await import(join(modules, '@sparticuz/chromium/build/lambdafs.js'));
await inflate(join(modules, '@sparticuz/chromium/bin/al2023.tar.br'));
process.env.LD_LIBRARY_PATH = '/tmp/al2023/lib' + (process.env.LD_LIBRARY_PATH ? ':' + process.env.LD_LIBRARY_PATH : '');
const origin = process.argv[2];
const browser = await chromium.launch({headless: true, executablePath: await chrome.executablePath(),
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage']});
const page = await browser.newPage({viewport: {width: 1360, height: 900}});
const errors = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('dialog', dialog => dialog.accept());
try {
  const base64 = await page.evaluate(() => {
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = 2;
    canvas.getContext('2d').fillRect(0, 0, 2, 2);
    return canvas.toDataURL('image/jpeg').split(',')[1];
  });
  assert((await page.request.post(origin + '/__test/frame', {
    data: Buffer.from(base64, 'base64'), headers: {'Content-Type': 'image/jpeg'}
  })).ok());
  await page.goto(origin + '/devices', {waitUntil: 'domcontentloaded'});
  await page.waitForSelector('[data-role="camera_FL"] .preview img', {timeout: 12000});
  assert.equal(await page.locator('.card').count(), 12);
  assert.equal(await page.locator('[data-role="camera_FL"]').getAttribute('data-state'), 'warning');
  assert(await page.locator('#options-camera_FL option[value="/dev/video0"]').count());
  const input = page.locator('#hint-camera_FL');
  await input.fill('/dev/video');
  await page.waitForTimeout(3000);
  assert.equal(await input.inputValue(), '/dev/video', 'poll must not erase an edit');
  await input.fill('/dev/video0');
  await page.locator('[data-role="camera_FL"] .save').click();
  await page.waitForSelector('[data-role="camera_FL"] .confirm:visible', {timeout: 12000});
  await page.locator('[data-role="camera_FL"] .confirm').click();
  await page.waitForFunction(() => document.querySelector('[data-role="camera_FL"]')?.dataset.state === 'verified',
                             null, {timeout: 12000});
  assert((await page.request.post(origin + '/__test/swap-camera')).ok());
  await page.locator('#devices-refresh').click();
  await page.waitForFunction(() => document.querySelector('[data-role="camera_FL"] .card-status')
    ?.textContent.includes('Подсказка не совпала'), null, {timeout: 12000});
  assert(await page.locator('[data-role="camera_FL"] .confirm').isHidden());
  await page.waitForFunction(() => !document.querySelector('[data-role="camera_FL"] .preview img'),
                             null, {timeout: 12000});
  assert((await page.request.post(origin + '/__test/restore-camera')).ok());
  await page.locator('#devices-refresh').click();
  await page.waitForFunction(() => document.querySelector('[data-role="camera_FL"]')?.dataset.state === 'verified',
                             null, {timeout: 12000});
  await page.locator('[data-role="camera_FL"] .clear').click();
  await page.waitForFunction(() => document.querySelector('[data-role="camera_FL"] .card-status')
    ?.textContent.includes('Камера отключена'), null, {timeout: 12000});
  await page.reload({waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => document.querySelector('[data-role="camera_FL"] .card-status')
    ?.textContent.includes('Камера отключена'), null, {timeout: 12000});
  assert(await page.locator('[data-role="camera_FL"] .confirm').isHidden());
  assert.deepEqual(errors, []);
  console.log('camera confirmation, replacement and reset passed');
} finally {
  await page.close();
  await browser.close();
}
