/* Optional Chromium: real onboard API, wheel/BMS/lidar PTYs, manual port input. */
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {join} from 'node:path';
const modules = process.env.ZAPRET_BROWSER_NODE_MODULES;
const [url, bmsPort] = process.argv.slice(2);
if (!modules || !url || !bmsPort) throw new Error('browser modules, URL and BMS path required');
const require = createRequire(import.meta.url);
const {chromium} = require(join(modules, 'playwright-core'));
const chrome = (await import(join(modules, '@sparticuz/chromium/build/index.js'))).default;
// @sparticuz/chromium exports differ between versions: older ones export a
// standalone inflate(), newer ones a LambdaFS class with a static inflate().
const lambdafs = await import(join(modules, '@sparticuz/chromium/build/lambdafs.js'));
const lambdaFS = lambdafs.default?.default || lambdafs.default;
const inflate = typeof lambdafs.inflate === 'function' ? lambdafs.inflate
  : (typeof lambdaFS?.inflate === 'function' ? lambdaFS.inflate.bind(lambdaFS) : null);
assert(inflate, 'unsupported @sparticuz/chromium lambdafs version');
await inflate(join(modules, '@sparticuz/chromium/bin/al2023.tar.br'));
process.env.LD_LIBRARY_PATH = '/tmp/al2023/lib' + (process.env.LD_LIBRARY_PATH ? ':' + process.env.LD_LIBRARY_PATH : '');
const browser = await chromium.launch({executablePath: await chrome.executablePath(), args: chrome.args});
const page = await browser.newPage({viewport: {width: 1000, height: 750}});
const errors = [], posts = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('request', req => {
  if (req.method() === 'POST' && req.url().includes('/api/battery/')) {
    try { posts.push([new URL(req.url()).pathname, req.postDataJSON()]); } catch (_) {}
  }
});
try {
  await page.goto(url + '/kiosk', {waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => typeof window._openSwipe === 'function');
  await page.evaluate(() => window._openSwipe('battery'));
  await page.waitForSelector('#bms-setup:not([hidden])');
  assert.equal(await page.locator('#bms-scan').count(), 0);
  assert.equal(await page.locator('#bms-port-manual').inputValue(), '');
  await page.locator('#bms-probe').click();
  assert.match(await page.locator('#bms-setup-msg').innerText(), /Выберите порт/);
  assert.equal(posts.length, 0);
  await page.locator('#bms-port-manual').fill(bmsPort);
  await page.locator('#bms-probe').click();
  await page.waitForFunction(() => document.querySelector('#bms-setup-msg').textContent.includes('Найдена BMS'),
                             null, {timeout: 10000});
  await page.locator('#bms-save').click();
  await page.waitForFunction(() => document.querySelector('#bms-setup-msg').textContent.includes('Сохранено'),
                             null, {timeout: 10000});
  await page.waitForFunction(() => document.querySelector('#bms-soc').textContent === '55',
                             null, {timeout: 15000});
  assert.deepEqual(posts.filter(([path]) => path === '/api/battery/probe').map(([, data]) => data.port), [bmsPort]);
  assert.deepEqual(posts.filter(([path]) => path === '/api/battery/port').map(([, data]) => data.port), [bmsPort]);
  assert(!posts.some(([path]) => path === '/api/battery/scan'));
  assert.deepEqual(errors, []);
  console.log('Chromium: explicit BMS path, single-port probe, persistent selection and telemetry passed');
} catch (error) {
  console.error(error);
  process.exitCode = 1;
} finally {
  try { await Promise.race([browser.close(), new Promise(resolve => setTimeout(resolve, 5000))]); }
  catch (error) { console.error(error); process.exitCode = 1; }
}
process.exit(process.exitCode || 0);
