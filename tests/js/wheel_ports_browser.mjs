/* Chromium → onboard HTTP → four simulated firmware USB ports, plus web peer.
 * Fixtures provide the two servers and a test-only unplug/replug endpoint. */
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {join} from 'node:path';
const modules = process.env.ZAPRET_BROWSER_NODE_MODULES;
const [onboard, remote, encodedPorts] = process.argv.slice(2);
if (!modules || !onboard || !remote || !encodedPorts) throw new Error('browser modules, two URLs and ports required');
const ports = JSON.parse(encodedPorts);
const require = createRequire(import.meta.url);
const {chromium} = require(join(modules, 'playwright-core'));
const chrome = (await import(join(modules, '@sparticuz/chromium/build/index.js'))).default;
const {inflate} = await import(join(modules, '@sparticuz/chromium/build/lambdafs.js'));
await inflate(join(modules, '@sparticuz/chromium/bin/al2023.tar.br'));
process.env.LD_LIBRARY_PATH = '/tmp/al2023/lib' + (process.env.LD_LIBRARY_PATH ? ':' + process.env.LD_LIBRARY_PATH : '');
// The Lambda-specific single-process/SwiftShader flags can freeze a busy page
// in this environment. Exercise ordinary multiprocess Chromium instead.
const browser = await chromium.launch({executablePath: await chrome.executablePath(), headless: true,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage']});
const page = await browser.newPage({viewport: {width: 1280, height: 800}});
const errors = [];
const pending = new Map();
const network = [];
let cameraStreams = 0;
let cameraFrames = 0;
function safeUrl(raw) {
  const url = new URL(raw);
  url.searchParams.delete('st');
  return url.toString();
}
function record(message) { network.push(message); if (network.length > 120) network.shift(); }
page.on('pageerror', error => errors.push(String(error)));
page.on('domcontentloaded', () => record(`${Date.now()} DOMContentLoaded ${safeUrl(page.url())}`));
page.on('load', () => record(`${Date.now()} load ${safeUrl(page.url())}`));
page.on('request', req => {
  pending.set(req, req.method() + ' ' + safeUrl(req.url()));
  const path = new URL(req.url()).pathname;
  if (/\/api\/camera\/\d+\/stream/.test(path)) cameraStreams += 1;
  if (/\/api\/camera\/\d+\/frame/.test(path)) cameraFrames += 1;
});
page.on('response', response => {
  const url = new URL(response.url());
  if (!url.pathname.endsWith('.css')) record(`${Date.now()} ${response.status()} ${url.pathname}`);
});
page.on('requestfinished', req => pending.delete(req));
page.on('requestfailed', req => {
  pending.delete(req);
  if (req.url().includes('/api/')) record(`${Date.now()} failed ${safeUrl(req.url())} ${req.failure()?.errorText}`);
});
const ids = ['FL', 'FR', 'RL', 'RR'];
async function waitPorts(selector, expected = ports) {
  await page.waitForFunction(({ids, selector, expected}) => ids.every(id => {
    const node = document.querySelector(selector.replace('%s', id));
    return node && node.textContent.trim() === expected[id];
  }), {ids, selector, expected}, {timeout: 12000});
}
try {
  await page.goto(onboard + '/kiosk', {waitUntil: 'domcontentloaded'});
  await waitPorts('#usb-port-%s');
  await page.waitForFunction(() => typeof window._openSwipe === 'function');
  await page.evaluate(() => { window._openSwipe('motors'); document.querySelector('#steer-fine').open = true; });
  await waitPorts('#steer-usb-port-%s');
  assert.match(await page.locator('#steer-source-status').textContent(), /4\/4/);
  await page.setViewportSize({width: 390, height: 844});
  const small = page.locator('#steer-usb-port-FL');
  await small.scrollIntoViewIfNeeded();
  const box = await small.boundingBox();
  assert(box && box.x >= 0 && box.x + box.width <= 390, 'wheel port overflows the mobile viewport');
  await page.setViewportSize({width: 1280, height: 800});

  // Physical-looking replug: the actual PTY-backed alias is removed and then
  // restored by the Python test, not merely a fake browser response.
  assert((await page.request.post(onboard + '/__test/unplug')).ok());
  const missing = {...ports, RL: '—'};
  await waitPorts('#usb-port-%s', missing);
  await waitPorts('#steer-usb-port-%s', missing);
  await page.waitForFunction(() => document.querySelector('#steer-source-status').textContent.includes('RL'));
  assert((await page.request.post(onboard + '/__test/replug')).ok());
  await waitPorts('#usb-port-%s');
  await waitPorts('#steer-usb-port-%s');

  // A dead server must not leave yesterday's USB assignments on screen.
  await page.route('**/api/kiosk_state', route => route.abort());
  await waitPorts('#usb-port-%s', Object.fromEntries(ids.map(id => [id, '—'])));
  await waitPorts('#steer-usb-port-%s', Object.fromEntries(ids.map(id => [id, '—'])));
  await page.unroute('**/api/kiosk_state');
  await waitPorts('#usb-port-%s');

  // The authenticated remote is a peer; it never claims the USB ports itself.
  await page.goto(remote + '/login', {waitUntil: 'domcontentloaded'});
  await page.locator('input[name="username"]').fill('operator');
  await page.locator('input[name="password"]').fill('correct-horse-battery');
  await Promise.all([page.waitForURL(url => !url.pathname.endsWith('/login')),
                     page.locator('button[type="submit"]').click()]);
  await page.goto(remote + '/controls', {waitUntil: 'domcontentloaded'});
  await waitPorts('#steer-usb-port-%s');
  await page.goto(remote + '/', {waitUntil: 'domcontentloaded'});
  await waitPorts('#usb-port-%s');

  // Exercise real camera frame endpoints while wheel status continues to
  // update. No MJPEG connections should remain open when cameras are absent.
  const emptyIds = ['cam-empty', 'cam-rear-empty', 'cam-right-empty', 'cam-rear-right-empty'];
  await page.waitForFunction(ids => ids.every(id => !document.getElementById(id).hidden), emptyIds);
  const base64 = await page.evaluate(() => {
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = 2;
    canvas.getContext('2d').fillRect(0, 0, 2, 2);
    return canvas.toDataURL('image/jpeg').split(',')[1];
  });
  const jpeg = Buffer.from(base64, 'base64');
  for (let id = 0; id < 4; id++) {
    // External camera feeds go to the onboard owner, not a second manager.
    assert((await page.request.post(`${onboard}/api/camera/${id}/frame`,
      {data: jpeg, headers: {'Content-Type': 'image/jpeg'}})).ok());
  }
  await page.waitForFunction(ids => ids.every(id => document.getElementById(id).hidden), emptyIds);
  assert.match(await page.locator('#cam-meta').textContent(), /4× LIVE/);
  assert.doesNotMatch(await page.locator('#cam-meta').textContent(), /30 fps/);
  await waitPorts('#usb-port-%s');
  // Stale frames disappear, then an individual camera can reconnect on its own.
  await page.waitForFunction(ids => ids.every(id => !document.getElementById(id).hidden), emptyIds);
  assert((await page.request.post(`${onboard}/api/camera/3/frame`,
    {data: jpeg, headers: {'Content-Type': 'image/jpeg'}})).ok());
  await page.waitForFunction(id => document.getElementById(id).hidden, emptyIds[3]);
  assert.equal(await page.locator('#cam-empty').evaluate(el => el.hidden), false);
  assert.equal(await page.locator('#cam-front-badge').evaluate(el => el.hidden), true);
  assert.equal(await page.locator('#cam-rear-right-badge').evaluate(el => el.hidden), false);
  assert.equal(cameraStreams, 0, 'main screen must not hold MJPEG streams open');
  assert(cameraFrames >= 4, 'all camera roles must be polled');

  await page.goto(remote + '/console', {waitUntil: 'domcontentloaded'});
  const framesAfterLeaving = cameraFrames;
  await waitPorts('#modules [data-wheel-port="%s"]');
  await waitPorts('#mod-cards [data-wheel-port="%s"]');
  await page.waitForTimeout(1200);
  assert.equal(cameraFrames, framesAfterLeaving, 'camera polling continued after leaving the main page');
  await page.goto(remote + '/', {waitUntil: 'domcontentloaded'});
  await waitPorts('#usb-port-%s');
  await page.waitForFunction(() => document.getElementById('cam-meta').textContent.length > 0);
  assert(cameraFrames > framesAfterLeaving, 'camera polling did not resume on a repeated visit');
  await page.goto(remote + '/console', {waitUntil: 'domcontentloaded'});
  await waitPorts('#modules [data-wheel-port="%s"]');
  await page.waitForFunction(() => !!window.RCSteeringPanel);
  await Promise.all([
    page.waitForURL(url => url.pathname === '/controls'),
    page.evaluate(() => window.RCSteeringPanel.open())
  ]);
  await waitPorts('#steer-usb-port-%s');

  // The dedicated page presents all twelve roles on both surfaces and lets an
  // operator save an untrusted hint without overriding the live wheel role.
  await page.goto(onboard + '/devices', {waitUntil: 'domcontentloaded'});
  await page.waitForSelector('[data-role="wheel_FL"] .fact-value.port');
  assert.equal(await page.locator('.card').count(), 12);
  await page.waitForFunction(expected => document.querySelector('[data-role="wheel_FL"] .fact-value.port')
    ?.textContent.trim() === expected, ports.FL, {timeout: 12000});
  const onboardHint = page.locator('#hint-wheel_FL');
  await onboardHint.fill('/dev/ttyUSB88');
  await page.locator('[data-role="wheel_FL"] .save').click();
  await page.waitForFunction(() => document.querySelector('[data-role="wheel_FL"] .hint-caption')
    ?.textContent.includes('/dev/ttyUSB88'));
  assert.equal((await page.locator('[data-role="wheel_FL"] .fact-value.port').textContent()).trim(), ports.FL);
  await page.reload({waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => document.querySelector('#hint-wheel_FL')?.value === '/dev/ttyUSB88');
  assert((await page.request.post(onboard + '/__test/unplug')).ok());
  await page.waitForFunction(() => document.querySelector('[data-role="wheel_RL"] .fact-value.port')
    ?.textContent.includes('Не подтверждён'), null, {timeout: 12000});
  assert((await page.request.post(onboard + '/__test/replug')).ok());
  await page.waitForFunction(expected => document.querySelector('[data-role="wheel_RL"] .fact-value.port')
    ?.textContent.trim() === expected, ports.RL, {timeout: 12000});
  await page.locator('[data-role="bms"] .card-link').click();
  await page.waitForURL('**/kiosk?open=bms');
  await page.waitForFunction(() => document.querySelector('#bms-setup') &&
    !document.querySelector('#bms-setup').hidden, null, {timeout: 12000});

  await page.goto(remote + '/devices', {waitUntil: 'domcontentloaded'});
  await page.waitForSelector('[data-role="wheel_FL"] .fact-value.port');
  assert.equal(await page.locator('.card').count(), 12);
  await page.locator('#hint-wheel_FR').fill('/dev/ttyUSB99');
  await page.locator('[data-role="wheel_FR"] .save').click();
  await page.waitForFunction(() => document.querySelector('[data-role="wheel_FR"] .hint-caption')
    ?.textContent.includes('/dev/ttyUSB99'));
  assert.equal((await page.locator('[data-role="wheel_FR"] .fact-value.port').textContent()).trim(), ports.FR);
  await page.setViewportSize({width: 390, height: 844});
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), 'device page overflows mobile');
  await page.route('**/api/devices*', route => route.abort());
  await page.waitForFunction(() => document.querySelector('[data-role="wheel_FL"] .card-status')
    ?.textContent.includes('неизвестно'), null, {timeout: 12000});
  assert.equal((await page.locator('[data-role="wheel_FL"] .fact-value.port').textContent()).trim(), 'Неизвестно');
  await page.unroute('**/api/devices*');
  await page.locator('#devices-refresh').click();
  await page.waitForFunction(expected => document.querySelector('[data-role="wheel_FL"] .fact-value.port')
    ?.textContent.trim() === expected, ports.FL, {timeout: 12000});
  assert.deepEqual(errors, []);
  console.log('Chromium: verified ports on kiosk, peer and console; unplug, replug and stale server passed');
} catch (error) {
  console.error(error);
  const snapshot = await page.evaluate(ids => ({
    url: location.origin + location.pathname, ready: document.readyState, now: Date.now(),
    last: document.body?.lastElementChild?.outerHTML?.slice(0, 250),
    scripts: [...document.scripts].map(s => s.getAttribute('src') || 'inline'),
    resources: performance.getEntriesByType('resource').map(r => ({name: new URL(r.name).pathname, duration: r.duration})).slice(-30),
    ports: Object.fromEntries(ids.map(id => [id, document.querySelector('#usb-port-' + id)?.textContent])),
    steerPorts: Object.fromEntries(ids.map(id => [id, document.querySelector('#steer-usb-port-' + id)?.textContent])),
    link: window.RS_MAIN?.link && {live: window.RS_MAIN.link.live, error: window.RS_MAIN.link.error,
      data: window.RS_MAIN.link.data && {source: window.RS_MAIN.link.data.source,
        motors: window.RS_MAIN.link.data.motors, masterPort: window.RS_MAIN.link.data.masterPort}},
  }), ids).catch(() => null);
  console.error('snapshot:', JSON.stringify(snapshot), '\npage errors:', errors,
    '\npending requests:', [...pending.values()].slice(0, 30), '\nnetwork:', network);
  process.exitCode = 1;
} finally {
  try { await Promise.race([browser.close(), new Promise(resolve => setTimeout(resolve, 5000))]); }
  catch (error) { console.error(error); process.exitCode = 1; }
}
process.exit(process.exitCode || 0);
