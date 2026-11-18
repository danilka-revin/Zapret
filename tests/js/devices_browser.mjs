/* Synthetic camera/USB integration: no physical camera is exercised. */
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {join} from 'node:path';
const modules = process.env.ZAPRET_BROWSER_NODE_MODULES;
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
const origin = process.argv[2];
const board = process.argv[3] || origin;
const remote = process.argv[4] === 'remote';
const browser = await chromium.launch({headless: true, executablePath: await chrome.executablePath(),
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage']});
// The web CSP forbids eval; Playwright's serialized test predicates use it.
const context = await browser.newContext({viewport: {width: 1360, height: 900}, bypassCSP: true});
const page = await context.newPage();
const errors = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('dialog', dialog => dialog.accept());
try {
  // Distinct JPEGs for FL/RL/FR/RR: verify actual canvas pixels, not just
  // connection badges. Chromium has no permission to capture a local camera.
  const colors = [[200, 30, 30], [30, 200, 30], [30, 30, 200], [220, 180, 30]];
  for (const [id, rgb] of colors.entries()) {
    const base64 = await page.evaluate(rgb => {
      const canvas = document.createElement('canvas');
      canvas.width = 32; canvas.height = 18;
      const ctx = canvas.getContext('2d');
      ctx.fillStyle = `rgb(${rgb})`; ctx.fillRect(0, 0, 32, 18);
      return canvas.toDataURL('image/jpeg').split(',')[1];
    }, rgb);
    assert((await page.request.post(board + '/__test/frame/' + id, {
      data: Buffer.from(base64, 'base64'), headers: {'Content-Type': 'image/jpeg'}
    })).ok());
  }
  if (remote) {
    await page.goto(origin + '/login');
    await page.locator('input[name="username"]').fill('operator');
    await page.locator('input[name="password"]').fill('correct-horse-battery');
    await page.locator('form button[type="submit"]').click();
  }
  const screens = remote ? [
    ['/', ['cam-canvas', 'cam-rear-canvas', 'cam-right-canvas', 'cam-rear-right-canvas']],
    ['/console', ['cam', 'cam-rear', 'cam-right', 'cam-rear-right']],
  ] : [['/kiosk', [1, 2, 3, 4].map(n => `cam${n}-canvas`)]];
  for (const [route, ids] of screens) {
    await page.goto(origin + route, {waitUntil: 'domcontentloaded'});
    if (!remote) await page.locator('#k-cameras-open').click();
    await page.waitForFunction(({ids, colors}) => ids.every((id, i) => {
      const c = document.getElementById(id);
      if (!c) return false;
      const pixel = c.getContext('2d').getImageData(c.width * .55, c.height * .45, 1, 1).data;
      return colors[i].every((v, channel) => Math.abs(v - pixel[channel]) < 8);
    }), {ids, colors}, {timeout: 12000});
  }
  await page.goto(origin + '/devices', {waitUntil: 'domcontentloaded'});
  await page.waitForSelector('[data-role="camera_FL"] .preview img', {timeout: 12000});
  assert.equal(await page.locator('#camera-cards .card').count(), 4);
  assert.equal(await page.locator('[data-role="camera_FL"]').getAttribute('data-state'), 'warning');
  assert.equal(await page.locator('.topbar nav .remote-only').isHidden(), !remote);
  await page.waitForFunction(() => document.body.dataset.surface === 'kiosk' || document.body.dataset.surface === 'remote');
  assert.equal(await page.locator('[data-role="lidar"]').getByRole('button', {name: 'Проверить и запустить LDS‑01'}).isHidden(), remote);
  const theme = await page.evaluate(() => ({
    body: getComputedStyle(document.body).backgroundColor,
    card: getComputedStyle(document.querySelector('.card')).backgroundColor,
    summary: getComputedStyle(document.querySelector('.summary')).backgroundColor,
    theme: document.querySelector('meta[name="theme-color"]').content,
  }));
  assert.deepEqual(theme, {body: 'rgb(0, 0, 0)', card: 'rgb(16, 16, 18)',
                          summary: 'rgb(16, 16, 18)', theme: '#000000'});
  for (const width of [320, 390, 700, 768, 1100, 1360, 1920]) {
    await page.setViewportSize({width, height: 900});
    const layout = await page.evaluate(() => {
      const issues = [];
      if (document.documentElement.scrollWidth > innerWidth) issues.push('document overflows viewport');
      for (const label of document.querySelectorAll('.summary-stat div span')) {
        if (label.scrollWidth > label.clientWidth + 1) issues.push('summary label clipped: ' + label.textContent);
      }
      for (const card of document.querySelectorAll('.card')) {
        if (card.scrollWidth > card.clientWidth + 1) issues.push('card clipped: ' + card.dataset.role);
        // У колёс нет своей формы: четыре порта назначаются общим блоком выше.
        const rowNode = card.querySelector('.hint-row');
        if (!rowNode) continue;
        const row = rowNode.getBoundingClientRect();
        for (const control of card.querySelectorAll('.hint-row input, .hint-row button')) {
          const rect = control.getBoundingClientRect();
          if (rect.left < row.left - 1 || rect.right > row.right + 1 || rect.height < 39) {
            issues.push('control clipped/too small: ' + card.dataset.role);
          }
        }
      }
      return issues;
    });
    assert.deepEqual(layout, [], `layout at ${width}px: ${layout.join(', ')}`);
  }
  // Exercise the same controls on a narrow touchscreen layout as well.
  await page.setViewportSize({width: 390, height: 900});
  assert(await page.locator('#pick-camera_FL option[value="/dev/video0"]').count());
  // Списки показывают физические порты, а не псевдонимы by-id/by-path: один
  // порт — одна строка, подходящий тип первым.
  const cameraPorts = await page.evaluate(() =>
    [...document.querySelectorAll('#pick-camera_FL option')].map(o => o.value).filter(Boolean));
  const lidarPorts = await page.evaluate(() =>
    [...document.querySelectorAll('#pick-lidar option')].map(o => o.value).filter(Boolean));
  const wheelPorts = await page.evaluate(() =>
    [...document.querySelectorAll('#wheel-pick-1 option')].map(o => o.value).filter(Boolean));
  assert(cameraPorts.includes('/dev/video0'), 'камера: свой видеопорт в списке: ' + cameraPorts);
  assert.deepEqual(cameraPorts, ['/dev/video0', '/dev/video2', '/dev/video5', '/dev/video7']);
  assert(lidarPorts.includes('/dev/ttyUSB0'), 'лидар: последовательные порты: ' + lidarPorts);
  assert(wheelPorts.includes('/dev/ttyUSB0'), 'колёса: последовательные порты: ' + wheelPorts);
  assert(new Set(wheelPorts).size === wheelPorts.length, 'колёса: без повторов: ' + wheelPorts);
  const found = (await page.locator('#ports-found').textContent()).trim();
  assert(found.startsWith('Найдено физических портов:'), 'строка о найденных портах: ' + found);
  assert(await page.locator('#ports-found-list li').count(), 'список реальных USB-портов');
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
  assert((await page.request.post(board + '/__test/swap-camera')).ok());
  await page.locator('#devices-refresh').click();
  await page.waitForFunction(() => document.querySelector('[data-role="camera_FL"] .card-status')
    ?.textContent.includes('Подсказка не совпала'), null, {timeout: 12000});
  assert(await page.locator('[data-role="camera_FL"] .confirm').isHidden());
  await page.waitForFunction(() => !document.querySelector('[data-role="camera_FL"] .preview img'),
                             null, {timeout: 12000});
  assert((await page.request.post(board + '/__test/restore-camera')).ok());
  await page.locator('#devices-refresh').click();
  await page.waitForFunction(() => document.querySelector('[data-role="camera_FL"]')?.dataset.state === 'verified',
                             null, {timeout: 12000});
  await page.locator('[data-role="camera_FL"] .clear').click();
  await page.getByRole('button', {name: 'Точно сбросить?'}).click();
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
