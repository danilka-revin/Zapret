/* Actual onboard and authenticated web pages; telemetry and commands are
 * intercepted in the browser. No USB, robot, or physical motion is involved. */
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {join} from 'node:path';

const modules = process.env.ZAPRET_BROWSER_NODE_MODULES;
const [onboard, remote] = process.argv.slice(2);
if (!modules || !onboard || !remote) throw new Error('browser modules and two URLs required');
const require = createRequire(import.meta.url);
const {chromium} = require(join(modules, 'playwright-core'));
const chrome = (await import(join(modules, '@sparticuz/chromium/build/index.js'))).default;
const {inflate} = await import(join(modules, '@sparticuz/chromium/build/lambdafs.js'));
await inflate(join(modules, '@sparticuz/chromium/bin/al2023.tar.br'));
process.env.LD_LIBRARY_PATH = '/tmp/al2023/lib' + (process.env.LD_LIBRARY_PATH ? ':' + process.env.LD_LIBRARY_PATH : '');
const browser = await chromium.launch({executablePath: await chrome.executablePath(), headless: true,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage']});
const ids = ['FL', 'FR', 'RL', 'RR'];
const allErrors = [];
const wait = page => page.waitForTimeout(350);

async function checkLayout(page) {
  for (const width of [1440, 1000, 700, 390, 320]) {
    await page.setViewportSize({width, height: 900});
    await wait(page);
    const result = await page.evaluate(() => {
      const section = document.querySelector('#steer-calibration');
      const selectors = ['.calibration-safety', '.calibration-zero', '.pot-mode', '.pot-hold'];
      const rect = el => { const r = el.getBoundingClientRect(); return {x:r.x, y:r.y, width:r.width, height:r.height, right:r.right, bottom:r.bottom}; };
      const items = selectors.map(selector => {
        const el = section.querySelector(selector);
        return {selector, ...rect(el), client: el.clientWidth, scroll: el.scrollWidth};
      });
      const controls = [...section.querySelectorAll('button, select')].map(el => ({
        ...rect(el), parent: rect(el.closest('.calibration-card')), minHeight: getComputedStyle(el).minHeight
      }));
      return {section: rect(section), items, controls};
    });
    assert(result.section.x >= -1 && result.section.right <= width + 1, `section overflow at ${width}`);
    for (const item of result.items) {
      assert(item.x >= result.section.x - 1 && item.right <= result.section.right + 1,
        `${item.selector} escaped panel at ${width}`);
      assert(item.scroll <= item.client + 1, `${item.selector} has horizontal overflow at ${width}`);
    }
    for (const control of result.controls) {
      assert(control.x >= control.parent.x - 1 && control.right <= control.parent.right + 1,
        `control escaped card at ${width}`);
      assert(parseFloat(control.minHeight) >= 44, `control target too small at ${width}`);
    }
    const [safety, zero, mode, hold] = result.items;
    assert(safety.bottom <= zero.y + 1, `warning must precede controls at ${width}`);
    if (width > 1100) {
      assert(Math.abs(zero.y - mode.y) < 1 && Math.abs(mode.y - hold.y) < 1, 'three-column desktop');
      assert(zero.right <= mode.x && mode.right <= hold.x, 'desktop cards overlap');
    } else if (width > 700) {
      assert(Math.abs(zero.y - mode.y) < 1 && mode.bottom <= hold.y + 1, 'two-column tablet');
    } else {
      assert(zero.bottom <= mode.y + 1 && mode.bottom <= hold.y + 1, 'stacked mobile cards');
      await page.locator('.pot-hold__switch').scrollIntoViewIfNeeded();
      const box = await page.locator('.pot-hold__switch').boundingBox();
      assert(box && box.x >= 0 && box.x + box.width <= width && box.y >= 0 && box.y + box.height <= 900,
        'mobile switch must be reachable by scrolling');
    }
  }
  await page.setViewportSize({width: 1280, height: 900});
}

async function exerciseSurface(origin, isRemote) {
  const context = await browser.newContext({viewport: {width: 1280, height: 900}});
  const page = await context.newPage();
  page.on('pageerror', error => allErrors.push(`${isRemote ? 'web' : 'kiosk'}: ${error}`));
  const calibrationRequests = [];
  page.on('request', req => {
    const path = new URL(req.url()).pathname;
    if (req.method() === 'POST' && /\/(kiosk_(calibrate|full_zero|pot_mode|pot_hold)|control\/(calibrate|full_zero|pot_mode|pot_hold))$/.test(path)) {
      calibrationRequests.push(path);
    }
  });
  try {
    if (isRemote) {
      await page.goto(origin + '/login', {waitUntil: 'domcontentloaded'});
      await page.locator('input[name="username"]').fill('operator');
      await page.locator('input[name="password"]').fill('correct-horse-battery');
      await Promise.all([page.waitForURL(url => url.pathname !== '/login'),
        page.locator('button[type="submit"]').click()]);
    }
    const path = isRemote ? '/controls?open=calibration' : '/kiosk?open=calibration';
    await page.goto(origin + path, {waitUntil: 'domcontentloaded'});
    await page.waitForSelector('#swipe-motors-view:not([hidden])');
    await page.waitForSelector('.pot-hold__switch');
    await page.waitForFunction(() => {
      const y = document.querySelector('#steer-calibration').getBoundingClientRect().top;
      return y >= 0 && y < window.innerHeight / 2;
    }, null, {timeout: 5000});
    await page.waitForFunction(() => document.querySelector('#steer-calib-status')?.textContent.includes('недоступно'));
    assert.equal(calibrationRequests.length, 0, 'deep link must never start calibration');
    assert.equal(await page.locator('.calibration-card').count(), 3);
    assert(await page.locator('#steer-calib-all').isDisabled());
    assert(await page.locator('.pot-mode select').isDisabled());
    assert(await page.locator('.pot-hold__switch').isDisabled());
    assert.match(await page.locator('.pot-hold__switch').textContent(), /нет подтверждения/);
    assert.equal(await page.locator('.pot-hold__switch').getAttribute('aria-checked'), null);
    assert.match(await page.locator('.calibration-safety').textContent(), /125°.*250°.*±60°/s);
    await checkLayout(page);

    // The dialog remains reachable with the keyboard; without boards its
    // movement buttons stay disabled. The server has the real no-source mode.
    await page.locator('#full-zero-open').focus();
    assert.notEqual(await page.locator('#full-zero-open').evaluate(el => getComputedStyle(el).outlineStyle), 'none');
    await page.keyboard.press('Enter');
    await page.waitForSelector('#full-zero-modal:not([hidden])');
    await page.waitForFunction(() => document.querySelector('#full-zero-message')?.textContent.includes('не поддерживает'));
    assert(await page.locator('#full-zero-calibrate').isDisabled());
    assert(await page.locator('#full-zero-save').isDisabled());
    await page.locator('#full-zero-close').click();
    assert.equal(await page.locator('#full-zero-modal').isHidden(), true);
    assert.equal(await page.evaluate(() => document.activeElement.id), 'full-zero-open');

    // Keep both existing battery routes intact; a calibration query cannot
    // switch the explicitly requested remote battery page to motor controls.
    if (isRemote) {
      await page.goto(origin + '/battery?open=calibration', {waitUntil: 'domcontentloaded'});
      await page.waitForSelector('#swipe-battery-view:not([hidden])');
      assert.equal(await page.locator('#swipe-motors-view').isHidden(), true);
      assert.equal(await page.locator('#bms-setup').isHidden(), true);
    } else {
      await page.goto(origin + '/kiosk?open=bms', {waitUntil: 'domcontentloaded'});
      await page.waitForSelector('#swipe-battery-view:not([hidden])');
      await page.waitForSelector('#bms-setup:not([hidden])');
    }

    const base = await (await page.request.get(onboard + '/api/kiosk_state')).json();
    let state = {...base.data, linkOk: true,
      steeringControl: {...base.data.steeringControl, available: true, calibration: true, source: 'master'},
      estop: {latched: false, unknown: false}, neutral: {locked: false},
      motors: ids.map(id => ({id, online: true, homed: true, homeSource: 'opto',
        potMode: true, pot: true, potHold: false, firmware: 24, moving: false,
        calibrating: false, cycling: false, drivePercent: 0, angle: 0, targetAngle: 0, opto: false})),
      drive: {...base.data.drive, active: false, commandedPercent: 0}};
    let loseTelemetry = false;
    const commands = [], brakeRequests = [];
    await page.route(/\/api\/(kiosk_neutral|control\/neutral)(\?.*)?$/, route => {
      const req = route.request();
      brakeRequests.push({path: new URL(req.url()).pathname, body: req.postDataJSON(), headers: req.headers()});
      state.neutral = {locked: true, phase: 'restoring'};
      return route.fulfill({status: 200, contentType: 'application/json',
        body: JSON.stringify({ok: true, phase: 'restoring'})});
    });
    await page.route(/\/api\/(kiosk_state|control\/state)(\?.*)?$/, route => loseTelemetry
      ? route.abort('failed')
      : route.fulfill({status: 200, contentType: 'application/json', body: JSON.stringify({ok: true, data: state})}));
    // All test control actions terminate here, never at either Flask server.
    await page.route(/\/api\/(kiosk_(calibrate|pot_mode|pot_hold)|control\/(calibrate|pot_mode|pot_hold))(\?.*)?$/, route => {
      const req = route.request();
      commands.push({path: new URL(req.url()).pathname, body: req.postDataJSON(), headers: req.headers()});
      return route.fulfill({status: 200, contentType: 'application/json', body: JSON.stringify({ok: true, queued: true, offline: []})});
    });
    await page.goto(origin + path, {waitUntil: 'domcontentloaded'});
    await page.waitForFunction(() => document.querySelector('.pot-mode select')?.value === 'on');
    await page.waitForFunction(() => document.querySelector('.pot-hold__switch')?.getAttribute('aria-checked') === 'false');
    const select = page.locator('.pot-mode select');
    const potButton = page.locator('.pot-mode button');
    const hold = page.locator('.pot-hold__switch');
    assert.match(await page.locator('.pot-mode [role="status"]').textContent(), /датчики откалиброваны/);
    assert.doesNotMatch(await page.locator('.pot-mode [role="status"]').textContent(), /доводка включена/);
    assert.equal(await hold.getAttribute('role'), 'switch');
    assert.equal(await hold.getAttribute('aria-checked'), 'false');
    // A calibrated fw24 still reports only steps, never a fabricated A0 value.
    await page.waitForFunction(() => document.querySelector('#steer-angle-source-FR')?.textContent.includes('fw26'));
    assert.equal(await page.locator('#steer-current-FR').textContent(), '0.0°');
    await page.waitForFunction(() => document.querySelector('#angle-source-FR')?.textContent.includes('шаги'));
    await page.evaluate(() => {
      window.testLiveUpdates = 0;
      window.testLiveObserver = new MutationObserver(records => { window.testLiveUpdates += records.length; });
      for (const selector of ['#steer-calib-status', '.pot-mode [role="status"]', '.pot-hold__status']) {
        window.testLiveObserver.observe(document.querySelector(selector), {subtree: true, childList: true, characterData: true});
      }
    });
    await page.waitForTimeout(800); // several unchanged telemetry polls
    assert.equal(await page.evaluate(() => { window.testLiveObserver.disconnect(); return window.testLiveUpdates; }), 0,
      'unchanged live regions must not repeatedly announce their text');

    // A missing board is disclosed even though partial optocalibration is
    // allowed by the existing server. Hold and sensor mode fail closed.
    state.motors[3].online = false;
    await page.waitForFunction(() => document.querySelector('#steer-calib-status')?.textContent.includes('3/4'));
    assert(await select.isDisabled());
    assert(await hold.isDisabled());
    assert.equal(await hold.getAttribute('aria-checked'), null);
    state.motors[3].online = true;
    await page.waitForFunction(() => document.querySelector('.pot-hold__switch')?.getAttribute('aria-checked') === 'false');

    state.estop = {unknown: true};
    await page.waitForFunction(() => document.querySelector('#steer-calib-status')?.textContent.includes('E-STOP'));
    assert(await page.locator('#steer-calib-all').isDisabled());
    assert(await select.isDisabled() && await hold.isDisabled());
    state.estop = {latched: false, unknown: false};
    assert.equal(await page.locator('#neutral-panel, #neutral-toggle').count(), 0);
    assert(await page.locator('#brake-recovery').isHidden());
    state.neutral = {locked: true, phase: 'fault', reason: 'neutral_unconfirmed'};
    await page.waitForFunction(() => document.querySelector('#steer-calib-status')?.textContent.includes('тормоз'));
    assert(await page.locator('#steer-calib-all').isDisabled());
    assert(await select.isDisabled() && await hold.isDisabled());
    const recovery = page.locator('#brake-recovery'), restore = page.locator('#brake-restore');
    await recovery.waitFor({state: 'visible'});
    assert(await page.locator('#keyboard-enable').isDisabled());
    await restore.click();
    await page.waitForFunction(() => document.querySelector('#brake-recovery-status')?.textContent.includes('ждём подтверждение'));
    assert.deepEqual(brakeRequests[0].body, {action: 'restore'});
    assert.equal(brakeRequests[0].headers['x-robot-control'], 'drive');
    if (isRemote) {
      assert.equal(brakeRequests[0].path, '/api/control/neutral');
      assert(brakeRequests[0].headers['x-csrf-token']);
    } else assert.equal(brakeRequests[0].path, '/api/kiosk_neutral');
    assert(await restore.isDisabled());
    // Let at least one real UI poll observe the restoring phase before the
    // simulated controller reports a timeout.
    await page.waitForTimeout(650);

    // A failed or incomplete ACK remains locked, including when telemetry is lost.
    state.neutral = {locked: true, phase: 'fault', reason: 'brake_unconfirmed'};
    await page.waitForFunction(() => document.querySelector('#brake-recovery-status')?.textContent.includes('Не все платы'));
    loseTelemetry = true;
    await page.waitForFunction(() => document.querySelector('#brake-recovery-status')?.textContent.includes('Нет связи'));
    assert(await recovery.isVisible() && await restore.isDisabled());
    loseTelemetry = false;
    await page.waitForFunction(() => !document.querySelector('#brake-restore').disabled);
    await restore.click();
    await page.waitForFunction(() => document.querySelector('#brake-restore').disabled);
    assert.deepEqual(brakeRequests.map(item => item.body), [{action: 'restore'}, {action: 'restore'}]);
    state.neutral = {locked: false, phase: 'idle'};
    await page.waitForFunction(() => document.querySelector('#brake-recovery').hidden);
    await page.waitForFunction(() => !document.querySelector('.pot-hold__switch').disabled);

    // Telemetry loss invalidates the cards immediately rather than leaving a
    // previously confirmed ON/OFF state on the screen.
    loseTelemetry = true;
    await page.waitForFunction(() => document.querySelector('.pot-mode select')?.value === 'unknown', null, {timeout: 5000});
    assert(await select.isDisabled());
    assert(await hold.isDisabled());
    assert.equal(await hold.getAttribute('role'), null);
    assert.equal(await hold.getAttribute('aria-checked'), null);
    assert(await page.locator('#steer-calib-all').isDisabled());
    assert.match(await page.locator('#steer-calib-status').textContent(), /нет связи с сервером/i);
    loseTelemetry = false;
    await page.waitForFunction(() => document.querySelector('.pot-mode select')?.value === 'on');

    // Changing a mode sends exactly the original payload and retains the old
    // selection until all four boards report the new one.
    await select.selectOption('off');
    await page.waitForFunction(() => document.querySelector('.pot-mode [role="status"]')?.textContent.includes('Ожидание подтверждения'));
    await page.waitForFunction(() => document.querySelector('.pot-mode select')?.value === 'on');
    assert(await select.isDisabled());
    assert.deepEqual(commands.at(-1).body, {action: 'off'});
    state.motors.forEach(motor => { motor.potMode = false; });
    await page.waitForFunction(() => document.querySelector('.pot-mode select')?.value === 'off' && !document.querySelector('.pot-mode select').disabled);
    assert(await potButton.isDisabled());
    await select.selectOption('on');
    await page.waitForFunction(() => document.querySelector('.pot-mode [role="status"]')?.textContent.includes('Ожидание подтверждения'));
    state.motors.forEach(motor => { motor.potMode = true; });
    await page.waitForFunction(() => !document.querySelector('.pot-mode button').disabled);

    // The ±60° prompt remains mandatory.
    const beforePotCalibrate = commands.length;
    page.once('dialog', dialog => { assert.match(dialog.message(), /−60° и \+60°/); void dialog.dismiss(); });
    await potButton.click();
    assert.equal(commands.length, beforePotCalibrate);
    page.once('dialog', dialog => { assert.match(dialog.message(), /−60° и \+60°/); void dialog.accept(); });
    await potButton.click();
    await page.waitForTimeout(100);
    assert.deepEqual(commands.at(-1).body, {action: 'calibrate'});

    // Enabling hold asks for confirmation. The button shows OFF until board
    // telemetry confirms ON; turning it OFF remains possible while moving.
    const beforeHold = commands.length;
    page.once('dialog', dialog => { assert.match(dialog.message(), /15°/); void dialog.dismiss(); });
    await hold.click();
    assert.equal(commands.length, beforeHold);
    page.once('dialog', dialog => { assert.match(dialog.message(), /15°/); void dialog.accept(); });
    await hold.focus();
    await page.keyboard.press('Enter');
    await page.waitForFunction(() => document.querySelector('.pot-hold__status')?.textContent.includes('Ждём подтверждения'));
    assert.deepEqual(commands.at(-1).body, {action: 'on'});
    assert.equal(await hold.getAttribute('aria-checked'), 'false');
    state.motors.forEach(motor => { motor.potHold = true; motor.moving = true; });
    await page.waitForFunction(() => document.querySelector('.pot-hold__switch')?.getAttribute('aria-checked') === 'true');
    assert.equal(await hold.isDisabled(), false);
    assert.match(await page.locator('.pot-hold__status').textContent(), /подтягивает/);
    await hold.focus();
    await page.keyboard.press('Space');
    await page.waitForFunction(() => document.querySelector('.pot-hold__status')?.textContent.includes('Ждём подтверждения'));
    assert.deepEqual(commands.at(-1).body, {action: 'off'});
    assert.equal(await hold.getAttribute('aria-checked'), 'true');
    state.motors.forEach(motor => { motor.potHold = false; motor.moving = false; });
    await page.waitForFunction(() => document.querySelector('.pot-hold__switch')?.getAttribute('aria-checked') === 'false' && !document.querySelector('.pot-hold__switch').disabled);

    // Global optical calibration still warns and sends ALL, with no implied
    // telemetry confirmation merely because the server accepted a command.
    const all = page.locator('#steer-calib-all');
    await page.waitForFunction(() => !document.querySelector('#steer-calib-all').disabled);
    const beforeAll = commands.length;
    page.once('dialog', dialog => { assert.match(dialog.message(), /125°.*250°/s); void dialog.dismiss(); });
    await all.click();
    assert.equal(commands.length, beforeAll);
    page.once('dialog', dialog => { assert.match(dialog.message(), /125°.*250°/s); void dialog.accept(); });
    await all.click();
    await page.waitForFunction(() => document.querySelector('#steer-calib-status')?.textContent.includes('ожидаем телеметрию'));
    assert.deepEqual(commands.at(-1).body, {module: 'ALL'});
    assert(await all.isDisabled());
    for (const command of commands) {
      assert.equal(command.headers['x-robot-control'], 'steering');
      if (isRemote) {
        assert(command.path.startsWith('/api/control/'), 'web command must use same-origin proxy');
        assert(command.headers['x-csrf-token'], 'web command must carry CSRF');
      } else assert(command.path.startsWith('/api/kiosk_'));
    }
  } finally {
    await context.close();
  }
}

try {
  await exerciseSurface(onboard, false);
  await exerciseSurface(remote, true);
  assert.deepEqual(allErrors, []);
  console.log('calibration layout and simulated controls passed');
} catch (error) {
  console.error(error);
  process.exitCode = 1;
} finally {
  try { await Promise.race([browser.close(), new Promise(resolve => setTimeout(resolve, 5000))]); }
  catch (error) { console.error(error); process.exitCode = 1; }
}
process.exit(process.exitCode || 0);
