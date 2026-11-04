/* Chromium -> kiosk or authenticated web proxy -> fake BMS and real PTY UART.
 * All equipment is simulated; no physical printer is connected. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const [origin, kiosk, mode, port] = process.argv.slice(2);
const remote = mode === 'remote';
const browser = await chromium.launch({executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'], headless: true});
// Playwright's waitForFunction serializes test predicates with eval; allow
// those test helpers on the web page, whose production CSP forbids eval.
const context = await browser.newContext({viewport: {width: 1200, height: 900}, bypassCSP: true});
const page = await context.newPage();
// A remote tablet may have an unsynchronized wall clock. The report button
// must use server-provided sample age, not compare two machines' timestamps.
if (remote) await page.addInitScript(() => {
  const now = Date.now.bind(Date);
  Date.now = () => now() + 60 * 60 * 1000;
});
page.setDefaultTimeout(10000);
const errors = [], posts = [];
page.on('pageerror', error => errors.push(String(error)));
page.on('request', request => {
  if (request.method() === 'POST' && request.url().includes('/printer/')) {
    posts.push({url: request.url(), body: request.postDataJSON(), headers: request.headers()});
  }
});
async function bytes() { return (await (await fetch(kiosk + '/__test_printer_bytes')).json()).data; }
try {
  if (remote) {
    await page.goto(origin + '/login');
    await page.locator('input[name="username"]').fill('operator');
    await page.locator('input[name="password"]').fill('correct-horse-battery');
    await page.locator('form button[type="submit"]').click();
    await page.goto(origin + '/battery');
  } else {
    await page.goto(origin + '/kiosk');
    await page.locator('#battery-panel').click();
  }
  await page.locator('#receipt-panel').waitFor({state: 'visible'});
  await page.locator('#receipt-settings summary').click();
  // The test inventory exposes an allowed by-id name for its PTY stand-in;
  // production only enables actual USB-UART candidates from sysfs.
  await page.waitForFunction(p => [...document.querySelector('#receipt-port').options].some(o => o.value === p), port);
  await page.locator('#receipt-port').selectOption(port);
  await page.locator('#receipt-delay').fill('0');
  await page.locator('#receipt-delay').dispatchEvent('change');
  await page.locator('#receipt-confirm').check();
  await page.locator('#receipt-save').click();
  await page.waitForFunction(() => document.querySelector('#receipt-badge').dataset.tone === 'ok');
  await page.waitForFunction(() => !document.querySelector('#receipt-print').disabled);
  await Promise.all([
    page.waitForResponse(r => r.request().method() === 'GET' && /\/printer$/.test(new URL(r.url()).pathname) && r.status() === 200),
    page.locator('#receipt-refresh').click(),
  ]);
  await page.waitForFunction(p => {
    const select = document.querySelector('#receipt-port');
    return select.value === p && !select.selectedOptions[0].textContent.includes('сейчас не найден');
  }, port);
  assert.equal((await bytes()).length, 0, 'configuring/status must not print');
  await page.locator('#receipt-settings summary').click();

  await page.locator('#receipt-preview-toggle').click();
  await page.waitForFunction(() => document.querySelector('#receipt-preview-text').textContent.includes('BATTERY REPORT'));
  const medium = await page.locator('#receipt-preview-text').textContent();
  await page.locator('input[name="receipt-level"][value="short"]').check({force:true});
  await page.waitForFunction(() => document.querySelector('#receipt-preview-text').textContent.includes('short'));
  const short = await page.locator('#receipt-preview-text').textContent();
  assert.ok(short.length < medium.length);
  await page.locator('input[name="receipt-level"][value="full"]').check({force:true});
  await page.waitForFunction(() => document.querySelector('#receipt-preview-text').textContent.includes('CELL DETAILS'));
  const full = await page.locator('#receipt-preview-text').textContent();
  assert.ok(full.includes('#12') && full.length > medium.length);
  if (process.env.RC_TEST_RECEIPT_SCREENSHOT) {
    await page.locator('#receipt-panel').screenshot({path: process.env.RC_TEST_RECEIPT_SCREENSHOT});
  }
  assert.equal((await bytes()).length, 0, 'previews must not print');
  await page.locator('input[name="receipt-level"][value="short"]').check({force:true});

  await page.setViewportSize({width: 320, height: 700});
  const layout = await page.evaluate(() => {
    const card = document.querySelector('#receipt-panel');
    const rect = card.getBoundingClientRect();
    return {window: innerWidth, left: rect.left, right: rect.right,
            client: card.clientWidth, scroll: card.scrollWidth};
  });
  // The underlying kiosk dashboard has its own intentionally fixed layout;
  // check the visible receipt card rather than that hidden dashboard.
  assert.ok(layout.left >= 0 && layout.right <= layout.window && layout.scroll <= layout.client,
    'receipt UI horizontal overflow: ' + JSON.stringify(layout));
  await page.setViewportSize({width: 1200, height: 900});
  await page.locator('#receipt-print').click();
  await page.waitForFunction(() => document.querySelector('#receipt-message').textContent.includes('Данные переданы'), null, {timeout: 15000});
  const content = await bytes();
  assert.ok(content.includes('BATTERY REPORT') && content.includes('51.20'), content.slice(0, 200));
  assert.ok(!content.includes('\x1b') && !content.includes('\x1d'), 'plain profile contains no printer commands');
  const sent = posts.filter(p => p.url.includes('/printer/print'));
  assert.equal(sent.length, 1);
  assert.equal(sent[0].body.level, 'short');
  assert.equal(sent[0].headers['x-robot-control'], 'printer');
  if (remote) assert.ok(sent[0].headers['x-csrf-token']);
  await page.route('**' + (remote ? '/api/control/battery' : '/api/battery'), route => route.abort());
  await page.waitForFunction(() => document.querySelector('#receipt-print').disabled, null, {timeout: 8000});
  assert.match(await page.locator('#receipt-requirements').textContent(), /BMS/);
  assert.deepEqual(errors, []);
  console.log('Browser -> onboard USB-UART receipt: OK (' + mode + ')');
} finally {
  await browser.close();
}
