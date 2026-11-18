/* Real Chromium -> operator web panel -> signed offline Git update (temp repos). */
import assert from 'node:assert/strict';
import {writeFileSync} from 'node:fs';
import {pathToFileURL} from 'node:url';

const {chromium} = await import(pathToFileURL(process.env.RC_TEST_PLAYWRIGHT_CORE).href);
const [origin, packagePath, estopFile, base, revision] = process.argv.slice(2);
const browser = await chromium.launch({
  executablePath: process.env.RC_TEST_CHROMIUM,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'],
  headless: true,
});
const context = await browser.newContext();
const page = await context.newPage();
page.setDefaultTimeout(8000);

const phase = (text) => page.locator('#rc-update-phase').filter({hasText: text}).waitFor();
const run = page.locator('#rc-update-run');
const file = page.locator('#rc-update-file');
const send = page.locator('#rc-update-send');
const version = () => page.evaluate(async () => (await fetch('/api/update/version')).json());
const upload = async (status) => {
  const response = page.waitForResponse((r) => r.url().includes('/api/update/upload?st='));
  await send.click();
  assert.equal((await response).status(), status);
};

try {
  await page.goto(origin + '/login?next=/panel');
  await page.locator('[name=username]').fill('operator');
  await page.locator('[name=password]').fill('correct-horse-battery');
  await page.locator('button[type=submit]').click();
  await page.waitForURL(/\/panel\?.*st=/);
  await context.clearCookies(); // Exercise the URL-session wrapper on real XHR/fetch.
  await page.locator('#rc-update-launch').click();
  await page.locator('#rc-update-source').selectOption('upload');
  await phase('Загрузите файл обновления');
  assert.ok(await run.isDisabled());

  await file.setInputFiles({name: 'Zapret-main.zip', mimeType: 'application/zip', buffer: Buffer.from('not an update')});
  await upload(422);
  await phase('Файл не принят');
  assert.ok(await run.isDisabled());

  await file.setInputFiles(packagePath);
  await upload(201);
  await phase('Нужен E-STOP');
  assert.equal(await page.locator('#rc-update-to').textContent(), revision.slice(0, 7));
  assert.equal((await version()).sha, base);
  assert.ok(await run.isDisabled());

  // Simulate latching the stop on the robot; no real hardware is involved.
  writeFileSync(estopFile, JSON.stringify({latched: true}));
  await page.locator('#rc-update-check').click();
  await phase('Доступна новая версия');
  assert.ok(await run.isEnabled());
  await file.setInputFiles(packagePath);
  assert.ok(await run.isDisabled(), 'Do not install an older pending offer for an unsent selection');
  await file.setInputFiles([]);
  assert.ok(await run.isEnabled());

  const dismissed = page.waitForEvent('dialog');
  const clicked = run.click();
  const first = await dismissed;
  assert.match(first.message(), new RegExp(revision.slice(0, 7)));
  await first.dismiss();
  await clicked;
  assert.equal((await version()).sha, base);

  page.once('dialog', (dialog) => dialog.accept());
  const started = page.waitForResponse((r) => r.url().includes('/api/update/run?st='));
  await run.click();
  const response = await started;
  assert.equal(response.status(), 202);
  assert.deepEqual(response.request().postDataJSON(), {source: 'upload', expectedSha: revision});
  await phase('Обновление установлено.');
  assert.equal((await version()).sha, revision);
} finally {
  await browser.close();
}
