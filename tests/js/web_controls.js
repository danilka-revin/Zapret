'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const calls = [];
const events = {};
const documentEvents = {};
const location = {href: 'http://robot.example:8080/controls?st=signed-session',
  origin: 'http://robot.example:8080', search: '?st=signed-session'};
async function nativeFetch(input, init) {
  const req = new Request(input, init);
  const body = req.method === 'GET' ? null : await req.clone().json();
  calls.push({url: req.url, headers: Object.fromEntries(req.headers), body, keepalive: req.keepalive});
  return new Response(JSON.stringify({ok: true, csrfToken: 'signed-csrf'}),
    {headers: {'Content-Type': 'application/json'}});
}
function XHR() {}
XHR.prototype.open = function () {};
const context = {URL, URLSearchParams, Headers, Request, Response, XMLHttpRequest: XHR,
  document: {addEventListener(name, fn) { documentEvents[name] = fn; }, getElementById() { return null; }},
  window: {location, fetch: nativeFetch, addEventListener(name, fn) { events[name] = fn; }}};
vm.createContext(context);
const root = path.resolve(__dirname, '../..');
for (const name of ['slam_auth.js', 'web_controls.js']) {
  vm.runInContext(fs.readFileSync(path.join(root, 'slam_gui', name), 'utf8'), context);
}
(async () => {
  assert(context.window.RCRemoteControls);
  await context.window.fetch('api/kiosk_drive', {method: 'POST',
    headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'drive'},
    body: JSON.stringify({percent: 8})});
  const drive = calls.at(-1);
  assert.equal(drive.url, 'http://robot.example:8080/api/control/drive?st=signed-session');
  assert.equal(drive.headers['x-csrf-token'], 'signed-csrf');
  assert.equal(drive.headers['x-robot-control'], 'drive');
  assert.equal(drive.body.percent, 8);
  await context.window.fetch('api/kiosk_logs?limit=400');
  const logs = new URL(calls.at(-1).url);
  assert.equal(logs.pathname, '/api/control/logs');
  assert.equal(logs.searchParams.get('limit'), '400');
  assert.equal(logs.searchParams.get('st'), 'signed-session');
  await context.window.fetch('api/kiosk_state');
  assert.equal(new URL(calls.at(-1).url).pathname, '/api/control/state');
  await context.window.fetch(new Request('http://robot.example:8080/api/kiosk_zero', {
    method: 'POST', headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'steering'},
    body: JSON.stringify({module: 'FR'})}));
  assert.equal(new URL(calls.at(-1).url).pathname, '/api/control/zero');
  assert.equal(calls.at(-1).body.module, 'FR');
  await context.window.fetch('https://unrelated.example/api', {method: 'POST', body: '{}'});
  assert(!calls.at(-1).headers['x-csrf-token']);
  events.pagehide();
  await new Promise(resolve => setTimeout(resolve, 50));
  assert.equal(calls.at(-1).body.percent, 0);
  assert(calls.at(-1).keepalive);
  assert(calls.every(call => !call.url.includes('127.0.0.1') && !call.url.includes('localhost')));
  console.log('web controls transport passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
