// Run the actual page script in a DOM stub; no browser, server or vehicle needed.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {webcrypto} = require('node:crypto');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const handlers = new Map(), elements = new Map(), requests = [], intervals = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    value: html.match(new RegExp('<input id="' + id + '"[^>]* value="([^"]*)"'))?.[1] ?? '0',
    textContent: '', classList: {toggle() {}},
    addEventListener(name, action) { handlers.set(`${id}.${name}`, action); },
  });
  return elements.get(id);
}
function fetchStub(url, options) {
  return new Promise(resolve => {
    requests.push({
      url, keepalive: options?.keepalive, body: options && JSON.parse(options.body),
      reply(phase) {
        const state = {phase, velocity_milli: [0, 0, 0], remaining_s: 0, last_frame: [true, 0, 0, 0], frames: 0};
        resolve({ok: true, json: async () => state});
      },
    });
  });
}
const context = vm.createContext({
  crypto: webcrypto, Uint8Array, console, performance,
  document: {getElementById: element, querySelectorAll: () => [],
    addEventListener: (name, action) => handlers.set(name, action)},
  window: {addEventListener: (name, action) => handlers.set(name, action)},
  setInterval: (fn, delay) => { intervals.set(delay, fn); return delay; }, clearInterval() {},
  fetch: fetchStub,
});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
vm.runInContext(`
  show({phase: 'waiting', limits: {linear_mps: 0.5, angular_radps: 4, timed_s: 3600},
        velocity_milli: [0, 0, 0], last_frame: [false, 0, 0, 0]});
`, context);
assert.equal(Number(element('linear').value), 0.4);
assert.equal(Number(element('angular').value), 3);
assert.equal(Number(element('t_vx').value), 0.4);
assert.equal(Number(element('linear').max), 0.5);
assert.equal(Number(element('angular').max), 4);
vm.runInContext(`
  show({phase: 'waiting', limits: {linear_mps: 0.25, angular_radps: 0.5, timed_s: 1.5},
        velocity_milli: [0, 0, 0], last_frame: [false, 0, 0, 0]});
`, context);
assert.equal(Number(element('linear').value), 0.25);
assert.equal(Number(element('angular').value), 0.5);
context.document.hidden = true;
handlers.get('blur')();
handlers.get('visibilitychange')();
handlers.get('pagehide')();
assert.equal(requests.length, 0, 'an observing page must leave powered standby alone');
let prevented = false;
handlers.get('keydown')({code: 'Space', target: {tagName: 'INPUT'}, preventDefault() { prevented = true; }});
assert.equal(prevented, true);
assert.equal(requests.at(-1).url, '/api/stop');
vm.runInContext("limits = {linear_mps: 0.25, angular_radps: 0.5, timed_s: 1.5}; press('forward'); stop();", context);
assert.deepEqual(requests.map(r => r.url), ['/api/stop', '/api/hold', '/api/stop']);
assert.deepEqual(requests.map(r => r.body.sequence), [1, 2, 3]);
assert.equal(new Set(requests.map(r => r.body.client_id)).size, 1);
assert.match(requests[0].body.client_id, /^[0-9a-f]{32}$/);
handlers.get('keydown')({code: 'KeyW', target: {tagName: 'INPUT'}, preventDefault() { throw Error('typing should not move'); }});
assert.equal(requests.length, 3);
handlers.get('keydown')({code: 'KeyW', repeat: true, target: {tagName: 'BODY'}, preventDefault() {}});
assert.equal(requests.length, 3, 'held-key autorepeat must not undo Space stop');
handlers.get('keyup')({code: 'KeyW'});
handlers.get('keydown')({code: 'KeyW', repeat: false, target: {tagName: 'BODY'}, preventDefault() {}});
assert.equal(requests.at(-1).url, '/api/hold', 'a fresh key press after release may start again');
handlers.get('keyup')({code: 'KeyW'});
assert.equal(requests.at(-1).url, '/api/release', 'ordinary key release should hold zero with power');

async function testFocusDuringMotion() {
  const settle = () => new Promise(resolve => setImmediate(resolve));
  const last = () => requests.at(-1);
  last().reply('holding');
  await settle();
  handlers.get('blur')();
  assert.equal(last().url, '/api/release', 'a confirmed ordinary release stays at powered zero');

  const press = () => vm.runInContext("press('forward');", context);
  press();
  handlers.get('blur')();
  assert.equal(last().url, '/api/stop', 'focus loss stops a held direction even before its reply');
  assert.equal(last().keepalive, true);
  const stopped = requests.length;
  handlers.get('visibilitychange')();
  assert.equal(requests.length, stopped);

  element('t_vx').value = '0.1';
  element('t_s').value = '1';
  handlers.get('timed.click')();
  const timed = last();
  assert.equal(timed.url, '/api/timed');
  const oldPoll = intervals.get(250)();
  const earlier = last();
  timed.reply('prepare');
  await settle();
  earlier.reply('holding');
  await oldPoll;
  handlers.get('visibilitychange')();
  assert.equal(last().url, '/api/stop', 'a delayed observation predating this timed command cannot clear ownership');

  handlers.get('timed.click')();
  last().reply('active');
  await settle();
  const newPoll = intervals.get(250)();
  last().reply('holding');
  await newPoll;
  const finished = requests.length;
  handlers.get('pagehide')();
  assert.equal(requests.length, finished, 'an acknowledged timed run that ended normally stays at powered zero');
  console.log('manual web: ordered requests, explicit stop and focus ownership passed');
}
testFocusDuringMotion().catch(error => { console.error(error); process.exitCode = 1; });
