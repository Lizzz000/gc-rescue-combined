// Run the actual page script in a DOM stub; no browser, server or vehicle needed.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {webcrypto} = require('node:crypto');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const handlers = new Map(), elements = new Map(), requests = [];
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    value: id === 'linear' ? '0.25' : id === 'angular' ? '0.5' : '0',
    textContent: '', classList: {toggle() {}}, addEventListener() {},
  });
  return elements.get(id);
}
const context = vm.createContext({
  crypto: webcrypto, Uint8Array, console,
  document: {getElementById: element, querySelectorAll: () => [],
    addEventListener: (name, action) => handlers.set(name, action)},
  window: {addEventListener() {}},
  setInterval: () => 1, clearInterval() {},
  fetch: (url, options) => { requests.push({url, body: JSON.parse(options.body)}); return new Promise(() => {}); },
});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
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
console.log('manual web: focused-input stop and ordered motion requests passed');
