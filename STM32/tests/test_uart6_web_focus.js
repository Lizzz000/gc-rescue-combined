// Exercise the COM7 page with a second browser tab and controlled HTTP response ordering.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const script = fs.readFileSync(path.join(__dirname, '../tools/uart6_web/app.js'), 'utf8');
const idle = {connected: true, fresh: true, armed: true, mode: 'idle', state: 2, can_release: true};
const moving = mode => ({...idle, mode, state: mode === 'return' ? 5 : 3, can_release: false});

function tab() {
  const handlers = {}, elements = new Map(), posts = [];
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      dataset: {}, valueAsNumber: 0.4, querySelector: () => element('fieldset'), querySelectorAll: () => [],
      addEventListener() {}, setAttribute() {},
    });
    return elements.get(id);
  }
  function fetchStub(url, options) {
    return new Promise(resolve => {
      if (options.method !== 'POST') return;
      posts.push({
        path: url.slice(5), keepalive: Boolean(options.keepalive), body: JSON.parse(options.body),
        reply(snapshot, ok = true) { resolve({ok, json: async () => snapshot}); },
      });
    });
  }
  const context = vm.createContext({performance, fetch: fetchStub,
    document: {hidden: true, hasFocus: () => false, querySelector: () => ({content: 'test'}),
      getElementById: element, addEventListener: (name, fn) => { handlers[name] = fn; }},
    window: {setTimeout() {}, addEventListener: (name, fn) => { handlers[name] = fn; }},
  });
  vm.runInContext(script, context);
  const run = code => vm.runInContext(code, context);
  const report = (snapshot, order = '++issueOrder') => run(`applyState(${JSON.stringify(snapshot)}, ${order})`);
  report(idle);
  return {handlers, posts, last: () => posts.at(-1), run, report};
}

async function main() {
  const observer = tab();
  for (const snapshot of [idle, moving('keys'), moving('timed'), moving('return')]) {
    observer.report(snapshot);
    observer.handlers.blur();
    observer.handlers.visibilitychange();
    observer.handlers.pagehide();
  }
  observer.report(moving('keys'));
  observer.run(`setInputMode('keys'); setInputMode('timed');`);
  assert.equal(observer.posts.length, 0, 'an observing page never stops or releases someone else’s motion');
  observer.run('stopMotion()');
  assert.equal(observer.last().path, 'stop', 'explicit emergency stop is available in every tab');

  const idleOwner = tab();
  const arming = idleOwner.run(`action('arm')`);
  idleOwner.last().reply(idle);
  await arming;
  idleOwner.handlers.blur();
  assert.equal(idleOwner.last().path, 'release');
  assert.equal(idleOwner.last().keepalive, true);

  for (const name of ['run', 'return']) {
    const owner = tab();
    const started = owner.run(`action('${name}')`);
    const earlierPoll = owner.run('++issueOrder');
    owner.last().reply(moving(name === 'run' ? 'timed' : 'return'));
    await started;
    owner.report(idle, earlierPoll);
    owner.handlers.pagehide();
    assert.equal(owner.last().path, 'stop', `${name} remains owned across a delayed idle poll`);
    assert.equal(owner.last().keepalive, true);
  }

  const timed = tab();
  const start = timed.run(`action('run')`);
  timed.last().reply(moving('timed'));
  await start;
  timed.report(idle);
  timed.handlers.visibilitychange();
  assert.equal(timed.last().path, 'release', 'a completed timed run hands back the idle session');

  const keys = tab();
  keys.run(`postDrive(['w']); postDrive([]);`);
  keys.last().reply(moving('keys'));
  await new Promise(resolve => setImmediate(resolve));
  assert.deepEqual(keys.last().body.keys, []);
  keys.last().reply({...idle, state: 3});  // Zero reached the UART, the last board report still says RUNNING.
  await new Promise(resolve => setImmediate(resolve));
  keys.handlers.blur();
  assert.equal(keys.last().path, 'release', 'acknowledged key release hands back the idle session');

  const unconfirmed = tab();
  unconfirmed.run(`postDrive(['q']);`);
  unconfirmed.handlers.blur();
  assert.equal(unconfirmed.last().path, 'stop', 'a command in flight still requires focus protection');

  const refused = tab();
  const failed = refused.run(`action('run')`);
  refused.last().reply({error: 'another tab is running'}, false);
  await failed;
  refused.report(moving('timed'));
  refused.handlers.blur();
  assert.equal(refused.posts.length, 1, 'a rejected request does not own another tab’s motion');
  console.log('uart6 web: local motion ownership, idle handoff and emergency focus stop passed');
}
main().catch(error => { console.error(error); process.exitCode = 1; });
