import test from 'node:test';
import assert from 'node:assert/strict';
import { Session, WORKER_FAILED, isExitUnwind, isWorkerFailure, isWindowWorkerFailure } from '../ui/src/lib/session.js';
import { errorText } from '../ui/src/lib/logic.mjs';

function fakeTimers() {
  const q = [];
  return {
    setTimeout: (fn, ms) => { const h = { fn, ms, done: false }; q.push(h); return h; },
    clearTimeout: (h) => { if (h) h.done = true; },
    fire(ms) { for (const h of q) if (!h.done && h.ms === ms) { h.done = true; h.fn(); } },
    pending: (ms) => q.some((h) => !h.done && h.ms === ms),
  };
}

function fakeModuleFactory({ exitOnStop = true, failStart = false } = {}) {
  const made = [];
  const create = async (opts) => {
    if (failStart) throw new Error('boom');
    const files = {};
    const m = {
      opts, files, recCalls: [], localCalls: [], utf8: [], freed: [], idrCalls: 0,
      FS: { writeFile: (p, t) => { files[p] = t; } },
      _webgs_stop() { m.stopped = true; if (exitOnStop) queueMicrotask(() => opts.onExit(0)); },
      _webgs_set_rec(on) { m.recCalls.push(on); },
      stringToNewUTF8(str) { m.utf8.push(str); return 42; },
      _free(p) { m.freed.push(p); },
      _webgs_set_local_rec(on, p) { m.localCalls.push([on, p]); },
      _webgs_request_idr() { m.idrCalls++; },
    };
    for (const f of opts.preRun || []) f(m);
    made.push(m);
    return m;
  };
  return { create, made };
}

const mk = (over = {}) => {
  const t = fakeTimers();
  const f = fakeModuleFactory(over.factory);
  let reloads = 0;
  const s = new Session({
    createModule: f.create, requestDevice: over.requestDevice || (async () => {}),
    onAu: () => {}, onStats: () => {}, onRecClosed: over.onRecClosed || (() => {}), reload: () => { reloads++; }, timers: t,
  });
  const states = [];
  s.subscribe((v) => states.push(v.state));
  return { s, t, f, states, reloads: () => reloads };
};
const tick = () => new Promise((r) => setTimeout(r, 0));

test('gs connect passes args + overlay, disconnect tears down to idle', async () => {
  const { s, f, states } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '[link]\nmax_mcs = 3\n' });
  assert.equal(states.at(-1), 'live');
  const m = f.made[0];
  assert.deepEqual(m.opts.arguments,
    ['live', '--mode', 'gs', '--ch', '136', '--w', '40', '--overlay', '/overlay.toml']);
  assert.equal(m.files['/overlay.toml'], '[link]\nmax_mcs = 3\n');
  await s.disconnect();
  await tick();
  assert.ok(m.stopped);
  assert.deepEqual(states.slice(-2), ['stopping', 'idle']);
});

test('spotter sends no overlay', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'spotter', ch: 136, w: 40, overlayToml: 'x' });
  assert.deepEqual(f.made[0].opts.arguments, ['live', '--mode', 'spotter', '--ch', '136', '--w', '40']);
  assert.deepEqual(f.made[0].files, {});
});

test('device chooser cancelled -> idle with a notice, no module', async () => {
  const { s, f, states } = mk({ requestDevice: async () => { throw Object.assign(new Error('cancel'), { name: 'NotFoundError' }); } });
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  assert.equal(states.at(-1), 'idle');
  assert.match(s.snapshot.notice, /No device selected/);
  assert.equal(f.made.length, 0);
});

test('module start failure -> error', async () => {
  const { s } = mk({ factory: { failStart: true } });
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  assert.equal(s.snapshot.state, 'error');
  assert.match(s.snapshot.error, /boom/);
});

test('card_lost_goes_to_error_and_reconnects', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  f.made[0].opts.onError('card lost');
  f.made[0].opts.onExit(1);
  assert.equal(s.snapshot.state, 'error');
  assert.match(s.snapshot.error, /Card lost/);
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  assert.equal(s.snapshot.state, 'live');
  assert.equal(f.made.length, 2, 'a fresh module instance');
});

test('connect_is_noop_unless_idle_or_error', async () => {
  const { s, f } = mk();
  const p1 = s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  const p2 = s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  await Promise.all([p1, p2]);
  assert.equal(f.made.length, 1);
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  assert.equal(f.made.length, 1);
});

test('disconnect_is_noop_unless_live', async () => {
  const { s, states } = mk();
  await s.disconnect();
  assert.deepEqual(states, ['idle']);
});

test('stop timeout falls back to reload', async () => {
  const { s, t, reloads } = mk({ factory: { exitOnStop: false } });
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  await s.disconnect();
  assert.equal(s.snapshot.state, 'stopping');
  assert.ok(t.pending(3000));
  t.fire(3000);
  assert.equal(reloads(), 1);
});

test('rec off before stop: waits for the drone to report off (or 1 s)', async () => {
  const { s, f, t } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  const m = f.made[0];
  s.setRec(true);
  m.opts.onStats(JSON.stringify({ rec_state: 1, rec_err: 0 }));
  const done = s.disconnect();
  await tick();
  assert.deepEqual(m.recCalls, [1, 0]);
  assert.ok(!m.stopped, 'still waiting for rec off');
  m.opts.onStats(JSON.stringify({ rec_state: 0, rec_err: 0 }));
  await done; await tick();
  assert.ok(m.stopped);
  assert.equal(s.snapshot.state, 'idle');
  assert.ok(!t.pending(1000));
});

test('rec off wait gives up after 1 s', async () => {
  const { s, f, t } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  const m = f.made[0];
  s.setRec(true);
  m.opts.onStats(JSON.stringify({ rec_state: 1, rec_err: 0 }));
  const done = s.disconnect();
  await tick();
  t.fire(1000);
  await done; await tick();
  assert.ok(m.stopped);
});

test('setRec_noop_in_spotter and when not live', async () => {
  const { s, f } = mk();
  s.setRec(true);   // idle: nothing to call
  await s.connect({ mode: 'spotter', ch: 136, w: 40, overlayToml: '' });
  s.setRec(true);
  assert.deepEqual(f.made[0].recCalls, []);
  assert.equal(s.snapshot.recWish, false);
});

test('requestIdr calls the core in live GS mode only', async () => {
  const { s, f } = mk();
  s.requestIdr();                               // idle: nothing to call
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  s.requestIdr();
  s.requestIdr();
  assert.equal(f.made[0].idrCalls, 2);
});

test('requestIdr is a no-op in spotter', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'spotter', ch: 136, w: 40, overlayToml: '' });
  s.requestIdr();
  assert.equal(f.made[0].idrCalls, 0);
});

// Final review finding 2: the caller (video.js's IdrRequester) needs to
// tell a real send from one swallowed by state, so it doesn't count a
// swallowed request against its own retry pacing.
test('requestIdr returns true in live GS, false when idle or spotter', async () => {
  const { s, f } = mk();
  assert.equal(s.requestIdr(), false);   // idle: nothing to call
  await s.connect({ mode: 'spotter', ch: 136, w: 40, overlayToml: '' });
  assert.equal(s.requestIdr(), false);   // spotter: no send path
  assert.equal(f.made[0].idrCalls, 0);
  await s.disconnect();
  await tick();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  assert.equal(s.requestIdr(), true);
  assert.equal(f.made[1].idrCalls, 1);
});

test('external failure (worker) -> error', async () => {
  const { s } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  s.fail('Page worker failed');
  assert.equal(s.snapshot.state, 'error');
  assert.equal(s.snapshot.error, 'Page worker failed');
});

// Ruling (Task 3 review): onExit and onError can arrive in either order.
// The session must end in 'error' showing the MAPPED onError text
// regardless of which callback the runtime fires first.
test('exit_before_error_still_shows_mapped_error', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  const m = f.made[0];
  const line = 'bad config: config: /maburgs.toml:88: link.x: why';
  m.opts.onExit(2);
  m.opts.onError(line);
  assert.equal(s.snapshot.state, 'error');
  assert.equal(s.snapshot.error, errorText(line));
});

test('exit_before_error_during_connecting_still_shows_mapped_error, no live reached', async () => {
  const line = 'bad config: config: /maburgs.toml:88: link.x: why';
  const create = async (opts) => {
    opts.onExit(2);
    opts.onError(line);
    return { FS: { writeFile() {} }, _webgs_stop() {}, _webgs_set_rec() {} };
  };
  const t = fakeTimers();
  const states = [];
  const s = new Session({
    createModule: create, requestDevice: async () => {},
    onAu: () => {}, onStats: () => {}, reload: () => {}, timers: t,
  });
  s.subscribe((v) => states.push(v.state));
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  assert.equal(s.snapshot.state, 'error');
  assert.equal(s.snapshot.error, errorText(line));
  assert.ok(!states.includes('live'));
});

// A late onError from an already-stopped module must not flip a clean
// idle back into error.
test('late_onError_after_clean_stop_does_not_flip_idle_to_error', async () => {
  const { s, f, states } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  const m = f.made[0];
  await s.disconnect();
  await tick();
  assert.equal(s.snapshot.state, 'idle');
  m.opts.onError('card lost');
  assert.equal(s.snapshot.state, 'idle');
  assert.ok(!states.includes('error'));
});

test('device chooser NotFoundError -> idle with the chooser notice', async () => {
  const err = Object.assign(new Error('No device selected.'), { name: 'NotFoundError' });
  const { s, f } = mk({ requestDevice: async () => { throw err; } });
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  assert.equal(s.snapshot.state, 'idle');
  assert.equal(s.snapshot.notice, 'No device selected — press Connect to try again.');
  assert.equal(f.made.length, 0);
});

test('requestDevice non-chooser failure -> error with its message', async () => {
  const { s, f } = mk({ requestDevice: async () => { throw new Error('WebUSB unavailable'); } });
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  assert.equal(s.snapshot.state, 'error');
  assert.equal(s.snapshot.error, 'WebUSB unavailable');
  assert.equal(s.snapshot.notice, null);
  assert.equal(f.made.length, 0);
});

test('fail() on a live session stops the running module', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  const m = f.made[0];
  s.fail('Page worker failed');
  assert.ok(m.stopped, '_webgs_stop called');
  await tick();
  assert.equal(s.snapshot.state, 'error');
  assert.equal(s.snapshot.error, 'Page worker failed');
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  assert.equal(s.snapshot.state, 'live');
});

test('fail() while connecting stops the module once it resolves', async () => {
  let release;
  const gate = new Promise((r) => { release = r; });
  const f = fakeModuleFactory();
  const s = new Session({
    createModule: async (opts) => { await gate; return f.create(opts); },
    requestDevice: async () => {}, onAu: () => {}, onStats: () => {}, reload: () => {}, timers: fakeTimers(),
  });
  const p = s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  await tick();
  s.fail('Page worker failed');
  release();
  await p;
  assert.ok(f.made[0].stopped, '_webgs_stop called on the orphaned module');
  assert.equal(s.snapshot.state, 'error');
});

test('fail() tolerates a throwing _webgs_stop', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  f.made[0]._webgs_stop = () => { throw new Error('runtime gone'); };
  s.fail('x');
  assert.equal(s.snapshot.state, 'error');
});

test('onOsd reaches the page only from the current module', async () => {
  const got = [];
  const f = fakeModuleFactory();
  const s = new Session({
    createModule: f.create, requestDevice: async () => {}, onAu: () => {}, onStats: () => {},
    onOsd: (...a) => got.push(a), reload: () => {}, timers: fakeTimers(),
  });
  await s.connect({ mode: 'spotter', ch: 136, w: 40 });
  const m1 = f.made[0];
  m1.opts.onOsd(18, 50, new Uint16Array(900));
  assert.equal(got.length, 1);
  await s.disconnect();
  await tick();
  await s.connect({ mode: 'spotter', ch: 136, w: 40 });
  m1.opts.onOsd(18, 50, new Uint16Array(900));   // straggler from the old module
  assert.equal(got.length, 1);
  f.made[1].opts.onOsd(16, 30, new Uint16Array(480));
  assert.equal(got.length, 2);
  assert.deepEqual(got[1].slice(0, 2), [16, 30]);
});

test('setLocalRec passes the name through the heap and frees it', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'spotter', ch: 136, w: 40, overlayToml: '' });
  const m = f.made[0];
  s.setLocalRec('mabur-20260928-070509.mp4');
  assert.deepEqual(m.utf8, ['mabur-20260928-070509.mp4']);
  assert.deepEqual(m.localCalls, [[1, 42]]);
  assert.deepEqual(m.freed, [42]);
  assert.equal(s.snapshot.localWish, true);
  s.setLocalRec(null);
  assert.deepEqual(m.localCalls, [[1, 42], [0, 0]]);
  assert.equal(s.snapshot.localWish, false);
});

test('setLocalRec is ignored unless live', () => {
  const { s, f } = mk();
  s.setLocalRec('a.mp4');
  assert.equal(f.made.length, 0);
  assert.equal(s.snapshot.localWish, false);
});

test('onRecClosed reaches the page after the module exited (disconnect seals the file)', async () => {
  const got = [];
  const { s, f } = mk({ onRecClosed: (...a) => got.push(a) });
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  const m = f.made[0];
  s.setLocalRec('a.mp4');
  await s.disconnect();
  await tick();                              // onExit(0) has run: idle
  assert.equal(s.snapshot.state, 'idle');
  assert.equal(s.snapshot.localWish, false);
  m.opts.onRecClosed('a.mp4', 100, 0);       // the core's seal lands after exit
  assert.deepEqual(got, [['a.mp4', 100, 0]]);
});

test('onRecClosed clears localWish while still live (a local error needs only one press to restart)', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  const m = f.made[0];
  s.setLocalRec('a.mp4');
  assert.equal(s.snapshot.localWish, true);
  m.opts.onRecClosed('a.mp4', 10, 2);   // core sealed with a write error
  assert.equal(s.snapshot.localWish, false);
  assert.equal(s.snapshot.state, 'live');
});

test('onRecClosed from a superseded module is dropped', async () => {
  const got = [];
  const { s, f } = mk({ onRecClosed: (...a) => got.push(a) });
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  await s.disconnect();
  await tick();
  await s.connect({ mode: 'gs', ch: 136, w: 40, overlayToml: '' });
  f.made[0].opts.onRecClosed('old.mp4', 5, 0);
  assert.deepEqual(got, []);
  f.made[1].opts.onRecClosed('new.mp4', 7, 0);
  assert.deepEqual(got, [['new.mp4', 7, 0]]);
});

test('relay connect skips WebUSB, passes --relay, starts the worker on onRelayRing', async () => {
  let asked = 0; const started = [];
  const t = fakeTimers(); const f = fakeModuleFactory();
  const s = new Session({
    createModule: f.create, requestDevice: async () => { asked++; },
    startRelay: (o) => { started.push(o); return { terminate() { o.terminated = true; } }; },
    onAu: () => {}, onStats: () => {}, reload: () => {}, timers: t,
  });
  await s.connect({ mode: 'gs', ch: 136, w: 40, relay: '192.168.1.1:8311' });
  assert.equal(asked, 0);
  const args = f.made[0].opts.arguments;
  assert.deepEqual(args.slice(args.indexOf('--relay'), args.indexOf('--relay') + 2), ['--relay', '192.168.1.1:8311']);
  const buf = new ArrayBuffer(8);
  f.made[0].opts.onRelayRing(buf, 1024);
  assert.equal(started.length, 1);
  assert.equal(started[0].buffer, buf); assert.equal(started[0].ptr, 1024);
  assert.equal(started[0].url, 'ws://192.168.1.1:8311');
  f.made[0].opts.onExit(1);
  assert.equal(started[0].terminated, true);
});

test('usb connect still asks for the device and never starts a relay', async () => {
  let asked = 0, started = 0;
  const f = fakeModuleFactory();
  const s = new Session({ createModule: f.create, requestDevice: async () => { asked++; },
    startRelay: () => { started++; return { terminate() {} }; },
    onAu: () => {}, onStats: () => {}, reload: () => {}, timers: fakeTimers() });
  await s.connect({ mode: 'gs', ch: 136, w: 40, relay: null });
  assert.equal(asked, 1); assert.equal(started, 0);
  assert.equal(f.made[0].opts.arguments.includes('--relay'), false);
});

// Emscripten 4.0.12: exit() after an ASYNCIFY await (the startup OPFS probe)
// rethrows its 'unwind' exit sentinel uncaught, so every core exit prints
// "worker sent an error! ...: Uncaught unwind". That is a normal exit, not a
// failed worker: the core's own error text must survive it.
test('uncaught unwind on exit does not mask the core error', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, relay: '192.168.1.1:8311' });
  const o = f.made[0].opts;
  o.onError('ERROR relay owned by another client');
  o.printErr('worker sent an error! http://127.0.0.1:8808/webgs.js:1: Uncaught unwind');
  o.onExit(1);
  assert.equal(s.snapshot.state, 'error');
  assert.equal(s.snapshot.error, errorText('ERROR relay owned by another client'));
});

test('a real worker load failure still maps to WORKER_FAILED', async () => {
  const { s, f } = mk();
  await s.connect({ mode: 'gs', ch: 136, w: 40, relay: '192.168.1.1:8311' });
  f.made[0].opts.printErr('worker sent an error! undefined:undefined: undefined');
  assert.equal(s.snapshot.state, 'error');
  assert.equal(s.snapshot.error, WORKER_FAILED);
});

test('isExitUnwind / isWorkerFailure classify both surfaces of the exit sentinel', () => {
  // printErr line and the window ErrorEvent message the page sees on every exit
  assert.equal(isExitUnwind('worker sent an error! http://127.0.0.1:8808/webgs.js:1: Uncaught unwind'), true);
  assert.equal(isExitUnwind('Uncaught unwind'), true);
  assert.equal(isWorkerFailure('worker sent an error! http://127.0.0.1:8808/webgs.js:1: Uncaught unwind'), false);
  assert.equal(isWorkerFailure('worker sent an error! undefined:undefined: undefined'), true);
  assert.equal(isExitUnwind('worker sent an error! undefined:undefined: undefined'), false);
});

test('isWindowWorkerFailure ignores both window events of a normal exit', () => {
  const file = 'http://127.0.0.1:8808/webgs.js';
  // shapes captured from headless Chrome on a core exit (see session.js)
  assert.equal(isWindowWorkerFailure({ message: 'Uncaught [object ErrorEvent]', filename: file,
    error: { message: 'Uncaught unwind' } }), false);
  assert.equal(isWindowWorkerFailure({ message: 'Uncaught unwind', filename: file, error: null }), false);
  // anything else from the module's worker still counts as a worker failure
  assert.equal(isWindowWorkerFailure({ message: 'Uncaught RuntimeError: unreachable', filename: file, error: {} }), true);
  assert.equal(isWindowWorkerFailure({ message: 'worker sent an error! undefined:undefined: undefined' }), true);
  assert.equal(isWindowWorkerFailure({ message: 'Script error.', filename: 'http://x/app.js' }), false);
});

test('relay connect waits for prepareRelay; its rejection is the error, core never starts', async () => {
  const t = fakeTimers(); const f = fakeModuleFactory();
  let release; const gate = new Promise((r) => { release = r; });
  const urls = [];
  const s = new Session({
    createModule: f.create, requestDevice: async () => { throw new Error('no usb on relay'); },
    startRelay: () => ({ terminate() {} }),
    prepareRelay: async (url) => { urls.push(url); await gate; },
    onAu: () => {}, onStats: () => {}, reload: () => {}, timers: t,
  });
  const p = s.connect({ mode: 'spotter', ch: 136, w: 40, relay: '192.168.1.1:8311' });
  await tick();
  assert.deepEqual(urls, ['ws://192.168.1.1:8311']);
  assert.equal(f.made.length, 0);          // core held until the permission answer
  release(); await p;
  assert.equal(f.made.length, 1);

  const f2 = fakeModuleFactory();
  const s2 = new Session({
    createModule: f2.create, requestDevice: async () => {},
    startRelay: () => ({ terminate() {} }),
    prepareRelay: async () => { throw new Error('Local network access is blocked'); },
    onAu: () => {}, onStats: () => {}, reload: () => {}, timers: fakeTimers(),
  });
  await s2.connect({ mode: 'gs', ch: 136, w: 40, relay: '192.168.1.1:8311' });
  assert.equal(f2.made.length, 0);
  assert.equal(s2.snapshot.state, 'error');
  assert.match(s2.snapshot.error, /Local network access is blocked/);
});
