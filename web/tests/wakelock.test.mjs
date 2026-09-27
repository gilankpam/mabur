import test from 'node:test';
import assert from 'node:assert/strict';
import { ScreenWake } from '../ui/src/lib/wakelock.js';

function fakeApi() {
  const api = { requests: 0, held: [], resolvers: [] };
  api.request = (type) => {
    assert.equal(type, 'screen');
    api.requests++;
    return new Promise((res) => api.resolvers.push(() => {
      const listeners = [];
      const s = { released: false,
        release: async () => { s.released = true; },
        addEventListener: (ev, fn) => { if (ev === 'release') listeners.push(fn); },
        sysRelease: () => { s.released = true; listeners.forEach((f) => f()); } };
      api.held.push(s);
      res(s);
    }));
  };
  api.grant = async () => { api.resolvers.shift()(); await new Promise((r) => setTimeout(r)); };
  return api;
}

test('holds the lock while wanted, releases on set(false)', async () => {
  const api = fakeApi();
  const w = new ScreenWake(api, () => false);
  w.set(true);
  w.set(true);                      // duplicate while in flight: one request
  await api.grant();
  assert.equal(api.requests, 1);
  assert.ok(w.sentinel);
  w.set(false);
  assert.equal(api.held[0].released, true);
  assert.equal(w.sentinel, null);
});

test('a request resolving after set(false) is released at once', async () => {
  const api = fakeApi();
  const w = new ScreenWake(api, () => false);
  w.set(true);
  w.set(false);
  await api.grant();
  assert.equal(api.held[0].released, true);
  assert.equal(w.sentinel, null);
});

test('re-acquires after the browser drops it on hide', async () => {
  const api = fakeApi();
  let hidden = false;
  const w = new ScreenWake(api, () => hidden);
  w.set(true);
  await api.grant();
  hidden = true;
  api.held[0].sysRelease();
  assert.equal(w.sentinel, null);
  w.onVisible();                    // still hidden: no request
  assert.equal(api.requests, 1);
  hidden = false;
  w.onVisible();
  await api.grant();
  assert.equal(api.requests, 2);
  assert.ok(w.sentinel);
});

test('no API or a refused request is a no-op', async () => {
  new ScreenWake(undefined, () => false).set(true);
  const w = new ScreenWake({ request: async () => { throw new Error('NotAllowedError'); } }, () => false);
  w.set(true);
  await new Promise((r) => setTimeout(r));
  assert.equal(w.sentinel, null);
  assert.equal(w.pending, false);
});
