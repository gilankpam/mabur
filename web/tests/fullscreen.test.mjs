import test from 'node:test';
import assert from 'node:assert/strict';
import { settle, fsBeforeConnect, FS_STUCK, FS_SETTLE_MS } from '../ui/src/lib/fullscreen.js';

test('settle: fulfil, reject, and a request that never settles', async () => {
  assert.deepEqual(await settle(Promise.resolve(), 50), { ok: true });
  const e = new TypeError('Fullscreen request denied');
  assert.deepEqual(await settle(Promise.reject(e), 50), { err: e });
  // A bare rejection (undefined) still yields an Error to report.
  const r = await settle(Promise.reject(), 50);
  assert.ok(r.err instanceof Error);
  // Android Chrome with the toolbar locked: the promise stays pending forever.
  assert.deepEqual(await settle(new Promise(() => {}), 10), { timeout: true });
});

test('settle clears its timer once the promise lands', async () => {
  const cleared = [];
  const r = await settle(Promise.resolve(), 1000, () => 'tok', (t) => cleared.push(t));
  assert.deepEqual(r, { ok: true });
  assert.deepEqual(cleared, ['tok']);
});

test('phone Connect goes fullscreen first for relay, or USB once granted', () => {
  assert.equal(fsBeforeConnect({ mobile: true, radio: 'relay', usbGranted: false }), true);
  assert.equal(fsBeforeConnect({ mobile: true, radio: 'usb', usbGranted: true }), true);
  // First-time WebUSB chooser needs the tap's activation: no fullscreen first.
  assert.equal(fsBeforeConnect({ mobile: true, radio: 'usb', usbGranted: false }), false);
  // Desktop layouts never do this.
  assert.equal(fsBeforeConnect({ mobile: false, radio: 'relay', usbGranted: true }), false);
});

test('stuck message names the cause and the way out', () => {
  assert.match(FS_STUCK, /ws:\/\//);
  assert.match(FS_STUCK, /Not secure/);
  assert.match(FS_STUCK, /Reload/);
  assert.ok(FS_SETTLE_MS >= 1000 && FS_SETTLE_MS <= 3000);
});
