import test from 'node:test';
import assert from 'node:assert/strict';
import { RING, rxRing, txRing, ringWrite, ringRead } from '../ui/src/lib/relay_ring.js';

// Same goldens as tests/test_relay_ring.cpp (cap 32).
function small() {
  const buf = new ArrayBuffer(64);
  return { i32: new Int32Array(buf, 0, 8), u8: new Uint8Array(buf), head: 0, tail: 1, drops: 5, off: 32, cap: 32 };
}
const enc = (s) => new TextEncoder().encode(s);

test('write/read golden', () => {
  const r = small();
  assert.equal(ringWrite(r, enc('ABCDE')), true);
  assert.deepEqual([...r.u8.subarray(32, 44)], [5, 0, 0, 0, 65, 66, 67, 68, 69, 0, 0, 0]);
  assert.equal(r.i32[0], 12);
  assert.deepEqual([...ringRead(r)], [...enc('ABCDE')]);
  assert.equal(ringRead(r), null);
});

test('wrap marker golden', () => {
  const r = small();
  assert.equal(ringWrite(r, enc('012345678')), true);
  assert.ok(ringRead(r));                                    // head 16, tail 16
  assert.equal(ringWrite(r, enc('wxyz!')), true);
  assert.ok(ringRead(r));                                    // head 28, tail 28
  assert.equal(ringWrite(r, enc('abcde')), true);
  assert.deepEqual([...r.u8.subarray(32 + 28, 32 + 32)], [255, 255, 255, 255]);
  // Only the header + real payload are pinned; the 3 pad bytes at u8[41..43]
  // are leftover from the first write's own payload (which reached that
  // offset) and are not part of what ringRead exposes.
  assert.deepEqual([...r.u8.subarray(32, 41)], [5, 0, 0, 0, 97, 98, 99, 100, 101]);
  assert.equal(r.i32[0], 44);
  assert.deepEqual([...ringRead(r)], [...enc('abcde')]);
  assert.equal(r.i32[1], 44);
});

test('ring full drops newest and recovers', () => {
  const r = small();
  const m = enc('0123456789');
  assert.equal(ringWrite(r, m), true);
  assert.equal(ringWrite(r, m), true);
  assert.equal(ringWrite(r, m), false);
  assert.equal(r.i32[5], 1);
  assert.ok(ringRead(r));
  assert.equal(ringWrite(r, m), true);
  assert.equal(ringWrite(r, enc('0123456789abcdefg')), false);
  assert.equal(r.i32[5], 2);
});

test('read returns a copy, not a view', () => {
  const r = small();
  ringWrite(r, enc('xyz'));
  const got = ringRead(r);
  assert.notEqual(got.buffer, r.u8.buffer);
});

test('real layout', () => {
  const buf = new ArrayBuffer(RING.HDR + RING.RX_CAP + RING.TX_CAP + 8);
  const rx = rxRing(buf, 8), tx = txRing(buf, 8);
  assert.equal(rx.off, 8 + RING.HDR); assert.equal(rx.cap, RING.RX_CAP);
  assert.equal(tx.off, 8 + RING.HDR + RING.RX_CAP); assert.equal(tx.cap, RING.TX_CAP);
  assert.equal(tx.head, RING.TX_HEAD); assert.equal(rx.drops, RING.RX_DROPS);
});
