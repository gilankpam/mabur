import test from 'node:test';
import assert from 'node:assert/strict';
import { recView, RecClock, formatClock } from '../ui/src/lib/rec.js';

test('rec view from core stats', () => {
  assert.deepEqual(recView(null), { state: 'unknown', err: null });
  assert.deepEqual(recView({ rec_state: null }), { state: 'unknown', err: null });
  assert.deepEqual(recView({ rec_state: 0, rec_err: 0 }), { state: 'off', err: null });
  assert.deepEqual(recView({ rec_state: 1, rec_err: 0 }), { state: 'recording', err: null });
  assert.deepEqual(recView({ rec_state: 2, rec_err: 5 }), { state: 'error', err: 'SD card full' });
  assert.equal(recView({ rec_state: 2, rec_err: 42 }).err, 'VTX recorder error 42');
});

test('clock starts on the first recording report, resets when it stops', () => {
  const c = new RecClock();
  c.update('off', 0);
  assert.equal(c.elapsedMs(500), 0);
  c.update('recording', 1000);
  c.update('recording', 2000);
  assert.equal(c.elapsedMs(4500), 3500);
  c.update('off', 5000);
  assert.equal(c.elapsedMs(6000), 0);
  assert.equal(formatClock(3723000), '01:02:03');
});
