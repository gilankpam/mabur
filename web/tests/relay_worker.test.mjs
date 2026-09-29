// Pure per-iteration decision for the relay Worker's loop (relay_worker.js).
// Pulled out here so the no-spin-before-open fix (review round 1, defect 1)
// is unit-testable without a browser/Worker: the Worker just calls this and
// switches on the string.
import test from 'node:test';
import assert from 'node:assert/strict';
import { RING, workerStep } from '../ui/src/lib/relay_ring.js';

test('workerStep: STOP_REQ always stops, regardless of open/txEmpty', () => {
  assert.equal(workerStep(RING.STOP_REQ, false, false), 'stop');
  assert.equal(workerStep(RING.STOP_REQ, true, true), 'stop');
});

// This is the spin case the review caught: the core queues HELLO+TUNE
// before the Worker's socket opens, so TX is non-empty while CONNECTING.
// The old loop read that as "pump (does nothing, !open) then TX not empty
// so don't wait" -> synchronous infinite loop, ws.onopen never gets a turn.
test('workerStep: not open waits for open instead of spinning on a non-empty TX', () => {
  assert.equal(workerStep(RING.CONNECTING, false, false), 'wait-open');
  assert.equal(workerStep(RING.CONNECTING, false, true), 'wait-open');
});

test('workerStep: open with an empty TX ring pumps then waits on TX_HEAD', () => {
  assert.equal(workerStep(RING.OPEN, true, true), 'pump-then-wait-tx');
});

test('workerStep: open with pending TX just pumps (no wait)', () => {
  assert.equal(workerStep(RING.OPEN, true, false), 'pump');
});
