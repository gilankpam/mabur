import test from 'node:test';
import assert from 'node:assert/strict';
import { Telemetry } from '../ui/src/lib/telemetry.js';

// 200 ms samples read only the 1 s window; the 60 s window refreshes at 1 Hz.
test('Telemetry.sample: 60 s snapshot at 1 Hz, 1 s window every tick', () => {
  const calls = { full: 0, one: 0 };
  const video = {
    hitchTimes: [],
    periodMs: () => 16.7,
    metrics: { sample: () => ({ fps: 60 }) },
    segWindow: {
      snapshot: () => { calls.full++; return { w1: { present: { p50: 1, p99: 1, max: 1, n: 1 } }, w60: { present: { p50: 9, p99: 9, max: 9, n: 9 } } }; },
      snapshot1: () => { calls.one++; return { w1: { present: { p50: 2, p99: 2, max: 2, n: 1 } } }; },
    },
  };
  const t = new Telemetry(video);
  for (let i = 0; i < 10; i++) t.sample(1000 + i * 200, 'spotter');   // 2 s of ticks
  assert.equal(calls.full, 2);
  assert.equal(calls.one, 8);
  assert.equal(t.seg.w1.present.p50, 2, 'fresh 1 s window between full snapshots');
  assert.equal(t.seg.w60.present.p50, 9, 'w60 carried from the last 1 Hz snapshot');
  t.reset();
  t.sample(5000, 'spotter');
  assert.equal(calls.full, 3, 'reset forces a full snapshot');
});
