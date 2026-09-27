import test from 'node:test';
import assert from 'node:assert/strict';
import { PageMetrics, sparkPoints } from '../ui/src/lib/metrics.js';

test('bitrate, fps, jitter over the trailing second', () => {
  const m = new PageMetrics();
  for (let i = 0; i < 60; i++) { m.addAu(1000 + i * 16.6667, 25000); m.addDraw(1000 + i * 16.6667); }
  const s = m.sample(1990, 16.6667);
  assert.ok(Math.abs(s.bitrateMbps - 12.0) < 0.1, `bitrate ${s.bitrateMbps}`);
  assert.equal(s.fps, 60);
  assert.ok(s.jitterMs < 0.01);
});

test('jitter is mean |interval - period|', () => {
  const m = new PageMetrics();
  [0, 16, 36, 52].forEach((t) => m.addDraw(1000 + t));   // intervals 16, 20, 16
  const s = m.sample(1060, 16);
  assert.ok(Math.abs(s.jitterMs - 4 / 3) < 1e-9);
});

test('empty and stale windows give nulls; old entries pruned', () => {
  const m = new PageMetrics();
  assert.deepEqual(m.sample(5000, 16), { bitrateMbps: null, fps: null, jitterMs: null });
  m.addAu(1000, 100); m.addDraw(1000);
  assert.deepEqual(m.sample(5000, 16), { bitrateMbps: null, fps: null, jitterMs: null });
  assert.equal(m.aus.length, 0);
});

test('sparkline points', () => {
  assert.equal(sparkPoints([]), '0,24 100,24');
  const p = sparkPoints([10, 20]).split(' ');
  assert.equal(p.length, 2);
  assert.equal(p[0].split(',')[0], '0.0');
  assert.ok(sparkPoints([10, null, 20]).split(' ').length === 2, 'nulls skipped');
});
