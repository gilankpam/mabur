// THROWAWAY SPIKE: node --test spike/wasm/test_viewer_logic.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import { nalTypes, Gate, PtsUnwrap, PeriodEstimator, HitchMeter, pctl } from './viewer_logic.mjs';

const nal = (type, len = 4, four = true) =>
  [...(four ? [0, 0, 0, 1] : [0, 0, 1]), type << 1, 1, ...Array(len).fill(0x55)];
const au = (types, extra = {}) => ({
  sid: 0, flags: 0, complete: true,
  data: new Uint8Array(types.flatMap((t, i) => nal(t, 4, i % 2 === 0))), ...extra });
const PARAMS = [32, 33, 34, 1];

test('nalTypes finds 3- and 4-byte start codes', () => {
  assert.deepEqual(nalTypes(au([32, 33, 34, 19]).data), [32, 33, 34, 19]);
});

test('gate holds until a complete AU carries VPS+SPS+PPS, then key then delta', () => {
  const g = new Gate();
  assert.deepEqual(g.onAu(au([1])), { type: null, reset: false, skip: 'gated' });
  assert.deepEqual(g.onAu(au([32, 33, 1])), { type: null, reset: false, skip: 'gated' });
  assert.deepEqual(g.onAu(au(PARAMS)), { type: 'key', reset: false, skip: null });
  assert.deepEqual(g.onAu(au([1])), { type: 'delta', reset: false, skip: null });
});

test('truncated AUs are always skipped, armed or not (review focus 3)', () => {
  const g = new Gate();
  assert.equal(g.onAu(au(PARAMS, { complete: false })).skip, 'truncated');
  assert.equal(g.armed, false);
  g.onAu(au(PARAMS));
  assert.deepEqual(g.onAu(au([1], { complete: false, sid: 1 })), { type: null, reset: false, skip: 'truncated' });
  assert.equal(g.armed, true);
});

test('DISCONT while armed resets and re-gates (review focus 2)', () => {
  const g = new Gate();
  g.onAu(au(PARAMS));
  assert.deepEqual(g.onAu(au([1], { flags: 0x02 })), { type: null, reset: true, skip: 'gated' });
  assert.equal(g.armed, false);
  assert.deepEqual(g.onAu(au(PARAMS, { flags: 0x02 })), { type: 'key', reset: false, skip: null });
});

test('decoder error disarms until the next parameter-set AU (review focus 3)', () => {
  const g = new Gate();
  g.onAu(au(PARAMS));
  g.onDecoderError();
  assert.equal(g.onAu(au([1])).skip, 'gated');
  assert.equal(g.onAu(au(PARAMS)).type, 'key');
});

test('mid-stream join: many plain AUs before params never produce a decode (review focus 1)', () => {
  const g = new Gate();
  for (let i = 0; i < 200; i++) assert.equal(g.onAu(au([i % 2 ? 0 : 1])).type, null);
  assert.equal(g.onAu(au(PARAMS)).type, 'key');
});

test('PtsUnwrap is monotone across the u32 wrap', () => {
  const u = new PtsUnwrap();
  assert.equal(u.add(0xffff0000), 0xffff0000);
  assert.equal(u.add(0x00001000), 0x100001000);
});

test('PeriodEstimator returns the median delta in ms', () => {
  const p = new PeriodEstimator();
  assert.equal(p.periodMs(), null);
  [0, 16667, 33334, 50001, 90000].forEach((t) => p.add(t));
  assert.equal(p.periodMs(), 16.667);
});

test('HitchMeter flags gaps over 1.5 periods only', () => {
  const h = new HitchMeter();
  assert.equal(h.addDraw(0, 16.7), null);
  assert.equal(h.addDraw(17, 16.7), null);
  assert.equal(h.addDraw(60, 16.7), 43);
  assert.equal(h.addDraw(61, null), null);
});

test('pctl', () => {
  assert.equal(pctl([], 0.99), 0);
  assert.equal(pctl([5, 1, 3, 2, 4], 0.5), 3);
  assert.equal(pctl([5, 1, 3, 2, 4], 1), 5);
});

import { annexbToLengthPrefixed } from './viewer_logic.mjs';

test('annexbToLengthPrefixed rewrites 3- and 4-byte start codes as u32 BE lengths', () => {
  const ab = new Uint8Array([0, 0, 0, 1, 0x40, 1, 7, 0, 0, 1, 0x02, 1, 9, 9]);
  assert.deepEqual([...annexbToLengthPrefixed(ab)],
    [0, 0, 0, 3, 0x40, 1, 7, 0, 0, 0, 4, 0x02, 1, 9, 9]);
});
