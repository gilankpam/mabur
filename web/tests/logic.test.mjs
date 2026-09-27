// node --test web/tests/logic.test.mjs (promoted from the wasm-spike throwaway)
import test from 'node:test';
import assert from 'node:assert/strict';
import { nalTypes, Gate, PtsUnwrap, PeriodEstimator, HitchMeter, pctl } from '../www/logic.mjs';

const nal = (type, len = 4, four = true) =>
  [...(four ? [0, 0, 0, 1] : [0, 0, 1]), type << 1, 1, ...Array(len).fill(0x55)];
const au = (types, extra = {}) => ({
  sid: 0, flags: 0, complete: true,
  data: new Uint8Array(types.flatMap((t, i) => nal(t, 4, i % 2 === 0))), ...extra });
const PARAMS = [32, 33, 34, 19];  // parameter sets + IDR_W_RADL (IRAP)
const GDR = [32, 33, 34, 1];       // parameter sets + TRAIL_R refresh (drone's GDR)

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

import { annexbToLengthPrefixed } from '../www/logic.mjs';

test('annexbToLengthPrefixed rewrites 3- and 4-byte start codes as u32 BE lengths', () => {
  const ab = new Uint8Array([0, 0, 0, 1, 0x40, 1, 7, 0, 0, 1, 0x02, 1, 9, 9]);
  assert.deepEqual([...annexbToLengthPrefixed(ab)],
    [0, 0, 0, 3, 0x40, 1, 7, 0, 0, 0, 4, 0x02, 1, 9, 9]);
});

test('GDR parameter-set AUs never arm the gate: WebCodecs needs an IRAP key (final review)', () => {
  const g = new Gate();
  assert.deepEqual(g.onAu(au(GDR)), { type: null, reset: false, skip: 'awaiting-irap' });
  assert.equal(g.armed, false);
  assert.equal(g.onAu(au(PARAMS)).type, 'key');
});

import { DecoderSlot } from '../www/logic.mjs';

test('DecoderSlot.replace closes the previous decoder unless already closed (final review)', () => {
  const mk = () => ({ state: 'configured', closed: 0, close() { this.closed++; this.state = 'closed'; } });
  const slot = new DecoderSlot();
  const a = mk(), b = mk(), c = mk();
  slot.replace(a);
  slot.replace(b);
  assert.equal(a.closed, 1);
  b.state = 'closed';        // WebCodecs already closed it (error callback)
  slot.replace(c);
  assert.equal(b.closed, 0);
  assert.equal(slot.d, c);
});

import {
  SegWindow, capToGlass, rcfHeardPct, errorText, ht40Offset, checkChannelWidth,
  pruneSubmitted, trimBefore, copyStatsPayload,
} from '../www/logic.mjs';

test('SegWindow p50/p99/max over 1 s and 60 s', () => {
  let now = 0; const w = new SegWindow(() => now);
  for (let i = 1; i <= 100; i++) { now = i * 5; w.add('decode', i); }  // 0..500 ms
  now = 1400;
  w.add('decode', 1000);
  const s = w.snapshot();
  // t=5,10,...,500 (i=1..100) plus t=1400 (value 1000); cutoff = now-1000 = 400.
  // t>400 => i=81..100 (20 entries) + the t=1400 entry = 21, not the brief's 81
  // (fixed per task-7-brief.md's own escape hatch: test number was
  // arithmetically inconsistent with the stated "t > 400" semantics).
  assert.equal(s.w1.decode.n, 21);            // entries with t > 400
  assert.equal(s.w1.decode.max, 1000);
  assert.equal(s.w60.decode.n, 101);
  assert.equal(s.w60.decode.p50, 51);
});

test('capToGlass null without offset, sum with it', () => {
  assert.equal(capToGlass({ capToCompleteUs: -1, tEmitMs: 0, tRecvMs: 1, tPresentMs: 5 }), null);
  assert.equal(capToGlass({ capToCompleteUs: 30000, tEmitMs: 100, tRecvMs: 102, tPresentMs: 120 }), 50);
});

test('rcfHeardPct deltas, resets and missing fields', () => {
  assert.equal(rcfHeardPct(null, { drone_rcf_rx: 5, rcf_sent: 5 }), null);
  assert.equal(rcfHeardPct({ drone_rcf_rx: 100, rcf_sent: 100 }, { drone_rcf_rx: 119, rcf_sent: 120 }), 95);
  assert.equal(rcfHeardPct({ drone_rcf_rx: 100, rcf_sent: 100 }, { drone_rcf_rx: 3, rcf_sent: 120 }), null);
  assert.equal(rcfHeardPct({ drone_rcf_rx: null, rcf_sent: 1 }, { drone_rcf_rx: null, rcf_sent: 2 }), null);
});

test('rcfHeardPct divides by rcf_sent, not sends (DISC keep-alives are not RCFs)', () => {
  // 20 RCFs all heard, plus 5 DISC keep-alives in the same second: 100 %,
  // not 80 %.
  const prev = { drone_rcf_rx: 100, rcf_sent: 100, sends: 110 };
  const cur = { drone_rcf_rx: 120, rcf_sent: 120, sends: 135 };
  assert.equal(rcfHeardPct(prev, cur), 100);
  // No rcf_sent (old core): no number rather than a wrong one.
  assert.equal(rcfHeardPct({ drone_rcf_rx: 1, sends: 1 }, { drone_rcf_rx: 2, sends: 2 }), null);
});

test('ht40Offset matches common/include/mabur/ht40.h', () => {
  assert.equal(ht40Offset(36), 1); assert.equal(ht40Offset(40), 2);
  assert.equal(ht40Offset(132), 1); assert.equal(ht40Offset(136), 2);
  assert.equal(ht40Offset(149), 1); assert.equal(ht40Offset(161), 2);
  assert.equal(ht40Offset(165), 0); assert.equal(ht40Offset(6), 0);
});

test('checkChannelWidth refuses bad channel/width before connect', () => {
  assert.equal(checkChannelWidth('136', '40'), null);
  assert.equal(checkChannelWidth('165', '20'), null);
  assert.match(checkChannelWidth('abc', '20'), /not a number/);
  assert.match(checkChannelWidth('', '20'), /not a number/);
  assert.match(checkChannelWidth('13.5', '20'), /not a number/);
  assert.match(checkChannelWidth('0', '20'), /out of range/);
  assert.match(checkChannelWidth('201', '20'), /out of range/);
  assert.match(checkChannelWidth('136', '80'), /20 or 40/);
  assert.match(checkChannelWidth('165', '40'), /no 40 MHz pair/);
});

test('pruneSubmitted drops records older than 1 s', () => {
  const m = new Map([[1, { tSubmit: 0 }], [2, { tSubmit: 1000 }], [3, { tSubmit: 1500 }]]);
  pruneSubmitted(m, 1950);
  assert.deepEqual([...m.keys()], [2, 3]);
});

test('trimBefore trims ascending times in place', () => {
  const a = [1, 2, 3, 10];
  assert.equal(trimBefore(a, 3), a);
  assert.deepEqual(a, [10]);
  assert.deepEqual(trimBefore([], 5), []);
});

test('copyStatsPayload carries core stats and page segments', () => {
  const snap = { w1: { decode: { p50: 1, p99: 2, max: 3, n: 4 } }, w60: { decode: { p50: 1, p99: 2, max: 3, n: 40 } } };
  const out = JSON.parse(copyStatsPayload(['{"aus":1}', 'garbage'], snap));
  assert.deepEqual(out.core, [{ aus: 1 }, 'garbage']);
  assert.deepEqual(out.segments, snap);
});

test('errorText maps glue errors', () => {
  assert.match(errorText('ERROR claim failed rc=-6'), /Card busy/);
  assert.match(errorText('ERROR card lost'), /Card lost/);
  assert.match(errorText('bad channel/width: radio.width: 40 MHz needs a standard 5 GHz pair'),
    /Channel\/width refused: radio\.width: 40 MHz needs/);
  assert.equal(errorText('ERROR something new'), 'ERROR something new');
});
