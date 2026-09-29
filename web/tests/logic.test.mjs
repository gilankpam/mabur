// node --test web/tests/logic.test.mjs (promoted from the wasm-spike throwaway)
import test from 'node:test';
import assert from 'node:assert/strict';
import { nalTypes, Gate, PtsUnwrap, PeriodEstimator, HitchMeter, pctl, IdrRequester,
  parseRelayAddr, relayBlocker } from '../ui/src/lib/logic.mjs';

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

import { annexbToLengthPrefixed } from '../ui/src/lib/logic.mjs';

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

import { DecoderSlot } from '../ui/src/lib/logic.mjs';

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
  SegWindow, capToGlass, rcfHeardPct, rcfHeardPctWindowed, errorText, ht40Offset,
  checkChannelWidth, pruneSubmitted, trimBefore, copyStatsPayload, controlsLocked,
  isStalePresentSample,
} from '../ui/src/lib/logic.mjs';

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

test('rcfHeardPctWindowed diffs newest vs. ~10 ticks back, not adjacent ticks (F2)', () => {
  // Jittery per-tick rcf_sent deltas (10, 1, 19, 1, 19, 1, 19, 1, 19, 10) with
  // rx advancing evenly by 9/tick: tick 2 alone is 9/1 = 900 %, but the
  // 10-tick window (idx 0 vs idx 10) is 90/100 = 90 %.
  const sent = [0, 10, 11, 30, 31, 50, 51, 70, 71, 90, 100];
  const ring = sent.map((s, i) => ({ drone_rcf_rx: i * 9, rcf_sent: s }));
  assert.equal(rcfHeardPct(ring[0], ring[1]), 90);
  assert.ok(rcfHeardPct(ring[1], ring[2]) > 100);
  assert.equal(rcfHeardPctWindowed(ring), 90);
});

test('rcfHeardPctWindowed uses the oldest available entry when the ring is shorter than the window', () => {
  const ring = [
    { drone_rcf_rx: 0, rcf_sent: 0 },
    { drone_rcf_rx: 10, rcf_sent: 10 },
    { drone_rcf_rx: 19, rcf_sent: 20 },
  ];
  assert.equal(rcfHeardPctWindowed(ring), 95);   // vs ring[0], not ring[1]
});

test('rcfHeardPctWindowed null on an empty or single-entry ring', () => {
  assert.equal(rcfHeardPctWindowed([]), null);
  assert.equal(rcfHeardPctWindowed([{ drone_rcf_rx: 1, rcf_sent: 1 }]), null);
});

test('controlsLocked locks while connecting or connected, not when idle (R12/F1)', () => {
  assert.equal(controlsLocked({ connecting: false, connected: false }), false);
  assert.equal(controlsLocked({ connecting: true, connected: false }), true);
  assert.equal(controlsLocked({ connecting: false, connected: true }), true);
  assert.equal(controlsLocked({ connecting: true, connected: true }), true);
});

test('isStalePresentSample drops rAF callbacks that fired long after the draw (F3)', () => {
  assert.equal(isStalePresentSample(1000, 1016), false);   // a normal ~16 ms frame
  assert.equal(isStalePresentSample(1000, 1999), false);   // right at the edge
  assert.equal(isStalePresentSample(1000, 2001), true);    // just over the 1000 ms gap
  assert.equal(isStalePresentSample(1000, 19000), true);   // the observed ~18 s outlier
});

test('errorText maps glue errors', () => {
  assert.match(errorText('ERROR claim failed rc=-6'), /Card busy/);
  assert.match(errorText('ERROR card lost'), /Card lost/);
  assert.match(errorText('bad channel/width: radio.width: 40 MHz needs a standard 5 GHz pair'),
    /Channel\/width refused: radio\.width: 40 MHz needs/);
  assert.equal(errorText('ERROR something new'), 'ERROR something new');
});

test('errorText maps bad config and strips the file:line prefix', () => {
  assert.equal(errorText('bad config: config: /maburgs.toml:88: link.ladder[0].overhead_base: must be >= overhead_enh'),
    'Config refused: link.ladder[0].overhead_base: must be >= overhead_enh. Fix it in Config and press Connect.');
});

test('SegWindow snapshot1: 1 s window only, same numbers as snapshot().w1', () => {
  let now = 0; const w = new SegWindow(() => now);
  for (let i = 1; i <= 100; i++) { now = i * 5; w.add('decode', i); }
  now = 1400;
  w.add('decode', 1000);
  w.add('present', 3);
  const full = w.snapshot();
  const s1 = w.snapshot1();
  assert.deepEqual(Object.keys(s1), ['w1']);
  assert.deepEqual(s1.w1, full.w1);
  assert.equal(s1.w1.decode.n, 21);
  assert.equal(s1.w1.decode.p50, 91);
  assert.equal(s1.w1.decode.p99, 1000);
  // a segment whose samples all aged out of the 1 s window reads empty
  now = 2500;
  assert.deepEqual(w.snapshot1(now).w1.decode, { p50: 0, p99: 0, max: 0, n: 0 });
});

test('IdrRequester fires at once when unarmed, then every retryMs', () => {
  const r = new IdrRequester(300);
  assert.equal(r.poll(false, 1000), true);    // gate starts unarmed: ask now
  assert.equal(r.poll(false, 1100), false);
  assert.equal(r.poll(false, 1299), false);
  assert.equal(r.poll(false, 1300), true);    // IDR presumed lost: ask again
  assert.equal(r.poll(false, 1400), false);
});

test('IdrRequester is silent while armed and re-fires on the next drop', () => {
  const r = new IdrRequester(300);
  assert.equal(r.poll(true, 1000), false);
  assert.equal(r.poll(true, 5000), false);
  assert.equal(r.poll(false, 5010), true);    // drop edge: immediate, not 300 ms later
  assert.equal(r.poll(true, 5100), false);
  assert.equal(r.poll(false, 5150), true);    // re-armed in between resets the clock
});

test('IdrRequester does not fire faster than retryMs', () => {
  const r = new IdrRequester(300);
  let n = 0;
  for (let t = 0; t < 3000; t += 16) if (r.poll(false, t)) n++;   // 60 AU/s, unarmed 3 s
  assert.equal(n, 10);
});

test('IdrRequester backs off after consecutive key-frame failures, capped at maxRetryMs', () => {
  const r = new IdrRequester(300, 4800);
  assert.equal(r.poll(false, 0), true);       // unarmed edge, n=0: immediate
  r.noteKeyFailed();                          // n=1: next interval 600
  assert.equal(r.poll(false, 300), false);
  assert.equal(r.poll(false, 599), false);
  assert.equal(r.poll(false, 600), true);
  r.noteKeyFailed();                          // n=2: next interval 1200
  assert.equal(r.poll(false, 1200 + 600 - 1), false);
  assert.equal(r.poll(false, 1200 + 600), true);
  // Run n up past the cap: interval never exceeds maxRetryMs (4800).
  for (let i = 0; i < 8; i++) r.noteKeyFailed();
  const t0 = 100000;
  assert.equal(r.poll(false, t0), true);
  assert.equal(r.poll(false, t0 + 4799), false);
  assert.equal(r.poll(false, t0 + 4800), true);
});

test('IdrRequester.noteOutput resets the backoff to the base retryMs', () => {
  const r = new IdrRequester(300);
  r.noteKeyFailed();
  r.noteKeyFailed();
  r.noteOutput();
  assert.equal(r.poll(false, 0), true);
  assert.equal(r.poll(false, 299), false);
  assert.equal(r.poll(false, 300), true);     // back to the base 300 ms, not 1200
});

test('IdrRequester.cancel un-fires the last poll so the next one requests again', () => {
  const r = new IdrRequester(300);
  assert.equal(r.poll(false, 1000), true);    // unarmed edge: fires
  r.cancel();                                 // onWantIdr returned false: didn't really send
  assert.equal(r.poll(false, 1001), true);    // immediate retry, not gated by retryMs
});

test('IdrRequester.cancel after a backed-off retry restores the pre-retry timestamp', () => {
  const r = new IdrRequester(300);
  r.poll(false, 0);                           // edge fire, lastMs=0
  assert.equal(r.poll(false, 300), true);     // retry fires, lastMs=300
  r.cancel();                                 // restore lastMs to 0
  assert.equal(r.poll(false, 299), false);    // still within retryMs of the restored 0
  assert.equal(r.poll(false, 300), true);
});

test('parseRelayAddr', () => {
  assert.equal(parseRelayAddr('192.168.1.1'), '192.168.1.1:8311');
  assert.equal(parseRelayAddr(' 192.168.1.1:9000 '), '192.168.1.1:9000');
  assert.equal(parseRelayAddr('cpe.local:8311'), 'cpe.local:8311');
  assert.equal(parseRelayAddr(''), null);
  assert.equal(parseRelayAddr('192.168.1.1:0'), null);
  assert.equal(parseRelayAddr('192.168.1.1:70000'), null);
  assert.equal(parseRelayAddr('ws://192.168.1.1'), null);
  assert.equal(parseRelayAddr('a b'), null);
});

test('relayBlocker: https needs an LNA-exempt host, bad address blocks', () => {
  // Revert check: the pre-2026-09-29 blocker refused every https origin.
  assert.equal(relayBlocker('https:', '192.168.1.1:8311'), null);
  assert.equal(relayBlocker('https:', '10.0.0.5'), null);
  assert.equal(relayBlocker('https:', '172.20.1.1:9000'), null);
  assert.equal(relayBlocker('https:', 'cpe.local'), null);
  assert.match(relayBlocker('https:', 'relay.example.com'), /private IP/);
  assert.match(relayBlocker('https:', '172.32.0.1'), /private IP/);
  assert.match(relayBlocker('https:', '8.8.8.8'), /private IP/);
  assert.equal(relayBlocker('http:', 'relay.example.com'), null);
  assert.equal(relayBlocker('http:', '192.168.1.1:8311'), null);
  assert.match(relayBlocker('http:', 'nope nope'), /address/);
  assert.match(relayBlocker('https:', 'nope nope'), /address/);
});

test('errorText relay lines', () => {
  assert.match(errorText('ERROR relay unreachable', 'http:'), /CPE relay not reachable/);
  assert.doesNotMatch(errorText('ERROR relay unreachable', 'http:'), /Chrome 142/);
  assert.match(errorText('ERROR relay unreachable', 'https:'), /Chrome 142\+/);
  assert.match(errorText('ERROR relay unreachable: cannot resolve x:1'), /CPE relay not reachable/);
  assert.match(errorText('ERROR relay owned by another client'), /owned by another client/);
  assert.match(errorText('ERROR relay taken by another client'), /Another client took over the CPE relay/);
  assert.match(errorText('ERROR relay lost'), /connection lost/);
  assert.match(errorText('ERROR relay cannot tune'), /could not tune/);
});
