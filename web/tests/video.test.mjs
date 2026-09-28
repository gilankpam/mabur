import test from 'node:test';
import assert from 'node:assert/strict';

// video.js is browser code; stub the few globals its per-AU path touches.
globalThis.performance ??= { timeOrigin: 0, now: () => Date.now() };
globalThis.document ??= { hidden: false };
globalThis.requestAnimationFrame ??= () => 0;
const made = [];
class FakeDecoder {
  constructor(init) { this.init = init; this.state = 'unconfigured'; this.chunks = []; made.push(this); }
  configure() { this.state = 'configured'; }
  decode(c) { this.chunks.push(c); }
  close() { this.state = 'closed'; }
}
globalThis.EncodedVideoChunk ??= class { constructor(o) { Object.assign(this, o); } };
const { VideoPipeline } = await import('../ui/src/lib/video.js');

// VPS/SPS/PPS + IDR_W_RADL (type 19), Annex-B start codes.
const nal = (t) => [0, 0, 0, 1, t << 1, 1, 0xaa];
const IRAP = new Uint8Array([...nal(32), ...nal(33), ...nal(34), ...nal(19)]).buffer;
const P = new Uint8Array(nal(1)).buffer;
const hvcc = new Uint8Array([1]).buffer;
const au = (v, buf, pts, hv = null) => v.onAu(buf, pts, 1, 0, 1, 0, hv, -1, 0, 0);

test('no VideoDecoder at construction/reset: the page loads without WebCodecs', () => {
  delete globalThis.VideoDecoder;
  const v = new VideoPipeline({ getCanvas: () => null, getMode: () => 'gs' });
  v.reset();
  au(v, IRAP, 1000, hvcc);   // key frame without WebCodecs: logged, no throw
  assert.equal(v.decoder, null);
  assert.equal(v.gateArmed(), false);
});

test('decoder is lazy, and reset()/close() close it (no leak per Connect)', () => {
  globalThis.VideoDecoder = FakeDecoder;
  made.length = 0;
  const v = new VideoPipeline({ getCanvas: () => null, getMode: () => 'gs' });
  assert.equal(made.length, 0, 'nothing built before the first key frame');
  au(v, IRAP, 1000, hvcc);
  assert.equal(made.length, 1);
  au(v, P, 17000);
  assert.equal(made[0].chunks.length, 2);
  v.reset();                                  // next Connect
  assert.equal(made[0].state, 'closed');
  assert.equal(v.decoder, null);
  au(v, IRAP, 1000, hvcc);
  assert.equal(made.length, 2);
  v.close();                                  // session left live
  assert.equal(made[1].state, 'closed');
  assert.equal(v.decoder, null);
  au(v, P, 17000);                            // straggler delta: no decoder rebuilt
  assert.equal(made.length, 2);
});

const DISCONT = 0x02;
const auF = (v, buf, pts, flags, hv = null) => v.onAu(buf, pts, 1, flags, 1, 0, hv, -1, 0, 0);

test('onWantIdr: once at start, once on a DISCONT drop, not per AU', () => {
  globalThis.VideoDecoder = FakeDecoder;
  let t = 0;
  const realNow = performance.now;
  performance.now = () => t;
  try {
    const asks = [];
    const v = new VideoPipeline({ getCanvas: () => null, getMode: () => 'gs', onWantIdr: () => asks.push(t) });
    t = 0;   au(v, P, 1000);                     // unarmed from the start
    t = 16;  au(v, P, 17000);
    assert.deepEqual(asks, [0]);
    t = 32;  au(v, IRAP, 33000, hvcc);          // armed
    t = 48;  au(v, P, 49000);
    assert.deepEqual(asks, [0]);
    t = 64;  auF(v, P, 65000, DISCONT);         // drop edge: ask immediately
    t = 80;  au(v, P, 81000);
    assert.deepEqual(asks, [0, 64]);
    t = 364; au(v, P, 365000);                  // still unarmed 300 ms later: retry
    assert.deepEqual(asks, [0, 64, 364]);
  } finally { performance.now = realNow; }
});

test('onWantIdr: a decoder error drops the gate and asks once', () => {
  globalThis.VideoDecoder = FakeDecoder;
  made.length = 0;
  let t = 0;
  const realNow = performance.now;
  performance.now = () => t;
  try {
    const asks = [];
    const v = new VideoPipeline({ getCanvas: () => null, getMode: () => 'gs', onWantIdr: () => asks.push(t) });
    t = 0;   au(v, IRAP, 1000, hvcc);           // armed on the first AU: no ask
    assert.deepEqual(asks, []);
    t = 100; made[0].init.error(new Error('EncodingError'));
    assert.deepEqual(asks, [100]);
    t = 120; au(v, P, 121000);                  // inside retry window: no second ask
    assert.deepEqual(asks, [100]);
  } finally { performance.now = realNow; }
});

test('no onWantIdr given: the pipeline still runs (spotter / tests)', () => {
  globalThis.VideoDecoder = FakeDecoder;
  const v = new VideoPipeline({ getCanvas: () => null, getMode: () => 'spotter' });
  au(v, P, 1000);
  au(v, IRAP, 17000, hvcc);
  assert.equal(v.gateArmed(), true);
});

// Final review finding 1: a device-local decode failure (unsupported codec,
// reclaimed hardware decoder) must not hammer the drone with an IDR request
// every 300 ms forever -- the drone serving it never fixes anything local.

test('no VideoDecoder: pollIdr never asks -- nothing a key frame could fix', () => {
  delete globalThis.VideoDecoder;
  const asks = [];
  const v = new VideoPipeline({ getCanvas: () => null, getMode: () => 'gs', onWantIdr: () => asks.push(1) });
  au(v, P, 1000);
  au(v, IRAP, 17000, hvcc);
  au(v, P, 33000);
  assert.deepEqual(asks, []);
});

test('backoff: a decoder error after every key frame spreads retries out, not every 300 ms', () => {
  globalThis.VideoDecoder = FakeDecoder;
  made.length = 0;
  let t = 0;
  const realNow = performance.now;
  performance.now = () => t;
  try {
    const asks = [];
    const v = new VideoPipeline({ getCanvas: () => null, getMode: () => 'gs', onWantIdr: () => asks.push(t) });
    // A key frame only arrives in response to our own request (the drone
    // serves one IDR per idr_epoch bump -- docs/web-gs.md "Key-frame
    // recovery"), so a permanently-unsupported codec settles into one
    // ask-then-fail cycle per retry, not a free-running local loop. Space
    // cycles generously (5 s, past the 4.8 s cap) so each one's failure
    // always lands after its own backoff window has elapsed, and check the
    // resulting ask gaps grow well past the flat 300 ms retryMs.
    let asked = 0;
    for (let cycle = 0; cycle < 4; cycle++) {
      au(v, IRAP, 1000 + t, hvcc);           // re-arms (driven by the last ask)
      made[made.length - 1].init.error(new Error('NotSupportedError'));   // fails right after
      assert.equal(asks.length, asked + 1, `cycle ${cycle} should produce exactly one ask`);
      asked = asks.length;
      t += 5000;
    }
    const gaps = asks.slice(1).map((a, i) => a - asks[i]);
    assert.equal(gaps.length, 3);
    assert.ok(gaps[0] >= 600, `gap0 ${gaps[0]} should be >= 600 ms (n=1's interval)`);
    assert.ok(gaps[1] >= 1200, `gap1 ${gaps[1]} should be >= 1200 ms (n=2's interval)`);
    assert.ok(gaps[2] >= 2400, `gap2 ${gaps[2]} should be >= 2400 ms (n=3's interval)`);
  } finally { performance.now = realNow; }
});

test('mid-stream decoder error after output does not back off (the normal missing-reference case)', () => {
  globalThis.VideoDecoder = FakeDecoder;
  made.length = 0;
  let t = 0;
  const realNow = performance.now;
  performance.now = () => t;
  try {
    const asks = [];
    const v = new VideoPipeline({ getCanvas: () => null, getMode: () => 'gs', onWantIdr: () => asks.push(t) });
    t = 0; au(v, IRAP, 1000, hvcc);           // armed, decoder made[0]
    const frameStub = { timestamp: 1000, displayWidth: 2, displayHeight: 2, close() {} };
    made[0].init.output(frameStub);           // a real decode landed: n resets
    t = 50; au(v, P, 1050);                   // ordinary delta frame
    t = 100; made[0].init.error(new Error('missing reference'));   // mid-stream error
    assert.deepEqual(asks, [100]);            // immediate: no backoff pending
    t = 399; au(v, P, 1399);
    assert.deepEqual(asks, [100]);            // still gated at the base retryMs
    t = 400; au(v, P, 1400);
    assert.deepEqual(asks, [100, 400]);       // retry stayed 300 ms, not backed off
  } finally { performance.now = realNow; }
});

test('onWantIdr returning false is not counted: the next AU asks again immediately', () => {
  globalThis.VideoDecoder = FakeDecoder;
  let t = 0;
  const realNow = performance.now;
  performance.now = () => t;
  try {
    const asks = [];
    let swallow = true;
    const v = new VideoPipeline({
      getCanvas: () => null, getMode: () => 'gs',
      onWantIdr: () => { asks.push(t); return swallow ? false : undefined; },
    });
    t = 0; au(v, P, 1000);      // unarmed from the start: asks, but it's swallowed
    assert.deepEqual(asks, [0]);
    t = 1; au(v, P, 1001);      // the cancelled poll didn't count: asks again right away
    assert.deepEqual(asks, [0, 1]);
    swallow = false;            // now a real send (default onWantIdr's undefined return)
    t = 2; au(v, P, 1002);
    assert.deepEqual(asks, [0, 1, 2]);
    t = 3; au(v, P, 1003);      // that one counted: gated at retryMs now
    assert.deepEqual(asks, [0, 1, 2]);
  } finally { performance.now = realNow; }
});
