import test from 'node:test';
import assert from 'node:assert/strict';

// video.js is browser code; stub the few globals its per-AU path touches.
globalThis.performance ??= { timeOrigin: 0, now: () => Date.now() };
globalThis.document ??= { hidden: false };
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
