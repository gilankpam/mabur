// Pure, node-testable logic for the web-GS page (promoted from the
// wasm-spike throwaway). Gate rules = maburplay's (gs/player/src/main.cpp
// sink + RingClient), fixed for the 2-stream space: arm on a complete AU
// carrying VPS+SPS+PPS (sid 0 alone no longer implies parameter sets).

export function nalTypes(u8) {
  const out = [];
  for (let i = 0; i + 3 < u8.length; i++) {
    if (u8[i] === 0 && u8[i + 1] === 0) {
      if (u8[i + 2] === 1) { out.push((u8[i + 3] >> 1) & 0x3f); i += 3; }
      else if (u8[i + 2] === 0 && u8[i + 3] === 1 && i + 4 < u8.length) {
        out.push((u8[i + 4] >> 1) & 0x3f); i += 4;
      }
    }
  }
  return out;
}

const VPS = 32, SPS = 33, PPS = 34, DISCONT = 0x02;

export class Gate {
  constructor() { this.armed = false; }
  onAu({ flags, complete, data }) {
    let reset = false;
    if ((flags & DISCONT) && this.armed) { this.armed = false; reset = true; }
    if (!complete) return { type: null, reset, skip: 'truncated' };
    if (!this.armed) {
      const t = nalTypes(data);
      if (t.includes(VPS) && t.includes(SPS) && t.includes(PPS)) {
        // WebCodecs rejects a non-IRAP first key chunk (Annex-B and hvcC
        // alike), and the drone is GDR: its parameter sets usually ride a
        // TRAIL_R refresh. Arm only on an IRAP (BLA/IDR/CRA, types 16-21).
        if (!t.some((x) => x >= 16 && x <= 21)) return { type: null, reset, skip: 'awaiting-irap' };
        this.armed = true;
        return { type: 'key', reset, skip: null };
      }
      return { type: null, reset, skip: 'gated' };
    }
    return { type: 'delta', reset, skip: null };
  }
  onDecoderError() { this.armed = false; }
}

export class PtsUnwrap {
  constructor() { this.hi = 0; this.last = null; }
  add(pts) {
    if (this.last !== null && pts < this.last && this.last - pts > 0x80000000) this.hi += 0x100000000;
    this.last = pts;
    return this.hi + pts;
  }
}

export class PeriodEstimator {
  constructor() { this.prev = null; this.deltas = []; }
  add(ptsUs) {
    if (this.prev !== null && ptsUs > this.prev) {
      this.deltas.push(ptsUs - this.prev);
      if (this.deltas.length > 64) this.deltas.shift();
    }
    this.prev = ptsUs;
  }
  periodMs() {
    if (!this.deltas.length) return null;
    return pctl(this.deltas, 0.5) / 1000;
  }
}

export class HitchMeter {
  constructor() { this.last = null; }
  addDraw(tMs, periodMs) {
    const gap = this.last === null ? null : tMs - this.last;
    this.last = tMs;
    return gap !== null && periodMs && gap > 1.5 * periodMs ? gap : null;
  }
}

export function pctl(values, q) {
  if (!values.length) return 0;
  const s = [...values].sort((a, b) => a - b);
  return s[Math.min(s.length - 1, Math.floor(s.length * q))];
}

// Annex-B -> 4-byte big-endian length-prefixed NAL units (the hvcC framing
// WebCodecs expects once a `description` is configured).
export function annexbToLengthPrefixed(u8) {
  const starts = [];  // [payloadStart, startCodeStart]
  for (let i = 0; i + 2 < u8.length; i++) {
    if (u8[i] === 0 && u8[i + 1] === 0 && u8[i + 2] === 1) {
      starts.push([i + 3, i > 0 && u8[i - 1] === 0 ? i - 1 : i]);
      i += 2;
    }
  }
  const nals = starts.map(([s], k) => u8.subarray(s, k + 1 < starts.length ? starts[k + 1][1] : u8.length));
  const out = new Uint8Array(nals.reduce((n, x) => n + 4 + x.length, 0));
  let o = 0;
  for (const x of nals) {
    new DataView(out.buffer).setUint32(o, x.length);
    out.set(x, o + 4);
    o += 4 + x.length;
  }
  return out;
}

// Holds the live VideoDecoder; replacing it closes the old one (a rejected
// or reset decoder left open pins a hardware decode slot until GC).
export class DecoderSlot {
  constructor() { this.d = null; }
  replace(next) {
    if (this.d && this.d.state !== 'closed') this.d.close();
    this.d = next;
    return next;
  }
}

// Rolling per-segment latency stats (e.g. 'decode', 'present') over a 1 s
// and a 60 s trailing window, both measured back from `snapshot`'s `now`
// (default: nowFn()). Entries older than 60 s are pruned as they age out
// on add, so the backing arrays never grow unbounded.
export class SegWindow {
  constructor(nowFn) {
    this.nowFn = nowFn || (() => Date.now());
    this.byName = new Map();
  }
  add(name, ms) {
    const now = this.nowFn();
    let arr = this.byName.get(name);
    if (!arr) { arr = []; this.byName.set(name, arr); }
    arr.push({ t: now, v: ms });
    const cutoff60 = now - 60000;
    let drop = 0;
    while (drop < arr.length && arr[drop].t <= cutoff60) drop++;
    if (drop > 0) arr.splice(0, drop);
  }
  snapshot(now) {
    if (now === undefined) now = this.nowFn();
    const cutoff1 = now - 1000;
    const cutoff60 = now - 60000;
    const w1 = {}, w60 = {};
    for (const [name, arr] of this.byName) {
      const v1 = [], v60 = [];
      for (const e of arr) {
        if (e.t > cutoff60) v60.push(e.v);
        if (e.t > cutoff1) v1.push(e.v);
      }
      w1[name] = summarize(v1);
      w60[name] = summarize(v60);
    }
    return { w1, w60 };
  }
}

function summarize(values) {
  return {
    p50: pctl(values, 0.5),
    p99: pctl(values, 0.99),
    max: values.length ? Math.max(...values) : 0,
    n: values.length,
  };
}

// Capture (drone encode-complete) to glass (present) latency, ms:
// cap-to-complete (drone-side, us) plus the two GS-side hop deltas. null
// when the drone didn't report a cap-to-complete offset (absent, or a
// negative sentinel meaning "no measurement").
export function capToGlass({ capToCompleteUs, tEmitMs, tRecvMs, tPresentMs }) {
  if (capToCompleteUs == null || capToCompleteUs < 0) return null;
  return capToCompleteUs / 1000 + (tRecvMs - tEmitMs) + (tPresentMs - tRecvMs);
}

// % of drone->GS RCF sends the drone reports having heard, over the delta
// between two stats snapshots. null when either snapshot is missing, either
// is missing drone_rcf_rx/sends, sends didn't advance, or drone_rcf_rx went
// backwards (counter reset, e.g. drone restart) rather than wrapping cleanly.
export function rcfHeardPct(prev, cur) {
  if (!prev || !cur) return null;
  if (prev.drone_rcf_rx == null || cur.drone_rcf_rx == null) return null;
  if (prev.sends == null || cur.sends == null) return null;
  const dSends = cur.sends - prev.sends;
  if (!(dSends > 0)) return null;
  const dRcf = cur.drone_rcf_rx - prev.drone_rcf_rx;
  if (dRcf < 0) return null;
  return 100 * dRcf / dSends;
}

// Maps a glue `ERROR ...` line (web/src/web_gs.cpp) to user-facing text.
export function errorText(line) {
  if (line.includes('no RTL card')) {
    return 'No RTL8812EU/8812AU card found — plug it in and press Connect.';
  }
  if (line.includes('claim failed')) {
    return 'Card busy — maburgs or another tab has it. Close that and press Connect.';
  }
  if (line.includes('card lost')) {
    return 'Card lost (unplugged?). Press Connect to restart.';
  }
  if (line.includes('libusb_init')) {
    return 'WebUSB unavailable in this browser.';
  }
  if (line.includes('unsupported chip')) {
    return 'Unsupported card chip.';
  }
  return line;
}
