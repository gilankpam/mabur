// THROWAWAY SPIKE (branch wasm-spike): pure viewer logic, unit-tested in
// Node. Gate rules = maburplay's (gs/player/src/main.cpp sink + RingClient),
// fixed for the 2-stream space: arm on a complete AU carrying VPS+SPS+PPS
// (sid 0 alone no longer implies parameter sets).

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
