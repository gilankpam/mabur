// Page-side video metrics sampled every 200 ms (spec §4.2): the core's stats
// are 1 Hz, the handoff's refresh is 200 ms. Trailing 1 s windows.
const WINDOW_MS = 1000;

export class PageMetrics {
  constructor() { this.aus = []; this.draws = []; }
  addAu(tMs, bytes) { this.aus.push({ t: tMs, b: bytes }); }
  addDraw(tMs) { this.draws.push(tMs); }
  sample(nowMs, periodMs) {
    const cut = nowMs - WINDOW_MS;
    while (this.aus.length && this.aus[0].t <= cut) this.aus.shift();
    while (this.draws.length && this.draws[0] <= cut) this.draws.shift();
    const bitrateMbps = this.aus.length ? this.aus.reduce((n, a) => n + a.b, 0) * 8 / 1e6 : null;
    const fps = this.draws.length ? this.draws.length : null;
    let jitterMs = null;
    if (this.draws.length >= 2 && periodMs) {
      let sum = 0;
      for (let i = 1; i < this.draws.length; i++) sum += Math.abs(this.draws[i] - this.draws[i - 1] - periodMs);
      jitterMs = sum / (this.draws.length - 1);
    }
    return { bitrateMbps, fps, jitterMs };
  }
}

// Prototype's sparkline: 100x24 viewBox, headroom x1.15, nulls skipped.
export function sparkPoints(values, n = 60) {
  const vals = values.filter((v) => v != null && Number.isFinite(v));
  if (!vals.length) return '0,24 100,24';
  const mx = Math.max(1, ...vals) * 1.15;
  const out = [];
  values.forEach((v, i) => {
    if (v == null || !Number.isFinite(v)) return;
    out.push(`${(i / (n - 1) * 100).toFixed(1)},${(24 - v / mx * 22).toFixed(1)}`);
  });
  return out.join(' ');
}
