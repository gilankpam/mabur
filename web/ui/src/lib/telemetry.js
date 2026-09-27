// Core 1 Hz stats + 200 ms page samples (spec §3.4/§4.2). Holds the rings the
// old page kept (60 raw lines for Copy stats, 60 parsed ticks for the 10 s
// RCF-heard window) and the 60-sample latency sparkline (12 s @ 200 ms).
import { rcfHeardPctWindowed, copyStatsPayload, trimBefore } from './logic.mjs';
import { latencyNow } from './view.js';

export class Telemetry {
  constructor(video) { this.video = video; this.reset(); }
  reset() {
    this.statsRing = [];
    this.rcfRing = [];
    this.core = null;
    this.rcfPct = null;
    this.ausRate = 0;
    this.page = null;
    this.spark = [];
    this.version = 0;
  }
  onCoreStats(text, s) {
    this.statsRing.push(text);
    if (this.statsRing.length > 60) this.statsRing.shift();
    this.ausRate = this.core ? Math.max(0, (s.aus ?? 0) - (this.core.aus ?? 0)) : 0;
    this.rcfRing.push(s);
    if (this.rcfRing.length > 60) this.rcfRing.shift();
    // F2: a 10 s window, not the adjacent tick -- see rcfHeardPctWindowed.
    this.rcfPct = rcfHeardPctWindowed(this.rcfRing);
    this.core = s;
    this.version++;
  }
  sample(nowMs, mode) {
    const m = this.video.metrics.sample(nowMs, this.video.periodMs());
    const snap = this.video.segWindow.snapshot();
    const latencyMs = latencyNow(snap, mode);
    this.page = { ...m, latencyMs };
    this.seg = snap;
    this.spark.push(latencyMs);
    if (this.spark.length > 60) this.spark.shift();
    this.version++;
  }
  hitches60(nowMs) { return trimBefore(this.video.hitchTimes, nowMs - 60000).length; }
  copyPayload() {
    return copyStatsPayload(this.statsRing, this.video.segWindow.snapshot());
  }
}
