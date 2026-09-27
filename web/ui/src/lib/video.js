// Decode + draw (port of the old web/www/app.js onAu/makeDecoder). Owns the
// IRAP gate, the WebCodecs decoder, the canvas draw, the page-side latency
// segments and the 200 ms page metrics. Never touched by Svelte per frame.
import {
  Gate, PtsUnwrap, PeriodEstimator, HitchMeter, annexbToLengthPrefixed, DecoderSlot, SegWindow,
  capToGlass, pruneSubmitted, trimBefore, isStalePresentSample,
} from './logic.mjs';
import { PageMetrics } from './metrics.js';

const now = () => performance.timeOrigin + performance.now();

export class VideoPipeline {
  constructor({ getCanvas, getMode }) {
    this.getCanvas = getCanvas;
    this.getMode = getMode;
    this.reset();
  }

  reset() {
    this.gate = new Gate();
    this.unwrap = new PtsUnwrap();
    this.period = new PeriodEstimator();
    this.hitch = new HitchMeter();
    this.decoderSlot = new DecoderSlot();
    this.submitted = new Map();
    this.decoder = this.decoderSlot.replace(this.makeDecoder());
    this.hvcc = null;
    this.configuredWith = null;
    this.segWindow = new SegWindow(now);
    this.metrics = new PageMetrics();
    this.hitchTimes = [];
    this.hitchesTotal = 0;
    this.videoSize = null;
    this.waitingSinceMs = performance.now();
  }

  gateArmed() { return this.gate.armed; }
  periodMs() { return this.period.periodMs(); }

  // A replaced decoder never outputs its queued chunks: drop their records.
  replaceDecoder() {
    this.submitted.clear();
    this.decoder = this.decoderSlot.replace(this.makeDecoder());
  }

  makeDecoder() {
    return new VideoDecoder({
      output: (frame) => {
        const t = now();
        const rec = this.submitted.get(frame.timestamp);
        if (rec) {
          this.segWindow.add('decode', t - rec.tSubmit);
          this.submitted.delete(frame.timestamp);
        }
        this.videoSize = { w: frame.displayWidth, h: frame.displayHeight };
        const canvas = this.getCanvas();
        if (canvas) {
          if (canvas.width !== frame.displayWidth || canvas.height !== frame.displayHeight) {
            canvas.width = frame.displayWidth;
            canvas.height = frame.displayHeight;
          }
          canvas.getContext('2d').drawImage(frame, 0, 0);
        }
        frame.close();
        this.metrics.addDraw(t);
        const gap = this.hitch.addDraw(t, this.period.periodMs());
        if (gap !== null) {
          this.hitchTimes.push(t);
          trimBefore(this.hitchTimes, t - 60000);
          this.hitchesTotal++;
        }
        // Hidden tab: rAF is paused, so a callback queued now fires only when
        // the tab returns and would book a minutes-long "present" sample. No
        // glass while hidden -> no present / capture->glass sample.
        if (document.hidden) return;
        requestAnimationFrame((ts) => {
          const presentMs = performance.timeOrigin + ts;
          // F3: an rAF queued just before the tab went hidden fires only once
          // it's visible again -- not a real present sample.
          if (isStalePresentSample(t, presentMs)) return;
          this.segWindow.add('present', presentMs - t);
          if (rec && this.getMode() === 'gs') {
            const cap = capToGlass({ capToCompleteUs: rec.capUs, tEmitMs: rec.tEmitMs,
              tRecvMs: rec.tRecvMs, tPresentMs: presentMs });
            if (cap != null) this.segWindow.add('capture→glass (GS)', cap);
          }
        });
      },
      error: (e) => {
        console.error('[webgs] decoder error', e);
        this.gate.onDecoderError();
        this.replaceDecoder();
      },
    });
  }

  // Module.onAu argument order is web/src/web_main.cpp emit_au's.
  onAu(buf, ptsUs, sid, flags, complete, tCompleteUs, hvccBuf, capUs, tEmitMs, tFirstUs) {
    const tRecvMs = now();
    this.segWindow.add('handoff', tRecvMs - tEmitMs);
    if (tFirstUs > 0 && tCompleteUs > 0) this.segWindow.add('fec', (tCompleteUs - tFirstUs) / 1000);
    this.metrics.addAu(tRecvMs, buf.byteLength);

    const data = new Uint8Array(buf);
    const g = this.gate.onAu({ sid, flags, complete: !!complete, data });
    if (g.reset) {
      this.waitingSinceMs = performance.now();
      this.replaceDecoder();
    }
    if (hvccBuf) this.hvcc = hvccBuf;
    if (!g.type) return;

    const pts = this.unwrap.add(ptsUs);
    this.period.add(pts);
    if (g.type === 'key' && (this.decoder.state !== 'configured' || this.configuredWith !== this.hvcc)) {
      if (!this.hvcc) { this.gate.onDecoderError(); return; }
      this.decoder.configure({
        codec: 'hvc1.1.6.L120.B0', description: this.hvcc,
        hardwareAcceleration: 'prefer-hardware', optimizeForLatency: true,
      });
      this.configuredWith = this.hvcc;
    }
    const tSubmit = now();
    pruneSubmitted(this.submitted, tSubmit, 1000);   // dropped/never-output chunks
    this.submitted.set(pts, { tSubmit, tRecvMs, tEmitMs, capUs });
    try {
      this.decoder.decode(new EncodedVideoChunk({ type: g.type, timestamp: pts, data: annexbToLengthPrefixed(data) }));
    } catch (e) {
      console.error('[webgs] decode() threw', e);
      this.gate.onDecoderError();
      this.replaceDecoder();
    }
  }

  close() { this.decoderSlot.replace(null); }
}
