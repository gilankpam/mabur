// THROWAWAY SPIKE: page side -- gate, decode (WebCodecs, latency-first),
// draw, and the 1 Hz metrics that tie a visible hitch to its cause.
import { Gate, PtsUnwrap, PeriodEstimator, HitchMeter, pctl } from './viewer_logic.mjs';

const q = new URLSearchParams(location.search);
const ch = q.get('ch') || '136', width = q.get('w') || '40';
const canvas = document.getElementById('v'), ctx = canvas.getContext('2d');
const $m = document.getElementById('m'), $log = document.getElementById('log');
const log = (t) => { $log.textContent += t + '\n'; $log.scrollTop = 1e9; console.log(t); };

const gate = new Gate(), unwrap = new PtsUnwrap(), period = new PeriodEstimator(), hitch = new HitchMeter();
const now = () => performance.timeOrigin + performance.now();  // epoch ms, matches SPIKE stamps
let win = { in: 0, dec: 0, trunc: 0, gated: 0, late: [], decMs: [] };
const tot = { in: 0, dec: 0, trunc: 0, gated: 0, resets: 0, hitches: [] };
const submitted = new Map();  // chunk timestamp -> submit time (ms)
let decoder = null, lastStat = '';

function makeDecoder() {
  const d = new VideoDecoder({
    output: (frame) => {
      const t = now();
      const s = submitted.get(frame.timestamp);
      if (s !== undefined) { win.decMs.push(t - s); submitted.delete(frame.timestamp); }
      if (canvas.width !== frame.displayWidth) { canvas.width = frame.displayWidth; canvas.height = frame.displayHeight; }
      ctx.drawImage(frame, 0, 0);
      frame.close();
      win.dec++; tot.dec++;
      const gap = hitch.addDraw(t, period.periodMs());
      if (gap !== null) { tot.hitches.push(t); log(`HITCH at ${t.toFixed(1)} ms gap ${gap.toFixed(1)} ms`); }
    },
    error: (e) => { log('ERROR decoder ' + e.message); gate.onDecoderError(); tot.resets++; decoder = makeDecoder(); },
  });
  d.configure({ codec: 'hev1.1.6.L120.B0', hardwareAcceleration: 'prefer-hardware', optimizeForLatency: true });
  return d;
}

function onAu(m) {
  const t = now();
  win.in++; tot.in++;
  const pts = unwrap.add(m.pts);
  period.add(pts);
  win.late.push(t * 1000 - pts);  // µs; rebased per window below
  const g = gate.onAu({ sid: m.sid, flags: m.flags, complete: m.complete, data: new Uint8Array(m.buf) });
  if (g.reset) { tot.resets++; decoder.reset(); decoder = makeDecoder(); log('reset (DISCONT)'); }
  if (g.skip === 'truncated') { win.trunc++; tot.trunc++; }
  if (g.skip === 'gated') { win.gated++; tot.gated++; }
  if (!g.type) return;
  submitted.set(pts, t);
  try {
    decoder.decode(new EncodedVideoChunk({ type: g.type, timestamp: pts, data: m.buf }));
  } catch (e) {
    log('ERROR decode ' + e.message); gate.onDecoderError(); tot.resets++; decoder = makeDecoder();
  }
}

setInterval(() => {
  const late = win.late.length ? (() => { const mn = Math.min(...win.late); return win.late.map((x) => (x - mn) / 1000); })() : [];
  const lastMin = tot.hitches.filter((t) => t > now() - 60000).length;
  $m.textContent =
    `AUs/s in ${win.in} decoded ${win.dec} truncated ${win.trunc} gated ${win.gated} | ` +
    `total in ${tot.in} dec ${tot.dec} trunc ${tot.trunc} resets ${tot.resets}\n` +
    `AU lateness ms p50 ${pctl(late, 0.5).toFixed(1)} p99 ${pctl(late, 0.99).toFixed(1)} max ${pctl(late, 1).toFixed(1)} | ` +
    `decode ms p50 ${pctl(win.decMs, 0.5).toFixed(1)} max ${pctl(win.decMs, 1).toFixed(1)} | ` +
    `period ${(period.periodMs() || 0).toFixed(2)} ms | hitches last 60 s: ${lastMin} (total ${tot.hitches.length}) | ` +
    `decodeQueue ${decoder ? decoder.decodeQueueSize : '-'}\n${lastStat}`;
  console.log('METRIC ' + $m.textContent.replace(/\n/g, ' || '));
  win = { in: 0, dec: 0, trunc: 0, gated: 0, late: [], decMs: [] };
}, 1000);

document.getElementById('go').onclick = async () => {
  if (!crossOriginIsolated) { log('ERROR not crossOriginIsolated (serve with serve.py)'); return; }
  try {
    const granted = (await navigator.usb.getDevices()).filter((d) => d.vendorId === 0x0bda);
    const d = granted[0] || await navigator.usb.requestDevice({ filters: [{ vendorId: 0x0bda }] });
    log(`picked ${d.productName} ${d.vendorId.toString(16)}:${d.productId.toString(16)}`);
  } catch (e) { log('ERROR requestDevice ' + e); return; }
  decoder = makeDecoder();
  const w = new Worker('gsweb-worker.js');
  w.onmessage = ({ data: m }) => {
    if (m.t === 'au') onAu(m);
    else if (m.line.startsWith('STAT')) lastStat = m.line;
    else log(m.line);
  };
  w.onerror = (e) => log('ERROR worker ' + e.message);
  w.postMessage({ args: ['live', ch, width] });
};
