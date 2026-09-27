// mabur web GS page: mode picker, WebUSB connect, WebCodecs decode/draw,
// status overlay, latency/stats panel. Talks to the Emscripten module
// (webgs.js, loaded as a classic script -> global `createWebGs`) through
// Module.onAu/onStats/onError (web/src/web_main.cpp).
import {
  Gate, PtsUnwrap, PeriodEstimator, HitchMeter,
  annexbToLengthPrefixed, DecoderSlot, SegWindow, capToGlass, rcfHeardPct, errorText,
} from './logic.mjs';

// ---- elements ---------------------------------------------------------
const canvas = document.getElementById('v');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connect');
const chInput = document.getElementById('ch');
const widthSel = document.getElementById('w');
const statsTable = document.getElementById('statstable');
const copyBtn = document.getElementById('copystats');
const copyMsg = document.getElementById('copystats-msg');

// ---- localStorage (per brief: try/catch on every access) --------------
function loadLast() {
  try {
    const raw = localStorage.getItem('webgs.last');
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
}
function saveLast(v) {
  try {
    localStorage.setItem('webgs.last', JSON.stringify(v));
  } catch {
    /* ignore: page must work without localStorage */
  }
}

// ---- initial mode/ch/w: URL params > localStorage > hardcoded default -
const qs = new URLSearchParams(location.search);
const last = loadLast() || {};
const initMode = qs.get('mode') || last.mode || 'spotter';
const initCh = qs.get('ch') || last.ch || '136';
const initW = qs.get('w') || last.w || '40';

document.getElementById(initMode === 'gs' ? 'mode-gs' : 'mode-spotter').checked = true;
chInput.value = initCh;
widthSel.value = (initW === '20' || initW === '40') ? initW : '40';

function currentMode() {
  return document.querySelector('input[name=mode]:checked')?.value === 'gs' ? 'gs' : 'spotter';
}

// ---- per-connection state ----------------------------------------------
const now = () => performance.timeOrigin + performance.now();

let mode = currentMode();
let ch = chInput.value;
let w = widthSel.value;
let gsModule = null;
let connected = false;
let stopped = false;      // true after onError / worker failure: ignore AUs
let connectStartMs = 0;   // performance.now(): when the module actually started (fix round 1)
let waitingSince = 0;     // performance.now(): start of "waiting for key" (same instant)
let hiddenBannerShownOnce = false;
let hiddenBannerActive = false;

let gate, unwrap, period, hitch, decoderSlot, decoder, hvcc, configuredWith;
let submitted;          // pts (unwrapped) -> {tSubmit, tRecvMs, tEmitMs, capUs}
let segWindow;
let statsRing = [];     // last 60 raw JSON lines, for "Copy stats"
let lastStats = null;
let hitchTimes = [];     // epoch ms of each detected hitch (all-time; trimmed to last 60s view)
let hitchesTotal = 0;
let ausTotalRate = 0;    // AUs/s, delta of stats.aus between ticks

function resetPageState() {
  gate = new Gate();
  unwrap = new PtsUnwrap();
  period = new PeriodEstimator();
  hitch = new HitchMeter();
  decoderSlot = new DecoderSlot();
  decoder = decoderSlot.replace(makeDecoder());
  hvcc = null;
  configuredWith = null;
  submitted = new Map();
  segWindow = new SegWindow(now);
  statsRing = [];
  lastStats = null;
  hitchTimes = [];
  hitchesTotal = 0;
  ausTotalRate = 0;
  stopped = false;
}

// ---- decode -------------------------------------------------------------
function makeDecoder() {
  return new VideoDecoder({
    output: (frame) => {
      const t = now();
      const rec = submitted.get(frame.timestamp);
      if (rec) {
        segWindow.add('decode', t - rec.tSubmit);
        submitted.delete(frame.timestamp);
      }
      if (canvas.width !== frame.displayWidth || canvas.height !== frame.displayHeight) {
        canvas.width = frame.displayWidth;
        canvas.height = frame.displayHeight;
      }
      ctx.drawImage(frame, 0, 0);
      frame.close();
      const gap = hitch.addDraw(t, period.periodMs());
      if (gap !== null) {
        hitchTimes.push(t);
        hitchesTotal++;
      }
      requestAnimationFrame((ts) => {
        const presentMs = performance.timeOrigin + ts;
        segWindow.add('present', presentMs - t);
        if (rec && mode === 'gs') {
          const cap = capToGlass({
            capToCompleteUs: rec.capUs, tEmitMs: rec.tEmitMs,
            tRecvMs: rec.tRecvMs, tPresentMs: presentMs,
          });
          if (cap != null) segWindow.add('capture→glass (GS)', cap);
        }
      });
    },
    error: (e) => {
      console.error('[webgs] decoder error', e);
      gate.onDecoderError();
      decoder = decoderSlot.replace(makeDecoder());
    },
  });
}

// buf, pts_us, sid, flags, complete, t_complete_us, hvcc, capUs, tEmitMs,
// t_first_us -- the actual Module.onAu arg order (web/src/web_main.cpp
// emit_au), which overrides the brief's own sketch. t_first_us is the last
// argument (fix round 1 / Ruling R7): core clock, 0 when unknown.
function onAu(buf, ptsUs, sid, flags, complete, tCompleteUs, hvccBuf, capUs, tEmitMs, tFirstUs) {
  if (stopped) return;
  const tRecvMs = now();
  segWindow.add('handoff', tRecvMs - tEmitMs);
  if (tFirstUs > 0 && tCompleteUs > 0) {
    segWindow.add('fec', (tCompleteUs - tFirstUs) / 1000);
  }

  const data = new Uint8Array(buf);
  const g = gate.onAu({ sid, flags, complete: !!complete, data });
  if (g.reset) {
    waitingSince = performance.now();
    decoder = decoderSlot.replace(makeDecoder());
  }
  if (hvccBuf) hvcc = hvccBuf;
  updateOverlay();
  if (!g.type) return;

  const pts = unwrap.add(ptsUs);
  period.add(pts);

  if (g.type === 'key' && (decoder.state !== 'configured' || configuredWith !== hvcc)) {
    if (!hvcc) { gate.onDecoderError(); return; }
    decoder.configure({
      codec: 'hvc1.1.6.L120.B0', description: hvcc,
      hardwareAcceleration: 'prefer-hardware', optimizeForLatency: true,
    });
    configuredWith = hvcc;
  }
  submitted.set(pts, { tSubmit: now(), tRecvMs, tEmitMs, capUs });
  try {
    decoder.decode(new EncodedVideoChunk({
      type: g.type, timestamp: pts, data: annexbToLengthPrefixed(data),
    }));
  } catch (e) {
    console.error('[webgs] decode() threw', e);
    gate.onDecoderError();
    decoder = decoderSlot.replace(makeDecoder());
  }
}

// ---- stats ----------------------------------------------------------------
function onStats(text) {
  let s;
  try {
    s = JSON.parse(text);
  } catch (e) {
    console.error('[webgs] bad stats line', text, e);
    return;
  }
  statsRing.push(text);
  if (statsRing.length > 60) statsRing.shift();
  const prev = lastStats;
  ausTotalRate = prev ? Math.max(0, (s.aus ?? 0) - (prev.aus ?? 0)) : 0;
  const rcfPct = rcfHeardPct(prev, s);
  lastStats = s;
  renderStats(s, rcfPct);
  updateOverlay();
}

function fmt(v, digits = 1, suffix = '') {
  if (v === null || v === undefined || (typeof v === 'number' && Number.isNaN(v))) return '–';
  return Number(v).toFixed(digits) + suffix;
}

function segRow(name, w1) {
  const e = w1[name];
  if (!e || e.n === 0) return '–';
  return `p50 ${fmt(e.p50)} p99 ${fmt(e.p99)} max ${fmt(e.max)} (n=${e.n})`;
}

function renderStats(s, rcfPct) {
  const snap = segWindow.snapshot();
  const rows = [
    ['section', 'Link'],
    ['mode', s.mode],
    ['session', s.session ? 'yes' : 'no'],
    ['peer_acked', s.peer_acked ? 'yes' : 'no'],
    ['rung', s.rung],
    ['mcs', s.mcs],
    ['width', s.bw ? s.bw + ' MHz' : '–'],
    ['probe', s.probe || '–'],

    ['section', 'Radio'],
    ['pre-FEC loss', fmt(s.pre_fec_loss, 3)],
    ['residual', fmt(s.residual, 3)],
    ['SNR', fmt(s.snr_db, 1, ' dB')],
    ['RSSI', fmt(s.rssi_dbm, 1, ' dBm')],
    ['RTT', s.rtt_ms != null ? `${fmt(s.rtt_ms)} ms (min ${fmt(s.rtt_min_ms)} ms)` : '–'],
    ['RCF heard %', rcfPct != null ? fmt(rcfPct, 1, ' %') : '–'],

    ['section', 'Drone'],
    ['drone_state', s.drone_state ?? '–'],
    ['drone_rcf_rx', s.drone_rcf_rx ?? '–'],

    ['section', 'Client'],
    ['AUs/s', ausTotalRate],
    ['hitches', `${hitchTimes.filter((t) => t > now() - 60000).length} last 60s (total ${hitchesTotal})`],

    ['section', 'Latency (1 s window, ms)'],
    ['usb (core)', s.usb_p99_us != null
      ? `p99 ${fmt(s.usb_p99_us / 1000)} max ${fmt(s.usb_max_us / 1000)}`
      : '–'],
    ['fec (core, first→complete)', segRow('fec', snap.w1)],
    ['handoff', segRow('handoff', snap.w1)],
    ['decode', segRow('decode', snap.w1)],
    ['present', segRow('present', snap.w1)],
    ['capture→glass (GS)', mode === 'gs' ? segRow('capture→glass (GS)', snap.w1)
      : 'capture→glass needs the control link (GS mode)'],

    ['section', 'Counters'],
    ['bodies', s.bodies], ['aus', s.aus], ['trunc', s.trunc], ['sends', s.sends],
    ['txfail', s.txfail ?? '–'], ['qdrop', s.qdrop ?? '–'],
  ];

  statsTable.innerHTML = '';
  for (const [k, v] of rows) {
    const tr = document.createElement('tr');
    if (k === 'section') {
      tr.className = 'section';
      const td = document.createElement('td');
      td.colSpan = 2;
      td.textContent = v;
      tr.appendChild(td);
    } else {
      const tdK = document.createElement('td');
      tdK.textContent = k;
      const tdV = document.createElement('td');
      tdV.className = 'val';
      tdV.textContent = String(v);
      tr.appendChild(tdK);
      tr.appendChild(tdV);
    }
    statsTable.appendChild(tr);
  }
}

copyBtn.addEventListener('click', async () => {
  const text = statsRing.join('\n');
  try {
    await navigator.clipboard.writeText(text);
    copyMsg.textContent = 'copied';
  } catch (e) {
    try {
      const ta = document.createElement('textarea');
      ta.value = text;
      ta.style.position = 'fixed';
      ta.style.opacity = '0';
      document.body.appendChild(ta);
      ta.select();
      document.execCommand('copy');
      document.body.removeChild(ta);
      copyMsg.textContent = 'copied';
    } catch (e2) {
      console.error('[webgs] copy failed', e, e2);
      copyMsg.textContent = 'copy failed (see console)';
    }
  }
  setTimeout(() => { copyMsg.textContent = ''; }, 3000);
});

// ---- status overlay -------------------------------------------------------
function setStatusText(text, isErr) {
  statusEl.textContent = text;
  statusEl.classList.toggle('hidden', !text);
  statusEl.classList.toggle('err', !!isErr);
}

function updateOverlay() {
  if (stopped) return;    // error/worker-failure text stays until next Connect
  if (!connected) return; // idle text (blank, or a cancel/start-failure message) stays too
  const nowMs = performance.now();
  let primary = '';
  if (mode === 'gs' && lastStats && lastStats.peer_acked === false &&
      nowMs - connectStartMs >= 10000) {
    primary = `No drone on ch ${ch} / ${w} MHz (still trying)`;
  } else if (!gate || !gate.armed) {
    const secs = Math.max(0, Math.round((nowMs - waitingSince) / 1000));
    primary = `Waiting for key frame (sent on the next rung change) — ${secs} s`;
  }
  let text = primary;
  if (hiddenBannerActive) {
    text = text ? text + '\n' + 'GS mode keeps flying the link while this tab is hidden.'
                : 'GS mode keeps flying the link while this tab is hidden.';
  }
  setStatusText(text, false);
}

function showWorkerFailure() {
  if (stopped) return;
  stopped = true;
  setStatusText('Page worker failed — if served over LAN, the TLS CA is not trusted on '
                + 'this device (see docs/web-gs.md).', true);
  connectBtn.disabled = false;
  connectBtn.textContent = 'Connect';
  connected = false;
}

function onError(text) {
  console.error('[webgs] ' + text);
  stopped = true;
  setStatusText(errorText(text), true);
  connectBtn.disabled = false;
  connectBtn.textContent = 'Connect';
  connected = false;
}

window.addEventListener('error', (e) => {
  const msg = String((e && e.message) || '');
  console.error('[webgs] window error', e);
  // Precise match (same signature the printErr path checks), or an error
  // reported against the worker script itself -- not any message that
  // happens to mention the word "worker" (fix round 1).
  const fromWorkerFile = typeof e?.filename === 'string' && /webgs\.js|worker/i.test(e.filename);
  if (msg.includes('worker sent an error') || fromWorkerFile) showWorkerFailure();
});

document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    if (mode === 'gs' && connected && !hiddenBannerShownOnce) {
      hiddenBannerActive = true;
      hiddenBannerShownOnce = true;
    }
  } else {
    hiddenBannerActive = false;
  }
  updateOverlay();
});

// A periodic tick keeps the "waiting for key frame -- N s" / "no drone"
// countdowns moving even between AU/stats arrivals.
setInterval(updateOverlay, 1000);

// ---- connect / module lifecycle -------------------------------------------
async function startModule(modeArg, chArg, wArg) {
  mode = modeArg; ch = chArg; w = wArg;
  resetPageState();
  connected = false;   // not yet -- becomes true once the module actually starts, below;
                        // updateOverlay's `if (!connected) return;` keeps "Starting..." on
                        // screen (untouched by the 1 Hz tick) until then.
  hiddenBannerShownOnce = false;
  hiddenBannerActive = false;
  setStatusText('Starting…', false);
  try {
    gsModule = await createWebGs({
      arguments: ['live', '--mode', mode, '--ch', String(ch), '--w', String(w)],
      onAu, onStats, onError,
      print: (t) => console.log('[webgs]', t),
      printErr: (t) => {
        console.error('[webgs]', t);
        if (t.includes('worker sent an error')) showWorkerFailure();
      },
    });
    connectBtn.textContent = 'Connected';
    // The module is up now: start the "waiting for key frame" / "no drone"
    // clocks from here, not from the Connect click (fix round 1).
    connectStartMs = performance.now();
    waitingSince = connectStartMs;
    connected = true;
    updateOverlay();
  } catch (e) {
    console.error('[webgs] module start failed', e);
    setStatusText('ERROR starting module: ' + ((e && e.message) || e), true);
    connectBtn.disabled = false;
    connected = false;
  }
}

connectBtn.addEventListener('click', async () => {
  if (connectBtn.disabled) return;
  connectBtn.disabled = true;
  setStatusText('Requesting device…', false);

  const chosenMode = currentMode();
  const chosenCh = chInput.value || '136';
  const chosenW = widthSel.value || '40';

  try {
    if (!window.crossOriginIsolated) {
      throw new Error('page is not cross-origin isolated (serve with web/serve.py, not file://)');
    }
    if (!navigator.usb) throw new Error('WebUSB unavailable in this browser.');
    let granted = [];
    try {
      granted = (await navigator.usb.getDevices()).filter((d) => d.vendorId === 0x0bda);
    } catch {
      /* getDevices() failing is unusual; requestDevice below is the real gate */
    }
    if (!granted.length) {
      await navigator.usb.requestDevice({ filters: [{ vendorId: 0x0bda }] });
    }
  } catch (e) {
    console.warn('[webgs] device request did not complete', e);
    setStatusText('No device selected — press Connect to try again.', false);
    connectBtn.disabled = false;
    return;
  }

  saveLast({ mode: chosenMode, ch: chosenCh, w: chosenW });
  await startModule(chosenMode, chosenCh, chosenW);
});

// Initial overlay: nothing connected yet -- an idle hint, never the
// waiting-for-key/no-drone text (those need a live `gate`/`stats`, fix round 1).
setStatusText('Press Connect to begin.', false);
