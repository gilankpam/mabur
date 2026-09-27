// mabur web GS page: mode picker, WebUSB connect, WebCodecs decode/draw,
// status overlay, latency/stats panel. Talks to the Emscripten module
// (webgs.js, loaded as a classic script -> global `createWebGs`) through
// Module.onAu/onStats/onError (web/src/web_main.cpp).
import {
  Gate, PtsUnwrap, PeriodEstimator, HitchMeter,
  annexbToLengthPrefixed, DecoderSlot, SegWindow, capToGlass, rcfHeardPctWindowed, errorText,
  checkChannelWidth, pruneSubmitted, trimBefore, copyStatsPayload, controlsLocked,
  isStalePresentSample,
} from './logic.mjs';

// ---- elements ---------------------------------------------------------
const canvas = document.getElementById('v');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connect');
const modeField = document.getElementById('modefield');
const chInput = document.getElementById('ch');
const widthSel = document.getElementById('w');
const lockHint = document.getElementById('lockhint');
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
let connecting = false;   // true from Connect click to the resolved/rejected module start (F1/R12)
let connected = false;
let stopped = false;      // true after onError / worker failure: ignore AUs
let connectStartMs = 0;   // performance.now(): when the module actually started (fix round 1)
let waitingSince = 0;     // performance.now(): start of "waiting for key" (same instant)
let hiddenBannerShownOnce = false;
let hiddenBannerActive = false;

let gate, unwrap, period, hitch, decoderSlot, decoder, hvcc, configuredWith;
let submitted;          // pts (unwrapped) -> {tSubmit, tRecvMs, tEmitMs, capUs};
                        // cleared on decoder replace, entries > 1 s pruned
let segWindow;
let statsRing = [];     // last 60 raw JSON lines, for "Copy stats"
let rcfRing = [];       // last 60 parsed stats ticks, for the 10 s RCF-heard window (F2)
let lastStats = null;
let hitchTimes = [];     // epoch ms of each detected hitch, trimmed to the last 60 s
let hitchesTotal = 0;
let ausTotalRate = 0;    // AUs/s, delta of stats.aus between ticks

function resetPageState() {
  gate = new Gate();
  unwrap = new PtsUnwrap();
  period = new PeriodEstimator();
  hitch = new HitchMeter();
  decoderSlot = new DecoderSlot();
  submitted = new Map();
  decoder = decoderSlot.replace(makeDecoder());
  hvcc = null;
  configuredWith = null;
  segWindow = new SegWindow(now);
  statsRing = [];
  rcfRing = [];
  lastStats = null;
  hitchTimes = [];
  hitchesTotal = 0;
  ausTotalRate = 0;
  stopped = false;
}

// ---- decode -------------------------------------------------------------
// A replaced decoder never outputs its queued chunks: drop their records.
function replaceDecoder() {
  submitted.clear();
  decoder = decoderSlot.replace(makeDecoder());
}

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
        trimBefore(hitchTimes, t - 60000);
        hitchesTotal++;
      }
      // Hidden tab: rAF is paused, so a callback queued now fires only when
      // the tab returns and would book a minutes-long "present" sample. No
      // glass while hidden -> no present / capture->glass sample.
      if (document.hidden) return;
      requestAnimationFrame((ts) => {
        const presentMs = performance.timeOrigin + ts;
        // F3: an rAF queued just before the tab went hidden fires only once
        // it's visible again, ~seconds to minutes later -- not a real
        // present sample. Drop it rather than let it skew p99/max.
        if (isStalePresentSample(t, presentMs)) return;
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
      replaceDecoder();
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
    replaceDecoder();
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
  const tSubmit = now();
  pruneSubmitted(submitted, tSubmit, 1000);   // dropped/never-output chunks
  submitted.set(pts, { tSubmit, tRecvMs, tEmitMs, capUs });
  try {
    decoder.decode(new EncodedVideoChunk({
      type: g.type, timestamp: pts, data: annexbToLengthPrefixed(data),
    }));
  } catch (e) {
    console.error('[webgs] decode() threw', e);
    gate.onDecoderError();
    replaceDecoder();
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
  rcfRing.push(s);
  if (rcfRing.length > 60) rcfRing.shift();
  // F2: a 10 s window, not the adjacent tick -- see rcfHeardPctWindowed.
  const rcfPct = rcfHeardPctWindowed(rcfRing);
  lastStats = s;
  renderStats(s, rcfPct);
  updateOverlay();
}

function fmt(v, digits = 1, suffix = '') {
  if (v === null || v === undefined || (typeof v === 'number' && Number.isNaN(v))) return '–';
  return Number(v).toFixed(digits) + suffix;
}

function segCell(e) {
  if (!e || e.n === 0) return '–';
  return `p50 ${fmt(e.p50)} p99 ${fmt(e.p99)} max ${fmt(e.max)} (n=${e.n})`;
}

// Spec §4: every segment row shows the 1 s and the 60 s window.
function segRow(name, snap) {
  const a = snap.w1[name], b = snap.w60[name];
  if ((!a || a.n === 0) && (!b || b.n === 0)) return '–';
  return `1 s: ${segCell(a)} | 60 s: ${segCell(b)}`;
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
    ['hitches', `${trimBefore(hitchTimes, now() - 60000).length} last 60s (total ${hitchesTotal})`],

    ['section', 'Latency (ms; 1 s | 60 s windows)'],
    ['usb (core, 1 s)', s.usb_p99_us != null
      ? `p99 ${fmt(s.usb_p99_us / 1000)} max ${fmt(s.usb_max_us / 1000)}`
      : '–'],
    ['fec (core, first→complete)', segRow('fec', snap)],
    ['handoff', segRow('handoff', snap)],
    ['decode', segRow('decode', snap)],
    ['present', segRow('present', snap)],
    ['capture→glass (GS)', mode === 'gs' ? segRow('capture→glass (GS)', snap)
      : 'capture→glass needs the control link (GS mode)'],

    ['section', 'Counters'],
    ['bodies', s.bodies], ['aus', s.aus], ['trunc', s.trunc], ['sends', s.sends],
    ['rcf_sent', s.rcf_sent ?? '–'],
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
  // {core: last 60 s of the core's 1 Hz stats, segments: page-side w1/w60}.
  const text = copyStatsPayload(statsRing, segWindow ? segWindow.snapshot() : { w1: {}, w60: {} });
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

// R12 / F1: lock the mode radios and channel/width inputs while a connect
// attempt is in flight or a module is connected -- the running module
// doesn't observe a later change, so leaving them live invites fighting
// maburgs after switching mode mid-flight (bench finding). Every path that
// re-enables the Connect button also flips `connecting`/`connected` back
// and calls this.
function updateControlsLock() {
  const locked = controlsLocked({ connecting, connected });
  modeField.disabled = locked;
  chInput.disabled = locked;
  widthSel.disabled = locked;
  lockHint.hidden = !locked;
}

function showWorkerFailure() {
  if (stopped) return;
  stopped = true;
  setStatusText('Page worker failed — if served over LAN, the TLS CA is not trusted on '
                + 'this device (see docs/web-gs.md).', true);
  connectBtn.disabled = false;
  connectBtn.textContent = 'Connect';
  connected = false;
  connecting = false;
  updateControlsLock();
}

function onError(text) {
  console.error('[webgs] ' + text);
  stopped = true;
  setStatusText(errorText(text), true);
  connectBtn.disabled = false;
  connectBtn.textContent = 'Connect';
  connected = false;
  connecting = false;
  updateControlsLock();
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
  // Lock the mode/channel/width controls for the whole attempt (R12/F1):
  // every early-return below restores them, and a successful startModule
  // keeps them locked via `connected`.
  connecting = true;
  updateControlsLock();
  setStatusText('Requesting device…', false);

  const chosenMode = currentMode();
  const chosenCh = (chInput.value || '').trim();
  const chosenW = widthSel.value || '40';
  // Refuse a bad channel/width here with a clear message; the glue re-checks
  // (and adds the GS-mode ladder check) as `ERROR bad channel/width: ...`.
  const bad = checkChannelWidth(chosenCh, chosenW);
  if (bad) {
    setStatusText(bad, true);
    connectBtn.disabled = false;
    connecting = false;
    updateControlsLock();
    return;
  }

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
    connecting = false;
    updateControlsLock();
    return;
  }

  saveLast({ mode: chosenMode, ch: chosenCh, w: chosenW });
  await startModule(chosenMode, chosenCh, chosenW);
  // startModule resolves `connected` either way (true on success, false on
  // module-start failure); either way the attempt is no longer "in flight".
  connecting = false;
  updateControlsLock();
});

// Initial overlay: nothing connected yet -- an idle hint, never the
// waiting-for-key/no-drone text (those need a live `gate`/`stats`, fix round 1).
setStatusText('Press Connect to begin.', false);
