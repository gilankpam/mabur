<script>
  import { onMount } from 'svelte';
  import Header from './Header.svelte';
  import Sidebar from './Sidebar.svelte';
  import ConfigPanel from './ConfigPanel.svelte';
  import Recordings from './Recordings.svelte';
  import DisconnectedOverlay from './DisconnectedOverlay.svelte';
  import FsOverlay from './FsOverlay.svelte';
  import FloatStats from './FloatStats.svelte';
  import ConfigSide from './ConfigSide.svelte';
  import { ui, saveMode, reloadToConfig } from '../lib/ui.svelte.js';
  import { Session, WORKER_FAILED } from '../lib/session.js';
  import { VideoPipeline } from '../lib/video.js';
  import { Telemetry } from '../lib/telemetry.js';
  import { statsView, debugGroups, statusText, linkTag } from '../lib/view.js';
  import { recView, RecClock, formatClock } from '../lib/rec.js';
  import { sparkPoints } from '../lib/metrics.js';
  import { layoutMode, keyAction, isMobile, uiFrame } from '../lib/layout.js';
  import { connectBlocker, toOverlayToml, saveConfig } from '../lib/config.js';
  import { effectiveTarget, targetCovers, recFileName, localView, combinedRec, headroomWarning,
    formatBytes, listRecordings, downloadRecording, deleteRecording, opfsRoot } from '../lib/localrec.js';
  import { OsdLayer, OsdPainter } from '../lib/osd.js';
  import { ScreenWake } from '../lib/wakelock.js';

  let canvas = $state(null);
  let glCanvas = $state(null);
  let osdCanvas = $state(null);
  let W = $state(innerWidth), H = $state(innerHeight);
  let sess = $state({ state: 'idle', mode: ui.mode, ch: null, w: null, error: null, notice: null, startedAt: null, recWish: false });
  let sessionCfg = $state(ui.cfg);        // the config the running session started with
  let copyMsg = $state('');
  let hiddenBanner = $state(false), hiddenShown = false;
  const recClock = new RecClock();
  const wake = new ScreenWake();

  let recItems = $state([]), recStorage = $state(''), recWarn = $state(null), opfsOk = $state(true);
  let lrec = $state(null), lrecPrevBytes = 0, lrecPrevT = 0;
  let persistAsked = false;
  async function refreshRecordings() {
    const root = await opfsRoot();
    if (!root) { opfsOk = false; recItems = []; return; }
    try { recItems = await listRecordings(root); } catch (e) { console.error('[webgs] list recordings', e); }
    try {
      const est = await navigator.storage.estimate();
      recStorage = `${formatBytes(est.usage || 0)} used · ${formatBytes(Math.max(0, (est.quota || 0) - (est.usage || 0)))} free`;
      recWarn = headroomWarning(est);
    } catch { recStorage = ''; recWarn = null; }
  }
  async function onRecClosed(name, bytes, err) {
    if (bytes > 0) {
      const root = await opfsRoot();
      if (root) {
        try { await downloadRecording(root, name); }
        catch (e) { console.error('[webgs] download', name, e); }
      }
    }
    if (err) console.error('[webgs] local recording ended with error', err, name);
    refreshRecordings();
  }
  async function delRecording(name) {
    const root = await opfsRoot();
    if (!root) return;
    try { await deleteRecording(root, name); } catch (e) { console.error('[webgs] delete', name, e); }
    refreshRecordings();
  }
  async function dlRecording(name) {
    const root = await opfsRoot();
    if (root) try { await downloadRecording(root, name); } catch (e) { console.error('[webgs] download', name, e); }
  }

  const video = new VideoPipeline({ getCanvas: () => canvas, getGlCanvas: () => glCanvas, getMode: () => sess.mode,
    getColortrans: () => ui.cfg.colortrans, onWantIdr: () => session.requestIdr() });
  const tele = new Telemetry(video);
  // MSP OSD (spec 2026-09-27-web-msp-osd): latest DisplayPort grid, painted on
  // a second canvas over the video. The atlas URL resolves against the page
  // (works under the Pages /mabur/ base as well as at a local root).
  const osdLayer = new OsdLayer();
  const osd = new OsdPainter({
    layer: osdLayer,
    loadAtlas: async () => {
      const r = await fetch(new URL('font_btfl.png', document.baseURI));
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      return createImageBitmap(await r.blob());
    },
  });
  function clearOsd() { if (osdLayer.screen) { osdLayer.clear(); osd.schedule(); } }

  const session = new Session({
    createModule: (opts) => window.createWebGs(opts),
    requestDevice: async () => {
      if (!window.crossOriginIsolated) throw new Error('Page is not cross-origin isolated (COOP/COEP headers missing) — serve it as docs/web-gs.md describes.');
      if (!navigator.usb) throw new Error('WebUSB unavailable in this browser — use Chrome or Edge.');
      let granted = [];
      try { granted = (await navigator.usb.getDevices()).filter((d) => d.vendorId === 0x0bda); } catch { /* requestDevice is the real gate */ }
      if (!granted.length) await navigator.usb.requestDevice({ filters: [{ vendorId: 0x0bda }] });
    },
    // Plain-JS state read (no Svelte on the per-frame path): a stopping or
    // failed module's straggler AUs must not rebuild the decoder close() freed.
    onAu: (...a) => { const st = session.snapshot.state; if (st === 'live' || st === 'connecting') video.onAu(...a); },
    onStats: (text, s) => { tele.onCoreStats(text, s); ui.tick++; },
    // Same live/connecting gate as onAu: a stopping module's straggler
    // screens must not repaint after Disconnect cleared the layer.
    onOsd: (rows, cols, cells) => {
      const st = session.snapshot.state;
      if (st !== 'live' && st !== 'connecting') return;
      osdLayer.onScreen(rows, cols, cells, performance.now());
      osd.schedule();
    },
    onRecClosed: (n, b, e) => onRecClosed(n, b, e),
    reload: reloadToConfig,
  });

  let prevState = 'idle';
  session.subscribe((s) => {
    sess = s;
    // Left live (stopping/idle/error): free the hardware decoder now.
    if (s.state !== 'live' && s.state !== 'connecting') { video.close(); clearOsd(); }
    wake.set(s.state === 'live' || s.state === 'connecting');
    if (s.state === 'live' && prevState !== 'live') { ui.tab = 'stats'; ui.cfgOpen = false; ui.statsVisible = true; }
    if (prevState === 'stopping' && s.state === 'idle') { ui.tab = 'config'; ui.cfgOpen = true; }
    if (s.state === 'error' && prevState === 'live') { ui.tab = 'config'; }
    prevState = s.state;
  });

  // Upright phone: the UI is drawn rotated (layout.js uiFrame). LW x LH is
  // its own landscape size; everything below lays out in it, not in W x H.
  const frame = $derived(uiFrame(W, H));
  const LW = $derived(frame.lw), LH = $derived(frame.lh);
  const layout = $derived(layoutMode({ w: LW, h: LH, fs: ui.fs }));
  const live = $derived(sess.state === 'live');
  const busy = $derived(sess.state === 'connecting' || sess.state === 'stopping');
  const shownMode = $derived(live || busy ? sess.mode : ui.mode);
  const chLine = $derived(live || busy ? `${sess.ch} · ${sess.w} MHz` : `${ui.cfg.channel} · ${ui.cfg.width} MHz`);
  const blocker = $derived(connectBlocker(ui.cfg, ui.mode));

  // 200 ms view refresh (handoff "Telemetry refresh every 200 ms"). tele.* is
  // plain JS (not reactive), so everything the template reads from it is
  // copied into $state here -- including the core snapshot the header's link
  // tag and the record button's enable rule depend on.
  let view = $state(null), groups = $state([]), spark = $state('0,24 100,24'), status = $state('');
  let tag = $state({ label: 'Disconnected', on: false });
  let core = $state.raw(null);
  let rec = $state({ state: 'unknown', err: null }), recMs = $state(0);
  function refresh() {
    const nowMs = performance.timeOrigin + performance.now();
    if (osdLayer.isStale(performance.now())) clearOsd();   // 5 s without MSP
    if (live) tele.sample(nowMs, sess.mode);
    core = live ? tele.core : null;
    view = statsView({ connected: live, mode: shownMode, ch: live ? sess.ch : ui.cfg.channel, w: live ? sess.w : ui.cfg.width,
      core, page: live ? tele.page : null, sessionCfg: live ? sessionCfg : ui.cfg, videoSize: video.videoSize,
      colour: live ? video.colour : null });
    const target = effectiveTarget(sessionCfg.dvr, sess.mode);
    const lv = live ? localView(core) : { state: 'unknown', err: null, bytes: 0 };
    rec = live ? combinedRec({ target, vtx: recView(core), local: lv }) : { state: 'unknown', err: null };
    if (lv.bytes !== lrecPrevBytes) {
      const dt = (nowMs - lrecPrevT) / 1000;
      lrec = { ...lv, rateBps: lrecPrevT && dt > 0 ? Math.max(0, lv.bytes - lrecPrevBytes) / dt : 0 };
      lrecPrevBytes = lv.bytes; lrecPrevT = nowMs;
    } else if (!live || lv.state !== (lrec && lrec.state)) {
      lrec = live ? { ...lv, rateBps: lrec ? lrec.rateBps : 0 } : null;
    }
    groups = debugGroups({ connected: live, mode: shownMode, core, rcfPct: tele.rcfPct, ausRate: tele.ausRate,
      hitches60: tele.hitches60(nowMs), hitchesTotal: video.hitchesTotal, seg: tele.seg || { w1: {}, w60: {} }, lrec });
    spark = live ? sparkPoints(tele.spark) : '0,24 100,24';
    tag = linkTag({ state: sess.state, mode: sess.mode, core });
    recClock.update(rec.state, nowMs);
    recMs = recClock.elapsedMs(nowMs);
    status = statusText({ state: sess.state, mode: sess.mode, ch: sess.ch, w: sess.w, core,
      sinceStartMs: sess.startedAt ? Date.now() - sess.startedAt : 0, hiddenBanner });
  }

  async function connect() {
    if (blocker) return;
    saveMode(ui.mode);
    saveConfig(localStorageSafe(), $state.snapshot(ui.cfg));
    sessionCfg = structuredClone($state.snapshot(ui.cfg));
    // Session.connect() is a no-op unless idle/error; don't wipe a running
    // session's pipeline for a press it will ignore.
    const st = session.snapshot.state;
    if (st !== 'idle' && st !== 'error') return;
    // Phones: Connect also goes fullscreen + landscape-locked -- but only once
    // the card is already granted: fullscreen consumes the tap's activation,
    // which the first-time WebUSB chooser needs.
    if (isMobile(LW, LH) && usbGranted) goLandscape();
    video.reset(); tele.reset(); osd.resetAtlasFailure(); hiddenShown = false; hiddenBanner = false;
    const p = session.connect({ mode: ui.mode, ch: ui.cfg.channel, w: ui.cfg.width, overlayToml: toOverlayToml(sessionCfg) });
    refresh();   // the connecting tag/overlay without waiting for the next tick
    await p;
    refresh();
    checkUsbGranted();
  }
  function localStorageSafe() { try { return localStorage; } catch { return null; } }
  function disconnect() {
    const p = session.disconnect();
    refresh();   // the stopping tag/overlay without waiting for the next tick
    return p;
  }

  // Immersive connect/disconnect button: one control, ignored mid-transition.
  function toggleConn() {
    if (busy) return;
    if (live) disconnect(); else connect();
  }

  function onCfgChange(next, label) {
    ui.cfg = next;
    saveConfig(localStorageSafe(), $state.snapshot(next));
    ui.applied = `${label} · saved ${new Date().toTimeString().slice(0, 8)}`;
  }

  const recTarget = $derived(effectiveTarget((live || busy ? sessionCfg : ui.cfg).dvr, live || busy ? sess.mode : ui.mode));
  const recCovers = $derived(targetCovers(recTarget));
  const recOn = $derived(rec.state === 'recording');
  const recWaiting = $derived(rec.state === 'waiting');
  const vtxReady = $derived(live && sess.mode === 'gs' && !!core?.session && !!core?.peer_acked);
  const localReady = $derived(live && opfsOk && core?.lrec_avail !== 0);
  // Local needs only a live module with OPFS; VTX-only keeps today's rule.
  // Both falls back to the VTX-only rule when there's no browser storage --
  // spec §2.4 still wants Record to work, just VTX-only (spec 2026-09-28
  // final review finding 4).
  const bothVtxFallback = $derived(recTarget === 'both' && !localReady);
  const recDisabled = $derived(recTarget === 'vtx' || bothVtxFallback ? !vtxReady : !localReady);
  const recLabel = $derived(recOn ? formatClock(recMs) : recWaiting ? 'waiting for sync…'
    : rec.state === 'error' ? 'REC!' : 'Record');
  const recTitle = $derived(rec.state === 'error' ? rec.err
    : bothVtxFallback && live ? 'Browser storage unavailable — recording on the VTX only'
    : recCovers.web && live && !localReady ? 'Browser storage unavailable — local recording is off'
    : { web: 'Record in this browser (R)', vtx: 'Record on the VTX (R)', both: 'Record in this browser and on the VTX (R)' }[recTarget]);
  // Toggle from what is shown (a drone already recording after a reconnect
  // is stopped by the press), plus our own wishes.
  function toggleRec() {
    if (recDisabled) return;
    const on = !(recOn || recWaiting || sess.recWish || sess.localWish);
    if (recCovers.vtx && sess.mode === 'gs') session.setRec(on);
    if (recCovers.web && localReady) {
      if (on) {
        if (!persistAsked) { persistAsked = true; navigator.storage?.persist?.().then((p) => console.log('[webgs] storage persist', p)).catch(() => {}); }
        session.setLocalRec(recFileName(new Date(), recItems.map((r) => r.name)));
        setTimeout(refreshRecordings, 1500);   // show the new file as "recording"
      } else {
        session.setLocalRec(null);
      }
    }
  }

  function toggleFs() {
    const on = !ui.fs;
    ui.fs = on;
    try {
      if (on && !document.fullscreenElement) document.documentElement.requestFullscreen?.().catch(() => {});
      else if (!on && document.fullscreenElement) document.exitFullscreen();
    } catch { /* layout still switches */ }
  }

  // Phones: the layout is already immersive, but the browser bars stay until
  // the page asks for real fullscreen. Tracks the Fullscreen API state (not
  // ui.fs, the desktop layout flag); no button where the API is unavailable
  // (iPhone Safari).
  let realFs = $state(!!document.fullscreenElement);
  const fsSupported = !!document.fullscreenEnabled;
  async function toggleRealFs() {
    if (document.fullscreenElement) { try { await document.exitFullscreen(); } catch { /* stay */ } return; }
    await goLandscape();
  }
  // Fullscreen + lock to 'landscape' (either side: the sensor still flips it
  // 180°, never to portrait). Android Chrome allows the lock only while
  // fullscreen; must run inside the tap, before any await.
  async function goLandscape() {
    if (!fsSupported) return;
    try {
      if (!document.fullscreenElement) await document.documentElement.requestFullscreen();
      const lock = screen.orientation?.lock?.('landscape');
      if (lock) lock.catch(() => {});
    } catch { /* refused: stay as is */ }
  }
  let usbGranted = false;
  function checkUsbGranted() {
    navigator.usb?.getDevices().then((d) => { usbGranted = d.some((x) => x.vendorId === 0x0bda); }).catch(() => {});
  }

  let copyTimer = null;
  async function copyStats() {
    const text = tele.copyPayload();
    try { await navigator.clipboard.writeText(text); copyMsg = 'copied'; }
    catch (e) { console.error('[webgs] copy failed', e); copyMsg = 'copy failed (see console)'; }
    clearTimeout(copyTimer);
    copyTimer = setTimeout(() => { copyMsg = ''; }, 3000);
  }

  function onKey(e) {
    const a = keyAction(e);
    if (!a) return;
    if (a === 'fs') toggleFs();
    else if (a === 'rec') toggleRec();
    else if (a === 'stats') ui.statsVisible = !ui.statsVisible;
    else if (a === 'esc') ui.cfgOpen = false;
  }

  onMount(() => {
    const iv = setInterval(refresh, 200);
    checkUsbGranted();
    refreshRecordings();
    refresh();
    osd.attach(osdCanvas);
    const ro = new ResizeObserver(([e]) => osd.resize(e.contentRect.width, e.contentRect.height, window.devicePixelRatio || 1));
    ro.observe(osdCanvas);
    const onFsChange = () => {
      realFs = !!document.fullscreenElement;
      if (!realFs) ui.fs = false;
    };
    document.addEventListener('fullscreenchange', onFsChange);
    const onVis = () => {
      if (document.hidden) { if (sess.mode === 'gs' && live && !hiddenShown) { hiddenBanner = true; hiddenShown = true; } }
      else { hiddenBanner = false; wake.onVisible(); }
    };
    document.addEventListener('visibilitychange', onVis);
    const onWinErr = (e) => {
      const msg = String((e && e.message) || '');
      const fromWorker = typeof e?.filename === 'string' && /webgs\.js|worker/i.test(e.filename);
      if (msg.includes('worker sent an error') || fromWorker) session.fail(WORKER_FAILED);
    };
    window.addEventListener('error', onWinErr);
    return () => { clearInterval(iv); ro.disconnect(); clearTimeout(copyTimer); document.removeEventListener('fullscreenchange', onFsChange);
      document.removeEventListener('visibilitychange', onVis); window.removeEventListener('error', onWinErr); wake.set(false); };
  });
</script>

<svelte:window bind:innerWidth={W} bind:innerHeight={H} onkeydown={onKey} />

<div class="root" class:immersive={layout !== 'windowed'} style={frame.style}>
  {#if layout === 'windowed'}
    <Header {tag} {chLine} {live} {busy} blocked={!!blocker}
      onConnect={connect} onDisconnect={disconnect} rec={recOn} {recLabel} {recDisabled} {recTitle}
      onRec={toggleRec} onFs={toggleFs} />
  {/if}
  <div class="row">
    <div class="videocol">
      <div class="videobox">
        <!-- ONE canvas for the page lifetime: never inside a layout-dependent {#if}. -->
        <div class="videoinner">
          <canvas bind:this={canvas} width="1280" height="720"></canvas>
          <!-- colortrans (WebGL): stacked on the flat canvas; VideoPipeline shows whichever drew the last frame. -->
          <canvas class="vgl" bind:this={glCanvas} width="1280" height="720" style="visibility:hidden"></canvas>
          <!-- MSP OSD: page-lifetime like the video canvas; .videoinner IS the visible area in every layout. -->
          <canvas class="osd" bind:this={osdCanvas}></canvas>
          {#if live && status}<div class="status glass">{status}</div>{/if}
          {#if !live}
            <DisconnectedOverlay mode={ui.mode} onMode={(m) => (ui.mode = m)} onConnect={connect}
              error={sess.state === 'error' ? sess.error : null} notice={sess.notice} {blocker} {busy} stopping={sess.state === 'stopping'}
              padRight={layout === 'immersive' && ui.cfgOpen} mobile={layout !== 'windowed' && LW < 1000} />
          {/if}
        </div>
      </div>
    </div>
    {#if layout === 'windowed' && view}
      <Sidebar tab={ui.tab} onTab={(t) => (ui.tab = t)} v={view} {spark} {groups} onCopy={copyStats} {copyMsg}>
        {#snippet config()}
          <ConfigPanel cfg={ui.cfg} onChange={onCfgChange} locked={live || busy} spotter={ui.mode === 'spotter'}
            onDisconnect={live ? disconnect : null} variant="rule" applied={ui.applied} recordings={recList} />
        {/snippet}
      </Sidebar>
    {/if}
  </div>
  {#if layout === 'immersive' && view}
    <FsOverlay {live} mode={sess.mode} {chLine} {recOn} {recWaiting} recClock={formatClock(recMs)} recErr={rec.state === 'error' ? (rec.err || 'error') : null} {recDisabled} {recTitle}
      onConn={toggleConn} onRec={toggleRec} onStats={() => (ui.statsVisible = !ui.statsVisible)}
      onCfg={() => (ui.cfgOpen = !ui.cfgOpen)} fsButton={isMobile(LW, LH) ? (fsSupported ? { on: realFs } : null) : { on: true }}
      onFs={isMobile(LW, LH) ? toggleRealFs : toggleFs} />
    {#if ui.statsVisible && !ui.cfgOpen}
      <FloatStats v={view} mobile={isMobile(LW, LH)} open={ui.floatOpen} pos={ui.fpos}
        onOpen={(o) => (ui.floatOpen = o)} onMove={(p) => (ui.fpos = p)} cw={LW} ch={LH} toLocal={frame.toLocal} />
    {/if}
    {#if ui.cfgOpen}
      <ConfigSide onClose={() => (ui.cfgOpen = false)}>
        <ConfigPanel cfg={ui.cfg} onChange={onCfgChange} locked={live || busy} spotter={ui.mode === 'spotter'}
          onDisconnect={live ? disconnect : null} variant="card" applied={ui.applied} recordings={recList} />
      </ConfigSide>
    {/if}
  {/if}
</div>

{#snippet recList()}
  <Recordings items={recItems} active={live && (sess.localWish || lrec?.state === 'waiting' || lrec?.state === 'recording') ? (core?.lrec_name || null) : null}
    storage={recStorage} warn={recWarn} available={opfsOk} onDownload={dlRecording} onDelete={delRecording} />
{/snippet}

<style>
  .root { position: absolute; inset: 0; display: flex; flex-direction: column; background: var(--color-bg); color: var(--color-text); font-family: var(--font-body); }
  .row { flex: 1; min-height: 0; display: flex; gap: var(--space-6); padding: 0 var(--space-6) var(--space-6); }
  .videocol { flex: 1; min-width: 0; display: flex; flex-direction: column; }
  .videobox { flex: 1; min-height: 0; display: grid; place-items: center; container-type: size; background: var(--color-bg); }
  .videoinner { position: relative; aspect-ratio: 16 / 9; width: min(100cqw, calc(100cqh * 16 / 9)); border-radius: var(--radius-sm); overflow: hidden; background: #000; }
  canvas { display: block; width: 100%; height: 100%; object-fit: contain; }
  .osd, .vgl { position: absolute; inset: 0; pointer-events: none; }
  .immersive .row { position: absolute; inset: 0; padding: 0; gap: 0; }
  /* Letterboxed, never cropped: the 16:9 box fits the viewport, black bars fill the rest. */
  .immersive .videobox { background: #000; }
  .immersive .videoinner { border-radius: 0; }
  .status { position: absolute; left: 50%; top: 50%; transform: translate(-50%, -50%); padding: 6px 12px; border-radius: 6px;
            font-size: 12px; color: var(--color-neutral-100); white-space: pre-wrap; text-align: center; pointer-events: none; }
</style>
