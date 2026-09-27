<script>
  import { onMount } from 'svelte';
  import Header from './Header.svelte';
  import Sidebar from './Sidebar.svelte';
  import ConfigPanel from './ConfigPanel.svelte';
  import DisconnectedOverlay from './DisconnectedOverlay.svelte';
  import FsOverlay from './FsOverlay.svelte';
  import FloatStats from './FloatStats.svelte';
  import ConfigSide from './ConfigSide.svelte';
  import PortraitNotice from './PortraitNotice.svelte';
  import { ui, saveMode, reloadToConfig } from '../lib/ui.svelte.js';
  import { Session, WORKER_FAILED } from '../lib/session.js';
  import { VideoPipeline } from '../lib/video.js';
  import { Telemetry } from '../lib/telemetry.js';
  import { statsView, debugGroups, statusText, linkTag } from '../lib/view.js';
  import { recView, RecClock, formatClock } from '../lib/rec.js';
  import { sparkPoints } from '../lib/metrics.js';
  import { layoutMode, keyAction, isMobile } from '../lib/layout.js';
  import { connectBlocker, toOverlayToml, saveConfig } from '../lib/config.js';

  let canvas = $state(null);
  let W = $state(innerWidth), H = $state(innerHeight);
  let sess = $state({ state: 'idle', mode: ui.mode, ch: null, w: null, error: null, notice: null, startedAt: null, recWish: false });
  let sessionCfg = $state(ui.cfg);        // the config the running session started with
  let copyMsg = $state('');
  let hiddenBanner = $state(false), hiddenShown = false;
  const recClock = new RecClock();

  const video = new VideoPipeline({ getCanvas: () => canvas, getMode: () => sess.mode });
  const tele = new Telemetry(video);
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
    reload: reloadToConfig,
  });

  let prevState = 'idle';
  session.subscribe((s) => {
    sess = s;
    // Left live (stopping/idle/error): free the hardware decoder now.
    if (s.state !== 'live' && s.state !== 'connecting') video.close();
    if (s.state === 'live' && prevState !== 'live') { ui.tab = 'stats'; ui.cfgOpen = false; ui.statsVisible = true; }
    if (prevState === 'stopping' && s.state === 'idle') { ui.tab = 'config'; ui.cfgOpen = true; }
    if (s.state === 'error' && prevState === 'live') { ui.tab = 'config'; }
    prevState = s.state;
  });

  const layout = $derived(layoutMode({ w: W, h: H, fs: ui.fs }));
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
    if (live) tele.sample(nowMs, sess.mode);
    core = live ? tele.core : null;
    view = statsView({ connected: live, mode: shownMode, ch: live ? sess.ch : ui.cfg.channel, w: live ? sess.w : ui.cfg.width,
      core, page: live ? tele.page : null, sessionCfg: live ? sessionCfg : ui.cfg, videoSize: video.videoSize });
    groups = debugGroups({ connected: live, mode: shownMode, core, rcfPct: tele.rcfPct, ausRate: tele.ausRate,
      hitches60: tele.hitches60(nowMs), hitchesTotal: video.hitchesTotal, seg: tele.seg || { w1: {}, w60: {} } });
    spark = live ? sparkPoints(tele.spark) : '0,24 100,24';
    tag = linkTag({ state: sess.state, mode: sess.mode, core });
    rec = live ? recView(core) : { state: 'unknown', err: null };
    recClock.update(rec.state, nowMs);
    recMs = recClock.elapsedMs(nowMs);
    const p = performance.now();
    status = statusText({ state: sess.state, mode: sess.mode, ch: sess.ch, w: sess.w, core,
      gateArmed: video.gateArmed(), sinceStartMs: sess.startedAt ? Date.now() - sess.startedAt : 0,
      waitingMs: p - video.waitingSinceMs, hiddenBanner });
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
    video.reset(); tele.reset(); hiddenShown = false; hiddenBanner = false;
    const p = session.connect({ mode: ui.mode, ch: ui.cfg.channel, w: ui.cfg.width, overlayToml: toOverlayToml(sessionCfg) });
    refresh();   // the connecting tag/overlay without waiting for the next tick
    await p;
    refresh();
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

  const recOn = $derived(rec.state === 'recording');
  // GS mode AND live AND the core reports session && peer_acked (spotter never).
  const recDisabled = $derived(!(live && sess.mode === 'gs' && !!core?.session && !!core?.peer_acked));
  const recLabel = $derived(recOn ? formatClock(recMs) : rec.state === 'error' ? 'REC!' : 'Record');
  const recTitle = $derived(rec.state === 'error' ? rec.err
    : sess.mode === 'spotter' && (live || busy) ? 'Spotter cannot start or stop VTX recording' : 'Record on the VTX (R)');
  // Toggle from what is shown: if the drone already records (e.g. after a
  // reconnect, recWish false), the press stops it.
  function toggleRec() { if (!recDisabled) session.setRec(!(recOn || sess.recWish)); }

  function toggleFs() {
    const on = !ui.fs;
    ui.fs = on;
    try {
      if (on && !document.fullscreenElement) document.documentElement.requestFullscreen?.().catch(() => {});
      else if (!on && document.fullscreenElement) document.exitFullscreen();
    } catch { /* layout still switches */ }
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
    refresh();
    const onFsChange = () => { if (!document.fullscreenElement) ui.fs = false; };
    document.addEventListener('fullscreenchange', onFsChange);
    const onVis = () => {
      if (document.hidden) { if (sess.mode === 'gs' && live && !hiddenShown) { hiddenBanner = true; hiddenShown = true; } }
      else hiddenBanner = false;
    };
    document.addEventListener('visibilitychange', onVis);
    const onWinErr = (e) => {
      const msg = String((e && e.message) || '');
      const fromWorker = typeof e?.filename === 'string' && /webgs\.js|worker/i.test(e.filename);
      if (msg.includes('worker sent an error') || fromWorker) session.fail(WORKER_FAILED);
    };
    window.addEventListener('error', onWinErr);
    return () => { clearInterval(iv); clearTimeout(copyTimer); document.removeEventListener('fullscreenchange', onFsChange);
      document.removeEventListener('visibilitychange', onVis); window.removeEventListener('error', onWinErr); };
  });
</script>

<svelte:window bind:innerWidth={W} bind:innerHeight={H} onkeydown={onKey} />

<div class="root" class:immersive={layout !== 'windowed'}>
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
          {#if live && status}<div class="status glass">{status}</div>{/if}
          {#if !live}
            <DisconnectedOverlay mode={ui.mode} onMode={(m) => (ui.mode = m)} onConnect={connect}
              error={sess.state === 'error' ? sess.error : null} notice={sess.notice} {blocker} {busy} stopping={sess.state === 'stopping'}
              padRight={layout === 'immersive' && ui.cfgOpen} mobile={layout !== 'windowed' && W < 1000} />
          {/if}
        </div>
      </div>
    </div>
    {#if layout === 'windowed' && view}
      <Sidebar tab={ui.tab} onTab={(t) => (ui.tab = t)} v={view} {spark} {groups} onCopy={copyStats} {copyMsg}>
        {#snippet config()}
          <ConfigPanel cfg={ui.cfg} onChange={onCfgChange} locked={live || busy} spotter={ui.mode === 'spotter'}
            onDisconnect={live ? disconnect : null} variant="rule" applied={ui.applied} />
        {/snippet}
      </Sidebar>
    {/if}
  </div>
  {#if layout === 'immersive' && view}
    <FsOverlay {live} mode={sess.mode} {chLine} {recOn} recClock={formatClock(recMs)} recErr={rec.state === 'error' ? (rec.err || 'error') : null} {recDisabled} {recTitle}
      onConn={toggleConn} onRec={toggleRec} onStats={() => (ui.statsVisible = !ui.statsVisible)}
      onCfg={() => (ui.cfgOpen = !ui.cfgOpen)} onExitFs={toggleFs} showExitFs={!isMobile(W, H)} />
    {#if ui.statsVisible && !ui.cfgOpen}
      <FloatStats v={view} mobile={isMobile(W, H)} open={ui.floatOpen} pos={ui.fpos}
        onOpen={(o) => (ui.floatOpen = o)} onMove={(p) => (ui.fpos = p)} cw={W} ch={H} />
    {/if}
    {#if ui.cfgOpen}
      <ConfigSide onClose={() => (ui.cfgOpen = false)}>
        <ConfigPanel cfg={ui.cfg} onChange={onCfgChange} locked={live || busy} spotter={ui.mode === 'spotter'}
          onDisconnect={live ? disconnect : null} variant="card" applied={ui.applied} />
      </ConfigSide>
    {/if}
  {/if}
  {#if layout === 'portrait'}<PortraitNotice />{/if}
</div>

<style>
  .root { position: absolute; inset: 0; display: flex; flex-direction: column; background: var(--color-bg); color: var(--color-text); font-family: var(--font-body); }
  .row { flex: 1; min-height: 0; display: flex; gap: var(--space-6); padding: 0 var(--space-6) var(--space-6); }
  .videocol { flex: 1; min-width: 0; display: flex; flex-direction: column; }
  .videobox { flex: 1; min-height: 0; display: grid; place-items: center; container-type: size; background: var(--color-bg); }
  .videoinner { position: relative; aspect-ratio: 16 / 9; width: min(100cqw, calc(100cqh * 16 / 9)); border-radius: var(--radius-sm); overflow: hidden; background: #000; }
  canvas { display: block; width: 100%; height: 100%; object-fit: contain; }
  .immersive .row { position: absolute; inset: 0; padding: 0; gap: 0; }
  .immersive .videobox { display: block; }
  .immersive .videoinner { position: absolute; inset: 0; width: auto; aspect-ratio: auto; border-radius: 0; }
  .immersive canvas { object-fit: cover; }
  .status { position: absolute; left: 50%; top: 50%; transform: translate(-50%, -50%); padding: 6px 12px; border-radius: 6px;
            font-size: 12px; color: var(--color-neutral-100); white-space: pre-wrap; text-align: center; pointer-events: none; }
</style>
