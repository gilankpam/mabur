<script>
  import Icon from './Icon.svelte';
  import RungBar from './RungBar.svelte';
  import { clampFloatPos, dragStarted, FLOAT_W } from '../lib/layout.js';
  let { v, mobile, open, pos, onOpen, onMove, cw, ch } = $props();

  const fw = $derived(mobile ? FLOAT_W.mobile : FLOAT_W.desktop);
  let justDragged = false;
  const style = $derived.by(() => {
    if (!pos) return `top:64px;right:12px;max-height:${ch - 76}px`;
    const p = clampFloatPos(pos, { cw, ch }, open ? fw : 120);
    return `left:${p.x}px;top:${p.y}px;max-height:${ch - p.y - 8}px`;
  });

  // Prototype startDrag: 5 px dead zone, then follow the pointer, clamped
  // 8 px inside the parent with 48 px kept reachable at the bottom.
  function startDrag(e) {
    if (e.button && e.button !== 0) return;
    const el = e.currentTarget.closest('[data-float]');
    const parent = el && el.offsetParent;
    if (!parent) return;
    const pr = parent.getBoundingClientRect(), r = el.getBoundingClientRect();
    const ox = e.clientX - r.left, oy = e.clientY - r.top, sx = e.clientX, sy = e.clientY;
    let moved = false;
    const move = (ev) => {
      if (!moved && !dragStarted(sx, sy, ev.clientX, ev.clientY)) return;
      moved = true;
      onMove(clampFloatPos({ x: ev.clientX - pr.left - ox, y: ev.clientY - pr.top - oy },
        { cw: pr.width, ch: pr.height }, r.width));
    };
    const up = () => {
      window.removeEventListener('pointermove', move);
      window.removeEventListener('pointerup', up);
      window.removeEventListener('pointercancel', up);
      if (moved) { justDragged = true; setTimeout(() => { justDragged = false; }, 0); }
    };
    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
    window.addEventListener('pointercancel', up);
  }

  // Keyboard equivalent of the drag handle: arrows nudge (Shift = 40 px),
  // Home resets to the default corner (the double-click action).
  const STEP = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] };
  function onHeadKey(e) {
    if (e.target !== e.currentTarget) return;
    if (e.key === 'Home') { e.preventDefault(); onMove(null); return; }
    const d = STEP[e.key];
    if (!d) return;
    e.preventDefault();
    const el = e.currentTarget.closest('[data-float]');
    const parent = el && el.offsetParent;
    if (!parent) return;
    const pr = parent.getBoundingClientRect(), r = el.getBoundingClientRect();
    const k = e.shiftKey ? 40 : 10;
    onMove(clampFloatPos({ x: r.left - pr.left + d[0] * k, y: r.top - pr.top + d[1] * k },
      { cw: pr.width, ch: pr.height }, r.width));
  }
</script>

<div data-float="1" class="wrap" {style}>
  {#if open}
    <div class="panel glass-panel scroll-thin num" style="width:{fw}px">
      <div class="head" role="toolbar" tabindex="0" aria-label="Flight stats: drag or use arrow keys to move, Home to reset"
        onpointerdown={startDrag} ondblclick={() => onMove(null)} onkeydown={onHeadKey} title="Drag to move · double-click to reset">
        <Icon name="dots-six-vertical" size="15px" style="color:var(--color-neutral-500)" />
        <span class="card-kicker" style="margin-right:auto">Flight stats</span>
        <button class="btn btn-ghost" type="button" title="Collapse" aria-label="Collapse flight stats" style="width:28px;height:28px;padding:0"
          onpointerdown={(e) => e.stopPropagation()} onclick={() => onOpen(false)}><Icon name="caret-up" /></button>
      </div>
      <div style="display:flex;flex-direction:column;gap:6px">
        <div style="display:flex;align-items:baseline;gap:6px"><span style="font-size:20px;line-height:1">MCS {v.mcs}</span>
          <span class="dim4" style="font-size:12px">{v.bw} MHz{#if v.rungMode === 'Adaptive'} · rung {v.rungNum}/{v.rungCount}{/if}</span></div>
        <RungBar segs={v.segs} height={3} />
        <div class="dim4" style="font-size:11px">CH {v.chLine.split(' · ')[0]} · {v.rungMode}</div>
      </div>
      <div style="display:flex;flex-direction:column;gap:5px">
        {#each v.cards as c (c.idx)}
          <div style="display:flex;gap:6px;font-size:12px"><span class="dim4 nw">Card {c.idx}{c.tx ? ' · TX' : ''}</span>
            <span class="nw" style="margin-left:auto">{c.rssi} dBm · {c.snr} dB</span></div>
        {/each}
      </div>
      <div class="g2">
        <div class="c"><span class="l">Pre / post FEC</span><span class="n">{v.preLoss} / {v.postLoss}<span class="l"> %</span></span></div>
        <div class="c"><span class="l">Bitrate</span><span class="n">{v.bitrate}<span class="l"> Mbps</span></span></div>
        <div class="c"><span class="l">Latency</span><span class="n">{v.latency}<span class="l"> ms</span></span></div>
        <div class="c"><span class="l">FPS · jitter</span><span class="n">{v.fps} · {v.jitter}<span class="l"> ms</span></span></div>
      </div>
    </div>
  {:else}
    <button class="pill num" type="button" title="Tap to expand · drag to move" onpointerdown={startDrag}
      onclick={() => { if (!justDragged) onOpen(true); }}>
      <span style="color:var(--color-accent-300)">MCS {v.mcs}</span><span>{v.bestRssi} dBm</span>
      <span>{v.bitrate} Mbps</span><span>{v.latency} ms</span><Icon name="caret-down" style="color:var(--color-neutral-400)" />
    </button>
  {/if}
</div>

<style>
  .wrap { position: absolute; display: flex; flex-direction: column; z-index: 3; }
  .panel { min-height: 0; overflow: auto; display: flex; flex-direction: column; gap: var(--space-4); padding: var(--space-4); }
  .head { display: flex; align-items: center; gap: 6px; cursor: grab; touch-action: none; user-select: none;
          margin: calc(var(--space-4) * -1) calc(var(--space-4) * -1) 0; padding: var(--space-4) var(--space-4) 0; }
  .nw { white-space: nowrap; }
  .g2 { display: grid; grid-template-columns: 1fr 1fr; gap: var(--space-3) var(--space-4); }
  .c { display: flex; flex-direction: column; } .l { font-size: 11px; color: var(--color-neutral-400); } .n { font-size: 15px; }
  .pill { touch-action: none; user-select: none; display: flex; align-items: center; gap: 10px; padding: 8px 10px 8px 12px; border: 0;
          border-radius: var(--radius-md); background: color-mix(in srgb, var(--color-surface) 80%, transparent);
          backdrop-filter: blur(12px); box-shadow: var(--shadow-md); color: var(--color-text); font: inherit; font-size: 12px; cursor: pointer; }
  .pill:hover { background: color-mix(in srgb, var(--color-surface) 95%, transparent); }
</style>
