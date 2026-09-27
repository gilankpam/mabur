<script>
  import RungBar from './RungBar.svelte';
  import Sparkline from './Sparkline.svelte';
  let { v, spark } = $props();
</script>

<div class="group">
  <div class="kick-row"><span class="card-kicker">Link</span>
    <span class="dim4 nowrap" style="font-size:12px">{v.rungMode}{#if v.rungMode === 'Adaptive'}{' · '}rung {v.rungNum}/{v.rungCount}{/if}</span></div>
  <div style="display:flex;align-items:baseline;gap:8px;flex-wrap:wrap">
    <span class="num nowrap" style="font-size:28px;line-height:1;font-weight:500">MCS {v.mcs}</span>
    <span class="dim4 nowrap" style="font-size:13px">{v.bw} MHz</span>
  </div>
  <RungBar segs={v.segs} height={4} />
  <div style="display:flex;justify-content:space-between;font-size:12px">
    <span class="dim5">Channel</span><span class="num nowrap">{v.chLine}</span></div>
</div>

<div class="group">
  <div class="kick-row"><span class="card-kicker">Radio cards</span><span class="dim5" style="font-size:11px">RSSI · SNR</span></div>
  {#each v.cards as c}
    <div style="display:flex;flex-direction:column;gap:5px">
      <div style="display:flex;align-items:center;gap:6px;font-size:12px">
        <span class="nowrap">Card {c.idx}</span>
        {#if c.tx}<span class="tag tag-outline" style="padding:0 6px;font-size:10px">TX</span>{/if}
        <span class="num nowrap" style="margin-left:auto">{c.rssi} <span class="dim5">dBm</span> · {c.snr} <span class="dim5">dB</span></span>
      </div>
      <div style="height:3px;border-radius:2px;background:var(--color-neutral-800);overflow:hidden">
        <div style="height:100%;width:{c.barPct}%;background:var(--color-accent-500);border-radius:2px"></div></div>
    </div>
  {/each}
</div>

<div class="group">
  <span class="card-kicker">Loss</span>
  <div class="grid2" style="gap:var(--space-3)">
    <div class="cell"><span class="lbl">Pre-FEC</span><span class="val num">{v.preLoss}<span class="unit">{' %'}</span></span></div>
    <div class="cell"><span class="lbl">Post-FEC</span><span class="val num">{v.postLoss}<span class="unit">{' %'}</span></span></div>
  </div>
</div>

<div class="group">
  <div class="kick-row"><span class="card-kicker">Video</span><span class="dim4 num" style="font-size:12px">{v.codecLine}</span></div>
  <div class="grid2" style="gap:var(--space-4) var(--space-3)">
    <div class="cell"><span class="lbl">Bitrate</span><span class="val num">{v.bitrate}<span class="unit">{' Mbps'}</span></span></div>
    <div class="cell"><span class="lbl">Latency</span><span class="val num">{v.latency}<span class="unit">{' ms'}</span></span></div>
    <div class="cell"><span class="lbl">Frame rate</span><span class="val num">{v.fps}<span class="unit">{' fps'}</span></span></div>
    <div class="cell"><span class="lbl">Jitter</span><span class="val num">{v.jitter}<span class="unit">{' ms'}</span></span></div>
  </div>
  <Sparkline points={spark} />
  <span class="dim6" style="font-size:11px">{v.latencyCaption}</span>
</div>

<style>
  .nowrap { white-space: nowrap; }
  .grid2 { display: grid; grid-template-columns: 1fr 1fr; }
  .cell { display: flex; flex-direction: column; gap: 2px; }
  .lbl { font-size: 12px; color: var(--color-neutral-500); }
  .val { font-size: 20px; }
  .unit { font-size: 12px; color: var(--color-neutral-500); }
</style>
