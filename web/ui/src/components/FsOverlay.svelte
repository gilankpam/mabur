<script>
  import Icon from './Icon.svelte';
  import RecDot from './RecDot.svelte';
  let { live, mode, chLine, recOn, recClock, recDisabled, recTitle, onConn, onRec, onStats, onCfg, onExitFs, showExitFs } = $props();
</script>
<div class="tl">
  <span class="tag glass" style="gap:6px;color:var(--color-neutral-100)">
    <span style="width:6px;height:6px;border-radius:50%;{live ? 'background:var(--color-accent)' : 'border:1px solid var(--color-neutral-400)'}"></span>
    {live ? `${mode === 'spotter' ? 'Spotter · ' : ''}CH ${chLine}` : 'Disconnected'}</span>
  {#if recOn}
    <span class="tag glass num" style="gap:6px;color:var(--color-neutral-100)">
      <span style="width:7px;height:7px;border-radius:50%;background:var(--color-accent-400);animation:recpulse 1.2s infinite"></span>REC {recClock}</span>
  {/if}
</div>
<div class="tr">
  <button class="btn btn-icon glass round44" type="button" title={live ? 'Disconnect' : 'Connect'} aria-label={live ? 'Disconnect' : 'Connect'} onclick={onConn}>
    <Icon name={live ? 'plugs' : 'plugs-connected'} size="19px" /></button>
  <button class="btn btn-icon glass round44" type="button" title={recTitle} aria-label="Record" onclick={onRec} disabled={recDisabled}><RecDot on={recOn} size={14} /></button>
  <button class="btn btn-icon glass round44" type="button" title="Flight stats (S)" aria-label="Flight stats" onclick={onStats}><Icon name="chart-line-up" size="19px" /></button>
  <button class="btn btn-icon glass round44" type="button" title="Configuration" aria-label="Configuration" onclick={onCfg}><Icon name="gear-six" size="19px" /></button>
  {#if showExitFs}
    <button class="btn btn-icon glass round44" type="button" title="Exit fullscreen (F)" aria-label="Exit fullscreen" onclick={onExitFs}><Icon name="corners-in" size="19px" /></button>
  {/if}
</div>
<style>
  .tl { position: absolute; top: 12px; left: 12px; display: flex; gap: 6px; flex-wrap: wrap; pointer-events: none; z-index: 3; }
  .tr { position: absolute; top: 10px; right: 12px; display: flex; gap: 6px; z-index: 3; }
</style>
