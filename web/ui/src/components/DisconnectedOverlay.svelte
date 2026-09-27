<script>
  import Icon from './Icon.svelte';
  let { mode, onMode, onConnect, error = null, notice = null, blocker = null, busy = false, stopping = false,
        padRight = false, mobile = false } = $props();
  const HINT = {
    gs: 'Full link control. Sends link feedback to the drone and runs the adaptive ladder.',
    spotter: 'Receive only. No uplink to the drone, so link and ladder settings are not applied.',
  };
</script>
<div class="ov" style="padding-right:{padRight ? 444 : 24}px">
  <Icon name="plugs" size="32px" style="color:var(--color-neutral-500)" />
  <span style="font-size:16px">{busy ? (stopping ? 'Disconnecting…' : 'Connecting…') : 'Not connected'}</span>
  {#if error}<span class="msg err">{error}</span>{:else if notice}<span class="msg">{notice}</span>{/if}
  <div class="seg" style="width:min(300px,100%);margin-top:4px">
    <label class="seg-opt" style={mobile ? 'min-height:40px' : ''}><input type="radio" name="webgs-mode" checked={mode === 'gs'} disabled={busy}
      onchange={() => onMode('gs')}><Icon name="broadcast" />Ground station</label>
    <label class="seg-opt" style={mobile ? 'min-height:40px' : ''}><input type="radio" name="webgs-mode" checked={mode === 'spotter'} disabled={busy}
      onchange={() => onMode('spotter')}><Icon name="binoculars" />Spotter</label>
  </div>
  <span class="dim5" style="font-size:12px;max-width:300px;text-wrap:pretty">{HINT[mode]}</span>
  {#if blocker}<span class="msg warn">{blocker}</span>{/if}
  <button class="btn btn-primary" type="button" onclick={onConnect} disabled={busy || !!blocker}
    style="margin-top:4px;{mobile ? 'min-height:44px' : ''}">
    <Icon name="plugs-connected" />{mode === 'spotter' ? 'Connect as spotter' : 'Connect'}</button>
</div>
<style>
  .ov { position: absolute; inset: 0; display: flex; flex-direction: column; align-items: center; justify-content: center;
        gap: 10px; background: color-mix(in srgb, var(--color-bg) 88%, transparent); text-align: center; padding: 24px; z-index: 2; }
  .msg { font-size: 12px; max-width: 360px; color: var(--color-neutral-300); white-space: pre-wrap; }
  .msg.err { color: var(--color-accent-200); background: var(--color-accent-900); padding: 6px 10px; border-radius: var(--radius-md); }
  .msg.warn { color: var(--color-accent-300); }
</style>
