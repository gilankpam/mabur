<script>
  import Icon from './Icon.svelte';
  import bg from '../assets/fpv-feed.jpg';
  let { mode, onMode, onConnect, error = null, notice = null, blocker = null, busy = false, stopping = false,
        padRight = false, mobile = false, radio = 'usb', onRadio = () => {}, relayAddr = '', onRelayAddr = () => {},
        showAddr = false } = $props();
  const HINT = {
    gs: 'Full link control. Sends link feedback to the drone and runs the adaptive ladder.',
    spotter: 'Receive only. No uplink to the drone, so link and ladder settings are not applied.',
  };
</script>
<div class="ov" style="padding-right:{padRight ? 444 : 24}px; --ov-bg:url({bg})">
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
  <div class="seg" style="width:min(300px,100%)">
    <label class="seg-opt" style={mobile ? 'min-height:40px' : ''}><input type="radio" name="webgs-radio" checked={radio === 'usb'} disabled={busy}
      onchange={() => onRadio('usb')}><Icon name="plugs" />USB card</label>
    <label class="seg-opt" style={mobile ? 'min-height:40px' : ''}><input type="radio" name="webgs-radio" checked={radio === 'relay'} disabled={busy}
      onchange={() => onRadio('relay')}><Icon name="broadcast" />CPE relay</label>
  </div>
  {#if radio === 'relay' && showAddr}
    <input class="addr" type="text" value={relayAddr} disabled={busy} spellcheck="false" aria-label="CPE relay address"
      placeholder="10.83.11.1" oninput={(e) => onRelayAddr(e.currentTarget.value)} />
  {/if}
  {#if blocker}<span class="msg warn">{blocker}</span>{/if}
  <button class="btn btn-primary" type="button" onclick={onConnect} disabled={busy || !!blocker}
    style="margin-top:4px;{mobile ? 'min-height:44px' : ''}">
    <Icon name="plugs-connected" />{mode === 'spotter' ? 'Connect as spotter' : 'Connect'}</button>
</div>
<style>
  .ov { position: absolute; inset: 0; display: flex; flex-direction: column; align-items: center; justify-content: center;
        gap: 10px; text-shadow: 0 1px 3px rgb(0 0 0 / 0.6);
        /* No video: an aerial still behind a tint dark enough for the controls. */
        background: linear-gradient(color-mix(in srgb, var(--color-bg) 62%, transparent), color-mix(in srgb, var(--color-bg) 62%, transparent)),
                    var(--ov-bg) center / cover no-repeat, var(--color-bg); text-align: center; padding: 24px; z-index: 2; }
  .msg { font-size: 12px; max-width: 360px; color: var(--color-neutral-300); white-space: pre-wrap; }
  .msg.err { color: var(--color-accent-200); background: var(--color-accent-900); padding: 6px 10px; border-radius: var(--radius-md); }
  .msg.warn { color: var(--color-accent-300); }
  .addr { width: min(300px, 100%); font: inherit; padding: 6px 8px; border-radius: var(--radius-md); }
</style>
