<script>
  import Icon from './Icon.svelte';
  import { CHANNELS, MAX_RUNGS, applyEdit, applyRungEdit, rungWarnings, channelWarning, describeEdit, describeRungEdit } from '../lib/config.js';
  let { cfg, onChange, locked = false, spotter = false, onDisconnect = null, variant = 'rule', applied = '', recordings = null,
    keyFp = 'default', onLoadKey = null, onClearKey = null } = $props();
  const uid = $props.id();   // unique per mounted instance -- two panels (sidebar + Task 11 side panel) never share a radio group.

  const groupClass = $derived(variant === 'card' ? 'card cardg' : 'group');
  const set = (key, val) => onChange(applyEdit(cfg, key, val), describeEdit(key, val));
  const setRung = (i, key, val) => {
    const next = applyRungEdit(cfg, i, key, val);
    onChange(next, describeRungEdit(next, i, key, val));
  };
  const MCS = [0, 1, 2, 3, 4, 5, 6, 7];
  const chWarn = $derived(channelWarning(cfg));
  const pinned = $derived(cfg.staticMcs >= 0);

  // Rung drag-to-reorder. Pointer events, not HTML5 DnD (no touch support);
  // the drop target is hit-tested with elementFromPoint on client coords, so
  // it stays right when App draws the UI rotated on an upright phone.
  let drag = $state(null);   // { from, over } while a handle is held
  function rungAt(x, y) {
    const el = document.elementFromPoint(x, y)?.closest('[data-rung]');
    return el ? Number(el.dataset.rung) : null;
  }
  function startRungDrag(e, i) {
    if (locked || (e.button && e.button !== 0)) return;
    e.preventDefault();
    e.currentTarget.setPointerCapture?.(e.pointerId);
    drag = { from: i, over: i };
  }
  function moveRungDrag(e) {
    if (!drag) return;
    const i = rungAt(e.clientX, e.clientY);
    if (i !== null) drag.over = i;
  }
  function endRungDrag() {
    if (!drag) return;
    const { from, over } = drag;
    drag = null;
    if (over !== from) setRung(from, '__move', over);
  }
  // Keyboard equivalent: arrows on the handle move the rung one step.
  function rungKey(e, i) {
    const to = e.key === 'ArrowUp' ? i - 1 : e.key === 'ArrowDown' ? i + 1 : null;
    if (to === null) return;
    e.preventDefault();
    if (to < 0 || to >= cfg.ladder.length) return;
    setRung(i, '__move', to);
    const h = e.currentTarget.closest('.rungs');
    requestAnimationFrame(() => h?.querySelector(`[data-rung="${to}"] .grip`)?.focus());
  }
</script>

<div class="cfg" style="display:flex;flex-direction:column;gap:var(--space-6);font-size:13px;color:var(--color-text)">
  {#if locked}
    <div class="lock"><Icon name="lock-simple" size="15px" /><span style="margin-right:auto">Disconnect to change settings.</span>
      {#if onDisconnect}<button class="btn btn-ghost" type="button" onclick={onDisconnect} style="font-size:12px">Disconnect</button>{/if}</div>
  {/if}
  <fieldset disabled={locked} class="fs" style="opacity:{locked ? 0.5 : 1}">
    <div class={groupClass}>
      <span class="card-kicker">Radio</span>
      <div class="field">
        <label for="cfg-ch">Channel</label>
        <select id="cfg-ch" class="input" style="min-height:34px;padding:5px 8px" onchange={(e) => set('channel', +e.currentTarget.value)}>
          {#each CHANNELS as c}<option value={c} selected={c === cfg.channel}>{c}</option>{/each}
        </select>
        <div class="hint">Must match the drone.</div>
        {#if chWarn}<div class="warn"><Icon name="warning" />{chWarn}</div>{/if}
      </div>
      <div class="field">
        <span id="cfg-width-label" class="fieldlabel">Channel width</span>
        <div class="seg" role="radiogroup" aria-labelledby="cfg-width-label">
          <label class="seg-opt"><input type="radio" name="{uid}width" checked={cfg.width === 20} onchange={() => set('width', 20)}>20 MHz</label>
          <label class="seg-opt"><input type="radio" name="{uid}width" checked={cfg.width === 40} onchange={() => set('width', 40)}>40 MHz</label>
        </div>
      </div>
    </div>

    <div class={groupClass}>
      <span class="card-kicker">Link</span>
      {#if spotter}<div class="note"><Icon name="binoculars" size="14px" /><span>Not applied in spotter mode.</span></div>{/if}
      <div class="field">
        <label for="cfg-static">Fixed MCS</label>
        <select id="cfg-static" class="input" style="min-height:34px;padding:5px 8px" onchange={(e) => set('staticMcs', +e.currentTarget.value)}>
          <option value="-1" selected={cfg.staticMcs < 0}>Adaptive</option>
          {#each MCS as m}<option value={m} selected={m === cfg.staticMcs}>MCS {m}</option>{/each}
        </select>
      </div>
      {#if pinned}
        <div class="note"><Icon name="info" size="14px" /><span>Link pinned to MCS {cfg.staticMcs}. The ladder is kept but unused until Fixed MCS is set back to Adaptive.</span></div>
      {/if}
      {#if !spotter}
        <div class="field">
          <span id="{uid}key-label" class="fieldlabel">Link key</span>
          <div style="display:flex;gap:8px;align-items:center">
            <span class="tag {keyFp === 'default' ? 'tag-neutral' : 'tag-accent'}">{keyFp}</span>
            <input id="{uid}key-file" type="file" style="display:none" onchange={(e) => { const f = e.currentTarget.files?.[0]; if (f && onLoadKey) onLoadKey(f); e.currentTarget.value = ''; }}>
            <button class="btn btn-secondary" type="button" onclick={() => document.getElementById(`${uid}key-file`).click()}>Load</button>
            <button class="btn btn-ghost" type="button" disabled={keyFp === 'default'} onclick={() => onClearKey && onClearKey()}>Clear</button>
          </div>
          <div class="hint">The pairing key file (/etc/mabur.key on the drone and GS). The fingerprint must match both daemons' boot logs. Default pairs with any default install.</div>
        </div>
      {/if}
    </div>

    {#if !pinned}
    <div class={groupClass}>
      <div style="display:flex;align-items:center;justify-content:space-between;gap:8px">
        <div style="display:flex;flex-direction:column;gap:1px">
          <span class="card-kicker">Ladder rungs</span>
          <span class="dim6" style="font-size:11px">Most robust first, fastest last</span>
        </div>
        <button class="btn btn-ghost" type="button" style="font-size:13px" disabled={cfg.ladder.length >= MAX_RUNGS}
          onclick={() => setRung(-1, '__add')}><Icon name="plus" />Add rung</button>
      </div>
      {#if spotter}<div class="note"><Icon name="binoculars" size="14px" /><span>Not applied in spotter mode.</span></div>{/if}
      <div class="rg hdr"><span></span><span>MCS</span><span>BW</span><span>FEC base</span><span>FEC enh</span><span></span></div>
      <div class="rungs">
      {#each cfg.ladder as r, i}
        {@const warns = rungWarnings(cfg, i)}
        <div data-rung={i} class="rung" class:dragging={drag?.from === i}
          class:drop-above={drag && drag.over === i && drag.over < drag.from}
          class:drop-below={drag && drag.over === i && drag.over > drag.from}>
          <div class="rg">
            <button class="grip" type="button" title="Drag to reorder (or focus and use ↑/↓)"
              aria-label="Move rung {i + 1} (MCS {r.mcs}); arrow keys move it"
              onpointerdown={(e) => startRungDrag(e, i)} onpointermove={moveRungDrag}
              onpointerup={endRungDrag} onpointercancel={() => (drag = null)}
              onkeydown={(e) => rungKey(e, i)}><Icon name="dots-six-vertical" size="16px" /></button>
            <select class="input sm" aria-label="Rung {i} MCS" onchange={(e) => setRung(i, 'mcs', e.currentTarget.value)}>
              {#each MCS as m}<option value={m} selected={m === r.mcs}>{m}</option>{/each}
            </select>
            <select class="input sm" aria-label="Rung {i} bandwidth" onchange={(e) => setRung(i, 'bw', e.currentTarget.value)}>
              <option value="20" selected={r.bw === 20}>20</option><option value="40" selected={r.bw === 40}>40</option>
            </select>
            <input class="input sm num" type="number" step="0.05" min="0.1" max="2" value={r.ob} aria-label="Rung {i} FEC base"
              onchange={(e) => setRung(i, 'ob', e.currentTarget.value)}>
            <input class="input sm num" type="number" step="0.05" min="0.1" max="2" value={r.oe} aria-label="Rung {i} FEC enh"
              onchange={(e) => setRung(i, 'oe', e.currentTarget.value)}>
            <button class="btn btn-secondary trash" type="button" title="Remove rung" disabled={cfg.ladder.length <= 1}
              onclick={() => setRung(i, '__remove')}><Icon name="trash" /></button>
          </div>
          {#if warns.length}<div class="warn" style="padding-left:26px"><Icon name="warning" /><span>{warns.join(' · ')}</span></div>{/if}
        </div>
      {/each}
      </div>
      <div class="dim5" style="font-size:11px;text-wrap:pretty">FEC overhead is extra repair data per layer (0.5 = +50%). Keep base ≥ enh on every rung.</div>
    </div>
    {/if}

    <div class={groupClass}>
      <span class="card-kicker">DVR</span>
      <div class="field">
        <span id="{uid}dvr-label" class="fieldlabel">Recording target</span>
        <div class="seg" role="radiogroup" aria-labelledby="{uid}dvr-label">
          <label class="seg-opt"><input type="radio" name="{uid}dvr" checked={cfg.dvr === 'web'} onchange={() => set('dvr', 'web')}>mabur web</label>
          <label class="seg-opt"><input type="radio" name="{uid}dvr" checked={cfg.dvr === 'vtx'} disabled={spotter} onchange={() => set('dvr', 'vtx')}>VTX</label>
          <label class="seg-opt"><input type="radio" name="{uid}dvr" checked={cfg.dvr === 'both'} disabled={spotter} onchange={() => set('dvr', 'both')}>Both</label>
        </div>
        <div class="hint">{{ web: 'Recorded in this browser and downloaded when you stop.', vtx: 'Recorded on the VTX storage.', both: 'Recorded in this browser and on the VTX.' }[cfg.dvr]}</div>
        {#if spotter && cfg.dvr !== 'web'}<div class="note"><Icon name="binoculars" size="14px" /><span>Spotter records in this browser only.</span></div>{/if}
      </div>
    </div>
  </fieldset>

  <!-- Page-only, applied on the next frame: stays editable while live. -->
  <div class={groupClass}>
    <span class="card-kicker">Display</span>
    <div class="field">
      <span id="{uid}ct-label" class="fieldlabel">Colour correction</span>
      <div class="seg" role="radiogroup" aria-labelledby="{uid}ct-label">
        <label class="seg-opt"><input type="radio" name="{uid}ct" checked={cfg.colortrans} onchange={() => set('colortrans', true)}>On</label>
        <label class="seg-opt"><input type="radio" name="{uid}ct" checked={!cfg.colortrans} onchange={() => set('colortrans', false)}>Off</label>
      </div>
      <div class="hint">Undoes the drone camera's colortrans tuning. Turn off for a drone without the colortrans sensor file.</div>
    </div>
  </div>

  {#if applied}
    <div style="display:flex;gap:6px;align-items:center;font-size:12px" class="dim4">
      <Icon name="check-circle" size="15px" style="color:var(--color-accent)" /><span>{applied}</span></div>
  {/if}

  {#if recordings}
    <div class={groupClass}>
      <span class="card-kicker">Recordings</span>
      {@render recordings()}
    </div>
  {/if}
</div>

<style>
  .fieldlabel { display: block; font-size: 12px; margin-bottom: 5px; color: color-mix(in srgb, var(--color-text) 70%, transparent); }
  .lock { display: flex; gap: 8px; align-items: center; padding: var(--space-3) var(--space-4); border-radius: var(--radius-md);
          background: var(--color-accent-900); color: var(--color-accent-200); font-size: 12px; }
  .fs { border: 0; padding: 0; margin: 0; min-width: 0; display: flex; flex-direction: column; gap: var(--space-3); transition: opacity .15s; }
  .hint { font-size: 11px; color: var(--color-neutral-500); margin-top: 5px; }
  .note { display: flex; gap: 6px; align-items: flex-start; font-size: 12px; color: var(--color-accent-300); }
  .warn { display: flex; gap: 5px; align-items: center; font-size: 11px; color: var(--color-accent-300); margin-top: 4px; }
  .rg { display: grid; grid-template-columns: 20px minmax(0,1.1fr) minmax(0,1fr) minmax(0,1fr) minmax(0,1fr) 28px; gap: 6px; align-items: center; }
  .rungs { display: flex; flex-direction: column; gap: var(--space-4); }
  .rung { display: flex; flex-direction: column; gap: 3px; border-radius: var(--radius-sm, 4px);
          box-shadow: 0 0 0 transparent; transition: opacity .1s; }
  .rung.dragging { opacity: .45; }
  .rung.drop-above { box-shadow: 0 -2px 0 var(--color-accent); }
  .rung.drop-below { box-shadow: 0 2px 0 var(--color-accent); }
  .grip { display: flex; align-items: center; justify-content: center; width: 20px; height: 28px; padding: 0;
          border: 0; background: none; color: var(--color-neutral-500); cursor: grab; touch-action: none; user-select: none; }
  .grip:hover { color: var(--color-text); }
  .grip:active { cursor: grabbing; }
  .grip:disabled { cursor: default; }
  .rg.hdr { font-size: 10px; letter-spacing: .06em; text-transform: uppercase; color: var(--color-neutral-500); }
  .input.sm { min-height: 30px; padding: 3px 6px; font-size: 13px; }
  .trash { width: 28px; height: 28px; padding: 0; border-color: transparent; color: var(--color-neutral-500); }
  :global(.cardg) { gap: var(--space-4); flex: none; background: color-mix(in srgb, var(--color-bg) 50%, transparent); }
  .cfg :global(.group) { gap: var(--space-4); }
</style>
