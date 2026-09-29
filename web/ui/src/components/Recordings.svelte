<script>
  // OPFS recordings (spec §2.3): list, download, delete; never auto-deleted.
  import Icon from './Icon.svelte';
  import { formatBytes } from '../lib/localrec.js';
  let { items = [], active = null, storage = '', warn = null, available = true, onDownload, onDelete } = $props();
</script>

<div style="display:flex;flex-direction:column;gap:6px">
  {#if !available}
    <div class="note"><Icon name="warning" size="14px" /><span>Browser storage unavailable — local recording is off. VTX recording still works.</span></div>
  {:else}
    {#if storage}<div class="hint">{storage}</div>{/if}
    {#if warn}<div class="warn"><Icon name="warning" />{warn}</div>{/if}
    {#if !items.length}<div class="hint">No recordings in this browser.</div>{/if}
    {#each items as r (r.name)}
      <div style="display:flex;align-items:center;gap:8px;font-size:12px">
        <span class="num" style="flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap" title={r.name}>{r.name}</span>
        {#if r.name === active}
          <span class="tag tag-accent">recording</span>
        {:else}
          <span class="dim6 num">{r.size == null ? '–' : formatBytes(r.size)}</span>
          <button class="btn btn-ghost" type="button" style="font-size:12px" onclick={() => onDownload(r.name)}>Download</button>
          <button class="btn btn-ghost btn-icon" type="button" aria-label="Delete {r.name}" title="Delete" onclick={() => onDelete(r.name)}><Icon name="trash" /></button>
        {/if}
      </div>
    {/each}
  {/if}
</div>
