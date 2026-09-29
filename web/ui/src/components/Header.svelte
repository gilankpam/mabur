<script>
  import Icon from './Icon.svelte';
  import RecDot from './RecDot.svelte';
  // Set by the Pages workflow: 'edge' for PR builds, with '#PR sha'.
  const edge = import.meta.env.VITE_CHANNEL === 'edge';
  const buildRef = import.meta.env.VITE_BUILD_REF || '';
  let { tag, chLine, live, busy, blocked = false, onConnect, onDisconnect, rec, recLabel, recDisabled, recTitle, onRec, onFs } = $props();
</script>
<header class="nav" style="gap:var(--space-4);padding:var(--space-3) var(--space-6);min-height:52px">
  <div style="display:flex;align-items:baseline;gap:8px;margin-right:auto">
    <span class="nav-brand" style="margin:0">mabur</span><span class="dim5" style="font-size:13px">web</span>
    {#if edge}<span class="tag tag-accent num" title="Edge build from a pull request; stable is at ../">edge {buildRef}</span>{/if}
  </div>
  {#if tag.on}
    <span class="tag tag-accent" style="gap:6px"><span style="width:6px;height:6px;border-radius:50%;background:var(--color-accent-300)"></span>{tag.label}</span>
  {:else}
    <span class="tag tag-neutral" style="gap:6px"><span style="width:6px;height:6px;border-radius:50%;border:1px solid var(--color-neutral-400)"></span>{tag.label}</span>
  {/if}
  <span class="tag tag-neutral num">CH {chLine}</span>
  {#if live}
    <button class="btn btn-secondary" type="button" onclick={onDisconnect}><Icon name="plugs" />Disconnect</button>
  {:else}
    <button class="btn btn-primary" type="button" onclick={onConnect} disabled={busy || blocked}><Icon name="plugs-connected" />Connect</button>
  {/if}
  <button class="btn btn-secondary num" type="button" style="gap:8px" onclick={onRec} disabled={recDisabled} title={recTitle}>
    <RecDot on={rec} size={9} />{recLabel}</button>
  <button class="btn btn-secondary btn-icon" type="button" title="Fullscreen (F)" aria-label="Fullscreen" onclick={onFs}><Icon name="corners-out" size="18px" /></button>
</header>
