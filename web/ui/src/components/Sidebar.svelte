<script>
  import Icon from './Icon.svelte';
  import StatsGroups from './StatsGroups.svelte';
  import DebugTables from './DebugTables.svelte';
  let { tab, onTab, v, spark, groups, onCopy, copyMsg, config } = $props();
</script>
<aside class="side rule-v mono">
  <div class="seg" style="flex:none">
    <label class="seg-opt"><input type="radio" name="webgs-tab" checked={tab === 'stats'} onchange={() => onTab('stats')}><Icon name="chart-line-up" />Stats</label>
    <label class="seg-opt"><input type="radio" name="webgs-tab" checked={tab === 'config'} onchange={() => onTab('config')}><Icon name="sliders-horizontal" />Config</label>
    <label class="seg-opt"><input type="radio" name="webgs-tab" checked={tab === 'debug'} onchange={() => onTab('debug')}><Icon name="bug" />Debug</label>
  </div>
  {#if tab === 'stats'}
    <div class="scroll scroll-thin" style="display:flex;flex-direction:column;gap:var(--space-3);padding:1px 8px 1px 1px">
      <StatsGroups {v} {spark} />
    </div>
  {:else if tab === 'config'}
    <div class="scroll scroll-thin" style="padding:var(--space-2) 8px var(--space-6) 1px">{@render config?.()}</div>
  {:else}
    <div class="scroll scroll-thin" style="display:flex;flex-direction:column;gap:var(--space-6);padding:1px 8px var(--space-6) 1px">
      <DebugTables {groups} {onCopy} {copyMsg} />
    </div>
  {/if}
</aside>
<style>
  .side { width: 336px; flex: none; display: flex; flex-direction: column; gap: var(--space-4); min-height: 0; padding-left: var(--space-6); }
  .scroll { flex: 1; min-height: 0; overflow-y: auto; overflow-x: hidden; margin-right: -9px; }
</style>
