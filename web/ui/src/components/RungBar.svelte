<script>
  let { segs = [], height = 4 } = $props();
</script>
<div class="bar">
  {#each segs as on}<div class="seg-b" class:on style="height:{height}px"></div>{/each}
</div>
<style>
  .bar { display: flex; gap: 3px; }
  .seg-b { flex: 1; border-radius: 2px; background: var(--color-neutral-800); }
  .seg-b.on { background: var(--color-accent); }
</style>
