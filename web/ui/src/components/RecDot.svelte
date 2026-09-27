<script>
  let { on = false, size = 9 } = $props();
</script>
<span class="dot" class:on style="width:{size}px;height:{size}px"></span>
<style>
  .dot { border-radius: 50%; flex: none; background: transparent; border: 1.5px solid var(--color-neutral-400); }
  .dot.on { background: var(--color-accent-400); border: 0; animation: recpulse 1.2s infinite; }
</style>
