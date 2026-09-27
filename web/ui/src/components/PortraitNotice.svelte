<script>
  import Icon from './Icon.svelte';
</script>
<div class="pn">
  <Icon name="device-rotate" size="40px" style="color:var(--color-accent)" />
  <span style="font-size:16px">Rotate to landscape</span>
  <span class="dim5" style="font-size:13px">The ground station runs in landscape on phones.</span>
</div>
<style>
  .pn { position: absolute; inset: 0; display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 12px;
        background: var(--color-bg); text-align: center; padding: 24px; z-index: 10; }
</style>
