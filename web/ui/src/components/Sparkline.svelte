<script>
  let { points = '0,24 100,24' } = $props();
</script>
<svg viewBox="0 0 100 24" preserveAspectRatio="none" style="width:100%;height:32px;display:block" aria-hidden="true">
  <polyline {points} fill="none" stroke="var(--color-accent)" stroke-width="1.2" vector-effect="non-scaling-stroke" />
</svg>
