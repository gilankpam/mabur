<script>
  let { groups, onCopy, copyMsg = '' } = $props();
</script>
{#each groups as grp}
  <table class="table" style="font-size:13px;flex:none">
    <thead><tr><th colspan="2" style="padding-left:0">{grp.title}</th></tr></thead>
    <tbody>
      {#each grp.rows as row}
        <tr><td style="padding-left:0;color:var(--color-neutral-300)">{row.k}</td>
          <td class="mono num" style="padding-right:0;text-align:right;font-size:12px;white-space:nowrap">{row.v}</td></tr>
      {/each}
    </tbody>
  </table>
{/each}
<div style="display:flex;align-items:center;gap:8px">
  <button class="btn btn-secondary" type="button" onclick={onCopy}>Copy stats</button>
  <span class="dim5" style="font-size:12px" aria-live="polite">{copyMsg}</span>
</div>
