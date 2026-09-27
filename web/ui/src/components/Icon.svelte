<script>
  // Phosphor regular, inlined as SVG (vendored via @phosphor-icons/core,
  // no CDN). Only the icons the handoff lists.
  import plugs from '@phosphor-icons/core/assets/regular/plugs.svg?raw';
  import plugsConnected from '@phosphor-icons/core/assets/regular/plugs-connected.svg?raw';
  import cornersOut from '@phosphor-icons/core/assets/regular/corners-out.svg?raw';
  import cornersIn from '@phosphor-icons/core/assets/regular/corners-in.svg?raw';
  import chartLineUp from '@phosphor-icons/core/assets/regular/chart-line-up.svg?raw';
  import slidersHorizontal from '@phosphor-icons/core/assets/regular/sliders-horizontal.svg?raw';
  import bug from '@phosphor-icons/core/assets/regular/bug.svg?raw';
  import gearSix from '@phosphor-icons/core/assets/regular/gear-six.svg?raw';
  import x from '@phosphor-icons/core/assets/regular/x.svg?raw';
  import caretUp from '@phosphor-icons/core/assets/regular/caret-up.svg?raw';
  import caretDown from '@phosphor-icons/core/assets/regular/caret-down.svg?raw';
  import dotsSixVertical from '@phosphor-icons/core/assets/regular/dots-six-vertical.svg?raw';
  import broadcast from '@phosphor-icons/core/assets/regular/broadcast.svg?raw';
  import binoculars from '@phosphor-icons/core/assets/regular/binoculars.svg?raw';
  import lockSimple from '@phosphor-icons/core/assets/regular/lock-simple.svg?raw';
  import info from '@phosphor-icons/core/assets/regular/info.svg?raw';
  import warning from '@phosphor-icons/core/assets/regular/warning.svg?raw';
  import checkCircle from '@phosphor-icons/core/assets/regular/check-circle.svg?raw';
  import plus from '@phosphor-icons/core/assets/regular/plus.svg?raw';
  import trash from '@phosphor-icons/core/assets/regular/trash.svg?raw';
  import stop from '@phosphor-icons/core/assets/regular/stop.svg?raw';
  import record from '@phosphor-icons/core/assets/regular/record.svg?raw';

  const ICONS = {
    plugs, 'plugs-connected': plugsConnected, 'corners-out': cornersOut, 'corners-in': cornersIn,
    'chart-line-up': chartLineUp, 'sliders-horizontal': slidersHorizontal, bug, 'gear-six': gearSix,
    x, 'caret-up': caretUp, 'caret-down': caretDown, 'dots-six-vertical': dotsSixVertical,
    broadcast, binoculars, 'lock-simple': lockSimple, info, warning, 'check-circle': checkCircle,
    plus, trash, stop, record,
  };
  let { name, size = '1em', style = '' } = $props();
</script>

<span class="ph" aria-hidden="true" style="width:{size};height:{size};font-size:{size};{style}">{@html ICONS[name]}</span>

<style>
  .ph { display: inline-flex; flex: none; line-height: 0; }
  .ph :global(svg) { width: 100%; height: 100%; fill: currentColor; }
</style>
