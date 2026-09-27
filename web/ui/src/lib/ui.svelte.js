// App-level UI state (handoff "State"). Svelte 5 runes module.
import { loadConfig } from './config.js';

function initialMode() {
  const q = new URLSearchParams(location.search).get('mode');
  if (q === 'gs' || q === 'spotter') return q;
  try {
    const last = JSON.parse(localStorage.getItem('webgs.last') || 'null');
    if (last && (last.mode === 'gs' || last.mode === 'spotter')) return last.mode;
  } catch { /* storage unavailable */ }
  return 'spotter';   // safety default: never command a drone by accident
}

function initialCfg() {
  let storage = null;
  try { storage = localStorage; } catch { /* unavailable */ }
  return loadConfig(storage, new URLSearchParams(location.search));
}

function resumeFlags() {
  // Set by the reload fallback (session stop timeout): land on Config.
  try {
    const v = sessionStorage.getItem('webgs.resume');
    sessionStorage.removeItem('webgs.resume');
    return v ? JSON.parse(v) : null;
  } catch { return null; }
}

const resume = resumeFlags();

export const ui = $state({
  tab: resume ? 'config' : 'stats',
  fs: false,
  cfgOpen: !!resume,
  statsVisible: true,
  floatOpen: true,
  fpos: null,
  applied: '',
  mode: initialMode(),
  cfg: initialCfg(),
  tick: 0,
});

export function saveMode(mode) {
  try { localStorage.setItem('webgs.last', JSON.stringify({ mode })); } catch { /* ignore */ }
}

export function reloadToConfig() {
  try { sessionStorage.setItem('webgs.resume', '1'); } catch { /* ignore */ }
  location.reload();
}
