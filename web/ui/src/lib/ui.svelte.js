// App-level UI state (handoff "State"). Svelte 5 runes module.
import { loadConfig, loadKey } from './config.js';
import { loadRelayCustom } from './logic.mjs';

function storageSafe() {
  try { return localStorage; } catch { return null; }
}

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
  return loadConfig(storageSafe(), new URLSearchParams(location.search));
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

function lastSaved() {
  try { return JSON.parse(localStorage.getItem('webgs.last') || 'null') || {}; } catch { return {}; }
}
const saved = lastSaved();

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
  key: loadKey(storageSafe()),
  radio: saved.radio === 'relay' ? 'relay' : 'usb',
  relayAddr: loadRelayCustom(saved),   // typed override; '' = use RELAY_DEFAULT
  relayFieldOpen: false,               // a Connect this visit found no relay
  tick: 0,
});

// Field visibility is decided by what was saved when the page loaded, so
// clearing the field hides it from the next visit, not mid-edit.
export const relaySavedAtLoad = ui.relayAddr !== '';

export function saveMode(mode, radio = 'usb', relayCustom = '') {
  try { localStorage.setItem('webgs.last', JSON.stringify({ mode, radio, relayCustom: String(relayCustom).trim() })); } catch { /* ignore */ }
}

export function reloadToConfig() {
  try { sessionStorage.setItem('webgs.resume', '1'); } catch { /* ignore */ }
  location.reload();
}
