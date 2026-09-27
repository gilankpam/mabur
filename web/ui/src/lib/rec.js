// VTX recorder state from the core's stats (Telem.rec_status, spec §4.5).
// drone/src/vtx_recorder.h RecErr codes.
export const REC_ERR_TEXT = {
  1: 'Recorder disabled on the VTX', 2: 'No SD slot', 3: 'No SD card',
  4: 'SD card not mounted', 5: 'SD card full', 6: 'SD write error',
};

export function recView(core) {
  if (!core || core.rec_state == null) return { state: 'unknown', err: null };
  if (core.rec_state === 1) return { state: 'recording', err: null };
  if (core.rec_state === 2) {
    return { state: 'error', err: REC_ERR_TEXT[core.rec_err] || `VTX recorder error ${core.rec_err}` };
  }
  return { state: 'off', err: null };
}

// The clock runs from the drone's FIRST "recording" report, not the click.
export class RecClock {
  constructor() { this.since = null; }
  update(state, nowMs) {
    if (state === 'recording') { if (this.since === null) this.since = nowMs; }
    else this.since = null;
  }
  elapsedMs(nowMs) { return this.since === null ? 0 : nowMs - this.since; }
}

export function formatClock(ms) {
  const s = Math.floor(ms / 1000);
  const p = (x) => String(x).padStart(2, '0');
  return `${p(Math.floor(s / 3600))}:${p(Math.floor(s / 60) % 60)}:${p(s % 60)}`;
}
