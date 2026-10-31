// Pure view logic for the web GS "VTX" tab (no DOM). The core pushes the
// drone's parameter state as JSON via Module.onParams (common/include/
// mabur/config_params.h kTable, 12 rows); VtxPanel.svelte renders what these
// helpers compute. docs/mavlink.md documents the push shape.

// docs/camera-modes.md: sensor field-of-view mode by fps, and the encoder
// sizes that fit it.
export const MODE_SIZES = [
  { maxFps: 90, sizes: ['1920x1080', '1280x720'] },
  { maxFps: 100, sizes: ['1728x972', '1280x720'] },
  { maxFps: 120, sizes: ['1728x816', '1152x544', '864x408'] },
];

export function modeSizes(fps) {
  if (!Number.isInteger(fps) || fps < 1 || fps > 120) return [];
  return MODE_SIZES.find((m) => fps <= m.maxFps).sizes;
}

export function modeText(fps, size) {
  return `${fps}@${size}`;
}

// The camera-mode dropdown's frame rates, each offering every size of its
// sensor mode.
export const MODE_FPS = [60, 90, 100, 120];

const sizeLabel = (size) => size.replace('x', '×');

function modeLabel(fps, size) {
  return `${fps} fps · ${sizeLabel(size)}${fps > 90 ? ' · cropped view' : ''}`;
}

// How far the display's refresh may sit from a camera rate and still
// match it: the phase lock's own rate-mismatch window (docs/phase-lock.md
// "Rate mismatch"). A whole multiple is not a match.
export const DISPLAY_MATCH_HZ = 0.55;

const matchesDisplay = (fps, hz) => Math.abs(fps - hz) <= DISPLAY_MATCH_HZ;
const knownHz = (hz) => Number.isFinite(hz) && hz > 0;

// The VENC_MODE dropdown, as { value: "<fps>@<WxH>", label }: every size
// of each MODE_FPS rate, plus the drone's own value first when it runs one
// outside that list (an ssh-set rate or size), so the dropdown always
// shows what the drone runs. With the display's measured refresh
// (`displayHz`), only the rates that match it are offered -- a mismatched
// pair can't phase-lock and judders -- unless none does, or the rate isn't
// known yet, when every choice is.
export function modeChoices(liveValue, displayHz = null) {
  let choices = [];
  for (const fps of MODE_FPS) {
    for (const size of modeSizes(fps)) choices.push({ value: modeText(fps, size), label: modeLabel(fps, size) });
  }
  if (knownHz(displayHz) && MODE_FPS.some((fps) => matchesDisplay(fps, displayHz))) {
    choices = choices.filter((c) => c.value === liveValue || matchesDisplay(parseMode(c.value).fps, displayHz));
  }
  const live = parseMode(liveValue);
  if (live && !choices.some((c) => c.value === liveValue)) {
    choices.unshift({ value: liveValue, label: modeLabel(live.fps, live.size) });
  }
  return choices;
}

// The camera-mode hint's line about the display: whether the dropdown
// follows its refresh, naming it; '' while the rate isn't known.
export function modeDisplayNote(displayHz) {
  if (!knownHz(displayHz)) return '';
  const hz = Math.round(displayHz);
  return MODE_FPS.some((fps) => matchesDisplay(fps, displayHz))
    ? `Showing the rates that match this display (${hz} Hz).`
    : `No camera rate matches this display (${hz} Hz).`;
}

// The REC_SIZE dropdown for the camera mode the drone runs (`modeValue`,
// VENC_MODE's value): the sizes of that sensor mode, which share the
// camera's aspect and fit the capture window the recorder's scaler reads
// (docs/vtx-recorder.md), plus the camera's own size and the drone's
// current recording size when either is outside that list. Nothing above
// 1920x1080 is offered: that needs the 30 fps 4K sensor mode, which the
// link cannot run beside.
export function recSizeChoices(modeValue, recValue) {
  const m = parseMode(modeValue);
  const sizes = m ? [...modeSizes(m.fps)] : [];
  if (m && sizes.length && !sizes.includes(m.size)) sizes.push(m.size);
  if (recValue && !sizes.includes(recValue)) sizes.push(recValue);
  return sizes.map((size) => ({ value: size, label: sizeLabel(size) }));
}

// VENC_SLICES is shown as On/Off: on is 4 row slices, off is 1 (whole
// frames, so no slice salvage).
export const SLICES_ON = '4';
export const SLICES_OFF = '1';

// Which widget a row gets: 'mode' and 'recsize' dropdowns, an On/Off
// 'toggle', or a text/number 'input'.
export function rowWidget(row) {
  if (row.id === 'VENC_MODE') return 'mode';
  if (row.id === 'REC_SIZE') return 'recsize';
  if (row.type === 'bool' || row.id === 'VENC_SLICES') return 'toggle';
  return 'input';
}

// The wire text an On/Off row sends for each side.
export function toggleValues(row) {
  return row.id === 'VENC_SLICES' ? { on: SLICES_ON, off: SLICES_OFF } : { on: '1', off: '0' };
}

// Which side of an On/Off row `shown` (its value) lights: 'on', 'off', or
// null while the value is unknown. Any slice count above 1 is on.
export function toggleState(row, shown) {
  const t = toggleValues(row);
  if (shown === t.off) return 'off';
  if (row.id === 'VENC_SLICES') return /^\d+$/.test(shown ?? '') && Number(shown) > 1 ? 'on' : null;
  return shown === t.on ? 'on' : null;
}

// Mirror of config_params.h parse_mode_text: "<fps>@<WxH>", each number a
// run of 1..6 digits.
const MODE_RE = /^(\d{1,6})@(\d{1,6}x\d{1,6})$/;
export function parseMode(text) {
  const m = MODE_RE.exec(String(text));
  return m ? { fps: Number(m[1]), size: m[2] } : null;
}

// Turns a typed or toggled value into the wire text VENC_MODE etc. expect,
// or null when it cannot hold the drone's type. Returns the canonical text
// (trimmed, not re-formatted) so a round trip through the drone's own value
// reads back the same string the operator typed.
export function commitText(type, input) {
  if (type === 'bool') {
    if (typeof input === 'boolean') return input ? '1' : '0';
    if (typeof input !== 'string') return null;
    const s = input.trim();
    if (s === '1' || s === 'on' || s === 'true') return '1';
    if (s === '0' || s === 'off' || s === 'false') return '0';
    return null;
  }
  const s = String(input).trim();
  if (type === 'int') return /^-?\d+$/.test(s) ? s : null;
  if (type === 'real') {
    if (s === '') return null;
    return Number.isFinite(Number(s)) ? s : null;
  }
  if (type === 'str') {
    if (s.length < 1 || s.length > 127) return null;
    for (let i = 0; i < s.length; i++) {
      const c = s.charCodeAt(i);
      if (c < 0x20 || c > 0x7e) return null;
    }
    return s.includes('"') || s.includes('\\') ? null : s;
  }
  return null;
}

// The result line for text the page could not turn into the row's type
// (commitText returned null): shown locally, nothing is sent.
export function invalidText(type) {
  if (type === 'int') return 'not a valid integer';
  if (type === 'real') return 'not a valid number';
  if (type === 'str') return 'not valid text: 1–127 printable characters, no " or \\';
  return 'not a valid value';
}

// The "no heartbeat" tag: GS mode, linked, and the drone's heartbeat was
// seen on this link and has since gone quiet (alive is null before the
// first one, which is no verdict yet).
export function showNoHeartbeat(params, mode) {
  return mode !== 'spotter' && !!params.linked && params.alive === false;
}

// Whether the answer to a commit has arrived. `snap` is the row as the
// last push showed it when the commit went out ({ state, age_ms, status,
// sawPending }); the caller sets sawPending once a push shows the row
// sending/in_progress. Answered: the row went through sending/in_progress
// and left it, or it was refused at once -- a terminal result newer than
// the snapshot's (another state or status, or an age_ms that restarted).
export function commitAnswered(snap, row) {
  if (row.state === 'sending' || row.state === 'in_progress') return false;
  if (snap.sawPending) return true;
  if (row.state !== snap.state || (row.status ?? '') !== (snap.status ?? '')) return true;
  if (row.age_ms == null) return false;
  return snap.age_ms == null || row.age_ms < snap.age_ms;
}

// The tab's section order and headings: Radio last, after the settings an
// operator changes per flight.
export const SECTION_KICKER = {
  venc: 'Camera',
  encoder: 'Encoder',
  record: 'Recorder',
  low_power: 'Power',
  msp: 'Flight controller',
  radio: 'Radio',
};
const SECTION_ORDER = Object.keys(SECTION_KICKER);

// Rows the tab leaves off: still in the drone's table (and editable over
// MAVLink from elsewhere), but not an operator setting for this page.
export const HIDDEN = new Set(['RADIO_WALL_MARGN']);

// Each row's label: what the setting is, not its MAVLink id or config key.
export const LABELS = {
  RADIO_LDPC: 'LDPC coding',
  VENC_MODE: 'Camera mode',
  VENC_SLICES: 'Row slices',
  ENC_BITRATE_MAX: 'Max bitrate (kbps)',
  ENC_AIRTIME: 'Airtime budget',
  REC_ENABLE: 'Onboard recording',
  REC_BITRATE: 'Bitrate (kbps)',
  REC_FPS: 'Frame rate',
  REC_SIZE: 'Resolution',
  LOWPWR_ENABLE: 'Low power while disarmed',
  MSP_ENABLE: 'MSP',
};

export function rowLabel(row) {
  return LABELS[row.id] ?? row.id;
}

// Help text in place of the table's, where the table's spells the wire
// format or a config key rather than what the tab shows.
const HELP = {
  VENC_MODE: 'Camera frame rate and encoder size. Above 90 fps the sensor crops its view.',
  VENC_SLICES: 'Encodes each frame as 4 row slices, so the ground station can patch a frame that lost a slice instead of dropping it. Off sends whole frames.',
  REC_SIZE: 'Onboard recording resolution; the choices follow the camera mode.',
  LOWPWR_ENABLE: 'Lower bitrate and frame rate while disarmed to keep the VTX cool. Needs MSP on for the arm state.',
};

// The hint line under a row's widget: its help, then the one sentence
// about a row it interacts with, if any, each ending in a full stop.
export function rowHint(row) {
  const help = HELP[row.id] ?? row.help ?? '';
  return [help, HINTS[row.id]].filter(Boolean).map((t) => t.replace(/\.?$/, '.')).join(' ');
}

// The recorder's own settings, left off the tab while REC_ENABLE reads off
// (`recEnable`: its shown value). An unknown value hides nothing.
export const REC_SETTINGS = new Set(['REC_BITRATE', 'REC_FPS', 'REC_SIZE']);

export function rowShown(row, recEnable) {
  return !(REC_SETTINGS.has(row.id) && recEnable === '0');
}

// Groups the pushed rows by section, in SECTION_KICKER order (a section it
// doesn't name goes last), rows in table order, leaving out HIDDEN rows.
export function groupRows(rows) {
  const groups = [];
  const bySection = new Map();
  for (const row of rows) {
    if (HIDDEN.has(row.id)) continue;
    let g = bySection.get(row.section);
    if (!g) {
      g = { key: row.section, kicker: SECTION_KICKER[row.section] ?? row.section, rows: [] };
      bySection.set(row.section, g);
      groups.push(g);
    }
    g.rows.push(row);
  }
  const rank = (g) => { const i = SECTION_ORDER.indexOf(g.key); return i < 0 ? SECTION_ORDER.length : i; };
  return groups.sort((a, b) => rank(a) - rank(b));
}

// One sentence next to the rows this knob interacts with.
export const HINTS = {
  VENC_MODE: 'A change of aspect also sets the recording size to the new camera size',
  MSP_ENABLE: 'Turn low power off first',
};

export function headerText(params, mode) {
  if (mode === 'spotter') return 'VTX settings need GS mode.';
  if (!params.linked) return 'Waiting for the drone link…';
  if (params.mismatch) return "The drone's parameter table differs from this page — redeploy.";
  // Only the first read: the re-list after an accepted rebuild set keeps
  // every value on show, so it reads silently.
  if (params.list === 'pending' && !(params.rows?.length && params.rows.every((r) => r.value != null))) {
    return "Reading the drone's settings…";
  }
  if (params.list === 'partial') return 'Some settings did not answer.';
  return '';
}

// One row's render: the widget always shows the drone's value, never the
// typed text, once a terminal state arrives -- only while Sending/InProgress
// does it keep showing what the operator typed. `accepted` fades 3 s after
// the push that carried it; age_ms is frozen at push time, so the caller
// (on a ~1 s clock) passes how long ago that push arrived.
export function rowView(row, idx, pending, typed, sincePushMs = 0) {
  const busy = pending === idx;
  const disabled = pending !== -1 && !busy;
  const sendingLike = row.state === 'sending' || row.state === 'in_progress';
  const shown = sendingLike && typed != null ? typed : row.value ?? '';
  let result = '';
  let resultKind = '';
  switch (row.state) {
    case 'accepted':
      // A warning the drone sent with its ACCEPTED (RADIO_LDPC against walls
      // measured under the other setting) stays until the next set clears it.
      if (row.status) {
        result = `accepted: ${row.status}`;
        resultKind = 'warn';
      } else if ((row.age_ms ?? 0) + sincePushMs < 3000) {
        result = 'accepted';
        resultKind = 'ok';
      }
      break;
    case 'failed':
      result = `failed: ${row.status || 'refused'}`;
      resultKind = 'err';
      break;
    case 'unsupported':
      // VALUE_UNSUPPORTED carries no STATUSTEXT: the drone refused the
      // value's type or shape (or the id) without a reason line.
      result = `unsupported: ${row.status || "the drone refused this value's shape"}`;
      resultKind = 'err';
      break;
    case 'timeout':
      result = 'no answer';
      resultKind = 'err';
      break;
    case 'in_progress':
      result = 'applying…';
      resultKind = 'busy';
      break;
    case 'sending':
      result = 'sending…';
      resultKind = 'busy';
      break;
    default:
      break;
  }
  return { shown, disabled, busy, result, resultKind };
}

// The status at the right end of a row's label line, from its rowView `v`:
// { kind: 'busy' | 'ok' | 'warn' | 'err', text, reason? }, or null when
// there is nothing to say. `reason` is the line shown under the hint: an
// error's, or the drone's warning on an accepted set. `localErr` (an edit the page could not send) wins over the state.
export function rowStatus(row, v, localErr = null) {
  if (localErr) return { kind: 'err', text: 'Invalid', reason: localErr };
  if (v.resultKind === 'busy') return { kind: 'busy', text: row.state === 'sending' ? 'Sending…' : 'Applying…' };
  if (v.resultKind === 'ok') return { kind: 'ok', text: '✓ Saved' };
  if (v.resultKind === 'warn') return { kind: 'warn', text: '✓ Saved', reason: row.status };
  if (v.resultKind !== 'err') return null;
  if (row.state === 'timeout') {
    return { kind: 'err', text: 'No answer', reason: 'The drone did not answer; it may not have applied the change.' };
  }
  return { kind: 'err', text: 'Failed', reason: v.result.replace(/^(failed|unsupported): /, '') };
}
