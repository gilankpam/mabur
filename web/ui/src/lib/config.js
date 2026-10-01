// The page's config form model (spec 2026-09-27-web-ui §4.4). The form edits
// channel/width (passed as --ch/--w) and, in GS mode, static_mcs / ladder
// (written as a TOML overlay, with max_mcs = 7 so the ladder flies as listed, the core merges into its embedded
// maburgs.default.toml, then validates with maburgs's own loader).
import { checkChannelWidth, ht40Offset, DEFAULT_KEY_HEX } from './logic.mjs';

export { DEFAULT_KEY_HEX };

export const CHANNELS = [36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124,
  128, 132, 136, 140, 144, 149, 153, 157, 161, 165];
export const MAX_RUNGS = 8;
const STORE_KEY = 'webgs.cfg';
// gs/src/config.cpp: overhead_base/overhead_enh range.
const OV_MIN = 0.1, OV_MAX = 2.0;

// gs/bundle/maburgs.default.toml
export function defaultConfig() {
  return {
    channel: 136, width: 40, staticMcs: -1,
    ladder: [0, 1, 2, 3, 4].map((mcs) => ({ mcs, bw: 40, ob: 0.5, oe: 0.25 })),
    // Page-only (never sent to the core): reverse the drone's colortrans
    // sensor tuning on the video (lib/colortrans.js). maburplay's bundle
    // default is on too.
    colortrans: true,
    // Page-only: which recorder(s) Record drives (lib/localrec.js).
    dvr: 'web',
  };
}

const int = (v) => (typeof v === 'number' || (typeof v === 'string' && /^-?\d+$/.test(v.trim()))
  ? Number(v) : NaN);
const inRange = (v, lo, hi) => Number.isInteger(v) && v >= lo && v <= hi;

function normRung(r) {
  if (!r || typeof r !== 'object') return null;
  const mcs = int(r.mcs), bw = int(r.bw), ob = Number(r.ob), oe = Number(r.oe);
  if (!inRange(mcs, 0, 7) || (bw !== 20 && bw !== 40) || !Number.isFinite(ob) || !Number.isFinite(oe)) return null;
  return { mcs, bw, ob, oe };
}

// Per-field fallback: anything missing or malformed takes the default, so a
// stale or hand-edited localStorage entry can never break the page.
export function normalizeConfig(raw) {
  const d = defaultConfig();
  if (!raw || typeof raw !== 'object') return d;
  const ch = int(raw.channel);
  const w = int(raw.width);
  const sm = int(raw.staticMcs);
  let ladder = d.ladder;
  if (Array.isArray(raw.ladder) && raw.ladder.length >= 1 && raw.ladder.length <= MAX_RUNGS) {
    const rungs = raw.ladder.map(normRung);
    if (rungs.every(Boolean)) ladder = rungs;
  }
  return {
    channel: inRange(ch, 1, 200) ? ch : d.channel,
    width: w === 20 || w === 40 ? w : d.width,
    staticMcs: inRange(sm, -1, 7) ? sm : d.staticMcs,
    ladder,
    colortrans: typeof raw.colortrans === 'boolean' ? raw.colortrans : d.colortrans,
    dvr: ['web', 'vtx', 'both'].includes(raw.dvr) ? raw.dvr : d.dvr,
  };
}

export function loadConfig(storage, qs) {
  let raw = null;
  try {
    const t = storage && storage.getItem(STORE_KEY);
    raw = t ? JSON.parse(t) : null;
  } catch { raw = null; }
  const c = normalizeConfig(raw);
  const qch = int(qs?.get('ch') ?? '');
  const qw = int(qs?.get('w') ?? '');
  if (inRange(qch, 1, 200)) c.channel = qch;
  if (qw === 20 || qw === 40) c.width = qw;
  return c;
}

export function saveConfig(storage, cfg) {
  try { storage && storage.setItem(STORE_KEY, JSON.stringify(cfg)); } catch { /* page works without storage */ }
}

export function rungWarnings(cfg, i) {
  const r = cfg.ladder[i];
  const out = [];
  if (r.bw === 40 && cfg.width === 20) out.push('40 MHz needs channel width 40');
  if (Number(r.oe) > Number(r.ob)) out.push('Enh overhead above base');
  return out;
}

export function channelWarning(cfg) {
  if (cfg.width === 40 && ht40Offset(cfg.channel) === 0) {
    return `Channel ${cfg.channel} has no 40 MHz pair — pick 20 MHz or another channel.`;
  }
  return null;
}

// Page-side mirror of what the core's loader would refuse (so Connect never
// ends in `ERROR bad config`). null = OK, else a user-facing sentence.
export function connectBlocker(cfg, mode) {
  const cw = checkChannelWidth(String(cfg.channel), String(cfg.width));
  if (cw) return cw;
  if (mode !== 'gs') return null;   // spotter sends no overlay
  if (cfg.staticMcs >= 0) return null;   // pinned: the ladder is hidden and not sent (toOverlayToml)
  for (let i = 0; i < cfg.ladder.length; i++) {
    const { ob, oe } = cfg.ladder[i];
    const b = Number(ob), e = Number(oe);
    const ok = (v) => (typeof v === 'number' || String(v).trim() !== '') && Number.isFinite(Number(v))
      && Number(v) >= OV_MIN && Number(v) <= OV_MAX;
    if (!ok(ob) || !ok(oe)) return `Rung ${i}: FEC overhead must be a number from ${OV_MIN} to ${OV_MAX}.`;
    if (e > b) return `Rung ${i}: enh overhead is above base — keep base ≥ enh.`;
  }
  if (cfg.width === 20) {
    const bad = cfg.ladder.findIndex((r) => r.bw === 40);
    if (bad >= 0) return `Rung ${bad} is 40 MHz but channel width is 20 — set width to 40 or the rung to 20.`;
  }
  return null;
}

const num = (v) => String(Number(v));

export function toOverlayToml(cfg, keyHex = null) {
  const pinned = cfg.staticMcs >= 0;
  // max_mcs 7: the form has no Max MCS -- the ladder is the whole policy, so
  // override the bundle's max_mcs filter rather than let it drop rungs.
  let t = `[link]\n${keyHex ? `key = "${keyHex}"\n` : ''}static_mcs = ${cfg.staticMcs}\nstatic_bw = ${cfg.width}\nmax_mcs = 7\n`;
  // Pinned, the form hides the ladder and the link never walks it, but
  // maburgs's loader still validates it (a 40 MHz rung at width
  // 20 fails boot). Send one rung that always loads instead; the saved
  // ladder is untouched for when Fixed MCS goes back to Adaptive.
  const ladder = pinned ? [{ mcs: cfg.staticMcs, bw: cfg.width, ob: 0.5, oe: 0.25 }] : cfg.ladder;
  for (const r of ladder) {
    t += `\n[[link.ladder]]\nmcs = ${r.mcs}\nbw = ${r.bw}\noverhead_base = ${num(r.ob)}\noverhead_enh = ${num(r.oe)}\n`;
  }
  return t;
}

export function applyEdit(cfg, key, val) {
  return { ...cfg, ladder: cfg.ladder.map((r) => ({ ...r })), [key]: val };
}

export function applyRungEdit(cfg, i, key, val) {
  const L = cfg.ladder.map((r) => ({ ...r }));
  if (key === '__add') {
    if (L.length >= MAX_RUNGS) return { ...cfg, ladder: L };
    const last = L[L.length - 1] || { mcs: 0, bw: 40 };
    L.push({ mcs: Math.min(7, last.mcs + 1), bw: last.bw, ob: 0.5, oe: 0.25 });
  } else if (key === '__remove') {
    if (L.length > 1) L.splice(i, 1);
  } else if (key === '__move') {
    // val = destination index; the rung is taken out and re-inserted there.
    const to = Number(val);
    if (Number.isInteger(to) && to >= 0 && to < L.length && i >= 0 && i < L.length && to !== i) {
      L.splice(to, 0, L.splice(i, 1)[0]);
    }
  } else {
    L[i] = { ...L[i], [key]: key === 'mcs' || key === 'bw' ? Number(val) : val };
  }
  return { ...cfg, ladder: L };
}

const EDIT_LABEL = { channel: 'Channel', width: 'Channel width', staticMcs: 'Fixed MCS' };
export function describeEdit(key, val) {
  if (key === 'dvr') return `Recording target set to ${{ web: 'mabur web', vtx: 'VTX', both: 'Both' }[val]}`;
  if (key === 'colortrans') return `Colour correction ${val ? 'on' : 'off'}`;
  const shown = key === 'width' ? `${val} MHz` : key === 'staticMcs' && val < 0 ? 'Adaptive' : val;
  return `${EDIT_LABEL[key]} set to ${shown}`;
}
const RUNG_LABEL = { mcs: 'MCS', bw: 'BW', ob: 'FEC base', oe: 'FEC enh' };
export function describeRungEdit(cfg, i, key, val) {
  if (key === '__add') return `Rung ${cfg.ladder.length - 1} added`;
  if (key === '__remove') return `Rung ${i} removed`;
  // cfg is the post-move config: the rung now sits at val.
  if (key === '__move') return `MCS ${cfg.ladder[val].mcs} rung moved to position ${val + 1} of ${cfg.ladder.length}`;
  return `Rung ${i} ${RUNG_LABEL[key]} set to ${val}`;
}

export const KEY_STORE = 'webgs.key';
export function parseKeyText(text) {
  const tokens = String(text).split(/\r?\n/).map((l) => l.trim()).filter((l) => l && !l.startsWith('#'));
  if (tokens.length === 0) throw new Error('no key in file');
  if (tokens.length > 1) throw new Error('more than one key in file');
  const t = tokens[0].toLowerCase();
  if (!/^[0-9a-f]{32}$/.test(t)) throw new Error('key is not 32 hex characters');
  return t;
}
export function loadKey(storage) {
  try {
    const v = storage && storage.getItem(KEY_STORE);
    return v && /^[0-9a-f]{32}$/.test(v) ? v : null;
  } catch { return null; }
}
export function saveKey(storage, hex) {
  try {
    if (!storage) return;
    if (hex) storage.setItem(KEY_STORE, hex); else storage.removeItem(KEY_STORE);
  } catch { /* page works without storage */ }
}
