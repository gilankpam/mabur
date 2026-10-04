// The page's config form model (spec 2026-09-27-web-ui §4.4). The form edits
// the channel set + link channel (the overlay's [radio] channels/channel, in
// both modes; --ch carries the start member, startChannel), width (--w and
// [radio] width) and,
// in GS mode, static_mcs / ladder (the overlay's [link], with max_mcs = 7 so
// the ladder flies as listed). The core merges the overlay into its embedded
// maburgs.default.toml, then validates with maburgs's own loader.
import { ht40Offset, DEFAULT_KEY_HEX } from './logic.mjs';

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
    channels: [40, 64, 112, 144], link: 'auto', width: 40, staticMcs: -1,
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

function normChannels(raw) {
  if (!Array.isArray(raw) || raw.length < 1 || raw.length > 8) return null;
  const out = raw.map(int);
  if (!out.every((c) => inRange(c, 1, 177))) return null;
  if (new Set(out).size !== out.length) return null;
  return out;
}

// Per-field fallback: anything missing or malformed takes the default, so a
// stale or hand-edited localStorage entry can never break the page.
export function normalizeConfig(raw) {
  const d = defaultConfig();
  if (!raw || typeof raw !== 'object') return d;
  const channels = normChannels(raw.channels) ?? d.channels;
  const lk = raw.link === 'auto' ? 'auto' : int(raw.link);
  const w = int(raw.width);
  const sm = int(raw.staticMcs);
  let ladder = d.ladder;
  if (Array.isArray(raw.ladder) && raw.ladder.length >= 1 && raw.ladder.length <= MAX_RUNGS) {
    const rungs = raw.ladder.map(normRung);
    if (rungs.every(Boolean)) ladder = rungs;
  }
  return {
    channels,
    link: lk === 'auto' || channels.includes(lk) ? lk : 'auto',
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
  // ?chs=40,64 sets the set; ?ch=N pins N (added when absent and room
  // remains); ?ch=auto unpins.
  const chs = normChannels(String(qs?.get('chs') ?? '').split(',').filter(Boolean));
  if (chs) {
    c.channels = chs;
    if (c.link !== 'auto' && !chs.includes(c.link)) c.link = 'auto';
  }
  const qch = qs?.get('ch');
  if (qch === 'auto') c.link = 'auto';
  else {
    const n = int(qch ?? '');
    if (inRange(n, 1, 177)) {
      if (!c.channels.includes(n) && c.channels.length < 8) c.channels = [...c.channels, n].sort((a, b) => a - b);
      if (c.channels.includes(n)) c.link = n;
    }
  }
  const qw = int(qs?.get('w') ?? '');
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

// Add or remove a set member. The set stays 1..8 (removing the last member or
// adding a ninth is a no-op); a removed pinned link channel falls back to auto.
export function toggleChannel(cfg, ch) {
  const has = cfg.channels.includes(ch);
  if (has && cfg.channels.length === 1) return cfg;
  if (!has && cfg.channels.length >= 8) return cfg;
  const channels = has ? cfg.channels.filter((c) => c !== ch) : [...cfg.channels, ch].sort((a, b) => a - b);
  const link = cfg.link !== 'auto' && !channels.includes(cfg.link) ? 'auto' : cfg.link;
  return { ...cfg, ladder: cfg.ladder.map((r) => ({ ...r })), channels, link };
}

// The chips the form shows: the chip list plus any member outside it (a
// ?ch= / ?chs= / stored member), so every member can be removed.
export function chipChannels(cfg) {
  return [...new Set([...CHANNELS, ...cfg.channels])].sort((a, b) => a - b);
}

// Mirror of common/include/mabur/channel_set.h channel_set_issue + the pin
// rule. null = OK, else a user-facing sentence.
export function checkChannelSet(channels, link, width) {
  if (!Array.isArray(channels) || channels.length < 1 || channels.length > 8) return 'Pick 1 to 8 channels.';
  for (const ch of channels) if (!inRange(ch, 1, 177)) return `Channel ${ch} is out of range (1–177).`;
  if (new Set(channels).size !== channels.length) return 'A channel is listed twice.';
  if (Number(width) === 40) {
    const off0 = ht40Offset(channels[0]);
    for (const ch of channels) {
      const off = ht40Offset(ch);
      if (off === 0) return `Channel ${ch} has no 40 MHz pair — pick 20 MHz or drop it.`;
      if (off !== off0) {
        return `Channel ${ch} is on the other side of the pair grid from channel ${channels[0]} — every member must share one offset at 40 MHz.`;
      }
    }
  }
  if (link !== 'auto' && !channels.includes(link)) return `Link channel ${link} is not in the set.`;
  return null;
}

export function channelWarning(cfg) {
  return cfg.width === 40 ? checkChannelSet(cfg.channels, cfg.link, 40) : null;
}

// Page-side mirror of what the core's loader would refuse (so Connect never
// ends in `ERROR bad config`). null = OK, else a user-facing sentence.
export function connectBlocker(cfg, mode, radio = 'usb') {
  const cs = checkChannelSet(cfg.channels, cfg.link, cfg.width);
  if (cs) return cs;
  if (radio === 'relay' && cfg.channels.includes(144)) {
    return 'The CPE relay cannot tune channel 144 — drop it from the set.';
  }
  if (mode !== 'gs') return null;   // spotter's [link] is fixed (one rung at the page width)
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

// [radio] in both modes (the core validates --ch against this set, and the
// set against this width -- not the bundle's); [link] + the ladder in GS
// mode. The spotter never walks a ladder, but the loader validates the
// bundle's against radio.width, so it gets one rung at the page width.
export function toOverlayToml(cfg, keyHex = null, mode = 'gs') {
  let t = `[radio]\nchannels = [${cfg.channels.join(', ')}]\nchannel = ${cfg.link === 'auto' ? '"auto"' : cfg.link}\nwidth = ${cfg.width}\n`;
  if (mode !== 'gs') {
    return t + `\n[link]\nstatic_mcs = -1\nstatic_bw = ${cfg.width}\nmax_mcs = 7\n`
      + `\n[[link.ladder]]\nmcs = 0\nbw = ${cfg.width}\noverhead_base = 0.5\noverhead_enh = 0.25\n`;
  }
  const pinned = cfg.staticMcs >= 0;
  // max_mcs 7: the form has no Max MCS -- the ladder is the whole policy, so
  // override the bundle's max_mcs filter rather than let it drop rungs.
  t += `\n[link]\n${keyHex ? `key = "${keyHex}"\n` : ''}static_mcs = ${cfg.staticMcs}\nstatic_bw = ${cfg.width}\nmax_mcs = 7\n`;
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

export const CHANNEL_STORE = 'webgs.channel';
export function loadRememberedChannel(storage) {
  try {
    const v = int(storage && storage.getItem(CHANNEL_STORE));
    return inRange(v, 1, 177) ? v : null;
  } catch { return null; }
}
export function saveRememberedChannel(storage, ch) {
  try { storage && storage.setItem(CHANNEL_STORE, String(ch)); } catch { /* page works without storage */ }
}
// What --ch carries: "start on this member" (spec 2026-10-04 §5.2). The pin
// wins; else the remembered channel if it is still a member; else the first.
export function startChannel(cfg, remembered) {
  if (cfg.link !== 'auto') return cfg.link;
  if (remembered !== null && remembered !== undefined && cfg.channels.includes(remembered)) return remembered;
  return cfg.channels[0];
}

const EDIT_LABEL = { channels: 'Channels', link: 'Link channel', width: 'Channel width', staticMcs: 'Fixed MCS' };
export function describeEdit(key, val) {
  if (key === 'dvr') return `Recording target set to ${{ web: 'mabur web', vtx: 'VTX', both: 'Both' }[val]}`;
  if (key === 'colortrans') return `Colour correction ${val ? 'on' : 'off'}`;
  const shown = key === 'channels' ? val.join(', ') : key === 'link' ? (val === 'auto' ? 'Auto' : val)
    : key === 'width' ? `${val} MHz` : key === 'staticMcs' && val < 0 ? 'Adaptive' : val;
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
