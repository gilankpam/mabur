// Core stats + page metrics -> display strings for the Stats tab, the
// floating panel, the Debug tab and the live status line. Pure.
import { formatBytes } from './localrec.js';

const D = '–';
const has = (v) => v !== null && v !== undefined && !(typeof v === 'number' && Number.isNaN(v));
const fx = (v, d) => (has(v) ? Number(v).toFixed(d) : D);
const fmt = (v, d = 1, suf = '') => (has(v) ? Number(v).toFixed(d) + suf : D);

export function latencyNow(snap, mode) {
  const w1 = snap?.w1 || {};
  if (mode === 'gs') {
    const e = w1['capture→glass (GS)'];
    return e && e.n > 0 ? e.p50 : null;
  }
  const parts = ['fec', 'handoff', 'decode', 'present'].map((k) => w1[k]);
  if (parts.some((e) => !e || e.n === 0)) return null;
  return parts.reduce((n, e) => n + e.p50, 0);
}

export function statsView({ connected, mode, ch, w, core, page, sessionCfg, videoSize, colour }) {
  const on = !!connected && !!core;
  const spot = mode === 'spotter';
  const pinned = !spot && sessionCfg.staticMcs >= 0;
  const n = sessionCfg.ladder.length;
  const rung = on && !spot && !pinned && core.rung >= 0 ? core.rung : -1;
  const rssi = on ? core.rssi_dbm : null;
  const barPct = has(rssi) ? Math.max(5, Math.min(100, (rssi + 90) / 60 * 100)) : 0;
  return {
    mcs: on && core.mcs >= 0 ? String(core.mcs) : D,
    bw: on && core.bw ? String(core.bw) : D,
    rungNum: rung >= 0 ? String(rung + 1) : D,
    rungCount: String(n),
    rungMode: spot ? 'Observed' : pinned ? 'Pinned' : 'Adaptive',
    // A spotter drives no ladder: one unlit full-width line, not a rung per segment.
    segs: spot ? [false] : sessionCfg.ladder.map((_, i) => rung >= 0 && i <= rung),
    chLine: `${ch} · ${w} MHz`,
    cards: [{ idx: 0, tx: !spot, rssi: fx(rssi, 0), snr: fx(on ? core.snr_db : null, 0), barPct }],
    bestRssi: fx(rssi, 0),
    // Telem.soc_temp_c, 1 Hz; the core sends null until a Telem with a
    // reading lands (-128 on the wire = the drone has no thermal source).
    droneTemp: on ? fx(core.drone_temp_c, 0) : D,
    preLoss: on && has(core.pre_fec_loss) ? (core.pre_fec_loss * 100).toFixed(1) : D,
    postLoss: on && has(core.residual) ? (core.residual * 100).toFixed(2) : D,
    bitrate: on && page ? fx(page.bitrateMbps, 1) : D,
    latency: on && page ? fx(page.latencyMs, 0) : D,
    fps: on && page ? fx(page.fps, 0) : D,
    jitter: on && page ? fx(page.jitterMs, 1) : D,
    codecLine: (on && videoSize ? `H.265 · ${videoSize.w}×${videoSize.h}` : 'H.265')
      + (colour === 'colortrans' ? ' · colortrans' : colour === 'unavailable' ? ' · colortrans unavailable' : ''),
    latencyCaption: spot ? 'rx→glass latency, last 12 s' : 'Latency, last 12 s',
  };
}

function segCell(e) {
  if (!e || e.n === 0) return D;
  return `p99 ${fmt(e.p99)} max ${fmt(e.max)}`;
}
function segRow(name, seg) {
  const a = (seg.w1 || {})[name], b = (seg.w60 || {})[name];
  if ((!a || a.n === 0) && (!b || b.n === 0)) return D;
  return `${segCell(a)} | ${segCell(b)}`;
}

// rateBps = bytes/s over the last stats tick (App computes it).
function lrecText(l) {
  if (!l || l.state === 'off' || l.state === 'unknown') return 'off';
  if (l.state === 'waiting') return 'waiting for sync';
  const size = formatBytes(l.bytes || 0);
  if (l.state === 'error') return `error · ${size}`;
  return `${l.state} · ${size} · ${((l.rateBps || 0) * 8 / 1e6).toFixed(1)} Mb/s`;
}

export function debugGroups({ connected, mode, core, rcfPct, ausRate, hitches60, hitchesTotal, seg, lrec }) {
  const on = !!connected && !!core;
  const spot = mode === 'spotter';
  const isRelay = on && core.radio === 'relay';
  const v = (x) => (on ? x : D);
  const rows = (pairs) => pairs.map(([k, val]) => ({ k, v: String(val) }));
  const groups = [
    { title: 'Link', rows: rows([
      ['mode', v(core?.mode)], ['session', v(core?.session ? 'yes' : 'no')],
      ['peer_acked', v(spot ? 'n/a' : core?.peer_acked ? 'yes' : 'no')],
      ['rung', v(core?.rung >= 0 ? core.rung : D)], ['mcs', v(core?.mcs >= 0 ? core.mcs : D)],
      ['width', v(core?.bw ? core.bw + ' MHz' : D)], ['probe', v(core?.probe || 'off')],
    ]) },
    { title: 'Radio', rows: rows([
      ['pre-FEC loss', v(has(core?.pre_fec_loss) ? (core.pre_fec_loss * 100).toFixed(1) + ' %' : D)],
      ['residual', v(has(core?.residual) ? (core.residual * 100).toFixed(2) + ' %' : D)],
      ['SNR', v(fmt(core?.snr_db, 1, ' dB'))], ['RSSI', v(fmt(core?.rssi_dbm, 1, ' dBm'))],
      ['RTT', v(!spot && has(core?.rtt_ms) ? `${fmt(core.rtt_ms)} ms (min ${fmt(core.rtt_min_ms)} ms)` : D)],
      ['RCF heard %', v(!spot && rcfPct != null ? rcfPct.toFixed(1) : D)],
    ]) },
    { title: 'Drone', rows: rows([
      ['drone_state', v(core?.drone_state ?? D)], ['drone_rcf_rx', v(core?.drone_rcf_rx ?? D)],
      ['IDR req', v(spot ? 'n/a' : core?.idr_req ?? D)],
    ]) },
    { title: 'Client', rows: rows([
      ['AUs/s', v(on ? ausRate : D)], ['hitches', v(`${hitches60} last 60s (total ${hitchesTotal})`)],
      ['local rec', v(lrecText(lrec))],
    ]) },
    // The USB latency/txfail rows are meaningless over a relay radio (no
    // USB transfers, no local TX queue -- tx/fail/refused live in the Relay
    // group instead), so they're left out entirely rather than shown as D.
    { title: 'Latency (ms; 1 s | 60 s windows)', rows: rows([
      ...(isRelay ? [] : [['usb (core, 1 s)',
        v(has(core?.usb_p99_us) ? `p99 ${fmt(core.usb_p99_us / 1000)} max ${fmt(core.usb_max_us / 1000)}` : D)]]),
      ['fec (core, first→complete)', v(segRow('fec', seg))],
      ['handoff', v(segRow('handoff', seg))], ['decode', v(segRow('decode', seg))],
      ['present', v(segRow('present', seg))],
      ['capture→glass', v(spot ? 'needs GS mode' : segRow('capture→glass (GS)', seg))],
    ]) },
    { title: 'Counters', rows: rows([
      ['bodies', v(core?.bodies ?? D)], ['aus', v(core?.aus ?? D)], ['trunc', v(core?.trunc ?? D)],
      ['sends', v(core?.sends ?? D)], ['rcf_sent', v(core?.rcf_sent ?? D)],
      ...(isRelay ? [] : [['txfail', v(core?.txfail ?? D)]]),
      ['qdrop', v(core?.qdrop ?? D)],
    ]) },
  ];
  if (isRelay) {
    const SEC = ['HT20', 'HT40+', 'HT40-'];
    groups.push({ title: 'Relay', rows: rows([
      ['channel', core.relay_state === 1 ? 'retuning' : `${core.relay_ch} ${SEC[core.relay_sec] ?? '?'}`],
      ['owner', core.relay_you_own ? 'yes' : 'no'],
      ['tuned', core.relay_owned ? 'yes' : 'no'],
      ['frames', core.relay_frames], ['seq gaps', core.relay_gaps],
      ['ring drops rx / tx', `${core.relay_rx_drops} / ${core.relay_tx_ring_drops}`],
      ['tx / fail / refused', `${core.relay_tx} / ${core.relay_tx_fail} / ${core.relay_tx_refused}`],
      ['your drops', core.relay_your_drops],
    ]) });
  }
  return groups;
}

// Live-state message over the video. Before the first picture of a Connect
// the screen is black, so say what the page is waiting for; after it, a
// frozen gate shows nothing here: the last frame stays up while the page
// requests an IDR (spec 2026-09-28).
export function statusText({ state, mode, ch, core, sinceStartMs, w, hiddenBanner, hasPicture }) {
  if (state !== 'live') return '';
  let primary = '';
  const noPeer = mode === 'gs' && core?.peer_acked !== true;
  if (noPeer && core && core.peer_acked === false && sinceStartMs >= 10000) {
    primary = `No drone on ch ${ch} / ${w} MHz (still trying)`;
  } else if (!hasPicture) {
    primary = noPeer ? `Searching for drone on ch ${ch} / ${w} MHz…` : 'Waiting for video…';
  }
  const banner = hiddenBanner ? 'GS mode keeps flying the link while this tab is hidden.' : '';
  return [primary, banner].filter(Boolean).join('\n');
}

export function linkTag({ state, mode, core }) {
  if (state === 'connecting') return { label: 'Connecting…', on: false };
  if (state === 'stopping') return { label: 'Disconnecting…', on: false };
  if (state !== 'live') return { label: 'Disconnected', on: false };
  if (mode === 'spotter') return { label: 'Spotter', on: true };
  return { label: core?.session && core?.peer_acked ? 'Linked' : 'Searching', on: true };
}
