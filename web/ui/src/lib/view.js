// Core stats + page metrics -> display strings for the Stats tab, the
// floating panel, the Debug tab and the live status line. Pure.
import { effectiveLadder } from './config.js';

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

export function statsView({ connected, mode, ch, w, core, page, sessionCfg, videoSize }) {
  const on = !!connected && !!core;
  const spot = mode === 'spotter';
  const pinned = !spot && sessionCfg.staticMcs >= 0;
  const eff = effectiveLadder(sessionCfg);
  const rung = on && !spot && !pinned && core.rung >= 0 ? core.rung : -1;
  const rssi = on ? core.rssi_dbm : null;
  const barPct = has(rssi) ? Math.max(5, Math.min(100, (rssi + 90) / 60 * 100)) : 0;
  return {
    mcs: on && core.mcs >= 0 ? String(core.mcs) : D,
    bw: on && core.bw ? String(core.bw) : D,
    rungNum: rung >= 0 ? String(rung + 1) : D,
    rungCount: String(eff.length),
    rungMode: spot ? 'Observed' : pinned ? 'Pinned' : 'Adaptive',
    segs: eff.map((_, i) => rung >= 0 && i <= rung),
    chLine: `${ch} · ${w} MHz`,
    cards: [{ idx: 0, tx: !spot, rssi: fx(rssi, 0), snr: fx(on ? core.snr_db : null, 0), barPct }],
    bestRssi: fx(rssi, 0),
    preLoss: on && has(core.pre_fec_loss) ? (core.pre_fec_loss * 100).toFixed(1) : D,
    postLoss: on && has(core.residual) ? (core.residual * 100).toFixed(2) : D,
    bitrate: on && page ? fx(page.bitrateMbps, 1) : D,
    latency: on && page ? fx(page.latencyMs, 0) : D,
    fps: on && page ? fx(page.fps, 0) : D,
    jitter: on && page ? fx(page.jitterMs, 1) : D,
    codecLine: on && videoSize ? `H.265 · ${videoSize.w}×${videoSize.h}` : 'H.265',
    latencyCaption: spot ? 'rx→glass latency, last 12 s' : 'Latency, last 12 s',
  };
}

function segCell(e) {
  if (!e || e.n === 0) return D;
  return `p99 ${fmt(e.p99)} max ${fmt(e.max)}`;
}
function segRow(name, seg) {
  const a = seg.w1[name], b = seg.w60[name];
  if ((!a || a.n === 0) && (!b || b.n === 0)) return D;
  return `${segCell(a)} | ${segCell(b)}`;
}

export function debugGroups({ connected, mode, core, rcfPct, ausRate, hitches60, hitchesTotal, seg }) {
  const on = !!connected && !!core;
  const spot = mode === 'spotter';
  const v = (x) => (on ? x : D);
  const rows = (pairs) => pairs.map(([k, val]) => ({ k, v: String(val) }));
  return [
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
    ]) },
    { title: 'Client', rows: rows([
      ['AUs/s', v(on ? ausRate : D)], ['hitches', v(`${hitches60} last 60s (total ${hitchesTotal})`)],
    ]) },
    { title: 'Latency (ms; 1 s | 60 s windows)', rows: rows([
      ['usb (core, 1 s)', v(has(core?.usb_p99_us) ? `p99 ${fmt(core.usb_p99_us / 1000)} max ${fmt(core.usb_max_us / 1000)}` : D)],
      ['fec (core, first→complete)', v(segRow('fec', seg))],
      ['handoff', v(segRow('handoff', seg))], ['decode', v(segRow('decode', seg))],
      ['present', v(segRow('present', seg))],
      ['capture→glass', v(spot ? 'needs GS mode' : segRow('capture→glass (GS)', seg))],
    ]) },
    { title: 'Counters', rows: rows([
      ['bodies', v(core?.bodies ?? D)], ['aus', v(core?.aus ?? D)], ['trunc', v(core?.trunc ?? D)],
      ['sends', v(core?.sends ?? D)], ['rcf_sent', v(core?.rcf_sent ?? D)],
      ['txfail', v(core?.txfail ?? D)], ['qdrop', v(core?.qdrop ?? D)],
    ]) },
  ];
}

// Live-state message over the video (the old page's status texts).
export function statusText({ state, mode, ch, w, core, gateArmed, sinceStartMs, waitingMs, hiddenBanner }) {
  if (state !== 'live') return '';
  let primary = '';
  if (mode === 'gs' && core && core.peer_acked === false && sinceStartMs >= 10000) {
    primary = `No drone on ch ${ch} / ${w} MHz (still trying)`;
  } else if (!gateArmed) {
    primary = `Waiting for key frame (sent on the next rung change) — ${Math.max(0, Math.round(waitingMs / 1000))} s`;
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
