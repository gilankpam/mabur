import test from 'node:test';
import assert from 'node:assert/strict';
import { statsView, debugGroups, latencyNow, statusText, linkTag } from '../ui/src/lib/view.js';
import { defaultConfig, applyEdit } from '../ui/src/lib/config.js';

const core = { mode: 'gs', session: true, peer_acked: true, rung: 3, mcs: 3, bw: 40, probe: 'armed',
  pre_fec_loss: 0.032, residual: 0.0004, snr_db: 28.4, rssi_dbm: -58.2, rtt_ms: 8, rtt_min_ms: 4.2,
  drone_state: 2, drone_rcf_rx: 1533, rec_state: 0, rec_err: 0, usb_p99_us: 200, usb_max_us: 250 };
const page = { bitrateMbps: 17.84, fps: 59.6, jitterMs: 1.94, latencyMs: 38.2 };
const base = { connected: true, mode: 'gs', ch: 136, w: 40, core, page,
  sessionCfg: defaultConfig(), videoSize: { w: 1280, h: 720 } };

test('stats view, GS adaptive', () => {
  const v = statsView(base);
  assert.equal(v.mcs, '3'); assert.equal(v.bw, '40');
  assert.equal(v.rungNum, '4'); assert.equal(v.rungCount, '5'); assert.equal(v.rungMode, 'Adaptive');
  assert.deepEqual(v.segs, [true, true, true, true, false]);
  assert.equal(v.chLine, '136 · 40 MHz');
  assert.equal(v.cards.length, 1);
  assert.deepEqual({ ...v.cards[0], barPct: undefined }, { idx: 0, tx: true, rssi: '-58', snr: '28', barPct: undefined });
  assert.ok(Math.abs(v.cards[0].barPct - 53) < 1e-6);
  assert.equal(v.preLoss, '3.2'); assert.equal(v.postLoss, '0.04');
  assert.equal(v.bitrate, '17.8'); assert.equal(v.fps, '60'); assert.equal(v.jitter, '1.9');
  assert.equal(v.latency, '38'); assert.equal(v.codecLine, 'H.265 · 1280×720');
});

test('codec line names the colour path', () => {
  assert.equal(statsView({ ...base, colour: 'colortrans' }).codecLine, 'H.265 · 1280×720 · colortrans');
  assert.equal(statsView({ ...base, colour: 'unavailable' }).codecLine, 'H.265 · 1280×720 · colortrans unavailable');
  assert.equal(statsView({ ...base, colour: 'flat' }).codecLine, 'H.265 · 1280×720');
});

test('rung count is the effective ladder (max MCS filter)', () => {
  const v = statsView({ ...base, sessionCfg: applyEdit(defaultConfig(), 'maxMcs', 2),
    core: { ...core, rung: 1, mcs: 1 } });
  assert.equal(v.rungCount, '3'); assert.equal(v.segs.length, 3);
});

test('pinned and spotter modes', () => {
  const p = statsView({ ...base, sessionCfg: applyEdit(defaultConfig(), 'staticMcs', 2),
    core: { ...core, rung: -1, mcs: 2 } });
  assert.equal(p.rungMode, 'Pinned'); assert.equal(p.rungNum, '–');
  assert.ok(p.segs.every((s) => !s));
  const s = statsView({ ...base, mode: 'spotter', core: { ...core, mode: 'spotter', rung: -1 } });
  assert.equal(s.rungMode, 'Observed'); assert.ok(s.segs.every((x) => !x));
  assert.equal(s.cards[0].tx, false);
  assert.equal(s.latencyCaption, 'rx→glass latency, last 12 s');
});

test('disconnected: every value is a dash, bars empty', () => {
  const v = statsView({ ...base, connected: false, core: null, page: null });
  for (const k of ['mcs', 'bw', 'preLoss', 'postLoss', 'bitrate', 'latency', 'fps', 'jitter', 'bestRssi']) {
    assert.equal(v[k], '–', k);
  }
  assert.ok(v.segs.every((s) => !s));
  assert.equal(v.cards[0].rssi, '–'); assert.equal(v.cards[0].barPct, 0);
});

test('rssi bar clamps to 5..100 %', () => {
  assert.equal(statsView({ ...base, core: { ...core, rssi_dbm: -95 } }).cards[0].barPct, 5);
  assert.equal(statsView({ ...base, core: { ...core, rssi_dbm: -20 } }).cards[0].barPct, 100);
});

test('latency pick: GS capture->glass, spotter rx->glass sum', () => {
  const seg = (p50, n = 5) => ({ p50, p99: p50, max: p50, n });
  const snap = { w1: { 'capture→glass (GS)': seg(24), fec: seg(6), handoff: seg(0.1), decode: seg(0.8), present: seg(8) } };
  assert.equal(latencyNow(snap, 'gs'), 24);
  assert.ok(Math.abs(latencyNow(snap, 'spotter') - 14.9) < 1e-9);
  assert.equal(latencyNow({ w1: {} }, 'gs'), null);
  assert.equal(latencyNow({ w1: { fec: seg(6) } }, 'spotter'), null);
});

test('debug groups carry the handoff keys verbatim + counters', () => {
  const g = debugGroups({ connected: true, mode: 'gs', core, rcfPct: 97.2, ausRate: 60,
    hitches60: 0, hitchesTotal: 18, seg: { w1: {}, w60: {} } });
  assert.deepEqual(g.map((x) => x.title), ['Link', 'Radio', 'Drone', 'Client',
    'Latency (ms; 1 s | 60 s windows)', 'Counters']);
  assert.deepEqual(g[0].rows.map((r) => r.k), ['mode', 'session', 'peer_acked', 'rung', 'mcs', 'width', 'probe']);
  assert.deepEqual(g[1].rows.map((r) => r.k), ['pre-FEC loss', 'residual', 'SNR', 'RSSI', 'RTT', 'RCF heard %']);
  assert.equal(g[3].rows[1].v, '0 last 60s (total 18)');
  const sp = debugGroups({ connected: true, mode: 'spotter', core: { ...core, mode: 'spotter' }, rcfPct: null,
    ausRate: 60, hitches60: 0, hitchesTotal: 0, seg: { w1: {}, w60: {} } });
  assert.equal(sp[0].rows[2].v, 'n/a'); assert.equal(sp[1].rows[4].v, '–');
  const off = debugGroups({ connected: false, mode: 'gs', core: null, rcfPct: null, ausRate: 0,
    hitches60: 0, hitchesTotal: 0, seg: { w1: {}, w60: {} } });
  assert.ok(off.every((grp) => grp.rows.every((r) => r.v === '–')));
});

test('debug Client group shows the local recorder', () => {
  const base = { connected: true, mode: 'gs', core: { mode: 'gs' }, rcfPct: null, ausRate: 60,
    hitches60: 0, hitchesTotal: 0, seg: { w1: {}, w60: {} } };
  const row = (g) => g.find((x) => x.title === 'Client').rows.find((r) => r.k === 'local rec').v;
  assert.equal(row(debugGroups({ ...base, lrec: { state: 'recording', bytes: 3 * 1024 * 1024, rateBps: 4e6 } })),
    'recording · 3.0 MB · 32.0 Mb/s');
  assert.equal(row(debugGroups({ ...base, lrec: { state: 'waiting', bytes: 0, rateBps: 0 } })), 'waiting for sync');
  assert.equal(row(debugGroups({ ...base, lrec: null })), 'off');
  assert.equal(row(debugGroups({ ...base, connected: false, lrec: null })), '–');
});

test('debug Drone group shows IDR req / served in GS, n/a in spotter', () => {
  const args = { connected: true, mode: 'gs', core: { mode: 'gs', idr_req: 4, drone_idr_gs: 3 },
    rcfPct: null, ausRate: 60, hitches60: 0, hitchesTotal: 0, seg: { w1: {}, w60: {} } };
  const row = (g) => g.find((x) => x.title === 'Drone').rows.find((r) => r.k === 'IDR req / served').v;
  assert.equal(row(debugGroups(args)), '4 / 3');
  assert.equal(row(debugGroups({ ...args, core: { mode: 'gs', idr_req: 1, drone_idr_gs: null } })), '1 / –');
  assert.equal(row(debugGroups({ ...args, mode: 'spotter' })), 'n/a');
});

test('status text before the first picture says what the page is waiting for', () => {
  const at = (over) => statusText({ state: 'live', mode: 'gs', ch: 136, w: 40, sinceStartMs: 3000,
    hiddenBanner: false, hasPicture: false, ...over });
  // GS, drone not answered yet (and not yet the 10 s "No drone" verdict).
  assert.equal(at({ core: { peer_acked: false } }), 'Searching for drone on ch 136 / 40 MHz…');
  assert.equal(at({ core: null }), 'Searching for drone on ch 136 / 40 MHz…');
  // GS, drone answered, no frame drawn yet.
  assert.equal(at({ core: { peer_acked: true } }), 'Waiting for video…');
  // Spotter never has peer_acked: it only waits for video.
  assert.equal(at({ mode: 'spotter', core: {} }), 'Waiting for video…');
  // The 10 s verdict still wins over the searching text.
  assert.equal(at({ core: { peer_acked: false }, sinceStartMs: 12000 }),
    'No drone on ch 136 / 40 MHz (still trying)');
  // The hidden-tab banner stacks under it.
  assert.equal(at({ core: { peer_acked: true }, hiddenBanner: true }),
    'Waiting for video…\nGS mode keeps flying the link while this tab is hidden.');
  // Once a picture has been drawn, nothing (a freeze keeps the last frame, uncovered).
  assert.equal(at({ core: { peer_acked: true }, hasPicture: true }), '');
  assert.equal(at({ mode: 'spotter', core: {}, hasPicture: true }), '');
});

test('status text and link tag', () => {
  assert.equal(statusText({ state: 'live', mode: 'gs', ch: 136, w: 40, core: { peer_acked: false },
    sinceStartMs: 12000, hiddenBanner: false, hasPicture: false }),
    'No drone on ch 136 / 40 MHz (still trying)');
  // A frozen gate never paints over the video once a picture exists (spec 2026-09-28).
  assert.equal(statusText({ state: 'live', mode: 'spotter', ch: 136, w: 40, core: {},
    sinceStartMs: 3000, hiddenBanner: false, hasPicture: true }), '');
  assert.equal(statusText({ state: 'live', mode: 'gs', ch: 136, w: 40, core: { peer_acked: true },
    sinceStartMs: 3000, hiddenBanner: true, hasPicture: true }),
    'GS mode keeps flying the link while this tab is hidden.');
  assert.equal(statusText({ state: 'idle' }), '');
  assert.deepEqual(linkTag({ state: 'idle', mode: 'gs', core: null }), { label: 'Disconnected', on: false });
  assert.deepEqual(linkTag({ state: 'connecting', mode: 'gs', core: null }), { label: 'Connecting…', on: false });
  assert.deepEqual(linkTag({ state: 'live', mode: 'gs', core: { session: true, peer_acked: true } }), { label: 'Linked', on: true });
  assert.deepEqual(linkTag({ state: 'live', mode: 'gs', core: { session: false } }), { label: 'Searching', on: true });
  assert.deepEqual(linkTag({ state: 'live', mode: 'spotter', core: {} }), { label: 'Spotter', on: true });
});
