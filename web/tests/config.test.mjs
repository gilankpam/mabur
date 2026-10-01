import test from 'node:test';
import assert from 'node:assert/strict';
import {
  CHANNELS, defaultConfig, normalizeConfig, loadConfig, saveConfig, rungWarnings, channelWarning,
  connectBlocker, toOverlayToml, applyEdit, applyRungEdit, describeEdit,
  describeRungEdit, parseKeyText, loadKey, saveKey, KEY_STORE, DEFAULT_KEY_HEX,
} from '../ui/src/lib/config.js';

const mem = (init = {}) => {
  const m = new Map(Object.entries(init));
  return { getItem: (k) => (m.has(k) ? m.get(k) : null), setItem: (k, v) => m.set(k, v),
    removeItem: (k) => m.delete(k), m };
};

test('defaults match maburgs.default.toml', () => {
  const c = defaultConfig();
  assert.deepEqual(c, { channel: 136, width: 40, staticMcs: -1,
    ladder: [0, 1, 2, 3, 4].map((mcs) => ({ mcs, bw: 40, ob: 0.5, oe: 0.25 })), colortrans: true, dvr: 'web' });
  c.ladder[0].mcs = 7;
  assert.equal(defaultConfig().ladder[0].mcs, 0, 'fresh copy each call');
  assert.ok(CHANNELS.includes(136) && CHANNELS.includes(165) && CHANNELS.length === 25);
});

test('normalize_config_falls_back_per_field', () => {
  const d = defaultConfig();
  assert.deepEqual(normalizeConfig(null), d);
  assert.deepEqual(normalizeConfig('junk'), d);
  const c = normalizeConfig({ channel: '149', width: 33, staticMcs: 9, maxMcs: 3,   // stale maxMcs key: dropped
    ladder: [{ mcs: 1, bw: 20, ob: 0.5, oe: 0.25 }, { mcs: 'x' }] });
  assert.equal(c.channel, 149);          // numeric string accepted
  assert.equal(c.width, 40);             // bad -> default
  assert.equal(c.staticMcs, -1);         // out of range -> default
  assert.ok(!('maxMcs' in c));
  assert.deepEqual(c.ladder, d.ladder);  // any bad rung -> whole default ladder
  const nine = Array.from({ length: 9 }, () => ({ mcs: 0, bw: 20, ob: 0.5, oe: 0.25 }));
  assert.deepEqual(normalizeConfig({ ladder: nine }).ladder, d.ladder);
  assert.deepEqual(normalizeConfig({ ladder: [] }).ladder, d.ladder);
  assert.equal(normalizeConfig({ colortrans: false }).colortrans, false);
  assert.equal(normalizeConfig({ colortrans: 'no' }).colortrans, true);   // non-boolean -> default on
  assert.equal(normalizeConfig({}).colortrans, true);
});

test('load/save round trip, storage failures tolerated, URL ch/w override', () => {
  const s = mem();
  const c = applyEdit(defaultConfig(), 'staticMcs', 3);
  saveConfig(s, c);
  assert.deepEqual(loadConfig(s, new URLSearchParams()), c);
  assert.equal(loadConfig(s, new URLSearchParams('ch=149&w=20')).channel, 149);
  assert.equal(loadConfig(s, new URLSearchParams('ch=149&w=20')).width, 20);
  const bad = { getItem() { throw new Error('denied'); }, setItem() { throw new Error('denied'); } };
  assert.deepEqual(loadConfig(bad, new URLSearchParams()), defaultConfig());
  assert.doesNotThrow(() => saveConfig(bad, c));
  assert.deepEqual(loadConfig(mem({ 'webgs.cfg': '{not json' }), new URLSearchParams()), defaultConfig());
});

test('rung warnings use the handoff strings, in order', () => {
  let c = applyEdit(defaultConfig(), 'width', 20);
  c = applyRungEdit(c, 4, 'mcs', 7);
  c = applyRungEdit(c, 4, 'oe', '0.6');
  assert.deepEqual(rungWarnings(c, 4),
    ['40 MHz needs channel width 40', 'Enh overhead above base']);
  assert.deepEqual(rungWarnings(defaultConfig(), 0), []);
});

test('channel warning for an unpaired channel at 40', () => {
  assert.equal(channelWarning(defaultConfig()), null);
  assert.match(channelWarning(applyEdit(defaultConfig(), 'channel', 165)), /no 40 MHz pair/);
  assert.equal(channelWarning(applyEdit(applyEdit(defaultConfig(), 'channel', 165), 'width', 20)), null);
});

test('connect_blockers_gs_refuses_what_the_core_refuses', () => {
  const d = defaultConfig();
  assert.equal(connectBlocker(d, 'gs'), null);
  assert.match(connectBlocker(applyRungEdit(d, 1, 'oe', '0.9'), 'gs'), /Rung 1/);
  assert.match(connectBlocker(applyRungEdit(d, 2, 'ob', '5'), 'gs'), /Rung 2.*0\.1.*2/);
  assert.match(connectBlocker(applyRungEdit(d, 2, 'ob', ''), 'gs'), /Rung 2/);
  assert.match(connectBlocker(applyEdit(d, 'width', 20), 'gs'), /Rung 0.*40 MHz/);
  // Every rung flies (no max MCS filter): a 40 MHz MCS 7 top rung still blocks at width 20.
  const w = { ...applyEdit(d, 'width', 20),
    ladder: [{ mcs: 0, bw: 20, ob: 0.5, oe: 0.25 }, { mcs: 7, bw: 40, ob: 0.5, oe: 0.25 }] };
  assert.match(connectBlocker(w, 'gs'), /Rung 1/);
  assert.match(connectBlocker(applyEdit(d, 'channel', 165), 'gs'), /no 40 MHz pair/);
});

test('connect_blockers_spotter_only_checks_channel_width', () => {
  const d = defaultConfig();
  assert.equal(connectBlocker(applyRungEdit(d, 1, 'oe', '0.9'), 'spotter'), null);
  assert.equal(connectBlocker(applyEdit(d, 'width', 20), 'spotter'), null);
  assert.match(connectBlocker(applyEdit(d, 'channel', 165), 'spotter'), /no 40 MHz pair/);
});

test('overlay TOML, pinned: one always-loadable rung, max_mcs 7, saved ladder untouched', () => {
  const c = applyEdit(applyEdit(defaultConfig(), 'staticMcs', 3), 'width', 20);
  const t = toOverlayToml(c);
  assert.equal(t, '[link]\nstatic_mcs = 3\nstatic_bw = 20\nmax_mcs = 7\n'
    + '\n[[link.ladder]]\nmcs = 3\nbw = 20\noverhead_base = 0.5\noverhead_enh = 0.25\n');
  assert.equal(c.ladder.length, 5);
  // hidden fields never block a pinned connect
  assert.equal(connectBlocker(applyRungEdit(c, 1, 'oe', '0.9'), 'gs'), null);
  assert.match(connectBlocker(applyEdit(applyEdit(c, 'width', 40), 'channel', 165), 'gs'), /no 40 MHz pair/);
});

test('overlay TOML carries static_mcs, static_bw=width, max_mcs 7 and the ladder', () => {
  const t = toOverlayToml(defaultConfig());
  assert.match(t, /^\[link\]\nstatic_mcs = -1\nstatic_bw = 40\nmax_mcs = 7\n/);
  assert.equal((t.match(/\[\[link\.ladder\]\]/g) || []).length, 5);
  assert.match(t, /\[\[link\.ladder\]\]\nmcs = 0\nbw = 40\noverhead_base = 0\.5\noverhead_enh = 0\.25\n/);
  assert.ok(!/NaN|undefined/.test(t));
});

test('rung add/remove limits and labels', () => {
  let c = defaultConfig();
  c = applyRungEdit(c, -1, '__add');
  assert.equal(c.ladder.length, 6);
  assert.deepEqual(c.ladder[5], { mcs: 5, bw: 40, ob: 0.5, oe: 0.25 });
  for (let i = 0; i < 5; i++) c = applyRungEdit(c, -1, '__add');
  assert.equal(c.ladder.length, 8, 'capped at 8');
  let one = { ...defaultConfig(), ladder: [{ mcs: 0, bw: 40, ob: 0.5, oe: 0.25 }] };
  assert.equal(applyRungEdit(one, 0, '__remove').ladder.length, 1, 'never below 1');
  assert.equal(describeEdit('width', 40), 'Channel width set to 40 MHz');
  assert.equal(describeEdit('staticMcs', -1), 'Fixed MCS set to Adaptive');
  assert.equal(describeEdit('colortrans', false), 'Colour correction off');
  assert.equal(describeEdit('channel', 149), 'Channel set to 149');
  assert.equal(describeRungEdit(c, 7, '__add'), 'Rung 7 added');
  assert.equal(describeRungEdit(c, 2, '__remove'), 'Rung 2 removed');
  assert.equal(describeRungEdit(c, 1, 'ob', '0.6'), 'Rung 1 FEC base set to 0.6');
});

test('rung move: reorders, bounds-checked, described', () => {
  const d = defaultConfig();   // mcs 0..4
  const m = applyRungEdit(d, 4, '__move', 0);
  assert.deepEqual(m.ladder.map((r) => r.mcs), [4, 0, 1, 2, 3]);
  assert.deepEqual(d.ladder.map((r) => r.mcs), [0, 1, 2, 3, 4], 'input untouched');
  assert.deepEqual(applyRungEdit(d, 0, '__move', 2).ladder.map((r) => r.mcs), [1, 2, 0, 3, 4]);
  for (const to of [-1, 5, 1.5, 'x']) {
    assert.deepEqual(applyRungEdit(d, 1, '__move', to).ladder.map((r) => r.mcs), [0, 1, 2, 3, 4], String(to));
  }
  assert.equal(describeRungEdit(m, 4, '__move', 0), 'MCS 4 rung moved to position 1 of 5');
});

test('dvr target: default web, normalized, described', () => {
  assert.equal(defaultConfig().dvr, 'web');
  assert.equal(normalizeConfig({ dvr: 'both' }).dvr, 'both');
  assert.equal(normalizeConfig({ dvr: 'vtx' }).dvr, 'vtx');
  assert.equal(normalizeConfig({ dvr: 'sd' }).dvr, 'web');
  assert.equal(normalizeConfig({}).dvr, 'web');
  assert.equal(describeEdit('dvr', 'both'), 'Recording target set to Both');
});

const HEX = '3f9a1c77e04b5d2290ab6ef1c8d34e5a';
test('parseKeyText: comments, blanks, CRLF, case; rejects bad input', () => {
  assert.equal(parseKeyText(`# key\r\n\r\n  ${HEX.toUpperCase()}  \r\n`), HEX);
  assert.throws(() => parseKeyText(''), /no key/);
  assert.throws(() => parseKeyText(`${HEX}\n${HEX}\n`), /more than one key/);
  assert.throws(() => parseKeyText(HEX.slice(0, 31)), /32 hex/);
});
test('key store round trip; stale or broken storage falls back to null', () => {
  const s = mem();
  assert.equal(loadKey(s), null);
  saveKey(s, HEX);
  assert.equal(loadKey(s), HEX);
  saveKey(s, null);
  assert.equal(loadKey(s), null);
  assert.equal(loadKey(mem({ [KEY_STORE]: 'not-a-key' })), null);
  const bad = { getItem() { throw new Error('denied'); }, setItem() { throw new Error('denied'); }, removeItem() { throw new Error('denied'); } };
  assert.equal(loadKey(bad), null);
  assert.doesNotThrow(() => saveKey(bad, HEX));
});
test('overlay carries link.key only when a key is loaded', () => {
  assert.ok(!/key =/.test(toOverlayToml(defaultConfig())));
  assert.match(toOverlayToml(defaultConfig(), HEX), new RegExp(`^\\[link\\]\\nkey = "${HEX}"\\nstatic_mcs = -1\\n`));
});
