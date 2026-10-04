import test from 'node:test';
import assert from 'node:assert/strict';
import {
  CHANNELS, defaultConfig, normalizeConfig, loadConfig, saveConfig, rungWarnings, channelWarning,
  connectBlocker, toOverlayToml, chipChannels, applyEdit, applyRungEdit, describeEdit,
  describeRungEdit, parseKeyText, loadKey, saveKey, KEY_STORE, DEFAULT_KEY_HEX,
  toggleChannel, checkChannelSet, CHANNEL_STORE, loadRememberedChannel, saveRememberedChannel,
  startChannel,
} from '../ui/src/lib/config.js';

const mem = (init = {}) => {
  const m = new Map(Object.entries(init));
  return { getItem: (k) => (m.has(k) ? m.get(k) : null), setItem: (k, v) => m.set(k, v),
    removeItem: (k) => m.delete(k), m };
};

test('defaults match maburgs.default.toml', () => {
  const c = defaultConfig();
  assert.deepEqual(c, { channels: [40, 64, 112, 144], link: 'auto', width: 40, staticMcs: -1,
    ladder: [0, 1, 2, 3, 4].map((mcs) => ({ mcs, bw: 40, ob: 0.5, oe: 0.25 })), colortrans: true, dvr: 'web' });
  c.ladder[0].mcs = 7;
  assert.equal(defaultConfig().ladder[0].mcs, 0, 'fresh copy each call');
  assert.ok(CHANNELS.includes(136) && CHANNELS.includes(165) && CHANNELS.length === 25);
});

test('normalize_config_falls_back_per_field', () => {
  const d = defaultConfig();
  assert.deepEqual(normalizeConfig(null), d);
  assert.deepEqual(normalizeConfig('junk'), d);
  const c = normalizeConfig({ channels: ['40', 64, 64, 112], link: 64, width: 33, staticMcs: 9, maxMcs: 3,   // stale maxMcs key: dropped
    ladder: [{ mcs: 1, bw: 20, ob: 0.5, oe: 0.25 }, { mcs: 'x' }] });
  assert.deepEqual(c.channels, d.channels);   // a duplicate -> whole default set
  assert.equal(c.link, 64);                   // a member of the (default) set
  assert.deepEqual(normalizeConfig({ channels: ['40', 64] }).channels, [40, 64]);   // numeric strings accepted
  assert.equal(normalizeConfig({ channels: [40, 64], link: 112 }).link, 'auto');    // non-member pin -> auto
  assert.deepEqual(normalizeConfig({ channels: [40, 300] }).channels, d.channels);  // out of range
  assert.deepEqual(normalizeConfig({ channels: Array.from({ length: 9 }, (_, i) => 36 + 4 * i) }).channels, d.channels);
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
  const q = loadConfig(s, new URLSearchParams('chs=40,64&ch=64&w=20'));
  assert.deepEqual(q.channels, [40, 64]);
  assert.equal(q.link, 64);
  assert.equal(q.width, 20);
  assert.equal(loadConfig(mem({ 'webgs.cfg': JSON.stringify({ ...c, link: 64 }) }), new URLSearchParams('ch=auto')).link, 'auto');
  const s2 = mem({ 'webgs.cfg': JSON.stringify({ ...c, channels: [40, 64] }) });
  const q2 = loadConfig(s2, new URLSearchParams('ch=112'));
  assert.deepEqual(q2.channels, [40, 64, 112]);
  assert.equal(q2.link, 112);
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
  assert.match(channelWarning(applyEdit(defaultConfig(), 'channels', [40, 165])), /165/);
  assert.match(channelWarning(applyEdit(defaultConfig(), 'channels', [40, 165])), /no 40 MHz pair/);
  assert.equal(channelWarning(applyEdit(applyEdit(defaultConfig(), 'channels', [40, 165]), 'width', 20)), null);
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
  assert.match(connectBlocker(applyEdit(d, 'channels', [165]), 'gs'), /no 40 MHz pair/);
});

test('connect_blockers_spotter_only_checks_channel_width', () => {
  const d = defaultConfig();
  assert.equal(connectBlocker(applyRungEdit(d, 1, 'oe', '0.9'), 'spotter'), null);
  assert.equal(connectBlocker(applyEdit(d, 'width', 20), 'spotter'), null);
  assert.match(connectBlocker(applyEdit(d, 'channels', [165]), 'spotter'), /no 40 MHz pair/);
});

test('overlay TOML, pinned: one always-loadable rung, max_mcs 7, saved ladder untouched', () => {
  const c = applyEdit(applyEdit(defaultConfig(), 'staticMcs', 3), 'width', 20);
  const t = toOverlayToml(c);
  assert.equal(t, '[radio]\nchannels = [40, 64, 112, 144]\nchannel = "auto"\nwidth = 20\n'
    + '\n[link]\nstatic_mcs = 3\nstatic_bw = 20\nmax_mcs = 7\n'
    + '\n[[link.ladder]]\nmcs = 3\nbw = 20\noverhead_base = 0.5\noverhead_enh = 0.25\n');
  assert.equal(c.ladder.length, 5);
  // hidden fields never block a pinned connect
  assert.equal(connectBlocker(applyRungEdit(c, 1, 'oe', '0.9'), 'gs'), null);
  assert.match(connectBlocker(applyEdit(applyEdit(c, 'width', 40), 'channels', [165]), 'gs'), /no 40 MHz pair/);
});

test('overlay TOML carries static_mcs, static_bw=width, max_mcs 7 and the ladder', () => {
  const t = toOverlayToml(defaultConfig());
  assert.match(t, /\n\[link\]\nstatic_mcs = -1\nstatic_bw = 40\nmax_mcs = 7\n/);
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
  assert.match(toOverlayToml(defaultConfig(), HEX), new RegExp(`\\n\\[link\\]\\nkey = "${HEX}"\\nstatic_mcs = -1\\n`));
});

test('toggleChannel adds/removes, keeps 1..8, never drops the last, clears a removed pin', () => {
  let c = { ...defaultConfig(), channels: [40], link: 40 };
  c = toggleChannel(c, 64); assert.deepEqual(c.channels, [40, 64]);
  c = toggleChannel(c, 40); assert.deepEqual(c.channels, [64]); assert.equal(c.link, 'auto');   // the pin went
  c = toggleChannel(c, 64); assert.deepEqual(c.channels, [64]);                                 // last member stays
  for (const ch of [36, 40, 44, 48, 52, 56, 60]) c = toggleChannel(c, ch);
  assert.equal(c.channels.length, 8);
  assert.equal(toggleChannel(c, 100).channels.length, 8);                                        // 9th refused
});

test('checkChannelSet: count, range, duplicates, 40 MHz pairs on one offset, link a member', () => {
  assert.equal(checkChannelSet([40, 64, 112, 144], 'auto', 40), null);
  assert.match(checkChannelSet([], 'auto', 40), /1 to 8/);
  assert.match(checkChannelSet([40, 44], 'auto', 40), /other side|offset/);     // 40 HT40-, 44 HT40+
  assert.equal(checkChannelSet([40, 44], 'auto', 20), null);
  assert.match(checkChannelSet([40, 165], 'auto', 40), /no 40 MHz pair/);
  assert.match(checkChannelSet([40, 64], 112, 40), /not in the set/);
});

test('connectBlocker refuses 144 on the relay radio only (Review Focus 5)', () => {
  const c = defaultConfig();                       // contains 144
  assert.equal(connectBlocker(c, 'gs', 'usb'), null);
  assert.match(connectBlocker(c, 'spotter', 'relay'), /144/);
  assert.equal(connectBlocker(toggleChannel(c, 144), 'spotter', 'relay'), null);
});

test('toOverlayToml carries the set + width in both modes; the spotter gets a minimal [link]', () => {
  const c = defaultConfig();
  const gs = toOverlayToml(c, null, 'gs');
  assert.match(gs, /^\[radio\]\nchannels = \[40, 64, 112, 144\]\nchannel = "auto"\nwidth = 40\n/);
  assert.match(gs, /\[link\]/);
  const sp = toOverlayToml({ ...c, link: 64 }, 'ab'.repeat(16), 'spotter');
  assert.equal(sp, '[radio]\nchannels = [40, 64, 112, 144]\nchannel = 64\nwidth = 40\n'
    + '\n[link]\nstatic_mcs = -1\nstatic_bw = 40\nmax_mcs = 7\n'
    + '\n[[link.ladder]]\nmcs = 0\nbw = 40\noverhead_base = 0.5\noverhead_enh = 0.25\n');
  // A 20 MHz-only set (no 40 MHz pair) carries width 20, so the loader checks it at 20.
  const w20 = { ...c, channels: [165], link: 'auto', width: 20 };
  assert.match(toOverlayToml(w20, null, 'gs'), /^\[radio\]\nchannels = \[165\]\nchannel = "auto"\nwidth = 20\n/);
  assert.match(toOverlayToml(w20, null, 'spotter'), /\nwidth = 20\n[\s\S]*static_bw = 20\n[\s\S]*bw = 20\n/);
});

test('chipChannels shows an off-list member; ?ch= keeps the set sorted', () => {
  assert.deepEqual(chipChannels(defaultConfig()), CHANNELS);
  const chips = chipChannels({ ...defaultConfig(), channels: [40, 169] });
  assert.ok(chips.includes(169));
  assert.deepEqual(chips, [...chips].sort((a, b) => a - b));
  const q = loadConfig(mem(), new URLSearchParams('ch=169'));
  assert.deepEqual(q.channels, [40, 64, 112, 144, 169]);
  assert.equal(q.link, 169);
  const s = mem({ 'webgs.cfg': JSON.stringify({ ...defaultConfig(), channels: [64, 112] }) });
  assert.deepEqual(loadConfig(s, new URLSearchParams('ch=40')).channels, [40, 64, 112]);
});

test('startChannel: the pin wins, a non-member remembered is ignored (Review Focus 1, 2)', () => {
  const c = defaultConfig();
  assert.equal(startChannel({ ...c, link: 40 }, 64), 40);     // pinned: never the remembered one
  assert.equal(startChannel(c, 64), 64);                      // auto: remembered member
  assert.equal(startChannel(c, 136), 40);                     // auto: stale non-member -> first member
  assert.equal(startChannel(c, null), 40);
});

test('remembered channel round trip', () => {
  const s = mem();
  saveRememberedChannel(s, 112);
  assert.equal(loadRememberedChannel(s), 112);
  assert.equal(loadRememberedChannel(mem({ [CHANNEL_STORE]: 'junk' })), null);
  assert.equal(loadRememberedChannel(mem({ [CHANNEL_STORE]: '300' })), null);
});

test('describeEdit names the set and the link channel', () => {
  assert.equal(describeEdit('channels', [40, 64]), 'Channels set to 40, 64');
  assert.equal(describeEdit('link', 'auto'), 'Link channel set to Auto');
  assert.equal(describeEdit('link', 64), 'Link channel set to 64');
});
