import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {
  MODE_SIZES, modeSizes, modeText, parseMode, commitText, SECTION_KICKER, groupRows,
  HINTS, LABELS, HIDDEN, rowLabel, rowHint, headerText, rowView, invalidText, showNoHeartbeat, commitAnswered,
  MODE_FPS, modeChoices, recSizeChoices, rowWidget, toggleValues, toggleState, SLICES_ON, SLICES_OFF, REC_SETTINGS, rowShown, DISPLAY_MATCH_HZ, modeDisplayNote, rowStatus,
} from '../ui/src/lib/vtxparams.js';

test('modeSizes per sensor mode and at the boundaries', () => {
  assert.deepEqual(modeSizes(90), ['1920x1080', '1280x720']);
  assert.deepEqual(modeSizes(91), ['1728x972', '1280x720']);
  assert.deepEqual(modeSizes(100), ['1728x972', '1280x720']);
  assert.deepEqual(modeSizes(101), ['1728x816', '1152x544', '864x408']);
  assert.deepEqual(modeSizes(120), ['1728x816', '1152x544', '864x408']);
  assert.deepEqual(modeSizes(0), []);
  assert.deepEqual(modeSizes(121), []);
  assert.deepEqual(modeSizes(60.5), []);
  assert.deepEqual(modeSizes('90'), []);
  assert.equal(MODE_SIZES.length, 3);
});

test('modeText and parseMode round trip', () => {
  assert.equal(modeText(120, '1728x816'), '120@1728x816');
  assert.deepEqual(parseMode('60@1920x1080'), { fps: 60, size: '1920x1080' });
  assert.equal(parseMode('60@'), null);
  assert.equal(parseMode('x'), null);
  assert.equal(parseMode(''), null);
  assert.equal(parseMode('60@1920x1080extra'), null);
  // config_params.h's six-digit cap per number, mirrored.
  assert.deepEqual(parseMode('999999@999999x999999'), { fps: 999999, size: '999999x999999' });
  assert.equal(parseMode('1000000@1x1'), null);
  assert.equal(parseMode('4294967356@1x1'), null);
});

test('modeChoices: every size of each MODE_FPS rate, crop modes labelled', () => {
  const c = modeChoices('90@1920x1080');
  assert.deepEqual(MODE_FPS, [60, 90, 100, 120]);
  assert.deepEqual(c.map((x) => x.value), [
    '60@1920x1080', '60@1280x720', '90@1920x1080', '90@1280x720',
    '100@1728x972', '100@1280x720', '120@1728x816', '120@1152x544', '120@864x408',
  ]);
  assert.equal(c[0].label, '60 fps · 1920×1080');
  assert.equal(c[4].label, '100 fps · 1728×972 · cropped view');
});

test('modeChoices puts an off-list live value first, and only then', () => {
  assert.deepEqual(modeChoices('30@1280x720')[0], { value: '30@1280x720', label: '30 fps · 1280×720' });
  assert.equal(modeChoices('30@1280x720').length, 10);
  assert.equal(modeChoices('120@1152x544').length, 9);
  assert.equal(modeChoices(null).length, 9);
  assert.equal(modeChoices('').length, 9);
});

const values = (c) => c.map((x) => x.value);

test('modeChoices keeps only the rates within DISPLAY_MATCH_HZ of the display', () => {
  assert.equal(DISPLAY_MATCH_HZ, 0.55);
  assert.deepEqual(values(modeChoices('60@1920x1080', 60.0)), ['60@1920x1080', '60@1280x720']);
  assert.deepEqual(values(modeChoices('60@1920x1080', 59.94)), ['60@1920x1080', '60@1280x720']);
  assert.deepEqual(values(modeChoices('100@1728x972', 100.4)), ['100@1728x972', '100@1280x720']);
  assert.deepEqual(values(modeChoices('120@1728x816', 119.88)), ['120@1728x816', '120@1152x544', '120@864x408']);
  // a multiple is not a match: 60 fps on a 120 Hz panel trips the rate check
  assert.ok(!values(modeChoices('120@1728x816', 120)).includes('60@1920x1080'));
  assert.deepEqual(values(modeChoices('60@1920x1080', 60.6)), values(modeChoices('60@1920x1080')));
});

test('modeChoices keeps the drone value when it does not match the display', () => {
  assert.deepEqual(values(modeChoices('90@1280x720', 60)), ['60@1920x1080', '60@1280x720', '90@1280x720']);
  assert.deepEqual(values(modeChoices('30@1280x720', 60)), ['30@1280x720', '60@1920x1080', '60@1280x720']);
});

test('modeChoices lists everything with no display rate or no matching rate', () => {
  const all = values(modeChoices('90@1920x1080'));
  assert.equal(all.length, 9);
  assert.deepEqual(values(modeChoices('90@1920x1080', null)), all);
  assert.deepEqual(values(modeChoices('90@1920x1080', 0)), all);
  assert.deepEqual(values(modeChoices('90@1920x1080', NaN)), all);
  assert.deepEqual(values(modeChoices('90@1920x1080', 144)), all);
});

test('modeDisplayNote names the display rate and whether the list follows it', () => {
  assert.equal(modeDisplayNote(59.94), 'Showing the rates that match this display (60 Hz).');
  assert.equal(modeDisplayNote(144.0), 'No camera rate matches this display (144 Hz).');
  assert.equal(modeDisplayNote(null), '');
  assert.equal(modeDisplayNote(0), '');
});

test('recSizeChoices follow the camera mode the drone runs', () => {
  const v = (m, r) => recSizeChoices(m, r).map((x) => x.value);
  assert.deepEqual(v('90@1280x720', '1920x1080'), ['1920x1080', '1280x720']);
  assert.deepEqual(v('100@1728x972', '1728x972'), ['1728x972', '1280x720']);
  assert.deepEqual(v('120@1728x816', '864x408'), ['1728x816', '1152x544', '864x408']);
  assert.equal(recSizeChoices('60@1920x1080', '1280x720')[1].label, '1280×720');
});

test('recSizeChoices keep the camera size and the drone value when off the list', () => {
  const v = (m, r) => recSizeChoices(m, r).map((x) => x.value);
  assert.deepEqual(v('90@1600x900', '1600x900'), ['1920x1080', '1280x720', '1600x900']);
  // an upscale left on the drone stays selectable as what it runs
  assert.deepEqual(v('100@1728x972', '1920x1080'), ['1728x972', '1280x720', '1920x1080']);
  assert.deepEqual(v(null, '1280x720'), ['1280x720']);
  assert.deepEqual(v(null, ''), []);
});

test('rowShown hides the recorder settings only while REC_ENABLE reads off', () => {
  for (const id of REC_SETTINGS) {
    assert.equal(rowShown({ id }, '0'), false);
    assert.equal(rowShown({ id }, '1'), true);
    assert.equal(rowShown({ id }, ''), true);
    assert.equal(rowShown({ id }, null), true);
  }
  assert.equal(rowShown({ id: 'REC_ENABLE' }, '0'), true);
  assert.equal(rowShown({ id: 'ENC_BITRATE_MAX' }, '0'), true);
});

// rowStatus reads the row's state through rowView, as the panel does.
const statusOf = (row, localErr = null) => rowStatus(row, rowView(row, 0, -1, null), localErr);

test('rowStatus: label-line text while a set is in flight', () => {
  assert.deepEqual(statusOf({ state: 'sending', value: '1' }), { kind: 'busy', text: 'Sending…' });
  assert.deepEqual(statusOf({ state: 'in_progress', value: '1' }), { kind: 'busy', text: 'Applying…' });
});

test('rowStatus: saved while the accepted result is fresh, nothing once idle', () => {
  assert.deepEqual(statusOf({ state: 'accepted', value: '1', age_ms: 0 }), { kind: 'ok', text: '✓ Saved' });
  assert.equal(statusOf({ state: 'accepted', value: '1', age_ms: 5000 }), null);
  assert.equal(statusOf({ state: 'idle', value: '1' }), null);
});

test('rowStatus: an accepted set with a drone warning says Saved and keeps the warning', () => {
  // The drone accepts RADIO_LDPC against walls measured under the other
  // setting and follows the ack with a WARNING; it stays on the row (no
  // fade) until the next set clears it.
  const warn = 'radio: 20 MHz walls were measured with ldpc on; radio.ldpc is off — re-run maburcal';
  for (const age_ms of [0, 60000]) {
    assert.deepEqual(statusOf({ state: 'accepted', value: 'false', age_ms, status: warn }),
      { kind: 'warn', text: '✓ Saved', reason: warn });
  }
  assert.equal(rowView({ state: 'accepted', value: 'false', age_ms: 0, status: warn }, 0, -1, null).resultKind, 'warn');
});

test('rowStatus: a refusal says Failed and carries the reason without its prefix', () => {
  assert.deepEqual(statusOf({ state: 'failed', value: '1', status: 'rec fps above camera fps' }),
    { kind: 'err', text: 'Failed', reason: 'rec fps above camera fps' });
  assert.deepEqual(statusOf({ state: 'failed', value: '1', status: '' }), { kind: 'err', text: 'Failed', reason: 'refused' });
  assert.deepEqual(statusOf({ state: 'unsupported', value: '1', status: '' }),
    { kind: 'err', text: 'Failed', reason: "the drone refused this value's shape" });
});

test('rowStatus: a timeout says No answer and that the change may not have applied', () => {
  assert.deepEqual(statusOf({ state: 'timeout', value: '1' }),
    { kind: 'err', text: 'No answer', reason: 'The drone did not answer; it may not have applied the change.' });
});

test('rowStatus: a local invalid edit wins over the row state', () => {
  assert.deepEqual(statusOf({ state: 'accepted', value: '1', age_ms: 0 }, 'not a valid integer'),
    { kind: 'err', text: 'Invalid', reason: 'not a valid integer' });
});

test('rowWidget per row', () => {
  assert.equal(rowWidget({ id: 'VENC_MODE', type: 'str' }), 'mode');
  assert.equal(rowWidget({ id: 'REC_SIZE', type: 'str' }), 'recsize');
  assert.equal(rowWidget({ id: 'VENC_SLICES', type: 'int' }), 'toggle');
  assert.equal(rowWidget({ id: 'RADIO_LDPC', type: 'bool' }), 'toggle');
  assert.equal(rowWidget({ id: 'REC_FPS', type: 'int' }), 'input');
});

test('slices toggle: on sends 4, off sends 1, any count above 1 reads on', () => {
  const row = { id: 'VENC_SLICES', type: 'int' };
  assert.deepEqual(toggleValues(row), { on: SLICES_ON, off: SLICES_OFF });
  assert.equal(SLICES_ON, '4');
  assert.equal(SLICES_OFF, '1');
  assert.equal(commitText('int', SLICES_ON), '4');
  assert.equal(toggleState(row, '1'), 'off');
  assert.equal(toggleState(row, '4'), 'on');
  assert.equal(toggleState(row, '6'), 'on');
  assert.equal(toggleState(row, ''), null);
  assert.equal(toggleState(row, '0'), null);
});

test('bool toggle: 1 on, 0 off, unknown neither', () => {
  const row = { id: 'RADIO_LDPC', type: 'bool' };
  assert.deepEqual(toggleValues(row), { on: '1', off: '0' });
  assert.equal(commitText('bool', '1'), '1');
  assert.equal(toggleState(row, '1'), 'on');
  assert.equal(toggleState(row, '0'), 'off');
  assert.equal(toggleState(row, ''), null);
});

test('commitText per type', () => {
  assert.equal(commitText('bool', true), '1');
  assert.equal(commitText('bool', false), '0');
  assert.equal(commitText('bool', 'on'), '1');
  assert.equal(commitText('bool', 'off'), '0');
  assert.equal(commitText('bool', 'maybe'), null);
  assert.equal(commitText('int', ' 30000 '), '30000');
  assert.equal(commitText('int', '3e4'), null);
  assert.equal(commitText('int', '1.5'), null);
  assert.equal(commitText('real', '0.65'), '0.65');
  assert.equal(commitText('real', 'abc'), null);
  assert.equal(commitText('real', 'Infinity'), null);
  assert.equal(commitText('str', '1920x1080'), '1920x1080');
  assert.equal(commitText('str', ''), null);
});

test('commitText rejects strings the drone cannot hold', () => {
  assert.equal(commitText('str', 'a"b'), null);
  assert.equal(commitText('str', 'a\\b'), null);
  assert.equal(commitText('str', 'ñ'), null);
  assert.equal(commitText('str', 'a'.repeat(128)), null);
  assert.equal(commitText('str', 'a'.repeat(127)), 'a'.repeat(127));
});

const ROWS = [
  { id: 'RADIO_WALL_MARGN', section: 'radio' },
  { id: 'RADIO_LDPC', section: 'radio' },
  { id: 'VENC_MODE', section: 'venc' },
  { id: 'VENC_SLICES', section: 'venc' },
  { id: 'ENC_BITRATE_MAX', section: 'encoder' },
  { id: 'ENC_AIRTIME', section: 'encoder' },
  { id: 'REC_ENABLE', section: 'record' },
  { id: 'REC_BITRATE', section: 'record' },
  { id: 'REC_FPS', section: 'record' },
  { id: 'REC_SIZE', section: 'record' },
  { id: 'LOWPWR_ENABLE', section: 'low_power' },
  { id: 'MSP_ENABLE', section: 'msp' },
];

test('groupRows: SECTION_KICKER order with Radio last, rows in table order', () => {
  const groups = groupRows(ROWS);
  assert.deepEqual(groups.map((g) => g.kicker),
    ['Camera', 'Encoder', 'Recorder', 'Power', 'Flight controller', 'Radio']);
  assert.equal(groupRows([{ id: 'X', section: 'other' }, ...ROWS]).at(-1).key, 'other');
  const camera = groups.find((g) => g.key === 'venc');
  assert.deepEqual(camera.rows.map((r) => r.id), ['VENC_MODE', 'VENC_SLICES']);
});

test('groupRows leaves out HIDDEN rows', () => {
  const radio = groupRows(ROWS).find((g) => g.key === 'radio');
  assert.deepEqual(radio.rows.map((r) => r.id), ['RADIO_LDPC']);
  assert.ok(groupRows(ROWS).every((g) => g.rows.every((r) => !HIDDEN.has(r.id))));
});

test('invalidText names the row type', () => {
  assert.equal(invalidText('int'), 'not a valid integer');
  assert.equal(invalidText('real'), 'not a valid number');
  assert.match(invalidText('str'), /^not valid text/);
  assert.equal(commitText('int', '1.5'), null);   // the case it is shown for
});

test('rowLabel names the setting, falling back to the id', () => {
  assert.equal(rowLabel({ id: 'RADIO_LDPC' }), 'LDPC coding');
  assert.equal(rowLabel({ id: 'VENC_MODE' }), 'Camera mode');
  assert.equal(rowLabel({ id: 'NEW_ROW' }), 'NEW_ROW');
});

test('rowHint: help then the interaction hint, each a full sentence', () => {
  assert.equal(rowHint({ id: 'ENC_BITRATE_MAX', help: 'Bitrate ceiling, kbps' }), 'Bitrate ceiling, kbps.');
  assert.equal(rowHint({ id: 'MSP_ENABLE', help: 'Flight-controller link for the OSD and arm state' }),
    'Flight-controller link for the OSD and arm state. Turn low power off first.');
  assert.equal(rowHint({ id: 'X', help: 'Already ends.' }), 'Already ends.');
  assert.equal(rowHint({ id: 'X', help: '' }), '');
  assert.doesNotMatch(rowHint({ id: 'VENC_MODE', help: 'Camera frame rate and encoder size, <fps>@<WxH>' }), /<fps>/);
});

test('showNoHeartbeat only once a heartbeat was seen and then lost', () => {
  assert.equal(showNoHeartbeat({ linked: true, alive: null }, 'gs'), false);   // none yet on this link
  assert.equal(showNoHeartbeat({ linked: true, alive: true }, 'gs'), false);
  assert.equal(showNoHeartbeat({ linked: true, alive: false }, 'gs'), true);
  assert.equal(showNoHeartbeat({ linked: false, alive: false }, 'gs'), false);
  assert.equal(showNoHeartbeat({ linked: true, alive: false }, 'spotter'), false);
});

test('commitAnswered: through pending, or an immediate refusal newer than the snapshot', () => {
  const idle = { state: 'idle', age_ms: null, status: '' };
  // Went through sending and left it.
  assert.equal(commitAnswered({ ...idle, sawPending: false }, { state: 'sending', age_ms: null, status: '' }), false);
  assert.equal(commitAnswered({ ...idle, sawPending: true }, { state: 'accepted', age_ms: 5, status: '' }), true);
  // Refused at once by the client: idle -> failed.
  assert.equal(commitAnswered({ ...idle, sawPending: false }, { state: 'failed', age_ms: 0, status: 'apply in progress' }), true);
  // Refused again on a row already failed for the same reason: the age restarts.
  const failed = { state: 'failed', age_ms: 4000, status: 'apply in progress', sawPending: false };
  assert.equal(commitAnswered(failed, { state: 'failed', age_ms: 3, status: 'apply in progress' }), true);
  // An unrelated push before the core took the commit: same result, older.
  assert.equal(commitAnswered(failed, { state: 'failed', age_ms: 4200, status: 'apply in progress' }), false);
  assert.equal(commitAnswered({ ...idle, sawPending: false }, { state: 'idle', age_ms: null, status: '' }), false);
});

test('headerText by state', () => {
  assert.equal(headerText({ linked: false, mode: 'spotter', rows: [] }, 'spotter'),
    'VTX settings need GS mode.');
  assert.equal(headerText({ linked: false }, 'gs'), 'Waiting for the drone link…');
  assert.equal(headerText({ linked: true, mismatch: true }, 'gs'),
    "The drone's parameter table differs from this page — redeploy.");
  assert.equal(headerText({ linked: true, mismatch: false, list: 'pending' }, 'gs'),
    "Reading the drone's settings…");
  assert.equal(headerText({ linked: true, mismatch: false, list: 'pending', rows: [{ value: '1' }, { value: null }] }, 'gs'),
    "Reading the drone's settings…");
  assert.equal(headerText({ linked: true, mismatch: false, list: 'partial' }, 'gs'),
    'Some settings did not answer.');
  assert.equal(headerText({ linked: true, mismatch: false, list: 'complete' }, 'gs'), '');
});

// An accepted rebuild set makes the core re-list with every value still
// held; that re-read is silent, unlike the first read after linking.
test('headerText: a re-list over values already held says nothing', () => {
  assert.equal(headerText({ linked: true, mismatch: false, list: 'pending', rows: [{ value: '1' }, { value: '60@1920x1080' }] }, 'gs'), '');
});

test('rowView shows typed text only while pending', () => {
  const sending = { state: 'sending', value: '60', age_ms: null, status: '' };
  assert.equal(rowView(sending, 2, 2, '90').shown, '90');
  assert.equal(rowView(sending, 2, 2, '90').busy, true);
  const accepted = { state: 'accepted', value: '90', age_ms: 1000, status: '' };
  // A terminal state ignores stale typed text, even when it differs from
  // the drone's own value -- '91' here must not leak into `shown`.
  assert.equal(rowView(accepted, 2, -1, '91').shown, '90');
  const old = { state: 'accepted', value: '90', age_ms: 3500, status: '' };
  assert.equal(rowView(old, 2, -1, null, 0).result, '');
  assert.equal(rowView(accepted, 2, -1, null, 0).result, 'accepted');
  const failed = { state: 'failed', value: '90', age_ms: null, status: '' };
  assert.equal(rowView(failed, 2, -1, '91').shown, '90');
  assert.equal(rowView(failed, 2, -1, null).result, 'failed: refused');
});

test('rowView accepted fades by age_ms plus elapsed time since the push', () => {
  // age_ms is frozen at push time; sincePushMs is how long ago that push
  // landed, on the page's ~1 s re-render clock -- the sum is what ages out.
  const row = { state: 'accepted', value: '90', age_ms: 1000, status: '' };
  assert.equal(rowView(row, 0, -1, null, 2500).result, '');
  assert.equal(rowView(row, 0, -1, null, 1500).result, 'accepted');
});

test('rowView disables other rows while one is pending', () => {
  const row = { state: 'idle', value: '1', age_ms: null, status: '' };
  const v = rowView(row, 5, 4, null);
  assert.equal(v.disabled, true);
  assert.equal(v.busy, false);
  const v2 = rowView(row, 4, 4, null);
  assert.equal(v2.disabled, false);
  assert.equal(v2.busy, true);
  const v3 = rowView(row, 5, -1, null);
  assert.equal(v3.disabled, false);
  assert.equal(v3.busy, false);
});

test('rowView result/resultKind for the remaining states', () => {
  assert.deepEqual(
    (({ result, resultKind }) => ({ result, resultKind }))(
      rowView({ state: 'unsupported', value: '1', age_ms: null, status: 'out of range' }, 0, -1, null)),
    { result: 'unsupported: out of range', resultKind: 'err' });
  // VALUE_UNSUPPORTED never brings a STATUSTEXT: a fallback, not a bare colon.
  assert.deepEqual(
    (({ result, resultKind }) => ({ result, resultKind }))(
      rowView({ state: 'unsupported', value: '1', age_ms: 10, status: '' }, 0, -1, null)),
    { result: "unsupported: the drone refused this value's shape", resultKind: 'err' });
  assert.deepEqual(
    (({ result, resultKind }) => ({ result, resultKind }))(
      rowView({ state: 'timeout', value: '1', age_ms: null, status: '' }, 0, -1, null)),
    { result: 'no answer', resultKind: 'err' });
  assert.deepEqual(
    (({ result, resultKind }) => ({ result, resultKind }))(
      rowView({ state: 'in_progress', value: '1', age_ms: null, status: '' }, 0, -1, null)),
    { result: 'applying…', resultKind: 'busy' });
  assert.deepEqual(
    (({ result, resultKind }) => ({ result, resultKind }))(
      rowView({ state: 'sending', value: '1', age_ms: null, status: '' }, 0, -1, null)),
    { result: 'sending…', resultKind: 'busy' });
  assert.deepEqual(
    (({ result, resultKind }) => ({ result, resultKind }))(
      rowView({ state: 'idle', value: '1', age_ms: null, status: '' }, 0, -1, null)),
    { result: '', resultKind: '' });
});

// Drift guard: a renamed or re-sectioned row in the parameter table must
// reach this page's labels, hints and section kickers, and a new row needs
// a label (or a place in HIDDEN), or this test fails.
test('HINTS, LABELS, HIDDEN and SECTION_KICKER name real rows in config_params.h', () => {
  const h = fs.readFileSync(new URL('../../common/include/mabur/config_params.h', import.meta.url), 'utf8');
  const ids = new Set();
  const sections = new Set();
  const re = /\{"([A-Z0-9_]+)",\s*"([a-z_]+)",/g;
  let m;
  while ((m = re.exec(h))) {
    ids.add(m[1]);
    sections.add(m[2]);
  }
  assert.ok(ids.size >= 12, 'parsed the row table');
  for (const key of Object.keys(HINTS)) assert.ok(ids.has(key), `HINTS.${key} is not a row id`);
  for (const key of Object.keys(LABELS)) assert.ok(ids.has(key), `LABELS.${key} is not a row id`);
  for (const key of HIDDEN) assert.ok(ids.has(key), `HIDDEN ${key} is not a row id`);
  for (const id of ids) assert.ok(HIDDEN.has(id) || LABELS[id], `row ${id} has no label`);
  for (const section of Object.keys(SECTION_KICKER)) {
    assert.ok(sections.has(section), `SECTION_KICKER.${section} is not a table section`);
  }
});
