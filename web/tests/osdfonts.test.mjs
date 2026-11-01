import test from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULT_FONT, normalizeFontList, loadFontList, fontFile } from '../ui/src/lib/osdfonts.js';
import { defaultConfig, normalizeConfig, describeEdit } from '../ui/src/lib/config.js';

const BTFL = { id: 'btfl', name: 'Betaflight', file: 'fonts/btfl.png' };

test('normalizeFontList: bad entries dropped, ids unique, btfl is an ordinary entry', () => {
  const l = normalizeFontList([
    BTFL,
    { id: 'paddy', name: 'Paddy', file: 'fonts/paddy.png' },
    { id: 'paddy', name: 'dup', file: 'fonts/x.png' },
    { id: 'Bad Id', file: 'fonts/y.png' },
    { id: 'esc', file: 'fonts/../../etc.png' },
    { id: 'abs', file: 'https://evil.example/a.png' },
    { id: 'noname', file: 'fonts/n.png' },
    null,
  ]);
  assert.deepEqual(l.map((f) => f.id), ['btfl', 'paddy', 'noname']);
  assert.equal(l[2].name, 'noname');
  assert.deepEqual(normalizeFontList('nope'), []);   // no built-in list to fall back on
});

test('loadFontList: a missing, broken or btfl-less list rejects instead of faking a default', async () => {
  await assert.rejects(loadFontList(async () => ({ ok: false, status: 404 }), 'http://x/'), /HTTP 404/);
  await assert.rejects(loadFontList(async () => { throw new Error('net'); }, 'http://x/'), /net/);
  await assert.rejects(loadFontList(async () => ({ ok: true, json: async () => { throw new Error('bad json'); } }), 'http://x/'), /bad json/);
  await assert.rejects(loadFontList(async () => ({ ok: true, json: async () => [{ id: 'p', file: 'fonts/p.png' }] }), 'http://x/'), /no "btfl"/);
  let url = '';
  const l = await loadFontList(async (u) => { url = String(u); return { ok: true, json: async () => [BTFL, { id: 'p', file: 'fonts/p.png' }] }; }, 'http://x/m/');
  assert.equal(url, 'http://x/m/fonts/index.json');
  assert.deepEqual(l.map((f) => f.id), ['btfl', 'p']);
});

test('fontFile: unknown id uses the btfl entry; a list without one throws', () => {
  const l = normalizeFontList([BTFL, { id: 'p', file: 'fonts/p.png' }]);
  assert.equal(DEFAULT_FONT, 'btfl');
  assert.equal(fontFile(l, 'p'), 'fonts/p.png');
  assert.equal(fontFile(l, 'gone'), 'fonts/btfl.png');
  assert.throws(() => fontFile([], 'btfl'), /no "btfl"/);
});

test('config: osdFont defaults to btfl, survives normalize, rejects junk', () => {
  assert.equal(defaultConfig().osdFont, 'btfl');
  assert.equal(normalizeConfig({ osdFont: 'paddy_green' }).osdFont, 'paddy_green');
  assert.equal(normalizeConfig({ osdFont: '../x' }).osdFont, 'btfl');
  assert.equal(normalizeConfig({ osdFont: 7 }).osdFont, 'btfl');
});

test('describeEdit shows the font name when given the list, the id otherwise', () => {
  const fonts = [BTFL, { id: 'paddy_green', name: 'PADDY green', file: 'fonts/paddy_green.png' }];
  assert.equal(describeEdit('osdFont', 'paddy_green', fonts), 'OSD font PADDY green');
  assert.equal(describeEdit('osdFont', 'gone', fonts), 'OSD font gone');
  assert.equal(describeEdit('osdFont', 'paddy_green'), 'OSD font paddy_green');
});
