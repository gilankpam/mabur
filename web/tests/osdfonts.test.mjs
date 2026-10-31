import test from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULT_FONT, normalizeFontList, loadFontList, fontFile } from '../ui/src/lib/osdfonts.js';
import { defaultConfig, normalizeConfig, describeEdit } from '../ui/src/lib/config.js';

test('normalizeFontList: default first, bad entries dropped, ids unique', () => {
  const l = normalizeFontList([
    { id: 'paddy', name: 'Paddy', file: 'fonts/paddy.png' },
    { id: 'btfl', name: 'dup of default', file: 'fonts/x.png' },
    { id: 'Bad Id', file: 'fonts/y.png' },
    { id: 'esc', file: 'fonts/../../etc.png' },
    { id: 'abs', file: 'https://evil.example/a.png' },
    { id: 'noname', file: 'fonts/n.png' },
    null,
  ]);
  assert.deepEqual(l.map((f) => f.id), ['btfl', 'paddy', 'noname']);
  assert.equal(l[2].name, 'noname');
  assert.deepEqual(normalizeFontList('nope'), [DEFAULT_FONT]);
});

test('loadFontList: missing or broken manifest leaves the shipped font', async () => {
  assert.deepEqual(await loadFontList(async () => ({ ok: false }), 'http://x/'), [DEFAULT_FONT]);
  assert.deepEqual(await loadFontList(async () => { throw new Error('net'); }, 'http://x/'), [DEFAULT_FONT]);
  const l = await loadFontList(async (u) => ({ ok: true, json: async () => [{ id: 'p', file: 'fonts/p.png' }], u }), 'http://x/m/');
  assert.deepEqual(l.map((f) => f.id), ['btfl', 'p']);
});

test('fontFile: unknown id falls back to the shipped atlas', () => {
  const l = normalizeFontList([{ id: 'p', file: 'fonts/p.png' }]);
  assert.equal(fontFile(l, 'p'), 'fonts/p.png');
  assert.equal(fontFile(l, 'gone'), 'font_btfl.png');
});

test('config: osdFont defaults to btfl, survives normalize, rejects junk', () => {
  assert.equal(defaultConfig().osdFont, 'btfl');
  assert.equal(normalizeConfig({ osdFont: 'paddy_green' }).osdFont, 'paddy_green');
  assert.equal(normalizeConfig({ osdFont: '../x' }).osdFont, 'btfl');
  assert.equal(normalizeConfig({ osdFont: 7 }).osdFont, 'btfl');
  assert.match(describeEdit('osdFont', 'paddy_green'), /paddy_green/);
});
