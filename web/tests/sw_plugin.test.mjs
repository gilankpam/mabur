import { test } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { precacheList, buildId, injectManifest, shouldRegister } from '../ui/sw/plugin.mjs';

test('precacheList: bundle + public + emscripten outputs, never stale dist files or the sw itself', () => {
  // Review Focus 5: only what THIS build emitted; web/dist also holds stale
  // hashed bundles (emptyOutDir false), which are not passed in at all.
  const files = precacheList(
    ['index.html', 'assets/index-AAA.js', 'assets/index-BBB.css', 'assets/relay_worker-CCC.js', 'assets/index-AAA.js.map'],
    ['font_btfl.png', 'sw.js', '.gitkeep'],
    ['webgs.js', 'webgs.wasm']);
  assert.deepEqual(files, ['assets/index-AAA.js', 'assets/index-BBB.css', 'assets/relay_worker-CCC.js',
    'font_btfl.png', 'index.html', 'webgs.js', 'webgs.wasm']);
});

test('buildId changes with names and with content', () => {
  const a = buildId(['x'], [Buffer.from('1')]);
  assert.match(a, /^[0-9a-f]{12}$/);
  assert.notEqual(a, buildId(['y'], [Buffer.from('1')]));
  assert.notEqual(a, buildId(['x'], [Buffer.from('2')]));
  assert.equal(a, buildId(['x'], [Buffer.from('1')]));
});

test('injectManifest replaces the token once, refuses a source without it', () => {
  const out = injectManifest('const M = /*__MABUR_PRECACHE__*/ null || {};', { id: 'abc', files: ['index.html'] });
  assert.equal(out, 'const M = {"id":"abc","files":["index.html"]} || {};');
  assert.throws(() => injectManifest('nothing here', { id: 'x', files: [] }), /__MABUR_PRECACHE__/);
});

test('shouldRegister: not isolated -> yes; isolated by the OLD coi worker -> yes; ours or real headers -> no', () => {
  const own = 'https://gilankpam.github.io/mabur/sw.js';
  assert.equal(shouldRegister({ isolated: false, controllerUrl: null, ownUrl: own }), true);
  // Review Focus 1: the old coi-serviceworker makes the page isolated already.
  assert.equal(shouldRegister({ isolated: true, controllerUrl: 'https://gilankpam.github.io/mabur/coi-serviceworker.js', ownUrl: own }), true);
  assert.equal(shouldRegister({ isolated: true, controllerUrl: own, ownUrl: own }), false);
  assert.equal(shouldRegister({ isolated: true, controllerUrl: null, ownUrl: own }), false);   // serve.py headers
});

test('sw.js carries the precache token and an identical shouldRegister', () => {
  const src = fs.readFileSync(new URL('../ui/sw/sw.js', import.meta.url), 'utf8');
  assert.ok(src.includes('/*__MABUR_PRECACHE__*/ null'));
  assert.ok(src.includes(shouldRegister.toString()), 'sw.js inline shouldRegister drifted from plugin.mjs');
});
