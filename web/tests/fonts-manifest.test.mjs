import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { normalizeFontList, DEFAULT_FONT } from '../ui/src/lib/osdfonts.js';
import { GLYPH_W, GLYPH_H, ATLAS_COLS } from '../ui/src/lib/osd.js';

// The committed font list the page reads (web/ui/public/fonts/index.json) and the atlases it names.
const PUBLIC = new URL('../ui/public/', import.meta.url);
const list = JSON.parse(readFileSync(new URL('fonts/index.json', PUBLIC), 'utf8'));

// PNG IHDR: width and height are big-endian uint32 at byte 16 and 20.
function pngSize(file) {
  const b = readFileSync(new URL(file, PUBLIC));
  assert.equal(b.subarray(1, 4).toString('latin1'), 'PNG', file + ' is not a PNG');
  return { w: b.readUInt32BE(16), h: b.readUInt32BE(20) };
}

test('fonts/index.json is a valid list with Betaflight first', () => {
  assert.ok(Array.isArray(list) && list.length > 0);
  assert.deepEqual(list[0], { id: DEFAULT_FONT, name: 'Betaflight', file: 'fonts/btfl.png' });
});

test('every listed font passes normalizeFontList unchanged', () => {
  assert.deepEqual(normalizeFontList(list), list);
});

test('every listed atlas exists at 32 x 32 glyphs of 36x54 (1152x1728)', () => {
  for (const f of list) {
    const { w, h } = pngSize(f.file);
    assert.equal(w, ATLAS_COLS * GLYPH_W, f.id + ' width');
    assert.equal(h, (1024 / ATLAS_COLS) * GLYPH_H, f.id + ' height');
  }
});
