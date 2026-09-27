import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { GLYPH_W, GLYPH_H, ATLAS_COLS, OSD_STALE_MS, atlasRect, cellRect, isBlank, OsdLayer, OsdPainter }
  from '../ui/src/lib/osd.js';

test('atlas PNG matches the page constants', () => {
  const png = fs.readFileSync(new URL('../ui/public/font_btfl.png', import.meta.url));
  assert.equal(png.readUInt32BE(16), ATLAS_COLS * GLYPH_W);          // IHDR width
  assert.equal(png.readUInt32BE(20), (1024 / ATLAS_COLS) * GLYPH_H); // IHDR height
});

test('atlasRect corners', () => {
  assert.deepEqual(atlasRect(0), { sx: 0, sy: 0, sw: 36, sh: 54 });
  assert.deepEqual(atlasRect(31), { sx: 1116, sy: 0, sw: 36, sh: 54 });
  assert.deepEqual(atlasRect(32), { sx: 0, sy: 54, sw: 36, sh: 54 });
  assert.deepEqual(atlasRect(1023), { sx: 1116, sy: 1674, sw: 36, sh: 54 });
});

function tiles(boxW, boxH, rows, cols) {
  let rowW = 0, colH = 0;
  for (let c = 0; c < cols; c++) rowW += cellRect(boxW, boxH, rows, cols, 0, c).w;
  for (let r = 0; r < rows; r++) colH += cellRect(boxW, boxH, rows, cols, r, 0).h;
  const last = cellRect(boxW, boxH, rows, cols, rows - 1, cols - 1);
  return { rowW, colH, right: last.x + last.w, bottom: last.y + last.h };
}

test('cellRect: HD 50x18 on a 16:9 box spans it exactly', () => {
  assert.deepEqual(cellRect(1920, 1080, 18, 50, 0, 0), { x: 0, y: 0, w: 38, h: 60 });
  assert.deepEqual(tiles(1920, 1080, 18, 50), { rowW: 1920, colH: 1080, right: 1920, bottom: 1080 });
});

test('cellRect: SD 30x16, a 20:9 cover box, and DPR 2 all reach the edges', () => {
  assert.deepEqual(cellRect(1280, 720, 16, 30, 15, 29), { x: 1237, y: 675, w: 43, h: 45 });
  assert.deepEqual(tiles(1280, 720, 16, 30), { rowW: 1280, colH: 720, right: 1280, bottom: 720 });
  assert.deepEqual(tiles(2400, 1080, 18, 50), { rowW: 2400, colH: 1080, right: 2400, bottom: 1080 });
  assert.deepEqual(tiles(2 * 1373, 2 * 772, 18, 50), { rowW: 2746, colH: 1544, right: 2746, bottom: 1544 });
});

test('isBlank: NUL and space on any page', () => {
  assert.equal(isBlank(0), true);
  assert.equal(isBlank(0x20), true);
  assert.equal(isBlank(0x120), true);
  assert.equal(isBlank(0x41), false);
  assert.equal(isBlank(0x141), false);
});

function grid(rows, cols, put = {}) {
  const cells = new Uint16Array(rows * cols);
  for (const [i, v] of Object.entries(put)) cells[Number(i)] = v;
  return cells;
}

test('drawList: only non-blank cells, glyph from the atlas, cell from the box', () => {
  const l = new OsdLayer();
  l.onScreen(18, 50, grid(18, 50, { 0: 0x48, 1: 0x20, 899: 0x141 }), 1000);
  const d = l.drawList(1920, 1080);
  assert.equal(d.length, 2);
  assert.deepEqual(d[0], { sx: 0x48 % 32 * 36, sy: Math.floor(0x48 / 32) * 54, sw: 36, sh: 54, dx: 0, dy: 0, dw: 38, dh: 60 });
  assert.deepEqual([d[1].dx + d[1].dw, d[1].dy + d[1].dh], [1920, 1080]);
  assert.deepEqual(new OsdLayer().drawList(1920, 1080), []);
  assert.deepEqual(l.drawList(0, 0), []);
});

test('drawList follows an SD 30x16 screen', () => {
  const l = new OsdLayer();
  l.onScreen(16, 30, grid(16, 30, { 479: 0x41 }), 1000);
  const d = l.drawList(1280, 720);
  assert.equal(d.length, 1);
  assert.deepEqual([d[0].dx, d[0].dy, d[0].dw, d[0].dh], [1237, 675, 43, 45]);
});

test('stale clock: never before the first screen, at 5 s after the last', () => {
  const l = new OsdLayer();
  assert.equal(l.isStale(1e9), false);
  l.onScreen(18, 50, grid(18, 50), 1000);
  assert.equal(l.isStale(1000 + OSD_STALE_MS - 100), false);
  assert.equal(l.isStale(1000 + OSD_STALE_MS), true);
  l.onScreen(18, 50, grid(18, 50), 7000);
  assert.equal(l.isStale(7000 + 4900), false);
  l.clear();
  assert.equal(l.screen, null);
  assert.equal(l.isStale(1e9), false);
});

function fakeCanvas(w = 1920, h = 1080) {
  const ctx = { clears: 0, draws: [], clearRect() { this.clears++; }, drawImage(...a) { this.draws.push(a); } };
  return { width: w, height: h, ctx, getContext: () => ctx };
}
function fakeRaf() {
  const q = [];
  const raf = (f) => q.push(f);
  raf.flush = () => { while (q.length) q.shift()(); };
  raf.pending = () => q.length;
  return raf;
}
const settle = () => new Promise((r) => setTimeout(r, 0));

test('painter: fetches the atlas once, lazily, then draws non-blank cells', async () => {
  const layer = new OsdLayer();
  const raf = fakeRaf();
  let loads = 0;
  const atlas = { id: 'atlas' };
  const p = new OsdPainter({ layer, raf, loadAtlas: async () => { loads++; return atlas; } });
  const cv = fakeCanvas();
  p.attach(cv);
  raf.flush();
  assert.equal(loads, 0);                       // no screen, no fetch
  layer.onScreen(18, 50, grid(18, 50, { 0: 0x48, 5: 0x49 }), 1000);
  p.schedule(); p.schedule(); p.schedule();
  assert.equal(raf.pending(), 1);               // bursts coalesce
  raf.flush();
  assert.equal(cv.ctx.draws.length, 0);         // atlas not loaded yet
  await settle();
  raf.flush();                                  // atlas arrival schedules a repaint
  assert.equal(loads, 1);
  assert.equal(cv.ctx.draws.length, 2);
  assert.equal(cv.ctx.draws[0][0], atlas);
  assert.deepEqual(cv.ctx.draws[0].slice(5), [0, 0, 38, 60]);
});

test('painter: a failed atlas load logs once, never refetches, never throws', async () => {
  const layer = new OsdLayer();
  const raf = fakeRaf();
  let loads = 0;
  const errors = [];
  const p = new OsdPainter({ layer, raf, log: { error: (...a) => errors.push(a) },
    loadAtlas: async () => { loads++; throw new Error('404'); } });
  const cv = fakeCanvas();
  p.attach(cv);
  for (let i = 0; i < 20; i++) {
    layer.onScreen(18, 50, grid(18, 50, { 0: 0x48 }), 1000 + 33 * i);
    p.schedule();
    raf.flush();
    await settle();
  }
  assert.equal(loads, 1);
  assert.equal(errors.length, 1);
  assert.equal(cv.ctx.draws.length, 0);
});

test('painter: resize repaints at the new backing size; same size is a no-op', async () => {
  const layer = new OsdLayer();
  const raf = fakeRaf();
  const p = new OsdPainter({ layer, raf, loadAtlas: async () => ({}) });
  const cv = fakeCanvas(10, 10);
  p.attach(cv);
  layer.onScreen(18, 50, grid(18, 50, { 899: 0x41 }), 1000);
  p.schedule(); raf.flush(); await settle(); raf.flush();
  p.resize(960, 540, 2);
  assert.deepEqual([cv.width, cv.height], [1920, 1080]);
  cv.ctx.draws.length = 0;
  raf.flush();
  const last = cv.ctx.draws.at(-1);
  assert.deepEqual([last[5] + last[7], last[6] + last[8]], [1920, 1080]);
  p.resize(960, 540, 2);
  assert.equal(raf.pending(), 0);
});

test('painter: a cleared layer paints an empty canvas', async () => {
  const layer = new OsdLayer();
  const raf = fakeRaf();
  const p = new OsdPainter({ layer, raf, loadAtlas: async () => ({}) });
  const cv = fakeCanvas();
  p.attach(cv);
  layer.onScreen(18, 50, grid(18, 50, { 0: 0x48 }), 1000);
  p.schedule(); raf.flush(); await settle(); raf.flush();
  const clearsBefore = cv.ctx.clears;
  cv.ctx.draws.length = 0;
  layer.clear();
  p.schedule(); raf.flush();
  assert.equal(cv.ctx.clears, clearsBefore + 1);
  assert.equal(cv.ctx.draws.length, 0);
});

test('painter: a failed atlas load is retried once per session after resetAtlasFailure()', async () => {
  const layer = new OsdLayer();
  const raf = fakeRaf();
  let loads = 0;
  const p = new OsdPainter({ layer, raf, log: { error: () => {} },
    loadAtlas: async () => { loads++; if (loads === 1) throw new Error('offline'); return { id: 'atlas' }; } });
  const cv = fakeCanvas();
  p.attach(cv);
  layer.onScreen(18, 50, grid(18, 50, { 0: 0x48 }), 1000);
  p.schedule(); raf.flush(); await settle(); raf.flush();
  assert.equal(loads, 1);
  assert.equal(cv.ctx.draws.length, 0);
  p.schedule(); raf.flush(); await settle();
  assert.equal(loads, 1);                       // still latched within the session
  p.resetAtlasFailure();                        // next Connect
  p.schedule(); raf.flush(); await settle(); raf.flush();
  assert.equal(loads, 2);
  assert.equal(cv.ctx.draws.length, 1);
  p.resetAtlasFailure();                        // a loaded atlas is kept
  p.schedule(); raf.flush(); await settle(); raf.flush();
  assert.equal(loads, 2);
});
