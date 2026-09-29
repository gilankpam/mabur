// MSP OSD overlay (spec 2026-09-27-web-msp-osd). The core publishes the FC's
// DisplayPort grid as Module.onOsd(rows, cols, Uint16Array), cell =
// char | page << 8; this file lays it out over the visible video box and
// paints glyphs from the Betaflight atlas (web/ui/public/font_btfl.png, made
// by tools/msp/gen_webfont.py from maburplay's font_btfl.mfont).

export const GLYPH_W = 36;
export const GLYPH_H = 54;
export const ATLAS_COLS = 32;
export const OSD_STALE_MS = 5000;   // maburplay's osd.stale_ms default

export function atlasRect(gi) {
  return { sx: (gi % ATLAS_COLS) * GLYPH_W, sy: Math.floor(gi / ATLAS_COLS) * GLYPH_H, sw: GLYPH_W, sh: GLYPH_H };
}

// Cell (r, c) of a rows x cols grid spread over the whole box. Edges are
// rounded from the exact pitch, so neighbours share an edge (no seams in
// multi-cell art) and the last cell ends exactly on the box edge.
export function cellRect(boxW, boxH, rows, cols, r, c) {
  const x0 = Math.round((c * boxW) / cols), x1 = Math.round(((c + 1) * boxW) / cols);
  const y0 = Math.round((r * boxH) / rows), y1 = Math.round(((r + 1) * boxH) / rows);
  return { x: x0, y: y0, w: x1 - x0, h: y1 - y0 };
}

// MspScreen::is_blank: NUL or space, whatever the font page.
export function isBlank(cell) {
  const ch = cell & 0xff;
  return ch === 0 || ch === 0x20;
}

// The latest screen and when it arrived (page clock, ms).
export class OsdLayer {
  constructor() { this.clear(); }
  onScreen(rows, cols, cells, nowMs) { this.screen = { rows, cols, cells }; this.lastMs = nowMs; }
  // Never stale before the first screen: a link without MSP has nothing to blank.
  isStale(nowMs) { return this.lastMs !== null && nowMs - this.lastMs >= OSD_STALE_MS; }
  clear() { this.screen = null; this.lastMs = null; }
  drawList(boxW, boxH) {
    const s = this.screen;
    const out = [];
    if (!s || boxW <= 0 || boxH <= 0) return out;
    for (let r = 0; r < s.rows; r++) {
      for (let c = 0; c < s.cols; c++) {
        const cell = s.cells[r * s.cols + c];
        if (isBlank(cell)) continue;
        const d = cellRect(boxW, boxH, s.rows, s.cols, r, c);
        out.push({ ...atlasRect(cell), dx: d.x, dy: d.y, dw: d.w, dh: d.h });
      }
    }
    return out;
  }
}

// Paints an OsdLayer onto the overlay canvas: one rAF repaint per burst of
// schedule() calls, full clear + redraw (<= ~900 drawImage from one bitmap).
// The atlas is fetched on the first paint that has a screen; a failed fetch
// logs once and leaves the OSD off for the rest of that session (never a
// refetch per screen) -- video is unaffected. resetAtlasFailure(), called on
// Connect, allows one more attempt per session.
export class OsdPainter {
  constructor({ layer, loadAtlas, raf = (f) => requestAnimationFrame(f), log = console }) {
    Object.assign(this, { layer, loadAtlas, raf, log });
    this.canvas = null;
    this.atlas = null;
    this.atlasP = null;
    this.atlasFailed = false;
    this.pending = false;
  }

  attach(canvas) { this.canvas = canvas; this.schedule(); }

  // CSS box size x devicePixelRatio -> backing store. Setting width/height
  // wipes the bitmap, so a real change always repaints.
  resize(cssW, cssH, dpr) {
    const c = this.canvas;
    if (!c) return;
    const w = Math.max(0, Math.round(cssW * dpr)), h = Math.max(0, Math.round(cssH * dpr));
    if (c.width === w && c.height === h) return;
    c.width = w;
    c.height = h;
    this.schedule();
  }

  schedule() {
    if (this.pending) return;
    this.pending = true;
    this.raf(() => { this.pending = false; this.paint(); });
  }

  paint() {
    const c = this.canvas;
    if (!c) return;
    const ctx = c.getContext('2d');
    ctx.clearRect(0, 0, c.width, c.height);
    if (!this.layer.screen) return;
    if (!this.atlas) { this.fetchAtlas(); return; }
    ctx.imageSmoothingEnabled = true;
    ctx.imageSmoothingQuality = 'high';
    for (const d of this.layer.drawList(c.width, c.height))
      ctx.drawImage(this.atlas, d.sx, d.sy, d.sw, d.sh, d.dx, d.dy, d.dw, d.dh);
  }

  fetchAtlas() {
    if (this.atlasP) return;
    this.atlasP = Promise.resolve().then(() => this.loadAtlas()).then(
      (bmp) => { this.atlas = bmp; this.schedule(); },
      (e) => {
        this.atlasFailed = true;
        this.log.error('[webgs] OSD font atlas failed to load; OSD off until the next Connect', e);
      });
  }

  // A loaded or in-flight atlas is kept; only a failed one is forgotten.
  resetAtlasFailure() {
    if (!this.atlasFailed) return;
    this.atlasFailed = false;
    this.atlasP = null;
  }
}
