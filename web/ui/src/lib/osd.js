// MSP OSD overlay. The core publishes the FC's
// DisplayPort grid as Module.onOsd(rows, cols, Uint16Array), cell =
// char | page << 8; this file lays it out over the visible video box and
// paints glyphs from the selected font's atlas (web/ui/public/fonts/*.png, listed
// in fonts/index.json; Betaflight is fonts/btfl.png, made from maburplay's
// font_btfl.mfont by tools/msp/gen_webfont.py).
import { DEFAULT_FONT, FontListError } from './osdfonts.js';

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
// The atlas is fetched on the first paint that has a screen. Two ids are kept:
// requestedId is the font the app asked for, fontId the one being drawn. They
// differ after a fallback: a non-default font that fails to load is logged once
// and the default atlas (always precached) is loaded instead, while requestedId
// stays, so resetAtlasFailure() (called on Connect) tries the requested font
// again. If the default atlas itself fails, or the font list does (whatever
// font was asked for), that is logged once and the OSD stays off (never a
// refetch per screen) until resetAtlasFailure() or setFont() allows another
// try -- video is unaffected.
export class OsdPainter {
  constructor({ layer, loadAtlas, fontId = null, raf = (f) => requestAnimationFrame(f), log = console }) {
    Object.assign(this, { layer, loadAtlas, fontId, raf, log });
    this.requestedId = fontId;
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
    if (this.atlasP || this.atlasFailed) return;
    const id = this.fontId;
    const p = Promise.resolve().then(() => this.loadAtlas(id)).then(
      (bmp) => { if (this.atlasP !== p) { bmp?.close?.(); return; } this.atlas = bmp; this.schedule(); },
      (e) => {
        if (this.atlasP !== p) return;   // a font switch superseded this load
        if (id && id !== DEFAULT_FONT && !(e instanceof FontListError)) {
          // The requested font failed (missing file, bad download). The default atlas is always
          // precached, so draw it instead of switching the OSD off; requestedId is kept. A failed
          // font list is not retried with the default: that resolves through the same list.
          this.log.error(`[webgs] OSD font "${id}" failed to load; falling back to the default font`, e);
          this.fontId = DEFAULT_FONT;
          this.atlasP = null;
          this.schedule();
          return;
        }
        this.atlasFailed = true;
        this.log.error('[webgs] OSD font atlas failed to load; OSD off until the next Connect', e);
      });
    this.atlasP = p;
  }

  // Switch the OSD font (loadAtlas receives the id). The old atlas is dropped
  // and the next paint fetches the new one; a load still in flight for the
  // previous font is ignored when it lands.
  setFont(id) {
    if (id === this.requestedId) return;
    this.requestedId = id;
    this.fontId = id;
    this.atlas?.close?.();   // free the old ~8 MB decoded bitmap
    this.atlas = null;
    this.atlasP = null;
    this.atlasFailed = false;
    this.schedule();
  }

  // Called on Connect. A loaded or in-flight atlas of the requested font is kept;
  // after a failure or a fallback the requested font is tried again.
  resetAtlasFailure() {
    if (!this.atlasFailed && this.fontId === this.requestedId) return;
    this.atlasFailed = false;
    this.fontId = this.requestedId;
    this.atlas?.close?.();
    this.atlas = null;
    this.atlasP = null;
    this.schedule();
  }
}
