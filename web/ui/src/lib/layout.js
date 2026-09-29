// Layout rules from the handoff ("Screens / views", "Mobile", floating panel).
export const FLOAT_W = { desktop: 272, mobile: 236 };

export function isMobile(w, h) {
  return w < 760 || (h < 500 && w < 1000);
}

// Phones are landscape-only: held upright (w x h = the window), the whole UI
// is drawn rotated 90° -- its top edge along the screen's right edge, the
// landscape-primary view -- so turning the phone just un-rotates it. lw x lh
// is the UI's own (landscape) size; toLocal maps a client (screen) point into
// it, the inverse of the transform in style.
export function uiFrame(w, h) {
  if (!(isMobile(w, h) && h > w)) return { lw: w, lh: h, style: '', toLocal: (x, y) => ({ x, y }) };
  return { lw: h, lh: w,
    style: `inset:auto;left:0;top:0;width:${h}px;height:${w}px;transform-origin:0 0;transform:translateX(${w}px) rotate(90deg)`,
    toLocal: (x, y) => ({ x: y, y: w - x }) };
}

// w x h is the UI's own size: App already swaps it for an upright phone,
// which it draws rotated.
export function layoutMode({ w, h, fs }) {
  return isMobile(w, h) || fs ? 'immersive' : 'windowed';
}

// 8 px inside the screen; keep 48 px of the panel reachable at the bottom.
export function clampFloatPos({ x, y }, { cw, ch }, fw) {
  return {
    x: Math.max(8, Math.min(cw - fw - 8, x)),
    y: Math.max(8, Math.min(ch - 48, y)),
  };
}

export function dragStarted(sx, sy, x, y) {
  return Math.hypot(x - sx, y - sy) >= 5;
}

export function keyAction(e) {
  if (!e || e.ctrlKey || e.metaKey || e.altKey) return null;
  if (/^(INPUT|SELECT|TEXTAREA)$/.test(e.target?.tagName || '')) return null;
  const k = String(e.key || '').toLowerCase();
  if (k === 'f') return 'fs';
  if (k === 'r') return 'rec';
  if (k === 's') return 'stats';
  if (k === 'escape') return 'esc';
  return null;
}
