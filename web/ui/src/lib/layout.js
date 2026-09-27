// Layout rules from the handoff ("Screens / views", "Mobile", floating panel).
export const FLOAT_W = { desktop: 272, mobile: 236 };

export function isMobile(w, h) {
  return w < 760 || (h < 500 && w < 1000);
}

export function layoutMode({ w, h, fs }) {
  const mobile = isMobile(w, h);
  if (mobile && h > w) return 'portrait';
  return mobile || fs ? 'immersive' : 'windowed';
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
