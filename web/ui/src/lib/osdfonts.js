// OSD font manifest: public/fonts/index.json is a list of
// { id, name, file } entries (file is relative to the page), written by
// tools/msp/gen_webfonts.py. The shipped Betaflight atlas is always entry
// 'btfl', whether or not a manifest exists.
export const DEFAULT_FONT = { id: 'btfl', name: 'Betaflight', file: 'font_btfl.png' };

const ID_RE = /^[a-z0-9_-]{1,32}$/;
const FILE_RE = /^(?:fonts\/)?[A-Za-z0-9_.-]+\.png$/;   // same-origin, no path escapes

export function normalizeFontList(raw) {
  const out = [DEFAULT_FONT];
  if (!Array.isArray(raw)) return out;
  for (const f of raw) {
    if (!f || typeof f !== 'object' || !ID_RE.test(f.id) || !FILE_RE.test(f.file)) continue;
    if (f.file.includes('..') || out.some((o) => o.id === f.id)) continue;
    out.push({ id: f.id, name: typeof f.name === 'string' && f.name ? f.name.slice(0, 40) : f.id, file: f.file });
  }
  return out;
}

export async function loadFontList(fetchFn, base) {
  try {
    const r = await fetchFn(new URL('fonts/index.json', base));
    if (!r.ok) return [DEFAULT_FONT];
    return normalizeFontList(await r.json());
  } catch { return [DEFAULT_FONT]; }
}

// The file for an id; an unknown id (a font removed since it was chosen)
// falls back to the shipped atlas.
export function fontFile(list, id) {
  return (list.find((f) => f.id === id) || DEFAULT_FONT).file;
}
