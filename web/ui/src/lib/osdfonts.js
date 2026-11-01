// OSD font manifest: public/fonts/index.json is the one list of fonts the page
// reads, Betaflight included: [{ id, name, file }] (file is relative to the page),
// written by tools/msp/gen_webfont.py. The list ships with every build and is
// precached; there is no built-in fallback list.
export const DEFAULT_FONT = 'btfl';   // the id of the Betaflight entry, and of every saved config's default

const ID_RE = /^[a-z0-9_-]{1,32}$/;
const FILE_RE = /^(?:fonts\/)?[A-Za-z0-9_.-]+\.png$/;   // same-origin, no path escapes

export function normalizeFontList(raw) {
  const out = [];
  if (!Array.isArray(raw)) return out;
  for (const f of raw) {
    if (!f || typeof f !== 'object' || !ID_RE.test(f.id) || !FILE_RE.test(f.file)) continue;
    if (f.file.includes('..') || out.some((o) => o.id === f.id)) continue;
    out.push({ id: f.id, name: typeof f.name === 'string' && f.name ? f.name.slice(0, 40) : f.id, file: f.file });
  }
  return out;
}

// The font list itself is unusable (unreachable, not JSON, or no DEFAULT_FONT
// entry). Distinct from a failed atlas: falling back to the default font cannot
// help, since the default is resolved through the same list.
export class FontListError extends Error {}

// Rejects with a FontListError when the list cannot be loaded or has no
// DEFAULT_FONT entry; the caller reports it (the OSD stays off until a later
// attempt succeeds).
export async function loadFontList(fetchFn, base) {
  let raw;
  try {
    const r = await fetchFn(new URL('fonts/index.json', base));
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    raw = await r.json();
  } catch (e) {
    throw new FontListError(`fonts/index.json: ${e?.message ?? e}`, { cause: e });
  }
  const list = normalizeFontList(raw);
  if (!list.some((f) => f.id === DEFAULT_FONT)) throw new FontListError(`fonts/index.json has no "${DEFAULT_FONT}" font`);
  return list;
}

// The file for an id; an id the list lacks (a font removed since it was chosen)
// falls back to the DEFAULT_FONT entry.
export function fontFile(list, id) {
  const f = list.find((x) => x.id === id) || list.find((x) => x.id === DEFAULT_FONT);
  if (!f) throw new FontListError(`no "${DEFAULT_FONT}" font in the list`);
  return f.file;
}
