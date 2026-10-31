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

// Rejects when the list cannot be loaded or has no DEFAULT_FONT entry; the caller
// reports it (the OSD stays off until a later attempt succeeds).
export async function loadFontList(fetchFn, base) {
  const r = await fetchFn(new URL('fonts/index.json', base));
  if (!r.ok) throw new Error(`fonts/index.json: HTTP ${r.status}`);
  const list = normalizeFontList(await r.json());
  if (!list.some((f) => f.id === DEFAULT_FONT)) throw new Error(`fonts/index.json has no "${DEFAULT_FONT}" font`);
  return list;
}

// The file for an id; an id the list lacks (a font removed since it was chosen)
// falls back to the DEFAULT_FONT entry.
export function fontFile(list, id) {
  const f = list.find((x) => x.id === id) || list.find((x) => x.id === DEFAULT_FONT);
  if (!f) throw new Error(`no "${DEFAULT_FONT}" font in the list`);
  return f.file;
}
