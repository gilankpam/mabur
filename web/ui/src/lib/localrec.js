// Local (browser, OPFS) recording: pure view logic + OPFS helpers (spec
// 2026-09-28-web-local-recording §1.3-§2.4). The core records; this file
// names files, reads its lrec_* stats and manages what OPFS holds.

export const DVR_TARGETS = ['web', 'vtx', 'both'];

// Spotter has no send path, so only the local recorder exists there.
export function effectiveTarget(dvr, mode) {
  if (mode === 'spotter') return 'web';
  return DVR_TARGETS.includes(dvr) ? dvr : 'web';
}

export function targetCovers(target) {
  return { web: target === 'web' || target === 'both', vtx: target === 'vtx' || target === 'both' };
}

// mabur-YYYYMMDD-HHMMSS.mp4 from the browser's local clock (it is right,
// unlike the GS RTC). [A-Za-z0-9.-] only: the core echoes it into JSON.
export function recFileName(date, existing) {
  const p = (x) => String(x).padStart(2, '0');
  const base = `mabur-${date.getFullYear()}${p(date.getMonth() + 1)}${p(date.getDate())}`
    + `-${p(date.getHours())}${p(date.getMinutes())}${p(date.getSeconds())}`;
  const have = new Set(existing);
  if (!have.has(`${base}.mp4`)) return `${base}.mp4`;
  for (let i = 2; ; i++) if (!have.has(`${base}-${i}.mp4`)) return `${base}-${i}.mp4`;
}

export const LREC_ERR_TEXT = {
  1: 'Local recording: cannot create file',
  2: 'Local recording: storage full or write failed',
  3: 'Local recording: browser storage unavailable',
};

export function localView(core) {
  if (!core || core.lrec_state == null) return { state: 'unknown', err: null, bytes: 0 };
  const bytes = core.lrec_bytes || 0;
  switch (core.lrec_state) {
    case 1: return { state: 'waiting', err: null, bytes };
    case 2: return { state: 'recording', err: null, bytes };
    case 3: return { state: 'error', err: LREC_ERR_TEXT[core.lrec_err] || `Local recording error ${core.lrec_err}`, bytes };
    default: return { state: 'off', err: null, bytes };
  }
}

// One indicator for the target's recorders: error beats recording beats
// waiting; "recording" as soon as EITHER covered recorder confirms.
export function combinedRec({ target, vtx, local }) {
  const c = targetCovers(target);
  const v = c.vtx ? vtx : { state: 'off', err: null };
  const l = c.web ? local : { state: 'off', err: null };
  if (l.state === 'error') return { state: 'error', err: l.err };
  if (v.state === 'error') return { state: 'error', err: c.web ? `VTX: ${v.err}` : v.err };
  if (v.state === 'recording' || l.state === 'recording') return { state: 'recording', err: null };
  if (l.state === 'waiting') return { state: 'waiting', err: null };
  return { state: 'off', err: null };
}

export const LOW_HEADROOM_BYTES = 1024 * 1024 * 1024;

export function headroomWarning(est) {
  if (!est || !Number.isFinite(est.quota) || !Number.isFinite(est.usage)) return null;
  const free = est.quota - est.usage;
  if (free >= LOW_HEADROOM_BYTES) return null;
  return `Low browser storage: ~${Math.max(0, Math.round(free / (1024 * 1024)))} MB free`;
}

export function formatBytes(n) {
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KB`;
  if (n < 1024 * 1024 * 1024) return `${(n / (1024 * 1024)).toFixed(1)} MB`;
  return `${(n / (1024 * 1024 * 1024)).toFixed(2)} GB`;
}

export async function opfsRoot() {
  try { return navigator.storage?.getDirectory ? await navigator.storage.getDirectory() : null; }
  catch { return null; }
}

// A file whose sync access handle the core still holds (the one being
// recorded) can refuse getFile(): skip it rather than fail the list.
export async function listRecordings(root) {
  const out = [];
  for await (const [name, h] of root.entries()) {
    if (h.kind !== 'file' || !name.endsWith('.mp4')) continue;
    try { const f = await h.getFile(); out.push({ name, size: f.size, lastModified: f.lastModified }); }
    catch { out.push({ name, size: null, lastModified: Infinity }); }
  }
  return out.sort((a, b) => b.lastModified - a.lastModified);
}

// getFile() is disk-backed: no RAM copy of a multi-GB flight.
export async function downloadRecording(root, name, doc = document, later = (fn) => setTimeout(fn, 60000)) {
  const file = await (await root.getFileHandle(name)).getFile();
  const url = URL.createObjectURL(file);
  const a = doc.createElement('a');
  a.href = url;
  a.download = name;
  a.click();
  later(() => URL.revokeObjectURL(url));
}

export async function deleteRecording(root, name) {
  await root.removeEntry(name);
}
