import test from 'node:test';
import assert from 'node:assert/strict';
import {
  effectiveTarget, targetCovers, recFileName, localView, combinedRec, headroomWarning,
  formatBytes, listRecordings, downloadRecording, deleteRecording,
} from '../ui/src/lib/localrec.js';

test('spotter always records locally, whatever the saved target', () => {
  assert.equal(effectiveTarget('vtx', 'spotter'), 'web');
  assert.equal(effectiveTarget('both', 'spotter'), 'web');
  assert.equal(effectiveTarget('both', 'gs'), 'both');
  assert.equal(effectiveTarget('bogus', 'gs'), 'web');
  assert.deepEqual(targetCovers('both'), { web: true, vtx: true });
  assert.deepEqual(targetCovers('vtx'), { web: false, vtx: true });
  assert.deepEqual(targetCovers('web'), { web: true, vtx: false });
});

test('file name from the local clock, suffixed on collision', () => {
  const d = new Date(2026, 8, 28, 7, 5, 9);   // local time, month 0-based
  assert.equal(recFileName(d, []), 'mabur-20260928-070509.mp4');
  assert.equal(recFileName(d, ['mabur-20260928-070509.mp4']), 'mabur-20260928-070509-2.mp4');
  assert.equal(recFileName(d, ['mabur-20260928-070509.mp4', 'mabur-20260928-070509-2.mp4']),
    'mabur-20260928-070509-3.mp4');
  assert.match(recFileName(d, []), /^[A-Za-z0-9.-]+$/);   // core writes it into JSON unescaped
});

test('local view from core stats', () => {
  assert.deepEqual(localView(null), { state: 'unknown', err: null, bytes: 0 });
  assert.deepEqual(localView({ lrec_state: 0, lrec_err: 0, lrec_bytes: 0 }), { state: 'off', err: null, bytes: 0 });
  assert.equal(localView({ lrec_state: 1, lrec_err: 0, lrec_bytes: 0 }).state, 'waiting');
  assert.deepEqual(localView({ lrec_state: 2, lrec_err: 0, lrec_bytes: 5 }), { state: 'recording', err: null, bytes: 5 });
  const e = localView({ lrec_state: 3, lrec_err: 2, lrec_bytes: 9 });
  assert.equal(e.state, 'error');
  assert.equal(e.err, 'Local recording: storage full or write failed');
  assert.equal(localView({ lrec_state: 3, lrec_err: 3 }).err, 'Local recording: browser storage unavailable');
});

test('combined indicator', () => {
  const off = { state: 'off', err: null }, rec = { state: 'recording', err: null };
  const lOff = { state: 'off', err: null, bytes: 0 }, lWait = { state: 'waiting', err: null, bytes: 0 };
  const lRec = { state: 'recording', err: null, bytes: 1 };
  const lErr = { state: 'error', err: 'Local recording: cannot create file', bytes: 0 };
  // web target: VTX state ignored (a maburgs-started VTX rec is not "ours")
  assert.equal(combinedRec({ target: 'web', vtx: rec, local: lOff }).state, 'off');
  assert.equal(combinedRec({ target: 'web', vtx: off, local: lWait }).state, 'waiting');
  assert.equal(combinedRec({ target: 'web', vtx: off, local: lRec }).state, 'recording');
  // stop before sync: waiting -> off, no error
  assert.deepEqual(combinedRec({ target: 'web', vtx: off, local: lOff }), { state: 'off', err: null });
  // vtx target: local ignored
  assert.equal(combinedRec({ target: 'vtx', vtx: rec, local: lErr }).state, 'recording');
  // both: first confirmation wins; any error shows
  assert.equal(combinedRec({ target: 'both', vtx: off, local: lRec }).state, 'recording');
  assert.equal(combinedRec({ target: 'both', vtx: rec, local: lWait }).state, 'recording');
  assert.deepEqual(combinedRec({ target: 'both', vtx: rec, local: lErr }),
    { state: 'error', err: 'Local recording: cannot create file' });
  assert.equal(combinedRec({ target: 'both', vtx: { state: 'error', err: 'SD card full' }, local: lRec }).err,
    'VTX: SD card full');
});

test('headroom warning below 1 GiB free', () => {
  const G = 1024 * 1024 * 1024;
  assert.equal(headroomWarning(null), null);
  assert.equal(headroomWarning({ quota: 10 * G, usage: 1 * G }), null);
  assert.equal(headroomWarning({ quota: G + 500 * 1024 * 1024, usage: G }), 'Low browser storage: ~500 MB free');
  assert.equal(formatBytes(0), '0 B');
  assert.equal(formatBytes(1536), '1.5 KB');
  assert.equal(formatBytes(3 * 1024 * 1024 * 1024), '3.00 GB');
});

// Minimal OPFS directory-handle fake.
function fakeRoot(files) {
  const map = new Map(Object.entries(files));
  return {
    removed: [],
    async *entries() { for (const [n, f] of map) yield [n, { kind: 'file', getFile: async () => f }]; },
    async getFileHandle(n) { if (!map.has(n)) throw new Error('NotFoundError'); return { getFile: async () => map.get(n) }; },
    async removeEntry(n) { map.delete(n); this.removed.push(n); },
  };
}

test('recordings list: mp4 only, newest first', async () => {
  const root = fakeRoot({
    'mabur-a.mp4': { size: 10, lastModified: 1 },
    'mabur-b.mp4': { size: 20, lastModified: 3 },
    '.probe': { size: 1, lastModified: 9 },
  });
  assert.deepEqual(await listRecordings(root), [
    { name: 'mabur-b.mp4', size: 20, lastModified: 3 },
    { name: 'mabur-a.mp4', size: 10, lastModified: 1 },
  ]);
});

test('download clicks an object-URL anchor; delete removes the entry', async () => {
  const file = { size: 1, lastModified: 1 };
  const root = fakeRoot({ 'x.mp4': file });
  const clicked = [];
  const doc = { createElement: () => ({ click() { clicked.push({ href: this.href, download: this.download }); } }) };
  const origC = URL.createObjectURL, origR = URL.revokeObjectURL;
  URL.createObjectURL = (f) => { assert.equal(f, file); return 'blob:x'; };
  URL.revokeObjectURL = () => {};
  try {
    await downloadRecording(root, 'x.mp4', doc, (fn) => fn());
  } finally { URL.createObjectURL = origC; URL.revokeObjectURL = origR; }
  assert.deepEqual(clicked, [{ href: 'blob:x', download: 'x.mp4' }]);
  await deleteRecording(root, 'x.mp4');
  assert.deepEqual(root.removed, ['x.mp4']);
});
