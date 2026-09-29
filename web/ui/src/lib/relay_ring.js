// SPSC byte ring between the WASM core and the relay Worker. The C++ twin
// is web/src/relay_ring.h -- keep them byte-identical (shared goldens in
// tests/test_relay_ring.cpp and web/tests/relay_ring.test.mjs).
export const RING = {
  HDR: 32, RX_CAP: 1 << 20, TX_CAP: 1 << 16,
  RX_HEAD: 0, RX_TAIL: 1, TX_HEAD: 2, TX_TAIL: 3, STATE: 4, RX_DROPS: 5, TX_DROPS: 6,
  CONNECTING: 0, OPEN: 1, CLOSED: 2, STOP_REQ: 3,
};
const WRAP = 0xFFFFFFFF;

function view(buffer, base, head, tail, drops, off, cap) {
  return { i32: new Int32Array(buffer, base, 8), u8: new Uint8Array(buffer), head, tail, drops, off, cap };
}
export const rxRing = (buffer, base) =>
  view(buffer, base, RING.RX_HEAD, RING.RX_TAIL, RING.RX_DROPS, base + RING.HDR, RING.RX_CAP);
export const txRing = (buffer, base) =>
  view(buffer, base, RING.TX_HEAD, RING.TX_TAIL, RING.TX_DROPS, base + RING.HDR + RING.RX_CAP, RING.TX_CAP);

function putU32(u8, at, v) { u8[at] = v & 255; u8[at + 1] = (v >>> 8) & 255; u8[at + 2] = (v >>> 16) & 255; u8[at + 3] = v >>> 24; }
function getU32(u8, at) { return (u8[at] | (u8[at + 1] << 8) | (u8[at + 2] << 16) | (u8[at + 3] << 24)) >>> 0; }

export function ringWrite(r, bytes) {
  const n = bytes.length;
  const need = 4 + ((n + 3) & ~3);
  const head = Atomics.load(r.i32, r.head) >>> 0;
  const tail = Atomics.load(r.i32, r.tail) >>> 0;
  const pos = head & (r.cap - 1);
  const roomEnd = r.cap - pos;
  const total = roomEnd < need ? roomEnd + need : need;
  if (need > r.cap / 2 || total > r.cap - ((head - tail) >>> 0)) {
    Atomics.add(r.i32, r.drops, 1);
    return false;
  }
  let h = head, at = pos;
  if (roomEnd < need) { putU32(r.u8, r.off + pos, WRAP); h = (h + roomEnd) >>> 0; at = 0; }
  putU32(r.u8, r.off + at, n);
  r.u8.set(bytes, r.off + at + 4);
  Atomics.store(r.i32, r.head, (h + need) | 0);
  return true;
}

export function ringRead(r) {
  let tail = Atomics.load(r.i32, r.tail) >>> 0;
  for (;;) {
    const head = Atomics.load(r.i32, r.head) >>> 0;
    if (tail === head) return null;
    const pos = tail & (r.cap - 1);
    const len = getU32(r.u8, r.off + pos);
    if (len === WRAP) { tail = (tail + r.cap - pos) >>> 0; Atomics.store(r.i32, r.tail, tail | 0); continue; }
    const out = r.u8.slice(r.off + pos + 4, r.off + pos + 4 + len);   // copy: never a shared view
    Atomics.store(r.i32, r.tail, (tail + 4 + ((len + 3) & ~3)) | 0);
    return out;
  }
}
