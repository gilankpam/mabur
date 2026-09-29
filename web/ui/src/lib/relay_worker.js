// Relay Worker: owns the WebSocket to mabur-relay and moves whole messages
// between it and the WASM core through the SharedArrayBuffer rings
// (relay_ring.js). Off the UI thread so rendering/GC never delays an uplink
// frame. Dumb pipe: the core speaks the protocol.
//
// The core queues HELLO+TUNE into TX before this Worker's socket has even
// opened, so the loop must never spin synchronously while the socket isn't
// open (review round 1, defect 1): workerStep is the pure per-iteration
// decision, unit-tested in relay_worker.test.mjs without a browser.
import { RING, rxRing, txRing, ringWrite, ringRead, workerStep } from './relay_ring.js';

self.onmessage = (e) => {
  const { buffer, ptr, url } = e.data;
  const rx = rxRing(buffer, ptr), tx = txRing(buffer, ptr);
  const i32 = rx.i32;
  const notifyState = () => { Atomics.notify(i32, RING.STATE); Atomics.notify(i32, RING.RX_HEAD); };
  const setState = (s) => { Atomics.store(i32, RING.STATE, s); notifyState(); };
  let ws;
  try { ws = new WebSocket(url); } catch { setState(RING.CLOSED); self.close(); return; }
  ws.binaryType = 'arraybuffer';
  let open = false;
  const pump = () => { if (!open) return; let m; while ((m = ringRead(tx))) ws.send(m); };
  ws.onopen = () => {
    // A STOP_REQ (or, in principle, a CLOSED) written while connecting must
    // not be clobbered back to OPEN (defect 3).
    if (Atomics.compareExchange(i32, RING.STATE, RING.CONNECTING, RING.OPEN) !== RING.CONNECTING) {
      try { ws.close(); } catch { /* already gone */ }
      self.close();
      return;
    }
    open = true;
    notifyState();
    pump();
  };
  ws.onmessage = (ev) => {
    if (!(ev.data instanceof ArrayBuffer)) return;   // ignore stray text frames (defect 4)
    ringWrite(rx, new Uint8Array(ev.data));
    Atomics.notify(i32, RING.RX_HEAD);
  };
  ws.onclose = ws.onerror = () => {
    // Same clobber guard as onopen: only overwrite CONNECTING/OPEN, never a
    // STOP_REQ the core already wrote.
    const prev = Atomics.load(i32, RING.STATE);
    if (prev === RING.CONNECTING || prev === RING.OPEN) Atomics.compareExchange(i32, RING.STATE, prev, RING.CLOSED);
    notifyState();
    self.close();
  };
  (async () => {
    for (;;) {
      const state = Atomics.load(i32, RING.STATE);
      const h = Atomics.load(i32, RING.TX_HEAD);
      const txEmpty = h === Atomics.load(i32, RING.TX_TAIL);
      const step = workerStep(state, open, txEmpty);
      if (step === 'stop') { try { ws.close(); } catch { /* */ } self.close(); return; }
      if (step === 'wait-open') {
        // Nothing can be sent until onopen (or onclose/onerror) fires; never
        // check TX/pump in this state, or we spin sync forever (defect 1).
        const w = Atomics.waitAsync(i32, RING.STATE, RING.CONNECTING, 500);
        if (w.async) await w.value;
        continue;
      }
      pump();
      if (step === 'pump-then-wait-tx') {
        const w = Atomics.waitAsync(i32, RING.TX_HEAD, h, 500);
        if (w.async) await w.value;
      }
    }
  })();
};
