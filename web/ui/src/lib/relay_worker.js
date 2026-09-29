// Relay Worker: owns the WebSocket to mabur-relay and moves whole messages
// between it and the WASM core through the SharedArrayBuffer rings
// (relay_ring.js). Off the UI thread so rendering/GC never delays an uplink
// frame. Dumb pipe: the core speaks the protocol.
import { RING, rxRing, txRing, ringWrite, ringRead } from './relay_ring.js';

self.onmessage = (e) => {
  const { buffer, ptr, url } = e.data;
  const rx = rxRing(buffer, ptr), tx = txRing(buffer, ptr);
  const i32 = rx.i32;
  const setState = (s) => { Atomics.store(i32, RING.STATE, s); Atomics.notify(i32, RING.RX_HEAD); };
  let ws;
  try { ws = new WebSocket(url); } catch { setState(RING.CLOSED); self.close(); return; }
  ws.binaryType = 'arraybuffer';
  let open = false;
  const pump = () => { if (!open) return; let m; while ((m = ringRead(tx))) ws.send(m); };
  ws.onopen = () => { open = true; setState(RING.OPEN); pump(); };
  ws.onmessage = (ev) => { ringWrite(rx, new Uint8Array(ev.data)); Atomics.notify(i32, RING.RX_HEAD); };
  ws.onclose = ws.onerror = () => { setState(RING.CLOSED); self.close(); };
  (async () => {
    for (;;) {
      if (Atomics.load(i32, RING.STATE) === RING.STOP_REQ) { try { ws.close(); } catch { /* */ } self.close(); return; }
      pump();
      const h = Atomics.load(i32, RING.TX_HEAD);
      if (h === Atomics.load(i32, RING.TX_TAIL)) {
        const w = Atomics.waitAsync(i32, RING.TX_HEAD, h, 500);
        if (w.async) await w.value;
      }
    }
  })();
};
