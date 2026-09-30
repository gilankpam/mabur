// CPE relay connect check: one WebSocket probe to one address. The probe
// sends HELLO and waits for any mabur v3 message (the relay answers HELLO
// with STATUS), so a found relay is proven to be mabur-relay v3.
//
// On https it also carries Chrome's Local Network Access (LNA) gate: a
// public page reaching a private IP needs the 'local-network' permission
// (verified Chrome 147, 2026-09-29; 'local-network-access' does NOT unblock
// WebSocket). While the permission is at 'prompt', Chrome holds the socket
// on a one-time prompt — the probe is what raises it, so it gets the long
// timeout. See docs/web-gs.md "CPE relay radio".

export const HELLO = new Uint8Array([0x4D, 0x52, 0x03, 0x02]);   // magic 0x524D LE, v3, HELLO

export const LNA_DENIED = 'Local network access is blocked for this site, so the page cannot reach the CPE relay. '
  + 'Allow it in the site settings (icon left of the address bar → Local network access), then press Connect.';
export const LNA_UNANSWERED = 'Allow local network access when Chrome asks, then press Connect.';
const BROWSER_BLOCK = ' This browser may also block ws:// from an https page: use Chrome 142+, or the local page '
  + '(python3 web/serve.py 8808, open http://127.0.0.1:8808).';

export function classifyReply(data) {
  if (!(data instanceof ArrayBuffer) || data.byteLength < 3) return 'none';
  const b = new Uint8Array(data);
  if (b[0] !== 0x4D || b[1] !== 0x52) return 'none';
  return b[2] === 3 ? 'relay' : { version: b[2] };
}

export function relayDisplay(addr) {
  return String(addr).endsWith(':8311') ? String(addr).slice(0, -5) : String(addr);
}

function fail(code, message) { const e = new Error(message); e.code = code; return e; }

export async function connectRelay({ addr, protocol, query, probe, promptMs = 60000, probeMs = 1500 }) {
  let lna = 'none';            // http: no LNA gate for a local page
  if (protocol === 'https:') {
    try { lna = (await query()).state; } catch { lna = 'unsupported'; }
  }
  if (lna === 'denied') throw fail('lna-denied', LNA_DENIED);
  const r = await probe('ws://' + addr, lna === 'prompt' ? promptMs : probeMs);
  if (r === 'relay') return;
  if (lna === 'prompt') {
    let after;
    try { after = (await query()).state; } catch { after = 'granted'; }
    if (after === 'denied') throw fail('lna-denied', LNA_DENIED);
    if (after !== 'granted') throw fail('lna-unanswered', LNA_UNANSWERED);
  }
  if (typeof r === 'object') {
    throw fail('relay-version', `CPE relay at ${relayDisplay(addr)} runs protocol v${r.version}; this page needs v3 — update the CPE firmware.`);
  }
  throw fail('relay-not-found', `No CPE relay found at ${relayDisplay(addr)} — check the Ethernet cable and that the CPE is powered, or enter its address.`
    + (lna === 'unsupported' ? BROWSER_BLOCK : ''));
}

export const lnaQuery = () => navigator.permissions.query({ name: 'local-network' });

// Browser probe. Settles once: first reply, close, or timeout.
export function probeRelay(url, ms) {
  return new Promise((resolve) => {
    let ws, settled = false;
    const done = (r) => {
      if (settled) return;
      settled = true; clearTimeout(t);
      try { ws.close(); } catch { /* never opened */ }
      resolve(r);
    };
    const t = setTimeout(() => done('none'), ms);
    try { ws = new WebSocket(url); } catch { done('none'); return; }
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => { try { ws.send(HELLO); } catch { done('none'); } };
    ws.onmessage = (e) => done(classifyReply(e.data));
    ws.onclose = () => done('none');
  });
}
