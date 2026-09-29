// Chrome's Local Network Access (LNA) gate for the CPE relay socket.
//
// A public https page (the hosted GitHub Pages build) may open plain ws:// to
// a private IP / .local host (LNA exempts it from mixed-content blocking), but
// Chrome 147 then holds the socket on a one-time "local network access"
// prompt. The core gives up on an unopened relay in ~2 s, which would fail the
// session underneath the prompt. So on https, before the core starts, a
// throwaway socket raises the prompt and Connect waits for the user's answer.
// Verified 2026-09-29, Chrome 147: permission 'local-network' (not
// 'local-network-access') is the one that unblocks WebSocket; see
// docs/web-gs.md "CPE relay radio".

export const LNA_DENIED = 'Local network access is blocked for this site, so the page cannot reach the CPE relay. '
  + 'Allow it in the site settings (icon left of the address bar → Local network access), then press Connect.';
export const LNA_UNANSWERED = 'Allow local network access when Chrome asks, then press Connect.';

// query(): Promise<{state}> for the 'local-network' permission (throws where
// the browser has no such permission). openSocket(url, ms): Promise<'open'|
// 'closed'|'timeout'>. Resolves when the core may start; throws Error(msg)
// when it can't usefully. Anything unexpected falls through to the core,
// whose own "relay unreachable" handling then applies.
export async function ensureLocalNetwork({ protocol, url, query, openSocket, promptMs = 60000 }) {
  if (protocol !== 'https:') return;
  let state;
  try { state = (await query()).state; } catch { return; }   // no LNA in this browser
  if (state === 'granted') return;
  if (state === 'denied') throw new Error(LNA_DENIED);
  await openSocket(url, promptMs);   // raises the prompt; settles on the answer
  try { state = (await query()).state; } catch { return; }
  if (state === 'denied') throw new Error(LNA_DENIED);
  if (state !== 'granted') throw new Error(LNA_UNANSWERED);
}

export const lnaQuery = () => navigator.permissions.query({ name: 'local-network' });

export function openProbeSocket(url, ms) {
  return new Promise((resolve) => {
    let ws;
    const done = (r) => { clearTimeout(t); try { ws.close(); } catch { /* gone */ } resolve(r); };
    const t = setTimeout(() => done('timeout'), ms);
    try { ws = new WebSocket(url); } catch { clearTimeout(t); resolve('closed'); return; }
    ws.onopen = () => done('open');
    ws.onclose = () => done('closed');
  });
}
