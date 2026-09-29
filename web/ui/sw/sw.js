/*! Based on coi-serviceworker v0.1.7 - Guido Zuidhof and contributors, licensed under MIT.
 *  mabur: + offline precache and network-first fetch (docs/web-gs.md "Serve").
 *  One file, two contexts: as a service worker it adds COOP/COEP (GitHub
 *  Pages can't send them) and caches the page; as a page <script> it
 *  registers itself. */
const MANIFEST = /*__MABUR_PRECACHE__*/ null || { id: 'dev', files: [] };
let coepCredentialless = false;

function shouldRegister({ isolated, controllerUrl, ownUrl }) {
  if (!isolated) return true;
  return !!controllerUrl && controllerUrl.split('?')[0] !== ownUrl.split('?')[0];
}

if (typeof window === 'undefined') {
  const SCOPE = self.registration.scope;
  const PREFIX = 'mabur:' + SCOPE + ':';
  const CACHE = PREFIX + MANIFEST.id;
  const NET_TIMEOUT_MS = 3000;

  self.addEventListener('install', (event) => {
    self.skipWaiting();
    // Per-file, not addAll: one missing file must not abort the whole install.
    event.waitUntil(caches.open(CACHE).then((c) => Promise.allSettled(
      MANIFEST.files.map((f) => c.add(new URL(f, SCOPE).href).catch((e) => console.warn('[sw] precache', f, e))))));
  });

  self.addEventListener('activate', (event) => event.waitUntil((async () => {
    for (const k of await caches.keys()) if (k.startsWith(PREFIX) && k !== CACHE) await caches.delete(k);
    await self.clients.claim();
  })()));

  self.addEventListener('message', (ev) => {
    if (ev.data && ev.data.type === 'coepCredentialless') coepCredentialless = ev.data.value;
  });

  const withCoi = (response) => {
    if (response.status === 0) return response;
    const h = new Headers(response.headers);
    h.set('Cross-Origin-Embedder-Policy', coepCredentialless ? 'credentialless' : 'require-corp');
    if (!coepCredentialless) h.set('Cross-Origin-Resource-Policy', 'cross-origin');
    h.set('Cross-Origin-Opener-Policy', 'same-origin');
    return new Response(response.body, { status: response.status, statusText: response.statusText, headers: h });
  };
  const fetchCoi = (r) => {
    const req = (coepCredentialless && r.mode === 'no-cors') ? new Request(r, { credentials: 'omit' }) : r;
    return fetch(req).then(withCoi);
  };
  const cacheKey = (req) => {
    const u = new URL(req.url); u.search = ''; u.hash = '';
    return u.href === SCOPE ? SCOPE + 'index.html' : u.href;
  };
  const timeout = (ms) => new Promise((_, rej) => setTimeout(() => rej(new Error('timeout')), ms));

  async function networkFirst(req) {
    const cache = await caches.open(CACHE);
    const key = cacheKey(req);
    const net = fetchCoi(req).then((res) => {
      if (res.status === 200) cache.put(key, res.clone());
      return res;
    });
    try {
      return await Promise.race([net, timeout(NET_TIMEOUT_MS)]);
    } catch {
      const hit = await cache.match(key);
      return hit ? withCoi(hit) : net;   // no copy: keep waiting on the network
    }
  }

  self.addEventListener('fetch', (event) => {
    const r = event.request;
    if (r.cache === 'only-if-cached' && r.mode !== 'same-origin') return;
    const mine = r.method === 'GET' && r.url.startsWith(SCOPE);
    event.respondWith(mine ? networkFirst(r) : fetchCoi(r));
  });
} else {
  (() => {
    const n = navigator;
    if (!n.serviceWorker) return;
    if (n.serviceWorker.controller) {
      n.serviceWorker.controller.postMessage({ type: 'coepCredentialless', value: !(window.chrome || window.netscape) });
    }
    const ownUrl = window.document.currentScript.src;
    const ctl = n.serviceWorker.controller;
    if (!shouldRegister({ isolated: window.crossOriginIsolated === true, controllerUrl: ctl && ctl.scriptURL, ownUrl })) return;
    if (!window.isSecureContext) { console.log('[sw] not registered: a secure context is required'); return; }
    n.serviceWorker.register(ownUrl).then((reg) => {
      reg.addEventListener('updatefound', () => {
        const w = reg.installing;
        if (w) w.addEventListener('statechange', () => { if (w.state === 'activated') window.location.reload(); });
      });
      if (reg.active && !n.serviceWorker.controller) window.location.reload();
    }, (err) => console.error('[sw] register failed:', err));
  })();
}
