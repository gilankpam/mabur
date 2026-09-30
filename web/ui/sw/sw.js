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

// Which fetch strategy a same-scope GET gets (docs/web-gs.md "Serve"):
// hashed build assets are cache-first (a gh-pages deploy prunes old hashed
// files, so racing a fast 404 against the cache must never win); navigations
// get the 3 s network-first timeout; everything else (webgs.js/.wasm,
// font_btfl.png, ...) is network-first with no timeout.
function fetchStrategy({ url, mode, scope }) {
  if (mode === 'navigate') return 'network-timeout';
  if (url.startsWith(scope + 'assets/')) return 'cache-first';
  return 'network';
}

// Fallback rule shared by the no-timeout 'network' strategy: a non-ok
// network response (e.g. a deleted asset's 404) or a network error only
// falls back to the cache when there IS a cached copy; otherwise the network
// result (bad response or error) stands, same as before this fell back at all.
function pickResponse({ netOk, hasCache }) {
  return !netOk && hasCache;
}

if (typeof window === 'undefined') {
  const SCOPE = self.registration.scope;
  const PREFIX = 'mabur:' + SCOPE + ':';
  const CACHE = PREFIX + MANIFEST.id;
  const NET_TIMEOUT_MS = 3000;

  self.addEventListener('install', (event) => {
    self.skipWaiting();
    // Per-file, not addAll: one missing file must not abort the whole install
    // *while it's happening* -- but a partial precache must not become the
    // active worker either (a page later fetching that missing file offline
    // would have nothing to serve). So: warn per file, then if any rejected,
    // throw so the install itself fails -- the previous worker and its
    // complete cache stay in control, and the browser retries this install
    // on a later navigation.
    // cache: 'reload' bypasses the HTTP cache -- fixed-name files (index.html,
    // webgs.js/.wasm, font_btfl.png) are served by GitHub Pages with
    // max-age=600, and a worker installing within that window could otherwise
    // precache a stale index.html against this build's asset hashes, or a
    // mismatched webgs.js/webgs.wasm pair.
    event.waitUntil((async () => {
      const c = await caches.open(CACHE);
      const results = await Promise.allSettled(
        MANIFEST.files.map((f) => c.add(new Request(new URL(f, SCOPE).href, { cache: 'reload' }))));
      results.forEach((r, i) => { if (r.status === 'rejected') console.warn('[sw] precache', MANIFEST.files[i], r.reason); });
      if (results.some((r) => r.status === 'rejected')) throw new Error('sw: precache incomplete, install refused');
    })());
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

  // Hashed build assets: cache-first. A gh-pages deploy deletes the old
  // build's hashed files, so racing a fast 404 against the cache (the old
  // networkFirst did, for everything) can serve a stale index.html whose
  // now-gone assets/index-OLD.js 404s from the network -> blank page.
  async function cacheFirst(req) {
    const cache = await caches.open(CACHE);
    const key = cacheKey(req);
    const hit = await cache.match(key);
    if (hit) return withCoi(hit);
    const res = await fetchCoi(req);
    if (res.status === 200) cache.put(key, res.clone()).catch(() => {});
    return res;
  }

  // Navigations only: network first, 3 s timeout, cached copy on timeout/error.
  async function networkTimeout(req) {
    const cache = await caches.open(CACHE);
    const key = cacheKey(req);
    const net = fetchCoi(req).then((res) => {
      if (res.status === 200) cache.put(key, res.clone()).catch(() => {});
      return res;
    });
    try {
      return await Promise.race([net, timeout(NET_TIMEOUT_MS)]);
    } catch {
      const hit = await cache.match(key);
      return hit ? withCoi(hit) : net;   // no copy: keep waiting on the network
    }
  }

  // Everything else (webgs.js/.wasm, font_btfl.png, ...): network first, no
  // timeout; fall back to the cache on a network error OR a non-ok response
  // (e.g. a mismatched webgs.wasm 404ing against a fresher webgs.js) when a
  // cached copy exists; otherwise the network result stands.
  async function networkOnly(req) {
    const cache = await caches.open(CACHE);
    const key = cacheKey(req);
    const net = fetchCoi(req).then((res) => {
      if (res.status === 200) cache.put(key, res.clone()).catch(() => {});
      return res;
    });
    let res;
    try {
      res = await net;
    } catch {
      const hit = await cache.match(key);
      return pickResponse({ netOk: false, hasCache: !!hit }) ? withCoi(hit) : net;
    }
    const hit = res.ok ? null : await cache.match(key);
    return pickResponse({ netOk: res.ok, hasCache: !!hit }) ? withCoi(hit) : res;
  }

  self.addEventListener('fetch', (event) => {
    const r = event.request;
    if (r.cache === 'only-if-cached' && r.mode !== 'same-origin') return;
    const mine = r.method === 'GET' && r.url.startsWith(SCOPE);
    if (!mine) { event.respondWith(fetchCoi(r)); return; }
    const strategy = fetchStrategy({ url: r.url, mode: r.mode, scope: SCOPE });
    event.respondWith(
      strategy === 'cache-first' ? cacheFirst(r) :
      strategy === 'network-timeout' ? networkTimeout(r) :
      networkOnly(r));
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
