// WASM module lifecycle (spec 2026-09-27-web-ui §3.1/§4.1):
// idle -> connecting -> live -> stopping -> idle, plus error. Disconnect is
// in-page: Module._webgs_stop() ends the core loop, main returns and
// Module.onExit fires; Connect then builds a FRESH module. If onExit never
// comes (stopTimeoutMs), the caller-supplied reload() is the recovery path.
//
// onError and onExit can arrive in EITHER order (onError is posted via
// MAIN_THREAD_ASYNC_EM_ASM, onExit via the proxied runtime exit -- see the
// Task 3 review ruling). Every connect attempt gets its own `token` object;
// the module's callbacks are closed over that token and only act while it
// is still `this.token`, so a stale/superseded module's late callbacks are
// inert. Within one attempt: an unexpected onExit (not the result of our
// own disconnect()) puts the session in 'error' immediately with a generic
// fallback message; a later onError always overwrites that error's text
// with the mapped one, whichever callback happened to fire first. Only
// exception: once disconnect() has driven the session cleanly back to
// 'idle', a late onError must not flip it back into 'error'.
import { errorText } from './logic.mjs';

export const WORKER_FAILED = 'Page worker failed — if served over LAN, the TLS CA is not trusted on '
  + 'this device (see docs/web-gs.md).';

export class Session {
  constructor({ createModule, requestDevice, onAu, onStats, onOsd = () => {}, onRecClosed = () => {}, reload,
                // Wrapped: a bare window.setTimeout called as timers.setTimeout(...) throws "Illegal invocation".
                timers = { setTimeout: (f, ms) => setTimeout(f, ms), clearTimeout: (h) => clearTimeout(h) },
                stopTimeoutMs = 3000, recOffTimeoutMs = 1000 }) {
    Object.assign(this, { createModule, requestDevice, onAu, onStats, onOsd, onRecClosed, reload, timers,
      stopTimeoutMs, recOffTimeoutMs });
    this.subs = new Set();
    this.mod = null;
    this.token = null;        // identifies the current connect attempt/module instance
    this.stopRequested = false; // true once disconnect() has asked the current module to stop
    this.lastCore = null;
    this.stopTimer = null;
    this.recWaiter = null;   // resolve fn while waiting for rec_state != 1
    this.snapshot = { state: 'idle', mode: 'spotter', ch: null, w: null, error: null, notice: null,
                      startedAt: null, recWish: false, localWish: false };
  }

  subscribe(fn) { this.subs.add(fn); fn(this.snapshot); return () => this.subs.delete(fn); }
  set(patch) { this.snapshot = { ...this.snapshot, ...patch }; for (const f of this.subs) f(this.snapshot); }

  async connect({ mode, ch, w, overlayToml }) {
    const st = this.snapshot.state;
    if (st !== 'idle' && st !== 'error') return;
    this.lastCore = null;
    const token = {};
    this.token = token;
    this.stopRequested = false;
    this.set({ state: 'connecting', mode, ch, w, error: null, notice: null, recWish: false, localWish: false, startedAt: null });
    try {
      await this.requestDevice();
    } catch (e) {
      if (this.token !== token) return;
      // The chooser closed without a pick (WebUSB rejects with NotFoundError)
      // is not a failure; anything else (not cross-origin isolated, no
      // WebUSB, SecurityError) is, and its own message says why.
      if (e && e.name === 'NotFoundError') {
        this.set({ state: 'idle', notice: 'No device selected — press Connect to try again.' });
      } else {
        this.set({ state: 'error', error: (e && e.message) || String(e) });
      }
      return;
    }
    const args = ['live', '--mode', mode, '--ch', String(ch), '--w', String(w)];
    const overlay = mode === 'gs' && overlayToml ? overlayToml : null;
    if (overlay) args.push('--overlay', '/overlay.toml');
    let mod = null;
    try {
      mod = await this.createModule({
        arguments: args,
        preRun: overlay ? [(m) => m.FS.writeFile('/overlay.toml', overlay)] : [],
        onAu: (...a) => { if (this.token === token) this.onAu(...a); },
        onStats: (text) => { if (this.token === token) this.handleStats(text); },
        onOsd: (rows, cols, cells) => { if (this.token === token) this.onOsd(rows, cols, cells); },
        onError: (text) => { if (this.token === token) this.handleError(text); },
        // Not gated on state: the core seals the file on its way out, so this
        // can land after onExit. The token still drops a superseded module's.
        // The core only ever has one armed recording, and it is sealed now
        // -- clear localWish so a press right after an error starts a new
        // one instead of sending a redundant stop first.
        onRecClosed: (name, bytes, err) => {
          if (this.token !== token) return;
          this.set({ localWish: false });
          this.onRecClosed(name, bytes, err);
        },
        onExit: (code) => { token.exited = true; if (this.token === token) this.handleExit(code); },
        print: (t) => console.log('[webgs]', t),
        printErr: (t) => {
          console.error('[webgs]', t);
          if (this.token === token && String(t).includes('worker sent an error')) this.fail(WORKER_FAILED);
        },
      });
    } catch (e) {
      if (this.token !== token) return;
      console.error('[webgs] module start failed', e);
      this.set({ state: 'error', error: 'ERROR starting module: ' + ((e && e.message) || e) });
      return;
    }
    if (this.token !== token) return;               // superseded by a newer connect()
    if (this.snapshot.state !== 'connecting') {
      // onExit/onError already resolved this attempt -- or fail() did while
      // the module was still starting, which leaves it running: stop it so
      // the next Connect doesn't find the card claimed.
      if (!token.exited) stopQuietly(mod);
      return;
    }
    this.mod = mod;
    this.set({ state: 'live', startedAt: Date.now() });
  }

  handleStats(text) {
    let s;
    try { s = JSON.parse(text); } catch { return; }
    this.lastCore = s;
    if (this.recWaiter && s.rec_state !== 1) { const r = this.recWaiter; this.recWaiter = null; r(); }
    this.onStats(text, s);
  }

  // Always overwrites the error text -- whichever of onExit/onError fired
  // first, the mapped onError text wins once it arrives. The one guard: a
  // late onError after a clean stop (state already back to 'idle') is inert.
  handleError(text) {
    console.error('[webgs] ' + text);
    if (this.snapshot.state === 'idle') return;
    this.set({ state: 'error', error: errorText(text) });
  }

  handleExit(code) {
    if (this.stopTimer) { this.timers.clearTimeout(this.stopTimer); this.stopTimer = null; }
    this.mod = null;
    if (this.stopRequested) {
      this.stopRequested = false;
      this.set({ state: 'idle', recWish: false, localWish: false });
      return;
    }
    const st = this.snapshot.state;
    if (st === 'live' || st === 'connecting' || st === 'stopping') {
      // Unexpected exit. If onError already ran first, keep its mapped
      // text; otherwise fall back to a generic message that a following
      // onError (handleError always overwrites) will replace.
      this.set({ state: 'error', error: this.snapshot.error || `Core exited (code ${code}).`, localWish: false });
    }
  }

  setRec(on) {
    if (this.snapshot.state !== 'live' || this.snapshot.mode !== 'gs' || !this.mod) return;
    this.mod._webgs_set_rec(on ? 1 : 0);
    this.set({ recWish: !!on });
  }

  // Local (OPFS) recording, both modes. null = stop. The core copies the
  // name synchronously, so the heap string is freed right away.
  setLocalRec(name) {
    if (this.snapshot.state !== 'live' || !this.mod) return;
    if (name) {
      const p = this.mod.stringToNewUTF8(name);
      this.mod._webgs_set_local_rec(1, p);
      this.mod._free(p);
    } else {
      this.mod._webgs_set_local_rec(0, 0);
    }
    this.set({ localWish: !!name });
  }

  async disconnect() {
    if (this.snapshot.state !== 'live' || !this.mod) return;
    const mod = this.mod;
    if (this.snapshot.recWish) {
      mod._webgs_set_rec(0);
      this.set({ recWish: false });
      if (this.lastCore && this.lastCore.rec_state === 1) {
        await new Promise((resolve) => {
          const h = this.timers.setTimeout(() => { this.recWaiter = null; resolve(); }, this.recOffTimeoutMs);
          this.recWaiter = () => { this.timers.clearTimeout(h); resolve(); };
        });
      }
    }
    if (this.snapshot.state !== 'live') return;   // died while waiting
    this.set({ state: 'stopping' });
    this.stopRequested = true;
    this.stopTimer = this.timers.setTimeout(() => {
      // Recovery, not the normal path: onExit never came.
      console.error('[webgs] core did not exit after stop; reloading');
      this.reload();
    }, this.stopTimeoutMs);
    mod._webgs_stop();
  }

  // External failure (page worker). A still-running module is stopped,
  // best-effort, so a later Connect doesn't hit "Card busy". stopRequested
  // stays false: its onExit must leave the session in 'error', not 'idle'.
  fail(text) {
    const st = this.snapshot.state;
    if (st === 'idle') return;
    if ((st === 'live' || st === 'connecting') && this.mod) stopQuietly(this.mod);
    this.set({ state: 'error', error: text });
  }
}

function stopQuietly(mod) {
  try { mod._webgs_stop(); } catch (e) { console.error('[webgs] stop after failure threw', e); }
}
