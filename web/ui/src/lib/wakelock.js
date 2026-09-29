// Screen Wake Lock while streaming: the OS would otherwise dim and sleep the
// screen mid-flight (nothing here is "user activity"). The browser drops the
// lock whenever the page is hidden, so onVisible() re-requests it; a request
// still in flight when set(false) lands is released as soon as it resolves.
export class ScreenWake {
  constructor(wakeLock = globalThis.navigator?.wakeLock, isHidden = () => globalThis.document?.hidden) {
    this.api = wakeLock;
    this.isHidden = isHidden;
    this.want = false;
    this.sentinel = null;
    this.pending = false;
  }

  set(want) {
    this.want = want;
    if (want) this.acquire();
    else if (this.sentinel) { const s = this.sentinel; this.sentinel = null; s.release().catch(() => {}); }
  }

  onVisible() { if (this.want) this.acquire(); }

  async acquire() {
    if (!this.api || this.sentinel || this.pending || this.isHidden()) return;
    this.pending = true;
    try {
      const s = await this.api.request('screen');
      if (!this.want) { s.release().catch(() => {}); return; }
      this.sentinel = s;
      s.addEventListener?.('release', () => { if (this.sentinel === s) this.sentinel = null; });
    } catch { /* refused (battery saver, no user activation): the screen just sleeps as before */ }
    finally { this.pending = false; }
  }
}
