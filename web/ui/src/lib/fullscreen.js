// Phone fullscreen rules (App.svelte goLandscape / connect).
//
// Android Chrome parks an HTML fullscreen request until its own toolbar has
// hidden, and it never hides the toolbar on a tab whose security level is
// DANGEROUS (chrome/android TabStateBrowserControlsVisibilityDelegate:
// isContentDangerous -> controls SHOWN; FullscreenHtmlApiHandlerBase keeps
// mPendingFullscreenOptions until the controls hide). The https page becomes
// DANGEROUS the moment it constructs the plain ws:// relay socket -- Blink's
// MixedContentChecker::IsWebSocketAllowed lets it through under Local Network
// Access but reports "insecure content ran" (the red "Not secure" in the
// address bar), even when LNA then blocks the connection. So after any relay
// Connect the requestFullscreen() promise neither resolves nor rejects for
// the rest of the document's life. Two consequences handled here:
//   - a phone goes fullscreen BEFORE the relay socket exists (connect()):
//     fullscreen already held outranks the toolbar lock;
//   - a request that has not settled in FS_SETTLE_MS is reported as stuck
//     instead of hanging silently.

export const FS_SETTLE_MS = 1500;

export const FS_STUCK = 'Fullscreen blocked: Chrome keeps its bars once this page has opened the plain ws:// relay '
  + 'socket (address bar shows "Not secure"). Reload the page, then go fullscreen before Connect.';

// Resolves to { ok: true } when p fulfils, { err } when it rejects, or
// { timeout: true } when it has done neither after ms. p's late outcome is
// then ignored (a late fulfil still fires fullscreenchange on its own).
export function settle(p, ms, setTimer = setTimeout, clearTimer = clearTimeout) {
  return new Promise((resolve) => {
    const t = setTimer(() => resolve({ timeout: true }), ms);
    Promise.resolve(p).then(
      () => { clearTimer(t); resolve({ ok: true }); },
      (err) => { clearTimer(t); resolve({ err: err || new Error('refused') }); });
  });
}

// Connect on a phone goes fullscreen first when nothing needs the tap's
// activation afterwards: a relay connect never opens a chooser (and must
// precede the ws:// socket, see above); a USB connect only once the card is
// already granted, since the first-time WebUSB chooser needs the activation
// that fullscreen would consume.
export function fsBeforeConnect({ mobile, radio, usbGranted }) {
  return !!mobile && (radio === 'relay' || !!usbGranted);
}
