# Web GS — mabur ground station in the browser

`web/` is a browser page (WebUSB + WebAssembly) that runs the same receive
and ladder-control core as `maburgs`, on one RTL8812EU or RTL8812AU card, with no
native daemon involved. It supersedes the throwaway `spike/wasm/` (deleted on this
branch; branch `wasm-spike`, d54a9b2) — the spike answered "can devourer run
in Chrome over WebUSB and keep up with live video" (yes), this is that
answer turned into a real feature. Design: PR that added this page (see the
git log for `web/`); the working spec lived at
`docs/superpowers/specs/2026-09-27-web-gs-design.md`, which is gitignored
and machine-local, so it is not linked here — this page is meant to stand on
its own.

## What it is

A single page, one card, a fixed channel — no channel scan, no in-flight
hop, no DVR/recording, no multi-card. Two modes, picked before
Connect:

- **GS mode** — the *only* ground station for the drone. Runs rendezvous
  (DISC until DISC_ACK), the same measured-loss ladder as `maburgs`
  (probe-before-promote, fade, failsafe), and slots its RCFs into the
  drone's inter-AU idle exactly like `RcfSlotter`. It proposes its own
  channel on DISC, so the drone never moves to chase it.
- **Spotter mode** — receive, decode, display; **never transmits**. Built
  to sit next to a real `maburgs` that is already flying the link. There is
  no send path in this mode by construction (`web/src/web_gs.h`): the
  constructor drops the `Io::send` callback and neither `VrxController`
  nor `RcfSlotter` exists, so there is no code path that can build a
  control frame.

Only one GS may command a given drone. The mode picker's own text says so
("GS mode commands the drone — only one GS per drone. Use Spotter next to a
running ground station.") — the page cannot detect a second commander and
does not try to (see Known limits).

## Build

Host build + tests (the WebGs core, `webgs` CLI, host ctest targets):

```sh
nix-shell -p pkg-config libusb1 --run "cmake -S . -B build -DDEVOURER_DIR=$PWD/../devourer && cmake --build build -j && ctest --test-dir build -R 'test_web_gs|web_au_parity'"
```

libusb for WASM (once; builds upstream libusb with the Emscripten WebUSB
backend into a repo-local `EM_CACHE` sysroot — `web/tools/build-libusb-wasm.sh`):

```sh
nix-shell -p emscripten autoconf automake libtool pkg-config --run web/tools/build-libusb-wasm.sh
```

The WASM page build (`webgs.js`/`webgs.wasm` into `web/dist/`, `www/` copied
alongside, plus the `webgs_node` replay build used by the parity gate):

```sh
nix-shell -p emscripten cmake pkg-config python3 --run "export EM_CACHE=$PWD/web/.deps/emcache; emcmake cmake -S web -B web/build-wasm -DCMAKE_BUILD_TYPE=Release && cmake --build web/build-wasm -j"
```

WASM/native parity (`web/tests/test_wasm_parity.sh`; needs both the host
`build/web/webgs` and `web/build-wasm/webgs_node.js` already built):

```sh
nix-shell -p nodejs python3 --run "BUILD=$PWD/build web/tests/test_wasm_parity.sh"
```

The page UI itself (`web/ui/`, Vite + Svelte 5, lockfile committed) is a
separate build step folded into the same WASM CMake invocation above: after
the Emscripten module is built, the standalone `web/` CMakeLists also runs
`npm ci` (when the lockfile changed) and `npm run build` (`vite build`) in
`web/ui`, writing the page's JS/CSS bundle into `web/dist` next to
`webgs.js`/`webgs.wasm`. It needs `nodejs` in the shell alongside
`emscripten`:

```sh
nix-shell -p emscripten cmake pkg-config python3 nodejs --run "export EM_CACHE=$PWD/web/.deps/emcache; emcmake cmake -S web -B web/build-wasm -DCMAKE_BUILD_TYPE=Release && cmake --build web/build-wasm -j"
```

For UI-only iteration, `npm run dev` in `web/ui` (inside `nix-shell -p
nodejs`) runs Vite's dev server, serving `webgs.js`/`webgs.wasm` straight out
of `web/dist` (build that once via the CMake step above) with the same
COOP/COEP headers the static server sets — no rebuild-the-WASM-module round
trip for a page-only change.

Page logic tests (session lifecycle, config form, rec state, hitch/stat
math, segment alignment, layout) — pass the files explicitly, a bare
directory argument fails on this machine's Node 22:

```sh
nix-shell -p nodejs --run "node --test web/tests/*.test.mjs"
```

## Serve

**Hosted: <https://gilankpam.github.io/mabur/>** — the simplest way in, and
the only one that works on a phone without extra setup. The `Pages`
workflow (`.github/workflows/pages.yml`) builds the WASM core and the page
on every push to `web-gs` or `master` that touches `web/`, `gs/src/`,
`gs/bundle/` or `common/` (or by hand: Actions → Pages → Run workflow) and
deploys `web/dist` as-is. It is real HTTPS on a public certificate, so no
local CA is needed on any device. GitHub Pages cannot send the COOP/COEP
headers WASM pthreads need, so `web/ui/public/coi-serviceworker.js`
(vendored coi-serviceworker v0.1.7, MIT) installs a service worker that
adds them: the very first visit reloads the page once, after which it is
cross-origin isolated. The WebUSB grant is per origin, so each device picks
the card once on this site. The page is the bundle-default ladder build,
same as a local one.

`web/serve.py` is a static server for `web/dist` that sets the COOP/COEP
headers WASM pthreads need (`Cross-Origin-Opener-Policy: same-origin`,
`Cross-Origin-Embedder-Policy: require-corp`).

**Localhost** (Chrome only — see Known limits for why): `python3
web/serve.py 8808`, then open `http://127.0.0.1:8808/`. Plain HTTP is a
secure context only on `localhost`, which is enough for a card plugged into
the same machine running Chrome.

**LAN (a phone, or any browser that isn't on the serving machine)** needs
HTTPS: WebUSB and `SharedArrayBuffer` both require a secure context, and a
click-through on a bare self-signed cert is **not** enough — the bypass
covers the tab, but Emscripten's pthread pool is workers spawned from a
worker, and those workers' script fetches still fail the certificate check
("worker sent an error! undefined:undefined: undefined" — `app.js` maps
that to the "Page worker failed" banner). The certificate has to be
genuinely trusted:

```sh
nix-shell -p openssl --run ./web/mkcert.sh   # local CA (once, reused) + server cert
python3 web/serve.py 8808 0.0.0.0 --tls
```

`mkcert.sh` writes `web/tls/` (gitignored): a local CA, name-constrained to
`localhost` and the private IPv4 ranges (so trusting it can never vouch for
a public site), and a server cert signed by it covering `localhost` plus
every IPv4 this host currently has. Re-running it reuses an existing
`tls/ca.key` rather than minting a new CA. **On this machine the CA already
sitting in `web/tls/` is the one from the `spike/wasm/` days, moved here
verbatim** — it is already trusted in this host's Chrome NSS database under
the nickname `mabur-spike-ca` (not `mabur-webgs-ca`; that is only the name
`mkcert.sh`'s own comment suggests for a *new* install). Install the CA on
each viewing device before opening the page there:

- **Android**: Settings > Security > Encryption & credentials > Install a
  certificate > CA certificate, pick `web/tls/ca.crt`.
- **Linux Chrome**: `certutil -d sql:$HOME/.pki/nssdb -A -t "C,," -n
  mabur-webgs-ca -i web/tls/ca.crt` (use a fresh nickname for a device that
  has never trusted it before; `nix-shell -p nssTools` if `certutil` is
  missing; remove with `certutil -d sql:$HOME/.pki/nssdb -D -n
  mabur-webgs-ca`), then restart Chrome.

Then open `https://<host-ip>:8808/`. WebUSB's device grant is per browser
origin, so the card chooser asks again on the new `https://` origin even if
`http://127.0.0.1` already had a grant.

**NixOS firewall**, if the LAN device can't reach the port:
`sudo iptables -I nixos-fw -p tcp --dport 8808 -j nixos-fw-accept` (lost on
firewall reload — not persistent).

## Use

Pick a mode, channel and width, press Connect. The page defaults to
**Spotter** mode (URL `?mode=` or the last-used choice override it) — a
deliberate safety default, although the design spec named `gs`: opening
the page must never start commanding a drone that a `maburgs` or another
tab may already be flying; GS mode is an explicit opt-in.

Channel/width are checked before the device is requested (an integer in
1–200, width 20 or 40, 40 only on a standard HT40 pair), and the core
re-checks them with `maburgs`'s own config-loader checks
(`maburgs::radio_width_issue` / `link_width_issue`, `gs/src/config.cpp`,
via `webgs::channel_width_error`): in GS mode a 20 MHz tuning is refused
while the ladder has any 40 MHz rung, because the GS could not receive
it. A refusal reads `Channel/width refused: <reason>` (core line
`ERROR bad channel/width: <reason>`).

This page's ladder starts from whatever `web/CMakeLists.txt` embeds into the
WASM module at build time — the bundle default,
`gs/bundle/maburgs.default.toml` (5 rungs, all 40 MHz, top = rung 4
mcs4/40) — not a GS's own tuned `/etc/maburgs.toml`; the Config form (below)
overrides it per session. A GS box on the same drone may run a different
(e.g. mcs-wider or narrower) ladder; don't read this page's rung numbers as
if they were that GS's unless the form was left at its defaults.

**Connect/Disconnect are in-page** — no reload to change mode, channel,
width or (GS mode) the ladder, and no reload between flights.
`Session` (`web/ui/src/lib/session.js`) drives a small state machine
(`idle → connecting → live → stopping → idle`, plus `error`): Connect
requests the device, builds a **fresh** WASM module every time (never
reused across connects) with `--mode`/`--ch`/`--w` and, in GS mode always
(even when the form matches the embedded default), `--overlay
/overlay.toml` (the form's overlay TOML written into the module's virtual
FS before `main()` runs).
Disconnect calls the exported `webgs_stop()`, which sets an atomic the core
loop polls; the core tears down cleanly and returns from `main()`, and
`-sEXIT_RUNTIME` fires `Module.onExit()`, which the page treats as the
signal that the module is gone and the UI can go back to `idle` (or
straight into a new Connect). If `onExit` doesn't arrive within 3 s
(`stopTimeoutMs`), the page treats that as the module being stuck rather
than waiting indefinitely: it reloads the page as a fallback, landing back
on the Config tab. Recording is stopped first if it was on (see Record,
below) before the stop request goes out.

WebUSB has no transfer cancel (libusb's emscripten backend `cancel` is a
no-op), so on a quiet channel — drone off, nothing on air — the RX loop's
pending `transferIn` calls would never complete and the core could not
exit. The teardown waits 300 ms for the RX loop to end on its own; if it
hasn't, it releases interface 0, which makes Chrome abort those reads
(they surface in the console as `NetworkError: ... A transfer error has
occurred`), joins the RX thread, re-claims the interface for the chip's
power-down writes, and continues. The core prints one line per stop:
`teardown: rx <ms> [(reads aborted)], chip stop <ms>, close <ms>`. A stop
requested while the card is still initialising (~5 s after Connect) takes
effect only once init finishes.

The browser's own device
chooser asks for the card — an RTL8812EU (Jaguar3) or an RTL8812AU
(Jaguar1); the WASM core builds both devourer drivers (WebUSB's per-origin
device grant — pick it once and later `getDevices()` calls see it without asking again on the same
origin). What the status overlay shows after that:

- `Starting…` / `Requesting device…` — module bring-up, before the core
  loop is up.
- `No drone on ch N / W MHz (still trying)` — GS mode, no DISC_ACK within
  10 s of module start; it keeps sending DISC and the message keeps
  counting, it does not give up.
- `Waiting for key frame (sent on the next rung change) — N s` — the video
  gate is NOT armed yet: it arms only on a complete AU carrying
  VPS+SPS+PPS plus an IRAP, and none has arrived. See "Spotter key frames"
  below for why this can run indefinitely.
- `GS mode keeps flying the link while this tab is hidden.` — a one-time
  banner (GS mode only) confirming the ladder and RCF sends do not pause
  when the tab loses visibility; the worker keeps running headless.
- `Card busy — maburgs or another tab has it. Close that and press
  Connect.` / `No RTL8812EU/8812AU card found — plug it in and press
  Connect.` / `Card lost (unplugged?). Press Connect to restart.` /
  `WebUSB unavailable in this browser.` / `Unsupported card chip.` — mapped
  from the core's `ERROR <reason>` lines (`web/ui/src/lib/logic.mjs`'s
  `errorText()`); Connect re-enables so trying again costs nothing.
- `Page worker failed — if served over LAN, the TLS CA is not trusted on
  this device (see docs/web-gs.md).` — the untrusted-cert failure mode
  above; that is this file.

**Spotter key frames.** WebCodecs will only start decoding on a real IRAP
(IRAP NAL types 16–21: BLA/IDR/CRA), never on the drone's GDR parameter-set refresh
(`32 33 34 1`, TRAIL_R). The drone only emits a real IDR on a rung change
or link-up, so a spotter opened mid-flight on a steady link can sit on
"Waiting for key frame" until the ladder next moves — there is no drone,
wire or config change to force one (see Follow-ups). GS mode does not have
this problem: linking a previously-unlinked drone changes its bitrate
(max-range floor → rung), and that `SetChnAttr` costs an IDR on its own, so
a self-linking GS's own link-up produces the first key frame roughly a
second after DISC_ACK.

## Config form

The Config tab (`web/ui/src/components/ConfigPanel.svelte`, and
`ConfigSide.svelte` for the immersive/mobile layout) edits channel and width
in both modes, plus — GS mode only — Fixed MCS (`static_mcs`, −1 =
adaptive), Max MCS, and the ladder rungs (mcs/bw/FEC overhead per rung, add/
remove up to 8). `web/ui/src/lib/config.js` normalizes and persists the
form to `localStorage` under `webgs.cfg` (per-field fallback to the bundle
default, so a stale or hand-edited entry can never break the page), and
`?ch=`/`?w=` query params override the saved channel/width on load the same
way `?mode=` overrides the saved mode. On Connect, the GS-mode form (always, even
when it matches the embedded default) is serialized to TOML
(`toOverlayToml()`: `[link] static_mcs/static_bw/max_mcs` plus one
`[[link.ladder]]` block per rung, `static_bw` carrying the form's width) and
handed to the core as `--overlay /overlay.toml`; `maburgs::load_config`'s
`overlay_path` argument (`gs/src/config.h`/`.cpp`) deep-merges it over the
embedded `maburgs.default.toml` (an overlay `[[link.ladder]]` replaces the
file's ladder whole, it does not append) and re-runs the same validation
the native loader runs. A rejected overlay surfaces as `ERROR bad config:
<reason>` from the core; `connectBlocker()` mirrors the same checks
page-side (channel/width, FEC overhead range 0.1–2.0, enh ≤ base, at least
one rung at-or-below Max MCS, no 40 MHz rung under a 20 MHz width) so a bad
form is refused before Connect ever starts the device request, and a
per-field warning (`rungWarnings()`/`channelWarning()`) flags a rung that's
above Max MCS or a channel with no HT40 pair without blocking the rest of
the form. Spotter mode sends no overlay — it only listens, so only channel/
width apply.

## Record

VTX recorder control only — there is no browser-side recording and no DVR
target in this build (see Known limits). GS mode's Record button calls
`Session.setRec()`, which calls the exported `webgs_set_rec(1|0)`; the core
(`WebGs::set_vtx_rec`, `web/src/web_gs.h`/`.cpp`) sets the RCF's recorder
wish byte the same way `maburgs` does (`kRecKnown | kRecOn`), so the drone's
onboard VTX SD recorder starts/stops exactly as it would under a native GS
(`docs/vtx-recorder.md`). The indicator (`RecDot.svelte`, both modes)
follows the drone's own report, not the click: it reads `Telem.rec_status`
back off the stats stream (core `rec_state`/`rec_err` keys, `rec.js`'s
`recView()`) and only shows "recording" once the drone confirms it, with
the on-screen clock (`RecClock`) starting from that first confirmation, not
button-press time. Spotter mode has no send path, so it mirrors the same
`rec_state`/`rec_err` read-only — it can watch a `maburgs`-started
recording (as in the Validation section's spotter run) but cannot start or
stop one. Disconnect in GS mode with the wish still on sends the "off" RCF
and waits up to 1 s (`recOffTimeoutMs`) for the drone to confirm
`rec_state != 1` before stopping the module, so a Disconnect doesn't leave
the VTX recording with nothing left to turn it off.

## Stats panel

A 1 Hz snapshot from the core (`webgs::Stats` / `stats_json()` in
`web/src/web_gs.cpp`), plus page-side timing JS adds on receipt, including
the recorder's `rec_state`/`rec_err` (Record, above). In the windowed
layout this lives in the sidebar's Stats/Config/Debug tabs
(`Sidebar.svelte`); in immersive/fullscreen or on mobile it's a floating,
draggable panel (`FloatStats.svelte`, `web/ui/src/lib/layout.js`) instead,
so it stays visible over the fullscreen video. Every page-side segment row
shows p50/p99/max over both a 1 s and a 60 s window (USB lateness is the
core's own 1 s window). Page-side video metrics
(`web/ui/src/lib/metrics.js`'s `PageMetrics`, distinct from the core's 1 Hz
stats) resample every 200 ms over a trailing 1 s window: bitrate (AU bytes),
fps (draw count), and jitter (mean deviation from the nominal draw period);
latency there is capture→glass p50 in GS mode, and the rx→glass segment sum
in Spotter mode (no capture-side clock to measure capture→glass from
without the RTT estimator's offset — see Capture→glass below). "Copy
stats" copies `{core: [last 60 s of 1 Hz stats lines], segments: {w1, w60}}`
as JSON. Segments (spec §4):

| Segment | What it measures | Mode |
|---|---|---|
| Control RTT (EWMA / min) | `RttEstimator`: RCF seq echoed back in `Telem`, clock-sync-free | GS |
| USB lateness p99/max | host arrival time − chip TSF, above the window's own minimum (`UsbLate` in `web_main.cpp`) | both |
| FEC/assembly | first body seen for the AU → AU complete, both timestamps on the core clock (`AuLatMeta`, `gs/src/frame_stream.cpp`) | both |
| Hand-off | AU complete (core/worker thread) → page receives it, both sides aligned to `performance.timeOrigin` | both |
| Decode | `VideoDecoder` chunk submit → decoded output callback | both |
| Present | decoded output → the first `requestAnimationFrame` after `drawImage` — an **estimate**, not `requestVideoFrameCallback`'s presentation time, because MSE/DRM-level present timestamps are not exposed to a WebCodecs canvas path; a callback that fires >1000 ms after its `drawImage` (a hidden tab suspends rAF, so the callback only runs once it's visible again — ~18 s outliers seen on the bench) is dropped rather than counted (`isStalePresentSample()`, `web/ui/src/lib/logic.mjs`) | both |
| Capture→glass (estimate) | drone `pts` + `RttEstimator`'s min-RTT-filtered `pts_off_us` → page present, same method as `maburplay`'s `lat.log`, but computed GS-side; shares Present's rAF path, so the same >1000 ms stale-sample drop applies (`isStalePresentSample()`) | GS only |

Capture→glass is GS-only because it needs the RTT estimator's pts-clock
offset, which only exists when this page is itself exchanging RCF/Telem
with the drone — a spotter has no control channel to measure it over, and
the panel says so rather than showing a stale or borrowed number.

Link state shown alongside: rung/MCS/width, probe gate state, pre-FEC loss,
residual, SNR/RSSI, RCF heard % (Δ`Telem.rcf_rx` / Δ`rcf_sent` over a
trailing ~10 s window of the stats ring, not the adjacent 1 s tick —
`rcfHeardPctWindowed()` in `web/ui/src/lib/logic.mjs`; the two counters are
sampled at different instants, so diffing consecutive ticks read
100–105 % on the bench, and clamping would have hidden a real fault
instead of fixing the measurement. `rcf_sent` counts RCFs only, not the
DISC beacons/keep-alives that `sends` also counts, because the drone's
`rcf_rx` counts RCFs only; dividing by `sends` capped the ratio below the
95 % pass mark), AUs/s, and hitches (gaps
> 1.5× the frame period). No playout regulator and no vsync lock exist in
this build — the panel is measurement-only, by design (see the spec's
"Decided" list).

The core stats line also carries `osd_snaps` (MSP snapshots out of the FEC
sink) and `osd_screens` (OSD screens handed to the page); see MSP OSD below.

## MSP OSD

The flight controller's MSP DisplayPort OSD is drawn over the video in both
modes (it is receive-only, so Spotter's no-send guarantee is untouched, and
it does not wait for the GS session). The path is `maburgs`'s: `WebGs` feeds
stream-id 4 bodies to the same `maburgs::MspSink` (SBI unpack + sliding-
window FEC, `symbol_size`/`window` from the embedded config's `[msp]`, which
must match the drone; `[msp] enable = false` turns it off, `[msp.out]` is
ignored — there is no UDP here). `webgs::OsdScreen`
(`web/src/osd_screen.h`) runs each snapshot through the shared
`MspParser`/`MspScreen` and publishes a completed screen at most every 30 ms,
latest wins — `OsdSource`'s gate. The glue posts it as
`Module.onOsd(rows, cols, Uint16Array)`.

The page (`web/ui/src/lib/osd.js`) spreads the grid over the video box,
which is 16:9 in every layout — immersive (fullscreen, phones) letterboxes
the video instead of cropping it, so the OSD sits exactly on the picture and
an HD grid's (53x20) cells keep the font's 2:3 shape; an SD grid is widened
to 16:9 as on `maburplay`. It paints on a second canvas over the video, once per animation
frame at most, and blanks after 5 s without a screen (`maburplay`'s
`stale_ms` default) and on Disconnect. There is no toggle — trim elements in
the Betaflight OSD tab.

Font: Betaflight only, `web/ui/public/font_btfl.png` (32 glyphs per row,
36×54), generated from `maburplay`'s `gs/player/bundle/font_btfl.mfont` by
`tools/msp/gen_webfont.py` (re-run it if the `.mfont` changes; it
self-checks the round trip). The page fetches it on the first screen; a
failed fetch logs once to the console and leaves the OSD off for the rest of
that session (it is not refetched per screen), and the next Connect tries
once more. The 5 s blank counts from the last *published* screen, where
`maburplay` counts from the last datagram — the same thing in practice,
since Betaflight sends DRAW_SCREEN continuously.

## Colour correction

The drone flies the colortrans sensor file, which flattens the picture on
purpose; `maburplay` undoes it in the CRTC LUT (`docs/colortrans.md`). The
page does the same on the video, **on by default**, with an On/Off toggle
in the Config form's Display group. It is page-only (never sent to the
core, persisted in `webgs.cfg` as `colortrans`) and stays editable while
live — it takes effect on the next frame.

On: `VideoPipeline.draw()` (`web/ui/src/lib/video.js`) hands each decoded
`VideoFrame` to `ColorTransGl` (`web/ui/src/lib/colortrans.js`), which
uploads it with `texImage2D` and runs `frame_colortrans.cpp`'s shader over
one full-screen quad on a WebGL canvas. Off: the pre-existing 2D
`drawImage` on its own canvas — a canvas cannot change context type, so
the two are stacked in `App.svelte` and whichever drew the last frame is
the visible one; turning it off costs exactly what the page cost before.
The MSP OSD canvas sits above both, so no OSD pre-inversion is needed.
No WebGL, a shader that fails to build, or a lost context falls back to
flat for the rest of the page's life (one console line). The Stats
panel's Video line says which path is live: `· colortrans`,
`· colortrans unavailable` (wanted but fell back), or nothing (off).

Checked in headless Chromium (SwiftShader) against `ctForward`: 8 sample
pixels identical, frame the right way up. The constants are pinned by
`web/tests/colortrans.test.mjs` (see `docs/colortrans.md` for the retune
rule).

## Known limits

- **RTL8812AU on a Linux host: unload `rtw88_8812au` first.** The kernel
  has an in-tree driver for the AU (the EU has none). devourer's
  `claim_interface_then_reset` resets the port after claiming, the kernel
  re-probes the interface, and `rtw88_8812au` takes it back mid-bring-up;
  Chrome cannot detach a kernel driver, so every transfer after init fails
  and the page reports `card lost` (an unbound card fails earlier, at
  `claimInterface`, once a re-enumeration has rebound it). Native `webgs`
  / `maburgs` don't see this — libusb detaches the kernel driver itself.
  Fix: `sudo modprobe -r rtw88_8812au` (holds until a replug or reboot; a
  reset keeps the device number, so nothing reloads it), or blacklist it
  (NixOS: `boot.blacklistedKernelModules = [ "rtw88_8812au" ];`). Bench
  2026-09-28: with the module unloaded, GS mode on an 8812AU (C-cut 2T2R,
  USB 3) connects, flies the ladder and plays video.
- **Oilpan GC spikes.** Blink's incremental GC sweep of per-transfer WebUSB
  objects (`cppgc::Sweeper::IncrementalSweepTask`) runs on the thread that
  owns those objects and, after a major GC roughly every 8–13 s, costs one
  periodic spike of 8–14 ms — measured at both 38 and 54 Mb/s on air
  (native `maburgs`'s equivalent tail: ≤1.7 ms). Moving the runtime into a
  dedicated worker moves the sweep with it — the spike is unchanged. The
  only lever found is fewer USB transfers (chip-side RX aggregation), which
  is a latency trade, not taken here.
- **Spotter can never force an IDR.** No GS-relayed IDR request path exists
  (see "Spotter key frames" above and Follow-ups).
- **Spotter cannot gate on `CAP_FRAME_WIRE`.** Spotter mode has no
  rendezvous or caps exchange of its own — `web/src/web_gs.h`'s Spotter
  gate is always true, unlike GS mode, which only decodes while in
  SESSION with a peer that has advertised `CAP_FRAME_WIRE`. So a spotter
  cannot detect a caps-mismatched drone (a half-deployed `RC_VERSION`
  flag-day pair, `docs/deploy.md`): instead of refusing the stream the way
  a caps-aware GS would, it feeds the decoder whatever bitstream arrives,
  parseable or not.
- **Chrome must run via the Nix wrapper** (`bin/google-chrome-stable`), not
  the raw `share/google/chrome/chrome` binary — the raw binary has no
  libEGL, its GPU process dies, and WebCodecs then rejects the drone's HEVC
  stream outright ("Unsupported configuration").
- **Two GS-mode pages against one drone fight.** Nothing detects a second
  commander; both DISC and both send RCFs, and the ladder each one drives
  fights the other's. Spotter mode has no such conflict since it never
  transmits.
- **Browser-side recording not built** (DVR target hidden). This page only
  ever controls the VTX's onboard SD recorder (see Record, above); there is
  no local capture of the decoded stream, and so no recorded OSD either.
- **Capture→glass can use a stale RTT offset for up to ~30 s after a drone
  restart.** `RttEstimator`'s pts-clock offset is never explicitly reset on
  a detected drone restart, in this page or in `maburgs` itself — the
  estimator re-converges from live samples over its own window rather than
  snapping to a fresh baseline, so the number briefly reflects the previous
  boot's clock relationship.
- **`drone_state` renders as a raw number**, not a decoded state name — the
  panel does not carry the enum-to-text mapping `maburtop.py` has.

## Design

The ladder's per-tick input is `LinkHealthAssembler`
(`gs/src/link_health.h`/`.cpp`), extracted verbatim out of `maburgs`'s
`main.cpp` control loop so both front ends compute the SAME windows
(`s1_loss`, `s3_loss`, residuals, probe tracking, RF-label staleness) the
same way — the point being that the web GS's ladder behaves exactly like
`maburgs`'s, not merely similarly. `maburgs` still owns log writing (ctl/
fec/probe logs, the sideport) and everything the assembler does not need to
decide the ladder itself: hop verdict, scan/hop/scout, TX selector, cal
radio silence. `web/src/web_gs.h`'s `WebGs` composes the assembler with the
same `Aggregator`, `FrameStream`, `VrxController`, `RcfSlotter` and
`RttEstimator` maburgs uses, just with one card and no devourer types
reaching the core (`web/CMakeLists.txt`'s `webgs_core` links only the
receive + control subset of `mabur_common` — never `fec_worker` /
`sw_encoder` / `uep_encoder`, which call `pthread_setaffinity_np` and do not
build under Emscripten). The dot11 frame-building/parsing pure functions
(`build_control_frame`, foreign-SA drop, CRC-fail pass-through) live in
`gs/src/dot11.{h,cpp}`, shared with `RadioFrontend` the same way.
`web/src/web_main.cpp` is the glue: libusb (WebUSB backend under
Emscripten, native libusb1 on the host) for the card, a devourer RX loop
into a `BodyQueue`, and the core loop driving `WebGs`; it builds as a
native CLI (`webgs live`/`webgs replay`, used for bench A/B against the
browser and for the parity gates) and, under `emcmake`, as the page's WASM
module.

## Validation

Bench, 2026-09-27, bench drone on ch136 HT40, drone `low_power` off:

- **maburgs regression** (this page's changes don't regress the native
  daemon — same `LinkHealthAssembler`): two A/B pairs (master vs. this
  branch's `maburgs`, second pair run in reversed order), each leg fed two
  15 s `benchjam` episodes (power 63, 2000 pps, 1500 B, 6M) from a host
  8822EU. Identical demote reasons, ~1 s cascade to rung 0, probed
  re-climb 0→5 in 7–9 s in every leg; rung-5 `s3_residual` bounce rate 1.21
  vs. 1.13/min in the reversed pair (the *first* pair's 2.6 vs. 1.7/min
  tracked the time slot, not the binary — not attributable to this
  branch). `ausniff` 30 s clean in every leg (~1815 AUs, 60.5 fps, ≤3
  incomplete enh).
- **Web GS mode, GS daemon stopped**: linked (DISC→ACK→SESSION), climbed
  to the bundle ladder's top (rung 4, mcs4/40). Hand-covering the host
  card's antenna (RSSI −83..−86 dBm, SNR 9–13 dB) demoted to rung 1–2 and
  it recovered via clean probes in ~15–20 s; after a 30 s total outage it
  re-linked by itself. txfail 0. capture→glass p50 ~24 ms, p99 ~45 ms;
  link RTT ~9 ms (min 3–4 ms); FEC/assembly p50 ~6.6 ms; decode ~0.8 ms;
  hand-off 0.1 ms.
- **Spotter mode next to a running `maburgs`**: browser `sends` 0 /
  `rcf_sent` 0 / `txfail` 0 (Spotter never transmits, confirmed on the
  wire, not just by code inspection) — drone RCF rx rate 18.7/s over 30 s
  matched `maburgs`'s own sideport `drone.rcf.rx_pps` 18.5 (a transmitting
  browser would roughly double it). Video started after an 11 s wait for
  a natural IDR (see "Spotter key frames" above), then ran at 61 AUs/s.
- **Seen but not fixed**: 2 WebCodecs `EncodingError` decoder resets in
  ~10 min (each recovered on the next IRAP — the existing `gate.armed =
  false` / re-arm path, `onDecoderError()`). Left as a follow-up; no
  drone/wire signature was found in the ~10 min sample to attribute it to.

**Redesigned page, bench 2026-09-27** (Chrome 147 driven over DevTools,
bench drone on ch136 HT40, GS box off for GS mode):

- **Connect → Disconnect → Connect ×10** (GS mode, drone on): 10/10, no
  reload fallback; Disconnect ~200 ms (`teardown: rx 10–20 ms, chip stop
  45–100 ms`), relink ~6.2 s (dominated by the card's ~5 s init), video
  back at once (link-up IDR).
- **Disconnect with the drone powered off**: before the interface-release
  abort (above) the core never exited and the 3 s reload fallback fired
  every time (recovery itself worked: page back on Config, next Connect
  claimed the card, "No drone on ch 136 / 40 MHz (still trying)" after
  10 s). With the abort: 3/3 in-page, click → Disconnected 584 ms
  (`rx 305 ms (reads aborted), chip stop 79 ms`), next Connect claims the
  card; drone-on cycles unchanged afterwards (5/5, 150–200 ms).
- **Config form**: Max MCS 3 → 4 rungs, link tops out at MCS 3; Fixed
  MCS 2 → `Pinned`, steady MCS 2 at 40 MHz.
- **VTX record** round trip: Record reached the drone and its `Telem`
  answer came back — this drone has no SD card, so the page showed
  `REC!` with "No SD card" (windowed button title and immersive tag). A
  recording that actually runs was not exercised.
- **Latency** (60 s steady, rung 4 mcs4/40): capture→glass p50 23 ms /
  p99 34 ms, FEC p99 11–13 ms, decode p99 1.0 ms, hand-off 0.1–0.2 ms,
  60 fps, jitter 2.6 ms — within noise of (or better than) the numbers
  above.
- **Layouts**: windowed, F → fullscreen immersive, config side panel,
  844×390 phone emulation (236 px float panel; its fullscreen button
  toggles the real Fullscreen API — see below),
  390×844 portrait notice; float-panel drag follows the pointer and clamps
  to x = 844 − 236 − 8, y = 390 − 48; video kept playing across every
  layout switch (one canvas).
- **Spotter next to `maburgs`**: `sends 0 / rcf_sent 0 / txfail 0`,
  60 AUs/s, Record disabled with its reason, Disconnect 200 ms.
- Seen, not attributed to this page: 4 TX `bulk_send` timeouts (rc −7) in
  ~5 min of GS mode, each followed by one decoder reset; one drone boot
  came up with a deaf radio (no RX on the host card natively either) until
  the drone was power-cycled.

On a phone the layout is always the immersive one, but the browser's own
bars stay until the page asks for real fullscreen: the fifth round button
(corners-out / corners-in) calls `requestFullscreen()` and then tries
`screen.orientation.lock('landscape')` (Android Chrome allows the lock only
while fullscreen), and follows the actual `fullscreenchange` state. It is
hidden where the Fullscreen API isn't available (`document.fullscreenEnabled`
false — iPhone Safari).

Not yet exercised: a real phone over LAN TLS (layouts were checked in
Chrome's device emulation only), and the spotter mirroring a
`maburgs`-started recording (the GS's record wish was reset to off right
after `vtx_rec on` on the bench).

## Follow-ups

Not built here, all noted in the spec as later work:

- A periodic or GS-relayed IDR request, so a spotter opened mid-flight does
  not wait indefinitely for the ladder to move on its own.
- A PWA wrapper (install to home screen, offline cache) on top of the
  GitHub Pages hosting.
- Foreign-GS detection, so a second GS-mode page against the same drone
  gets a warning instead of silently fighting the first one's ladder.
- Auto-reconnect after the card is unplugged and replugged, instead of
  requiring a manual Connect.
