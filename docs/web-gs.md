# Web GS — mabur ground station in the browser

`web/` is a browser page (WebUSB + WebAssembly) that runs the same receive
and ladder-control core as `maburgs`, on one RTL8812EU card, with no native
daemon involved. It supersedes the throwaway `spike/wasm/` (deleted this
commit; branch `wasm-spike`, d54a9b2) — the spike answered "can devourer run
in Chrome over WebUSB and keep up with live video" (yes), this is that
answer turned into a real feature. Design: PR that added this page (see the
git log for `web/`); the working spec lived at
`docs/superpowers/specs/2026-09-27-web-gs-design.md`, which is gitignored
and machine-local, so it is not linked here — this page is meant to stand on
its own.

## What it is

A single page, one card, a fixed channel — no channel scan, no in-flight
hop, no DVR/recording, no MSP OSD, no multi-card. Two modes, picked before
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
nix-shell -p nodejs --run "BUILD=$PWD/build web/tests/test_wasm_parity.sh"
```

Page logic tests (IRAP gate, decoder slot, hitch/stat math, segment
alignment) — pass the file explicitly, a bare directory argument fails on
this machine's Node 22:

```sh
nix-shell -p nodejs --run "node --test web/tests/logic.test.mjs"
```

## Serve

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

Pick a mode, channel and width, press Connect. The browser's own device
chooser asks for the RTL8812EU (WebUSB's per-origin device grant — pick it
once and later `getDevices()` calls see it without asking again on the same
origin). What the status overlay shows after that:

- `Starting…` / `Requesting device…` — module bring-up, before the core
  loop is up.
- `No drone on ch N / W MHz (still trying)` — GS mode, no DISC_ACK within
  10 s of module start; it keeps sending DISC and the message keeps
  counting, it does not give up.
- `Waiting for key frame (sent on the next rung change) — N s` — the video
  decoder is armed but has not seen an IRAP yet. See "Spotter key frames"
  below for why this can run indefinitely.
- `GS mode keeps flying the link while this tab is hidden.` — a one-time
  banner (GS mode only) confirming the ladder and RCF sends do not pause
  when the tab loses visibility; the worker keeps running headless.
- `Card busy — maburgs or another tab has it. Close that and press
  Connect.` / `No RTL8812EU/8812AU card found — plug it in and press
  Connect.` / `Card lost (unplugged?). Press Connect to restart.` /
  `WebUSB unavailable in this browser.` / `Unsupported card chip.` — mapped
  from the core's `ERROR <reason>` lines (`web/www/logic.mjs`'s
  `errorText()`); Connect re-enables so trying again costs nothing.
- `Page worker failed — if served over LAN, the TLS CA is not trusted on
  this device (see docs/web-gs.md).` — the untrusted-cert failure mode
  above; that is this file.

**Spotter key frames.** WebCodecs will only start decoding on a real IRAP
(NAL type 19), never on the drone's GDR parameter-set refresh
(`32 33 34 1`, TRAIL_R). The drone only emits a real IDR on a rung change
or link-up, so a spotter opened mid-flight on a steady link can sit on
"Waiting for key frame" until the ladder next moves — there is no drone,
wire or config change to force one (see Follow-ups). GS mode does not have
this problem: linking a previously-unlinked drone changes its bitrate
(max-range floor → rung), and that `SetChnAttr` costs an IDR on its own, so
a self-linking GS's own link-up produces the first key frame roughly a
second after DISC_ACK.

## Stats panel

A 1 Hz snapshot from the core (`webgs::Stats` / `stats_json()` in
`web/src/web_gs.cpp`), plus page-side timing JS adds on receipt. Segments
(spec §4):

| Segment | What it measures | Mode |
|---|---|---|
| Control RTT (EWMA / min) | `RttEstimator`: RCF seq echoed back in `Telem`, clock-sync-free | GS |
| USB lateness p99/max | host arrival time − chip TSF, above the window's own minimum (`UsbLate` in `web_main.cpp`) | both |
| FEC/assembly | first body seen for the AU → AU complete, both timestamps on the core clock (`AuLatMeta`, `gs/src/frame_stream.cpp`) | both |
| Hand-off | AU complete (core/worker thread) → page receives it, both sides aligned to `performance.timeOrigin` | both |
| Decode | `VideoDecoder` chunk submit → decoded output callback | both |
| Present | decoded output → the first `requestAnimationFrame` after `drawImage` — an **estimate**, not `requestVideoFrameCallback`'s presentation time, because MSE/DRM-level present timestamps are not exposed to a WebCodecs canvas path | both |
| Capture→glass (estimate) | drone `pts` + `RttEstimator`'s min-RTT-filtered `pts_off_us` → page present, same method as `maburplay`'s `lat.log`, but computed GS-side | GS only |

Capture→glass is GS-only because it needs the RTT estimator's pts-clock
offset, which only exists when this page is itself exchanging RCF/Telem
with the drone — a spotter has no control channel to measure it over, and
the panel says so rather than showing a stale or borrowed number.

Link state shown alongside: rung/MCS/width, probe gate state, pre-FEC loss,
residual, SNR/RSSI, RCF heard % (`Telem.rcf_rx`), AUs/s, and hitches (gaps
> 1.5× the frame period). No playout regulator and no vsync lock exist in
this build — the panel is measurement-only, by design (see the spec's
"Decided" list).

## Known limits

- **Oilpan GC spikes.** Blink's incremental GC sweep of per-transfer WebUSB
  objects (`cppgc::Sweeper::IncrementalSweepTask`) runs on the thread that
  owns those objects and, after a major GC roughly every 8–13 s at ≥38 Mb/s
  on air, costs one periodic spike of 8–14 ms (native `maburgs`'s
  equivalent tail: ≤1.7 ms). Moving the runtime into a dedicated worker
  moves the sweep with it — the spike is unchanged. The only lever found is
  fewer USB transfers (chip-side RX aggregation), which is a latency trade,
  not taken here.
- **Spotter can never force an IDR.** No GS-relayed IDR request path exists
  (see "Spotter key frames" above and Follow-ups).
- **Chrome must run via the Nix wrapper** (`bin/google-chrome-stable`), not
  the raw `share/google/chrome/chrome` binary — the raw binary has no
  libEGL, its GPU process dies, and WebCodecs then rejects the drone's HEVC
  stream outright ("Unsupported configuration").
- **Two GS-mode pages against one drone fight.** Nothing detects a second
  commander; both DISC and both send RCFs, and the ladder each one drives
  fights the other's. Spotter mode has no such conflict since it never
  transmits.
- **No C++ exceptions in the WASM build** (Emscripten build flags omit
  them): a config load error aborts the module rather than surfacing a
  caught exception's message the way the native CLI's `load_cfg` does.
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

Pending — bench hardware acceptance (GS mode flying the bench link with the
GS box off, spotter mode next to a live `maburgs`) is a follow-up task, not
covered by this commit.

## Follow-ups

Not built here, all noted in the spec as later work:

- A periodic or GS-relayed IDR request, so a spotter opened mid-flight does
  not wait indefinitely for the ladder to move on its own.
- Hosting beyond a local `serve.py` — GitHub Pages plus a
  `coi-serviceworker` shim (for the COOP/COEP headers a static host cannot
  set) and a PWA wrapper. The build output is already static and
  host-agnostic, so this is a deploy step, not a redesign.
- Foreign-GS detection, so a second GS-mode page against the same drone
  gets a warning instead of silently fighting the first one's ladder.
- Auto-reconnect after the card is unplugged and replugged, instead of
  requiring a manual Connect.
