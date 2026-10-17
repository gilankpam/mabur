# Web GS — mabur ground station in the browser

`web/` is a browser page (WebUSB + WebAssembly) that runs the same receive
and ladder-control core as `maburgs` on one USB card (RTL8812EU, RTL8812AU,
RTL8812CU or TP-Link RTL8821AU) or on the CPE510 relay, with no native daemon
involved. The design spec (`docs/superpowers/specs/2026-09-27-web-gs-design.md`
and its follow-ups) is gitignored and machine-local; this page stands on its
own. Bench records: `docs/findings/2026-09-27-web-gs-validation.md`,
`docs/findings/2026-10-07-web-gs-nack-bench.md`,
`docs/findings/2026-10-10-web-gs-8812cu.md`.

## What it is

A single page. GS mode runs the same channel set, boot pick/relocation and
one-card reactive hop as `maburgs`, through the shared `ChannelCore`
(`gs/src/channel_core.h`, `docs/channel-select.md`,
`docs/inflight-channel-hop.md`). There is no multi-card. Recording is either
local raw recording into the browser's own storage or the VTX's onboard SD
recorder (see Record, below). The page's radio is either one USB card
(WebUSB) or the CPE510 relay (`ws://`, see "CPE relay radio" under Use,
below). The radio and one of two modes are picked before Connect:

- **GS mode** — the *only* ground station for the drone. Runs rendezvous
  (DISC until DISC_ACK), the same measured-loss ladder as `maburgs`
  (probe-before-promote, fade, failsafe), and slots its RCFs into the
  drone's inter-AU idle exactly like `RcfSlotter`. Its DISC proposes the
  channel it is sent on; the link forms where the drone is found and then
  relocates once to the pick (or the pin), exactly as `maburgs` does.
- **Spotter mode** — receive, decode, display; **never transmits**. Built
  to sit next to a real `maburgs` that is already flying the link. There is
  no send path in this mode by construction (`web/src/web_gs.h`): the
  constructor drops the `Io::send` callback and neither `VrxController`
  nor `RcfSlotter` exists, so there is no code path that can build a
  control frame. A spotter starts on its start member and then follows the
  link on its own (see "Spotter follower" under Use).

Only one GS may command a given drone. The page cannot detect a second
commander and does not try to (see Known limits). The mode picker's hints
read "Full link control. Sends link feedback to the drone and runs the
adaptive ladder." (GS) and "Receive only. No uplink to the drone, so link
and ladder settings are not applied." (Spotter)
(`web/ui/src/components/DisconnectedOverlay.svelte`).

## Build

Host build + tests (the WebGs core, `webgs` CLI, host ctest targets):

```sh
nix-shell -p pkg-config libusb1 --run "cmake -S . -B build -DDEVOURER_DIR=$PWD/../devourer && cmake --build build -j && ctest --test-dir build -R 'test_web_gs|test_spotter_follow|test_relay_ring|web_relay_e2e|web_au_parity'"
```

libusb for WASM (once; builds upstream libusb with the Emscripten WebUSB
backend into a repo-local `EM_CACHE` sysroot — `web/tools/build-libusb-wasm.sh`):

```sh
nix-shell -p emscripten autoconf automake libtool pkg-config --run web/tools/build-libusb-wasm.sh
```

The WASM page build. The standalone `web/` CMakeLists builds `webgs.js` /
`webgs.wasm` into `web/dist/` and the `webgs_node` replay module used by the
parity gate. It then runs `npm ci` in `web/ui` (when the lockfile changed or
`node_modules` is gone) and `npm run build` (`vite build`). That writes the
page UI (`web/ui/`, Vite + Svelte 5, lockfile committed) into `web/dist`
next to the module. CMake requires `npm`, so `nodejs` must be in the shell:

```sh
nix-shell -p emscripten cmake pkg-config python3 nodejs --run "export EM_CACHE=$PWD/web/.deps/emcache; emcmake cmake -S web -B web/build-wasm -DCMAKE_BUILD_TYPE=Release && cmake --build web/build-wasm -j"
```

WASM/native parity (`web/tests/test_wasm_parity.sh`; needs both the host
`build/web/webgs` and `web/build-wasm/webgs_node.js` already built):

```sh
nix-shell -p nodejs python3 --run "BUILD=$PWD/build web/tests/test_wasm_parity.sh"
```

For UI-only iteration, `npm run dev` in `web/ui` (inside `nix-shell -p
nodejs`) runs Vite's dev server. It serves `webgs.js`/`webgs.wasm` straight
out of `web/dist` (build that once via the CMake step above) with the same
COOP/COEP headers the static server sets, so a page-only change needs no
WASM rebuild.

Page logic tests (session lifecycle, config form, rec state, hitch/stat
math, segment alignment, layout). Pass the files explicitly; a bare
directory argument fails on this machine's Node 22:

```sh
nix-shell -p nodejs --run "node --test web/tests/*.test.mjs"
```

**The OPFS sink** (`web/src/opfs_file.h`/`.cpp`, page build only). Local
recording writes through `OpfsFile`, which holds `FileSystemSyncAccessHandle`s
on the core pthread (the dedicated Worker where that API exists). Files are
opened and removed with `EM_ASYNC_JS`, awaited through ASYNCIFY and called
only from the core loop, never from inside an AU callback; writes after that
are synchronous. A startup probe (`opfs_probe()`) creates, writes, checks and
removes a throwaway file once at module start and prints `webgs: opfs
ok|unavailable`, gating whether local recording is offered at all. This is
**not WASMFS**: `-sWASMFS` (Emscripten 4.0.12) makes `fcntl(F_SETFL,
O_NONBLOCK)` return `EINVAL`, which breaks the WebUSB libusb backend's
`libusb_init` — so the page keeps its ordinary Emscripten filesystem and
talks to OPFS directly instead. `webgs_node` and the native `webgs` CLI keep
the plain `FILE*` `DvrSink` (`common/include/mabur/dvr_mux.h`) that both raw
DVR paths share.

## Serve

**Hosted: <https://gilankpam.github.io/mabur/>** (stable) and
**<https://gilankpam.github.io/mabur/edge/>** (edge) — the simplest way in,
and the only one that works on a phone without extra setup. The `Pages`
workflow (`.github/workflows/pages.yml`) builds the WASM core and the page
whenever a change touches `web/`, `gs/src/`, `gs/bundle/` or `common/`. It
pushes `web/dist` into the `gh-pages` branch (Pages source: deploy from a
branch). A `master` push (or a manual Actions → Pages → Run workflow on
master; any other ref goes to edge) rewrites the site root and leaves
`edge/` alone. A push to any same-repo PR rewrites `edge/` only, so edge is
whichever PR pushed last. The edge page
says `edge #PR sha` next to the brand. Both channels are one origin, so
they share the WebUSB grant, `localStorage` settings and the OPFS
recordings. It is real HTTPS on a public certificate, so no
local CA is needed on any device. GitHub Pages cannot send the COOP/COEP
headers WASM pthreads need, so
`web/ui/sw/sw.js` (built into `web/dist/sw.js`; based on coi-serviceworker,
MIT) is a service worker that adds them — the very first visit reloads the
page once. The same worker makes the page **work offline**: on install it
precaches every file of that build (list injected at build time by
`web/ui/sw/plugin.mjs` from Vite's bundle + `public/` + `webgs.{js,wasm}`).
If any single file fails to precache, the whole install fails (a per-file
warning is still logged). The previous worker and its complete cache stay in
control, and the browser retries the install on a later navigation. This
keeps an incomplete cache from going live and failing offline on the
missing file.

Same-origin requests then get one of three fetch strategies
(`fetchStrategy()`, `web/ui/sw/plugin.mjs`), picked per request so a
gh-pages deploy that deletes old hashed files can never serve a stale
`index.html` alongside a 404ing asset:

- Hashed build assets (`assets/…`) are **cache-first**: the cached copy if
  present, else the network.
- A **navigation** (loading the page itself) is network-first with a 3 s
  timeout, falling back to the cache on a slow or failed network.
- Everything else (`webgs.js`, `webgs.wasm`, `font_btfl.png`, …) is
  network-first with **no** timeout. It falls back to the cache, when a
  cached copy exists, on a network error or a non-ok response (e.g. a
  mismatched `webgs.wasm` 404ing against a fresher `webgs.js`).

So: open the hosted page once with internet; afterwards it loads with none
(e.g. a laptop whose only network is the CPE). Online visits always get the
latest deploy; reload while online to pick one up. Stable and edge keep
separate caches (`mabur:<scope>:<build id>`). A page still controlled by the
old vendored coi worker registers the new one on its next visit
(`shouldRegister()`). The WebUSB grant is per origin, so each device picks
the card once on this site. The page is the bundle-default ladder build,
same as a local one.

**Install as an app.** The page is an installable PWA: Chrome's menu
(Android: "Add to Home screen" / "Install app"; desktop: the install icon
in the address bar) installs it. `web/ui/public/manifest.webmanifest`
launches it `display: fullscreen`, `orientation: landscape`, with relative
`start_url`/`scope`, so each channel installs as its own app (`/mabur/` and
`/mabur/edge/`) and a local `serve.py` build is installable too. The
manifest and icons sit in `public/`, so the offline precache above carries
them; an installed app opens offline exactly like the tab. Launched
installed, the phone overlay drops its Fullscreen button and Connect skips
the `requestFullscreen` + `orientation.lock` step
(`matchMedia('(display-mode: fullscreen)')`, `App.svelte`): the window is
already fullscreen, while `document.fullscreenElement` stays null there.
Icons: `web/ui/icons/icon.svg` (the design handoff's mark) and
`icon-maskable.svg` (full-bleed background, glyph scaled 0.8 into the
maskable safe zone) are the sources. The PNGs in `public/` are committed
renders (`rsvg-convert` then `oxipng --strip all`, `nix-shell -p librsvg
oxipng`), and `favicon.svg` is the handoff's simplified small-size mark.
`web/tests/pwa.test.mjs` pins the manifest fields, the icon sizes and that
every linked file is precached.

`web/serve.py` is a static server for `web/dist` that sets the COOP/COEP
headers WASM pthreads need (`Cross-Origin-Opener-Policy: same-origin`,
`Cross-Origin-Embedder-Policy: require-corp`).

**Localhost** (Chrome only — see Known limits for why): `python3
web/serve.py 8808`, then open `http://127.0.0.1:8808/`. Plain HTTP is a
secure context only on `localhost`, which is enough for a card plugged into
the same machine running Chrome.

**LAN (a phone, or any browser that isn't on the serving machine)** needs
HTTPS: WebUSB and `SharedArrayBuffer` both require a secure context.
Clicking through a bare self-signed certificate is **not** enough. The
bypass covers the tab, but Emscripten's pthread pool is workers spawned from
a worker, and those workers' script fetches still fail the certificate
check ("worker sent an error! undefined:undefined: undefined"). `session.js`
maps that to the "Page worker failed" banner (`WORKER_FAILED`). The
certificate has to be genuinely trusted:

```sh
nix-shell -p openssl --run ./web/mkcert.sh   # local CA (once, reused) + server cert
python3 web/serve.py 8808 0.0.0.0 --tls
```

`mkcert.sh` writes `web/tls/` (gitignored). That holds a local CA,
name-constrained to `localhost` and the private IPv4 ranges (so trusting it
can never vouch for a public site), and a server cert signed by it covering
`localhost` plus every IPv4 this host currently has. Re-running it reuses an
existing `tls/ca.key` rather than minting a new CA. Install the CA on each
viewing device before opening the page there:

- **Android**: Settings > Security > Encryption & credentials > Install a
  certificate > CA certificate, pick `web/tls/ca.crt`.
- **Linux Chrome**: `certutil -d sql:$HOME/.pki/nssdb -A -t "C,," -n
  mabur-webgs-ca -i web/tls/ca.crt` (`nix-shell -p nssTools` if `certutil`
  is missing; remove with `certutil -d sql:$HOME/.pki/nssdb -D -n
  mabur-webgs-ca`), then restart Chrome. A host that already trusts the CA
  under another nickname needs nothing.

Then open `https://<host-ip>:8808/`. WebUSB's device grant is per browser
origin, so the card chooser asks again on the new `https://` origin even if
`http://127.0.0.1` already had a grant.

**NixOS firewall**, if the LAN device can't reach the port:
`sudo iptables -I nixos-fw -p tcp --dport 8808 -j nixos-fw-accept` (lost on
firewall reload — not persistent).

## Use

Pick a mode, a radio, a channel set and width, press Connect. The page
defaults to **Spotter** mode; the URL's `?mode=` or the last-used choice
(`localStorage` `webgs.last`) overrides that. This is a deliberate safety
default: opening the page must never start commanding a drone that a
`maburgs` or another tab may already be flying, so GS mode is an explicit
opt-in.

On a phone the layout is always the immersive one, but the browser's own
bars stay until the page asks for real fullscreen. The overlay's fullscreen
button (corners-out / corners-in) calls `requestFullscreen()` and then tries
`screen.orientation.lock('landscape')`; Android Chrome allows the lock only
while fullscreen. The button follows the actual `fullscreenchange` state. It
is hidden where the Fullscreen API isn't available
(`document.fullscreenEnabled` false, e.g. iPhone Safari) and in an installed
app (see Serve).

### Connect and Disconnect

**Connect/Disconnect are in-page** — no reload to change mode, channel set,
width or (GS mode) the ladder, and no reload between flights.
`Session` (`web/ui/src/lib/session.js`) drives a small state machine
(`idle → connecting → live → stopping → idle`, plus `error`). Connect
requests the device and builds a **fresh** WASM module every time (never
reused across connects) with `--mode`/`--ch`/`--w` and `--overlay
/overlay.toml`. The overlay is passed in both modes, always, even when the
form matches the embedded default. It is the form's overlay TOML, written
into the module's virtual FS before `main()` runs. A spotter's overlay
carries `[radio]` plus a minimal one-rung `[link]` at the page width.

Disconnect calls the exported `webgs_stop()`, which sets an atomic the core
loop polls. The core tears down cleanly and returns from `main()`, and
`-sEXIT_RUNTIME` fires `Module.onExit()`. The page treats that as the signal
that the module is gone and the UI can go back to `idle` (or straight into a
new Connect). If `onExit` doesn't arrive within 3 s (`stopTimeoutMs`), the
page treats the module as stuck rather than waiting indefinitely: it reloads
the page as a fallback, landing back on the Config tab. If recording is on
(see Record, below), it is stopped before the stop request goes out.

WebUSB has no transfer cancel (libusb's emscripten backend `cancel` is a
no-op). On a quiet channel (drone off, nothing on air) the RX loop's pending
`transferIn` calls would never complete, so the core could not exit. The
workaround lives in `RadioFrontend::stop()` under `__EMSCRIPTEN__`
(`gs/src/radio_frontend.cpp`):

1. Wait 300 ms for the RX loop to end on its own.
2. If it hasn't, release interface 0. Chrome then aborts those reads (they
   surface in the console as `NetworkError: ... A transfer error has
   occurred`).
3. Join the RX thread.
4. Re-claim the interface for the chip's power-down writes, and continue.

It prints one line per stop, on stderr: `maburgs radio card 0: teardown rx
<ms> [(reads aborted)]`. A stop requested while the card is still
initialising (~5 s after Connect) takes effect only once init finishes.

The browser's own device chooser asks for the card. The chooser filters on
vendors 0bda and 2357 (`USB_FILTERS`, `web/ui/src/lib/logic.mjs`). The core
opens exact VID:PIDs (`web_main.cpp`'s `kCards`, handed to
`RadioFrontend::Cfg::ids`):

- 0bda:a81a / 0bda:881a: RTL8812EU (Jaguar3).
- 0bda:8812: RTL8812AU or EU.
- 0bda:c812: RTL8812CU (Jaguar3 8822C; tested on a Comfast CF-924AC V3,
  sold as "RTL8812BU" — chip-id 0x13 says 8822C).
- 2357:011e / 0120 / 0122: TP-Link RTL8821AU (Jaguar1 1T1R; the Archer T2U
  Plus is 0120).

devourer's chip-id read picks the driver; the WASM core builds the Jaguar1
driver and both Jaguar3 variants (8822E and 8822C; the 8822C adds ~620 KB to
`webgs.wasm`). WebUSB's device grant is per origin:
pick the card once and later `getDevices()` calls see it without asking
again on the same origin. A screen wake lock (`web/ui/src/lib/wakelock.js`)
is held while connecting or live and re-requested when the tab becomes
visible again.

**Comfast CF-924AC (RTL8812CU): eject its driver disk before Connect.**
The stick powers up as a USB disk (0bda:1a2b "DISK", Realtek's driver-CD
mode) and only becomes the Wi-Fi card (0bda:c812) once that disk is
ejected — on every plug-in. The page cannot do it for you: in that mode the
stick is mass storage, a WebUSB protected class, so Chrome neither lists
nor opens it. On Android, where nothing ejects it, the CU does not work in
the web GS — use an 8812EU/AU. The CU's bring-up also runs ~20 s (efuse
read 6.7 s, BB/AGC/RF tables 9.8 s, firmware 2.2 s over native libusb)
against ~5 s for the others.

**The RTL8821AU cannot decode LDPC.** The drone codes every frame (video,
probe, DISC_ACK, telemetry, MSP) with LDPC by default (`[radio] ldpc = true`
in `bundle/mabur.default.toml`). With this card the page therefore sees only
CRC-failed bodies and never links. To fly BCC instead, set `[radio] ldpc =
false` in the drone's `/etc/mabur.toml` (config before binary,
`docs/deploy.md`). That costs every GS the LDPC coding gain (~2-3 dB), so
set it back to `true` when flying an 8812EU/AU GS. The 8821AU's own uplink,
LDPC included, decodes fine. devourer drops the STBC flag on the 8821AU's TX
(one chain) and logs a one-time warning.

What the status overlay shows (`statusText()`, `web/ui/src/lib/view.js`):

- `Connecting…` / `Disconnecting…` — on the Disconnected screen, while the
  module comes up or tears down.
- `No drone on ch N / W MHz (still trying)` — GS mode, no DISC_ACK within
  10 s of module start. It keeps sending DISC and the message stays up; it
  does not give up.
- `Searching for drone on ch N / W MHz…` (GS, no DISC_ACK yet, first 10 s)
  and `Waiting for video…` (GS after the ack, or spotter) — shown only
  until the first frame of this Connect is decoded, while the screen is
  still black (`VideoPipeline.hasPicture`, cleared by `reset()`).
- `Drone rejects our link key (<fp>) — load the same mabur.key on both
  ends` — GS mode, the drone answers our DISCs with key_mismatch (our key
  differs from its key; `VrxState::KEY_MISMATCH`); the header tag reads
  `Key mismatch`. See `docs/link-pairing.md`.
- After the first frame, no overlay text covers the video while the
  key-frame gate is unarmed. A freeze holds the last frame and GS mode
  requests an IDR instead of waiting; see "Key-frame recovery" below.
  A spotter has no request path and can still sit unarmed indefinitely; see
  "Spotter key frames" below.
- `GS mode keeps flying the link while this tab is hidden.` — a one-time
  banner (GS mode only) confirming the ladder and RCF sends do not pause
  when the tab loses visibility; the worker keeps running headless.
- Errors, mapped from the core's `ERROR <reason>` lines by `errorText()`
  (`web/ui/src/lib/logic.mjs`):
  - `Card busy — maburgs or another tab has it. Close that and press Connect.`
  - `No RTL8812EU/8812AU/8812CU/8821AU card found — plug it in and press
    Connect.`
  - `Card lost (unplugged?). Press Connect to restart.`
  - `WebUSB unavailable in this browser.`
  - `Unsupported card chip.`

  Connect re-enables, so trying again costs nothing.
- `Page worker failed — if served over LAN, the TLS CA is not trusted on
  this device (see docs/web-gs.md).` — the untrusted-cert failure mode
  (see Serve, above).

This page's ladder starts from whatever `web/CMakeLists.txt` embeds into the
WASM module at build time. That is the bundle default,
`gs/bundle/maburgs.default.toml` (5 rungs, all 40 MHz, top = rung 4
mcs4/40), not a GS's own tuned `/etc/maburgs.toml`. The Config form (below)
overrides it per session. A GS box on the same drone may run a different
ladder, so don't read this page's rung numbers as that GS's unless the form
was left at its defaults.

### Channel set

The Config form's radio group has two controls:

- A **Channels** chip row: 1–8 members of the 5 GHz list (`CHANNELS` in
  `web/ui/src/lib/config.js`), plus any member outside it (`chipChannels()`),
  so every member can be removed.
- A **Link channel** select: **Auto** (measure the set, hop to the
  cleanest) or one member, which pins the link to it.

URL params override the saved form on load:

- `?chs=40,64` sets the set.
- `?ch=N` pins N. N is added to the set (kept sorted) when absent and there
  is room.
- `?ch=auto` unpins.

On Connect the form goes to the core as the overlay's `[radio]` table
(`channels = [...]`, `channel = "auto"|N` and `width`) in **both** modes
(`toOverlayToml()`). The core's loader therefore checks the set at the
page's width, not the bundle's 40. A 20 MHz set like `[165]` would otherwise
fail with `ERROR bad config`.

`--ch` is "start on this member" (`startChannel()`). It is the pin when
there is one. Otherwise it is the remembered member, if that is still in
the set: `localStorage` `webgs.channel`, written from the core's `CHANNEL
<n>` stdout lines. The glue prints one on every change of the plan's op
(`ChannelCore`'s store). Failing both, it is the first member.

In GS mode the link then forms where the drone is found and relocates once
to the pick (or the pin). In Auto, the one-card reactive hop runs on top,
exactly as in `maburgs`; a pinned member is static (`docs/channel-select.md`,
`docs/inflight-channel-hop.md`). This is pinned by `test_web_gs`'s
`gs_one_card_auto_hops_on_interference_but_pinned_never`. The same
interference that makes the Auto rig order a one-card hop leaves the pinned
rig on its member with `hop.state` idle, `hops`/`holds` 0 and the verdict
still reading `interfered` for the page. The scan/hop knobs are the bundle
defaults (`gs/bundle/maburgs.default.toml`); the form does not expose them.
A spotter starts on its start member and follows from there ("Spotter
follower" below). It never writes `webgs.channel` (no `ChannelCore`, so no
`CHANNEL` line), so it always starts on the pin or the first member.

The set is validated twice:

- **Page-side**, `checkChannelSet()` runs in `connectBlocker()` before the
  device is requested. It mirrors `common/include/mabur/channel_set.h`'s
  rules plus the pin rule:
  - 1–8 members, each 1–177, no duplicates.
  - At 40 MHz, every member on an HT40 pair and all on one side of the pair
    grid.
  - The pin is a member.

  There is no per-radio channel rule: the CPE relay tunes the whole 20 MHz
  grid 36–177, channel 144 included (`docs/cpe510-relay.md`).
- **Core-side**, `webgs::channel_width_error` re-checks with `maburgs`'s own
  loader checks:
  - The set rules.
  - The start channel is a member.
  - Pinned ⇒ start == the pin.
  - In GS mode, no 40 MHz rung at width 20 (the GS could not receive it).

  A core refusal reads `Channel/width refused: <reason>` (core line `ERROR
  bad channel/width: <reason>`).

While live, the header shows the live channel (the core's `channel`). After
it comes `scanning` (scout running, `scan_state` `scouting`), `moving` (the
relocation) or `hop` (a hop in flight). Disconnected, it shows the form's
link channel or `auto`. A spotter shows `following` or `sweeping` while it
is not locked. The Debug tab's **Channel** group carries the channel, scan
state/rounds/pick, hop state/verdict, hop target/epoch and hops/holds. A
spotter's group shows the channel and its follow state instead (see
"Spotter follower"). The core's `maburgs channel: …` / `maburgs hop: …`
stderr lines appear verbatim in the browser console log.

### Spotter follower

A spotter has no control link, so it cannot be told where the link is;
it listens for it. `webgs::SpotterFollow` (`web/src/spotter_follow.{h,cpp}`)
runs in the core on the spotter's one card:

- **Sweeping** — round-robin over the channel set, 150 ms per member
  counted from when the card reports it is on the member. A USB
  FastRetune is immediate; a CPE relay TUNE reports tuned after ~50 ms.
  The first CRC-good mabur frame on a member locks. A member the card
  never reports (a relay that cannot tune it) is skipped after 1 s. The
  spotter starts here, on its start member.
- **Locked** — every frame on the channel refreshes a 1 s silence timer;
  silence sweeps again from the next member. The real GS's RCFs carry
  its standing hop order (`hop_ch`/`hop_epoch`) in the clear; a new pair
  naming another member → **Following**. Locked means hearing any mabur
  traffic on the channel, not necessarily video; that includes the GS's own
  DISC search bursts. With the drone off, a spotter sits locked wherever
  the GS is beaconing, and sweeps again within ~1 s once the GS links
  elsewhere and stops beaconing.
- **Following** — the card retunes to the target; a frame there locks.
  `follows` counts every order acted on (confirmed, retargeted, withdrawn
  or timed out), so one hop that was retargeted once reads `follows 2`.
  Nothing on the target within 2 s (the drone's own `move_confirm_ms`) →
  back to the previous channel, with a fresh silence window. A withdrawal
  heard before the card moved returns at once. The same pair repeated is
  ignored, so a standing order after a timeout cannot ping-pong.

The RCF tag is **not verified** on a spotter, by design. It cannot be in
general, because the tag's `seq32` wrap count is private to the two link
ends. The pairing design also defends the aircraft, not a display: a forged
order drags a spotter off-channel for at most 2 s. No key is read in
spotter mode. A move resets nothing in the video path: the FrameStream
closes the gap as truncated AUs and the decoder re-arms on the next IRAP
(see "Spotter key frames"). STATS carries `follow_state` (`locked` |
`following` | `sweeping`) and `follows`. The header shows the state word
after the width while not locked. Spotter mode still never transmits.

### CPE relay radio

The radio picker on the Disconnected screen offers **USB card** (default) or
**CPE relay**. The relay is a TP-Link CPE510 running `mabur-relay` protocol
v4 (`docs/cpe510-relay.md`), reached over WebSocket instead of a locally
plugged-in card; no card chooser, no WebUSB grant.

**Address.** The CPE firmware puts the relay at **`10.83.11.1`**
(mabur-openwrt `90-mabur-lan`, DHCP for the client; see
`docs/cpe510-relay.md`). Connect therefore uses `10.83.11.1:8311`
(`RELAY_DEFAULT`, `logic.mjs`) with no address field on screen. Each Connect
probes exactly one address (`connectRelay()`,
`web/ui/src/lib/relay_connect.js`): open `ws://<addr>`, send HELLO, and wait
≤ 1.5 s for any mabur v4 message. No reply → *"No CPE relay found at
10.83.11.1 — …, or enter its address."* and the address field appears. A
typed address is saved (`webgs.last.relayCustom`), keeps the field visible
on later visits, and is then the **only** address Connect tries. Clearing it
returns to the default and hides the field from the next visit.

**Hosted page works; relay host must be a private IP; one-time prompt.**
An `https://` page normally cannot open a plain `ws://` socket (mixed
content), and the relay doesn't speak `wss://` (see `docs/cpe510-relay.md`,
"Why not wss"). Chrome's Local Network Access (LNA, Chrome 142+) changes
both halves (verified on Chrome 147; record in
`docs/findings/2026-09-27-web-gs-validation.md`):

- **Mixed content:** `ws://` to a **private IP literal** or a **`.local`
  name** is exempt. A public DNS name that resolves to the LAN is still
  blocked outright. So on an `https:` origin `relayBlocker()` accepts only a
  private/loopback/link-local IPv4 literal or a `.local` name
  (`isLocalRelayHost()`). Anything else is refused with *"On the hosted page
  the relay address must be a private IP (e.g. 10.83.11.1) or a .local
  name."*
- **Permission:** a public page reaching a private address needs the
  `local-network` permission (the older `local-network-access` name does
  not unblock it). Until it is granted, Chrome holds the socket on a
  one-time "local network access" prompt; headless, or denied, the socket
  dies with close 1006.
- **Connect waits for the prompt.** The core gives up on an unopened relay
  in ~2 s, which would fail the session underneath the prompt. On `https:`,
  `connectRelay()` queries `local-network` first:
  - `granted` probes with the 1.5 s timeout.
  - `denied` errors with a site-settings hint.
  - `prompt` lets the probe socket itself raise the prompt and waits up to
    60 s.

  A browser with no `local-network` permission (Firefox, Chrome < 142) just
  probes. Its not-found text then also names the browser block and the
  local page.

To test this, use the real hosted origin. A spoofed origin (github.io mapped
to a LAN address) is itself private, so Chrome skips the LNA check.

**What Connect does.** The page opens a dedicated Worker
(`web/ui/src/lib/relay_worker.js`) that owns the WebSocket. It is bridged to
the WASM core through a `SharedArrayBuffer` ring pair (`web/src/relay_ring.h`
/ `web/ui/src/lib/relay_ring.js`, byte-identical twins pinned by shared
golden tests; see Design, below), so the socket and its GC/render-thread
neighbors never delay an RCF. The core builds a `maburgs::RemoteCard`
(`gs/src/remote_card.{h,cpp}`) over that ring
(`web/src/relay_ring_transport.h`), with `Cfg::restart_when_refused` off.
That is the same card class `maburgs` runs for `radio.relays`. Its
`RelayClient` does the same HELLO/TUNE dance as natively
(`docs/cpe510-relay.md`):

- HELLO every 500 ms.
- TUNE retried every 500 ms while not owner, only within the first 2.5 s.
- Once owner but the relay reads back mistuned, TUNE retried every 500 ms
  indefinitely.

**Both modes own the relay** (one relay, one client): Connect waits for
`RemoteCard::health()` to read `Owned` before the session starts, in GS mode
and in Spotter mode alike.

**Search-only scout on the relay.** With no USB card, the core's boot
scout runs on the relay with search bursts only. DISC goes out on each
member in turn, but the CPE has no energy reads, so no member is ever
ranked at boot and the stats' `scan_state` reads `off`. Auto on a relay
therefore means "start member plus search" at boot. Each TUNE costs ~50 ms
of beaconing, because the scout offers no DISC until the relay reports
tuned. Once linked, the page drives the same `ChannelCore` `maburgs` does
(`docs/channel-select.md` "Where the code lives"). Protocol v4 gives a
relay-only page the full in-flight verdict → sweep → ranker → hop flow
(`docs/cpe510-relay.md` "Interference + hop on a relay"): it sends a `SCAN`
for the channel set, ranks the `SCAN_RESULT`, and hops exactly as `maburgs`
does.

**Errors** (core `ERROR` line → page text, `errorText()` in
`web/ui/src/lib/logic.mjs`), driven by `RemoteCard::health()`, in both
modes:

- `relay unreachable` — no `STATUS` within 2 s → *"CPE relay not reachable
  — check the Ethernet cable and the relay address, then press Connect."*
- `relay owned by another client` — still not owner 2.5 s after connecting
  (a spotter too, since both modes own the relay) → *"CPE relay is owned by
  another client (maburgs or another tab). Stop it, wait 2 s, then press
  Connect."*
- `relay cannot tune` — owner, but the relay isn't on our channel/width
  2.5 s after connecting (e.g. a regdomain that refuses the channel) →
  *"CPE relay could not tune to this channel/width — check the channel is
  allowed on the CPE (regulatory domain), then press Connect."*
- `relay lost` — `STATUS` stopped arriving, or the socket closed, after a
  successful connect → *"CPE relay connection lost. Press Connect to
  restart."*
- `relay taken by another client` — fires only after the page has owned the
  relay (`owned_and_tuned()` true at least once). A `STATUS` says
  `you_own == 0` and keeps saying so for >= 1 s straight. A UDP subscriber
  can cause this, e.g. native `webgs --relay` or a running `maburgs` with
  `radio.relays`, because the relay prefers the oldest UDP subscriber. →
  *"Another client took over the CPE relay (maburgs or another tab). Stop
  it, then press Connect."* This differs from `relay owned by another
  client`, which fires before ownership was ever won.

**One owner, both modes.** The page is one client of the relay whichever
mode it runs. A spotter therefore needs ownership exactly as GS mode does,
and `relay owned by another client` / `relay taken by another client` fire
for it too. A spotter next to a `maburgs` that already holds the relay
cannot share it; give the spotter a USB card instead.

**Relay group** (Debug tab, shown only when the stats line's `radio` is
`relay`; `debugGroups()` in `web/ui/src/lib/view.js`) shows these rows:

- **channel/width**, or "retuning" when `relay_state == 1` (mid-TUNE).
- **owner** (`relay_you_own`, yes/no): this client holds the relay.
- **tuned** (`relay_owned`, `owned_and_tuned()`, yes/no): owner AND on our
  channel/width. The two can disagree, e.g. right after another client
  takes ownership.
- **frames** and **seq gaps**.
- **ring drops rx/tx**: the `SharedArrayBuffer` ring, not the relay itself
  (`RemoteCard::relay_stats()`).
- **tx / fail / refused**: mirrors the relay's own `STATUS` counters.
- **your drops**: the relay's own `your_drops`, frames it had to drop for
  this client specifically (e.g. a full send queue).

The USB-only rows elsewhere on the Debug tab (usb latency, txfail) are
hidden for the relay radio; they have no relay equivalent. The general
stats line carries `radio: "usb"|"relay"` so log/replay tooling can tell
which path a session used.

**Native bench equivalent:** `webgs live --relay host:port` (default UDP
port **8310**, not the browser's WebSocket 8311) runs the same
`RemoteCard` over UDP as a CLI.

### Key-frame recovery

The gate arms only on an IRAP, and the drone ships with no periodic refresh
at all (bundle `venc.gop_s` 0). After a DISCONT or a decoder error the page
therefore cannot heal on its own, so in GS mode it asks:

1. `IdrRequester` (`logic.mjs`) fires on the drop and every 300 ms while
   unarmed.
2. `Module._webgs_request_idr()` bumps a count whose low byte rides every
   RCF as `idr_epoch`.
3. RcAgent serves one paced IDR per change.

Expected freeze ≈ 150 ms (≈ 450 ms if the IDR itself is lost). A spotter
cannot ask and waits for the next drone-side IDR. No text is drawn over the
video while frozen. The Debug tab's `IDR req` row is the page's own request
count since the current Connect.

`IdrRequester` never asks when `VideoDecoder` doesn't exist in this
browser, since no request fixes a missing codec API. It also backs off. A
key chunk that fails to decode doubles the retry interval each time,
300 ms → 600 ms → … capped at 4.8 s. Causes include an unsupported codec
rejected by `configure()`, a reclaimed hardware decoder, or the hvcC racing
its own key AU at stream start. A normal mid-stream error that follows an
actual decoded frame (the missing-reference case this feature exists for)
never backs off. Without the backoff, a device that simply can't decode HEVC
would ask the drone for a fresh IDR (the biggest frame on the link) about
3.3 times a second for the whole flight.

The drone collapses multiple `idr_epoch` bumps that land within one RCF
interval into a single IDR, so a burst of asks legitimately yields fewer key
frames than requests.

**Spotter key frames.** WebCodecs will only start decoding on a real IRAP
(IRAP NAL types 16–21: BLA/IDR/CRA), never on the drone's GDR parameter-set
refresh (`32 33 34 1`, TRAIL_R). The drone emits a real IDR on a rung
change, link-up, or chain break; on a GS-mode page's own request (see
"Key-frame recovery" above); and on maburgs's request when its base
picture is damaged (`docs/fec-nack.md` "Demand refresh") — there is no
periodic refresh any more, so a spotter on a steady link waits for one of
those. Spotter mode has no request path of its own. A spotter opened
mid-flight on a steady link can therefore sit unarmed until one of those
IDRs happens to land, and there is no drone, wire or config change to
force one (see Follow-ups).

GS mode does not have this gap. On first link, a self-linking GS's own
link-up already produces a key frame roughly a second after DISC_ACK.
Linking a previously-unlinked drone changes its bitrate (max-range floor →
rung), and that `SetChnAttr` costs an IDR on its own. Any later drop is
covered by the request path above.

The same applies after a follower move (a sweep find or a followed hop):
the spotter waits for the drone's next unsolicited IDR on the new channel.
A followed hop usually gets one soon, since the drone's retune and the GS's
post-hop rung change both cost an IDR, but nothing on the spotter can ask
for it.

## Config form

The Config tab (`web/ui/src/components/ConfigPanel.svelte`, and
`ConfigSide.svelte` for the immersive/mobile layout) edits the following.

In both modes:

- The channel set, the link channel and the width (see "Channel set" under
  Use).
- The DVR target (see Record).
- The Display group's colour correction (see Colour correction).

In GS mode only:

- **Fixed MCS** (`static_mcs`, −1 = Adaptive).
- The **Retransmit (NACK)** switch (`[link.nack] enable`, default on; see
  "Software NACK" below).
- The **Link key**: Load/Clear a `mabur.key` file. It is stored in
  `localStorage` `webgs.key`, and its fingerprint is shown as a tag. See
  `docs/link-pairing.md`.
- The ladder rungs: mcs/bw/FEC overhead per rung, add/remove up to 8. To
  reorder, drag the ⋮⋮ handle, or focus it and use ↑/↓
  (`applyRungEdit(cfg, from, '__move', to)`).

`web/ui/src/lib/config.js` normalizes and persists the form to
`localStorage` under `webgs.cfg`. Each field falls back to the bundle
default, so a stale or hand-edited entry can never break the page.
`?chs=`/`?ch=`/`?w=` query params override the saved set, link channel and
width on load, the same way `?mode=` overrides the saved mode.

On Connect, the form is always serialized to TOML, even when it matches the
embedded default (`toOverlayToml()`), and handed to the core as `--overlay
/overlay.toml`. The overlay contains:

- `[radio] channels/channel/width`, in both modes.
- In GS mode, `[link]` with:
  - `key = "<hex>"` (only when a key is loaded; otherwise the bundle's
    default key applies).
  - `static_mcs`.
  - `static_bw`, carrying the form's width.
  - `max_mcs`, always 7. The form has no Max MCS, so the ladder as listed
    is the whole policy.
- In GS mode, a `[link.nack] enable = true|false` table. The bundle's other
  NACK keys stay.
- In GS mode, one `[[link.ladder]]` block per rung.

While Fixed MCS is pinned, the form hides the ladder, and the overlay sends
a single `{mcs = static_mcs, bw = width}` rung instead of the saved ladder.
The loader validates the ladder even when pinned (a 40 MHz rung at width 20
would fail boot), and the user can't see those fields to fix them.
`connectBlocker()` skips the ladder checks likewise, and the saved ladder
comes back untouched with Adaptive.

`maburgs::load_config`'s `overlay_path` argument (`gs/src/config.h`/`.cpp`)
deep-merges the overlay over the embedded `maburgs.default.toml`. An overlay
`[[link.ladder]]` replaces the file's ladder whole; it does not append. It
then re-runs the same validation the native loader runs. A rejected overlay
surfaces as `ERROR bad config: <reason>` from the core.

`connectBlocker()` mirrors the same checks page-side, so a bad form is
refused before Connect ever starts the device request. It checks
channel/width, FEC overhead range 0.1–2.0, enh ≤ base, and no 40 MHz rung
under a 20 MHz width. Separately, per-field warnings (`rungWarnings()` /
`channelWarning()`) flag a 40 MHz rung under a 20 MHz width, enh above base,
or a channel with no HT40 pair, without blocking the rest of the form.

Spotter mode's overlay carries `[radio]` plus a fixed minimal `[link]`:
`static_bw` and one MCS 0 rung at the page width, so the loader's
ladder-vs-width check passes at 20 MHz. A spotter only listens, so only the
channel set, link channel and width apply.

## Record

One Record button drives up to two recorders, picked by the Config form's
DVR target (`sessionCfg.dvr`, `web/ui/src/lib/localrec.js`'s
`DVR_TARGETS`):

- `web`: local browser recording (the default).
- `vtx`: the drone's onboard SD recorder.
- `both`.

The target is editable only while disconnected. It takes a fresh Connect to
change what one press covers, same as channel/width. Spotter mode has no
send path, so `effectiveTarget()` forces it to `web` regardless of the saved
config, and the target control shows VTX/Both disabled with the reason.

**VTX target.** `Session.setRec()` calls the exported `webgs_set_rec(1|0)`.
The core (`WebGs::set_vtx_rec`, `web/src/web_gs.h`/`.cpp`) sets the RCF's
recorder wish byte the same way `maburgs` does (`kRecKnown | kRecOn`), so the
drone's onboard VTX SD recorder starts and stops exactly as it would under a
native GS (`docs/vtx-recorder.md`). The VTX part of the indicator follows
the drone's own report, not the click. It reads `Telem.rec_status` back off
the stats stream (core `rec_state`/`rec_err` keys) and only counts as
"recording" once the drone confirms it. A GS-mode Disconnect with the wish
still on sends the "off" RCF and waits up to 1 s (`recOffTimeoutMs`) for the
drone to confirm `rec_state != 1` before stopping the module. That way a
Disconnect doesn't leave the VTX recording with nothing left to turn it off.

**Local (`web`) target.** `RawDvr` (`common/include/mabur/raw_dvr.h`, shared
with `maburplay`'s raw DVR mode; see `docs/observability.md`) runs on the
core thread. It is fed every AU in `io.on_au` before the page hand-off. Its
sync point is the first *complete* IRAP (`au_is_irap()`, in practice an IDR)
once VPS+SPS+PPS are known, and only IRAPs are marked sync inside the file.
Arming also bumps the GS-requested IDR counter (a no-op for a spotter), so
pressing Record shows the button label `waiting for sync…` for about one
request round trip. Then that IDR arrives and `RawDvr` starts muxing samples
into the file. Files recorded before 2026-09-29 used a different sync point;
their history and repair recipe are in
`docs/findings/2026-09-29-web-gs-raw-dvr-irap-sync.md`.

The file exists before the sync point, though. The (empty) OPFS file is
created and opened right at the press (`OpfsFile::open()`, see Build,
above), wrapped as a `DvrSink` (`OpfsSink` in `web_main.cpp`), and held by
`RawDvr` while it waits for the sync AU. A press-then-release inside that
window leaves it empty, and the core removes it when it seals rather than
reporting a 0-byte download. `open()` never overwrites an existing non-empty
file. A same-second name collision (or a leftover file from an earlier
session) fails the start with "cannot create file" instead of truncating it.
Files are named `mabur-YYYYMMDD-HHMMSS.mp4` (`recFileName()`, browser local
clock, with a `-2`/`-3` suffix chosen against the already-listed recordings)
and created directly in the OPFS root.

`Module.onRecClosed(name, bytes, err)` fires once per recording, on every
way it can end:

- an explicit stop;
- a `RawDvr::Error` (sticky, surfaces once);
- Disconnect;
- card loss.

`seal_local_rec()` runs on every exit path, including the module teardown,
so nothing is left open when the WASM module goes away. `bytes > 0` triggers
a download: `downloadRecording()` reads the file back with the disk-backed
`getFile()` (no in-memory copy of a multi-gigabyte flight) and drives a
synthetic `<a download>` click. A recording sealed at 0 bytes (stopped
before the sync point, or the open failed) is removed from OPFS rather than
downloaded or listed.

**Recordings list** (Config tab, `Recordings.svelte`): every `.mp4` left in
OPFS, newest first, with Download/Delete per row and a running storage-used
line. Nothing is ever auto-deleted. A recording survives a page reload or a
crashed tab and reappears in the list next time the page opens; `Config →
Recordings` is how you get a file whose download the crash itself
interrupted. The first local recording of a session calls
`navigator.storage.persist()` (`App.svelte`) so the browser is less likely
to evict OPFS under storage pressure. `headroomWarning()` checks
`navigator.storage.estimate()` and shows a "Low browser storage: ~N MB free"
note under 1 GiB free, without blocking Record. `opfsOk` (the startup probe
result) gates the whole feature. With no usable OPFS the Recordings list
says so and local recording is off; VTX recording is unaffected.

**Stats and errors.** The 1 Hz stats line carries these keys:

- `lrec_avail`: 0/1, the startup probe result.
- `lrec_state`: 0 off, 1 waiting-for-sync, 2 recording, 3 error.
- `lrec_bytes`.
- `lrec_err`: 1 = file open failed, 2 = write failed (storage full or
  gone), 3 = no OPFS in this browser.
- `lrec_name`.

The Debug tab's Client group has a `local rec` row (state · size · Mb/s,
from `lrec_state`/`lrec_bytes` deltas).

## Stats panel

The panel shows a 1 Hz snapshot from the core (`webgs::Stats` /
`stats_json()` in `web/src/web_gs.cpp`), plus page-side timing JS adds on
receipt. The snapshot includes the recorder's `rec_state`/`rec_err` (Record,
above) and `drone_temp_c`. That is Telem `soc_temp_c`, null until a Telem
lands or when the drone reports −128 (no thermal source), and it is shown as
"Drone temp" on both the Stats tab and the floating panel.

Where the panel lives depends on the layout:

- **Windowed:** the sidebar's Stats/Config/Debug tabs (`Sidebar.svelte`).
- **Immersive/fullscreen or mobile:** a floating, draggable panel
  (`FloatStats.svelte`, `web/ui/src/lib/layout.js`), so it stays visible
  over the fullscreen video.

Every page-side segment row shows p50/p99/max over both a 1 s and a 60 s
window; USB lateness uses the core's own 1 s window. Page-side video metrics
(`web/ui/src/lib/metrics.js`'s `PageMetrics`, distinct from the core's 1 Hz
stats) resample every 200 ms over a trailing 1 s window:

- bitrate (AU bytes);
- fps (draw count);
- jitter (mean deviation from the nominal draw period);
- latency: capture→glass p50 in GS mode, and the rx→glass segment sum in
  Spotter mode. A spotter has no capture-side clock to measure capture→glass
  from without the RTT estimator's offset; see Capture→glass below.

"Copy stats" copies `{core: [last 60 s of 1 Hz stats lines], segments: {w1,
w60}}` as JSON. Segments:

| Segment | What it measures | Mode |
|---|---|---|
| Control RTT (EWMA / min) | `RttEstimator`: RCF seq echoed back in `Telem`, clock-sync-free | GS |
| USB lateness p99/max | host arrival time − chip TSF, above the window's own minimum (`UsbLate`, `gs/src/usb_late.h`, read by `web_main.cpp`) | both (USB radio) |
| FEC/assembly | first body seen for the AU → AU complete, both timestamps on the core clock (`AuLatMeta`, `gs/src/au_ring.h`, stamped by `FrameStream`) | both |
| Hand-off | AU complete (core/worker thread) → page receives it, both sides aligned to `performance.timeOrigin` | both |
| Decode | `VideoDecoder` chunk submit → decoded output callback | both |
| Present | decoded output → the first `requestAnimationFrame` after `drawImage`. This is an **estimate**, not `requestVideoFrameCallback`'s presentation time, because MSE/DRM-level present timestamps are not exposed to a WebCodecs canvas path. A hidden tab suspends rAF, so its callback only runs once the tab is visible again. A callback that fires >1000 ms after its `drawImage` is therefore dropped rather than counted (`isStalePresentSample()`, `web/ui/src/lib/logic.mjs`) | both |
| Capture→glass (estimate) | drone `pts` + `RttEstimator`'s min-RTT-filtered `pts_off_us` → page present, the same method as `maburplay`'s `lat.log` but computed GS-side. It shares Present's rAF path, so the same >1000 ms stale-sample drop applies (`isStalePresentSample()`) | GS only |

Capture→glass is GS-only because it needs the RTT estimator's pts-clock
offset. That offset only exists when this page is itself exchanging
RCF/Telem with the drone. A spotter has no control channel to measure it
over, and the panel says so rather than showing a stale or borrowed number.

Link state shown alongside:

- **rung/MCS/width.** A spotter shows its configured width and the MCS read
  off the air: the RX-descriptor MCS of CRC-clean base-stream bodies, never
  enh or probe. It drives no ladder.
- probe gate state.
- pre-FEC loss and residual.
- SNR/RSSI.
- **RCF heard %:** Δ`Telem.rcf_rx` / Δ`rcf_sent` over a trailing ~10 s
  window of the stats ring, not the adjacent 1 s tick
  (`rcfHeardPctWindowed()` in `web/ui/src/lib/logic.mjs`). The two counters
  are sampled at different instants, so adjacent ticks can read over 100 %.
  `rcf_sent` counts RCFs only, not the DISC beacons/keep-alives that `sends`
  also counts, because the drone's `rcf_rx` counts RCFs only.
- AUs/s.
- hitches (gaps > 1.5× the frame period).

No playout regulator and no vsync lock exist in this build; the panel is
measurement-only, by design.

The core stats line also carries `osd_snaps` (MSP snapshots out of the FEC
sink) and `osd_screens` (OSD screens handed to the page); see MSP OSD below.

`trunc` counts truncated AUs of both layers. `trunc_base` is the sid-0
share of it (Debug tab Counters, `trunc base`). That is the layer the NACK
protects, and the one whose loss smears until the next GDR/IDR.

`salvaged` (stats key `salvaged`, `WebGs::Stats::aus_salvaged`) counts AUs
the core rebuilt by H.265 slice salvage (`docs/slices.md`): a truncated AU
whose lost row slices were filled with skip slices instead of being passed
through truncated. The JS Gate decodes a salvaged AU once armed, same as any
other AU, but never arms the decoder on one; only a complete AU is a sync
point.

The Debug tab's **NACK** group (`debugGroups()` in `view.js`) reads the
stats line's `nack` block. The block is `null` in spotter mode or with the
switch off. Otherwise it holds:

- The tracker's cumulatives: `req rep syms tail fill late waste drop sup
  lead`.
- `sent`: T_NACK frames that left the card.
- `settle_ms`.
- The last 1 s window's fill latencies (`fill_p50/p90/max`, `fill_pps`;
  null while no fill landed) and `late_max`.
- The drone's Telem counters, summed once per `tlm_seq`: `drone_rx`,
  `drone_syms`, `drone_refused`.

See "Software NACK" below.

## MSP OSD

The flight controller's MSP DisplayPort OSD is drawn over the video in both
modes. It is receive-only, so Spotter's no-send guarantee is untouched, and
it does not wait for the GS session. The path is `maburgs`'s: `WebGs` feeds
stream-id 4 bodies to the same `maburgs::MspSink` (SBI unpack + sliding-
window FEC). `symbol_size`/`window` come from the embedded config's `[msp]`
and must match the drone. `[msp] enable = false` turns it off, and
`[msp.out]` is ignored (there is no UDP here). `webgs::OsdScreen`
(`web/src/osd_screen.h`) runs each snapshot through the shared
`MspParser`/`MspScreen` and publishes a completed screen at most every 30 ms,
latest wins (`OsdSource`'s gate). The glue posts it as
`Module.onOsd(rows, cols, Uint16Array)`.

The page (`web/ui/src/lib/osd.js`) spreads the grid over the video box,
which is 16:9 in every layout. Immersive (fullscreen, phones) letterboxes
the video instead of cropping it, so the OSD sits exactly on the picture and
an HD grid's (53x20) cells keep the font's 2:3 shape. An SD grid is widened
to 16:9 as on `maburplay`. The OSD paints on a second canvas over the video,
at most once per animation frame. It blanks after 5 s without a screen
(`maburplay`'s `stale_ms` default) and on Disconnect. There is no toggle;
trim elements in the Betaflight OSD tab.

Font: Betaflight only, `web/ui/public/font_btfl.png` (32 glyphs per row,
36×54). It is generated from `maburplay`'s `gs/player/bundle/font_btfl.mfont`
by `tools/msp/gen_webfont.py` (re-run it if the `.mfont` changes; it
self-checks the round trip). The page fetches it on the first screen. A
failed fetch logs once to the console and leaves the OSD off for the rest of
that session (it is not refetched per screen); the next Connect tries once
more. The 5 s blank counts from the last *published* screen, where
`maburplay` counts from the last datagram. That is the same thing in
practice, since Betaflight sends DRAW_SCREEN continuously.

## Software NACK

GS mode runs `docs/fec-nack.md`'s base-layer selective repeat with the same
`mabur::NackTracker` maburgs polls. It uses the same `SwDecoder` erasure
view, `FrameStream` tail view and ladder util (`WebGs::poll_nack_`). The
tracker is built only in GS mode with `[link.nack] enable`; a spotter has no
tracker, as it has no `VrxController`.

Per `tick()`, after the FrameStream poll:

- A new `vtx_nonce` restarts the counter.
- No nonce, or a dropped frame wire, clears the entries. The counter is
  kept, as the drone's `accept_nack_counter` holds it under the same nonce.
- Otherwise at most one `T_NACK` goes out through `send_()`. It is sent
  directly, never through the `RcfSlotter`, gated by the scout's `may_send`
  like every control frame, and counted in `sends` but not `rcf_sent`.

There is no calibration radio-silence gate (the page has no calibration). There is also no
`hdr_retx` latency-anchor rule (the page's capture→glass uses the RTT pts
offset, not a first-fragment anchor). The `FragArrival` carries
`sw_seq`/`retx`, which is what makes the tail trigger live. As on maburgs
(Option A), a retransmit-filled symbol still counts as loss for the ladder.
Over the CPE relay radio the request rides the relay's TX round trip.

Readout is the Debug tab's NACK group (Stats panel, above); the switch is
on the Config tab (Config form, above). This is pinned by
`tests/test_web_gs.cpp` (tagged counter-1 frame under unrepairable loss,
counter across a same-nonce rejoin vs a new nonce, the stats block, the
overlay loader) and `web/tests/{config,view}.test.mjs`. The page requests
no IDR on a truncated AU (`Gate.onAu` → `skip: 'truncated'`). Bench record
(USB, CPE relay, browser): `docs/findings/2026-10-07-web-gs-nack-bench.md`.

## Colour correction

The drone flies the colortrans sensor file, which flattens the picture on
purpose; `maburplay` undoes it in the CRTC LUT (`docs/colortrans.md`). The
page does the same on the video, **on by default**, with an On/Off toggle
in the Config form's Display group. It is page-only (never sent to the
core, persisted in `webgs.cfg` as `colortrans`) and stays editable while
live; it takes effect on the next frame.

On: `VideoPipeline.draw()` (`web/ui/src/lib/video.js`) hands each decoded
`VideoFrame` to `ColorTransGl` (`web/ui/src/lib/colortrans.js`). That
uploads it with `texImage2D` and runs `gs/player/src/frame_colortrans.cpp`'s
shader over one full-screen quad on a WebGL canvas.

Off: a plain 2D `drawImage` on its own canvas. A canvas cannot change
context type, so the two are stacked in `App.svelte`, and whichever drew the
last frame is the visible one. Turning it off costs nothing extra.

The MSP OSD canvas sits above both, so no OSD pre-inversion is needed.
No WebGL, a shader that fails to build, or a lost context falls back to
flat for the rest of the page's life (one console line). The Stats
panel's Video line says which path is live: `· colortrans`,
`· colortrans unavailable` (wanted but fell back), or nothing (off). The
constants are pinned by `web/tests/colortrans.test.mjs` (see
`docs/colortrans.md` for the retune rule).

## Known limits

- **CPE relay radio from https needs Chrome 142+, a private-IP relay
  address, and a one-time "local network access" Allow.** See "CPE relay
  radio" above; Firefox needs the local page.
- **CPE relay: one owner at a time.** The relay itself enforces this, not
  the page. A `maburgs`/native `webgs` already running against the relay
  (or another tab) holds ownership. This page reports `relay owned by
  another client` rather than fighting for it, in Spotter mode as well as
  GS mode. Same one-commander rule as two GS-mode pages against a
  USB-connected drone, below.
- **CPE relay: search only at boot.** The boot scout searches but never
  ranks on a relay, and each search-burst TUNE costs ~50 ms of beaconing.
  See "Search-only scout on the relay" above.
- **Spotter follows the link only by listening.** It needs ~0.15 s per
  member on a USB card to find the link at start, worst case the whole set.
  On the CPE relay it is about 0.2 s per member (a 50 ms TUNE plus the
  dwell), so up to ~1.6 s for 8 members. It takes up to 2 s to give up on a
  followed order that never showed. It cannot ask the GS where the link is,
  and it does not verify the hop order it follows ("Spotter follower").
- **macOS: Chrome needs the system Local Network permission** (System
  Settings → Privacy & Security → Local Network) to reach any LAN address,
  the CPE's 10.83.11.1 included. This is on top of Chrome's own prompt.
- **Offline needs one online visit per channel** (stable and edge cache
  separately), and a new deploy is picked up on the next online reload.
- **RTL8812AU on a Linux host: unload `rtw88_8812au` first.** The kernel
  has an in-tree driver for the AU (the EU has none). The failure goes like
  this:
  1. devourer's `claim_interface_then_reset` resets the port after claiming.
  2. The kernel re-probes the interface, and `rtw88_8812au` takes it back
     mid-bring-up.
  3. Chrome cannot detach a kernel driver, so every transfer after init
     fails and the page reports `card lost`.

  An unbound card fails earlier, at `claimInterface`, once a re-enumeration
  has rebound it. Native `webgs` / `maburgs` don't see this, because libusb
  detaches the kernel driver itself. Fix: `sudo modprobe -r rtw88_8812au`
  (holds until a replug or reboot; a reset keeps the device number, so
  nothing reloads it), or blacklist it (NixOS:
  `boot.blacklistedKernelModules = [ "rtw88_8812au" ];`). The TP-Link
  RTL8821AU has the same problem with `rtw88_8821au`; unload that one.
  The RTL8812CU has `rtw88_8822cu`, and worse: right after the eject its
  bring-up holds the kernel's network lock for 30 s+ and freezes a GNOME
  desktop (NetworkManager, rfkill and the shell's net widgets all wait on
  it) — blacklist it before switching a CU on a Linux host.
- **Oilpan GC spikes.** Blink's incremental GC sweep of per-transfer WebUSB
  objects (`cppgc::Sweeper::IncrementalSweepTask`) runs on the thread that
  owns those objects. After a major GC, roughly every 8–13 s, it costs one
  periodic spike of 8–14 ms; native `maburgs`'s equivalent tail is ≤1.7 ms.
  Moving the runtime into a dedicated worker moves the sweep with it, so the
  spike is unchanged. The only lever found is fewer USB transfers (chip-side
  RX aggregation), which is a latency trade, not taken here.
- **A decoder error freezes the picture until the next key frame.** On any
  WebCodecs decoder error the page drops its decoder, unarms the gate, and
  waits for an IRAP (`VideoPipeline`'s `error` callback → `onDecoderError()`
  → `replaceDecoder()`, `web/ui/src/lib/video.js`). The usual cause is a
  missing reference after a base AU that was truncated and not salvaged, or
  dropped whole. In GS mode the IDR request bounds the freeze (~150 ms);
  a spotter waits for the drone's next unsolicited IDR. MPP on `maburplay`
  conceals and keeps decoding instead. The page therefore shows a salvaged
  picture (`docs/slices.md`) only between such resets.
- **Spotter can never force an IDR.** No GS-relayed IDR request path exists
  (see "Spotter key frames" above and Follow-ups).
- **Spotter cannot gate on `CAP_FRAME_WIRE`.** Spotter mode has no
  rendezvous or caps exchange of its own, so its frame-wire gate is always
  true (`web/src/web_gs.cpp`). GS mode, by contrast, only decodes while in
  SESSION with a peer that has advertised `CAP_FRAME_WIRE`. A spotter
  therefore cannot detect a caps-mismatched drone (a half-deployed
  `RC_VERSION` flag-day pair, `docs/deploy.md`). Instead of refusing the
  stream the way a caps-aware GS would, it feeds the decoder whatever
  bitstream arrives, parseable or not.
- **Chrome must run via the Nix wrapper** (`bin/google-chrome-stable`), not
  the raw `share/google/chrome/chrome` binary. The raw binary has no libEGL,
  its GPU process dies, and WebCodecs then rejects the drone's HEVC stream
  outright ("Unsupported configuration").
- **Two GS-mode pages against one drone fight.** Nothing detects a second
  commander; both DISC and both send RCFs, and the ladder each one drives
  fights the other's. Spotter mode has no such conflict since it never
  transmits.
- **No OSD in the local recording.** `RawDvr` captures the raw HEVC
  bitstream ahead of the MSP OSD compositing, which only happens on the
  decoded/presented side. This is the same limitation as `maburplay`'s raw
  DVR mode (`docs/observability.md`). Burning the OSD into the local file
  would need a decode+recompose+re-encode pass this page doesn't do.
- **Capture→glass can use a stale RTT offset for up to ~30 s after a drone
  restart.** `RttEstimator`'s pts-clock offset is never explicitly reset on
  a detected drone restart, in this page or in `maburgs` itself. The
  estimator re-converges from live samples over its own window rather than
  snapping to a fresh baseline, so the number briefly reflects the previous
  boot's clock relationship.
- **`drone_state` renders as a raw number**, not a decoded state name; the
  panel does not carry the enum-to-text mapping `maburtop.py` has.

## Design

The ladder's per-tick input is `LinkHealthAssembler`
(`gs/src/link_health.h`/`.cpp`), extracted out of `maburgs`'s `main.cpp`
control loop so both front ends compute the SAME windows the same way:
`s1_loss`, `s3_loss`, residuals, probe tracking and RF-label staleness. The
point is that the web GS's ladder behaves exactly like `maburgs`'s, not
merely similarly. `maburgs` still owns log writing (ctl/fec/probe logs, the
sideport). It also owns everything the assembler does not need to decide the
ladder itself: the TX selector and cal radio silence. Scan/hop/scout and the
hop verdict are the shared `ChannelCore`, below.

`web/src/web_gs.h`'s `WebGs` composes the assembler with the same
`Aggregator`, `FrameStream`, `VrxController`, `RcfSlotter` and
`RttEstimator` maburgs uses, just with one card and no devourer types
reaching the core. `web/CMakeLists.txt`'s `webgs_core` links only the
receive + control subset of `mabur_common`, never `fec_worker` /
`sw_encoder` / `uep_encoder`, which call `pthread_setaffinity_np` and do not
build under Emscripten. The dot11 frame-building/parsing pure functions
(`build_control_frame`, foreign-SA drop, CRC-fail pass-through) live in
`gs/src/dot11.{h,cpp}`, shared with `RadioFrontend` the same way. In GS mode
`WebGs` also runs `maburgs::ChannelCore` (`gs/src/channel_core.h`) over the
card roster. That covers the send gate, link-where-found, the boot scout,
the relocation and the one-card hop: the same object `maburgs` drives.

`web/src/web_main.cpp` is the glue. It builds a card roster of `maburgs`'s
own card classes:

- a `maburgs::RadioFrontend` (`gs/src/radio_frontend.{h,cpp}`): libusb with
  the WebUSB backend under Emscripten, native libusb1 on the host; its RX
  thread feeds a `BodyQueue`;
- or a `maburgs::RemoteCard` for the CPE relay.

It also runs the core loop driving `WebGs` over that roster. It builds as a
native CLI (`webgs live`/`webgs replay`, used for bench A/B against the
browser and for the parity gates) and, under `emcmake`, as the page's WASM
module. A FastRetune over WebUSB runs at native speed.

**CPE relay radio.** `maburgs::RemoteCard` (`gs/src/remote_card.{h,cpp}`) is
the radio-source alternative to the WebUSB path. It is the same card class
`maburgs` runs for `radio.relays`. It wraps `RelayClient` (the shared,
protocol-agnostic `gs/src` core) with a `RelayTransport` and its own RX
thread, feeding the same `BodyQueue` the USB path drains, so `WebGs` doesn't
know which radio fed it. The page sets `Cfg::restart_when_refused` false
(one relay, one client: a refusal is reported, not retried), and both modes
wait for ownership.

Native builds get `RelayTransport` over UDP directly. Under Emscripten it is
`web/src/relay_ring.h`'s SPSC byte ring instead, because a WASM pthread
can't own a browser WebSocket. The actual socket lives in the page's
dedicated `relay_worker.js` Worker. The ring is the SharedArrayBuffer bridge
between that Worker and the WASM core's RX thread. Its JS twin is
`web/ui/src/lib/relay_ring.js`, kept byte-identical and pinned by shared
golden vectors in both languages' test suites.

The core queues HELLO and TUNE into the TX ring immediately on start,
before the Worker's `WebSocket` has necessarily finished opening. The
Worker's loop therefore never spins synchronously waiting for `onopen`. Its
per-iteration logic is `workerStep()`, defined in `relay_ring.js`. The loop
parks on `Atomics.waitAsync` against the ring's state word instead, and
pumps the queued TX once the socket actually opens. The same loop honours a
core `STOP_REQ` written into that state word even while still connecting.
A Disconnect during the ~500 ms-2.5 s HELLO/TUNE dance therefore closes the
socket right away rather than waiting for it to open first.

## Follow-ups

Not built:

- A GS-relayed IDR request for Spotter mode, so a spotter opened mid-flight
  does not wait indefinitely for the drone's next unsolicited IDR. GS mode
  has its own periodic request ("Key-frame recovery" above).
- Foreign-GS detection, so a second GS-mode page against the same drone
  gets a warning instead of silently fighting the first one's ladder.
- Auto-reconnect after the card is unplugged and replugged, instead of
  requiring a manual Connect.
