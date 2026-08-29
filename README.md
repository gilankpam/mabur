# mabur

`mabur` turns an SSC338Q (SigmaStar Infinity6E) FPV camera into an adaptive
video transmitter. `maburd` owns the whole drone side in one process: it
drives the SigmaStar MI encoder pipeline directly (H.265/SVC-T, the venc
core ported from **waybeam** at `../waybeam_venc` f956a52 and folded in on
2026-08-29 — waybeam itself is retired), classifies each frame by temporal
layer, applies per-layer sliding-window UEP FEC with SBI framing (the
window sealed at every frame boundary), injects the bodies with per-layer
modulation/power through **devourer** (RTL8812EU raw-injection driver, used
as a library), consumes the ground-station adaptive-link feedback (RC/RCF),
and actuates the encoder knobs (bitrate, ROI QP, IDR) in response — as
in-process function calls now, not HTTP to a second daemon. The ground
station publishes reassembled access units to a shm ring consumed by
maburplay (MPP hardware decode to DRM/KMS, plus the fMP4 DVR); there is no
RTP anywhere in the system.

The full design — architecture, wire formats, protocol, and the testing
strategy — is in
[`docs/superpowers/specs/2026-07-10-mabur-system-bundle-design.md`](docs/superpowers/specs/2026-07-10-mabur-system-bundle-design.md).
Start there for anything beyond "how do I build/run this."

This repo builds `maburd` and its deploy bundle. The ground station (GS)
adaptive-link controller is out of scope for v1; `common/` holds the
shared wire-format code so a GS can be added to this repo later.

## Host build + test

Requires `pkg-config` and `libusb1` (used by devourer's build even on the
host, where the real USB device isn't touched by the unit/E2E tests). On
NixOS, or any system without those preinstalled, wrap commands in
`nix-shell -p pkg-config libusb1 --run "..."`:

```sh
nix-shell -p pkg-config libusb1 --run "
  cmake -S . -B build && cmake --build build -j && ctest --test-dir build --output-on-failure -R 'test_|host_e2e'
"
```

This builds `maburd` plus the unit test suites and runs them all (14 suites +
`host_e2e`). The `-R 'test_|host_e2e'` filter selects mabur's own test suites;
devourer's bundled selftests are known-broken upstream and out of scope. To run only the host end-to-end scenario (clean pipe, 20% body
loss, RCF profile switch, full 4-stream recovery) directly:

```sh
bash tests/integration/run_host_e2e.sh
```

## Regenerating test vectors

Golden vectors for FEC/SBI/frag round-trip tests live under `tests/vectors/`
and are generated from `tools/genvectors/gen_vectors.py`:

```sh
python3 tools/genvectors/gen_vectors.py
```

Re-run this after changing any wire format the vectors cover, then re-build
and re-test to confirm the generated vectors still round-trip.

## Cross build (ARM target)

`tools/build-arm.sh` produces the drone-side binaries with the OpenIPC
Buildroot toolchain: `maburd` at `out/arm/maburd`, plus the bench TX
harnesses `out/arm/linkbench-tx` and `out/arm/txagcbench-tx`. `maburd` is a
DYNAMIC glibc `armv7-a` hard-float executable (`libstdc++`, `libm`,
`libgcc_s`, `libc`, `ld-linux-armhf` — all present on the OpenIPC rootfs,
with `libstdc++.so.6` under `/usr/lib`). It stopped being a static musl
binary on 2026-08-29, when the venc fold-in put the SigmaStar MI vendor
libraries — prebuilt glibc shared objects — inside the process; the musl
script and `cmake/arm-musl.cmake` were deleted in the same commit. `libusb`
is still linked statically, since the drone rootfs does not ship it.

```sh
bash tools/build-arm.sh
```

Prerequisite: the OpenIPC Buildroot toolchain at
`../openipc-builder/openipc/output/host/bin` (override with
`OPENIPC_HOST_BIN`). The script stages `pkg-config` and a static ARM
`libusb` itself, caching them under `toolchain/` (gitignored, pinned as a
Nix GC root so they survive `nix-collect-garbage`); subsequent runs are
fast incremental rebuilds. No `nix-shell` wrapper is needed.

## Deploy to a camera

```sh
bash bundle/install.sh root@<camera-ip>
```

This copies `out/arm/maburd` to `/usr/bin/maburd`, seeds `/etc/mabur.json`
from `bundle/mabur.default.json` if one isn't already present, installs
`bundle/S96mabur` (a BusyBox-compatible init script with a respawn loop) to
`/etc/init.d/S96mabur`, and (re)starts `S96mabur`. There is no waybeam step
any more: since 2026-08-29 `maburd` owns the encoder itself, and a camera
that still runs waybeam must have it retired first — see `docs/deploy.md`.

Prerequisites: `bash tools/build-arm.sh` has been run so `out/arm/maburd`
exists, and the target is an OpenIPC-based SSC338Q with the SigmaStar MI
libraries in its rootfs.

Logs: `ssh root@<camera-ip> 'cat /tmp/mabur.log'` (or via serial console if
networking is down).

### Config notes

`radio.channel` and `radio.width` are both validated by `load_config()`, but
in v1 the radio is always tuned at 20 MHz regardless of `radio.width` — a
configured 40/80 is accepted (no config error) but ignored, logging a
startup warning (`radio.width=N not supported in v1, using 20 MHz`) instead
of silently doing something the config didn't ask for. 40/80 MHz tuning
support is not implemented in v1.

## MSP DisplayPort OSD (ground-side)

Set `msp.enable=true` on the drone (`mabur.json`) and GS (`maburgs.json`);
`msp.symbol_size`/`msp.window` must match on both ends. The drone taps the
flight controller's MSP DisplayPort UART (`msp.serial`, default `/dev/ttyS2`)
and forwards full-screen keyframe snapshots at `msp.update_rate_hz` (default
1 Hz) as their own FEC-protected air stream. `maburgs` reassembles the
snapshots and re-emits the MSP bytes as loopback UDP datagrams to `msp.out`
(default `127.0.0.1:14560`) — it renders no pixels itself. `maburplay`
receives that feed on `osd.port`, rasterizes the character grid with a
runtime glyph atlas and shows it on a DRM overlay plane above the decoded
video. Neither msposd nor PixelPilot is involved any more (both they and
the old `msp.render = "shm"` shared-memory path are gone). One-way display
only — see `docs/superpowers/specs/2026-07-15-msp-displayport-osd-design.md`
for the drone-side-burn-in and GS→FC-menu future doors.

Player-side config lives in `maburplay.json` under `osd`: `enable`, `port`,
`font` (path to a `.mfont` atlas), `scale` (`sharp` | `fill`) and `stale_ms`
(blank the overlay after this much silence; keep it at several multiples of
the drone's `msp.update_rate_hz` period, default 5000 ms).

Two deploy-time invariants, neither of which a daemon can check for itself:

- **`osd.port` (maburplay) must equal `msp.out.port` (maburgs).** They are
  separate processes with separate config files. On a mismatch the OSD stays
  empty; maburplay logs a one-shot warning naming the bound port ~10 s after
  start.
- **The `.mfont` atlas named by `osd.font` must exist on the GS**, by
  default `/usr/local/share/mabur/font_btfl.mfont`. The generated Betaflight
  36x54 atlas is committed as `gs/player/bundle/font_btfl.mfont` and staged
  into `out/arm64/` by `tools/build-arm64.sh`; push it alongside the
  binaries. Regenerate it from an msposd checkout next to this repo with:

  ```
  tools/msp/gen_font.py ../msposd/fonts/original/betaflight/font.png \
      gs/player/bundle/font_btfl.mfont
  ```

  If the file is missing, maburplay logs `osd disabled -- cannot open ...`
  and runs video-only.

## GS link-status OSD (player-side)

A second, independent overlay drawn by maburplay from the maburgs stats
sideport: rung/airtime, per-card RSSI+SNR, the pre/post-FEC loss pair,
fps/jitter/bitrate and the DVR recording clock, hugging the four corners so
the centre of frame stays clear. It shares the MSP overlay's plane and
surface (MSP draws first, then the GS fields repaint over any collision, so
GS pixels always win), and it is burned into the DVR alongside the MSP grid.

Config lives in `maburplay.json` under `osd.gs`: `enable` (default false),
`port` (the sideport fan-out destination maburgs sends to — see `stats.out`
in `maburgs.json`), `font` (path to a `.gfont` atlas) and `stale_ms` (dim
every link-derived field after this much silence; the player-measured fields
ignore it, being current by construction).

- **The `.gfont` atlas named by `osd.gs.font` must exist on the GS**, by
  default `/usr/local/share/mabur/gs_osd.gfont`. It is committed as
  `gs/player/bundle/gs_osd.gfont` and staged into `out/arm64/` by
  `tools/build-arm64.sh`; push it alongside the binaries, exactly like the
  `.mfont`. Both fonts are mmapped by a running maburplay, so stop the
  player (`/etc/init.d/S97maburplay stop`) before overwriting either — an
  in-place `scp` over a live mapping is how you get a SIGBUS instead of a
  new font.
- The asset bakes **30 pixel sizes**, the eight design sizes at ×⅔, ×1,
  ×4/3 and ×2, so every type role lands on an exact atlas size at 720p,
  1080p, 1440p and 2160p with no runtime scaling anywhere. That is what
  makes it ~13 MB — a mask atlas, two bytes per pixel, at every size.
  Regenerate it (JetBrains Mono is a dev-host dependency only; the asset is
  committed so a deploy needs no font toolchain) with:

  ```
  nix-shell -p jetbrains-mono python3Packages.freetype-py --run \
    "tools/msp/gen_gsfont.py \
       <jetbrains-mono>/share/fonts/truetype/JetBrainsMono-Medium.ttf \
       gs/player/bundle/gs_osd.gfont --sizes <the 30-value list>"
  ```

  The exact size list and expected byte count are in `gen_gsfont.py`'s
  header; `tests/test_gs_asset.cpp` is the gate that the committed asset
  still bakes exactly those sizes and that the layout holds against real
  JetBrains Mono metrics at all four resolutions.

**When the overlay is on screen.** It appears with the first decoded frame
and stays there for the rest of the session, including through video loss.
The CRTC-active latch it is gated on (`drm_presenter.cpp`, set once on the
first modeset and never cleared) is deliberately *not* the "video is
healthy" flag: an AU-ring discontinuity, a decoder wedge, the decode
watchdog dropping every queued frame and recreating the backend all re-arm a
modeset without deactivating the CRTC, and the OSD keeps committing on its
own throughout — which is exactly when on-screen link telemetry is worth
having. The one dead window is between GS power-on and the first decoded
frame, where the CRTC is not up and an OSD-only commit would have nothing to
display on.

**Adding verbose/debug/off levels.** Essential is one of four levels in the
design handoff (essential / verbose / debug / off, operator-cycled), and the
handoff's constraint on the other three is that **shared anchors keep
identical pixel positions across levels**: switching level may add or remove
blocks, but it must not move the four this branch ships. Recorded here
because it constrains code that does not exist yet, and the handoff itself
is not in the repository.

## Benchmarking

On-target and PC-side bench tooling (loss/recovery measurement against
captured or live RF) lives under `tools/bench/`; see
`tools/bench/decode_bodies.py` and the spec's Testing section for the full
bench/on-target validation procedure (rings 3-4), which requires real
hardware and isn't part of the automated test suite above.
