# mabur

`mabur` turns an SSC338Q (SigmaStar Infinity6E) FPV camera into an adaptive
video transmitter. It bridges **waybeam** (H.265/SVC-T encoder, unmodified,
`../waybeam_venc`) and **devourer** (RTL8812EU raw-injection driver, used as a
library): `maburd` reads whole encoded frames off waybeam's frame-shm ring,
classifies each frame by temporal layer, applies per-layer sliding-window UEP
FEC with SBI framing (the window sealed at every frame boundary), injects the
bodies with per-layer modulation/power, consumes the ground-station
adaptive-link feedback (RC/RCF), and drives waybeam's encoder knobs (bitrate,
ROI QP, IDR) in response. The ground station publishes reassembled access
units to a shm ring consumed by maburplay (MPP hardware decode to DRM/KMS,
plus the fMP4 DVR); there is no RTP anywhere in the system.

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

`tools/build-arm.sh` produces a fully static `armv7l-unknown-linux-musleabihf`
`maburd` binary at `out/arm/maburd`, ready to copy onto the camera. It is
self-contained: it stages its own cross compiler, `pkg-config`, static
`libusb1`, and `file`, all via Nix (`pkgsCross.armv7l-hf-multiplatform.pkgsStatic`),
independent of whatever toolchain is or isn't on `PATH`. No `nix-shell`
wrapper is needed — just run it directly:

```sh
bash tools/build-arm.sh
```

First run fetches/builds the toolchain and caches it under `toolchain/`
(gitignored, but pinned as a Nix GC root so it survives
`nix-collect-garbage`); subsequent runs are fast incremental rebuilds. See the
comments at the top of the script for why this diverges from the original
Bootlin-toolchain plan (NixOS doesn't have `/lib64/ld-linux-x86-64.so.2`, so
prebuilt dynamically-linked toolchains can't execute here).

## Deploy to a camera

```sh
bash bundle/install.sh root@<camera-ip>
```

This copies `out/arm/maburd` to `/usr/bin/maburd`, seeds `/etc/mabur.json`
from `bundle/mabur.default.json` if one isn't already present, installs
`bundle/S96mabur` (a BusyBox-compatible init script with a respawn loop) to
`/etc/init.d/S96mabur`, configures waybeam's `frame-shm://` output via
`json_cli`, and (re)starts both `S95waybeam` and `S96mabur`.

Prerequisites: `bash tools/build-arm.sh` has been run so `out/arm/maburd`
exists, and the target is an OpenIPC-based SSC338Q already running waybeam
with `json_cli` available. The `json_cli` flag names in `install.sh` are
unverified against a live device — confirm them against `../waybeam_venc/tools`
on the first real deploy.

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
1 Hz) as their own FEC-protected air stream. The GS re-emits the MSP bytes over
UDP to `msp.out` (default `127.0.0.1:14560`); render them with msposd on the
GS, e.g. `msposd 127.0.0.1:14560 --osd`, which draws to the shm surface
PixelPilot composites over the video. One-way display only — see
`docs/superpowers/specs/2026-07-15-msp-displayport-osd-design.md` for the
drone-side-burn-in and GS→FC-menu future doors.

`maburgs` only reassembles snapshots and re-emits them over UDP — it no
longer renders pixels itself (the old `msp.render = "shm"` PixelPilot
shared-memory path was removed with PixelPilot; `maburplay` now renders the
OSD directly from the UDP snapshot feed onto its DRM overlay plane).

## Benchmarking

On-target and PC-side bench tooling (loss/recovery measurement against
captured or live RF) lives under `tools/bench/`; see
`tools/bench/decode_bodies.py` and the spec's Testing section for the full
bench/on-target validation procedure (rings 3-4), which requires real
hardware and isn't part of the automated test suite above.
