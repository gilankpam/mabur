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

This repo builds `maburd`, its deploy bundle, and `common/`: the wire
formats and FEC every ground station builds against. Our ground station
(`maburgs`, `maburplay`, the web GS) is
[mabur-ground](https://github.com/gilankpam/mabur-ground), which pins this
repo as a submodule.

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
harness `out/arm/linkbench-tx`. `maburd` is a
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
`firmware/openipc/output/host/bin`, left there by a firmware build
(`firmware/builder.sh <board>`; override with
`OPENIPC_HOST_BIN`). The script stages `pkg-config` and a static ARM
`libusb` itself, caching them under `toolchain/` (gitignored, pinned as a
Nix GC root so they survive `nix-collect-garbage`); subsequent runs are
fast incremental rebuilds. No `nix-shell` wrapper is needed.

## Deploy to a camera

```sh
bash bundle/install.sh root@<camera-ip>
```

This copies `out/arm/maburd` to `/usr/bin/maburd`, seeds `/etc/mabur.toml`
from `bundle/mabur.default.toml` if one isn't already present, installs
`bundle/S96mabur` (a BusyBox-compatible init script with a respawn loop) as
`/etc/init.d/S00mabur` — the name the OpenIPC image uses — removing any old
`S96mabur`, and (re)starts it. There is no waybeam step
any more: since 2026-08-29 `maburd` owns the encoder itself, and a camera
that still runs waybeam must have it retired first (the header of
`bundle/install.sh` says how).

Prerequisites: `bash tools/build-arm.sh` has been run so `out/arm/maburd`
exists, and the target is an OpenIPC-based SSC338Q with the SigmaStar MI
libraries in its rootfs.

Logs: `ssh root@<camera-ip> 'cat /tmp/mabur.log'` (or via serial console if
networking is down).

### Config notes

`radio.width` is honoured (20 or 40); at 40 the drone tunes the standard 5 GHz
pair of `radio.channel` and airs each rung at the width the GS ladder names
(`docs/bw40.md`).

## Benchmarking

On-target and PC-side bench tooling (loss/recovery measurement against
captured or live RF) lives under `tools/bench/`; see
`tools/bench/decode_bodies.py` and the spec's Testing section for the full
bench/on-target validation procedure (rings 3-4), which requires real
hardware and isn't part of the automated test suite above.
