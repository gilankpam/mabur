# CLAUDE.md

Guidance for Claude Code when working in this repository.

mabur is an RTP-free FPV video link: `maburd` (drone, OpenIPC/SigmaStar,
armv7) drives the SigmaStar MI encoder in-process and injects — one daemon,
no waybeam, since the 2026-08-29 venc fold-in (`drone/venc/`, ported from
`../waybeam_venc` f956a52; the binary is glibc-dynamic now, not musl-static);
`maburgs` (ground station, aarch64) receives,
FEC-decodes, and publishes whole access units to a shm AU ring; `maburplay`
(gs/player/, same GS binary family) consumes the ring — MPP hardware decode
straight to DRM/KMS, plus the fMP4 DVR on /media/dvr. `common/` holds the
shared wire formats and FEC; `third_party/devourer` (plus the sibling
checkout `../devourer`) is the userspace radio driver. PixelPilot and the
RTP output were deleted in PR C.

## Where to look — read the doc, don't guess

This file is deliberately short. The detail lives in `docs/`; load the one
page the task needs rather than carrying all of it.

| If the task touches… | Read |
|---|---|
| ladder rungs, promote/demote, s3 probes, fade, attribution, RCF drain, RcAgent's encoder verbs + IDR pacing | `docs/link-adaptation.md` |
| the stats sideport, maburtop, recorders, ausniff, capture tools, player OSD/DVR/record button | `docs/observability.md` |
| comparing recordings, metric scales, removed sideport keys, "why do these two flights disagree" | `docs/data-provenance.md` |
| shipping a binary or config to a device | `docs/deploy.md` |
| a specific past investigation | the dated `docs/*-findings-*.md` / `docs/handover-*.md` |
| airtime/serialization math, per-stream FEC overhead, why jitter ∝ frame size, encoder size knobs (max_ipprop, presets, dead SDK caps) | `docs/airtime-model.md` |
| radio/PHY below mabur | `third_party/devourer/CLAUDE.md` |

Design specs live in `docs/superpowers/specs/` — **gitignored**, so they
are on this machine only and anything that must survive belongs in a
committed `docs/` page instead.

## Compatibility policy — there isn't one

One drone, one GS, one operator, and both ends always run the latest
build. mabur owes nothing to old peers, old configs, or old consumers.

- **Wire formats are free to change.** Bump `RC_VERSION`, resize the Telem
  struct, drop dead RCF fields. No deprecation cycle, no reserved bytes,
  no placeholder fields kept "so a peer could still refuse us" — delete
  dead weight instead of preserving it for a peer that will never exist.
- **The sideport schema is NOT additive-only.** Remove, rename and re-type
  keys under `v: 1` as convenient; `v` does not need bumping. The
  obligation is to fix the in-repo consumers — `tools/maburtop.py`,
  `tools/flightreport.py`, `gs/player`'s OSD — **in the same commit**, not
  to keep the old shape alive.
- **Config keys are free to change.** A removed key failing boot is the
  intended forcing function, not a regression. No migration shims.
- **There is no rollback contract.** An old binary needs its old config
  restored alongside it, always. Prefer rolling forward.

Three things this does NOT excuse, because they bite a single-operator
deployment just as hard as a fleet:

1. **Deploy is two devices and is never atomic.** Any wire change is a
   flag day: between swapping `maburd` and swapping `maburgs` the pair is
   mismatched, and a mismatched pair has no control link and — since
   DISC_ACK carries `CAP_FRAME_WIRE` — no video at all, which looks exactly
   like the stale-caps deadlock and will send you to `restart maburd`. That
   does not help; finish the deploy. See `docs/deploy.md`.
2. **Config-before-binary, still.** A daemon that sees an unknown key
   exits, and its wrapper respawns it forever at 2 s. See `docs/deploy.md`.
3. **Recordings outlive the code that wrote them.** `flightreport.py` reads
   jsonl and ctl logs from earlier builds, so format markers (`ctllog N`),
   the dated scale-break notes and the list of removed sideport keys all
   stay — they document data sitting on the DVR, not a promise to anyone.
   See `docs/data-provenance.md`.

## Build & test

NixOS host: wrap every cmake/ctest invocation in
`nix-shell -p pkg-config libusb1 --run "..."`. Configure with
`-DDEVOURER_DIR=$PWD/../devourer` when the default `../devourer` doesn't
resolve. Host suite: `ctest --test-dir build -R 'test_|host_e2e'`.
Cross-builds: `tools/build-arm64.sh` (maburgs), `tools/build-arm.sh`
(maburd + the bench TX tools, OpenIPC glibc toolchain at
`../openipc-builder`). Devices: drone `root@192.168.10.152`, GS
`root@10.18.0.1`.
Never load the 8812eu kernel module on the GS cards.

**The standing regression gate for any maburgs change is
`tools/bench/ausniff.py`**, which reads the AU ring from outside the
daemon — the sideport is maburgs self-reporting, so gating a maburgs
change on the sideport is circular. Host-side the equivalent is
`ctest -R 'gs_e2e|gs_au_e2e|player_e2e'`. Details in
`docs/observability.md`.
