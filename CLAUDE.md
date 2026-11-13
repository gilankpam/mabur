# CLAUDE.md

Guidance for Claude Code when working in this repository.

**Never create git worktrees** (no `.claude/worktrees/*`, no
`git worktree add`), even when a skill or plan-execution flow suggests one.
Branch in the main checkout instead. Split checkouts drift here — a PR
merges on GitHub while the main checkout sits behind, and merged work looks
missing.

**Code review:** do NOT run Claude Code's built-in `/review` /
`code-review` skill (nor `/code-review ultra`) on mabur work — too
expensive for diffs that are gated on tests plus an inline read. This
does NOT cover the superpowers workflows: `subagent-driven-development`'s
per-task implementer + spec/quality reviewer subagents are wanted and
should be used when a plan is executed that way.

mabur is an RTP-free FPV video link. This repo is the drone and the
protocol: `maburd` (drone, OpenIPC/SigmaStar, armv7) drives the SigmaStar
MI encoder in-process and injects — one daemon, no waybeam, since the
2026-08-29 venc fold-in (`drone/venc/`, ported from `../waybeam_venc`
f956a52; the binary is glibc-dynamic now, not musl-static). `common/` holds
the shared wire formats and FEC (encoder and decoder both — the decoder is
half of the protocol's definition); `third_party/devourer` (plus the
sibling checkout `../devourer`) is the userspace radio driver. PixelPilot
and the RTP output were deleted in PR C.

**The ground station lives in `gilankpam/mabur-ground`** (sibling checkout
`../mabur-ground`, its own CLAUDE.md): `maburgs`, `maburplay`, the web GS,
and their tools (maburtop, flightreport, ausniff, aucadence, gsparam,
`build-arm64.sh`). It pins this repo as the submodule `third_party/mabur`
and builds it as a CMake subproject (`MABUR_DIR`); keep mabur buildable
that way (`PROJECT_SOURCE_DIR`, never `CMAKE_SOURCE_DIR`, in `common/` and
`drone/`). Other GS implementations (palianytsia) also build against this
repo. CI's `ground-downstream` job builds mabur-ground master against every
PR here and runs its host suite, including the drone↔GS glue tests
(`test_cal_e2e`, `test_link_auth_e2e`, `test_frame_e2e`, `gs_e2e`,
`gs_au_e2e`, `player_e2e`, `web_au_parity`), which live there with the GS
code.

**A change that needs both sides** (a wire field, `RC_VERSION`, a caps bit,
a drone parameter row) is a PR here that links its mabur-ground PR. Write
the GS side against this branch with `-DMABUR_DIR=$PWD/../mabur`. Merge
here first; mabur-ground then bumps its submodule and merges. Until it
does, `ground-downstream` on master is red, which is the signal that the
pair is mid-flight.

## Where to look — read the doc, don't guess

This file is deliberately short. The detail lives in `docs/`; load the one
page the task needs rather than carrying all of it. The GS-only pages
(observability, colortrans, channel selection and hops, live config, the
goggles menu, the web GS, the OPi board, the CPE510 relay) are in
mabur-ground's `docs/` (`../mabur-ground/docs/`).

The pages here that describe link behaviour — `link-adaptation`,
`calibration`, `fec-nack`, `phase-lock`, `slices`, `bw40`, `link-pairing`,
`tx-rx-timing`, `airtime-model`, `deploy` — stay here even where much of the
code is in mabur-ground (ladder, cal session, NACK tracker, PhaseLock, slice
salvage): they are what any GS implementation reads. A mabur-ground PR that
changes behaviour one of them describes comes with a docs PR here, and
each links the other. Their GS code paths are mabur-ground GitHub links.

Two kinds of doc, two rules:

- **`docs/*.md` are reference pages: they describe the code at HEAD.** A
  change that alters behaviour, a default, a config key, a wire field or a
  log format fixes the page in the same commit. Present tense; no bench
  narratives, dated asides or SUPERSEDED chains — those go to findings.
- **`docs/findings/YYYY-MM-DD-topic.md` are dated records** (spikes,
  benches, flights, handovers), indexed in `docs/findings/README.md`. They
  are snapshots and are never edited to match later code; each carries a
  one-line `**Status:**` pointing at the reference page that owns the
  topic now.
- **Code comments say what the code does and why — no dates, no
  spec/plan citations.** Never cite `docs/superpowers/` specs or plans,
  `.superpowers/` reports, "spec section N", "Task 11", "fix round 2" or
  review ids: those files are machine-local, so state the rationale in the
  comment itself. No date stamps ("since 2026-09-16", "bench 2026-07-13"):
  a comment is true at HEAD or it is wrong; history belongs in git and
  `docs/findings/`. Code that still reads old recordings names the old
  format by its version marker, not its date. Links to committed docs
  (including dated `docs/findings/` paths) are fine.
- **No bench or host specifics in code, docs or commits.** Device IPs and
  ssh logins (drone, GS, spare boards), credentials, MACs, and this
  machine's paths/users/NICs stay out of the repo — they live only in
  Claude's memory (`device-addresses`). Write "the drone", "the GS",
  `root@<gs-ip>` or `$GS`; bench scripts take targets from the
  environment (`DRONE=`, `GS=`) with no default. Product addresses the
  software itself ships with (the CPE relay's firmware-fixed `10.83.11.1`,
  loopback ports) and synthetic test values are not bench specifics.

| If the task touches… | Read |
|---|---|
| ladder rungs, promote/demote, s3 probes, fade, attribution, RCF drain, RcAgent's encoder verbs + IDR pacing | `docs/link-adaptation.md` |
| the disarmed low-power mode, arm state over MSP, the live fps verb | `docs/link-adaptation.md` ("Low-power (disarmed) mode") |
| the GS itself: sideport, maburtop, session logs, ausniff, player OSD/DVR, colortrans, channel selection and in-flight hops, `ChannelCore`, live GS config, the goggles menu, the web GS / spotter, the OPi Zero 2W, the CPE510 relay | mabur-ground's `CLAUDE.md` and `docs/` (`../mabur-ground/`) |
| comparing recordings, metric scales, removed sideport keys, "why do these two flights disagree" | `docs/data-provenance.md` |
| shipping a binary or config to a device | `docs/deploy.md` |
| calibrating a VTX's TX-power walls and capacity / efficiency tables (per width), maburcal, cal.log, the mabur MAVLink dialect and its cal lane | `docs/calibration.md`; the dialect and the run-pair latch → `docs/mavlink.md` |
| a specific past investigation, or why something is the way it is | `docs/findings/README.md` (dated index) |
| camera modes and field of view per `venc.fps` (1920x1080@90 full view, 1728x972@100, 1728x816@120), allowed `venc.size`, binning, the waybeam IMX415 driver the image ships, matching the GS `screen_mode` / the fps mismatch warning / letterboxing, SD-recorder risks above 90 fps, the GS burned recorder at 100 fps | `docs/camera-modes.md` |
| airtime/serialization math, per-stream FEC overhead, why jitter ∝ frame size, the encoder bitrate formula, encoder size knobs (max_ipprop, min_iqp, superframe_p_pct, `ref_*`/intra-refresh, dead SDK caps) | `docs/airtime-model.md` |
| half-duplex timing: who transmits when, the slotter, airtime budget, why RCFs get lost | `docs/tx-rx-timing.md` |
| boot time, the OpenIPC init chain, U-Boot/serial console on the drone | `docs/findings/2026-09-07-boot-time.md` |
| carrier sense (CCA) on/off, the GS-send self-collision, the drone's RX channel view (`drone.radio.rx`), why the OSD LOSS row reads what it reads, the pending cca-on flight gate | `docs/findings/2026-09-23-cca-on.md` |
| 40 MHz (HT40): which rungs fly 40, per-width tables, pair candidates, the boot scan's pair pick, no_agg, scanlog 6 / ctllog 12, the derived receiver width | `docs/bw40.md` (as built: per-rung width, pairs, scan, no_agg) + `docs/findings/2026-09-23-bw40-sweep.md` ("Start here") |
| the VTX onboard SD recorder ([record], dvr.target, the RCF rec byte, OSD REC field), VENC bind-peer order, the MI /proc/mi_modules/mi_venc per-stage delay instrument, the startup vanish-counter ratchet | docs/vtx-recorder.md (as built) + docs/findings/2026-09-26-sd-record.md (spike) |
| pairing / the link key file, SipHash tag, session nonces, KEY MISMATCH, auth_reject | docs/link-pairing.md |
| the software NACK (base-layer selective repeat): T_NACK, the retx ring, the air bucket, the retx accounting class, feclog 3 | docs/fec-nack.md (as built) + docs/findings/2026-10-05-fec-nack-spike.md (spike) |
| H.265 row slices: slice salvage (SliceAssembler, skip-slice fills, `[venc] slices`, FrameHdr `slice_rows`, `kRecFlagSliceSalvaged`, aulog 5) and streamed decode (AU ring v4 open slots, StreamFeeder, the decoder-input thread (FeedLoop), `[decoder] stream`, the MPP patches in [`gs/player/mpp-patches/`](https://github.com/gilankpam/mabur-ground/blob/master/gs/player/mpp-patches/), the stream-mode GS image and its p1 flash) | docs/slices.md (as built) + docs/findings/2026-10-09-venc-slice.md; flashing: docs/deploy.md |
| the phase lock (sensor-rate servo against the GS vblank: PhaseLock, the regulator's margin and placement switch, RCF `snr_off`, `display.phase_lock`/`phase_margin_ms`, `screen_mode` exact-rate pick, reg.log `lock=` fields, drone `[sensor] steer`) | docs/phase-lock.md + docs/findings/2026-10-17-phase-lock-spike-2.md |
| MAVLink tunnel (`T_MAV`, ids, the drone component's routing/heartbeat), the GS MAVLink UDP endpoint and its trust boundary | docs/mavlink.md |
| drone config parameters over MAVLink (`PARAM_EXT_*` table, applying a set, `.pre-param`, ordering rules, `ParamClient`, mavparam.py) | docs/vtx-config-mavlink.md; the web GS VTX tab → mabur-ground's [`docs/web-gs.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/web-gs.md) |
| radio/PHY below mabur | `third_party/devourer/CLAUDE.md` |

Design specs and plans live in `docs/superpowers/` — **gitignored**, so
they are on this machine only and anything that must survive belongs in a
committed `docs/` page instead. Never cite them from code comments.

## Compatibility policy — there isn't one

One drone, one GS, one operator, and both ends always run the latest
build. mabur owes nothing to old peers, old configs, or old consumers.

- **Wire formats are free to change.** Bump `RC_VERSION`, resize the Telem
  struct, drop dead RCF fields. No deprecation cycle, no reserved bytes,
  no placeholder fields kept "so a peer could still refuse us" — delete
  dead weight instead of preserving it for a peer that will never exist.
- **The sideport schema, and the debug-log session layout it pairs with,
  are NOT additive-only.** Remove, rename and re-type sideport keys under
  `v: 1` as convenient; `v` does not need bumping. The per-session log
  files (`ctl.log`, `probe.log`, `au.log`, `flight.jsonl`, `lat.log`) are
  under the same rule — their own format-marker lines (`ctllog N`,
  `aulog N`, `probelog N`, `latlog N`) carry the version instead. The
  obligation is to fix the consumers — `tools/maburtop.py`,
  `tools/flightreport.py` (which reads both the sideport jsonl and the
  session directory's log files), `gs/player`'s OSD, all in mabur-ground —
  **in the same mabur-ground commit** that changes the format, not to keep
  the old shape alive.
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
resolve. Host suite: `ctest --test-dir build -R 'test_|sim_|host_e2e'`.
Cross-build: `tools/build-arm.sh` (maburd + the bench TX tools, OpenIPC
glibc toolchain at `../openipc-builder`). The GS cross-build
(`build-arm64.sh`) is in mabur-ground.

**The gate for a change here** is this suite plus `ground-downstream`
(mabur-ground's host suite against this branch). Any change touching venc,
UEP overhead, or the bitrate policy additionally gates on mabur-ground's
`tools/bench/aucadence.py` (base−enh completion offset vs the recorded
per-rung baseline); a change the GS sees on the air gates on its
`tools/bench/ausniff.py` against a deployed pair. Details in mabur-ground's
[`docs/observability.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/observability.md).
