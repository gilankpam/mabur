# maburgs — Handover: What's Done, What's Next, What to Validate

**Audience:** the next agent (or operator) taking `maburgs` from code-complete to
bench-accepted. Read this first, then `docs/superpowers/specs/2026-07-12-mabur-gs-design.md`
(design) and `docs/bench-validation.md` (bench log + the E-series this parallels).

**Branch:** `gs-control-plane` (stacked on `gs-datapath` = Plan 1). Both plans are on
this branch; neither is merged to `master` yet.

---

## 1. Current state (what is DONE)

`maburgs` is **code-complete** for v1. Plans 1 and 2 are implemented, task-reviewed,
and whole-branch-reviewed (both "ready to merge: yes", no Critical/Important open).

- **Plan 1 (datapath):** air frames → byte-exact RTP. SBI unpack → per-layer RS decode
  → FRAG reassembly, multi-card union/dedup at the FEC-symbol level, UDP RTP out, the
  `node::RxBody` wire format. Dry-run mode replays maburd frame files.
- **Plan 2 (control plane):** the full adaptive-link brain, byte/decision-exact ports of
  devourer's Python (replay-pinned): generated `p_deliver` table → energy model →
  LinkTable/op-table → ScoreWindow/RungWindow → energy-min Controller → VrxRendezvous →
  VrxController → TxSelector → BodyQueue → RadioFrontend (devourer per-card) → real-radio
  `main` → aarch64 static build + Radxa systemd bundle.

**Verified in software (no hardware):**
- All **35 mabur ctest suites pass** (Plans 1+2), including the controller replay-parity
  tests and the `gs_e2e` / `host_e2e` integration gates.
- Full build **links** against devourer/libusb (maburd + maburgs + tests).
- Real-radio mode **fails gracefully with no cards** (open-fails, retries, SIGINT-clean).
- **Static aarch64 binary builds**: `tools/build-arm64.sh` → `out/arm64/maburgs`
  (ELF aarch64, statically linked, stripped; `ldd` → not a dynamic executable).

**Deliberate v1 behaviors (so you interpret bench output correctly):**
- **Keep-alive DISC**: in SESSION, a DISC is sent ~every `beacon_keepalive_ms` (1000 ms)
  in place of that tick's RCF. A LINKED drone ignores it; a drone that silently fell back
  to rendezvous re-links immediately. **This is the fix for the 40–165 s proto-GS re-link
  pathology — G3 validates it.**
- **Control frames always fly at MAX_RANGE TxMode** (HT MCS0 / 20 MHz / LDPC+STBC).
- **Real 12-bit 802.11 hw seq** is the video seq (proto-GS used SBI magic bytes = junk).
- **`bw_set = [20]` (single rung).** RungWindow/rung-blocking is ported but **INERT** in
  v1. Do not expect rung-blocking behavior; see §6 before enabling multi-bw.

---

## 2. What's LEFT = Task 13, the hardware bench gate (G1–G7)

**No code is required for acceptance** — Task 13 is the on-air validation with real
hardware. This is the only remaining work to declare the plan DONE. It cannot be done in
software; it needs the physical rig.

### Hardware / rig
- **Drone:** SSC338Q camera running `maburd` (the proven drone bring-up; see
  `docs/bench-validation.md` E-series — E1/E2/E4/E5 already PASS on the proto-GS).
- **GS:** a PC (first) then the **Radxa Zero 3W** (G7). Needs **8812EU** cards
  (`0bda:a81a`). **TX+RX duplex is mandatory** — 8822E RX-only bring-up is near-deaf
  (devourer bug, bench 2026-07-12). **8812AU is disqualified as a receiver** (lossy — it
  failed E1 strict; do not use it for the RX path).
- Video sink: anything that plays RTP on UDP `:5600` — `ffplay`, gstreamer, PixelPilot.

### Build & deploy (exact commands)
```bash
# 1. Build the static aarch64 binary (NixOS host; first run fetches the cross toolchain, minutes)
bash tools/build-arm64.sh                 # -> out/arm64/maburgs (ELF aarch64, static)

# 2. On a PC GS you can run the native build directly:
nix-shell -p pkg-config libusb1 --run "cmake --build build -j"
build/gs/maburgs -c gs/bundle/maburgs.default.json         # radio mode is the DEFAULT (no --dry-run)

# 3. Deploy to the Radxa (binary + unit + config; never clobbers a tuned /etc/maburgs.json):
bash gs/bundle/install.sh root@<radxa-ip>                  # optional 2nd arg: path to binary
#   installs /usr/local/bin/maburgs, /etc/systemd/system/maburgs.service, /etc/maburgs.json,
#   then daemon-reload + enable --now + status
journalctl -u maburgs -f                                   # watch its stats/log on the target
```

### Config (`/etc/maburgs.json`, default in `gs/bundle/maburgs.default.json`)
```
radio: channel 149, width 20, cards [8812EU auto-scan], tx_card -1 (auto-select)
fec:   k 8, symbol_size 64, block_max_age_ms 2000
link:  vtx_id 1, feedback_ms 100, beacon_keepalive_ms 1000, video_silence_ms 3000
video_out: 127.0.0.1:5600
```
For **G5 (dual card)** add a second card entry to `radio.cards`. For a pinned TX card set
`radio.tx_card` to its index (default −1 = auto-select best card).

---

## 3. The validation gates (record each; G1–G7 gate DONE, G8 is record-only)

Run in order. Each has an E-series parallel already passing on the proto-GS, so a failure
means a GS regression, not a drone problem.

| Gate | What to do | PASS criteria | How to verify |
|---|---|---|---|
| **G1** PC single card (E1 parity) | One 8812EU, drone streaming | RENDEZVOUS→SESSION; drone **LINKED** (`stats: state=2` on the drone); RTP on `:5600` playable; **post-FEC 0.000% on streams 0/1** at bench range | `ffplay`/gst on `:5600`; drone stat line `state=2`; maburgs stderr per-stream `unrec=0`, `delivery=100%` |
| **G2** commands on air (E2 parity) | Let controller adapt | Drone TX rate follows the controller's rungs; RCF `pwr_idx`/overhead applied | Monitor capture: drone MCS tracks maburgs `op` line; drone log shows applied pwr/overhead |
| **G3** feedback kill (E4 parity) + **re-link fix** | Kill maburgs, then restart | Drone → **FAILSAFE ≤ 1.2 s**; on restart, **re-link < 5 s over 5 trials** (the 40–165 s proto-GS pathology **must be gone**); keep-alive DISC visible at ~1 s cadence in a capture | Time kill→failsafe; 5× restart trials, all < 5 s; sniff DISC cadence |
| **G4** cold rendezvous (E5 parity) | Reboot drone under a running maburgs | Drone links in **~2 s** | Time reboot→LINKED |
| **G5** dual card | Two 8812EUs; then pull TX card antenna / unplug; then unplug+replug a card | `stats:` shows **both cards up**; `tx_card` **fails over**, link holds; replugged card **front-end reopens within ~2 s backoff, no process restart** | maburgs per-card stats; observe failover + reopen without restarting the daemon |
| **G6** E3 degradation staircase (**the item the proto-GS blocked**) | Attenuate, or run devourer `tests/sdr_interferer.py` | Controller **walks the ladder down** (MCS/overhead/power); **T2/T1 delivery collapse first** (RCF `layer_delivery`); drone bitrate follows **≤ 2 s**; **video never freezes**; recovery **walks back up with hysteresis, no flapping** (`hold_after_downgrade_ms` visible) | Watch maburgs op line + RCF `layer_delivery`; drone bitrate trace; confirm no video freeze; confirm no rung flapping |
| **G7** Radxa deploy | `build-arm64.sh` + `install.sh`; reboot the Radxa | systemd unit **survives reboot**; **G1 re-passes on the Radxa**; record CPU budget (`top`) and **USB stability with 2 cards on the Zero 3W** (a spec risk item) | `systemctl status maburgs` after reboot; re-run G1; note CPU% + any USB dropouts |
| **G8** range follow-ups (**record, don't block**) | At range | Path-B-AGC desense A/B (`DEVOURER_PROTECT_PATHB_AGC`, TX+RX mode — carried from 2026-07-12 notes); TX-selector reciprocity sanity | Record observations only |

### Recording results
Add a new **"maburgs"** section to `docs/bench-validation.md`, mirroring the E-series
entry style (e.g. the `### E1 … STRICT PASS` / `**E2 PASS.** …` format already in that
file). Mark each gate PASS/FAIL with the observed numbers (link time, post-FEC %, re-link
trials, CPU%, etc.), then commit.

---

## 4. Pre-bench software sanity (do this before touching hardware)

Cheap confidence that the build you're deploying is good:
```bash
nix-shell -p pkg-config libusb1 --run "cmake --build build -j && ctest --test-dir build"
#   expect: all 35 mabur suites pass. (17 FAILED/Not-Run are pre-existing devourer-subtree
#   tests — see §5. They are NOT yours.)
build/gs/maburgs -c gs/bundle/maburgs.default.json      # no cards attached -> graceful retry+exit, no crash
bash tools/build-arm64.sh && file out/arm64/maburgs     # -> ELF aarch64, statically linked
```

---

## 5. Merge caveats (NOT regressions — note when merging)

- **17 failing `ctest` entries** (`stream_stdin_binary`, `tone_mask_math`, `radiotap_txflags`,
  tdma/timesync/sweep/txpower/health/caps/tsf/txpkt/txagg/bf_report) are the **devourer
  submodule's own** radio-side tests under `third_party/devourer/`. This branch changes
  **zero** files there. They are pre-existing and out of scope — call this out in the merge
  commit so they aren't misattributed to maburgs.
- **Task 13 is hardware.** Its "incompleteness" is expected until the bench is run; it is
  not missing code.

---

## 6. Tracked follow-ups (non-gating; for a later iteration)

- **v1.1 multi-bw rung-blocking coverage (do BEFORE flipping `bw_set` to multi-rung):**
  `Controller::report_rung_delivery` / `rung_blocked` / `primary_dirty_` (`gs/src/controller.cpp`)
  is a faithful port but has **zero test coverage** — the replay trace is single-bw so the
  path never runs. It's correct-by-inspection and deliberately inert in v1. Add a multi-bw
  controller replay vector to `tools/genvectors/gen_control_vectors.py` +
  `tests/test_controller.cpp` before enabling multi-rung, or the parity of that path ships
  unpinned.
- **Per-layer TX power (v1.1)** still needs the devourer Jaguar3 `TXPWR_OFSET` port first
  (unchanged from the drone spec).
- **Multi-node forwarder** (OpenWrt / `node_listen`) is out of scope: the `node::RxBody`
  wire format is the contract a future forwarder implements. Nothing hard-codes single-host
  beyond `main.cpp`'s in-process queue wiring.
- **Optional cleanup:** `main.cpp` computes `frame_type` twice per body (once for score
  routing, once inside `Aggregator::on_rx_body`) — harmless micro-inefficiency, dedupe if
  touching that path.

---

## 7. Where everything lives

- **Plans:** `docs/superpowers/plans/2026-07-12-mabur-gs-datapath.md` (1),
  `…-mabur-gs-control-plane.md` (2). Task 13 is the last section of Plan 2.
- **Design spec:** `docs/superpowers/specs/2026-07-12-mabur-gs-design.md`.
- **Code:** `common/` (decode primitives + `node` codec), `gs/src/` (daemon + control
  stack), `gs/bundle/` (config, systemd unit, install.sh), `cmake/aarch64-musl.cmake`,
  `tools/build-arm64.sh`, `tools/genvectors/` (generators).
- **Python references (parity source of truth):** `../devourer/tools/precoder/`
  (`energy_model.py`, `link_model.py`, `op_table.py`, `score.py`, `controller.py`,
  `rendezvous.py`, `adaptive_link.py`, `rc_proto.py`).
- **SDD trail** (per-task briefs, implementer reports, review verdicts, every fix):
  `.superpowers/sdd/p2-task-*.md`; recovery ledger + all decisions: `.superpowers/sdd/progress.md`.
