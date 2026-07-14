# Per-layer big-symbol video corruption — debug handoff (2026-07-15)

**Status: PARTIALLY SOLVED. One real decoder bug found + fixed (commit
`e49e8f4`), which improved but did NOT cure the hardware symptom. A second,
uncracked loss mechanism remains.** Production is safely reverted to the
known-good scalar-164 config. This doc is a cold-start handoff for the next
debugger.

Branch: `sliding-window-fec`. Devices: drone `root@192.168.10.152` (TX,
armv7 SSC338Q), GS `root@10.18.0.1` (RX, aarch64 Radxa). Note: BusyBox on
BOTH — no `pgrep`/`pkill`, kill by PID.

---

## 1. The symptom

Changing the drone/GS FEC geometry from the working scalar config to a
per-layer big-symbol config **destroys live video** — heavy H.265
macroblocking, ~1.9–11 fps of ~60, PP shows ~4 Mbps decoded.

| Config | ok fps | bad frames | RTP loss (rtpsniff) |
|---|---|---|---|
| **Working:** `symbol_size=164`, `bpb=[8,8,8,8]` | **58–60** | **0–2%** | **0–0.5%** |
| **Broken (pre-fix):** `symbol_size=[164,1312,1312,1312]`, `bpb=[4,1,1,1]` | 1.9–3.4 | 94–97% | 2.6–3.65% |
| **Broken (post-fix `e49e8f4`):** same as above | **11.3** | 81% | 3.13% |

All at MCS5 / window 64 / overhead ladder `{1.0,0.75,0.5,0.25}` scaled by RC.
The 1312B video-layer symbols were chosen by `tools/bench/burst_sim.cpp`
(they win on burst-robustness in simulation). wfb-ng runs the same
whole-packet ~1400B geometry cleanly, so the geometry is NOT fundamentally
broken — mabur has an implementation defect it exposes.

**Why 3% loss → 90%+ bad frames:** long-GOP H.265 with FU fragmentation —
one lost packet truncates a frame, and P-frame references propagate the
damage until the next clean keyframe. So even a few % loss is catastrophic.

---

## 2. The diagnostic fingerprint (KEY — this is what to chase)

Under the broken config, on BOTH ends **every packet-loss counter is
FROZEN** while rtpsniff still measures 2.6–3.65% missing RTP seqs:

- GS `maburgs` per-layer stats (`/tmp/maburgs.log`, one `stats:` line/sec):
  `s0[...]`/`s1[...]` fields `abn` (syms_abandoned), `fe` (frag_evicted) —
  **frozen**. `sbf`=0, `bc`=0.
- GS reorder `ord[... skip=N late=M]` — `skip` grew ~+1/12s, `late`≈0 —
  effectively **frozen**.
- Drone `maburd` stats (`/tmp/mabur.log`): `drops`, `txq_drop`,
  `tx_failed=0`, `waybeam_failures=0` — **frozen**, `txq`≈0 (not backed up).
- Air frames arrive intact: `cf=0` (no CRC-fail), SNR healthy (c1 ~60, c0 ~45).

**Interpretation:** nothing that is *supposed* to register a lost packet
moves, yet packets vanish from the delivered stream. The loss is in a path
with NO instrumentation. This "uncounted" property is the whole puzzle and
must drive the next steps.

The gaps cluster at a **~1 s cadence** (rtpsniff gap-log timestamps), which
matches the keyframe/IDR cadence — a strong hint the residual is
**cross-layer** (stream-0 keyframes vs stream-1 video, decoded in separate
pipelines and merged by RTP seq in the reorder).

---

## 3. What was fixed (commit `e49e8f4`) — real, but insufficient

**Bug:** `SwDecoder` (`common/src/sw_decoder.cpp`) gated its admit/drop
checks on `base_` (the loss-accounting floor). At a stream (re)join, `base_`
is set to the FIRST source seq the decoder happens to see and only climbs to
`newest_v_ - horizon_` after the opening `horizon` symbols. During that
window any source with a *lower* seq — a leading source lost in a join burst,
or simply a lower seq arriving after a higher one (rampant at bpb=1 where
each symbol is its own independently-timed air frame) — was `< base_` and
dropped stale: never delivered, recovered, nor abandoned. Whole RTP packets
vanished with all counters frozen. This is exactly the fingerprint, and it's
why bpb=1 is hit far harder than bpb=8 (8 symbols/body arrive together, so
the anchor lands on the earliest survivor).

**Fix:** admit/drop checks now use `live_floor() = newest_v_ - horizon_`
(the true "older than the recovery horizon" threshold), identical to `base_`
in steady state but correct during the join window. `advance()` counts
abandonment only within `[base_, nb)` so below-`base_` deliveries don't
underflow the counter. See `sw_decoder.cpp` (`live_floor()`, the three
`< live_floor()` checks, and the `if (it->first >= base_)` guard in
`advance()`). Wire format UNCHANGED — decoder-internal only, backward
compatible, safe on scalar-164.

**Verification:**
- Regression test `tests/test_uep_sw.cpp :: opening_loss_is_accounted`
  (added by the bug hunt) — was FAILING, now passes.
- All 8 host suites green (`sw_decoder|sw_encoder|sw_wire|uep|gf256|config`).
- Host simulations (see §5): every uncounted-loss case — opening, reordered
  (jitter 2–16 bodies), bursts — now delivers/recovers/counts. Zero uncounted.
- **Hardware: improved 3.4→11.3 ok_fps, 94→81% bad — but NOT fixed.**

Full root-cause writeup: `.superpowers/sdd/bigsymbol-bug-report.md`.

---

## 4. The residual (UNCRACKED) — start here

Post-fix hardware still loses **3.13%**, and the fingerprint is UNCHANGED:
all GS+drone counters frozen (verified `abn`/`fe`/`skip`/`late`/`drops`/
`txq_drop` flat over 12s samples), yet rtpsniff sees the gaps.

**Since the GS decoder counters are frozen, the decoder is (now) delivering
everything it receives** — so the surviving loss is almost certainly NOT in
`SwDecoder`. Candidate locations, in rough priority:

1. **Cross-layer reorder / two-card merge.** `gs/src/aggregator.cpp` merges
   the two RX cards; `gs/src/rtp_reorder.h` (`RtpReorder`, `hold_ms=300`)
   re-orders the decoded output by RTP seq before the UDP sink. Stream 0
   (critical/keyframes, sym 164, separate `SwDecoder`) and stream 1 (video,
   sym 1312) decode on independent timelines. If a stream-0 keyframe packet
   is slow, the reorder holds the gap — but `skip` is frozen, so either the
   packet is dropped somewhere the reorder doesn't count, or the merge drops
   it. INSTRUMENT the aggregator + reorder for per-seq drops.

2. **The loss may be stream-0 (keyframe) packets specifically** — gaps at ~1s
   keyframe cadence. Classify the MISSING seqs by layer: extend rtpsniff (or
   a new sniffer) to parse the HEVC NAL type of each delivered packet and, on
   a gap, infer whether the missing seq range is keyframe-adjacent. `s0` uses
   sym 164 / bpb 4 — its geometry ALSO changed (bpb 8→4). Consider testing
   per-layer with stream-0 left at the OLD bpb 8, or scalar-164 for stream 0
   only, to isolate.

3. **rtpsniff measurement artifact.** Cross-check before trusting 3.13%:
   `tools/bench/rtpsniff.py` de-dups loopback (delta==0) and does FU-chain
   accounting. With this specific traffic (heavy FU, two-card, big symbols)
   confirm the seq-gap count with an INDEPENDENT raw seq-continuity dump
   (log every delivered RTP seq to a file, diff for gaps offline). If the
   independent dump shows 0 gaps, the residual is a measurement artifact and
   the fix may actually be sufficient.

**First move for the next debugger:** resolve #3 (is the loss real?) with an
independent seq dump, THEN instrument the aggregator/reorder (#1) and
classify missing seqs by layer (#2).

---

## 5. Tooling & reproduction

**Frame-integrity measurement (THE gate — byte-rate/skip checks missed this
bug entirely):** `tools/bench/rtpsniff.py` (committed). Run ON the GS:
```
scp tools/bench/rtpsniff.py root@10.18.0.1:/tmp/         # or already there
ssh root@10.18.0.1 'python3 /tmp/rtpsniff.py lo 5600 <seconds>'
```
Reports pkts/Mbps, seq gaps (%), FU truncations, and ok/bad frames + ok_fps.
Passive AF_PACKET sniff of the maburgs→pixelpilot loopback — does not disturb
the session. **Use this, not byte-rate, to judge any FEC change.**

**Host simulations (recreate — they were in session scratch, not committed).**
Build any `<sim>.cpp` against the common sources:
```
nix-shell -p pkg-config libusb1 --run "g++ -O2 -std=c++17 -I common/include <sim>.cpp \
  common/src/uep_decoder.cpp common/src/uep_encoder.cpp common/src/sw_decoder.cpp \
  common/src/sw_encoder.cpp common/src/sw_wire.cpp common/src/sbi.cpp common/src/frag.cpp \
  common/src/frag_reassembler.cpp common/src/gf256.cpp common/src/nal.cpp common/src/crc16.cpp -o <sim>"
```
Pattern that reproduced the SIGNATURE (uncounted loss) pre-fix and showed it
GONE post-fix: build the per-layer `UepLayerCfg` (sym `[164,1312,1312,1312]`,
bpb `[4,1,1,1]`, window 64), feed ~1400B stream-1 RTP through
`UepEncoder → drop/reorder bodies → UepDecoder`, and assert
`missing_RTP <= syms_abandoned + frag_evicted`. Drop patterns that mattered:
opening bodies `[0,2)`, mid-stream bursts of ~10, and out-of-order delivery
(random release from a jitter buffer of N bodies). NOTE: the sim drives
`UepDecoder` DIRECTLY — it does NOT include the `RtpReorder` or the two-card
aggregator, which is exactly why it now shows 0 loss while hardware still
loses. **The next sim must model the reorder + cross-layer (s0/s1) merge.**

**Host tests:** `nix-shell -p pkg-config libusb1 --run 'cmake --build build
-j$(nproc) && ctest --test-dir build -R "sw_decoder|uep|sw_wire" --output-on-failure'`
(17 devourer-subtree failures are pre-existing, ignore).

---

## 6. Build / deploy / revert

**Cross-build:** `./tools/build-arm.sh` (→ `out/arm/maburd`),
`./tools/build-arm64.sh` (→ `out/arm64/maburgs`). Decoder fix is in `common`,
so both pick it up; only `maburgs` strictly needs it (decode side).

**Deploy per-layer test config** (both ends, config-only, no wire break —
but symbol_size MUST match both ends or that layer drops as `sbf`, not `bc`):
- Drone `/etc/mabur.json`: `fec.symbol_size=[164,1312,1312,1312]`,
  `blocks_per_body=[4,1,1,1]`, keep `window:64`.
- GS `/etc/maburgs.json`: `fec.symbol_size=[164,1312,1312,1312]`.
- Both daemons print a startup FEC-geometry banner (grep `fec:` in the logs)
  — use it to confirm what loaded.

**Restart:** `/etc/init.d/S96mabur restart` (drone), `/etc/init.d/S96maburgs
restart` (GS). Each restart triggers a one-time waybeam venc re-init transient
(a brief loss burst); settle ~15–20 s before measuring.

**REVERT to known-good (currently deployed):** GS
`/tmp/maburgs.json.scalar-good` and, both ends, `/etc/*.json.pre-perlayer` +
`/usr/...maburgs.pre-perlayer` / `maburd.pre-perlayer`. Restore binary AND
config together per end (old scalar binaries can't parse array configs).
Current production = scalar-164, verified clean (58.7 fps, 2% bad).

---

## 7. Landmines / things already ruled out (don't re-chase)

- **NOT latency.** wfb-ng works at a 30 ms reorder budget; mabur's is 200 ms
  decode + 300 ms reorder — far more generous. `late_dropped`≈0 throughout.
- **NOT the link.** `cf=0`, SNR healthy, drone `tx_failed=0`.
- **NOT `unpack_symbol` early-break.** Ruled out by an exhaustive size sweep
  (every RTP size 8..2600 at sym 1312, zero early breaks) — see bug report.
- **NOT the fragmenter oversize path.** `oversize_drops` never fired.
- **NOT the RC bitrate model.** `rc_agent.cpp:133` `phy*budget/(1+ov)` has no
  symbol/bpb term. The 6.6 vs 9.3 Mbps source-rate dip under the broken
  config was the RC *shedding to level 3* (0.7×, `rc_agent.cpp:192`) in
  REACTION to the loss — a symptom, not a cause.
- **`burst_sim.cpp` is blind to this class** — it counts permanent loss, not
  timely in-order delivery, so it green-lit a config that fails on air. Any
  future FEC-geometry decision must be gated on `rtpsniff` frame integrity on
  hardware, not on `burst_sim` or byte-rate.
- Two over-claimed dead-ends during this session (recorded so you don't
  repeat them): "recovery latency from big symbols" and "the join-anchor bug
  is the whole cause." Both were wrong/incomplete. Verify magnitude on
  hardware, not just signature in sim.
