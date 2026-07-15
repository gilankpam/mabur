# Per-layer big-symbol video corruption — debug handoff (2026-07-15)

**Status: SOLVED (mechanism identified). The residual is NOT transport loss
and NOT a decoder/reorder bug — it is source-side FU-chain truncation from
drone venc-ring overflow, because the per-layer geometry costs the drone TX
path ~1.63× more air bytes than scalar and the hot thread cannot drain the
SHM ring fast enough. Same mechanism as the previously-known waybeam ring-full
truncation; the per-layer geometry just re-triggers it. Making per-layer work
is a drone-TX-throughput problem, not a transport problem.** Production is
safely on the known-good scalar-164 config (re-verified clean this session:
rtpsniff 55.9 ok_fps, 4% bad — all gaps in the venc restart transient).

The decoder fix `e49e8f4` was **sufficient for the transport side**: an
independent set-membership seq dump measures true transport loss at **0.02%**
under the broken config (13 of 79346 seqs), not the 3.13% rtpsniff reported.
See §4 for the correction and the evidence.

Branch: `sliding-window-fec`. Devices: drone `root@192.168.10.152` (TX,
armv7 SSC338Q), GS `root@10.18.0.1` (RX, aarch64 Radxa). Note: BusyBox on
BOTH — no `pgrep`/`pkill`, kill by PID. Also: drone has **no sftp-server** —
`scp` needs the legacy `-O` flag (`scp -O local remote`).

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

## 4. The residual — RESOLVED (2026-07-15 session 2)

The prior handoff called the residual an "uncracked" ~3.13% loss and pointed
the next debugger at the aggregator/reorder and cross-layer merge. That was
chasing a **measurement ghost**. The three candidate paths (#1 reorder, #2
stream-0 keyframes, #3 rtpsniff artifact) were investigated by measuring
transport loss INDEPENDENTLY of rtpsniff. Result: **#3 is correct — rtpsniff
over-counts, and the real transport loss is ~0.**

### 4a. The loss is real to the eye but is NOT transport loss

Built `tools/bench/seqdump.py` (raw AF_PACKET dump of every RTP seq +
kernel `PACKET_STATISTICS` drop counter) and `seqdump_analyze.py`
(**set-membership** gap detection: a seq is missing only if NEITHER loopback
copy — lo TX + lo RX — ever appears anywhere in the capture; immune to the
duplicate-interleaving that fools sequential-delta accounting).

120 s capture under the broken per-layer config, `kernel_drops=0` (capture
trustworthy):

| Metric | rtpsniff (delta-based) | seqdump (set-membership) |
|---|---|---|
| missing RTP seqs | 1130 (**1.42%**) | **13 (0.02%)** |

The baseline (scalar) capture cross-checks the tools: rtpsniff 343 missing
(0.67%), seqdump **0 missing**. rtpsniff's sequential-delta logic false-
positives on loopback dual-copy interleaving; set-membership is authoritative
(copies histogram was `{2: 79332}` — every seq present exactly twice). **True
transport loss under per-layer is ~0.02%. The decoder fix `e49e8f4` fully
solved the transport side.** The GS decoder counters being frozen was
CORRECT all along — nothing was lost for it to recover.

### 4b. Where the 74% bad frames actually come from — source truncation

`tools/bench/fu_chain_analyze.py` walks FU chains in RTP-seq order on the
deduped set and separates two failure modes:

| | broken (per-layer) | baseline (scalar) |
|---|---|---|
| FU chains | 7199 | 3598 |
| ok (end reached) | 1879 | 3597 |
| **trunc_contig** (chain broken with NO missing seq) | **5306** | 0 |
| trunc_at_gap (chain broken across a lost seq) | 13 | 0 |

**5306 chains truncate with fully contiguous RTP seqs** — a FU-start appears
while a chain is still open, and nothing was lost in transit between them.
That is only possible if the encoder/packetizer emitted a partial NAL (no end
fragment) and moved on: **source-side slice-tail truncation**. This is the
previously-documented waybeam ring-full abort (see mabur-project memory root
cause #2, and the `TxQueue` comment at `drone/src/main.cpp:611`). The
truncated tails have contiguous seqs, so EVERY transport/decoder counter
stays frozen — this is the whole "uncounted loss" fingerprint, explained.

### 4c. Why per-layer triggers it and scalar doesn't — drone TX throughput

`tools/bench/encbench.cpp` runs the real `UepEncoder` (classify → fragment →
SW-FEC → SBI pack) over a synthetic 60 fps / 8.8 Mbps HEVC-FU stream. On the
SSC338Q (`gf=neon-vtbl2-q16`):

| | air bytes produced | inflation vs input |
|---|---|---|
| scalar `[164]×4`, bpb 8 | 21.80 Mbps | ×2.47 |
| per-layer `[164,1312,1312,1312]`, bpb `[4,1,1,1]` | **35.49 Mbps** | ×4.03 |

Per-layer emits **1.63× the air bytes** for identical video (tiny-payload-in-
big-symbol waste: a 1400 B packet → two 1312 B symbols, each its own body at
bpb 1, plus the scaled overhead ladder). Live per-thread sampling under the
broken config: the hot thread ran at **~79% of one core (of 2)** and the loop
completed only **~10.7 iterations/s** (`hot_beat` 601→708 over 10 s). Each
iteration drains ≤64 ring packets → ≤685 pkt/s, **below the ~780 pkt/s video
packet rate** (60 fps × ~13 FU/frame) → the venc SHM ring fills → waybeam
aborts slice tails. Scalar under the same sampling ran `hot_beat` ~14/s
(≈896 pkt/s drain, > 780, keeps up) and stays clean. The 685/896 pkt/s drain
rates straddle the 780 pkt/s demand exactly as the starvation model predicts.

### 4d. What this means for making per-layer work

The bottleneck is **drone TX cost per second**, not FEC recovery. To land
per-layer you must cut the drone-side air-byte + GF workload back under the
hot thread's sustainable drain, e.g. any of:

- Raise `blocks_per_body` for the big-symbol video layers (bpb 1 → 4/8) so
  each body amortizes SBI/PHY framing over more symbols — directly attacks
  the 1.63× inflation, and bpb 1 was also what hit the join-anchor bug hardest.
- Lower the per-layer overhead ladder / `base_overhead` for the video layers.
- Increase the hot-thread ring-drain burst cap above 64, or give the hot
  thread more CPU (it already owns ~1 of 2 cores; the venc + waybeam contend).
- Or keep scalar-164 (current production) — it is the known-good point.

Whatever the change, **gate it on `rtpsniff` frame integrity AND a `seqdump`
set-membership pass on hardware** — byte-rate and burst_sim both green-lit
the failing config.

### Landmines corrected from the prior handoff
- The "3.13% residual transport loss" was a **rtpsniff over-count**, not real
  loss. Do not re-instrument the aggregator/reorder for it (prior §4 #1) —
  transport is clean. The `RtpReorder`/two-card merge are not implicated.
- The "~1 s keyframe-cadence gaps" (prior §4 #2) were rtpsniff delta artifacts
  clustered by the dual-copy interleave, not stream-0 keyframe loss. seqdump
  shows only 13 scattered single-seq gaps, not keyframe-aligned runs.

---

## 5. Tooling & reproduction

**Frame-integrity measurement (THE gate — byte-rate/skip checks missed this
bug entirely):** `tools/bench/rtpsniff.py` (committed). Run ON the GS:
```
scp -O tools/bench/rtpsniff.py root@10.18.0.1:/tmp/      # or already there
ssh root@10.18.0.1 'python3 /tmp/rtpsniff.py lo 5600 <seconds>'
```
Reports pkts/Mbps, seq gaps (%), FU truncations, and ok/bad frames + ok_fps.
Passive AF_PACKET sniff of the maburgs→pixelpilot loopback — does not disturb
the session. **Use this for frame integrity — but its seq-gap % OVER-counts
on the loopback dual-copy (1.42% vs true 0.02%); cross-check gaps with
seqdump.** ok/bad-frame and FU accounting are reliable.

**Authoritative transport-loss measurement (this session, committed):**
`tools/bench/seqdump.py` + `seqdump_analyze.py` + `fu_chain_analyze.py`.
```
scp -O tools/bench/seqdump.py root@10.18.0.1:/tmp/
# capture (run alongside rtpsniff for a frame-integrity read on the same window):
ssh root@10.18.0.1 'python3 /tmp/seqdump.py lo 5600 <seconds> /tmp/seqdump.txt'
scp -O root@10.18.0.1:/tmp/seqdump.txt /tmp/
python3 tools/bench/seqdump_analyze.py /tmp/seqdump.txt   # set-membership gaps + lateness
python3 tools/bench/fu_chain_analyze.py /tmp/seqdump.txt  # trunc_contig vs trunc_at_gap
```
seqdump logs `kernel_drops` at exit — if nonzero the capture is untrustworthy
(raise `SO_RCVBUF`). analyze uses set-membership (immune to loopback dup
interleaving); fu_chain separates source truncation (`trunc_contig`) from
transport loss (`trunc_at_gap`) — the distinction that cracked this bug. NOTE:
to classify FU packets by layer (s0 vs s1) the analyzers read a `fu_rt` 8th
column; re-`scp -O` the current `seqdump.py` to the GS before capturing (the
copy used this session predated that column, so its dumps show `fu_rt=-1`).

**Drone encoder-capacity bench (this session, committed):**
`tools/bench/encbench.cpp` — measures air-byte inflation + sustainable pkt/s
of a given FEC geometry, ON the drone (the throughput ceiling that per-layer
blows past). Cross-build + run with maburd stopped:
```
./toolchain/cc/bin/armv7l-unknown-linux-musleabihf-g++ -O2 -std=c++17 -static \
  -mfpu=neon-vfpv4 -I common/include tools/bench/encbench.cpp \
  common/src/uep_encoder.cpp common/src/sw_encoder.cpp common/src/sw_wire.cpp \
  common/src/sbi.cpp common/src/frag.cpp common/src/gf256.cpp common/src/nal.cpp \
  common/src/crc16.cpp -o /tmp/encbench-arm
scp -O /tmp/encbench-arm root@192.168.10.152:/tmp/encbench
ssh root@192.168.10.152 '/etc/init.d/S96mabur stop; sleep 1; \
  /tmp/encbench scalar 10; /tmp/encbench perlayer 10; /etc/init.d/S96mabur start'
```
(`-mfpu=neon-vfpv4` matches `common/CMakeLists.txt:10` so the bench uses the
same `neon-vtbl2-q16` GF path as the deployed maburd.)

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
