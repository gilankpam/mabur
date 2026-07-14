# Handover: the video-delivery gap — findings & next plan (2026-07-13, post-midnight session)

**Audience:** the next session picking up where the marathon bench day ended.
Read `docs/handover-maburgs-bench.md` first (Task 13 context), then
`docs/bench-validation.md` findings 1–12 (the day's full ledger). This file is
the deep-dive on the ONE remaining problem and the concrete plan to fix it.

**Branch:** `gs-control-plane`, all of today's work committed through the RTP
reorder buffer (`30b9a9a`) + this doc. The devourer fork merge lives on
`../devourer` branch `merge-upstream-b5a6df7` @ `38fdde4` — **push it to
github.com/gilankpam/devourer before pushing this branch** (the submodule pins
it).

---

## 1. The problem in one paragraph

Everything works except the thing that matters: **PixelPilot renders garbage
while every transport counter says the link is healthy.** The radio does
27 Mbps raw at MCS5 (99.1 % delivery, benchmarked). The link holds LINKED
100 % (after the seq-lockout fix). FEC blocks all decode (`du=0` in most
configs). And yet 7–9 % of RTP packets never reach the decoder in usable
form, which at ~18 packets/frame corrupts ~70 % of video frames — PP shows
0.3–2 Mb/s and 5 fps of smear out of a 5–7 Mbps stream.

## 2. Root cause (proven by simulation): NO SYMBOL INTERLEAVING

`blocks_per_body` packs N sub-blocks **of the same RS block** into one air
frame. A block at s1's reference overhead (n=14 symbols, k=8) spans just
**2 air frames** at the classic geometry — losing ONE frame kills 8 of 14
symbols and the whole block dies ~50 % of the time. The FEC protects against
*symbol corruption within delivered frames* (which radios never give you —
CRC drops the whole frame) and barely at all against *frame loss* (the only
real loss mode). Every "post-FEC 0.000 %" window all day was simply a
zero-frame-loss window.

Proof: `tools/bench/fec_geometry_sim.cpp` (encoder→decoder loopback with
loss injection). Key numbers at 5 % random frame loss:

| geometry (symbol / bpb) | body size | block spans | delivered |
|---|---|---|---|
| 64 / 8 (repo default) | 600 B | ~2 frames | (same failure class) |
| 164 / 8 (bench today) | 1400 B | 2 frames | **91.1 %** ← matches the on-air tap exactly |
| 340 / 4 | 1404 B | 4 frames | 99.08 % |
| 700 / 2 | 1422 B | 7 frames | 99.91 % |
| 700 / 1 | 711 B | 14 frames | 100 % |

Zero-loss loopback is 100 % at every geometry — the pipeline is correct,
the *framing layout* is the vulnerability.

## 3. Why we couldn't just config our way out tonight

Chasing the table above on air exposed three interacting constraints:

1. **Big blocks starve the critical stream.** s0 (VPS/SPS/PPS + IDR) trickles
   at ~3.6 KB/s between keyframe bursts. A symbol-700 block holds 5.6 KB —
   fill time measured in *seconds*, so blocks age out (`block_max_age_ms`)
   before they complete. On-air at 690/2: s0 AND s1 collapsed (u in the
   thousands) despite 90 % frame arrival.
2. **The latency triangle.** `block_max_age_ms` (decode horizon) vs the RTP
   reorder buffer's `hold_ms` vs FragReassembler's pending cap (512) form an
   uncoordinated latency budget. age=2000 + hold=350 → straggler blocks emit
   late and get dropped (`late=8001` in one 30 s window); age=300 → s0 blocks
   die; FRAG pending overflows evict boundary-crossing packets (`fe`
   thousands). Any fix must set ONE end-to-end deadline.
3. **Air frame size ceiling.** Total frame must stay ≤ ~1428 B (proven good;
   the silent 8822E PSDU-drop threshold is somewhere in (1461, 3037) — see
   bench finding 8; don't gamble).

Best measured compromise tonight (left running): symbol 340 / bpb [4,4,8,8],
static mcs4/ov1.0/agc32 — **7.8 % packets missing, 72 % frames bad.** Not
acceptable; the real fix is code, not config.

## 4. THE FIX (next session, v1.1): encoder-side symbol interleaving

Change `UepEncoder` body assembly: instead of packing `blocks_per_body`
sub-blocks of the SAME block per body, maintain a window of W open blocks and
emit bodies carrying one symbol from each of W different blocks (classic
convolutional/block interleaver). Then a block's n symbols ride n different
air frames without shrinking frames or growing blocks:

- symbol 164, W=8: body = 8×175 = 1400 B (unchanged air geometry!), block
  spans **14 frames** → sim-equivalent of the 700/1 row: ~100 % at 5–10 %
  loss, while blocks stay 1.3 KB (s0-friendly, low latency).
- **Decoder needs NO change** — SBI sub-blocks are self-describing (stream,
  block, index) and `UepDecoder`/multi-card union is already
  symbol-idempotent and order-agnostic (verified: the geometry sims pass
  through the real decoder).
- This is a deliberate, documented Python-parity break (svc_uep_fec.py has no
  interleaver). Guard with a golden-vector regen + a new loss-injection unit
  test derived from `tools/bench/fec_geometry_sim.cpp`.
- Latency cost: one block's packets complete after W bodies ≈ W×body_airtime
  (~10 ms at 1400 fps) — negligible; SET the end-to-end budget while here:
  block_max_age ≈ 250 ms, reorder hold ≈ 300 ms, FRAG pending eviction by
  AGE not count.

Plan of attack (half-day):
1. Unit-test first: port `fec_geometry_sim.cpp` into `tests/` as a
   loss-injection matrix (geometry × loss pattern × in/out-of-order bodies).
2. Interleaver in `UepEncoder` behind a config flag (`fec.interleave: true`),
   drone-side only.
3. Re-pin golden vectors for the flag-on path; keep flag-off byte-identical
   to Python.
4. Bench: tap with `tools/bench/rtptap.py` (procedure in §6) → expect ≥99 %
   seqs delivered at static mcs4-5; then PP visual.

## 5. Secondary open items (ranked)

1. **maburd flat TX-power override costs ~25 dB-equivalent of delivery at
   high MCS**: 72 % arrival at MCS5 vs streamtx's 99 % at per-rate table
   power. Switch `RealActuator` to `SetTxPowerOffsetQdb` (devourer's
   recommended shape-preserving lever) instead of
   `SetTxPowerIndexOverride(flat)`. Valid MCS5 sweep (static, seq-fix era):
   txagc 63/56/48/40 → 70–72 %, cliff below 40 (32→24 %, 16→1 %).
2. **Adaptive controller re-enable** needs the delivery-closed power loop
   (bench finding 9a — survivor-biased SNR reads 35–55 dB while the channel
   delivers 10 %; `margin_db` shim is insufficient). Design: feed the
   RCF-window delivery/residual into rung+power choice; the static-mode
   scaffolding (`link.static_*`) is the fallback and A/B baseline.
3. **Upstream devourer reports to file** (all bench-proven on 8812EU):
   silent PSDU drop >~1.4 KB (bisect the bound), STBC TX dead at MCS4+
   post-`b5a6df7` (57.5 k frames → ~0 heard; fine at MCS0-2), dual-card RX
   >~5600 URB/s overruns the Radxa EHCI, per-bring-up dead-RX-chain lottery
   (IGI fix helped but chip-state contamination persists across soft
   re-inits — VBUS-cycle levers: GS = full power cycle w/ 8812eu.ko
   disabled; drone = `Sstar-ehci-1` unbind/bind).
4. **G6 staircase** still needs the attenuation rig (or SDR interferer) —
   run only after the interleaver + power fixes, or the results will alias.
5. Keep-alive/DISC ergonomics: `keepalive DISC while LINKED is ignored`
   (parity) means recovery from GS restart rides FAILSAFE→DISC; fine, but
   re-check E5/G4 timings after any changes here.

## 6. Bench tooling built tonight (reuse, don't rebuild)

- **`tools/bench/rtptap.py`** — ground-truth RTP analyzer. Procedure: on the
  GS, `sed` `video_out.port` 5600→5601 in `/etc/maburgs.json`, restart
  maburgs, `python3 rtptap.py 5601 30`, then revert. Reports Mbps, seq
  gaps/reorder/dups, FU-chain integrity, per-frame corruption. PP's OSD
  bitrate is a *decoded-usable* number — never debug against it directly.
- **`tools/bench/fec_geometry_sim.cpp`** — lossy loopback through the real
  encoder/decoder. `g++ -std=c++20 -I common/include ... common/src/*.cpp`.
- **maburgs stats line** now carries: per-chain SNR (`a=`/`b=`), per-stream
  `fe=` (FRAG evictions), `ord[ok/gap/back/buf/skip/late]` (post-reorder
  stream health). `back>0` should never happen (reorder bug if it does).
- **`link.static_mcs/static_overhead/static_txagc`** — static-link mode
  (controller bypassed, all machinery else intact). THE debugging tool;
  found the seq-lockout bug within minutes of existing.

## 7. Current deployed state (for reproducibility)

- GS `/etc/maburgs.json`: single card (index 0), feedback_ms 50, static
  mcs4/ov1.0/agc32, fec k8/symbol 340/age 2000, video_out :5600.
- Drone `/etc/mabur.json`: symbol 340, blocks_per_body [4,4,8,8],
  bitrate 6800–6900 pinned, failsafe_ms 3000, crit/t0 stbc **false**,
  max_txagc 63.
- Binaries: both ends built from `gs-control-plane` + `../devourer`
  `merge-upstream-b5a6df7` (38fdde4). GS maburgs includes the reorder buffer
  (hold 350 — uncommitted-at-time-of-writing hold bump from 100, commit
  pending with this doc).
- GS image: `8812eu.ko` disabled (`/root/8812eu.ko.disabled`) — do not
  re-enable; kernel driver contact requires a full GS power cycle after.
- Last tap (30 s): 4.87 Mbps, 0 out-of-order, 7.8 % seqs missing, 72 %
  frames bad, ok_fps 16.5.

---

# UPDATE 2026-07-13 (overnight autonomous session): FIX LANDED — video delivery closed

The plan in §4 was executed and it worked, but only after finding and fixing
a second, hidden bug the interleaver exposed. Post-FEC delivery went from
**92% (7.8% missing, 72% frames bad)** to **≥99.9% at 10.1 Mbps / MCS5**,
with several 60 s windows at exactly **0 lost sequences** at 6.5 Mbps.

## What landed (commits, this branch)

1. `0ffcd91` **feat(fec): encoder-side symbol interleaving** (`fec.interleave`)
   — exactly §4's design: `SymbolInterleaver` between RsEncoder and SbiPacker,
   one symbol from each of W different blocks per body. Decoder unchanged;
   flag-off byte-identical to Python (golden vectors pass). Loss-injection
   matrix in `tests/test_uep_interleave.cpp`.
2. `7d01768` **latency budget** — FRAG eviction by AGE (block-horizon-tied,
   count cap kept as backstop), reorder hold 350→300 ms.
3. `1e6ece3` **fix(gs): stale-clock underflow insta-expired FEC blocks** —
   THE hidden killer, see below.
4. `8b151d0` **feat(drone): `radio.power_mode`** = override|offset|none
   (`none` = leave the efuse per-rate table alone, streamtx-proven;
   `offset` = fixed `SetTxPowerOffsetQdb` trim; offset/none bypass thermal
   derate — documented in config.h).
5. `c7eabb0` **feat(fec): `fec.interleave_depth`** — window W decoupled from
   blocks_per_body. Depth 32 ⇒ a block spans n·W/bpb bodies (~33–47 ms of
   air) and shrugs off the 5–20 ms fades that killed depth-8 blocks.
6. `96f2cde` **tools/bench/rtpsniff.py** — passive AF_PACKET tap of the LIVE
   session (rtptap's port-swap restarts maburgs = kills the session under
   test). Caveat: python drops capture above ~2 k pkt/s on the Radxa — at
   high rates trust the maburgs stats line, not the sniffer.

## The second bug (this is the one to remember)

Interleaving alone made things WORSE on air (u≈1300/s ≈ 2× block rate,
fl≈1200) while the same code was clean in a realistic-timing loopback sim.
Diagnostic counters added to the stats line (`dec/si/st/bc/sbf/fl`, `mis=`)
showed clean bodies (bc=0 sbf=0) but stale-drops ~1000/s below expectation:
**decoded-block markers were vanishing**. Root cause: `gs/src/main.cpp`
captured `now_ms`, then **blocked up to 10 ms in `queue.drain()`** — bodies
processed in that iteration carry NEWER stamps, so `agg.poll(now_ms)` /
`reorder.poll(now_ms)` ran with a clock OLDER than blocks just created:
`now - first_seen` (uint64) underflowed ⇒ instant expiry of everything
created that iteration. Invisible pre-interleave because a block lived and
died inside ONE drain batch; fatal once blocks spanned many batches. Fixed
at the call site (re-read clock after drain) + guards in
`RsDecoder::expire_blocks_older_than`, `RtpReorder::flush`,
`FragReassembler::sweep_expired`. Tests:
`expiry_clock_behind_block_is_noop`, `poll_with_stale_clock_never_skips`.

## Measured ladder (established sessions, 3 m NLOS bench, single card)

| config | video rate | post-FEC loss |
|---|---|---|
| pre-fix (depth 8 + clock bug, age 250) | 6.5 Mbps | ~31% missing |
| clock fix, depth 8, mcs4/ov0.25 | 6.5 Mbps | 1.8% (u=11/s, bursty fades) |
| + mcs5/ov0.5 (same airtime) | 6.5 Mbps | 0.5% |
| + interleave_depth 32 | 6.5 Mbps | **0 lost seqs / 60 s** |
| + bitrate pin 11 M (waybeam emits ~10.1) ov0.25 | 10.1 Mbps | 0.11% |
| + ov0.375 (final overnight config) | ~8–10 Mbps | **0.011% (17 seqs / 180 s, u=3 blocks)** |

Also proven: waybeam terminates frame-final FU chains WITHOUT the E bit
(rtptap's `lost_end`≈11/s and "18% bad" at ZERO transport loss is stream
idiosyncrasy, present under wfb-ng too — do not chase it). Waybeam config
on the bench drone: 1080p60 CBR, `gopSize: 0.1` (sub-second GOP = the big
s0/IDR share). The 8.5 M pin emitted only 6.5 M on the night scene; the
11 M pin emitted ~10.1 M.

## Deployed end state (bench, 2026-07-13 ~02:00)

- Drone `/etc/mabur.json`: symbol 164, bpb [8,8,8,8], `interleave: true`,
  `interleave_depth: 32`, `power_mode: "none"`, max_txagc 40 (irrelevant in
  none-mode), bitrate pinned 11000–11100, backup at
  `/etc/mabur.json.bak-il`.
- GS `/etc/maburgs.json`: symbol 164, `block_max_age_ms: 250`, static
  mcs5/ov0.375/agc48 (agc irrelevant in none-mode), video_out :5600 (PP),
  backup at `/etc/maburgs.json.bak-il`. Binaries both rebuilt from this
  branch (maburgs includes the diagnostic stats fields).
- Worst-case airtime at the pin: ~10 M × 2.125 ≈ 21 Mbps of MCS5's measured
  27.4 — ~78% duty. If a daylight scene pushes the encoder to the pin and
  the link degrades, drop the pin or overhead first.

## Still open (next session)

1. **PP visual confirmation** — all overnight evidence is transport-level
   (0 gaps / 0 reorder at the UDP sink). Latency check too: depth 32 adds
   ~25–50 ms encoder buffering at these rates (config knob if too much).
2. Adaptive re-enable: delivery-closed power/rung loop (finding 9a) — the
   controller still can't be trusted with open-loop SNR; static mode is the
   shipped workaround. `power_mode` semantics for the adaptive path TBD
   (commanded pwr_idx is ignored in none/offset modes).
3. Per-rung geometry/overhead table (symbol/bpb/depth currently global).
4. Upstream devourer filings (unchanged list, §5.3) + push
   `merge-upstream-b5a6df7` (38fdde4) to the fork BEFORE pushing this branch.
5. G6 staircase (unchanged).

---

# UPDATE 2 — 2026-07-13 morning: the OTHER half of the glitches (drone pipeline)

PP still glitched after the overnight transport fixes ("~5 Mbps usable of
11 M set"). Morning forensics (rtpsniff + rtpdump payload capture) found the
transport CLEAN (0 seq gaps) while ~26 FU chains/s were missing their tail
fragments — **packets that never got RTP seq numbers**. waybeam's packetizer
consumes a seq only on writer success and ABORTS the rest of the NAL on
failure (`rtp_packetizer.c`), so producer-side drops are invisible to every
seq-gap counter. The failing writer was the mabur SHM ring (wfb never used
it — why wfb was "clean"): `/dev/shm/mabur` header showed the ring PINNED at
457–511/512.

Chased through three layers (each real, each necessary):
1. **USB inline TX** — hot thread did blocking bulk-OUT per body.
   → `97abe6e`: bounded TxQueue (drop-oldest = FEC erasures) + TX writer
   thread + `RadioTx::send_bodies` → `send_packets` with
   `tx.usb_agg_max=3` (3 frames/URB, HalMAC limit). Watch `txq=`/`txq_drop=`
   and devourer `tx.agg` events (logger now Warn — info flooded tmpfs).
2. **One-ring-read-per-loop** — per-iteration overhead capped drain at
   ~800 pkt/s vs 1000+ produced. → `f5729b1`: burst-drain ≤64/iteration.
3. **Scalar GF256** — the real ceiling: log/exp lincomb ≈ 38 MB/s on the
   Cortex-A7; RS parity at 60 fps rates ate the core (ring still pinned).
   → `ff080fd`: NEON vtbl split-nibble lincomb (aarch64 + ARMv7 paths),
   **4.1×** (156.6 MB/s), encoder 1037→1389 pkt/s on-target
   (`tools/bench/gf_bench.cpp`, includes a byte-exact verify mode).

**Airtime is the final wall, not a bug**: at MCS5/20 MHz (~26-27 Mbps
practical), video×(1+eff_overhead) must stay ≲75% duty. 11 M @ ov0.375 =
23.4 M air = 90% → chip TX FIFO saturates (txq_drop climbing). The tuned
operating point: **9 Mbps @ ov0.375 (73% duty)**.

## Verified end state (2026-07-13 ~10:00, PP feed live)

- rtpsniff 20 s: **9.32 Mbps, 0 gaps, 0/1199 bad frames, 59.9 ok-fps**
  (was 60% bad at 8 AM). GS 60 s: skip 0.026%, late=0, back=0. Drone:
  ring_fill≈7/512, txq=0, thermal Δ1.
- Configs: drone pin 9000–9100, interleave depth 32, power_mode none;
  GS mcs5/ov0.375/age250. Bitrate ladder beyond 9 M needs ov0.25 (less
  fade margin), 40 MHz, or MCS6/7 — measure before promising.
- rtpsniff caveat: python capture drops >~2 k pkt/s; run it at ≤10 M rates
  or trust in-process stats.

## Post-tuning addendum (same morning): depth/flush interaction — READ THIS BEFORE TOUCHING EITHER

`interleave_depth` small (0/8) + `flush_ms` 15 at 60 fps = **tiny-body storm**:
every inter-frame gap (16.7 ms > flush_ms) triggers drain_layer, and the
drain emits one short body PER ROUND of the leftover window — at depth 8
that's ~17 bodies of 1–2 symbols per frame, +~500 frames/s of air for zero
extra payload. Bench-proven: depth 0 → drone TX 2540 fps, 16.7% seqs lost,
PP at 30 fps. Deployed cure: **depth 16 + flush_ms 25** — the idle flush
never fires mid-stream at 60 fps, the window persists across frames (true
cross-frame interleaving), bodies are always full, and the per-frame padded
flush block disappears (~8% airtime saved). Cost: frame-tail packets ship
~1 frame later. Verified: 9.31 Mbps, 0 gaps, 1197/1198 frames, 59.9 ok-fps.
v1.1 code fix if small depths are ever needed: drain should pack rounds
consecutively and flush the packer once (trades tail-block spread for body
efficiency) — see drain_round() note in interleaver.h.

**Final deployed config (2026-07-13):** drone: 9000–9100 pin, symbol 164,
bpb [8,8,8,8], interleave on, depth 16, flush 25, power_mode none. GS:
mcs5 / ov0.375 / age 250 / hold 300. PP visually confirmed clean by user.

# UPDATE 2026-07-14: block RS + interleaver retired — sliding-window FEC is the only scheme

Everything above (`interleave_depth`, `blocks_decoded`/`blocks_unrec`,
`interleaver.h`, block-RS `k`/geometry tuning) describes the **old** scheme
and is now historical record only — the code it refers to is deleted.
Design spec: `docs/superpowers/specs/2026-07-14-sliding-window-fec-design.md`;
migration plan: `docs/superpowers/plans/2026-07-14-sliding-window-fec.md`.

**What changed:** RS block-FEC + the `SymbolInterleaver` (reorder-and-wait
time-diversity buffer) are gone. Sources now ship immediately in a
systematic sliding-window RLC code over GF(256) (`SwEncoder`/`SwDecoder`,
wire magic `0xF541`, one unified 14-byte envelope header shared by source
and repair symbols — sources carry `window_len=0, repair_key=0`). Time
diversity comes from *overlapping repair windows* riding subsequent air
frames instead of delaying sources through a reorder stage.

**Knobs (drone `fec.window`, per-layer `overhead`, `symbol_size`,
`flush_ms`; GS `decode_deadline_ms`, `seq_horizon`):** `k` and
`interleave_depth` are gone. `window` (default 128 in
`bundle/mabur.default.json`) replaces both — it is the encoder's repair
span in symbols and the decoder's Gaussian-elimination row horizon.

**Burst budget:** a contiguous loss burst of `L` symbols is recoverable in
steady state when `L <= W * ov / (1 + ov)` (W = window, ov = per-layer
overhead) — enough subsequent repairs must arrive, each covering up to `W`
symbols, to independently cover the `L` missing unknowns before the window
slides past them. This is a steady-state bound: very short streams (total
length comparable to or shorter than one window) can still see rank-deficient
gaps even under budget, because too few distinct repair equations exist yet
— see `tests/integration/run_gs_e2e.sh`'s third stanza comment for a worked
example (stream 2 capacity edge, root-caused and fixed by seed selection,
not by loosening gates).

**Stats renamed:** `blocks_decoded`/`blocks_unrecoverable` →
`syms_recovered`/`syms_abandoned` (`UepDecoder::LayerStats`, see
`common/include/mabur/uep_decoder.h`); `syms_delivered` covers the
immediate (non-repaired) path.

**Unchanged:** the airtime-cap formula, `video × (1 + ov) <= ~75% of the
MCS's practical duty ceiling` (see "Airtime is the final wall" above) —
overhead is still overhead, whichever code computes the repair symbols.

**Migration note for whoever deploys this:** wire format changed
(envelope headers, magic 0xF541) — drone and GS must be flashed together,
no mixed-version link.

# UPDATE 2026-07-15: ARMv7 lincomb q16 kernel + symbol-size finding — encoder ceiling anatomy

**SW-vs-RS TX regression root-caused on hardware** (linkbench matrix, MCS5/
pwr45/ch149/ov0.5, 30M offered, reps deterministic): the ceiling is SwEncoder
repair GF cost on the drone's 2x Cortex-A7 — window-scaled, NOT devourer
(master af18c61 vs chainb-hunt f79c3e9 = identical ceilings), NOT the radio
(window-2 control pushes the same ~22.4M app / ~38M air as RS k=8).

**Kernel:** `gf::lincomb`'s ARMv7 path now processes 16 B/iteration
x4-unrolled with q-register vtbl inline asm (`neon-vtbl2-q16` in banners,
commit 2d6da89; freshly written, zfex-technique, verified byte-identical vs
zfex over random unaligned cases on target). Drone kernel: 156 -> 271 MB/s at
symbol 164, 200 -> 442 MB/s at 1450B (parity with wfb-ng's zfex). Live TX
ceiling at w128: 8.97 -> 11.4M app; w64: 13.3 -> 15.9M app; 0% pkt loss at
ceiling both ends of the change.

**Two stacked ceilings** (measure against both when tuning): (1) the
transmission ceiling — USB URB feed + MCS5 airtime — at ~22.4M app / ~38M
air with ov 0.5 (RS and window-2 both sit on it); (2) the GF ceiling, which
scales with window count and now sits at 15.9M (w64) / 11.4M (w128) with
164B symbols.

**Symbol size is the remaining big lever** (this is why wfb-ng swfec reaches
full raw bandwidth: their symbols are whole ~1400B packets). GF demand per
app byte is ov x window *count*, but small symbols run the kernel in its
slow regime (271 vs 442 MB/s) and pay per-symbol overheads 8.5x more often.
On-air, window 64 / ov 0.5 / MCS5: sym 164 = 15.9M, sym 656 (bpb 4) =
20.3M, sym 1312 (bpb 2) = 22.35M app — flush against the transmission
ceiling, all 0.00% pkt loss. Bigger symbols also widen the window's airtime
span (64 x 1312B ~ 30ms at 22M vs ~4ms at 164B) — more burst protection per
unit of math. Production trade-offs before adopting: symbol_size is a
both-ends config (not a wire break — header carries it, but decoder cfg
must match), low-rate layers pad/latency out bigger symbols (flush_ms), and
per-layer symbol size does not exist yet.

**Deployment state (2026-07-15):** q16 linkbench-tx deployed to the drone
(rollback: /tmp/linkbench-tx.pre-q16); q16 maburd built in out/arm but NOT
deployed; GS binaries untouched (aarch64 path unchanged).

# UPDATE 2026-07-15 (2): per-layer symbol_size shipped — sim-chosen defaults, bench-verified

**Config change (both ends, commits c7ec7ef/c8f4b34):** `fec.symbol_size` is
now per-layer (scalar fans out; 4-array sets each stream). Bundle defaults
are the burst_sim winners: `[164, 1312, 1312, 1312]` with
`blocks_per_body [4, 1, 1, 1]`, window 64 — video layers ride whole-packet
symbols (no fragmentation), layer 0 (critical NALs) keeps small ones.
Validation: per-layer `bpb*(14+sym) <= kMaxBodyBytes (2900, sbi.h)`;
bounds [32,1500] both ends.

**Why (tools/bench/burst_sim.cpp, ctest `burst_sim_selfcheck`):** body-loss
sweep over the real UepEncoder->UepDecoder at 9.1M video-shaped traffic.
mixed-164/1312 is the only config with ZERO bulk residual across periodic
bursts B=1..32 (deployed-164 breaks from B~2-4); Gilbert-Elliott ~3%/burst-5
residual 0.039% vs deployed 1.700% (~44x). Self-check pins the sim to the
analytic guarantee region — NB `L <= W*ov/(1+ov)` is a ONE-SIDED bound on
lost sources (GE suffix-chaining recovers beyond it; empirical single-layer
edge at sym1312/bpb1/w64/ov0.25 is 20 consecutive bodies).

**Bench-verify (test binaries /tmp, production restored after):** 150s live
video x2 + forced-IDR probes, per-layer config vs production 164-global:
bc=0 every layer both ends (config match proven live), abn=0 entire run
(prod carries 30), rec climbing, s1 frag_evicted 1 vs 803 (packets no
longer fragment). Forced-IDR (4x): end-to-end ord skips EQUAL (9 vs 8 per
150s) — the small IDR-driven trickle is shared, not a regression; IDR
slices ride stream 0 which still fragments at 164B in both configs (future
lever: s0 geometry, vs parameter-set latency). `mis` counter grows ~4-8/s
in BOTH configs = foreign ch149 frames, ambient, harmless. Drone maburd
CPU ~equal (72.9 vs 69.8% of 2xA7, single samples) — GF is a modest share
of daemon CPU at 9.1M; the q16+symbol wins show up in linkbench ceilings
and robustness, not daemon CPU at this rate.

**Rollout is manual and BOTH-ENDS (config-only, no wire break):** a layer
whose symbol_size mismatches is caught at the SBI layer, not by the
sliding-window decoder's bad_cfg counter — that counter is unreachable for
this failure mode. The visible signal is per-layer `sbf` (subblocks_failed)
climbing, or — when the configured stride exceeds the body region —
`bodies` incrementing while `si` (symbols_in) stays frozen at 0. Not
silent, but not bad_cfg. Production /etc configs still 164-global + the
pre-q16 binaries; staged test artifacts left at
/tmp/{maburd,maburgs}-test{,.json} on the devices.
