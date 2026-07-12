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
