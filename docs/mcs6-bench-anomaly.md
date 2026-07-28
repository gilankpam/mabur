# mcs6 bench anomaly: dirty at bench range, unresponsive to TX power (2026-07-27)

**Status: ROOT-CAUSED + FIXED IN PROD 2026-07-29** (see "Root cause"
section at the bottom). mabur's FEC geometry (328×4 → 1396-byte air
frames) lands exactly on a narrow Jaguar3 PHY corner: at **mcs6 + STBC,
frames whose air length falls in a ±4-byte window (Nsym-48, nearly-full
last OFDM symbol) vanish whole at RX** — 8% loss at 1396 B, 24.5% at
1400 B, clean at 1388/1404 B. **Adopted on the rig same day:
`fec.symbol_size: 332` both ends + full 6-rung ladder restored with the
mcs6 rung at ov 0.25** (belt-and-suspenders). Acceptance: sym332 clean at
wall power on all rates (positive control mcs6@328 dirty); mcs6/ov0.25
pin = 0.0% hard truncation, 59.5 fps; adaptive re-park gate 0/0/59.5.
The mcs6-skip ladder is retired. Rollback: `*.pre-adopt332` configs on
both devices (drone had no python3 — config edits there are sed-only).

## Symptom

With the measured-loss ladder controller (PR #8) and `max_mcs: 7`, the bench
link parks on the mcs6 rung and bleeds video: rtpsniff shows 6–9% bad frames
(`end_lost_hard` FU tails — enhance-frame territory), ok_fps ~53–55. The
neighbouring rungs are clean at the same bench geometry, same day, same
binaries:

| op (parked) | rtpsniff 30 s | notes |
|---|---|---|
| mcs5 / ov0.25 | 0 gaps, 1/1786 bad (0%), 59.5 fps | ladder park, `max_mcs: 5` |
| **mcs6 / ov0.15** | **0 gaps, 112–158/~1750 bad (6–9%), 53–55 fps** | ladder park, `max_mcs: 6` |
| mcs7 / ov0.25 | 0 gaps, 0/1784 bad, 59.5 fps | static pin |
| mcs7 / ov0.10 | 0 gaps, 0/1783 bad, 59.4–59.5 fps | ladder park, mcs6-skip ladder |

Sideport agrees it is real loss, not the repair-race accounting artifact fixed
in 8c26e88: at mcs6 s1 `recovered_s` ≈ 152/s vs `recovered_arrived_s` ≈ 46/s
(≈100 syms/s genuinely never arrived, FEC repairing them; occasional residual
demotes 6→5). At mcs5/mcs7 `recovered_arrived` tracks `recovered` almost 1:1
(races, no real loss).

The inversion is the puzzle: mcs6 (64-QAM 3/4) needs ~1 dB *less* SNR than
mcs7 (64-QAM 5/6), and per the wall table transmits at most ~0.5 dB hotter —
yet mcs7 is spotless and mcs6 is not.

## TX-power hypothesis: tested, refuted at the ±1.5 dB scale

The per-rate power stack (offset-power, merged 2026-07-17: `power_plan.h` →
devourer `SetTxPowerRateDiffs`) makes this a drone-config experiment: each
rate parks at `walls_idx[r] − margin` at offset 0. Measured walls (qdB
TXAGC indices, 0.25 dB/step): mcs5 = 56, mcs6 = 51, mcs7 = 49.

Sweep of `radio.rate_walls_idx[6]` on the drone, GS ladder parked at mcs6,
30 s rtpsniff gate per point:

| mcs6 wall | Δ power vs measured | bad frames |
|---|---|---|
| 51 (measured) | 0 | 9% |
| 49 (= mcs7 parity) | −0.5 dB | 6% |
| 45 | −1.5 dB | 9% |

No monotonic response — the spread is run-to-run variance. GS-side per-card
SNR EMA did not shift measurably across the sweep either. Conclusion: at
bench range, mcs6's bleed is **not** a sub-dB TX power/overdrive problem, and
backing its wall off further only donates link budget for nothing. Wall
restored to the measured 51.

**Beware the units gotcha that motivated this doc:** `rate_walls_idx` /
`wall_margin_db` math is quarter-dB (`m = margin_db * 4`,
`drone/src/power_plan.h`). An "idx 51 → 45" edit is −1.5 dB, not −3 dB.

## Condition drift

The offset-power acceptance matrix (2026-07-17, same walls, same rig) had
**100% delivery on all MCS 0–7** at offset 0 — mcs6 included. Whatever hurts
mcs6 today (antenna geometry, multipath at this exact rate's
interleaving/timing, an RX-side rate-specific behavior) appeared between then
and 2026-07-27. That, plus the mcs6-worse-than-mcs7 inversion, is why this is
flagged for the attenuation rig rather than more config bisection: the rig
can sweep received power over a wide controlled range and separate TX
distortion from RX effects per rate.

> **Resolved 2026-07-29: there was no drift.** The 07-17 acceptance ran
> 64-byte txagcbench frames — far from the 1392–1400 B hole — and nothing
> before the ladder ever parked real ~1.4 KB traffic on mcs6. The corner
> was always there, untested. See "Root cause" below.

## Mitigation (bench-local, NOT shipped defaults)

Bench GS `/etc/maburgs.json` runs an explicit ladder without the mcs6 rung:

```json
"link": {
  "static_mcs": -1,
  "max_mcs": 7,
  "ladder": [
    {"mcs": 0, "overhead": 1.0},
    {"mcs": 2, "overhead": 0.5},
    {"mcs": 4, "overhead": 0.25},
    {"mcs": 5, "overhead": 0.25},
    {"mcs": 7, "overhead": 0.10}
  ]
}
```

Climbs 0 → mcs7/ov0.10 in ~60 s and parks clean (gate above). Known
trade-off: a demote from mcs7 now falls to mcs5 (~19 Mbps src, −40%) instead
of mcs6 (~26 Mbps) — acceptable on the bench, wrong for flight where mcs6 at
range is presumably fine. Shipped defaults keep the full 6-rung ladder.

## Packet-level dissection (2026-07-29, bench, static mcs6/ov0.15 pin)

Step 3 executed at bench range: 20 s `fu_probe` + 60 s `seqdump` + 60 s
sideport capture, GS pinned `static_mcs: 6` / `static_overhead: 0.15`.
Three independent views agree exactly; the video-level mechanism is now
settled.

**Which bodies die: s3 enhance-frame tails, nothing else.**

- Frame dissection of the seqdump (group by rtp_ts, FU chain completeness):
  8.8% of frames truncated; **303 of 308 truncated frames are TRAIL_N
  (SVC-T enhance, stream 3)**, 5 are base. Truncated frames carry a median
  51% of the clean-median frame bytes — cut mid-frame, not at the head.
  Timing is a steady drizzle: median 140 ms between truncations, max clean
  gap 1.08 s in the whole minute — constant-rate loss, not bursts.
- `fu_probe`: 8.7% hard truncation, `missing=0` seqs. **Caveat discovered:
  fu_probe's "SOURCE OVERLOAD" verdict is stale under frame-shm** — maburgs
  regenerates the RTP seq stream, so link loss can never appear as seq
  gaps; `missing=0` + hard ends no longer implies source-side truncation.
- Sideport, same window: the **drone is exonerated** — radio drops 0/s,
  usb_fail 0, txq drops 0 (depth 0 throughout), venc `ring_drops` 0,
  `failsafe_shed` false, encoder steady at 7.7 Mbps of cmd 8000. Nothing
  is shed or truncated at the source.
- The kill is air loss × UEP: both streams lose ~3% of symbols genuinely
  (never arrive, race-corrected): s1 `recovered−recovered_arrived` ≈
  90 syms/s of ~3150/s in (2.8%); s3 ≈ 29+51 of ~2430/s (3.3%). s1 at
  effective ov 0.45 absorbs it fully (**abandoned 1/s**); s3 at ov 0.15
  cannot: **abandoned 51.4 syms/s** ≈ 5.1 dead frame-tails/s — which is
  exactly the 308 truncations / 60 s the seqdump counted.

So the causal chain at mcs6 is: **~3% genuine pre-FEC air loss (mcs6-
specific, absent at mcs5/mcs7) → s3's thin 0.15 overhead can't repair →
enhance tails die mid-frame at ~5/s**.

**Why 15% overhead loses to 3% loss — and the ov0.25 experiment
(2026-07-29).** Bodies die whole and `bpb: 4` puts 4 consecutive symbols
in each body, so 3% *body* loss arrives as 4-symbol clusters. An s3
window (w32) at ov 0.15 holds ~4.8 repair symbols — one dead body is
borderline, two in a window (~2–3% of windows at this loss rate) is
unrecoverable. Prediction: ov 0.25 (~8 repair syms ⇒ survives 2 dead
bodies) should collapse the residual. Measured, static mcs6/ov0.25 pin,
op verified on both ends, raw air loss unchanged (~2–3% both streams):

| | mcs6/ov0.15 | mcs6/ov0.25 |
|---|---|---|
| s3 abandoned syms/s | 51.4 | **9.1** |
| fu_probe hard truncation | 8.7% | **1.2%** |
| s1 abandoned syms/s | 1.0 | 0 |

A 7× reduction from +0.10 overhead (~9% airtime, no delivered-rate cost
under ONE-RATE), exactly on the window-burst arithmetic; the residual
1.2% is the 3-dead-bodies-in-a-window tail. Consequence: an
`{mcs 6, overhead 0.25}` rung is *usable* on this bench (base layer
clean, ~1.2% enhance-tail loss) but not spotless like mcs5/mcs7 — the
mcs6-skip ladder stays the bench default until the PHY root cause falls.
For shipped defaults, raising the mcs6 rung from ov 0.15 to 0.25 is
cheap graceful-degradation insurance if mcs6 marginality also occurs at
range. The remaining root cause — *why*
mcs6's PHY drops ~3% raw at bench range while mcs7 is clean — is the part
that still needs the attenuation rig. This dissection also quantifies the
"s3-residual visibility" open item: the ladder's s1-based health metric
saw a healthy link (u≈0.02) while s3 was abandoning 51 syms/s.

## Next steps

1. Attenuation-rig study: per-rate delivery vs received power for mcs5/6/7,
   enough attenuation to rule RX saturation in or out (same protocol as
   `docs/txagc-calibration.md`).
2. ~~Re-measure the mcs6 wall~~ **DONE 2026-07-29**: full 8-MCS matrix
   re-run — walls reproduce the 2026-07-16 table exactly (mcs6 = 51,
   mcs7 = 49; only mcs5 moved, 56 → 54). No PA condition drift, and
   mcs6's parked idx 47 delivers 99% of raw bench frames. The bleed is
   therefore specific to the mabur traffic regime (large aggregated
   bodies / sustained airtime), not the PHY operating point — which
   sharpens step 3. See the 2026-07-29 section of
   `docs/txagc-calibration.md`.
3. ~~Dissect at the packet level~~ **DONE 2026-07-29** (section above):
   s3 enhance tails die to ~3% genuine air loss × ov 0.15; drone-side
   mechanisms all exonerated. What remains for the rig is only the PHY
   question — why mcs6 drops ~3% raw where mcs7 drops ~0.
4. s3-residual visibility for the ladder controller is no longer abstract:
   at mcs6 the controller read u≈0.02 (healthy) while s3 abandoned
   51 syms/s. The controller needs an s3-abandonment input (or
   `layer_delivery_pct[3]`) before the full 6-rung ladder can be trusted
   to police rungs like this one.

## Root cause (2026-07-29): a ±4-byte frame-length hole at mcs6+STBC

Found by systematic isolation after the per-card sideport analysis showed
the loss is **RX-side-visible, per-card independent, and enormous before
diversity** (card0 −15%, card1 −40% at wall power; the 2-card union hides
it down to the 3% the FEC sees). All experiments below are `linkbench`
cells (no daemons), ch149, w32/bpb4/ov0.15, `--pwr-mode none`, 15 s TX,
GS card 0.

**Isolation chain:**

| experiment | result |
|---|---|
| mcs5 / mcs6 / mcs7 @ sym328 (1396 B air), LDPC+STBC | 0.08% / **5.0%** / 0.11% |
| mcs6 @ sym64 (small frames) | 0.05% — size-dependent |
| mcs6 @ sym328, BCC+STBC | 4.96% — LDPC exonerated |
| mcs6 @ sym328, LDPC, **no STBC** | 0.38% — STBC is the trigger |
| mcs6 @ sym164 / sym656 (0.7 / 2.7 KB) | clean — a window, not a cliff |
| coarse sweep sym 250–500 | only 328 dirty (7.3%) |
| devourer RX A/B (pre-merge bf3cb61 vs 13998ec) | 6.5% vs 7.1% — **merge exonerated**, incl. the 16 KB-URB/agg change; bug predates it and was simply never exercised (no prior test ran mcs6 with ~1.4 KB frames) |
| fine sweep, ±4 B air steps: sym 324/326/**328**/329/330/332/336 | 0.10 / 0.10 / **8.2** / **24.5** / 0.04 / 0.09 / 0.05 % |
| next-Nsym-window low-pad probe sym 343 (pad 30) | clean — not a pure "low pad" law |
| sym332 across mcs 0/2/4/5/7 (+ mcs6 earlier) | all ≤0.11% — escape size validated at every ladder rate |
| **daemon-level proof**: symbol_size 332 both ends, static mcs6/ov0.15 pin | fu_probe **0.1%** hard truncation (was 8.7%), 59.4 fps |

**The law as measured:** at mcs6 (64-QAM 3/4) with STBC, air MPDUs of
1392–1400 bytes (the Nsym = 48 window with a nearly-full final OFDM
symbol; pad 42 bits → 8%, pad 10 bits → 24.5%) are lost whole at the
receiver — no CRC error, no detection, `crc_bad = 0` throughout. ±8 bytes
of air length escapes completely. Same lengths at mcs5/mcs7 (which have
different Nsym/pad landings) are clean; mcs0/2/4 clean at the validated
escape size. Loss is probabilistic per frame, independent per RX card,
and worsens with RX level (daemon runs at wall power lost 15–40% per
card vs 5–8% at `none` power) — consistent with the reproducible
per-index dips ("comb") in the 2026-07-29 wall re-measurement being the
same class of effect.

**Attribution (open):** TX is the drone 8812EU, RX both GS 8822E — all
Jaguar3 family. Whether the frame is corrupted at TX (vector/modulator
corner) or dropped at RX (demod corner) is not yet separated; a
direction-swapped linkbench (needs cross-building rx for arm / tx for
arm64) or a third-party sniffer would decide. Practical impact is
identical either way on this hardware pair.

**Fixes, in preference order:**

1. `fec.symbol_size: 332` (both ends) — validated clean at every ladder
   rate and at the daemon level. Costs ~1.2% air efficiency vs 328.
   Any other size whose air length dodges the hole works equally;
   *before shipping, sweep the candidate size across all 8 rates* (the
   hole is (rate × length)-specific and other holes may exist).
2. STBC off — reduces mcs6 to 0.38% but forfeits STBC diversity at every
   rate; inferior.
3. mcs6-skip ladder (current bench mitigation) — wastes a rung.

**Residual questions** (curiosity, not blockers): the RX-level dependence
(attenuation rig would characterize the comb); TX-vs-RX attribution;
whether the corner is silicon errata (worth reporting to devourer
upstream as a Jaguar3 hardware note either way).
