# Probe stream — first flights (2026-09-05)

First air data for the probe stream (PR #44, `docs/link-adaptation.md`
"Probe stream (2026-09-04)", bench acceptance in
`docs/probe-stream-findings-2026-09-04.md`, slotter fix in
`docs/probe-blanking-fix-findings-2026-09-05.md`). Two flights on the
same build, same route, one knob changed between them:

| flight | GS records | `link.probe.max_util` | airborne window | note |
|---|---|---|---|---|
| 20 | `ctl-0280`, `probe-0280`, `log/{flight,au,lat}-0020` | 0.2 | 72–454 s (381 s) | landed, drone powered off at the GS |
| 21 | `ctl-0281`, `probe-0281`, `log/{flight,au,lat}-0021` | 0.15 | 28–356 s (329 s) | crashed at 356.5 s |

Both on `/run/media/gilankpam/DVR`. `flightreport.py` for the ctl+au
pair, plus ad-hoc scripts over the S/E/P lines, the probe log and the
AU log (the numbers below name which). Pre-probe comparison flights:
`ctl-0219/0220/0221` (ctllog 9, the 2 s ENH-probe design, 2026-09-04).

## TL;DR

1. **The gate's loss quantum is 0.2 per lost body, so 0.2 vs 0.15 was
   "one lost probe per window is Clean" vs "zero"**. `u_probe = loss /
   (ov_enh/(1+ov_enh)) = loss/0.333` over a 500 ms window of ~15 bodies;
   one lost body reads exactly 0.2 and the compare is `<=`. At 0.2 (and
   at the 0.35 default) the gate admits ~6.7 % PPDU loss; at 0.15 it
   demands 60 consecutive clean bodies. Every value in 0..0.199 is the
   same gate, 0.2..0.399 the same looser one.
2. **One lost body per window is too loose for rung 5.** Flight 20
   promoted into rung 5 fourteen times, every one was demoted, eight
   within 1 s, median hold 1.0 s. Flight 21: five promotes, one fast
   fail, median hold 16 s. A rung-5 enh AU is ~5–6 agg6 aggregates and
   survives one lost aggregate, not two; at 5 % PPDU loss that is one
   unrepairable enh AU per second, which is exactly the hold time.
3. **The promote transient is not the killer.** In all 12 flight-20
   fast fails the promote IDR (50–101 kB, +60–135 ms) completed; the
   demote came 200–1400 ms later from one incomplete enh AU
   (`s3_residual`) or a burst of incomplete AUs (`residual`).
4. **Mechanics held in the air**: completion→probe p50 1.6–1.8 ms /
   p99 5.3–5.6 ms, off-profile ~1.1–1.4 bodies per rung change, zero
   partial bodies.
5. **GS card 0 heard only 51–70 % of probe bodies at mcs 3–5**, card 1
   89–98 %. Diversity is effectively single-card at range (flight-0016
   again, now per body).
6. **Flight 21's crash was not the link**: rung 5, SNR 34–35 dB, RSSI
   −11 dBm, AUs continuous to 356.12 s, e2e p50 ~50 ms, silence at
   356.5 s. The drone SoC read 81 °C at the end (flight 20 peaked at
   48 °C).
7. **No DVR from either flight**: `record-0020.mp4` is 0 B (no moov),
   `record-0021.mp4` is 8 s.
8. `flightreport.py`'s probe-lead report was contaminated by
   pre-promote edges; fixed in `a9ea738`.
9. **Latency (lat-0047 / lat-0048, §9): the demote cascades ARE the
   latency spikes.** 15–18 % of flight seconds have e2e p50 ≥ 68 ms,
   every one carried by `air`; 65–70 % of cascades spike (peak p50
   92/118 ms, max 155/309), promotes 37–49 % (p50 40/60). The excess is
   zero through the fade, starts with the first IDR, peaks ~0.8 s after
   the first demote and drains over ~2 s: one IDR per step (2–3 per
   cascade, 2–2.5× a P frame, ×2 on air under base ov 1.0) plus the
   encoder still running the OLD bitrate for up to ~1 s into the new,
   lower MCS. The drone's `txq` gauges barely see it: the backlog sits
   downstream of the TxQueue pop.
10. ⚠ `lat-NNNN` is the player's own next-free index, not flightrec's:
   flight 20/21 are `lat-0047`/`lat-0048`, and `lat-0020`/`lat-0021`
   are older sessions whose wall–mono bridge is indistinguishable.

## 1. Ladder behaviour, flight vs flight vs the old probe

`flightreport.py` EVENTS/DWELL plus an ad-hoc per-promote hold-time
pass over the E lines (hold = time from a promote to the next E line
when that line is a demote; a further promote counts as "held").

| | flight 20 (0.2) | flight 21 (0.15) | ctl-0219 | ctl-0220 | ctl-0221 |
|---|---|---|---|---|---|
| promotes / demotes | 57 / 52 | 35 / 29 | 34 / 29 | 36 / 31 | 35 / 30 |
| transitions/min | 17.1 | 11.7 | 10.8 | 11.2 | 12.9 |
| dwell mean rung | 3.34 | 3.48 | 3.05 | 2.54 | 2.68 |
| dwell % r5 | 20 | 42 | 17 | 13 | 13 |
| into rung 5: promotes | 14 | 5 | 6 | 2 | 3 |
| … demoted < 1 s / < 3 s | 8 / 10 | 1 / 2 | 1 / 3 | 0 / 0 | 1 / 1 |
| … median hold | 1.0 s | 16.3 s | 3.5 s | 28.5 s | 14.2 s |
| into rung 4: promotes → demoted (<3 s) | 16 → 6 (2) | 8 → 4 (2) | 10 → 6 (3) | 9 → 7 (5) | 9 → 7 (2) |
| into rung 3: promotes → demoted (<3 s) | 15 → 5 (3) | 9 → 2 (2) | 9 → 2 (1) | 12 → 7 (4) | 11 → 5 (3) |
| fade demotes | 7 | 7 | 15 | 11 | 7 |

Flight 21's 42 % at rung 5 is mostly the final 97 s hovering next to
the GS at 34–35 dB, so its mean rung is not a like-for-like gain. The
rung 3/4 columns are the same across all five flights: the gate change
only moved the rung-5 boundary, which is where the loss is.

Promote latency ~3.07 s per rung on a cold climb (5 rungs in 12.3 s in
both flights), matching the bench's ~3.1 s.

## 2. The gate quantum

`LadderController::update_probe_gate`: `probe_u_ = h.probe_loss /
budget_enh_for(idx_+1)` with `budget = ov/(1+ov) = 0.5/1.5 = 0.333`,
Clean iff `probe_u_ <= max_util`. `h.probe_loss` is the 500 ms
`S1LossWindow` over ProbeTrack's block counters — 4 blocks per body,
~15 bodies per window at ~27–30 enh AU/s.

S-line `probe_u` value histogram, flight 20 (4,900 samples with a probe
commanded): 0 ×3953, **0.2 ×369**, 0.4 ×194, 0.6 ×120, 0.214 ×107,
0.188 ×106, 0.8 ×49, 0.429 ×42. 0.2 = one body of 15 (0.0667 / 0.333);
0.188 and 0.214 are one body of 16 and 14. Flight 21 has the same
comb. There is no value between 0 and 0.188.

So:

| `max_util` | lost bodies per 500 ms window still Clean | ≈ PPDU loss admitted |
|---|---|---|
| 0.0 – 0.199 (flight 21 at 0.15) | 0 | none over the 2 s streak (~60 bodies) |
| 0.2 – 0.399 (flight 20 at 0.2; the 0.35 default / `down_util` fallback) | 1 | ~6.7 % |
| 0.4 – 0.599 | 2 | ~13 % |

Flight 20's fast fails were mostly admitted from the flapping regime:
6 of the 12 promotes that failed within 1.5 s had `probe_u` 0.2–0.64
within the 2.5 s before the promote; 32–35 % of all rung-5-probing
S samples read ≥ 0.2 (flight 21: 42–45 %).

## 3. Why one lost body per window is too loose

AU sizes from `au-0020.log` joined to the rung timeline (bodies at
1328 B):

| rung | base AU p50 / p90 (bodies) | enh AU p50 / p90 (bodies) |
|---|---|---|
| 3 | 21.5 / 26.2 kB (16) | 14.2 / 17.4 kB (11) |
| 4 | 32.4 / 38.6 kB (24) | 21.7 / 26.4 kB (16) |
| 5 | 38.8 / 47.1 kB (29) | 27.7 / 37.3 kB (21) |

At rung 5 an enh AU is ~21 bodies + 50 % repair ≈ 31 bodies ≈ 5–6 agg6
aggregates and tolerates 33 % loss: one lost aggregate survives, two
do not. With independent per-PPDU loss p, P(≥2 of 6) ≈ 15 p²: 3–5 % of
AUs at p = 5–6 %, i.e. one unrepairable enh AU per ~1 s at 27 AU/s —
flight 20's 1.0 s median hold. At p ≤ 1.5 % it is ~0.3 % of AUs, one
per ~12 s — flight 21's 16 s. Base (ov 1.0, ~10 aggregates, 50 %
tolerance) only fails on a burst, which is what the `residual` fast
fails look like (see §4). The sustainable PPDU loss at rung 5 is
~1–2 %, not 6.7 %, and the gate has to be set below the quantum.

Probe body loss per MCS (seq gaps, card union) and by card-1 SNR:

| | flight 20 | flight 21 |
|---|---|---|
| mcs3 | 2.4 % (2145 rx) | 8.9 % (1447) |
| mcs4 | 5.0 % (2134) | 9.7 % (1266) |
| mcs5 | 4.8 % (3505) | 8.9 % (1107) |

Flight 20 mcs5 by SNR: 8 % at 12–20 dB, 4 % at 21–23, 0.5 % at 24–26,
0 above 30. mcs3/4 are ≤ 1 % above 21 dB. Lost-run lengths, flight
20: 211 singles, 32 doubles, 16 runs of 3–6 (a run of 6 ≈ 200 ms).

## 4. Fast-fail post-mortems (flight 20, AU log)

For each promote followed by a demote within 1.5 s, the AUs completing
between the promote and the demote (`flags` bit 0x80 = complete, 0x01
= IDR; `t_complete` mono):

| promote | demote after | first IDR | incomplete AUs before the demote |
|---|---|---|---|
| 4→5 @108.9 s | s3_residual 763 ms | +122 ms 69 kB ✓ | 1 enh (12.6 kB partial) @+611 |
| 4→5 @135.7 s | s3_residual 511 ms | +103 ms 62 kB ✓ | 1 enh @+578 |
| 2→3 @157.0 s | s3_residual 658 ms | +135 ms 51 kB ✓ | 1 enh @+374 |
| 4→5 @192.3 s | residual 211 ms | (none yet) | 5 AUs both layers @+79…+282 — a burst |
| 3→4 @209.6 s | s3_residual 1462 ms | +92 ms 78 kB ✓ | 1 enh @+1374 |
| 2→3 @214.5 s | s3_residual 1329 ms | +136 ms 29 kB ✓ | 1 enh @+1040 |
| 4→5 @228.4 s | residual 359 ms | +106 ms 101 kB ✓ | 2 base @+245/+297, then 2 enh |
| 4→5 @273.6 s | residual 204 ms | +90 ms (IDR itself incomplete, 6.8 kB) | 3 AUs @+90…+195 — a burst |
| 4→5 @283.9 s | s3_residual 554 ms | +62 ms 66 kB ✓ | 4 AUs @+334…+606 |
| 4→5 @351.3 s | residual 971 ms | +89 ms 100 kB ✓ | 2 AUs @+902/+952 |
| 4→5 @373.2 s | residual 465 ms | +104 ms 80 kB ✓ | 2 enh @+264/+473 |

In 11 of 12 the promote IDR — 2–2.5× the base p50, at the new MCS,
within ~100 ms — is complete; the SuperFrame cap is doing its job and
the demote is steady-state loss at the new rung, not the transition.
Every demote is followed by another IDR (bitrate change), so a fast
fail costs two IDRs plus the ~300 ms cascade.

## 5. Probe mechanics in the air

- Completion→probe first sight (enh `t_complete` → `first_ms`): flight
  20 p50 1.78 / p90 3.02 / p99 5.26 ms over 8,897 joins; flight 21
  1.57 / 3.09 / 5.59. Bench was 0.9 / 4 ms. Per-MCS p50 1.4–2.1 ms.
- `link.probe.off_profile` at the end: 120 over 110 transitions
  (flight 20), 92 over 66 (flight 21) — inside the 2–6 per change
  expectation, `kProbeSwitchBlankMs` 150 is long enough.
- 0 partial bodies in 15,000 rows: blocks are lost all-or-nothing,
  i.e. whole-PPDU loss, as on the bench.
- `link.rcf_slot` at the end of flight 20: probe-released sends 4287,
  AU 3187, passthru 2246, timeout 796.
- Per card, bodies heard / expected at mcs3/4/5: card 0 51/67/68 %
  (flight 20), 57/64/70 % (flight 21); card 1 98/95/95 % and 89/89/89
  %. The union loss (2.4/5.0/4.8 %) is card 1's loss minus a sliver.

## 6. Probe as a demote predictor (v2 input)

`flightreport.py` before `a9ea738` took the last lossy edge before a
demote, so an edge from the pre-promote flapping became a "lead" on the
hold that followed (8,864 ms at 136.2 s, where the promote was at 135.7
s). Bounded to edges after the last transition, and counting an edge
the gate overrode with a promote as a false alarm:

| | flight 20 | flight 21 |
|---|---|---|
| lossy edges | 117 | 55 |
| false alarms (next E not a demote within 10 s) | 60 | 34 |
| episodes with a lossy edge inside the hold | 12 of 29 | 11 of 19 |
| lead p10 / p50 / p90 (ad-hoc, first edge) | 0.6 / 1.5 / 6.1 s | 0.4 / 2.3 / 8.5 s |

17 of flight 20's 29 episodes had no warning inside their hold — most
of them the rung-5 fast fails, where the probe is off (top rung). A v2
probe-driven demote cannot see the top rung by construction; that
needs `rung_offset 0`-style self-probing or the residual path it has
today.

## 7. Flight 21 — the crash, and the drone temperature

From 259.7 s the drone held rung 5 with `u` ≈ 0, SNR 34–35 dB, RSSI
climbing from −45 to −11 dBm by 354.5 s (drone at the GS). The AU log
is continuous to 356.12 s (max gap in the last 30 s 72 ms, 1 incomplete
AU), `lat:` e2e p50 49–59 ms; the S line goes `nan` at 356.2 s and
`starved` fires at 356.48 s. The 448–453 s reconnect (`noinfo → clean`,
0→1 `promote_probed`, `starved` again) is the still-powered drone heard
briefly. Nothing on the link side precedes the loss.

`drone.sys.soc_temp_c` (SigmaStar thermal zone, telemetry-only,
`thermal_delta` is devourer's radio reading and nothing acts on
either):

| t | 24 s | 84 | 144 | 204 | 265 | 325 | 385 |
|---|---|---|---|---|---|---|---|
| flight 21 °C / delta | 36 | 48 / 4 | 48 / 4 | 51 / 5 | 63 / 7 | 76 / 10 | 81 / 12 |
| flight 20 °C / delta | 35 | 46 / 3 | 46 / 3 | 47 / 3 | 46 / 3 | 44 / 2 | 46 / 2 |

Flights 15–17 peaked at 50–55 °C, the long bench sessions 18/19 at
70–73. 81 °C is the highest on the DVR and it was climbing ~10 °C/min
from 204 s. The encoder was at 19.1 Mb/s against a 16 Mb/s command
with 5 `venc_full_drops` at the end. Whether it relates to the crash is
not answerable from the GS logs; it is the one anomaly.

Both flights show sporadic one-second `air` p50 spikes of 50–115 ms
(flight 21 at 110–150 s, flight 20 at 139/170 s), at demote times and
range, with `dq`/`enc` flat — not new, not near the crash.

## 8. Recommendations

1. **`link.probe.max_util` stays below 0.2** (0.15 is fine; note in the
   config that the knob's real steps are 0.2 apart). Raise
   `link.probe.clean_ms` from 2000 toward 3000–4000: at 3 % PPDU loss a
   60-body streak passes 16 % of the time, a 120-body one 3 %, and the
   ladder retries the streak continuously while parked at rung 4.
2. Re-fly with those two and compare the into-rung-5 row of §1.
3. Card 0's range collapse is now measurable per body from the probe
   log (`card_mask`); worth a look at the antenna/feed before tuning
   anything else around single-card loss.
4. The drone temperature deserves a thermal check on the airframe.
5. DVR: both files unusable; the GS logs carry no DVR state, so check
   the button/autostart path on the next ground run.

## 9. Latency — the cascades are the spikes

Player `lat:` lines from `lat-0047` (flight 20) and `lat-0048` (flight
21) — see the pairing warning below — plus a per-frame replay of the
player's `air` arithmetic over the AU log (`t_first − enc − q − pts`
minus a leaky running-min floor, i.e. `PtsAnchor`), joined to the E
lines.

| | flight 20 | flight 21 |
|---|---|---|
| e2e p50-of-seconds: median / p90 / max | 53 / 82 / 169 ms | 53 / 83 / 303 ms |
| e2e p99-of-seconds: median / p90 / max | 72 / 123 / 247 | 67 / 130 / 376 |
| segment p50 medians enc/dq/air/fec/dec/reg/dsp | 7/1/3/11/7/12/5 | 7/1/2/12/7/13/5 |
| spike seconds (e2e p50 ≥ 68) | 65 of 371 (17.5 %) | 47 of 327 (14.4 %) |
| … dominant segment | air 65/65 | air 47/47 |
| … within 2.5 s of: cascade / promote / single / none | 22 / 19 / 3 / 21 | 13 / 19 / 6 / 9 |
| cascades that spike within 2.5 s | 11 of 17 (65 %) | 7 of 10 (70 %) |
| single demotes that spike | 2 of 11 | 4 of 7 |
| promotes that spike | 21 of 57 (37 %) | 17 of 35 (49 %) |
| per-frame air excess, whole flight p50 / p90 / p99 | 4 / 42 / 99 ms | 3 / 41 / 129 ms |

Cascade profile (per-frame air excess, 250 ms bins, n = 17 + 10):

| | flight 20 | flight 21 |
|---|---|---|
| excess in the 1 s BEFORE the first demote, p50 | 5 ms (0 of 17 ≥ 20 ms) | 4 ms (1 of 10) |
| peak after the first demote, p50 / max | 92 / 155 ms | 118 / 309 ms |
| time to peak, p50 | 0.82 s | 0.80 s |
| settled (< 15 ms for 5 frames), p50 | +1.8 s | +2.4 s |
| IDRs in the 1.5 s after, p50 / max | 2 / 4, 65 kB / 178 kB | 2 / 3, 117 / 174 kB |
| single demote: peak p50 | 40 ms | 89 ms |
| promote: peak p50 / p90 | 40 / 105 ms | 60 / 131 ms |

Three things the profile says:

1. **The fade itself adds no latency.** Excess is ~0 up to the first
   demote in 26 of 27 cascades; loss is paid in `fec`, not `air`.
2. **The spike is a drain.** It starts with the first IDR (every step
   of a cascade is a bitrate change, every bitrate change is an IDR —
   `docs/link-adaptation.md`, venc attr-change), rises for ~0.8 s and
   decays for another 1–1.5 s: a backlog being served at the new,
   lower MCS. Two inputs fill it: 2–3 IDRs at 2–2.5× the P-frame size,
   ×2 on air under base ov 1.0 (192.5 s in flight 20: 63 + 46 + 40 kB
   of IDR inside 500 ms at mcs2 ≈ 120 ms of airtime by itself), and the
   encoder still producing the OLD rung's bitrate for up to ~1 s after
   the RCF has already dropped the MCS (211.2 s: mcs 3, encoder 13.7
   Mb/s, `air_pct` 117 %; 155.9 s flight 21: mcs 0, encoder 8.7 Mb/s,
   command 8700 → 2200 only at 156.1). Bytes on air in the first 500 ms
   after a cascade were 70–115 % of the new rung's NOMINAL PHY rate
   (185 % for the 3→0 at 155.0 s, the 309 ms peak).
3. **The drone's queue gauges barely see it.** `dq` (per-frame q_ms)
   stays at 1 ms p99 and `txq.depth` reads 0–6 while `air` carries
   100–150 ms; only `txq_wait_ms` blips to 20–56 ms for one sample.
   The backlog sits past the TxQueue pop — USB pool / bulk-out / chip
   FIFO — where the congestion shed (`docs/link-adaptation.md`, half
   cap) cannot trigger and nothing is measured.

Promotes spike less and shorter: one IDR at the new (higher) bitrate,
roughly its own serialization time (40–60 ms p50), no sustained
overshoot because capacity went up.

**Rung 2 standing excess (flight 20 only).** Frames > 3 s from any
transition: r4 3 ms / r5 2 ms p50, but r2 62 ms p50 / 81 ms p99 (235
frames, 161–171 s). Encoder 6.6–6.9 Mb/s on a 6500 command, `air_pct`
66–77 %, `txq` 0 — a standing ~60 ms backlog at mcs2 with no gauge
admitting to it. Flight 21's rung 1/3 sat at 2–5 ms, so it is a
window, not a rule; same class as the open mcs1 budget overshoot
(memory `mcs1-budget-overshoot`).

Levers, not done: coalesce a cascade's bitrate writes into one (one
IDR per episode instead of one per step; the ladder already knows it
is stepping at 150 ms), have the bitrate step-down land with the MCS
step rather than ~1 s later, or size the demote IDR against the NEW
rung's budget. Any of them is testable on the bench with the loss-sim
cascade and `flightjitter.py`'s `rung-change` class.

**⚠ Pairing `lat-NNNN` to a flight.** The lat index is maburplay's
own next-free counter; flightrec's `au`/`flight` index and maburgs'
`ctl` index are separate counters, and the player restarts far more
often. Flight 20 is `lat-0047` (mono 71–443 s), flight 21 is
`lat-0048` (27–357 s); `lat-0020`/`lat-0021` are 33-min and 5-min
sessions from earlier boots that happen to overlap the same mono
range, and the first pass of this analysis was run on them. The
`# sync` wall–mono bridge does NOT disambiguate: the GS RTC restarts
at the same bogus epoch every boot, so every lat and au log on the DVR
carries one of two bridge values (…208.530 or …208.591). Match by mono
span against the ctl log (first/last `S` line) and take the highest
index as the latest boot.

