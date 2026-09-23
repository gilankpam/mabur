# HT40 per-MCS air capacity sweep — 2026-09-23

Question: what does 40 MHz actually deliver per MCS on our hardware, and
does the 20 MHz efficiency (0.71–0.78 of nominal,
`docs/bandwidth-sweep-findings-2026-09-17.md`) carry over? First step of
the 40 MHz experiment; nothing in maburd/maburgs flies HT40 yet.

Method: identical to the 2026-09-17 sweep — `linkbench-tx` (drone) →
`linkbench-rx` (GS card 0), ch136, prod FEC geometry (symbol 332, bpb 4,
window 32, overhead 0.5), `--pwr-mode none`, tx-threads 4, offered load
above capacity at every point (`under target` every second, checked per
log), 17 s TX, median of the steady seconds. Both daemons stopped;
restored after (ausniff 0 gaps / 0 incomplete / 0 resyncs at the
disarmed low-power 30 fps). New: `--bw 20|40` on both linkbench tools
(at the time: always HT40+, `SelectedChannel{ch, 1, CHANNEL_WIDTH_40}`, so
ch136 = 136/140, RF centre 138 — **off the standard 5 GHz grid**, see
"Correction" below; 149 = 149/153 was on-grid) and a `width_mhz` field on
`RadioFrontend::Cfg` (default 20). "Clean air" = rx air × (1 − crc_bad/frames).

**LDPC+STBC on (as every prod rung flies, `common/src/profile.cpp`)**
unless noted. The HT20 flags-matched control (mcs2/5/7 agg6: 13.78 /
39.38 / 49.12) reproduces the 2026-09-17 flags-off table within 0.6 %, so
the coding flags do not move capacity and the two tables compare directly.

## Delivered air, HT40, clean

| mcs | HT40 nominal | agg6 (prod) | eff | singles | eff | HT20 agg6 (09-17) | HT40 / HT20 |
|---|---|---|---|---|---|---|---|
| 0 | 13.5  | 10.33  | **0.77** | 11.69 | 0.87 | 5.66  | 1.83 |
| 1 | 27    | 20.21  | **0.75** | — | — | 10.17 | 1.99 |
| 2 | 40.5  | 30.31  | **0.75** | 28.79 | 0.71 | 13.85 | 2.19 |
| 3 | 54    | 40.53  | **0.75** | — | — | 19.78 | 2.05 |
| 4 | 81    | 60.21  | **0.74** | — | — | 29.59 | 2.03 |
| 5 | 108   | 82.82  | **0.77** | 52.92 | 0.49 | 39.47 | 2.10 |
| 6 | 121.5 | 92.58  | **0.76** | — | — | 45.34 | 2.04 |
| 7 | 135   | 101.02 | **0.75** | 59.67 | 0.44 | 49.42 | 2.04 |

Repeats: mcs2 agg6 30.29 (vs 30.31), mcs7 agg6 98.98 (vs 101.02 — mcs7
seconds swing 95–103). `--tx-threads 8` at mcs7 agg6: 102.84, drone CPU
65 % busy — the ceiling is still the air, not USB or the A7. CRC-bad ≤ 2
frames per point; post-FEC pkt loss 0–0.2 %, and the same 0.14–0.18 % shows
at HT20 mcs5/7 today, so it is not an HT40 effect.

## Findings

1. **HT40 delivers ~2.0–2.2x HT20 at every MCS from 1 up** (1.83x at mcs0,
   where HT20's 0.87 was the outlier). mcs7 agg6 carries ~100 Mbit/s of air.
2. **Efficiency is flat at 0.74–0.77 across all eight MCS**, mcs0 included —
   HT40 loses HT20's rung-0 exception. A per-width efficiency table is
   needed if HT40 rungs ever fly: `[0.77,0.75,0.75,0.75,0.74,0.77,0.76,0.75]`.
3. **A-MPDU pays from mcs2 up at HT40** (HT20: from ~mcs4). Singles win only
   at mcs0 (0.87 vs 0.77); at mcs5/7 singles collapse to 0.49/0.44 — frames
   are half as long on air, so the fixed per-PPDU cost doubles its share.
   `ampdu.min_mcs` would be 1 or 2 for HT40 rungs, not 4.
4. **LDPC+STBC are mandatory at HT40 64-QAM.** Flags OFF, the same bench
   link failed CRC on 37 % (mcs6) and 80–96 % (mcs7) of frames at RX SNR
   ~22 dB; mcs4/5 already showed a few hundred CRC fails. A TX-power sweep
   on mcs7 singles (flags off) was non-monotonic — offset −24/−12/0/+8/+16
   qdB → 0 / 0.6 / 4 / 15 / 0.2 % frames clean — i.e. SNR-starved below and
   PA-compressed above, no clean window. With the flags ON: 0 CRC fails
   through mcs7. The ~3 dB HT40 SNR penalty lands right at the 64-QAM edge
   on this bench; the coding/diversity gain buys it back. Range margin
   at HT40 64-QAM is therefore thin — a flight question.
5. Harness gotcha: zsh arrays are 1-indexed (`${a[0]}` is empty → `--bitrate M`
   → usage error; every other index shifts by one MCS). Every sweep point was
   re-checked TX-bound from its own log; mcs0/1 were re-run with explicit rates.

## Second channel: 149/153 (HT40+, RF centre 151 — an 8822E spur combo)

Same method, LDPC+STBC agg6:

| mcs | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| ch136/140 | 10.33 | 20.21 | 30.31 | 40.53 | 60.21 | 82.82 | 92.58 | 101.02 |
| ch149/153 | 9.41 | 18.90 | 28.45 | 38.31 | 57.96 | 79.66 | 89.35 | 96.66 |
| ratio | 0.91 | 0.94 | 0.94 | 0.95 | 0.96 | 0.96 | 0.97 | 0.96 |

Unlike HT20 (channel-independent within 0.5 %), the 149/153 pair runs
3–9 % low, with wider second-to-second swing and 2–6x the mac_lost.
The deficit is on the TX side (drone puts 825 vs 901 frames/s on air at
mcs0; GS hears all of them). NOT carrier sense: `--no-cca` (new flag,
`dev_cfg.tuning.disable_cca`) reads the same (9.78 vs 9.89 with CCA,
136 no-CCA 10.36). NOT a busy half: HT20 mcs2 on ch149 13.65, ch153 13.77,
ch136 13.76. Cause open; the spur-combo centre (devourer applies an NBI
notch / CSI mask / packet-detect tweak there) is the leading suspect —
check 157/161 (centre 159, also a spur combo) vs 36/40 (not) to confirm.

## TX-power walls at HT40 vs `rate_walls_rel`

Method: `linkbench-tx --wall-sweep` + two `linkbench-rx --wall` (both GS
cards). Same frames as maburcal (`mabur/cal_wire.h` 64-byte payloads,
singles), same anchor read (mcs7 ref with rate diffs zeroed), every rel
index −41..63 at full resolution, 100 frames/cell, and the same wall rule
(`gs/src/cal_analysis.cpp`: end of the first contiguous ≥90 % run on the
best card; top-of-range = no_dip → rail 63). ch136, anchor **39 at both
widths** (the efuse reference devourer reads does not move with bandwidth
here). 1 rel index = 1 qdB = 0.25 dB.

| mcs | config | A HT20 off | D HT20 LDPC+STBC | B / B2 HT40 off | C / C2 HT40 LDPC+STBC |
|---|---|---|---|---|---|
| 0 | 63 | 63 no_dip | 63 no_dip | 63 / 63 no_dip | 63 / 63 no_dip |
| 1 | 63 | 63 no_dip | 63 no_dip | 63 / 63 no_dip | 63 / 63 no_dip |
| 2 | 62 | 63 no_dip | 63 no_dip | 63 / 61 | 63 / 63 no_dip |
| 3 | 45 | 41 | 63 no_dip | 44 / 39 | 63 / 63 no_dip |
| 4 | 22 | 26 | 45 | 24 / 20 | 46 / 46 |
| 5 | 6 | 7 | 25 | 11 / 11 | 30 / 30 |
| 6 | 9 | 2 | 24 | −23 / −33 narrow | 30 / 30 |
| 7 | 5 | 6 | 22 | −8 / −6 narrow | 24 / 25 |

Peak RSSI −49..−64 dBm on every row, never `saturated`. `card_disagree`
fires on several rows (the second card's first run ends a few indices
off); the best-card walls reproduce within 0–5 (B/B2) and 0–1 (C/C2).

1. **The harness reproduces the configured walls.** Run A (maburcal's
   exact recipe: HT20, flags off) lands within 0–7 of `rate_walls_rel`,
   inside the 0–9 same-channel repeatability in `docs/calibration.md`.
2. **With the prod flags, HT40 walls are ≥ HT20 walls** — mcs4 46 vs 45,
   mcs5 30 vs 25, mcs6 30 vs 24, mcs7 24–25 vs 22. The shipped table is
   therefore safe for HT40 prod-flag frames (it never overdrives them).
3. **The collapse point is set by the coding, not the width.** Delivery
   curves show the cliff (≥90 % → ~0) at rel ~10–14 for mcs7 flags-off at
   BOTH widths, and ~24–28 flags-on at both. HT40 flags-off's negative
   "walls" (mcs6 −23/−33, mcs7 −8/−6, `narrow`) are not an earlier PA
   wall — delivery below the cliff hovers at 60–99 % (the same SNR/EVM
   starvation the capacity sweep hit), so the first ≥90 % run ends early.
4. **maburcal measures a frame nobody flies.** It sweeps with LDPC/STBC
   OFF (`drone/src/cal_sweep.cpp` `LayerTxSpec{HT, rate, 20}`, flags
   default false); every prod rung flies them ON (`common/src/profile.cpp`).
   With the flags the delivery wall sits **+17..+23 indices (≈4–6 dB)
   higher** at mcs4–7 at HT20 too (D vs A), and mcs3 stops dipping. Part
   of that is STBC driving the second PA chain (RSSI peaks ~5 dB higher),
   so the index→air-power mapping differs between the two frame types.
   Whether the flags-on wall is usable headroom (EVM past the old wall
   costs range margin even while frames still decode) is a separate
   question — nothing was changed; flagging it for a maburcal follow-up.

## Mixed width: 20 MHz frames into a 40 MHz-tuned receiver

The question behind "width as a per-rung TX choice": can a GS (or drone)
tuned HT40 on 136/140 keep receiving the 20 MHz frames — control
(`control_tx_mode()` is MCS0/20 MHz LDPC+STBC), low rungs, rendezvous — so a
width change needs no coordinated retune?

TX `--bw 20` on ch136, LDPC+STBC, RX `--bw 20` vs `--bw 40`:

| test | RX tuned 20 | RX tuned 40 |
|---|---|---|
| mcs0 singles, TX-bound | 6.02 Mb/s | 6.04 Mb/s |
| mcs7 agg6, TX-bound | 49.12 (HT20 table) | 49.30 Mb/s |
| 20 MHz frames on the SECONDARY (ch140) → RX 136/40 | — | nothing received |
| HT40 frames → RX tuned 20 | nothing received | — |

**Works, at full capacity, zero CRC fails.** Only the primary 20 is heard,
as 802.11n specifies; the reverse (20-tuned hearing 40) cannot work, so
whichever end receives HT40 video must sit at 40.

Sensitivity proxy (the bench is too strong to reach a delivery floor —
every rate delivered at TX index 0 in the wall sweeps): both GS cards on
the SAME frames simultaneously, one tuned 20 and one 40, then cards
swapped (singles, so every frame carries phy status). RX SNR readout,
tuned-40 minus tuned-20:

| | card 0 (chain A / B) | card 1 (chain A / B) | mean |
|---|---|---|---|
| mcs0, 2 Mb/s | −1.2 / −0.9 dB | −0.1 / −0.9 dB | **−0.8 dB** |
| mcs7, 10 Mb/s | −1.0 / −1.2 dB | −1.2 / −1.1 dB | **−1.1 dB** |

About 1 dB, not the 3 dB a full 40 MHz noise bandwidth would cost — the
chip evidently filters the primary 20 for a 20 MHz PPDU. mac_lost is not
worse tuned 40 (0–23 per run vs 1–125 tuned 20). Caveat: readout at
31–34 dB SNR is near its ceiling; the real number is a floor measurement
with attenuation (or a walk-out), which the bench cannot do.

## Correction: 136/140 is not a standard 40 MHz channel

The 5 GHz HT40 pairs are 36+40, 44+48, … 116+120, 124+128, **132+136,
140+144**, 149+153, 157+161. The first `--bw 40` always put the secondary
above the primary, so every "ch136" HT40 result above ran on 136+140
(centre 138), straddling the real 132+136 and 140+144 — fine for a
private link's physics, wrong for channel planning (other 40 MHz networks
and devourer's spur table are keyed on the standard centres). Fixed:
`common/include/mabur/ht40.h` `ht40_offset()` picks the standard side
(static_asserts pin the pairs), used by both linkbench tools and
`RadioFrontend`; `--bw 40` on a channel with no pair (165) is an error.
Re-checked on the standard 132+136 (primary 136, LDPC+STBC agg6): mcs0
10.19, mcs4 59.93, mcs7 101.69 vs 10.33 / 60.21 / 101.02 on 136+140 — the
capacity table holds. The wall and mixed-width results are physics of the
chip and the 20 MHz primary, not of the pairing, and are not re-run.

## Scouting a 40 MHz candidate

Two questions for an in-flight hop at 40 MHz (`docs/inflight-channel-hop.md`
§3): can one 40 MHz dwell score a pair, and what does a 40 MHz retune cost?

### Does a 40 MHz dwell see the secondary?

Interferer: an 8812EU on the dev PC (`linkbench-tx` host build, new
`--foreign-sa` so the GS books its frames as foreign), MCS0 20 MHz, light
(1 Mb/s app ≈ 30 % airtime) or heavy (5 Mb/s ≈ 95 %). GS card 0 tuned
40 MHz on 136+140 (this run predates the pairing fix; the question is
primary vs secondary, not which pair), card 1 tuned 20 MHz on 140 — the
per-half view. New `linkbench-rx --energy-ms 500`: `GetRxEnergy(with_nhm)`
FA/CCA + decoded own/foreign deltas, per 500 ms. Daemons stopped.

| interferer | card 0, 40 MHz 136+140 | card 1, 20 MHz on 140 |
|---|---|---|
| idle | FA ≈ CCA ≈ 100, foreign 0 | FA ≈ CCA ≈ 55, foreign 0 |
| secondary 140, light | CCA 45–90 (inside idle scatter), **foreign 0** | CCA 125–150, foreign ~100 |
| secondary 140, heavy | CCA 170–260 (~2.5x), **foreign 0** | CCA 230–290, foreign ~300 |
| primary 136, light | CCA 110–145, foreign ~100 | FA/CCA 450–590, foreign 0 |
| primary 136, heavy | CCA ~285, foreign ~300 | FA/CCA ~1000, foreign 0 |

1. **A 40 MHz dwell is nearly blind to the secondary.** Secondary frames are
   never decoded (foreign 0 — the ranker's heaviest term, `4·foreign`), and
   CCA only moves once the secondary is close to saturated. Scored with
   `fa + max(cca − own, 0) + 4·foreign`, a heavily busy secondary reads
   ~1.6x idle on the 40 MHz dwell against ~13x on the 20 MHz one. **Score
   each 20 MHz half with its own 20 MHz dwell** (the existing scout, twice).
2. **Adjacent-channel leakage is large on the 20 MHz dwell.** Traffic on
   136 put 5–10x idle FA/CCA on a card tuned 20 MHz to 140 with zero decoded
   frames. The interferer was a strong, close source, so this is a worst
   case — but a per-half ranker should lean on `foreign` and on CCA of the
   half itself, and expect a busy neighbour to smear into its FA.
3. Harness notes: the host bring-up takes ~15 s (register tables ≈ 10 s,
   port is high-speed 480), so interferer phases must sit inside one long
   GS capture; host `linkbench-tx` must run under `setsid` (a signal from
   the tool's process group interrupts libusb → stop flag, error −10).

### Retune cost at 40 MHz

`linkbench-rx --retune-bench a,b,..` times `RadioFrontend::retune()`
(`FastRetune`, which keeps width and offset). 10 cycles each, ms
median / max, standard pairs (all HT40−, primary = upper 20):

| hop (40 MHz) | centre(s) | ms |
|---|---|---|
| 136 ↔ 144 (132+136 ↔ 140+144) | 134 ↔ 142 | 1.9–12.0 / 44.7 |
| 136 ↔ 120 (↔ 116+120) | ↔ 118 (spur) | 65–76 / 87 |
| 136 ↔ 153 (↔ 149+153) | ↔ 151 (spur) | 73–91 / 97 |
| 136 ↔ 161 (↔ 157+161) | ↔ 159 (spur) | 79–81 / 88 |

20 MHz reference (same card): 1.8–1.9 ms between 120/136/140, 6.5–6.8 ms
to/from 149/157, max 28.

4. **Three of the four HT40 escape pairs are 8822E spur combos** (centres
   118, 151, 159 are in `is_spur_combo_8822e`'s 40 MHz list), and devourer
   declines the fast path into or out of every one of them → full channel
   set, **65–91 ms per retune**. A scout dwell is two retunes, so ~150–180 ms
   off-channel per visit instead of ~10; a hop pays it once on each end.
   Only 132+136 ↔ 140+144 stays fast. Either the candidate list changes
   (36+40/44+48 are spur-free but UNII-1; 124+128 is spur-free DFS), or
   devourer's lean path learns to carry the spur state.
5. The 149+153 capacity deficit above sits on one of these spur centres —
   consistent with the spur handling (NBI notch / CSI mask) being the cause,
   still unconfirmed.

## Not measured

- Range/sensitivity: bench only (RSSI ~−68 dBm singles). The per-rate
  margin comparison (HT40 mcs n vs HT20 mcs ~n+2 at equal throughput) needs
  attenuation or a walk-out.
- Walls on a second channel (anchor moves per channel; rel walls are
  expected to hold, per `docs/calibration.md`, not re-checked at HT40).
- The 149/153 deficit's cause.
- The mixed-width sensitivity penalty at the actual floor (attenuated or
  range test); secondary-channel interference desensing a 40-tuned RX.
