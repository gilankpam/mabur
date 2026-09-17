# Per-MCS air capacity sweep — 2026-09-17

Question: is the real per-rung air capacity close enough to the nominal
PHY rate that a "bandwidth calibration" (per-MCS efficiency next to the
TX-power walls) is not worth building? Context: `encoder.airtime_budget`
0.5 flies clean, 0.6 spikes latency at the low rungs even with the air
clock's enh shed armed (`docs/link-adaptation.md` "Drone air clock").

Method: `linkbench-tx` (drone, `--ampdu` flag added this day so the blast
rides maburd's A-MPDU setup: max_num 6, density 7, no-ack, max_time 0x20)
against `linkbench-rx` (GS card 0), ch136, prod FEC geometry (symbol 332,
bpb 4, window 32, overhead 0.5 → ~1405 B bodies), `--pwr-mode none`,
tx-threads 4, offered load above capacity at every point (the sender
reports `under target` = TX-bound), 15 s per point, median of the steady
seconds. Both daemons stopped for the sweep; restored after, ausniff
60.5 fps / 0 gaps. Raw logs in the session scratchpad only. Every point
reproduced to three digits on repeat (mcs2 agg6 x3, agg0 x2).

## Delivered air (payload bits on air / nominal HT20 rate), zero loss everywhere

| mcs | nominal Mbit/s | agg6 (prod) | eff | singles (agg off) | eff |
|---|---|---|---|---|---|
| 0 | 6.5  | 5.66  | **0.87** | 6.04  | 0.93 |
| 1 | 13   | 10.17 | **0.78** | 11.42 | 0.88 |
| 2 | 19.5 | 13.85 | **0.71** | 16.32 | 0.84 |
| 3 | 26   | 19.78 | **0.76** | 20.86 | 0.80 |
| 4 | 39   | 29.59 | **0.76** | — | — |
| 5 | 52   | 39.47 | **0.76** | 35.41 | 0.68 |
| 6 | 58.5 | 45.34 | **0.78** | — | — |
| 7 | 65   | 49.42 | **0.76** | 41.03 | 0.63 |

`--tx-threads 8` at mcs7 agg6: 49.35 (same) — the ceiling is the air, not
USB or the drone CPU. `--ampdu-max-time` 1 or 8 changes nothing at mcs0/2
(the fill timer is not what costs the low rungs); max_time 8 at mcs5
collapses to 0.47 (aggregates cut short).

## Findings

1. **Capacity is not near nominal.** 0.71–0.78 at mcs1–7 with the prod
   aggregation, 0.87 at mcs0. This is the flight-fitted air-clock
   efficiency exactly (flight 0030: ~0.73 at rungs 1/3/4, ~0.70 at rung 2,
   ~0.82 at rung 0 — `docs/air-clock-flight-findings-2026-09-06.md` §2), so
   the bench saturation number and the flight queue fit agree, and the
   shipped `air_clock.efficiency` 0.73 is within ±0.03 at rungs 1–5.
   Rung 0 is over-priced by ~19 %.
2. **Why 0.6 spikes and 0.5 does not.** `run_bitrate_policy` commands
   `rate/3` at budget 0.6, which with framing/probe/MSP puts ~0.70 of
   nominal on air (flight table) — at mcs2's 0.71 capacity that is ~99 %
   utilisation, ~90 % at mcs3–5. Budget 0.5 lands at ~0.58 of nominal =
   82 % at mcs2, 76 % at mcs3–5, 67 % at mcs0. The budget knob is a
   fraction of a number the link never delivers, and the low rungs sit
   closest to the edge.
3. **A-MPDU costs the low rungs.** agg6 loses 6/10/13/4 points of
   efficiency at mcs0/1/2/3 versus singles and gains 8/13 points at
   mcs5/7. The crossover is around mcs4. agg3 = agg6 at mcs2 (0.711 both)
   and the RX SNR readout only drops (a phy-status-per-aggregate artefact)
   at mcs3+, i.e. aggregates barely form at mcs0–2 yet the mode still
   costs ~100 µs per PPDU there — consistent with a fixed per-A-MPDU
   post-PPDU wait (BA/ack timeout shape) that singles do not pay and that
   6-MPDU aggregates amortise. Mechanism not confirmed; the number is.

## Channel check (same day): capacity is channel-independent

The agg6 sweep and the mcs2/mcs5 singles were repeated on ch149 and
ch161 (daemons stopped, bench restored after each: ausniff 60.5 fps /
0 gaps). Every point lands within 0.5 % of ch136:

| mcs | ch136 | ch149 | ch161 |
|---|---|---|---|
| 0 | 0.871 | 0.868 | 0.868 |
| 1 | 0.782 | 0.782 | 0.782 |
| 2 | 0.710 | 0.707 | 0.712 |
| 3 | 0.761 | 0.757 | 0.762 |
| 4 | 0.759 | 0.757 | 0.759 |
| 5 | 0.759 | 0.757 | 0.755 |
| 6 | 0.775 | 0.774 | 0.770 |
| 7 | 0.760 | 0.757 | 0.756 |
| 2 singles | 0.837 | 0.835 | 0.836 |
| 5 singles | 0.681 | 0.677 | 0.679 |

Loss stayed at or under 0.6 % on every point (ch149 ran slightly lossier,
0.2–0.6 %, at the same delivered rate). Unlike the TX-power walls, which
are per-channel (`docs/calibration.md`), the capacity table is a property
of MCS plus the MAC/USB path and one table covers every channel — a
static 8-entry table in the config would never need re-measuring per
channel.

## What this means for "bandwidth calibration"

- A per-MCS table is real but small at rungs 1–5 (±0.03 around 0.73)
  and only rung 0 deviates (0.87). One scalar plus a rung-0 exception, or
  a static 8-entry table copied from this page, covers it — a maburcal-
  style sweep tool is not justified by the spread. The larger error is
  that `run_bitrate_policy` prices off nominal at all: making the budget a
  fraction of `nominal × eff[mcs]` (same number the air clock uses) is the
  consumer change that turns `airtime_budget` into a real utilisation.
- The bigger lever the sweep exposed is **per-rung A-MPDU**: singles at
  mcs0–3 are +7 % to +18 % capacity at exactly the rungs that spike;
  `SetAmpduMode` is a live register write, so the RcAgent could switch it
  with the rung. Latency side of A-MPDU (fec −2.3/−2.7 ms, `ampdu-probe`
  memory) was measured at rung 5 only; the low-rung latency cost of
  singles is unmeasured — A/B before shipping.

Related: `docs/airtime-model.md`, `docs/tx-rx-timing.md` §"open-loop
airtime budget", `docs/link-adaptation.md` "Drone air clock".

## Shipped: per-rung A-MPDU (same day, branch `ampdu-per-rung`)

The rung-pinned A/B (GS `static_mcs` 0..3 with the prod 1.0/0.5 pair,
drone `ampdu.max_num` 6 vs 0, 90 s per arm, sessions 0121-0128, ausniff
60.8 fps / 0 gaps every arm) found singles never worse at rungs 0-3:

| rung | arm | e2e p50/p99 | fec p50/p99 | air p50/p99 | AU span p50/p99 ms |
|---|---|---|---|---|---|
| 0 | agg6 | 42 / 57 | 6 / 17 | 1 / 15 | 6.5 / 18.6 |
| 0 | singles | 40 / 59 | 6 / 16 | 1 / 14 | 6.0 / 17.3 |
| 1 | agg6 | 43 / 56 | 8 / 16 | 1 / 12 | 8.4 / 16.9 |
| 1 | singles | 43 / 57 | 6 / 15 | 1 / 10.5 | 6.8 / 15.6 |
| 2 | agg6 | 44 / 57 | 8 / 18 | 1 / 17 | 8.9 / 18.3 |
| 2 | singles | 42.5 / 54 | 7 / 16 | 1 / 13 | 7.6 / 17.5 |
| 3 | agg6 | 42 / 57 | 8 / 24 | 1 / 12 | 8.2 / 25.0 |
| 3 | singles | 44 / 56 | 8 / 23 | 1 / 12 | 8.1 / 23.8 |

(lat.log per-second window medians; AU span = `t_complete − t_first` from
`au.log`.) Rungs 1-2 gain 1-2 ms of fec p50 and 2-4 ms of air p99 on
singles; rungs 0 and 3 are a wash. So the mode now follows the rung:
`ampdu.min_mcs` (new drone config key, bundle 4; 0 = the old aggregate-
everywhere behaviour) — `drone/src/ampdu_policy.h` derives the
devourer `AmpduMode` from the op MCS, `RealActuator::apply_op` programs
the chip on the agent thread only across a change (one 0x455 register
write plus the per-frame descriptor state, live on this family), and the
bring-up `SetAmpduMode` call is gone: the chip leaves InitWrite in the
singles state, which is what the boot MAX_RANGE op wants. `/tmp/mabur.log`
prints `maburd radio: A-MPDU OFF at mcs0 (chip default …)` at the first
op and `A-MPDU ON at mcs4 …` at the 3→4 promote. Deployed to the bench
drone (rollback `maburd.pre-ampdurung` + `mabur.toml.pre-ampdurung`;
binary BEFORE config, the key is unknown to the old binary); three
restarts each climbed 0→5 with the switch at mcs4, ausniff 60.5 fps /
0 gaps / 0 incomplete after each. Host suite 148/148. UNFLOWN.

## Shipped: per-MCS efficiency table (2026-09-18, same branch)

`air_clock.efficiency` is an 8-entry per-MCS array (bundle = the table
above with singles at mcs0-3, agg6 above); `drone/src/air_rate.h`
`delivered_mbps()` = nominal × entry, consumed by BOTH
`RcAgent::run_bitrate_policy` (every rate term) and the air clock
(base/enh/probe each at their own MCS). Struct default all ones =
nominal = the old policy, so an absent section changes nothing.
`encoder.airtime_budget` 0.5 → 0.65 in the bundle: identical command at
rungs 4-5 (0.5/0.76), +23/16/11/5 % at rungs 0/1/2/3. Config-shape
change (scalar → array): binary BEFORE config, rollback paired
(`maburd.pre-efftable` + `mabur.toml.pre-efftable`).
