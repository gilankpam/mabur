# Air clock — first flights (2026-09-06)

First air data for the drone air clock (branch `air-clock`, PR #45,
`docs/link-adaptation.md` "Drone air clock (2026-09-06)", spec
`2026-09-06-air-clock-enh-shed-design.md`). Three flights on the same
build, one knob changed between each:

| flight | GS records | `encoder.airtime_budget` | `air_clock` | window | note |
|---|---|---|---|---|---|
| 0030 | `ctl-0351`, `log/{flight,au}-0030`, `lat-0056` | 0.6 | shed 0, eff 0.70 (observe) | 55–448 s | 9 cascades, no link loss |
| 0031 | `ctl-0352`, `log/{flight,au}-0031`, `lat-0057` | **0.5** | shed 0, eff 0.70 (observe) | 26–592 s | 9 cascades; starved at 98/180/576 s, drone pts reset at 231 s |
| 0032 | `ctl-0353`, `log/{flight,au}-0032`, `lat-0058`, `record-0044` | 0.5 | **shed 25, eff 0.73 (armed)** | 42–344 s | range-limited, mostly mcs1–3, 7 cascades, 11 `s3_util` |

All on `/run/media/gilankpam/DVR`. Instruments: `tools/bench/airdrain.py
--model` (per-frame model vs GS-measured air+q), the flight jsonl for the
drone's `air_backlog_max_ms` / `air_shed_drops` / `enc.cmd_kbps` and the
cards' `rx_mbps`, the lat log for the segment decomposition of the
end-to-end tail, plus ad-hoc scripts over the AU log (the reconstruction
in §3 — session scratchpad only, not committed; the formula is here so it
can be redone).

## TL;DR

1. **At budget 0.6 the model over-predicted the backlog ~2.5× and was
   uncalibratable, because it ran at 90–95 % of its own capacity.** The
   bitrate policy commands `nominal/3` (0.6 budget over a 1.8× FEC
   bracket); framing, repair quantization, probe and MSP push the bytes
   on air to ~70 % of nominal, and nominal × efficiency 0.7 is ~70 % of
   nominal. Zero headroom by coincidence — the two knobs were tuned in
   different months. A queue at 95 % load integrates every burst, so a
   3 % pricing error is a 2–3× backlog error. The bench "0.35–0.4 at
   mcs5" datapoint came from a sub-8 ms jitter regime and is void.
2. **`airtime_budget` 0.5 removed the drain at the source, with the gate
   still off.** Cascade peak p50 47 → 20 ms, max 197 → 68, settle 1.9 s →
   0.3 s, steady-state p99 69–112 ms → 8–13 ms at every rung, spike
   seconds 62/394 → 0/484. A one-step demote at 0.5 costs ~1.01× of the
   new rung's real air (vs 1.22× at 0.6), so there is nothing to drain.
   Price: 17 % bitrate everywhere (mcs5 15.8 → 14.6 Mbit/s).
3. **The model is calibrated at 0.5.** Model utilisation 0.74–0.81,
   quiet-frame p99 166 → 6 ms, cascade peaks track (47/45, 79/68, 34/34).
   Efficiency crosses 1.0 at ~0.72–0.73 in all three flights; at 0.5 it
   is no longer knife-edge (0.70 vs 0.75 = 2×, not 10×). Rung 0 is the
   one rung still over-priced (small n).
4. **Armed at shed 25 / eff 0.73 the gate behaved:** 14 in-flight drops
   in 10 windows, every one at a transition (demote drains, and the IDR
   of two promotes — a ~60 kB IDR alone is ~25 ms at mcs3), zero
   phantoms, air tail clipped at 35 ms (unarmed: 68), no demote it
   clearly caused. It never fired on the 129 s 4→0 cascade because the
   backlog peaked at 23 ms and the GS agreed.
5. **The remaining > 100 ms tails are FEC repair waits, not air.** In
   both 0.5 flights every > 100 ms second at a demote had `fec` 57–102 ms
   at the tail with `air` 10–25 — the loss that caused the demote, often
   a promote into a fading link (probe-approved 3→4, loss within 800 ms,
   1.2 s and four steps down). Out of this feature's reach; do not lower
   `shed_ms` chasing them.
6. **Verdict: adopt.** Merge, ship budget 0.5 + shed 25 + eff 0.73. The
   observability half diagnosed the budget; the gate is a modest clean
   win. Do not arm it if the budget ever goes back toward 0.6 — there it
   becomes a steady-state enh throttle (replay: 14 % of enh dropped,
   flickering 60↔30 fps).

## 1. Flight 0030 — budget 0.6, observe-only: the model at zero headroom

`airdrain --model`: slope measured/model 0.39 overall, residual p50
−14 ms, quiet-frame model p99 166 ms. Per rung:

| rung | slope | model p50/p90/p99 | GS p50/p90/p99 | frames model ≥ 25 & GS < 5 |
|---|---|---|---|---|
| 0 | 0.05 | 4/176/222 | 2/17/39 | 98 |
| 1 | 0.29 | 99/249/306 | 5/132/191 | 210 |
| 2 | 0.89 | 22/121/178 | 19/127/164 | 79 |
| 3 | 0.38 | 16/101/142 | 4/42/87 | 468 |
| 4 | 0.30 | 13/75/137 | 4/25/65 | 1010 |
| 5 | 0.31 | 1/17/78 | 2/10/30 | 108 |

Whole seconds sat at 100+ ms of modelled backlog while the GS saw < 5
(e.g. 174–177 s at rung 4: model 106/95/75/72, GS 4/5/4/4). The GS
floor is a running min with a 60 ppm leak, so a GS reading of 4 ms means
the frame arrived within 4 ms of the best-ever path: the model was
wrong, not the measurement.

**Why.** From the flight jsonl, per rung:

| mcs | `cmd_kbps` | on-air `rx_mbps` / nominal | / (nominal × 0.7) | model util (§3) |
|---|---|---|---|---|
| 0 | 2200 | 73 % | 105 % | 0.84 |
| 1 | 4300 | 72 % | 104 % | 0.93 |
| 2 | 6500 | 70 % | 100 % | 0.95 |
| 3 | 8700 | 69 % | 99 % | 0.94 |
| 4 | 13000 | 68 % | 97 % | 0.95 |
| 5 | 16000 (cap) | 57 % | 82 % | 0.90 |

`run_bitrate_policy` (drone/src/rc_agent.cpp) solves `kbps =
airtime_budget / [0.6·(1+ov_b)/rate + 0.4·(1+ov_e)/rate]`; with the
flat 1.0/0.5 pair that is `0.6·rate/1.8 = rate/3`. The uncounted
excess — SBI/fragment framing and repair quantization (~1.09× base,
~1.14× enh, §3), probe 0.34 Mbit/s, MSP 0.1 — takes the bytes to ~0.70
of nominal. Time occupancy is then 0.70 / 0.73 ≈ 0.95 of the real air
(§2), which is why the GS's own steady-state p90 was 25–42 ms at rungs
3–4: the real link was nearly full too.

Gate replay on this flight (no feedback, so an upper bound on drops):
eff 0.70 / shed 25 would have dropped 36 % of enh AUs, 730 of them
phantom; eff 0.73–0.74 / shed 25 drops 10–14 %, < 50 phantom, 8/9
cascades caught. Not armable at 0.7; marginal at the fitted value.

## 2. Efficiency fit — and why it was a knife edge

The AU log's `air_ms` column lets the drone's clock be reconstructed
exactly (§3), then re-run at other efficiencies against the GS's
measured air+q:

| eff | r0 | r1 | r2 | r3 | r4 | r5 | all |
|---|---|---|---|---|---|---|---|
| 0.70 | 0.05 | 0.29 | 0.88 | 0.38 | 0.30 | 0.31 | 0.39 |
| 0.75 | 0.24 | 1.22 | 2.60 | 1.34 | 1.35 | 1.72 | 1.43 |
| 0.80 | 0.67 | 1.79 | 4.56 | 2.38 | 2.88 | 3.81 | 2.23 |

(slope measured/sim, 1.0 = fit; flight 0030). The crossing is ~0.73–0.74
at rungs 1/3/4, ~0.70 at rung 2, ~0.82 at rung 0 — but a 5 % change in
efficiency moves the backlog 2–3× at this load. That sensitivity is
structural for any queue model near ρ = 1, so per-rung calibration alone
would not have made the gate safe at budget 0.6.

Physically the hardware ceiling fits the picture: agg6 A-MPDUs amortise
preamble/SIFS/backoff, the half-duplex RCF/telemetry slots and USB
pacing take the rest; ~0.73 of nominal is what the flights say the
link moves at rungs 1–4.

## 3. Reconstructing the drone's clock from the AU log

`air_ms` is the backlog at each AU's arrival, 1 ms quantised. With
`b[i]` the backlog on AU i, `dt` the pts step to AU i+1 and `c[i]` the
cost booked for AU i:

    b[i+1] = max(0, b[i] + c[i] − dt)   ⇒   c[i] = b[i+1] − b[i] + dt  when b[i+1] > 0

Frames whose successor reads 0 get the per-(rung, sid) median of
`c/(len·(1+ov)·8/rate)` from the known ones (that ratio is the framing
excess: ~1.09 base / ~1.14 enh at eff 0.70; 1.05 / 1.09–1.14 at 0.73,
which is how flight 0032's efficiency was confirmed). Re-simulating the
leaky clock from the reconstructed costs reproduces `air_ms` with slope
0.999 (0030), 0.973 (0031), 0.871 (0032, gate feedback). Scaling `c` by
`0.7/eff` gives the model at any efficiency; `Σc / Σdt` per rung is the
model utilisation quoted above.

⚠ Sort by GS arrival (`tf`), not pts, and reset the clock on a > 2 s pts
step — flight 0031 restarted maburd mid-flight and a pts-sorted replay
carried a 60 s phantom backlog across the restart. Worth folding into
`airdrain.py` as a `--fit` mode rather than keeping as a scratch script.

## 4. Flight 0031 — budget 0.5, observe-only: the drain is gone

Confirmed in force: `cmd_kbps` 1800/3600/5400/7200/10800/14400. On-air
53–59 % of nominal = 76–85 % of model capacity; model utilisation
0.74–0.81.

| | 0030 (0.6) | 0031 (0.5) |
|---|---|---|
| cascade peak p50 / max | 47 / 197 ms | 20 / 68 ms |
| time-to-peak p50 | 1510 ms | 179 ms |
| settle p50 | 1943 ms | 314 ms |
| single-demote peak p50 | 48 ms | 13 ms |
| steady-state p99, r2 / r4 | 112 / 69 ms | 11 / 10 ms |
| spike seconds | 62 of 394 | 0 of 484 |
| first-500 ms on-air vs new rung nominal, max | 119 % | 100 % |
| model slope / quiet-frame p99 | 0.39 / 166 ms | 0.65 / 6 ms |

Arithmetic behind it: a one-step demote (mcs4→3) serves the old
bitrate on the new rate at `2.03 × budget` of the real air (eff 0.74) —
1.22 at 0.6 (backlog grows ~220 ms/s until the encoder catches up),
1.01 at 0.5. Two-step cascades: 1.62 → 1.35.

Gate replay at 0.5: eff 0.73 / shed 25 = 47 drops, 0 phantom, 3/9
cascades exceed 25 ms at all. Rung 0 still 127 phantom frames of 766 at
eff 0.70 — the low-rung pricing gap of §2.

Not a controlled A/B: 0031 had three starved link losses (98, 180,
576 s) and a drone restart at 231 s, and a different route. The
steady-state and per-rung numbers are consistent enough to stand.

## 5. Flight 0032 — armed (shed 25, eff 0.73): what the gate did

`air_shed_drops` read 26 at the end but 12 at the first telemetry row
(the counter runs from link-up; those were on the ground). 14 in-flight
drops, from the counter steps and the AU log:

| ctl | DVR (`record-0044`, ≈ ctl − 54 s) | drops | context |
|---|---|---|---|
| 82 s | 0:28 | 2 | 4→3 `s3_util` |
| 93 s | 0:39 | 1 | 4→3 fade, 3→2 `s3_util` |
| 163 s | 1:49 | 1 | promote 3→4, IDR |
| 259–260 s | 3:25 | 3 | 4→3→2→1→0 cascade |
| 272 s | 3:38 | 1 | promote 1→2, IDR |
| 276 s | 3:42 | 2 | 2→1 `s3_util` |
| 286 s | 3:52 | 1 | 2→1 `s3_util` |
| 298 s | 4:04 | 3 | 4→3 residual, 3→2 `s3_util` |

Never more than 3 in a row (≈ 50 ms of 30 fps). The 129 s 4→0 cascade
(DVR 1:15) got none: backlog max 23 ms, GS 20 ms, the 7 missing enh
that second were RF (base lost 4 in the same second). Missing-enh events
in the AU log total 90 ≈ ~1 RF-lost enh per transition (~60) + the
drops — `ausniff`'s missing-enh count will not equal `air_shed_drops`
on a lossy flight even with CONG off.

Model: slope 1.36 (slightly under, the safe side), quiet p99 4 ms, zero
phantoms at every rung. Latency vs 0031: per-frame air max 68 → 35 ms,
lat-log air-segment tail max 68 → 35, seconds with air tail ≥ 40: 3 → 0,
cascade peak p50/max 20/68 → 21/35. Seconds with e2e tail ≥ 100: 4 of
304 — one regulator hiccup (52 s, `reg` 213) and three demotes with
`fec` 57–102 at the tail and `air` 10–25.

Ladder: no `s3_util`/`util`/probation demote followed a drop within 2 s
except 3→2 at 298.02 s, 0.3 s after two drops — but a residual demote
at 297.6 s was already in progress, so it reads as a normal cascade.
One ambiguous case; enh silence is `NoInfo` by design, keep an eye on
it.

### The 3:25 cascade, frame by frame

| phase | event | model | GS air | FEC wait |
|---|---|---|---|---|
| 257.2 s | promote 3→4 (probed) | 0 | 0 | ~10 ms |
| 258.0–258.25 | RF loss at rung 4, u 0.08→0.35 | 0–3 | 0–5 | 41–84 ms |
| 258.2 | probation 4→3, IDR | 0–9 | ≤ 13 | 5–13 |
| 258.8–259.1 | loss burst, u3 → 0.73, 3 enh lost RF | 0–1 | 1–3 | 50–101 ms |
| 259.1–259.5 | 3→2, 2→1 fade, 1→0, three IDRs in 300 ms | 10–17 | 11–21 | 6–36 |
| 259.5 → | mcs0, 4 kB frames | 0–3 | 2–3 | 5–15 |

The 105 ms tail is the first two loss bursts (`fec` 66, `air` 13); the
three sheds are in the last phase, where the stacked IDRs plus one base
frame's cost reach 25 ms. Lowering `shed_ms` would have taken 10–15 ms
off a few frames in that phase and nothing off the 66–101 ms repair
waits.

## 6. What `airtime_budget` 0.6 would do with the gate armed

Back at ~0.95 model utilisation, shed 25 / eff 0.73 replays to ~14 % of
enh dropped over flight 0030, a fifth of it far from any transition —
and with the real feedback it settles into a duty cycle (drop enh →
load falls to ~0.6 → drains → gate opens → back to 0.95) that holds the
modelled backlog at the threshold by flickering 60↔30 fps. Raising the
threshold to ~60 to stop that gives back the 0030 latency. If bitrate
is wanted back, 0.55 with shed 40 is the experiment, flown and read the
same way; not 0.6.

## 7. Recommendations

1. Merge PR #45; commit budget 0.5 + `air_clock {shed_ms 25, efficiency
   0.73, body_us 0}` as the production drone config.
2. ~~`airdrain.py`: the default window stops at the FIRST starved E line~~
   FIXED same day (window runs to the last S line; `flightreport.py`'s
   per-mcs probe loss now re-anchors on a restart/starve instead of
   booking a seed jump as loss, and its au-log auto-match picks the log
   whose enh fids join rather than the biggest mono overlap). Still
   open: fold the §3 reconstruction in as a `--fit` mode.
3. GS `S95flightrec stop` still uses `pkill` (absent on BusyBox) — kill
   the old recorder by hand, or fix the wrapper.
4. Stop investing in the gate. The next latency win is on the ladder
   side: promote-into-fade (probe approves, loss within a second, four
   steps down) and repair latency under burst loss. §5's fec tails are
   that problem.
5. Rung-0 pricing is still off (phantoms at eff 0.70 in both 0.5
   flights); a per-rung efficiency would fix it but the sample is small
   and the gate at 25 ms never fired there.
