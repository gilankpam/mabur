# EVM vs TX power sweep — why raw EVM is op-point-dependent (2026-08-10)

## Why this was run

The first day of EVM observability (PR #20) showed a 14 dB EVM swing during
a single one-minute ladder climb on an unchanging bench channel:

| Rung | MCS / overhead | SNR (dB) | reported EVM (dB) |
|------|----------------|----------|--------------------|
| 0    | mcs0 / 100%    | ~32.6    | −16.0              |
| 1    | mcs2 / 50%     | ~33.7    | −16.4              |
| 2    | mcs4 / 25%     | ~33.2    | −23.6              |
| 3    | mcs5 / 25%     | ~30.3    | −30.9              |
| 4–5  | mcs6/7         | ~29      | ~−30               |

SNR barely moved; EVM stepped exactly at rung transitions — and in the
"wrong" direction (higher MCS reading *cleaner*). Two candidate causes:
(a) per-MCS TX power (the offset-power walls) driving PA distortion, or
(b) the chip's EVM estimator normalizing differently per modulation. The
discriminating experiment: hold MCS fixed, sweep TX power, watch EVM.

## Method

txagcbench on the bench rig (drone 8812EU TX → GS 8822EU RX, both daemons
stopped; harness runs carrier sense ON, plain 1SS, no STBC — NOT the
daemons' MAC/rate config). `txagcbench-rx` was extended to record
`evm_a`/`evm_b` per frame (raw half-dB). Three sweeps, each at fixed MCS,
offset-power mode sweeping −10 → 0 dB relative to that MCS's measured
wall in qdB steps, ~50 frames/step:

```
TX_ARGS="--mcs N --pwr-mode offset --lo 24 --hi 64" bench/txagcbench/run_sweep.sh
```

~4000 bench frames per sweep (mcs7: 2242 — loss at the deep-backoff end),
0 CRC-bad throughout. Raw jsonls were not kept; the aggregates below are
the finding, and the command above reproduces them.

## Results (chain-A EVM, dB; SNR is chain B, dB)

| offset vs wall | mcs0 EVM | mcs0 SNR | mcs4 EVM | mcs4 SNR | mcs7 EVM | mcs7 SNR |
|---:|---:|---:|---:|---:|---:|---:|
| −10.00 | −21.7 | 18.2 | −19.6 | 17.7 | −21.4 | 17.5 |
| −8.00  | −23.7 | 20.7 | −19.4 | 17.6 |   —   |  —   |
| −6.00  | **−25.9** | 23.3 | −20.9 | 20.3 | −21.0 | 17.4 |
| −4.00  | −24.8 | 24.8 | −22.5 | 21.4 | −21.4 | 18.2 |
| −2.00  | −22.3 | 26.6 | **−22.8** | 23.9 | −22.4 | 20.6 |
| −0.25  | −18.0 | 27.8 | −21.1 | 25.0 | −23.5 | 21.7 |
|  0.00  | −18.0 | 27.7 | −20.6 | 25.3 | **−24.1** | 22.9 |

## Findings

1. **mcs0 is distortion-limited at its wall.** From −6 dB offset up to the
   wall, SNR keeps rising (23 → 28) while EVM *degrades 8 dB* (−25.9 →
   −18.0). Only transmit-side dirt does that: the PA compressing as it
   approaches the high-power mcs0 wall. This confirms cause (a) — and note
   the estimator-artifact hypothesis (b) is refuted as the *primary* cause:
   at fixed modulation the EVM moved 8 dB with power alone.
2. **mcs7 is noise-limited.** EVM improves monotonically with power (−21.4
   → −24.1), tracking SNR. Its wall sits at much lower absolute power, so
   the PA never leaves the linear region. mcs4 turns over in between
   (optimum ≈ −2 dB offset).
3. This fully explains the ladder-climb table above: low rungs run near
   high-power walls (compression → EVM ≈ −16), high rungs run backed off
   (linear → EVM ≈ −30).

## Interpretation — raw power vs clean signal

TX distortion behaves as noise that scales with the signal: it puts a
**ceiling** on effective SNR (≈ −EVM) that no amount of power can lift.
Thermal noise is a **floor** you climb away from with raw power, and it is
what dominates at range. Adding power past compression trades ceiling for
floor. Whether that trade is good depends on the modulation's effective-SNR
requirement:

- mcs0 (BPSK ½, needs ~5 dB): a −18 dB ceiling is irrelevant — the dirt is
  free, raw power wins, run it hot for range.
- mcs7 (64-QAM ⅚, needs 20+ dB): an mcs0-style ceiling would make it
  undecodable at any distance — it must stay linear.

**Consequently the delivery-defined walls (txagcbench first-dip comb) are
already the correct per-MCS optimum for FPV** — delivery *is* effective
SNR vs the MCS requirement, so the comb implicitly found the max power
whose distortion ceiling still clears each MCS. Do not re-derive walls
from EVM or SNR alone.

## Implications for using EVM

- **Never threshold raw EVM globally.** −24 dB is "healthy, parked at
  mcs4" and simultaneously "6 dB degraded" at mcs7. Any controller input
  must be *deviation from that rung's own baseline*; the ctl log's
  per-rung DWELL EVM ranges accumulate those baselines on every flight.
- **Live PA-compression watchdog.** The walls were bench-measured at bench
  temperature; PA compression moves with heat and age. The signature
  "EVM worsening while RSSI/SNR holds or rises" is an in-flight
  compression detector that previously required a bench sweep.
- **Margin audit at top rungs.** mcs7's at-wall EVM of −24.1 dB with clean
  delivery is a measured ceiling margin; it can now be watched for erosion.

## Gotcha found by this sweep: the −128 sentinel

Every 1SS non-STBC frame carries `evm[1] = −128` (int8 min) — the chip's
"absent spatial stream" sentinel, not a measurement (`RxAtrib.evm[]` is
per-SPATIAL-STREAM, not per-antenna-chain). The daemons' STBC traffic
fills both slots, so the live link never showed it; the plain-1SS bench
harness did. maburgs' aggregator treats −128 like 0 ("not sampled") since
commit 90b89c5 — without that guard the best-of `min()` would peg exported
EVM at an impossible −64 dB.

## Caveats

One bench link, chain-A EVM, bench temperature, near-field. The July
adaptive-link acceptance noted "constant FEC repair, mild 9a" while parked
at the PA-floor offset — consistent with living near the distortion knee.
Next dataset that would firm this up: a walk-out or hot-day recording with
the EVM columns live (per-rung baselines vs temperature/range).
