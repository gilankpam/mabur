# mcs6 bench anomaly: dirty at bench range, unresponsive to TX power (2026-07-27)

**Status:** OPEN — mitigated in bench config by an explicit ladder that skips
the mcs6 rung. Drone `rate_walls_idx` restored to measured values. Root-cause
work needs the attenuation rig (the standing RX-overdrive study).

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

## Next steps

1. Attenuation-rig study: per-rate delivery vs received power for mcs5/6/7,
   enough attenuation to rule RX saturation in or out (same protocol as
   `docs/txagc-calibration.md`).
2. Re-measure the mcs6 wall while at it — the 2026-07-16 walls predate the
   condition drift.
3. If mcs6 stays anomalous under controlled RSSI, dissect at the packet level
   (`tools/bench/fu_probe.py` / `seqdump.py`) to see *which* bodies die.
