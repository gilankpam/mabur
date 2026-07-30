# Ladder util-path stress calibration — campaign result: BLOCKED

Date 2026-07-30. Branch `feat/ladder-stress-calib`. Bench GS
`root@10.18.0.1`, drone `root@192.168.10.152`.

## Method

Spec: `.superpowers/sdd/2026-07-30-ladder-stress-calibration/task-5-brief.md`
(design spec referenced therein). The GS carries a bench-only
`link.stress_offset` config knob (Tasks 1-3) that subtracts a configurable
quarter-dB offset from the TX power the drone applies, either as a static
level or a ramp (`step_qdb` per `period_s`, clamped at `floor_qdb`),
logged loudly (`*** STRESS OFFSET ACTIVE ... — bench instrument, NOT for
flight ***`) and deployed+smoke-tested on hardware in Task 4. This
campaign's goal: record the ladder controller's behavior under induced
power stress and use `tools/flightreport.py --calib` to derive new
`down_util`/`up_util`/`confirm_ms` values so the util-path (a proactive,
leading-indicator demote) catches degradation *before* residual loss is
observed, per a binding selection rule (below). Recordings were taken via
a passive AF_PACKET tap on `lo:8300` (`.superpowers/sdd/2026-07-30-ladder-stress-calibration/statstap.py`)
rather than binding the sideport UDP port, since the bench GS's `maburtop`
held it exclusively throughout.

Run matrix executed: 600 s clean baseline (knob off); static staircase at
`qdb` = −8, −16, −24, −32, −40 (300 s each, `−40` = the knob's floor,
added after `−32` showed no effect per the brief's escalation rule); two
720 s slow ramps (`qdb: 0, step_qdb: -2, period_s: 30, floor_qdb: -40` —
reaches the floor at t=600 s, then holds it for the final 120 s); three
120 s fast ramps (`step_qdb: -8, period_s: 10`, reaches the floor at
t=50s). 11 recordings, 8931 total sideport samples, 0 bad JSON decodes.

## Clean-percentiles table

`CLEAN U PER RUNG` (stress-off, residual-free samples), from the 600 s
baseline, 1191 samples, 100% time at rung 5 (mcs7):

| rung | p50 | p95 | p99 | n |
|---|---|---|---|---|
| 5 | 0.000 | 0.032 | 0.042 | 1191 |

## Sweep table

Every staircase level and both slow ramps show the identical shape: the
ladder climbs rung0->5 in ~30-35 s post-restart (the existing residual/
promote machinery reacting to cold-start acquisition — not what this
campaign calibrates), then parks at rung 5 for the rest of the run with
util/residual statistics indistinguishable from the clean baseline:

| run | offset | time at rung5 | rung5 u p50/p95/max | post-climb residual episodes |
|---|---|---|---|---|
| stair-8 | −8 qdb (−2 dB) | 90% | 0.00/0.03/0.07 | 0 |
| stair-16 | −16 qdb (−4 dB) | 88% | 0.00/0.03/0.06 | 0 |
| stair-24 | −24 qdb (−6 dB) | 84% | 0.00/0.03/0.08 | 0† |
| stair-32 | −32 qdb (−8 dB) | 88% | 0.00/0.03/0.05 | 0 |
| stair-40 | −40 qdb (−10 dB, floor) | 88% | 0.00/0.03/0.06 | 0 |
| rampslow-1 | 0→−40 over 600s, floor for last 120s | 95% | 0.00/0.03/0.08 | 0 |
| rampslow-2 | 0→−40 over 600s, floor for last 120s | 95% | 0.00/0.03/0.07 | 0 |
| rampfast-1..3 | 0→−40 over 50s, floor for last 70s | 68% | 0.00/0.0{2,3}/0.05 | 0 |

† stair-24 has one mid-run `rung 4->3 reason=residual u=0.01` blip
(recovers within 10 s) that does not register as a `residual_loss>0`
sample at the sideport's 2 Hz export cadence, so it does not enter the
`--calib` episode set; `u=0.01` would not have crossed even the lowest
swept candidate (0.05) regardless.

`CANDIDATE down_util SWEEP`, all 11 files (13 total detected episodes):
identical result for every candidate `0.05..0.60` — **3/13 caught,
median lead 5.5 s**. All 3 "caught" episodes are the second sample of a
cold-start pair (rung 0→1 climb attempts right after restart), not a
steady-state demotion.

Restricted to the binding rule's actual scope — **SLOW-ramp files
only** (`calib-rampslow-1.jsonl`, `calib-rampslow-2.jsonl`):

```
CANDIDATE down_util SWEEP (episodes=2, confirm_ms=250)
  0.05 .. 0.60: caught 0/2, median lead 0.0s   — every candidate, no exceptions
EPISODES
  calib-rampslow-1.jsonl t=8528889 rung=0 offset=0 uncaught at all candidates
  calib-rampslow-2.jsonl t=9252563 rung=0 offset=0 uncaught at all candidates
```

Both of the only two episodes detected in the slow-ramp data are
cold-start artifacts at the very first sample of their recording (no
preceding util history exists to measure a lead from) — **zero genuine
slow-ramp episodes were observed in the collected data, at any offset up
to and including the knob's floor.**

## Threshold derivation — result: BLOCKED

Applying the binding rule (verbatim from the spec):

- `up_util > clean p99 x >=2` → 2 × 0.042 = 0.084. The current default
  (0.15) already clears this — not the blocker.
- `down_util = highest candidate catching >=80% of SLOW-ramp episodes
  with median lead >= 2s` → **0/2 (0%) at every candidate in 0.05..0.60**.
  80% is unreachable: the only two slow-ramp "episodes" are cold-start
  artifacts, not induced-stress demotions.

**No candidate satisfies the rule. Per the spec, this is a STOP
condition: report BLOCKED, do not invent thresholds.**
`down_util`/`up_util`/`confirm_ms` are left at their compiled defaults
(0.6/0.15/250ms) — they were never overridden in `/etc/maburgs.json` to
begin with, so no config write was needed or made.

### Root cause (evidenced, not speculative)

The knob itself works correctly: Task 4 measured a −8 qdb setting as a
−2.00/−1.98 dB RSSI delta on both cards, and this campaign reconfirmed
`link.op.offset_qdb`/`drone.applied.offset_qdb` propagate correctly at
every level tested via boot-log + first-sample checks. The block is that
**the bench link's fade margin at mcs7 vastly exceeds the knob's full
range** (`floor_qdb=-40` = −10 dB): RSSI stayed in the −50..−62 dBm band
with SNR 58-68 dB throughout every recording, including at the floor.
That much headroom means even the maximum offset the knob supports
leaves the link nowhere near its actual RF wall — consistent with prior
bench findings (`docs/mcs6-bench-anomaly.md`, txagcbench work: "wall =
first-dip of a reproducible comb — islands above != headroom"; mcs6 park
at raw TX-power idx 47 = 99% still clean). The ladder controller behaves
exactly as designed throughout: hard demote during cold-start acquisition
(existing, uncalibrated-here machinery), clean climb back to rung5, and
a fully clean park regardless of offset level, because nothing in the
knob's range actually stresses this bench's link.

## Acceptance

Not applicable — no thresholds were changed, so there is nothing to
accept. In lieu of acceptance, the GS was verified restored to its exact
pre-campaign state:

- `diff /etc/maburgs.json /etc/maburgs.json.pre-calib` → IDENTICAL (no
  `stress_offset` block, no threshold overrides).
- Boot log after final restart: 0 `STRESS` lines.
- rtpsniff gate (30 s, post-campaign): `pkts=21631 (7.78 Mbps)`,
  `seq gaps=0 (0.00%)`, `frames=1785 ok=1784 bad=1 (0% bad)`,
  `ok_fps=59.5` — clean, matching every other gate reading taken across
  this campaign and Task 4.

## Recommendation for a future attempt

Completing the util-path calibration needs a stress mechanism that can
actually reach this bench link's RF wall at rung5. Options for a future,
separately-designed campaign:
1. Deepen `stress_offset.floor_qdb` past −40 (currently hard-clamped to
   `[-40, 0]` in `gs/src/config.cpp` — a deliberate bench-safety bound
   from Task 1/2, would need its own review to relax).
2. Physically increase bench range/attenuation to cut the ~60 dB SNR
   margin directly, rather than relying on TX-power offset alone.
3. Target a lower, already-marginal rung (e.g. temporarily cap
   `max_mcs`/`static_mcs`) and apply the offset on top of that rung
   instead of offsetting from the naturally-parked mcs7 point.

## Inventory

GS `/root/calib/`: `calib-baseline.jsonl`, `calib-stair-{8,16,24,32,40}.jsonl`,
`calib-rampslow-{1,2}.jsonl`, `calib-rampfast-{1,2,3}.jsonl` (plus
per-run `.log` capture-summary files, `driver.sh`, `DRIVER_SUMMARY.txt`,
`DRIVER_DONE` — unattended-sequencing artifacts, harmless, left in place).

Host `out/calib-2026-07-30/`: the same 11 `.jsonl` files, scp'd from the
GS, line counts verified to match exactly.

## Rollback note

`/etc/maburgs.json.pre-calib` on the GS is the pre-campaign config
snapshot (identical to the post-campaign live config — nothing needed
rolling back). `/usr/local/bin/maburgs.pre-stress` (from Task 4) remains
as the pre-stress-knob binary rollback, unused.
