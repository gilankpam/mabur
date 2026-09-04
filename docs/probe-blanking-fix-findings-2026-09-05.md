# Probe blanking fix — measured, fixed, A/B'd (2026-09-05)

Closes `docs/handover-probe-blanking-2026-09-04.md` (Finding 2 of
`docs/probe-stream-findings-2026-09-04.md`). Branch `probe-stream`,
commits `b2a7fd0` (instrumentation) + `c453023` (fix). Bench rig as in
the handover: pinned mcs4, `link.probe.pin_mcs 4`, `feedback_ms` 50,
`rcf_slot_hold_ms` 30, agg 6, 60 AU/s (30 fps base + 30 fps enh, enh AU
~37 kB p50).

## TL;DR

The probe stream lost ~2.5× what the enh stream lost because
`RcfSlotter` released the GS's control send at the ENH completion, and
the probe — the last PPDU of the burst — lands **0.9 ms p50 / 4 ms p99
after the completion stamp**, while the send's real USB+chip latency is
1–1.5 ms. The blast was aimed at the probe by construction; the 1 ms
tail of `bee51c6` merely moved it from one part of the probe's arrival
spread to another. The slotter now **releases on the probe's own
arrival** (the exact "burst off air" signal at any MCS or frame size),
with a self-learned deadline for lost probes. Interleaved A/B: probe
body loss **0.21 % → 0.06 %**, now *below* the enh stream's own 0.08 %,
enh loss unchanged, send-reason mix otherwise unchanged.

## 1. Closing the instrumentation gap (`b2a7fd0`)

The handover's blocker was that `ProbeFinalized::t_ms` is the ~10 ms
finalize tick. `ProbeTrack` already held the radio's µs arrival stamp of
the first sight (`Pending::first_ms`, fed from `RxBody::mono_us`); it is
now carried out as `ProbeFinalized::first_ms` and logged as the 11th
column of `probelog 2`. `flightreport.py` joins it to flightrec's
`au-NNNN.log` `t_complete` (same CLOCK_MONOTONIC) on `enh_fid`, nearest
in time, and prints the completion→probe percentiles; a probe log alone
is a valid input, and the au log (different directory and index) is
found by mono-time overlap. `tools/bench/probesend.py` adds the send
stamps of a `MABUR_GAPLOG=1` run.

## 2. What the measurement said

Instrumentation-only binary, 12 min (arm A1 below):

| quantity | p10 | p50 | p90 | p99 | max |
|---|---|---|---|---|---|
| probe first sight − enh `t_complete` (ms) | −0.70 | **0.86** | 2.15 | 4.31 | ~6 |

Not the 3–4 ms the handover guessed: the AU completes at the *end* of the
burst (the FEC path delivers with the last aggregate — `au_tail` gauge:
span mean 12.7 ms, publish tail mean 0.8 ms), and the probe is one short
PPDU behind it. Negative values are probes riding in the same aggregate
as the AU's last body, stamped at the radio before the core loop stamped
the completion. Enh AUs here are ~37 kB → a 12.7 ms burst on a 16.7 ms
period: **only ~4 ms of idle**, so this is a heavy operating point.

Sends vs probes (`probesend.py`), same arm: of the **55 lost probes, 45
had a GS send within −2…+6 ms of their AU's completion**, against 21.8 %
of delivered probes — ~60 % of all probe loss is GS-inflicted, which is
the `feedback_ms` 200 → 0 result from another angle. The send phases
form two clusters:

- **+0.5…+1.7 ms**: `SlotReason::Au` releases at the ENH completion
  (+1 ms tail). On air 1–1.5 ms later = on the probe. The fix's target.
- **−2.6…−0.4 ms**: hold **timeouts**. 43 % of all sends on this bench
  are timeouts (idle ~4 ms vs `lead_ms` 3 + `guard_ms` 1 leaves no
  margin), and because `feedback_ms` 50 is exactly three 16.7 ms AU
  periods, the offer→timeout phase is locked to the burst for minutes at
  a time (the two clocks drift slowly). When that phase sits ~1 ms
  before the enh completion the blast lands on the probe, and it does so
  on consecutive frames — the runs of 2–3 adjacent lost probes.

## 3. The fix (`c453023`)

`RcfSlotter::on_au_complete(now, probe_follows=true)` (an ENH AU while
a probe is commanded) arms **no release**. `on_probe_tail(now)`, called
from the probe sink on every sight (any card, any profile, parseable or
not), is the release (`SlotReason::Probe`, sideport
`link.rcf_slot.probe`), with `idle_ahead()` checked at that instant and
nothing charged at the completion. A lost probe never arrives, so the
ENH completion also arms a deadline at completion + `tail_ub_ms` —
`ceil(decaying max of observed offsets, floored at the probe body's
airtime) + 1` (exported as `link.rcf_slot.tail_ub_ms`; learned 5–6 ms on
the bench) — which releases only if idle-ahead when it fires. The grace
window opens at the burst end (probe arrival, or the estimate while the
probe is still expected). Base-AU completions release at once, as
before. Tests: `tests/test_rcf_slot.cpp` (22).

## 4. A/B

Interleaved, same rig, only the binary varying, `MABUR_GAPLOG=1` in
every arm (both binaries write `probelog 2`). Probe loss = seq span −
rows; enh pre-FEC from the flightrec jsonl's `link.streams[1]`
(`recovered + abandoned` over `syms_in_s`·dt); "lost w/ send" = lost
probes whose AU had a GS send within −2…+6 ms of its completion.

| arm | binary | min | probes | lost | probe loss | enh pre-FEC | lost w/ send | sends: au / probe / timeout |
|---|---|---|---|---|---|---|---|---|
| A1 | stamp-only (`b2a7fd0`) | 12 | 21757 | 46 | **0.211 %** ± 0.031 | 0.083 % | 45/55* | 54 % / — / 43 % |
| B1 | fix (`c453023`) | 8 | 11127 | 7 | **0.063 %** ± 0.024 | 0.077 % | 7/7 | 26 % / 20 % / 43 % |
| A2 | stamp-only | 8 | 14083 | 20 | **0.142 %** ± 0.032 | 0.089 % | 20/20 | 59 % / — / 37 % |
| B2 | fix (`fb721cf`†) | 8 | 14097 | 7 | **0.050 %** ± 0.019 | 0.085 % | 9/13* | 37 % / 29 % / 22 % |

\* The AU-attributed lost count can exceed the seq-span count (a probe
lost on air whose AU is also abandoned books twice in the join, and B2's
au log was fetched mid-write so only 4352 of its rows joined); the
seq-span number is the loss rate.
† `fb721cf` = `c453023` + the spent-deadline guard (a lost-probe deadline
that passed with nothing pending no longer fires on a later offer); the
normal path is identical. B1 ran `c453023`. A1 ran for 12 min because it
doubled as the measurement run; the B1 daemon was not killed by the first
A/B script (process-name mismatch, `killall maburgs` vs
`maburgs.probe-arrival`) and spoiled the first A2/B2 attempt — those were
rerun one daemon at a time; the numbers above are the clean runs.

**Pooled: pre-fix 66/35840 = 0.184 %, fix 14/25224 = 0.056 %, difference
−0.128 ± 0.027 pp (1 sd, 4.7 sd).** Enh pre-FEC 0.083/0.089 % → 0.077/0.085 %:
unchanged. The canary now reads below the stream it predicts in both fix
arms. The send mix moved as designed: `au` 54–59 % → 26–37 %, `probe` 20–29 %
takes the ENH bursts, `grace` 4 % → 10–13 % (the grace window now opens at
the burst end); `timeout` 43/37 % → 43/22 % — not a target of this fix and
noisy across arms.

The residual loss in the fix arms is the timeout cluster (all remaining
lost probes had a send at −1.8…+0.6 ms), which is mechanism 2 above and
not probe-specific: a phase-locked timeout kills video aggregates the
same way. It is left as the next lever (below).

## 5. What is deployed

GS: `/usr/local/bin/maburgs` = fix (`fb721cf`), under `S96maburgs` since
2026-09-05 ~11:59 GS time (probe-0270). Rollbacks:
`maburgs.pre-probearrival` (= `b2a7fd0`, stamp-only) and
`maburgs.pre-probestamp` (= `bee51c6`, the handover's binary). No config
change (no new keys). Drone untouched. `ausniff` gate on the deployed
binary: 60 s, `aus=3584 complete 1792/1792 incomplete={} fid_gaps=0
resyncs=0 fps=59.7`. Bench helper `/root/probeab.sh` (one daemon at a
time, `MABUR_GAPLOG=1`, restarts `S96maburgs` when done) is on the GS for
the next A/B; ⚠ `killall maburgs` does not match `maburgs.<suffix>`
process names — kill both, or by pid.

## 6. Next levers (not done)

1. **Timeouts at high duty cycle.** `lead_ms` 3 is ~2× the measured
   1–1.5 ms send latency; with ~4 ms idle that turns ~43 % of sends into
   30 ms hold timeouts at a burst-locked phase. `lead_ms` 2 (or a
   measured value) and/or `rcf_slot_hold_ms` 50 would let most of them
   ride a completion instead. Gate on `ausniff` + enh pre-FEC + the
   `probesend.py` window test; the bench at pinned mcs4 is the right
   rig because it is the heavy case.
2. **`feedback_ms` ≠ k × AU period.** 50 ms is exactly 3 periods at
   60 AU/s, which is what makes a timeout phase stick. Any value that is
   not a small multiple (e.g. 55) de-locks it for free; untested.
3. Flight: compare per-mcs `probe-NNNN.log` loss against
   `link.streams[1]` over the same window — the ratio should now be ≤ 1.
