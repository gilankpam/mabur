# Handover: the probe stream loses ~2× what video loses, and the GS does it (2026-09-04)

Status: **OPEN.** Root cause is established and reproducible — the loss is
GS-inflicted, not RF — but the fix is NOT found. The obvious correction
(make `RcfSlotter` wait out the probe body's own airtime, `bee51c6`,
deployed) is **measurably ineffective**: it is the right mechanism with
the wrong number. Everything else about the probe stream passed
acceptance; see `docs/probe-stream-findings-2026-09-04.md`. Branch
`probe-stream` / PR #44, bench-deployed, unflown.

## TL;DR

The probe stream is one 1403 B body at the tail of every ENH burst. On a
clean bench link it loses **0.15 % of bodies while the enh stream loses
0.07 % of symbols at the same MCS** — the canary reads about twice the
loss of the thing it is supposed to predict. The loss is not RF: it
scales with the ground station's own control-send rate and falls to
**zero at `feedback_ms` 200**. `RcfSlotter` releases GS control frames at
the AU *completion*, and the probe is the last thing on air after that
completion, so the slotter is steering the GS's own ~180 µs transmit
blast onto the probe. Turning the slotter off **halves** probe loss while
raising enh loss — it is protecting video at the probe's expense.

Adding the probe body's serialization (288 µs at mcs4, ceil → 1 ms) to
the slotter's release did nothing — measured over 57 k probes in an
interleaved A/B, −0.007 ± 0.037 pp. The gap between "AU decodable" and
"burst off air" is evidently several milliseconds — roughly a third of
each enh burst is repair symbols that follow decode-completion — so one
body is far too short a wait. **The next person needs to measure that
offset rather than guess it, and the instrumentation to do so does not
exist yet (see "Instrumentation gap").**

## Why it matters

`u_probe = probe_loss / budget_enh`. With the shipped geometry a 500 ms
window holds 15 probe bodies, so **one lost body = `u_probe` 0.2 exactly**
(4 blocks of 60, budget 0.333), 0.214 in a 14-body window, 0.4 for two.
The bench's tuned `link.probe.max_util` is 0.2, so a single self-inflicted
loss trips the gate to `lossy` for the ~10 consecutive 50 ms samples the
sliding window covers, resetting the clean streak.

Promotes still land today (loss events are 7–15 s apart, a 2 s streak fits
between them), and the whole cold climb 0→5 ran on `promote_probed`. But
the margin is thin and it is spent on a fault the GS creates: at range,
where real probe loss appears, the gate will be reading roughly double.
Anyone tuning `max_util` from bench data is tuning against this artefact.

## Evidence

### 1. The probe loses more than the enh stream it predicts

Pinned mcs4 with `probe.pin_mcs 4` — same MCS, same air, 91 s, first
measurement (pre-`bb8d5d7` binary):

| | value |
|---|---|
| probe bodies received / lost | 2682 / 12 → **0.45 %** |
| enh pre-FEC symbol loss | **0.06 %** (917 k symbols) |
| per-card delivery | both cards delivered every probe they heard (c0 = c1 = 2682) |

Later runs on the fixed binary measured 0.15 % probe vs 0.07 % enh, so the
absolute rate moves with conditions; the **ratio ~2× is what reproduces**.

### 2. It is GS-inflicted: loss scales with the control-send rate

Pinned mcs4 / probe mcs4, 91 s per arm, probe loss from the per-body log's
seq span, enh from the sideport's sid-1 counters:

| arm | GS ctrl sends | probe body loss | enh pre-FEC |
|---|---|---|---|
| `feedback_ms` 50 | 25/s | 0.149 % | 0.073 % |
| `feedback_ms` 200 | 5/s | **0.000 %** | 0.037 % |
| slotter OFF (`rcf_slot_hold_ms` 0) | 20/s | 0.074 % | 0.106 % |
| `feedback_ms` 50 (repeat) | 24.8/s | 0.111 % | 0.081 % |

Two independent signals in one table. Probe loss tracks the send rate and
**vanishes** when the GS talks 5× less often. And the slotter, whose whole
job is to put those sends in the inter-AU idle, **halves probe loss when
switched off while raising enh loss** — i.e. with it on, sends are aimed
exactly where the probe now sits.

This is the same mechanism as
`docs/gs-uplink-self-blanking-findings-2026-09-02.md`: a GS transmit
blasts the sibling RX card at ~−4 dBm and deafens the TX card, so a drone
PPDU whose preamble starts within ~180 µs is lost on BOTH cards. The
probe is the last PPDU of the burst, which makes it the easiest one to
hit with a send released "at the end" of that burst.

### 3. The probe-serialization tail does not fix it

`bee51c6` teaches `RcfSlotter` that the burst ends one probe body after
the AU completes (`RcfSlotCfg::probe_tail_ms`, computed per commanded
probe MCS: 1403 B ⇒ 2 ms at mcs0, 1 ms from mcs1 up; charged only to ENH
completions, in both the deferral and the cadence gate). Same four arms,
same rig, immediately after deploying it:

| arm | probe loss pre-fix | post-fix | lost bodies post |
|---|---|---|---|
| `feedback_ms` 50 | 0.149 % | 0.148 % | 4 / 2695 |
| `feedback_ms` 200 | 0.000 % | 0.037 % | 1 / 2694 |
| slotter OFF | 0.074 % | 0.074 % | 2 / 2693 |
| `feedback_ms` 50 (repeat) | 0.111 % | 0.260 % | 7 / 2693 |

**No effect**, and those arms were too small to prove one either way — 1 to
7 lost bodies each, Poisson error ±2 to ±3. So the A/B was repeated long
and interleaved (fix / pre-fix / fix / pre-fix, 8 min each, pinned mcs4 /
probe mcs4 / `feedback_ms` 50, only the binary varying):

| arm | probes | lost | probe loss | enh pre-FEC |
|---|---|---|---|---|
| fix 1 | 14292 | 24 | 0.168 % | 0.105 % |
| pre-fix 1 | 14290 | 27 | 0.189 % | 0.086 % |
| fix 2 | 14297 | 31 | 0.217 % | 0.088 % |
| pre-fix 2 | 14291 | 30 | 0.210 % | 0.085 % |

**Pooled: fix 55/28589 = 0.192 %, pre-fix 57/28581 = 0.199 %, difference
−0.007 ± 0.037 pp (1 sd).** The error bar is now several times smaller
than any effect that would matter, so this is a genuine null, not an
underpowered one: **the probe-serialization tail does not reduce probe
loss.** (The ~2× probe-to-enh ratio reproduces across all four arms:
0.19 % probe vs 0.09 % enh.)

## Mechanism, and why 1 ms was the wrong number

The GS raises "AU complete" when the access unit is **decodable**, not
when the drone has stopped transmitting it. With `overhead_enh` 0.5,
roughly a third of every enh burst is repair symbols, and the probe is
sent after all of them. From `aucadence`, an enh AU is ~36 kB p50; at
mcs4 (~39 Mb/s PHY) source+repair is on the order of 10–11 ms of air, so
the tail after decode-completion plausibly runs **3–4 ms**, not the 288 µs
of the probe body alone.

So `probe_tail_ms` is the right knob in the right place, set to a number
derived from the wrong quantity. It should be the distance from the
completion signal to the end of the burst, of which the probe body is only
the last fraction.

## Instrumentation gap (this is what blocks the fix)

The obvious measurement — join the per-body probe log to `au-NNNN.log` on
`enh_fid` (the join key exists precisely for this) and histogram
`probe_arrival − au.t_complete` — **cannot be done with what is logged
today**: `ProbeFinalized::t_ms` is the *finalize* time, i.e. first sight
plus `kProbeFinalizeMs` (100 ms), resolved at core-loop tick granularity
(~10 ms). That is two orders of magnitude coarser than the 1–4 ms offset
being measured.

Cheapest close: add a `first_seen_ms` (or µs) to `ProbeFinalized`, stamped
in `ProbeTrack::on_body` on first sight, and log it as an extra column in
`probe-NNNN.log`. Then the offset histogram is a few lines of
`flightreport.py`. Do this before attempting any further tail value —
without it, every candidate number is another guess.

## What would fix it, ranked

1. **Measure, then set `probe_tail_ms` from data.** Needs the stamp above.
   Cheap, keeps the mechanism already reviewed and deployed. Expected
   result: probe loss falls toward the `feedback_ms` 200 floor with enh
   unchanged.
2. **Release on the probe's arrival instead of a duration.** The GS
   *observes* the probe land; that event is the exact "burst is over"
   signal, self-correcting across MCS and frame size. Add
   `RcfSlotter::on_probe_tail(now_ms)` called from the probe sink, and
   keep `probe_tail_ms` as an upper bound so a lost probe cannot stall the
   release past `hold_max_ms`. Strictly more accurate than (1); slightly
   more coupling.
3. **Send the probe BEFORE the enh AU rather than after.** Removes the
   probe from the burst tail entirely, so the slotter's existing rationale
   holds unmodified. This changes the design's cadence claim ("after every
   enh AU", chosen so the probe cannot land in the RCF blast window at a
   *random* time) and would need the whole slotting argument re-validated —
   but it may be the structurally correct answer.
4. **Raise `feedback_ms`.** Already proven to take probe loss to zero, and
   `docs/gs-uplink-self-blanking-findings-2026-09-02.md` measured
   50→200 cutting *video* loss 3×. Costs control latency
   (`link.attrib.close_ms`); it is a link-wide tuning decision, not a probe
   fix.
5. **Accept it and raise `link.probe.max_util`.** Zero code. The gate then
   tolerates one self-inflicted body per window, at the cost of tolerating
   one real one too. Reasonable stop-gap for the first flight.

## Bench procedure (as run)

```sh
# binaries on the GS
/usr/local/bin/maburgs               # current (bee51c6, probe tail)
/usr/local/bin/maburgs.pre-slotfix   # 2a17832's parent — A/B partner
/usr/local/bin/maburgs.losssim       # -DMABUR_LOSS_SIM=ON build
/usr/local/bin/maburgs.pre-probe     # pre-branch rollback
# host: tools/build-arm64.sh; a loss-sim variant needs -DMABUR_LOSS_SIM=ON
# added to its cmake line and only the maburgs target built.
```

Arms are config swaps + a restart; each records the probe log's row count
and seq span over the window, plus a sideport snapshot at both ends
(`/root/sidesnap.py`, which sniffs loopback and does not steal the port).
Probe loss = `(last_seq − first_seq + 1) − rows`. Enh pre-FEC comes from
`link.streams[1]`: `(recovered + abandoned) / (syms_in_s·dt + recovered +
abandoned)`.

⚠ **`losssim.py` defaults to `--port 8302`; `S96maburgs.losssim` starts the
daemon with `--loss-sim 8390`.** Pass `--port 8390` or the tool times out
with "is maburgs running with --loss-sim?" and you will think injection is
broken. The in-repo tool has the same stale default.

⚠ At the **top rung** the S-line probe columns read `-1 nan 0` and the gate
is `off` — there is nothing above to probe. To sample `u_probe`, catch the
climb or hold the ladder below top (`link.clean_ms: 60000` works).

## Open questions

- What IS the completion→burst-end offset, and how much does it move with
  rung, frame size and `overhead_enh`? (Blocked on the stamp above.)
- Does the slotter's release actually land on the probe, or does it land
  in the repair tail and take the probe out via the aggregate? The A-MPDU
  batching means one blast can cost several bodies
  (`docs/gs-uplink-self-blanking-findings-2026-09-02.md`), so "which PPDU"
  may be the wrong question — count *whole aggregates* lost instead.
- Why did the first pinned run measure 0.45 % probe loss and later runs
  0.15 %? Same rig, same config. Bench RF drift, or a real dependence on
  something not controlled (MSP traffic? DVR write bursts?).
- Is the ~2× ratio stable at range, or does real RF loss swamp it? The
  flight will answer this for free: compare per-mcs probe loss in
  `probe-NNNN.log` against `link.streams[1]` over the same window.

## Related

- `docs/probe-stream-findings-2026-09-04.md` — deploy + acceptance record
  (this is Finding 2 there, with the rest of the acceptance evidence).
- `docs/gs-uplink-self-blanking-findings-2026-09-02.md` — the parent
  mechanism and the reason `RcfSlotter` exists at all.
- `docs/link-adaptation.md` "RCF slotting" and "Probe stream".
- Commits: `bb8d5d7` (AU/body finalize phase lag — a *different*, fixed
  cause of phantom `u_probe` 0.2), `2a17832` + `bee51c6` (the probe tail),
  PR #44.
