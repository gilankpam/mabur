# Bench protocols — latency follow-ups, next power-on

2026-08-31. Four measurement protocols queued by
`docs/latency-budget-findings-2026-08-31.md` and the vsync-locked-regulator
spec (`docs/superpowers/specs/2026-08-31-vsync-locked-regulator-design.md`,
gitignored — this page is the durable copy of protocol 1). All four are
bench-blocked (need a live drone + GS pair and, for #4, a phone). Written
so the next power-on session is pure measurement, no design work.

## 1. Vsync A/B

Same-binary A/B, config toggle only, identical scene and op point (mcs5
park). This is the bench acceptance gate for the vsync-locked regulator
(`display.vsync_lock`) — a fail here means ship dark, not iterate on hw.

**Arm procedure** (copied from the spec's bench-acceptance section, as
amended):

- **Arm A (baseline)**: `display.vsync_lock: false` — today's D=12 rule.
- **Arm B (servo)**: `display.vsync_lock: true`.
- ≥ 5 min per arm (≥ 18 beat periods), player restarted between arms so
  each lands in its own `lat-NNNN.log`. Order **A → B → A′** (short A′
  repeat to catch scene drift).
- Evidence = the persisted lat logs (`docs/observability.md` has the
  format) plus the 1 Hz `regulator:` stderr line in `/tmp/maburplay.log`.

**Commands**: edit `display.vsync_lock` in `/etc/maburplay.json`, restart
`maburplay` between arms, then on the GS:

```
python3 tools/bench/latab.py latA.log latB.log
```

**Log-derived gates** (from `latab.py`, computed over `anchor=ok` windows
only — `anchor=warm` windows are excluded and the script prints the
excluded count):

| metric (from lat lines) | arm A expected | arm B gate |
|---|---|---|
| `dsp` p50 | ~15 ms, sweeping 10–25 over 16 s | **≤ 6 ms and flat** (no beat sweep) |
| `dsp` p99 | ~25–30 ms | **≤ A − 8 ms** |
| `e2e` p50 (player) | baseline | **≤ A − 8 ms** |
| `dsp` p50, 4 s-bucket sweep | — | **≤ 3 ms** (beat-signature check) |

`latab.py` prints PASS/FAIL per row and an overall verdict; it wants
≥300 windows per arm (≥5 min at 1 Hz).

**Manual gates** (read from the `regulator:` line and `--fps-log` in
`/tmp/maburplay.log`, not from the persisted logs):

| metric | arm A expected | arm B gate |
|---|---|---|
| present-jitter EMA | ~2.94 ms | **≤ A + 0.5 ms** |
| vsync repeat/skip pairs (`--fps-log`) | baseline | **≤ A** |
| fps / ausniff | ~60 / 0 gaps | unchanged |

Plus, in **arm B only**:

1. Presenter in-flight-mailbox engagements (`pend=` on the `regulator:`
   line) ≈ 0/min — record arm A's `pend` value too, for the comparison.
2. `vsync_skips` ≈ 0, bounded above by `duration / 16.4 s` under decode
   jitter. ⚠ **Corrected physics**: the real sensor (59.939 fps) is
   SLOWER than the 60.000 Hz panel, so the beat wrap (period ≈ 16.4 s)
   produces one PANEL REPEAT, not a frame drop — `vsync_skips` in flight
   is expected ≈ 0. Repeats show up in `--fps-log`, not as `skips`. A
   nonzero `skips` count roughly tracking `duration / 16.4 s` is the
   expected contention-per-wrap regime, not a regression; a count well
   above that bound is a real fault.
3. Fallback drill: kill/restart video mid-session — `fallback=` on the
   `regulator:` line climbs then stops after the estimator re-warms
   (8 exact flips); no stall, no regression vs arm A behavior during the
   cold window.
4. `--fps-log async=` check — sizes what flip serialization remains.
5. `lat-NNNN.log` sync headers (`# sync <mono_us> <wall_us>`) present and
   sane in both arms.

**Failure action**: any gate fails → ship dark (`display.vsync_lock:
false`, same binary, config-only) and bring both logs home for a redesign
pass rather than iterate live.

**Evidence location**: `lat-NNNN.log` pair (one per arm) under
`display.lat_log_dir` (default `/media/dvr/log`), plus
`/tmp/maburplay.log` for the manual-gate lines.

## 2. Pair policy A/B (latency-budget follow-up #2)

Decision input, not a pass/fail gate — the fade A/B that originally
motivated the asymmetric pair is still open, so this protocol makes both
sides of the trade measurable rather than picking a winner outright.

**Setup**: `/etc/maburgs.json` ladder config.

- **Arm A (current)**: today's asymmetric pairs, `1/100:50 … 5/100:50`.
- **Arm B (flat)**: `overhead_base == overhead_enh = 0.5` at the mcs4/5
  rungs, per the repo-default ladder.

**Commands/duration**: ≥10 min per arm at the mcs5 park, plus one
loss-sim fade pass per arm (`MABUR_LOSS_SIM`, `bench/shed-lag-validation`
rig) to see the asym pair's fade-protection case exercised, not just the
clean-air cost.

**Compare** (sideport, aulog sid split for the per-class read):

- `lat.fec` p50 per class (base vs enh)
- `dq` p50
- `jitter_ms`
- aucadence base−enh completion offset (`tools/bench/aucadence.py`)
- fade damage windows (frames lost/smeared during the loss-sim pass)

**Evidence location**: sideport jsonl capture per arm
(`flight-NNNN.jsonl`) + `aucadence.py --json` output per arm. Both arms'
numbers go in the flight log side by side — this protocol produces a
comparison table, not a verdict.

## 3. Drain-ceiling measurement (latency-budget follow-up #3)

Finds the number that should replace nominal `phy_rate_mbps` in the
bitrate blend. Background and the interim mitigation (trim
`airtime_budget` until `dq` p50 ≈ 0) are in
`docs/latency-budget-findings-2026-08-31.md` follow-up #3 — this
protocol is the measurement plan, not a restatement of that context.

**Setup**: pinned mcs5 and pinned mcs1 (two separate sweeps — the ceiling
is expected to differ by op point).

**Commands**: at each pinned MCS, sweep `radio.tx_threads` over
1 / 2 / 4 / 8, one config-only restart per value. Watch sideport
`drone.txq_wait_ms` and `dq` p50 for each.

**Measurement**: record the effective burst-drain Mb/s per
`radio.tx_threads` setting, using the airtime model's calibrated slope
(`docs/airtime-model.md`: completion delay ≈ 4.2 ms + 0.85 µs per payload
byte) against the observed `air` + `fec` completion numbers — same
method as the model's existing calibration, applied per sweep point
instead of once.

**Deliverable**:

1. The effective burst-drain number (Mb/s) that should replace nominal
   `phy_rate_mbps` in a follow-up blend change, per pinned MCS.
2. The interim `airtime_budget` trim value at which `dq` p50 ≈ 0, and the
   video rate it costs — usable immediately, independent of the blend
   change.

**Evidence location**: sideport captures per sweep point
(`drone.txq_wait_ms`, `dq` p50) + the derived Mb/s table, written up as a
dated findings note alongside this page.

## 4. LED absolute calibration (latency-budget follow-up #5)

Pins the two unmeasured bookends (sensor exposure/readout before pts, and
vblank→mid-screen scanout) against the same session's `lat:` e2e, turning
the floor-referenced glass-to-glass numbers into absolute ones.

**Setup**: an LED in frame, wired to a GPIO toggle synchronized with a
known event (e.g. the same GPIO record-button header, or a bench pulse
generator); a phone camera in slow motion (≥240 fps) filming the LED and
the goggle screen side by side in one shot.

**Procedure**: ≥20 toggles. For each toggle, read the frame delta between
the LED's video-visible transition and the goggle screen's corresponding
visible change; glass-to-glass = frame delta × camera frame period (e.g.
at 240 fps, 1 frame ≈ 4.17 ms resolution).

**Gate**: none — this is a calibration pass, not pass/fail. The output is
a single absolute glass-to-glass number (median + spread over the 20
toggles) to compare against the same session's `lat:` e2e p50, which
pins the two physics-bookend rows in the latency budget table
(`docs/latency-budget-findings-2026-08-31.md`) that today read "no —
physics bookend".

**Evidence location**: the slo-mo video file (kept off-repo) + a written
frame-count tally per toggle, plus the session's own `lat-NNNN.log` for
the concurrent `e2e` reading.
