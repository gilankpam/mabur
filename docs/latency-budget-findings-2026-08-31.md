# Latency budget — first instrumented flight, and why glass-to-glass brushes 100 ms

2026-08-31. First flight with the seven-segment latency accounting live
(branch `latency-accounting`, GS at the enc-excess build 191c177; drone at
196f507). Sources: `flight-0035.jsonl` (maburgs' anchored `link.video.lat`
aggregates, 1240 warm windows) + `au-0035.log` (aulog 2: exact per-AU
`t_first t_complete enc dq`, 14 955 AUs). Player tail segments
(`dec/reg/dsp`) were NOT recorded — `lat:` lines live in `/tmp` (tmpfs)
and died with the power-off (follow-up #4) — bench values fill those rows.
Config context: **asymmetric overhead pair (base 1.0 / enh 0.5) at every
rung** (fade-A/B leftover, deliberately kept), ~half the flight below
mcs5 (rung dwell s: mcs1 66, mcs2 31, mcs3 47, mcs4 54, mcs5 87).

## The budget (typical flight frame, p50)

| stage | ms | measured? |
|---|---|---|
| sensor exposure + readout (before pts) | ~10–16 | no — physics bookend |
| `enc` encode (capture→VENC out) | 7.1 | yes (p99 8.0 — flat) |
| `dq` TxQueue wait | 7.0 | yes (p99 22, max 88) |
| transit floor + `air` excess | ~2 + 4.1 | air yes; floor estimated |
| `fec` first-body→AU complete | 9.8 (base 12.6 / enh 6.2) | yes (base p99 41) |
| `dec` ring→decoded | ~6 | bench value |
| `reg` regulator hold (D=12) | ~2–5 | bench value |
| `dsp` release→vblank | ~15 | bench value (10–25 beat sweep) |
| vblank→mid-screen scanout | ~8 | no — physics bookend |
| **total glass-to-glass** | **≈ 75–85 typical** | |

Tails: per-frame e2e-above-floor p99 measured 62–74 ms on the bench build;
add the ~25–30 ms of bookends and any tail coincidence (dq p99 22, base
fec p99 41, `air` excursions to 300 ms in fades) and **worst-case frames
exceed 100 ms glass-to-glass** — past the comfortable FPV envelope. The
operator's in-flight impression (~80 ms) matches the p50 column.

Caveats on the two unmeasured bookends: `e2e` is floor-referenced (the
min-anchor eats the absolute minimum path — first-body air, USB, chip
FIFO floor — by design, no clock sync), and nothing here sees the sensor
or the panel. Standing injection backlog past the TxQueue pop (USB pool,
bulk-out, ~65–70 KB chip FIFO) IS visible, but only as `air` excess above
the floor, not absolutely. Follow-up #7 pins the absolutes once.

## Attribution highlights

- **The asym pair costs ~7–8 ms of the p50 + most of the jitter.** Base
  `fec` 12.6 vs enh 6.2 (base flies 2× air on equal payloads) and the
  standing `dq` both trace to the pair: the stream (~28 Mb/s at the mcs5
  park) rides the ~26 Mb/s USB bulk-out ceiling, which the bitrate blend
  does not model. `jitter_ms` median 14.9 in flight vs 5–8 on the flat
  pair. aucadence (t_complete clock) reads **+4.73 ms base-late** on this
  config — sign-flipped vs the flat-pair baseline (−1.1..−3.0); the gate
  number is config-dependent, record the pair next to it.
- **`dq` has a standing ~7 ms p50 even at the park** — invisible before
  the enc-excess fix (it clamped to 0). Congestion transients reach 88 ms
  without a single `txq_drop`: `dq` is now the early-warning gauge.
- **`enc` is 7 ms flat** (p99 8.0) — the first true encoder-latency
  number; the old "pts→publish <1 ms" was cross-timebase and only proved
  flatness (`docs/airtime-model.md` note).
- **`dsp` fluctuates 10–25 ms with a ~16 s period** — vsync quantization
  swept by the 59.939-vs-60.000 Hz sensor↔panel beat, plus one extra
  vsync of flip serialization at p99. Largest single reducible segment.

## Follow-ups (ranked by ms recovered / effort)

1. **Vsync-locked regulator** (~10 ms, player-only): servo release to
   ~2 ms before the vblank estimated from the now-captured kernel flip
   timestamps. Needs the 2-deep release queue first (single-slot mailbox
   self-defeats at holds ≥16 ms) and an `--fps-log async=` check to size
   the win. Memory: `vsync-locked-regulator`.
2. **Pair policy decision** (~7–8 ms + jitter, config-only to test):
   flat 0.5/0.5 A/B against the asym pair's fade protection — the fade
   A/B that motivated asym is still open; now both sides of the trade are
   measurable (`fec` per class, `dq`, jitter, fade damage windows).
3. **Model the USB ceiling in the bitrate blend** (removes the standing
   `dq`): the budget is airtime-based and happily commands past the
   ~26 Mb/s bulk-out cap. Same error family as the open
   `mcs1-budget-overshoot`. Alternative cheap mitigation: trim
   `airtime_budget` until `dq` p50 ≈ 0 and measure what video rate that
   actually costs.
4. **Persist the player tail segments** (observability, tiny): `lat:`
   lines die in tmpfs; have flightrec (or the player itself) append them
   to `/media/dvr/log/` per session so the next flight records all seven
   segments, not four.
5. **Absolute glass-to-glass calibration, once** (turns floor-referenced
   numbers absolute): LED-in-frame + high-fps phone camera, one bench
   session; pins the sensor and scanout bookends and the anchor floor.
6. **Sideport `lat` window undercount** (`n`=12 per 500 ms vs ~30
   expected) — harmless to correctness, unexplained, worth a look.
7. `enc` 7 ms: encoder-side; star6e RC knobs mostly dead
   (`airtime-model.md` §3) — park unless something new appears.

Items 1–3 together take a typical frame from ~80 to ~60 ms — about the
floor for this sensor+panel at 60 fps — and pull the tail back under
100 ms, which is the actual FPV-feel requirement.
