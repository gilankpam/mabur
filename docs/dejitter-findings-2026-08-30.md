# Dejitter findings — vsync straddles, the class offset, and the display regulator

2026-08-30. Post-flight analysis of the pair-A/B recordings (`au-0028`
flat 50:50, `au-0029` asym 100:50 — same flights as
`pair-ab-field-findings-2026-08-30.md`) asking: what does the operator's
in-flight "jitter/stutter" actually consist of, and what removes it at
the least latency? Method: detrend the per-AU completion delay
(`t_us − pts` per segment, linear clock fit), then simulate the real
mailbox presenter against a 60 Hz vsync grid, phase-averaged (the
sensor and panel clocks free-run, so the phase sweeps in real flight).

## What the stutter is

- **~758 (0028) / ~942 (0029) repeat+skip pairs per minute** — a frame
  shown twice then a frame skipped, 13–16 per second. Near-continuous
  micro-judder, ~100× more frequent than flightjitter's >25 ms stutter
  *events*. A frame 2 ms late past a vsync boundary straddles just as
  visibly as a 20 ms one.
- **The largest single component is deterministic**: base completes
  6.4 ms (flat) / 9.1 ms (asym) later than enh in every pair — SVC-T
  reference frames simply carry more bytes, and the asym pair's extra
  base overhead added air on top. Equalizing only this class offset
  removes 33–47% of all repeats. (Independently confirms flat > asym.)
- Completion delay above floor: p50 12–15 ms, p90 23–31 ms, p99
  **78–97 ms**. The tail is rung-change sawtooth / loss recovery /
  holes — no display-side mechanism reaches it.
- Clock beat floor: the sensor runs 59.939 fps effective vs the 60.000
  panel → ~3.7 unavoidable slips/min. Irrelevant at current levels.

## Why a pts-anchored display regulator (and not TX pacing)

Simulated on the recorded arrivals, release at `floor(pts) + D`, late
frames shown immediately:

| scenario | 0028 rep/min | 0029 rep/min | added mean latency |
|---|---|---|---|
| today (mailbox) | 758 | 942 | — |
| D = 12 ms | ~419 | ~640 | +2.3 / +1.7 ms |
| D = 16 ms | 249 | 427 | +4.9 / +3.6 ms |
| class-eq + D = 16 | 205 | 289 | +4.7 / +3.4 ms |

The regulator absorbs the class offset automatically (the early class
just waits longer), so the last row's gain arrives without touching UEP
— and inverse overhead pairs are ruled out anyway (base keeps ≥ enh
protection, operator rule). TX-side pacing to the same schedule pays
identical latency *before* the radio, where fades stack on top, and
destroys catch-up slack: strictly worse. A D past ~16–20 ms buys little
— the tail is not reachable by any fixed D.

FEC interaction: none. The delivery walls (seq horizon ≈ 270 ms of
traffic at 10 Mb/s; FrameStream `frame_gap_timeout_ms` 50 ms — the real
deadline; row expiry deleted, see `wall2-deadline-findings-2026-08-30`)
all sit upstream of the AU ring; D ≈ 12–16 ms never waits on anything
they haven't already resolved. Related follow-up: below ~3.4 Mb/s pair
the 50 ms gap timeout clips the sliding window's reach-back
(`w_useful ≈ gap_timeout · stream_rate / (8·332)`) — a rate-aware
gap_timeout is queued separately.

## The implementation (maburplay `display.regulate_ms`, default 12)

`gs/player/src/frame_regulator.{h,cpp}` + wiring in `main.cpp`. Clock
map anchors on the fastest observed frame (min of `mono − pts64`,
snap-down instant, upward leak 60 ppm ≈ 3× measured drift); mailbox
discipline preserved (newer frame displaces held); late frames present
immediately; pts jump >2 s resets the map. The regulator is a third
holder of MPP buffers, so both `drop_all()` flush points flush it too.
Observability: `regulator:` exit line (held/late/replaced/disconts/
hold_ema) plus `present_jitter` — an |Δ present-interval| EMA printed
even when off, which is the A/B number. 0 = bit-exact old behavior.

**Bench A/B (live link, 90 s per leg, exit-line `present_jitter`):**

| regulate_ms | present_jitter | held / late / replaced |
|---|---|---|
| 0 (off) | 9.68 ms | — |
| **12 (shipped default)** | **2.94 ms (−70%)** | 3920 / 1384 / 5 |
| 16 | 5.32 ms | 5123 / 236 / **804** |

⚠ **The single-slot mailbox caps useful D at ~the burst spacing.** At
D=16 the hold outlives the gap to the next burst-decoded frame often
enough that the regulator's mailbox displaces ~15% of held frames
(`replaced=804`) — each a never-shown frame, i.e. self-inflicted judder
that more than cancels the smoothing. 12 ms sits under that knee
(replaced=5). If a future rung/scene profile wants D≥16, the regulator
needs a 2-deep release queue first, not a bigger number in the config.

Encoder-side class-size equalization (making base/enh payloads
comparable at the source) would shrink the needed D further; parked —
star6e RC size knobs are mostly dead (`airtime-model.md` §3).
