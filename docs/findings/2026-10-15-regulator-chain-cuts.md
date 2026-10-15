# The ready-time margin per frame, and why the regulator's chain cut is the clean-link judder

**Status:** `display.chain_budget` ships 0 and maburplay polls the decoder
through the pre-release wait (`gs/player/src/release_wait.h`); both are
described in `docs/observability.md` ("Regulator line", `frame.log`). The
remaining lever for latency is a sensor/panel phase lock, not built.

## Question

`docs/findings/2026-10-15-capture-sched.md` measured that a smooth display
costs about p50 + 10 ms by any scheduling method, because the slowest 1 %
of frames reach decode-ready that much later than the median. What is that
10 ms made of per frame, can it be shrunk, and what exactly happens on the
screen when it is not covered?

## Instrument

maburplay now writes `frame.log` (`# framelog 1`) beside `lat.log`: one
row per frame that completed the chain, with the AU meta, every absolute
stamp (submit, complete, decoded, release, flip) and the segment tuple
the `lat:` percentiles are ranked on. Joined to `au.log` on `pts`, the
refresh-start AU is the one whose first NAL is the VPS (`nal0 = 32`).

All runs: bench Radxa, rung 4 (mcs4 / 40 MHz / 0.50 overhead), the drone
at `gop_s 0.5`, `intra_refresh_frames 1`, `intra_refresh_qp 40`, 4 row
slices; ~260 s per arm (two beat cycles); A/B arms back to back in one
session, rows cut by stamp.

## The margin per frame (session 0107, 16 744 frames)

ready = enc + dq + air + fec + dec, i.e. capture → decoded, relative to
the anchor floor: p50 20.1, p90 23.8, p99 30.6 ms — a 10.6 ms margin.

| class | share | ready p50 / p90 / p99 | enc | dec after submit |
|---|---|---|---|---|
| base / enh | 93 % | 20.3 / 23.6 / 26.5 | 8.0 | 4.2 |
| refresh-start | 3.4 % | 29.1 / 32.2 / 37.1 | 13.2 | 6.6 |
| frame after it | 3.4 % | 20.3 / 23.7 / 28.4 | 7.9 | 4.2 |

The refresh-start frame is 83–88 % of everything above p50 + 8 ms: encode
+5.2 ms (all-intra rows), arrival span +2.7 ms (55.6 vs 35 kB; 3× a P
frame on the static bench at low light), decode +2.1 ms (one slice, so no
streamed decode). The frame behind it is not delayed. The encoder-side
levers stay closed: a multi-frame refresh does not heal under SVC-T 1:1
(`docs/findings/2026-10-09-venc-slice.md`), the SDK will not slice the
refresh picture, and an IDR is all-intra too.

## A blind window in the render loop (found, fixed)

Ordinary frames' decode time was bimodal — modes at 3–4 and 7–8 ms, 17 %
of frames in the slow one — at identical sizes and feed-thread timing.
`t_decoded` only ever landed at vblank phase 6–8 or 12–16 ms, never 8–12,
and frames completing at phase 4–8 ms were ~50 % slow, with the slow
share sweeping 2 → 62 % over the beat. The loop collects decoded frames
only from `backend->poll()`, and its micro-sleep into the regulator's
release point (vblank − 6 ms lead, up to 2.5 ms) ran without one, so a
picture finishing there waited for the release and the commit.

`wait_until_polling()` sleeps in 0.5 ms slices and polls before each.
A/B, session 0109, same light (base frames 25.7 kB):

| | blind sleep | polling wait |
|---|---|---|
| non-GOP dec p90 / p99 | 7.9 / 10.0 | **5.9 / 7.4** |
| slow-mode share | 17 % | 3 % |
| ready p50 / e2e p50 | 18.7 / 35.4 | **17.3 / 33.9** |
| judder seconds | 59 % | **73 %** |
| regulator replaced / cuts | 400 / 397 | 656 / 641 |

The fix is right and judder got worse, which is the next finding.

## What the judder is (session 0109, both arms)

Never-shown frames (in `au.log`, absent from `frame.log`) were ordinary
base/enh frames arriving at normal spacing, and `cuts ≈ replaced` in both
arms: nearly every drop was the regulator's chain-budget cut. Mechanism,
per beat cycle:

1. Through the part of the cycle where arrivals sit just before the
   release point, the refresh-start frame (+9–10 ms) lands past it and
   is shown one refresh late — 49–54 % of refresh frames, about 1/s.
2. Its successors then find their natural slot held and chain; after
   `chain_budget = 3` chained frames the fourth takes its natural slot and
   the held one is dropped. One refresh miss costs a repeat and a drop.

92 % of the short-hold seconds had a cut, 86 % a refresh miss. Offering
frames 1.4–4 ms earlier (the fix) widens that phase (60 → 70 % of
seconds) and raises the refresh frame's excess over the faster median
(9.1 → 10.3 ms), hence more cuts.

## `chain_budget = 0` (session 0110, same binary)

| | budget 3 | **budget 0** |
|---|---|---|
| judder seconds | 73 % | **14 %** |
| dropped frames | 2.56 /s | **0.16 /s** |
| refresh frames shown late | 54 % | **2 %** |
| frames on screen | 57.4 /s | **59.8 /s** |
| e2e p50 | 33.9 ms | 46.4 ms (+12.5) |
| regulator | cuts 641 | cuts 0, chained 12 822 |

With no cut the stream runs one slot late through the straddle phase, the
refresh frame's +10 ms fits inside that slack, and the display is smooth;
the cost is the +12.5 ms median and the 38 → 53 ms sawtooth over the
beat. That is the same bargain as capture-time scheduling at cap 16
(46.0 ms, 11 %): at arrival scheduling on a period-quantized display, a
smooth stream carries about a period of slack because the refresh frame
needs ~10 ms of it and the beat sweeps where that slack sits.

## Flights (sessions 0112 / 0113 / 0114, budget 0, polling wait)

No recording, GS recording, GS + VTX recording; 5–6 min each, rungs 0–4,
250–300 FEC loss episodes and 70–112 ladder events per flight.

| | 0112 | 0113 | 0114 (VTX rec) |
|---|---|---|---|
| judder seconds | 22 % | 20 % | 32 % |
| dropped frames | 0.41 /s | 0.45 /s | 0.83 /s |
| refresh frames late | 4 % | 3 % | 4 % |
| frames on screen | 59.5 /s | 59.5 /s | 58.9 /s |
| e2e p50 | 46.5 | 46.9 | 46.9 |
| ready p50 / p99 | 21 / 33 | 21 / 33 | 22 / 54 |

About half the judder seconds lie within a second of a loss episode;
seconds with no loss and no ladder event judder 13–16 % (the flight
arrival jitter: base frames 47 kB, arrival-span p99 ~16 ms, plus the
beat wrap). GS recording costs nothing. VTX recording doubles the drops
and pushes the base arrival p99 from 16 to 27 ms (the drone-side
recorder effect, `docs/vtx-recorder.md`).

## Conclusions

- The ~10 ms margin is the refresh-start frame; it cannot be shrunk at
  the encoder with the current SVC-T refresh, and the GS tail (1.3 ms)
  and the decode blind window (fixed) were the only GS-side pieces.
- The visible clean-link judder at arrival scheduling was mostly the cut
  policy: a latency-first rule that re-aligns by dropping. Budget 0 ships.
- Smooth at less than +12 ms needs the arrival phase pinned — a phase
  lock steering the drone's sensor rate from the GS-measured phase, with
  ~10 ms of refresh-frame margin, would sit at roughly the sawtooth's
  bottom + 10 ms with no sawtooth and no cuts. Not built.
