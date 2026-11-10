# Repair window per frame: model and host A/B — 2026-11-08

**Status:** `docs/airtime-model.md` "Repairs at the frame end" owns the rule as built. Model, host A/B and one on-air bench A/B/A; the `tools/bench/fecab` loss-sim gate against current master has not been run.

## Question

The frame-tail rule holds all repairs to the frame end only for frames that fit `fec.window`. What does it cost and buy to size the window from the frame instead, so that it is always larger than the frame?

## Method

- `tools/bench/fecab/fecsim.cpp`: encoder policy as in `sw_encoder.cpp` (credit, `frame_hold_syms`, packing into bodies of 4), decoder as streaming GF(256) elimination, base and enhance alternating at 100 fps, overheads 0.5 / 0.25, frame sizes lognormal (sigma 0.35) with a 3x frame every 200 frames, capped at 310 symbols. One body costs 130 us on air and one window row 0.95 us to build (61–63 us per repair at 64 rows, measured in the frame-tail gate). No NACK. 240 000 frames per cell.
- `tools/bench/fecab/realab.cpp`: the same frame and loss model through the real `SwEncoder` and `SwDecoder`, no build cost. 120 000 frames per cell.

## Results

Truncated % / more than 10 ms late %. D is the frame-tail rule, C continuous placement at window 64, R is `min(128, max(64, n + 32))`.

Model, n = 88 symbols per frame (about 24 Mbps):

| Rule | 2 % single | 5 % bursts of 3 | 5 % flight mix | Rows per frame |
|---|---|---|---|---|
| C | 0 / 3.55 | 1.69 / 14.06 | 5.07 / 11.76 | 2176 |
| D | 0.06 / 0.54 | 3.17 / 11.65 | 6.09 / 9.94 | 2156 |
| `n+0` | 0.07 / 0.43 | 2.00 / 2.36 | 6.19 / 6.49 | 3425 |
| `n+32` | 0.06 / 0.59 | 0.80 / 2.66 | 3.63 / 6.87 | 4486 |
| R | 0.006 / 0.08 | 1.11 / 3.55 | 4.10 / 7.32 | 3915 |

Real codec against the model at n = 88, 5 % bursts of 3: D 3.20 / 10.97 (model 3.17 / 11.65), R 1.09 / 3.40 (model 1.11 / 3.55).

Real codec, D against R:

| n | 2 % single | 5 % bursts of 3 | 5 % flight mix |
|---|---|---|---|
| 30 | 0.005 / 0.24 vs 0.001 / 0.24 | 1.40 / 7.22 vs 1.05 / 7.14 | 3.03 / 6.34 vs 2.66 / 6.33 |
| 60 | 0.057 / 0.29 vs 0.001 / 0.04 | 2.92 / 7.24 vs 0.92 / 3.61 | 5.07 / 8.03 vs 3.65 / 7.13 |
| 88 | 0.048 / 0.58 vs 0.008 / 0.09 | 3.20 / 10.97 vs 1.09 / 3.40 | 5.92 / 9.89 vs 4.04 / 7.29 |
| 130 | 0.019 / 0.75 vs 0.028 / 0.34 | 3.17 / 13.55 vs 1.52 / 6.71 | 7.61 / 12.19 vs 4.30 / 8.79 |

## Frames only one rule loses

`tools/bench/fecab/paired.cpp`: the same frames and the same per-frame loss pattern through the real codec under both rules. 119 904 frames per cell, counted as lost by both / only the frame-tail rule / only R. "After a failure" is the share of the R-only losses whose previous frame of the same layer was also lost under R.

| n | Loss | Both | Only frame-tail | Only R | After a failure |
|---|---|---|---|---|---|
| 30 | 5 % bursts of 3 | 1121 | 493 | 38 | 28 |
| 60 | 5 % bursts of 3 | 886 | 2678 | 227 | 67 |
| 88 | 5 % bursts of 3 | 847 | 3019 | 461 | 48 |
| 88 | 10 % bursts of 3 | 5040 | 7688 | 1351 | 429 |
| 88 | 5 % flight mix | 4265 | 2857 | 599 | 140 |
| 130 | 2 % single | 1 | 9 | 38 | 0 |
| 130 | 5 % bursts of 3 | 811 | 2981 | 1030 | 37 |

R loses some frames the tail rule saves. Every repair of an R frame spans the same symbols, so an unrecovered loss in the reach into the previous frame costs one repair of the budget of this frame, and a loss past the budget leaves nothing partly solved. The 2 % row at n = 130 is the one cell where R loses more in total. A longer run of that cell (479 616 frames): 1 lost by both, 70 only by the frame-tail rule, 127 only by R, and all 127 are frames past the 128-symbol cap (109 of them enhance, whose tail has the fewest repairs). Keeping those frames at window 64 changes nothing (126 only-R losses), so the 128-symbol reach is not the cause. The likely cause, not proven: the next frame of the layer, if it fits under the cap, holds all its repairs and reaches back only 32 symbols, where the frame-tail rule would send it front repairs reaching back 64. Not pursued: 0.03 % of frames in the one cell, and R has fewer late frames there (0.34 % against 0.75 %).

## What the model says

- A window equal to the frame shrinks small frames' reach: at n = 30 and 5 % bursts of 3, `n+0` truncates 6.93 % against D's 1.46 %. Hence the floor.
- Reach past the frame pays: at n = 88, 5 % bursts of 3, truncation is 2.00 % at `n+0`, 1.34 % at `n+16`, 0.80 % at `n+32`, 0.23 % at `n+64`.
- Build cost is quadratic in the frame. Without the cap, at n = 130, 2 % loss: 8.7 % late at `n+16` against 0.69 % for D; at half the row cost that excess is gone, so it is the CPU and not the code. With the cap R adds about 1.7 ms of build per frame at n = 88 and 2.9 ms at n = 130, and peaks at 7.8 ms inline.
- Outages of 2–15 ms leave every rule at about 5–6 %: the batch goes with the frame.

## GS gap timeout

The mid-frame timer runs from the last advance of the contiguous prefix, and the window-based timeout only knows frames already seen. A frame of 65 to 128 symbols used to send front repairs; now all its repairs follow its last source, so a hole near its head waits for the frame to serialize. At MCS 0, 20 MHz and about 1500 symbols/s a 120-symbol frame after 25-symbol frames gets 57 ms against about 54 ms of send time (arithmetic, not a run). `FrameStream` therefore stretches the wait for a hole in one frame to the time left to send that frame at the estimated rate plus the policy margin, up to the policy cap, and never shortens it. A frame missing its first chunk has no header yet and counts its whole size.

## On air, bench

SSC338Q with a 8822E card, MCS 3, 20 MHz, 60 fps, encoder peak about 10.5 Mbps, real interference (2.5–3.4 % of received frames fail CRC), one ground station, NACK on. Three consecutive 60 s arms after a 40 s settle: the earlier build (PR #117's `repairs_with_frame = true`, window 64), this rule, the earlier build again. The earlier build is not current master, so this does not compare R with the frame-tail rule. The ground station was a separate implementation of the same wire, so the `gap_timeout_policy` change was not in this loop. Frames were 54–60 symbols on average, p95 about 97, and at most 0.4 % were past 128 symbols.

| | earlier | R | earlier |
|---|---|---|---|
| Frames delivered | 3150 | 3707 | 3334 |
| Incomplete | 2 | 0 | 2 |
| Delivery gaps > 100 ms | 12 | 0 | 8 |
| Longest gap, ms | 345 | 34 | 331 |
| NACK requests | 71 | 0 | 66 |
| Delivered Mbps | 7.5 | 9.6 | 8.4 |
| `mbr-hot`, % of a core | 14.3 | 22.6 | 14.7 |
| `maburd`, % of a core | 50.6 | 62.8 | 54.0 |
| `txq` peak | 149 | 21 | 62 |

`frame_end_us/rep` was 110–136 us, mean 122 (61–63 us at 64 rows in the frame-tail gate). One arm of R on one bench; an earlier run of R alone had 2 gaps over 100 ms and 12 NACK requests in 62 s, so zero is not a guarantee. The `mbr-hot` rise also includes the extra throughput the earlier build did not deliver.

## Not measured

The `tools/bench/fecab` loss-sim gate against the frame-tail rule on hardware; the worst inline batch at high rates, and whether it has to move to the worker (at 0.95 us per row a 128-symbol frame costs about 7.8 ms inline at overhead 0.5, 15.6 ms at overhead 1.0 and 31 ms at the overhead 2.0 of the first rung, against a 10 ms frame period; whether a frame that large occurs at overhead 2.0 is not established); decode cost on phones and in WASM; flight loss patterns.
