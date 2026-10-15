# Capture-time frame scheduling: judder gone, median +10 ms

**Status:** built, off by default (`display.sched_cap_ms = 0`); behaviour
and fields in `docs/observability.md` ("Regulator line", `alw=`/`lslot=`).

## Question

On a clean link the OSD LAT row alternates between "40/41" (smooth) and
"40/60" (visible judder) over a ~2-minute cycle. What causes the judder,
and does placing frames by capture time instead of arrival remove it?

## Mechanism

The sensor (~59.99 Hz after the rate trim) and the panel (60.000 Hz) beat
with a ~2-minute period, so the moment a typical frame becomes ready to
display slides steadily across the refresh period. The servo regulator
gave each frame the first vblank after it was ready. Whenever the margin
between a typical frame and the latch was smaller than the late tail
(~8 ms at arrival, ~10 ms at decode-ready: GOP-start and large base frames,
plus decode), the late ~1 % missed the latch: the previous frame repeated,
the following frames chained one vblank late, and after `chain_budget`
frames one was dropped. That is the judder, and it lasts for the fraction
of the beat cycle where margin < tail, about tail / period of the time.

Session 0105 (flight, glibc maburgs, rung 4): 43 % of clean seconds
juddered, and those seconds put 58 frames per second on screen instead of
60.

## A/B

Bench Radxa GS, rung 4, session 0106, the same player binary with
`display.sched_cap_ms` 0 and 16, 259 s each (about two beat cycles), back
to back. Seconds within 3 s of a rung change excluded.

| | cap 0 (arrival) | cap 16 (capture time) |
|---|---|---|
| seconds with p99 ≥ p50 + 10 | 152 / 252 (60 %) | 28 / 253 (11 %) |
| frames on screen per second | 58.75 (min 54) | 60.01 (min 56) |
| `replaced` (dropped) | 387 | 26 |
| `cuts` (chain cuts) | 366 | 0 |
| e2e p50, mean (range) | 35.9 ms (27–45) | 46.0 ms (37–55) |
| e2e p99, mean | 46.3 ms | 47.9 ms |

The allowance (`alw`) settled at 15.7 ms, at the cap: lateness is measured
at decode-ready, so it includes decode time, whose spread (`dec` p50 ~4,
p99 ~10 ms) is about as large as the arrival tail. Nearly every frame
therefore took the later slot, which is the +10 ms on the median.

## What smooth costs, by any method

Binning the cap-0 arm by e2e p50 shows the judder stopping at ~40–42 ms
(≤ 13 % of seconds) and constant below 36 ms. Smooth display needs each
frame to leave room for the slowest ~1 %, about p50 + 10 ms here, whatever
the scheduling:

- arrival scheduling averages 36 ms and judders for most of the cycle;
- capture-time scheduling averages 46 ms: the margin plus the beat's
  0–16.7 ms wait for a refresh, which it does not remove;
- a phase lock (drone sensor rate steered to hold the arrival point just
  ahead of the latch) would sit at ~41 ms, the margin alone.

Every option moves with the ~10 ms margin, so shrinking it — the
one-slice GOP-start frame's slower encode and decode, the remaining GS
tail — lowers the cost of smoothness for all of them.

## Not measured

- Intermediate caps (e.g. 8–10 ms).
- Behaviour under real loss in flight; the loss path is covered only by
  the host simulations in `tests/test_frame_regulator.cpp`.
