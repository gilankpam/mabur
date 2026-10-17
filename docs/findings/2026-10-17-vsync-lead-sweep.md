# How much of the vsync lead is still needed once the present path is cached?

**Status:** `display.vsync_lead_ms` ships 5 (was 6); the budget it covers is
stated in `gs/player/src/player_config.h` and the per-frame stamps that
measure it in `docs/observability.md` (`frame.log` `t_commit`, `reg.log`).

## Question

`docs/findings/2026-10-16-missed-latch.md` cut the present path from 2.6 ms
to 0.3 ms. The regulator releases each frame `vsync_lead_ms` before its
vblank, and that lead sits in every frame's glass latency. How low can it
go before frames start missing their latch, and what sets the floor?

## Setup

Bench Radxa ZERO 3, rung 4, static scene, the framebuffer-cache build,
chain budget 0, session 0127. One maburplay restart per lead (6, 5, 4, 3,
2), 240 s captured after the link settled; `frame.log` gives each frame's
release, commit-returned and flip stamps.

## Result

| lead ms | e2e p50 / p99 | commit → flip p0.1 / p1 | missed latches | per minute |
|---|---|---|---|---|
| 6 | 46.4 / 55.3 | 4.0 / 4.6 ms | 0 | 0 |
| 5 | 45.4 / 54.1 | 3.2 / 3.6 | 0 | 0 |
| 4 | 44.7 / 53.8 | 2.2 / 2.6 | 1 | 0.25 |
| 3 | 43.3 / 52.2 | 1.3 / 1.6 | 2 | 0.5 |
| 2 | 42.2 / 51.2 | 0.4 / 0.6 | 19 | 4.75 |

Each millisecond of lead is a millisecond of median latency. The floor is
three terms:

- **Kernel latch deadline, ~1 ms.** A commit landing 0.4 ms before the
  vblank still latched, but all 19 misses at lead 2 had their commit
  between 1.3 ms after and 0.95 ms before the vblank.
- **Release lateness.** The main loop fires the release late by 0.11 ms
  at p50 but 1.1 ms at p99 and 1.45 ms at p99.9 on the bench: it services
  the AU feed before checking the deadline, and `wait_until_polling`
  steps in 0.5 ms slices.
- **Present path**, 0.3 ms p50 / 0.8 ms p99 with the cache, with rare
  spikes (the one miss at lead 4 was a 7 ms framebuffer acquire; under GS
  recording the max was 4 ms).

## Flight projection

The release lateness is the term a flight changes: the GS is busier with
47 kB frames, FEC repair and NACK traffic. From the latched frames of
flights 0122–0124 and the recording A/B of session 0126, frames with less
than 1.8 ms (present p99 + kernel deadline) left at a given lead:

| session | lateness p99 / p99.9 / max | at risk per minute at lead 5 / 4 / 3 |
|---|---|---|
| bench, lead-6 arm | 1.12 / 1.45 / 3.69 ms | 0.25 / 0.75 / 15 |
| bench, GS recording | 1.44 / 2.20 / 4.33 | 0.7 / 3.9 / 78 |
| flight 0122 | 1.26 / 1.86 / 3.40 | 0.17 / 1.9 / 50 |
| flight 0123 | 1.25 / 2.08 / 3.06 | 0 / 2.6 / 45 |
| flight 0124 | 1.27 / 2.07 / 3.23 | 0.16 / 3.0 / 54 |

The projection is conservative by about 3× (bench lead 4: 0.75 projected,
0.25 measured). Lead 5 stays clear in flight and under recording; 4 costs
about a missed latch a minute in flight; 3 is not flyable with the loop as
it is.

## Conclusions

- Ship 5: 1 ms off every frame, no misses in any measured condition.
- The next millisecond or two is the main loop's release lateness, not the
  kernel: checking the deadline around feed service (or a finer wait step)
  would pull the flight tail back toward the ground-session 1.3 ms and
  make 3–4 safe. A separate change; re-sweep after it.
