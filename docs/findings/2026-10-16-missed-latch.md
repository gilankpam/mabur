# The missed latch: a frame released on time that still flips a vblank late

**Status:** fixed — the presenter keeps each pool buffer's imported GEM
handle and framebuffer across flips (`gs/player/src/fb_cache.h`); the
per-frame `t_commit`/`pbits` columns (`frame.log`, `framelog 3`) and the
`reg.log` present() stage split that found it are described in
`docs/observability.md`.

## Question

With `display.chain_budget = 0` shipped
(`docs/findings/2026-10-15-regulator-chain-cuts.md`), flights 0122–0124 and
the ground session 0121 still showed 18–29 hitches a minute and 19–30 %
judder seconds. Classifying every flip gap by its context (loss episode or
AU hole, ladder event, refresh-start frame late, other late arrival) left
one class that fit none of them and was the largest single one on a clean
link: a frame with an ordinary ready time, released at the normal
`vsync_lead_ms` (5.7–6.0 ms before a real vblank, the same lead as every
frame that latched), whose flip stamp landed one vblank later (`dsp`
22.4 ms instead of 5.9), followed by the next frame being dropped. What
happened between the release and the vblank?

## The pattern in the flight logs

| session | hitches/min | missed latch | loss / AU hole | ladder | refresh frame late |
|---|---|---|---|---|---|
| 0121 (ground, 240 s) | 22.5 | 19.7 | 0.2 | 0.2 | 0.7 |
| 0122 | 29.1 | 7.9 | 16.4 | 1.9 | 2.2 |
| 0123 | 18.2 | 4.9 | 3.5 | 3.7 | 5.4 |
| 0124 | 18.4 | 5.6 | 3.1 | 2.3 | 5.4 |

- The missed frame is ordinary: size, decode time, layer all at the
  median. Frames released with only 4 ms of lead still latched, so the
  lead itself was not short.
- After the miss the frame behind it is released while the late flip is
  still in flight, parks in the presenter mailbox and is dropped at flip
  completion (paced mode): 66 of 79, 43 of 47, 33 of 38 and 23 of 34
  misses were followed by that drop. On screen: a repeat and a skip.
- Not new: sessions 0107, 0109 and 0110 carry it at 0.09–0.21/s; in 0110
  (the `chain_budget = 0` bench) it was 48 of 62 hitches, hidden under
  the chain cuts before that.
- Not the 1 Hz log burst (misses sit nowhere near the `lat.log` stamps),
  and not the OSD: frames whose commit carried the OSD plane missed at
  the same rate as the rest.
- On the ground session the misses sat at the top of the e2e sawtooth
  (regulator hold 32–36 ms: 18 per 1000 frames against under 6
  elsewhere); in flight the rate was flat at 1–2 per 1000.

Two correlations computed from `frame.log` alone (regulator queue depth
at release, a decode landing inside the lead window) are artifacts: the
dropped frame has no row, so every miss reads as "nothing else there".

## Instrument

`frame.log` grew `t_commit` (when `drmModeAtomicCommit` returned) and
`pbits` (OSD attached, ASYNC, parked, failed, modeset), and the 1 Hz
`regulator:` line went into the session directory as `reg.log` with a
per-stage split of present(): dma-buf import (`drmPrimeFDToHandle`),
`drmModeAddFB2`, request build, commit ioctl. Bench Radxa ZERO 3, rung 4,
session 0125, the drone at `gop_s 0.5`, chain budget 0, lead 6 ms.
`PAGE_FLIP_ASYNC` is rejected on this kernel, so every commit is
vsync-paced.

## What the split showed (session 0125, first 1055 s)

| | hits (62 579) | misses (89) |
|---|---|---|
| release → commit returned, p50 / p90 / p99 / max | 2.60 / 3.71 / 4.39 / 5.55 ms | 6.73 / 7.54 / 8.72 / 8.72 ms |
| commit returned → flip | 3.3 ms p50 | 15.7 ms p50 |

Every miss spent more than the lead inside present(); the kernel then
latched it at the next vblank with its usual ~3 ms. Per stage, across
165 one-second windows:

| stage | p50 | typical max | worst max |
|---|---|---|---|
| `drmPrimeFDToHandle` | 2586 µs | 3952 µs | 7399 µs |
| `drmModeAddFB2` | 46 | 121 | 445 |
| request build | 26 | 272 | 1064 |
| commit ioctl | 199 | 564 | 1200 |

In all 14 seconds with a mailbox drop the import's max was 5.5–7.4 ms,
and it held the max every time. The presenter closed the GEM handle after
every flip, so each frame re-imported its dma-buf: a dma-buf attach plus
an IOMMU map of the whole picture, inside the release lead, on every
frame.

## Fix and A/B (same session, back to back)

`FbCache` keeps the imported handle and its framebuffer per pool buffer
(LRU of 32, slots on screen or pending never evicted, flushed on
`drop_all()` and at teardown). While the handle stays open the kernel's
second import of the same dma-buf is a lookup returning the existing
handle, so the handle is the cache key.

| | before (715 s) | after (339 s) |
|---|---|---|
| present(): release → commit returned, p50 / p99 | 2.60 / 4.39 ms | 0.28 / 1.21 ms |
| acquire stage p50 / typical max | 2586 / 3952 µs | 26 / 58 µs |
| missed latches | 88 (7.4/min) | 1 (0.2/min; the pool's first imports at startup) |
| mailbox drops | 1 per miss | 1 |
| judder seconds | 14 % | 11 % |
| refresh-start frame late | 1.2/min | 5.0/min |

What remains on the bench is the refresh-start (GOP) frame landing past
the release point, the +10 ms margin of
`docs/findings/2026-10-15-regulator-chain-cuts.md`; its rate differs
between the two windows with the beat phase they covered, not with the
cache.

## Conclusions

- The clean-link repeat+skip that survived `chain_budget = 0` was the
  presenter's own dma-buf import, 2.6 ms typical and 6–8 ms in the tail,
  overrunning a 6 ms lead. Caching the import removes it.
- With the import cached the present() path costs ~0.3 ms, so the lead
  now has ~5 ms of headroom; `vsync_lead_ms` could come down, which would
  take latency off every frame — a separate measurement.
- `frame.log`'s `t_commit − t_release` is the regression gate for this:
  on a healthy GS it reads well under 1 ms at p99.
