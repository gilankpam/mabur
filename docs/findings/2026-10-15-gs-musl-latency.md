# GS publish tail: musl-static maburgs costs ~2 ms per frame

**Status:** shipped — maburgs is built glibc-dynamic per board; deploy facts
in `docs/deploy.md` ("Device facts", GS).

## Question

Every AU waited ~3.3 ms on the GS between its last body arriving and its
record being published to the AU ring (the "GS tail"), on a clean link at
rung 4 (mcs4/40 MHz, 4 slices). Refresh-start frames waited longer. Where
does that time go, and what removes it?

## Setup

Bench Radxa ZERO 3 GS, two RTL8822E cards, the bench drone at close range,
60 fps, rung 4 for nearly the whole of every run. Per-AU split from
`au.log`'s `t_last`, `t_drain` and `t_push` columns (`aulog 6`, PR #84):

- queue = `t_drain − t_last`: wait in the RX queue before the core thread
  drained the batch holding the AU's completing body;
- batch/FEC = `t_push − t_drain`: work on earlier bodies of that batch plus
  FEC decode;
- emit = `t_complete − t_push`: FrameStream emit and the ring write.

Core-thread CPU came from a scratch stage profiler (never merged). A/B runs
were 90 s each, back to back in one session, with each run's rows cut from
the session `au.log` by its line count at start, because maburgs can open a
new session directory on restart mid-bench (0103 → 0104 here) and a
"last N rows" cut then silently reads an earlier run.

## Findings

**1. The core thread is the gate, during each frame's burst.** ~5 500
bodies/s reach it (both cards hear every body). The queue wait spreads from
0 to ~6 ms (p50 1.7, p90 4.2): bodies land while the thread is mid-iteration.
At similar frame size, the tail grows with how fast the frame's bodies
arrive — 2.85 ms at 6.8 kB per ms of span, 4.09 ms at 9.8 kB/ms — so the
backlog the thread builds during a burst is what a frame waits out.

**2. Cutting CPU that is off the burst path does not move the tail.** Two
changes freed real core time without changing the tail (P frames 3.24 /
3.38 ms baseline, 3.50 ms with both; run noise ±0.15 ms):

- skipping FEC repairs that can solve nothing (all of them on a clean link,
  ~35 µs each, ~21 % of the core thread) — repairs follow a frame's data, so
  the frame had already completed;
- building the sideport snapshot only when it is sent (it was rebuilt on
  each of ~290 wake-ups/s for a 5 Hz datagram).

Core-thread idle rose from ~24 % to ~34 %.

**3. The C library is the cause.** Same source and session, back to back:

| GS tail, median (p90) | musl-static | glibc-dynamic |
|---|---|---|
| P frames | 3.52 ms (5.75) | **1.33 ms** (3.22) |
| — queue | 1.73 | 0.68 |
| — batch/FEC | 1.46 | 0.53 |
| refresh-start frames | 3.71 ms (7.02) | **1.16 ms** (2.87) |
| core cost per body | 78.5 µs | 50.4 µs |

The per-body path (SBI unpack, the sliding-window decoder, FrameStream)
allocates and copies `std::vector`s per body and per fragment. A
microbenchmark of one such step on the GS (copy a 54 kB AU out of 42
fragments) ran 362 µs under musl-static and 114 µs under glibc; a pure
byte scan of the same AU (`split_nals`) ran the same on both. A ~1.6×
slower per-body cost becomes a ~2.6× longer tail because the thread runs
close to saturation during each burst. The fixed per-iteration work
(`lha.tick`, the gap/NACK block, `chan.tick`) does not get cheaper on glibc.

**4. Refresh-start frames also paid `feed_params`.** Before PR #84 their
emit was 2.0 ms against 0.04 ms for P frames: the whole AU was copied and
scanned to read the parameter sets in its first ~150 bytes. That PR reads
only the leading parameter sets; emit is now 0.05 ms for every frame type.

## Verification

The glibc build installed with `gs/bundle/install.sh` (board picked by
`/proc/device-tree/model`) passed `tools/bench/ausniff.py` for 30 s: 1 815
AUs at 60.5 fps, all complete, 0 incomplete, 0 resyncs, 0 oversize drops,
one frame-id gap.

## Not measured

- The Orange Pi Zero 2W build (`out/arm64/maburgs-opi`) is built and links
  the same libraries, but was not run on that board for this record.
- Whether an allocator swap under musl (mimalloc or similar) would match
  glibc; glibc was taken because the image repo already builds maburgs that
  way.
