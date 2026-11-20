# Where maburd's CPU goes, and three cheap cuts — 2026-11-20

**Status:** the one-pass NAL scan (`scan_au`), slicing-by-8 CRC16 and the carry-less ARMv7 repair kernel are in the code; worker placement was measured and left as it is. Current: docs/airtime-model.md "4. Measuring it — rigs and gotchas"

Question: which part of `maburd` uses how much CPU on the drone, and what
is the most that can be cut for the least work?

**Answer: about 20 % of maburd's CPU at 17 Mb/s and 23 % at the top rate,
almost all of it from a new FEC repair kernel.**

## Method

Bench drone (SSC338Q, two Cortex-A7 at 800 MHz), 1080p60, link up to the
bench GS. Per-thread CPU from `/proc/<pid>/task/*/stat` over 60 s after a
60–75 s settle, divided by the elapsed time between the two snapshots (a
snapshot takes seconds on the A7; dividing by the nominal sleep overstated
everything by 1.4x on the first try). Function shares from
`tools/bench/pcsample`. Binaries ran from `/tmp` in alternating order.

The encoder's rate wanders between 21 and 24 Mb/s at the top rung on a
static bench scene, so arms are compared as CPU per packet sent (`sent=`
delta over the window), and a second set of arms ran with
`bitrate_max_kbps = 18000`, where the rate holds at ~17 Mb/s.

## Where it goes (baseline, top rate)

maburd is 100–110 % of one core. Core 0 carries everything but `mbr-hot`
and is close to full; core 1 is about 75 % idle.

| thread | % of a core | mostly |
|---|---|---|
| `mbr-fecw` | 29–42 | 87 % `gf::lincomb_rows` |
| `mbr-hot` | 21–29 | CRC16 23 %, `lincomb_rows` 16 %, two whole-frame NAL scans 25 %, memcpy 10 % |
| `mbr-usb` x4 | 22 | half kernel (one USB submit per 2–3 bodies) |
| `venc-enc` | 8 | kernel (MI ioctls) |
| main | 6.5 | libusb completions |
| `mbr-txw` | 5.6 | memcpy, malloc |

Which thread pays for FEC depends on the rate. Up to ~18 Mb/s a frame fits
its repair window, every repair is built on `mbr-hot` at the frame end
(`jobs=0`), and repairs are 65 % of that thread. Above it the async worker
takes most of them, and on core 0 it is starved: 133 us of CPU and ~300 us
of wall time per repair against 92 us alone, `qdepth_max=256`, and
`backlog_drops` up to 713 per stats period in one window.

## The cuts

Microbench on the drone, maburd stopped:

| | before | after |
|---|---|---|
| NAL scans, 50 kB AU (`classify_frame` + `frame_is_trail_n` + `count_au_nals` vs `scan_au`) | 823 us | 234 us |
| CRC16, 332 B blocks (one table vs slicing-by-8) | 10.2 ns/B | 5.1 ns/B |
| repair, 128 rows x 336 B (nibble-table `vtbl` vs `vmull_p8` with one reduction per block) | 92.1 us | 52.9 us |

The repair kernel was the surprise. The table kernel was already at the
A7's limit for a nibble-table multiply (1.55 cycles/B at 32 rows, 1.71 at
128), and the larger window costs only that 10 %: prefetching and row
tiling bought ~5 %, two repairs per pass ~16 %. The carry-less form does
half the work per row (0.98 cycles/B) and needs no tables.
`tools/bench/gf_bench.cpp` verifies it byte-for-byte against the scalar
reference on the drone; the host suite does not compile that path.

In situ, CPU per packet sent, two alternating runs per binary:

| point | binary | `mbr-hot` | maburd | hot time per frame |
|---|---|---|---|---|
| ~17 Mb/s | baseline | 35 % of a core | 72–74 % | 5.8 ms |
| | + NAL + CRC | −9 % | −4 % | 5.4 ms |
| | + NAL + CRC + kernel | −35 % | −19.5 % | 3.8 ms |
| top rate | baseline | 21–29 % | 103–109 % | 3.2–5.3 ms |
| | + NAL + CRC + kernel | 16.5–17 % | 81–82 % (−23 %) | 2.5–2.6 ms |

## Worker placement (measured, not changed)

With all three cuts, the worker moved off core 0 through a spike-only
switch, one run each at the top rate:

| worker | maburd per packet | wall us/repair | `qdepth_max` |
|---|---|---|---|
| core 0 (as shipped) | 1.00 | 190–224 | 70–100 |
| core 1, with `mbr-hot` | 1.03 | 77 | 40 |
| both cores | 1.02 | 79–88 | 90 |

No CPU saved, but the worker stops waiting behind the TX and USB threads,
and `mbr-hot`'s per-frame time was no worse. Not adopted here: it needs a
GS-side latency A/B.

## Gate

The binary built from the three cuts, run against the bench GS at the top
rate: `ausniff.py` 2 x 30 s read 1815 AUs each, 0 frame-id gaps, 0 resyncs,
0 incomplete, 60.5 fps (the deployed binary just before: 1816, 0, 0, 0,
60.5). `gf_bench` `verify` and `verify_rows` OK on the drone. No
`backlog_drops` in a 150 s run.

## Not done

USB aggregation (`mbr-usb` + main are ~28 % of a core for ~1150 URBs/s),
the remaining memcpy/malloc in the feed path, and a long top-rate run to
count `backlog_drops` with the new kernel.
