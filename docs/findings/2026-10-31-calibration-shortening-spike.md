# Calibration shortening spike — how few frames, how short a dwell

**Status:** design input for the per-width calibration revamp (walls and
capacity over MAVLink); the mechanism it informs is described in
`docs/calibration.md` once built. Nothing built here.

Question: the revamped calibration runs four sweeps (walls and capacity,
each at 20 and 40 MHz) instead of today's one. Today's walls run is 72 s
at 20 MHz alone (three phases: coarse 20 frames per cell on a step-4
grid, fine 100 frames per cell, verify). How far can one sweep be
shortened and still reproduce the walls, and how long must a saturation
dwell be for a stable delivered-capacity number?

## Method

Bench pair, both daemons stopped, ch 144 (the pair 140+144 at 40 MHz),
close bench geometry. Drone: `linkbench-tx --wall-sweep` (the maburcal
stamp, `mabur/cal_wire.h` 64-byte payloads, singles), `--ldpc --stbc`
(the flags every production rung flies — today's maburcal sweeps with
both off), `--pwr-mode none`, anchor read with the rate diffs zeroed
(39 at both widths, as in September). GS: two `linkbench-rx --wall`, one
per card (`--index 0/1`; `--card` alone is only a label and both
processes open the same adapter). Every rel index −41..63 at full
resolution, so a coarse grid can be simulated offline from the same
data. Wall rule computed offline exactly as `gs/src/cal_analysis.cpp`:
best single card, end of the first contiguous ≥90 % run, `no_dip` when
the run reaches the top cell. mcs0-3 were `no_dip` (63) on every run at
both widths and are left out of the tables.

## Walls vs frames per cell and settle, 20 MHz

| run | frames/cell | settle ms | gap µs | sweep s | mcs4 | mcs5 | mcs6 | mcs7 |
|---|---|---|---|---|---|---|---|---|
| reference a | 100 | 20 | 1000 | 101 | 45 | 23 | 22 | 21 |
| reference b | 100 | 20 | 1000 | 101 | 45 | 23 | 23 | 20 |
| 20 / 20 | 20 | 20 | 1000 | 34 | 45 | 24 | 23 | 21 |
| 20 / 5 | 20 | 5 | 1000 | 21 | 46 | 23 | 23 | 21 |
| 10 / 2 | 10 | 2 | 1000 | 10 | 45 | 22 | 23 | 21 |
| 10 / 0 | 10 | 0 | 1000 | 8.4 | 45 | 22 | 23 | 21 |
| 20 / 5, gap 500 | 20 | 5 | 500 | 12.6 | **41** | 24 | 22 | 20 |

## Walls, 40 MHz

| run | frames/cell | settle ms | sweep s | mcs4 | mcs5 | mcs6 | mcs7 |
|---|---|---|---|---|---|---|---|
| reference a | 100 | 20 | 101 | 45 | 28 | 28 | 23 |
| reference b | 100 | 20 | 101 | 46 | 27 | 28 | 23 |
| 20 / 5 | 20 | 5 | 21 | 45 | 28 | 27 | 23 |
| 10 / 2 | 10 | 2 | 10 | 46 | 27 | **25** | 23 |

Findings:

1. **Run-to-run noise of the 100-frame reference is 0-1 index** at both
   widths (two interleaved references each).
2. **20 frames per cell with a 5 ms settle reproduces the reference
   within 1 index at both widths, in 21 s per width, in one pass.**
   Simulating maburcal's coarse step 4 + fine ±8, or step 8 + fine ±8,
   from the reference data gives the same walls as full resolution — the
   phase split buys nothing at this frame count.
3. 10 frames / 2 ms (10 s) is within 1 index at 20 MHz but 3 off on one
   40 MHz row (mcs6 25 vs 28). Settle 0 ms changes nothing at 10 frames.
4. A 500 µs gap lost 4 indices on mcs4 (41 vs 45): one noisy cell ended
   the ≥90 % run — the known one-bad-cell defect (`docs/calibration.md`).
   Keep 1 ms.
5. The walls under LDPC+STBC ([63,63,63,63,45,23,22,21] at 20 MHz,
   [63,63,63,63,45,28,28,23] at 40) sit well above the bench drone's
   current flags-off table ([63,63,63,40,25,12,12,8]) and above each
   other by width on mcs5-7, as `docs/findings/2026-09-23-bw40-sweep.md`
   found. `saturated` fired on some 20 MHz rows: card 1's peak RSSI read
   −45.0 (the readout rail) at this close geometry.

## Capacity dwell length

`linkbench-tx` at an offered load far above capacity (100 Mb/s app,
production FEC geometry 332/4/32/0.5, `--tx-threads 4`, LDPC+STBC, the
production aggregation mode per MCS: singles below `ampdu.min_mcs_<bw>`,
agg6 at and above) for 15 s; `linkbench-rx --json` per-second clean air
(air bytes × (1 − crc_bad/frames)).

| point | steady median Mb/s | spread (stdev) | 1 s dwell err | 2 s dwell err |
|---|---|---|---|---|
| 20 MHz mcs0 singles | 6.04 | 0.2 % | 0.0 % | 0.0 % |
| 20 MHz mcs2 singles | 16.20 | 0.3 % | +0.3 % | +0.1 % |
| 20 MHz mcs4 agg6 | 28.41 | 0.3 % | −0.3 % | −0.1 % |
| 20 MHz mcs7 agg6 | 39.80 | 0.5 % | +0.1 % | +0.1 % |
| 40 MHz mcs0 singles | 11.75 | 0.2 % | +0.4 % | +0.1 % |
| 40 MHz mcs2 agg6 | 29.04 | 0.2 % | 0.0 % | −0.1 % |
| 40 MHz mcs5 agg6 | 53.79 | 0.2 % | −0.1 % | −0.1 % |

6. **The delivered rate is flat from the first full second**; only the
   partial start second ramps. A 2 s dwell per (MCS, width) is within
   0.4 % of a 15 s one; sixteen points take ~40 s.
7. **Caveat — the high points were not air-bound.** mcs7 at 20 MHz read
   39.8 Mb/s (0.61 of nominal) and mcs5 at 40 MHz 53.8 (0.50), far below
   the September bench (49.4 / 81), while mcs0-4 match it. `linkbench-tx`
   builds FEC on the drone's CPU and reported `under target` every second
   at every point, so the top points measured the drone's TX path, not the
   air. A capacity sweep inside the kit must send pre-built frames and
   detect a point whose sender was not blocked by the air, or the table
   records the drone's CPU.

## What this fixes in the design

- Walls: one full-resolution pass, 20 frames per cell, 5 ms settle, 1 ms
  gap — ~21 s per width, no coarse/fine phases, so one ack round and one
  slack tail per sweep.
- Capacity: 2 s per (MCS, width) at the parked power, 16 points, ~40 s,
  with a not-air-bound flag.
- A four-sweep run is under two minutes of air versus 72 s for today's
  walls-only 20 MHz run.
