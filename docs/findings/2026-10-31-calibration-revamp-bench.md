# Calibration revamp — bench gates

**Status:** acceptance run of the per-width walls + capacity calibration over
the MAVLink tunnel; the mechanism is described in `docs/calibration.md`
(as built, including the two fixes this bench produced).

Question: does the revamped `maburcal` (four sweeps — power and capacity at
20 and 40 MHz — over the mabur MAVLink dialect, `RC_VERSION` 18) pass the
hardware acceptance gates on the bench pair, and do its numbers agree with
the 2026-10-31 shortening spike?

Setup: bench drone (8812EU-class card, LDPC on, `wall_margin_db` 1.0) and
the Radxa GS (two cards, `radio.width = 40`, channel set [64, 112, 144], op
112), close bench geometry. Flag day done config-first: the drone's
`rate_walls_rel` renamed to `rate_walls_rel_20` with `cal_flags_20 =
"none"`, then the new `maburd`, then `maburgs` + `maburcal` + `maburtop`.
The drone's new `maburd` ran from tmpfs over a bind mount (its flash was
93 % full); the GS took the build through `gs/bundle/install.sh`.

## Gates

| gate | result |
|---|---|
| ausniff before / after | 61.1 fps, 0 frame-id gaps / 61.0 fps, 0 gaps |
| full run (bare `maburcal start`, GS at 40) | four sweeps `power20,cap20,power40,cap40`, `done` in 82.4 s (first build) and 82.9 s (with the fixes below); pick `auto: 112 is the best member` |
| walls | see "Walls" — verify 100 % at every parked rate in both full runs |
| capacity | see "Capacity" — first build over-read; fixed; low-MCS singles then within ±0.03 of the spike |
| subset `maburcal start power20` | one sweep, `done` in 23.6 s; the `_40` lines byte-identical before/after; 20 MHz block only |
| 20 MHz GS (`radio.width = 20`, every rung bw 20) | `start power40` and `start power20 cap20 power40` → `err refused: GS radio.width is 20`, nothing started; bare `start` → `ok started power20,cap20`, 20 MHz block only |
| table swap | drone log alternates `power table 20 MHz live (mcs5 diff 23)` / `power table 40 MHz live (mcs5 diff 26)` with the rung width; with no `_40` table at boot it logged `no 40 MHz walls, flying the 20 MHz table at 40` and `power table 40 MHz live … (20 MHz table)` |
| config | `/etc/mabur.toml` vs `.pre-cal`: only `rate_walls_rel_20/_40`, `cal_flags_20/_40`, `air_clock.efficiency_20/_40` changed (`power_mode` was already `offset`); comments intact |
| interruption (kill `maburgs` mid power20) | video back 33.2 s after the kill (the sweep's remaining ~18 s plus the 15 s await window), config byte-identical |
| abort (`maburcal abort` mid power20) | AUs flowing in the first 1 s window after the abort; config untouched |
| re-link | SESSION again after every run |
| tunnel lane | first build: the drone's `mav … reject=` grew by 2 at every run's end; fixed (below): `reject=0 cal_late=2` over a full run |
| parameter set during a run | no window: the drone does not link during a run, so `mavparam.py set` saw no HEARTBEAT until `done` and was ACCEPTED after it. The `calibration in progress` refusal stays covered by host tests only |
| dirty `op` relocate | not run — needs a jammer on `op`, and both GS cards are in use |
| boot notes | `radio: no 40 MHz walls, flying the 20 MHz table at 40` and `radio: 20 MHz walls were measured with ldpc off; radio.ldpc is on — re-run maburcal` before the first run, as specified |

## Walls (rel index, LDPC+STBC, ch 112)

| | mcs0-3 | mcs4 | mcs5 | mcs6 | mcs7 |
|---|---|---|---|---|---|
| spike reference 20 MHz (ch 144) | 63 | 45 | 23 | 22 | 21 |
| run 1, 20 MHz | 63 | 43 | 27 | 25 | 23 |
| power20-only run | 63 | 47 | 27 | 27 | 23 |
| run 2, 20 MHz | 63 | 47 | 27 | 28 | 23 |
| spike reference 40 MHz (ch 144) | 63 | 45 | 28 | 28 | 23 |
| run 1, 40 MHz | 63,63,63,61 | 43 | 30 | 29 | 27 |
| run 2, 40 MHz | 63 | 46 | 31 | 29 | 27 |

The ±2 gate against the spike is not met on mcs5-7 (up to +4 at 20 MHz
mcs5 and 40 MHz mcs7), but the runs were on ch 112 and the spike on ch 144,
so it is not the same geometry. Run to run on ch 112, mcs4 moved 43 → 47
and mcs6 25 → 28; every parked rate verified at 100 % except mcs4 in the
power20-only run (95 % at park 43). The 40 ≥ 20 relation held on every row
except mcs4 in run 2 (46 vs 47).

## Capacity

First build (eff = delivered bits / commanded 2 s dwell):

| point | frames × 1435 B | eff | spike |
|---|---|---|---|
| 20 MHz mcs0 | 1206 | 1.00 (clamped; 6.92 Mb/s) | 0.93 (6.04 Mb/s) |
| 20 MHz mcs4 | 6066 | 0.89 | 0.73 |
| 40 MHz mcs0 | 2232 | 0.95 | 0.87 |

1206 frames of 1435 air bytes cannot fit 2 s at 6.5 Mb/s. The drone's blast
enqueues into the `tx_threads` pool for the dwell and the queue keeps
draining on air afterwards; the GS's receive span per point was 2.36 s at
20 MHz mcs0, ~2.1 s at mcs1-3 and ~2.02-2.06 s at the top points. The same
drain explains 75 stamp mismatches: the drone's `SWEEP_DONE` (sent outside
the TX queue) overtook the last point's tail, and the tail frames landed in
the GS's next phase. Fixed: the GS times each point over its own receive
span on the best card and closes a capacity sweep after 50 ms of quiet
(`callog 5` adds `span_ms` to the P row).

Run 2 (span-timed):

| | mcs0 | mcs1 | mcs2 | mcs3 | mcs4 | mcs5 | mcs6 | mcs7 |
|---|---|---|---|---|---|---|---|---|
| 20 MHz | .90 | .87 | .84 | .81 | .86 | .87 | .74 lossy | .85 |
| 40 MHz | .87 | .81 | .86 | .86 | .84 | .85 | .86 | .86 |
| spike 20 MHz | .93 | | .83 | | .73 | | | |
| spike 40 MHz | .87 | | .72 | | | | | |

The singles (20 MHz mcs0-3, 40 MHz mcs0-1) agree with the spike within
0.03. The aggregated points read 0.12-0.14 above it. The kit's points were
air-bound by its own measure (`blocked_pct` 85-98 % up to 40 MHz mcs6, 74 %
at mcs7, none `tx_bound`), and an agg6 PPDU of 1435 B MPDUs caps near 0.92
of nominal, so 0.84-0.86 is physically plausible; the spike's sender
reported "under target" every second, so its aggregated points may have been
sender-bound too. Not settled: a USB-bound high-MCS point still reads as
air-bound under `blocked_pct` (the sender blocks on the device whatever
limits it), so the top rows are not proven air capacity. Zero stamp
mismatches in run 2.

## Tunnel-lane rejects

The first build's drone stats line showed `reject=` +2 at every run's end:
the GS sends the closing `MABUR_CAL_ABORT` three times, the first closes the
run, and the repeats arrive under the run's pair after the drone re-paired,
outside the run-pair latch. Fixed: an authentic cal-only frame under the
last run's pair after the run closed is dropped as `cal_late=` instead of
an auth reject. Run 2: `reject=0 cal_late=2`.

## Device state after the bench

Both ends were put back on their pre-bench binaries and configs (the old
`RC_VERSION` 17 pair): the revamped `maburd` never reached the drone's
flash (it ran from tmpfs), so leaving the renamed config behind would have
crash-looped the flash binary at the next boot. Deploying this build for
real needs room on the drone's flash for one more `maburd` first.

## Other observations

- `maburcal start`'s closing table renders every run in the session's
  `cal.log`, not just the run it started.
- The patcher keeps an efficiency line's old trailing comment (`# at 40 MHz:
  flat 0.74-0.77 …`) after rewriting its numbers, and an inserted
  `cal_flags_40` line is not column-aligned with its neighbours.
- `S00mabur start` / `S96maburgs start` launched from a non-interactive ssh
  command die with the session unless detached with `setsid`
  (`docs/deploy.md`); `gs/bundle/install.sh`'s own start survived on the
  second install but not the first.
