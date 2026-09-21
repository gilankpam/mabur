# Drone bitrate ceiling re-measured — 2026-09-21

Question: how much video can `maburd` push before the SoC runs out of CPU?
The last number on record was ~12 Mb/s (docs/venc-ring-vanish-findings-
2026-08-12.md: mcs5, FEC ov 0.25, waybeam era, before the async FEC worker,
the venc fold-in and the cpu0/cpu1 affinity policy). This page is the same
measurement on the 2026-09-21 build (master 9e5fceb, drone binary of
2026-09-21 02:42, both ends the flight bundle config).

**Answer: the wall is ~20 Mb/s. Below ~18 Mb/s the drone is clean; 19 Mb/s
runs but the GS-side `fec` latency has tripled; 22 Mb/s pins the venc ring
and drops ~3 frames/s inside the drone.** The limiter is `mbr-hot`, the
single producer thread pinned to cpu1: it burns 4.5 % of its core per Mb/s
and hits ~90 % at 19 Mb/s.

## Method

Bench, drone `.152` + GS `10.18.0.1`, 1080p60, channel 136, RSSI −48/−60,
SoC 56–58 °C throughout (no thermal effect). Video = the static bench
scene; the encoder reached every requested rate (enc.mbps ≈ cmd_kbps), so
the scene was not a limiter.

The bitrate target is `run_bitrate_policy()`'s airtime formula
(`airtime_budget × delivered(mcs) / (0.6·(1+ovb) + 0.4·(1+ove))`), so the
rate was stepped by (a) pinning the GS at `static_mcs = 7` with the
production rung-5 pair `static_overhead_base 1.0 / _enh 0.5` (delivered
mcs7 = 65 × 0.76 = 49.4 Mb/s, plenty of air) and (b) raising the drone's
`bitrate_max_kbps` to 40000 and stepping `airtime_budget`. Each drone step
is a `maburd` restart; the GS pin is one `maburgs` restart. FEC overhead
stayed at the production pair at every step, so the FEC and USB cost per
video byte is the flight one.

Instruments, 60 s per step:
- GS sideport recorded by `/root/bwceil.py` (scratch summariser: drone
  `enc.mbps`, `sys.cpu_pct`, `venc_ring_fill_pct`, deltas of `vanished_*`,
  `venc_full_drops`, `air_shed_drops`; GS `video.fps`, `lat.fec`).
- `tools/bench/ausniff.py` on the AU ring (the standing gate).
- Drone `/proc/<maburd>/task/*/stat` utime+stime deltas per thread and
  `/proc/stat` — `sys.cpu_pct` on the sideport is busy % of BOTH cores
  (100 = both saturated), verified against the raw `/proc/stat` delta.
- Drone `GET :8301/venc` for `ring_fill_pct` / `full_drops` every 10 s.

## Results

| target (budget) | enc Mb/s | cpu % (2 cores) | mbr-hot % of its core | mbr-fecw | ring fill mean/max | full_drops /60 s | vanished /60 s | ausniff | GS fec p50/p99 ms |
|---|---|---|---|---|---|---|---|---|---|
| 14.3 (mcs5, 0.65 = flight) | 14.2 | 70.7 | — | — | 0/12 | 0 | 3 | 60.3 fps, 0 gaps | 16.1 / 24.0 |
| 16.0 (mcs7, cap 16000) | 15.9 | 77.9 | 71 | 29 | 0/12 | 0 | 3 | 60.3, 0 | 18.4 / 25.7 |
| 17.8 (mcs7, 0.65) | 17.6 | 84.4 | 79 | 32 | 1/12 | 0 | 2 † | 60.4, 0 | 19–22 / 27–28 |
| 19.2 (mcs7, 0.70) | 19.0 | 93.7 | 89 | 36 | 5/25 | 0 | 3 | 60.4, 0 | **52.4 / 62.8** |
| 22.0 (mcs7, 0.80) | 19.1 (fps 57.5) | 98 (idle 2 %) | 95 | 38 | **58/100** | **172** | 71/113 real | 53.4 fps | 310 / 328 |

† The first 60 s window at 17.8 booked vanished +526/+525 with every frame
arriving on the GS (ausniff 3621 complete AUs, clean +3614); the 40 s
re-sample booked +2. That is the vanish detector's period-EMA warm-up after
a restart, not loss — the same class as the boot pollution noted in
docs/observability.md. Discount vanish counts for the first ~2 min after a
maburd restart — the final restore run booked +1060/+1060 in its first
40 s while the ladder climbed 0→5 with the GS clean counter at +2399 and
`video.fps` 60.0.

Per-thread ticks over 60 s (6000 = one core) at 16 → 19 → 22 Mb/s:
`mbr-hot` 4260 → 5344 → 5695; `mbr-fecw` 1747 → 2154 → 2252; `mbr-txw`
382 → 467 → 480; 4× `mbr-usb` ≈ 275 → 342 → 363 each; `maburd` main 415 →
497 → 516; `venc-enc` 334 → 359 → 356; `mbr-msp` ~170 flat. The hot thread
scales at ~4.5 %-of-core per Mb/s with essentially no fixed part; the
whole cpu0 population (fecw + txw + usb×4 + main + venc + msp + the SDK
kernel threads) is at ~85 % of cpu0 by 19 Mb/s, so both cores hit the wall
within a Mb/s or two of each other.

## What this means

- **The 16 000 cap and `max_mcs 5` in the bundle are CPU-honest with
  margin**: the flight op point (14.3 Mb/s at rung 5) leaves ~30 % of the
  SoC and the hot thread at ~65 % of its core. The margin is what absorbs
  IDR bursts (2.2–4.5 kB with `min_iqp 34`), motion peaks (`enc_pk100`
  read 26 Mb/s at the 22 Mb/s step) and the thermal ramp with the fan off.
- Raising `max_mcs` to 7 with today's formula would ask 17.8 Mb/s — clean
  on the bench but with the hot thread at 79 % and `fec` already +4 ms;
  no room for the peaks above. Do not raise the cap past ~18 000 on this
  SoC without first cutting the hot thread's per-byte cost (the
  feed-own frag→SW→SBI path, ~4.7 ms/AU, is still the biggest single
  term — docs/dq-spike-findings-2026-08-31.md §19).
- Air is NOT the limiter at 20 MHz: at 19 Mb/s and the 1.0/0.5 pair the air
  read 58 % of delivered mcs7; the SoC gave out first. The August "~12"
  number is superseded — the async worker, the fold-in and the affinity
  policy together bought ~7 Mb/s.
- The onset symptom is latency, not loss: GS `lat.fec` p50 goes 18 → 22 →
  52 ms across 16 → 18 → 19 Mb/s while every counter still reads clean.
  A ladder that only watches loss will not see the wall coming; `cpu_pct`
  on the sideport (deployed 2026-09-21) is the early gauge — above ~90 %
  the drone is one IDR away from the ring pinning.

## Side observations

- `S00mabur restart`: the old `maburd` stays alive 45–80 s after `stop`
  (MI teardown), the wrapper's first respawn collides with it and dies,
  and the second respawn is the one that comes up. Every restart in this
  session went through one extra respawn (pids 1361→1375, 1636→1647). This
  is probably the "1 in 4 warm-restart crash" of the low-power spike; it
  is a stop-overlap, not a race in maburd's own init.
- Vendor `/sys/class/thermal` is absent on this image; SoC temperature is
  the sideport's `sys.soc_temp_c` only.

Raw: GS `/tmp/bw-*.jsonl` (sideport) and `/tmp/bw-*.ausniff.json`;
drone configs restored from `/etc/mabur.toml.pre-bwceil`, GS from
`/etc/maburgs.toml.pre-bwceil` (both kept). No code changed.

## Can it go higher? Same day, follow-up

Three levers found, ranked by what they buy per unit of work.

### 1. The CPU clock — the drone runs at 800 MHz of an available 1.2 GHz

`/etc/init.d/S00cpuboost` sets `performance` + `scaling_max_freq 1200000`
for boot and `S99cpurestore` drops it to `800000` + `ondemand` once boot is
done; steady state is 800 MHz (`scaling_available_frequencies` 400…1200
MHz, `cpuinfo_max_freq` 1200000). A runtime A/B at the flight op point
(14.2 Mb/s, mcs5, 1.0/0.5), 60 s each, same per-thread instrument:

| clock | mbr-hot user+sys ticks/60 s (6000 = one core) | mbr-fecw | sys.cpu_pct | GS fec p50/p99 | delivery |
|---|---|---|---|---|---|
| 800 MHz ondemand | 3709 (62 %) | 1581 | 69.8 | 15.6 / 22.6 ms | 60.0 fps, clean |
| 1200 MHz performance | 2585 (43 %) | 1018 | 50.1 | 14.8 / 21.3 ms | 60.0 fps, clean |

Hot-thread cost 3.0 → 2.1 %-of-core per Mb/s (ratio 0.70, versus 0.67 for
a pure clock scale — it is compute-bound, ~98 % user time, not memory- or
syscall-bound). Extrapolated wall at 1.2 GHz ≈ 30 Mb/s, i.e. beyond the
~27 Mb/s that mcs7/20 MHz can carry at the production FEC pair: **at
1.2 GHz the radio, not the SoC, becomes the limit.**

Thermal: the chip has no voltage scaling (low-power spike 2026-09-19), so
CPU power scales ~linearly with clock. A 4-minute hold at 1.2 GHz under
the 14.2 Mb/s load kept `soc_temp_c` at 58 flat (57.6 mean at 800 MHz
just before) with the bench drone's cooling as it stands today; the
fan-off/airframe case is NOT measured and is the reason the OpenIPC
scripts park it at 800 — re-run the low-power spike's fan-off hold at
1.2 GHz before shipping. Also unverified: whether `performance` at
1.2 GHz changes the MI/ISP kernel threads' behaviour over a long run
(the 4 min hold showed vring 0, no vanish, no drops).

Cheapest possible change if it holds thermally: delete `S99cpurestore`
(or have `S00mabur` own the policy), no maburd change.

### 2. CRC16 is bit-serial — `common/src/crc16.cpp`

`crc16_ccitt` is the 8-iterations-per-byte loop, run over every FEC
envelope (source and repair) in `SbiPacker::build_body`, i.e. over
(1+ov) × video bytes per frame — ~72 kB/frame at 19 Mb/s. At ~35–40
cycles/byte that is ~2.5–3 ms of the hot thread's ~15 ms/frame at
19 Mb/s. A 256-entry table (~4–5 cycles/byte) or slice-by-4 brings it
under 0.5 ms; the GS side (`sbi_unpack`) pays the same loop on RX. Wire
format unchanged. Needs a host microbench before/after (encbench feeds
the same path).

### 3. The copy/alloc chain in the feed path

Per 332 B symbol today: frag vector → `current_symbol_` (two push_backs +
insert) → `env` (new vector, header + insert) → ring row (memcpy) →
`SbiPacker::pending_` (copy-construct) → `batch` (vector-of-vectors
COPY, then `erase` from the front of `pending_`) → body (insert). Five
copies and four allocations per symbol, ~200 symbols/frame at 19 Mb/s.
Order-of-magnitude ~0.5–1 ms/frame; real but third. Fix shape: build the
SBI body in place (seal straight into the body buffer, CRC as you go),
and move envelopes instead of copying `batch`.

Not levers: `-Os` applies only to `venc_core` (SDK-facing translation
units, deliberately), not to the feed path; the GF256 repair path is
NEON and off-thread (`mbr-fecw` ~26 % at 14 Mb/s) and the worker keeps
pace. Bench end state: clock restored to 800 MHz ondemand; both plugs
off.

## Lever 2 delivered: table-driven CRC16 (2026-09-22, branch crc16-table)

`common/src/crc16.cpp` is now a compile-time 256-entry table, one lookup
per byte; same signature, same `init` semantics, wire bytes unchanged
(tests/vectors/crc16.json, the 0x29B1 check value, 2000 random
length/init cases against the bit-serial oracle kept in
`tests/test_crc16.cpp`, and the chaining property crc(a‖b) = crc(b,
init=crc(a)) all pass; host suite 148/148). `tools/bench/crc_bench.cpp`
(`--target crc_bench`) is the microbench.

**On the A7 at 800 MHz** (maburd stopped, `-O2 -static` cross-builds):

| | ns/byte | per 72 kB air frame |
|---|---|---|
| bit-serial | 71.5 | 5.27 ms |
| table | 10.2 | 0.75 ms |

`encbench scalar 1.0 28 8` (whole feed path, one thread, 39 kB frames,
ov 1.0): SUST_vid 15.7 → 21.7 Mb/s, ns/src byte 509 → 368.

**In situ, flight op point (14.2 Mb/s, mcs5, 1.0/0.5), 60 s each, same
per-thread instrument, all at 800 MHz.** Three binaries, because the
drone's deployed `maburd` (736 kB) turned out to be an `-Os` image-builder
build while `tools/build-arm.sh` produces `-O3` (1.08 MB, the toolchain's
`-Os` init flag is overridden by Release's `-O3`, verified in
`flags.make`) — so the A/B had to be done on equal flags:

| maburd | mbr-hot ticks/60 s | hot % core | sys.cpu_pct | GS fec p50 / p99 |
|---|---|---|---|---|
| deployed (-Os image build, old CRC) | 3709 | 62 | 69.8 | 15.6 / 22.6 |
| build-arm.sh, old CRC | 3395 | 57 | 65.6 | 15.2 / 22.6 |
| build-arm.sh, **table CRC** | **3040** | **51** | **61.1** | **12.4 / 18.7** |

The CRC alone is −10.5 % on the hot thread and −2.8 ms on `fec` p50 (the
enh half of the stream is production-bound at this rate, so the feed
speed-up lands straight in AU completion). `-O3` vs `-Os` is a further
−8.5 % that was there for the taking. The in-situ CRC delta (~1 ms/frame
at 53 kB air) is smaller than crc_bench's 3.2 ms because maburd's `-O3`
bit-serial loop was already ~3× faster than the `-O2` bench's; the table
is flag-insensitive.

**The knee re-tested at the 19.2 Mb/s point** (static mcs7, 1.0/0.5,
budget 0.70), new binary vs the 2026-09-21 row above:

| | mbr-hot % core | sys.cpu_pct | venc ring mean/max | full_drops /60 s | GS fec p50 / p99 | ausniff |
|---|---|---|---|---|---|---|
| deployed, 2026-09-21 | 89 | 93.7 | 5 / 25 | 0 | **52.4 / 62.8** | 60.4, 0 gaps |
| table CRC, -O3 | **71** | **77.4** | 1 / 12 | 0 | **13.7 / 23.5** | 60.3, 0 gaps |

19 Mb/s now looks like 16 Mb/s did yesterday. Linear extrapolation of the
hot thread (71 % at 19.0) puts the new wall near 25–26 Mb/s at 800 MHz;
not measured — the 22 Mb/s point was not re-run. **Deployed both ends**
2026-09-22 (drone `maburd.pre-crctab` = the old 736 kB image build, GS
`maburgs.pre-crctab`); no config change, no wire change, deploy order
irrelevant. The GS pays the same CRC on RX per body; not measured
separately (aarch64, not the constraint).

Remaining from the ranked list: the copy/alloc chain (lever 3), then the
clock (lever 1, last by operator preference, thermal fan-off hold first).

## Where the hot thread's time goes: a PC-sample profile (2026-09-22)

`tools/bench/pcsample/` — a 120-line static sampler around
`perf_event_open` (software cpu-clock, 1 kHz, per-thread; the drone kernel
has perf events but no `perf` binary) plus `symbolize.py`, which maps the
histogram through the toolchain's addr2line against the unstripped
Buildroot build (PIE: file offset → LOAD vaddr). Cross-build with the
openipc gcc, `-O2 -static`; run as
`pcsample <tid> 60 1000 out.txt` on the drone, symbolize on the host.

60 s at 18.3 Mb/s, rung 5, singles, 0.5/0.25, table CRC, 800 MHz:

**`mbr-hot`, 27 005 samples = 45 % of its core:**

| share | where |
|---|---|
| **64.2 %** | `SwEncoder::join()` — the frame-end spin-wait for the FEC worker |
| 8.0 % | `crc16_ccitt` (table) |
| 5.2 % | `memcpy` |
| 3.9 % | `classify_frame` (the whole-payload NAL scan) |
| ~5 % | `malloc`/`free`/`operator new` between them |
| ~4 % | `seal_current`, `SbiPacker::add/build_body`, `pack_header`, `fragment`, `add_packet`, `drain_done` |
| 3.3 % | kernel |

**`mbr-fecw`, 9 720 samples = 16 % of cpu0:** 73.7 % `gf::lincomb` (the
NEON GF256 kernel), 8.8 % `FecWorker::loop`, 3.8 % `gf::tables()`, 3.1 %
`repair_coeffs`. The worker's own gauge: 100–113 µs per repair job,
`join_wait_us mean=2860` (base) and `2115` (enh) per frame, `qdepth_max`
31/17.

So the "unexplained 100 cycles per byte" was never per-byte work. The
real feed — fragment, symbol, envelope, ring row, pack, CRC, copies,
allocations — is ~16 % of the core, ~2.7 ms per 38 kB frame, ~0.07 µs/B,
exactly what the code reads as. The other 29 % of the core, ~5 ms per
frame, is the hot thread spinning in `join()` until the worker has
finished every repair of the frame just fed. The worker needs
~100 µs × (symbols × ov) per frame — 5.7 ms for a 38 kB base frame at
ov 0.5, 12.7 ms for a 47 kB one at ov 1.0 — on the core it shares with
txw, the four USB threads, venc and the SDK. **That is the throughput
wall's mechanism:** when the worker's repair time per frame exceeds the
frame period minus the feed time, the join outlasts the ring, the ring
pins, frames vanish. The 22 Mb/s / ov 1.0 collapse on 2026-09-21 is that
arithmetic.

What this does NOT change: at rung 5 the fec segment is air. The au.log
slope (0.348 µs/B base, 0.311 enh, zero intercept) is the singles
serialization rate (≈34.5 Mb/s delivered at mcs5 without aggregation,
efficiency ~0.66 vs 0.76 aggregated), and the feed at 0.07 µs/B is
five times faster than the air. The drone's CPU is not on the clean-frame
latency path today; the join is a throughput, thermal and repair-timing
problem.

Levers, re-ranked with the profile:

1. **Delete the frame-end join** (§19's lever 3): let the tail repair
   ship when the worker finishes, one frame late at worst (`SwDecoder`
   has no ordering contract); keep only the ring-row backstop join. −29 %
   of the hot core immediately, and the wall moves to the worker's own
   pace. Needs a bound on the worker backlog (drop or inline the oldest
   when the queue would overrun a frame period) so a slow core degrades to
   fewer repairs, never to a pinned ring.
2. **Halve `gf::lincomb`'s cost per repair**: 100 µs for 32 rows × 332 B
   is 9.4 ns/B; a two-rows-per-pass kernel or a wider unroll should get
   under 5. Raises the worker's pace directly.
3. The copy/alloc diet: ~10 % of the hot core, a few hundred µs per
   frame. Third.
4. `classify_frame` + `frame_is_trail_n`: two byte-wise scans of the
   whole payload for one NAL header; the producer already knows the NAL
   type. 4 %.
