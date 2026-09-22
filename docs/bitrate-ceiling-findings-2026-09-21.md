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

## Lever 1 delivered: the frame-end join is gone (2026-09-22, branch fec-join-delete)

`SwEncoder::flush()` no longer spin-waits for the FEC worker. Repairs
ship when the worker finishes them: `UepEncoder::collect()` harvests the
done list on every venc-ring read timeout in the hot loop (≤5 ms apart)
and flushes the SBI packer's partial group once a layer has nothing
outstanding, so a frame's last repairs still ship "now" rather than with
the next frame. `finish()` keeps the joining form for shutdown
(`flush_all`) and tests; `poll()` never joins. Wire bytes, SBI framing
and `RC_VERSION` are unchanged; the GS was not rebuilt (SwDecoder has no
ordering contract and its 512-seq horizon, ~70–80 ms, dwarfs the harvest
cadence).

Two guards replace the join:

- **Backlog cap** `SwEncoder::kMaxBacklogJobs = 256`: a credited repair is
  skipped — booked as `backlog_drops=` on the 5 s `fec_worker` log line —
  when the shared worker queue (now 512 slots) already holds that many
  jobs, ~30 ms of worker time. A slow core degrades to fewer repairs,
  never to a blocked producer or a pinned ring. Skipped credits still
  consume their `repair_key`, so every repair that IS built is
  byte-identical to the sync encoder's (tests pin async ⊆ sync and
  sync repairs = async repairs + drops).
- **Ring slack** `kSlackRows` 64 → 512 (~170 kB per layer at 332 B): the
  row-safety backstop join in `seal_current` stays, but it is a backstop
  again — the worker may now legitimately trail by a frame or two.

Host: `test_sw_encoder_async` (11) and `test_uep_sw` (12) incl. held-worker
tests (`FecWorker::set_held`, test surface) for the non-joining flush,
`collect()`, the cap and the subset contract; suite green except the
pre-existing `sim_shed_lag` failure (RcAgent replay vs the current
bundle, unrelated).

### Flight op point — rung 5, mcs5, 0.5/0.25 singles, 18.1 Mb/s, 800 MHz

Equal `tools/build-arm.sh` -O3 builds, 60 s each, 120 s settle after each
restart, same per-thread instrument as above (6000 ticks = one core).
The deployed 867 kB binary of 2026-09-21 measured within 1.5 % of the
branch-point build (2736 vs 2774 hot ticks), so the flag question is
closed.

| maburd | mbr-hot ticks | hot % core | mbr-fecw | sys.cpu_pct | joins /5 s (mean wait) | qdepth_max | GS fec p50 / p99 | jitter | ausniff |
|---|---|---|---|---|---|---|---|---|---|
| branch point (join) | 2774 | 46 | 903 | 56.1 | 320 + 310 (2.8 / 1.8 ms) | 31 / 17 | 12.9 / 19.0 | 5.0 | 60.3, 0 gaps |
| no join, cap 192 | 991 | 16.5 | 883 | 40.7 | 0 | 99 / 66 | 10.3 / 14.6 | 3.3 | 60.3, 0 gaps |
| **no join, cap 256 (deployed)** | **988** | **16.5** | 864 | **40.0** | 0 | 81 / 45 | **10.2 / 14.4** | **3.2** | 60.3, 0 gaps |

`dq_split cpu_us` mean 2.0 ms (the feed itself — the profile's 16 %),
`backlog_drops` 0, GS `clean` +3597/60 s, 0 dropped, `pre_fec_loss` 0,
aucadence base−enh offset −0.87 ms (envelope −1.1 to −3.0, gate 4.0).
The `qdepth_max` rise is the trail the join used to hide, not new work
(`mbr-fecw` ticks are unchanged). The fec p50 gain (−2.7 ms) is the same
mechanism as the CRC table's: the enh half is production-bound at this
rate and a faster feed lands straight in AU completion. `sys.cpu_pct`
−16 points is the spin that used to burn cpu1 at full rate.

### The knee, re-tested — and re-attributed to the air

Static mcs7, 1.0/0.5, budget 0.70 (19.2 Mb/s), the 2026-09-21 row's
config, **but today's bundle runs singles at every rung** (7a4faec,
A-MPDU `max_num 1`), so the air side differs from the two earlier rows:

| | mbr-hot % core | mbr-fecw | sys.cpu_pct | venc ring mean/max | GS fec p50 / p99 | GS fps | congestion_shed samples | txq_wait mean / max |
|---|---|---|---|---|---|---|---|---|
| deployed, 2026-09-21 (A-MPDU) | 89 | — | 93.7 | 5 / 25 | 52.4 / 62.8 | 60.4 | — | — |
| table CRC, -O3 (A-MPDU) | 71 | — | 77.4 | 1 / 12 | 13.7 / 23.5 | 60.3 | — | — |
| branch point, singles | 71 | 1746 | 80.9 | 3.8 / 37 | 15.2 / 24.0 | 53.9 | 60 / 300 | 18.9 / 53 ms |
| **no join, singles** | **19** | 1654 | **54.1** | **0 / 0** | **10.0 / 14.9** | 54.2 | 56 / 298 | 17.4 / 50 ms |

Both singles rows lose ~18 % of enh AUs (ausniff enh 1441 / 1488 vs base
1810 / 1812) with zero loss counted anywhere — GS `dropped` 0,
`pre_fec_loss` 0, drone `vanished` +3 — because at ~3230 bodies/s the
per-body dead time of un-aggregated PPDUs saturates the channel, the
TxQueue wait runs 17–19 ms mean / 50 ms max, and the drone's
**congestion shed** drops the enh layer (`drone.congestion_shed` true in
a fifth of the samples). The FEC change is not implicated: the shed
fraction, queue wait and fps are the same on both binaries. What the
change buys at this point is the SoC: hot thread 71 → 19 % of its core,
cpu 81 → 54 %, ring 37 → 0 % max, and the GS `fec` segment back to the
flight-op-point numbers. **At 19 Mb/s and singles the air, not the SoC,
is the wall**; re-running this point with A-MPDU on is the way to find
the new SoC wall (linear extrapolation of `mbr-fecw` at 28 % of cpu0 for
1.0 × 120 repairs/frame puts the worker's own limit near 45 Mb/s at ov
1.0 — the GF256 kernel, lever 2, is now the next term).

The knee run also sized the backlog cap: at cap 192 the shared queue's
frame-end peak touched the cap in every window (77 + 21 skipped per 5 s,
0.4 % of repairs) although the worker sat at 28 % of cpu0 — the tx/usb
wakeups the feed triggers preempt the worker on cpu0 during the burst,
so the peak is deeper than feed-time arithmetic (~115) says. Cap 256,
queue 512.

### Deploy state

Drone `.152` runs the cap-256 build (`/usr/bin/maburd`, md5 466d8a2e…);
rollback `maburd.pre-nojoin` = the 867 kB binary of 2026-09-21 (binary
only — no config or wire change, deploy order irrelevant).
`maburd.pre-bro3` and `maburd.pre-crctab` were left in place (3.4 MB
free). GS unchanged. Both configs restored from `*.pre-nojoinceil`
(flight bundle: cap 24000, budget 0.65, adaptive ladder). Raw: GS
`/tmp/bw-nojoin-{A,B,C,K19,K19base,final}.jsonl`. Bench left powered on.

Remaining from the ranked list: lever 2 (`gf::lincomb` cost per repair —
the worker is now the SoC's throughput term), the copy/alloc diet (~10 %
of the hot core), `classify_frame`'s double scan, and the clock (lever 1
of the first list, last by operator preference). Re-measure the wall with
A-MPDU on before ranking further.

### The knee and the 22 Mb/s collapse, re-run with A-MPDU on (2026-09-22, same session)

Same pins (static mcs7, 1.0/0.5), drone `ampdu.max_num 6` +
`fec.feed_batch 6` (the pre-7a4faec values), no-join cap-256 build, 60 s
after a 120 s settle:

| target (budget) | enc Mb/s | sys.cpu_pct | mbr-hot % core | mbr-fecw % cpu0 | venc ring mean/max | full_drops | qdepth_max | backlog_drops /5 s | txq_wait mean/max | air_pct | GS fps | GS fec p50 / p99 | ausniff |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 19.2 (0.70), 2026-09-21 deployed | 19.0 | 93.7 | 89 | 36 | 5 / 25 | 0 | 31 | — | — | — | 60.4 | 52.4 / 62.8 | 60.4, 0 gaps |
| **19.2 (0.70), no join, agg6** | 19.0 | **53.7** | **20** | 28 | **0 / 0** | 0 | 171 / 146 | 0 | 2.9 / 4 ms | 56.4 | 60.0 | **8.0 / 11.9** | 60.3, 0 gaps |
| 22.0 (0.80), 2026-09-21 deployed | 19.1 (fps 57.5) | 98 | 95 | 38 | **58 / 100** | **172** | — | — | — | — | 53.4 | 310 / 328 | 53.4 fps |
| **22.0 (0.80), no join, agg6** | **21.8** | **58.9** | **23** | 32 | **0 / 0** | **0** | 245 / 256 | 0 + 15 | 3.2 / 7 ms | 65.0 | **60.0** | **9.5 / 14.2** | 60.2, 1 gap |

The 22 Mb/s point that pinned the ring and dropped ~3 frames/s on
2026-09-21 now delivers its full 21.8 Mb/s at 60 fps with the SoC at
59 %. The hot thread is at 23 % of its core, the worker at 32 % of cpu0,
`dq_split cpu_us` 2.6 ms/frame. The old wall is gone; **the SoC wall was
not reached in this session** — the highest point tried is clean, and
linear extrapolation of the two threads (hot ~1 %-of-core per Mb/s,
worker ~1.5 % of cpu0 per Mb/s at ov 1.0/0.5) puts both cores near their
limits only in the 45–50 Mb/s region, i.e. beyond what mcs7/20 MHz
carries. Two things did move at 22 Mb/s:

- **The air is the next term.** `air_pct` 65 % (p95 69, max 74),
  `air_backlog_max_ms` up to 19 ms, and the first real loss of the
  session: GS `dropped` +4 / `truncated` +1 in 60 s, ausniff 1
  `frame_id_gap`, `pre_fec_loss` max 0.1 %. With agg6 a lost aggregate
  is 12–16 symbols, which is exactly the 1.0/0.5-vs-0.25 trade 7a4faec
  documents. No congestion shed (0 of 299 samples) — the queue never
  built.
- **The backlog cap grazed at the frame-end peak**: `qdepth_max` 245
  (base window) / 256 (enh window), `backlog_drops` 15 per 5 s on the enh
  encoder = 0.05 % of repairs at 22 Mb/s and ov 1.0/0.5 with agg6. The
  peak is deeper than at singles because `feed_batch 6` releases bodies
  in groups, so the tx/usb wakeups that preempt the worker on cpu0 come
  in bursts too. Harmless at this rate; at the flight pair (0.5/0.25) the
  per-frame repair count is half and the cap is not in play (81/45 at
  18.1 Mb/s). If ov 1.0 is ever flown above ~22 Mb/s, the cap (or the
  worker's per-repair cost, lever 2) is what to revisit.

Vanish counts in the 22 Mb/s window (+1784/+1785) are the restart
warm-up class again: GS `clean` +3596 of 3600, `enc.fps` 60.0.

So the answer to "is 19.2 the limit?" is: with singles, yes, and it is a
body-rate limit (~3200 un-aggregated bodies/s saturate the air on
per-PPDU dead time, the drone's congestion shed drops enh). With agg6 the
same SoC and radio carry 22 Mb/s at the 1.0/0.5 pair with margin on both
cores; the next wall is airtime at mcs7/20 MHz (65 % at 22 Mb/s with that
pair), reached before the SoC. On the flight pair (0.5/0.25, singles) the
body rate at 18.1 Mb/s was 2470/s with a 2–3 ms queue wait, so its
singles ceiling sits roughly at 22–23 Mb/s — not measured.

Bench end state: both configs restored from `*.pre-aggceil` (= the flight
bundle: singles, 0.5/0.25, cap 24000, budget 0.65, adaptive ladder);
drone on the cap-256 build; GS unchanged; raw GS
`/tmp/bw-nojoin-K19agg.jsonl`, `K22agg.jsonl`; both plugs on.

## Lever 2 delivered: one-pass repair kernel (2026-09-22, branch fec-join-delete)

`gf::lincomb_rows(out, rows[], coeffs[], n, len)` builds a repair symbol
from its whole window in one pass: the 64 B output block lives in four q
registers across all n rows, so the accumulator is loaded and stored once
per repair instead of once per row (the per-row `lincomb` did 8 loads +
4 stores per 64 B for 4 multiplies). `SwEncoder::build_repair` calls it
over `stride_` (= symbol size rounded up to 16, already the ring's row
width) and trims the envelope back to `symbol_size`, which deletes the
12 B scalar tail every row used to end with at 332 — that tail was ~20 %
of the row's cycles and the whole of the profile's `gf::tables()` 3.8 %.
Wire bytes are unchanged (`vectors_byte_exact` pins the repairs on the
host; `gf_bench` verifies the NEON path byte-for-byte on the drone).

**Microbench on the drone** (`tools/bench/gf_bench.cpp`, `-O3`, maburd
stopped, one 32-row window per repair, 20 000 repairs; "cold" = a 544-row
183 kB ring with the window sliding one row per repair, as in situ):

| shape | per-row `lincomb` | `lincomb_rows` |
|---|---|---|
| 32 × 332 (shipped path: scalar tail) | 32.5 us/repair | 31.1 |
| 32 × 336 (padded, what build_repair now does) | 25.7 | **21.0** |
| 32 × 336 cold ring | 25.7 | 21.0 |

So −35 % CPU per repair: −22 % from the padding alone, −16 % more from
the one-pass kernel. It is not the halving the ranked list asked for,
and the disassembly says why: the 64 B block loop has no spills and is
6 loads + 16 `vtbl.8` + 20 ALU ops per row, i.e. the Cortex-A7's 64-bit
NEON datapath limit for a nibble-table multiply (21 us = 1.55
cycles/byte at 800 MHz). Cold rows cost nothing extra (the shared L2
serves the ring) and a `pld` of the next row bought nothing (tried,
removed). The remaining lever inside the kernel is sharing the nibble
split and source loads between two repairs of the same window, worth
~14 % on paper and needing the worker to pair jobs — not taken.

**In situ, flight bundle (adaptive ladder parked at rung 5, singles,
0.5/0.25), 17.1 Mb/s today, 60 s windows after a 120 s settle, same
per-thread instrument (6000 ticks = one core), same session A/B:**

| maburd | mbr-fecw ticks (% cpu0) | mbr-hot | sys.cpu_pct | worker build_us/job | GS fec p50 / p99 | ausniff |
|---|---|---|---|---|---|---|
| cap-256 no-join (`maburd.pre-gfrows`) | 1053 (17.6) | 978 | 46.5 | 100–113 (2026-09-22 morning) | 8.3 / 11.2 | 3615 AUs, 60.3 fps, 0 gaps |
| **lincomb_rows (deployed)** | **719 (12.0)** | 976 | **43.3** | 83–92 | 8.4 / 11.1 | 3615 AUs, 60.3 fps, 0 gaps |

Worker CPU −32 %, hot thread untouched, SoC −3.2 points, `pre_fec_loss`
0 both runs. The GS `fec` segment did not move, as §"Where the hot
thread's time goes" predicted: at rung 5 that segment is air, and the
drone's FEC worker was never on the clean-frame latency path. The
worker's wall gauge dropped less than its CPU (about −15 %) because on
cpu0 it is mostly waiting out tx/usb/venc preemption, not computing.
What the change buys is exactly what §"Levers" priced it at: the
worker's own throughput limit moves from ~45 to ~65 Mb/s at ov 1.0 (both
beyond mcs7/20 MHz), the 22 Mb/s / ov 1.0 / agg6 backlog-cap graze has
a third more headroom, and 5.6 points of cpu0 come back for the thermal
budget.

Deploy state: drone `.152` runs `lincomb_rows` (md5 fe0ba97e…);
rollback `maburd.pre-gfrows` = the cap-256 build (binary only, no config
or wire change, deploy order irrelevant). `maburd.pre-crctab` and
`maburd.pre-bro3` were pruned for space (3.8 MB free, three binaries).
GS unchanged. Raw: GS `/tmp/bw-gfrows-{base,new}.jsonl`. Host suite
148/148.

Remaining from the ranked list: the copy/alloc diet (~10 % of the hot
core), `classify_frame`'s double scan, and the clock (last, by operator
preference). The SoC is not the wall at any rate mcs7/20 MHz carries;
the air is (§"The knee and the 22 Mb/s collapse, re-run with A-MPDU on").
