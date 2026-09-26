# Onboard SD recording spike — findings 2026-09-26

**Question.** Can `maburd` write a constant high-bitrate copy of the video to
the drone's microSD while the link stream stays dynamic-bitrate?

**Answer.** Yes, and — after the follow-up the same day — **at no cost to
the link stream's latency if the recorder channel is bound to the VPE port
BEFORE the link channel.** A second H.265 CBR VENC channel bound 1:N to
the same VPE output port records a valid 1080p60 file while the link stream
keeps 60 fps at its rung with zero drops. The first pass measured **every
recorder frame delaying the link frame that shares the encoder with it by
one full 1080p encode (~8 ms)**; the follow-up ("Where the +8 ms comes
from", below) traced that to the single-task H.265 engine serving the VPE
port's bind peers newest-bound-first, so the recorder's frame was encoded
ahead of the link's. Reversing the bind order puts the wait on the recorder
(which does not care) and returns the link's `enc` segment to baseline at
60, 30 and 5 fps recorder rates. What remains is CPU (+14 points at
60 fps), SoC heat (+4–6 °C) and engine occupancy (147 % of the nominal
1080p60 macroblock rate with two 1080p60 streams — it fits, but the venc
clock is already at oc-level 1).

Spike code: branch `sd-record-spike` (ef0c1af), throwaway, env-gated
(`MABUR_SDREC=<kbps>[,<fps>]`, `MABUR_SDREC_DIR`), not merged; the
follow-up's probes are one commit on top of it on branch `sd-record-lat`
(`MABUR_SDREC_FIRST=1` binds the recorder before the link channel,
`MABUR_SDREC_CHN=<n>`, and the recorder logs its own pts→GetStream
histogram). Nothing in `master` changed for either.

## Bench numbers

Bench drone `.152` (SSC338Q, imx415, 1080p60), GS at rung 5 / mcs 4 / 24 Mb/s,
`low_power.enable = false` for the run (the bench FC reports DISARMED, which
would otherwise pin the encoder at 30 fps / 1 Mb/s). Each run: 40 s settle,
90 s of the `:8300` sideport (60 s for D). `enc` is the drone's
capture-timestamp → bitstream-ready segment (`link.video.lat.enc`).

| run | recorder ch1 | link enc p50 / p99 ms | drone cpu % | SoC °C | link fps / rung | recorder actual |
|---|---|---|---|---|---|---|
| A baseline | off | 7.9 / 8.6 | 44 | 54 | 60 / 5 | — |
| B | 40 Mb/s @ 60 fps | 15.7 / 16.7 | 58 | 58 | 60 / 5 | 40 Mb/s, 53–60 fps |
| C | 40 Mb/s @ 30 fps | 15.1 / 16.9 | 51 | 60 | 60 / 5 | 26 Mb/s, 30 fps |
| D | 40 Mb/s @ 5 fps | 8.0 / 16.5 | 47 | 60 | 60 / 5 | 4.6 Mb/s, 5 fps |

Reading it:

- **B vs A: +7.8 ms p50, +8.1 ms p99.** One 1080p encode. The link's own
  fps, rung, `ring_drops`, `venc_full_drops` and `starved_drops` did not move.
- **C: same p50 as B** although ch1 only takes every other frame. With ≥50 %
  of link frames delayed the median lands on the delayed mode.
- **D: p50 back to baseline, p99 still doubled.** One link frame in twelve
  is delayed, by the same ~8 ms. So the penalty is per shared frame, not a
  driver mode change. (The first pass concluded the only lever was the
  recorder's pixel count; the follow-up below found the real lever is the
  bind order.)
- Recorder at 30 fps delivered 26 Mb/s, not 40: the CBR budget vs. bind
  delivered-rate mismatch the main channel compensates for
  (`rc_compensate_kbps`) applies to ch1 too. Irrelevant to the spike.
- SoC temperature +4–6 °C with the recorder on; CPU +14 points at 60 fps
  (the drain thread and `fwrite` on the two A7s).

## Where the +8 ms comes from (follow-up, same day)

**Instrument.** `/proc/mi_modules/mi_venc/mi_venc0` on the drone prints,
per channel, the average delay of every stage of the input task
(`OnPreProcessInputTask` → `EnqueueInputTask` → `CheckInputTaskStatus` →
`DequeueInputTask`) and of `GetStream`, all in µs on one common origin, and
`/proc/mi_modules/mi_vpe/mi_vpe0` prints the VPE output port's
`FinDMADispatch` on the same origin. `echo reset_counter > <that file>`
zeroes the averages, so a 40 s settle + reset + 60 s read gives clean
steady-state numbers. The same file also states the H.265 device has
**`MaxTaskCnt 1`** — one encode task in flight at a time — and prints
engine occupancy as `MbRate %` (73 for one 1080p60 stream, 147 for two).
Link-side truth came from the GS session's `lat.log` (`enc=p50/p99`) and
`au.log` (per-AU `enc_us`, base and enhance separately; a 60 s baseline
window: p50 7.25, p90 8.02, p99 8.25 ms, identical for base and enhance).

**Runs** (same bench setup as above; `low_power.enable=false`; averages
over 60 s after the counter reset; all µs on the MI origin, so only the
differences matter):

| run | recorder ch1 | bound | VPE dispatch | ch0 enqueue | ch0 dequeue | ch0 GetStream | ch1 enqueue | ch1 dequeue | link enc p50/p99 (lat.log) | recorder own pts→GetStream p50 |
|---|---|---|---|---|---|---|---|---|---|---|
| R0 | off | — | 16 772 | 16 846 | 24 050 | 24 548 | — | — | 7 / 8 ms | — |
| R1 | 40 Mb/s @ 60 | **after** link (as in B) | 16 619 | **24 763** | 32 014 | 32 489 | 16 749 | 24 742 | ~15 / 16 (sideport, run B) | 9.75 ms |
| R2 | 40 Mb/s @ 60 | **before** link | 16 650 | 16 752 | 23 979 | 24 491 | 23 996 | 32 020 | **7 / 8 ms** | 17.25 ms |
| R3 | 40 Mb/s @ 30 | before link | 16 669 | 16 759 | 23 986 | 24 588 | 23 944 | 32 370 | **7 / 8 ms** | 17.25 ms |
| R4 | 40 Mb/s @ 5 | before link | 16 776 | 16 848 | 23 990 | 24 451 | 23 981 | 32 429 | **7 / 8 ms** | 17.00 ms |

Reading it:

- **Baseline anatomy (R0).** VPE dispatch → VENC enqueue 0.07 ms; enqueue
  → dequeue **7.2 ms** — that is the hardware encode of one 1080p frame
  (min 5.9 ms); dequeue → `GetStream` 0.5 ms. Sum 7.8 ms = the `enc`
  segment. So `enc` really is almost entirely engine time, which is why a
  second 1080p stream could cost exactly one `enc`.
- **R1 is the whole story.** With the recorder bound after the link, the
  recorder's input task is pre-processed and enqueued at VPE dispatch
  (16 749) and the link's is not even pre-processed until the recorder's
  task has been dequeued (24 696 ≈ 24 742): the VENC worker runs one
  peer's entire task before it starts the next peer's, and it starts with
  the **most recently bound** peer. The link then encodes 7.25 ms and lands
  at 32.5 ms — one recorder encode (8.0 ms; a 40 Mb/s frame takes ~0.8 ms
  longer than a 24 Mb/s one) later than baseline. Nothing about the
  recorder's *rate* matters, only whether its frame is present for this
  VPE frame, which is exactly the B/C/D pattern.
- **R2–R4: bind the recorder first and the link is untouched.** The link's
  enqueue is back at VPE dispatch, its `GetStream` is within 0.1 ms of R0,
  and `lat.log` reads the same 7/8 as R0. The recorder's own latency
  absorbs the full wait (9.75 → 17.25 ms), which a file writer does not
  notice. The order rule is bind-time recency, not channel id: ch1 stayed
  ch1 in every run and went first only when it was bound last.
- **Idle peers are free.** The JPEG snapshot channel (ch7) is bound after
  the link channel in production and costs nothing while `StopRecvPic`
  holds it idle (R0 = today's production topology). Only a peer with a
  frame to encode enters the queue.
- **Engine occupancy.** `MbRate %` 73 → 147 (two 1080p60) / 110 (60+30) /
  79 (60+5); `UtilHw` is not populated on this SDK. Two 1080p60 encodes
  serialize to ~15.3 ms per 16.7 ms frame period — it fits, with the venc
  clock already at oc-level 1 (480 MHz). A sub-1080p recorder is no longer
  needed for latency, only if headroom or heat becomes the constraint.

**Consequence for a real port.** Create + bind the `[record]` channel
BEFORE `star6e_pipeline_start_venc()` binds the link channel (the VPE port
tuple `{VPE,0,0,0}` is known then; the spike does this behind
`MABUR_SDREC_FIRST`), and never re-bind the link channel while the recorder
is bound. Anything that later binds a *new* active peer to VPE port 0
(e.g. a live-enabled recorder started mid-flight) would again take
priority over the link — a start/stop recorder must therefore stay bound
and toggle `Start/StopRecvPic`, the way the JPEG channel already does.

**Recorder-fps caveat withdrawn.** The first pass's "run D drove
`vanished_base` to 30/s" is unrelated to the recorder: see "Open" below.

## SD card facts (Runcam WiFiLink 2, `.152`)

- The board has a working slot. A 64 GB SDXC enumerates on `mmc0`,
  SD high-speed timing, 48 MHz, 4-bit. The second controller (`mmc1`) has
  card-detect wired but nothing on it.
- OpenIPC's mdev rule (`/lib/mdev/automount.sh`) mounts `/dev/mmcblk0p1` at
  `/mnt/mmcblk0p1` on insert and at boot. No mount script needed.
- **The kernel has only vfat and msdos** as block filesystems: no exFAT, no
  ext4, no modules. `mkfs.vfat` is on the image, `fsck.fat` is not. The
  card is MBR, one 0x0C FAT32-LBA partition, 32 KB clusters, label
  `DRONEDVR`. FAT32's 4 GB file limit means any recorder must rotate on
  size (13 min at 40 Mb/s).
- Sustained write with `fsync`, encoder running: 16.5 MB/s in both 1 MB and
  64 KB chunks. Bandwidth is not the constraint at 40–50 Mb/s.
- **Flash garbage collection stalls the writer**: single writes of up to
  390 ms, 34 stalls > 50 ms in ~2.5 min at 40 Mb/s. The spike's synchronous
  `fwrite` behind an 8/56-deep VENC output port let those stalls drop
  recorder frames (53 fps dips in B). A real port needs the off-encode-loop
  writer thread waybeam added after the fold-in point
  (`src/venc_rec_writer.c`, see below).
- Output verified on the host: `ffprobe` reads `rec-0000.hevc` as HEVC Main
  1920x1080, 60 fps.

## What waybeam already has

`../waybeam_venc` was pulled from the fold-in base f956a52 to 0c880d8
(v0.84.0) for this. Upstream's `record.mode=dual` is exactly this topology:
ch1 at `record.bitrate` / `record.fps` / `record.gop_size`, a drain thread
(`dual_rec_thread_fn`, `star6e_runtime.c`), MPEG-TS or raw HEVC output with
size/time rotation, and an SD-backpressure rule that lowers ch1's bitrate
only. Since f956a52 the record path gained ~2400 lines: the writer thread,
drain-as-barrier, rotation without a forced keyframe, and removal of the
2 GB file ceiling. That is the code to port, minus the HTTP API, under a
`[record]` table in `mabur.toml`.

## Open

- **`drone.enc.vanished_base` at ~30/s is a maburd startup bug, not a
  recorder effect.** In the follow-up it happened with the recorder OFF
  (R0), with it at 5 fps (R4, R5) and not at 60/30 fps (R1–R3): 3 of 6
  starts. Dumping the GS AU ring in arrival order across a drone restart
  (R5) shows the first ~15 link AUs arriving **11.06 ms apart (the imx415
  1080p mode's 90 fps)**, then an 11/22 ms alternation as the 90→60 bind
  rate control drops frames, then 16.6 ms once `[venc] cold-boot fps
  re-kick: SetFps(60)` lands. `FramePipeline::track_vanish` seeds its
  period EMA from those first deltas (≈11.1 ms) and, because it only
  learns from deltas under `kVanishFactor` (1.5×) the current period,
  16.6 ms ≥ 1.5 × 11.06 is booked as a one-frame hole on (nearly) every
  frame, forever. Consequences: the sideport counter is noise for that
  session, and the detector's self-IDR request fires at the guard rate —
  14 link IDRs in the 60 s after the R5 restart (4 in the last 30 s, ~4.6 kB
  each under `min_iqp`) where GDR would normally issue none. Fix: call
  `note_rate_change()` from the cold-boot fps re-kick (it already exists
  for the low-power fps verb), or let the estimator re-seed after N
  consecutive vanish events. Not fixed here. Whether the pilot's flights
  hit it depends on the same race; check `vanished_base` growth from the
  first sideport samples of a session before trusting the counter.
- Recording at a lower resolution than the link (VPE port1, the second
  scaler waybeam's `star6e_vpe_ports.c` drives) was not tried; it is no
  longer needed for latency, only if engine headroom or SoC heat bites.
- Whether the H.265 engine at 147 % `MbRate` still meets the link's
  encode deadline at rung 0's superframe / IDR sizes and at 40 MHz rungs
  was not measured (bench sat at rung 5, mcs 4, 24 Mb/s throughout).

## Gotchas hit during the spike

- `S00mabur stop` sleeps 1 s before `killall` returns control; the MI
  kernel worker of the old daemon disconnects 2–3 s later. Starting inside
  that window trips the "stale [maburd] MI kernel worker … REBOOT the drone"
  refusal in `venc_core.c` although nothing is wedged. Wait until no process
  with comm `maburd` remains in `/proc` before starting again; a power
  cycle is not needed.
- The stale-worker refusal also aborts libusb on exit
  (`usbi_mutex_destroy` assertion) — cosmetic, same cause.

## Device state after the spike

Original `maburd` and `/etc/mabur.toml` restored on the drone
(`low_power.enable = true` again), video verified up, after both passes.
Spike binaries pruned (the follow-up ran its binary from `/tmp`, so
nothing touched the rootfs). Seven recordings (`rec-0000` … `rec-0006`,
~3.6 GB) left on the card.
