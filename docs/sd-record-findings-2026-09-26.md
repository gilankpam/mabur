# Onboard SD recording spike — findings 2026-09-26

**Question.** Can `maburd` write a constant high-bitrate copy of the video to
the drone's microSD while the link stream stays dynamic-bitrate?

**Answer.** Yes, mechanically. A second H.265 CBR VENC channel bound 1:N to
the same VPE output port records a valid 1080p60 file while the link stream
keeps 60 fps at its rung with zero drops. The cost is on the link stream's
encode segment: **every recorder frame delays the link frame that shares the
encoder with it by one full 1080p encode (~8 ms)**, and that penalty does not
shrink with a slower recorder until the recorder is nearly idle. Shipping
this is a decision between ~8 ms extra glass-to-glass while recording and
a sub-1080p recording off VPE's second scaler port.

Spike code: branch `sd-record-spike` (ef0c1af), throwaway, env-gated
(`MABUR_SDREC=<kbps>[,<fps>]`, `MABUR_SDREC_DIR`), not merged. Nothing in
`master` changed for it.

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
  driver mode change, and the only lever on its size is the recorder's
  pixel count (encode time), not its rate.
- Recorder at 30 fps delivered 26 Mb/s, not 40: the CBR budget vs. bind
  delivered-rate mismatch the main channel compensates for
  (`rc_compensate_kbps`) applies to ch1 too. Irrelevant to the spike.
- SoC temperature +4–6 °C with the recorder on; CPU +14 points at 60 fps
  (the drain thread and `fwrite` on the two A7s).

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

- **Run D drove `drone.enc.vanished_base` to ~30/s** on the link stream
  (1791 in 60 s; runs A/B/C: 4–5 per run). A low-destination-fps 1:N bind
  on the same VPE port disturbs ch0's pts cadence enough to trip the
  vanish detector; link fps stayed 60. Unexplained. Do not ship a low-fps
  recorder without understanding it.
- Recording at a lower resolution than the link (VPE port1, the second
  scaler waybeam's `star6e_vpe_ports.c` drives) is the only route to a
  smaller per-frame penalty and was not tried.

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
(`low_power.enable = true` again), video verified up. Spike binary pruned
from the rootfs. Three recordings (~1.5 GB) left on the card.
