# H.265 slices on the SSC338Q: no early output — findings 2026-10-09

**Question.** With `MI_VENC_SetH265SliceSplit` on, does the SSC338Q VENC hand
out slice 1 of a picture before the last slice is encoded, and by how much?
If it did, maburd could FEC-encode and transmit the top of a frame while the
bottom was still in the encoder, which is the only latency lever slices
could offer (the GS decoder is per-picture regardless — MPP builds one
rkvdec job per picture, see below).

**Answer: no.** The encoder finishes the whole picture before any NAL is
readable. Per-packet delivery (`byFrame = 0`) hands the first NAL out 0.45 ms
before by-frame delivery would have handed out the frame, and the last NAL
0.2 ms after; the ~0.7 ms spread between a frame's first and last NAL is
driver spooling of an already-finished picture, not encode progress. The
transport rework that early output would need (a last-chunk flag instead of
the up-front chunk count in the fragment header, partial-AU ring slots)
would buy nothing on this SoC. **Drop the latency idea.**

Two secondary results, both useful if slices are ever wanted for loss
confinement instead:

- **In the shipped by-frame mode the split works and the plumbing needs no
  change.** The SDK concatenates all N slices into the ONE VCL `packetInfo`
  entry (3 inner start codes per frame at 4 slices, 16 at 17), so the frame
  ring, the 8-entry table cap, the TRAIL_N patch and the GS AU feed are all
  untouched. Delivery latency is unchanged at 4 slices and +0.22 ms at 17.
- **Per-packet mode is not fit for use.** It ignores the requested split and
  emits its own fixed 3-NAL geometry (slice_segment_address CTB 0 / 120 / 240
  = rows 0, 4, 8; byte shares 46 / 49 / 5 %), and in 5–50 % of frames the
  2nd or 3rd slice's `nal_unit_type` reads TRAIL_N (0) while the 1st reads
  TRAIL_R (1) — spec-invalid (all VCL NALs of a picture share a type), never
  on slice 1, never in by-frame mode, and more often the slower the consumer
  fetches. The bytes themselves are stable once handed out (0 of 3544 NALs
  changed in the 1 ms after `GetStream` returned).

## Setup

Bench drone (SSC338Q, `/etc/mabur.toml` = flight config, 1920x1080@60,
CBR, SVC-T refPred, GDR rows 34, SD recorder idle), no GS, so the bitrate
sat at the drone default and no RCF traffic arrived. Throwaway maburd on
branch `spike-venc-slices` (env knobs `MABUR_SPIKE_LOG`,
`MABUR_SPIKE_SLICE_ROWS`, `MABUR_SPIKE_BYFRAME`, `MABUR_SPIKE_RECHECK`) run
by hand from `/tmp` with `S00mabur` stopped; the installed binary and config
were never touched. Each case ~28 s of frames (1700 frames) after the ~10 s
MI bring-up. Logged per `MI_VENC_GetStream` return: CLOCK_MONOTONIC,
`MI_SYS_GetCurPts`, every pack's pts / endFrame / length / NAL table, the
first 8 bytes of every NAL (slice address parse), inner start codes per table
entry. Idle poll 200 µs (the production loop polls at 1 ms).

`u32SliceRowCount` is 32-px rows per slice; the SDK rounds up to CTU-64
rows. 1080p = 17 CTU rows. rows32=10 → 5-row slices → 4 slices; rows32=2 →
1-row slices → 17 slices. The readback (`MI_VENC_GetH265SliceSplit`) echoed
the request in every case.

## Results

Encode latency = `MI_SYS_GetCurPts − pack pts` at the moment `GetStream`
returned (same clock, no bridge). The baseline reproduces the 7.1 ms `enc`
segment of docs/latency-budget-findings-2026-08-31.md exactly.

| case | mode | VCL NALs delivered / frame | first NAL (ms after pts, p50) | last NAL | spread p50 / p90 |
|---|---|---|---|---|---|
| no split | by-frame | 1 | 7.17 | 7.17 | 0 |
| rows32=10 (4 slices) | by-frame | 1 table entry holding 4 slices | 7.17 | 7.17 | 0 |
| rows32=2 (17 slices) | by-frame | 1 table entry holding 17 slices | 7.39 | 7.39 | 0 |
| rows32=10 | per-packet | 3 (SDK's own geometry) | 6.71 | 7.42 | 0.75 / 0.99 |
| rows32=4 | per-packet | 3 | 6.71 | 7.38 | 0.68 / 0.99 |
| rows32=2 | per-packet | 3 | 6.72 | 7.37 | 0.70 / 1.02 |

Per-packet arrival by NAL index (p50, ms after pts): 6.71 → 7.16 → 7.38.
The gap to the previous NAL is ~450 µs for a ~15 kB NAL and ~215 µs for the
~1.5 kB tail NAL, i.e. roughly 200 µs + 17 µs/kB — a per-packet software
copy, not encode time (the VENC cannot encode 4 CTU rows of 1080p in
450 µs). With a deliberate 1 ms stall after each fetch (`RECHECK`), the next
NAL still arrived ~450 µs after the *fetch*, not at a fixed time after pts:
the driver prepares packet k+1 only after packet k is taken. Per-packet mode
therefore delivers the complete frame 0.2 ms LATER than by-frame mode.

Frames that are not split in either mode: the GDR refresh-start frames
(TRAIL_R + VPS/SPS/PPS, `refType` 2, one every 0.5 s GOP) come out as a
single slice; IDRs do split.

Bytes per frame are pinned by CBR (p50 25.6–26.8 kB in every case) so the
slice overhead cannot be read from size, and the SDK's `h265Info`
`startQual` / CU census fields are all zero on this SDK (dead), so the
quality cost of N slices is **unmeasured**. The slice headers themselves cost
~(N−1) × 10 B; the real cost is intra/MV prediction stopping at each
boundary.

## Is the SDK hiding an early-output path? Checked — no

Follow-up the same day, after the question "are we missing something in the
SDK". Evidence from the shipped OpenIPC infinity6e drivers
(`openipc-builder/.../sigmastar-osdrv-infinity6e/files/kmod/{mi_venc,mhal}.ko`),
the SigmaStar Pudding MI_VENC API page, and the live `/proc/mi_modules/mi_venc/mi_venc0`
node plus `dmesg` on the drone:

- **The vendor doc promises per-slice availability.** The SliceSplit note
  (H.264 text, H.265 refers to it): "when bSplitEnable is true, each
  u32SliceRowCount rows are encoded as a separate slice and *as soon as each
  slice finishes encoding the user can obtain that slice's stream data*; the
  smaller the row count the more frequent the notification". `bByFrame`:
  TRUE = fetch by frame, FALSE = fetch by packet; `bFrameEnd` marks the
  last packet of a frame.
- **The H.265 encoder is a Chips&Media WAVE5 core** (mhal: `Wave5VpuEncode`,
  firmware `/etc/firmware/chagall.bin`) with a hardware low-latency interrupt
  (`INT_WAVE5_ENC_LOW_LATENCY`). `mi_venc.ko` has `_MI_VENC_EnableSliceIrq`
  (called from `MI_VENC_IMPL_StartRecvPicEx`) and a per-device
  `IsrSliceDoneCnt` in the proc node.
- **The slice interrupt is only armed in per-packet mode.** Split on +
  by-frame: `IsrSliceDoneCnt` stays 0 (1 frame-done IRQ per frame). Split on
  + per-packet: 2 slice IRQs per frame (`IsrSliceDoneCnt` 2130 over 1101
  frames) and two `INT_WAVE5_ENC_LOW_LATENCY ll=1` kernel-log lines per
  frame whose chunk sizes equal the first two packs' NAL sizes. So the
  per-packet probe above *was* the hardware early-output path; the driver
  caps it at 3 packs per frame regardless of the requested rows.
- **The hardware itself raises slice 1 only ~0.7 ms before the frame is
  done.** With `printk` timestamps bridged to the spike's clock via
  `/dev/kmsg` markers, 1080p60 at the flight bitrate (25 kB frames,
  rows32=10 → 4-row slices), p50 after pts:

  | event | ms after pts |
  |---|---|
  | slice-1 IRQ (hardware) | 6.59 |
  | pack 0 fetched by maburd | 6.83 |
  | slice-2 IRQ (hardware) | 7.10 |
  | pack 1 fetched | 7.32 |
  | last pack fetched (frame done + spool) | 7.54 |
  | by-frame delivery, same config | 7.17 |

  The slice-1→slice-2 gap scales with slice bytes (190 µs at 1.5 kB, 510 µs
  at 12 kB ≈ 40 µs/kB): what the interrupts mark is the bitstream-writing
  tail of the encode. The first ~6.5 ms of the ~7 ms produce no slice at
  all — frame-level work inside the WAVE5 core/firmware. The MI driver adds
  only ~0.25 ms on top of the IRQ. A perfect driver would gain ≤0.7 ms.
- **Input-side pipelining is not available for H.265 here.** The other MI
  low-latency lever is `MI_VENC_SetInputSourceConfig` ring input (VPE
  writes while VENC reads the same buffer, bind type HW_RING). The doc's
  bind-type table says Pudding (this chip family) H.264/H.265 HW_RING = N,
  REALTIME = N, and only JPEG = Y for both; the proc node agrees:
  `SupportRing` 0 on dev 0 (the H.264/H.265 device, where chn 0/1 live) and
  1 only on dev 1 (the JPEG engine, chn 7). Macaron/Ispahan/Tiramisu do
  list H.264/H.265 HW_RING. The driver also refuses ring + split together
  ("chn is slice mode, can not set Ringmode").
- Module params of `mi_venc.ko` are only debug/limits
  (`max_h26x_task`, `use_ring_ref`, `thread_priority`, discard/reencode
  dqp…); `MI_VENC_ParamModH265e_t` offers `u32OneStreamBuffer`
  (multi/single packet buffer) and `u32H265eMiniBufMode` — neither changes
  when the hardware finishes a slice.

So: nothing missed. The SDK's early-output mechanism exists, fires, and is
worth ~0.7 ms on this core at 1080p60 — below the ~1 ms of jitter the
transport already carries.

## What this means for mabur

- Slices cannot shorten the drone's encode→air path. The 7 ms `enc` segment
  is upstream of NAL emission and ends with the whole picture.
- The GS cannot use them for latency either: MPP's HEVC path parses every
  slice and submits one rkvdec job per picture; `gs/player/src/mpp_backend.cpp`
  feeds whole AUs with the parser split mode off, and with it on MPP would
  only detect the picture end on the *next* picture's first slice.
- The one thing left is loss confinement, and it is cheap: one
  `MI_VENC_SetH265SliceSplit` call in the CreateChn→StartRecvPic window plus
  a config key, no GS change. mabur's loss mode is AU truncation (a tail cut
  past the FEC/NACK budget), so with N row slices a truncated AU keeps its
  top slices decodable and the damage shrinks to the bottom band — except on
  the unsplit refresh-start frames. Whether that is worth the unmeasured
  quality cost needs a loss-sim A/B on the player (`MABUR_LOSS_SIM`), 1 vs 4
  slices, judged on visible damage; the QP cost would need a slice-header
  parse since the SDK stats are dead.

## Flight logs: what slices would actually save (2026-10-09)

AU logs of sessions 0026, 0028–0033 (flights by their RSSI dynamics;
0027 and 0034–0039 are bench runs). `flags & 0x80` = complete; a truncated
AU's `len` against the complete AUs around it gives the head fraction.

| | base (sid 0) | enh (sid 1) |
|---|---|---|
| AUs | 78 k | 77 k |
| truncated | 145 (0.19 %) | 445 (0.55 %) |
| head < 10 % / 10–50 % / 50–90 % / ≥ 90 % | 15 / 89 / 32 / 10 | 55 / 233 / 122 / 35 |
| head fraction p50 | ~0.35 | ~0.35 |

Whole frames never assembled: 504 across the seven flights, 334 of them in
0032 (the past-range flight). About one truncated base AU every 20–60 s in a
normal flight.

**maburplay drops every truncated base AU whole** (`gs/player/src/main.cpp`,
`truncated_skipped`): submitting one hangs rkvdec2 — a truncated slice
declares more bitstream than exists, the VPU waits, the kernel resets the
session. So today each of those 145 events is a base frame gone and the
frames after it predicting from a missing reference until the 0.5 s
refresh. With row slices and a GS that submits only complete slices, a
median third of each such frame (over half in 29 % of cases) would be
shown. Not in the logs: whether pieces after the hole arrived too (one
counter in `FrameStream::finish`).

## fpvOS slice-streaming decode (gehee/fpvOS, 2026-10-09)

`br-external/package/rockchip-mpp/0002-h265d-decode-a-picture-as-its-slices-arrive.patch`
(+0001, 0003) with the kernel patch
`br-external/board/vrxpro/linux-patches/0003-video-rockchip-mpp-rkvdec2-stream-mode.patch`
and the consumer `gehee/kestrel-gnd` (`vdec/vdec_rk.cpp`,
`artosyn/ar8030_source.cpp`). RK3568 goggles, Rockchip 6.12 kernel, MPP
pinned at rockchip-linux/mpp `ed377c99`.

What it does: an undocumented rkvdec2 "stream mode" (h26x_stream_mode set,
lastpacket clear) makes the decoder pause on buf_empty instead of erroring,
and resume when the next part is written and dec_e re-armed with
dec_e_rewrite_valid. The kernel exposes `MPP_CMD_STREAM_APPEND` (+ PROBE,
WAIT); MPP exposes `MPP_PACKET_FLAG_STREAM_START` + `MPP_PACKET_STREAM_SLICES(n)`
on the first slice's packet, `MPP_DEC_SET_STREAM_APPEND` per later slice
(`MPP_STREAM_APPEND_LAST` on the last; **no data + LAST ends a picture
whose rest is lost** — the kernel pads 64 zero bytes and the picture
finishes with what it has, flagged), and `MPP_DEC_GET_STREAM_TOP` to show the
top half before the bottom arrives. Measured on RK3568 at 1080p100: bit-exact
against whole-picture decode, last slice → frame 2.0 ms vs 5.1 ms. The
kernel's `rkvdec2_stream_wait_ms` (400 ms) and `stall_ms` (50 ms) are the
safety nets; link mode is turned off (one task at a time). kestrel drops a
slice whose picture's first slice was lost (a missing first slice = picture
dropped), and holds middle slices to send in order.

Fit for our GS (Radxa Zero 3, RK3566, `RKVDEC HW_ID 0x032a3f03` =
`HWID_VDPU34X`, the same HAL the patch targets):

- kernel patch: `patch --dry-run` against the image repo's Radxa 6.1 tree
  (`sbc-groundstations-gilankpam/output/radxa_zero3_defconfig/build/linux-custom`)
  — 14 hunks, 0 failed, offsets only. `rk_vcodec` is built in
  (`CONFIG_ROCKCHIP_MPP_RKVDEC2=y`), so it is a kernel Image rebuild; the GS
  boots `/boot/Image` via extlinux on an overlay rootfs, so a rebuilt Image
  can be dropped in and the old one kept beside it.
- MPP 0002 against the image's MPP (HermanChen/mpp develop, 2026-01-14):
  3 of 20 hunks fail, all pure additions in headers (`rk_mpi_cmd.h`,
  `rk_vdec_cmd.h`, `h265d_parser.h`) — hand-port. 0003 applies after it;
  0001 needs a one-line context fix (`sps_need_upate`).
- maburplay does not set fast-parse mode, which the patch requires off.

What it would give mabur: (1) the resilience path — complete head slices in,
LAST with no data, picture out with the head decoded, no hang; (2) a GS-side
latency win independent of the SSC338Q result: decode overlaps the frame's
air serialization, saving most of the ~6 ms `dec` segment; (3) optional
early presentation of the top band. Cost: slices on the drone (by-frame
split + TRAIL_N/scan fixes), slice-aware reassembly in maburgs (resync at
start codes, in-order feed, drop the slice with the hole), the player feeding
slice by slice, a patched kernel + MPP in the GS image.

Test plan for the RK3566: port the three MPP patches, apply the kernel
patch, rebuild `linux` and `rockchip-mpp` in the image repo, stage
Image + `librockchip_mpp.so` on the GS (old Image kept, SD-card image as the
fallback boot), then `mpi_dec_stream_test` on a 4-slice 1080p60 capture from
the drone: stream 0 vs 1 bit-exact, `lose_every` for the lost-tail case,
`top 1` for the early-top timing.

## Where the code lives

- `spike-venc-slices` branch, one commit on top of master `b627b9a`:
  `drone/venc/star6e_pipeline.c` (byFrame + SetH265SliceSplit env knobs),
  `drone/venc/star6e_runtime.c` (logging, per-packet reassembly, recheck).
  Throwaway — do not merge.
- `MI_VENC_ParamH265SliceSplit_t` and the Set/Get loader binding already
  live on master (`drone/venc/star6e.h`, `star6e_mi.c`); nothing on master
  calls them.
- Analyzer: `spike_analyze.py` in the session scratchpad (not kept).

## RK3566 GS: fpvOS stream-mode decode works — spike 2026-10-09 (late)

Step 1 of the follow-up plan, run on the bench GS. **All three checks pass.**

Build: image-repo branch `spike-rkvdec2-stream` (sbc-groundstations-gilankpam).
The kernel patch went in as `board/radxa/zero3/linux-patches/0102-…` unchanged
(offsets only). The MPP patches needed more than the three header hunks
expected above: the image's MPP snapshot (HermanChen develop, `dl/` tarball
`develop-git4`) has `mpp.cpp`/`mpp_dec.cpp` converted to C, so 0002 got four
hunks hand-placed plus `mDec` → `mpp->mDec`; 0001 one context line; 0003 clean.
The patch leaves `Module.symvers` unchanged, and all imported-symbol CRCs of the
GS's loaded modules matched the rebuilt Image before it was flashed.

**How the GS boots — correction to the plan above.** U-Boot reads
`/boot/Image` out of the squashfs on p1 (`CONFIG_CMD_SQUASHFS`, p1 is the only
bootable partition), not through the overlay, and the kernel has no kexec. An
Image dropped beside the old one is never booted. The swap was done by
repacking the GS's own (CI-built) squashfs with only `/boot/Image` replaced
(`/boot/Image.orig` = the old kernel, file-by-file diff otherwise empty) and
writing it to p1; the backup of p1 is the recovery. The test MPP runs from
`/root/mpp-stream` via `LD_LIBRARY_PATH`. The system MPP and maburplay are
untouched, and live video decodes normally on the patched kernel with the
stock MPP. The GS's stock MPP is an *older* snapshot than the `dl/` one
(no `libmpp_ext.so`, where the newer one registers its codecs), so the image
needs an MPP pin before the patches can ship.

Input: 600 frames from the bench drone, spike binary by-frame +
`MABUR_SPIKE_SLICE_ROWS=10` + the new `MABUR_SPIKE_DUMP` (frames written as
the SDK hands them out, before the TRAIL_N rewrite), `bitrate_min_kbps 12000`,
1080p60. 582 pictures with 4 slices (byte shares p50 29 / 31 / 30 / 11 %; the
last slice is the 2-CTU-row bottom band), 18 single-slice (GDR refresh starts).
Picture p50 26.4 kB. Note: in this dump the SDK already writes TRAIL_N (type 0)
on *every* slice of the non-reference enhance pictures, never mixed with
TRAIL_R in one picture.

`mpi_dec_stream_test` (fpvOS), 1080p60, RK3566:

| check | result |
|---|---|
| streamed (slices 5 ms apart) vs whole | **bit-exact, 600/600 frames**; both also match an ffmpeg software decode 600/600 |
| lost tail, `lose_every 7` (empty LAST append) | no hang, 600/600 frames out; the first lost picture's top 952 rows bit-exact with the software decode; damage gone after p50 11 / max 28 frames (GDR) |
| lost tail, `lose_every -60` (nothing ends it) | no hang, the next picture's start ends it; 9/9 lost pictures' tops bit-exact; 599/600 frames out (one picture produced no frame) |
| last slice → frame, whole | 4.38 ms p50 / 4.80 p90 |
| last slice → frame, streamed (5 ms spread) | **1.69 ms p50 / 2.00 p90** (gap 0 control: 4.34, no overlap so no gain) |
| `top 1`: first slice sent → top slice decoded | 1.75 ms p50, 582/582 ready |

Every lost tail costs exactly one rkvdec reset (83/83, 10/10, 0 in clean runs).
The test feeds one picture at a time and waits for its frame, so it does not
exercise maburplay's pipelining. It also sets `disable_error`, so lost-tail
frames come out unflagged (0 "flagged with errors"); the player has to know
from its own feed that a picture was cut.
