# Orange Pi Zero 2W ground station (Allwinner H618)

The Orange Pi Zero 2W runs the same GS as the Radxa ZERO 3 — `maburgs`
(same source, its own glibc build `out/arm64/maburgs-opi` against the OPi
sysroot) and `maburplay` — with a different decoder:
Cedrus, the H618's V4L2 stateless (request API) HEVC decoder, driven
through libav's `v4l2request` hwaccel.

## Decode path

`backend = "v4l2req"` selects `V4l2ReqBackend`
(`gs/player/src/v4l2req_backend.cpp`, built with
`-DMABUR_PLAYER_DECODER=v4l2req`). An access unit from the ring either goes
to `avcodec_send_packet` whole (no parser) or, by default, slice by slice
("Streamed decode" below); decoded frames come back as DRM-PRIME linear
NV12 either way and are handed to `DrmPresenter` unchanged.

- **One thread for libav.** Decode and frame frees run on the feed
  thread; `poll()`/`release_frame()` on main only touch a locked handoff
  (`frame_handoff.h`). Released frames are freed before the next decode.
  `abort()` can also run on main while the feed thread is parked, same as
  `flush()`.
- **Entry gate.** After init and every flush, AUs are dropped until the
  first entry point (`mabur::au_is_entry_point`: an IRAP, or an AU carrying
  an SPS), counted as `gated`. With `venc.gop_s` nonzero the drone's
  periodic refresh AU (VPS/SPS/PPS + TRAIL_R, every `venc.gop_s`) is where
  a mid-session join usually starts; the bundle ships `venc.gop_s = 0`
  (no periodic refresh), so a join now waits for the next real IRAP
  instead — a rung change, link-up, chain break, or maburgs's own
  demand-refresh request (`docs/fec-nack.md` "Demand refresh"). Joining
  on a bare P picture would make Cedrus reference never-decoded buffers.
  Not re-armed on decode errors.
- **Synchronous.** The hwaccel waits for the capture buffer before
  returning a frame. Whole-AU decode (`stream = false`) blocks the feed
  thread in `submit_au` for the whole decode; when streaming (the
  default), each slice's `append()`/`start()` returns as soon as libav has
  queued it to Cedrus, and only the picture's last `append()` — the one
  that closes it — waits for the decode ("Streamed decode" below).
- **Shape check.** A frame not in single-object linear NV12 is dropped and
  counted (`shape_drops`); the first one logs its descriptor.

Counters: `--decode-only` JSON's `errors`/`errors_after_sync_3s` and the
`maburplay: v4l2req ...` / `v4l2req: frames= errors= bitstream= shape_drops=
gated=` lines (the latter every 5 s in `/tmp/maburplay.log`) both name
`errors` and `bitstream` separately. `errors` is hard failures only --
device/alloc failure, a rejected packet, a frame that comes back other
than DRM-PRIME -- the same meaning `errors_after_sync_3s` has
for MPP; a streamed picture that `start()` ends over an open one it
never closed (the new-picture-over-an-open-one guard) also counts there.
`bitstream` is `AVERROR_INVALIDDATA` from send/receive: bitstream
damage from a lossy link, not a decoder fault, and NOT included in
`errors`.

## Streamed decode

`[decoder] stream = true` (the default) streams on this board too.
`V4l2ReqBackend` is a `StreamDecoder`: `StreamFeeder` hands it the first
slice (with the NAL units before it), then one slice per call, and libav
gets one packet per slice — `AV_PKT_FLAG_PARTIAL` on every packet but the
last. mabur's FFmpeg patch (`gs/player/ffmpeg-patches/`, carried by the
image's `ffmpeg-v4l2request`) keeps the picture open across packets and
queues each slice to Cedrus as its packet arrives, holding the capture
buffer, so only the last slice's decode follows the AU's completion. An
abort sends one AUD NAL: libav ends the picture with what it has and the
held capture buffer is released (`V4L2_DEC_CMD_FLUSH`).

Against whole-AU decode on the same build, streaming takes the `lat:` `dec`
segment from 7 to 4 ms at the median (10 to 6 ms p99) on the bench link's
4-slice pictures; about 96 % of AUs stream, the rest being one-slice AUs
(`docs/findings/2026-10-12-opi-zero2w-stream-ab.md`).

The patch is required: `v4l2req_backend.cpp` does not compile without
`AV_PKT_FLAG_PARTIAL`, and `init()` fails (`libavcodec lacks the mabur
stream patch`) on a libavcodec without the `stream_slices` option.
`[decoder] stream = false` decodes whole AUs on the same library.

A hard (non-bitstream) decoder error in the middle of a streamed picture
can leave Cedrus holding that picture's capture buffer, costing up to one
2 s capture wait before decode resumes; whole-AU decode on Cedrus has the
same exposure.

`out/arm64/v4l2req_streamcheck <capture.265> [--abort N]` (stage 7) checks
on the board that streamed and whole decode give identical pictures; the
capture comes from `tools/bench/ausniff.py --oneshot --dump-annexb`.

## Not available on this board

| Feature | Why | Config |
|---|---|---|
| Burned DVR | no hardware encoder in mainline | `dvr.mode = "burned"` exits 2 at startup |
| Record button | the H618's GPIO lines carry no names, so `[input.rec]` cannot resolve | stripped from the image's default |

**Raw DVR is the only DVR here, and it starts on a real IRAP.** `RawDvr`
opens a file only at an IRAP (`docs/observability.md`), which the drone
sends at session start and on an encoder reconfiguration (a rate change),
not at every refresh. A recording started mid-session therefore waits
for the next one; with no record button, `dvr.autostart = true` is the
way to record a whole flight.

## Display

`DrmPresenter` probes the DE33 like any other CRTC. On a 1080p60 HDMI sink
the topology is:

- video on the overlay NV12 (VI) plane at zpos 1, scaled to the mode
  (the image's DE33 scaler kernel patch,
  `board/orangepi/zero2w/linux-patches/` in the image repo). NV12 goes
  through the scaler even at 1:1, for chroma upsampling, and its
  coefficient tables sit in SRAM that drops CPU writes unless the DE
  top's LUT access bit (`0xe0` bit 0, clock register space) is set; the
  image kernel sets it around those writes (patch `0004`). A kernel
  without that patch scans out with wrong colours after about half of all
  cold boots (grey reads white/green, the luma high-passed), and a soft
  reboot keeps the bad tables;
- the OSD on the primary plane (ARGB8888 linear, full mode) at zpos 2,
  its transparent buffer doubling as the backdrop;
- no `CUBIC_LUT` on the CRTC; colortrans runs in the video plane's CSC
  through the image kernel's `CTM` plane property (`docs/colortrans.md`,
  "Live picture (H618)"). The OSD plane is untouched by it.

`PAGE_FLIP_ASYNC` is refused (EINVAL), so flips are vsync-paced; the
regulator's vsync servo locks and runs in paced mode. Page-flip
timestamps are kernel CLOCK_MONOTONIC (`DRM_CAP_TIMESTAMP_MONOTONIC`, as
on every DRM driver), so the latency log's `dsp` segment is exact.

Decode keeps up with 1080p60: Cedrus decodes a whole AU in about 7 ms
(`lat:` `dec`; 4 ms after completion when streamed, "Streamed decode"),
and the GS OSD layer costs no measurable frame rate.
Bench record: `docs/findings/2026-10-12-opi-zero2w-bringup.md`.

## Build and deploy

`tools/build-arm64.sh` stage 7 builds `out/arm64/maburplay-opi` against
`../sbc-groundstations-gilankpam/output/orangepi_zero2w_defconfig`
(override with `MABUR_BR_HOST_OPI=`). NEEDED: `libavcodec.so.61`,
`libavutil`, `libdrm.so.2`, libstdc++. Stage 7 also needs that sysroot's
libavcodec to carry mabur's FFmpeg patch (`gs/player/ffmpeg-patches/`) and
builds `v4l2req_streamcheck` against it. Deploy: `docs/deploy.md`
("Orange Pi Zero 2W").

Display colortrans needs the image kernel's patch `0002` (the `CTM` plane
property); on a kernel without it maburplay logs `DrmPresenter: video
plane N CTM absent` and shows the picture flat.
