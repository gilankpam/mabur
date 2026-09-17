# colortrans — reversing the drone's ColorTrans sensor tuning on the GS

The drone's `venc.sensor_bin` (`imx415_greg_fpvXIX_colortrans.bin`) flattens the
picture on purpose (a 3x3 "ColorTrans" matrix plus a luma offset in the ISP)
so the encoder spends bits evenly. The GS undoes it in three places, all from
one evaluator, `gs/player/src/colortrans.{h,cpp}` (`kColorTrans3`, a
transcription of the operator's `colortrans3.glsl`):

| Where | Mechanism | Cost |
|---|---|---|
| Live picture | VOP2 CRTC `CUBIC_LUT`, 9x9x9, 12-bit entries, built by `build_cubic_lut()` and attached on every modeset commit (`drm_presenter.cpp`) | none (scanout hardware) |
| OSD (both overlays) | pre-inverted at the source: `OsdFont::set_inverse()` on the MSP atlas, `set_colour_inverse()` in `gs_draw` for the GS tokens and shadow | once at startup |
| Burned DVR | `FrameColorTrans` (`frame_colortrans.cpp`): NV12 dmabuf -> GLES2 shader (colortrans3 verbatim) -> ARGB GBM target -> RGA -> NV12, on the recorder's ct thread, pipelined with `encode()` (see "Latency" below) | ct thread only; `burn_ctfb=` on the fps-log counts frames that fell back to flat, `burn_ct_ms=`/`burn_enc_ms=` are the per-window mean stage times |

Config: `[colortrans] enable = true|false` in `maburplay.toml` (default false in
code, true in the bundle). Retuning = edit `kColorTrans3` in `colortrans.cpp`
AND the constants in `kFrag` in `frame_colortrans.cpp`, rebuild, redeploy;
`tests/test_colortrans.cpp` pins the C++ side to the glsl reference values.

## Kernel facts (GS kernel 6.1.84, `rockchip_drm_vop2.c`)

- Entries are 12-bit (`& 0xfff`), 729 per table; the blob is 729
  `struct drm_color_lut` (16-bit fields, only the low 12 bits used).
- The driver keeps a RAW pointer to the last applied blob's data and
  re-applies it on the next modeset even after the blob is destroyed;
  clearing the property to 0 does not disable the LUT. So maburplay ALWAYS
  installs a table (identity when `enable = false`) and keeps the blob alive
  for its whole run.
- No `CTM`/`DEGAMMA_LUT` on this CRTC; `GAMMA_LUT` (1024) is unused here.
- Axis order: **red varies fastest**, `index = r + 9g + 81b`
  (`gs/player/src/colortrans.cpp`, `cubic_lut_index()`, `LutAxis::kRedFastest`
  — the shipped default). Confirmed 2026-09-17 on the GS against real VOP2
  hardware: with the overlay up and the LUT installed
  (`DrmPresenter: CUBIC_LUT present (size 729)` followed by
  `colortrans: display LUT on (axis=rgb), OSD pre-inverted`), the REC
  marker read RED and body text read WHITE. A `kBlueFastest` table would
  have swapped the marker to blue and cast the text.
  Re-run this check on any new kernel — the axis is an undocumented VOP2
  implementation detail, not a spec guarantee. The check itself: boot the GS
  with the overlay up (the drone can be off) and look at the REC
  marker/"fault" colour; RED is correct, BLUE (or an obvious cast on body
  text) means the axis is inverted. Two field remedies need no rebuild —
  export `MABUR_COLORTRANS_AXIS=bgr` from `S97maburplay` to flip the axis
  persistently, or set `[colortrans] enable = false` for a clean total
  retreat to pre-branch behaviour. The permanent fix is to change the
  default in `main.cpp` (`lut_axis = LutAxis::kBlueFastest`), update the
  `axis=` string in the same log line, rebuild, and redeploy.

## Build

maburplay is glibc-dynamic since 2026-09-16 (Mesa EGL/GBM cannot be linked
statically). `tools/build-arm64.sh` stage 6 builds it with the Buildroot SDK
in `../sbc-groundstations-gilankpam/output/radxa_zero3_defconfig/host`
(`MABUR_BR_HOST` overrides). That sysroot must contain mesa3d + librga:

    cd ../sbc-groundstations-gilankpam
    make -C buildroot O=$PWD/output/radxa_zero3_defconfig BR2_EXTERNAL=$PWD radxa_zero3_defconfig
    make -j$(nproc) -C buildroot O=$PWD/output/radxa_zero3_defconfig BR2_EXTERNAL=$PWD mesa3d librga

The defconfig must also set `BR2_PACKAGE_MESA3D_LLVM=y` (already committed in
the sbc-groundstations repo). Without it, Buildroot's Kconfig **silently
drops** `BR2_PACKAGE_MESA3D_GALLIUM_DRIVER_PANFROST` — panfrost hard-`depends
on` LLVM in Buildroot 2025.08.1 — which in turn drops EGL, GBM and GLES from
the built sysroot. The `make mesa3d librga` step above still reports success;
there is no error, only a sysroot missing every header stage 6 links against.
Anyone regenerating this defconfig from scratch should set the LLVM symbol
explicitly before building, not just add panfrost/EGL/GLES and expect them to
stick.

Run `tools/build-arm64.sh` **bare** — do NOT wrap it in the
`nix-shell -p pkg-config libusb1` that this repo's own CLAUDE.md prescribes
for host cmake/ctest work. That wrapper does not apply to this script: when
`libusb1` is present in the shell, Nix's pkg-config wrapper overrides
`PKG_CONFIG_LIBDIR` unconditionally, which leaks the host's libusb into stage
6's Buildroot-sysroot build underneath it and silently points the glibc build
at host x86_64 headers instead of the Buildroot sysroot. The script detects
this leak and fails loudly with an explanatory message rather than let CMake
surface a confusing "path does not exist" error — but the safe move is to
just not wrap the script in the first place.

`out/arm64/maburplay-static` is the old musl build (no GPU stage; with
colortrans on it records flat and `burn_ctfb` climbs) — the rollback build.
maburgs is unchanged (static).

Confirm the dynamic build's actual link list with:

    ../sbc-groundstations-gilankpam/output/radxa_zero3_defconfig/host/bin/aarch64-none-linux-gnu-readelf -d out/arm64/maburplay | grep NEEDED

which on the current build reads: `libEGL.so.1`, `libGLESv2.so.2`,
`libgbm.so.1`, `librga.so.2`, `libdrm.so.2`, `librockchip_mpp.so.1`, plus the
usual `libstdc++.so.6`, `libm.so.6`, `libgcc_s.so.1`, `libc.so.6`, and the
loader `ld-linux-aarch64.so.1`.

## Latency while recording (2026-09-17)

The first build of this stage made `dsp` p99 sit at 22 ms (baseline 5) for
every second a recording ran: one vsync slip on ~10 % of flips. Three things
were found and fixed, all on the bench with the drone streaming:

1. **Implicit fence on the decoder's frame.** The GPU stage imports the
   decoder's NV12 dmabuf as a texture; panfrost (6.1) attaches a
   `DMA_RESV_USAGE_WRITE` fence to every BO in a job, the PRIME import shares
   one `dma_resv` between that texture and the KMS framebuffer of the same
   frame, VOP2 has no `prepare_fb` of its own, so `drm_gem_plane_helper_prepare_fb`
   picked the fence up and the commit worker blocked 7-15 ms on a job that only
   reads. ftrace showed 66 `dma_fence_wait` calls per 10 s in the DRM commit
   workers, p90 12.5 ms; zero with recording off. Fix: `DrmPresenter` mints
   one already-signalled `sync_file` (a `DRM_SYNCOBJ_CREATE_SIGNALED` syncobj
   on the panfrost render node, exported; rockchip-drm lacks
   `DRIVER_SYNCOBJ`) and sets it as `IN_FENCE_FD` on every video-plane
   commit — with an explicit fence the helper only takes KERNEL-usage fences,
   of which there are none. After: 576 waits per 10 s (one per commit, the
   signalled fence), max 79 µs, `dsp` 5/5. Log line at startup:
   `DrmPresenter: video plane commits carry an explicit IN_FENCE_FD (...)`.
2. **Serial pipeline.** GPU import 2.7 + draw/finish 14.6 + RGA 4.9 + encode
   13.0 ms = 35 ms per frame on one thread → 26-32 fps recorded, 30 % of
   admitted frames dropped, and the GPU idle enough that devfreq never left
   300 MHz. The stage now runs on its own thread (EGL is thread-bound) with
   three destination buffers and a latest-wins handoff to the encode thread
   (`burn_recorder.h`, THREADING). Two more shavings were needed once the
   admission cap (below) let the full 60 fps through: the stage pins the GPU
   devfreq governor to `performance` for its lifetime (simple_ondemand never
   clocks up at the ~60 % utilisation a pipelined stage shows; restored at
   stop, logged both ways), and the per-frame EGLImage import of the
   decoder's dma-buf is cached per (fd, inode) -- the pool is 24 fixed
   buffers, so imports happen 24 times per recording instead of 60 times a
   second. After all three: stage 12.2 ms mean (import 0, draw+finish 7.0,
   RGA 5.1) against 13.2 ms encode; 1770 admitted / 1760 encoded / 9 dropped
   over 30 s at a 58-60 fps source, `dsp` 5/5. The stage is memory-bound now
   (11 MB GPU + 11 MB RGA traffic per 1080p frame), so the next lever would
   be feeding the encoder BGRA directly and deleting the RGA pass, untested.
   `BurnRecorder: stopped` reports `ct_ms=mean/max enc_ms=mean/max`; the ct
   thread logs `colortrans stage mean per frame ...: import, draw+finish,
   rga` at stop.
3. **Record-start spike.** Every start blocked the main loop ~240 ms: 147 ms
   of it was `build_palette()` histogramming the whole MSP atlas, now built
   once at startup and memoised (`maburplay: burn palette built: N entries`
   at boot); the remaining ~75 ms is the first full OSD quantize (32-39 ms
   with a cold cache) plus the GPU bring-up (~165 ms on the ct thread, of
   which drm+gbm ~50, program ~65, targets ~30) and is accepted as a one-shot
   cost of pressing record. A similar ~80 ms one-shot happens at stop.

Bench affordance: `kill -USR1 $(pidof maburplay)` toggles a recording
exactly like the GPIO button, so a mid-session record start can be measured
without a hand on the GS.

Also fixed the same day, pre-existing: `dvr.burned.fps_cap = 60` admitted
only ~41-45 fps of a 60 fps stream — the cap rejected any frame arriving less
than `interval - interval/8` after the last admitted one, and decoded frames
arrive in pairs. The cap is now schedule-based (`gs/player/src/fps_cap.h`,
host-tested by `test_fps_cap`): a frame is admitted when its slot is due,
slots advance one interval per admit, so a source at or under the cap passes
whole and a faster one is thinned to exactly the cap.

## Not done / follow-ups

- Live retune (config + restart is the loop).
- EGL YUV colour-space / range hints on the dmabuf import (PixelPilot passes
  none; a mismatch is a slight tint between the display and the DVR).
- Raw DVR mode is the bitstream and stays flat: post-process with
  `ffmpeg -vf "libplacebo=custom_shader_path=colortrans3.glsl"`.
- The splash is not inverted.
