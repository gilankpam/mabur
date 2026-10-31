# colortrans — reversing the drone's ColorTrans sensor tuning on the GS

The drone's `venc.sensor_bin` (`imx415_greg_fpvXIX_colortrans.bin`) flattens the
picture on purpose (a 3x3 "ColorTrans" matrix plus a luma offset in the ISP)
so the encoder spends bits evenly. `maburplay` undoes it in three places (the
live picture by one of two mechanisms, whichever the display hardware has), all from
one evaluator, `gs/player/src/colortrans.{h,cpp}` (`kColorTrans3`, a
transcription of the operator's `colortrans3.glsl`); the web GS carries a
pinned JS/GLSL copy for its video:

| Where | Mechanism | Cost |
|---|---|---|
| Live picture (RK3566) | VOP2 CRTC `CUBIC_LUT`, 9x9x9, 12-bit entries, built by `build_cubic_lut()` and attached on every modeset commit (`drm_presenter.cpp`) | none (scanout hardware) |
| Live picture (H618) | the video plane's CSC: `affine_3x4()` folds `forward()` into one 3×4, attached as the DE33 VI plane's `CTM` property (Orange Pi image kernel patch `0002`), which the kernel combines with its YUV→RGB table. The OSD plane never passes through it, so the OSD is not pre-inverted. Used only when the CRTC has no CUBIC_LUT; a kernel that refuses the matrix (out of register range) costs one log line and a flat picture (`DrmPresenter: kernel refused the video-plane CTM`) | none (scanout hardware) |
| OSD (both overlays) | RK3566 only (the LUT covers the OSD planes too): pre-inverted at the source: `OsdFont::set_inverse()` on the MSP atlas, `set_colour_inverse()` in `gs_draw` for the GS tokens and shadow | once at startup |
| Burned DVR | `FrameColorTrans` (`frame_colortrans.cpp`): NV12 dmabuf -> GLES2 shader (colortrans3 verbatim) -> ARGB GBM target -> RGA -> NV12, on the recorder's ct thread, pipelined with `encode()` (see "Keeping the burned stage off the display path" below) | ct thread only; `burn_ctfb=` on the fps-log counts frames that fell back to flat, `burn_ct_ms=`/`burn_enc_ms=` are the per-window mean stage times |
| Web GS video (`web/`) | `web/ui/src/lib/colortrans.js`: each decoded `VideoFrame` -> `texImage2D` -> the same shader on a WebGL canvas stacked over the flat 2D one; the MSP OSD is its own canvas above, so nothing is pre-inverted. Page toggle, see `docs/web-gs.md` ("Colour correction") | one GPU pass per frame; off = the plain 2D `drawImage` path |

Config: `[colortrans] enable = true|false` in `maburplay.toml` (default false in
code, true in the bundle). Retuning = edit `kColorTrans3` in `colortrans.cpp`
AND the constants in `kFrag` in `frame_colortrans.cpp`, rebuild, redeploy — AND `CT_PARAMS` in `web/ui/src/lib/colortrans.js`;
`tests/test_colortrans.cpp` pins the C++ side to the glsl reference values,
`web/tests/colortrans.test.mjs` pins the web copy to the same values and to
`colortrans.h`'s defaults (a retune that misses the page fails it).
The H618's 3×4 is derived from `kColorTrans3` at startup, so a retune needs
no extra edit there; a retune to `gamma != 1` is not affine and leaves the
H618 flat (logged).

`colortrans.enable` is also one of maburplay's live keys
(`127.0.0.1:8403` — `docs/gs-live-config.md`): since the LUT/CTM setters
above are call-once per presenter, a live toggle recreates the presenter
rather than flipping a flag on the running one.

The ct thread also SCALES: the burned DVR's picture is the recording's own
size (`dvr.burned.size`, default the decoded size when the recording
starts), and every frame of another size is scaled to it — the encoder
latches its picture size from the first frame and lays the OSD 1:1 from
the top-left, and the burned overlay is composed at the record size, so a
decoded size that changes mid-recording (a rung change) or a `size` set
apart from the drone's (a 720p drone recorded at 1080p) keeps recording
with the OSD in place. With colortrans the shader draw scales (target =
record size, LINEAR sampler); without it, one RGA NV12 resize. The
encoder also latches the first frame's strides, so a frame whose
geometry (size or strides) differs from what the encoder latched goes
through the same path — at equal size that is a restride copy. So the ct
thread runs in every GPU build, colortrans or not. A picture of another
aspect ratio than the record size (1728x816 into 1920x1080) is
letterboxed like the screen shows it (`fit_rect.h`):
the shader clears to black and draws into the centred sub-rectangle; the
RGA path fills the bars once per destination buffer and scales into it.
A frame the stage cannot conform (both the shader and RGA failed, or the
destination buffers could not be allocated) is dropped and counted in
`burn_drop`, never recorded at another geometry: forwarded, it would fix
the encoder's latch at the wrong size and the record-size OSD would be
refused for the whole file.

## Kernel facts — RK3566 (GS kernel 6.1.84, `rockchip_drm_vop2.c`)

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
  persistently, or set `[colortrans] enable = false` to turn the whole
  feature off. The permanent fix is to change the
  default in `main.cpp` (`lut_axis = LutAxis::kBlueFastest`), update the
  `axis=` string in the same log line, rebuild, and redeploy.

## Kernel facts — H618 (Orange Pi image kernel 6.18.35, `sun8i_csc.c`)

- No `CUBIC_LUT`. Each VI channel has a 15-word CCSC: three pre-offsets
  (10-bit units, subtracted), a 3×3 in S2.17, and a post-constant per
  row. The image kernel's patch `0002` adds a `CTM` blob property to the
  VI planes: 12 × u64, row-major `[A | b]`, S31.32 sign-magnitude, on RGB
  in [0,1], folded in as `C' = A·C`. The post-constants cannot carry a
  negative offset (set to −0.345 of full scale the picture saturates to
  white), so the offset `b` goes into the **luma pre-offset** instead:
  raising `d_Y` by Δ shifts every output row by `−C'[r][Y]·Δ`. That holds
  `b` exactly when it is a grey shift, which colortrans's lift is
  (`kColorTrans3`: `d_Y` 64 → 190, U/V stay 512, post-constants stay 0).
  Unset = the stock table, bit for bit. The plane's default encoding is
  BT.709 limited.
- Coefficients are signed 20-bit: |coefficient| < 4.0, and a larger one
  wraps rather than saturates (black picture, or a colour cast that
  flips). `kColorTrans3` combined with BT.709 limited peaks at 3.41
  (Cb→B). The output clamps at full scale. A CTM that does not fit — a
  coefficient of 4.0 or more, an offset that is not a grey shift, a `d_Y`
  outside 0..1023 — fails the commit with `-ERANGE`.
- Plane state outlives the process that set it: a `CTM` left on the plane
  by one run stays applied under the next. So `maburplay` states the
  property on every modeset and splash-handoff commit — the matrix, or 0
  when colortrans is off or the kernel refused it — the same reason a LUT
  CRTC always gets a table.
- The 3×4 skips `forward()`'s clamp after the camera matrix: same mean
  error as the RK3566's 9×9×9 LUT (4.0 vs 3.7 LSB), worse on strongly
  saturated colours (max 36 vs 20 LSB). Record:
  `docs/findings/2026-10-12-opi-colortrans-probe.md`.

## Build

maburplay is glibc-dynamic (Mesa EGL/GBM cannot be linked
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

`out/arm64/maburplay-static` is the musl build from stage 5 (no GPU stage;
with colortrans on it records flat and `burn_ctfb` climbs) — the rollback
build. maburgs stays static.

Confirm the dynamic build's actual link list with:

    ../sbc-groundstations-gilankpam/output/radxa_zero3_defconfig/host/bin/aarch64-none-linux-gnu-readelf -d out/arm64/maburplay | grep NEEDED

which on the current build reads: `libEGL.so.1`, `libGLESv2.so.2`,
`libgbm.so.1`, `librga.so.2`, `libdrm.so.2`, `librockchip_mpp.so.1`, plus the
usual `libstdc++.so.6`, `libm.so.6`, `libgcc_s.so.1`, `libc.so.6`, and the
loader `ld-linux-aarch64.so.1`.

## Keeping the burned stage off the display path

A recording must not cost the live picture a vsync. What makes that hold:

- **Explicit in-fence on video-plane commits.** The GPU stage imports the
  decoder's NV12 dmabuf, and panfrost attaches a write fence to every BO in
  a job; through the shared `dma_resv` the KMS commit worker would wait on a
  job that only reads. `DrmPresenter` mints one already-signalled
  `sync_file` (a `DRM_SYNCOBJ_CREATE_SIGNALED` syncobj on the panfrost
  render node; rockchip-drm lacks `DRIVER_SYNCOBJ`) and sets it as
  `IN_FENCE_FD` on every video-plane commit, so the plane helper takes no
  implicit fence. Startup log:
  `DrmPresenter: video plane commits carry an explicit IN_FENCE_FD (...)`.
- **An O(1) tap on the flip.** `BurnRecorder::submit()` runs on the
  `SCHED_FIFO` release thread (docs/phase-lock.md "Threads"), from the
  display's flip through `RecorderTap` (`recorder_tap.h`), on each
  recorded slot of the vblank grid: one buffer reference and a mailbox
  post under a `PiMutex`, never a copy and never a buffer release. A
  displaced mailbox entry waits on a two-deep list for the stage thread to
  release it; with the list full, or four decoder buffers already held by
  the recorder (counted, in every mode), the flip takes no reference and
  counts as dropped. Record start and stop install and remove the hook from the
  housekeeping loop (`RecorderTap::set`/`clear`, cleared before `stop()`),
  so neither runs on the release thread.
- **Pipelined ct thread.** The stage runs on its own thread (EGL is
  thread-bound) with three destination buffers and a latest-wins handoff to
  the encode thread (`burn_recorder.h`, THREADING).
- **GPU governor pinned to `performance`** for the stage's lifetime
  (simple_ondemand never clocks up at a pipelined stage's utilisation);
  restored at stop, logged both ways (`FrameColorTrans: GPU governor …`).
- **Import cache.** The EGLImage import of the decoder's dma-buf is cached
  per source buffer (the decoder pool is 24 fixed buffers), so imports
  happen once per buffer per recording, not per frame.
- **Palette memoised at boot** (`maburplay: burn palette built: N entries`),
  so a record start does not histogram the MSP atlas on the housekeeping
  loop. A record start/stop still costs a one-shot GPU bring-up/teardown.
- **`dvr.burned.every`: counted thinning on the vblank grid**
  (`gs/player/src/glass_cadence.h`, `test_glass_cadence`): the recorder is
  fed from the display's flip, and one flip per `every` vblank periods is
  recorded, stamped with its ideal grid time; the encoder's rate is
  `round(1e6 / (period × every))`. No wall clock or rate estimate decides
  which frames go in, so the file samples exactly what the glass showed.
- **File I/O off the encode thread.** The mux writes through
  `mabur::ThreadedSink` (`common/include/mabur/threaded_sink.h`,
  `test_threaded_sink`). A closing fragment is ~2.5 MB at 20 Mbit/s and
  takes ~22 ms to write to the GS's vfat card. The encode thread's budget at
  60 fps is 16.7 ms with ~14 ms of encode, so written in line it would
  displace the next frame at the `ct_out` handoff, once per fragment. The
  writer thread (`mbr-dvrw`) trickles: each queued write reaches the file
  in 64 KB slices with an 8 ms sleep between them (`ThreadedSinkPacing`),
  so a fragment's kernel work (the page-cache copy and a FAT buffer head per
  512-byte sector) is spread over ~320 ms instead of landing in one ~22 ms
  burst. The burst was the live picture's cost: with the encoder, the GPU
  stage, the decoder and the writer all busy at once, the loop that
  released frames woke a few ms late, or its commit ioctl or the decode ran
  long, and the frame missed its vblank about once per 45-90 s of
  recording, measured with frames released from a normal-priority loop;
  the release thread runs `SCHED_FIFO` (`display.rt_priority`). Lowering the
  writer's priority does not prevent that (a thread that just woke from a
  long sleep keeps the CPU against the loop's wake-up until the tick), so
  the writer runs 10 nice steps below the player only as a tie-breaker.
  The gaps stop while the queue holds 4 MB or more (the card is slower than
  the trickle: catch up) and during the sink's destructor (record stop
  drains it on the housekeeping loop). Record of the search and the gate:
  `docs/findings/2026-10-19-burned-dvr-live-repeat.md`.

`BurnRecorder: stopped` reports `ct_ms=mean/max enc_ms=mean/max`; the ct
thread logs `colortrans stage mean per frame ...: import, draw+finish, rga`
at stop. Bench affordance: `kill -USR1 $(pidof maburplay)` toggles a
recording exactly like the GPIO button, so a mid-session record start can be
measured without a hand on the GS.

Bench record of the three fixes and their numbers:
`docs/findings/2026-09-17-colortrans-recording-latency.md`.

## Known limitations

- **The burned recorder cannot record every vblank at 100 Hz.** The period
  is 10 ms and both stages are slower at 1728x972: the colour stage takes
  ~15 ms a frame (GPU draw ~9.5 + RGA ~5.3) and `encode()` ~11.4 ms
  (rkvenc stays at 297 MHz: `venc_ondemand` never raises it). With
  `dvr.burned.every = 1` on a 100 Hz panel the latest-wins mailboxes drop
  20-35 % of the flips for the whole recording (`burn_drop=` climbs
  ~20-30/s), so the file stutters. Use `every = 2` (50 fps) at 100 Hz.
  With colortrans off and the rkvenc governor set to `performance` by hand
  (400 MHz, ~9.2 ms) every vblank fits; the player sets no encoder clock.
- **The colour stage costs live view at 100 fps.** Its GPU and RGA memory
  traffic slows the hardware decoder (`dec` p50 +1.5-2 ms, p99 roughly
  doubled), the regulator's allowance follows the wider tail, and e2e p50
  rises ~4 ms while recording (~3.5 ms at `every = 2`). With colortrans off
  the recording costs live view nothing measurable. At 60 fps the stage
  fits and the cost is ~0.5 ms of `dec`.

Measurements: `docs/findings/2026-10-22-burned-dvr-100fps-throughput.md`.

## Not done / follow-ups

- A record start or stop blocks the housekeeping loop (OSD, stats; not the
  decode-output or release thread) for 70-180 ms: `BurnRecorder::start()`
  (encoder init, palette upload) and `stop()` (thread joins, encoder
  teardown, mux close) run synchronously from `rec_start`/`rec_stop`.
- Live retune (config + restart is the loop).
- EGL YUV colour-space / range hints on the dmabuf import (none are passed;
  a mismatch is a slight tint between the display and the DVR).
- Raw DVR mode is the bitstream and stays flat: post-process with
  `ffmpeg -vf "libplacebo=custom_shader_path=colortrans3.glsl"`.
- The splash is not inverted.
- The stage is memory-bound (GPU + RGA traffic per 1080p frame); feeding
  the encoder BGRA directly and deleting the RGA pass is the untested next
  lever.
