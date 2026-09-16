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
| Burned DVR | `FrameColorTrans` (`frame_colortrans.cpp`): NV12 dmabuf -> GLES2 shader (colortrans3 verbatim) -> ARGB GBM target -> RGA -> NV12, on the recorder thread before `encode()` | recorder thread only; `burn_ctfb=` on the fps-log counts frames that fell back to flat |

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
- Axis order: **UNVERIFIED.** The code ships `LutAxis::kRedFastest` as the
  default — red varies fastest across the 729 entries, `index = r + 9g +
  81b` (`gs/player/src/colortrans.cpp`, `cubic_lut_index()`) — but this has
  not been confirmed against real VOP2 hardware; the bench probe that would
  confirm it (Task 13 step 2) is a live-device, by-eye check and was not run
  as part of the work that produced this page. **Do not assume the default
  is correct.** To confirm or correct it: boot the GS with the overlay up
  (the drone can be off) and look at the REC marker/"fault" colour. If it
  reads RED, the default is right and there is nothing to do. If it reads
  BLUE (or body text carries an obvious colour cast), the axis is inverted —
  restart with `MABUR_COLORTRANS_AXIS=bgr` (bench-only env override) and
  check again; if THAT reads correctly, the fix is to change the default in
  `main.cpp` (`lut_axis = LutAxis::kBlueFastest`), update the `axis=` string
  in the same log line, rebuild, and redeploy. Re-run this check on any new
  kernel — the axis is an undocumented VOP2 implementation detail, not a
  spec guarantee. Two cheaper field remedies exist before that rebuild round
  trip: export `MABUR_COLORTRANS_AXIS=bgr` from `S97maburplay` to flip the
  axis persistently with no rebuild, or set `[colortrans] enable = false`
  for a clean total retreat to pre-branch behaviour.

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

## Not done / follow-ups

- Live retune (config + restart is the loop).
- EGL YUV colour-space / range hints on the dmabuf import (PixelPilot passes
  none; a mismatch is a slight tint between the display and the DVR).
- Raw DVR mode is the bitstream and stays flat: post-process with
  `ffmpeg -vf "libplacebo=custom_shader_path=colortrans3.glsl"`.
- The splash is not inverted.
