# Camera modes: frame rate, resolution and field of view

The drone's IMX415 sensor offers a handful of readout modes, split across
two readouts (`venc.binning`): binned (2x2, the full view, the default) and
native (1:1 pixels, a narrower fps range, a fixed zoom). `venc.binning` and
`venc.fps` together pick one. This page says which, what each costs in
field of view, which `venc.size` values each can serve, and what the ground
station has to do to show it smoothly.

## The driver

`openipc-builder`'s `load_sigmastar` insmods waybeam_venc's star6e IMX415
driver, `sensor_imx415_star6e.ko`, for `imx415`; OpenIPC's own stock
`sensor_imx415_mipi.ko` is left out of the image. The image build does not
compile the driver or fetch waybeam: the module is a prebuilt binary,
built in `../waybeam_venc` (`make drivers-star6e`, `sensors/star6e/`) and
committed into `openipc-builder`'s mabur overlay
(`lib/modules/4.9.84/sigmastar/`). Changing the modes means rebuilding it
there and replacing that file. The module the image carries is built from
waybeam_venc 581804a: six modes, `imx415_linear_visible = { 1, 2, 4, 5, 6,
8 }` of a 9-row table. An older build (waybeam_venc 0c880d8, five modes,
`{ 1, 2, 4, 6, 8 }`) has no 2304x1296@100 mode and outputs its 60 fps
native mode at 2816x1584, so on it `venc.binning = false` fails boot with
`sensor mode WxH@F not offered by the driver`; the binned tiers work on
either build. The six-mode driver's visibility list enumerates:

| idx | mode | readout | sensor window | share of the view (w × h) | aspect |
|---|---|---|---|---|---|
| 0 | 3840x2160 @ ≤ 33 | all pixels, not binned | 3840x2160 | 100 % × 100 % | 16:9 |
| 1 | 2944x1656 @ ≤ 60 | non-binned window (native readout) | 2952x1656 | 77 % × 77 % | 16:9 |
| 2 | 1920x1080 @ ≤ 90 | 2x2 binned | 3840x2160 | 100 % × 100 % | 16:9 |
| 3 | 2272x1278 @ ≤ 100 | non-binned window (native readout) | sensor crop, centred | 59 % × 59 % | 16:9 |
| 4 | 1728x972 @ ≤ 100 | 2x2 binned crop | 3456x1944, centred | 90 % × 90 % (81 % of the area) | 16:9 |
| 5 | 1728x816 @ ≤ 120 | 2x2 binned crop | 3456x1632, centred | 90 % × 76 % | 2.12:1 |

Both native modes run the sensor at 10-bit depth; the binned modes run
12-bit. maburd finds the mode it wants in that list by output size and
maxFps (`drone/venc/camera_modes.c`, `sensor_select.c`), never by index,
and fails boot with `ERROR: sensor mode WxH@F not offered by the driver`
when the loaded driver lacks it (OpenIPC's own stock driver,
`sensor_imx415_mipi.ko`, lacks modes 1, 3, 4 and 5 entirely — it is never
loaded on this image). Check what is running with
`grep Cur /proc/mi_modules/mi_sensor/mi_sensor0`.

## Which mode `venc.binning` and `venc.fps` select

| `venc.binning` | `venc.fps` | mode | view |
|---|---|---|---|
| true | 1–90 | 1920x1080 binned | full |
| true | 91–100 | 1728x972 binned crop | 1.11x zoom |
| true | 101–120 | 1728x816 binned crop | 1.11x wider, 1.32x taller zoom |
| false | 1–60 | 2944x1656 non-binned | 1.29x zoom (measured 77 % × 77 % of the binned view) |
| false | 61–100 | 2272x1278 non-binned crop | 1.67x zoom (measured 61 % × 61 % of the binned view at the former 2304x1296 window, 1.4 % narrower now) |
| false | 101–120 | none: the loader refuses `venc.fps` | — |
| either | with a `record.size` that needs it (above 1920x1080 and above the selected mode; `docs/vtx-recorder.md`) | 3840x2160 (the loader then requires `venc.fps` ≤ 30) | full |

The native zoom figures are bench-measured, not computed: a fixed
landmark's width in a 1280-wide snapshot, against the same landmark in
the binned 1920x1080 mode (the 1728 finding's method, below).
`docs/findings/2026-11-02-native-readout-bench.md` has the raw pixel
counts.

The link size never changes the view: the sensor window sets the zoom and
the VPE only scales the window down to `venc.size`. A larger link size
carries more of the window's detail, not more of the scene.

The full binned view stops at 90 fps because of the sensor, not the
encoder: a frame is VMAX lines × the line time, full height binned needs
VMAX ≥ 2250, and a full-width binned line needs HMAX ≥ ~365 (4.92 µs) or the
readout is malformed (black frames with coloured lines at the right fps).
74.25 MHz / (2250 × 365) = 90.4 fps. Neither clock nor bit depth moves
either floor. At HMAX 365 a 100 fps frame holds 972 binned lines and a
120 fps frame 816, which is where modes 4 and 5 come from (waybeam's device
mapping: `../waybeam_venc/documentation/STAR6E_IMX415_HEADROOM.md` §2,
§5.6). The measured views:
`docs/findings/2026-10-19-waybeam-imx415-1728x972.md`.

Mode 5's real ceiling is 119.660 Hz (VMAX 1700 at the line rate), not 120:
at `venc.fps = 120` the rate trim locks at the mode's ceiling
(`[sensor] cal: locked at the mode's ceiling`). The trim, the shutter cap
above 90 fps and the steer's ceiling clamp are in `docs/airtime-model.md`
§3 and `docs/phase-lock.md`.

Both native modes are timed about 0.5 Hz above their nominal rate so the
phase lock has room to steer up: from the driver's table and the timing law
fps = 74.25 MHz / (VMAX × HMAX), 2944x1656 (VMAX 1880, HMAX 652) computes
to 60.57 Hz and 2272x1278 (VMAX 1348, HMAX 548) to 100.52 Hz; the output
width does not affect the timing. The sensor-rate calibration measures
60.573 and 100.513 Hz and trims to a line under 60 / 100 (59.966 /
99.917 Hz). The steer then reaches a panel at the nominal rate (60.000 Hz
monitor, the 100 Hz goggles) with a small offset. A mode whose ceiling sits *under* the
panel cannot be locked at all: the steer pins at the mode's maximum, the
lock cycles acquire/out-of-range, the fps cell turns caution, latency
drifts ~1 ms/s and a frame repeats every few seconds
(`docs/phase-lock.md`). That is why the 100 fps native window is 2272x1278
rather than 2304x1296: the wider window already ran at the ISP's
~300 MPix/s wall with no blank lines to give, and its 99.63 Hz ceiling
could not be steered to a 100 Hz panel. The 60 mode only gave up blank
lines. Bench record of the under-ceiling state:
`docs/findings/2026-11-02-native-readout-bench.md`.

## Sizes

The VPE only scales the mode down; it never crops or upscales it. So
`venc.size` must have the selected mode's aspect ratio and fit inside it
(even, at least 320x180), or the loader refuses the config. Over the air,
fps and size move together as the single `VENC_MODE` parameter
(`docs/vtx-config-mavlink.md`):

- 1–90 fps, binned: any 16:9 size up to 1920x1080 (1280x720, 1920x1080).
- 91–100 fps, binned: any 16:9 size up to 1728x972 (1280x720, 1728x972).
  1920x1080 is refused (larger than the mode).
- 101–120 fps, binned: the 1728:816 shape up to 1728x816 (1728x816,
  1152x544, 864x408). 1280x720 is refused (another aspect).
- 1–60 fps, native: any 16:9 size up to 2944x1656 (2944x1656, 2560x1440,
  1920x1080, 1600x900, 1280x720).
- 61–100 fps, native: any 16:9 size up to 2272x1278 (2272x1278, 1920x1080,
  1600x900, 1280x720). 2304x1296 and 2560x1440 are refused (larger than
  the mode).

Encoding the mode's own size keeps all its detail but costs encoder time:
1728x972 at 100 fps reads 158 % of the H.265 engine's rated macroblock rate
(1280x720 at 100 fps reads 86 %); it held 100 fps on the bench with the
480 MHz encoder clock, with enc 6/8 ms p50/p90 against 4/5 ms at 1280x720.
Both native tiers hold their rate at every offered size (0 ausniff gaps or
drops on the bench), measured enc p50/p90 (`curl 127.0.0.1:8301/venc`):

| native 60 size | enc p50/p90 (ms) | native 100 size | enc p50/p90 (ms) |
|---|---|---|---|
| 2944x1656 | 14–15 / 15 | 2272x1278 | 11 / 11–12 |
| 2560x1440 | 11 / 12 | — | — |
| 1920x1080 | 8 / 8–9 | 1920x1080 | 9 / 9–10 |
| 1600x900 | 5–6 / 6–7 | 1600x900 | 7–8 / 8 |
| 1280x720 | 5 / 5–6 | 1280x720 | 6 / 7 |

The loader and the UI do not refuse any of them; none is removed from the
offered list. Bench record: `docs/findings/2026-11-02-native-readout-bench.md`.

## The ground station

The GS output has to run at the drone's rate for the phase lock to hold
(`docs/phase-lock.md` "Configuration"). With `screen_mode = "auto"` (the
bundle) or a bare size (`"1920x1080"`) the display follows the camera: it
moves to the largest mode (or that size) the sink's EDID lists within
0.5 Hz of the drone's measured rate. A pinned `"WxH@RATE"` does not follow;
it is matched within 0.5 Hz and otherwise falls back to the sink's
preferred mode (a log line). Known sinks:

- Avatar Goggles X: 1280x720@100.000 and 1920x1080@100.000 only above
  60 Hz; range limits 50–76 Hz otherwise, so no 90 Hz. `"auto"` runs a
  91–100 fps drone at 1920x1080@100; a 90 fps drone has no matching mode
  and is shown at 100 Hz (judder) with the rate mismatch standing.

When the drone's measured rate and the panel's refresh are too far apart
for the phase lock (more than 0.55 Hz: a wrong `venc.fps`, or a sink that
fell back), the OSD fps cell reads `source/panel` in caution (`100/60`),
the phase lock pauses and sends nothing, and the player logs
`maburplay: rate mismatch`. A drone in low-power is never a mismatch. If
the rates match but the lock still gives up (the steer cannot reach the
panel, e.g. a panel a hair faster than a mode's ceiling) the fps cell turns
caution without the `/`.

A picture whose aspect differs from the panel (mode 5 on 16:9) is
letterboxed with black bars on the screen, and in the burned DVR when its
size (`dvr.burned.size`) has another aspect, never stretched
([`gs/player/src/fit_rect.h`](https://github.com/gilankpam/mabur-ground/blob/master/gs/player/src/fit_rect.h)). The burned DVR's default size is the
decoded picture's own, which needs no letterbox.

## The GS burned recorder at 100 fps

At a 100 Hz panel the GS's burned DVR cannot encode every vblank and
costs live view ~4 ms while it records; run it at `dvr.burned.every = 2`.
Details: [`docs/colortrans.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/colortrans.md) ("Known limitations").

## The GS burned recorder at native sizes above 1920x1080

The burned recorder's default picture size is the decoded picture's own
(`dvr.burned.size = ""`). At a decoded size above 1920x1080 — the native
tiers' larger sizes, 2944x1656, 2560x1440 and 2272x1278 — the colour
stage cannot keep up and the recorder drops frames outright rather than
just costing live latency the way 100 fps does: 22 % dropped, measured
at 2944x1656 on the bench. Set `dvr.burned.size = "1920x1080"` to keep
the burned recording at 1080p regardless of the link size; with it set,
the drop falls to under 1 %. Bench record:
`docs/findings/2026-11-02-native-readout-bench.md`.

## The SD recorder above 90 fps

`[record]` keeps its own rules (`docs/vtx-recorder.md`): `record.fps` ≤
`venc.fps`, `record.size` with `venc.size`'s aspect. Above 90 fps the link
runs a crop mode — binned 1728x972, or under `venc.binning = false` the
native 2272x1278 crop — and the same three risks apply:

- A `record.size` larger than the mode (the bundle's 1920x1080 against
  1728x972) is an upscale on VPE port 1.
- A `record.fps` that does not divide `venc.fps` (60 from 100) gives the
  recording uneven frame spacing: the bind's frame-rate limiter passes 60
  of every 100 frames.
- The H.265 engine runs one encode at a time and the live crop-mode encode
  alone is past its rated rate (158 % for 1728x972@100), so a recording
  adds encode time to the live link.
