# Native (non-binned) sensor readout — bench gates

**Status:** bench acceptance for the native readout design
(`venc.binning = false`); current behaviour recorded in
`docs/camera-modes.md`.

Question: do the two native tiers (`2944x1656@60`, `2304x1296@100`) meet
the design's eight bench gates on the real drone, and does the ground
side hold at link sizes above 1920x1080?

## Setup

Drone: the six-mode waybeam driver build (visibility list
`{ 1, 2, 4, 5, 6, 8 }`, table index 5 = `2304x1296@100` newly enumerated,
table index 2's output widened to `2944x1656`), `maburd` rebuilt with
`venc.binning`. GS: Radxa, monitor 2560x1440@60.000 Hz
(`display.phase_lock = true`, `phase_margin_ms = 4`). The 100 Hz goggles
were not connected for this run, so every phase-lock and image
observation below is against the 60.000 Hz monitor only.

All `VENC_MODE` / `REC_*` live sets were driven generically — mavparam.py
run on the GS against its loopback endpoint (the GS's python lacks
`_ssl` and the `xml` module pymavlink needs, so the tool and stub
modules were shipped to the GS first; a LAN bind was refused by the
classifier, so the sets go out over the GS's own loopback MAVLink
endpoint).

Both ends were power-cycled/restored to their pre-bench binaries and
configs afterward: the drone's `/etc/mabur.toml` back to `.pre-native`
plus `binning = true`, the module back to the 6-mode build (no change —
it stays); the GS's `maburgs.toml` was never touched and
`maburplay.toml`'s `dvr.burned.size` restored to `""`.

## Gate 1 — mode selection and timing

Native 60 (`venc.fps = 60`, `venc.size = 1280x720`): `Sensor pad
selected mode index 1 2952x1656 (maxfps 60)`; the driver logs `MIPI
frame 2952x1656, usable output 2944x1656 (cropping overscan)` and
`cropped to 2944x1656 (offset 4,0)`; `/proc/mi_modules/mi_vif/mi_vif0`
shows `VIF cap (4,0,2944,1656) GB_10BPP Fps 61` (no silent halve);
`/proc/mi_modules/mi_sensor/mi_sensor0` reads `Cur 2944x1656@60fps`. The
`[sensor] cal` line: "locked at the mode's ceiling, request 60000 mfps
measured 59.945 Hz" (steer clamped, +79 → 0). `dmesg` at start shows no
FIFO-FULL (only the normal `MI_SYS GetInputPortInfo` startup probes); a
5-minute soak (`t = 30..300 s`) read `fifo=0` throughout, not just after
the first 15 s.

Native 100 (`venc.fps = 100`, `venc.size = 1280x720`): the set applied in
531 ms; `[sensor] cal` "locked at the mode's ceiling ... measured 99.631
Hz"; `mi_sensor0` `Cur 2304x1296@100fps`; VIF `(0,0,2304,1296) GB_10BPP
Fps 100`; a 5-minute soak also read `fifo=0` throughout.

**Result: PASS**, both tiers. Neither needed the 2816x1584 fallback (see
gate 3).

## Gate 2 — image

Snapshots at 1280x720 for both tiers (`n60.jpg`, native 100's snapshot)
are clean: no black frames, no coloured lines, in a dark-room test
scene (grey, low contrast). Black level and colour read comparable to
the binned picture side by side, but — see "Open items" — the room was
too dark to tell the native modes' 10-bit depth from the binned modes'
12-bit visually.

View share, measured by the 2026-10-19 1728x972 finding's method (a
fixed landmark's width in a 1280-wide snapshot, against the binned
1920x1080 mode): binned 130 px, native 60 168 px → **1.29x (77 %)**,
native 100 213 px → **1.64x (61 %)**. (The design's computed-only
figures were 1.30x/77 % and 1.67x/60 %; the lens is fisheye-ish and the
landmark sits near centre, so the small gap is expected.) Native 100 vs
native 60 directly: 213 px vs 168 px → 1.27x, close to the sensor
windows' own ratio 2944/2304 = 1.28.

**Result: PASS.**

## Gate 3 — the 2944-out-of-2952 clamp

The pipeline log shows the clamp (`cropped to 2944x1656 (offset 4,0)`
quoted above under gate 1); the picture is not stretched — the measured
view-share ratios above track the sensor-window geometry, not a
distorted crop. Since the 5-minute soak at 2944x1656 read `fifo=0`
throughout (gate 1), the fallback to 2816x1584 was **not needed**.

**Result: PASS** (fallback not triggered).

## Gate 4 — encode budget

Enc p50/p90 (`curl 127.0.0.1:8301/venc`) and ausniff (15 s) at every
offered size, each tier at its ceiling rate:

| native 60 size | rebuild | enc p50/p90 (ms) | ausniff fps / gaps / drops |
|---|---|---|---|
| 2944x1656 | 512 ms | 14–15 / 15 | 61.0 / 0 / 0 (ring 12 %) |
| 2560x1440 | 538 ms | 11 / 12 | 61.0 / 0 / 0 (ring 12 %) |
| 1920x1080 | 546 ms | 8 / 8–9 | 61.0 / 0 / 0 |
| 1600x900 | 517 ms | 5–6 / 6–7 | 61.0 / 0 / 0 |
| 1280x720 | 537 ms | 5 / 5–6 | 61.0 / 0 / 0 |

| native 100 size | rebuild | enc p50/p90 (ms) | ausniff fps / gaps / drops |
|---|---|---|---|
| 2304x1296 | 490 ms | 11 / 11–12 | 100.7 / 0 / 0 |
| 1920x1080 | 559 ms | 9 / 9–10 | 99.7 / 0 / 0 |
| 1600x900 | 530 ms | 7–8 / 8 | 100.7 / 0 / 0 |
| 1280x720 | 531 ms | 6 / 7 | 100.7 / 0 / 0 |

(`sensor_mhz` 99621–99641 across the native 100 sweep.) For reference,
binned 60 @ 1280x720: rebuild 581 ms, enc 4/5 ms, fps 61.4, 0 gaps;
aucadence offset −0.951 ms (native 60's own offset was −0.77 ms) with
`len` p50 39.5k/39.7k against native 60's 36.4k/36.2k.

**Result: PASS, every offered size in both tiers flies** — none is
dropped from the offered list.

## Gate 5 — phase lock

Only the 60.000 Hz monitor was connected; **the 100 Hz goggles were not
run.**

Native 60 on the 60.000 Hz monitor: the steer pins at the mode's 59.945
Hz ceiling and cannot reach the panel, so the lock cycles `acq → oor →
acq` (`lctr = 500`) rather than holding; e2e shows the expected ~0.9
ms/s sawtooth (drift 0.055 Hz = one repeated frame about every 18 s);
the regulator logged late +1 in 60 s with skips/replaced unchanged over
120 s.

Native 100 on the same 60 Hz panel is a rate mismatch by design (a
100 fps source against a 60.000 Hz panel), not a lock test; it was not
exercised as one.

Binned 60 locks as always on this panel (cal 59.99 Hz), run only for
contrast.

**Result: PARTIAL — monitor case matches the design's prediction (out
of range, periodic repeat); the goggles case is NOT RUN.**

## Gate 6 — ground side above 1920x1080

Run at 2944x1656@60 (the largest size either tier offers; native 100's
own 2304x1296 was not separately re-run — only one GS was up at a time):

- **Radxa `maburplay`**: decodes and displays it, `dec` 4/7 ms, e2e
  ~54 ms p50 (drifting, consistent with the out-of-range lock above).
  **PASS.**
- **Burned DVR, `dvr.burned.size = ""`** (the decoded size, 2944x1656):
  the file is valid HEVC 2944x1656, 15.2 s, plays, OSD burned in, but
  `in=459 encoded=357 dropped=100` (22 % dropped), 23.5 fps against a
  30 fps target; colortrans averaged 42.5 ms/frame (draw+finish 24.6,
  rga 15.7), encode 32.2 ms mean. **FAIL at the decoded size.**
- **Burned DVR, `dvr.burned.size = "1920x1080"`** (live-set on 8403):
  `in=456 encoded=453 dropped=3`; colortrans 15.8 ms, encode 14.2 ms.
  **PASS** — this is the documented workaround (`docs/camera-modes.md`).
- **OPi**: not run (only one GS board can be up at a time on this
  bench). **NOT RUN.**
- **Web GS**: **NOT RUN.**

**Result: PARTIAL.** Radxa live decode passes outright; the burned DVR
passes only with `dvr.burned.size = "1920x1080"` and fails at the
decoded native size; OPi and the web GS are not run. No size is removed
from the offered list — the fix is the `dvr.burned.size` setting, not a
narrower `MODE_SIZES`.

## Gate 7 — live sets

Under native 60, recorder on and pinned: `REC_ENABLE 1`, `REC_SIZE
2560x1440` (rebuild 589 ms). `VENC_MODE 60@1920x1080/binned` logged the
drone's recorder follow (`"(venc.fps +record.size)"`); `REC_SIZE` read
back `1920x1080`, rebuild 494 ms. Back to `60@2944x1656/native`: rebuild
502 ms, no follow (the recorder was already at a size that fits the new
mode). `VENC_MODE 120@1280x720/native` answered **FAILED**, with
STATUSTEXT reading `"venc.fps: no non-binned sensor mode above 100
fps:"` — STATUSTEXT's 50-character limit truncates the loader's reason
before its "set venc.binning = true or venc.fps <= 100" tail; the
drone's own log carries the full message.

**Result: PASS** (the set/follow/refusal behaviour is all correct); the
STATUSTEXT truncation is a display-length limitation, not a refusal
bug — see "Open items".

## Gate 8 — `ausniff.py` / `aucadence.py`

The repo copy of `ausniff.py` was scp'd to the GS's `/tmp` first (the
on-board copy is stale). Native 60 @ 1280x720, 30 s: 1814 AUs, 0 gaps, 0
resyncs, fps 60.5; aucadence offset −0.77 ms, `len` p50 36.4k/36.2k.
Native 100's sweep (gate 4) read ausniff fps 100.7, 0 gaps throughout.

**Result: PASS.**

## Summary

| Gate | What | Result |
|---|---|---|
| 1 | Mode selection and timing | PASS (both tiers; no fallback needed) |
| 2 | Image | PASS (10-bit/12-bit comparison inconclusive — dark room) |
| 3 | 2944-out-of-2952 clamp | PASS (fallback not triggered) |
| 4 | Encode budget | PASS (every offered size, both tiers, flies) |
| 5 | Phase lock | PARTIAL (monitor only; goggles NOT RUN) |
| 6 | Ground side above 1920x1080 | PARTIAL (Radxa PASS; burned DVR FAIL at decoded size / PASS at 1920x1080; OPi NOT RUN; web GS NOT RUN) |
| 7 | Live sets | PASS (STATUSTEXT truncates the native-above-100 refusal reason) |
| 8 | ausniff / aucadence | PASS |

## Open items

- **Gate 5 on the goggles (100 Hz) is not run.** Only the 60.000 Hz
  monitor was on the bench; the native 100 tier's phase-lock behaviour
  against a panel near its own 99.63 Hz ceiling is unmeasured.
- **Gate 6 on the OPi and the web GS is not run.** Both decode paths
  above 1920x1080 are untested; only the Radxa's `maburplay` and its
  burned DVR were exercised.
- **The burned DVR drops frames at the decoded native size above
  1920x1080**: 22 % at 2944x1656 (23.5 of a 30 fps target), fixed by
  setting `dvr.burned.size = "1920x1080"` (3 of 456 dropped). This is
  not the same failure mode as the existing 100 Hz throughput limit —
  it drops frames outright rather than costing live latency — and is
  carried in `docs/camera-modes.md` as the operating advice.
- **STATUSTEXT's 50-character limit cuts the native-above-100 refusal
  message before its actionable tail** (`"venc.fps: no non-binned
  sensor mode above 100 fps:"` with `set venc.binning = true or
  venc.fps <= 100` missing from the ground side). The drone's own log
  has the full reason; nothing was changed for this bench round.
- **The 10-bit (native) vs 12-bit (binned) picture comparison was done
  only in a dark room** (grey, low-contrast scene); no visible
  difference was seen there, but the comparison has not been repeated
  in daylight or under colour-rich lighting.
- Native 100's own size (2304x1296) was not separately gate-6'd on the
  ground side; only the overall largest offered size (native 60's
  2944x1656) was run there.
