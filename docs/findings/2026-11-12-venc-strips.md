# Picture strips on the SSC338Q — kestrel-air's "P1 strips" ported as a spike

**Status:** spike; nothing built, strips not pursued. The encoder's
FRAMEBASE-only input is recorded with the encoder knobs in
`docs/airtime-model.md`; slices (`docs/slices.md`) stay the only picture
split. The throwaway code lives on the local branch `spike-venc-strips`,
never to merge.

Question: kestrel-air's BETAFPV P1 backend (`../fpvos/kestrel-air`,
`docs/ar9311.md`) encodes each 1080p picture as 2–4 **strips**: whole CTU
rows, each by its own WAVE5 encoder channel, each started as soon as its own
rows have landed in the camera buffer. The ground joins them. On their bench
at 4 strips the whole picture reached the goggle 25.8 ms after capture, down
from 34.7 ms. The SSC338Q's H.265 engine is the same Chips&Media WAVE5 core.
Can mabur do the same, and what would it gain?

Answer: half of it. Strips work: N cropped encoder channels on one VPE
frame, joined without a seam. But the encoder only takes a finished frame,
so a strip can never start before the last camera row has landed. That early
start is where most of kestrel's gain came from. What is left is overlapping
the air time of the top strips with the encode of the bottom ones. Replayed
over a real link capture, that is worth a median **−2.9 ms (2 strips) /
−3.2 ms (3) / −2.6 ms (4)** in AU completion at mid rungs, and −0.6 to
−1.7 ms on the top rung. The price is a second video pipeline end to end
(N streams on the drone, wire, FEC, AU ring, decoders, a join step, the DVR)
plus 1–3 ms of encoder time per frame. Not worth building next to slices.

## Setup

- **Drone:** the bench drone (SSC338Q, IMX415 binned 1920x1080@60, CBR,
  `slices = 4`). maburd at master `781e7845` plus three throwaway commits
  (env knobs `MABUR_SPIKE_*`). Run by hand from `/tmp` with `S00mabur`
  stopped; the installed binary and config were never touched. Each case was
  30–35 s, including the ~10 s MI bring-up; the first quarter of each log is
  dropped.
- **Logged:** per `MI_VENC_GetStream` return, `MI_SYS_GetCurPts − pack pts`
  (one SDK clock, no bridge). That is "done" below, in ms after pts. Strip
  channels were polled every 200 µs from their own thread; the link channel
  on maburd's production loop, which idles in 1 ms steps.
- **GS (air span only):** the Radxa on the deployed build, linked to the
  drone's installed maburd, `debug_log` already on. About 3 minutes of
  `au.log` (aulog 7) and `ctl.log`. Ladder HT40 MCS0–4 at 50/25 %
  overhead; the link moved between rungs 2 and 4.

## No early start: the encoder input is FRAMEBASE-only

- `MI_SYS_EnableChnOutputPortLowLatency` is exported by `libmi_sys.so`. Its
  ABI, from the disassembly, is `(MI_SYS_ChnPort_t *, MI_BOOL, MI_U32)`:
  16 B port + `bEnable` + param, sent as one 24 B ioctl.
- Called on VPE ch0 port 0 with `(1, 540)` before the VPE→VENC bind, it
  returns 0. `/proc/mi_modules/mi_vpe/mi_vpe0` then shows `MatchLine 540` on
  port 0. But `LowLatencyIsrCnt` stays 0 over 1100+ frames, and the link's
  done time is unchanged: 8.19 ms p50, against 8.21 ms without the call.
- With the enable and a `LOWLATENCY` (0x2) VPE→VENC bind, the bind fails
  with `-1610014712` (0xa0092008). The kernel logs why:
  `Output port MOD7 … supported bindmask0[org output0000001d, org input00000001], Orignal Proposed BindType00000002`.
  VPE's output offers FRAMEBASE | REALTIME | AUTOSYNC | RING (0x1d), but the
  VENC input accepts **FRAMEBASE only (0x01)**. This is the mechanism behind
  the earlier rejected `LOWLATENCY` bind.
- So no MI path hands the H.265 encoder a frame before VPE's frame-done.
  This adds to the 2026-10-09 negatives (no HW_RING input for H.265, slice
  early output ≤ 0.7 ms, `docs/findings/2026-10-09-venc-slice.md`).
  Sub-frame encode start on this SoC would need a user-injected buffer path
  (`MI_SYS_ChnInputPortGetBuf` or `InjectBuf`), which was not tried.

## Strips work: one channel per band via `MI_VENC_SetCrop`

- Channels 2…N+1, H.265 CBR, each sized to its band:
  `width × h_k` (whole CTU rows, extra rows to the top; 1080p → 576/504,
  384/384/312, 320/256/256/248). `MI_VENC_SetCrop(chn, {bEnable=1,
  {left 0, top_k, 1920, h_k}})` before StartRecvPic. Each channel bound
  FRAMEBASE to VPE port 0, the link's port.
- `SetCrop` returns 0 and the crop applies. Decoded strip frames stacked
  top to bottom reproduce the link's picture with no visible seam. The
  `MI_VENC_GetCrop` readback is wrong: `bEnable 0`, `top 0`, the channel
  size. Ignore it.
- **The engine runs strips one after another, newest bound peer first.**
  This is the same rule the recorder relies on
  (`docs/findings/2026-09-26-sd-record.md`). Binding the bottom strip first
  makes the top strip finish first. `MABUR_SPIKE_STRIP_TOPFIRST=1`
  reverses the order exactly.

## Strip timing (link channel paused, 1080p60)

`MI_VENC_StopRecvPic` on the link channel, so the strips had the engine to
themselves. A 1-strip run is the whole-picture baseline on the same poll and
bitrate. Done, ms after pts, p50 (p90 within +0.2–0.4 ms of p50):

| run | strip 0 | strip 1 | strip 2 | strip 3 | all done |
|---|---|---|---|---|---|
| whole, 12 Mbit/s | 7.02 | | | | 7.02 |
| 2 strips, 12 | 4.32 | 7.96 | | | 7.96 |
| 3 strips, 12 | 3.34 | 6.33 | 8.96 | | 8.96 |
| 4 strips, 12 | 3.00 | 5.31 | 7.61 | 9.90 | 9.90 |
| whole, 24 | 7.67 | | | | 7.67 |
| 4 strips, 24 | 3.26 | 5.70 | 8.14 | 10.56 | 10.56 |

- Fit: each strip costs about **1.2 ms fixed + 4.2 µs per row**. Splitting
  delivers the top strip 2.7–4.0 ms sooner. The whole picture finishes
  +0.9 / +1.9 / +2.9 ms later than unsplit, at 2 / 3 / 4 strips.
- With the link channel still encoding, the engine is overloaded at 4
  strips (~17 ms of work per 16.7 ms frame). Everything queues: strips p50
  22.8–28.1 ms, link p50 35.2 ms.
- Encoder-time budget per frame: whole picture ~7 ms, 2 strips ~8 ms,
  3 strips ~9 ms, 4 strips ~10 ms. Only 2 strips still fit 100 fps
  (10 ms). None fit 120 fps, and none fit next to the onboard recorder's
  second full encode.

## Air span and the replayed gain

`au.log` gives, per AU, the drone's whole-picture encode time (`enc`) and the
GS arrival span `t_last − t_first` (drone hand-off plus air). Model, per real
frame: whole completion = `enc + span`. Strip i is ready at
`enc × ready_i / 7.02` (the 12 Mbit/s ratios above), goes on air at
`max(ready_i, previous strip's air end)`, and takes `span × rows_i / 1080`.
Gain = whole completion − last strip's air end.

11,176 AUs, rung from the `ctl.log` S line at the AU's arrival:

| rung (HT40) | n | frame p50 | enc p50 | span p50 (p10–p90) | 2 strips | 3 strips | 4 strips |
|---|---|---|---|---|---|---|---|
| 0 | 280 | 14.2 kB | 7.18 | 9.50 (3.44–11.45) | +2.73 | +3.70 | +4.03 |
| 1 | 520 | 18.8 kB | 7.22 | 7.10 (1.28–12.99) | +2.70 | +3.04 | +2.48 |
| 2 | 4538 | 35.1 kB | 8.08 | 7.71 (7.37–9.25) | +2.99 | +3.26 | +2.64 |
| 3 | 3130 | 45.8 kB | 8.28 | 7.90 (1.96–8.90) | +3.08 | +3.29 | +2.63 |
| 4 | 2706 | 46.1 kB | 8.33 | 5.29 (4.37–5.69) | +1.70 | +1.44 | +0.63 |
| all | 11176 | 35.8 kB | 8.20 | 7.66 (3.88–8.92) | **+2.94** | **+3.20** | +2.55 |

Gains are the median, in ms of earlier completion. Strips arrive later than
the whole picture in 7 / 9 / 12 % of frames (2 / 3 / 4 strips).

- The span is longer than the encode in only 21 % of frames. On this link
  the air time per frame is shorter than the encode, which caps the gain
  well below the ~4 ms ceiling of `whole − first strip`.
- The gain splits into air hidden behind encoding minus the extra encode
  time. At 3 strips, mid rung: about 5.5 ms hidden − 2.3 ms extra ≈ 3.2 ms.
  More strips hide more air but add ~1.2 ms of encode each. So 3 is the
  peak, 4 never beats 3, and the top rung (short span) gains least.
- At higher frame rates frames get smaller and spend less time on air, so
  the gain falls toward the rung-4 figure.

## What the model leaves out

- No end-to-end measurement. The gain is in AU completion at the GS, not
  measured at the glass.
- Assumed: strip bytes scale with rows; the drone can put strip 0 on air the
  moment it is ready; the radio is idle when it is.
- GS decode: streamed slice decode already overlaps decode with arrival
  (`docs/slices.md`, dec −2.7 ms). Strips overlap decode naturally (one
  small picture each) but leave ⅓–½ of a picture to decode after the last
  packet, against ¼ with 4 slices today. The arrival gain mostly adds to
  what slices already give; it is not a second decode gain.
- Untested on the GS: several MPP contexts sharing rkvdec in stream mode,
  per-strip decode overhead, and the cost of joining N decoded buffers into
  one picture (display planes, RGA, or a CPU copy at ~190 MB/s).
- Not measured: CPU on either end, picture quality per byte at the strip
  joins (motion cannot be predicted across a join; kestrel encodes 16 extra
  rows past each join and trims them on the ground), 100 fps.

## Side observation: the link loop's 1 ms poll

The link channel's done time was 8.21 ms p50 (45 kB frames) on maburd's
production loop, which waits in 1 ms steps when the encoder has nothing for
it. The same picture on the spike's 200 µs poll was 7.02 ms (20 kB) /
7.67 ms (46 kB). About 0.5 ms of the link's figure looks like poll
granularity. Waiting on the channel fd would recover it with no strips at
all. That is a separate, cheap change; this spike did not A/B it.
