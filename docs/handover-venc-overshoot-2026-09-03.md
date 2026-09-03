# Handover: encoder overshoot above the commanded bitrate (2026-09-03)

Status: **OPEN — measured in flight, not yet reproduced on the bench.**
Companion to the two drone-side fixes on branch `probe-hold-bitrate`
(4d16e10 probe bitrate hold, fe1643b TxQueue-pressure shed). Those
contain the *damage*; this document is about the *cause* they contain.

## TL;DR

In flight-0011 the SigmaStar H.265 CBR encoder ran up to **41 % above
its commanded bitrate for a second** on scene changes, and ~9 % above
it at p90 with the command held. That is what filled the drone TxQueue
at 88 s and 102 s (36 dB SNR, rung 5/4), and the queue's drop-oldest is
what the GS ladder then scored as RF loss and demoted 5→4→3→2 for.
With fe1643b the queue sheds the enh layer at half-cap instead of
dropping, so the cascade cannot recur — but every such burst still costs
a 2 s enhancement gap. Making the burst rare is an encoder rate-control
question: which knob on this SoC bounds the *short-window* rate, given
that the size caps are dead and the I:P ratio cap is inert on P-frame
bursts. Nothing here is decided; the plan is a bench measurement.

## Evidence (flight-0011, `log/flight-0011.jsonl` + `ctl-0202`)

Drone telemetry is 1 Hz. `drone.enc.mbps` is the GS's delta of the
drone's emitted-kbyte counter over the telem receive interval; the
command is `drone.enc.cmd_kbps`. Units matter (next section): a command
of 16000 programs 16.384 Mbit/s, so an exact CBR reads ratio **1.024**.

Command held ≥ 2 s (transition seconds excluded):

| cmd_kbps | n | ratio p50 | p90 | max |
|---|---|---|---|---|
| 3900 | 8 | 0.933 | 0.990 | 0.990 |
| 7800 | 12 | 0.992 | 1.047 | 1.051 |
| 10400 | 33 | 0.999 | 1.090 | 1.129 |
| 15600 | 14 | 1.002 | 1.094 | 1.182 |
| 16000 | 117 | 0.982 | 1.086 | **1.413** |

The three events that mattered, all with the command held or *just*
lowered, none with an RF cause:

| t | rung | cmd | enc Mb/s (1 s) | before | txq depth | txq drops | what followed |
|---|---|---|---|---|---|---|---|
| 87.9 s | 5 | 16000 | 17.1 | 4.9 | 154/255 | ~330 | residual 5→4→3, util →2, 450 ms |
| 101.7 s | 4 | 15600 | 16.9 | 12.9 | (stale) | ~365 | residual 4→3→2 |
| 387.2 s | 5 | 16000 | **22.6** | 10.6 | 78/255 | 0 | txq wait 57 ms, e2e 107/141 (shed would now fire) |
| 114.5 s | 4 | 15600 | 18.4 | 11.6 | 100/255 | 0 | txq wait 83 ms, e2e 132/183 |

All 722 TxQueue drops of the flight sit in the first two events plus a
small one at 52 s. The shape is the same each time: the scene was
*quiet* (encoder well under its command — 4.9 Mb/s against 16, a static
frame at the QP floor), then motion arrived and the first second of it
came out 10–40 % over the command before rate control caught up.

Two things the flight data cannot tell you, which is why this needs the
bench:

- **1 s is too coarse.** A 154-body queue at rung 5 is ~200 KB ≈ 1.7
  Mbit; that could be a 100 ms burst at 17 Mb/s excess or a 1 s trickle
  at 1.7. The queue-filling quantity is the ~100 ms peak rate, which the
  telemetry averages away.
- **`drone.enc.qp` is NOT the encoder QP.** It is `actuator.last_roi_qp`
  (the ROI QP override, 0 = normal) — see the telem fill in
  `drone/src/main.cpp`. It read 0 for the whole flight and says nothing
  about where rate control sat. There is currently no gauge of the
  encoder's actual QP.

Also: `drone.enc.fps` over the same windows ranges 50–68 at a 60 fps
sensor, so the 1 s ratios carry ±10 % of pure window-alignment noise.
Treat p50 as the calibration (it should be ~1.024) and only the tail as
signal.

## What the encoder is actually programmed with

From `drone/venc/star6e_pipeline.c` / `star6e_controls.c` and the
2026-08-29 prod config (`out/drone-mabur-config-rally-2026-08-29.json`;
⚠ the *deployed* config has moved since — the flight shows
`bitrate_max_kbps` 16000 and `airtime_budget` 0.6, the backup says
10000/0.7 — read `/etc/mabur.json` on the drone before trusting any
knob value below):

- Codec H.265, rate mode **CBR only** (the VBR/AVBR arms were deleted
  with waybeam). `MI_VENC_ChnAttr.rate.h265Cbr = { gop, statTime = 1,
  fps 60/1, bitrate, avgLvl = 1 }`.
- **`bitrate = kbps × 1024`** — both at channel create and in
  `apply_bitrate()`. RcAgent's `run_bitrate_policy` thinks in decimal
  kbit/s (rate table 6.5/13/19.5… Mb/s, `airtime_budget` as a fraction
  of that), so every command is silently programmed 2.4 % high. Small,
  but it is a permanent bias in the direction of the problem, and it
  makes the policy's `airtime_budget` mean 0.614 when it says 0.60.
- `statTime = 1` s: the CBR's stat window. SigmaStar CBR converges the
  *average* over this window, which is exactly why a 1 s telemetry
  average looks nearly honest (p90 1.09) while the queue still fills —
  the RC is allowed to spend the whole window's budget in its first
  200 ms.
- `gop` = `venc.gop_s` 2.0 × fps under the `rally` preset (refPred, 150
  ms stripes; IDRs ~1.7× a P frame). `qp_delta` −4 (`s32IPQPDelta`).
- `venc.max_ipprop`: **not set** in the 2026-08-29 backup, i.e. firmware
  default (`apply_max_ipprop` prints the current value at boot — check
  `/tmp/mabur.log` on the drone). `docs/airtime-model.md` §3 says "prod
  runs 2"; one of the two is stale. It only bounds the I:P *ratio*
  anyway, so it is inert on the P-frame ramp seen here.
- `u32MaxISize` / `u32MaxPSize` are **dead on star6e** (hardware-proven
  2026-08-29, `docs/airtime-model.md` §3). Do not rebuild that feature.
- The live `MI_VENC_ParamH265Cbr_t` also carries `u32MaxQp`, `u32MinQp`,
  `u32MaxIQp`, `u32MinIQp` — never touched by mabur, firmware defaults.

## Mechanism (hypothesis, to be tested)

Classic CBR emergence from a quiet scene. With little to encode, rate
control rails at `u32MinQp` and the stream sits far under the command
(4.9 of 16 Mb/s). When motion starts, the first frames are encoded at
that floor QP — they are as large as the content makes them, with no
budget pressure yet because the 1 s stat window is nowhere near spent.
Rate control then raises QP over the next several frames until the
window's average is back on target. The excess of those first frames
over the command is the burst. Its size scales with how far below the
command the quiet scene was (i.e. how low QP was allowed to go) and how
much content arrives at once.

Predictions that distinguish this from alternatives:

1. Burst magnitude correlates with the *pre-burst deficit* (command −
   quiet-scene rate), not with the command level. flight-0011 agrees
   loosely (the 22.6 burst followed a 10.6 quiet second; the 17.1 burst
   a 4.9 one), but n = 3.
2. Raising `u32MinQp` shrinks the burst at the cost of quiet-scene
   quality (less "free" quality when static). This is the cheapest test.
3. A lower `statTime` would bound it directly, but 1 s is already the
   SDK minimum for this field as far as the vendor header shows — verify
   whether 0 or fractional values are accepted, expect not.
4. IDR-related alternatives are ruled out for 88 s/102 s: no rung change
   preceded the burst (the IDRs came *after*, from the cascade's bitrate
   writes), and `max_ipprop` would not act on a P ramp.

An alternative worth one bench cycle: the 60 fps sensor delivering
50–68 frames in some 1 s telem windows may mean the *encoder* fps is
not steady either (AE/AWB at 15 Hz, `star6e_controls.c` rc_fps
compensation only above 120). If frames bunch, CBR budgets per frame
and a bunch of frames is a burst. `vencprobe`'s per-frame pts will show
it immediately.

## What would fix it, ranked by expected cost

1. **Bound the quiet-scene QP floor** (`u32MinQp` via
   `MI_VENC_SetRcParam`, new `venc.min_qp` key, default = firmware).
   One field, same apply pattern as `apply_max_ipprop`. Reduces the
   deficit the RC has to climb out of. Downside: quiet scenes get no
   better than that QP — the FPV operator may prefer the burst.
2. **Give the policy real headroom at the cap.** `bitrate_max_kbps`
   16000 at rung 5 is 46 % nominal air; the burst went to 22.6 Mb/s ×
   1.5 overhead = 34 Mb/s of air bytes for a second, which the ~15 Mb/s
   effective burst drain (`docs/latency-budget-findings-2026-08-31.md`)
   cannot absorb. Either fix the `×1024` so the command is what the
   policy computed, or lower `airtime_budget` — both are config/one-line
   and buy 2–10 %, not 40 %. Not a fix, a margin.
3. **Shape the ramp in the policy instead of the encoder**: the main
   loop already reads `enc_bytes_total` every telem tick (it is not
   passed to RcAgent today; `RadioHealth` would grow one more field). A
   100 ms byte-rate EWMA crossing e.g. 1.3× the command could pre-empt
   the shed one tick earlier than the queue-depth trigger, or drop the
   command temporarily. This is a control loop on top of a control loop
   and every bitrate write is an IDR (`docs/airtime-model.md` §1) — do
   NOT do this via `set_bitrate_kbps`. A shed-only variant is cheap and
   IDR-free; whether it beats the depth trigger by enough to matter is a
   bench question.
4. **Encoder-side rate mode**: AVBR/VBR were deleted for good reasons
   (jitter, size variance). Not a path.

## Bench plan

Everything below runs against a live `maburd` without a GS in the
loop, using the existing rigs.

**Rig.** `tools/bench/vencprobe` (passive mode: it mmaps the venc frame
ring read-only and logs every frame's `mono_us pts len flags nal0` —
see its header). Run it while the command is *held* (RcAgent
re-asserts every 5 s; either pin via `POST /venc/set?bitrate=16000` or
just leave the link at rung 5). Log `txq.depth` alongside from the
`stats:` line in `/tmp/mabur.log`. Stimulus: static scene ≥ 5 s (lens
covered, or a wall), then a hard uncover / fast pan, ≥ 10 cycles.

**Metrics per cycle** (a small analyzer next to
`vencprobe_analyze.py`, which already computes per-step size levels):

- quiet-scene rate (last 1 s before the stimulus) and QP if obtainable
- peak 100 ms rate after the stimulus, in multiples of the programmed
  `kbps × 1024`
- time-to-settle: first 100 ms window back within +10 % of programmed
- integrated excess bytes over the ramp (this is what the queue holds)
- max `txq.depth` reached, and whether the fe1643b shed fired
  (enh silence on the ring, or a shed bit in Telem once one exists)

**Sweep**, one variable at a time, ≥ 10 cycles each, at cmd 16000 /
rung-5 op: `u32MinQp` {firmware, 20, 24, 28}; then `max_ipprop`
{firmware, 2} for completeness; then the `×1024` fix alone.

**Acceptance for a shipped change**: peak 100 ms rate ≤ 1.2× programmed
on the uncover cycle, integrated excess ≤ 64 KB (one IDR's worth, which
the queue is already sized for), *and* the standing gates — `aucadence`
base−enh offset inside the per-rung envelope, `ausniff` jitter EMA not
worse than the flat-pair baseline — because a QP floor changes size
variance too.

## Observability follow-ups (cheap, do these first)

- **Rename or replace `drone.enc.qp`.** It is the ROI QP. Either rename
  to `roi_qp` in Telem + sideport + maburtop + flightreport in one
  commit (schema is not additive-only, CLAUDE.md), or populate it from
  the encoder's actual QP if `MI_VENC_GetChnStat`/`GetRcParam` exposes
  one. Right now the field invites exactly the wrong conclusion.
- **Telem bit for the congestion shed** (fe1643b). Only
  `failsafe_shed` is visible today; a bench cannot count sheds without
  it, and `flightreport` cannot attribute enh gaps to congestion vs RF.
- **100 ms encoder rate in the `stats:` line** (drone side, stderr,
  free): peak-of-window `enc_bytes` delta. The 1 Hz Telem average will
  never show the burst; this would.
- **Deployed drone config into the repo's `out/`** with today's date —
  the 2026-08-29 backup no longer matches what flew.

## Open questions

- Is `statTime` accepted below 1 on this SDK? (Expect no; verify once.)
- Does `avgLvl` (create-time, set to 1) affect the window-spend shape?
  The vendor header does not document it; a 0/1/2 sweep is one cycle
  each.
- Is the 50–68 fps telemetry spread window alignment (dt is measured
  from telem *receive* times, so drone-side sample jitter plus RCF
  slotting hold — `link.rcf_slot`, up to one AU period — could explain
  ±10 %), or does the encoder actually bunch frames?
- How much of the 88 s event was the ramp and how much the *cascade's
  own* IDRs? The first demote fired 0.2 s after the burst began, so the
  queue was already at 154 before any IDR — but the 102 s event had a
  promote 3 s earlier (98.8 s), and its IDR may have been in the queue.

## Related

- `docs/airtime-model.md` §1 (bitrate formula, IDR cost per write), §3
  (which encoder knobs are live vs dead on star6e)
- `docs/link-adaptation.md` "Drone congestion shed (2026-09-03)"
- `docs/latency-budget-findings-2026-08-31.md` (burst drain, `dq`)
- `tools/bench/vencprobe.c`, `tools/bench/vencprobe_analyze.py`
- Memory: probe-bitrate-hold, venc-attr-change-idr, mcs1-budget-overshoot
