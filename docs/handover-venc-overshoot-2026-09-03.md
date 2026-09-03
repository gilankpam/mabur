# Handover: encoder overshoot above the commanded bitrate (2026-09-03)

Status: **BENCH-MEASURED 2026-09-03 evening — the QP-floor hypothesis
below is REFUTED; the burst is scene-content, not rate-control recovery.
See "Bench results" at the end.** Companion to the two drone-side fixes
on branch `probe-hold-bitrate` (4d16e10 probe bitrate hold, fe1643b
TxQueue-pressure shed). Those contain the *damage*; this document is
about the *cause* they contain.

> **Progress 2026-09-03 (evening), branch `venc-overshoot-observability`
> — instrumentation built, bench NOT run (both devices were off the
> network).** Everything in "Observability follow-ups" except the config
> backup is done, plus the bench prerequisites; see "Bench plan" for the
> exact procedure with the new tools. Nothing below the TL;DR was
> re-decided: the `×1024` stays as-is (it is a sweep arm, not a fix), no
> default changed, `venc.min_qp` ships at 0 = firmware. Not deployed to
> either device — and remember `probe-hold-bitrate` (287bf2d) is not on
> the drone yet either, so the next drone deploy is a two-feature flag
> day on Telem (83→84 bytes, GS must move too).

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
  ms stripes; IDRs ~1.7× a P frame). `qp_delta` (`s32IPQPDelta`): the
  2026-08-29 backup says −4, **the deployed config says +4** (read off the
  drone 2026-09-03 evening, `out/drone-mabur-config-2026-09-03.json`;
  boot log `> qpDelta changed to 4`). Positive = I frames coarser than P.
- `venc.max_ipprop`: **2 in the deployed config** (same read; boot log
  `> max_ipprop: current (firmware default or last-set) = 0` then
  `applied = 2`, so the firmware default is 0 = unbounded and
  `docs/airtime-model.md` §3 "prod runs 2" was right, the 2026-08-29
  backup was stale). It only bounds the I:P *ratio*, so it is inert on
  the P-frame ramp seen here — the `max_ipprop` sweep arm below is
  therefore {2 (current), firmware 0}, not {firmware, 2}.
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

**Tooling as of 2026-09-03 evening (all host-tested, none hardware-run):**

- `vencprobe --cycles 0 --duration 120000 > /tmp/vb.csv` — passive mode,
  never POSTs; logs every frame plus a 25 ms `s,mono_us,req_kbps,qp` poll,
  where `qp` is the encoder's own QP (`GET /venc` gained `"qp"`, from
  `MI_VENC_Stream_t.h265Info.startQual`). Build:
  `arm-openipc-linux-gnueabihf-gcc -O2 -Idrone/vendor -o out/arm/vencprobe tools/bench/vencprobe.c`.
- `tools/bench/vencburst_analyze.py vb.csv` — finds the quiet→motion
  cycles and prints per cycle: quiet rate (+qp), peak 100 ms rate as a
  multiple of the *programmed* rate (kbps×1024, so exact CBR = 1.00×),
  settle time back inside +10 %, integrated excess KB, IDRs in the ramp,
  and the acceptance verdict (peak ≤ 1.2×, excess ≤ 64 KB). With ≥ 4
  cycles it also splits by pre-burst deficit — prediction 1 of the
  hypothesis. Synthetic-capture tests in `tests/test_vencburst.py`.
- `venc.min_qp` (config, 0 = firmware) and `POST /venc/set?min_qp=N`
  program `u32MinQp`; the first apply prints the firmware's
  MinQp/MaxQp/MinIQp/MaxIQp to `/tmp/mabur.log` — **read that line first,
  it answers "where does the firmware floor sit" before any sweep.**
- Drone `stats:` line carries `enc_pk100=<kbit/s>k`, the busiest 100 ms
  window of the stats second — the same quantity the analyzer computes,
  without a capture, and the one to eyeball while covering the lens.
- Telem: `drone.enc.qp` is now the encoder QP (was the ROI override —
  the whole "qp read 0" observation above was that), `drone.enc.roi_qp`
  the override, `drone.congestion_shed` the fe1643b shed. maburtop shows
  `qp NN roi -NN` and `shed FS|CONG|off`.

**First look, 2026-09-03 evening, live drone, NO deploy** (passive
`vencprobe` from tmpfs against the deployed Sep-2 `maburd`, rung 5, cmd
16000, the bench's own static scene, 20 s, 1190 frames):

| what | value | meaning |
|---|---|---|
| mean rate | 15.61 Mb/s = **0.95× programmed** (16.384) | the static bench scene is NOT a quiet scene — CBR spends its whole budget on it |
| 100 ms rate p50 / p95 / max | 0.95× / 0.97× / 1.06× | steady state is tight; no spontaneous bursts |
| base / enh median | 33.3 / 32.5 kB | refPred equalised, as designed |
| IDR frames in 20 s | **0**; every frame carries the GDR flag | `rally` runs intra-refresh stripes, no periodic IDR at steady state — "IDR cost" is only the *requested* IDRs (rung/bitrate writes) |
| largest frames | 41 kB (1.24× median), all stripe frames | |
| commit interval p50 / p95 / max | 16.9 / 17.7 / 19.9 ms | **no frame bunching** — the alternative hypothesis (encoder fps not steady) is out, at least on a static scene |
| enc_us p50 / max | 7.3 / 8.4 ms | as in the latency accounting |

Consequences for the plan: (1) the pre-burst *deficit* the hypothesis
needs (4.9 of 16 Mb/s in flight) does not exist on the bench unless the
scene is made genuinely flat — **cover the lens**, a wall is not enough;
(2) prediction 1 (burst ∝ deficit) is testable by varying how long/dark
the cover is; (3) the flight's 4.9 Mb/s quiet seconds were flatter than
anything the bench camera sees pointed at the room — worth asking what
the camera was looking at at 87 s (sky? grass at hover?). The encoder
QP was not in this capture (old daemon); the new binary adds it.

**Procedure:** deploy config-then-binary to the drone (and `maburgs` to
the GS — Telem flag day). Park the link at rung 5 or pin
`POST /venc/set?bitrate=16000` (re-POST inside the 5 s re-assert).
Start `vencprobe --cycles 0 --duration 180000`, then ≥ 10 cover/uncover
cycles with ≥ 5 s covered each. Run the analyzer. Then repeat with
`min_qp` 20 / 24 / 28 via the debug endpoint (volatile, no restart), then
`max_ipprop=2`, one variable at a time. Log `enc_pk100` and `txq=` from
the stats line alongside; `shed CONG` in maburtop counts the sheds.

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

- ✅ **`drone.enc.qp` replaced** (2026-09-03 evening): now the encoder's
  QP from `h265Info.startQual` (no `GetChnStat` QP exists on this SDK;
  the stream info is the only readback), `roi_qp` split out. The ring
  meta was deliberately NOT grown for a per-frame QP — it is the FrameHdr
  wire format (`kFrameHdrLen == VENC_FRAME_META_SIZE`), so per-frame-ish
  QP goes through the 25 ms `GET /venc` poll instead.
- ✅ **Telem bit4 `congestion_shed`** — sideport `drone.congestion_shed`,
  maburtop `shed CONG`. flightreport does not yet *use* it for
  attribution; the data is recorded (flightrec is always-on).
- ✅ **`enc_pk100=` on the `stats:` line** (`drone/src/peak_rate.h`).
- ✅ **Deployed drone config into `out/`** —
  `out/drone-mabur-config-2026-09-03.json` (drone came back online late
  in the session): `bitrate_max_kbps` 16000, `airtime_budget` 0.6,
  `qp_delta` **+4**, `max_ipprop` **2**, rally, 60 fps, gop 2 s.

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

## Bench results (2026-09-03 evening, branch `venc-overshoot-observability` deployed both ends)

Rig: passive `vencprobe` on the drone, rung 5 (cmd 16000 → 16.384 Mb/s
programmed), operator covering/uncovering the lens ≥ 5 s per cycle, plus a
run of quick open/close cycles; 301 s, 17917 frames, 13 cycles.
Capture + report on the dev host: `log/vencburst-bench-2026-09-03-run2.*`
(log/ is untracked).

| metric | value |
|---|---|
| steady state | mean 0.95×, 100 ms p50 0.96×, p95 1.00× of programmed |
| covered lens | **no dip at all** — the trailing-1 s rate never left 0.92–1.02× |
| cycles found | 13, **all** steady→burst, **0** quiet→motion |
| peak 100 ms rate | median **1.55×**, max **1.96×** (2.29× on a 25 ms grid point) |
| excess bytes per cycle | median 92 KB, max 323 KB (the quick open/close run) |
| settle back inside +10 % | median **100 ms** (one frame), max 1375 ms |
| shape | 9/13 = a single frame > 2× budget (scene cut, 55–195 kB vs 34 kB budget); 4/13 = multi-frame ramps ≤ 1.4 s (quick open/close) |
| deficit dependence | none: smaller-deficit half 1.52×, larger 1.55× |
| IDRs | 0 in 301 s (rally = GDR stripes, IDRs only on request) |
| encoder QP | **unreadable**: `MI_VENC_StreamInfoH265_t` has only `refType` filled by this firmware — `size`, all CU counters, `updAttrCnt`, `startQual` are 0 on every frame |
| TxQueue during the bursts | depth ≤ 6 (1 Hz samples), wait ≤ 5 ms, 0 drops, `congestion_shed` never set, no rung move |

What this says:

1. **Prediction 2 fails, prediction 1 fails.** CBR spends its full budget
   on a black frame (sensor noise at whatever QP), so there is no
   quiet-scene deficit to climb out of, and the burst size does not track
   the pre-burst rate. `venc.min_qp` therefore has nothing to bound —
   **the min_qp sweep is pointless and was not run**; the knob stays in
   the tree at 0 as a documented dead end, or gets deleted.
2. **The mechanism is scene content.** A lens uncover is a scene cut: the
   next frame has no usable reference, is intra-coded inside a P frame,
   and rate control has no P-frame size cap on star6e (`u32MaxPSize`
   dead, `max_ipprop` applies to I frames only). It comes out at 2–6× the
   per-frame budget, and the RC then under-spends slightly (30.5 vs 33.6
   kB frames) for the rest of its 1 s window — it is honouring the window,
   as designed. Sustained motion (the quick open/close run, and the
   flight's pans) is the same thing spread over 0.7–1.4 s at 1.5–2×.
3. **The bench link absorbs it; the flight link did not.** 323 KB of
   excess at rung 5 on the bench left the TxQueue at ≤ 6 bodies. In flight
   the same excess hit a queue at 154/255 because the *drain* was lower
   (range, retries, half-duplex uplink share) — the overshoot is a fixed
   property of the encoder on a scene change, and whether it hurts depends
   entirely on drain headroom. That puts the fix back on the transport
   side, where fe1643b already is.

What remains worth doing (ranked):

- **CORRECTION (same night, after checking `libmi_venc.so` exports, the
  folded-in waybeam and upstream master): there IS a live P-frame size
  cap on this SoC, and it was already measured.**
  `MI_VENC_SetSuperFrameCfg` mode REENCODE with `u32SuperPFrmBitsThr`
  (bits) bounds every P frame from the NEXT frame, keyframe-free, with the
  I threshold left at `0xFFFFFFFF`. Measured 2026-08-27 on the local
  waybeam stack (`../mabur-fork/mabur-stack-20260828/`, waybeam 0.69.2
  `video0.superframePBytes`, mabur `docs/plan-frame-size-caps.md` S1;
  uncommitted, never upstreamed, and mabur's fold-in is f956a52 of
  2026-08-23 so it never arrived here — memory `venc-attr-change-idr`
  recorded the fact and this handover missed it): rung 2, 720p120,
  10.8 Mb/s, P threshold 6000 B → P frames 11.1 kB → **3.2 kB**, 122 fps
  encoded / 119 decoded, 0 drops, glass 28 → 12.5 ms, qpDelta and min/max
  QP unchanged by the call. Two hard caveats from the same probe: (a) an
  **I threshold below the IDR size stalls the channel** (SDK aborts and
  regenerates the GOP with the same oversize IDR, forever) — I must stay
  unlimited; (b) the RC **re-plans well under** the P threshold (3.2 kB
  under a 6 kB cap), so the cap is a quality lever too and must follow the
  rung's per-frame budget (pct × kbps×1024/(fps×8)), applied with each
  bitrate write. waybeam PR #113 also saw `SetSuperFrameCfg` reset qpDelta
  to 0 / maxQp to 48 on its path; the S1 probe did not, but re-stage the
  RC intent after the call regardless (see the #255 note below).
  **This is the experiment to run next with tonight's rig**: port
  `superframe_p_pct` into `drone/venc/star6e_controls.c`, sweep {off,
  300, 200, 150} % of the per-frame budget, read peak/excess with
  `vencburst_analyze.py` and quality by eye on the GS.
- **Also from upstream waybeam (bf8c3cb, issue #255): `MI_VENC_GetRcParam`
  returns stale driver defaults for ~0–5 s after `StartRecvPic`**, so a
  Get→modify→Set in that window writes the stale block back. mabur's
  `star6e_runtime_apply_startup_controls` does `qp_delta` then
  `max_ipprop` (then `min_qp`) back-to-back right after start — exactly the
  shape that reverted qpDelta to 0 on every star6e craft upstream. Our
  `qp_delta` +4 may never have reached the encoder. Port the `g_rc_intent`
  staging (every RC write writes the whole intent) and add a readback.
- **Dead ends confirmed by upstream** (6106f24, HISTORY 4337): the
  `MaxISize/MaxPSize` caps never bind on star6e (deleted upstream);
  `MI_VENC_SetFrameLostStrategy` makes the firmware answer every skipped
  frame with a keyframe (deleted upstream). `SetAdvCustRcAttr` and the
  custom QP map are exported but no header on this machine defines their
  layouts. `u32RowQpDelta` remains untested.
- Scene-detector-triggered IDRs
  (`drone/venc/scene_detector.c`, ported but NOT wired) would fire one
  frame *after* the big frame and add an IDR on top — do not wire it for
  this.
- **Drain headroom is the lever**: `airtime_budget` (0.6 today, 0.614
  with the ×1024) and the rung-5 `bitrate_max_kbps` 16000 set how much of
  a 2× burst the air can absorb. A ×1024 fix is a 2.4 % contribution.
- **The shed is the containment** and it never had to fire on the bench.
  Whether the half-cap threshold is right is a *range* question: repeat
  this capture with attenuation or a walk-out, and count
  `drone.congestion_shed` (now recorded) against `txq.drops`.
- **Remove the dead encoder-QP plumbing** (Telem `qp`, sideport
  `drone.enc.qp`, `GET /venc` `qp`, vencprobe's 3rd poll column, the
  strminfo dump). The SDK cannot supply it; a permanently-0 field is
  exactly what this document started by complaining about.

Corrections to the sections above, in light of this: the "Mechanism"
section's classic-CBR-emergence story is wrong for this encoder; the
"What would fix it" ranking's item 1 (`u32MinQp`) is dead; item 2
(headroom) and the shed are what stands.

## SuperFrame P-cap sweep (2026-09-03, later the same evening)

`venc.superframe_p_pct` ported (commit aa80e1c) and deployed to the drone;
GS unchanged. Four arms, each its own passive capture at rung 5 (cmd
16000 → 16.384 Mb/s programmed, 34.1 kB per-frame budget), operator on
the lens (6–10 cover/uncover cycles + quick open/close bursts per arm),
cap changed live through the debug endpoint between arms. Captures and
per-arm reports: `log/vencburst-bench-2026-09-03-arm-{off,300,200,150}.*`
(untracked). Every `SetSuperFrameCfg` was accepted with an exact readback
(REENCODE, P bits = pct × budget); `qp_restage` 0 (nothing to re-stage).

| arm (cap) | cycles | largest frame | peak 100 ms median / max | excess median / max | steady 1 s rate p10 / p50 / p90 |
|---|---|---|---|---|---|
| off (control) | 10 | **121 kB** (195 kB in the earlier control) | 1.37× / 1.86× | 67 / 348 KB | 15.3 / 15.6 / 15.9 Mb/s |
| 300 % (102 kB) | 8 | 92 kB | 1.40× / 1.63× | 105 / 306 KB | 15.3 / 15.6 / 15.9 |
| **200 % (68 kB)** | 9 | **63 kB**, 0 frames > 2× budget | 1.35× / **1.48×** | 68 / **213 KB** | 15.0 / 15.6 / 16.2 |
| 150 % (51 kB) | (6, all "quiet→motion") | 50 kB | — | — | **10.8 / 12.1 / 14.1** |

Readings:

1. **The cap binds, exactly and immediately.** Largest frame per arm sits
   just under the programmed ceiling (92 / 63 / 50 kB against 102 / 68 /
   51). The single-AU scene-cut frame — the thing no other knob could
   touch — is gone at 200 %.
2. **The bits are conserved, not removed.** Median peak and median excess
   are unchanged across off / 300 / 200: the RC re-encodes the cut frame
   at the cap and the remaining content lands in the next 1–3 frames.
   What improves is the *worst case* (max peak 1.86 → 1.48×, max excess
   348 → 213 KB) and the per-AU size, i.e. serialization jitter and the
   size of the largest thing the TxQueue ever holds. For the queue-fill
   question the 100 ms excess matters more than the single frame, and
   there the cap buys ~40 % on the worst cycle and nothing on the median.
3. **200 % is free at steady state on this scene; 150 % is not.** At
   150 % the encoder abandoned its target for the whole arm: 10.6–12
   Mb/s (0.65–0.73×) on the same static scene that runs 15.6 uncapped,
   frame p50 33 → 24 kB. This is the fork's "RC re-plans well under the
   cap" and upstream's "min_qp is a bit ceiling" — a P cap within ~1.5× of
   the budget is a rate collapse, not a burst bound. Where the knee sits
   is scene-dependent (this is a static room), so 200 % is *not* proven
   safe for a busy scene or for rung 0's 8.5 kB budget without a sweep
   there.
4. **The cap→off transition is itself a burst.** Switching 150 % → 0
   produced one telemetry second at **25.7 Mb/s** (1.57×), TxQueue wait
   67 ms, and the first `congestion_shed` of the evening (4 s of shed,
   GS record `t_ms` 5337723–5341758, no rung move, 0 drops). The RC
   dumped the window budget it had been prevented from spending. Any
   live *loosening* of the cap (debug pokes, or a rung promote raising
   the budget while the RC sits under the old cap) can do this; a
   tightening cannot. Not a reason to avoid the knob, a reason not to
   toggle it in flight and to bench a promote with the cap on.
5. **No frame was lost anywhere**: decoded fps held, ausniff 61 fps /
   0 frame-id gaps after the sweep, `txq_drop` 0 all evening,
   `venc_verb_fail` 0.

Decision status: default stays 0. **200 % is the candidate** for a
flight config, gated on (a) the same sweep at rungs 0–2 (budget 8.5–17
kB — GDR stripe frames of 41 kB at rung 5 scale with the rate, but the
collapse knee may not), (b) a promote/demote pass with the cap on
(item 4), and (c) the operator's quality verdict on the uncovers.
Whether it is worth shipping at all depends on what hurts in flight: if
it is the 100 ms excess filling the queue (flight-0011's reading), the
cap's ~40 % worst-case cut is modest and the shed is doing the real
work; if it is single-AU serialization latency and jitter (the 195 kB
frame is 130 ms of air at rung 5's ~12 Mb/s effective), the cap is the
fix.
