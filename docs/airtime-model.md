# The airtime model — how frame bytes become latency and jitter

The reference for how a frame's bytes turn into air time, completion delay
and jitter; for the **encoder bitrate formula** (this page is its one
home — other pages link here); for the per-rung UEP overhead policy; and
for the encoder size knobs that work, and the ones that don't, on this
SoC. Measured figures come from the bench, the prod pair at mcs5 /
10 Mbit/s 20 MHz, unless marked otherwise. Dated history (the AirBalancer
solver, the 4-stream flat-refs policy, the measured bitrate inputs, the
parked 90 fps canary): `docs/findings/2026-08-29-airtime-model-history.md`.

## 1. The model

The wire is a serial pipe only ~2.4× faster than the video it carries.
An AU is *complete* at the GS when its last needed symbol (data + FEC)
has arrived and decoded, so:

    completion(frame) ≈ capture + encode + air_bytes / drain_rate
    air_bytes         = payload_bytes × (1 + overhead(sid)) (+ framing)

The nominal `len·(1+ov)` term underestimates real air at large frames:
SBI/fragment framing and repair quantization measured a reproducible ~2×
gain error in the open-loop model
(`docs/findings/2026-08-29-airtime-balance-spike.md`), and framing excess
runs ~5 % at large frames, more as frames shrink. Nothing compensates for
it at run time; `encoder.airtime_budget` absorbs it (§7: real
time-occupancy ≈ budget × ~1.16).

### The encoder bitrate formula

`RcAgent::run_bitrate_policy` (`drone/src/rc_agent.cpp`) sets the
encoder's rate from the applied op:

    kbps = 1000 · airtime_budget / [ f0·(1+ov_base)/rate_b + (1−f0)·(1+ov_enh)/rate_e ]

- `rate_b`, `rate_e` = `delivered_mbps(ladder[0])`, `delivered_mbps(ladder[1])`
  — the nominal PHY rate × `air_clock.efficiency_<bw>[mcs]`
  (`drone/src/air_rate.h`, §7). Both layers fly the op's MCS, so the two
  rates are equal; the formula keeps two terms because base and enh carry
  different overhead.
- `ov_base`, `ov_enh` = the commanded per-rung pair from the RCF (§2), or
  the debug-HTTP `ov_base_pct`/`ov_enh_pct` override while both are set.
- `f0` = `kShareBase` = **0.60**, a fixed base byte share (the median of
  flight-0000; on an equal pair the term cancels exactly).
- `airtime_budget` = `encoder.airtime_budget`, (0, 1], bundle **0.65** — a
  fraction of the link's *delivered* capacity.
- The result is clamped to `[encoder.bitrate_min_kbps,
  encoder.bitrate_max_kbps]`, capped at `low_power.bitrate_kbps` while the
  low-power mode is active (`docs/link-adaptation.md`), and rounded to
  100 kbps.
- Write policy: forced on entering LINKED and on the 5 s re-assert
  (`kReassertMs`); otherwise every decrease is written at once and an
  increase at most once per second. State latches only when the encoder
  verb succeeds, so a failed write is retried on the next RCF.

**Why the inputs are fixed.** On Star6E every `MI_VENC_SetChnAttr` that
changes the rate emits a keyframe that bypasses `idr_rate_limit` (15 real
changes → 15 IDRs; same-value writes → none). A formula fed with live
measurements (a measured base share and framing excess) re-evaluated on
every RCF and dithered across the 100 kbps grid: ~150 keyframes per
12-minute flight for a few percent of airtime accuracy. A probe is not a
term either: the probe stream changes MCS only, never bitrate (§6), so it
costs zero encoder writes and zero IDRs.

### Completion delay and the encoder

Measured calibration: completion delay ≈ **4.2 ms + 0.85 µs per payload
byte** (≈ 9.4 Mbit/s effective payload drain at mcs5/20). The encoder
contributes almost nothing to its variance: `maburd` publishes into the
frame ring at a flat 16.7 ms cadence, publish jitter EMA 0.3–0.5 ms,
pts→publish < 1 ms for every frame type. That pts→publish figure is
cross-timebase (vendor pts leads CLOCK_MONOTONIC), so it proves flatness,
not absolute encode latency; the wire `enc_us` (MI_SYS_GetCurPts basis) is
the true per-frame figure.

### Jitter is the second difference of air sizes

Inter-completion interval = 16.7 ms + (air_this − air_prev)/rate, and the
player's jitter metric is an EMA of |Δ interval| — an alternation
detector. Any pattern that makes consecutive frames occupy different air
time shows up multiplied by ~2:

| pattern | EMA |
|---|---|
| payload sizes alternating 1.8:1 (a 2-back reference structure, TRAIL_R vs TRAIL_N) | ~21 ms |
| equal payloads, unequal per-class FEC overhead (base 2.5× air vs enh 1.5×) | ~10–15 ms |
| equal air per class | **5–8 ms** |

The remaining 5–8 ms is the floor: ~2–5 ms of transport noise
(FEC-generation close timing, USB/radio batching, scheduling) plus
scene-driven within-class size variance under CBR. The only lever below
it is more air-rate headroom — running at lower utilization or a higher
MCS margin halves the 0.85 µs/B slope — which is a ladder-policy trade,
not a config knob.

## 2. Overhead policy — a per-rung pair

UEP is expressed only through FEC overhead: both layers ride the op's
MCS, and every `[[link.ladder]]` rung carries an `overhead_base` /
`overhead_enh` pair (GS config, 0.1–2.0, `overhead_base >= overhead_enh`
enforced — the base layer is the one every frame depends on; bundle
0.5 / 0.25 on every rung). The GS sends the rung's pair in the RCF and the
drone applies it directly to the two `UepEncoder` layers
(`set_layer_overhead`, no per-frame redistribution). A pinned link uses
`link.static_overhead_base` / `static_overhead_enh`; before the first op
arrives the drone runs both layers at `fec.base_overhead` (0.5). The
values are actual overhead (0.5 = +50 % repair bytes).

The pair is a jitter trade (§1): any base/enh difference is a
per-class air difference, which the jitter EMA shows at ~2× Δair /
wire-rate. Measurement basis for the pairs:
`docs/findings/2026-08-30-same-rate-uep.md`.

## 3. Encoder-side size shaping — what works on this SoC

The encoder decides payload sizes; the transport faithfully converts
their variance into jitter. Knobs, in order of proven usefulness:

- **Reference structure is the big lever.** `venc.ref_base` /
  `ref_enhance` / `ref_pred` program the SVC-T hierarchy
  (`MI_VENC_ParamRef_t`); bundle 1 / 1 / true = every other frame
  droppable with enhance→base prediction, which keeps base and enhance
  payloads equal. A 2-back reference structure is structurally 1.8:1
  (windowed CBR won't equalize it) — don't run it if jitter matters.
  The rolling refresh (`venc.intra_refresh_frames` / `intra_refresh_qp`,
  `MI_VENC_IntraRefresh_t`) would spread intra cost across P frames, but
  it only runs at a GOP start and the bundle has no periodic refresh
  (`venc.gop_s` 0), so the sweep ships off too (`intra_refresh_frames`
  0). The QP field is a no-op on star6e CBR regardless — the stripe is
  coded at the RC's P QP and the I-frame knobs below do not catch it;
  only the `superframe_p_pct` re-encode cap does, at +4 ms encode:
  `docs/findings/2026-10-15-gdr-slices.md`. The picture now heals on
  demand instead (`docs/fec-nack.md` "Demand refresh");
  `drone/venc/venc_cfg.h` documents each field.
- **`venc.max_ipprop`** (config, 0=off; also volatile via
  `:8301 POST /venc/set?max_ipprop=N`) programs `u32MaxIPProp` — the ONE
  RC size cap star6e actually enforces (dose-response measured). It bounds
  the I:P *ratio*: the bundle runs 2, which clips the IDR size
  distribution flat at 2× the P average. Achieved-ratio floor ≈ 1.4 (I-QP
  rails); below that the encoder refuses. It's worst-case insurance —
  inert while the natural ratio is under the cap.
- **`venc.qp_delta`** (s32IPQPDelta, −12..12, bundle 4) is a weak bias:
  ~2% IDR size per QP step. Average-shifter, not a bound.
  **Every RC write stages the whole intent** (`qp_delta`, `max_ipprop`,
  `min_iqp`, `max_iqp` — `g_rc_intent` in `drone/venc/star6e_controls.c`):
  `MI_VENC_GetRcParam` returns stale driver defaults for ~5 s after
  `StartRecvPic`, so a Get→modify→Set of one field in that window would
  silently revert another. The encoder loop prints `rc_readback t+10s: …
  OK|MISMATCH` so the log proves what the encoder holds, not just what was
  written.
- **`venc.min_iqp`** (config, 0=leave firmware 12, bundle **34**; also
  volatile via `:8301 POST /venc/set?min_iqp=N`, 1..51, with `max_iqp=N`
  beside it) programs `u32MinIQp`, the I-frame QP floor — **the one
  direct IDR SIZE bound star6e honours.** Raising it shrinks IDRs steeply
  while P frames are unchanged; the `SetRcParam` write fires no IDR and
  the very next IDR honours it. It exists for the demote IDR at the bottom
  rung, which otherwise costs tens of ms of air at mcs0. Cost: one soft
  keyframe per transition, refined by the following P frames. Bench
  records: `docs/findings/2026-09-06-iqp-cap.md`,
  `docs/findings/2026-08-29-airtime-model-history.md`.
- **`venc.superframe_p_pct`** (config, 0=off — the code default; bundle
  **200**; also volatile via `:8301 POST /venc/set?superframe_p_pct=N`,
  100..1000) programs `MI_VENC_SetSuperFrameCfg` REENCODE with a
  **P-frame** bit threshold of N % of the rung's per-frame budget
  (kbps×1024/(fps×8)), re-derived on every bitrate write; I stays
  unlimited unconditionally because an I threshold below the IDR size
  stalls the channel for good. **The RC re-plans well under the cap**, so
  this is a quality lever as much as a bound; it targets scene-cut P
  frames at 2–6× budget (`docs/findings/2026-09-03-handover-venc-overshoot.md`).
- **`u32MinQp` — no knob, do not rebuild.** It acts as a bit *ceiling*
  that collapses the rate past the scene's natural QP, and the quiet-scene
  QP-floor hypothesis it was added for was refuted
  (`docs/findings/2026-09-03-handover-venc-overshoot.md`). The firmware's
  MinQp/MaxQp/MinIQp/MaxIQp stay visible in the `rc_readback t+10s:` line.
- **`u32MaxISize` / `u32MaxPSize` are DEAD on star6e** — SetRcParam
  accepts them, `SetRcPriority(FRAMEBITS_FIRST)` succeeds, output is
  bit-identical even at absurd caps (verified at 13× overshoot). The
  star6e.h comment promising hard ceilings describes maruko, not this
  chip. Do not rebuild that feature.
- **`I6_SYS_LINK_LOWLATENCY` on the VPE→VENC bind is REJECTED** —
  `MI_SYS_BindChnPort2` fails with `-1610014712` (0xa00a8008) and venc
  init dies (respawn loop, no video). It would have overlapped encode with
  sensor readout (~4-6 ms of the `enc` segment); on this SDK encode start
  stays gated on the last readout line. FRAMEBASE is the only accepted
  mode for this hop (comment at the bind site,
  `drone/venc/star6e_pipeline.c`).
- **GOP length is not a rate-control window.** A bitrate step emits its
  own keyframe and the rate control settles within a second whatever the
  GOP (`statTime` 1 s); the bundle ships `venc.gop_s = 0`, no periodic
  refresh, and heals on demand (`docs/fec-nack.md`). Measured:
  `docs/findings/2026-10-17-long-gop-spike.md`,
  `docs/findings/2026-10-17-sdk-gop-ceiling.md`.
- **The sensor does not run at `venc.fps`, and the VPE→VENC bind must not
  try to make it.** The IMX415's frame period is a whole number of sensor
  lines: the driver answers `SetFps(60)` with VMAX 3375 at HMAX 365, which
  is 60.27 Hz, and in dim light the auto-exposure pins the shutter at the
  frame period and stretches VMAX to 3386 (60.08 Hz). A bind whose dst is
  60 engages the SDK's frame-rate control, which passes "at most dst per
  second" by its own clock (effective ceiling ~59.95) and drops one frame
  per beat period — every 3.6 s at 60.27 Hz, every 10 s at 60.08 — booked
  as VPE chn `DropCnt` / out-port `FailCnt`. The wire closes seamlessly
  over the hole (no `frame_id` gap, no FEC counter); only the pts step
  shows it, and the player repeats a frame. Two things fix it, both in
  `drone/venc/sensor_cal.[ch]`:
  1. **Full rate binds src:src** (`sensor_cal_bind_dst_fps`): a request at
     or above the sensor's set rate binds VPE→VENC (and VPE→recorder) at
     the mode's rate on both sides, so no limiter sits in the path. Only a
     request below it — the low-power 30 fps — keeps dst = request and
     decimates. RC `fpsNum` and GOP stay at the requested fps.
  2. **The boot-time trim** (`SensorCal`, armed on the encoder thread
     right after the cold-boot fps re-kick): measures the real rate from
     the encoded frames' pts over 1 s windows (median step), and
     re-requests the sensor in milli-fps (the driver accepts
     `[min*1000, max*1000]`) until the real rate sits in
     `[fps − 0.030, fps − 0.004]` Hz, just under the configured rate; one
     VMAX line is 0.018 Hz at 60. A line is fps² / line rate (the IMX415
     modes share ~202.5k lines/s), so above 60 fps the band and the
     one-line nudge grow by (fps/60)²: at 100 fps a line is 0.049 Hz and
     the band is `[fps − 0.083, fps − 0.011]`; a band narrower than a line
     would be straddled forever. Locks in 2–3 rounds (`[sensor] cal:`
     lines, `snr_mhz=` on the stats line, `sensor_mhz` on `GET /venc`).
     The request never exceeds the sensor mode's ceiling (maxFps × 1000,
     which the driver refuses above); a mode whose fastest line sits under
     the band (1728x816@120 tops out at 119.660 Hz) locks at that ceiling
     (`locked at the mode's ceiling`). After 8 rounds without convergence, or a driver that refuses
     milli-fps, it restores the integer boot request and stops. It only
     measures a full-rate stream: while the bind decimates (a drone that
     boots disarmed into low-power 30 fps) it holds, and resumes with a
     fresh window once the fps verb returns to full rate, so that drone
     locks in the first ~13 s after arming. Measured
     on the bench: 60000 → 59720 mfps = VMAX 3391 = 59.990 Hz, zero pts
     holes in 120 s. What remains is the beat between the sensor and the
     GS panel (60.000 Hz nominal): one panel repeat per ~100 s, the floor
     without genlock. Bench record:
     `docs/findings/2026-10-14-sensor-rate-trim.md`.
- **Sensor mode and the shutter cap follow `venc.fps`.** The pipeline asks
  the driver for the mode `camera_mode_for` names (`drone/venc/camera_modes.c`):
  1920x1080@90 (2x2 binned, full view) up to 90 fps, 1728x972@100 up to
  100, 1728x816@120 above. It finds that mode in the driver's list by size
  and ceiling and fails boot if the driver lacks it. The VPE only scales
  that mode down, so the loader requires `venc.size` to keep the mode's
  aspect ratio and fit inside it; modes, views and size rules are in
  `docs/camera-modes.md`. The AE shutter ceiling (`sensor_cal_shutter_cap_us`) is the whole frame
  period up to 90 fps and 90 % of it above: in a dim scene the sensor's
  frame is the shutter plus ~22 lines, which at a full 10 ms cap holds
  a 100 fps mode at 98.94 Hz whatever is requested, out of the trim's and
  the phase lock's reach. Bench record: `docs/findings/2026-10-19-100fps.md`.

## 4. Measuring it — rigs and gotchas

- **Drone side** (encoder cadence): poll `write_idx` on
  `/dev/shm/mabur_f` (static armv7 sniffer; recipe = 8-byte-aligned
  slot stride `(4+slot_data_size+7)&~7`, header 192 B). Lines:
  `mono_us pts len flags nal0`. ⚠ Under SVC-T + intra refresh the
  periodic refresh AUs are VPS-led AUs whose slice NAL is not IDR type
  19/20, so the ring meta IDR flag misses them — count IDRs **GS-side** (`nal0==32`), and don't trust
  drone-side class buckets.
- **GS side** (the truth for jitter): poll the AU ring
  `/dev/shm/mabur-au` (ausniff layout), per-AU
  `t_us pts sid fid len flags nal0`. Jitter EMA formula matches the
  player: `ema += (|iv − prev_iv| − ema)/16`. Per-class completion
  offset: min-normalized `(t_us − pts) & 0xFFFFFFFF`.
- **Decomposition trick**: predict intervals purely from sizes
  (`16.7 ms + 0.85 µs/B × Δlen`) and compare EMAs — whatever the size
  model explains is encoder/policy-fixable; the residual is the floor.
- **Loss injection**: build maburgs with `-DMABUR_LOSS_SIM=ON`
  (bench-only; prod builds contain zero rig code). ⚠ The control
  socket's default port 8302 collides with the GS's secondary sideport
  fan-out — pass another port. ⚠ The ON build for the GS must be
  cross-compiled arm64; a naive host cmake of the ON config yields an
  x86 binary that will not run on the GS.

## 5. Where the model could go next

- **Below the floor**: headroom. At 2× the air-rate margin the
  serialization slope halves; that's a ladder policy question
  (utilization target per rung), not an encoder or FEC change.
- **Differentiated protection** costs jitter: §1 prices any per-class air
  difference (Δair × 0.85 µs/B, ×2 in the EMA), so a rung that wants a
  larger base/enh gap pays for it knowingly.
- A 90 fps canary layer (60 fps guaranteed + 30 fps disposable) was
  explored and parked on a 60 Hz display; the reasoning and the
  conditions for revisiting it are in
  `docs/findings/2026-08-29-airtime-model-history.md`.

## 6. Probe stream air share

The always-on probe stream (`docs/link-adaptation.md` "Probe stream")
adds one video-body-sized SBI body (1405 B at the bundle geometry,
`13 + bpb·(16 + symbol_size)` with bpb 4, symbol 332) after **every**
video AU that went on air, base or enh — 60 bodies/s at 60 fps — at the
MCS of rung `current + link.probe.rung_offset`, capped at the top rung
(`drone/src/probe_source.h`, `LadderController::probe_rung()`).
Its air share is 60 × the serialization of 1405 B at that MCS (lower MCS
serializes a fixed byte count more slowly). It is NOT budgeted by
`run_bitrate_policy` or the RcAgent airtime estimator — the probe
changes MCS only, never bitrate, so it costs zero encoder writes and zero
IDRs. At the top of the ladder (no probe commanded, `probe_rung() == -1`)
the cost is zero.

## 7. Per-MCS capacity and per-rung A-MPDU

- **`run_bitrate_policy` and the air clock price off `delivered_mbps` =
  nominal × `air_clock.efficiency_<bw>[mcs]`** (`drone/src/air_rate.h`),
  never the nominal rate alone. The config is one 8-entry per-MCS table
  per width (`docs/bw40.md`); absent = all ones = nominal.
  `encoder.airtime_budget` is therefore a fraction of measured capacity.
  Real time-occupancy ≈ budget × ~1.16 (SBI/fragment framing ~1.09-1.14
  plus probe and MSP, none of them in the formula), so the bundle's 0.65 ≈
  75 % of the air at every rung. The gap between a bench table (no GS
  uplink) and the flight-fitted value is the RCF/telemetry slot share; it
  sits inside that ~1.16, not in the table.
- **The tables are measured per unit.** `maburcal`'s capacity sweep
  (`docs/calibration.md`) saturates the air at each (MCS, width) for 2 s
  with production-sized frames, the production power and the production
  aggregation mode, and writes `efficiency_<bw>[mcs]` = delivered air bits
  / (receive span × nominal): timed from the GS's first to last frame of
  the point, since the drone's TX queue keeps the air busy past the
  commanded dwell. A point the drone's own TX path limited (its sender
  rarely blocked on the device) is flagged `tx_bound` and keeps the
  existing value. A new table takes effect at the next `maburd` start.
  The bundle's tables are the reference unit's bench numbers:
  `docs/findings/2026-09-17-bandwidth-sweep.md` measured 0.87 / 0.78 /
  0.71 / 0.76 / 0.76 / 0.76 / 0.78 / 0.76 of nominal at mcs0-7 with agg6
  and 0.93 / 0.88 / 0.84 / 0.80 with singles at mcs0-3 at 20 MHz (linkbench,
  prod geometry, three channels within 0.5 %), giving 20 MHz `[0.93, 0.88,
  0.84, 0.80, 0.76, 0.76, 0.78, 0.76]` (singles below mcs4, agg6 above)
  and 40 MHz `[0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75]` with agg6.
- **The aggregation mode is per rung and per width**: `ampdu.min_mcs_20`
  (bundle 4) / `ampdu.min_mcs_40` (bundle 2) — singles below the floor,
  the configured aggregate at and above (`drone/src/ampdu_policy.h`,
  switched live by `RealActuator::apply_op`). The bundle ladder is five
  40 MHz rungs, mcs0-4, so mcs0/1 fly singles and mcs2-4 aggregate. Do not
  read a fixed A-MPDU cost into the model.
