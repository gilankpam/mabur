# The airtime model — how frame bytes become latency and jitter

Written 2026-08-29 after the AU-completion-jitter investigation (EMA
21 → 5–8 ms). Everything here is bench-measured on the prod pair at
mcs5 / 10 Mbit/s unless marked otherwise. This page is the standing
reference for the serialization model, the current (flat) UEP overhead
policy, and the constraints anyone evolving either must not re-discover
the hard way.

## 1. The model

The wire is a serial pipe only ~2.4× faster than the video it carries.
An AU is *complete* at the GS when its last needed symbol (data + FEC)
has arrived and decoded, so:

    completion(frame) ≈ capture + encode + air_bytes / drain_rate
    air_bytes         = payload_bytes × (1 + eff_overhead(sid)) (+ ~6% pkt hdrs)

**Later the same day (airtime-balance-uep), the link collapsed to 2
streams (BASE sid0 at `mcs−1`, ENH sid1) and the single `air_bytes`
formula above became per-stream and measurement-anchored rather than
nominal, because the naive `len·(1+ov)` model measured a reproducible
~2× gain error at large frames (SBI/frag framing + repair quantization
it doesn't account for — spike 2,
`docs/airtime-balance-spike-findings-2026-08-29.md`). The split had a
structural limit: at 2:1 rate ratios (the mcs1 rung) the balancer's
rails could not reach air balance — 3.3 ms/pair structural residual —
which motivated a same-rate counter-study (flat vs asymmetric overhead
per rung, static and under motion,
`docs/same-rate-uep-findings-2026-08-30.md`). The drone's `AirBalancer`
tracked the true per-stream multiplier from what actually got
serialized, `m_s = emit_s / len_s` (`emit_s`/`len_s` are α=1/16 EWMAs of
emitted body bytes and frame-unit bytes, IDR frames excluded as 2–10×
outliers), and solved the per-stream air split around that measured
anchor:

    air_s(ov) = len_s · (m_s + (ov − ov_s_applied)) / rate_s

subject to `air_b = air_e` (balanced) and the budget invariant
`len_b·ov_b + len_e·ov_e = (len_b + len_e)·ov_cmd` (repair-byte-neutral:
the split redistributed, it did not add or remove overall FEC bytes).
`rate_b`/`rate_e` were `phy_rate(base_mcs)`/`phy_rate(profile_mcs)` from
the applied ladder, so an active ENH probe's rate flowed in
automatically.

⚠ **Superseded the next day (2026-08-30, same-rate-fixed-pairs).** Both
streams now ride the SAME scored mcs — the `mcs−1` base rate and the
`AirBalancer` solver described just above are both gone. UEP is
expressed only through FEC overhead, and that overhead is a fixed
**per-rung config pair** (`overhead_base`/`overhead_enh`, carried in the
v5 RCF, RC_VERSION 5) applied directly to UEP with no per-frame
redistribution. `AirFeed`, the deleted solver's measurement-only
successor, survived until 2026-09-01 publishing `share_base`/
`excess_base`/`excess_enh` into the bitrate blend; it is now deleted
too — see the bitrate paragraph below. GS-side, a single `budget()`/
`budget_for(rung)` no longer exists: `budget_base()`/
`budget_base_for(rung)` score sid 0 and `budget_enh_for(rung)` scores
sid 1 — see `docs/link-adaptation.md`.

The encoder's commanded bitrate keeps the two-term shape — a single-rate
`kbps = rate(ladder[1]) · budget / (1 + ov)` is wrong in both directions
once base and enh carry different overhead:

    kbps = airtime_budget / [ f0·(1+ov_base)/rate_b + (1−f0)·(1+ov_enh)/rate_e ]

where `ov_base`/`ov_enh` are the commanded pair off the v5 RCF and `f0`
is a **fixed** base byte share (`kShareBase`, 0.60). **`rate_e` is the
base profile's rate even during an enh probe** (since 2026-09-03): the
probe slot flies the candidate mcs, but feeding that rate in raised the
command 11–15% on every probe entry at rungs 0–3 and lowered it on exit —
two `SetChnAttr` writes, two forced IDRs, on a pipe already at ~65–70%
air — and was the largest single source of >80 ms p50 seconds in
flight-0011 (23 of 48). The 2026-08-05 probe spec always said the probe
changes MCS only; the rate feed-in came with the 2-slot op (2bbaa3f)
expecting the AirBalancer to absorb it, and the balancer is gone.

⚠ **Measured inputs removed 2026-09-01.** `f0` was AirFeed's live
`share_base`, and each term also carried a measured framing excess
`excess_s = m_s − (1 + ov_s_applied)`. That correction was real —
framing excess runs ~5% at large frames and grows as frames shrink —
but it closed a loop the encoder could not afford: lower rate → smaller
frames → proportionally more padding → higher excess → lower target,
re-evaluated on every RCF at 10–20 Hz against a 100 kbps rounding grid.
On Star6E every `MI_VENC_SetChnAttr` that changes the rate emits a
keyframe that bypasses `idr_rate_limit` (measured 2026-09-01: 15 real
changes → 15 IDRs, one frame later; 16 same-value writes → 0), and in
`flight-0000` an IDR cost a median 63.8 kB against a 21.4 kB base P
frame (38 ms of airtime at its rung, p95 92 ms) while **151 of 226
bitrate writes had no rung change behind them**. The blend was buying a
few percent of airtime accuracy for ~150 keyframes per 12-minute
flight. `f0 = 0.60` is the measured median of that flight (561
one-second windows, p50 0.601, sd 0.044); fixing it there costs
−2.7%/+1.0% of target on the worst pair in the flight (1.0/0.5) and
exactly nothing on an equal pair. The dropped framing excess is not
compensated in the formula — `airtime_budget` is the lever if the
resulting overshoot ever matters. Full derivation of the deleted
solver:
`docs/superpowers/specs/2026-08-29-airtime-balance-uep-design.md` §2;
current architecture and GS-side ladder/attribution consequences:
`docs/link-adaptation.md`; same-rate measurement basis:
`docs/same-rate-uep-findings-2026-08-30.md`.

Measured calibration: completion delay ≈ **4.2 ms + 0.85 µs per payload
byte** (≈ 9.4 Mbit/s effective payload drain at this op point). The
encoder contributes almost nothing: maburd publishes into the frame ring
at a flat 16.7 ms cadence, publish jitter EMA 0.3–0.5 ms, pts→publish
< 1 ms for every frame type and preset (including refPred — the old
"SigmaStar VENC alternating latency" claim from July 2026 is refuted).
That pts→publish figure was a cross-timebase measurement (vendor pts leads
CLOCK_MONOTONIC), so it proved flatness, not absolute encode latency; the
wire `enc_us` (MI_SYS_GetCurPts basis, 2026-08-31, SBI/SlotHdr v2) is the
true per-frame figure.

**Jitter is the second difference of air sizes.** Inter-completion
interval = 16.7 ms + (air_this − air_prev)/rate, and the player's
`video.jitter` metric is an EMA of |Δ interval| — an alternation
detector. Any pattern that makes consecutive frames occupy different
air time shows up multiplied by ~2. Two such patterns were found and
killed:

| era | mechanism | EMA |
|---|---|---|
| ltr:1, old UEP refs | payload sizes alternate 1.8:1 (TRAIL_R vs TRAIL_N) | ~21 ms |
| rally, old UEP refs | payloads equal, but per-stream FEC overhead made base fly 2.5× air vs enhance 1.5× | ~10–15 ms |
| rally, flat refs (now) | air sizes equal per class | **5–8 ms** |

The remaining 5–8 ms is the floor: ~2–5 ms of transport noise
(FEC-generation close timing, USB/radio batching, scheduling) plus
scene-driven within-class size variance under CBR. The only lever below
it is more air-rate headroom — running at lower utilization or a higher
MCS margin halves the 0.85 µs/B slope — which is a ladder-policy trade,
not a config knob.

## 2. Current overhead policy — flat

`kUepRefOverhead` (common/include/mabur/uep_encoder.h) is
**{0.50, 0.50, 0.50, 0.50}**, scaled by `cmd_overhead/0.25` and clamped
[0.125, 2.0]. At the mcs5 rung (cmd 0.5) every stream carries 1.0×
overhead → equal air per byte on every class; at rung 0 (cmd 1.0)
everything clamps to 2.0× uniformly. It was {1.00, 0.75, 0.50, 0.25}
(devourer heritage) until 2026-08-29; the flatten happened in two
reviewed steps (s1–s3, then s0) because frame→stream routing is
whole-frame (`classify_frame`: IDR/param-sets → s0, TRAIL_R tid0 → s1,
TRAIL_N and tid≥2 → s3; s2 is unused by current 1:1 SVC-T structures),
so unequal refs = unequal air for equal frames.

Resilience was re-proven at the reduced protection with the loss-sim
rig, not assumed: 5% injected s1 loss → 0.03–0.10% post-FEC residual
(the OLD refs demoted two ladder rungs at just 2%); 5% injected s0 loss
→ zero video gaps and no IDR-request even fired, because rally's intra
stripes heal without IDRs. That last fact is *why* s0 could join the
flat ladder: the belt-and-braces 2.0× on IDRs was sized for a preset
world (ltr) where a lost IDR froze the picture.

**The RcAgent airtime estimator is now exact, not conservative.** It
budgets video air as `bitrate × (1 + eff1)`; under the old refs eff1
(1.5) overstated the true pair-average (~1.0) by ~25%, a hidden margin.
With flat refs the estimate matches reality, so the same 75% airtime cap
admits more commanded bitrate at a given rung, and the ladder's
utilization/`u` readings sit on a new scale (thresholds deliberately NOT
retuned — see the dated scale-break entry in data-provenance.md).

⚠ **Superseded later the same day.** The 4-stream flat-refs policy above
was itself replaced within the day by the 2-stream (BASE sid0/ENH sid1)
literal-overhead policy + `AirBalancer` described in §1 — `kUepRefOverhead`
and the s0..s3 routing this section describes no longer exist.
`link.streams` shrank from 4 entries to 2; every overhead-shaped sideport
value from before this second change is cmd-scale (half the actual air
overhead the same nominal number means after it) — see the dated
"overhead literal + 4→2 stream collapse" entry in data-provenance.md. This
section stays as the record of the flatten investigation that made the
collapse safe (it's what proved base/enh could carry equal, then reduced,
protection without a resilience regression).

## 3. Encoder-side size shaping — what works on this SoC

The encoder decides payload sizes; the transport faithfully converts
their variance into jitter. Knobs, in order of proven usefulness:

- **Preset choice is the big lever.** `rally` (refPred) equalizes
  base/enhance payloads and spreads intra cost (2 s GOP + 150 ms
  stripes → IDRs only ~1.7× a P frame). `ltr:1` is structurally 1.8:1
  (TRAIL_R references 2-back; windowed CBR won't equalize) and measures
  ~11–12 ms EMA even on flat refs — don't run it if jitter matters.
- **`venc.max_ipprop`** (config, 0=off; also volatile via
  `:8301 /venc/set?max_ipprop=N`) programs `u32MaxIPProp` — the ONE RC
  size cap star6e actually enforces (dose-response measured). It bounds
  the I:P *ratio*: prod runs 2, which clips the IDR size distribution
  flat at 2× the P average (worst-case ratio 1.65, IDR completion
  +5.5 ms over base). Achieved-ratio floor ≈ 1.4 (I-QP rails); below
  that the encoder refuses. It's worst-case insurance — inert while the
  natural ratio is under the cap.
- **`venc.qp_delta`** (s32IPQPDelta) is a weak bias: ~2% IDR size per
  QP step (±12 range ≈ ±25% total). Average-shifter, not a bound.
- **`u32MaxISize` / `u32MaxPSize` are DEAD on star6e** — SetRcParam
  accepts them, `SetRcPriority(FRAMEBITS_FIRST)` succeeds, output is
  bit-identical even at absurd caps (verified at 13× overshoot). The
  star6e.h comment promising hard ceilings describes maruko, not this
  chip. Do not rebuild that feature.
- **`I6_SYS_LINK_LOWLATENCY` on the VPE→VENC bind is REJECTED** —
  hardware-tried 2026-08-31: `MI_SYS_BindChnPort2` fails with
  `-1610014712` (0xa00a8008) and venc init dies (respawn loop, no
  video). It would have overlapped encode with sensor readout (~4-6 ms
  of the `enc` segment); on this SDK encode start stays gated on the
  last readout line. FRAMEBASE is the only accepted mode for this hop
  (comment at the bind site, `drone/venc/star6e_pipeline.c`).
- **GOP is not a free knob**: SigmaStar CBR converges over the GOP
  window, so every bitrate step (= every ladder rung transition) becomes
  an overshoot/undershoot sawtooth as long as the GOP. gop_s 10 made
  every promote fail probation (TxQueue pinned, fabricated 13–41%
  "loss" on a clean channel). Keep GOP ≤ ~2 s while the ladder
  re-commands bitrate per rung.

## 4. Measuring it — rigs and gotchas

- **Drone side** (encoder cadence): poll `write_idx` on
  `/dev/shm/mabur_f` (static armv7 sniffer; recipe = 8-byte-aligned
  slot stride `(4+slot_data_size+7)&~7`, header 192 B). Lines:
  `mono_us pts len flags nal0`. ⚠ Under rally the periodic refreshes
  are VPS-led AUs whose slice NAL is not IDR type 19/20, so the ring
  meta IDR flag misses them — count IDRs **GS-side** (`nal0==32`), and
  don't trust drone-side class buckets under rally.
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
- **Per-rung overhead shape**: the flat ladder is uniform per *stream*;
  the rung's `cmd_overhead` still scales all streams together and the
  clamp at 2.0 flattens rung 0 fully. If a future rung wants
  differentiated protection back, it must pay the jitter price knowingly
  — the arithmetic in §1 prices it (Δair × 0.85 µs/B, ×2 in the EMA).
- **The IDR budget**: with `max_ipprop` bounding the ratio and s0 flat,
  the remaining IDR cost is pure payload (~1.7×P). The only further
  reductions are quality trades (`qp_delta`, tighter ipprop toward the
  1.4 floor) — measured, available, currently not worth it.
- **Explored and parked (2026-08-29): a 90 fps canary layer.** Base
  60 fps guaranteed + 30 fps disposable TRAIL_N on s3 (`ltr:2`-style
  period at 90 fps) would raise the shed floor from 30 to 60 fps, and
  a lighter-FEC canary is the one legitimate future use of asymmetric
  overhead (pair it with source-priority TX scheduling — the decoder's
  systematic fast path already delivers sources immediately; only the
  TX-side source/repair interleaving couples parity to completion time).
  Parked because the GS display is 60 Hz: the visible 90 fps vanishes,
  and the permanent cost (≈1/3 less bits per displayed frame at equal
  rate, or ~1.3–1.5× the air) buys only a rarely-entered shed mode plus
  ~4 ms freshness. Revisit on a >60 Hz display, or if flight data shows
  the 30 fps shed floor actually hurting. Per-stream higher MCS for the
  canary was hardware-refuted as general policy (July uniform-PHY
  ruling: s3 at +1/+2 rungs saw 42% RF loss) — canary-tolerable, but
  the wall is sharp. Any alternating bitrate ratio r:1 prices as
  EMA ≈ 2·Δair/wire-rate; 1-in-N droppable patterns cost ~2/N of the
  every-other-frame damage.
