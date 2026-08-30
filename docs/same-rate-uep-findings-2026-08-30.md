# Same-rate UEP vs rate-split — findings, 2026-08-30

Bench study answering "can we go back to one PHY rate for both streams,
with UEP expressed only through per-layer FEC overhead?" — run after the
pinned-mcs1 session below exposed the rate-split's structural limit at the
bottom of the MCS table. Ten static operating points plus a ten-point
motion sweep (operator waving at the camera). Code lives on branch
`uep-same-rate-sweep` (6851e9e); the override endpoint is durable tooling,
the `ladder_from` change is the experiment itself.

## Background: the pinned-mcs1 budget break that started this

With the link pinned `max_mcs 1` (rate-split: base mcs0 / enh mcs1), the
mcs1 rung's `overhead 1.0` + `airtime_budget 0.5` computed a ~1.9 Mbps
encoder target for 1080p60. Air duty measured 0.67 sustained / 0.8 waving
(operator OSD), with visible stutter. Compound cause, each part necessary:

- `run_bitrate_policy` is open-loop against the encoder: budget→kbps
  trusts VENC compliance; actual emitted air never feeds back. At the
  ~1.9 Mbps floor the encoder emitted +21% **on a static scene**.
- star6e H265 CBR converges over the GOP (2 s) and has no enforceable
  frame-size cap (`u32MaxISize`/`MaxPSize` dead — airtime-model.md), so
  motion bursts serialize whole: one 15 KB P frame at mcs0 = 23.5 ms of
  air, 1.4 frame-slots. 6% of base frames individually exceeded a slot.
- cmd 1900 < `roi_threshold_kbps` 3000 flipped the ROI center band to
  QP −24 — maximum quality boost exactly where the waving hand is,
  exactly when the pipe is smallest.
- No relief valve: single-rung ladder can't demote; the congestion guard
  keys off USB failures, not air (documented mis-wiring, rc_agent.cpp).

Config fixes adopted same day: mcs1 rung ov 1.0 → 0.5 (lifts cmd above
both the encoder floor and the ROI threshold) and `airtime_budget`
0.5 → 0.7. The open-loop policy remains a known gap.

Balancer observation while pinned there: the mcs1 rung's 2:1 base/enh
rate ratio is **mathematically unbalanceable** under a repair-neutral
budget — perfect air balance requires ov_base = 0, and the 0.5×ov_cmd
rail (which is the UEP protection floor, not an accident) stops at 0.25.
Residual: base 1.42× enh air = 3.3 ms/pair alternation. The rate ratio is
worst at the table's bottom (6.5→13 doubles; 39→52 is 1.33×), i.e. the
split is weakest exactly at the emergency rung.

## Tooling (durable)

- **Same-rate mapping**: `ladder_from` base rides the scored mcs
  (`common/src/profile.cpp`, was `max(m−1,0)`). Both ends must run the
  branch together — the GS mirrors the mapping for its air/phy display.
  No RC wire change, no config keys.
- **Volatile per-layer overhead override** (drone `:8301`):
  `POST /venc/set?ov_base_pct=N` / `ov_enh_pct=N` (−1 clears; both ≥ 0 to
  arm). Backed by `BalancerFeed::ovr_*_pct`: `AirBalancer::solve` returns
  the forced pair verbatim (rails bypassed, anchors kept coherent) and
  `run_bitrate_policy` blends the per-layer values so the budget target
  stays honest. Cleared on daemon restart.
- Per-AU capture: `audump.py` variant of the ausniff reader emitting
  `(t_ms, rec, fid, pts, sid, flags, len, nal0)` rows; jitter = player
  EMA on ring-arrival intervals; alternation = per-pair base−enh air gap.

## Static sweep (60 s/point, budget 0.7, GS `static_mcs` pins)

flat = forced 0.5/0.5, asym = forced 1.0/0.5. Balancer forced OFF in both.

| rung | config | cmd kbps | air % | jitter ms | alternation ms |
|---|---|---|---|---|---|
| mcs1 | flat | 5500 | 68 | 8.2 | ~0 |
| mcs1 | asym | 4800 | 70 | 8.4 | +3.4 |
| mcs2 | flat | 8300 | 68 | 4.4 | ~0 |
| mcs2 | asym | 7200 | 71 | 6.9 | +3.5 |
| mcs3 | flat | 11100 | 68 | 9.8 | **−5.6** |
| mcs3 | asym | 9600 | 66 | 8.4 | +3.3 |
| mcs4 | flat | 12000 | 49 | 7.8 | ~0 |
| mcs4 | asym | 12000 | 56 | 9.8 | +2.6 |
| mcs5 | flat | 12000 | 37 | 9.4 | −0.2 |
| mcs5 | asym | 12000 | 42 | 10.8 | +1.9 |

- Asym cost ≈ `L·8·Δov/rate`: +3.4 ms alternation at low rungs,
  shrinking where `bitrate_max` caps L. EMA cost ~+2–2.5 ms where sizes
  are flat, invisible at mcs1 (variance floor 8.2 ms swallows it).
- **mcs3 flat upset**: the encoder drifted enh-heavy (lenB 17.2 K vs
  lenE 28.8 K) → alternation −5.6, jitter 9.8, flat LOST. This is the
  spike-1 scene regime (enh ≫ base) resurfacing — with the balancer
  forced off, nothing absorbed it.

## Motion sweep (operator waving, 30 s/config/rung)

| rung | config | duty p50/max | jitter / peak ms | alternation |
|---|---|---|---|---|
| mcs1 | flat | 0.61 / 0.66 | **6.9** / 15.5 | 1.4 |
| mcs1 | asym | 0.49 / 0.66 | 8.8 / 15.2 | 3.3 |
| mcs2 | flat | 0.62 / 0.68 | **6.3** / 12.8 | 1.0 |
| mcs2 | asym | 0.61 / 0.73 | 7.8 / 18.8 | 3.6 |
| mcs3 | flat | 0.62 / 0.67 | **6.1** / 22.1 | 0.7 |
| mcs3 | asym | 0.55 / 0.69 | 8.4 / 23.1 | 3.3 |
| mcs4 | flat | 0.45 / 0.50 | **6.8** / 17.2 | 0.3 |
| mcs4 | asym | 0.53 / 0.60 | 9.7 / 22.2 | 2.7 |
| mcs5 | flat | 0.34 / 0.38 | **8.0** / 17.4 | 0.2 |
| mcs5 | asym | 0.40 / 0.44 | 10.8 / 20.0 | 2.0 |

- **Flat won every rung under motion** (+1.5–3 ms asym penalty). The
  static mcs3 size-drift upset did not recur: motion loads both layers
  and the class sizes stay matched.
- **Jitter peaks 15–23 ms are config-independent** — pure frame-size
  variance (base p99 hit 42–90 KB); no overhead arrangement touches
  them. Only PHY rate shrinks them.
- fps 59.7–60 and drops ≤1 at every point; duty peaked 0.73 vs the 0.7
  budget. The pinned-mcs1 stutter did not reproduce at the new operating
  point.

## Reading it (and squaring with spike 1)

Jitter is per-frame air balance `len·(1+ov)/rate` — the one rule that
explains every row here plus the 2026-08-29 spikes plus uep-flatten:

- Sizes matched (flat refs doing their job): same-rate + flat ov = zero
  alternation → best. This sweep.
- Sizes skewed enh-heavy (scene/bitrate-dependent): naive same-rate
  alternates; spike 1's rate-split fixed it *for that scene*, this
  sweep's mcs3-flat row reproduced the problem with the balancer off.
- The rate-split's own residual: at 2:1 rate ratios the balancer rails
  can't close the gap (3.3 ms structural at the mcs1 rung).

So the jitter-optimal shape is **same-rate + balancer ON**: at equal
rates the balancer's air-balance target is always reachable (no rail
pinning) and it absorbs size drift automatically, converging to ~flat
when sizes match. Overhead asymmetry is then a deliberate purchase:
~2–3 ms of EMA for repair-margin UEP, decided per rung, not a structural
constant.

**What same-rate gives up — unmeasured here**: base's PHY fade margin.
Both streams share one wall; in an SNR fade nothing degrades to a base
floor, and at a pinned bottom rung there is no demote below. The bench
sits at 33 dB SNR and cannot test this. That trade, not jitter, is the
open question before same-rate goes near a range flight.

## Loss-jitter sweep (same day, pinned mcs2, MABUR_LOSS_SIM)

GS restarted under `S96maburgs.losssim` (control udp :8390, `sN eff=<pct>
burst=<n>`), loss injected on BOTH streams, 30 s/point, static scene.
`eff` is the union dial; decoder-visible loss confirmed ≈ dialed via the
per-card sideport counters (per-card ≈ √eff at 2 cards). Same forced
flat/asym configs as above. Jitter EMA (Δ vs own clean baseline):

| eff loss | flat jit (Δ) | asym jit (Δ) | flat p99 ia | asym p99 ia |
|---|---|---|---|---|
| 0% | 7.8 (—) | 8.7 (—) | 34 ms | 39 ms |
| 2% | 9.3 (+1.5) | 9.0 (+0.3) | 39 | 41 |
| 5% | 12.0 (+4.2) | 10.9 (+2.2) | 45 | 50 |
| 10% | 15.8 (+8.0) | 14.3 (+5.6) | 50 | 54 |
| 5% burst4 | 13.9 (+6.1), pk 26 | 12.3 (+3.6), pk 22 | 58 | 57 |

- **Loss-jitter is recovery-wait**, and it dwarfs the clean-channel
  alternation question: +4 ms at 5%, +6–8 ms at 10%/bursty, vs the
  ~1–3 ms flat-vs-asym gap when clean.
- **The flat-vs-asym verdict flips under loss**: asym's extra base
  repair density roughly HALVES the jitter growth at every loss point
  (crossover ~3–4%). UEP protects cadence, not just delivery.
- Bursty 5% (B=4) ≈ Bernoulli 10% in severity; p99 inter-arrival
  reached 58 ms against the 75 ms decode deadline — margin thins fast.
- Caveats: 30 s single-shot points (drop/abandon counter cells are
  noisy); baselines here (7.8/8.7) sit above the morning static sweep's
  (4.4/6.9) — scene drifted between sessions; judge Δ columns, not
  absolute rows. Per findings-rig rule, loss quoted from sideport
  counters, not the dial.
- **Implication for deferred-repair scheduling**: deferral moves repair
  later, i.e. spends exactly the currency (recovery wait) this table
  shows is already the dominant jitter term at ≥5% loss. Its clean-air
  win (~2–3 ms) must be weighed against worsening the +4–8 ms column —
  measure deferral under THIS rig before believing in it.

## State / rollback

Bench runs the `uep-same-rate-sweep` binaries on BOTH ends (same-rate
mapping live at every rung, balancer active, override cleared). Rollback
pair: `maburd.pre-sweep` (drone) + `/usr/local/bin/maburgs.pre-sweep`
(GS) — swap both or the GS air/phy display lies about the base stream.
Prod configs as of today: drone `airtime_budget 0.7`, GS mcs1 rung
`overhead 0.5`.

## Open

- Fade behavior of same-rate vs split at the wall (range or attenuator).
- Balancer-ON same-rate baseline per rung (this sweep forced it off).
- Closed-loop bitrate policy (feed actual emitted air back into the
  target) — would have caught the pinned-mcs1 break by itself.
