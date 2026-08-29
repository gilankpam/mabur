# Air-time balance spikes — findings, 2026-08-29

Two same-day bench spikes behind the air-time-balanced UEP design (rate-split
streams + overhead balancer). All spike code was throwaway (env-gated,
reverted, never merged); the numbers here are the durable output.

## The metric: base−enh completion offset

Per-AU completion offset = `arrival_mono_us − pts`, min-normalized over a
capture (only relative delays are meaningful). The **offset** is
`p50(base AUs) − p50(enh AUs)`: the systematic, class-dependent part of
delivery delay. Because base/enh strictly alternate at frame cadence, a
nonzero offset is a long/short/long/short sawtooth in AU inter-arrival —
the *cause* of which the sideport `jitter_ms` EMA is the *symptom*.

Reproducibility across repeat runs: offset ±0.5 ms; jitter EMA ±1.4 ms
(the EMA also swallows random encoder frame-size variance, so it is the
noisier endpoint — judge balance interventions on the offset).

Capture tool: `aucadence.py` on the GS reading the AU ring from outside the
daemon (same non-circular posture as ausniff); rows
`(mono_us, pts, sid, fid, len, flags, nal0)`.

## Spike 1 — does base-at-lower-MCS reintroduce jitter?

Setup: GS `static_mcs 1`, drone `bitrate_max_kbps` pinned 2600 in BOTH arms
(the bitrate policy keys off `ladder[1]`, so the split run would otherwise
command a different bitrate). Throwaway env knob forced s0–s2 to mcs0 while
s3 stayed at the commanded mcs1. A/B/A/B, 4×25 s captures.

| run | jitter EMA | base−enh offset p50 | len p50 base/enh |
|---|---|---|---|
| baseline #1 (both mcs1) | 1.84 ms | −2.08 ms | 1090 / 2213 B |
| baseline #2 | 2.90 ms | −2.39 ms | 1320 / 2594 B |
| split #1 (base mcs0) | 1.07 ms | −0.06 ms | 1446 / 3277 B |
| split #2 | 1.22 ms | −0.11 ms | 1319 / 2699 B |

**The split HALVED jitter.** At this scene/bitrate the encoder emitted
enhance frames ~2× base size, so the baseline already alternated (enh
slower); halving base's PHY rate rebalanced air almost exactly. The offset
moved by precisely the physics-predicted amount (positive control that the
override took).

**Superseding insight** (vs the naive "any asymmetry = jitter" reading of
the uep-flatten result): jitter is per-frame air-time balance
`len·(1+ov)/rate`. Equal overhead only equalizes air when the class sizes
are equal — which is scene/bitrate-dependent.

## Spike 2 — size-ratio sweep + throwaway balancer

Throwaway balancer: closed-form air-neutral overhead split (nominal
`len·(1+ov)/rate` model), per-stream clamp `[0.5×, 2×]` of commanded,
len EWMAs (α 1/16) in the frame pipeline, per-layer `set_overhead`.

### Phase A — len ratio vs bitrate (mcs5 pinned, same rate, no balancer)

| cap kbps | actual Mbps | len ratio base/enh | jitter EMA | offset |
|---|---|---|---|---|
| 1500 | 0.24 | 1.19 | 0.93 | +0.13 |
| 3000 | 2.81 | 1.12 | 2.01 | +0.55 |
| 6000 | 5.73 | 1.08 | 2.58 | +0.84 |
| 10000 | 9.71 | 1.05 | 5.29 | +1.33 |

Ratio converges toward 1 as bitrate rises — and across scenes it FLIPS
(spike 1's scene and the evening mcs1 runs sat at 0.49–0.50, enh 2× base).
**Static split-compensation tables cannot work; the ratio must be measured
live.** Also: same-rate EMA at ~9.7 Mbps ≈ 5.3 ms matches the prod
flatten-era figure — most of it is frame-SIZE variance (encoder RC), which
no stream-level mechanism can remove.

### Phase B — balancer × rate matrix (offset OFF → ON per op point)

| op point | offset OFF → ON | note |
|---|---|---|
| mcs3, same rate | +0.73 → **+0.08** | exact |
| mcs3, split (base mcs2) | +0.84 → **+0.07** | exact |
| mcs1, same rate | −2.03 → −1.31 | fully railed at clamps (enh 2× base needed more trim than the rails allow) — correct partial behavior |
| mcs1, split (base mcs0) | −1.75 → **−0.54** | |
| mcs5, same rate | +1.33 → +0.82 | partial |
| mcs5, split (base mcs4) | +1.28 → **−2.32 / −2.79** | overshoots through zero, ~2× gain, REPRODUCIBLE |

Jitter EMA favored balancer-ON in 5/6 pairs (best: mcs3 same-rate
5.67 → 3.06 ms). The split alone (balancer off) never worsened jitter at
any rung.

### The overshoot ruling

Identical rate ratio (1.33) and identical ov split (0.28/0.80) landed the
offset at +0.07 with ~12.7 KB frames (mcs3) but −2.8 with ~20 KB frames
(mcs5): the nominal `len·(1+ov)/rate` model has an op-point-dependent gain
error up to ~2×. **A production balancer must anchor on ACTUAL emitted
air** — the drone holds the exact body bytes returned by every
`add_frame` (they include SBI/frag framing and repair quantization) — with
the nominal model only as an unseeded fallback. This is §2 of the design
spec (`docs/superpowers/specs/2026-08-29-airtime-balance-uep-design.md`,
this machine).

## Method gotchas (for the next bench session)

- `tools/build-arm.sh` outputs to **`build-arm-glibc/`**; `build-arm/` is a
  stale pre-fold-in tree whose maburd still parses `waybeam.*` keys — its
  "config: encoder: unknown key" boot failure is misleading. Strings-check
  a new key (e.g. `max_ipprop`) before shipping a binary.
- Run spike binaries from drone `/tmp` (tmpfs): sidesteps the 5.8 MB rootfs
  two-binary limit entirely.
- Pin `bitrate_max_kbps` identically across arms of an A/B whenever a
  ladder slot's rate changes — the bitrate policy reads `ladder[1]`.
- The drone rootfs has no python3; edit configs there with `sed`.
- `n_idr = 0` in captures is expected under the rally preset
  (intra-refresh, no periodic IDRs) — not a broken IDR-flag scan.
