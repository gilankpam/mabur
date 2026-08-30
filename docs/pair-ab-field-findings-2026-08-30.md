# Fixed-pair UEP field A/B — flat 50:50 vs asym 100:50, 2026-08-30

First real-dynamics answer to the fade A/B left open by the same-rate
fixed-pairs deploy (RC_VERSION 5): does the asymmetric per-rung overhead
pair buy protection worth its jitter tax outside the bench? Data comes
from the GS flight instrument (`tools/gs/flightrec.py` per-AU rows +
sideport jsonl, analyzer `tools/flightjitter.py` — see
`docs/observability.md`), two back-to-back sessions on the prod
fixed-pairs build, operator-driven RF dynamics (SNR swinging ~14↔36 dB,
full ladder workouts). Bench priors this tests:
`docs/same-rate-uep-findings-2026-08-30.md`.

## Sessions

| | 0028 (ctl-0089) | 0029 (ctl-0090) |
|---|---|---|
| pairs (all rungs) | 50:50 | 100:50 |
| video | 159 s @ 59.9 fps | 149 s @ 59.8 fps |
| lossy seconds (pre-FEC >0.5%) | 724/1082 | 564/1081 |
| SNR med / min | 29.4 / 0 dB | 31.6 / 0 dB |
| ladder events | 12 demotes (fade 4, resid 3, s3 4) + 15 promotes | 11 demotes (fade 5, resid 4, s3 2) + 15 promotes |
| median len base/enh | 18.4 / 11.9 KB | 17.1 / 10.9 KB |

Comparable scene and ladder workout; **0029 ran ~22% cleaner RF** — every
protection delta below must be read against that bias, per the
judge-Δ-columns rule (`docs/data-provenance.md`).

## Jitter: asym costs ~4 ms, at every rung

| | 0028 flat | 0029 asym |
|---|---|---|
| jitter EMA | 12.2 ms | 16.1 ms |
| residual (len-model) | 6.4 ms | 7.3 ms |
| size-explained | 47.5% | 54.8% |
| fitted slope | 0.70 µs/B | 0.93 µs/B |
| djit p50 per rung (mcs1..5) | 10.8–12.6 ms | 14.5–17.9 ms |

Mechanism pinned, not inferred: splitting the len-model residual by sid
transition isolates the overhead alternation —

| median residual | flat | asym |
|---|---|---|
| enh→base (base arriving) | +0.55 ms | **+1.91 ms** |
| base→enh (enh arriving) | −1.04 ms | **−2.63 ms** |

Base flies (1+1.0)× air per byte vs enh (1+0.5)× under the asym pair;
the every-other-frame air asymmetry (±2.3 ms vs flat's ±0.8) is the
+3–4 ms the EMA gained. Same mechanism the bench sweep predicted
(+1.5–3 ms); field motion sits at the top of that range.

Context from the same instrument (session 0028 + bench session 0026,
flat, same build): the transport residual is ~6.1–6.4 ms bench and
field alike — the field-vs-bench jitter gap is scene-driven encoder
size variance (size-explained 10% static → ~48% under motion; mcs5
djit p50 4.85 → 10.9 ms), NOT transport degradation. All 9 damaged AUs
in 0028 landed 40–300 ms *before* a demote — fade damage during
detection latency; the demote machinery itself reacted correctly.

## Perceptual: why asym *felt* much worse than +4 ms

The EMA averages every frame; eyes track holes and stalls, and those
doubled (operator report "much worse" — the metrics agree, on the
*cleaner* channel):

| | 0028 flat | 0029 asym |
|---|---|---|
| frame holes/min (missing frame → 33 ms display hold) | 14.0 (1 per 4.1 s) | **29.3 (1 per 1.9 s)** |
| stalls >50 ms /min | 18.5 | 25.3 |
| stalls >80 ms /min | 5.7 | 8.0 |
| interval p99 / max | 43 / 116 ms | 45 / 152 ms |

The dominant term is missing-enh holes (the airtime-pressure regression
below): a 33 ms hole every ~2 s barely moves an α=1/16 EMA but is
plainly visible. `flightjitter.py` reports `holes_per_min` /
`stalls50_per_min` / `stalls80_per_min` / `iv_p99_ms` in the summary
since this study — judge configs on those, EMA second.

## Protection: no visible win, and an enh regression

- Damaged AUs (post-FEC incomplete): 9 → 5 (base 3→1, enh 6→4). Right
  direction but small n on a cleaner channel — not creditable to the
  pair.
- Mid-stream base losses (sid 1→1 seams): 1 → 0. n=1, no signal.
- **Enh losses doubled**: missing-enh seams (sid 0→0) 31 → 64, fid gaps
  3 → 14 — despite enh's own overhead being unchanged (0.5 both
  configs) and the cleaner RF. Suspected mechanism: the larger total
  overhead squeezes the air budget (median lens −7% confirm the lower
  commanded rate) and enh pays the congestion. Plausible, unproven —
  worth a targeted look if asym is ever revisited.

## Verdict

Flat 50:50 wins this regime in the field, agreeing with the bench
sweep: asym's measured win only appears under sustained ≥3–4% loss,
which even these deliberately-faded sessions do not sustain on average.
The jitter tax, by contrast, is paid on every frame at every rung.

**Candidate follow-up (config-only):** the v5 architecture prices pairs
per rung, so protection can go where sustained loss actually lives —
50:50 on rungs 3–5, 100:50 on rungs 1–2 only. Untested; would need the
same two-session A/B through the instrument.

## Method notes / gotchas

- Sessions must be compared by Δ, not absolutes: scene and RF drift
  between sessions (0026's static bench vs 0028/0029 motion differ 2×
  in djit at the same rung and build).
- The len-only size model absorbs *average* overhead into its fitted
  slope (0.70 flat → 0.93 asym); per-sid asymmetry appears as
  sid-correlated residual, which is exactly what the transition split
  measures. Don't read `size_explained_frac` across configs with
  different pairs without that split.
- DISCONT-flagged AUs cluster in the first ~1 s of a session (drone
  start seam) — exclude them (the analyzer does) before counting gaps.
