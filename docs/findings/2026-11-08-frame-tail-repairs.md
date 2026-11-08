# Frame-end FEC repairs: A/B of PR #117 and the frame-tail gate — 2026-11-08

**Status:** `docs/airtime-model.md` "Repairs at the frame end" owns the rule as built on branch `fec-frame-tail`. The gate below failed the 24 Mbps burst rows against window-64 continuous placement; whether to ship is the operator's call.

## Question

PR #117 (merged as master `43c3905f`) holds a video frame's FEC repair
credit to the frame end, in 64-symbol blocks, with `fec.window = 64` and
`fec.repairs_with_frame = true` as the bundle default. Does it improve
latency and loss, and at what cost? Then: does the frame-tail rule on
`fec-frame-tail` (whole-frame hold only for frames that fit the window;
for bigger frames, continuous placement except the last
`ceil(blocks_per_body / overhead)` symbols, held to the frame end) keep
the gains without the losses?

## Rig

- GS: a `MABUR_LOSS_SIM` build of `maburgs` (built with
  `tools/bench/fecab/build-losssim-arm64.sh`) on a copy of the GS config
  pinned to `static_mcs 4`, `static_bw 40`, overheads 0.5 / 0.25, channel
  144. NACK on, as flown.
- Drone: 1280x720 at 100 fps, `blocks_per_body = [4, 4]`, symbol size 332.
- Per arm, three 60 s steps on both video layers: clean; 2 % union loss in
  single drops (`eff=2 burst=1`); 5 % union loss in bursts of 3
  (`eff=5 burst=3`). Steps run detached on the GS so a dropped host link
  cannot mislabel a step. Arms interleaved, two repetitions each.
- Metrics (`tools/bench/fecab/analyze.py`): AU truncation from `au.log`
  flags; per-AU latency excess over a rolling 2 s median of
  `t_complete − pts` per layer; display e2e and frames not displayed from
  `frame.log`; per-thread drone CPU from `/proc/<pid>/task/*/stat`.
- Arms:
  - A: window 32, `repairs_with_frame = false` (continuous placement).
  - B: window 64, `repairs_with_frame = true` (PR #117).
  - C: window 64, `repairs_with_frame = false`.
  - D: the frame-tail rule (no config key), window 64.

## PR #117 A/B (2026-11-07)

Values are rep 1 / rep 2.

**~20 Mbps, ~77 symbols per frame (sessions 0367–0373):**

| | A | B | C |
|---|---|---|---|
| 2 %: AUs > 10 ms late | 4.9 / 5.4 % | 2.1 / 2.8 % | 7.0 / 8.2 % |
| 2 %: AUs > 20 ms late | 0.6 / 0.8 % | 1.3 / 1.8 % | 0.9 / 0.8 % |
| 2 %: enhance AUs truncated | 0.07 / 0.19 % | 0.59 / 0.89 % | 0 / 0.04 % |
| 5 % burst: AUs truncated | 5.1 / 5.6 % | 5.0 / 4.3 % | 0.9 / 1.4 % |
| 5 % burst: not displayed | 23 / 24 % | 20 / 19 % | 12 / 14 % |
| maburd CPU, clean (one core) | 68 % | 76 % | 76 % |

**8 Mbps, ~30 symbols per frame, none over 64 (sessions 0376–0381):**

| | A | B | C |
|---|---|---|---|
| 2 %: AUs > 10 ms late | 3.9 / 5.5 % | 0.5 / 0.7 % | 5.9 / 5.4 % |
| 2 %: not displayed | 1.9 / 2.6 % | 0.4 / 0.6 % | 2.4 / 2.6 % |
| 2 %: display e2e p50 | 34 ms | 26–27 ms | 34 ms |
| 5 % burst: AUs truncated | 1.8 / 2.0 % | 0.65 / 0.76 % | 0.58 / 0.72 % |
| 5 % burst: not displayed | 13.8 / 14.4 % | 7.6 / 8.4 % | 10.8 / 9.8 % |
| maburd CPU, clean (one core) | 39 % | 37 % | 43–44 % |

**Why B truncates big frames:** a 77-symbol frame spends 16 repairs on
its first 64 symbols. The remaining ~13-symbol tail gets only its own
rounded share (3–4 repairs, often fewer than one body of 4 symbols). The
next frame's repairs are held to 64 symbols into that frame, so they never
reach back to this tail.

In `fec.log` at 2 % loss, every enhance loss B failed to recover was a
single-body loss with only 2–3 covering repairs. In A and C, a
single-body loss always had at least 8 (A) or 14 (C) covering repairs.
The enhance layer has no NACK, so these losses are not refilled.

**On a clean link** the three arms measure the same: the arrival span
from first to last body is 2.2 ms base and 3.3 ms enhance in every arm.
The whole frame enters the encoder at once, and worker-built repairs
mostly ship behind the frame's sources in every mode.

**Real flights:** frames exceed 64 symbols for 51–95 % of base and
29–85 % of enhance frames in flights 0361–0363.

## How well the loss sim matches flight (flights 0358–0363 vs the sim)

- **Calibration holds:** nominal 2 % and 5 % delivered 2.0–2.2 % and
  4.6–5.0 % union loss.
- **Per-card independence holds:** in flight seconds with > 1 % union
  loss, each card lost 10–27 % and the union 3–7 %. At 1 s resolution
  the union is about card 0 × card 1 (ratio 0.7–1.0), so a sim step
  matches a real fade second.
- **What the sim does not reproduce:**
  - Time clustering: real union loss averages 0.1–0.3 %, about 90 % of
    seconds have none, and half of all loss falls in 10–27 s per flight.
  - Burst mix: 60–80 % of real episodes are single bodies, but 15–57 % of
    lost symbols sit in bursts of 5 or more bodies; 2 % b1 has almost no
    bursts and 5 % b3 too few singles.
  - Layer coupling: only 35–49 % of real lossy seconds hit both layers.
  - Corrupt bodies (the sim only drops CRC-good ones), and uplink loss
    during a fade.
- **Metric trap:** the sideport's `recovered` counter includes repairs
  that won a race with a late source (`recovered_arrived`). With frame-end
  repairs this reads as ~0.5 % loss on a clean link; subtract it.

## Frame-tail gate (2026-11-08)

Gated `maburd` md5 `eb1a78c2`, built from `fec-frame-tail`; B and C ran
master's binary. Pass rule: the mean of D's two reps must fall inside the
reference arm's two-rep range widened by its spread, on the named rows.
The reference is B at 8 Mbps and C at 24 Mbps.

**8 Mbps (sessions 0386–0391): pass.** D sits at or better than B's range
on every row:

| | B | C | D |
|---|---|---|---|
| 2 %: AUs > 10 ms late | 0.83 / 0.69 % | 4.41 / 6.13 % | 0.54 / 0.54 % |
| 2 %: not displayed | 0.61 / 0.56 % | 1.61 / 2.56 % | 0.39 / 0.50 % |
| 2 %: display e2e p50 | 26.9 / 25.6 ms | 34.2 / 34.4 ms | 26.4 / 25.8 ms |
| 5 % burst: AUs truncated | 0.88 / 0.84 % | 0.61 / 0.56 % | 0.77 / 0.72 % |
| 5 % burst: not displayed | 9.48 / 8.47 % | (stall) / 10.92 % | 8.97 / 8.38 % |
| maburd CPU, clean | 36.8 / 36.0 % | 41.3 / 40.7 % | 36.1 / 36.0 % |

**24 Mbps, ~80–95 symbols per frame (sessions 0396–0401): fail on the
burst rows.**

| | B | C | D |
|---|---|---|---|
| 2 %: AUs > 10 ms late | 5.06 / 3.82 % | 9.36 / 11.81 % | 3.06 / 3.46 % |
| 2 %: not displayed | 1.95 / 1.48 % | 2.74 / 3.39 % | 1.00 / 1.11 % |
| 2 %: enhance AUs truncated | 0.37 / 0.44 % | 0 / 0 % | 0 / 0 % |
| 5 % burst: AUs truncated | 4.29 / 3.88 % | 1.06 / 1.11 % | 1.64 / 1.39 % |
| 5 % burst: enhance truncated | 8.61 / 7.80 % | 2.12 / 2.23 % | 3.29 / 2.80 % |
| 5 % burst: AUs > 10 ms late | 27.01 / 24.59 % | 21.71 / 22.66 % | 24.54 / 23.03 % |
| 5 % burst: not displayed | 18.66 / 17.60 % | 13.40 / 14.43 % | 15.89 / 14.09 % |
| maburd CPU, clean | 87.0 / 86.2 % | 87.2 / 89.3 % | 87.6 / 87.4 % |

- **Light loss:** D removes B's enhance truncation and has the fewest
  late AUs and frames not displayed of the three.
- **Bursty loss:** D truncates about 40 % more than C, though about
  2.7× less than B. Burst truncation, enhance truncation and late AUs are
  all above C's widened range.
- **Likely cause (not proven):** a big frame's tail repairs, about one
  body's worth, ship together in one body right behind the tail. A burst
  covering the frame's last bodies takes the tail and those repairs
  together, while C ships the same credit spread later.
- `backlog_drops` 0 in every arm at both rates; D's frame-end batch costs
  61–63 µs per repair.

## Run notes

- **Scene complexity:** a first 24 Mbps run (0392–0394) was discarded.
  The scene was too simple: the encoder produced ~6 Mbps at a 24 Mbps cap
  (`enc_pk100` ~6000k), so no frame exceeded the window. The rerun had
  `enc_pk100` 19–25 Mbps.
- **Player stall:** C rep 1 at 8 Mbps (0387) shows 77 % "not displayed"
  in its burst step. That is a player stall, not link loss: `frame.log`
  ends ~52 s before `au.log` and `reg.log` counters freeze. The next
  session logged normally. It happened on a master-binary arm.
- **Runner bug, fixed on the branch:** a bind mount over the running
  `/usr/bin/maburd` cannot be unmounted normally (busy), so binds stacked
  across arms; the runner now unmounts lazily. The md5 check of the
  running process held on every arm, so the data stands.
