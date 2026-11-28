# Wall measurement from two probes — spike, witnesses and bench

**Status:** built; the measurement this record led to is described in
`docs/calibration.md`. The spike's scripts and captures were throwaway and
are not in the repository; the reference for the fit is
[`tests/fixtures/crosspath/pafit_ref.py`](../../tests/fixtures/crosspath/pafit_ref.py).

Question: the drone's self-measured walls
([2026-11-26-drone-self-calibration-cross-path](2026-11-26-drone-self-calibration-cross-path.md))
cost 9 s (20 MHz) to 17 s (40 MHz) of radio silence per channel and width,
refuse the whole 5.2/5.3 GHz band on the bench drone, and rest on EVM
thresholds taken from one unit. Is there a better drone-only way?

Two units throughout: the bench drone, and a second RTL8812EU card of the
same RFE type ("the second unit").

## What was ruled out

- **The chip's TSSI detector.** The vendor's 5 GHz TSSI setup and readback
  sequence, run as a register script during a continuous MCS7 train on the
  second unit, channel 144: the detector answers (done bit set), reads its
  rail up to TXAGC index 80, then falls 12–14 units per dB of drive in a
  straight line to index 127 on path A (path B bends slightly at the top).
  The radiated level over the same indices rises 1.3 dB from 84 to 100 and
  is flat above. The detector follows the chip's drive, not the FEM: blind
  to the compression that sets the walls, like the DPK loopback.
- **The efuse anchor plus a constant.** On the drone the −17 dB crossing sits
  at anchor + 14.5 … + 22 on five channels while the anchor itself moves 36
  steps. The second unit's anchors are coarse (65, 69, 70 and 72 over the
  whole band) and its crossing sits at anchor + 12 … + 28. A constant table
  would be off by up to about ±2 dB between these two units.
- **A coherent parked-minus-unparked subtraction** to cancel the leak on
  weak-park channels: the phase between captures jumps by 90 or 180 degrees.
- **A clean pre-PA copy as the explanation of the unparked reading.** On
  channels 124–165, where the unparked listener reads 2–4 dB better than the
  parked one, the fitted unparked output is flat in deep saturation; a clean
  copy would keep rising.

## The fit

One captured frame holds the PA's whole curve: the envelope of an OFDM frame
swings to about 8.5 dB above its mean, and the drone knows the waveform it
sent. Fitting a complex gain against instantaneous input amplitude to two
frames of one probe (piecewise-linear over knots 2 dB apart, each tone's
phase pinned by the HT-LTF sequence, alternating least squares with the
per-tone channel) and scaling the waveform down predicts the EVM of lower
TXAGC indices.

Offline, on a survey of both units (every 5 GHz channel at 20 MHz, twelve
pairs at 40 MHz, 25 indices × up to 8 captures each), against the 25-point
sweep of the same run:

- A fit is accurate when its probe is moderately compressed: taken at the
  lowest index whose own EVM is at or above −11.5 dB it reproduces the
  −17 … −12.5 dB crossings within 1.8 steps on the drone at 20 MHz (28 runs)
  and 1.9 at 40 MHz. Taken deeper (own EVM −10 dB or above) it reads 1–5
  steps optimistic; at a fixed index (124 or 112) it fails by tens of steps
  where that index is far above the wall. One curve fitted over many indices
  at once reads 3 dB optimistic.
- The −9 dB crossing needs a second probe at the lowest index whose own EVM
  reaches −9 dB.
- A prediction rises again more than about 40 steps under its probe (few
  samples reach the low knots).
- With the rules as built (the drone's gain rule, knots relative to the
  frame's rms, an assumed −27 dB listener floor), fit minus sweep crossing in
  steps, mean / most optimistic:

  | | −9 | −12.5 | −15 | −15.5 | −17 | −19.5 | −21 |
  |---|---|---|---|---|---|---|---|
  | drone 20 MHz | −1.0 / −0.1 | −1.1 / −0.1 | −1.3 / −0.1 | −1.3 / +0.1 | −1.5 / +0.4 | −1.9 / +0.5 | −2.5 / +0.6 |
  | drone 40 MHz | +0.2 / +2.5 | −0.5 / +0.3 | −0.3 / +0.7 | −0.3 / +1.2 | −0.7 / +1.4 | −1.3 / +2.0 | −2.4 / +0.5 |
  | second unit 20 MHz | −0.9 / −0.2 | −1.3 / −0.4 | −1.9 / −0.3 | −1.9 / +0.2 | −2.0 / −0.3 | −1.6 / +7.9 | −1.8 / +10.3 |
  | second unit 40 MHz | −0.3 / +0.8 | −0.8 / −0.2 | −1.1 / +0.3 | −1.2 / +0.3 | −1.4 / +0.6 | −2.2 / −0.7 | (floor-bound) |

  The second unit's +7.9 and +10.3 are one run, at thresholds close to that
  unit's own sweep floor; not explained.
- A floor could not be measured cheaply: the park check's own capture (index
  28, weakest listener word) is noise-limited on the weak band, and a floor
  probe 40 steps under probe 1 underestimated the second unit's floor.
- Fit residual (unexplained over received power) minus the frame's own EVM
  on clean probes: 20 MHz −11.5 … −9.8 dB (probe 1) and −10.2 … −7.1 dB
  (probe 2); 40 MHz −7.3 … −5.7 dB and −6.0 … −3.6 dB. A second capture
  added to a clean probe at −14 dB relative or stronger makes every
  predicted point read worse and brings that difference to −3.9 dB or above;
  at −20 dB the prediction moves by up to ±0.75 dB and it reads −7.2 dB.

## Delivery witnesses

A burst per TXAGC index of MCS7 and of MCS4 (LDPC + STBC, both PAs), the
other unit receiving with per-frame output; cliff = 50 % delivery. The
self-measured EVM (25-point sweep, PA A) read at the cliff:

| | MCS7 | MCS4 |
|---|---|---|
| drone sends: channels with a park rise ≥ 12 dB (100, 112, 128, 144, 161) | −15.1 … −18.2 | −10.6 … −11.2 |
| drone sends: weak-park channels (40, 52, 56, 60, 64; rise 7.4 … 9.6 dB) | −18.4 … −19.4 | −12.1 … −12.8 |
| second unit sends: rise ≥ 12 dB (40, 52, 64, 100, 112, 128, 144, 153, 157) | −15.9 … −20.8 | −10.6 … −14.9 |
| second unit sends: weak park (149, 161, 165) | −18.8 … −19.7 | −12.5 … −13.3 |

- On the drone the weak-park band reads 1.5 dB (MCS4) to about 2.4 dB (MCS7)
  too good, within ±0.5 dB over the band: its curves are PA-shaped and
  usable with lower thresholds.
- On the second unit a weak park does not mark a channel as more optimistic
  than the others. With a curve re-measured the same day, its MCS7 cliff
  sits 4.6–6 steps under the −17 dB crossing on channels 128, 144 and 161
  and 3.3–4.6 steps over it on 100 and 64: the thresholds are off by about
  ±5 steps (±1.5 dB) per channel on that unit, more than the default 1 dB
  margin. That unit's sweeps also move 3–4 steps from day to day.
- The receiver's own EVM report at the last delivered index is the same on
  every channel and in both directions (about −40 … −45 in the receiver's
  unit).
- The router on channel 48 at 80 MHz did not disturb channel 40: delivery
  was 100 % up to its cliff.

## Bench, the built measurement

Bench drone, `radio.width = 40`, the measurement triggered over the cal
lane, the old 25-point binary and the new one in one session. Walls are
MCS3 … MCS7 relative to the anchor.

| channel / width | old: walls, time | new: walls (repeats), time |
|---|---|---|
| 144 / 20 | 63 39 29 28 22, 9.4 s | 63 35 25 23 19 · 63 36 25 24 19 · 63 38 28 26 22, 1.5 s |
| 144 / 40 | 57 36 28 26 21, 16.4 s | 54 34 25 24 20 · 55 37 29 27 23, 2.8–2.9 s |
| 112 / 20 | 62 35 24 23 17, 9.3 s | 58 33 23 21 16, 1.8 s |
| 112 / 40 | 52 34 25 23 18 · 53 34 25 23 19, 17 s | 51 33 24 23 18, 3.5 s |
| 161 / 20 | 63 34 23 21 16, 9.2 s | 61 32 21 19 15, 1.7 s |
| 161 / 40 | 53 31 22 20 16 · 51 31 22 21 17, 16.5 s | 53 31 22 20 16, 2.9 s |
| 64 / 20 | refused (`unparked`) | 35 21 14 13 9, weak park, 1.8 s |
| 64 / 40 | refused | 34 23 16 14 11, weak park, 3.3 s |
| 40 / 20 | refused | 35 20 13 11 8, weak park, 1.8 s |

- **Time.** 8–12 captures instead of 80–88. 20 MHz: 1.5–1.8 s. 40 MHz:
  2.8–3.5 s, bound by the captures (about 0.2 s each with their EVM
  measurement). A first build that rebuilt 200 prediction symbols on every
  call took 2.2–2.8 s and 4.2–5.4 s; the fits and predictions were 1.3 s and
  2.2 s of that.
- **Agreement.** No new wall is more than 2 steps above the old one of the
  same session. The new measurement repeats less tightly: up to 4 steps
  between runs (channel 144 at 40 MHz, MCS7 19 … 23), where the old sweep
  repeats within about 1.
- **Disturbed runs.** The old sweep ended `disturbed` on its first try at
  40 MHz on all three channels (3 of 8 attempts); the new one failed none of
  26.
- **The weak-park band over the air** (`power_mode = "offset"`, margin
  1.0 dB, each rung forced for 10–15 s): channel 40, no dropped frame on any
  rung. Channel 64: MCS5, MCS4, MCS3 0–2 dropped frames; MCS7 1–3; MCS6 1–9
  (at most 1.5 % of a window). Channel 144 for comparison: 0–1.
- **Abort and interruption.** An abort two seconds into a 40 MHz measurement
  on the link's channel: `result=Aborted` after 2.1 s, video back, nothing
  stored. The GS daemon killed during a measurement off the link's channel:
  the drone finished and stored it, the pair linked again after the restart,
  the GS config was byte-identical.

## Not done

- PA B is still not measured.
- The thresholds are still one unit's. The second unit's ±5-step spread
  against delivery is not addressed.
- The 40 MHz measurement is over the 2 s that was aimed for.
- Channel 64's top two rungs drop a few frames at the default margin.
- Why the second unit's fit reads about 2 steps under its own sweep.
