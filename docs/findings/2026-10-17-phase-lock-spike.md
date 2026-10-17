# Can the GS lock the drone's capture phase to its vblank, and does a GOP-aware target make the lock judder-free?

**Status:** spike, nothing shipped. The regulator, its chain budget and
the sawtooth it produces are described in `docs/observability.md`
("Regulator line") and `docs/findings/2026-10-15-regulator-chain-cuts.md`;
the sensor-rate trim the lock steers through is
`docs/findings/2026-10-14-sensor-rate-trim.md`. The throwaway servo and
the drone verb it drives live only in the machine-local spike directory.

## Question

With `chain_budget` 0 the display is smooth but glass latency rides a
38 → 54 ms sawtooth as the sensor/panel beat sweeps arrival across the
latch. If the GS steered the sensor rate so arrival sat at a fixed
phase, could it (1) flatten e2e near the bottom of the sawtooth and
(2) pick a phase that also lets the known-late GOP-start frame make its
slot, so the stream is smooth at that latency?

## Setup

Bench Radxa ZERO 3 and the bench drone, rung 4, static night scene
(P frames 26-33 kB, GOP-start frames 60-64 kB), `vsync_lead_ms` 5,
`chain_budget` 0, session 0129. A throwaway drone verb accepts a sensor
rate in milli-fps and applies it from the venc hot thread with
`MI_SNR_SetFps`; the sensor trim had locked at 59718 mfps, and 59728 is
one line faster. A host script streams `frame.log`, computes each
frame's hold in its natural slot, `(t_flip − lead − t_decoded) mod P`,
and once a second toggles the drone between the two rates to keep the
median hold at a target (bang-bang, 0.5 ms hysteresis).

- Run A: target 14 ms, 335 s of servo after an 80 s slope calibration.
- Run B: target 15.5 ms, hold unwrapped around the target so the median
  stays meaningful when the distribution straddles the period, 328 s.

GOP excess is measured per GOP-start frame as its decode-ready delta to
the previous frame minus one period, and as its ready time (capture →
decoded) against the typical frame's.

## Result

The lock works and is harmless. Run B held the median hold within
±1 ms of target for 270 s with 33 rate toggles; across both runs 107
`MI_SNR_SetFps` calls produced 0 pts holes in `au.log` and 0 frames with
`dsp` over 15 ms. Each toggle steps the phase by ~1.4 ms, the slopes are
±0.15 ms/s.

The latency goal is met. Run B's e2e p50 sat at 37-39 ms for every 10 s
bucket but one (a 10 s chain at 53), against the 46 ms average of the
sawtooth it replaces. The chain never persisted: at this hold the
regulator's deep-burst rule displaced the held frame itself, so a chain
budget was not needed for latency.

The smoothness goal is not met, and cannot be met by any hold target:

| | typical frame ready p50 | GOP frame ready p50 / p90 | GOP excess p50 / p90 / p99 |
|---|---|---|---|
| run A (14 ms) | 16.8 ms | 31.5 / 35.2 | 14.9 / 18.9 / 21.8 |
| run B (15.5 ms) | 18.5 | 31.8 / 35.6 | 15.0 / 18.4 / 21.3 |
| earlier bench sessions 0107/0110/0125 | 19.4-20.0 | 28.9-29.9 / 31.7-33.5 | 10.3-11.6 / 13.5-15.5 / 16.8-18.8 |
| flight 0122 | 22.4 | 29.6 / 35.5 | 8.6 / 14.5 / 23.4 |

The GOP-start frame is ready at a nearly constant 29-32 ms whatever the
scene (encode +6 ms, transport +3..5 ms for a 2x picture, decode +3 ms
as a single slice), so its excess over the typical frame grows exactly
when P frames shrink. The highest hold the lock can give a typical frame
is about 15.5 ms before early frames wrap into the previous slot, and
that is below the excess p50 on this bench and below p90 in every
session measured. In run B 39 % of GOP frames still missed their slot
(0.88 displaced frames/s, 66 % of seconds with p99 ≥ p50+10) even
though the typical frame had the most slack a period allows.

## Answer

A GS-side phase lock through the sensor rate is cheap, stable and
removes the latency sawtooth: ~38 ms flat instead of 38 → 54. It does
nothing for judder, because judder at this latency is the GOP-start
frame's excess, and the excess is too large to fit inside one display
period at any phase. Build the lock only together with something that
brings the GOP frame's ready time within ~12 ms (p90) of a typical
frame's, or accept one hitch per GOP. The regulator-side options that
remain (drop the post-GOP frame deterministically, or a chain budget)
turn each miss into one hitch instead of two; they do not remove it.
