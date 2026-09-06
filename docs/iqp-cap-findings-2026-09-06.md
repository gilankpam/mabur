# I-QP floor (`venc.min_iqp`) — bench A/B, 2026-09-06/07

Branch `iqp-cap`. Question: does a boot-time I-frame QP floor cap the
demote IDR, and what does it cost on the climb back? Background and the
knob itself: `docs/airtime-model.md` (`venc.min_iqp`); the flight
motivation (rung-0 demote IDR 27–41 kB = 90–140 ms on the wire at mcs0,
half of it `roi_qp_low` −24 in the keyframe) in the memory of flight 0356
and `docs/observability.md`'s airdrain "transition IDRs" table.

## Rig

- Drone: `maburd` from `iqp-cap` (`venc.min_iqp` 44 in config, ROI
  disabled), the arm toggled live between cycles through
  `:8301 /venc/set?min_iqp=44|12` (12 = firmware floor, i.e. the
  pre-change behaviour). Static bench scene, 1080p60, `max_ipprop` 2.
- GS: `maburgs.iqpab.losssim` (master GS source + `-DMABUR_LOSS_SIM=ON`,
  `--loss-sim 8390`), rejoined debug-log session `/media/dvr/log/0001`.
- Stimulus per cycle (`tools/bench/iqpab.sh`, `STREAM=s1`): `s1 eff=30
  burst=3` for 2.5 s, then off, 40 s for the climb. Enh-stream loss
  drives the ladder exactly like the flight (`s3_residual` then four
  `s3_util` steps, 5→0 in 1.6 s, then five `promote_probed` 0→5 over
  18 s) while leaving every base body untouched, so the base-only air
  excess is clean. Base-stream loss (`s0`) also cascades but pollutes the
  air metric with repair waits; probe-canary loss (`s5`) does not move
  the ladder at all (u3 stayed < 0.03).
- 5 cycles per arm, interleaved A/B/A/B. Readout:
  `tools/bench/iqpab_report.py <session> cycles.txt 40`, which windows
  `airdrain.analyze()`'s transition-IDR rows and a base-only air-excess
  peak per cycle.

## Result

| per arm, 5 cycles each | firmware floor (12) | `min_iqp` 44 |
|---|---|---|
| demote IDR kB, rungs 4/3/2/1/0 | 15.0 / 10.2 / 8.5 / 6.4 / 4.5 | 2.2 / 2.2 / 2.2 / 2.2 / 2.2 |
| promote IDR kB, rungs 1/2/3/4/5 | 6.4 / 8.5 / 10.1 / 14.8 / 23.7 | 2.2 at every rung |
| base air-excess peak after demotes (max over 1.5 s) | 9.9 ms p50, 16.5 max | 10.0 ms p50, 11.8 max |
| base air-excess peak after promotes | 7.9 ms p50 | 6.4 ms p50 |
| worst-second e2e p99 in the 20 s cycle window (`lat.log`) | 117 ms p50, 158 max | 113 ms p50, 141 max |
| e2e p50 | 42.5 ms | 43.0 ms |
| reached rung 0, climbed back to 5 | 5/5 | 5/5 |

Spread within an arm is under 0.5 kB on every IDR row: the floor is
deterministic and the ladder behaves identically under it (same step
timing, same probe outcomes, no probation).

## Reading

1. **The floor works as a cap.** Every attr-change IDR, demote or
   promote, at every rung, is 2.2 kB under `min_iqp` 44 — 2–11x smaller
   than the firmware floor's, and below the rung-0 per-frame budget
   (3.75 kB at 1800 kbps), so it fits its own slot.
2. **The bench cannot show the airtime win.** On the static scene the
   firmware-floor rung-0 demote IDR is only 4.5 kB (1.2x budget), not the
   flight's 27–41 kB, so the base air peak (10 ms either arm, the ordinary
   cascade drain) and the e2e tails (dominated by the injected enh loss's
   repair waits during the pulse, `fec=` in `lat.log`) are a wash. The
   flight is the test of the latency claim; this A/B proves the mechanism
   and shows no regression on the climb.
3. **Promotes pay the same coarseness.** A 23.7 kB rung-5 promote IDR
   became 2.2 kB: at 14400 kbps that is a needlessly soft keyframe. If it
   shows in the picture, the follow-up is a per-rung floor written by
   RcAgent before each bitrate write (high at rungs 0–1, firmware at
   the top), not a lower single value.
4. **Fade was not tested.** The bench has no RSSI stimulus: loss-sim
   injects body loss, and the fade predictor keys on `drssi`/`dsnr`. A
   fade demote fires the same SetChnAttr IDR as any other demote, so the
   cap applies; whether the fade path's timing interacts differently is
   a flight question (`airdrain` rows carry the E-line reason).

## State left behind

- GS back on the production wrapper (`/usr/local/bin/maburgs`), loss-sim
  daemon stopped, all streams zero. `maburgs.iqpab.losssim` stays in
  `/usr/local/bin` for a rerun.
- Drone on `iqp-cap` maburd with `venc.min_iqp` 44 in `/etc/mabur.json`
  (backup `mabur.json.pre-miniqp`; binary rollback `maburd.pre-iqp` needs
  that backup restored alongside it — strict keys).
- Session logs of the campaign: `/media/dvr/log/0001` (cycle stamps in
  the campaign's `cycles.txt`; mono ms, GS clock).
