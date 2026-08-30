# decode_deadline_ms bench measurement — row expiry (verdict superseded: DELETED)

**Addendum, same day:** the measurement below stands, but the KEEP
verdict was overruled by the operator on headroom grounds — +7–8 points
of ONE core on a 4-core GS whose daemon peaks ~69% is affordable, and
the project policy is to delete dead weight. `fec.decode_deadline_ms`,
`UepDecoder::poll()`, and the video decoders' row expiry were deleted
the same day (`link.deadline_ms` left the sideport with them — see
data-provenance). The video decoders now rely on the seq horizon alone;
**MSP's SwDecoder keeps its own 2 s `expire_rows_older_than`** (a
genuinely low-rate layer where the horizon barely advances), so the
mechanism survives in `sw_decoder.{h,cpp}` with linkbench as the other
user. Deploy verified: config-before-binary, ausniff 60.0 fps / 0 gaps
/ 1800 complete, and a 45% loss-sim confirm run reproduced the
deadline-5000 leg (rows p95 146/147 max 160, CPU 67.0%, fps 33.9).
Rollback pair on GS: `maburgs.pre-nodeadline` +
`/etc/maburgs.json.pre-nodeadline` (must be restored together).

2026-08-30, bench pair (drone prod maburd, GS `maburgs.losssim` built from
this branch with `-DMABUR_LOSS_SIM=ON`, control port 8305). Question:
is the 75 ms repair-row expiry (`fec.decode_deadline_ms`,
`UepDecoder::poll()` → `SwDecoder::expire_rows_older_than`) dead weight,
given the seq horizon (wall 1) also bounds `rows_`? Decision rule fixed
in advance: delete if rows plateau ~200 AND the CPU delta is noise.

## Method

Loss injected on BOTH video streams (Gilbert-Elliott, burst 3, union
`eff=` rates), rung pinned via `static_mcs`, 60 s dwell per level.
`deadline 75` (prod) vs `deadline 5000` (wall neutralized; horizon-only).
Observables per window: sideport `streams[].in_flight` (live repair
rows), `abandoned_s`/`recovered_s`, fps, and maburgs CPU (%
of one core, /proc tick deltas). Recording: GS `flight-0032.jsonl`
(session index space of that bench day) + `/tmp/w2-*.log` mono stamps.

## Results

mcs5 pin (~5.8 Mb/s inj/stream, horizon ≈ 270 ms of traffic):

| union loss | rows p95 (75) | rows p95 (5000) | CPU (75) | CPU (5000) | fps both |
|---|---|---|---|---|---|
| 0% | 0 | 0 | 65.6% | 65.4% | 59.5 |
| 15% | 18/16 | 36/37 | 63.3% | 64.5% | 58.3 |
| 30% | 35/32 | 108/106 | 63.3% | 67.0% | 50.7 / 50.6 |
| 45% | 39/35 | 146/146 | 60.9% | **69.1%** | 33.2 / 33.1 |

mcs1 pin (horizon ≈ 1 s of traffic — the long-lifetime regime):

| union loss | rows p95 (75) | rows p95 (5000) | CPU (75) | CPU (5000) | fps both |
|---|---|---|---|---|---|
| 0% | 0 | 1 | 60.7% | 60.6% | 59.5 |
| 30% | 25/23 | 108/101 | 55.0% | 59.7% | 50.3 / 50.3 |
| 45% | 25/25 | 150/147 | 51.5% | **58.7%** | 33.6 / 33.7 |

Fade cycles (15× eff=90 burst=10 3 s on / 5 s off, mcs5): rows p95
9/10 (75) vs 32/31 (5000), recovery and fps equivalent (~41–42), no
post-fade row lingering in either config — the fade case argues for
neither side.

## Reading

- **Wall 1's backstop holds**: without the deadline, rows plateau at
  ~150–160 per stream (≈ horizon-bounded), never runaway. The
  original "unbounded equations" fear is refuted.
- **But the CPU cost is not noise**: +7–8 points of a core at 45%
  sustained loss (both rungs), on a daemon already at ~60–65%. The
  baseline config gets *cheaper* under loss (less traffic to decode);
  the horizon-only config gets more expensive — the delta is pure
  dead-row elimination work, exactly in the regime where the GS is
  already stressed.
- **The 75 ms value costs zero recovery**: fps and abandoned/s are
  identical between configs at every level, both rungs. At mcs1 the
  window span (~68 ms of traffic) grazes the 75 ms deadline and the
  horizon-only config did log ~6–9% more recoveries at 30% loss — with
  zero fps difference, so the extra recoveries were futile (frames
  already truncated). Do not raise the deadline chasing that.

## Verdict

**Keep `decode_deadline_ms` at 75.** It is a measured ~7–8 CPU-point
saving under loss storms with no recovery penalty. The one legitimate
criticism stands unchanged — it is the only wall-clock in the decode
data path (see the 2026-07-13 stamp-vs-poll-clock underflow guard) — so
*if* that path is ever reworked, express the same bound window-relatively
(evict rows whose newest referenced seq trails `newest_v_` by ~2
windows); that is polish, not a scheduled task.

Rig recipe: configs were prod + `static_mcs` pin + deadline override
(generated, then deleted from /etc after the runs); drivers
`w2run.sh`/`w2fade.sh`/`w2cpu.sh` were session scratch. ⚠ Gotchas
rediscovered: `ls -t` on /media/dvr/log lies (GS RTC) — the live
recording is the **highest index**, and the sideport video block is
`link.video`, not top-level. A maburd restart that trips the stale-MI
"REBOOT the drone" exit needs exactly that.
