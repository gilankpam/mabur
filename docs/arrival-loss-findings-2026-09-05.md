# Arrival-time loss booking — bench validation (2026-09-05)

Successor to `docs/switch-loss-findings-2026-09-05.md`. Validates the
arrival tracker (branch `probe-stream`, commits d90d86b..089962f): the
GS util-loss inputs (`s1_loss_cur`/`s3_loss_cur`, hence `link.pre_fec_loss` and the
ladder's `u`/`u3`) are now booked at arrival time by
`mabur::ArrivalTracker` instead of read off the FEC completion counters,
the 150 ms util settle blank is gone (`TransitionEdge` blanks only the two
residual windows), the sideport carries `link.streams[].arr_*` and the ctl
log header is `ctllog 11`. See `docs/link-adaptation.md`.

Three campaigns, each control-vs-candidate against today's
completion-booked loss-sim build. Bench link clean throughout (SNR ~33 dB,
60 fps, rung 5 reachable). **Nothing on the drone changed.**

## Binaries

| role | file | md5 | on the GS as |
|---|---|---|---|
| control (completion-booked, LOSS_SIM=ON) | — | `8880940cd41c81cfb78af604895b125d` | `/usr/local/bin/maburgs.switchloss` |
| candidate, campaigns (LOSS_SIM=ON) | `out/arm64/maburgs.arrival.losssim` | `ff35ddf34c0526162a476635257b3440` | `/usr/local/bin/maburgs.arrival.losssim` |
| candidate, deploy (LOSS_SIM=OFF) | `out/arm64/maburgs` | `17d35fd044b15942acd9f51b8d0bd556` | staged `/usr/local/bin/maburgs.arrival` |
| GS prod at time of writing | — | `c55ca697931ee9dbf3573c04fd626e2b` | `/usr/local/bin/maburgs` (`ctllog 10`) |

Build checks: `strings … | grep -c 'loss-sim [port]'` = **2** on the
LOSS_SIM=ON binary (the brief said 1; the usage block spells the option on
two lines — the discriminating number is 2 vs 0) and **0** on the deploy
binary; `grep -c 'ctllog 11'` = 1 on both. The `build-arm64` CMake cache
was left at `MABUR_LOSS_SIM:BOOL=OFF`.

## Campaign 1 — steady loss sweep, rung pinned to mcs4 (NOT DECIDED)

`losssim.py --sweep 0,2,5,10,20 --dwell 25 --burst 3 --stream 0` against
`/etc/maburgs.pin4.json`; `err` = median `link.pre_fec_loss` over the last
15 s of the dwell minus the dialled `eff`; `settle_ms` = first sample
within 10 % of that median. Two replicates per arm.

| step | CTRL run 1 (ctl-0305) | ARR run 1 (ctl-0307) | CTRL run 2 (ctl-0335) | ARR run 2 (ctl-0337) |
|---|---|---|---|---|
| eff 0 % | err +0.0000, settle 35 | err +0.0000, settle 164 | err +0.0000, settle 67 | err +0.0000, settle 678 |
| eff 2 % | +0.0026, 1467 | +0.0002, 4005 | +0.0017, 2714 | +0.0018, 1522 |
| eff 5 % | −0.0019, 712 | −0.0013, 242 | −0.0017, 335 | −0.0041, 155 |
| eff 10 % | +0.0047, 1155 | +0.0068, 494 | −0.0009, 2184 | +0.0075, 411 |
| eff 20 % | −0.0008, 587 | −0.0104, 341 | −0.0109, 416 | −0.0013, 1057 |

Read literally the pass criterion (candidate `|err|` ≤ control at every
step, candidate `settle_ms` ≤ control at every step ≥ 2 %) **fails in both
replicates** — run 1 at eff 10/20 % (err) and eff 2 % (settle), run 2 at
eff 5/10 % (err) and eff 2/20 % (settle).

**The criterion does not discriminate at this noise level.** The sideport
publishes `pre_fec_loss` on ~200 ms buckets; at eff 2 % that is a handful
of lost bodies per bucket. Evidence:

- Two runs of the **same control binary** disagree by 1.0 pp at eff 20 %
  (0.1992 vs 0.1891) — larger than any control-vs-candidate difference at
  that step, and larger in the opposite direction each run (run 1 the
  candidate under-reads, run 2 the control does).
- Scored against each other under the same criterion, **control-vs-control
  fails 3 of 8 sub-criteria** (err at 20 %, settle at 2 % and 10 %) and
  candidate-vs-candidate fails 4 of 8. A pair of identical binaries cannot
  pass this test.
- 2000-sample bootstrap CIs of the run-1 15 s medians overlap at every
  step: eff 10 % CTRL [0.0975, 0.1126] vs ARR [0.1005, 0.1127]; eff 20 %
  CTRL [0.1881, 0.2047] vs ARR [0.1821, 0.1984].
- `settle_ms` is self-referential (a band around the step's own median), so
  at eff 2 % it measures which noise excursion happens to land inside a
  ±0.002 band first. A noise-robust rise time (first trailing-1 s mean at
  ≥ 50 % of the step median) puts the candidate **faster** at every step
  above 2 % in run 1: 42/94/139 ms vs the control's 112/155/184 ms.

The one signal that repeats across both replicates is a **+0.7 pp
over-read at eff 10 %** by the candidate (+0.0068, +0.0075) where the
control is ±0.3 pp. It is small, it is on the safe side (over-reading loss
demotes earlier, never later), and it is not separated from noise by the
bootstrap CIs — but it is the only per-step difference with a consistent
sign, so it is the thing to re-measure if the sweep is repeated with a
dwell long enough to beat the bucket noise.

**Verdict: not decided.** Neither "the candidate is worse" nor "the
candidate is as good" is supported by this instrument. A conclusive rerun
needs longer dwells (≥ 120 s) and several replicates per arm, or a loss
readout that is not a 200 ms bucket.

## Campaign 2 — pulse campaign (PASS)

`/root/switchloss.sh <config> <tag> <binary>` (now parametrised on the
binary as `$3`; backup at `/root/switchloss.sh.pre-arrival`): climb to
rung 5, then 8 × ~152 ms `s1 eff=25 burst=3` pulses 20 s apart.

| quantity | CTRL (ctl-0309) | ARR (ctl-0311) |
|---|---|---|
| pulses that demoted | 8 of 8 | **7 of 8** (pulse 3, on_mono 19803693, did not demote) |
| demotes with `steps=1` | 8/8 | 7/7 |
| demote reasons | 6 × s3_residual, 2 × s3_util | 7 × s3_residual |
| re-promotes to rung 5 | 8 | 7 |
| attribution-miss canary | 0 | 0 |
| s3-settle-refire canary | 0 | 0 |
| `probation` lines | 0 | 0 |
| util sample > `down_util` within 200 ms of an E line | 0 | **0** |
| drone switch after E (probe-log) | p50 57, p90 77, max 86 ms (n=21) | p50 42, p90 65, max 113 ms (n=19) |
| lost frames per transition (0..+600 ms) | 6.2 | 1.9 |

All four pass criteria hold: every demoting pulse is `steps=1`, both
canaries are 0, the post-transition util scan prints nothing for the ARR
arm, and switch latency is not worse (p50 42 ms vs 57 ms — the same
measurement's own p90/max spread makes this "unchanged", not "improved").

The missed pulse 3 is the only asymmetry. It is a sensitivity datum, not a
criterion failure (the criterion is about demoting pulses); with the blank
gone and the tracker's denominator intact, a 152 ms pulse sits right at the
threshold. Worth watching if demote latency ever looks slow in flight.

**Two corrections to the brief's commands, for whoever reruns this:**

1. `losssim.py` defaults to port **8302** but the campaign daemons bind
   `--loss-sim 8390`. Without `--port 8390` every step comes back
   `<no reply>` and the sweep aborts with `FAILED` on step 1. (First
   control run was lost this way.)
2. The brief's post-transition awk (`$3+0>0.35 || $6+0>0.15`) reads the
   wrong columns. The `ctllog 11` S line is
   `S t rung u snr resid u3 resid3 evm resid_cur drssi dsnr rssi probe_rung probe_u probe_n`,
   so the util inputs are `$4` (base `u`) and `$7` (`u3`); `$3` is the rung
   and is > 0.35 on every line above rung 0. The literal awk "finds" 65
   (CTRL) and 57 (ARR) phantom hits. The correct scan is
   `($4+0>0.35 || $7+0>0.35)` and prints **nothing on either arm**.

## Campaign 3 — restart climbs (PASS)

`/root/climbs.sh <binary> 10` — 10 cold starts, 40 s each, adaptive config.

| arm | ctl indices | `top=5` | `probation=0` | E lines per climb |
|---|---|---|---|---|
| ARR (candidate) | ctl-0313 .. ctl-0322 | **10/10** | **10/10** | 5 on 9 climbs, 7 on ctl-0321 |
| CTRL (control) | ctl-0324 .. ctl-0333 | 10/10 | 10/10 | 5 on all 10 |

Candidate passes. Note the control also shows `probation=0` on all 10: the
bench does not reproduce today the post-promote probation bounce seen on
flight-0023 and bench ctl-0299, so this campaign confirms no regression
rather than confirming the fix. The fix itself stays flight-pending.

## Standing gates — PASS on the LOSS_SIM=OFF binary

Run on `/usr/local/bin/maburgs.arrival` (md5 `17d35fd0…`), fresh start,
ctl-0339:

```
ctllog 11 ladder=0/100:50,...,5/100:50 down_util=0.35 up_util=0.15 probe_offset=1
E 21392831 0 1 promote_probed / 1->2 / 2->3 / 3->4 / 4->5    probation=0
ausniff  : aus=1800 complete={'0': 900, '1': 900} incomplete={} fid_gaps=0
           resyncs=0 bytes=60930271 dropped_oversize=0 fps=60.0
aucadence: {"n_base": 743, "n_enh": 744, "idr_excluded": 0, "resyncs": 0,
            "fallback_rows": 0, "clock": "t_complete", "offset_ms": 2.866,
            "len_p50": {"0": 32371, "1": 31987.5}}
```

60.0 fps, `fid_gaps=0`, `incomplete={}`, base−enh completion offset
2.87 ms inside the 4.0 ms gate, clean 0→5 climb with no `probation`.

## Deploy — NOT DONE

The GS still runs the completion-booked prod binary
(`c55ca697931ee9dbf3573c04fd626e2b`, `ctllog 10`) and `/etc/maburgs.json`
is untouched. Campaign 1's literal pass criterion failed, and the standing
rule for this task is that a failed criterion blocks the deploy — even
though §Campaign 1 shows the criterion cannot be passed by any binary,
including the control. That call belongs to whoever reads this, not to the
campaign.

The candidate is staged and gate-clean, so the deploy is one rotation
(no config change either way):

```sh
ssh root@10.18.0.1 '/etc/init.d/S96maburgs stop; sleep 2; killall maburgs 2>/dev/null; sleep 1; \
  mv /usr/local/bin/maburgs /usr/local/bin/maburgs.pre-arrival; \
  mv /usr/local/bin/maburgs.arrival /usr/local/bin/maburgs; \
  chmod 755 /usr/local/bin/maburgs; /etc/init.d/S96maburgs start'
```

Rollback: **`/usr/local/bin/maburgs.pre-arrival`** (created by the mv
above), no config restore needed.

## Artefacts on the GS

`/tmp/sweep-{CTRL,ARR,CTRL2,ARR2}.{txt,log}`,
`/tmp/gaplog-switch-{CTRL,ARR}.txt`, `/tmp/pulser-switch-{CTRL,ARR}.txt`,
`/tmp/switch-{CTRL,ARR}.out`, `/tmp/climbs-{CTRL,ARR}.txt`, `/tmp/gates.txt`;
ctl/probe logs ctl-0305 .. ctl-0339 on `/media/dvr`; sideport jsonl in
`/media/dvr/log/flight-0024.jsonl`. `/etc/maburgs.pin4.json` is the pinned
(`static_mcs=4`) sweep config. `/root/climbs.sh` is new;
`/root/switchloss.sh` gained the `$3` binary argument.
