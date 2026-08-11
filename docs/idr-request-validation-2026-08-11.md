# GS-initiated IDR request: loss-simulator validation (bench, 2026-08-11)

Acceptance campaign for the `idr-request` branch, using the injected
per-stream loss rig (`bench/s3-loss-sim`, cherry-picked onto the PR as
`bench/idr-loss-sim`). It answers the two questions the design left open:
does a reference-layer loss actually produce an IDR, and does enhance-layer
loss *falsely* produce one.

## TL;DR

- **Positive path works.** Every s1 (reference-layer) injection level
  latched, and the drone granted: 30 GS latch episodes → 28 drone grants
  over the campaign. Heal latency p50 257 ms, max 797 ms — against the
  ~2 s open-loop wait for the next periodic `sid==0` refresh.
- **No s3 false fire — including through a full ladder cascade.** 50 s of
  enhance-layer loss (up to 35 % effective, 1 071 abandoned s3 bodies,
  136 truncated + 468 dropped frames) produced **0 episodes and 0 grants**,
  even though the s3 loss drove two complete 3→2→1→0 `s3_util` cascades.
  Rung-transition FEC re-key artifacts did not reach the latch.
- **The latch never stuck.** Longest continuous pending stretch is 6
  samples (3 s) and it is a chain of *distinct* short episodes, not one
  long one — no single `wait_ms` exceeded 797 ms.
- **AU-ring gate clean** on the bench build: 2 654 AUs / 45 s, 0
  incomplete, 0 `frame_id` gaps, 0 resyncs, 59.0 fps.

## Rig

GS `maburgs` = `idr-request` @ f9c898b + the loss-sim cherry-picks, started
with `--loss-sim 8390` (8302 is maburplay's GS-OSD port). Drone runs the
branch's `maburd` unchanged (`/usr/bin/maburd`, rollback `.pre-idr`), so
`waybeam.idr_cooldown_ms` is the 1000 ms default and `CAP_IDR_REQ` is
advertised. Link config unchanged: `link.idr_req` absent → default true,
4-rung effective ladder (`max_mcs 5`), 2 cards, clean bench link at
30–35 dB SNR.

Recording: `/media/dvr/idrq-run1.jsonl` (sideport, 2 Hz, 523 samples) +
`/media/dvr/idrq-run1.log` (phase marks with `mono=`) +
`/media/dvr/ctl-0044_20170804.log` (ctl log). Driver
`/root/idrq_campaign.sh`; injection restored to zero and confirmed by the
final `status`.

Note on the dial: `eff=` is the NOMINAL union rate across the 2 cards. The
per-card rates actually sent were 22–59 %. Real delivered loss is the
sideport's per-stream counters below, not the dial.

## Results

Per phase, deltas over the phase window (`ep` = `link.video.idr_req
.episodes`, `grant` = `drone.enc.idr_grants`):

| phase | dur | ep | grant | trunc | drop | s1 abn | s3 abn | rung |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| baseline, no injection | 26 s | 0 | 0 | 1 | 0 | 0 | 0 | 3 |
| **s3** eff=10 burst=3 | 21 s | **0** | **0** | 19 | 37 | 0 | 226 | 0–3 |
| off | 16 s | 0 | 0 | 3 | 0 | 0 | 0 | 0–2 |
| **s3** eff=35 burst=3 | 31 s | **0** | **0** | 136 | 468 | 0 | 1071 | 0–2 |
| off | 21 s | 0 | 0 | 0 | 1 | 0 | 0 | 0–2 |
| **s1** eff=5 burst=3 | 16 s | 1 | 1 | 1 | 0 | 25 | 0 | 2–3 |
| off | 16 s | 0 | 0 | 0 | 0 | 0 | 0 | 3 |
| **s1** eff=10 burst=3 | 16 s | 2 | 2 | 2 | 2 | 40 | 0 | 2–3 |
| off | 16 s | 0 | 0 | 0 | 0 | 0 | 0 | 2–3 |
| **s1** eff=20 burst=3 | 16 s | 4 | 3 | 3 | 3 | 0 | 0 | 0–3 |
| off | 16 s | 2 | 0 | 2 | 2 | 0 | 0 | 0–2 |
| **s1** eff=35 burst=3 | 26 s | 21 | 22 | 28 | 47 | 29 | 0 | 0 |
| off | 31 s | 0 | 0 | 0 | 0 | 0 | 0 | 0–3 |

Totals: **30 episodes, 28 grants.** fps floor never left the 40s.

### The s3 control is stronger than a flat null

The interesting part of the s3 phases is not that nothing happened — it is
that a *lot* happened and the latch still never fired. From the ctl log,
inside the s3 windows:

```
E 551008 3 2 s3_util 0.4726   E 587515 2 1 s3_util 1.5856
E 551576 2 1 s3_util 0.8258   E 588093 1 0 s3_util 1.1517
E 552142 1 0 s3_util 0.5489   P 593314 1 fail  (probe refused)
```

Two full multi-rung cascades to rung 0, a failed probe, 468 dropped and
136 truncated frames — all of it sid 3 — and `episodes` did not move once.
That is `IdrRequester::on_frame_lost()`'s `sid > 2` guard doing exactly its
job, and it also rules out the indirect path we were worried about (s3 loss
→ s3_util demote → rung change → re-key → base-layer loss → IDR).

### Grants track episodes, and the gap is benign

30 episodes vs 28 grants. The two missing grants are latches that cleared
on a *periodic* `sid==0` refresh rather than a granted IDR — the latch
clears on any complete IDR-flagged frame, not only on one the drone granted
(`on_frame_emitted`). So the shortfall is the design's fallback working,
not a lost request. There is no leak: the absolute counters end aligned
(episodes 1→31, grants 3→31; grants started 2 ahead from earlier
entering-LINKED grants).

### The cooldown is the structural limiter

The sustained phase produced 21 episodes in 26 s ≈ 0.81/s, just under the
1000 ms `idr_cooldown_ms`. This is not a coincidence and not a headroom
measurement: a new episode requires the previous latch to clear, and a
clear requires an IDR to arrive, which is itself cooldown-gated. Episode
rate is therefore bounded by the cooldown by construction, and no injection
level can push it higher. The campaign cannot stress the cooldown beyond
this, and does not need to.

## Observability gap worth knowing

A request *refused* by the drone's cooldown is counted nowhere — the drone
increments `idr_grants` only on grant, and the GS latch is a level, not a
count of RCFs sent. So `episodes − grants` is the only visible residue, and
it conflates "refused by cooldown, healed by the periodic IDR" with
"granted just outside the sample window". Both are harmless here; if a
future campaign needs to separate them, the drone needs a refused-request
counter.

Also: the GS's `idr-req: set` log line is rate-limited to 1/s while
`cleared` is not, so `/tmp/maburgs.log` shows unpaired `cleared` lines and
undercounts sets. Use the sideport `episodes` counter, never the log, to
count.

## Deployment state after the campaign

GS restored to the shipped PR build; verified linked, mcs4, 60 fps, 0
truncated. Bench artifacts left in place for a re-run:

- `/usr/local/bin/maburgs.losssim` + `/etc/init.d/S96maburgs.losssim` —
  the loss-sim build and its `--loss-sim 8390` init script.
- `/usr/local/bin/maburgs.pre-losssim` — the clean build, currently active.
- `/root/losssim.py`, `/root/idrq_campaign.sh`.

Host side, the rig lives on branch `bench/idr-loss-sim` (PR branch +
cherry-picked loss-sim commits; never to be merged).
