# Pause-and-jump lag investigation — bench acquittals and first instrumented flights

Bench + field, 2026-08-13. Follow-up to the ctl-0003 crash ("massive lags
then crashed", obstructed slow flight) and to
`docs/venc-ring-vanish-findings-2026-08-12.md`. This documents where the
lag investigation stands after two bench transient-profile sessions and
the first two flights ever flown with full observability (vanish
detection + boot flight recorder).

## TL;DR

- The drone's TX pipeline is **fully exonerated at the flight operating
  point (≤8 Mbps) with a responsive uplink**: TxQueue never builds, venc
  output converges ≤2 s after a demote command, and the venc ring never
  exceeds 1/8 slots through full demote cascades. The 2026-08-09
  shed-sync fix does its job; the deferred TxQueue-flush leg has nothing
  to fix in this regime.
- The **stale-op-under-uplink-loss hypothesis is not supported** by the
  first recorded flights: `drone.applied` tracked the GS-commanded rung
  within ~1 s telemetry resolution through every ordinary fade.
- The pause-and-jump symptom **did not recur** under the retuned config
  (`down_util 0.35`, `s3_down_util 0.20`, `probe_max_util 0.20`).
- New live evidence for the venc-ring doc's burst-seeding theory: a
  promote was followed 157 ms later by a residual demote at SNR 33 /
  u = 0.0, coincident with +1 `vanished_base` +1 `self_idr_refused` —
  **promote → IDR burst → wire residual AND ring vanish on a clean
  link**, first time observed in flight.
- Vanish counters carry a **boot-window baseline artifact**: ~8-9 of
  each flight's total accrue before link-establish. Genuine in-flight
  rate: 1-3 per ~5 min flight at ≤8 Mbps.

## Background

Flight ctl-0003 (2026-08-13) crashed after the pilot saw "massive lags"
— later characterized as **pause-and-jump** (frames pause then clump
forward; latency, not framerate), worst at low rungs (mcs0-2), always
after demotes. Suspects at the outset: (a) TxQueue drain tail (the
shed-sync fix's deliberately deferred leg — `sim_shed_lag`'s own
counterfactual predicted it survives), (b) venc-ring / CPU-famine drain
stalls (the vanish doc's mechanism), (c) stale high op while fade kills
the GS→drone RCF channel, (d) GS repair-latency stacking, (e)
player-side presentation gaps.

## Bench session 1 — TxQueue acquitted

Loss-sim rig ported to current master (branch `bench/loss-sim-v2`; the
2026-08-09 GS sim binary is RC_VERSION 1 and cannot link to the current
drone). GS ran `maburgs.losssim2 --loss-sim 8390` beside the recorder;
campaign = 3× `s1 eff=35 burst=3` 3 s bursts + a stepped s1+s3 ramp
(eff 8→35), each from rung 5 @ 8 Mbps, ~590 sideport datagrams recorded.

Every episode: cascade to rung 0 in ~1-1.5 s, `cmd_kbps` steps down
0.5-1.5 s per rung, measured `enc.mbps` conforms to the 1300 floor
within ~2 s, **txq depth 0 throughout, zero drops**, fps ≥ 49, GS
AU-emit jitter peak 30 ms. No drain tail exists when the demote command
reaches the drone.

## Bench session 2 — venc ring acquitted at this op point

`tools/bench/ringwatch.c` (rebuilt from the famine experiment's
uncommitted sampler; 1 kHz read-only mmap of `/dev/shm/mabur_f`'s
header; key metric `rstall` = read_idx frozen while occupancy > 0 — the
hot-thread-famine signature) ran on the drone through an identical
campaign: **328 s sampled, max occupancy 1/8 slots, write-freeze
≤ 33 ms (2 frame times), read-stall ≤ 11 ms, zero anomalous seconds.**
Consistent with the vanish doc's threshold: famine needs >~12 Mbps at
mcs5 on the 2×A7; the flight ladder tops out at 8 Mbps.

## Deployment that made the flights observable

- Vanish detection ported detection-only to `venc-vanish-detection`
  (commit `22610fc`): Telem wire 61→67
  (`vanished_base`/`vanished_enh`/`self_idr_refused`), sideport
  `drone.enc.*`, 5 s `frame_ring:` stderr line. No self-IDR consumer
  (the 2026-08-12 IDR-storm redesign is still queued). Deployed both
  ends (rollbacks `maburd.pre-vanish`, `maburgs.pre-vanish`); ausniff
  gate clean.
- Flight recorder reinstated as a boot service: `/etc/init.d/S97flightrec`
  → `/root/rec8300.py` → `/media/dvr/flight-NNNN.jsonl` (per-boot
  index, no date — the GS RTC is wrong; newest 30 kept; ~1 MB/min).
  Gotcha that killed the first bench recording: the CLAUDE.md
  `socat | jq` recipe dies on ssh detach even under nohup — attended
  use only.

## Flight evidence (ctl-0011 + flight-0004, ctl-0012 + flight-0005)

Two ~5 min obstructed slow flights, both fully recorded at 2 Hz.

**Stale-op: not supported.** Across 31 rung transitions, no window
> ~1 s where fresh telemetry (advancing `tlm_seq`) showed
`drone.applied.mcs` ≠ the GS-commanded rung. The RCF path applied
demotes promptly even at uplink SNR 12-15 dB. Caveat: during total
blackout the telemetry freezes with everything else, so the method is
blind exactly there — but the pause-and-jump regime (ordinary fades)
is well covered.

**No lag recurrence.** Jitter < 30 ms everywhere except flight-0004
t = 262-280: an **18 s total RF blackout** (uplink SNR 3.5 → −3 dB,
`tlm_seq` frozen 10 s, fps 0 at rung 0/ov 100) — the flight's
`starved` event, working as designed; nothing survives negative SNR.
⚠ Analyzer trap: the sideport `link.video.jitter_ms` there reads a
flat 141.6 ms for ~5 s — that is the EWMA **holding its last value
while zero frames arrive**, not a measurement. fps is the truth signal
during an outage.

**Vanish counters, baselined.** Final counts were identical in both
flights (10/10/2) because ~8-9 base + ~9-10 enh + 1-2 refused accrue
during **drone boot before link-establish** (encoder bring-up churn;
first telemetry datagram already carries them). Genuine in-flight
steps: flight-0004 +1 base @ t=90.5 (see below), +1 enh @ 205.3
(rung 3), +1 base @ 271.5 (during the blackout); flight-0005 +1 base
@ 176.5 (rung 3, 7.5 Mbps, SNR 28). Fix direction: zero (or snapshot)
the counters at first link-establish; until then, analyzers must
baseline at first telemetry.

**The promote → IDR-burst chain, caught live.** ctl-0011 t=89.4:
promote 0→1 (clean link, SNR 33.5). t=89.5 — 157 ms later — residual
demote 1→0 at **u = 0.0, SNR 33.4**: post-FEC base loss with zero
utilization pressure on a pristine link. flight-0004's telemetry books
**+1 `vanished_base` +1 `self_idr_refused`** (= IDR-adjacent) in the
same second. Reading: the promote's bitrate step made the encoder emit
an IDR; the IDR's ~10× burst simultaneously (a) overwhelmed something
on the wire path enough for a one-window residual and (b) overran the
venc ring for one base frame, IDR-adjacent per the guard. Cost: a
spurious demote + rung-1 penalty on a perfect link. This is the
strongest in-flight confirmation yet of the vanish doc's burst-seeding
mechanism, and the concrete lead for "promotes are occasionally
expensive" — relevant to both the 2026-08-10 crash thread and ladder
tuning. Mitigation directions already queued in the vanish doc:
`maxIBytes` (cap the I-frame burst at the source), and the self-IDR
redesign's rate-based guard.

## Data

- Bench recordings: `transient3.jsonl` (+ scratch copies), ringwatch
  logs — GS `/media/dvr` + session scratch.
- Flights: ctl-0011/ctl-0012 (2026-08-13) + flight-0004/0005.jsonl —
  kept OUT of the repo (size); local copies under gitignored
  `docs/superpowers/flightlogs/`, originals on the DVR SD card.

## Follow-ups

1. ~~Zero/snapshot vanish counters at link-establish~~ DONE (same PR):
   `FramePipeline::reset_vanish_counters()` at the FIRST link-establish
   only — counters now read "vanishes since first link".
2. Bench repro of the promote → IDR-burst chain: promote cycling while
   watching `vanished_*` + wire residuals (loss-sim rig + detection now
   see both sides); evaluate `maxIBytes`.
3. Max-gap metric + ctl-log V-line: still unbuilt; deprioritized now
   that the recorder persists jitter and the lag did not recur — revisit
   if pause-and-jump returns.
4. Player-side presentation gaps: still unobserved by any metric;
   unchanged priority.
