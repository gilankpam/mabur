# Handover — the fec span is production-paced; attack the per-byte term

> **⚠ SESSION 4 UPDATE (2026-09-01, same day): the contradiction below
> is RESOLVED — see §19 of the findings doc.** The worker is exonerated
> (90 µs/repair, joins 0.35–0.5 ms mean); the per-byte term is the hot
> thread's OWN feed compute (~4.7 ms/AU isolated) plus ~2.7 ms of
> core-sharing interleave. mt2-row/spin is DEMOTED (would steal the hot
> core). **The affinity half is now SHIPPED and bench-accepted (§20):
> fec p50 10.3 → 9.7 ms, au_tail span 9.0 → 7.2 ms, all gates green,
> rollback `maburd.pre-affinity`** — flight-unvalidated. Remaining
> rank: (1) feed-own alloc/copy diet (~4.7 ms, the big one),
> (2) flush-join relaxation, then the unchanged air half
> (A-MPDU/656). The probe sequence below is kept for provenance; read
> §19 + §20 first.

2026-09-01, bench (drone `192.168.10.152`, GS `10.18.0.1`), branch
`ampdu`. Continues `docs/dq-spike-findings-2026-08-31.md` §14–§18 (the
lab notebook — §17 is the model, §18 the 656 trial; this page is the map
for the next session). Supersedes the attack list of
`handover-usb-feed-2026-09-01.md` (that page's premise was refuted).

## One-paragraph state

fec p50 ≈ 10.3 ms at 11 M/mcs5 (332/w32) decomposes as **~8.6 ms
arrival span + ~2 ms GS publish tail** (au_tail gauge). The span is an
equilibrium: the drone produces bodies at the same ~360 µs/body pace
the air flies them, so every single-side lever measured null (§§8–16).
The 332↔656 pair decomposed the pace: **≈ 0.21 µs/byte + ~100 µs/body**
— the per-byte term dominates, and it is production+air overlapped.
656 halved only the per-body taxes (−1.0 ms) and was rolled back
(jitter trade, §18 postscript). The remaining fec levers, in order:
**(A) cut the per-byte production term** (this page), (B) A-MPDU air
compression — real but double-blocked (needs a bunched feed AND the
aggregate PHY-report poisoning solved), (C) the 2 ms GS tail,
(D) `enc` 7.1 ms (venc, separate fight).

## What already exists — read before designing anything

- **`FecWorker` is already wired into maburd** (`drone/src/main.cpp:999`,
  `UepEncoder` ctor takes it): whole-repair jobs offload to ONE worker
  thread (SPSC, spin-then-sleep, ARMv7 barrier rules measured on the
  SSC338Q — see the header comment; do not "simplify" the spin loop).
- **`bench/fecbench/`** has a full candidate rig + `RESULTS.md`
  (2026-07-17): **mt2-row/spin** (both cores fold half the window rows
  each, private scratch, one XOR merge, atomic spin handoff) wins
  **+39% at the then-production geometry and +51–53% at large
  geometries** over baseline. `mt_encoder.h` + `candidates.cpp` hold
  the implementations. The winner was never promoted into `common/`.
- Baseline repair cost from RESULTS ≈ 0.27 µs/repair-byte single-core —
  **numerically the measured span slope** (0.21–0.28 µs/B). Suspicious.
- Instruments deployed and live: drone `usb_urb` + `dq_split`
  (⚠ cpu_us is ~half wakeup-chain interleave, NOT pure compute — §16),
  GS `rx_pace` (tsfl = air truth) + `au_tail` (span vs publish tail),
  `statstap`, `fecdump` (⚠ t_first anchor is A/B-unsafe under feed
  batching — §16).

## ⚠ Contradiction to resolve FIRST

Repairs are already off-thread — yet the per-byte pace matches the
single-core GF256 repair rate. Both can be true only if the hot thread
still **waits** for the worker at group-seal boundaries (streaming push
emits a body when its SBI group seals; a group containing repair
envelopes seals when the worker delivers). Trace
`UepEncoder`/`SwEncoder` seal logic + `FecWorker` handoff and answer:

1. Does the AU's body emission serialize behind worker latency?
2. What is the worker's per-repair latency + queue depth at 11 M?
   (Cheap gauge: count/time `execute_repair_job` + queue occupancy,
   5 s window, same pattern as usb_urb.)
3. How much of `dq_split cpu_us`'s "compute half" (~4.6 ms/AU) is the
   hot thread's own work (fragment/copy/CRC/SBI pack — should be well
   under 1 ms for 33 KB) vs wait-on-worker?

The answer picks the lever: if the span is worker-latency-bound →
mt2-row/spin (halves repair latency, both cores on one repair) is the
direct hit; if the hot thread's own path is heavier than it should be →
profile it first.

## Probe sequence (cheapest → most committal)

1. **Resolve the contradiction** (read + small gauge, one deploy).
   Worker wait vs own-compute split of the hot thread's per-AU wall.
2. **On-device floor measurement**: `bench/fecbench` (`run_drone.sh`,
   needs maburd stopped — ~5 min link downtime) at TODAY's geometry
   (332/w32/bpb4, per-rung ov pairs 0.5/0.5) for baseline vs
   mt2-row/spin. RESULTS.md predates the venc fold-in and the current
   geometry — re-measure, don't trust 07-17 numbers blind.
3. **Promote mt2-row/spin** into `common/` behind the existing
   FecWorker seam (the candidates are written; this is a port+tests
   job, host suite + `ctest -R 'test_'` + golden vectors must stay
   byte-exact). A/B on bench: au_tail span, fec p50, dq_split,
   ausniff/aucadence, **jitter EMA** (production speedups re-shape
   arrival cadence — the 656 lesson: watch it).
4. **Chain-overhead batching** (independent, smaller): coalesce the
   per-body wakeup chain (hot→txq→writer→pool) at SBI-group
   granularity WITHOUT added hold time — §16 measured ~0.5 ms of
   chain interleave in cpu_us; a designed version must not repeat the
   hog experiment's dq +3 ms.
5. **Only after production leads**: revisit A-MPDU (the air half of
   the compound lever, ~−1.5 ms more) — still gated on the aggregate
   PHY-report poisoning (third-adapter depth measurement or sparse-RF
   acceptance; §12 addendum + §16 C/D churn).
6. (Independent, any time) **GS tail split**: three-way gauge inside
   the finish path (repair vs assembly vs ring write) for the 2 ms
   p50 / 20 ms spikes.

## Success criteria + gates

Target: au_tail span 8.6 → ≤6.5 ms and fec p50 10.3 → ≤8.5 ms at
11 M/mcs5/332 with jitter EMA ≤6 ms and zero gate regressions
(ausniff 59.5+/0 gaps, aucadence within ±1 ms of +0.58, 0 stalls).
Measure everything against the same-scene baselines in this session's
scratchpad (`side_332_base.jsonl`, au_tail ~8.6/1.95, rx_pace 382 µs).

## Do NOT redo (the null table)

tx_threads sweeps, EDCA/CW, A-MPDU-alone, URB batching alone, CPU-hog
bunching, 656-symbol bodies (tried, rolled back — hole sweep PASSED and
stands if re-flagged), any fec A/B judged via fecdump's t_first anchor
or raw dq_split cpu_us (both instruments lie under feed-shape changes).

## Bench state (end of 2026-09-01)

332/w32 everywhere (656 rollback verified), adaptive ladder, 11 M cap,
ch136, agg 0. Deployed instrumented builds: drone maburd =
**fecgauge + thread-names + core-affinity policy** (session 4;
rollback `maburd.pre-affinity` = the gauge build without the policy;
pre-fecgauge/pre-urbgauge/pre-ampdu PRUNED), GS maburgs =
rx_pace+au_tail (rollback `maburgs.pre-rxpace`), fixed maburplay
(rollback `maburplay.pre-healslip`). GS `aucadence.py` refreshed to
ring v2. Config backups `/tmp/*.pre-656` (now equal
to live), `/tmp/*.pre-sat`, `/tmp/maburgs.json.pre-pin` on the devices.
Closing gate 60.0 fps / 0 gaps. ⚠ GS 2.4 GHz AP (AIC8800) drops its
station under strong nearby 5 GHz TX — observation nuisance, wired
uplink would fix.
