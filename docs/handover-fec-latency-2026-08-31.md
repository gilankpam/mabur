# Handover — the `fec`/`dq` latency thread and the A-MPDU probe

2026-08-31, bench (drone `192.168.10.152`, GS `10.18.0.1`), branch
`venc-foldin` (**merged to master 2026-09-02 via PR #34, fast-forward,
branch deleted** — every commit below is on master under its original
SHA). Continues `docs/latency-budget-findings-2026-08-31.md`
and `docs/dq-spike-findings-2026-08-31.md` (the running lab notebook —
read §7–§10 there for the measurements; this page is the map and the
open front).

## One-paragraph state

`dq` is fixed and gone (streaming push, shipped). The next-largest
reducible drone→glass segment is now `fec` — but `fec` is almost all
**air serialization**, not FEC math, and its pace is a fixed **~151 µs
per body** that is genuinely air-side (three independent refutations of
host-side causes). The two levers that can move it are (1) **A-MPDU**
(fold the per-body tax across an aggregate) and (2) **bigger bodies**
(pay it half as often). We were mid-**feasibility probe on #1** when
this handover was cut. Nothing about #1 is committed; the tree and both
devices are in their shipped state.

## What shipped this session (on `venc-foldin` at the time; on master since 2026-09-02)

| commit | what |
|---|---|
| `193745b` | **streaming push** — bodies go to the TxQueue as each SBI group seals, stamped at actual push. `dq` 6–7 ms → ~0; e2e p50 55.9 → 45.3 ms (bench A/B). `UepBodySink` overload on UepEncoder/FramePipeline. |
| `3a93f96` | dq-spike doc §7 (the split: ring 2.6 ms fictitious / FEC-CPU 3.4 ms real / true queue 40 µs) + provenance/observability scale-break notes. |
| `eb0f819`+`ba37049` | **LOWLATENCY VPE→VENC bind REFUTED** on hw (`MI_SYS_BindChnPort2` → `-1610014712`/`0xa00a8008`, venc dies). Would have overlapped encode with sensor readout (~4–6 ms of `enc`); dead on this SDK. Documented at the bind site + `docs/airtime-model.md` dead-knob list. Net code change: none. |
| `407d9ee` | **`tx_send` gauge** (tx thread: send_bodies wall time + body count) + dq-spike §8: fec = 1.2 ms + 0.41 ms/KB; burst pace invariant; **1341 bodies/s ceiling REFUTED** (offered load, link sustains 1826). |
| `0202bc9` | dq-spike §9: **rate sweep** (max_mcs 1/3/5) → pace = 11105 bits/rate + **150.6 µs fixed**; ~⅔ DIFS+backoff, ~¼ preamble, chip ≲15 µs. |
| `a9968eb` | dq-spike §10: **EDCA A/B abandoned** — injection is on the MGMT queue, which can't aggregate and is already near-min-CW; starvation ruled out (streaming vs batch-all drain at identical slope). |

PR #37 (`dq-scope`) covered the streaming-push work; it merged into
`venc-foldin` and closed on push, and reached master with PR #34.

## The load-bearing facts for whoever picks this up

- **`fec` = `t_complete − t_first`** = the AU's bodies 2..N arriving over
  the air (body 1 stamps `t_first`). At mcs5/14 Mb/s: p50 11.7 ms, ~28
  bodies/AU, ~1396 B/body. It scales with **frame size** (0.41 ms/KB) —
  the 10–30 ms "fluctuation" the operator saw is scene-driven frame-size
  variance on a GDR stream (no IDRs on the wire), **not** loss. Repair
  tail is only 1.3% of AUs.
- **Per-body pace = ~151 µs fixed + airtime.** Constant across a 4× rate
  swing (mcs1/3/5), so it's a per-MPDU cost, not per-byte, not duty. The
  ~150 µs matches 802.11 DIFS 34 + mean backoff ~67 (CWmin 15) + HT
  preamble ~36. **BUT** injection rides the **MGMT queue** (QSEL `0x12`,
  `../devourer/src/jaguar3/FrameParserJaguar3.h` `fill_data_tx_desc_8822c`),
  whose CW mapping is likely already VO (CWmin 3) — so the CWmin=15
  arithmetic is the *estimate's* basis, not necessarily the live queue's.
  Either way CW tuning has little headroom; see EDCA-abandoned below.
- **Host-side is NOT the bottleneck** (3 refutations): tx_threads 1/2/4/8
  flat (§8); `tx.send_bodies` returns ~20 µs at every setting incl. 1
  (async libusb, `UsbTransport.cpp`); streaming vs batch-all-push drain
  at identical slope (412 vs 420 ms/MB). USB agg already maxed
  (`usb_agg_max=3` = HalMAC 3-desc cap, `main.cpp:735`).
- **`enc` (7 ms) is SDK-floored** — LOWLATENCY refuted, encode can't
  start before the last readout line. Only a slice-granular transport
  redesign (big project, reopens the whole-AU invariant) beats it.

## The open front: A-MPDU feasibility probe (#1)

**Goal:** does the drone's **RTL8812EU (jaguar3 / 8822e family)**
aggregate host-injected QoS-Data frames on a *data* queue? If yes,
aggregation folds the ~151 µs across each aggregate → `fec` p50 ~11.7 →
~7 ms (−40%), the biggest available win, and it *shrinks* drone TX
occupancy (helps the RCF uplink). If no, #1 is dead on this silicon and
#2 (big bodies) becomes the only lever.

**Why it's non-trivial:** the MGMT queue mabur injects on **never
aggregates** ("AGG_EN wedges the queue" — `../devourer/docs/aggregation.md`).
Aggregation needs migrating injection to a data queue: QoS-Data TID0 +
AGG_EN + `no_ack` (retry-limit 0, FEC covers loss, else 92% wasted
re-airings) + the pacing registers (`REG_AMPDU_MAX_TIME` 0x455=0x20;
`0x4bc` BIT6 cleared) + a deep multi-URB feed. All of that is a wire flag
day (QoS-Data header is longer — GS RX parser must accept it).

**Where the probe was cut:** I was checking whether devourer's
aggregation is even *validated on 8822e* (all the aggregation docs are
8822BU / jaguar2). Last command was grepping `../devourer/docs/` and the
jaguar3 tree for ampdu/aggregation status — **inconclusive, not yet
answered.** Resume there.

> **ANSWERED 2026-09-01 — the 8822E aggregates.** Steps 1 *and* an
> SDR-free variant of step 2 are done: docs verdict (wired
> generation-wide, jaguar3-TX proven on the C die, E-die TX was the
> gap) + a drone→GS on-air A/B with txdemo/rxdemo cross-built from
> this repo's build trees. 93% of aggregated arrivals at pure MPDU
> airtime (216 µs at MCS5/1396 B, vs 309 µs mgmt singles / 429 µs
> un-aggregated BE), +32% delivered fps, clean delivery, no queue
> wedge, host seqs preserved. Full numbers, integration caveats and
> the step-3 shape: `docs/dq-spike-findings-2026-08-31.md` §11. The
> B210 SDR A/B is downgraded to confirmatory. Next: step 3 (the wire
> flag day), gated on fecdump drain slope + ausniff + aucadence +
> RCF `close_ms`.

### Recommended probe sequence (cheapest → most committal)

1. **Docs/code feasibility (zero risk, ~15 min).** Confirm in
   `../devourer/` whether data-queue A-MPDU is wired + on-air-validated
   for **8822e specifically** (not just 8822BU). Check
   `src/jaguar3/CLAUDE.md`, `docs/8822e-quirks.md`, `docs/aggregation.md`,
   and `SetAmpduMode`/`AmpduMode` call sites in `RtlJaguar3Device.cpp`.
   If the aggregation path is 8822BU-only or explicitly unvalidated on
   8822e → strong negative, stop and pivot to #2.
2. **SDR ground-truth A/B (the real answer, needs bench hardware).**
   `../devourer/tests/ampdu_onair_ab.sh` — `singles` vs `ampdu` mode,
   verdict = accepted-fps per SDR duty point. **Needs a B210 at the
   bench**; cannot be driven from this remote session. This is the
   canonical test and the reason bench discipline exists ("judge TX by
   SDR duty × PHY rate, never RX frame counts").
3. **Only if 1+2 say aggregation works:** wire QoS-Data + data-queue +
   AGG through maburd's TX path (`drone/src/radio_tx.cpp` builds frames;
   devourer knobs `DEVOURER_TX_QSEL` / `SetAmpduMode` exist), teach the
   GS RX parser the QoS-Data header, and A/B end-to-end with `fecdump`
   drain slope + ausniff + **RCF `close_ms` on screen** (shared-channel
   effect). Multi-day, wire flag day.

## Lever #2 (fallback / compounding): 656-symbol big bodies

Bodies 1396 → 2708 B via `symbol_size` 332 → 656 (shortlist also w32 →
w16). Half the bodies → the 151 µs paid half as often → ~20–25% off
`fec`. Mabur-side + FEC config, no devourer. **Hard gate:** the all-8-MCS
frame-length **hole sweep at wall power** (linkbench rig,
`out/arm/linkbench-tx` + GS `linkbench-rx`) — 2708 B is untested at all
rates, and the mcs6+STBC 1392–1400 B hole already bit the old 328×4=1396
in prod. Also: both-ends flag day; coarser loss granularity worsens the
repair tail (`fec` max), worst at low rungs (2708 B = 1.7 ms air at
mcs1). Composes with #1 (big bodies inside aggregates).

## Proportionality (size the prize honestly)

These are percentages of *serialization*, which scales with bitrate. At
the bench's 14 Mb/s, #1's full win ≈ 4–5 ms of `fec` (likely ~3–4 ms at
the glass after the vsync servo takes its cut); at a prod-like 8 Mb/s,
scale down ~40%. Neither lever touches `enc` (SDK floor) or the display
tail. This is specifically the frame-delivery segment.

## Bench + tree state (verified before this handover)

- Drone `/usr/bin/maburd` = streaming+gauge build (`dc3bdb4`);
  rollbacks `maburd.pre-dqsplit` (batch-all, pre-streaming) and
  `maburd.pre-lat` on rootfs — **3 binaries, `df` before next deploy**
  (5.7 M rootfs, was 55% used).
- Configs byte-identical to pre-session: drone `/etc/mabur.json`
  (14 Mb/s cap, but note the ladder is flat-0.5/0.5 only at mcs4/5;
  mcs1–3 still asym), GS `/etc/maburgs.json` (max_mcs 5). All sweep
  backups (`*.pre-txsweep`, `*.pre-mcssweep`, `mabur.json.pre-tx1`)
  restored + verified.
- `../devourer` clean (no changes made — probe never got past docs).
- Bench healthy at cut: ausniff 60.8 fps / 0 gaps / both classes 365/365.
- Scratchpad tools (reusable): `fecdump.py` (per-AU t_first/t_complete
  CSV from the AU ring — the drain-slope instrument), `fec60.csv` /
  `fec_mcs1.csv` / `fec_mcs3.csv` / `fec_batchall.csv` (the datasets
  behind §8–§10). In this session's scratchpad dir.

## Push

Done. `venc-foldin` was pushed and PR #37 closed; on 2026-09-02 master
fast-forwarded to `venc-foldin` (PR #34) and the branch was deleted, so
everything above lives on master. New work branches from master.
