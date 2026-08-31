# The `dq` spike — two hypotheses refuted, and `dq` does not measure what its name says

2026-08-31, bench (drone 192.168.10.152, GS 10.18.0.1), both ends at the
prod build. Motivated by follow-up #3 of
`docs/latency-budget-findings-2026-08-31.md` ("find and model the real
drain ceiling"), which ranks the standing ~7 ms `dq` as the third-largest
reducible segment.

The spike set out to name what limits the ~15 Mb/s effective TX drain. It
did not find the limit. It found that **the `dq` segment spans a wider
interval than its name and its documentation claim**, which changes what
every `dq` number recorded so far means — including the ones in the
latency-budget doc. That is the headline; the two refuted hypotheses are
secondary.

Config context for every measurement below: mcs5 park, 20 MHz, ladder
capped by `link.max_mcs`, **asymmetric overhead pair (base 1.0 / enh 0.5)
at every rung**, `encoder.bitrate_max_kbps: 8000`, `radio.tx_threads`
default 4, GS `display.vsync_lock: true` / `vsync_lead_ms: 6`.

## 1. `dq` includes the frame-ring wait and the FEC math

`dq` is documented and named as the TxQueue push→pop wait. It is not. The
stamp written into every body is sampled at the **top of the hot-loop
iteration**, not at the push:

| line | what happens |
|---|---|
| `drone/src/main.cpp:1008` | `uint64_t now = now_steady_ms()` — **`dq` clock starts** |
| `drone/src/main.cpp:1058` | `fsrc.read(fbuf, size, 5, &meta)` — blocking venc-ring read, 5 ms timeout |
| `drone/src/main.cpp:1075` | `pipe.encode(...)` → `frame_pipeline.cpp:66` `uep.add_frame(...)`: fragmentation → sliding-window FEC (GF256) → SBI packing |
| `drone/src/main.cpp:1085` | `feed.on_frame(...)` AirFeed accounting |
| `drone/src/main.cpp:1093` | `b.enqueued_ms = now` — **the same `now` from 1008**, never re-sampled |
| `drone/src/main.cpp:1184` | `w = pop_ms - b.enqueued_ms` — reported as `dq` |

So `dq` = venc-ring read wait **+** fragmentation/FEC/SBI-pack CPU **+**
the actual queue wait. The whole frame → chunk → FEC block → SBI body
chain sits inside it.

The ring-read term is systematic, not incidental. Frames arrive every
16.7 ms and the loop polls with a 5 ms timeout, so on the iteration that
succeeds the frame becomes available at a roughly uniform point in a 5 ms
window *after* `now` was stamped: **mean ~2.5 ms, bounded 0–5 ms**, with
no relation to the TX queue. The GF256 encode of a ~17 kB frame at
window 32 / bpb 4 / symbol 332 is the other pre-push term and has never
been measured anywhere.

⚠ `drone.txq_wait_ms` (the drone's own window-max gauge, `main.cpp:1189`)
is derived from the same `enqueued_ms` and carries the same inflation.

### Why `chk` never caught it

`air` is computed as a residual — `air = t_first − enc − dq − map(pts)`
(`gs/src/main.cpp:328`, mirrored in `lat_tracker.cpp:86`). Inflating `dq`
deflates `air` by exactly the same amount, so the additive invariant is
untouched and `chk` stays 0. Measured `chk` across 6729 real 1 Hz windows
from `/media/dvr/log/lat-00*.log`: min −0.00, max +0.30 ms, mean +0.001.

**`chk` validates the total and is blind to attribution by construction.**
Any future mis-scoping between two adjacent segments will pass it too.

### Consequence for `air`

`air` is not "first-body radio transit excess". It is the residual of
everything between the encoder and first-body arrival that is not `enc`
or `dq`, minus the best-ever case the min-anchor holds. With `dq`
over-wide, `air` is correspondingly narrow.

### `dq` is also the *minimum* of its AU

Every body of one AU is stamped with a single `enqueued_ms` and pushed in
one tight loop (`main.cpp:1093-1094`), so arrival at the queue is
genuinely instantaneous. `q_ms` is latched GS-side from the body carrying
fragment index 0 (`gs/src/frame_stream.cpp:47`), which is first into a
FIFO and therefore first out — so `dq` reports the **smallest** wait of
that AU's ~22 bodies. The remaining bodies' queueing surfaces at the GS
inside `fec` (t_first→t_complete), not in `dq`.

`now_steady_ms()` is millisecond resolution, so `dq` additionally carries
1 ms quantization, which `air` absorbs.

## 2. CPU starvation — refuted

Drone is 2 cores, ARMv7 rev 5. Sampled `/proc/stat` over 5 s during
steady TX:

```
CPU busy 40.9% of 2 cores   (idle 59.1%, iowait 0.0%)
thread states: R 2, S 80, D 12, Z 0
```

Not saturated. The hypothesis came from `drone.sys.load` reading 13.08 in
the sideport, which was a misreading on our part — see below.

### ⚠ `drone.sys.load` is a broken gauge on this platform

`sys.load` is `/proc/loadavg` field 1 (`drone/src/telemetry.cpp:127`
`read_load1`, wired as `load_x100` at `telemetry.cpp:93`, divided by 100
at `gs/src/stats_exporter.cpp:535`). The value is reported faithfully;
the problem is what it counts here.

Loadavg counts runnable **plus uninterruptible-sleep** tasks. All 12
D-state threads on this drone are SigmaStar MI SDK kernel workers, none
of them mabur's:

```
vif0/1/2_P0_MAIN, vpe0_P0..P3_MAIN, venc0/1_P0_MAIN
                                   → mi_sys_internal_main_worker_thread
mi_log, ehci_monitor, VEP_DumpTaskThr → msleep
```

They are parked permanently, so loadavg reads a flat **13.17 / 13.37 /
13.12** across the 1/5/15-minute windows — the signature of a constant
background, not a workload. It will read ~13 whether the drone is idle or
pegged. Do not use `sys.load` as a CPU signal; it tracks the vendor SDK,
not mabur.

## 3. USB round-trips — refuted

`radio.tx_threads` swept 1 / 2 / 4 / 8, 60 s per point, config-only on
the drone with a `maburd` restart between points. Each point: 60 player
`lat:` windows (n≥55) and ~298 sideport datagrams, all four confirmed on
rung 5 throughout.

| `tx_threads` | `dq` p50/max | `e2e` p50 | `fec` p50 | `txq_wait_ms` | `sent_pps` | air kb/s | video | jitter | txq drops |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 7.0 / 14.5 | 56.5 | 8.0 | 19 | 1342 | 15020 | 7.73 | 11.3 | 0 |
| 2 | 7.0 / 18.0 | 56.0 | 8.0 | 24 | 1343 | 15068 | 7.75 | 11.6 | 0 |
| 4 | 7.0 / 17.0 | 60.0 | 8.5 | 23 | 1339 | 15042 | 7.75 | 11.3 | 0 |
| 8 | 7.0 / 19.0 | 57.0 | 8.0 | 24 | 1341 | 14969 | 7.65 | 12.2 | 0 |

Flat. `dq` p50 is 7.0 ms at every setting; the drain is 1339–1343
bodies/s at every setting.

`tx_threads: 1` does not merely shrink the pool — `main.cpp:769` attaches
`UsbTxPool` only when `tx_threads > 1`, so the 1-thread point is a direct
inline blocking send with no pool at all. It matches 8 threads exactly.
Consistent with `usb_tx_pool.h`'s own note that the single-sender cap it
was built to lift is ~26 Mb/s: at 15 Mb/s offered, that cap was never
binding, and the 4-thread pool buys nothing at this operating point.

Finding 1 also explains the flatness: if a large fraction of `dq` is not
queueing, no amount of sender parallelism can move it.

## 4. Nothing in the pipeline is congested

Measured at every sweep point:

- `enc.cmd_kbps` pinned at exactly **8000, p50 *and* max** — equal to
  `encoder.bitrate_max_kbps` in `/etc/mabur.json`. The video rate sits on
  its configured ceiling, not on any link-derived one.
- `link.air_pct` 28.9%
- `enc.venc_ring_fill_pct` 0, `ring_drops` 0, `venc_full_drops` 0
- `txq.depth` sampled 0, `txq.drops` 0, `radio.usb_fail` 0
- CPU 40.9% of 2 cores

⚠ **Provenance caution on "the ~15 Mb/s effective drain ceiling".** The
latency-budget doc infers that ceiling from measured drain ≈ measured
injection. On this bench injection is set by a config constant, so the
equality is not evidence of a ceiling. Whether 1341 bodies/s is a wall or
simply today's offered load is **unresolved** — the spike did not settle
it.

Note also that `txq.depth` sampling at 0 while `txq_wait_ms` reads 19–24
is not a contradiction: depth is an instantaneous ~1 Hz sample that
almost always lands between bursts, and wait is a per-body max that sees
inside them.

## 5. Rung A/B: the ladder rung is a bitrate lever, not a latency lever

Run alongside the spike. `link.max_mcs` 1 vs 5, matched 120 s windows,
125 `lat:` windows each, single-rung throughout (595/595 datagrams on the
target rung both sides), identical overhead policy at both rungs.

| | mcs5 | mcs1 | Δ p50, bootstrap 95% CI |
|---|---|---|---|
| video | 7.67 Mb/s | 3.34 Mb/s | −56% |
| air bytes | 14.98 Mb/s | 6.72 Mb/s | −55% |
| `sent_pps` | 1340 | 558 | |
| bodies/AU | 22.4 | 9.4 | |
| bytes/body | ~1396 | ~1522 | |
| `air_pct` | 28.8% | 52.3% | |
| `jitter_ms` | 12.2 | 9.4 | |
| `enc` | 7 | 7 | +0.0 [+0, +0] |
| `dq` | 7 | 5 | **−2.0 [−2, −1]** |
| `air` | 4 | 4 | +0.0 [−2, +0] |
| `fec` | 8 | 6 | **−2.0 [−2, −2]** |
| `dec` | 7 | 7 | +0.0 [+0, +0] |
| `reg` | 18 | 19 | +1.0 [−1, +5] |
| `dsp` | 5 | 5 | +0.0 |
| `e2e` | 59 | 56 | **−3.0 [−5, −2]** |

Dropping four rungs costs 56% of the video bitrate and buys 3 ms.
Everything not byte-count-driven is unchanged. `reg` absorbed part of the
upstream saving (`fec` −2 yielded `e2e` −3 with `reg` +1), which is what
a vblank-servoed hold does: time saved upstream is partly handed back as
extra hold unless the hold depth moves with it.

⚠ **`max_mcs` and `static_mcs` are different experiments with
opposite-signed results.** `max_mcs` caps the ladder and the bitrate
blend re-targets to the rung, so `dq` **falls**. `static_mcs` pins the
rung while the blend keeps aiming at the old budget, deliberately
overloading the link, so `dq` **rises** — which is what the acceptance
runbook's "dq rises under pinned-mcs1 overload" step exercises. The
runbook does not distinguish them.

Also measured: bodies are ~1400 B, i.e. already at the practical MPDU
ceiling, so body count cannot be reduced at constant byte count.

## 6. Related measurement caveats found alongside

Surfaced while validating the OSD latency row; they bear on how the
tables above are read.

- **The "p99" columns are window maxima.** `percentile_` computes
  `idx = (n*99)/100`, which equals `n−1` for every `n ≤ 100`. Player
  windows carry n≈59 and sideport windows n≈12, so every value labelled
  p99 — in `lat:` lines, in `link.video.lat.*[1]`, in `maburtop.py:786`,
  and in the OSD headline — is the worst sample of its window. At these
  sample counts a real p99 is not resolvable; the label is what is wrong.
- **The OSD's seven segments need not sum to its headline.** All eight
  values are rounded to ms independently (`lat_tracker.cpp:200`).
  Simulated over the real distribution: non-zero drift in 52% of windows,
  |drift| ≥ 2 ms in 4.8%, worst ±3 ms.
- **Frames whose fragment-0 body failed FCS report `enc` = `dq` = 0.**
  `common/src/uep_decoder.cpp:167` degrades `q_ms`/`enc_us` to 0 =
  unknown from a corrupt body, and `lat_tracker.cpp:73` then subtracts
  nothing, so `air` absorbs both (≈ +14 ms at today's numbers). `e2e`
  stays correct; the breakdown does not. Pinned as intended behaviour by
  `tests/test_lat_tracker.cpp:157`. There is no counter for the
  incidence, and it lands hardest on the loss-affected frames most likely
  to be the max frame the OSD displays.
- **Cross-check that did pass:** maburgs' `LatWindow` and maburplay's
  `LatTracker` compute the four head segments independently, in separate
  processes with separate `PtsAnchor` instances. Over a live 35 s window
  they agree at p50 within 0.16 ms (`enc`), 0.32 (`air`), 1.00 (`dq`),
  1.26 (`fec`) — the residual explained by the `lat:` line flooring
  µs→ms with integer division (`main.cpp:1622`), a systematic −0.5 ms.

## 7. Validated + fixed (same day, follow-up session)

Open question 1 is answered by measurement and the latency is gone. µs
instrumentation (dq_split/dq_queue stderr gauges, branch `dq-scope`)
split the standing ~6 ms wire `dq` at the mcs5 park:

| component | measured | verdict |
|---|---|---|
| venc-ring wait (loop-top → read return) | mean 2.5–2.9 ms, max ~5.8 | the predicted uniform 0–5 ms; **fictitious** — the frame did not exist yet |
| FEC/SBI-pack CPU (read → all bodies pushed) | mean 3.4 ms, max 16.6 (IDR) | **real**, serial, in front of an idle radio — and this is *with* the async FecWorker already offloading repairs |
| true queue wait, AU-first body (what the GS latches) | mean **~40 µs**, max <1 ms | the thing `dq` is named for is essentially zero |

Sum 2.6 + 3.4 + 0.04 ≈ 6.0 ms = the GS `dq` p50 in the same run. §1's
claim is confirmed with a complete accounting; the ring-read futex wake
means the ring term is pure misattribution, not real frame latency.

**Fix: streaming push.** `UepEncoder`/`FramePipeline` gained a
`UepBodySink` overload; the hot loop pushes each body to the TxQueue the
moment its SBI group seals, stamped `enqueued_ms` at the actual push. The
radio drains early bodies while later FEC blocks are still packing (the
TX thread keeps pace: per-body queue wait fell 520–1140 µs → 30–50 µs),
and the wire `q_ms` becomes the true queue wait.

Same-config A/B (prod binary swapped back in between runs, asym pair,
mcs5 park, same hour): `e2e` p50 mean **55.9 → 45.3 ms**; `dq` 6–7 → 0;
`air` 2–3 → 0–1; `fec` ~+1; `reg` mean 16.5 → 13.9 (earlier, steadier
completion lets the vsync servo hold less). aucadence base−enh offset
4.57 → 3.79 ms (both under the asym pair — treat **+4.57** as the
asym-pair prod baseline this section supersedes). Gates: ausniff 60.0
fps / 0 gaps / 0 incomplete; host suite 101/101. `chk` stays 0, cadence
59.5+ fps throughout.

Consequences for this doc's earlier sections: the "standing ~7 ms dq"
segment no longer exists to reduce (follow-up #3 of the latency-budget
doc is closed by this, not by a drain-ceiling model); `dq` > a few ms is
now a genuine TxQueue-backlog signal. Scale break recorded in
`docs/data-provenance.md`. Open question 2 (is 1341 bodies/s a wall?)
remains open but is no longer load-bearing for latency: nothing queues
behind it at today's operating point.

Rollback: `maburd.pre-dqsplit` on the drone (prod build; config
untouched, wire unchanged — GS binary not involved).

## 8. `fec` follow-up (same day, evening): serialization pace measured,
## host-side pacing refuted, 1341 ceiling refuted

Operator raised `bitrate_max_kbps` 8000→14000 and flattened the
mcs4/5 pairs to 0.5/0.5; `fec` then read 10–30 ms. Per-AU regression
over 60 s / 3568 AUs from the AU ring (scratchpad `fecdump.py`, headers
only):

    fec = 1.2 ms + 0.41 ms/KB payload      (residual sd 2.7 ms)

- ~90% of the fluctuation is **frame-size variance** (GDR stream, no
  IDRs; frames 15–49 KB → 7–21 ms of pure serialization). Repair/loss
  tail: 1.3% of AUs with residual > 8 ms (worst +26 ms).
- Burst drain pace is **0.385 ms/body (~29 Mb/s air) — identical at
  8 and 14 Mb/s offered**, and identical to the §5-era burst pace. It is
  the invariant.
- The link sustains ~1826 bodies/s average at the new bitrate →
  **the "1341 bodies/s ceiling" of §4 was offered load, not a wall.**
  Open question 2 is closed.
- The pace is **not host-side**: TxQueue wait ~40 µs, and the new
  `tx_send` gauge shows `tx.send_bodies` returns in ~20 µs at every
  `tx_threads` setting including 1 — devourer's send path is async
  libusb submission all the way down
  (`third_party/devourer/src/UsbTransport.cpp`), so maburd never blocks
  and never paces. The 0.385 ms/body is set below libusb: per-MPDU
  ~0.16 ms of chip/medium-access overhead (descriptor processing, DIFS +
  backoff — injection still does CSMA) on top of ~0.22 ms mcs5 airtime.
  Splitting those two needs devourer/PHY instrumentation, not maburd's.

Levers, in order: (a) bigger bodies — the 656-symbol/w16 candidate
halves the per-MPDU tax per byte (~20–25% off `fec`), gated on the
all-MCS hole sweep; (b) PHY medium-access tuning (backoff/CW) in
devourer — shared-channel risk, RCF uplink lives there too; (c) bitrate,
which is just the quality trade. Host-side work (URB batching, tx
threads, pool depth) is refuted — do not spend there.

## 9. Rate sweep: the per-body dead time is CSMA, not silicon

`link.max_mcs` 1 / 3 / 5 (GS config-only, 60 s + 3568 AUs per point,
per-sid regression through each class's overhead multiplier — mcs1-3
rungs run the asym pair). Fit across all six (rung, sid) points:

    pace_us = 11105 bits / phy_rate + 150.6 µs     (residuals ±50 µs)

The rate term back-computes to 1388 B = exactly one body; the intercept
is **constant across a 4× rate swing** — the dead time is a fixed
per-MPDU cost, not a duty-cycle effect and not per-byte. Against known
802.11 constants (5 GHz): DIFS 34 + mean backoff ~67 (CWmin 15, 9 µs
slots) + HT-mixed preamble ~36 ≈ 137 of the 151 µs → **~⅔ is DIFS +
backoff, ~¼ preamble, chip/descriptor ≲ 15 µs**.

Consequences for the §8 levers: CW/EDCA tuning in devourer attacks the
~100 µs CSMA share directly (fec p50 −~25% at mcs5 if eliminated;
⚠ drone TX aggression trades against RCF uplink loss — watch close_ms
in any A/B); A-MPDU would fold preamble + DIFS across an aggregate but
is unverified in injection; bigger bodies halve how often the whole
151 µs is paid. Chip-side work is refuted — nothing to win there.

## 10. EDCA A/B abandoned — the injected queue can't use the lever

Set-up for the CW/EDCA A/B in `../devourer` (jaguar3, the drone's
RTL8812EU) found the premise doesn't hold, so no register was changed:

- **Injection rides the MGMT queue** (`QSEL 0x12`, hardcoded in
  `FrameParserJaguar3.h` `fill_data_tx_desc_8822c`; the monitor-inject
  default). The `REG_EDCA_BE_PARAM` CWmin=15 that the §9 backoff estimate
  assumed governs a queue mabur never touches. Realtek's most-aggressive
  AC is already VO (AIFS 38 / **CWmin 3**), so even the tunable ceiling is
  ~3-slot backoff, not 15 — little headroom to win by lowering CW.
- **The real per-MPDU remedy is aggregation, and the MGMT queue cannot
  aggregate** — `../devourer/docs/aggregation.md`: "the MGMT queue (0x12)
  never aggregates — AGG_EN there wedges the queue." Folding preamble +
  access across N frames needs migrating injection onto a data queue
  (QoS-Data TID0 + AGG_EN + no_ack retry-limit-0) plus the aggregation
  pacing stack — a real project devourer already has A/B rigs for
  (`tests/ampdu_onair_ab.sh`), not a one-register poke.
- **USB feed is already maxed** for the current queue: `usb_agg_max=3`
  (`main.cpp:735`) = the HalMAC 3-descriptor cap; mabur pops 3/batch.

**Starvation ruled out (mabur-side A/B, binaries already on device):**
streaming-push (dc3bdb4) vs batch-all-at-once (`maburd.pre-dqsplit`) at
identical config drain at the **same slope** — 412 vs 420 ms/MB (noise),
fec p50 11.7 vs 11.3, differing only in a +0.7 ms fixed intercept for
streaming. Handing the tx thread all ~22 bodies at once vs trickling
them as they seal makes no difference to the per-body air cadence, so
the drain pace is genuinely air-side, not URB gapping. Third independent
confirmation (with §8's tx_send 20 µs and TxQueue wait 40 µs).

**Net:** the §9 dead time is real air-side per-MPDU overhead, but it is
not CW-tunable on mabur's queue — the lever is A-MPDU (queue migration,
a devourer project) or bigger bodies (656-symbol, the hole-sweep gate),
not EDCA. Config + both devices restored; devourer untouched.

## 11. A-MPDU feasibility answered: the 8822E aggregates (2026-09-01)

The handover's open question — is devourer's aggregation validated on
8822E, or only on 8822BU/jaguar2? — closed in two halves.

**Docs/history half.** Aggregation is wired generation-wide (devourer
PR #239): jaguar3's `SetAmpduMode` programs the full recipe —
descriptor half (data QSEL + AGG_EN + MAX_AGG_NUM + density +
retry-limit, `RtlJaguar3Device.cpp:2053`) plus the `0x455`
aggregate-fill timer (`0x4bc` burst-mode is a no-op on this family; no
bring-up write exists to undo). On-air validation existed for jaguar3
TX **on the C die only** (`ampdu_ba_check.sh` defaults to an 8822CU TX;
devourer PR #373 ran its BA arm on an 8812CU). A-MPDU **RX** through an
8812EU was already demonstrated (PR #373's DUT). The E die as
*aggregating TX* — the drone's exact chip — had never been aired.
PR #373's "−8% at the ARQ shape, don't enable on the FPV link" verdict
is about the ACKed/BlockAck flavor at MCS3/512 B; mabur's shape is
broadcast + QoS No-Ack + retry 0 + FEC, and the goal is latency, not
goodput — different regime, but it is the reason to measure our own
shape.

**Hardware half — first on-air A-MPDU TX from the E die, this bench.**
txdemo (armv7) and rxdemo (aarch64) built straight from the mabur
cross-build trees (`--target txdemo`/`rxdemo`; devourer is
EXCLUDE_FROM_ALL but the targets resolve). Drone RTL8812EU TX →
GS 8812EU RX, ch161, MCS5, 1396 B QoS-Data — mabur's body size and
park rate — 8 s per cell, cells mirroring
`../devourer/tests/ampdu_spike.sh`:

| cell | recipe | uniq fps | paggr | inter-frame tsfl Δ |
|---|---|---|---|---|
| control | mgmt queue, singles | 2346 | 0% | median 309 µs (74% in 300–349) |
| qsel0 | BE data queue, no AGG | 1671 | 0% | median 429 µs (spread 350–499) |
| ampdu_rty0 | BE + AGG_EN 16/7/rty0, single URBs | **3094** | 99% | **median 216 µs, 93% in 200–249** |
| ampdu_rty0_urb | + batch-16/USB-agg | 3090 | 99% | same |
| mode | `SetAmpduMode 0/16` (0x455=0x20) | 2580 | 99% | median 232 µs, 74% in 200–249 |
| mode_thr4 | + 4 sender threads | 2594 | 99% | same |
| ampdu_mgmt | mgmt queue + AGG_EN | 2341 | 0% | = control |

1396 B at MCS5 is ~216 µs of pure MPDU airtime, so **93% of aggregated
arrivals at 200–249 µs means subframes back-to-back on air — the
per-MPDU dead time inside an aggregate is ~0**. That is the entire §9
lever, confirmed on the drone's own silicon. Delivery stayed clean in
every cell (unique == received, ~99.7% of offered, `retry_flagged` 0%,
no BA re-air storm — retry-limit-0 works). The equal-`tsfl` burst
marker from the 8822BU-era spike does NOT fire on an 8812EU RX (tsfl
stamps per-MPDU here); use the Δ-histogram instead.

Second-order findings that shape the integration:

- **The un-aggregated BE data queue is *slower* than mgmt** (429 vs
  309 µs — BE AIFS/CWmin 15 vs the near-VO mgmt queue). Migrating
  injection to a data queue only pays *with* AGG_EN on. Corollary: on
  the data queue the §10 EDCA lever (REG_EDCA_BE_PARAM) becomes usable
  as a second-order tune.
- **AGG_EN on the MGMT queue neither aggregates nor wedges on the E**
  (identical to control; milder than the 8822BU "wedges the queue"
  finding — but still useless).
- **The 0x455 fill timer trades depth for launch latency**: raw AGG_EN
  cells (bring-up 0x70 ≈ 3 ms fill window) formed ~20-deep aggregates
  (boundary deltas ~4% of arrivals); `SetAmpduMode`'s 0x20 (~0.8 ms)
  launched at ~4-deep (26% boundaries) and still delivered +10% over
  control. For mabur's bursty per-frame feed, the timer bounds the
  *tail* body's wait — a knob to sweep in step 3, not a fixed choice.
- **Host-stamped seq_ctl survives**: no renumbering observed (all seqs
  arrived exactly as stamped; devourer never sets EN_HWSEQ). The GS
  gap detector's seq walk should keep working unchanged — re-verify in
  the integration A/B since a constant-0 stamp can't distinguish
  "preserved" from "zeroed".
- GS parser flag day is small: `sa_canonical` (addr2) and seq_ctl keep
  their byte offsets in a QoS-Data header; only the body start moves
  24 → 26 (`radio_frontend.cpp` `kDot11`), keyed on FC type.
- **Only the video path (RadioTx) switches to QoS-Data.** The drone's
  control-plane TX (DISC_ACK, telemetry, MSP — main.cpp's local
  `build_dot11_header`) intentionally stays 0x40 mgmt-queue singles:
  DISC_ACK teaches caps and must never wait in an aggregate fill timer
  (its loss mode mimics the old stale-caps deadlock), and the GS keys
  header parsing on FC type, so both layouts coexist. Decided at the
  ampdu branch's final review, 2026-09-01.

**Projection for `fec`** (28 bodies/AU at mcs5): today ≈ 27 × (213 µs
air + 151 µs dead) ≈ 9.8 ms of serialization; aggregated ≈ 27 × 216 µs
+ a few boundary taxes ≈ 6.1 ms → the handover's ~−40% projection
stands, now with the enabling fact proven. Aggregation also *shrinks*
drone TX occupancy for the same payload (fewer preamble+backoff
periods), which is the RCF-uplink direction we want; gate step 3 on
`close_ms` as planned.

**What remains before wiring mabur:** the SDR duty A/B
(`ampdu_onair_ab.sh`, needs the bench B210) is now confirmatory —
occupancy/efficiency ground truth per bench discipline — rather than
the go/no-go it was when the E die was unproven. The real remaining
work is step 3: QoS-Data header in `radio_tx.cpp`, `SetAmpduMode` in
maburd bring-up, the GS body-offset change, 0x455/MAX_AGG_NUM tuning
against the burst-tail, and the end-to-end gates (fecdump drain slope,
ausniff, aucadence, RCF close_ms). Spike rig + analyzer:
`tools/bench/ampdu_e_spike.sh`, `tools/bench/ampdu_e_analyze.py`.

## 12. A-MPDU deployed — and the premise refuted on maburd's own feed
## (2026-09-01, bench)

The §11 spike led to the full integration (branch `ampdu`, spec
2026-09-01-ampdu-design.md): QoS-Data wire on the video path, FC-keyed GS
parser (order-free deploy — a mixed pair ran 59.9 fps/0 gaps live),
`SetAmpduMode` at bring-up behind `ampdu.max_num` (default 6). Deployed
GS-first then drone, rollbacks `*.pre-ampdu` both ends, drone rootfs
pruned to one rollback. Three hardware findings, in discovery order:

**1. A-MPDU subframes carry no PHY status → RF telemetry poisoned
(fixed).** ~94% of aggregated frames have no PHY report; devourer zeroes
their rssi/snr and the GS folded those zeros (plus rare garbage, rssi 241)
into every EMA. On-screen effect: RSSI/SNR "jumping", SNR 32 → 15
artifact, and pollution-driven mcs4 demotes. Two-part fix, both reviewed:
`RxBody.phy_valid` gating all `fold_rf` sites (mabur, commit bca7a42 —
EVM already had the skip-unsampled convention; rssi/snr did not), and
devourer `jgr3-physt` branch (f18bf1b, local): jaguar3 never set
`RxAtrib.physt` on ANY frame — `parse_phy_sts_jgr3` now returns whether
it parsed a page it understands, and devourer's own internal rx EMAs gate
on it too. Post-fix: s0 RSSI −29…−47 dBm / SNR 28–34 dB (sane), ladder
parked at mcs5 350/350 windows. ⚠ GS builds now require devourer
`jgr3-physt` until it merges to devourer master.

**2. Aggregation does NOT move maburd's drain pace — the §9 "DIFS+
backoff" attribution was wrong.** Depth sweep at 14 Mb/s saturation,
90/60 s fecdump each:

| config | fec p50 | p90 | p99 |
|---|---|---|---|
| pre-ampdu (0x40 mgmt singles) | 16.41 | 24.05 | 34.25 |
| agg OFF (QoS mgmt singles) | 16.16 | 24.36 | 32.82 |
| agg 6 / 0x20 (default) | 16.36 | 24.71 | 34.12 |
| agg 31 / 0x70 (max depth) | 15.47 | 23.40 | 34.76 |

Flat. Aggregates demonstrably form (finding 1's subframes are the proof),
yet fec and the saturated throughput ceiling (~20 Mb/s effective) don't
move. Together with §10 (EDCA null) and the rung A/B ("dq tracks byte
rate, not airtime"), the ~151 µs per-body tax is **host/USB-side**
(the ~0.4 ms bulk-OUT acceptance handshake ÷ 3-frame URBs ≈ 133 µs/body
matches), not medium access — the §11 spike's +32% came from txdemo's
flood regime, which doesn't reproduce maburd's feed. A-MPDU on air is
harmless here (all gates clean) but buys no fec latency until the USB
feed itself is reworked.

**3. The operator's high on-screen Lat = saturation queueing, and it
predates A-MPDU.** With the encoder at its 14 Mb/s cap × 1.5 FEC overhead
≈ 19.5–21 Mb/s offered against the ~20 Mb/s ceiling, the `air` segment
holds a ~100 ms standing queue (measured p50 98.7–117 ms, pre AND post).
Channel change 161 → 136 (operator, same symptom) had already ruled out
interference. Capping `bitrate_max_kbps` at 11000: **air p50 98.7 →
1.1 ms**, fec p50 12.7 ms — ~95 ms off glass latency. The bench keeps the
11 M cap; the real fix is the airtime-budget policy (`airtime_budget`
0.60 is not being honored at the cap — the known open-loop overshoot).

**Gate record (agg 6 default, 14 M cap era):** ausniff 59.8 fps/0 gaps/0
incomplete; fec A/B flat (gate target ≤ 8 ms unmet — premise refuted, no
regression); aucadence +0.37 ms (baseline +2.5/+3.1 — improved); close_ms
5 → 11 ms constant (both far under the 50 ms bar); seq walk clean
(fid_gaps 0 throughout — host seqs survive aggregation live, closing
§11's renumbering caveat).

**End state:** both ends on the `ampdu` builds, aggregation at default
max_num 6 (no `ampdu` block in the deployed config), 11 M bitrate cap,
ch136. Setting `ampdu.max_num 0` (config-only) returns to singles with
full-rate RF sampling if the sparser (~6%-of-frames) RF feed proves
annoying. Follow-ups: USB feed rework is the real fec lever now
(overlapped/async bulk-OUT, bigger URB batches — the HalMAC 3-desc cap
is per *transfer*, so this means multiple transfers in flight, which
tx_threads already does… measure why acceptance serializes); devourer
`jgr3-physt` upstreaming; airtime-budget honoring at the cap.

## 13. Open questions

1. ~~What is the true split of the 7 ms `dq`?~~ Answered in §7.
2. ~~Is 1341 bodies/s a real ceiling?~~ Refuted in §8 — offered load.
   The burst-pace invariant is fully modeled in §9.
3. ~~What does the fragmentation + GF256 + SBI-pack stage cost per
   frame?~~ Measured in §7: mean 3.4 ms, max 16.6 ms (IDR) — and now
   overlapped with the drain, visible continuously in the dq_split line.
4. How much of an upstream millisecond reaches the glass under the vsync
   servo? §5 showed `reg` absorbing part of one; §7 showed the opposite
   sign is also possible (`reg` mean fell 2.6 ms alongside the upstream
   saving). Segment savings and `e2e` savings remain non-1:1 in both
   directions.

## Provenance

Captures kept out-of-tree in the session scratchpad; nothing in this doc
is derived from a source other than those runs and the 6729-window
`lat-00*.log` corpus. Rollbacks used and verified byte-identical after
restore: `/etc/mabur.json.pre-txsweep` (drone),
`/etc/maburgs.json.pre-mcs1meas` (GS). Both devices returned to their
pre-spike configuration.
