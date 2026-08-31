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

## 8. Open questions

1. ~~What is the true split of the 7 ms `dq`?~~ Answered above.
2. Is 1341 bodies/s a real acceptance ceiling, or just today's offered
   load? Unresolved — §4. Everything upstream of the radio (CPU, USB
   threading, USB round-trips) has been eliminated, so if a ceiling
   exists it is in devourer/HalMAC/the chip FIFO.
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
