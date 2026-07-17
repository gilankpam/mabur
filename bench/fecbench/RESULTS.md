# SW-FEC candidate A/B — SSC338Q, quiesced (maburd AND waybeam stopped)

2026-07-17, mabur b65378d, `gf=neon-vtbl2-q16`. Baseline = shipped common/
code unmodified. All candidates verified byte-exact (envelope-set equality)
before every run. Run-to-run variance observed ~10% — single-run deltas
below that are noise.

## Winner table: us/repair (lower is better), best drone run per candidate

| candidate            | 164 w64 | 164 w128 | 1312 w64 | 1312 w128 |
|----------------------|--------:|---------:|---------:|----------:|
| baseline/deque       |    43.9 |     88.2 |    197.8 |     400.8 |
| flat-ring            |    46.4 |     91.1 |    204.0 |     399.3 |
| fused32+pf           |    42.8 |     83.1 |    206.2 |     419.5 |
| fused64              |    40.4 |     79.4 |    207.0 |     445.5 |
| fused64+pf           |    42.9 |     84.7 |    209.5 |     422.4 |
| mt2-byte/fused64     |    62.8 |     95.9 |    166.6 |     325.1 |
| mt2-row/fused64      |    55.8 |     80.3 |    163.8 |     297.5 |
| mt2-row/lincomb      |    60.1 |     86.3 |    140.9 |     243.7 |
| **mt2-row/spin**     |**31.6** | **59.1** |    186.2*|     261.8 |

(*1312 w64 spin number inconsistent with its w128/164 siblings — likely a
noisy run; re-measure before concluding.)

## Findings

1. **mt2-row/spin is the overall winner**: +39% at the PRODUCTION geometry
   (164×w64: 31.6 vs 43.9 µs), +51% at 164×w128, +53% at 1312×w128.
   Row-split (each core folds half the window rows, worker into a private
   scratch accumulator, one XOR merge) + atomic spin handoff instead of
   mutex/condvar. The condvar version lost at small geometries purely on
   ~2×futex wake latency per repair (~40 µs) — sync mechanism, not split
   shape, was the limiter.
2. **Row-split beats byte-split** consistently (~10% at big geometry):
   halving per-thread SOURCE traffic matters more than avoiding the scratch
   accumulator + merge.
3. **Fused two-source kernel (lincomb2): small win at ss=164 (+9%), LOSS at
   ss=1312 (−5..−11%)**. At 164 it amortizes per-call overhead; at 1312 the
   extra register pressure (4 nibble-table q-regs + 2 source streams)
   degrades the in-order A7 pipeline. Not worth pursuing for the current
   geometry mix — the mt winners use plain gf::lincomb.
4. **Flat aligned ring alone: no effect** (within noise of deque storage).
   The deque's scattered vectors were NOT a measurable cost; contiguity only
   matters as the enabler for row-split threading.
5. **Quiescing waybeam lifted baseline ~25%** (401 vs 521 µs at 1312×w128)
   — earlier numbers taken with waybeam alive systematically understate
   capacity; all comparisons here are same-run, same-conditions.
6. Production caveat for the spin handoff: the worker burns core 1 while
   idle. A production SwEncoder integration needs bounded spin-then-sleep
   (spin ~50 µs, then futex), or repair batching so the worker sleeps
   between credit bursts. The spin row measures the sync-latency ceiling.

## End-to-end: the wired encoder (mt_encoder.h, same session)

The repair-table winner was wired into the FULL encode pipeline
(UepEncoderT: mabur's Fragmenter/SbiPacker/classify_rtp unmodified, only the
sliding-window engine swapped). Three shapes were measured on the quiesced
drone; the journey is the finding:

| shape | scalar ov.375 SUST_air | perlayer ov.375 | verdict |
|---|---:|---:|---|
| base / copy (replica sanity) | 32.8-33.3 | 52.8-53.1 | copy ≡ base ✓ |
| mt2j fork-join, acquire-spin  | 29.3 | 44.0 | −10..−22%: ARMv7 acquire load = LDR+DMB; idle spinner's barrier storm slows the other core |
| mt2j + relaxed spin + pin + spin-then-sleep | 34.2 | 48.6 | parity: repairs arrive every 0.3-1 ms >> spin budget, futex wake eats the split gain |
| **mt2a async pipeline** | **38.6 (+18%)** | **77.8 (+47%)** | hot thread never waits: enqueue window-coords job, worker builds whole envelope, delivered at next drain |

Full mt2a table (6s/point, 800 MHz pinned, worker on cpu1):

```
eng   mode        ov   SUST_air   vs base
mt2a  scalar   0.100      56.1     +23%
mt2a  perlayer 0.100      82.7     +16%
mt2a  scalar   0.375      38.6     +18%
mt2a  perlayer 0.375      77.8     +47%
```

Correctness: every envelope byte-identical to stock SwEncoder (verified as
sorted-set equality over a 3000-packet feed with interleaved flushes — only
emission ORDER relaxes: repairs trail their sources by a beat, which
SwDecoder tolerates by design). The ring carries 64 slack rows so in-flight
jobs never see overwritten window rows; a join() backstop guards the bound.
flush() joins the worker, preserving tail-repair and idle-flush semantics.

Production-relevant constraints learned (must survive into any maburd
integration):
- Spin loops on ARMv7 must use relaxed loads + one acquire fence; never
  acquire-in-loop.
- The worker must sleep when idle (bounded spin ~50-100 us, then futex);
  a permanent spinner degrades the whole SoC.
- Fork-join per repair is the WRONG shape at real repair cadence; the
  async queue is the right one.
- Gain is largest exactly where the drain ceiling binds: heavy-FEC ops
  (+47% at perlayer ov0.375).
- Measurement artifact: ~0.1% of air bytes in flight at run end are
  uncounted for mt2a (repairs pending at the final poll) — negligible.

## Context: what this buys end-to-end

Encoder end-to-end (baseline, quiesced): SUST_air 34.8 Mbps at scalar-164
ov0.375. Repair generation is one component of encode cost (fragment/seal/
SBI-pack are untouched by these candidates), so end-to-end gain will be
smaller than the repair-table gain; integrating the winner into a bench-side
UepEncoder variant is the next measurement before touching mabur.

## Raw data

Three drone runs archived in this directory's git history / session log:
- run 1 (waybeam alive): baseline-only tables in BASELINE.md
- run 2 (quiesced): 7-candidate ladder
- run 3 (quiesced): + mt2-row/lincomb, mt2-row/spin
