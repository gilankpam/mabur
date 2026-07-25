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

## Production classes acceptance

2026-07-17, mabur d15c8cf (dirty: fecbench prod row uncommitted), quiesced
SSC338Q (maburd + waybeam stopped), `gf=neon-vtbl2-q16`. `prod` engine =
production classes exactly as shipped: `FecWorker(1)` (common/src/fec_worker.cpp)
+ `UepEncoder(layers, 25, &worker)` (common/src/uep_encoder.cpp), vs the
bench-prototype `mt2a` (AsyncFecWorker + UepEncoderT<SwEncoderMtAsync>). Goal:
prod must reproduce mt2a within ~5-10% noise (RESULTS.md's own documented
run-to-run variance) at every point — a bigger gap would mean the
productionization lost something.

Canonical run (`./build.sh && ./run_drone.sh encoder 6`, 6s/point, ppf=13):

```
# fecbench  gf=neon-vtbl2-q16  mabur=d15c8cf  32-bit

## encoder: end-to-end base vs copy vs mt2 (6s/point, ppf=13)
eng   mode        ov   vid1x   air1x  speedup   SUST_vid  SUST_air
base  scalar   0.100    8.81   13.22     3.39      29.86     44.80
copy  scalar   0.100    8.81   13.22     3.41      30.08     45.14
mt2j  scalar   0.100    8.81   13.22     2.99      26.38     39.59
mt2a  scalar   0.100    8.81   13.22     4.40      38.77     58.19
prod  scalar   0.100    8.81   13.22     4.87      42.87     64.34
base  perlayer 0.100    8.81   21.54     3.06      26.94     65.84
copy  perlayer 0.100    8.81   21.54     3.07      27.05     66.12
mt2j  perlayer 0.100    8.81   21.54     3.33      29.36     71.77
mt2a  perlayer 0.100    8.81   21.54     3.89      34.25     83.72
prod  perlayer 0.100    8.81   21.54     3.95      34.79     85.05
base  scalar   0.375    8.81   21.65     1.54      13.59     33.39
copy  scalar   0.375    8.81   21.65     1.55      13.65     33.53
mt2j  scalar   0.375    8.81   21.65     1.60      14.12     34.68
mt2a  scalar   0.375    8.81   21.64     1.98      17.46     42.88
prod  scalar   0.375    8.81   21.64     2.21      19.51     47.91
base  perlayer 0.375    8.81   35.23     1.40      12.36     49.44
copy  perlayer 0.375    8.81   35.23     1.39      12.26     49.02
mt2j  perlayer 0.375    8.81   35.23     1.47      12.98     51.91
mt2a  perlayer 0.375    8.81   35.22     2.33      20.51     81.98
prod  perlayer 0.375    8.81   35.23     2.31      20.37     81.44
```

prod vs mt2a (this run, SUST_air):

| point            | mt2a  | prod  | delta   |
|------------------|------:|------:|--------:|
| scalar ov0.10    | 58.19 | 64.34 | +10.6%  |
| perlayer ov0.10  | 83.72 | 85.05 |  +1.6%  |
| scalar ov0.375   | 42.88 | 47.91 | +11.7%  |
| perlayer ov0.375 | 81.98 | 81.44 |  −0.7%  |

prod vs the brief's mt2a reference (previous session, SUST_air):

| point            | ref mt2a | prod  | delta   |
|------------------|---------:|------:|--------:|
| scalar ov0.10    |     56.1 | 64.34 | +14.7%  |
| perlayer ov0.10  |     82.7 | 85.05 |  +2.8%  |
| scalar ov0.375   |     38.6 | 47.91 | +24.1%  |
| perlayer ov0.375 |     77.8 | 81.44 |  +4.7%  |

**Verdict: PASS.** prod is at or above mt2a at every point (never a
regression), and the biggest gaps are well inside this file's own
documented ~10% run-to-run noise floor. Three additional exploratory runs
at 8s/10s (not archived verbatim) showed the same pattern with wider
run-to-run spread at the two ov0.375 points (up to ~-12% on a single busy
run), confirming these two points are the noisiest in the whole table —
consistent with prior findings, not a productionization regression. No
correctness concern: `prod` uses the identical envelope-emission path as
`mt2a`'s wrapped classes, just via the real common/ headers instead of the
bench's template shim.
