# fecbench baseline — mabur b65378d, 2026-07-17

Baseline = shipped `common/` code, unmodified. Every future candidate row
(fused kernels, flat-ring storage, multi-repair batching, worker threads)
is judged against these tables on the same hardware.

## SSC338Q (drone, `gf=neon-vtbl2-q16`, 32-bit, maburd stopped, waybeam still running)

```
## kernel: L1-hot lincomb MB/s (single reused symbol)
candidate                    ss=164    ss=1312
baseline/gf::lincomb          280.2      478.7

## repair: make_repair-shaped throughput (distinct heap symbols)
candidate                geometry       repairs/s      srcMB/s    us/repair
baseline/make_repair     164x w64           19500        204.7         51.3
baseline/make_repair     164x w128          10039        210.7         99.6
baseline/make_repair     1312x w64           4112        345.2        243.2
baseline/make_repair     1312x w128          2035        341.7        491.5

## encoder: UepEncoder end-to-end (baseline only; 6s/point)
mode        ov   ppf   vid1x   air1x  speedup   SUST_vid  SUST_air
scalar   0.100    13    8.81   13.22     2.94      25.93     38.91
perlayer 0.100    13    8.81   21.54     2.15      18.95     46.31
scalar   0.375    13    8.81   21.65     1.35      11.92     29.29
perlayer 0.375    13    8.81   35.23     1.25      10.99     43.96
```

## Host (x86, `gf=scalar` — sanity/relative only, no NEON path)

```
## kernel: L1-hot lincomb MB/s
baseline/gf::lincomb         2742.1     2889.1

## repair
baseline/make_repair     164x w64          244251       2563.7          4.1
baseline/make_repair     164x w128         122728       2576.3          8.1
baseline/make_repair     1312x w64          32729       2748.2         30.6
baseline/make_repair     1312x w128         16554       2780.0         60.4

## encoder (4s/point)
scalar   0.100    13    8.81   13.22    59.63     525.39    788.43
perlayer 0.100    13    8.81   21.54    37.10     326.87    799.01
scalar   0.375    13    8.81   21.65    22.48     198.03    486.52
perlayer 0.375    13    8.81   35.23    13.64     120.17    480.50
```

## Reading the drone numbers

1. **Window streaming costs ~25–30% vs L1-hot**: 280 → 205 MB/s at ss=164,
   479 → 342 MB/s at ss=1312. That gap (memory traffic, scattered heap
   symbols, accumulator re-read/re-write per source) is exactly what the
   flat-ring + fused-kernel candidates attack.
2. **Big symbols are already cheaper per byte** (345 vs 205 MB/s): per-call
   overhead amortizes. A repair at 1312×w128 costs 491 µs — at production
   repair rates this is the single largest CPU line item.
3. **SUST_air falls as overhead rises** (scalar: 38.9 → 29.3 Mbps from
   ov 0.10 → 0.375): confirms the FEC-parity-work component of the ceiling.
4. **Calibration**: flat-out single-thread encode sustains 29–46 Mbps air —
   ~3–4× above the ~9–10 Mbps rig ceiling. So the observed platform ceiling
   is NOT raw single-core encode alone; it is encode sharing 2×A7 with
   waybeam/venc/ISP, USB writer, RX path and kernel SDK threads. FEC is the
   largest tractable CPU consumer, so cutting its cost still buys drain
   headroom — but a 2× FEC speedup does not mean a 2× ceiling raise.
   (Caveat: this bench ran with maburd stopped but waybeam alive, so ambient
   contention partially pollutes even these numbers — treat as slightly
   pessimistic single-core capacity.)

## Reproduce

```
./build.sh          # host + arm binaries (SHA-stamped)
./fecbench-host all # host sanity
./run_drone.sh all 6  # SSC338Q (stops maburd, restarts after)
```
