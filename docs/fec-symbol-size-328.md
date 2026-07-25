# FEC geometry: production flip to scalar-328/w32/bpb4 (2026-07-25)

**Status:** SHIPPED — deployed to the rig as production and set as the repo
bundle default (`bundle/mabur.default.json`, `gs/bundle/maburgs.default.json`)
on 2026-07-25. Previous production geometry was scalar-164/w64/bpb8.
Rollback configs are preserved on both devices as `/etc/*.pre-sym328`.

## The trade being tuned

For a fixed air-body byte budget, `symbol_size × blocks_per_body` slides
between two taxes. Each symbol pays ~26 B of fixed overhead (16 B SW envelope
header + 2 B SBI sub-block CRC outside; 2 B length + 6 B wide-FRAG header
inside), so useful-byte efficiency ≈ `(s−8)/(s+18)`: ~86% at 164, ~93% at 328,
~98% at 1400. Pulling the other way, smaller symbols buy finer erasure
granularity (more equations per latency-bounded window span → smoother FEC
behavior), sub-body corruption salvage, less frame-tail padding at the
frame-aligned `SwEncoder::flush()`, and lower per-loss burst quantization.
The sweep question: where is the knee?

## Evidence

### 1. linkbench RS sweep (same day, MCS5/10M/pwr48/ov0.25, constant ~2.7 kB body + ~21 kB window span)

| symbol/bpb/window | body | app/air efficiency | raw body loss | residual pkt loss |
|---|---|---|---|---|
| 82/28/192 | 2807 B | 64.3% | 3.02% | 0.20% |
| 164/16/128 | 2919 B | 70.9% | 2.53% | 0.00% |
| 328/8/64 | 2775 B | 74.6% | 2.97% | 0.03% |
| 656/4/32 | 2703 B | 76.6% | 2.92% | 0.09% |
| 1312/2/16 | 2667 B | 77.6% | 3.81% | 0.07% |
| 2624/1/8 | 2649 B | 78.2% | 3.42% | 0.10% |

Knee at 328–656. Hard side-findings:
- **Air bodies > 2900 B hit an RX cliff**: an 82/32 config (body 3207 B) lost
  61.6% of frames (vanishing whole, SNR normal); the same symbol size at
  2807 B ran at 3.0%. `kMaxBodyBytes = 2900` (`common/include/mabur/sbi.h`)
  is empirically confirmed as a hard limit, not a soft margin.
- **Zero sub-block CRC failures across ~640k sub-blocks**: on this link bodies
  are lost whole, never delivered corrupted. SBI's salvage margin is currently
  pure insurance.
- Drone TX is per-packet-cost bound (~14 k pps): the 82 B point could not
  sustain 10 Mbps offered.

This matches the earlier CPU-side encbench symsweep (2026-07-22): SUST_vid
17.8 → 24.6 → 32.0 → 23.7 Mbps along 164/64 → 328/32 → 656/16 → 1312/8
(same-span), i.e. encode cost is dominated by window ROW count, with 1312
losing to tail-padding air inflation. 328/w32 was its "conservative
+38%, near-free-air" pick.

### 2. On-air video A/B (frame-shm path, mcs5/ov0.25, 8.2 Mbps, 60 s arms)

| Metric | 164/w64/bpb8 (old prod) | 328/w32/bpb4 |
|---|---|---|
| fps / seq gaps / truncations | 59.5 / 0 / 0 | 59.5 / 0 / 0 |
| GS frames clean/trunc/drop | 3697/0/0 | 3638/0/0 |
| maburd CPU (63 s wall) | 67.0 s (~104% core) | 62.0 s (~97%) — **−7.5%** |
| drone total busy (dual A7) | 68.0% | 66.4% |

### 3. Robustness gates at 328 (both PASS)

- **IDR burst**: 20 forced IDRs at 1/s (`waybeam :80/request/idr`) — 59.4 fps,
  0 gaps, 0 truncations; drone `txq_drop`/`tx_failed`/`full_drops`/`oversize`
  all 0 throughout.
- **Marginal-link probe** (GS `link.static_overhead` 0.25→0.375→0.5): 328
  holds **full 59.5 fps / 8.2 Mbps at ov 0.5 with ring fill 18%**, where 164
  on the same probe (2026-07-23) throttled to 55.9 fps / 7.8 Mbps at
  fill=100%. The ~5% airtime saving converts directly into margin headroom —
  under squeeze, 328 degrades later than 164.

## What shipped

- Rig `/etc/mabur.json` (drone): `fec.symbol_size` 328, `fec.window` 32,
  `fec.blocks_per_body` [4,4,4,4]. Rig `/etc/maburgs.json` (GS):
  `fec.symbol_size` 328. MSP OSD side-channel (1312/w16) untouched.
- Repo bundle defaults updated to the same values (pinned by
  `tests/test_config.cpp`); `tests/integration/run_host_e2e.sh` decoder args
  updated to match. Note the bundles previously shipped a per-layer
  `[164,1312,1312,1312]`/bpb`[4,1,1,1]` geometry that predated the
  scalar-164 production decision and matched neither the rig nor the
  encbench/on-air findings (1312 loses to tail padding).

Geometry is pure both-ends config — no wire-format or negotiation change.
Any change must be applied to **both** devices and needs a maburd →
maburgs restart order (GS `peer_caps`/session refresh).

## Rollback

Restore `/etc/mabur.json.pre-sym328` (drone) and
`/etc/maburgs.json.pre-sym328` (GS), restart maburd then maburgs. Expect
164-era behavior (identical delivered quality at the nominal operating point,
~7.5% more maburd CPU, earlier throttle onset under airtime squeeze).

## Open follow-ups

- 656/w16 offers a further +2 pts air efficiency and (per encbench) +80%
  encode headroom, at 0.09% residual in the RS sweep — candidate for a later
  A/B at higher operating bitrates; body-burst tolerance quantizes coarser
  (2 bodies/window at ov 0.25), so it needs its own marginal-link gate.
- Longer soak at 328 under real flight RF (desk-only so far).
