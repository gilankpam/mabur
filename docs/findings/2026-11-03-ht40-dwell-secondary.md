# Can a 40 MHz dwell score the whole pair? — 2026-11-03

**Status:** YES with NHM busy %. The 2026-09-23 "a 40 MHz dwell is nearly
blind to the secondary" verdict (docs/findings/2026-09-23-bw40-sweep.md
"Scouting a 40 MHz candidate") was measured on FA/CCA/foreign only, before
the busy-airtime NHM existed. NHM busy % on a 40-tuned card reads
secondary-only traffic within 0.5 points of a 20 MHz dwell on that half.
The scout still dwells per half (`docs/bw40.md` "Boot scan scores every
half"); switching it to one 40 MHz dwell per pair is a follow-up, not built.

## Question

`radio.width = 40` costs the boot scout two 20 MHz dwells per pair, and the
in-flight scout keeps 40 MHz tuning and so (per the September record) sees
only the primary. Both rest on one measurement from 2026-09-23 that
predates NHM. Re-ask it with every counter the card has: FA, CCA, decoded
foreign frames, the raw page-0x2c PHY counters (looking for a
secondary-channel CCA count), and the busy-airtime NHM window.

## Setup

- GS card 0: `linkbench-rx --index 0 --channel 136 --bw 40` (standard pair
  132+136, primary 136, secondary 132). GS card 1: `--channel 132 --bw 20`,
  the per-half reference. Both run the spike's `--dwell-ms 200` mode
  (one `D` line per 200 ms: raw `0x2c04..0x2c18` read before the scout
  reset, `GetRxEnergyScout` FA/CCA, own/foreign deltas, a 190 ms busy-NHM
  window armed at the previous tick). maburgs/maburplay stopped.
- Interferer: this PC's 8812EU, host `linkbench-tx --no-cca --foreign-sa`,
  MCS0, ~1 m from the GS. Phases of 25 s: idle; 132 at 1 Mb/s app
  (light, ~32 % air) and 5 Mb/s (heavy, ~95 %; the host tool tops out at
  3 Mb/s app / 6 Mb/s air); the same on 136; 132 heavy at `--pwr 0`;
  a 40 MHz HT40 jam on 136 at 10 Mb/s; idle again.
- Per phase: the first 6 s dropped (host TX bring-up), medians over the
  valid NHM windows (n = 80–117 per cell).

## Result — busy-NHM (`busy83`, % of the window at ≥ −83 dBm)

| interferer | card 0, 40 MHz on 136 (sec 132) | card 1, 20 MHz on 132 |
|---|---|---|
| idle | 0.0 (max 0.0) | 0.0 (max 0.0) |
| secondary 132, light | **32.4** (31.1–37.5) | 31.9 (31.1–32.3) |
| secondary 132, heavy | **95.3** (94.9–97.2) | 94.9 (94.5–96.1) |
| primary 136, light | 31.8 (30.8–32.2) | 31.6 (leakage; see below) |
| primary 136, heavy | 95.3 (94.9–96.5) | 95.2 (leakage) |
| secondary 132, heavy, `--pwr 0` | **95.7** (94.9–97.2) | 94.9 |
| HT40 on 136, heavy | 91.4 | 91.7 |

Every valid window in a heavy phase read > 50 % (`blocked_pct`) on the
40 MHz card for secondary-only traffic — 113 of 113; every light-phase
window read > 5 %; every idle window read 0.0. The 40 MHz card's idle
noise sits one NHM bucket higher than the 20 MHz card's (twice the
bandwidth), still two buckets under the −83 dBm edge, so the idle reading
keeps its zero margin at 40.

## Result — the other counters (per 200 ms, medians)

| interferer | card 0 (40 MHz): fa / cca / foreign / `0x2c10[31:16]` | card 1 (20 MHz on 132): fa / cca / foreign |
|---|---|---|
| idle | 19 / 19 / 0 / 0 | 0 / 0 / 0 |
| sec light | 8 / 24 / **0** / 10 | 0 / 39 / 39 |
| sec heavy | 3 / 96 / **0** / 77 | 0 / 117 / 117 |
| pri light | 2 / 42 / 39 / 0 | 136 / 136 / 0 |
| pri heavy | 0 / 118 / 117 / 0 | 595 / 598 / 0 |
| sec heavy `--pwr 0` | 15 / 58 / **0** / 29 | 0 / 117 / 117 |
| HT40 heavy | 0 / 229 / 228 / 0 | 0 / 228 / 0 |

- The September verdict reproduces for FA/CCA/foreign: secondary frames
  are never decoded (`foreign` 0), CCA moves ~5x on heavy secondary traffic
  against ~6x for primary, and ~2x only at `--pwr 0`. The ranker's score
  terms (`fa + max(cca − own − leak, 0) + 4·foreign`) stay primary-biased.
- `0x2c10[31:16]` (and its mirror `0x2c18[31:16]`) count only on
  secondary traffic and `0x2c10[15:0]` only on primary — a per-half
  CCA-like counter. It is not a frame count (10 of 39 light, 77 of 117
  heavy, 29 of 117 at low power) and `0x2c0c` stays 0, so the vendor
  `cnt_bw_usc/lsc` mapping was not confirmed from source; not needed given
  NHM.
- The 20 MHz card on 132 reads a 136 interferer as 136–598 FA/CCA with
  zero decoded frames and the same NHM busy % as the 40 MHz card: the
  adjacent-channel leakage from the September record, now with NHM
  showing a strong neighbour as fully "busy" energy on a 20 MHz dwell
  too. One metre from the interferer is the worst case; a per-half dwell
  cannot tell that neighbour from in-channel traffic by NHM alone either.
- An HT40 jam reads 91 % busy on both widths; the 20 MHz card cannot
  decode it (`foreign` 0, CCA 228).

## What this means

- **One 40 MHz dwell per pair can replace the two 20 MHz half dwells**
  for the blocked decision and for NHM-based ranking: NHM busy % on a
  40-tuned card is the max-of-halves (energy over the whole 40 MHz
  passband) that `docs/bw40.md` "worse half" already wants. It cannot say
  which half is busy; the pair rule does not need to know.
- **In-flight dwells at 40 MHz already carry NHM busy %** in their `D`
  lines, so the "a busy secondary on the op pair is invisible to the
  scout" caveat in `docs/bw40.md` holds for the FA/CCA/foreign score, not
  for the NHM blocked tier.
- The secondary stays undecodable at 40 MHz, so anything keyed on
  `foreign` (the heaviest score term) remains primary-only.

## Not measured

Weak secondary interferers (the host card at `--pwr 0` a metre away is
still strong); the retune cost of one 40 MHz dwell per pair vs two
20 MHz (`docs/findings/2026-09-23-bw40-sweep.md` "Retune cost at 40 MHz"
has the 40 MHz numbers); 2.4 GHz; the 8822C. One GS unit, one channel pair.

## Tool

`linkbench-rx --dwell-ms N` (this record's bench mode; it downcasts to
`RtlJaguar3Device::DebugPeekBb` for the raw counters). The NHM period
register caps at 262 ms: a 500 ms dwell overflowed it and every bucket
read 255 (the first run, discarded).
