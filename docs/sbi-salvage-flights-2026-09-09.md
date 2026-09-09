# SBI sub-block salvage: first two flights (2026-09-09)

Flights 0043 (4 blocks per body) and 0044 (8 blocks per body, drone side
only — see the probe-gate caveat below). GS binary from `gs-keep-corrupted`
(PR #52), `rx.keep_corrupted` on, FCS-corrupt bodies delivered and their
CRC16-clean sub-blocks fed to the decoder. Read with
`tools/flightreport.py <session-dir>` (SALVAGE section) plus a per-record
diff of `link.streams[]` (`corrupt`, `salvaged`, `sub_fail`, `arr_expected`,
`arr_arrived`, `abandoned`, `abandoned_stale`), attributed to the rung at
the record's end. Bodies are `arr_expected / bpb`.

## Salvage fires, and hard

| flight | bpb | bodies | corrupt | corrupt % | salvaged sub-blocks | yield | pre-FEC miss |
|---|---|---|---|---|---|---|---|
| 0043 | 4 | 104 079 | 3 126 | 3.0 | 5 440 | 43.5 % | 1.60 % |
| 0044 | 8 | 175 624 | 3 538 | 2.0 | 16 147 | 57.0 % | 0.81 % |

yield = salvaged / (corrupt bodies x bpb): the share of a corrupt body's
sub-blocks that still pass their own CRC. 8-block bodies salvage a larger
share (a burst kills 1/8 of the body instead of 1/4), as SBI predicted, and
they get corrupted more often per body (they are twice as long): at the
low rungs 0044 saw 11-19 % of bodies corrupt against 5-7 % in 0043. The two
flights are different environments (0044 held rung 4 for 150 s), so only
the conditional yield is a geometry comparison.

Per rung (both streams pooled; miss = arr_expected − arr_arrived after
both cards and salvage; "avoided ≤" = salvaged / (miss + salvaged), the
UPPER bound on the share of pre-FEC misses salvage removed):

| flight | rung | bodies | corrupt % | yield | sub_fail | miss | avoided ≤ |
|---|---|---|---|---|---|---|---|
| 0043 | 0 | 8 490 | 6.6 | 53 % | 980 | 3 985 | 23 % |
| 0043 | 1 | 14 805 | 6.5 | 45 % | 2 078 | 882 | 66 % |
| 0043 | 2 | 14 646 | 2.7 | 44 % | 872 | 343 | 67 % |
| 0043 | 3 | 21 051 | 2.5 | 40 % | 1 255 | 853 | 49 % |
| 0043 | 4 | 45 089 | 1.5 | 36 % | 1 724 | 594 | 62 % |
| 0044 | 0 | 2 188 | 17.6 | 58 % | 1 246 | 175 | 91 % |
| 0044 | 1 | 6 583 | 12.7 | 55 % | 2 827 | 880 | 81 % |
| 0044 | 2 | 10 346 | 10.4 | 62 % | 3 159 | 963 | 85 % |
| 0044 | 3 | 15 657 | 4.4 | 57 % | 2 151 | 1 344 | 70 % |
| 0044 | 4 | 140 848 | 0.4 | 50 % | 2 197 | 7 989 | 22 % |

## What it bought is bounded, not measured

The counters cannot say whether a salvaged sub-block was *needed*. The
window dedups by seq (`SwDecoder::add_symbol`, arrival booked before the
dedup), and the other card usually delivers a clean copy of the same body.
The evidence that this shadowing is large: at 0043 rung 4, 1 724 sub-blocks
failed their CRC but only 594 symbols ended up missing — at least 65 % of
the failed sub-blocks were filled by the other card's clean copy, and the
salvaged sub-blocks come from the same bodies, so a similar share of them
was redundant. The honest range for rung 4 in 0043 is therefore roughly
"salvage removed between ~0 and 62 % of pre-FEC misses"; the true number is
probably in the lower half.

At rung 4 in 0044 (the rung that flew), miss (7 989) is far above
sub_fail (2 197): most loss there is whole bodies never received —
preamble kills / self-blanking ([[gs-uplink-self-blanking]]) — which salvage
cannot touch. Salvage matters at the low rungs during the range leg, where
corruption rates reach 10-19 % of bodies.

Post-FEC, salvage has nothing to show: current-window abandoned symbols
were 49 in 0044 and 3 010 in 0043, of which ~2 900 are one 6 s collapse at
t=166-172 s (rung 0, after the final s3_util demote at SNR 11 dB). Outside
total outages FEC already covers everything, so salvage's payoff can only
appear as lower `u` (fewer demotes, earlier promotes) and fewer FEC repair
waits. Neither is isolated by these two flights.

False accepts: CRC16 passes garbage at 2^-16; ~19 k failed sub-blocks over
both flights ⇒ ~0.3 expected false-accepted symbols. Negligible so far,
nonzero, and a false accept poisons every repair in its window.

## The instrument: `salvage_only` (built the same day, unflown)

Counts sources whose ONLY arrival was a salvaged sub-block from a corrupt
body — a second heard bit in the ArrivalTracker (`kClean`), booked at
settle time as `heard && !clean`. Per stream on the sideport as
`salvage_only` next to `salvaged`, in the exit summary line, and in
flightreport's SALVAGE section per stream and per rung. That turns the
upper bound into the number, and pairs with `miss` (arr_expected −
arr_arrived) to give the true avoided share: `salvage_only / (miss +
salvage_only)`. A UepDecoder-side "new to the window" count would NOT have
been enough: card order is random, so a corrupt copy that arrives first
looks new even when the clean copy follows. A salvaged *repair* symbol is
fed to the window but never counts here — it carries no seq of its own.
With one card there is no shadowing, so `salvage_only` should track the
salvaged source count; the two antennas of a card are combined before the
FCS verdict and never produce a second copy.

## Caveat: flight 0044's probe gate ran with the wrong geometry

`probe.log` header says `bpb=4` while every row carries `blocks_ok=8`:
the GS config stayed at 4 blocks per body while the drone sent 8.
`ProbeTrack` books `expected += cfg.bpb` per enh AU (gs/src/probe_track.cpp)
from the GS config, but `arrived` is the popcount of the body's real
bitmap, so probe loss read `1 − arrived/expected` with arrived up to twice
expected. Effect: a probe window only registered loss once more than half
its bodies were gone — 0044 shows loss on 11 % of probe-on records with a
p50 of 0.47 when it does, against 36 % / 0.22 in 0043 — so the promote
gate in 0044 was mostly blind, and flightreport's PROBE LOG `block_loss`
went negative. 0044's ladder behaviour (13 lossy edges vs 28, the
probation demote at t=207.7 s) is not an 8-bpb result. Fixes: ProbeTrack
should book expected from the body's own block count (wire), not from the
GS config, and the probe.log header/flightreport should take bpb from the
rows. Any future bpb A/B must change both ends until then.

## Flight 0046 (same day): the measured number

First flight with `salvage_only` (4 blocks per body, both ends). Same
method as above, plus `salvage_only` per rung; avoided =
salvage_only / (miss + salvage_only).

| stream | bodies | corrupt | salvaged | salvage_only | miss | avoided |
|---|---|---|---|---|---|---|
| 0 (base) | 169 432 | 4 302 | 7 081 | 471 | 4 803 | 8.9 % |
| 1 (enh) | 125 609 | 2 564 | 4 167 | 391 | 3 276 | 10.7 % |
| both | 295 040 | 2.3 % | 11 248 | 862 | 8 079 | **9.6 %** |

Per rung, avoided runs 26-34 % at rung 0, 15-17 % at rung 1 and 6-10 %
at rungs 2-4; `salvage_only` is 5-12 % of `salvaged` everywhere. So the
upper bound (58 % for this flight) was off by six times: about 92 % of
the salvaged sub-blocks were also delivered clean by the other card
(roughly half of the salvaged sub-blocks are repair symbols, which never
count; of the salvaged *sources*, ~86 % were shadowed). What salvage buys
is pre-FEC loss 0.68 % instead of 0.76 %, i.e. one in ten misses, for the
8 B/body of sub-block CRCs (~0.6 % of a 1.4 kB body). Post-FEC, current
abandoned symbols were 235 over the flight, 133 of them at rung 3.

Verdict: keep it (it is free at runtime and helps most exactly where the
link is worst, rung 0), but it is not a lever. Do not spend geometry on
it: the 8-block case's higher salvage yield cannot overcome a 92 %
shadowing rate, and whole-body loss remains the loss that matters.
