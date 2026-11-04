# Does a busy secondary half cost a 40-tuned receiver its 20 MHz frames? — 2026-11-04

**Status:** YES, and not as a dB desense: a 40 MHz-tuned 8812EU does not
decode a 20 MHz frame on its primary while the secondary half is busy, so
delivery tracks the secondary's idle airtime (≈2 % under a 95 % jammer,
≈50 % under a 32 % one) at every jammer power. The same card tuned 20 MHz
on the same frames at the same moment is unaffected until the jammer is
strong enough to leak through the adjacent-channel skirt. Reference page:
`docs/bw40.md` "What flies" / "Not measured".

## Question

Both ends tune `radio.width = 40`; the shipped ladder airs every rung at
20 MHz, and control, telemetry and MAVLink are always 20 MHz on the
primary. `docs/findings/2026-09-23-bw40-sweep.md` priced the 40 MHz tuning
at 0.2–1 dB of sensitivity on 20 MHz frames and left "secondary-channel
interference desensing a 40-tuned RX" unmeasured. Measure it: with a
transmitter on the secondary half, how much of the drone's 20 MHz traffic
does a 40-tuned GS card lose, against a 20-tuned card on the same frames?

## Setup

- Pair 132+136, primary 136, secondary 132 (the dwell spike's pair).
- Wanted signal: the drone, `linkbench-tx --channel 136 --bw 20 --ldpc
  --stbc --no-cca --range-sweep --range-lo -64 --range-hi 41 --range-step
  2 --range-frames 40 --range-mcs-mask 0x3f --range-bw-mask 1` (maburd
  stopped): 20 MHz MCS0–5 singles, 1400 B, 40 per cell, TX power from
  absolute index 0 (rel −64 under anchor 63) to +41, ~38 s per cycle,
  running continuously. `--no-cca` so the wanted transmitter cannot defer
  against the jammer.
- Receivers: the GS's two 8812EU cards on the same frames simultaneously,
  card 0 `linkbench-rx --index 0 --channel 136 --bw 40 --range`, card 1
  `--index 1 --channel 136 --bw 20 --range`, 150 s per arm (3–4 complete
  cycles after `rangean.py` drops the first); maburgs/maburplay stopped.
- Jammer: the host's 8812EU, ~1 m from the GS, `linkbench-tx --bw 20
  --mcs 0 --size 22 --no-cca --foreign-sa --pwr-mode override --pwr P`:
  1408 B MCS0 singles at `--bitrate 5M` (6.1 Mb/s air ≈ 95 % airtime) or
  `1M` (≈32 %). Power index 63 = max, 0 ≈ 16 dB below it. Arms: none;
  secondary at 63 / 40 / 20 / 0; secondary at 0 and 63 light; primary at
  63 as the control; none again.
- Bench geometry is too strong for a sensitivity floor: at the drone's
  index 0 the GS reads −64 dBm (40-tuned) / −61 dBm (20-tuned) and every
  rung delivers 100 %. The sweep therefore spans −64…−47 dBm of wanted
  signal, and the measurement is the jammer-to-signal ratio, not the
  noise floor. Delivery below is averaged over 5 TX-power bands (rel
  −64..−50, −48..−34, −32..−18, −16..−2, 0..40; the RSSI noted is the band's
  mean at the 40-tuned card). `tools/bench/rangean.py` and a scratch band
  tabulator produced the tables.

## Result — GS cards, 20 MHz frames, jammer on the SECONDARY (132)

Delivery % at the weakest band → strongest band, MCS0 / MCS3 / MCS5:

| jammer | card tuned 40 (pair 132+136) | card tuned 20 (136) |
|---|---|---|
| none, before | 100 / 100 / 100 (every band) | 99.4 / 99.1 / 99.1 |
| 95 % air, pwr 63 | **0.9→6.9** / 2.0→3.2 / 2.1→2.3 | 4.1→73.6 / 2.0→61.8 / 2.1→18.8 |
| 95 % air, pwr 40 | **0.6→7.2** / 2.2→4.1 / 2.3→2.2 | 10.9→86.4 / 10.2→77.2 / 3.7→73.5 |
| 95 % air, pwr 20 | **2.8→46.0** / 3.4→34.7 / 2.5→8.1 | **89.6→90.0** / 88.3→79.9 / 85.6→86.9 |
| 95 % air, pwr 0 | **6.0→79.3** / 3.0→59.1 / 2.3→14.0 | **94.7→94.6** / 95.4→85.2 / 93.5→91.9 |
| 32 % air, pwr 0 | **56.2→82.3** / 44.5→57.9 / 46.8→53.8 | **96.5→97.4** / 95.6→89.8 / 96.8→95.7 |
| 32 % air, pwr 63 | 12.6→56.2 / 40.4→42.8 / 46.4→46.8 | 49.2→87.5 / 40.2→79.3 / 44.9→61.9 |
| none, after | 99.9 / 100 / 99.9 | 99.6 / 99.5 / 99.3 |

(MCS5's top band is the PA wall on both cards — 0–1 % in every arm
including the clean ones — and is excluded from the reading.)

Control, jammer on the **PRIMARY** (136), 95 % air, pwr 63: card tuned 40
**25.4 / 61.8 / 58.4**, card tuned 20 **25.4 / 61.6 / 58.4** — identical in
every band, so the two cards are equivalent receivers and everything above
is the tuning. (Higher MCS surviving a co-channel jammer better than MCS0
is the shorter frame: a 1400 B frame is 1.8 ms at MCS0 and 0.35 ms at MCS5.)

Reading:

1. **A 40-tuned card is blind to its primary while the secondary is busy.**
   Under the 95 % jammer the 40-tuned card delivers ~2 % — the jammer's
   idle gaps — at pwr 63, 40 and 20 alike, and under the 32 % jammer
   45–56 %. Delivery follows the secondary's airtime, not the jammer's
   power: it is a packet-detect / CCA gate, not a noise-floor shift. RSSI
   of the frames that do get through is unchanged.
2. **It recovers only by capture.** At pwr 0 (jammer ~16 dB down) the
   40-tuned card climbs from 6 % at −58 dBm wanted signal to 79 % at
   −47 dBm on MCS0, and less on higher MCS: the primary is decoded when
   the wanted frame is within roughly 10 dB of the secondary's energy.
3. **The 20-tuned card on the same frames is untouched** by a secondary
   jammer at pwr 20 and 0 (88 % / 94 % flat across the signal range;
   96–97 % under the light jammer). Its losses at pwr 63 and 40 are the
   adjacent-channel leakage of a transmitter 1 m away (the dwell spike's
   20 MHz card read a strong neighbour as fully busy energy too) and scale
   with power, the opposite signature of the 40-tuned card.
4. Nothing drifted: the clean arms before and after agree within 0.5 pt.

## Result — the drone's radio tuned 40 (partial)

Mirrored: a GS card as the wanted transmitter (same range sweep on 136 at
20 MHz), the drone running `linkbench-rx --channel 136 --bw 40 --range`,
maburd stopped. Two arms ran before the operator stopped the drone side:

| jammer | drone tuned 40, MCS0 / MCS3 / MCS5, weakest → strongest band |
|---|---|
| none | 99.7→99.8 / 90.6→99.0 / 93.0→91.9 |
| secondary, 95 % air, pwr 63 | **12.8→68.3** / 6.8→24.6 / 2.8→15.4 |

The drone's 8812EU (RFE 21, external FEM) shows the same gate: 3–13 % at
the weak end, rising by capture at the strong end. Its clean-air 90–96 %
on MCS3–5 (MCS0–2 at 99.7–100 %) is a baseline the GS cards do not have;
whether that is the 40 MHz tuning or the drone's radio in general was not
measured (the 20-tuned drone arms did not run). The drone stays tuned 40.

## What this means for the link

- With `radio.width = 40` and the all-20 ladder, a neighbour on the
  **secondary** half costs the GS its airtime in video — up to everything —
  while the primary is clean, and costs the drone the same in RCFs. A
  20-tuned receiver would lose nothing until the neighbour is strong enough
  to leak through the filter skirt.
- The boot scan sees it: a 40 MHz dwell's NHM busy % covers both halves
  (`docs/findings/2026-11-03-ht40-dwell-secondary.md`), so a dirty secondary
  ranks the pair down. In flight the scout dwells on primaries and the
  verdict window's NHM busy % is the only secondary view; the loss it
  produces looks like co-channel interference and the hop machinery treats
  it as such, but nothing attributes it to the half that is dirty.
- The 0.2–1 dB figure in `docs/bw40.md` is the clean-air cost only. The
  trade for staying 40-tuned is the September range test's flat 4–10 %
  weak-signal HT20 loss of a 20-tuned receiver (`2026-09-23-bw40-sweep.md`
  §4, cause never chased) against this secondary exposure; neither page
  decides it, that is a design question for its own brainstorm.

## Not measured

- The drone tuned 20 (and the drone's clean-air MCS3–5 shortfall at 40).
- Jammer-to-signal ratios beyond the bench's: the wanted signal never went
  below −64 dBm, the jammer never below ~16 dB under its maximum.
- A bursty (duty-cycled) interferer; a 40 MHz interferer; the GS's own TX
  deferral (CCA) on secondary energy, which this receive-only bench cannot
  show.
- Whether the gate is the chip's 40 MHz packet detect, its secondary CCA
  (`0x2c10[31:16]` moved only on secondary traffic in the dwell spike) or
  AGC; a register-level look in devourer would say.
