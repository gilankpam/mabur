# Drone self-calibration, second spike — one chain listens to the other

**Status:** the 8812EU can observe its own post-FEM transmit quality in
software; the drone's own wall measurement built on this record is
described in `docs/calibration.md`. It reverses the conclusion of
[2026-11-01-drone-self-calibration-spike](2026-11-01-drone-self-calibration-spike.md).
The spike tool and scripts were throwaway and are not merged.

Question, unchanged: can the drone find each MCS's TX-power wall with no
receiver? The first spike tried the chip's same-path loopback (DPK) and
found it blind to the external FEM. It did not try the other chain.

## Where this stands

- **Works:** one chain captures the other chain's transmission and an
  offline demodulator turns it into EVM per TXAGC index. A sweep of one
  PA on one channel takes about 6 s on the drone.
- **PA A is measurable on every channel and both widths**, once chain B's
  antenna switch is parked for receive: clean curves, floors near −30 dB,
  20 and 40 MHz within about one index of each other.
- **PA B is not reliable yet.** Its listener (chain A) is only partly
  understood: forcing GPIO4 high fixes channel 161 and breaks channel 112.
- **Against a receiver:** on channels 144 and 161 the MCS7 LDPC+STBC wall
  lies within 3.5 indices of where PA A's curve crosses −17 dB, and the
  20 MHz single-path MCS7 wall at −21 dB. Channel 112 sits 5–8 indices
  off, and 40 MHz single-path walls 10–16 indices above the prediction;
  neither is explained.
- **Read the sections in order as a log.** The per-PA numbers in
  "Results", "Other channels and 40 MHz" and "Receiver witness" were taken
  before the listener was fixed and are superseded by "Re-take with
  corrected listeners".

## Idea

The module is 2T2R. One path transmits normally through its FEM; the other
path's receiver, held in receive at a fixed low gain, picks the signal up
through antenna-to-antenna coupling. That signal has been through the PA,
so it carries the compression the loopback cannot see. The capture is
phydm's LA mode (devourer `LaCapture`): raw IQ from the listening path's
ADC, read back over USB and demodulated offline into an EVM figure per
TXAGC index.

## What the listening path needs

Listener = path A below; path B uses the mirrored registers (`0x4130`,
`0x410c`, RF path 1, debug port `0xa80`) with TX moved to path A
(`0x820[7:0] = 0x31`).

- **ADC on during TX.** The per-path AFE state table (`0x1830`, 16 entries
  written as `0x70<n>…`) turns the ADC off in every TX state. All entries
  are rewritten to the RX value (`0x70<n>F0001`), then `0x1c38` is written
  0 and `0xffffffff`.
- **Receiver detached from the baseband.** `0x180c[1:0] = 0` (3-wire off)
  and RF `0x00` written once to a static RX word. The RF mode table
  (`0x1800`, tx / txoff / rx nibbles) alone is not enough: the baseband
  still rewrites RF `0x00` at each TX and parks the receive gain at
  minimum.
- **A gain word per unit and per direction.** RF `0x00` bits `[7:6]` step
  about 4 dB, `[11:10]` about 9 dB in total, bit 12 about +17 dB, bit 8
  about +14 dB. The level must sit well above the ADC floor at the lowest
  index and below clipping at saturation.
- **The listener connected to its antenna.** Found late in the spike:
  chain B's antenna switch follows the TX/RX state and must be parked for
  receive while path B listens; chain A needs something similar that is
  only partly found. See "The listener's front end".
- **TX FIFO halved** (the vendor's LA mode: half the TX pages). The LA
  window is the top half of the TX buffer; without the split, queued
  frames land in the capture and TX wedges after a capture. With it, TX
  runs through capture and readback.
- **A QPSK probe frame** (MCS1, 150 bytes, single path, no STBC). PA
  distortion does not depend on the modulation, and a QPSK slicer stays
  valid to about −8 dB where a 64-QAM slicer saturates at −17 dB.

Demodulation: frame start from the envelope, channel from HT-LTF refined
on the data, decision-directed EVM over the 52 data tones of the first 20
symbols. TX and RX share the synthesizer and the sample clock, so there
is no frequency offset or phase drift to track.

## Capture facts

- On the 8822E the **high** dword of each LA word is the debug-port tap
  (`0x880` = path A, `0xa80` = path B; I = `[55:44]`, Q = `[43:32]`). The
  low dword is a later tap selected by the dma type, seven samples behind
  and zeroed while the baseband transmits. It is not a second RX path.
- The TXFF debug window accepts multi-byte vendor reads: 248 bytes per
  control transfer instead of 4. A capture of 8192 samples takes 6.3 s
  with 4-byte reads and 0.14 s with 248-byte reads on the host; 63 ms on
  the drone. Pipelining 4-byte reads gains nothing and starves bulk TX.
- **The dma type must match the debug port's path** (the vendor's ADC
  preset: dma 0 with `0x880`, dma 1 with `0xa80`). With dma 0 and `0xa80`
  — path B listening — the drone returns capture words with a wrong I
  value and Q intact, in bursts covering 2–6 % of samples while path A
  transmits; **bit 61 of the word is clear on exactly those samples**.
  Left alone they put a false floor near −17 dB with capture-to-capture
  scatter. With dma 1 the bursts are gone: 0.1–0.2 % of in-frame samples
  stay flagged, clustered at the end of each frame (the TX→RX
  transition), and a 20 Msps sweep agrees with the repaired one below
  within 0.3 dB from index 44 up. The LA clock edge (`0x1ce4[26]`) and
  the MAC-PHY timing field (`0x1ce4[7:6]`) change nothing. The host card
  shows no flagged sample, mismatched pairing included.
- **Repairing flagged samples.** Capturing at 40 Msps and replacing the I
  of each flagged sample with a least-squares prediction from eight
  neighbours on each side makes repeated captures agree within
  0.1–0.5 dB even with the mismatched dma type. With the matched type it
  is only needed for the floor: at index 28–40 the few remaining flagged
  samples still spread a −27 dB reading by 2–3 dB. What the bit means in
  the chip is not known.

## Results

All at channel 144, 20 MHz, flat TXAGC index, EVM in dB.

**Host EU card** (20 Msps, one capture per index; four repeat runs with a
64-QAM probe agreed within ±0.3 dB at index 60, 72 and 80):

| index | path B transmits | path A transmits |
|---|---|---|
| 40–56 | −27 (measurement floor) | −25 (floor) |
| 60 | −25.8 | −24.6 |
| 68 | −22.7 | −22.3 |
| 72 | −20.5 | −19.7 |
| 76 | −18.6 | −18.3 |
| 80 | −16.9 | −16.3 |
| 96 | −12.3 | −11.1 |
| 112+ | −10.7 | about −8 |

The received level rises 1.1 dB per RF code below index 64 and 0.35 dB
per code above 76. The first spike's air measurement on this card put the
MCS7 wall at index 76–80 and the RSSI knee at index 80; its DPK loopback
saw no compression below index 116.

**Bench drone** (anchor 39 on this channel; 40 Msps with the I repair,
three captures per index, 150 captures, no TX failure, no wedge; the
path A column was taken with the mismatched dma type and repaired):

| index | path B transmits | path A transmits |
|---|---|---|
| 28–40 | about −25 (noise-limited) | −27.7 (floor) |
| 44 | −22.2 | −26.6 |
| 52 | −19.2 | −23.9 |
| 60 | −16.8 | −20.3 |
| 64 | −15.8 | −18.4 |
| 68 | −15.0 | −16.5 |
| 72 | −14.4 | −15.0 |
| 80 | −13.2 | −12.7 |
| 96 | −11.6 | −9.8 |
| 124 | −12.2 | −7.5 |

The drone's stored walls (`rate_walls_rel_20 = [.., 47, 27, 25, 22]`,
calibrated against the GS with LDPC+STBC) are absolute index 86, 66, 64
and 61 for MCS4–7. On the path B curve those sit at −12.4, −15.4, −15.8
and −16.6 dB. Path B is the weaker PA in the wall region — path A reaches
−16.6 dB about eight indices later — so the receiver-measured walls
follow the weaker PA. The host card's MCS7 wall sits at −18.6 to −16.9 dB
on its own curve: the same threshold within 1–2 dB on two units whose PAs
compress about 23 indices apart.

The drone's ADC floor is about 7 dB higher than the host card's, so the
gain word that works on the host leaves the drone noise-limited with a
false flat floor; path B as a listener sees about 16 dB more than path A
at the same word.

## Other channels and 40 MHz

Bench drone, channels 40, 64, 112, 144 and 161, 20 and 40 MHz (primary
upper), both PAs: 24 sweeps of 150 captures, no TX failure, no wedge.
Each index is captured at three listener gain words and the highest level
is kept; where the level is adequate the gain words agree within
0.2–0.5 dB. The 40 MHz probe is MCS1/40, 320 bytes, captured at 40 Msps
and demodulated over the 108 data tones.

The table gives the index, relative to the channel's anchor, where each
curve crosses the EVM the channel 144 / 20 MHz walls sit at (−16.6,
−15.7, −15.2, −12.4 dB for MCS7, 6, 5, 4).

| ch (anchor) | PA | 20 MHz: MCS7 / 6 / 5 / 4 | 40 MHz: MCS7 / 6 / 5 / 4 |
|---|---|---|---|
| 40 (75) | B | 32.9 / 34.9 / 36.1 / 46.3 | 24.0 / 26.2 / 27.4 / 35.4 |
| 40 (75) | A | anomalous | anomalous |
| 64 (66) | B | 25.1 / 27.8 / 29.9 / 40.6 | 25.9 / 28.5 / 30.0 / 41.3 |
| 64 (66) | A | 17.8 / 25.9 / 27.8 / 36.5 | 24.9 / 27.2 / 28.7 / 36.8 |
| 112 (46) | B | 20.1 / 23.3 / 25.5 / 40.4 | 19.1 / 21.2 / 22.6 / 34.7 |
| 112 (46) | A | 32.7 / 34.9 / 36.3 / 44.6 | 33.6 / 35.8 / 37.1 / 44.5 |
| 144 (39) | B | 21.5 / 25.1 / 27.1 / 46.7 | 19.0 / 22.0 / 24.2 / 38.8 |
| 144 (39) | A | 28.9 / 31.1 / 32.4 / 42.8 | 29.6 / 31.7 / 32.8 / 41.4 |
| 161 (53) | B | anomalous | anomalous |
| 161 (53) | A | 24.1 / 25.9 / 26.8 / 37.1 | 23.3 / 25.5 / 26.6 / 35.5 |

- **20 MHz against the receiver-measured walls** (the calibration-revamp
  bench: channel 64 `15/20/20/37`, 112 `23/27/27/46`, 144 `21/25/28/47`
  for MCS7/6/5/4): taking the lower of the two PAs per row, the MCS7 row
  reads 17.8, 20.1 and 21.5 — within 3 indices of the receiver on all
  three channels. The other rows are within 6 except channel 64 MCS5/6
  (+6, +8), on the sweep with the poorest level and on the receiver run
  that reported `card_disagree`. The receiver method's own run-to-run
  spread was ±4.
- **40 MHz does not reproduce with the 20 MHz thresholds.** The curves at
  40 MHz are slightly worse than at 20 MHz (about 0.8 dB at the same
  index on channel 144), so the same thresholds are crossed earlier,
  while the receiver's 40 MHz walls are higher than its 20 MHz ones:
  channel 112 reads 19 against 28 for MCS7, channel 144 19 against
  24–25, channel 64 25 against 16. Whether the 40 MHz delivery threshold
  differs or one of the two measurements is off is open.
- **Which PA is weaker changes with the channel.** Path B crosses first on
  112 and 144 (by 7–14 indices), path A on 64 and 161.
- **Two combinations are anomalous at both widths.** Path A on channel 40
  and path B on channel 161 arrive 6–15 dB weaker at the listener than
  the same path on other channels, and their curves are not PA-shaped:
  path A on 40 is at −17 dB by index 44 (31 below the anchor) and
  non-monotonic between 76 and 100; path B on 161 stays near −17 dB from
  index 52 to 116 without saturating. Either that PA or antenna path is
  genuinely poor at that band edge, or the single-path probe is not what
  flies there. Not resolved; a receiver witness comparing single-path A
  and B frames on those channels would settle it.

## Receiver witness

The GS's second card ran devourer's `rxdemo` (per-frame events, `maburgs`
stopped) while the drone sent bursts at each index in normal TX
configuration: single-path MCS7 (BCC, no STBC) from each PA, single-path
QPSK, and MCS7 and MCS4 with LDPC+STBC from both. Delivery is frames
received with a good FCS over frames submitted. Channels 144 and 112 at
20 and 40 MHz, channels 40 and 161 at 20 MHz.

| | ch 144 / 20 | ch 112 / 20 | ch 144 / 40 | ch 112 / 40 |
|---|---|---|---|---|
| single-path MCS7, PA B: last ≥ 90 % .. first ≤ 10 % | 44 .. 48 | 48 .. 56 | never ≥ 90 % | 56 .. 64 |
| single-path MCS7, PA A | 48 .. 52 | 60 .. 64 | 60 .. 64 | 68 .. 72 |
| MCS7 LDPC+STBC | 52 .. 60 | 64 .. 76 | 60 .. 64 | 68 .. 76 |
| MCS4 LDPC+STBC | 76 .. 84 | 88 .. 92 | 80 .. 88 | 88 .. 96 |
| self-measured EVM of PA A / PA B at the MCS7 STBC wall | −22.2 / −17.9 | −20.7 / −15.5 | −19.8 / −15.5 | −20.6 / −14.3 |
| … at the MCS4 STBC wall | −12.8 / −13.1 | −12.6 / −11.8 | −11.4 / −11.5 | −11.8 / −10.3 |

- **The PA ranking is confirmed.** Single-path frames from PA A survive 4
  indices higher than PA B's on channel 144 and 10 higher on 112; the
  self-measured curves put A 7 and 13 indices later.
- **The STBC wall does not follow the weaker PA.** It sits above both
  single-path walls, and on this bench the receiver hears PA A 5–12 dB
  louder than PA B. Read on PA A's curve, the MCS7 wall is at −20.8 dB
  within ±1.2 dB and the MCS4 wall at −12.2 dB within ±0.7 dB across both
  channels and both widths; read on PA B's curve the same walls spread
  over 3.6 dB. The 40 MHz mismatch reported above was the weaker-PA rule
  plus the scatter of the older receiver numbers, not the width.
- **Capturing 40 MHz at 80 Msps instead of 40 Msps changes nothing**
  (within 0.5 dB at every index, both PAs): the 40 MHz curves are not
  inflated by aliasing.
- **The two anomalous curves are measurement artefacts, not PA
  behaviour.** On channel 40 single-path MCS7 from PA A is still delivered
  at index 76, where the self-measured curve reads −10 dB; on channel 161
  PA B's single-path frames stop at index 56, where the self-measured
  curve shows no compression at all. In both cases the listener received
  the other path 6–15 dB weaker than on other channels, so something
  other than the PA's radiated output dominates what it captures. A sweep
  needs a validity check (level and a PA-shaped curve) before its numbers
  are used.
- **The witness itself is impaired on channels 40 and 161**: delivery
  tops out at 35–40 % for every mode and every index, including MCS4
  LDPC+STBC at low power. Cause not investigated.
- Today's MCS7 LDPC+STBC walls relative to the anchor (mid-point): +17
  and +24 at 20 MHz, +23 and +26 at 40 MHz on channels 144 and 112. The
  stored calibration has 22 / 23 and 25 / 28.

## The listener's front end — why two combinations were wrong

The listener was not connected to its antenna. On this module chain B's
antenna transfer switch (the DPDT on GPIO13) follows the baseband's TX/RX
state, and during a frame the whole chip is in TX state, so with path B
listening the switch sits in its TX position and the capture is whatever
crosses the open switch. That is 17–32 dB below an antenna connection and
varies with frequency — weakest on channel 40 — and it is not a clean copy
of the PA's output: on channel 40 it produced the anomalous curve, and on
channel 144 it read PA A's distortion about 3.4 dB too good.

- **PA A, fixed.** devourer's static `bit24` switch route
  (`DEVOURER_DPDT_MODE=bit24`: `0x4c[24]` and `[22]` set, no eFEM pin
  claim) parks the switch for receive; PA A still transmits normally. The
  listener then runs at its minimum gain word, and channel 40 gives a
  PA-shaped curve (floor −33 dB, −26.6 at index 76, −17.3 at 92, −9.1 at
  124; it read −10 dB at 76 before).
- **PA B, improved.** PA B needs the switch in its TX position, so `bit24`
  is not usable there (its own frames come out at about −10 dB EVM).
  Forcing GPIO4 high as a plain output (`0x42[1]` and `0x40[1:0]` cleared,
  `0x46[4]` and `0x45[4]` set) raises the path A listener's level by 10 dB
  on channel 161 and by nothing on channel 144, and channel 161 then gives
  a PA-shaped curve (floor −27 dB, −18.7 at index 60, −5.7 at 124) where
  it showed no compression before. On channel 144 the same lever moves the
  top of the curve from −12 dB to −4.8 dB at index 124: the default
  listener was also diluted there. What GPIO4 switches on chain A is not
  identified.
- **The baseband RFE selects do nothing here**: constants and inversions
  on pins 3, 5, 9 and 11 (`0x1840`/`0x1844`/`0x4140`/`0x4144`,
  `0x1850`/`0x4150`) leave the listener's level unchanged.
- **Against the witness, with the corrected listeners.** Self-measured
  EVM at the single-path MCS7 delivery wall: channel 144 PA A −21.4,
  PA B −20.1; channel 161 PA A −21.0, PA B −21.3; channel 112 PA A −17.9
  (PA B not re-taken). At the MCS7 LDPC+STBC wall both PAs now read alike:
  −18.7 / −16.7 on channel 144 and −17.7 / −16.5 on channel 161 (PA A /
  PA B), and about −11 dB at the MCS4 wall.
- **What this invalidates above.** Every path A column in "Results",
  "Other channels and 40 MHz" and "Receiver witness" was taken through the
  open switch and reads PA A too good by an amount that depends on the
  channel; the path B columns are diluted at the top of the curve. The PA
  ranking ("A crosses 7–14 indices later") and the per-PA wall thresholds
  in the witness section are artefacts of that: with the switch parked,
  PA A and PA B cross the stored MCS7 threshold at index 61.3 and 60.5 on
  channel 144. The capture recipe, the timing and the validity of the
  method are unaffected.
- The gain pick needs headroom: with an antenna-connected listener the
  highest non-clipping gain word can overdrive the listener and add its
  own distortion (channel 40, index 52: −21.8 dB at one word, −31 dB a
  step lower).

## Re-take with corrected listeners

Bench drone, five channels, both widths, both PAs: 20 sweeps of 200
captures (25 indices, four listener gain words, two captures each), no
TX failure, no wedge. PA A: static `bit24` switch route, listener B gain
words `0x3000e`/`0x300c0`/`0x30800`/`0x30c00`. PA B: eFEM route with
GPIO4 forced high, listener A gain words
`0x31400`/`0x310c0`/`0x30dc0`/`0x31cc0`. The gain word kept per index is
the strongest one whose level stays under 57 dB.

Index, relative to the anchor, where each curve crosses −21, −17 and
−11 dB (the single-path MCS7 wall, the MCS7 LDPC+STBC wall and the MCS4
LDPC+STBC wall on the channels where the witness agrees):

| ch (anchor) | PA A, 20 MHz | PA A, 40 MHz | PA B, 20 MHz | PA B, 40 MHz |
|---|---|---|---|---|
| 40 (75) | 9.7 / 18.1 / 39.5 | 10.8 / 18.6 / 37.6 | 24.0 / 33.2 / — | 24.7 / 33.0 / — |
| 64 (66) | 9.1 / 17.3 / 37.9 | 10.0 / 18.3 / 36.4 | 10.7 / 20.4 / 40.9 | 12.1 / 22.3 / 39.9 |
| 112 (46) | 8.0 / 18.5 / 42.4 | 7.7 / 17.7 / 40.1 | invalid | invalid |
| 144 (39) | 11.2 / 20.5 / 44.3 | 12.8 / 21.4 / 40.4 | 5.7 / 16.6 / 42.0 | 5.1 / 15.6 / 36.3 |
| 161 (53) | 5.3 / 15.4 / 41.8 | 6.9 / 15.0 / 36.8 | 2.1 / 12.1 / 32.2 | 2.0 / 12.3 / 30.5 |

Correction: the channel 161 PA A 20 MHz −11 dB crossing first read 28.8.
One of the two captures at index 84 held no probe frame and the script took
a "frame" demodulated out of its noise (level 12 dB, EVM −2.9 dB) into the
median; with frames under 30 dB level dropped the crossing is 41.8, in line
with the other channels. No other cell changes.

- **PA A** is clean everywhere: floors −30 to −34 dB, −7 to −9 dB at
  index 124. Relative to the anchor the −17 dB crossing stays within
  15–21 on all five channels although the absolute index runs from 59 to
  93: the efuse anchor carries most of the channel dependence. The two
  widths agree within about one index.
- **PA B** is valid on channels 144, 161 and 64 (floors −26 to −28 dB at
  20 MHz). On channel 112 the floor is −21 dB at 20 MHz and −17.7 dB at
  40 MHz — worse than the default listener gave there — and on channel 40
  the curve stops at −11.8 dB. GPIO4 is not the whole of chain A's front
  end.
- **Against the witness, PA A.** MCS7 LDPC+STBC wall mid-point minus the
  −17 dB crossing, in indices: channel 144 −3.5 at 20 MHz and +1.6 at
  40 MHz, channel 161 −2.4, channel 112 +5.5 and +8.3. Single-path MCS7
  wall against the −21 dB crossing at 20 MHz: channel 144 inside the wall
  bracket (50.2 in 48..52), channel 161 on it (58.3 against about 58),
  channel 112 8 indices early. At 40 MHz the single-path walls are 10
  (channel 144) and 16 (channel 112) indices above the −21 dB crossing.
  The stored channel 144 walls (22 at 20 MHz, 25 at 40 MHz for MCS7) sit
  1.5 and 3.6 indices above the −17 dB crossing.

## Run-time checks for an in-process build

Taken the day after the sweeps, bench drone unless noted, to see whether
the measurement can run inside a process that started in the normal
flying configuration (full-size TX buffer, eFEM switch route).

- **The TX buffer can be re-laid live.** Re-running the queue page
  configuration (`priority_queue_cfg`, half-size layout) on a drained
  queue is not enough on its own: bulk-OUT then times out, and it stays
  dead after the full layout is put back. Pulsing `REG_CR` bit 2
  (TXDMA enable) off and on immediately before the queue configuration
  makes it work: no send failure, captures clean, and 1400-byte MCS7
  frames go out at the same rate before, in the half layout, and after the
  restore (6109 / 6098 frames in 1.5 s before / after on the drone).
  Holding the bit off across the configuration leaves the free-page count
  at 0. The whole `init_trx_cfg` (MAC TRX off and on) also works but
  resets the EDCA, SIFS and CCA-gate registers (`0x500`–`0x574`) to their
  hardware values and costs 37 % of the TX rate until they are rewritten.
  After a half-then-full round trip the MAC pages read back unchanged
  apart from counters (host card).
- **The switch can be parked at run time.** The eFEM and `bit24` routes
  differ in two bits on this module: `0x4c[22]` and `0x3c[26]`. Setting
  the first and clearing the second in a running eFEM-route process gives
  PA A's reference curve within 0.3 dB at five of six indices (1.3 dB at
  the first, taken on a cooler PA); before parking the same captures read
  5.5 dB lower in level and up to 2.4 dB off. Writing the two bits back
  restores the register values.
- **A retune inside the process keeps the listener.** After `FastRetune`
  to channel 40 the AFE table, the 3-wire detach and the TX path select
  are still in place; only the listener's RF gain word is lost, which the
  sweep rewrites per capture anyway. Channel 40 then reads within 0.4 dB
  of its reference at every index, and channel 144 within 0.3 dB after
  retuning back. A process started at 40 MHz and switched to 20 and back
  with `FastSetBandwidth` reads within 0.8 dB of both references.
- **The listener can be undone** with the driver's own IQK AFE restore
  values (`iqk_afe_restore`), `0x1c38 = 0xffa1005e`, the 3-wire bits and
  the TX path select: those registers then read as before. Whether
  reception is unimpaired afterwards was not tested against a ground
  station.
- **Step the index gradually.** A 16-index jump followed 18 ms later by a
  capture read up to 3.3 dB worse at 40 MHz (index 44 to 60: −13.9 dB
  against −17.2 dB); stepping by 4 gives −16.5 to −17.2 dB over 24
  captures, with the switch parked at run time or at process start alike.
  About one capture in twenty holds no clean frame.

## Time

A 25-index sweep with three captures per index takes about 6 s for one
PA on the drone at 20 Msps (9.3 s wall clock including chip bring-up and
copying the 75 captures off the drone). The 40 Msps captures with repair
are twice the samples and were not timed. A PA measured cool (fast sweep) reads
about 1.2 dB better at index 72–84 than the same PA after minutes of
near-continuous TX — about one RF code.

## For a live loop

Measured on the host card only: with the FIFO split, TX keeps running
through a capture and its readback at 88 % of the maximum frame rate with
no send failure. EVM from fewer symbols reads optimistic because the
data-refined channel estimate overfits: −0.3 dB at ten symbols, −1 dB at
five, −2 dB at three. Not built or tested: a capture aligned to a single
probe frame, STBC video decoding at the GS while one path is parked as a
listener, the uplink on the parked path, a flight soak.

## Open

- The meaning of bit 61, and why only the drone shows flagged samples.
  The practical cause of the bursts — a dma type that does not match the
  debug port — is settled.
- EVM thresholds per MCS rest on one drone's stored walls and the first
  spike's host-card air data, not on a delivery sweep taken the same day.
- Chain A's front end: what GPIO4 switches, and a listener state that
  measures PA B on every channel (112 and 40 fail today).
- Channel 112: the receiver's walls sit 5–8 indices above PA A's curve.
- 40 MHz single-path walls 10–16 indices above the 20 MHz threshold
  while the curves at the two widths coincide.
- A validity test for a sweep (floor, level, a curve that keeps degrading
  to saturation). Reception after the listener is undone.
- How two PAs with different curves combine under STBC. On the bench the
  wall follows the PA the receiver hears louder; in flight that changes
  with geometry.
- Why the receiver witness tops out at 35–40 % delivery on channels 40
  and 161.
- The efficiency table stays a receiver-side quantity.

## Dead ends

- The RF mode table without the 3-wire detach and the AFE table: the ADC
  bus reads zero during TX.
- The vendor RX-spur recipe bits (`0x1864[31]`, `0x180c[27]`,
  `0x186c[7]`): they route the ADC to the calibration engine and zero the
  LA tap.
- Outlier-based despiking without the flag bit at 40 Msps: it repairs 11 %
  of samples and barely moves the EVM.
- The raw TSSI detector was not read; the cross-path capture made it
  unnecessary.
