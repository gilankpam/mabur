# 2026-11-25 — wide-TX IGI floor on the bench 8812EU drone: fixed value or self-calibration

**Status:** spike; the outcome is the IGI floor watchdog, described in
[`docs/deploy.md`](../deploy.md) ("wide-TX IGI floor"). The bench tooling is
on the local `igi-floor-spike` branches of devourer and mabur (throwaway, not
for merge).

## Question

devourer's `DeviceConfig::Tuning::wide_tx_igi_floor` raises the receiver's
lowest gain index (IGI) once the session has sent a 40 MHz frame, because at
low IGI on a quiet channel a Jaguar3 transmitter can hold 40 MHz frames
(devourer `docs/experiments/8822c-ht40-tx-stall.md`). Its default is 0x35;
`maburd` forces 0 on 8812EU images. The ask was a self-calibration at every
`maburd` start. This spike measured whether the bench drone needs a floor,
where its edge is, whether the drone can see the fault alone, and what a
floor costs the uplink.

## Setup

- Drone: the bench WiFiLink 2 (8812EU, USB high speed), `maburd` stopped,
  devourer `txdemo` with spike knobs: MCS2/40/LDPC, 1400-byte QoS-data
  frames, RX thread on, `no_cancel_multipkt`, carrier sense on.
- GS: `maburgs` stopped. One GS 8812EU card
  as a receive-only witness, the other as a traffic player (HT MCS0/20,
  100-byte frames).
- Channels, scanned from the GS card at 40 MHz before the runs: 144(-) and
  100(+) silent (0 frames in 10 s), 157(+) nearly silent (one burst of 11
  frames in 10 s), 112(-) carries a non-802.11 emitter (false alarms
  ~2800/s, no frames) and was not used.
- "Saturated" = the drone sends with no gap, so frames/s is the transmitter's
  capacity. The ceiling is ~2790 frames/s. The witness count matched the
  drone's own count in the runs where it was on.
- IGI was set two ways: the floor itself with DIG running (what `maburd`
  does), and a raw pin with DIG off, which also reaches values under the DIG
  window (0x1e..0x22; 0x20 fixed on DFS channels).

## 1. Does this unit stall at 40 MHz with floor 0?

It never stops. It loses capacity, sharply, below an edge.

Floor set at construction, DIG on, saturated, 10 s per cell:

| floor | 144(-) IGI | frames/s | 157(+) IGI | frames/s |
|---|---|---|---|---|
| 0 | 0x20 | 2739 (−1.9 %) | 0x1e | 2620 (−6.1 %) |
| 0x20 | 0x20 | 2751 (−1.4 %) | 0x20 | 2791 |
| 0x22 | 0x22 | 2790 | 0x22 | 2790 |
| 0x24, 0x28, 0x2c, 0x30, 0x35, 0x36 | = floor | 2791–2792 | = floor | 2790–2792 |

Pinned IGI, DIG off, saturated, silent 144(-), 3–5 s per step:

| IGI | frames/s | of ceiling |
|---|---|---|
| 0x35, 0x2c, 0x24, 0x23, 0x22 | 2786–2815 | 100 % |
| 0x21 | 2690–2790 | 97–100 % |
| 0x20 | 2670–2790 | 96–100 % |
| 0x1f | 950–1870 | 34–67 % |
| 0x1e | 930–2230 | 33–80 % |
| 0x1c | 960–2370 | 34–85 % |
| 0x18, 0x10, 0x08 | 1980–2430 | 71–87 % |

At a gap-limited 790 frames/s the unit kept sending at every IGI down to
0x04. Raising IGI back restores the full rate within the 1 s resolution.
`DEVOURER_DIS_CCA=1` gives the same table.

**20 MHz is not affected.** 20 MHz frames, saturated, read 1462–1472
frames/s at every pinned IGI from 0x08 to 0x24, on a 20 MHz tune and on the
40 MHz tune.

## 2. Can the drone see it by itself?

Yes. Pinned IGI on silent 144(-), sampled every 200 ms for 10 s per cell.
"FIFO empty" = PUB free pages (MAC 0x240, bits 27:16) at their idle value
1745.

| offered load | IGI | sent | FIFO empty in | lowest free pages | longest blocked send |
|---|---|---|---|---|---|
| 790 frames/s (28 % air) | 0x22, 0x20 | 100 % | 100 % of samples | 1745 | 5–8 ms |
| 790 frames/s | 0x1e | 94 % | 12 % | 17 | 124 ms |
| 1980 frames/s (70 % air) | 0x22 | 100 % | 100 % | 1713 | 0 ms |
| 1980 frames/s | 0x20 | 100 % | 91 % | 1281 | 0 ms |
| 1980 frames/s | 0x1e | 65 % | 2 % | 17 | 375 ms |

Both the free-page count and the time a send stays blocked separate the two
states without anything from the GS.

## 3. How much does heard traffic mask it?

Pinned IGI 0x1e on silent 144(-), drone saturated, the GS card playing
MCS0/20 frames:

| GS frames/s | drone frames/s | of ceiling |
|---|---|---|
| 0 | 1270, 1392 | 45–50 % |
| 5 | 1262 | 45 % |
| 20 | 1949 | 70 % |
| 50 | 2327 | 83 % |
| 80 | 2484 | 89 % |
| 200 | 2651 | 95 % |

In flight the GS sends an RCF every `link.feedback_ms` (bundle 50 ms, so 20
frames/s); a searching GS sends DISC every 20 ms while it dwells on the
channel. A floor that is too low therefore shows as a partly reduced,
irregular TX rate while the GS talks and as the full loss when the uplink
goes quiet.

## 4. Does the edge drift?

Pinned IGI, saturated, 3 s per step. "Cold" = right after a power cycle
(4 minutes off); "hot" = after 10 minutes of TX at 70 % air on 144(-) with
floor 0 (mean 1993 frames/s, no blocked send over 11 ms).

| IGI | 144 cold | 144 hot | 157 hot | 100 hot |
|---|---|---|---|---|
| 0x24, 0x22 | 100 % | 100 % | 100 % | 100 % |
| 0x21 | 97 % | 98 % | 100 % | 100 % |
| 0x20 | 98 % | 99 % | 100 % | 100 % |
| 0x1f | 41 % | 47 % | 94 % | 97 % |
| 0x1e | 45 % | 40 % | 95 % | 97 % |
| 0x1c | 56 % | 44 % | 35 % | 54 % |

The edge did not move with this heating, but the chip's thermal meter only
went from 21 to 22–23 (baseline 22): the encoder was off, so this is not
flight heat. The edge does move with the channel: 144 collapses at 0x1f, 157
and 100 lose ~5 % at 0x1e–0x1f and collapse at 0x1c.

## 5. What does the floor cost the uplink?

The GS card sent 90 MCS0/20 frames/s; the drone (transmitting 190 40 MHz
frames/s) counted them per floor. The bench could only lower the GS power
to an RSSI of 44 at the drone (three levels: 62, 47, 44), where every floor
up to 0x35 still received 99–100 %. Walking the floor further up found a
one-step cliff from ~99 % to 0 %:

| RSSI at the drone | last floor that receives | first floor that is deaf |
|---|---|---|
| 62 | 0x51 | 0x52 |
| 47 | 0x45 | 0x46 |
| 44 | 0x43 | 0x44 |

So the receiver goes deaf to a signal once IGI exceeds its RSSI by about 19
to 23, and the cliff moves 14 IGI steps for 18 dB of signal (~1.3 dB per
step). Extended down, 0x35 is deaf below an RSSI of roughly 28–30, 0x2c
below ~20, 0x22–0x24 below ~9–11. The weakest frames either bench card
decoded at IGI 0x1e–0x20 were at RSSI 16 with 13–15 dB SNR.

That bounds the cost without settling it: 0x35 costs at least 12 dB of
uplink (it would not hear those RSSI-16 frames) and about 20 dB if the
receiver's own MCS0 limit sits near RSSI 8; 0x22–0x24 costs between nothing
and ~4 dB. RSSI is in the chip's own units, not calibrated dBm.

## 6. Is a fast sweep feasible?

In silence, yes. Pinned IGI cycled in a fixed pattern, saturated, four
visits per value:

| dwell | 0x24, 0x22 | 0x20 | 0x1f | 0x1e | 0x1c |
|---|---|---|---|---|---|
| 250 ms | 99.6–100.4 % | 97.0–99.8 % | 0–52 % | 11–42 % | 19–63 % |
| 500 ms | 99.4–100.6 % | 95.9–99.0 % | 27–56 % | 14–59 % | 38–53 % |
| 1000 ms | 99.7–100.2 % | 98.4–99.1 % | 45–64 % | 36–56 % | 47–72 % |
| 500 ms, GS playing 50 frames/s | 99.4–100.6 % | 98.7–98.9 % | 90–98 % | 87–91 % | 76–91 % |

A 250 ms dwell gives the same edge as the slow walk. With the GS beaconing,
values under the edge read 76–98 %, and one 0x1f dwell (97.7 %) came within
1 point of 0x20. The response is not monotonic under the edge (0x18 is
better than 0x1c), so a search must compare against the session's own
ceiling, not look for a zero.

A sweep needs a runtime floor setter in devourer (the floor is
construction-time today); the spike added one, and changed
`set_igi_floor` so that any lowering parks IGI at once.

## 7. With the GS talking, at a video-like load

A later run the same day, after a power cycle. Pinned IGI on silent 144(-),
the drone at a fixed offered load instead of saturated, the GS card playing
MCS0/20 frames; 10 s per cell, sampled every 200 ms. "Worst 200 ms" is the
fewest frames sent in one sample, as a share of the offered load.

790 frames/s offered (28 % air, about a video stream):

| IGI | GS frames/s | sent | worst 200 ms | FIFO empty in | lowest free pages | longest blocked send |
|---|---|---|---|---|---|---|
| 0x22, 0x20 | 0, 20, 50 | 100 % | 95–100 % | 100 % of samples | 1745 | 0–9 ms |
| 0x1e | 0 (three runs) | 94, 98, 100 % | 15, 68, 91 % | 12, 14, 45 % | 17, 245, 1197 | 124, 0, 10 ms |
| 0x1e | 20 (two runs) | 100 % | 96–98 % | 35, 39 % | 1017, 1077 | 0–9 ms |
| 0x1e | 50 | 100 % | 99 % | 79 % | 1473 | 0 ms |
| 0x1c | 20 (two runs) | 100 % | 96–97 % | 45, 58 % | 1289, 1329 | 0–9 ms |

1980 frames/s offered (70 % air):

| IGI | GS frames/s | sent | worst 200 ms | FIFO empty in | lowest free pages | longest blocked send |
|---|---|---|---|---|---|---|
| 0x22 | 0, 20, 50 | 100 % | 92–99 % | 100 % | 1713 | 0 ms |
| 0x20 | 0, 20, 50 | 100 % | 96–99 % | 95–97 % | 1569–1689 | 0 ms |
| 0x1e | 0 | 60 % | 8 % | 0 % | 17 | 154 ms |
| 0x1e | 20 | 95 % | 48 % | 10 % | 17 | 21 ms |
| 0x1e | 50 | 100 % | 97 % | 33 % | 717 | 0 ms |
| 0x1c | 0 | 52 % | 0 % | 0 % | 17 | 263 ms |
| 0x1c | 20 | 93 % | 60 % | 4 % | 17 | 33 ms |

- The drone-local signal survives the uplink. With the GS at the RCF rate the
  FIFO is empty in 100 % of samples at 0x20 and above at the video-like load
  and in 35–58 % below the edge; at the DISC rate, 75–79 %.
- With the GS at the RCF rate, a video-like load loses no frames below the
  edge and no send blocks. What remains is a standing backlog: up to ~730
  pages, about 60 frames of 1400 B (128-byte pages), which at 790 frames/s
  is roughly 75 ms of queueing in the worst sample and 30–40 frames in the
  worst tenth. That delay is computed from the page count, not measured.
- At 70 % air the RCF rate still costs 5–7 % of the frames and halves the
  worst 200 ms.
- With the uplink silent the loss at a video-like load varies from run to
  run (94–100 % sent, worst 200 ms between 15 % and 91 %).

## 8. The real link: `maburd` video, A-MPDU, real RCFs

`maburd` (master plus a spike hook that sets the floor or pins IGI at run
time and samples the chip queue every 100 ms) against `maburgs` with the link
pinned to MCS2 at 40 MHz on one channel: 60 fps video, A-MPDU on (up to 6),
NACK on, RCFs every 50 ms, the session's air use 33-58 %. Each cell is three
windows of about 40 s, alternated A-B-A-B-A-B inside one session; the first
seconds after each switch are dropped. "Arrival" is frame complete at the GS
minus the capture stamp, relative to the session's fastest frame
(`au.log`). Ranges are across the three windows.

| | 144(-), gain control on | | 161(-), gain control on | | 144(-), IGI pinned | |
|---|---|---|---|---|---|---|
| | floor 0 | floor 0x22 | floor 0 | floor 0x22 | 0x1e | 0x22 |
| IGI the drone sat at | 0x20 | 0x22 | 0x1e | 0x22 | 0x1e | 0x22 |
| frames/s delivered (of 60) | 60.0 | 59.7-60.0 | 38.5-58.5 | 59.8-60.0 | 45.8-58.4 | 59.7-60.0 |
| arrival p50, ms | 3.0-5.1 | 1.5-3.6 | 40-55 | 5.0-6.2 | 34-46 | 1.6-5.8 |
| arrival p99, ms | 9.2-15.2 | 7.0-10.0 | 176-294 | 11.4-11.9 | 142-249 | 7.6-12.1 |
| longest blocked USB send, ms | 2.4-7.3 | 3.3-4.3 | 102-131 | 2.7, 55, 1.9 | 68-116 | 2.4-3.6 |
| longest wait in `maburd`'s TX queue, ms | 15.7-18.0 | 17.8-23.1 | 162-225 | 14.4-16.0 | 38-242 | 18.3-29.6 |
| `txq_drop` | 0 | 0 | 0, 0, 74 | 0 | 64, 0, 0 | 0 |
| chip queue empty in (100 ms samples) | 43-53 % | 44-53 % | 9-14 % | 51-62 % | 11-14 % | 53-61 % |
| lowest free pages | 1073-1441 | 1225-1436 | 15 | 1417-1452 | 15 | 1132-1546 |
| RCFs the drone accepted per s | 19.6-19.7 | 19.6-19.7 | 18.6-18.8 | 19.5-19.7 | 18.7-18.9 | 19.6 |
| NACK symbols requested per window | 0 | 0-181 | 2144-4413 | 0-135 | 1481-3438 | 12-49 |

- On a non-DFS channel the shipped configuration (gain control on, floor 0)
  lets IGI settle at 0x1e, and at 40 MHz the video suffers badly even though
  the drone hears 19 RCFs a second: frames are shed on the drone, the median
  frame arrives 40-55 ms late and the 99th percentile 180-290 ms late, and
  USB sends block for over 100 ms. Floor 0x22 in the same session is clean
  in every window.
- On DFS 144 the gain control holds 0x20, one step above this unit's edge
  there, and floor 0 and floor 0x22 cannot be told apart.
- Pinning 0x1e on 144 reproduces the 161 result, so the cause is the IGI
  value, not the channel's other traffic (161 had some: the GS cards counted
  about 5000 CRC failures in the session against 1 on 144).
- Almost no frame is lost on the air (pre-FEC loss stays under 1.5 %): the
  damage is delay and the drone shedding frames it cannot send in time. The
  NACK requests in the bad windows were almost never filled.
- The uplink is not worse at 0x22 at bench signal strength (RSSI far above
  any floor): the drone accepts slightly more RCFs than at 0x1e.
- The stand-in runs of sections 3 and 7 understated this. With single frames
  and the RCF-rate player, 28 % air lost nothing at 0x1e; the real A-MPDU
  video at 33-58 % air does.
- A healthy real link reads the chip queue empty in only 43-62 % of samples,
  because frames in flight count. A watchdog on this signal has to compare
  against a full queue (free pages at 15) or long blocked sends, not against
  "always empty". `maburd`'s existing send timing shows the fault without any
  register read: blocked sends over 60 ms and TX-queue waits over 35 ms
  appear only in the bad cells, bar one 55 ms send just after a switch.
- The switches themselves, read from the 1 s chip-queue lines: raising the
  floor to 0x22 clears a full queue within the next second, six times out of
  six. Going down on 161 the gain control passes through 0x20 for about 2 s
  and the queue is already full there (15-17 free pages, both linked
  switches), so on 161 the edge is above 0x20 and 0x22 has one or two steps
  of margin, not three. 0x21 was not held anywhere.
- With the GS silent the drone goes to failsafe after 3 s and sheds to about
  230 frames/s; at that load the chip queue is empty at either setting. The
  exposure with a quiet uplink is those 3 s.
- 144(-) and 161(-), both with the primary on the upper half, show the high
  edge; 157(+) and 100(+) showed the low one (section 4). Two channels each;
  not tested further. The GS's shipped channel set is all upper-primary.

## 9. Does raising the floor free a stuck 8812CU?

One 8812CU USB adapter (the unit devourer's record lists as clean at 0x24)
on a PC, devourer `txdemo` saturated at MCS3/40 on silent 144(-), a GS card
as the receive-only witness, the floor switched at run time every 5 s.
Frames accepted per 500 ms sample; full rate is about 1780.

| floor | accepted per 500 ms | longest blocked send |
|---|---|---|
| 0x20 | 0 in every sample | 5.3 s (the whole dwell) |
| 0x26 | 1767, 1314, 1770, 1790, ... | 9 ms |
| 0x20 | 1569, then 0 in every sample | 4.7 s |
| 0x26 | 1201, 1783, 1782, ... | 218 ms |
| 0x20 | 469, 328, 0, 115, 0, 0, 1240, 0, 0, 0 | 1.6 s |
| 0x24 | 1754, 1780, 1770, ... | 8 ms |
| 0x20 | 276, 0, 1263, 0, 189, 266, 0, 715, 791, 120 | 1.0 s |
| 0x35 | 1775, 1780, 1031, 1776, ... | 8 ms |

- A raise frees it at once: all four times the first 500 ms sample after the
  raise is at or near full rate, including after a send had been blocked for
  5.3 s. The witness shows the frames on the air.
- With a fixed floor of 0x22 the same adapter does not stop for good here:
  it stalls for up to 3 s at a time and resumes by itself. A fixed 0x35 ran
  14 s clean.
- One adapter, the easy one. The 8812CU that needs 0x35 was not here.

## What this means for the four options

- **Fixed 0x35 on the 8812EU too:** no boot time, covers every unit measured
  so far, costs at least 12 dB of uplink and plausibly ~20.
- **Self-calibration at boot:** works only on a silent channel. It needs
  ~2–3 s of full-duty 40 MHz transmission before the link is up, a searching
  GS blurs the result, and the edge it finds is for that channel only (144
  and 157 differ by three steps on this unit).
- **Runtime watchdog** (raise the floor when the drone's own TX FIFO stops
  draining): the signal exists and needs no GS; it follows the channel and
  the temperature; it costs uplink only on a unit that needs it. The signal
  holds with the GS sending at the RCF rate (section 7). Not built; a busy
  channel also backs the FIFO up, so it needs a cap.
- **One-time calibration stored in config:** no boot time, but it needs a
  value per channel or a margin over the worst one, and an operator step.

A fifth option the numbers suggest: a small fixed floor on 8812EU images.
0x22 is fully clean on all three channels on this unit, cold and after the
soak, and on the real link it removes the loss floor 0 causes on a non-DFS
channel (section 8). The shipped ladder is all 20 MHz, which the fault does
not touch (section 1); it applies once a 40 MHz rung or `static_bw = 40` is
loaded.

## Not shown

- One unit, mostly single runs. The devourer record lists an 8812EU that
  hangs outright at 0x20 and one 8812CU that needs 0x35; neither was here.
- Sections 1-7 are devourer `txdemo`, single frames, MCS2 only, with the
  MCS0/20 player as the stand-in for the GS. Section 8 is the real pair, but
  pinned to MCS2/40: the adaptive 40 MHz ladder was not run, and neither was
  a real non-DFS member of the shipped channel set (channel 40).
- No flight-hot state (encoder off), and "cold" was a 4-minute power-off.
- The uplink cost at the floors that matter (0x22–0x35) is extended from
  cliffs measured at 0x43–0x51; the bench had no way to attenuate further.
- 157(+) has light ambient traffic; no silent non-DFS channel was found.
- Whether raising the floor frees a unit that stops outright: shown on one
  8812CU adapter (section 9), not on an 8812EU and not on the harder CU.
