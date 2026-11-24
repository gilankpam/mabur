# 2026-11-25 — wide-TX IGI floor on the bench 8812EU drone: fixed value or self-calibration

**Status:** spike, nothing shipped; the decision is open. The floor as built
is described in [`docs/deploy.md`](../deploy.md) ("wide-TX IGI floor");
`maburd` still sets it to 0 on 8812EU images. The bench tooling is on the
local devourer branch `igi-floor-spike` (throwaway, not for merge).

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

## What this means for the four options

- **Fixed 0x35 on the 8812EU too:** no boot time, covers every unit measured
  so far, costs at least 12 dB of uplink and plausibly ~20.
- **Self-calibration at boot:** works only on a silent channel. It needs
  ~2–3 s of full-duty 40 MHz transmission before the link is up, a searching
  GS blurs the result, and the edge it finds is for that channel only (144
  and 157 differ by three steps on this unit).
- **Runtime watchdog** (raise the floor when the drone's own TX FIFO stops
  draining): the signal exists and needs no GS; it follows the channel and
  the temperature; it costs uplink only on a unit that needs it. Not built;
  a busy channel also backs the FIFO up, so it needs a cap.
- **One-time calibration stored in config:** no boot time, but it needs a
  value per channel or a margin over the worst one, and an operator step.

A fifth option the numbers suggest: a small fixed floor on 8812EU images.
0x22 is fully clean on all three channels on this unit, cold and after the
soak.

## Not shown

- One unit, mostly single runs. The devourer record lists an 8812EU that
  hangs outright at 0x20 and one 8812CU that needs 0x35; neither was here.
- devourer `txdemo`, single frames, MCS2 only — not `maburd` with A-MPDU and
  the real RCF/NACK exchange. RCF delivery and NACK success were not
  measured; the MCS0/20 player is the stand-in.
- No flight-hot state (encoder off), and "cold" was a 4-minute power-off.
- The uplink cost at the floors that matter (0x22–0x35) is extended from
  cliffs measured at 0x43–0x51; the bench had no way to attenuate further.
- 157(+) has light ambient traffic; no silent non-DFS channel was found.
- Whether raising the floor frees a unit that stops outright was not
  testable on this unit.
