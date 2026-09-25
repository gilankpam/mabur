# NHM busy-airtime spike — bench findings, 2026-09-25

Gate for the NHM airtime evidence work (spec
`docs/superpowers/specs/2026-09-25-nhm-airtime-design.md`, local only). Can
the GS read busy airtime from the 8822EU's NHM engine, and can it subtract
its own video's airtime well enough that "foreign airtime" means something?
**Yes.** Defaults chosen: `hop.verdict.busy_dbm = -83`, `blocked_pct = 50`.

## Setup

Daemons stopped. Drone: `linkbench-tx` on ch 136, LDPC+STBC, overhead 0.25,
loading each rung to ~50–80 % airtime (the top of the air model). GS:
`linkbench-rx --channel 136 --bw 40 --nhm-busy-ms 150` (new spike mode: a
busy-NHM window armed for 140 ms — period 35000 × 4 µs — read, re-armed, one
`N` line per window with the 12 buckets, busy at three edges, and our own
airtime reconstructed from the received frames). Interferers: this PC's
8812EU `linkbench-tx --no-cca --foreign-sa` on ch 64, drone silent, GS tuned
40 MHz on 60+64. devourer `nhm-busy` branch (ArmNhmBusy/ReadNhmBusy).

## (a) Normalisation

Bucket sums read 251–255 at a 140 ms period and every window reports ready
— the NHM divider keeps the histogram at 255 samples regardless of period.
The idle floor sits in buckets 4–7 (below −83 dBm); our frames and the
jammer's land in bucket 11 (above −70). busy at −83, −80 and −75 read
identical in every window: at bench range there is nothing between −83 and
−70.

## (b) FA/CCA counter reset mid-window

`--nhm-reset` runs the verdict's `read_energy_scout()` reset half-way
through each window: mean sum 251.8 vs 252.1, busy 74.2 vs 74.4 — the reset
does not touch NHM. The daemon's read → reset → re-arm order is tidy, not
required.

## (c) Our own airtime vs NHM busy on a clean channel

`err = busy(−83) − own`, per 150 ms window, ~160 windows each:

| rung | own air % | mean err | min | max |
|---|---|---|---|---|
| 20/0 singles, 3 Mb/s | 79 | 1.5 | — | — |
| 20/4 agg6, 16 Mb/s | 73 | 2.2 | −1.7 | 7.0 |
| 20/4 singles | 80 | 2.3 | — | — |
| 40/3 agg6, 24 Mb/s | 77 | 2.1 | −7.3 | 4.3 |
| 40/4 agg6, 24 Mb/s | 53 | 1.4 | −9.5 | 4.7 |
| 40/4 singles | 66 | 5.8 | −6.1 | 9.4 |

The first 40 MHz run read own airtime at **2×** (158 % / 105 %): devourer's
Jaguar3 PHY-status parser mapped `rxsc 0` — "the receiver's full configured
width" in vendor phydm — to 20 MHz, so every HT40 frame on a 40-tuned card
was timed at the 20 MHz rate. Fixed in devourer (`nhm-busy` 4263e46, the
same rule the RTL8733B parser already had); the table above is after the
fix. PPDUs counted from `physt` (one per PPDU) are right: agg6 and singles
agree within a few points; 40/4 singles run ~4 points high (STBC HT-LTF /
preamble at 40 MHz slightly under-counted), still under 10.

## (d) PPDU counting

Confirmed by (c): `physt`-per-PPDU counting gives the same error for agg6
and singles at 20 MHz. No RX descriptor change needed.

## (e) Interferers (drone silent, GS 40 MHz on 60+64)

| interferer on ch 64 (20 MHz, MCS0, 5 Mb/s app, CCA off) | busy(−83) |
|---|---|
| none | 0.0 |
| short frames (~1 kB, ~600/s) | 95.3–95.7 |
| long frames (~16 kB, ~50/s — the jam the verdict missed) | 98–99 |

Analog VTX and DJI O4 readings move to the bench validation (they need the
operator).

## Choice

- `busy_dbm = -83`: the most sensitive edge with a clean idle reading (idle
  noise never reaches bucket 8).
- `blocked_pct = 50`: midway between the worst clean error (~10 in a single
  window) and the interferers (95–99). The margin also covers a range-edge
  artefact: our own frames heard above −83 dBm but failing CRC are not
  subtracted and read as foreign.

## Tool note

The first long-jam captures alternated valid / not-ready windows: the spike
mode scheduled reads off the previous deadline, so a late window's successor
was read before its period had elapsed (fixed, 112a6db). The daemon's verdict
loop re-arms at read time and reads ≥ 150 ms later.
