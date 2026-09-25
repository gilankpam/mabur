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

## Bench validation, 2026-09-26 (full daemon, GS session 0232)

Long-frame jam (`linkbench-tx --symbol-size 1000 --no-cca --foreign-sa`,
~16 kB frames at MCS0, ~95 % airtime) from this PC's EU card on the op
channel, the jammer within a metre of both the GS and the drone.

**Detection works.** Onset to hop order 143-297 ms (was: never, verdict
`unknown`); first window already `interfered` with evidence 0x61
(starved|blocked|impaired). Both cards read 96-100 % busy against ~0 % own
airtime.

**What went wrong around it, and the fixes (all on `nhm-airtime`):**

| run | failure | fix |
|---|---|---|
| 1 | a false `raised`+recovered trigger on 112 (loss 0, ambient FA 240-460/s) hopped the link into 136, which the dwells already read 100 % busy (jam leakage); hold on a blocked channel ~33 s | never hop into a blocked channel; `recovered_min` 8; escape from a blocked hold (27459aa) |
| 2 | a trickle of frames broke the starved rule for a second; the order never reached the drone (uplink jammed at the drone too) and the withdraw blacklisted the clean target | AU-rate near-starved (`starved_frac`); keep an unconfirmed order up to `confirm_extend_ms` while the op is blocked, withdraw as `undelivered` (9e22fa0, 8709d87, d79949a) |
| 3-5 | GS and drone apart 33-47 s after a hop | **root cause below** (bcf398e) |

**Root cause of the splits.** Instrumented with `MABUR_HOP_DEBUG` (every
video body stamped with the hop target, the other card's latest drone seq,
and the chip's RF18 readback; every retune with thread and readback).
1444 retunes: RF18 always matched the requested channel, so the chip was
where the GS thought. The confirming frames were genuine — the drone had
hopped. Its log then showed the bounce:
`144 -> 112 (hop)`, immediately `112 -> 144 (disc)`. During a hop the TX
card stays on the old channel and sends both the RCF carrying the order and
the ~1 Hz keep-alive DISC proposing `plan.op()` = the OLD channel; the drone
took the order, then processed the DISC it had received on the old channel
just after it, and retuned back. Fix: hold the keep-alive DISC while
`plan.hopping()`.

**After the fix:** 5 jams on the op channel, the drone followed every hop
(no `disc` bounce in its log), dead video per jam ~0.2 s for a single hop,
~1.0 s for a double hop (144 blocked by leakage → 112), versus 33-47 s
before. `ausniff` clean throughout.

**Open, not fixed:**
- Verify on 112 can still fail on the pre-existing event evidence: `raised`
  (112's ambient FA background) plus recovered symbols from the hop's own
  gap. The link stays up (hold with video flowing), but 112 is backed off
  as Failed for 30 s.
- After a GS restart the drone oscillated `disc` ↔ `move_unconfirmed`
  between 136 and 112 three times before settling (boot-pick move; not
  hop-related).
- With the jammer beside the drone, orders can still be undeliverable on
  the jammed channel; the extension + undelivered rules only bound the
  damage. In the field the jammer and drone are usually far apart.
- Analog VTX and DJI O4 rows (operator), candidate-only and clean-rung rows
  of the matrix: not run yet.
