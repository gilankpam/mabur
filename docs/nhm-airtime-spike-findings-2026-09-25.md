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

That run could not tell a cleared window from an intact one: the histogram
is normalised (a), so on a steady channel a window cut to its tail reads
the same. Settled 2026-09-26 with a time-varying jam (`bench/nhmreset`
probe on one GS 8812EU, ch 144; host EU card `linkbench-tx --mcs 7
--bitrate 40M --tx-threads 1 --duty 100:100 --no-cca --foreign-sa`), 200 ms
windows so each spans exactly one on- and one off-phase:

| windows | reset | busy mean | sd |
|---|---|---|---|
| 200 × 200 ms | none (coex 2 s tick only) | 48.3 % | 0.5 |
| 200 × 200 ms | `GetRxEnergy(false)` at +150 ms | 48.3 % | 0.4 |
| 100 × 50 ms (positive control) | none | 49.2 % | 39.5 (0–97 %) |

A window cleared to its last 50 ms would have scattered like the control;
none did, and ready latency (~201 ms) and the 255 sum were unchanged. The
0x1eb4[25] reset — scout read, energy read or the coex tick — does not
disturb an armed busy window. One unit, 5 GHz only.

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
- Candidate-only, busy-but-healthy (analog) and clean-rung rows of the
  matrix: not run yet. Analog and O4 co-channel: below.

## Analog VTX and DJI O4, 2026-09-26 (full daemon, GS session 0233)

Two-card GS, drone on the desk, disarmed (30 fps). Interferers placed by
the operator: analog VTX ~3 m from the GS, DJI O4 air unit moved further
away after the first run with its goggles ~3 m from the GS.

**Analog VTX (home 136 = 132+136, candidates [144, 112]).** The case
2026-09-25 could not see at all (`docs/analog-vtx-findings-2026-09-25.md`)
now triggers on the blocked bit. First window with the VTX on the op pair:
`interfered 0x21`, both cards 100 % busy, own ~4 %; `H order` 150 ms later,
`lead_confirm` 83 ms, `verify_pass`. The operator then cycled through the
E band, which chased the link 144 → 112 (`0x61`), and on switch-off it
went 112 → 144 → 136 → (escape) 112 → 136. Six hops, every
`lead_confirm` 82–116 ms, the drone followed each (no `disc` bounce),
`ausniff` 0 gaps on every pass.
- `verify_fail` on 112 and ~20 `hold_exhausted` there: 112's FA
  background (100–200/s, present with the VTX off too) sets `raised`, and
  with 136 and 144 both blocked the GS had nowhere to go. It correctly
  never hopped into a blocked channel. The known open item, reproduced.
- 144 (5690–5730) read 100 % busy on every dwell while the VTX was
  reported on E4 (5645). Unexplained. It was probably not really on E4, or
  the VTX desensed the scout at 3 m.

**DJI O4 (home 153 = 149+153; O4 ch2, 20 MHz).** Three runs:

| run | candidates, `confirm_ms` | what happened |
|---|---|---|
| 1 | [161, 144], 500 | Blocked at onset (`0x61`/`0x71`, ~80 %), but loss 0 for ~3 s, so no hop (busy but healthy). Hopped at 48 % loss → 161, `lead_confirm` 460 ms. The O4 read ~80 % on **both** 144 and 153 (also after moving the air unit away) and 25–37 % on 161, so the link sat lossy on 161 at rung 0–2 with `hold_exhausted`, 13 gaps / 10 s. Also one 161→153 hop into a momentary dip, then a `withdraw` at 506 ms. |
| 2 | [161, 112], 500 | The O4 had shifted (boot scan: 157/161 ~33 %, 153 clean). 153 → 161 → **112**, a clean escape (rung 4, 20 s clean). Then two `interfered 0x11` windows on 112 (FA background + 6.7 % loss) ordered 153. The drone moved (`112 -> 153 (hop)`), and the GS **withdrew at 510 ms** and went back to 112. **60 s split**: GS on 112 in `hold_cap`, drone on 153, 5–9 fps. It ended when `hold_cap` (50 s) expired and the GS re-ordered 153. |
| 3 | [161, 112], **1000** | Order **152 ms** after the first blocked window (82–83 % both cards), `lead_confirm` 468 ms, `verify_pass`, no further `H`; 161 carried no foreign airtime; `ausniff` 31.8 fps, 0 gaps, 0 incomplete. |

**Confirm times depend on the pair.** Hops between 136/144/112 confirmed
in 82–116 ms. Hops touching 149+153 or 157+161 (8822E spur centres 151/159,
which `fast_retune` declines) took 423–460 ms, and 214 ms once. The
retune path accounts for only ~65–91 ms of that; the rest is not yet
explained (O4 loss on the new channel, the full-retune cost with
spur-notch setup, and whether the drone's 8812EU also takes the slow path
are all unchecked). A 500 ms `confirm_ms` left ~40 ms margin. The
**bundle now ships `hop.confirm_ms = 1000`** (the drone's
`move_confirm_ms` 2000 stays above it).

**The split's root cause is a design/code mismatch, still open.**
`docs/inflight-channel-hop.md` §1 says a drone that moved before a
withdraw times out on `move_confirm_ms` and converges on the old
channel. `RcAgent::tick` instead calls `go_home_("move_unconfirmed")`.
With home == the hop target, the drone was already "home" and stayed.
On the GS side, the scout's periodic home dwells caught the drone's
frames, so the session never counted as lost, `split_after_ms` never
engaged, and only `hold_cap` ended it. Fix candidates: the drone falls
back to the pre-hop channel, not home; and/or the GS's split detector
ignores frames that arrive only on scout dwells.

**Split fixes, hardware check (same day, GS session 0234).** Drone
`dd9478e` (an unconfirmed hop reverts to the pre-hop channel before going
home) + GS `ea872a5` (only op/hop-target video refreshes the rendezvous
silence timer). To force the failure, the GS ran `confirm_ms` 100 and
`confirm_extend_ms` 100 (extension off), and the host EU card jammed op 144
for 4 s (`linkbench-tx --symbol-size 1000 --no-cca --foreign-sa`). Twice:
the GS ordered 144 → **136 = home** (the failing case) and withdrew at
100–104 ms after the drone had already moved; then 112, withdrawn too,
which the drone never heard. The drone logged `144 -> 136 (hop)`, then
`136 -> 144 (move_unconfirmed)` ~2 s later (the old code would have stayed
silent on home). `au.log`: 0 AUs for ~2 s, partial while the jam lasted,
full 30 fps within ~1 s of the jam ending; both times, versus 60 s before
the fix. The 4 s of 75–100 % `link_loss_pct` after the jam in `scan.log` is
that metric's averaging window, not dead video. Config restored to
`confirm_ms` 1000 / `confirm_extend_ms` 3000; `ausniff` 0 gaps.

**One-card GS (`[[radio.cards]]` pinned to card 0), same day, session 0234.**

| row | result |
|---|---|
| long-frame jam, adjacent (op 136, jam on 144, ~1 m) | leakage read 99 % busy, but loss 0: no hop (busy but healthy). Then starved: 136 → 112 → 144, `one_card_retune` +254–266 ms, `lead_confirm` +280 ms; AUs dipped to 23/s, never 0 |
| long-frame jam, co-channel (op 144) | 144 → 136 (`lead_confirm` +250 ms, `verify_fail`: 136 also reads the jammer), `escape` → 112 (+308 ms, `verify_fail` on 112's FA background), then `hold_cap` on 112; AUs 22–26/s for ~3 s, never 0 |
| analog VTX, 3 m, operator swept the E band onto E3 | detected (`0x61`, 100 %), 136 → 112 +296 ms; followed the sweep 112 → 144 → 136 → `escape` 112; all 6 hops confirmed 224–300 ms, drone followed each. **This card read the WHOLE band ~100 % busy, 112 included** (two cards at the same 3 m read 112 ~10 %), so the link sat on 112 at 10–34 % loss, 20–27 fps, cycling `hold_exhausted` until the VTX went off; then 112 → 136 `verify_pass`, 32 fps |
| DJI O4 ch2, air unit away, goggles 3 m | op 153 ~10–15 % foreign, loss 3–5 %: no hop, correct (busy but healthy), 32 fps |
| DJI O4 ch3 | 153 hit → 112, `one_card_retune` +511 ms, **`lead_confirm` +570 ms** (a withdraw under the old 500 ms `confirm_ms`). Then **~3 min stuck on 112 at 36–52 % loss** (RSSI −51.6 / SNR 32.8, same as when it was clean; NHM invalid in 139/142 windows; 8–24 AUs/s), cycling `hold_exhausted`, until a 112 → 153 (`verify_fail`) → 161 (`verify_pass`) hop restored 30 fps. Not resolved: whether the O4 was still on when that hop fired, i.e. whether the O4 desensed the card on 112 or 112 was bad by itself |

One-card confirms run 224–300 ms on fast pairs and 516–570 ms on the spur
pairs (the order's 5 RCF repeats come first), inside `confirm_ms` 1000.
**Gap (open):** one card has no in-session dwells, so once a hop lands on a
lossy channel and its verify fails, the GS has no fresh evidence about the
alternatives and can loop `hold_exhausted` for minutes (3 min at 40–50 %
loss above). Two cards did not hit this. A one-card GS may need a
time-bounded retry of the backed-off candidates, or short dwells while
held on an impaired channel.

**20 MHz only (both ends `radio.width = 20`, every rung `bw = 20`, two
cards), same day.** Host EU card as the jammer (`linkbench-tx --bw 20
--symbol-size 1000 --no-cca --foreign-sa`, 6 s), ~1 m from both ends.

| run | result |
|---|---|
| jam on op 136 | first window `interfered 0x61` (both cards 100 %) → order 144 **150 ms** later, `lead_confirm` **217 ms**, `verify_pass`. 3.8 s later 144 → 112 (`lead_confirm` 190 ms, `verify_pass`): on 144, 64 % loss at ~6 % busy while the jam lasted. AUs 18/s at the first hop, 6–16/s for ~3 s around the second |
| jam on 112, but the link had already left 112 for 136 (112's `raised` FA trigger) | off-channel jam, yet **video on 136 fell to 1–4 AUs/s for 7 s with no hop**. The GS cards on 136 read clean (busy ~10 %, foreign 0, RSSI −54, SNR 30); the dwells read 112 at 100 %. Verdict `unknown 0x41` (starved + impaired, no blocked/raised), so no `interfered`, no order |

The second row, and the 64 %-loss-at-6 %-busy leg of the first, point to
loss at the DRONE's end with the jammer loud and ~1 m away: the drone's
carrier sense (CCA is on both ends) deferring its TX, or its front end
desensed. **Unverified:** maburd exports no CCA-deferral counter (`drops`,
`tx_failed`, `txq_drop` all 0), and the GS cannot tell this from a clean
channel, so a hop is neither triggered nor obviously useful. It is a bench
geometry artifact first (in flight the jammer is not beside the drone),
but it is the same blind spot as the drone's RX view in
`docs/cca-on-findings-2026-09-23.md`. 112's `raised` trigger fired at 20 MHz
too, so that open item is width-independent.

**Unexplained baseline:** with nothing on air, home 153 and 161 ran at
rung 0–1 with 3–15 % loss in some windows and no foreign airtime
(run 3). Run 1 held rung 3–4 on 153.

Bench restored: home 136 / [144, 112] both ends (`confirm_ms` 1000 kept
on the GS); the boot scan picked 144 (136 ambient 17.7 %); `ausniff`
0 gaps, 30.8 fps.
