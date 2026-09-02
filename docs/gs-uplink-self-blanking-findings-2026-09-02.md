# GS uplink self-blanking: the bench's ~0.35% per-PPDU video loss (2026-09-02)

**Status: ROOT-CAUSED on the bench; lever 1 (RCF slotting) IMPLEMENTED + bench-deployed 2026-09-03 — see the addendum at the end.** Every video PPDU
whose preamble lands while the ground station is transmitting its own
uplink (RCF, ~20 sends/s at `feedback_ms 50`) is lost on BOTH GS cards.
With A-MPDU each hit costs a whole aggregate (4 bodies), and a hit in a
frame's tail is what produces the 34 ms FEC repair stalls behind the
player's p99 and replaced frames (see the same-day jitter decomposition,
memory `jitter-decomposition-2026-09-02`).

## Symptom

Bench, mcs5 park, 11 Mb/s cap, `ampdu.max_num 6`, `feed_batch 6`, flat
0.5/0.5 pairs, SNR ~31 dB, two RX cards:

| metric | value |
|---|---|
| sideport `pre_fec_loss` | ~0.3–0.4% |
| per-card `loss_pct` | 0.67% median, identical on both cards |
| decoder `recovered` | ~12 symbols/s ≈ 3 bodies/s |
| AU completion (`fec` segment) | discrete +34–36 ms cluster, ~0.4–0.5/s |

Losses did not correlate with MCS (per-rung `u` is nonzero at rungs 2–5),
with GS TX rate at 200 ms resolution, or with load. Drone `txq`/`radio`
drop counters were flat. A-MPDU RX-side damage was already excluded by the
physt fix. The 2026-09-01 txdemo/rxdemo spike had shown the same ~0.3%
("~99.7% of offered") in every cell, singles included — it was never
aggregation.

## Instrument

`MABUR_GAPLOG=1` (env-gated, branch `gaplog-diag`, GS binary left at
`/usr/local/bin/maburgs.gaplog`) makes maburgs print one stderr line per
per-card 12-bit seq gap with the chip TSF and host-mono advance across the
gap, the aggregate position of the frame before it (`prev_agg`, frames
since the last `phy_valid` = aggregate-first) and whether the frame after
it starts a new aggregate (`after_physt`); plus `late` (behind-the-mark
arrivals), `nonvid` (MSP/RC frames), `gstx` (each `send_control`, card +
mono), `self` (the uplink as heard by the sibling card: RSSI, TSF since
its last video frame) and `crcfail` (inert on the 8822E, see below).
Analysis scripts: scratchpad `gapan.py` / `gstxan.py` (session-local).

Run recipe (150 s, bench):

```
/etc/init.d/S96maburgs stop
MABUR_GAPLOG=1 timeout 150 /usr/local/bin/maburgs.gaplog -c /etc/maburgs.json > /tmp/gaplog.log 2>&1
/etc/init.d/S96maburgs start
```

## Evidence (100 s steady windows, card 0; card 1 is identical)

1. **Half the seq gaps are phantom.** The drone numbers every body from
   one software counter (`RadioTx::seq_` in `drone/src/radio_tx.cpp`,
   `stream_id >= 2 → layer 0`), so MSP bodies (stream 4, 1343 B, ~5–8/s)
   consume video seqs that the GS excludes from its seq walk. 477/481
   short gaps matched an MSP or drone RC frame with that exact seq within
   50 ms. The `aggregator.cpp` comment claiming MSP and RC have their own
   802.11 counters was wrong for both: maburd does keep separate software
   counters (`RadioTx::seq_`, `control_seq`), but devourer sets EN_HWSEQ on
   the TX descriptor and the chip overwrites the header with its single
   hardware counter (confirmed 2026-09-03 after the MSP-only fix left
   ~2 gaps/s that matched T_TELEM/DISC_ACK seqs exactly). Consequence:
   per-card `loss_pct` was ~2× too high (phantom + 0.35% real ≈ 0.67%).
   Fixed on branch `msp-seq-fix`: the walk now counts every drone frame.
2. **Real losses are joint and whole-PPDU.** 569/570 card-0 gaps had a
   card-1 gap with the same seqs, same `n`, same TSF delta. Zero `late`
   lines (nothing ever arrived behind the mark). (A `keep_corrupted` run was attempted and is VOID: maburgs does not
   plumb `DEVOURER_RX_KEEP_CORRUPTED`, and devourer implements it only on
   jaguar1/2, not the GS's 8822E.)
   With agg 6: `n` = 4 (64) or 3 (16) of 87, `prev_agg` = 3 in 82/87,
   `after_physt` = 1 in 87/87 → exactly one whole aggregate (aggregates
   run 4 deep on this feed, not 6). TSF advance across the hole ≈ 1.55 ms
   ≈ the aggregate's air time: it was transmitted.
3. **Per-PPDU probability is duration-independent.** agg 6: 0.87
   events/s over ~260 PPDU/s = 0.33%; singles (`max_num 0`): 9.0/s over
   ~2300 PPDU/s = 0.39%. A random-time interferer would scale with PPDU
   length (4×); this does not → the failure is at the PPDU *start*.
   Singles therefore lose ~3× more frames/s than agg 6 (9.1 vs 3.2).
4. **Timing lock to the GS's own sends.** ms from the preceding
   `send_control` to the lost PPDU's start (loss = frame after the hole,
   minus the TSF delta):

   | run | gstx/s | losses 0–6 ms after a send | chance | peak |
   |---|---|---|---|---|
   | agg 6 (`gaplog2`) | 20.6 | 98% (85/87) | 12% | 1.0–1.5 ms |
   | singles (`gaplog4`) | 20.7 | 99% (906/915) | 12% | 0.5–2.0 ms |
   | singles, `feedback_ms 200` (`gaplog5`) | 6.0 | 99% (315/318) | 3% | 0.5–2.0 ms |

   Both TX cards are blamed equally (singles: 468 vs 447); the
   non-transmitting card loses the same PPDU.
5. **Manipulation.** `feedback_ms` 50 → 200 on a temp GS config only
   (drone untouched, singles): real loss 9.15 → 3.18 frames/s, tracking
   the send rate (20.7 → 6.0/s incl. repeats). Restored afterwards.
6. **Arithmetic.** Implied blanking window per send = loss rate ÷
   (sends/s × PPDU/s) = 192 µs (singles) / 162 µs (agg) — an RCF's
   airtime plus the drone preamble it overlaps. 20 sends/s × ~180 µs =
   0.36% of PPDUs, the number the sideport has been reporting.

Excluded along the way: drone RC frames (T_TELEM/DISC, 1 Hz each) and MSP
frames are not time-correlated with losses (2%/8% within ±10 ms, at
chance); reorder (no `late`); MCS
margin; USB RX stalls (rx_pace host gaps are the inter-AU idle).

## Mechanism (resolved, run `gaplog7`)

Three measured facts, from the `self` lines (the uplink frame as heard by
the *non*-transmitting card):

1. **The sibling card is blasted, not merely listening.** RCF (HT MCS0,
   20 MHz, LDPC+STBC, 20–27 B body) arrives at the sibling at **−4 dBm on
   chain A / −21 dBm on chain B** — ~55 dB above the drone's −60 dBm.
   That is front-end saturation; the card cannot acquire a preamble for
   the RCF's airtime plus AGC recovery (the implied ~160–190 µs window).
   The transmitting card is deaf by definition. Hence joint loss.
2. **The TX card fires only after the PPDU it is receiving ends.**
   Devourer's `dis_cca` (0x520 bits 14/15) disables CCA/EDCCA *deferral*,
   but the chip still does not start a TX while an RX PPDU is in
   progress. Measured: TSF from the sibling's last received video frame
   to the RCF is **200–250 µs in 44% of sends (p10 = p25 = 233 µs)** —
   one subframe slot after the last subframe — and that last frame was
   the 4th subframe of an aggregate in 66% of cases. The remaining sends
   fall in the inter-AU idle (2–10 ms bins), where nothing was being
   received. So the blast lands in the gap *between* PPDUs, never inside
   one, which is why losses are whole-PPDU and partial-aggregate losses
   are rare (7/96) even at −4 dBm.
3. **Lethality per send is set by the gap to the next PPDU.** Whether
   the next drone PPDU starts inside the ~180 µs blast window: singles
   follow each other after ~90 µs inside an AU burst → 44% of sends kill
   one (≈ burst duty cycle); aggregates follow after ~0.5 ms → ~4% of
   sends kill one. Same per-PPDU rate either way (~0.35%), and it does
   not scale with PPDU length — the puzzle that first pointed away from
   an interferer.

Chain: GS `send_control` → USB+chip ~1–1.5 ms → TX held to the end of the
RX PPDU in progress → RCF on air at the end of a drone aggregate → both
cards blind for ~180 µs → the next aggregate, if it starts inside that
window, is lost whole on both cards → 16-symbol hole → 34 ms tail stall
when it sits in a frame tail.

## Levers (not chosen, in rough payoff order)

1. **Slot the uplink into the drone's inter-AU idle.** The chip already
   holds TX to the end of the current PPDU; what kills is the *next* PPDU
   starting within ~180 µs. Sending right after an AU's `t_complete` puts
   the blast ~8 ms from the next AU's first aggregate. At 11 Mb/s the
   drone is silent ~8–10 ms between AUs (rx_pace `gaps` ≈ 60/s ≥ 5 ms).
   The GS knows each AU's completion time; sending the RCF right after
   `t_complete` (instead of on the `feedback_ms` timer) would put every
   send in silence. Zero video cost, keeps 20 Hz feedback. Needs the
   send latency (~0.5–2 ms USB+chip) budgeted inside the idle window.
2. **Fewer sends.** `feedback_ms` 50 → 100 halves the loss for free but
   doubles ladder reaction latency; the repeat burst (`rcf_repeat_copies
   3 / 10 ms`) adds sends exactly when the link is changing.
3. **Make a whole-aggregate tail hole repairable in-frame.** The
   encoder's flush emits one tail repair; a 16-symbol hole needs 16.
   Costs overhead only at frame tails.
4. **Keep A-MPDU.** Singles lose 3× more frames/s from the same
   mechanism; the aggregate merely concentrates it.

## Bench end state

Drone: `/etc/mabur.json` restored byte-identical from
`/etc/mabur.json.pre-gapdiag` (agg 6), maburd restarted, ladder re-parked
mcs5. GS: production `maburgs` from `/etc/maburgs.json` (feedback_ms 50);
`maburgs.gaplog` and `/tmp/maburgs-fb200.json` left on the GS. Code:
branch `gaplog-diag`, uncommitted (node.h `tsfl`, aggregator gap/late/
nonvid/crcfail lines, main.cpp `gstx` lines). Side finding, unrelated:
S95flightrec was not running on the bench GS when this started (maburtop
held :8300).

## Addendum 2026-09-03: lever 1 implemented (RCF slotting)

`RcfSlotter` (`gs/src/rcf_slot.h/.cpp`, config `link.rcf_slot_hold_ms`
default 30, sideport `link.rcf_slot`), documented in
`docs/link-adaptation.md` "RCF slotting". Two iterations on the bench:

- v1 (release at any AU completion, DISC bypassed): real losses 1.03 →
  0.37/s. Residuals: the 1 Hz DISC keepalive bypassing the slotter (6/32),
  and completions followed by the next burst within ~2 ms (14/32) — AU
  completion intervals spread 5–30 ms because completion moves with frame
  size while the drone's burst starts on its fixed frame cadence.
- v2 (release only when `now + 3 ms < last_t_first + period − 1 ms`,
  period = EMA of first-body intervals; DISC slotted): **1.03 → 0.11
  losses/s, 3.9 → 0.41 frames/s**; 45 % of the remaining 11 losses are not
  within 6 ms of any send (background ~0.06/s). Sends: au 88 %, grace 12 %,
  timeout 0.5 % (hold p50 6 / p90 17 / max 35 ms). RCF rx at the drone
  19.4/s (off: 18.5), rtt 7.2 ms (off: 7.8), `close_ms` 22 (off: 5).
  Gates after the prod swap: ausniff 60.0 fps / 0 gaps / 0 incomplete;
  player `fec` p99 ≥ 20 ms in 7/100 windows (31/90 before). aucadence
  read −3.6…−5.2 ms on and −3.0…−4.2 ms off (same binary, same session):
  the enh frames were 27 KB vs 19 KB base at the time — scene, not the
  slotter.

Deploy: GS `/usr/local/bin/maburgs` = slotter build (md5 5093a640…),
rollback `maburgs.pre-rcfslot`; config untouched (default applies).
