# FEC + software NACK spike: selective repeat on top of the sliding window

2026-10-05. Branch `spike-nack` (b124e42 + this doc). Bench only, no
flight. Question: can mabur keep broadcast FEC and two-card receive
diversity, and still ask the drone to re-send the symbols FEC could not
recover, inside the latency budget? Answer: **yes, and it is cheap.**

## Why not devourer's hardware ARQ

Devourer's ARQ is the MAC's positive-ACK loop (unicast RA, SIFS-timed
ACK/BlockAck from ONE GS card, autonomous re-air to a retry limit;
`third_party/devourer/CLAUDE.md`, `docs/aggregation.md`). There is no NACK
primitive in 802.11 on these chips: silence means re-air. So the cost is
paid on every PPDU (ACK or BA after each aggregate, the ACK wait, and the
re-air decided by one card even when the sibling or FEC already had the
frame). Devourer measured the ACKed A-MPDU shape at −8 % goodput at MCS3,
and the E die (the drone's 8812EU) has never been measured as a solicitor.
Sub-millisecond repair is only possible that way; a host-side NACK is
bounded by two USB hops each way plus the drone's half-duplex burst.

## What was built (throwaway rig)

- `T_NACK` uplink frame (`rc_proto.h`): up to 4 runs of (sid, first_seq,
  32-bit bitmap), SipHash-tagged under the session nonces with its own
  seq32 as ctx. No RC_VERSION bump (additive; an old peer ignores it).
- GS `NackTracker` (`gs/src/nack_tracker.h`): per layer, the decoder's
  erasure set (`SwDecoder::missing_sources`: unknown seqs behind newest);
  a seq missing for `settle_ms` is requested, repeated every `repeat_ms`
  up to `max_tries`. Books every requested seq as filled (direct copy
  arrived, with latency from first request), wasted (a repair recovered
  it first) or abandoned (fell off the floor). `max_tries 0` = observe
  only, which logs how long never-requested seqs stay "missing" (the
  natural reorder / second-card lateness). Config `[link.nack]`; the
  send is direct (`send_control_frame`, mid-burst, carrier sense
  arbitrates) or `slotted` through the RcfSlotter.
- Drone: `UepEncoder::set_source_tap` → `RetxRing` (512 source envelopes
  per layer, ~180 kB); the RX thread verifies the tag against the
  published session, packs the requested envelopes into fresh SBI bodies
  (layer geometry) and `TxQueue::push_front`s them.
- `tools/bench/nack/`: arm configs, runner, UDP loss helper, report.

## Natural lateness sets `settle_ms`

Observe-only run (injected loss on, NACK off), 16 292 seqs that showed
up by themselves after first being counted missing:

| late_ms | p50 | p90 | p99 | max |
|---|---|---|---|---|
| | 2 | 4 | 7 | 12 |

With `settle_ms 0` the tracker fired on every reorder / second-card copy:
42 NACKs/s, 114 retransmit bodies/s, nearly all "filled" by the late
original. `settle_ms` must sit above this window; 12 ms was used.

## Bench A/B

Pinned mcs2 / 40 MHz, base ov 0.5, enh ov 0.25 (the deployed ladder's
pair), 60 AU/s, two GS cards, `maburgs.nack.losssim` with loss injected
at the GS per (card, stream): `s0 eff=1.5 burst=4`, `s1 eff=1.5 burst=4`
(12.25 % per card, 1.5 % nominal union, mean run 4 bodies). 5 min per arm,
`frame_gap_timeout_ms 50`. Sessions `/media/dvr/log/0008..0012` on the GS
(copies: this session's scratchpad only).

| arm | session | truncated AUs | dropped AUs | abandoned FEC episodes | first→finish p99 ms | NACKs | syms requested | wasted | fill ms p50 / p90 / max |
|---|---|---|---|---|---|---|---|---|---|
| A0 control, enh ov 0.25 | 0008 | 314 | 50 | 344 | 43.3 | – | – | – | – |
| A1 NACK direct, settle 12 | 0009 | **8** | **0** | **2** | 32.7 | 831 | 9 868 | 1 787 | 11 / 17 / 44 |
| A2 NACK slotted, settle 12 | 0010 | 247 | 3 | 11 | 45.0 | 1 394 | 17 858 | 4 074 | 44 / 52 / 62 |
| A1f NACK direct, settle 6 | 0011 | **1** | **0** | 2 | 28.7 | 1 540 | 15 208 | 7 354 | 10 / 16 / 37 |
| A3 no NACK, enh ov 0.50 | 0012 | 28 | 13 | 39 | 38.3 | – | – | – | – |

`link.air_pct` median 53.7–54.5 in every arm; drone cpu 35–36 %; GS
`tx_pps` 20 (A1) and 25 (A1f) vs 20 control. Drone side: 0 bad tags, 0
ring misses across 7 406 NACKs.

Readings:

- **The direct NACK removes 97–100 % of the truncations and all the
  dropped frames** at this loss, for ~10 kB/s of retransmit air (A1:
  9 868 × 346 B over 318 s ≈ 86 kb/s, under 1 % of the stream) and 2.6
  uplink sends/s. Fill latency 11 ms p50, 17 ms p90, 44 ms max from the
  request, well inside the 50 ms the GS already waits before truncating.
- **The slotted variant is useless**: riding the RcfSlotter puts the
  request at the AU boundary, fill p50 44 ms, and the frame has usually
  been truncated by the time the copy lands (247 truncated with only 11
  abandoned episodes).
- **settle 6 vs 12**: one fewer truncation per 5 min, but half the
  requests were wasted (7 354 of 15 208 recovered by a repair before the
  copy arrived) and uplink sends doubled. 12 ms is the better operating
  point; the knob should be derived from the observed late_ms, not fixed.
- **The static alternative (enh ov 0.25 → 0.50) is an order of magnitude
  weaker** (28 truncated, 13 dropped, 39 abandoned) and costs ~+0.5 pt of
  airtime always. The NACK pays only when FEC fails.
- Mid-burst GS sends did not measurably hurt the downlink (complete-AU
  p50 identical, air_pct identical); carrier sense on both ends is what
  makes that safe (`docs/cca-on-findings-2026-09-23.md`).

## Caveats

- Loss is injected in GS software, per card and independent, so the
  union's hole structure is not a real per-PPDU loss on both cards. The
  retransmit itself is subject to the same injection, which is fair.
- No flight. The DVR card holds no flight sessions any more; the only
  flight number found is 235 abandoned symbols over one flight
  (`docs/sbi-salvage-flights-2026-09-09.md`). Bench session 0004 (21 min,
  HT40 rungs 1–4) showed the same structure as the prize here: 147 of 149
  abandoned episodes on enh at ov 0.25, each one lost aggregate.
- Uplink delivery was 88 % in session 0004; `max_tries 2` with
  `repeat_ms 16` covered it here (33 repeats of 831).
- `filled` includes late originals that arrived after a request; with
  settle 12 that share is small (late_ms max 12–38).

## Recommendation

Build it for real, direct-send only, `settle_ms` ≈ observed late_ms max
(12 ms here; make it adaptive), `repeat_ms` one AU period, `max_tries 2`,
`lookback` ≤ the decoder horizon. Then revisit the ladder's enh overhead
downward — the NACK is "overhead on demand". Hardware ARQ stays off the
table unless a sub-10 ms repair is ever required.

Rollback on the bench: drone `/usr/bin/maburd.pre-nack` (the spike maburd
is passive without a NACK sender and was left in place); GS production
`maburgs` and `/etc/maburgs.toml` were never touched, `maburgs.nack` is
a side binary.
