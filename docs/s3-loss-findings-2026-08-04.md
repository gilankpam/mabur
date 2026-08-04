# s3 injected-loss bench findings — 2026-08-04

## What this is, and a warning about the recordings

These numbers come from **synthetic loss injected at the ground station**, not
from a degraded radio link. The rig is branch `bench/s3-loss-sim` (never
merged): `maburgs` drops received radio bodies for a chosen stream inside
`Aggregator::on_rx_body`, between the SBI `stream_id` peek and any accounting,
so a dropped body is indistinguishable from one the air ate — no counters, no
EMAs, no per-class tracks, never reaches the FEC decoder, and the 12-bit
hardware-sequence gap appears on its own.

**Any `/tmp/maburgs.log` or sideport data from 2026-08-04 between roughly
22:35 and 23:10 UTC contains injected loss and must not be read as real link
behaviour.**

Bench conditions: GS `10.18.0.1`, 2 cards, link parked at `mcs7/20/ov0.10`,
symbol_size 332, SVC-T live. Baseline verified clean before injection
(`ausniff`: 0 gaps, 0 resyncs, 60.5 fps, twice consecutively).

## Headline: s3 degrades gracefully, with a hard floor at the base layer

Sweep on s3, effective (union) loss, mean burst 3, 25 s dwell. Frame counters
are `FrameStream`'s, per second, from the 1 Hz stderr stats line.

| step (eff) | clean/s | trunc/s | drop/s | trunc% | s3 abn/s | s1 bodies/s |
|---|---|---|---|---|---|---|
| 0% (base) | 60.0 | 0.0 | 0.0 | 0.0% | 0 | 1487 |
| 2% | 58.2 | 1.4 | 0.6 | 2.3% | 15 | 1490 |
| 5% | 53.4 | 5.6 | 2.5 | 9.1% | 65 | 1470 |
| 10% | 48.9 | 9.0 | 4.1 | 14.5% | 111 | 1469 |
| 20% | 37.9 | 17.0 | 10.2 | 26.1% | 273 | 1476 |
| 35% | 31.2 | 19.0 | 19.4 | 27.3% | 498 | 1479 |
| 50% | 29.9 | 16.4 | 27.1 | 22.4% | 723 | 1489 |
| restore | 60.0 | 0.0 | 0.0 | 0.1% | 7 | 1473 |

`clean/s` decays 60 → 58 → 53 → 49 → 38 → 31 → **29.9 and stops**. That floor
is the base layer: with s3 effectively annihilated at 50% (723 abandonments/s),
the base layer alone still delivers a clean 30 fps. There is no failure mode
below the floor, which is why the picture never breaks.

The knee is between 10% and 20%. The truncated fraction peaks at 35% (27.3%)
and then *falls* at 50%, because frames start being lost outright rather than
truncated (27.1 drop/s against 16.4 trunc/s).

Recovery is complete and immediate — restore returns to exactly 60.0/0.0/0.0.

## The base layer is untouched

`s1` bodies/s holds 1470–1490 at every step including 50%. A separate 90 s
controlled comparison put s1 complete AUs at 2660 (injection off) vs 2658
(s3 at eff=20) — a 0.08% delta, i.e. noise. Loss confined to s3 costs the base
layer nothing, which is the property SVC-T exists to provide.

## Operator observation: no visible corruption

At eff=20% on live video the pilot reported **no artifacts and no corruption**.
The only perceptible change was jitter rising to ~30 ms from the 11–17 ms
baseline. That matches the mechanism exactly: at 60 fps the frame interval is
16.7 ms, and a lost enhance frame makes it 33.3 ms, so the displayed cadence
alternates between the two.

This refutes an earlier reading of mine. The counters showed 1525 *incomplete*
s3 AUs reaching the ring at eff=20, which I initially called ungraceful
failure. It is not: a truncated enhance frame is discarded rather than decoded
into a visible artifact, so the cost lands entirely on timing, never on pixels.

## s3 carries no referenced frames in practice — hypothesis refuted

`drone/src/frame_pipeline.cpp:30` notes that `classify_frame` routes `TRAIL_R`
with `tid >= 2` to sid 3 alongside true `TRAIL_N` enhance frames, and TRAIL_R
frames *can* be referenced. `maburplay.log` also showed MPP decoder
`h265d: refs: cur_frm N missing ref poc M` messages, so the concern was that
dropping s3 breaks references.

Three-phase controlled test, 90 s per phase, counting those messages:

| phase | missing-ref events | s3 complete | s3 incomplete | frame_id gaps | fps |
|---|---|---|---|---|---|
| off | 1 | 2706 | 0 | 0 | 60.1 |
| on, eff=20 | **0** | 677 | 1525 | 500 | 54.5 |
| off again | 0 | 2705 | 0 | 0 | 60.1 |

Injection destroyed or truncated ~75% of s3 access units and produced **zero**
dangling references. The messages are ambient background from real link loss at
roughly one per 90 s. **s3 is genuinely non-referenced in practice**, including
its TRAIL_R traffic.

The off-phases also bracket the experiment tightly (5411 vs 5410 AUs, 2706 vs
2705 s3, zero gaps, 60.1 fps both), so the middle row is trustworthy and the rig
perturbs nothing when zeroed.

## The ladder is blind to s3 — quantified

The op point held `mcs7/20/ov0.10` unchanged through every step of the sweep,
up to and including 50% effective loss on s3 with 723 abandonments/s. The
controller took no action of any kind. This is the s3-abandonment blindness
recorded in `docs/mcs6-bench-anomaly.md`, now measured against a knob rather
than inferred: **no amount of s3 loss alone will move the ladder.** If s3
abandonment should influence the operating point, it needs an explicit input;
nothing in the current measured-loss path can see it.

## Body-level loss is not frame-level damage

The dial is body-level; the damage is frame-level, and the amplification is
large. At eff=20% body loss, s3 complete AUs fell 2705 → 677 — **75% of enhance
frames destroyed by a 20% body-loss rate** — because a single lost body
truncates an entire multi-body access unit.

Do not write "s3 tolerated 20% loss". The correct statement is that the frame
layer absorbed three-quarters of its enhance frames being destroyed and still
presented a clean picture. That is a considerably stronger result than the dial
suggests.

## Caveats on these numbers

- **`eff` is nominal.** The tool dials per-card and reports the union rate as
  `percard^ncards`, which assumes every card heard every body — true only on a
  clean link. On this bench (2 cards, both healthy, 0 CRC failures) the
  assumption holds; on a marginal link the true injected loss is higher.
- **Burst held at 3.0 for the whole sweep**, verified in the step log. Per-card
  reached 70.71% at eff=50, just under the 0.75 feasibility ceiling for burst 3,
  so the feasibility rule never fired and no step was silently re-parameterised.
- **The sideport recording is unusable for this session.** `statsrec` was not
  running — `maburtop` held UDP :8300 directly — so `/media/dvr/flight-0006*.jsonl`
  froze at 17:27, hours before the work. All figures here come from the 1 Hz
  stderr stats line in `/tmp/maburgs.log`, the documented fallback. Restart
  `S97statsrec` before any session where post-hoc sideport analysis is wanted.
- **On the `burst <= 1` fast path the model reports the parameter, not the
  delivered run length.** Bernoulli loss at rate L has mean drop-run `1/(1-L)`
  (measured 1.02/1.25/1.54/1.99 at L = 2/20/35/50%), so a nominal "burst 1" arm
  is really ~2 at the top steps. This is separate from, and compounds with, the
  feasibility raise noted in the burstiness section.
- **`vrx.on_video()` still fires for injected-dropped bodies**, so injection is
  not indistinguishable from real loss *above* the Aggregator. Harmless here,
  but this rig must not be used to test link-loss or rendezvous behaviour.

## Burstiness barely matters at the frame level

Same sweep on s3 at burst 1 (Bernoulli) against the burst-3 arm. Baselines
differed slightly between arms (60.0 vs 57.6 clean/s), so compare within-arm
percentages, not raw values.

| eff | burst 3 clean/s | (% of base) | burst 1 clean/s | (% of base) |
|---|---|---|---|---|
| 2% | 58.2 | 97% | 56.6 | 98% |
| 5% | 53.4 | 89% | 53.0 | 92% |
| 10% | 48.9 | 82% | 45.8 | 80% |
| 20% | 37.9 | 63% | 34.9 | 61% |
| 35% | 31.2 | 52% | 30.0 | 52% |
| 50% | 29.9 | 50% | 29.0 | 50% |

The two curves track each other within a few points the whole way, and both
land on the same 50%-of-baseline floor. **Burstiness does not materially change
frame-level damage on s3.** If anything burst 1 is marginally worse in the
10–20% band, which is the opposite of the FEC-centric intuition: uniformly
spread losses touch *more distinct* access units, whereas clustered losses
concentrate their damage into fewer frames that were going to be lost anyway.

Caveat on the top two steps: at eff 35% and 50% the per-card rate exceeds 50%,
so the feasibility rule raised burst to 1.4 and 2.4 respectively (visible in the
step log). Those two rows are therefore not a true burst-1 contrast. The clean
comparison range is 2–20%, where burst was genuinely 1.0 against 3.0.

## The s1 contrast: the ladder is the base layer's defence

Same rig pointed at s1, burst 3, at deliberately gentler steps.

| step (eff) | clean/s | s1 abn/s | op point |
|---|---|---|---|
| 0% (base) | 57.6 | 0 | mcs7/ov0.10 |
| 2% | 50.2 | 2 | **mcs5/ov0.25 → mcs6/ov0.15** |
| 5% | 50.0 | 4 | mcs5/ov0.25, mcs6/ov0.15 |
| 10% | 57.4 | 0 | mcs5/ov0.25 |
| after restore | 52.2 | 0 | mcs4→mcs5, climbing back |

This is the exact mirror image of s3, and it is the sharpest result of the
campaign:

- **At 2% loss on s1 the ladder immediately demoted two rungs**, from
  `mcs7/ov0.10` to `mcs5/ov0.25`. Compare s3, where 50% loss and 723
  abandonments/s moved it not at all.
- **The demotion then absorbed the loss.** By the 10% step the link had settled
  at `ov0.25` and clean/s was back to 57.4 — i.e. 10% injected loss on the base
  layer cost essentially nothing in frames, because the extra FEC overhead
  repaired it. `s1 abn/s` never rose above 4, against 233–670 for s3.
- **The visible cost was the transition, not the loss.** The dip to ~50 clean/s
  at the 2% and 5% steps is the ladder moving, and the link was still climbing
  back through mcs4/mcs5 after restore.

So the prediction that base-layer loss would "degrade badly and early" is wrong
in outcome and right in mechanism. It degrades *early* — the controller reacts
at 2% — but not badly, because reacting is precisely what protects it. What
base-layer loss actually costs you is **rungs, and the bitrate that goes with
them**, not picture.

Taken together the two arms explain the whole asymmetry: s1 has a closed control
loop and s3 has none. s3 survives without one only because nothing references
it and the base layer floors the frame rate at 30 fps.
