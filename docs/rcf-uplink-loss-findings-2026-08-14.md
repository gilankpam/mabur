# RCF uplink loss: why Part C's rc_drain decouple shows no close_ms gain

2026-08-14, follow-up to `docs/handover-rc-drain-close-ms-2026-08-14.md`.
Branch `fade-demote`, PR #29. Bench measurements on the deployed binaries
(drone `1fccda526a72fbe306c81aefe9f2d0f9`, GS `767db45d495a4f94531511c66bd2edaa`).

**Status: RESOLVED — root cause found and verified on hardware.** Part C
(the 5 ms RCF drain) works exactly as built. It shows no `close_ms`
improvement because op-actuation latency is dominated by a term Part C
never touched: **30–50% of the GS's rate-control frames never reach the
drone's receiver, and the loss fraction tracks the drone's own transmit
airtime.** A lost commit-RCF costs exactly one `feedback_ms` (50 ms)
period, so `close_ms` is a geometric ladder of +50 ms quanta on top of a
~11–28 ms fast path — and at the climb rungs the A/B measured, the drone
runs 50–60% airtime, so most boundaries eat one or more quanta.

## 1. The drain is running at 5 ms on the device (handover check #1)

Verified read-only via `/proc`, no instrumentation build needed:

- maburd's thread table on the bench drone has exactly one thread parked
  in `hrtimer_nanosleep` advancing ~217 voluntary context switches/s
  (10 s sample) — the ~200/s a 5 ms sleep predicts, plus 1 Hz work.
- `/proc/timer_list` shows 1 ns resolution (high-res timers active), so
  no jiffy-rounding ambiguity: only a genuine ~5 ms sleeper produces that
  rate. Thread creation order (msp → hot → tx → agent) independently maps
  the same tid to the agent loop.
- The code path was re-read end-to-end: `rx_callback` queues RC frames
  directly from the radio RX loop; the agent loop drains every wake and
  `RcAgent::on_rc_frame()` applies the ladder op inline
  (`rc_agent.cpp:287`), not in `tick()`; the hot thread picks up the
  published op within its 5 ms frame-read timeout (`main.cpp:904-908`).
  Nothing defers actuation to the 100 ms tick.

Method note for reuse: per-thread `voluntary_ctxt_switches` deltas over
10 s, plus `wchan`, identify every sleeper in the process without
touching the binary — cheaper than the wake-counter patch the handover
proposed.

## 2. close_ms at n=24: a +50 ms quantized ladder, not a shifted mean

Six forced climbs (`S96maburgs restart`, the handover's repro) on the new
binaries, 4 sampled boundaries each (→1 never exports a sample; see §5):

| → rung | samples (ms) |
|--------|--------------|
| 2 | 63, 115, 47, 75, 62, 62 |
| 3 | 96, 125, 62, 95, 97, 26 |
| 4 | 46, 163, 93, 27, 71, 94 |
| 5 | 11, 28, 66, 72, 130, 13 |

Median 64.5, mean 72.5, range 11–163. The distribution clusters at
~11–28 (5 samples), ~46–75 (10), ~93–97 (5), 115–163 (4): a fast floor
of one drain + one T0 frame interval, plus k ∈ {0,1,2,3} extra 50 ms
feedback periods. The 11–28 ms cluster IS Part C working — those
boundaries hit the acceptance target. The rest waited for a retransmitted
RCF.

Yesterday's n=4-per-arm A/B (old 41–100 vs new 35–173) is exactly what
sampling this distribution twice looks like; its "new is slower" reading
was noise, as the handover suspected.

## 3. The dominant term: airtime-correlated uplink RCF loss

From `flight-0013.jsonl` (parked, mcs5/ov0.25) and a full-datagram
capture across 2 climbs (2026-08-14 16:29–16:32):

- GS control injection rate: **20.3 frames/s** (19.3 RCF + ~1 keepalive
  DISC; sum of `cards[].tx_pps`, `tx_fail` = 0).
- Drone accepted-RCF rate (`drone.rcf.rx_pps`, from the T_TELEM
  `rcf_accepted` counter): **13.6/s parked — ~30% loss, in 235 of 237
  windows.** Uplink SNR 29–34 dB on both chains throughout: this is not
  a link-budget problem.
- Per-rung, during climbs:

| rung | link.air_pct | RCF delivery |
|------|--------------|--------------|
| 0 | 60% | 59% |
| 1 | 54% | 55% |
| 2 | 51% | 56% |
| 3 | 50% | 51% |
| 4 | 33% | 59% |
| 5 | 25% | 69% |

Delivery is monotone in the drone's own transmit airtime. Mechanism: the
drone's single half-duplex radio cannot receive while it transmits, and
**carrier sense is off on both daemons** (2026-08-05, by design), so the
GS injects RCFs blind into the drone's TX bursts. An RCF landing inside
a burst is gone (missed or CRC-smashed — both are the same self-collision
mechanism; `rcf.rx_pps` counts parsed-and-accepted). Loss slightly
exceeds `air_pct` at park because `air_pct` estimates video+FEC airtime
only — the drone's telem and MSP OSD TX add on top.

The airtime is by design: `waybeam.airtime_budget: 0.6` means climb
rungs 0–3 fill ~60% of air (their bitrate is unclamped), predicting ~40%
delivery there; the `bitrate_max_kbps: 8000` clamp is what drops rungs
4–5 to 25–33% air. That is why →5 has the best samples (its commit RCF
is sent while the drone still runs rung 4's clamped airtime) and why the
pre-branch baseline — median ~110, tails 295/971 — was measured mostly
against high-airtime rungs.

## 4. What this means

- **Part C's claim needs restating, not retracting.** The drain decouple
  removed U(0,100) of drone-side latency, verifiably. But the acceptance
  criterion (`close_ms` median ≤ 30 ms) is unreachable while ~30–50% of
  commit-RCFs are lost: the uplink retry quantum (50 ms) replaced the
  drain as the binding constraint. The criterion is met only by the
  k=0 boundaries (11–28 ms observed).
- **The blast radius is wider than close_ms.** Everything on the uplink
  control path sees the same loss: probe RCFs, the keepalive DISC, IDR
  requests, and any future GS→drone command. Design accordingly — a
  single-shot uplink command has a 30–50% chance of waiting ≥50 ms.
- **Remediation candidates:**
  1. ~~Enable CCA at the GS only.~~ **TESTED — WORSE than nothing
     (aligned collisions). See §6.**
  2. Enable CCA on BOTH sides — **TESTED, works partially: +15–22
     delivery points, close_ms median 49.5, zero video cost on the
     clean bench. But it re-exposes the video downlink to co-channel
     deferral (the 2026-08-05 CCA-off rationale), so it needs the
     congested-channel A/B before becoming the default. See §6.**
  3. Repeat the RCF a few times back-to-back (or at ~10 ms) after an op
     change until the drone's telemetry echoes the new op
     (`drone.applied` generation) — cheapest, bounded extra airtime,
     orthogonal to (and composable with) the CCA decision.
  4. Time RCF injection into the gap right after a received video frame
     burst — §6's GS-only arm shows naive burst-edge timing has the
     aligned-collision failure mode; aim mid-gap if ever built.
  5. Accept it: the ladder already tolerates the latency; close_ms just
     measures it now. Document and move on.
- Anyone tuning `feedback_ms` should know it is also the uplink retry
  quantum — halving it roughly halves the loss-induced actuation delay
  at constant loss probability per attempt.

## 5. Loose ends

- **→1 never exports a close_ms sample** in any of 8 observed climbs
  (yesterday's 2 + today's 6): `close_ms` stays null through rung 1 and
  the first non-null value is always →2's own close. Not root-caused.
  The obvious suspect is wrong: the startup `mark_transition` to rung 0
  consumes the `cur_mcs == kMcsUnknown` no-open case
  (`last_marked_op_mcs` starts at −1, `gs/src/main.cpp:318`), so →1
  should open with prev=mcs0. Remaining candidates: the 1 s
  `kBoundaryExpiryMs` force-close (disarms without recording), or
  something specific to the post-restart rendezvous window. Worth one
  look if →1 latency ever matters; it does not affect §2–§3.
- The 971 ms tail in the pre-branch baseline is deeper than a geometric
  tail at p≈0.5 comfortably explains (P ≈ 1e-5 per boundary); it may
  have been a coincident event (probe interplay, feedback gap). Not
  re-examined.
- Loss at park (~30%) modestly exceeds video `air_pct` (~25%); the gap
  is attributed to telem/MSP TX airtime above, but that attribution is
  an inference, not a measurement.

## 6. A/B: carrier sense re-enabled — GS-only REFUTED, both-sides helps

Both arms used the identical 6-climb protocol and were rolled back to
stock afterwards. One-line flips of `dev_cfg.tuning.disable_cca`
(`gs/src/radio_frontend.cpp:127`, `drone/src/main.cpp:683`); test
binaries kept: `maburgs.cca-test` on the GS, `maburd.cca-test` in the
drone's tmpfs `/tmp` (lost on reboot; rebuildable from the one-line
flip).

| rung | delivery CCA-off | GS-only CCA | both-sides CCA |
|------|------------------|-------------|----------------|
| 0 | 59% | 56% | **82%** |
| 1 | 55% | 46% | **76%** |
| 2 | 56% | 31% | **69%** |
| 3 | 51% | 23% | **66%** |
| 4 | 59% | 39% | **73%** |
| 5 (park) | 69% | 59% | **81%** |

close_ms: CCA-off n=24 median 64.5 / mean 72.5 / max 163; GS-only n=26
median **169.5** / 180 / 365; both-sides n=24 median **49.5** / 67.2 /
215 — with →5 boundaries collapsing to a tight 12–29 ms fast-path
cluster (12, 12, 19, 23, 26, 29). GS `tx_pps` ~20.5/s and `tx_fail` = 0
in every arm: every frame aired; all loss differences are at the drone's
receiver.

**GS-only is worse than nothing.** Deferral does not randomize
collisions, it ALIGNS them: the GS waits out each drone burst and
transmits the moment the medium goes idle, but a CCA-off drone with a
backlog launches its next MPDU right into the now-synchronized RCF.
Random-phase loss (p ≈ airtime) became aligned loss at burst
boundaries, worst at the densest rungs (rung 3: 51% → 23%). Any future
timing-based fix must target the middle of a verified gap, not the edge
of a burst.

**Both-sides is a real but partial fix**: +15–22 delivery points at
every rung and the →5 fast path restored, because the drone now defers
its next burst start while an RCF is on air — closing exactly the
aligned-collision mode. Residual 20–35% loss at climb rungs remains
(slot-aligned starts CSMA cannot arbitrate, plus mid-burst arrivals).
**Video cost on the clean bench: none measurable** — fps mean 59.5 /
min ~55 (climb transients), residual mean 0.0013→0.0016, RX pps
identical, ausniff clean throughout.

⚠ Decision caveat: this does NOT simply overturn the 2026-08-05 CCA-off
decision. The bench is a clean channel; the original rationale was
41–45% injection deferral against CO-CHANNEL 802.11, and both-sides CCA
re-exposes the video downlink to exactly that in congested
environments. What this measures is the OTHER side of that trade: on a
clean channel, CCA-off costs ~15–22 points of uplink control delivery
to self-collision. A congested-channel A/B (the mabur-side bench test
the 2026-08-05 spec still lists as not-run) is the missing piece before
changing the shipped default; a per-daemon split (CCA on for the GS's
20 pps control, off for the drone's video) is NOT viable — that is
precisely the GS-only arm refuted above.

Incidental observations: →1 boundaries exported close_ms samples in
both CCA-on arms but never under CCA-off (§5) — unexplained;
climb-transient residual spikes (~0.23 max) appear equally in all arms.

## 7. In-flight confirmation (2026-08-14, flight-0017/0018 + ctl-0054/0055)

Two ~6 min flights on the stock CCA-off PR #29 binaries confirm the
bench mechanism at range. RCF delivery vs the drone's airtime, pooled
per `link.air_pct` bin (both flights agree within a few points):

| air_pct | delivery (0017) | delivery (0018) |
|---------|------------------|------------------|
| 20–30% | 70% | 68% |
| 30–40% | 62% | 63% |
| 40–50% | 57% | 55% |
| 50–60% | 52% | 53% |
| 60–70% | 54% | 56% |

In-flight `close_ms`: n=53 median 97 / p95 289 / max 348 (0017);
n=56 median 76 / p95 258 / max 266 (0018) — the same fast-path +
50 ms-quantum geometric ladder, now with ~50% per-attempt loss at the
airtimes a flying link actually runs. Actuation latency in flight is
uplink-delivery-bound, full stop. (The <10% air bin shows ~35–38%
"delivery" — that bin is boot/landing windows where the drone is in
rendezvous and accepts nothing; ignore it.)

Same flights, for the record (fade wave, not this doc's subject): 26
loss-driven demote episodes across both flights, zero `fade`-reason
demotes — correctly, because these were ramp-type range fades whose
dual-EWMA deltas peaked at drssi −7.8 dB / dsnr −4.4 dB without ever
jointly crossing the −8/−4 thresholds; zero false fades, zero
attribution-miss canary hits, `link.attrib.suppressed` +3 (0017),
multi-rung cascades stepping at ~410–440 ms, and zero video-damage
windows (`residual_loss` never exceeded 5% mid-flight; fps continuous
59.5 outside boot/landing).

## 8. Repro / method

- close_ms sampler + forced climb: handover §"Repro recipe" (stop
  `S97flightrec` first; it was restarted after — recording resumed at
  `flight-0014.jsonl`).
- Uplink loss, no instrumentation: compare `sum(cards[].tx_pps)` against
  `drone.rcf.rx_pps` in any sideport recording; per-rung, bin by
  `link.ctl.rung.idx` against `link.air_pct`.
- Drain wake rate, no instrumentation: 10 s deltas of
  `voluntary_ctxt_switches` across `/proc/<pid>/task/*/status` plus
  `wchan` (§1).
