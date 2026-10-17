# With demand refresh, what does the phase lock buy, and what stands between it and a flat ~35 ms?

**Status:** spike, nothing shipped. Follows
`2026-10-17-phase-lock-spike.md` (the lock mechanism, and the GOP-excess
verdict that demand refresh has since removed). The regulator's two
placement modes are described in `docs/observability.md` ("Regulator
line": `chained`/`chain` and `alw`/`lslot`), demand refresh in
`docs/fec-nack.md`. The throwaway servo and the drone verb it drives live
only in the machine-local spike directory.

## Question

The first spike found the GS-side sensor-rate lock cheap and stable but
pointless for smoothness, because the periodic refresh picture's excess
(15-21 ms) could not fit inside a display period at any phase. Demand
refresh (`venc.gop_s` 0) makes that picture rare. With it in place: does
the lock now give a flat, smooth stream, at what hold, and with which
regulator mode?

## Setup

Bench Radxa ZERO 3 and the bench drone on the demand-refresh build, rung
4 (mcs4/40, ov 0.50), static daytime scene, `vsync_lead_ms` 5,
`chain_budget` 0, one GS session (0143, five windows of ~5 min each). The
drone ran the spike verb (`snr_mfps`, applied from the venc hot thread);
this boot's sensor trim locked at 59718 mfps, 59728 is one line faster.
The servo is the first spike's bang-bang on the typical frame's hold in
its natural slot, `(t_flip − lead − t_decoded) mod P`, plus dead
reckoning at ±0.15 ms/s across the stretches where `frame.log` arrives in
lumps (its idle-gated burst came as late as 38 s in one piece).

| window | lock | `sched_cap_ms` | e2e p50/p90/p99 | judder s | never-shown /s | chained s |
|---|---|---|---|---|---|---|
| baseline | — | 0 | 45.5 / 51.3 / 52.8 (sawtooth 36→52) | 2 % | 0.01 | 205 / 308 |
| D | hold 15.5 | 0 | 37.4 / 38.4 / 53.2 | 8 % | 0.07 | 1 / 300 |
| E | hold 11 | 0 | 49.5 / 50.9 / 51.8 | 1 % | 0.00 | 275 / 309 |
| F | hold 11 | 16 | **35.2 / 36.1 / 50.0** | 2 % | 0.18 | 0 |
| G | hold 8 | 16 | 46.4 (bistable 31 ↔ 47) | 7 % | 0.46 | 0 |

Judder = `lat.log` seconds with p99 ≥ p50 + 10 ms; never-shown = AUs in
`au.log` with no `frame.log` row. No window had a refresh picture, a pts
hole or a `dsp` over 15 ms; the ready-time spread (capture → decoded)
was the same in all five: p1/p50/p99/p99.9 = 15 / 18 / 25.5 / 33 ms.

## Result

**The GOP obstacle is gone.** Five minutes of clean link produced no
refresh picture at all, and the no-lock baseline's judder is 2 % (it was
15 % at gop 0.5 and 5 % at gop 10), all of it at the sawtooth's wrap.

**The arrival-path regulator turns a mid-period lock into +1 period.**
A frame's natural slot is the first release after it arrives; once a
predecessor holds that slot, every successor takes the next one, and the
chain only ends when a frame arrives after its predecessor's delayed
release (hold near 0, a repeat) or the deep-burst displacement claims a
slot (a skip). With the lock holding the typical frame mid-period
nothing ever ends it: run E is smooth but shows every frame a period
late (`hold_ema` 27 ms = 11 + P). Locking at the top of the period (run
D) keeps the chain broken, but the early tail then wraps past the
period and is shown a slot early — 3961 wrapped frames and 8 % judder.
Across the baseline's beat the same two states alternate, which is the
36→52 sawtooth. So on this path the lock can choose between the
sawtooth's two ends, never anything better.

**Capture-time placement is the regulator mode for a locked arrival.**
With `sched_cap_ms` 16 the slot follows the capture clock plus the
measured delivery floor and allowance (9.8 ms here), a late frame is
displaced by its on-time successor and no chain forms. Run F: 35.2 ms
flat (p90 36.1) against the baseline's 45.5 average, with the baseline's
2 % judder. The lock target still matters, but for a different reason:
it sets where `want` falls on the vblank grid. At hold 8 (run G) the
margin between `want` and its release is 0-3 ms, so the window's
floor/allowance jitter moves the whole stream across the edge and e2e
flips between 31 and 47 in 10 s stretches; hold 11 keeps ~4 ms of
margin and is stable.

**The cost of run F is a maburgs stall, not the link.** Its 52 skipped
frames (0.18/s) have a normal arrival span (4.2 ms) but
`t_drain − t_last` of 11.8 ms p50 / 20.7 p90: the body waited in the RX
queue for the core thread. The same stalls happen at 0.17-0.20 events/s
in every window, including the baseline, one AU each, 8-26 ms, uniform
within the second and uncorrelated with the 1 Hz log stamps (and
`LogWriter` already takes the session logs off the core thread). The
regulator chain's full period of slack had hidden them; a 35 ms lock has
no slack to hide them in.

## Answer

Demand refresh removes the first spike's objection. The lock plus
capture-time placement gives a flat 35 ms with no more judder than the
unlocked stream, which is what the lock was for. Two things stand
between that and shipping it:

1. The maburgs core-thread stall (0.2/s, 8-26 ms). Find it first —
   instrument which `poll` in the core loop owns the `t_drain` gap — or
   every one of those AUs becomes a visible skip at 35 ms.
2. The design: the servo belongs in maburplay (it has the per-frame
   stamps), the rate request travels in the RCF (an `RC_VERSION` bump)
   and the drone applies it from the venc hot thread as the verb does;
   capture-time placement becomes the regulator mode under lock, with a
   floor/allowance that is steady enough to let `want` sit closer to the
   release than 4 ms.

Below the lock, the ready-time spread itself (10.6 ms p99 − p1) sets the
floor at roughly 33-35 ms; the sensor-rate lock cannot go under it.
