# Where the loss at a rung change comes from — bench spike + repeat-burst A/B (2026-09-05)

Companion to `docs/probe-stream-flight-findings-2026-09-05.md`. Flight-0023
(the first flight with the util+residual settle blank, 51dd79e) validated
the cascade fix — attribution-miss canary 0, s3-settle-refire canary 0, no
demote pair closer than 1.1 s — and exposed the next thing: two of six
promotes to rung 5 were demoted by the `probation` path 151/160 ms after
the E line, i.e. on the first controller tick after the 150 ms settle
blank, on a single ~60 ms util bucket reading `u` 1.0000 / 0.8000 while
the drone had applied the op ~35 ms after the E line and the probe canary
at mcs5 (rung 5's rate under same-rate pairs) had 4/4 blocks on every body
through the transition. The sideport showed ~100 base symbols REPAIRED
(not abandoned) across each promote, so the loss was pre-FEC and landed as
`recovered` — a class the transition watermark never splits (only
`abandoned` has a `_stale` side). Before designing around that, this spike
asked the prerequisite question the 2026-08-14 attribution spec left open
(§10 "root-causing the zero-completion windows"): **what physically loses
the burst at a rung change?**

## 1. Instrument

- `MABUR_GAPLOG=1` gap lines gained `prev_mcs= mcs= prev_sid=` — the RX
  MCS and stream id of the frames bracketing each per-card 12-bit seq gap
  (`gs/src/aggregator.cpp`), so a gap can be placed before/after the
  drone's rate switch without a wire change.
- `tools/bench/switchloss.py gaplog ctl probe au [pulser]` joins the gap
  lines with the ctl log's E lines, the probe log (the probe body's own
  MCS follows the applied op, so the first row at the new probe MCS —
  or the last row at the old one for a promote to the top rung — dates
  the drone's switch), flightrec's au log (IDR AUs = nal0 32, their
  `t_first..t_complete` = the IDR's air burst) and the `gstx` send stamps.
  Each gap is classed `pre` / `post` / `straddle` against the switch,
  flagged `IDR` when inside an IDR burst and `SEND` when a GS control send
  sits in `[gap_start − 2 ms, gap_end + 0.5 ms]` (the self-blanking
  signature of `docs/gs-uplink-self-blanking-findings-2026-09-02.md`);
  gaps inside a loss-sim pulse (`/root/s3pulser.py` on/off stamps) are
  excluded.
- Session (`/root/switchloss.sh <config> <tag>` on the GS): loss-sim
  gaplog build, 30 s climb to rung 5, then 8 × 150 ms `s1 eff=25 burst=3`
  pulses 20 s apart → 8 demotes 5→4 (`s3_residual`/`s3_util`) + 8
  re-promotes + the 5-rung climb = 21 transitions per arm. Bench SNR
  ~34 dB (clean).

## 2. Spike result — nothing intrinsic to a rung change loses frames

Arm A1 (ctl-0296, prod config, `rcf_repeat_copies` 3), 21 transitions:

| quantity | value |
|---|---|
| lost frames per transition, E..+600 ms (pulses excluded) | 4.1 |
| expected from the steady-state loss rate in 600 ms | 7.3 |
| gaps straddling the drone's switch | 0 |
| gaps inside an IDR burst | 0 |
| drone switch after the E line (probe-log estimate) | p50 48, p90 66, max 88 ms |
| transition-window gaps with a GS send in the collision window | 98 of 102 |

So the PHY-rate switch, the FEC re-key and the post-op IDR are all clean on
the bench: a transition adds no loss beyond the steady-state self-blanking
residual. The only transition-SPECIFIC loss is the GS's own **op-change
repeat burst**: the op-changing RCF plus 3 repeat copies 10 ms apart make
5–7 sends in the 100 ms after the E line (2.1 in a steady 100 ms), the
`RcfSlotter` holds the copies and releases them as one triple within
0.6 ms at an AU completion, the triple overruns the inter-AU idle into the
next aggregate's preamble, and both cards lose that aggregate. Signature:
a both-card `n=4` gap 13–42 ms after the E line with `SEND` set, on 5 of
21 transitions in A1 and 7 of 21 in A2.

What the bench cannot show: the flight's ~100-symbol post-promote burst at
22–25 dB. At 34 dB the util window reads 0 after every promote. By
elimination that burst was loss AT THE NEW RATE (marginal mcs5 plus the
send hits, landing on the first frames after the switch) — rate-based
attribution can never remove it, so extending the watermark to
`recovered` symbols (the obvious "fix the architecture" move) would not
have prevented the probation bounces. The bounce mechanism itself is not
range-specific either: the prod restart after arm B (ctl-0299, 34 dB)
reproduced it — promote 1→2, then `u` 1.125 on the +164 ms tick and a
`probation` demote back. The first post-blank bucket has a tiny
denominator (deliveries are booked at block completion, and after a
re-key the first blocks complete late), so a handful of repaired symbols
reads as 50–60 % loss. That is the decision-side guard's job (minimum
symbols / more than one tick before an instant demote) — separate
follow-up.

## 3. A/B — `rcf_repeat_copies` 3 vs 0, interleaved

| arm | copies | transitions losing a whole aggregate in E..+100 ms | lost frames / transition (E..+600) | sends in E..+100 ms | drone switch p50 / p90 / max | uplink delivery (drone `rcf.rx_pps` / GS sends) |
|---|---|---|---|---|---|---|
| A1 ctl-0296 | 3 | 5 / 21 | 4.1 | 5–6 | 48 / 66 / 88 ms | 92 % (p10 84 %) |
| B1 ctl-0298 | 0 | 1 / 21 (an MSP frame) | 3.3 | 2–4 | 48 / 65 / 78 ms | 90 % (p10 84 %) |
| A2 ctl-0300 | 3 | 7 / 21 | 5.1 | 5–7 | 47 / 78 / 90 ms | 92 % (p10 83 %) |

The repeats buy nothing on the bench: every one of B1's 21 single
op-change sends was applied within 78 ms (a lost one would show as a
switch delayed by a full 50 ms `feedback_ms`, i.e. >120 ms; none did).
Steady-state uplink delivery is the same in every arm because repeats only
ever fire at op changes. At range the flight numbers apply: flight-0023's
drone heard a median 93 % of GS control sends (68 % at the worst moment),
so without repeats ~1 op change in 14 waits one extra feedback period
(~1 in 3 at the far edge) — against a whole aggregate lost on both cards
on 25–33 % of ALL transitions with them.

## 4. Decision — repeat burst REMOVED (not zeroed)

Per the compatibility policy (delete dead weight): `link.rcf_repeat_copies`
and `link.rcf_repeat_ms` are gone and now FAIL BOOT (strip them from
`/etc/maburgs.json` BEFORE swapping the binary — config-before-binary);
`VrxController::poll_repeat`, its arm-on-change state and the main-loop
repeat drain are deleted; 9 tests that pinned the burst are deleted with
it. The 2026-08-14 findings doc and `docs/tx-rx-timing.md` keep the
history. If repeats ever come back, release them one per AU completion,
never batched.

Steady-state loss on the bench (12–14 lost frames/s, 70 % with a send in
the collision window, `feedback_ms` 50) is the known slotter residual
(`docs/probe-blanking-fix-findings-2026-09-05.md`), unchanged by this.
