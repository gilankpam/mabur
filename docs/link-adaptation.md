# Link adaptation — ladder, attribution, fade

How maburgs decides the operating point: the measured-loss ladder, s3
probe-before-promote, transition attribution, the fade regime and its
predictive RF trigger, and the drone-side RCF drain that actuates it.

Read `docs/data-provenance.md` before comparing any two recordings across
dates, and `docs/observability.md` for how to record one in the first
place. Specs (local, gitignored):
`docs/superpowers/specs/2026-08-05-s3-probe-promote-design.md`,
`docs/superpowers/specs/2026-08-15-pooled-rf-and-instant-s3-design.md`.

## 2-stream UEP + air balancer (2026-08-29, superseded 2026-08-30 — see below)

The video link collapsed from 4 UEP streams (CRIT/T0/T1/T2, wire sids
0-3) to 2: BASE (sid 0) and ENH (sid 1). BASE rode `mcs−1` of the
ladder's scored rung — an always-on, fixed UEP-via-rate rule mirrored
identically on drone and GS (RC_VERSION 4) — while ENH rode the scored
mcs itself; nothing flew above the scored mcs (2026-07-26 rule, still in
force). FEC overhead was LITERAL: the config/wire `overhead` value
*was* the command overhead (`repair/data`), not a per-layer scaling
(the old `uep_layer_overhead`) of it — a single `budget()`/`budget_for(rung)`
pair derived directly as `overhead / (1 + overhead)`, the identical
formula for both sids; `budget3_for()` was gone, `budget_for()` alone
covered what it used to do (the enh/probe rung's budget).

The drone's `AirBalancer` redistributed that one commanded overhead
between BASE and ENH every frame, anchored on ACTUAL emitted bytes
rather than the nominal `len*(1+ov)/rate` model (which measured a
reproducible ~2x gain error at large frames). See
`docs/airtime-balance-spike-findings-2026-08-29.md` for the bench spikes
behind that design. The rate split had a structural floor, though: at
the mcs1 rung's 2:1 base/enh rate ratio the balancer's rails could not
reach air balance (3.3 ms/pair residual alternation) — the motivation
for the same-rate counter-study below.

## Same-rate fixed pairs (2026-08-30) — current architecture

Both streams now ride the SAME scored mcs; the mcs−1 base-rate split
above is gone (`ladder_from` in `common/src/profile.cpp`, 2026-08-30
RULING). UEP is expressed only through FEC overhead, and that overhead
is a fixed **per-rung config pair** — `overhead_base`/`overhead_enh` —
carried in the v5 RCF (RC_VERSION 5), not a single scalar the drone
reallocates at runtime. `LadderController` scores each sid against its
own budget: `budget_base()`/`budget_base_for(rung)` for sid 0 (base),
`budget_enh_for(rung)` for sid 1 (enh/probe/s3) — see
`gs/src/ladder_controller.h`. There is no shared `budget()`/
`budget_for()` anymore.

**The runtime `AirBalancer` solver is deleted, and so is `AirFeed`.**
The drone applies the commanded overhead pair directly to UEP; there is
no per-frame redistribution or solve. `AirFeed`, the solver's
measurement-only successor, kept per-stream EWMAs of frame-unit bytes
vs. emitted body bytes and published `share_base`/`excess_base`/
`excess_enh` into `run_bitrate_policy`'s blended target; it was deleted
on 2026-09-01 when that blend became a fixed per-rung formula, because
every bitrate write it provoked cost a keyframe (see
`docs/airtime-model.md` §1). `run_bitrate_policy` is once again a pure
function of the operating point: a held rung commands a held bitrate.

The drone's `:8301` `ov_base_pct`/`ov_enh_pct` HTTP override (bench
tooling, not a production control) is now the ONLY source of
commanded-vs-applied divergence: with no override armed, applied always
equals commanded. `tools/maburtop.py`'s DRONE and LADDER panels reflect
this (`_ov_cmd_cell`/`_ov_applied_cell`) — a mismatch there means an
armed override or a stale/old-daemon snapshot, not a balancer at work.

Measurement basis for the same-rate decision: ten static operating
points plus a motion sweep, `docs/same-rate-uep-findings-2026-08-30.md`.

Everywhere below that still says "s1" or "s3" is pre-2026-08-29
vocabulary for what is now sid0 (BASE) and sid1 (ENH — still the
probe/canary layer the ladder attributes demotes to). The demote-input
names, the surviving config key `s3_settle_ms`, and ctl-log reason
strings (`s3_residual`, `s3_util`) were kept as-is rather than renamed
(see `gs/src/ctl_log.h`'s ctllog 7 note) — read every s1/s3 mention past
this point as sid0/sid1. (`s3_residual_confirm_ms` itself is gone, but
that removal predates this change — see the pooled-RF note below.)

## RCF slotting into the inter-AU idle (2026-09-03)

Every GS control-frame send (RCF, DISC keepalive, repeat copies) blasts the
sibling RX card at ~−4 dBm and deafens the TX card, so a drone PPDU whose
preamble starts within ~180 µs of the send is lost on BOTH cards — one
whole aggregate per hit, the source of the bench's ~0.35 %/PPDU loss and
the 34 ms tail-repair stalls (`docs/gs-uplink-self-blanking-findings-2026-09-02.md`).
The chip already holds the TX until the PPDU it is receiving ends; what
kills is the NEXT PPDU starting right behind the blast, i.e. sends inside
an AU burst.

`RcfSlotter` (`gs/src/rcf_slot.h`) sits between `VrxController` and
`send_control`: while video flows, a control frame waits for an AU
completion at which the send will be on air before the next AU's burst is
due — predicted from the first-body arrival cadence (`t_first` of
consecutive AUs is flat to <1 ms; completion is not, it moves with frame
size and FEC repair). Completions too close to the next burst are skipped;
a frame offered within 2 ms after a good completion goes out at once; a
hold longer than `link.rcf_slot_hold_ms` (default 30, 0 = off) is released
anyway. With no AU in the last 100 ms (rendezvous, stalled video)
everything passes through, which is why DISC needs no bypass.

Bench A/B (mcs5 park, 11 Mb/s, agg 6, 100 s windows, gap-log instrument):

| | real losses/s | frames lost/s | RCF rx at drone | `close_ms` |
|---|---|---|---|---|
| slotter off | 1.03 | 3.9 | 18.5/s | 5 |
| slotter on | 0.11 | 0.4 | 19.4/s | 22 |

Sends: 88 % released by an AU completion (hold p50 6 ms, p90 17 ms), 12 %
in-grace immediate, 0.5 % hold-timeout. Cost: a commanded op change now
reaches the drone up to one AU period later (`link.attrib.close_ms`
5 → 22 ms). Player `fec` p99 ≥ 20 ms windows: 31/90 → 7/100.
Sideport counters: `link.rcf_slot.{au,timeout,passthru}`.

## Tuning invariant

Tuning invariant: the controller's s3 loss/residual
windows are 500 ms wide, while the post-transition blanking
(`s3_settle_ms`, default 300) and probe settle (`probe_settle_ms`, 150)
are shorter — so up to ~200 ms of pre-transition symbols remain in view
after blanking expires. Shipped defaults are safe (stale weight decays
fast against the 250/500 ms confirm windows), but do NOT lower
`s3_settle_ms`/`s3_residual_confirm_ms` toward their floors together: a
rung transition's FEC re-key artifacts could then satisfy the s3-residual
confirm and self-demote on every promote. ⚠ SUPERSEDED 2026-08-15 — see
the pooled-RF note below: `s3_residual_confirm_ms` is REMOVED and FAILS
BOOT, so there is no longer a config knob to lower — the s3-residual
confirm window this paragraph warns about is now permanently at 0 ms
(its floor), unconditionally, for every deployment. That is deliberately
NOT the unsafe floors-together configuration described here: it is safe
only because attribution is exact rather than fast (the watermark is in
symbol-sequence space, so debris is absent from the input rather than
outrun by a shorter window) — see the 2026-08-15 note for the residual
risk that distinction does not cover.

Since 2026-08-14 the ladder's demote inputs are transition-attributed
(kill switch `link.attrib`, default true. ⚠ SUPERSEDED 2026-08-15 — see
the pooled-RF note below: `link.attrib` is REMOVED and now FAILS BOOT,
attribution is unconditional, and this is no longer a kill switch):
per-stream watermarks — with
the RX PHY rate as the generation boundary — split every loss counter
into current-rung vs pre-transition debris, and all four demote inputs
(instant s1 residual, s3 residual, both utils) read the current-only
side, so a rung change's own FEC debris can no longer fire a follow-up
demote. The sideport reports `link.streams[].abandoned_stale`; the ctl log
went `ctllog 1` → `ctllog 2` (S line gained `resid_cur`; `resid` stays the
total). ⚠ It also reported `link.attrib.suppressed` until 2026-09-02, when
that counter was deleted along with the packet-level delivery window it was
defined against — it counted windows where the packet total and attributed
views disagreed, which the symbol-based measure cannot ask.
`flightreport.py` parses every version. Date recordings against this
line: pre-2026-08-14
residual/util figures include transition debris that later recordings
attribute away. `link.attrib: false` reverts the decisions (not the
bookkeeping) to the old totals. ⚠ SUPERSEDED 2026-08-15 — see the
pooled-RF note below: `link.attrib` no longer exists, so this revert path
is gone too — the config key FAILS BOOT and the only way back to
pre-attribution decisions is a binary rollback.

**Since 2026-08-14 (same day, second wave) demotes are fade-aware.** Two
independent pieces, both default-on, both killable, and the whole
`link.fade` block is optional with working defaults. Every loss-driven
demote — residual, s3 residual, s3 util, confirmed util, and a fade
demote itself, but NOT probation, starved or timeout — arms a 2.5 s fade
regime (`hold_ms`), and arms it UNCONDITIONALLY so that the exported
regime state stays truthful. `link.fade.cascade` gates only the effect:
while the regime is open and the cascade is on, the demote confirm
windows drop 250/500 ms to 100 ms (`confirm_ms`), so a real fade steps
down at fade speed rather than at steady-state speed. Kill the cascade
and the regime is still armed and still reported — it just stops
shortening anything. ⚠ SUPERSEDED 2026-08-15 — see the pooled-RF note
below: `link.attrib` is gone and so is this gate. The paragraph below
describes 2026-08-14 second-wave behaviour only; since 2026-08-15 the s3
residual path has no confirm window to gate at all (it demotes instantly,
unconditionally), and the s3 util confirm — the only one left — always
runs at `in_fade_regime(now_ms) ? fade.confirm_ms : confirm_ms`, with no
attrib-off branch. Both s3 confirms were additionally gated on
`link.attrib`: with attribution OFF the regime kept the full
`s3_residual_confirm_ms` / `confirm_ms` there, because a 100 ms confirm
behind the unshortened 300 ms `s3_settle_ms` is exactly the
floors-together configuration the tuning invariant above forbids — the
~200 ms of debris that outlives the blank satisfies 100 ms and not the
legacy window. Measured in review, a single genuine demote then cascaded
rung 4 → 0 in 1.6 s on nothing but its own FEC debris. So `attrib: false`
used to revert Part A's s3 paths along with everything else. (The s1 util
confirm is NOT gated: it has no blanking at all, so its legacy 250 ms
window already sits inside the same 500 ms loss window the debris
occupies — amplitude decides it there, not duration.)

`link.fade.predict` adds an RF trigger ahead of any loss: both `rssi_db`
(8 dB) AND `snr_db` (4 dB) below their slow baselines, sustained
`trigger_ms` (300 ms), demotes one rung with reason `fade`. Those
baselines are a dual-timescale EWMA — fast tau 300 ms, slow tau
asymmetric at 2 s rising / 20 s falling; structural constants, not config
— so a multi-second fade cannot drag its own baseline down and erase its
own delta. ⚠ **Those two numbers are thresholds on a high-pass response,
not fade depths, so they are NOT trip points.** A step of depth D reaches
`delta = D·(e^−t/20000 − e^−t/300)`, which peaks at 0.92·D at 1.28 s and
is down to 0.74·D by 6 s, so the smallest step that fires is ≈1.08× the
configured number: `rssi_db: 8.0` trips on a ≈8.7 dB fade, `snr_db: 4.0`
on a ≈4.3 dB one, and a fade of exactly 8/4 dB never fires. A fade that
stops descending falls back under threshold as the baseline catches up
(this detects fading, not faded), and on a steady ramp the response is
slope-driven — ~19.7 dB of delta per dB/s — so ramps under ~0.45 dB/s
never reach `rssi_db` 8 however deep they eventually get. `trigger_ms` is
not the binding constraint either: the delta needs ~0.8–1.3 s to climb to
its peak, so the 300 ms fast tau sets the reaction time. Those are
harness/model figures (they reproduce the review's measured deltas
exactly) — and the defaults are deliberately unchanged, since tightening
them without bench data is what the spec's tuning invariant forbids.
**First flight validation, 2026-08-14 (flight-0017/0018 + ctl-0054/0055,
two ~6 min flights):** exactly the predicted behavior. 26 loss-driven
demote episodes, all ramp-type range fades; the predictive trigger
correctly never fired (deepest deltas drssi −7.8 / dsnr −4.4, never
jointly over threshold — slope-blind on ramps as the transfer function
says), zero false fades, zero attribution-miss canary hits, in-regime
cascades stepping multi-rung episodes at ~410–440 ms, and zero
mid-flight video-damage windows. A genuine FAST fade (obstruction,
multipath null) has still never been recorded against this trigger. Three things to know before reading any of it: (i) the
predictive trigger is LATCHED — exactly ONE predictive demote per fade
EVENT, and the latch releases only on an *observed* recovery, a tick where
both deltas are measurably back under threshold. A NaN window (absent
evidence) deliberately does NOT release it, so further steps during a
continuing fade are the cascade's job, on measured loss at the shortened
in-regime confirms; `link.ctl.counters.demotes_fade` therefore counts fade
events that produced a step, not rungs lost to fade. (ii)
`link.fade.min_rung` (default 2) is the lowest rung the trigger fires
FROM, not a floor — the effective floor is `min_rung - 1`, so the shipped
default can land the link on rung 1. (iii) fade demotes are RF evidence,
not rung evidence: they never book a probation failure or a penalty and
never count in the RungStore's `exits_bad`.

Observability for that wave: the sideport adds `link.ctl.fade` = {active,
drssi, dsnr} plus `counters.demotes_fade` (additive under `v: 1`;
drssi/dsnr serialize as `null` when NaN). `fade.active` is the RAW regime
state and is deliberately NOT gated on `cascade`, so the regime stays
visible with the cascade killed. The ctl log went `ctllog 2` → `ctllog 3`,
the S line gaining `drssi dsnr` after `resid_cur`; `flightreport.py`
parses v1, v2 and v3 and gained an episode analyzer (`find_episodes()`,
`print_episode_report()`): first-demote reason per episode, fade lead
times, false fades with time-to-repromote, plus an attribution-miss canary
(`attribution_misses()` — a `residual` demote within 200 ms of any
previous transition, which should be ~zero with `link.attrib` on). The
jsonl branch also prints the flight-wide `link.attrib.suppressed` delta for
recordings old enough to carry that key (removed 2026-09-02).
⚠ The s1 RF labels are now freshness-gated per card
(`gs/src/rf_labels.h`, `select_label_card()`, unit-tested in
`tests/test_rf_labels.cpp`): the best-card argmax only considers cards
whose s1 frame count advanced in the current feedback window, so
`s1_snr_db`, `s1_evm_db` and `s1_rssi_dbm` read NaN — and the ctl log
prints `nan` — whenever no card measured s1 that window, where a frozen
EMA previously printed a stale-but-present number. Because `s1_evm_db`
now NaNs on stale windows, per-rung EVM sample counts in the RungStore
drop on a marginal link: that is a deliberate honesty improvement, but it
means EVM sample counts are NOT comparable across this date.

**The expected false-fade source is a label-source card hop, and it is
what to look for when a `fade` demote has no fade behind it.** That
argmax does not stick: a front-end that wedges for ~1 s (a documented,
recovering failure mode on the two-card bench GS) hands the labels to a
weaker sibling, and `s1_rssi_dbm`/`s1_snr_db` then step down TOGETHER —
bit for bit the trigger's joint condition, so a ≥9 dB RSSI / ≥4.3 dB SNR
gap between cards is a spurious `fade`. ⚠ SUPERSEDED 2026-08-15 — see the
pooled-RF note below: the defence described in the rest of this paragraph
is DELETED, not merely inactive. The operational guidance (how to
recognise and correlate a card-hop false fade) below is unchanged and, if
anything, matters more now, because this branch makes a hop-driven false
fade possible for the first time — read it as "what to look for", not
"why it can't happen". ~~The controller defends itself: the selected card
index rides along in `LinkHealth` and a change re-baselines both EWMAs
(so a hop reads as a new reference, not a fade), at the cost of making a
fade already in progress re-accumulate its delta on the new card —
conservative in the direction everything else here is.~~ The latch is
deliberately NOT released by a hop. If a false fade shows up anyway,
correlate the ctl log's `drssi`/`dsnr` step against per-card
`classes.s1.*` in the sideport: a hop moves both by the card GAP in one
window, a real fade moves them along the transfer function above.

**Since 2026-08-15 the RF labels are s1+s3 pooled, the card-hop
re-baseline is gone, attribution is unconditional, and s3 residual
demotes instantly.** Four coupled changes, spec
`docs/superpowers/specs/2026-08-15-pooled-rf-and-instant-s3-design.md`.
(i) `s1_snr_db`/`s1_evm_db`/`s1_rssi_dbm` became `rf_*` and are sourced
from a new per-card s1+s3 pooled track (97% of frames at one PHY rate);
`msp`/`ctrl` are excluded because per-rate TX power makes their
contribution depend on a mix ratio that drifts with rung and shed state.
⚠ This is a second discontinuity in RungStore's per-rung EVM baselines,
on top of the 2026-08-14 freshness gate — EVM sample populations are NOT
comparable across either date. ⚠ The jsonl sideport carries no version
marker at all (the ctl log's header bump doesn't reach it): the same
pooled-track change silently flips the meaning of
`link.ctl.last_event.snr`/`.evm` and `link.ctl.last_probe.snr`/`.evm` from
s1-only to s1+s3 pooled, with nothing in the jsonl itself to say so — date
any jsonl recording against 2026-08-15 the same way you would a ctl log's
`ctllog` header. (ii) The card-hop EWMA re-baseline is
deleted: it was zeroing `drssi`/`dsnr` on 25% of ticks (372 of 1483
across flight-0017/0018), so **every fade delta recorded before this date
is suppressed and must not be pooled with later ones** — the trigger
could not fire regardless of fade depth, which is a second explanation
for its silence alongside ramp slope-blindness. Its premise was also
wrong: the argmax runs on SNR, so the SNR label is `max(snr)` over live
cards and is continuous across a hop; only RSSI stepped, and measured hop
steps were indistinguishable from ordinary variation. (iii) `link.attrib`
is REMOVED and now FAILS BOOT; attribution is unconditional and there is
no config rollback, only a binary one. `residual_cur` / `close_ms` remain
on the sideport (`suppressed` was removed later, 2026-09-02), but
`link.attrib.on` is REMOVED — a `v: 1` schema removal in the same class as the 2026-08-12
`offset_qdb` removals, with `maburtop.py` updated in the same wave.
(iv) `link.s3_residual_confirm_ms` is REMOVED and FAILS BOOT: s3 residual
now demotes on the first window, exempt from `min_between_changes_ms`,
like s1's. That is safe only because attribution is EXACT rather than
fast for debris the transition watermark actually classifies as stale —
the watermark is in symbol-sequence space, so that debris is absent from
the input rather than outrun — and `s3_settle_ms` blanking is retained.
⚠ That does NOT cover every pre-transition case: genuine current-rung s3
abandonment that was correctly attributed to the OLD rung can still sit
inside `s3_residual_loss`'s underlying 500 ms sliding window (never
cleared at a transition, only blanked from the decision) and fire a
follow-up instant demote at the tick `s3_settle_ms` expires — pre-existing
behaviour, bounded to ~2 firings, not something this branch changed but
worth knowing when reading `demotes_s3_residual`. Predicted cost ~4× that
path's demote rate (~19 events per two 6-minute flights vs the 5
observed); materially above that on the first flight has two candidate
causes that want different responses — the estimate's 500 ms sampling
hiding back-to-back firing (confirm window returns in reduced form), or
the sliding-window mechanism just described (bound/blank the window at a
transition instead). Count s3-residual demotes landing within roughly
`s3_settle_ms` + one tick of a PREVIOUS transition separately to tell
them apart; `flightreport.py`'s `find_episodes()` already has the
machinery. The ctl log went
`ctllog 3` → `4` (formats byte-identical, meanings changed);
`flightreport.py` parses v1–v4 and warns on pre-v4. Deploy is GS-only and
config-before-binary: `grep -nE '"(attrib|s3_residual_confirm_ms)"'
/etc/maburgs.json` and delete any hit before starting the new binary,
or maburgs crash-loops at 2 s. **Also swap `tools/maburtop.py` in the same
step, not as an afterthought:** an old maburtop against a new maburgs
renders the now-absent `link.attrib.on` as `attrib:OFF` — indistinguishable
from a real problem, and exactly the kind of thing that sends someone
hunting for a switch (`link.attrib`) that no longer exists and would fail
boot if they tried to set it. Deploy maburtop alongside the binary, not
after. Rollback for this wave happens to be binary-only: both `attrib`
and `s3_residual_confirm_ms` were optional with live defaults on the old
binary too, so the stripped config boots either way and there is nothing
to restore alongside the binary.

**Since 2026-09-02 (second wave, same day as ctllog 9) the s1 residual
demote input carries a 150 ms post-transition settle blank**
(`S1LossWindow::blank_until`, wired at `main.cpp`'s sid-0 transition
edge-detect next to `mark_transition`; the constant is `kResidSettleMs`,
not config). Root cause, from flight ctl-0160: the ctllog-9 rewrite fed
block 4 from a 500 ms sliding window over the cumulative abandonment
counters and deleted the old measure's per-step `reset_window()`, so loss
correctly booked as current-rung stayed `> 0` across the demote it caused
and — block 4 being instant, threshold-free and exempt from
`min_between_changes_ms` — re-fired every 50 ms tick until rung 0. Every
`first=residual` episode in that flight ran 4-5 rungs to the floor, one
at 26-32 dB SNR; flightreport's attribution-miss canary read 14.
Attribution itself was innocent (it classifies at booking time; already-
booked current loss is never reclassified). The blank clears the window
at the transition AND swallows deltas booked during the settle — the
~80 ms abandonment-horizon lag books old-rung loss late, and an A-MPDU
burst can kill the in-flight old-rate tail ABOVE the watermark, which
books non-stale. 150 ms = one edge-detect tick + horizon lag + margin,
half of `s3_settle_ms`; the s1 UTIL path keeps its no-blank design
(amplitude decides it) and still carries a genuine sustained fade down at
in-regime speed. This is the same window-outlives-the-blank hazard the
s3 note above bounds with `s3_settle_ms` — s3's window is deliberately
NOT blanked here (its ~2-firing bound is documented, accepted behaviour).
Only the decision input is blanked: S-line `resid`/`resid_cur`, the
sideport and the RungStore observability all still see the loss.
Bench-validated 2026-09-02 (ctl-0165, loss-sim build): a 500 ms
eff=20/burst=4 s0 pulse = ONE residual demote + two measured-util steps
at ~155 ms, floor at rung 2, repromote — where ctl-0160's equivalent was
a 50 ms/rung drop to rung 0; sustained eff=15 still walks 5→0 (util,
150-320 ms/rung) and recovers; ausniff 60.0 fps / 0 gaps. Expect
`residual` E-line pairs closer than 150 ms never again; a demote storm
now shows `util` reasons and real per-rung `u`.

On the drone, RCF drain is decoupled from the agent tick
(`link.rc_drain_ms`, optional, default 5, bounds 1–1000): the agent loop
wakes every `rc_drain_ms` to drain queued RC frames, with ALL per-tick
housekeeping (USB health polls, `RcAgent::tick()`, watchdog, 1 Hz
stats/telem) behind a `TickGate` deadline so its cadence is bit-for-bit
unchanged, and `rc_drain_ms == tick_ms` reproduces the legacy loop
exactly. `link.tick_ms` is now bounded 1–1000 as well, and
`rc_drain_ms > tick_ms` FAILS BOOT (it would silently retime every
per-tick job to the drain period): the gate turned a bad `tick_ms` from
the old "spins at 100% CPU but works" into a ~1.8e19 ms period that fires
once at startup and never again — no failsafe, no rendezvous fallback, no
watchdog, no telemetry, nothing logged. Both bounds are new on
2026-08-14; note that a config setting `tick_ms` under 5 without also
setting `rc_drain_ms` now fails boot, since the default drain of 5 would
exceed it (nothing deployed does this). Op actuation used to be U(0, `tick_ms` = 100) ms, and
`link.attrib.close_ms` measured a ~110 ms median with tails at 295 and
971 ms. **Measured 2026-08-14 (follow-up session): the drain runs at 5 ms
on the device (verified via /proc thread wake rates), but close_ms median
is ~65 ms at n=24, not ≤30 — because 30–50% of uplink RCFs are lost to
the drone's own half-duplex TX airtime (CCA off, GS injects blind into
the drone's bursts; loss tracks `link.air_pct`, 51–59% delivery at climb
rungs 0–4, 69% parked). A lost commit-RCF costs one `feedback_ms` (50 ms)
quantum, so close_ms = an 11–28 ms fast path (Part C working, target met)
plus a geometric +50 ms ladder that Part C cannot touch. The ≤30 ms
acceptance criterion is unreachable without an uplink-delivery fix
(candidates, none built: repeat the RCF after an op change until
`drone.applied` echoes it; time injection into post-frame-burst gaps).
The same loss applies to ALL uplink control — probe RCFs, keepalive DISC,
IDR requests — and `feedback_ms` is effectively the uplink retry quantum.
Full analysis: `docs/rcf-uplink-loss-findings-2026-08-14.md`.** `link.fade`
and `link.rc_drain_ms` are optional with live defaults, so the new binary
runs against an untouched config on either device. Once either is
hand-tuned into `/etc/maburgs.json` or `/etc/mabur.json` — and the bench
GS is exactly the machine that will tune `link.fade.rssi_db` — that config
stops loading on an older binary (`unknown key` → the 2 s crash-loop
described further down). Write the key anyway when tuning wants it; that
is a rollback cost, and rolling forward is the answer. The former rule
that `bundle/mabur.default.json` must NOT list `rc_drain_ms` was purely an
old-binary concession and no longer applies.



## RcAgent owns the encoder (venc fold-in, 2026-08-29)

The drone-side actuator used to be an HTTP client against `waybeam`. Since the
fold-in the encoder runs inside `maburd`, so `RcAgent`'s three verbs
(`set_bitrate_kbps`, `set_roi_qp`, `request_idr`) are direct calls into
`venc_core` on the agent thread. Nothing about the *policy* moved: the venc
core is a pure mechanism with zero encoder-local policy, and there is no
`venc.bitrate` config key — the commanded rate is only ever the output of
`run_bitrate_policy()`, i.e. `phy_rate(T0) × encoder.airtime_budget /
(1 + overhead)`, clamped to `encoder.bitrate_min_kbps`/`bitrate_max_kbps`.

What the fold-in did change:

- **Failed verbs are now visible and retried.** Each verb returns real status
  and RcAgent latches "what the encoder is running" only on success, so one
  dropped MI call is re-issued on the next policy tick instead of wedging the
  rate for the rest of the flight (that wedge was the old waybeam failure
  mode, and it is why the latch is conditional).
- **RcAgent is the only IDR authority, and it paces.** Every producer —
  GS-requested IDRs, the entering-LINKED heal, and the encoder's own
  chain-break signal — goes through one pacer: a 100 ms floor between any two
  IDRs, plus a 1 s holdoff between chain-break IDRs specifically. A refused
  request is DROPPED, not queued; the next real break re-raises it. The
  chain-break path is an atomic flag set from the venc callback and consumed
  at the top of `tick()`, evaluated against the state as of tick entry so a
  break that arrives on the same tick as a missed feedback deadline still
  heals. Dropping rather than deferring is affordable only because the
  encoder's GOP is the backstop: at the shipped `venc.gop_s = 2.0` an
  unhealed break self-clears within ~2 s, so raising `gop_s` stretches that
  safety net and the drop-vs-defer choice needs re-arguing. Measured on
  hardware 2026-08-29 under a deliberate ring-full storm (~24 drops/s): 0.596
  IDR/s, minimum observed spacing 919 ms — an unpaced path would have emitted
  roughly one IDR per drop.
- **The ring is now visible from both ends.** `drone.enc.venc_ring_fill_pct`
  and `drone.enc.venc_full_drops` report the PRODUCER side (the encoder
  discarding AUs because maburd had not drained), against the existing
  consumer-side `drone.enc.ring_drops`. See `docs/observability.md`.

### The bitrate policy pushes on CHANGE, plus a 5 s re-assert

`run_bitrate_policy()` only calls the encoder when its computed target
differs from the last value actually applied (decreases always go out
immediately; non-decreases are throttled to 1 Hz; state transitions force).

On top of that, `RcAgent::tick()` **re-asserts** the current computed target
every `RcAgent::kReassertMs` = **5000 ms**, with `force=true` (a `force=false`
re-assert would be a no-op by construction — an unchanged target is exactly
what the change gate suppresses). The clock runs from the last bitrate the
encoder actually *accepted*, so a link that keeps genuinely changing rung
never adds a re-assert on top of its own pushes; a parked one gets one every
5 s. The re-assert is gated to LINKED and FAILSAFE — RENDEZVOUS is the
pre-link state with nothing to defend the value against.

This closes two holes:

- **A failed verb in FAILSAFE is now retried.** `run_bitrate_policy()` latches
  its "last commanded" state only on a `true` return, so a failed apply is
  retried on the next policy run — but the only policy runs are on RCF, DISC
  and max-range entry. In FAILSAFE there are no RCFs by definition, so a verb
  that failed *on the failsafe entry itself* went unrepaired for up to
  `rendezvous_ms` (30 s) with the encoder flooding an mcs0-sized pipe at the
  previous rung's rate. A failed apply now short-circuits the interval and
  retries on the next tick.
- **Overrides are bounded.** Anything that moves the encoder rate behind
  RcAgent's back used to win until the ladder happened to change rung — the
  debug endpoint's `POST /venc/set?bitrate=` held for 20 s+ on a parked link
  in the 2026-08-29 bench run, and the same gap is the root of the historical
  waybeam-restart wedge (`docs/deploy.md`, rollback runbook). Such an override
  now survives **at most one re-assert interval (5 s)**. Bench procedures that
  relied on an override sticking need to re-POST inside that window.

### Encoder faults are process faults — there is no in-process rebuild

If the MI pipeline dies or has to be rebuilt (a resolution/sensor-mode change,
a wedged ISP), `maburd` must **exit and let `S96mabur` respawn it**.
`venc_core_stop()` + `venc_core_start()` in the same PID can never recover it,
and no amount of care in mabur's code changes that. Upstream implemented and
bench-tested every in-process reset lever on this SoC — disabling userspace
3A before VPE destroy, `MI_SYS_Exit`/`MI_SYS_Init` in-PID, closing the
`/dev/mi_vif` and `/dev/mi_vpe` fds, even `dlclose`/`dlopen` of the whole MI
vendor library set — and each one either wedges or comes back with a dead
stream (`ISP channel readiness timeout after 2000 ms` → `CmdLoadBinFile
failed -1` → `not sync err` floods → no frames). The residual state the
rebuild needs is per-task VIF/VPE/ISP channel state in the kernel driver,
released only by `execv`; userspace cannot reach it. Closed as a negative
result 2026-06-07 in `../waybeam_venc/documentation/STAR6E_SINGLE_PID_REINIT_FINDINGS.md`
— do not re-attempt without new SigmaStar SDK/kernel insight. In mabur this
is why fold-in bring-up failure is fatal-by-design (exit, respawn, cold
bring-up ~14–17 s measured, 5/5 unaided) rather than something the daemon
tries to heal in place.
