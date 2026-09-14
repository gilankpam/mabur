# In-flight channel hop

Reactive, sub-second migration of the live link off an interfered channel,
built on the boot-time auto channel select (`docs/channel-select.md`).
Design spec: `docs/superpowers/specs/2026-09-14-inflight-channel-hop-design.md`
(gitignored — this page is the durable record, and where the two disagree
this page describes what shipped). Reactive only: a healthy link never
moves, and there is no proactive re-ranking while the link is clean.

No hardware was available for the session that built this feature: every
host-side module is done, reviewed and ctest-covered, both cross-builds
(`tools/build-arm64.sh`, `tools/build-arm.sh`) link clean, but nothing has
been measured on air. `docs/handover-inflight-hop-bench-2026-09-15.md`
covers what a bench/flight session still owes — read it before flying with
`hop.enable = true`.

## 1. Wire and hop protocol

`RC_VERSION` 8 → 9 (`common/include/mabur/rc_proto.h`), a flag day like
every prior bump: an old/new pair rejects each other in both directions and
has no video, since `DISC_ACK` carries `CAP_FRAME_WIRE`. See
`docs/deploy.md` for the binary-swap sequence that applies to every
`RC_VERSION` bump.

`Rcf` gains:
- `hop_ch` (u8): the channel the drone must be on. Present in every RCF in
  every state — the standing truth, not an event. 0 means "no order ever
  issued"; the drone ignores `hop_ch == 0`.
- `hop_epoch` (u8): bumped by the GS on every order **and** every
  withdrawal, so a repeated `(hop_ch, hop_epoch)` pair is a no-op.

`Telem` gains `channel` (u8, `RcAgent::channel()` at build) and `hop_epoch`
(u8, echo of the last pair applied) — observability only, nothing waits on
either.

**Drone (`RcAgent`, `drone/src/rc_agent.cpp`).** On an accepted RCF whose
`(hop_epoch, hop_ch)` pair differs from the last one applied — the code
compares the **pair**, not the epoch alone (`r->hop_epoch != hop_epoch_ ||
r->hop_ch != hop_ch_`), matching the spec's "differs from the last pair
applied" over the plan's epoch-only draft — and whose `hop_ch` differs from
the drone's current channel: `Actuator::retune(hop_ch, "hop")` (the same TX-
gate-exclusive, 5 ms-drain, `FastRetune` path every other retune uses,
including TX-power re-application), the move is marked unconfirmed under
the existing `move_confirm_ms` rule, and the drone's stderr line prints
`maburd: retune <a> -> <b> (hop)` — the fourth `reason` value alongside
`disc`/`move_unconfirmed`/`rendezvous`. The retune-deferral during a
calibration sweep applies unchanged. The hop-tracking state (`have_hop_`,
`hop_epoch_`, `hop_ch_`) is reset at all **three** places
`have_last_seq_` is already reset for the same reason (a restarted GS
restarts its epoch numbering from 0, so a latched epoch from an old session
must not swallow the new session's first order): a fresh DISC
(`RcAgent::on_disc`), the move-unconfirmed fallback into RENDEZVOUS, and
FAILSAFE entry.

**GS (`ChannelPlan`, `gs/src/channel_plan.h/.cpp`) — two cards:**
1. `HopController::order()` bumps `epoch_`, sets the standing `hop_ch()`,
   and `VrxController::set_hop()`/`restore_rung()`/`blank_store()` run
   synchronously so **this same tick's** RCF already carries the new pair
   and `ref_rung`'s profile (`gs/src/main.cpp`'s `HopAction::Order` case).
2. `ChannelPlan::hop_order(now, target, lead_card)` sends the non-TX card
   (`lead_card`, whichever the TX selector is not currently using) to the
   target; the TX card stays on the old channel (`ChannelPlan::desired()`
   returns `op_` for every other card while `hopping_`). Every hop, order
   or withdraw, logs an `M` line (`hop_lead`, `hop_follow`, `hop_withdraw`,
   or `hop_one_card` — new `MoveReason` values alongside `commit`/
   `ack_override`/`split_home`/`reunite`).
3. First video AU received **on the target, by the lead card**
   (`last_video_ch == plan.hop_target()`) is confirmation:
   `HopController` emits `Confirm`, `ChannelPlan::hop_confirmed()` moves
   `op_` to the target and the TX card follows (`M ... hop_follow`).
4. No video within `hop.confirm_ms` (500 ms default): `HopController`
   withdraws — epoch bumps again, `hop_ch()` reverts to the old channel,
   the lead card returns, the target is backed off (§5). A drone that had
   already moved sees the withdrawal RCF or times out on its own
   `move_confirm_ms`; both converge on the old channel.

A second `hop_order()` while one is already in flight (a fresh trigger
before the first attempt resolved) does not leave the abandoned lead card
silently snapping back: `ChannelPlan` self-emits a `HopWithdraw` M-line for
the abandoned attempt before starting the new one, so the flight log
carries a trace of every attempt, even one the controller itself
superseded (`ChannelPlan::hop_order`, `gs/src/channel_plan.cpp`).

**One card, as built.** `HopController`'s own state machine models the
spec's "send `hop.one_card_repeats` (5) times on the old channel, then
retune" sequence: `HopAction::Order` fires first (state → `Ordered`), and
only once `rcf_sent_since_order >= hop.one_card_repeats` with still no
video does it emit a second action, `HopAction::OneCardRetune`
(`gs/src/hop_controller.cpp:ordered_tick`, pinned by
`tests/test_hop_controller.cpp`'s `one_card_retunes_after_repeats`).
`ChannelPlan`'s own one-card contract moves every card — the sole radio —
the instant `hop_order()` runs (`hop_lead_ < 0` → the target for every
card; pinned by `tests/test_channel_plan.cpp`'s
`one_card_hop_moves_all_and_confirms_or_withdraws`), so `gs/src/main.cpp`'s
`HopAction::Order` handler **only calls `plan.hop_order()` when
`act.lead_card >= 0`** (i.e. two cards) — for one card it is skipped on
`Order` and deferred to `OneCardRetune`, so the sole radio stays on the
old channel while `vrx.set_hop()`/`restore_rung()`/`blank_store()` still
fire immediately and the RCF starts carrying the order that same tick.
Only once `OneCardRetune` fires — after `one_card_repeats` RCFs have gone
out on the old channel with no video — does `plan.hop_order(now_ms,
act.target, -1)` run and the radio actually move. This matches the
spec's §1 sequence as written.

During a two-card hop the FEC decodes from one card only; the split is
bounded by `hop.confirm_ms`. `ChannelPlan::desired(card)` answers the
per-card channel exactly as it does for the boot-time split, so the main
loop's mechanical per-card retune (`gs/src/main.cpp`, the loop that drives
every `RadioFrontend` toward `plan.desired(i)`) needed no hop-specific
branch.

## 2. Verdict engine (`gs/src/hop_verdict.{h,cpp}`, pure)

`HopVerdict::window()` runs every `hop.window_ms` (150 ms) in every link
state, fed straight from a dedicated read block in `gs/src/main.cpp` (OFDM
FA/CCA via `read_energy_scout()`, foreign-frame and CRC-fail deltas from
the aggregator, RSSI/SNR EMAs) — **a card mid-dwell that window is skipped**
(`VerdictCardIn::valid = false`). Link-level: the s1 (BASE) pre-FEC loss
window and the FEC recovered-symbol delta.

Five independent evidence bits, OR'd together every window
(`gs/src/hop_verdict.h`'s `kEvImpaired/kEvWeak/kEvFading/kEvContended/
kEvRaised`):

| bit | rule | key (default) |
|---|---|---|
| impaired | link pre-FEC loss % > `loss_pct` **or** recovered count > `recovered_x` × its 5 s trailing mean | `hop.verdict.loss_pct` (3.0), `recovered_x` (3.0) |
| weak | best card's RSSI < `weak_rssi_dbm` **and** SNR < `weak_snr_db` | `weak_rssi_dbm` (−78), `weak_snr_db` (12) |
| fading | best card's RSSI more than `fading_drop_db` below its frozen/trailing reference | `fading_drop_db` (6) |
| contended | any card's foreign frames/s > `foreign_pps` | `foreign_pps` (50) |
| raised | any card's FA/s > `fa_pps` | `fa_pps` (100) |

Verdict, evaluated in this order (`HopVerdict::window`,
`gs/src/hop_verdict.cpp:79-82`):

```cpp
if (!impaired) o.v = Verdict::Healthy;
else if (weak) o.v = Verdict::Fade;
else if ((contended || raised) && !fading) o.v = Verdict::Interfered;
else o.v = Verdict::Unknown;
```

`interfered` needs **both** contention/a raised floor **and** the absence
of fading — an impaired, non-weak window that is fading (RSSI dropped off
its reference) but also shows contention or a raised floor classifies
`unknown`, not `interfered`: `weak` is checked first and wins outright if
true, and only among the non-weak windows does `!fading` gate `interfered`
against `unknown`. An edge-of-range but otherwise stable link showing
jammer-like FA/foreign symptoms therefore classifies `fade` (if also
weak) or `unknown` (if fading but not weak) — never `interfered` — and
neither case hops; the ladder owns both. The `weak`-before-`contended`/
`raised` edge case (`weak` true, `fading` false, `contended`/`raised`
true) was found unpinned by a branch-reorder mutation during review and
is now covered by `weak_takes_priority_over_contended_when_not_fading`.
The hop trigger is
`interfered` in `hop.persist` (2 default) of the last 3 windows
(`VerdictOut::trigger`). `fade` and `unknown` never hop in v1 — the ladder
handles both.

**References.** Per card, a 5 s trailing median of RSSI; link-level, the 5 s
trailing mean of the FEC recovered rate; both frozen at the first impaired
window, along with `ref_rung` (the ladder's rung at that instant), so the
interferer cannot raise its own baseline mid-episode. A card with no
trailing history yet at freeze time (out of range, or mid-dwell) keeps its
reference untouched rather than adopting a fabricated 0 dBm — a real RSSI
reading is never exactly 0, so a frozen reference reading back as 0 is
treated as "no reference yet" and the card's own current RSSI is used
instead (`fading` can't fire off a fabricated reference). Both references
thaw after 3 consecutive `healthy` windows, or after a hop's verify window
ends (`HopVerdict::reset()` — the trailing histories themselves survive a
reset; only the frozen snapshot is cleared).

## 3. Scout and ranking (in-session)

**As built, this is a separate pure module, not a `ChannelScout` mode.**
`gs/src/inflight_scout.{h,cpp}`'s `InflightScout` is its own class — the
boot-time `ChannelScout`'s scheduler (round-robin over
candidates-then-home, its own quiet/beacon interleave) is not reusable for
a periodic mid-flight dwell, and the project's new-module convention
(pure, ctest-covered, no hardware) applies the same way it did to
`channel_scout`/`channel_plan`.

Two-card GS only: a dedicated thread (`gs/src/main.cpp`'s `scout_loop`)
runs once the boot scout has released every card, gated on
`hop.enable || hop.scout_when_disabled` — **`scout_when_disabled` defaults
`true`**, so the scout dwells and logs even with `hop.enable = false`, to
collect ranking/calibration data for the first (observe-only) flights; set
it `false` to fly with literally no dwells. Every `hop.dwell_period_ms`
(333 ms default):
1. Card = whichever the TX selector is not using this cycle (read once at
   cycle start; the selector defers switching onto a card mid-dwell —
   `dwell_busy`/`dwell_card`).
2. Wait for the next AU boundary on that card.
3. `InflightScout::dwell()`: `FastRetune(candidate)` → discard read →
   sleep `hop.dwell_observe_ms` (5 ms) → real read (FA/CCA/frame counters)
   → `FastRetune(back)`.
4. A `D` scan.log line with `sess=1` and the four step-timing columns.

`InflightScout::dwell()` checks **both** retunes' results, not just the
first: if the retune to the candidate fails, or the observation completes
but the return retune to `back` itself fails, the card's true position is
uncertain — a stranded scout card means the aircraft is down to one radio
with no diversity — so `dwell()` sets `kFlagRetuneFailed` on the record and
**returns `false` without populating the `HopVisit`**, rather than booking
a visit gathered around an unknown card position. Recovery is not this
module's job: `gs/src/main.cpp`'s mechanical per-card retune loop (driven
off `plan.desired(card)` every tick, resynced from live hardware —
`cur_ch[card] = fe.channel()` — on every drained dwell) is what notices
and pulls a stranded card back. A failed dwell also does not leak a
phantom zero-score `HopVisit` into the ranker — an early bug (a
default-constructed `HopVisit{ch=0}` pushed unconditionally, which would
have ranked channel 0 as artificially best) was caught and fixed before
this shipped.

**The periodic scout thread stays two-card-only.** `gs/src/main.cpp`
gates its start on `n_cards >= 2` ("no point spinning it up on one card"
— a one-card GS has no spare radio to dedicate to it). A one-card GS's
ranking data comes entirely from the freshness burst below instead.

**Freshness burst.** `gs/src/main.cpp`: rather than act on a ranking that
may be up to `hop.rank_max_age_ms` old, every candidate is swept once,
back to back, via `InflightScout::burst()` whenever the controller has
no hop in flight (`HopState::Idle` **or** `HopState::Hold`) and the
verdict's trigger is set — **on both card counts**, matching the spec's
§3 "the only card on a one-card GS since the link is already impaired."
The gate does not require `ht.best` to already hold a value (an earlier
build's gate did, which on a one-card GS is circular: the burst is the
only source of visits, so requiring a ranked candidate before running it
made the ranker permanently empty — fixed to run the sweep unconditionally
whenever hop-free-and-triggered). Two cards are unaffected either way,
since the periodic scout thread already keeps the ranker warm before any
burst runs.

The burst is **rate-limited to at most one per `hop.dwell_period_ms`**
(333 ms default, `gs/src/main.cpp`'s `last_burst_ms`/`burst_due`) — reusing
the scout thread's own duty-cycle knob rather than adding a new key.
Without this, a sustained `Hold` (every candidate backed off, interference
persisting) re-enters `idle_tick` on every ~10 ms control tick with the
trigger still latched true, and nothing else paces it: `cooldown_ms` only
applies after a confirmed hop, and `max_hops_per_min` is only counted
inside `order()`, neither of which a stuck `Hold` ever reaches. Unlimited,
the burst would fire back to back — and because it runs synchronously on
the core thread, each pass also stalls RX processing for its duration, so
a one-card GS's sole radio would be off-air almost continuously exactly
when the link is already in trouble. Runs with `inflight_mu` held for its
duration, keeping the periodic scout thread (when one is running, i.e.
two-card) off the same `InflightScout`/`RadioFrontend` at once.

**Ranking** (`gs/src/hop_ranker.{h,cpp}`, pure). Per candidate, over the
last `hop.rank_visits` (5) visits not older than `hop.rank_max_age_ms`
(10 000 ms): `score = Σ (fa + max(cca − own, 0) + 4·foreign)`. Fewer than
2 fresh visits = unranked, never chosen. Lowest score wins; deterministic
tiebreak is **boot-time pick, then home, then config order** — the spec's
binding text (§3: "ties → boot-time pick, then home"), not the plan's
draft which dropped "then home"; both are ranked-clean by definition on a
tie so the runtime risk either way is negligible, but home is what makes
GS and drone converge rather than diverge, and it is now pinned by
`tie_break_prefers_home_over_config_order`. Candidates =
`radio.scan.candidates` ∪ `{home}`, the same set the boot scout ranks — a
strict superset of what `InflightScout` ever dwells on, so
`HopRanker::add()`'s silent no-op for a channel outside the candidate list
can never actually drop a real visit.

## 4. Ladder interaction: restore the pre-onset rung

- `ref_rung` is snapshotted by `HopVerdict` at the first impaired window
  (§2) and rides the hop-order RCF as `profile`/`fec_overhead_*`.
- **The ladder runs unfrozen during detection** — the spec's explicit
  requirement ("a demote or two, each an IDR" is accepted, not
  suppressed). What gates is narrower than the plan's original brief: only
  `LadderController::blank_store()` (the rung store's residual/util EWMA
  writes) is suspended — s3 demote **decisions** (`s3_live`, the residual
  and util checks) keep running the whole time, so a genuinely-fading enh
  layer can still demote itself before the base layer degrades, exactly as
  the spec intends. A demote taken on mid-hop, one-card evidence is priced
  in by the same spec sentence, and `restore()` overwrites it once the hop
  lands; the per-rung EWMA store stays uncontaminated regardless, because
  it was never written to during the blank.
- **The blank window, as built, is a fixed span from Order time**, not
  "from the first impaired window until confirmation" as the spec's prose
  reads: `gs/src/main.cpp`'s `HopAction::Order` case calls
  `vrx.blank_store(now_ms + hcfg.confirm_ms + 150.0)` — `confirm_ms` +
  the existing 150 ms post-transition settle, computed once, at the
  moment the order is placed. Detection time (before the persistence gate
  trips an actual order) and any part of the verify window past
  `confirm_ms + 150 ms` are outside the blank and do update the store
  normally.
- On the GS, `HopAction::Order` also calls
  `VrxController::restore_rung(ref_rung, now_ms)` → `LadderController::
  restore()`: rung set directly, probation cleared, probe-before-promote
  skipped, logged as ctl.log event reason `hop_restore`. This fires
  **synchronously at order time**, not at lead-confirm or verify-pass — so
  on a real flight `video → restore` can be negative, and
  `tools/flightreport.py` anchors its restore-match window to the order
  timestamp for exactly this reason (§8, and Known limitations).
- After the blank window, normal store-write rules resume; encoder
  bitrate follows the rung as always.
- `fade`/`unknown` verdicts: unchanged ladder behaviour — no restore, no
  blank, the ladder alone decides.

## 5. Verify, roll back, rate limits

`HopController` (`gs/src/hop_controller.{h,cpp}`, pure) owns all of this;
`gs/src/main.cpp` only translates its `HopAction`s into `ChannelPlan`/
`VrxController`/scan.log calls.

- **Verify window** `hop.verify_ms` (1000 ms) after confirmation:
  - `healthy`/`fade`/`unknown` throughout → stands (only a raw
    `Verdict::Interfered` window inside `verifying_tick` breaks it early);
    `verify_pass` logged, the landed channel's backoff cleared, `hops_++`.
  - `interfered` inside the window → the target is backed off
    `hop.backoff_ms` (30 000 ms, **doubling per repeat, capped at
    300 000 ms**), and the controller hops again to the next-ranked
    candidate (excluding anything currently backed off) with no new
    detection delay — `ref_rung` from the triggering verdict is reused, not
    re-snapshotted.
  - No video within `hop.confirm_ms` while `Ordered` → withdrawal (§1),
    same backoff.
- **Exhaustion.** All candidates backed off or unranked: home if not
  already there; else `hold` (`hold_exhausted`, no retune — the ladder
  copes). Automatically retried once a shorter backoff expires and a new
  trigger fires (there is no timer of its own; the next `interfered`
  window re-evaluates `ranker.best()`).
- **Rate limits.** `hop.max_hops_per_min` (4) **includes verify-fail
  retries** — only `hop.cooldown_ms` (2000 ms, between a confirmed hop and
  the next fresh trigger) exempts them. This is a deliberate reading of
  the spec's own sentence structure ("including retries, then hold" vs.
  "verify-window retries exempt" applying only to the cooldown clause,
  not the cap) — the original implementation generalised the cooldown
  exemption to the cap too, letting a genuinely bad link retry unbounded
  (8 orders in under a second against a cap of 4, in one review probe);
  fixed and pinned by `verify_fail_retries_count_against_rate_cap`. Hitting
  the cap mid-verify-retry logs `hold_cap` and does not self-perpetuate —
  the 60 s trailing window (`hop_times_`) slides and the next trigger
  after it clears retries normally.
- **Kill switch.** `hop.enable = false`: the whole state machine still
  runs — verdict windows, the ranker, `HopController`'s ticks, scan.log —
  but `HopController::tick()`'s last statement forces
  `out = HopAction{}` (`None`) whenever `!cfg_.enable`, so nothing is ever
  actually ordered; every event that would have fired logs with a
  `would_` prefix (`would_order`, `would_withdraw`, …) instead. Bundle
  default for the first flights.
- **Convergence.** The drone obeys only the newest `(hop_epoch, hop_ch)`
  pair and homes on `move_confirm_ms`; the GS waits only for video and
  withdraws on `confirm_ms`; the boot-time rendezvous machinery is
  untouched. Worst-case disagreement window is bounded by
  `move_confirm_ms + split_after_ms`, same as at boot.

## 6. devourer (Jaguar3)

`GetRxEnergyScout()` (OFDM FA + CCA, delta-on-read like `GetRxEnergy`) and
a batched-control-transfer path (precedent: the InitWrite async write
queue) landed on `../devourer` ahead of this plan and are consumed
unmodified — no devourer change was part of this task's scope. The
spike's "~10 ms with the batched path, 26 ms without" planning figures are
in the design spec; the measured figure from the real GS path is one of
the numbers `docs/handover-inflight-hop-bench-2026-09-15.md` still owes.

## 7. Config

`gs/bundle/maburgs.default.toml`, `[hop]`/`[hop.verdict]` (drone config
unchanged — the drone has no hop config of its own, it just obeys
whatever `hop_ch`/`hop_epoch` arrive on the RCF):

```toml
[hop]
enable               = false   # first flights observe-only
scout_when_disabled  = true
window_ms            = 150
persist              = 2
dwell_observe_ms     = 5
dwell_period_ms      = 333
rank_visits          = 5
rank_max_age_ms      = 10000
confirm_ms           = 500
verify_ms            = 1000
cooldown_ms          = 2000
max_hops_per_min     = 4
backoff_ms           = 30000
one_card_repeats     = 5

[hop.verdict]
loss_pct             = 3.0
recovered_x          = 3.0
weak_rssi_dbm        = -78
weak_snr_db          = 12
fading_drop_db       = 6
foreign_pps          = 50
fa_pps               = 100
```

Validation (`gs/src/config.cpp`), same fail-fast style as `radio.scan` —
unknown keys fail boot:

| key | range |
|---|---|
| `window_ms` | 50–2000 |
| `persist` | 1–3 |
| `dwell_observe_ms` | 1–250 |
| `dwell_period_ms` | 20–60000 |
| `rank_visits` | 1–100 |
| `rank_max_age_ms` | 1000–600000 |
| `confirm_ms` | 100–5000 |
| `verify_ms` | 200–10000 |
| `cooldown_ms` | 0–60000 |
| `max_hops_per_min` | 1–60 |
| `backoff_ms` | 1000–600000 |
| `one_card_repeats` | 1–50 |
| `hop.verdict.loss_pct` | 0.1–100.0 |
| `hop.verdict.recovered_x` | 1.0–100.0 |
| `hop.verdict.weak_rssi_dbm` | −110 – −20 |
| `hop.verdict.weak_snr_db` | 0–40 |
| `hop.verdict.fading_drop_db` | 1–40 |
| `hop.verdict.foreign_pps` | 1–100000 |
| `hop.verdict.fa_pps` | 1–100000 |

`[hop]`/`[hop.verdict]` are wholly new sections with defaults for every
key, so an old config without them boots unchanged on the new binary
(`enable` defaults `false`) — but `RC_VERSION` still forces the binary
flag day described in §1, since the RCF/Telem shapes changed regardless of
config.

## 8. Observability

**`scan.log`, marker `scanlog 2`** (`gs/src/scan_log.h/.cpp`; formats
locked by `tests/test_scan_log.cpp`). The `A` (1 Hz in-flight energy)
record and `radio.scan.energy_period_ms` are **gone** — the verdict
engine's window reads replace them, feeding `cards[i].energy` on the
sideport continuously in-session instead of once a second (§below). Two
new record kinds:

```
V <t> <verdict> <evidence_hex> <ref_rung|-> <link_loss_pct> <recovered>
  [<card> <foreign> <fa> <cca> <crc> <rssi> <snr> <drssi>]... # a verdict window
H <t> <kind> <epoch> <target> <score> <elapsed_ms>            # a hop event
```

- **V** — one per `HopVerdict::window()` call **that changed the verdict
  or was non-healthy** (not every 150 ms window unconditionally). Per-card
  block order is `foreign fa cca crc rssi snr drssi` (task review caught
  and fixed a fixture/golden mismatch here before shipping — the wire
  order is authoritative). `drssi` is the **link-level** `d_rssi_db`
  (best-card RSSI minus its reference), repeated identically in every
  card's chunk, not a genuinely per-card value.
- **H** — one per `HopController` state transition or logged decision.
  `kind` is always a single snake_case token — `order`, `lead_confirm`,
  `one_card_retune`, `verify_pass`, `verify_fail`, `withdraw`, `hold_cap`,
  `hold_exhausted` (the last two used to be the two-word C++ strings
  `"hold cap"`/`"hold exhausted"`, a space-delimited field containing the
  delimiter — fixed at the emitter rather than kept as a parser
  workaround, since scan.log is designed to outlive the code that wrote
  it). `hop.enable = false` prefixes every kind with `would_` — the
  machine still logs its decisions, it just never acts on them. Note:
  there is no `trail_follow` H-kind — the trailing card following the lead
  is an `M` line (`hop_follow`), not an `H` line; the spec's sketch listed
  it as an H event.
- **D** — extended, not replaced: the existing boot-scout dwell line gains
  a trailing `<sess> <to_us> <read_us> <back_us>` (in-session flag + the
  three step timings). Boot-time dwells still emit valid lines with these
  four fields at their defaults (`0 0 0 0`).

Full boot-time record formats (`C`/`K`/`M`) are unchanged and still
documented in `docs/channel-select.md`.

**ctl.log:** new event reason `hop_restore` (§4).

**Sideport** (`gs/src/stats_exporter.{h,cpp}`). New top-level `hop` object,
straight from `HopController`'s own accessors plus the latest
`HopVerdict::window()` output, unconditional (a disabled feature still
exports `enable: false` and idle defaults, matching the existing
`link.probe` pattern):

```
hop: {
  enable, verdict, evidence, ref_rung (null while unfrozen),
  epoch, state (idle|ordered|verifying|hold),
  target (null before the first-ever order), hops, holds,
  last_ms (elapsed_ms of the most recent HopEvent, null until one fires)
}
```

Per-card `cards[i].dwell` (null until that card has completed at least one
dwell): `{visits, score, cost_us}` — `visits` is cumulative over every
completed dwell (success or failure), `score`/`cost_us` are the **last**
successful dwell's, not an aggregate (a failed retune produces no
`HopVisit` to score, so a stale score is kept rather than zeroed).
`cards[i].energy` is unchanged in shape but now refilled every ~150 ms
verdict window instead of once a second from the deleted A-record poll —
noisier tick-to-tick, but it never goes stale for up to a second the way
the old poll could.

**`tools/maburtop.py`:** the header line gains `hop <state>/<verdict>`
next to the existing `scan <state>:<rounds>`; the per-card `busy` column
(`(cca − min(cca,own)) + fa + foreign`, the same score the ranker uses)
now tracks `cards[i].energy` at verdict-window cadence rather than the old
1 Hz poll.

**Player OSD:** the compact bar's `ch:` field appends `(h)` while a hop's
target is the live channel and that target is not home
(`GsSnapshot::hopped`, `gs/player/src/gs_snapshot.cpp`) — **not**
`hop.hops > 0`, which is a monotonic counter that never resets on
withdrawal or on hopping back home and would stay lit forever after the
first confirmed hop. `(h)` and the boot-scan's `(a)` share one suffix slot
(worst_case() reserves exactly `(a)`'s width) with `(h)` taking priority,
since a mid-flight hop is the more actionable of the two for the pilot.

**`tools/flightreport.py` HOP section** (`print_hop_report`,
`load_scanlog`). Session-mode only (`scan.log` is a sibling of `ctl.log`
in the debug-log session directory; there is no legacy filename
heuristic), and skipped entirely on a `scanlog` version below 2. One row
per hop **attempt** — every event that places a fresh `HopAction::Order`
(`order` and retry-triggering `verify_fail`), matched against ctl.log's
`hop_restore` E-lines by nearest timestamp within a 5 s window anchored on
the attempt's own order/retry time (not lead-confirm, per §4's
synchronous-restore finding):

```
t=<order_ts> target=<ch> [SHADOW]  onset->order <ms>  order->video <ms>  video->restore <ms>  outcome <kind>
```

With `hop.enable = false` every row is `would_`-prefixed and printed as
"SHADOW hop(s)" in a separate table rather than being silently folded into
"zero hops" — video never confirms while disabled, so a shadow row's
outcome is structurally almost always a `would_withdraw` timeout, which is
expected, not a bug. On a session with **zero real hops** (the expected
shape of every observe-only flight), the report instead prints:

- the verdict histogram (`healthy`/`fade`/`interfered`/`unknown` counts),
- an evidence-bit tally (`impaired`/`weak`/`fading`/`contended`/`raised`,
  counted over **all** windows, since a window can carry a bit without
  that bit driving the top-level verdict),
- per-card medians (not means — these are counter deltas with outliers)
  of `foreign`/`fa`/`rssi_dbm`/`snr_db` over non-healthy windows —

this is the whole point of the first, `hop.enable = false` flights: the
spec's own Open Items call them "the calibration" for five threshold
defaults that are spike numbers, not measurements, and a verdict name
alone cannot say whether `foreign_pps` or `weak_rssi_dbm` was the one
that nearly tripped.

Also printed: a per-card dwell-cost summary (count and
median(`to_us+read_us+back_us`) for dwells with `sess == 1`, i.e. dwells
that actually cost airtime inside a live session — the free off-session
boot-scan dwells are excluded).

## Measurements

No hardware was available in the session that built this feature, so
**every cell below is unmeasured** — these are the structures the bench
run and the first flights are expected to fill, not results. Run
`tools/bench/ausniff.py` (fps/gaps with the scout running) and the spike's
condition matrix with `hop.enable = true` (`benchjam` + O4 on the op
channel), per design spec §9; `docs/handover-inflight-hop-bench-2026-09-15.md`
is the actual runbook and owns filling these in.

**Dwell cost** (`tools/flightreport.py`'s DWELL COST section, or read
`D` lines directly):

| step | planning figure (spike, not measured on this build) | measured |
|---|---|---|
| retune to candidate | 4 ms | UNMEASURED |
| discard read | 6.5 ms | UNMEASURED |
| observe (`dwell_observe_ms`) | 5 ms | UNMEASURED |
| real read | 6.8 ms | UNMEASURED |
| retune back | 4 ms | UNMEASURED |
| **total** | **26 ms** (target ~10 ms with the devourer batched path) | UNMEASURED |

**Bench condition matrix** (design spec §9 acceptance list):

| condition | onset→video (H lines) | rung restored (ctl.log) | ausniff fps/gaps with dwells on |
|---|---|---|---|
| co-channel 802.11 neighbour, 333 ms dwell | UNMEASURED | UNMEASURED | UNMEASURED |
| non-802.11 (O4/analog) interferer, 333 ms dwell | UNMEASURED | UNMEASURED | UNMEASURED |
| fade (no hop expected — verify zero hops) | UNMEASURED | n/a | UNMEASURED |
| off-channel near-field blocking | UNMEASURED | UNMEASURED | UNMEASURED |
| dwell period 100 ms (regression check) | UNMEASURED | UNMEASURED | UNMEASURED |
| RCF `rx_pps` with dwells on vs off | UNMEASURED | UNMEASURED | n/a |

## Known limitations

- **A one-card GS needs two freshness-burst passes before it can hop to
  a real candidate.** `HopRanker` requires at least 2 fresh visits before
  a channel counts as ranked (§3), and on one card the burst — rate-limited
  to one pass per `hop.dwell_period_ms` (§3) — is the only source of
  visits. So the first trigger's burst (which fires immediately) leaves
  every candidate still unranked, `hopc`'s `best` stays `nullopt`, and the
  controller orders home (or holds, if already home); only the *second*
  burst, one `dwell_period_ms` later, gives the ranker enough visits to
  name a candidate. Reported (not measured — no hardware this session)
  derived figures for the shipped defaults: **~633 ms** to the order
  decision (300 ms detection + one 333 ms wait for burst 2), **~883 ms**
  to the physical retune (+ `one_card_repeats` × the RCF period), **~0.9–
  1.1 s** to a confirmed hop — around the spec's sub-second target, not
  comfortably under it. **These figures are provisional**: derived from
  config defaults and code paths only, pending independent re-derivation
  and, eventually, a bench measurement. Two-card GSes are unaffected — the
  periodic scout thread keeps the ranker continuously warm, so a two-card
  burst typically finds already-ranked candidates on its first pass.
- **A one-card freshness burst takes the sole radio off-air for ~30 ms
  per sweep** (three default candidates, a few ms each). Reconciled the
  same tick: `InflightScout::dwell()`'s own return-to-`back` retune, then
  a `cur_ch` resync from live `fe.channel()`, then (if either failed) the
  ordinary mechanical per-card retune loop — all before the tick ends, not
  deferred to the next one.
- **`dwell_busy`/`dwell_card`'s publish is not atomic** with the
  cross-thread read that gates the mechanical retune loop, the verdict
  window, and the TX selector against a card the scout has mid-dwell
  (`gs/src/main.cpp`). A reader can observe `dwell_busy == false` a few
  instructions before the scout thread flips it and calls `retune()`.
  Deliberately left unfixed: closing it means a lock in the hot per-tick
  path at three call sites, for a nanosecond-scale window whose worst case
  (one mistimed dwell) self-heals within the next tick via
  `plan.desired()`'s own retune loop and the `cur_ch` resync.
- **Burst dwells are invisible to the per-card sideport counters.** The
  pre-order freshness burst (§3) calls `InflightScout::burst()` directly
  and does not feed `dwell_recs`/`dwell_visits`, the vectors that populate
  `cards[i].dwell`. Reading `cards[i].dwell.visits` therefore undercounts
  real scouting activity — the periodic scout's dwells are all there, a
  freshness burst's are not.
- **The 5 s `flightreport.py` `hop_restore` match window is an
  uncalibrated judgement call**, chosen with no flight data to check it
  against. It fails safe: an unmatched restore line prints "not found"
  rather than mis-pairing with the wrong hop attempt.
- **`V` line's `drssi` field is link-level, not per-card**, repeated
  identically across every card's chunk on that line — see §8.
- **`scout_when_disabled` defaults `true`.** The first (`hop.enable =
  false`) flights still pay the scout's retune cost on the spare card
  every `dwell_period_ms`, by design (it is what makes the observe-only
  flights a calibration instrument) — but it means the very first flight
  after this deploy already has dwells happening on air, not a
  fully-inert kill switch.
