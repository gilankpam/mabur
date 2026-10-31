# In-flight channel hop

Reactive, sub-second migration of the live link off an interfered channel,
on top of the channel set and pick in `docs/channel-select.md`. Reactive
only: a healthy link never moves, and there is no proactive re-ranking
while the link is clean. The reactive hop runs in auto mode
(`radio.channel = "auto"`) only; a pinned channel is static — the verdict
is still measured and reported, but nothing is ordered except the one
relocation onto the pin (`docs/channel-select.md` "Which knob drives which
piece").

Ownership between the two pages: `docs/channel-select.md` owns the drone
side (`RcAgent`), the channel set, the boot scan/pick and relocation; this
page owns the verdict engine, the in-session scout and ranking, the hop
controller (order / confirm / verify / withdraw / rate limits) and the
`[hop]` config.

**Where the code lives.** All GS hop wiring is in `ChannelCore`
(`gs/src/channel_core.{h,cpp}`, shared by maburgs and the web GS; see
`docs/channel-select.md` "Where the code lives"). The pure pieces it
drives: `HopVerdict` (`gs/src/hop_verdict.{h,cpp}`), `InflightScout`
(`gs/src/inflight_scout.{h,cpp}`), `HopRanker` (`gs/src/hop_ranker.{h,cpp}`),
`HopController` (`gs/src/hop_controller.{h,cpp}`), `ChannelPlan`
(`gs/src/channel_plan.{h,cpp}`), plus the gates in `gs/src/hop_burst_gate.h`,
`gs/src/hop_blank.h` and `gs/src/scout_pick.h`. The hop inject seam is
tested in `tests/test_channel_core.cpp`
(`two_card_order_rcf_carries_hop_and_plan_leads_then_follows`,
`one_card_order_rides_repeats_then_retunes`).

Design spec: `docs/superpowers/specs/2026-09-14-inflight-channel-hop-design.md`
(gitignored, local only). How the fixes below were found:
`docs/findings/2026-09-26-inflight-hop-fix-history.md`. Bench record:
`docs/findings/2026-09-15-inflight-hop-bench-matrix.md` and
`docs/findings/2026-09-15-handover-inflight-hop-bench.md`.

## 1. Wire and hop protocol

`Rcf` (`common/include/mabur/rc_proto.h`) carries:
- `hop_ch` (u8): the channel the drone must be on. Present in every RCF in
  every state — the standing truth, not an event. 0 means "no order ever
  issued"; the drone ignores `hop_ch == 0`.
- `hop_epoch` (u8): bumped by the GS on every order **and** every
  withdrawal, so a repeated `(hop_ch, hop_epoch)` pair is a no-op.

Any change here is an `RC_VERSION` flag day (`docs/deploy.md`). Telem
carries no hop readback: the drone's channel is whichever channel its
video arrives on.

**Drone.** The drone side — a new `(hop_epoch, hop_ch)` pair retunes to a
member `hop_ch` (`"hop"` reason), the move is unconfirmed until an RCF is
heard on the new channel, and an unconfirmed move returns after
`link.move_confirm_ms` (2000 ms) to the channel it came from
(`move_from_ch_`) — is described in `docs/channel-select.md` "Drone
behaviour". A drone that moved before a withdraw therefore converges back
on the GS's old op on its own.

**GS, two cards:**
1. `HopController::order()` bumps the epoch and sets the standing
   `hop_ch()`; `ChannelCore::apply_hop_action_()` (`HopAction::Order`)
   runs `VrxController::set_hop()`/`restore_rung()`/`blank_store()`
   synchronously, so **this same tick's** RCF already carries the new pair
   and `ref_rung`'s profile.
2. `ChannelPlan::hop_order(now, target, lead_card)` sends the lead card to
   the target. The lead is `pick_hop_lead()` (`gs/src/scout_pick.h`): the
   first `ready()` card the TX selector is not using, any type; picked
   while no hop is `Ordered` and held through it, since a relay lead reads
   `!ready()` from its own `TUNE` until the relay confirms the target. With
   no such card the one-card path runs (`HopTick::n_cards` 1). A CPE510
   relay can lead: its retune sends `TUNE`, and frames are dropped until
   the relay confirms (`docs/cpe510-relay.md`). The TX card stays on the
   old channel (`ChannelPlan::desired()` returns `op_` for every other
   card while `hopping_`). Every hop logs an `M` line (`hop_lead`,
   `hop_follow`, `hop_withdraw` or `hop_one_card`, alongside `commit`,
   `ack_override` and `link_found`).
3. The first video AU **received on the target** confirms:
   `HopController` emits `Confirm`, `ChannelPlan::hop_confirmed()` moves
   `op_` to the target and the TX card follows (`M ... hop_follow`).
   "Received on" is `RxBody::rx_channel` (`common/include/mabur/node.h`),
   stamped by the producing `RadioFrontend` on its RX thread at the
   instant it lifted the frame, not "where is this card tuned now" — the
   core loop drains `BodyQueue` up to a control tick behind the RX threads
   and a retune does not flush that queue, so a downstream stamp would let
   old-channel frames confirm a hop the drone never heard. `retune()` sets
   the stamp to 0 (unknown) for the duration of the move; 0 never equals a
   hop target. The core loop passes each video body's `rx_channel` to
   `ChannelCore::note_video()`, and `HopTick::video_on_target` is
   `plan.hopping() && last_video_ch == hop_target`.
4. No video within `hop.confirm_ms` (bundle 1000 ms; code default 500):
   `HopController` withdraws — epoch bumps again, `hop_ch()` reverts to the
   old channel, the lead card returns, the target is backed off (§5),
   unless the confirm extension (§5) holds the order.

Only video received on op (or on the hop target while a hop is in flight)
refreshes the rendezvous silence timer (`ChannelPlan::is_link_video`, keyed
on `rx_channel`; 0 = unknown still counts), so a scout dwell catching the
drone on another member cannot hold the GS in SESSION.

A second `hop_order()` while one is in flight makes `ChannelPlan` self-emit
a `hop_withdraw` M-line for the abandoned attempt before starting the new
one, so the log carries every attempt.

**One card.** `HopController` models "send the order `hop.one_card_repeats`
(5) times on the old channel, then retune": `HopAction::Order` fires first
(state `Ordered`), and only once `rcf_sent_since_order >= one_card_repeats`
with still no video does it emit `HopAction::OneCardRetune`
(`HopController::ordered_tick`, pinned by `tests/test_hop_controller.cpp`'s
`one_card_retunes_after_repeats`). `ChannelPlan::hop_order()` with
`lead_card < 0` moves every card — the sole radio — at once
(`tests/test_channel_plan.cpp`'s
`one_card_hop_moves_the_only_card_and_withdraw_restores`), so
`apply_hop_action_()` calls `plan.hop_order()` on `Order` only when
`act.lead_card >= 0`; for one card it waits for `OneCardRetune`, while
`set_hop()`/`restore_rung()`/`blank_store()` still fire on `Order` and the
RCF starts carrying the order that same tick.

During a two-card hop the FEC decodes from one card only; the split is
bounded by `hop.confirm_ms` (or `confirm_extend_ms`). `ChannelPlan::desired(card)`
answers the per-card channel, and `ChannelCore`'s mechanical per-card
retune loop drives every card toward it, so the hop needs no retune code
of its own.

## 2. Verdict engine (`gs/src/hop_verdict.{h,cpp}`, pure)

`HopVerdict::window()` runs every `hop.window_ms` (150 ms) while
`hop_active()` (`gs/src/hop_burst_gate.h`: the link is in `SESSION` and no
calibration run is active).

The card reads the window needs -- the NHM busy counters and their re-arm,
the OFDM FA/CCA delta, the frame counters, a relay's survey window -- are
synchronous USB register I/O, and devourer serializes them with the card's
own coex tick (10-26 ms every 2 s) on one per-device lock. They therefore
run on `WindowSampler` (`gs/src/window_sampler.{h,cpp}`), not on the core
thread: every `hop.window_ms` the sampler samples each card and publishes
one `WindowRecord`; `ChannelCore::step_hop_edge_and_window_` takes the
latest record and runs the verdict on it, reading its own in-memory inputs
(per-card CRC failures, RSSI/SNR EMAs, recovered symbols, AU count) at that
moment. Per-window spans (`own_air_pct`) are measured between sample
instants. The sampler runs on its own thread wherever `ChannelCore` runs
threaded -- maburgs, and the web GS page build (`core_threads`); with
`threaded = false` (tests, the web GS without `core_threads`) `tick()` runs
one sampler step inline. The core tells the sampler what it may touch
through atomics: the op channel, a card the boot scout owns or a relay
sweep holds, a dwell in progress, and two per-card generation counters --
one bumped after every in-flight scout dwell, one around every core-thread
retune, width change or dwell burst -- that pin an NHM arm to the state the
card had when it was armed. Device use itself is serialized per card by
the sampler's device-use lock (`WindowSampler::hold_card`, forwarded as
`ChannelCore::hold_card`): a sampler pass holds it across all of a card's
reads, the core's dwell burst and the in-flight scout's dwell hold it for
their whole retune-away-and-back, and maburgs's card lifecycle holds it
across `stop()` + `open_and_start()`, so no read lands on a card being
torn down or tuned away. Lock order: `inflight_mu_` before the card lock.

On every edge of `hop_active`, `ChannelCore::
step_hop_edge_and_window_()` resets the engine, re-primes the `au_seq` and
recovered baselines and calls `HopVerdict::new_session()`; on the falling
edge it also hands `HopController::on_session_lost()`'s action to the
dispatcher. Without the gate the boot rendezvous (80–90 % s1 loss while
the link comes up) and a calibration run (video stopped on purpose) read as
`interfered`.

Per-card inputs come from the same block: OFDM FA/CCA via
`read_energy_scout()`, foreign-frame and CRC-fail deltas, RSSI/SNR EMAs
converted from devourer raw units by `rssi_raw_to_dbm()`/`snr_raw_to_db()`
(`hop_verdict.h`). A card contributes only when `verdict_card_usable()`
holds: `ready()`, not mid-dwell (in-flight scout, boot scout, or a pending
relay sweep on that card), and tuned to `plan.op()`. The verdict describes
the op channel, so a hop's lead card parked on the target during `Ordered`
is excluded; during `Ordered` the op verdict runs on the TX card alone, and
a one-card GS after `OneCardRetune` has no valid card (`unknown`, no
`kEvBlocked`).

Link-level inputs: the s1 (BASE) pre-FEC loss and the FEC recovered-symbol
delta. The loss input is the verdict's own 500 ms window
(`ChannelCore::s1_hop_loss_`), blanked when a hop lands (`HopAction::Confirm`
+ 150 ms settle, `hop_verdict_loss_blank_until()` in `gs/src/hop_blank.h`)
so a verify is judged on the channel it verifies; `s1_loss`, which feeds
the sideport/ctl-log/OSD gauge, is never blanked.

Seven independent evidence bits, OR'd together every window
(`hop_verdict.h`'s `kEvImpaired/kEvWeak/kEvFading/kEvContended/kEvRaised/
kEvBlocked/kEvStarved`):

| bit | rule | key (default) |
|---|---|---|
| impaired (0x01) | link pre-FEC loss % > `loss_pct` **or** (recovered count > `recovered_x` × its 5 s trailing mean **and** recovered count >= `recovered_min`) **or** `starved` | `hop.verdict.loss_pct` (3.0), `recovered_x` (3.0), `recovered_min` (8) |
| weak (0x02) | best card's RSSI < `weak_rssi_dbm` **and** SNR < `weak_snr_db` (RSSI alone when that card has no real SNR — `snr_valid` false on a CPE510 relay) | `weak_rssi_dbm` (−78), `weak_snr_db` (12) |
| fading (0x04) | best card's RSSI more than `fading_drop_db` below its frozen/trailing reference | `fading_drop_db` (6) |
| contended (0x08) | any card's foreign frames/s > `foreign_pps` | `foreign_pps` (50) |
| raised (0x10) | any card's FA/s > `fa_pps` | `fa_pps` (100) |
| blocked (0x20) | the MINIMUM, across cards with a busy reading, of that card's busy % minus its own reconstructed airtime % >= `blocked_pct` | `radio.scan.busy_dbm` (−83), `radio.scan.blocked_pct` (50) |
| starved (0x40) | at least one card valid this window and **zero** own frames on every valid card (`VerdictLinkIn::starved`), **or** the window's AU count (`VerdictLinkIn::au_count`, the `au_seq` delta) < `starved_frac` × its trailing mean, when that mean is >= 2 AUs/window | `hop.verdict.starved_frac` (0.25; 0 = zero-frames rule only) |

Why the two floors exist: on a clean channel the recovered term's trailing
mean sits near 0.2/window, so a handful of recovered symbols reads "3× the
reference" at 0 % loss — `recovered_min` keeps that from combining with a
noisy FA background into a false `interfered`. A jammed link delivers no
frames and therefore no loss; `starved` makes that impaired, and its AU-rate
term catches a long-frame jam that still lets a trickle of frames through.
60 → 30 fps low-power is a 0.5 AU-rate drop, so keep `starved_frac` below
0.5. The AU reference behaves like the recovered one: a 5 s history pushed
only while unfrozen, captured at the freeze, cleared by `reset()`; the AU
history and reference (only those) are also dropped by `new_session()` —
the frame rate before an outage is no baseline for the session after it.

**`blocked` and NHM busy.** The other evidence bits come from the preamble
detector (`foreign`, `fa`) or RSSI/SNR, so an interferer that fills airtime
without tripping the correlator — a long-frame 802.11 jam at low pps, or a
co-channel analog VTX carrier that desenses the front end and reads as a
fade with SNR intact (`docs/findings/2026-09-25-analog-vtx.md`) — is
invisible to them. `nhm_busy_pct` comes from the chip's NHM histogram
(airtime above `radio.scan.busy_dbm`, which config validation requires to
be an `nf::kNhmAbsThDbm` bucket edge), armed and read once per verdict
window per op card; `own_air_pct` is the GS's own reconstruction of that
card's airtime from the CRC-good frames it decoded (bytes·8 / HT rate, plus
one preamble per PPDU from the `physt` A-MPDU boundary). A busy reading is
invalid — `blocked` can't fire off it — when the chip has no NHM, the card
was mid-dwell, its channel changed mid-window, or an in-flight scout dwell
landed inside the armed window (per-card dwell-generation counter, since
`InflightScout::dwell()`'s `FastRetune` does not disturb devourer's
NHM-ready state); the arm is also pinned to the card's tune-generation
counter, so a window armed before a core-thread retune or width change is
never read against the card's post-retune state. The MINIMUM across cards
is taken because a weakly receiving diversity card counts our own frames
in its NHM busy but decodes few of them, so it reads "foreign" busy with
no interferer present. Bench numbers:
`docs/findings/2026-09-25-nhm-airtime-spike.md`.

**A CPE510 relay card's inputs** (protocol v4, `docs/cpe510-relay.md`
"Interference + hop on a relay"): `cca` is always 0 (ath9k has no CCA-event
count), so `raised` reads off `fa` alone — the OFDM/HT PHY-error delta
(`RemoteCard::read_energy_scout()`), same `fa_pps` key. `blocked` reads the
relay's `SURVEY` stream instead of NHM: busy % is `100·Δbusy/Δactive` (same
`gen`, span >= 100 ms) and `own_air_pct` is `100·Δrx/Δactive` — on the op
channel `rx` is our own video. `blocked_pct` is shared with the NHM path.

**Classification**, in this order (`HopVerdict::window`,
`gs/src/hop_verdict.cpp`):

```cpp
if (!impaired) o.v = Verdict::Healthy;
else if (weak) o.v = Verdict::Fade;
else if (blocked) o.v = Verdict::Interfered;
else if ((contended || raised) && !fading) o.v = Verdict::Interfered;
else o.v = Verdict::Unknown;
```

`weak` wins outright (range edge; a hop does not help). `blocked`
classifies `interfered` **even while `fading`**: a real fade lowers RX
power and cannot raise NHM busy airtime, so fading-and-blocked is desense
from an interferer (the analog-VTX shape: RSSI −58 → −70…−79, SNR intact).
`contended`/`raised` need `!fading`, so an edge-of-range link with
jammer-like FA/foreign symptoms classifies `fade` (weak) or `unknown`
(fading, not weak) — never `interfered` — and the ladder owns both. The
`weak`-over-`contended` ordering is pinned by
`weak_takes_priority_over_contended_when_not_fading`. The hop trigger
(`VerdictOut::trigger`) is `interfered` in `hop.persist` (2) of the last 3
windows. `fade` and `unknown` never hop.

**References.** Per card, a 5 s trailing median of RSSI; link-level, the
5 s trailing means of the FEC recovered count and of the AU count. All are
frozen at the first impaired window, along with `ref_rung` (the ladder's
rung at that instant), so an interferer cannot raise its own baseline
mid-episode; impaired windows push nothing into the trailing histories
(`starved_windows_do_not_feed_the_trailing_references`). A card with no
trailing history at freeze time keeps no fabricated reference: a frozen
reference of exactly 0 dBm reads as "no reference yet" and the card's
current RSSI is used, so `fading` can't fire off it. References thaw after
3 consecutive `healthy` windows, or on `HopVerdict::reset()`, which
`apply_hop_action_()` calls on `HopAction::VerifyPass` and only there. A
`verify_fail` or `withdraw` re-orders with the pre-onset `ref_rung` (§5);
thawing on those would let the next impaired window re-freeze `ref_rung` at
the mid-hop rung. `reset()` clears only the frozen snapshot and the latched
persistence window; the trailing histories survive it.

Every `VerdictOut` carries the span its deltas cover (`t_start_ms`, `t_ms`).
`ChannelCore` recomputes a verdict every `hop.window_ms` but ticks
`HopController` every ~10 ms control tick, so a cached `VerdictOut`
routinely outlives its window; §5 depends on the span.

## 3. Scout and ranking (in-session)

`InflightScout` (`gs/src/inflight_scout.{h,cpp}`) is its own pure module,
separate from the boot-time `ChannelScout`, whose scheduler does not fit a
periodic mid-flight dwell.

**Card picks** (`gs/src/scout_pick.h`):
- periodic dwells: `pick_inflight_scout()` — the last scout-capable card
  that is not the TX card, re-picked every period (none = skip; the loop
  also skips a card that is not `ready()`). A CPE510 relay is never
  scout-capable, so with one USB card plus a relay the USB card dwells only
  while the relay holds the uplink.
- freshness burst: `pick_burst_card()`, below.
- hop lead: `pick_hop_lead()` — the one pick a relay can win (§1).

The TX card is read once at cycle start; the TX selector does not switch
onto a card mid-dwell (`dwell_busy`/`dwell_card`) or onto a hop's lead card
while the hop is in flight (`tx_selection_frozen()` in `hop_burst_gate.h`)
— a selector switch onto the lead would move the RCFs carrying the order to
the target before the drone is there.

**Periodic scout** (two cards, auto mode). A dedicated thread
(`ChannelCore::scout_loop_`) starts once the boot pick is closed and the
boot scout owns no card, and only when `n_cards >= 2` and the GS is not
pinned (`ChannelCore::step_drains_`). Pinned, it never starts and the spare
card stays on diversity. Every `hop.dwell_period_ms` (333 ms):
1. Card = `pick_inflight_scout()`.
2. Channel = `InflightScout::next_candidate(fe.channel())`: round-robin over
   the **dwell set** — `radio.channels` in config order — skipping the
   channel the card sits on (op; this loop never runs mid-hop). One dwell
   per period whatever the set size.
3. Wait for the next AU boundary on that card.
4. `InflightScout::dwell()`: `FastRetune(candidate)` → discard read → arm
   NHM, observe `hop.dwell_observe_ms` (5 ms) → real read (FA/CCA/frame
   counters, NHM busy) → `FastRetune(back)`.
5. A `D` scan.log line with `sess=1` and the step timings.

`dwell()` checks **both** retunes. If either fails, the card's position is
uncertain: it sets `kFlagRetuneFailed` and returns `false` without a
`HopVisit`. Recovery is `ChannelCore`'s mechanical per-card retune loop
(driven off `plan.desired(card)`, with `cur_ch` resynced from
`fe.channel()` on every drained dwell), which pulls a stranded card back.

**Freshness burst.** Rather than act on a ranking up to
`hop.rank_max_age_ms` old, `ChannelCore::step_controller_` sweeps every
dwell-set channel except op once, back to back (`InflightScout::burst()`),
whenever `hop_burst_due()` holds: no hop in flight (`HopState::Idle` or
`Hold`), the verdict trigger is live, and the last burst is at least one
period old. It also requires auto mode, no boot-scout ownership, no
relocation in progress and no relay sweep pending. It runs on both card
counts — on a one-card GS it is the only source of ranking data — and does
not require a ranked candidate first (on one card that would be circular).
The burst holds `inflight_mu_`, keeping the periodic scout thread off the
same `InflightScout`/`RadioFrontend`, and runs synchronously on the core
thread, so it stalls RX processing while it runs.

The period is `hop.dwell_period_ms` (333 ms) for a USB burst card and
`hop.relay_burst_period_ms` (1000 ms) for a relay. Without a rate limit a
sustained `Hold` re-enters the burst on every ~10 ms control tick (neither
`cooldown_ms` nor `max_hops_per_min` paces a hold that never orders), and a
one-card GS's sole radio would be off air almost continuously; a relay
sweep takes ~280 ms, so it gets the longer period.

**Which card bursts** (`pick_burst_card()`), first match wins:

1. scout-capable, not TX (USB spare — the link card stays on air)
2. sweep-capable, not TX (relay spare — a v4 `SCAN`; the USB TX card keeps the link)
3. scout-capable TX (one-card USB GS)
4. sweep-capable TX (relay-only GS)
5. none → skip (`-1`)

Sweep-capable is evaluated per tick (`can_sweep` and `ready()`). A
sweep-capable pick runs the relay's async `SCAN` instead of `burst()`:
`ChannelCore` sends one `SCAN` for the dwell set minus op (2 passes, 20 ms
observe), polls for `SCAN_RESULT` on later ticks (`poll_sweep_()`), then
feeds the entries to the ranker as `HopVisit`s with `src = VisitSrc::Relay`
(`gs/src/relay_sweep_map.h`). No result within
`max(1000, passes·n·(observe_ms+40)+300)` ms drops the burst (sideport
`hop.sweep_timeouts`). While a sweep is pending, `ChannelCore` holds
`ht.verdict.trigger` false so the controller cannot enter `hold_exhausted`
before the result lands.

**Ranking** (`gs/src/hop_ranker.{h,cpp}`, pure). Per candidate, over the
last `hop.rank_visits` (5) visits not older than `hop.rank_max_age_ms`
(10 000 ms): `score = Σ (fa + max(cca − own, 0) + 4·foreign)`.
`ranking()` uses only the **newest visit's source kind**, so USB dwells
(5 ms, CCA-event counts) and relay sweeps (20 ms, survey counts) are never
averaged together. Fewer than 2 fresh visits = unranked, never chosen.
Lowest score wins; ties fall to the boot-time pick, then config order
(`tie_break_falls_through_to_config_order`). The boot pick is published
with `HopRanker::set_boot_pick()` when `ChannelCore` freezes the pick (next
to the `K` scan.log line), and only when the scan measured something (some
channel reached `min_rounds`); until then `boot_pick` is 0 and ties fall
straight to config order (`no_boot_pick_falls_through_to_config_order`).
Candidates = `radio.channels`, the same set the boot scout ranks and
`InflightScout` dwells on.

**Blocked tier.** Each `HopVisit` carries an NHM busy reading
(`busy_valid`/`busy_pct`, armed by `dwell()` for the observe span; no
own-airtime subtraction, since none of our frames land on a candidate).
`HopRankEntry.busy_pct` is the **mean** over an entry's fresh, valid
visits (a 5 ms window on a bursty interferer reads 0 or 100). An entry is
`blocked` when that mean is `>= radio.scan.blocked_pct`. This is tiering,
not a score weight: `ranking()` puts every blocked ranked entry after every
unblocked one, tiebreaking blocked pairs by lower `busy_pct` before the
score/boot-pick/config-order chain. A channel with no busy evidence ranks on
score alone. `best()` with `require_unblocked` skips blocked entries; every
hop target comes from that call (§5).

**40 MHz.** At `radio.width = 40` dwells keep the 40 MHz tuning and score
the primary only, and the ranker is built over the same primaries. Only
`burst()` and in-flight dwells feed `HopRanker::add()`, both on primaries,
and every member shares one `ht40_offset` (a config rule), which
`FastRetune` keeps — so every ranked channel is a valid hop target. The
per-half boot scan is `ChannelScout`'s alone (`docs/bw40.md`).

## 4. Ladder interaction: restore the pre-onset rung

- `ref_rung` is snapshotted by `HopVerdict` at the first impaired window
  (§2) and rides the hop-order RCF as `profile`/`fec_overhead_*`.
- **The ladder runs unfrozen during detection** ("a demote or two, each an
  IDR" is accepted). Only `LadderController::blank_store()` — the rung
  store's residual/util EWMA writes — is suspended; s3 demote decisions
  keep running, so a fading enh layer can still demote itself. A demote
  taken on mid-hop evidence is overwritten by `restore()`; the per-rung
  store stays clean because it was not written during the blank.
- **The blank starts at the first `interfered` window**, from two call
  sites that compose because `blank_store()` keeps the LATER deadline:
  1. `hop_store_blank_until()` (`gs/src/hop_blank.h`, pure), called from
     `ChannelCore::step_hop_edge_and_window_()`, returns
     `t_ms + confirm_ms + 150 ms` **once per impaired episode**, on its
     first `interfered` window (`VerdictOut::first_interfered`).
  2. `HopAction::Order` calls `blank_store(now_ms + hop.confirm_ms + 150)`.

  Three gates on (1):
  - **Pinned.** Not reactive: nothing is ordered, so there is no hop to
    protect the store from, and blanking would change what a pinned flight
    records. (2) never fires pinned either.
  - **`interfered`, not `impaired`.** `fade` and `unknown` are impaired too;
    only interference hops, so only interference blanks.
  - **One edge per frozen episode.** The deadline is computed once, at
    onset, and not re-extended by later `interfered` windows, so a long or
    flickering jam cannot roll it forward. The store resumes
    `confirm_ms + 150 ms` after onset whether or not a hop was ordered;
    re-arming needs a thaw (3 healthy windows, or `reset()` after a verify).

  Any part of the verify window past `confirm_ms + 150 ms` updates the store
  normally.
- `HopAction::Order` also calls `VrxController::restore_rung(ref_rung, now)`
  → `LadderController::restore()`: rung set directly, probation cleared,
  probe-before-promote skipped, logged as ctl.log reason `hop_restore`.
  This fires **synchronously at order time**, so `video → restore` can be
  negative, and `tools/flightreport.py` anchors its restore match to the
  order timestamp (§8).
- `fade`/`unknown` verdicts: no restore, no blank; the ladder alone decides.

## 5. Verify, roll back, rate limits

`HopController` (`gs/src/hop_controller.{h,cpp}`, pure) owns all of this;
`ChannelCore` fills a `HopTick` each control tick and translates the
returned `HopAction` into `ChannelPlan`/`VrxController`/scan.log calls
(`apply_hop_action_()`, `dispatch_hop_action_()`). `HopController(HopCfg)`
has no home or fallback channel.

- **Orders.** From `Idle`/`Hold` with a live trigger, past `cooldown_ms`
  and under `max_hops_per_min`: order `HopTick::best` (H `order`) and back
  off the channel being fled (`flee()`, same schedule as a failed target),
  so a failed target's retry cannot go straight back to the jam — the 5 ms
  event-count dwells score a long-frame jammer low. A withdraw still returns
  to the fled channel: it restores op, it does not consult the ranker.
- **Relocation** (`HopTick::relocate`, H `relocate`): the order that moves
  the link to `ChannelPlan::want()` (the pin, or the boot pick), placed
  through this controller (`docs/channel-select.md` "Finding the drone").
  No flee, exempt from `cooldown_ms`, counted against `max_hops_per_min`.
  **No verify for a pinned relocation** (`HopTick::no_verify`, set only
  when not reactive): the confirm lands it and `VerifyPass` follows on the
  next tick, the pin is never backed off, `holds` stays 0. A relocation
  onto an auto pick keeps the verify.
- **Verify window** `hop.verify_ms` (1000 ms) after confirmation:
  - `healthy`/`fade`/`unknown` throughout → stands. `verify_pass` logged,
    the landed channel's back-off cleared, `hops++`, `HopAction::VerifyPass`
    emitted (the caller thaws the verdict references, §2).
  - **Only a verdict measured entirely after landing can fail it.**
    `verifying_tick` ignores any `VerdictOut` whose `t_start_ms` is earlier
    than the confirm plus the 150 ms landing settle (`kHopSettleBlankMs`).
    The cached verdict right after a `Confirm` is the pre-hop one —
    `interfered` by construction — and the settle span still carries the
    retune gap's repairs as `recovered` symbols. The first eligible window
    lands ~3 × `window_ms` after the confirm, leaving four in a 1000 ms
    verify.
  - A raw `Verdict::Interfered` window inside it → the target is backed
    off, and the controller orders the next-ranked candidate (not backed
    off) at once, reusing the triggering `ref_rung` (H `verify_fail`); with
    no candidate it tries the escape (below), else holds (`verify_fail`).
    Retries are exempt from `cooldown_ms` but counted against
    `max_hops_per_min` (`hold_cap` when full).
- **Back-off.** `hop.backoff_ms` (30 000 ms), doubling per repeat of the same
  channel, capped at 300 000 ms. Each entry records why: `fled`, `failed`
  (verify fail, withdraw, session lost) or `undelivered` (a withdraw after a
  confirm extension). `backed_off()` returns all three;
  `backed_off_failed()` only `failed`.
- **Withdraw.** No video within `hop.confirm_ms` while `Ordered` →
  `withdraw` (§1), target backed off as failed — unless extended.
- **Confirm extension: undelivered orders are not failures.** A jammer near
  the drone also jams the uplink, so the order never arrives; withdrawing at
  `confirm_ms` would back the clean target off as failed. Once `confirm_ms`
  has passed, if `hop.confirm_extend_ms` (3000) > `confirm_ms` **and** the
  current verdict carries `kEvBlocked` **and** the order is younger than
  `confirm_extend_ms`, the controller stays `Ordered` (the RCFs keep
  carrying the order) and logs one H `confirm_extend`. Once engaged it holds
  to `confirm_extend_ms` whatever later windows say — the jam lifting is
  exactly when the order lands, and withdrawing then would race the drone's
  retune. A confirm during the extension proceeds to `Verifying`; expiry
  logs `withdraw_undelivered` and backs the target off as undelivered.
  `confirm_extend_ms <= confirm_ms` (e.g. 0) disables it. A one-card GS
  after `OneCardRetune` has no valid op card (§2), so the extension does not
  engage there.
- **Never hop into a blocked channel.** Every `ranker.best()` that feeds
  `HopTick::best` or `HopTick::escape` passes `require_unblocked`; the
  blocked tier in `ranking()` is for display and logs only. When every
  option is blocked the controller holds.
- **Escape.** `HopTick::escape` = `ranker.best(op, backed_off_failed(),
  require_unblocked)` — fled and undelivered channels allowed, failed and
  blocked never. The controller orders it (H `escape`) only when there is no
  `best`, the current verdict carries `kEvBlocked` (the link's own channel
  is blocked) and the hop cap allows it (`HopController::escape_allowed`):
  in `idle_tick` before `hold_exhausted` (after the cooldown check), and in
  `verifying_tick`'s verify-fail branch before its `verify_fail` hold.
- **Session lost mid-hop** (`HopController::on_session_lost`, on
  `hop_active`'s falling edge). An `Ordered` hop is withdrawn like a
  `confirm_ms` timeout (target backed off as failed, epoch bumped,
  `session_lost` logged), which frees `ChannelPlan`'s own link-loss path
  (`ChannelPlan::release_scout()` hands the spare card to the search scout
  after `radio.scan.search_after_ms`; `docs/channel-select.md`). A
  `Verifying` hop just drops its verify.
- **Exhaustion.** All candidates backed off, unranked or blocked and no
  escape: hold (`hold_exhausted`, no retune — the ladder copes). There is no
  timer; the next live trigger after a back-off expires re-evaluates
  `ranker.best()`.
- **Holds are states; only their edges are logged.** `idle_tick()` runs from
  `Hold` as well as `Idle`, so a held controller re-enters the hold branch
  every control tick. Entering logs one event naming the reason (`hold_cap`,
  `hold_exhausted` or `verify_fail`) and bumps `holds()` once; re-entering
  logs nothing; leaving logs one `hold_end` whose `elapsed_ms` is the
  episode's length. A hold also ends when the trigger clears (otherwise
  every exit runs through `order()`, which needs a trigger). `hop.holds`
  counts episodes.
- **Rate limits.** `hop.max_hops_per_min` (4, 60 s trailing window
  `hop_times_`) **includes verify-fail and escape retries**; only
  `hop.cooldown_ms` (2000 ms, between a confirmed hop and the next fresh
  trigger) exempts them (`verify_fail_retries_count_against_rate_cap`).
- **Convergence.** The drone obeys only the newest `(hop_epoch, hop_ch)`
  pair and falls back to the channel it came from after `move_confirm_ms`;
  the GS waits only for video and withdraws (or extends) on
  `confirm_ms`. A drone stranded on a withdrawn target is back on the GS's
  op within `move_confirm_ms` of its move.

## 6. devourer (Jaguar3)

The verdict and the dwells read OFDM FA + CCA through devourer's
`GetRxEnergyScout()` (delta-on-read, like `GetRxEnergy`) and NHM busy
through the NHM API; `RadioFrontend::read_energy_scout()` wraps it.
Measured dwell cost: `docs/findings/2026-09-15-inflight-hop-bench-matrix.md`.

## 7. Config

`gs/bundle/maburgs.default.toml`, `[hop]` / `[hop.verdict]`. The drone has
no hop config; it obeys whatever `hop_ch`/`hop_epoch` arrive on the RCF.
`[hop]` has no on/off switch: the reactive hop runs in auto mode only.

```toml
[hop]
window_ms            = 150
persist              = 2
dwell_observe_ms     = 5
dwell_period_ms      = 333
relay_burst_period_ms = 1000
rank_visits          = 5
rank_max_age_ms      = 10000
confirm_ms           = 1000   # code default 500
confirm_extend_ms    = 3000
verify_ms            = 1000
cooldown_ms          = 2000
max_hops_per_min     = 4
backoff_ms           = 30000
one_card_repeats     = 5

[hop.verdict]
loss_pct             = 3.0
recovered_x          = 3.0
recovered_min        = 8
weak_rssi_dbm        = -78
weak_snr_db          = 12
fading_drop_db       = 6
foreign_pps          = 50
fa_pps               = 100
starved_frac         = 0.25
```

The bundle ships `confirm_ms = 1000` because hops to or from the 8822E
spur pairs (149+153, 157+161) confirm in 423–460 ms, too close to 500.
The busy threshold and level the `blocked` bit and both rankers use are
`radio.scan.busy_dbm` / `radio.scan.blocked_pct`, shared with the boot scan
(`docs/channel-select.md` "Config"). `blocked_pct` 50 sits midway between
the worst clean-channel error (~10) and a real interferer (95–99)
(`docs/findings/2026-09-25-nhm-airtime-spike.md`).

Validation (`gs/src/config.cpp`) — unknown keys fail boot:

| key | range |
|---|---|
| `window_ms` | 50–2000 |
| `persist` | 1–3 |
| `dwell_observe_ms` | 1–250 |
| `dwell_period_ms` | 20–60000 |
| `relay_burst_period_ms` | 500–60000 |
| `rank_visits` | 1–100 |
| `rank_max_age_ms` | 1000–600000 |
| `confirm_ms` | 100–5000 |
| `confirm_extend_ms` | 0–30000 (<= `confirm_ms` = no extension) |
| `verify_ms` | 200–10000 |
| `cooldown_ms` | 0–60000 |
| `max_hops_per_min` | 1–60 |
| `backoff_ms` | 1000–600000 |
| `one_card_repeats` | 1–50 |
| `hop.verdict.loss_pct` | 0.1–100.0 |
| `hop.verdict.recovered_x` | 1.0–100.0 |
| `hop.verdict.recovered_min` | 0–1000 (0 = no floor) |
| `hop.verdict.weak_rssi_dbm` | −110 – −20 |
| `hop.verdict.weak_snr_db` | 0–40 |
| `hop.verdict.fading_drop_db` | 1–40 |
| `hop.verdict.foreign_pps` | 1–100000 |
| `hop.verdict.fa_pps` | 1–100000 |
| `hop.verdict.starved_frac` | 0.0–1.0 (0 = zero-own-frames rule only) |
| `radio.scan.busy_dbm` | −104 – −70, must sit on an `nf::kNhmAbsThDbm` bucket edge |
| `radio.scan.blocked_pct` | 1.0–100.0 |

## 8. Observability

**`scan.log`** (`gs/src/scan_log.{h,cpp}`, marker `scanlog 7`; formats
locked by `tests/test_scan_log.cpp`; marker history in
`docs/data-provenance.md`). The hop adds two record kinds and extends `D`:

```
V <t> <verdict> <evidence_hex> <ref_rung|-> <link_loss_pct> <recovered>
  [<card> <foreign> <fa> <cca> <crc> <rssi> <snr> <drssi> <nhm_busy|->
   <own_air>]...                                             # a verdict window
H <t> <kind> <epoch> <target> <score> <elapsed_ms> <reason|->  # a hop event
```

- **V** — one per `HopVerdict::window()` call **that changed the verdict or
  was non-healthy**. Per-card block order is
  `foreign fa cca crc rssi snr drssi nhm_busy own_air`. `nhm_busy` is `-`
  when the card's busy reading was invalid for this window (§2); `own_air`
  is always present, % of the window. `drssi` is the **link-level**
  `d_rssi_db` (best-card RSSI minus its reference), repeated identically in
  every card's chunk. The evidence field is the §2 bitmask in hex: impaired
  0x01, weak 0x02, fading 0x04, contended 0x08, raised 0x10, blocked 0x20,
  starved 0x40.
- **H** — one per `HopController` state transition or logged decision.
  `kind` is a single snake_case token: `order`, `relocate`, `escape`,
  `lead_confirm`, `one_card_retune`, `confirm_extend`, `verify_pass`,
  `verify_fail`, `withdraw`, `withdraw_undelivered`, `session_lost`,
  `hold_cap`, `hold_exhausted`, `hold_end`. The trailing card following the
  lead is an `M` line (`hop_follow`), not an H line. `hold_end` closes a
  hold episode opened by `hold_cap`/`hold_exhausted`/`verify_fail`; its
  `elapsed_ms` is the episode's length (§5). Recordings from before
  2026-10-04 may carry `would_`-prefixed kinds (the deleted shadow mode);
  `flightreport.py` prints those as SHADOW. `reason` is `cal` on the
  relocate a calibration run orders (`docs/channel-select.md` "Calibration
  pick") and `-` on every other event; `scanlog 6` and older H lines end at
  `elapsed_ms`.
- **D** — the dwell line shared with the boot scan, with trailing
  `<sess> <to_us> <read_us> <back_us>` (in-session flag + the three step
  timings; `0 0 0 0` for boot dwells), `<busy|->` (NHM busy % over the
  observe span) and `<rx|->` (a relay sweep entry's rx %, `-` for a USB
  dwell; built by `sweep_dwell()` in `gs/src/relay_sweep_map.h`). Full
  `D`/`K`/`C`/`M` formats are in `docs/channel-select.md`.

**ctl.log:** event reason `hop_restore` (§4).

**Sideport** (`gs/src/stats_exporter.{h,cpp}`). Top-level `hop` object,
from `HopController`'s accessors plus the latest `HopVerdict::window()`
output, always present:

```
hop: {
  verdict, evidence, ref_rung (null while unfrozen),
  epoch, state (idle|ordered|verifying|hold),
  target (null before the first-ever order), hops,
  holds (hold EPISODES entered, not ticks held — §5),
  last_ms (elapsed_ms of the most recent HopEvent, null until one fires),
  sweep_timeouts (relay SCANs that never returned, §3)
}
```

Per-card `cards[i].dwell` (null until that card has completed a periodic
dwell): `{visits, score, cost_us}` — `visits` is cumulative over every
completed periodic dwell (success or failure); `score`/`cost_us` are the
**last** successful dwell's (a failed retune produces no `HopVisit`, so the
old score is kept). `cards[i].energy` is refilled every verdict window and
also carries `busy_pct`/`own_air_pct` (both `null` when the window's busy
reading was invalid) — the same values the verdict uses (§2,
`docs/observability.md`).

**`tools/maburtop.py`:** the header shows `hop <state>/<verdict>` next to
`scan <state>:<rounds>`. The per-card `busy` column
(`(cca − min(cca,own)) + fa + foreign`, the ranker's score) tracks
`cards[i].energy` at verdict-window cadence; `fbusy`
(`max(0, busy_pct − own_air_pct)`) is the foreign busy airtime the
`blocked` bit and both rankers use.

**Player OSD:** the compact bar's `ch:` field appends `(h)` while a hop's
target is the live channel and that target is not `scan.pick`
(`GsSnapshot::hopped`, `gs/player/src/gs_snapshot.cpp`). `scan.pick` is the
channel the GS froze its boot pick on (or the pin) — latched in
`ChannelCore::frozen_pick_` at the freeze, never the live op. `hopped` is
false while the pick is still open (`scan.pick` null), which is the boot
scan's own `(a)` mark. `(h)` and `(a)` share one suffix slot
(`worst_case()` reserves `(a)`'s width), `(h)` taking priority. `hop.hops`
is not used: it never resets on a withdrawal or a hop back to the pick.

**`tools/flightreport.py` HOP section** (`print_hop_report`,
`load_scanlog`). Session-mode only (`scan.log` beside `ctl.log` in the
debug-log session directory), skipped on a `scanlog` version below 2 — the
highest marker in the file, since a GS restart rejoins the session
directory under the old binary's header. One row per hop **attempt** —
every event that places a fresh `HopAction::Order` (`order`, a retrying
`verify_fail`, `escape`) — matched against ctl.log's `hop_restore` E-lines
by nearest timestamp within a 5 s window anchored on the attempt's own
order time (§4):

```
t=<order_ts> target=<ch> [SHADOW]  onset->order <ms>  order->video <ms>  video->restore <ms>  outcome <kind>
```

`hold_end` is treated as a terminal kind defensively: an unrecognised kind
arriving with a row open would fall through to `unterminated` ("the log
ends mid-attempt"). On a session with zero real hops the report prints
instead the verdict histogram, an evidence-bit tally over all windows, and
per-card medians of `foreign`/`fa`/`rssi_dbm`/`snr_db` over non-healthy
windows — the data for calibrating the `[hop.verdict]` thresholds. Always
printed: `escapes: N  starved windows: M`, `confirm extensions: N
undelivered withdraws: M` (`withdraw_undelivered` closes its attempt row
like `withdraw`; `confirm_extend` leaves it open), and a per-card
dwell-cost summary (count and median `to_us+read_us+back_us` over `sess == 1`
dwells).

## Measurements

Bench and flight records: `docs/findings/2026-09-15-inflight-hop-bench-matrix.md`
(dwell cost, condition matrix 2026-09-15 → 2026-10-04, verdict at rest),
`docs/findings/2026-09-15-handover-inflight-hop-bench.md` (run-by-run
2026-09-15), `docs/findings/2026-09-25-nhm-airtime-spike.md` (analog VTX and
DJI O4), `docs/findings/2026-09-25-analog-vtx.md` (one-card analog VTX) and
`docs/findings/2026-10-04-channel-set-bench.md` "Bench 2026-10-04 — channel core extraction".

## Known limitations

- **A withdrawn two-card order strands the drone for `move_confirm_ms`.**
  The drone retunes on the first RCF carrying the order; the withdrawal
  rides RCFs on the old channel, which the drone — now on the target —
  cannot hear. It returns after `move_confirm_ms` (2 s), and any order the
  GS places meanwhile goes to a drone that is not listening. The confirm
  extension (§5) avoids the withdraw when the op reads blocked; a guard
  that refuses a new order until the drone is heard again is the candidate
  fix.
- **One-card hops cost the ladder rungs.** The `one_card_repeats` order
  window leaves the drone ahead of the GS's sole radio by ~200 ms, the
  ladder's loss windows are not blanked, and a demote cascade lands after
  the confirm. `blank_store` covers the rung store only.
- **A one-card GS needs two freshness bursts before it can hop.** A
  channel needs 2 fresh visits to be ranked and the burst makes one per
  channel, rate-limited to one per `dwell_period_ms`; the first trigger's
  burst leaves every candidate unranked and the controller holds
  (`hold_exhausted`) until the second, ~333 ms later.
- **A one-card freshness burst takes the sole radio off air** for one dwell
  per dwell-set channel (a few ms each). The return retune, the `cur_ch`
  resync and, if needed, the mechanical retune loop all run in the same
  tick.
- **`dwell_busy`/`dwell_card` are two separate atomics**
  (`gs/src/channel_core.h`), read by the mechanical retune loop, the
  verdict window and the TX selector. A reader can see `dwell_busy == false`
  a few instructions before the scout thread flips it and retunes. Left
  unfixed: a lock in the per-tick path for a nanosecond-scale window whose
  worst case (one mistimed dwell) self-heals next tick.
- **Burst dwells are invisible to `cards[i].dwell`.** Burst dwells are
  logged to scan.log but do not feed the sideport dwell counters, so
  `cards[i].dwell.visits` undercounts scouting.
- **`RxBody::rx_channel` has a residual mis-stamp window** of however far
  the USB RX pipeline lags real time beyond `FastRetune`'s own duration
  (~4 ms): a frame received on the old channel but delivered after the
  retune completed reads as the new channel. devourer exposes no RX flush.
- **The 5 s `flightreport.py` `hop_restore` match window is uncalibrated.**
  It fails safe: an unmatched restore prints "not found".
- **`fa_pps` 100 is marginal on a channel with a busy FA background**
  (bench: 70–133 FA/s on a candidate after the follow failed its verify).
  Calibration, not code.
