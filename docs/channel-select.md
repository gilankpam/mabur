# Auto channel selection over a shared channel set

The operator sets nothing about channels on either end. Both ends carry
the same **channel set** (`radio.channels`); the drone always sits on
exactly one member and remembers which; the GS is always on one member
(`op`), remembers which, and knows the set is the only place the drone can
be. With `[radio] channel = "auto"` the GS measures the set once, at
process start, and moves the link to the cleanest member (the **boot
hop**), then freezes for good — later link losses re-propose the same
frozen pick and the scout never runs again. A healthy link never moves on
its own; the sub-second reactive migration layer
(`docs/inflight-channel-hop.md`) still handles an interfered channel after
the freeze. `channel = <member>` pins the link there instead and skips
measurement entirely.

**There is no home channel.** The 2026-09-13 design (candidates plus a
privileged home, boot-only pick frozen at first ack, split/reunite on link
loss) is superseded by this one, shipped 2026-10-03. Everything below
describes what shipped; the parts of the old design that are unchanged in
spirit (ack-then-move, the silent dwell, the leak subtraction) carry
forward, renamed to fit the new model.

Design spec: `docs/superpowers/specs/2026-10-03-auto-channel-set-design.md`
(gitignored — this page is the durable record, and where the two disagree
this page describes what shipped). Builds on
`docs/inflight-channel-hop.md` (2026-09-14), which stays as the reactive
layer and needed its own home references removed to match (see that
page's §1/§3/§5/§7).

## Config

Both ends carry the set. The drone's list may be a superset of the GS's,
never a subset: the GS only ever proposes or orders members, and the drone
parks only on members.

GS, `gs/bundle/maburgs.default.toml`:

```toml
[radio]
channels = [40, 64, 112, 144]   # the set; the drone parks on a member, the GS picks among them
channel  = "auto"               # or a member: pin the link there, no measurement
width    = 40
tx_card  = -1
relays   = []

[radio.scan]
dwell_ms        = 250   # measurement observe per 20 MHz half (unchanged)
settle_ms       = 30    # after every retune (unchanged)
min_rounds      = 3     # full passes before the ranking is mature (unchanged)
search_ms       = 100   # DISC burst at the start of every unlinked dwell on a primary (5 beacon periods)
op_window_ms    = 300   # one card: how long the sole card beacons on op between dwells
search_after_ms = 5000  # on link loss, keep every card on op this long before sweeping
pick_margin     = 20    # auto: the pick must beat the channel the link is on by this many busy units
one_card_ms     = 5000  # one card, auto: measure silently this long before the first DISC
max_ms          = 30000 # auto: the pick is frozen at the latest this long after GS start
```

Drone, `bundle/mabur.default.toml`:

```toml
[radio]
channels = [40, 64, 112, 144]   # parks on the remembered member (else the first); follows the GS to any member
width    = 40
```

Removed, failing boot as every removed key does: GS `radio.channel` as a
number meaning home, `radio.scan.enable`, `candidates`, `home_window_ms`,
`split_after_ms`, `home_margin`. Drone `radio.channel`, `radio.follow_gs`.
Drone `link.move_confirm_ms` and `link.rendezvous_ms` stay (the latter now
only drives the FAILSAFE → RENDEZVOUS state change, never a retune):

```toml
[link]
failsafe_ms     = 3000     # no word from the ground station this long = drop to the most robust link setting
rc_drain_ms     = 5        # how often commands from the ground station are applied
rendezvous_ms   = 30000    # still silent this long after failsafe = back to RENDEZVOUS (the drone stays on its channel)
move_confirm_ms = 2000     # after a channel move, hear the ground station within this or return to the channel we came from
```

Validation (`gs/src/config.cpp`, `drone/src/config.cpp`, shared code in
`common/include/mabur/channel_set.h`):

| key | range |
|---|---|
| `radio.channels` | 1–8 members, unique, each in `[1,177]`; at `width = 40` every member must be a standard HT40 pair primary and all members must share one `ht40_offset` (`docs/bw40.md` "Channels") |
| GS `radio.channel` | the string `"auto"`, or an integer that is a member of `radio.channels` — anything else fails |
| `radio.scan.dwell_ms` | 50–10000 |
| `radio.scan.settle_ms` | 0–1000 |
| `radio.scan.min_rounds` | 1–100 |
| `radio.scan.search_ms` | 40–2000 |
| `radio.scan.op_window_ms` | 40–10000 |
| `radio.scan.search_after_ms` | 0–600000 |
| `radio.scan.pick_margin` | 0–100000 |
| `radio.scan.one_card_ms` | 0–60000 |
| `radio.scan.max_ms` | 1000–600000 |

### State files

Both daemons remember the member the link last lived on
(`common/include/mabur/channel_file.h`):

- Drone: `/etc/mabur.channel` (the writable overlay). Written by
  `RcAgent::remember_channel()` on every accepted RCF while a move is
  still pending — a DISC move or a hop move alike, confirmed the instant
  the GS is heard from on the new channel; a handful of writes per
  session, flash wear is a non-issue.
- GS: `/etc/maburgs.channel`. Written on every change of `ChannelPlan::op()`.

Format: the channel number as decimal text and a newline. Written via a
temp file in the same directory and `rename()`, so a power cut mid-write
leaves the old value, never a torn one. Read once at start; a missing,
unreadable or non-member value silently falls back to the first member —
nothing to repair, nothing to configure (the paths are compiled-in
constants). Deleting a state file makes that end forget its remembered
channel; on the GS that also forces the next boot to search from scratch
rather than re-finding a drone sitting on a channel the GS already
"knows."

## Drone behaviour (`RcAgent`, `drone/src/`)

- **Boot channel.** `channel_` = the remembered member (if the state file
  names one that is still in `radio.channels`), else `radio.channels[0]`.
  InitWrite tunes there at `radio.width`. No scan, no search; the drone
  listens and, as always, sends `T_TELEM` at 1 Hz in every state — that is
  its "shout".
- **Follows members only.** A DISC proposing a member the drone is not on:
  ack agreement from the current channel at DISC time; the retune itself
  happens once that agreed pair is promoted by the first verifying RCF
  (`act_.retune(ch, "disc")`), and the move is marked unconfirmed. A DISC
  proposing a **non-member**: the drone acks its own current channel
  instead (`maburgs` logs `ack_override` on `ChannelPlan::on_ack`). An RCF
  `hop_ch` that is not a member is ignored for the retune, but the
  `(hop_epoch, hop_ch)` pair is still recorded as seen, so a repeated
  non-member order is not re-evaluated on every RCF.
- **Unconfirmed move goes back, never home.** After `move_confirm_ms` with
  nothing heard from the GS on the new channel: retune to
  `move_from_ch_` — the channel the drone came from, DISC move or hop
  alike — and enter RENDEZVOUS there. "There is no home to fall through
  to" (`drone/src/rc_agent.cpp`). `go_home_()` is deleted.
- **Long loss: stay put.** FAILSAFE entry and the `rendezvous_ms` timeout
  change state only (LINKED → FAILSAFE → RENDEZVOUS); neither retunes.
  The drone is always on exactly one member, which is what makes the GS
  sweep below complete — a search burst will eventually land on whatever
  channel the drone is parked on.
- **Retune mechanics unchanged**: TX-gate exclusive, 5 ms drain,
  `FastRetune`, TX power re-applied, calibration deferral (latched and
  replayed on the sweep's falling edge), stderr
  `maburd: retune A -> B (reason)`. Reasons: `disc`, `hop`,
  `move_unconfirmed`. `rendezvous` is gone — there is no retune on that
  path any more.
- **Wire: no change.** `Disc.op_channel`/`DiscAck.agreed_channel` and
  `Rcf.hop_ch`/`hop_epoch` carry members exactly as before. No
  `RC_VERSION` bump.

Invariant: the drone is always on a member of its set, and only moves on
a GS proposal, a GS order, or an unconfirmed move back to where it came
from.

## GS: search and measure (`ChannelScout`, `gs/src/channel_scout.{h,cpp}`)

`ChannelScout` is the long-lived scheduler that owns the spare USB card
(two-card) or the sole card between op windows (one card) whenever it has
work. It replaces the old boot-only scan: it no longer stops at the first
ack. `has_work = search || (measure && !frozen)` — it searches while the
link is down and/or measures while the pick is still open (auto mode
only); with no work it parks the card on `op` at `radio.width` and sits
idle.

**Dwell schedule.** Round-robin over the 20 MHz **half set**
(`pair_pick.h`'s `scan_half_set` over the set — no home — each pair's low
half then high half, config order; at `width = 20` the members
themselves). Per half:

| phase | when | what |
|---|---|---|
| retune + settle | always | `FastRetune(half, 20)`, `settle_ms` |
| **search burst** | link down AND half is a pair primary (or width 20) | send DISC proposing `op` every beacon period for `search_ms`; listen for the ack |
| gap | after a burst | one beacon period, so the card's own TX is out of its receiver |
| discard read | measuring | as today |
| **observe** | pick open (auto) | silent `dwell_ms`, read FA/CCA/own/foreign/NHM |

Pinned mode: no observe, ever (`measure = false`) — a sweep of four
primaries ≈ 0.6 s. Auto, link down: burst + observe (≈ 3.2 s per round
over eight halves at the shipped defaults). Auto, linked, pick open:
observe only.

**Op's own pair is never measured while linked** (final review I1,
2026-10-04). With the search off (linked, or inside `search_after_ms`
after a loss) and the pick open, the scout skips both halves of `op`'s
pair (`op` alone at `width = 20`): no retune, no observe. They carry the
drone's own video — at width 40 undecodable to a 20 MHz observe and so
counted busy, and in NHM's airtime either way — and scoring them would
make the boot hop leave a clean `op` almost every time. A skipped half is
completed in the scheduler (stamped one dwell ahead) so the round-robin
and `rounds()` keep moving. `op` keeps only its pre-link visits:
`ChannelScout::mature()` waits for every member **except** `op` (and for
`op` too while searching, when it is visited like any other), and
`op_ranked()` says whether `op`'s pre-link visits reached the effective
`min_rounds`. When they did not, the pick freezes in place (`"op
unmeasured"`, below).

**The leak.** Unlinked, the TX card holds its DISC during every observe
(`quiet()`, unchanged from the 2026-09-13 fix — see History below).
Linked, RCFs must flow, so the TX card is never held; its TX leaks into the
adjacent scout as undecodable energy. The ranker subtracts
`k × tx_frames_during_observe` from the dwell's busy score
(`ChannelRanker::busy()`: `max(cca − own − leak, 0) + fa + foreign`), where
`tx_frames_during_observe` is the TX card's frame-counter delta over the
observe span (`ChannelScout::set_tx_frames()`, fed `ctrl_sent_total`,
incremented once per successful `send_control()` across every card) and
`k` (`leak_per_frame`) is a **compiled constant**, `1.0`
(`gs/src/main.cpp`, "bench row 7 pins this"), not a config key. The leak is
uniform across candidates, so it biases only the stay-or-move comparison,
and `pick_margin` absorbs the residual. Bench row 7 (below) is the
standing validation of that constant: it has not run yet, so `k` is
provisional.

**One card.** Cycle: op window of `op_window_ms` on `op` (DISC every
beacon period, listen; leave only after one quiet beacon period), then one
dwell on the next half. Auto mode adds a **silent prelude**: for the first
`one_card_ms` after start the card only observes (no DISC anywhere); once
the deadline passes, the core commits the prelude's ranking (if not
already linked and it differs from `op`) and calls
`ChannelScout::ack_prelude(op)`, which hands the scout the committed
channel and releases the first op window — the scout holds (no tune, no
beaconing, `working()` stays true) until that ack arrives, so no window
ever runs against the stale pre-commit `op`. Linked, the sole card carries
the link and cannot measure: the pick freezes at link-up (`"one-card
linked"`).

**Width.** The scout card tunes 20 MHz for its dwells and rejoins the link
at `radio.width` on `op` when it has no work (`ChannelScout::run()` parks
there; the core loop's one-shot width resync covers the scout-card-died
and freeze-before-scout-started edge cases, same as the 40 MHz page
describes).

**Send gating** (`gs/src/main.cpp`): the scout owns a card
(`scout_owns()`: `working() || search requested || (!pinned &&
pick_open())`) whenever the core must not also touch it — mechanical
retune, TX selection, verdict input, width resync, the in-flight scout and
freshness burst, the boot-order gate all key on it. The send gate and DISC
routing use the narrower `working()`/`beaconing()`: one-card mode drops
the sole card's non-DISC frames while it is off `op`; two-card mode holds
the TX card's DISC while `quiet()`. The scout's own DISC bursts are sent
by the scout card from the scout thread's own schedule, not the core
loop's beacon.

**Relays** keep beaconing DISC on `op` whenever `ready()`, never as the
only path (`scan_disc_targets()` in `gs/src/scout_pick.h` has no home
branch any more, but keeps the relay rule: a CPE still booting, unplugged
or owned by another client must not be the only way a DISC leaves the
GS).

**Who scouts, who beacons** is otherwise unchanged from before this
feature (`gs/src/scout_pick.h`): the boot/in-flight scout is the last
scout-capable card — the spare USB card on a two-USB GS, the only card on
a one-USB GS. A CPE510 relay (`docs/cpe510-relay.md`) has no FA/CCA/NHM
reads and never scouts. No `[[radio.cards]]` block pins nothing: auto-scan
uses every supported USB card found, which is two-card mode; an explicit
`[[radio.cards]]` list (one entry) is how you fly one card.

## The pick: maturity, commit, boot hop, freeze

**Ranking.** `pair_proposal(all, current, set, min_rounds, pick_margin,
blocked_pct)` (`gs/src/pair_pick.h`): a pair is ranked once both halves
have `min_rounds` visits (the one-card deadline uses the *effective*
`min_rounds` — 1, once the prelude is done and fewer than `min_rounds`
full passes exist, else the configured value — so a one-card prelude pick
counts as measured). Score = the worse half's worst-visit busy; the
blocked tier (either half's NHM busy ≥ `blocked_pct`) ranks after every
unblocked pair. The channel the link currently sits on (`op`) plays the
role home used to play: a candidate must beat it by `pick_margin`, ties
stay on `op`. Config order breaks remaining ties. Returns `op` when
nothing else is ranked.

**At maturity** (every pair ranked, `max_ms`, or the one-card deadline —
`ChannelScout::mature()`):

- **No link** (GS powered first): if the proposal ≠ `op`,
  `ChannelPlan::commit()` moves `op_`, the TX card retunes, stderr
  `maburgs channel: commit <from> -> <to> (no link)`, `M all <from> <to>
  commit`, the state file is written. Freeze (`"commit"`). The drone, when
  it appears on its remembered member, is found by a search burst and
  moved by DISC.
- **Linked, two cards**: if the proposal ≠ `op`, the scout is frozen (stops
  measuring, parks, frees the lead card — `pick_ranking()` stays readable
  for the hop machinery) and a **boot hop** is wanted
  (`maburgs channel: boot hop wanted <op> -> <pick>`). Once the scout has
  actually let go of the card, one `HopController` order is placed — event
  kind `boot_order` (H line) — through the unchanged Order → Confirm →
  Verify → VerifyPass / Withdraw / verify-fail machinery, lead card = the
  scout card (the non-TX card by construction), `restore_rung` = the
  verdict's `ref_rung` (the current rung at boot — there is no prior
  impairment to restore from, so this is simply "keep the current rung").
  While the pick is open, `HopTick::best` is filled from the boot
  ranker's ordering (`pick_ranking()`, excluding backed-off pairs), not
  the in-flight hop's own ranker, so a verify fail retries the next-best
  **measured** pair. Logged `maburgs channel: boot hop placed <op> ->
  <target>`. Freeze fires once the controller returns to `Idle` or `Hold`:
  on `verify_pass` the link is on the pick (`"boot hop landed"`); on
  exhaustion it is back on the old `op` (`"boot hop exhausted"`); on a
  withdraw that never confirmed, `"boot hop withdrawn"`; on a confirm that
  landed but whose verify window never finished (a session loss mid-verify),
  `"boot hop confirmed, verify cut short"`.
- **Linked, one card**: the pick froze at link-up — there was never a
  second card to lead a hop with.
- **Proposal == op**: freeze in place (`"in place"`).

**Freeze** closes the pick for the GS process's lifetime: the scout stops
measuring and rejoins the link at full width. Logged
`maburgs channel: pick frozen on <op> (<why>) after <rounds> rounds` plus
a `K` line (shape unchanged: `K <t> <picked|none> <rounds>
<ch>:<worst_busy>:<floor|nan>:<busy|-> ... pair=<lo>+<hi>|-`; `picked` is
`op` after the decision). From here only the reactive hop
(`docs/inflight-channel-hop.md`) moves the link; later link losses
re-propose `op`. The reasons seen in code, in no particular priority order
(the first condition the tick loop meets wins): `"commit"`, `"in place"`,
`"calibration running"`, `"hop disabled"` (the controller would only log
`would_boot_order` and never move the link, so the pick stays where it
is), `"boot hop landed"` / `"boot hop exhausted"` / `"boot hop
withdrawn"` / `"boot hop confirmed, verify cut short"`, `"boot hop: no
eligible pair"` (every measured pair is backed off or none is ranked —
stay on `op` rather than synthesize an exhausted hold), `"link lost before
boot hop"` (the link dropped between wanting the hop and placing its
order — keep `op`, the drone is most likely still there), `"one-card
linked"`, `"scout card died"`, and `"max_ms"` (a half that never ranks
cannot keep the diversity card away forever).

No eligibility/arm gate: the boot hop is placed armed or not. It is a
measured, verified, withdrawn-on-failure move inside the first `max_ms` of
a GS process; the only way it meets an armed drone is a GS restart
mid-flight, and then it costs what a reactive hop costs.

Timing at the defaults, two cards, both powered together (design figure,
not yet bench-confirmed — see Bench validation row 1): link on the
remembered member within ~1 s; three rounds of eight halves ≈ 10 s; boot
hop lands ≈ 0.5 s later.

## In-flight hop after the freeze

Once the pick freezes, only the reactive hop
(`docs/inflight-channel-hop.md`) can move the link, and only off an
`interfered` verdict. That page's own candidate list is now
`radio.channels` (no home appended) and its exhaustion path holds rather
than falling back anywhere — see its §1, §3, §5 and §7 for the as-built
detail, updated for this feature.

## Observability

- **`scan.log`**: marker `scanlog 5` (bumped from `scanlog 4`). Header
  line: `scanlog 5 channels=<c1,c2,...> mode=<auto|pinned>
  dwell_ms=<n> min_rounds=<n> cards=<n>`. `C`/`D`/`K` record shapes are
  unchanged from `scanlog 4` (`docs/bw40.md`,
  `docs/nhm-airtime-spike-findings-2026-09-25.md`). `M` loses the
  `split_home`/`reunite` reasons — there is nothing to split from or
  reunite to any more — and keeps `commit`/`ack_override` plus the
  in-flight hop's `hop_lead`/`hop_follow`/`hop_withdraw`/`hop_one_card`.
  `H` gains the `boot_order` kind alongside the in-flight hop's
  `order`/`verify_fail`/`escape` (`gs/src/scan_log.h`). A `scanlog 4` or
  earlier file never has `boot_order` and still has `split_home`/
  `reunite` in its `M` lines — read it as what it was
  (`docs/data-provenance.md`).
- **Sideport**: `scan.state` ∈ `off | scouting | moving | frozen` (`off` —
  pinned, or no scout-capable card; `scouting` — the pick is open;
  `moving` — a boot hop is in flight; `frozen` — the pick is closed).
  `scan.rounds` (the scout's round count). `scan.pick` (the frozen
  channel, only once `!pick_open`). `link.home` is **deleted** — there is
  no home to report. `link.channel` is unchanged: the live channel of the
  TX card.
- **stderr**: `maburd: channel set [<c1,c2,...>], parking on <n>[
  (remembered)]` (drone boot); `maburgs channel: set [<c1,c2,...>] mode
  <auto|pinned> start <n>[ (remembered)]` (GS boot); `maburgs channel:
  commit <from> -> <to> (no link)`; `maburgs channel: boot hop wanted
  <from> -> <to>`; `maburgs channel: boot hop placed <from> -> <to>`;
  `maburgs channel: pick frozen on <op> (<why>) after <n> rounds`;
  `maburgs channel: one-card prelude ranking picks <n> (op <n>)[, linked:
  not committed]`; `maburgs channel: drone acked <n>, not in our set;
  ignored`; `maburgs channel: <reason> card <n> <from> -> <to>` (every `M`
  line, echoed to stderr); `maburgs channel: could not write
  /etc/maburgs.channel`; `maburd: retune <a> -> <b> (<reason>)`.
- **`tools/maburtop.py`**: drops the `h{home}` field from the compact
  channel line; shows `scan.state` (including `moving`) and `scan.rounds`
  next to the channel.
- **`tools/flightreport.py`**: reads `scanlog 5`; treats `boot_order` as a
  HOP-section order kind alongside `order`/`verify_fail`/`escape`; still
  parses `split_home` out of an older-marker file, since a recording made
  before this date still carries it (`docs/data-provenance.md`).

## Deploy

Binary then config on **each** device (old binaries reject the new keys;
new binaries reject `radio.channel`/`follow_gs`). Drone first or GS first
does not matter — there is no `RC_VERSION` bump, so a half-deployed pair
still links on whichever member both ends happen to be on (the old
binary's `radio.channel` home and the new binary's remembered/first member
may well differ, in which case the pair simply does not link until the
deploy finishes — the usual two-devices-never-atomic story, not a new
risk).

Deleting a state file (`/etc/mabur.channel` or `/etc/maburgs.channel`)
makes that end forget its remembered channel on the next boot and fall
back to `radio.channels[0]` (drone) or search from `channels[0]`/the pin
(GS). Useful when a device was last parked somewhere you no longer want it
defaulting to.

See `docs/deploy.md`'s `## 2026-10-03 channel set` section for the full
sequence and verification lines.

## Bench validation

Spec `docs/superpowers/specs/2026-10-03-auto-channel-set-design.md` §8.
None of these has been run yet — the "result" column is the one place in
this repo's docs a `pending` value is allowed, because it records that the
row has not run, not an unmeasured constant masquerading as a result.

| # | check | result |
|---|---|---|
| 1 | Cold start both, two cards, auto: link ≤ 1 s on the remembered member; `K` + boot hop within ~12 s; `verify_pass`; no gap beyond the hop's | pending |
| 2 | GS first, drone two minutes later: `commit` at maturity; drone found by a burst; moves on ack | pending |
| 3 | Battery swap: drone returns on `op`, re-links on the first DISC, no sweep | pending |
| 4 | GS restart with the drone on the old `op` and the GS state file deleted: sweep finds it, proposes, moves | pending |
| 5 | One card auto: 5 s silent then link on the pick; one card pinned: link < 1 s | pending |
| 6 | Jam the remembered member before power-up: boot hop leaves it. Jam a candidate: never picked | pending |
| 7 | The leak constant `k`: scout on a clean candidate while the TX card sends RCFs at the low-power and 60 fps cadences; busy per frame sent | pending |
| 8 | `maburcal` on 64 and 112; fixed-rung linkbench 40/2 on 64 vs 136; decide 64 vs 128 for the default set | pending |

## Out of scope

The web GS (`web/`) gets **no changes** in this work; if the shared
`maburgs::Config` change breaks its build, it stays broken until its own
design (`docs/web-gs.md` is untouched by this page). A runtime auto/pin
switch. Width negotiation over DISC (both ends keep `radio.width = 40`).
Relay channel capability — the operator rule that every member must be
CPE-tunable stays a documented rule, `docs/cpe510-relay.md` (a relay GS
drops 144 from its own set by hand; the scan has no way to ask the relay).
2.4 GHz, per-card channels.

## History

The material below is retained from the 2026-09-13 design
(`docs/channel-scan-findings-2026-09-13.md`) because it explains why every
scout dwell is silent and why the ranker subtracts a leak term — both
still true, under the new no-home scheduler, exactly as they were under
the old boot-only one.

### The `cca − own` assumption (2026-09-13)

The ranker's busy score (`gs/src/channel_ranker.{h,cpp}`) was:

```
busy = (cca_ofdm − canonical_frames) + fa_ofdm + foreign_frames
```

`cca_ofdm` counts every OFDM CCA event the chip saw, including the ones
caused by our own beacon and RCF traffic — subtracting `own` (canonical-SA
frames decoded) is meant to leave foreign busy only, on the assumption
that one decoded own-frame costs exactly one CCA event. If a decoded frame
actually costs more than one CCA event (aggregation, retries at the PHY),
the subtraction under-corrects and channels get scored busier than they
are. The formula has since grown a third term, `leak` (above) — the same
assumption, extended to cover TX that is never decoded as "own" at all
because the scout card isn't the one receiving it.

### Bench findings 2026-09-13: own beacons dominated every reading (fixed the same day)

With two cards the beaconing card's DISC TX (every 20 ms, a few cm from
the scout) leaked into the scout on EVERY channel: on home the leak
decoded as our own frames and `cca − own` subtracted it (home read ~2), on
a candidate it was undecodable energy and counted as busy (~25-40 per
250 ms with zero frames). The same channel read 5 as home and 152 as a
candidate minutes apart. With one card the card's own TX leaked into its
receiver during the home window (home read ~25 against candidates at
0-12). Silencing the beacon only for the home dwell, and parking the home
card 40 MHz away, changed nothing — the leak is on the candidate side — so
parking was dropped again.

Fix: every dwell is silent. Two cards: the TX card holds its DISC while
the scout is observing (`quiet()`). One card: the home/op cycle is a beacon
phase, a quiet gap, then a silent observe with its own discard read. After
the fix (drone off, home 153, candidates 136/149/161): two cards, scout =
card 1: 136 0.7 / 149 6.6 / 153 2.9 / 161 1.6 mean busy (worst 5-10),
floors −92…−96 — home was no longer privileged. One card, card 0: 0.0 /
5.8 / 4.1 / 1.1; card 1: 0.4 / 8.6 / 5.0 / 3.8. Clean channels sat within
~10 units of each other, so with no improvement margin the pick among
clean channels is effectively arbitrary — this is exactly why
`pick_margin` exists and why the shipped default channel set
(`docs/bw40.md` "Channels") is curated to channels worth flying rather
than left to an automatic search of the whole band.
