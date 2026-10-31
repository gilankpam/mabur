# Auto channel selection over a shared channel set

The operator sets nothing about channels on either end. Both ends carry
the same **channel set** (`radio.channels`); the drone always sits on
exactly one member and remembers which; the GS is always on one member
(`op`), remembers which, and knows the set is the only place the drone can
be. With `[radio] channel = "auto"` the GS measures the set once, at
process start, and moves the link to the cleanest member (the **boot
hop**), then freezes for good — later link losses re-propose the same
frozen pick and the scout never measures again. A healthy link never moves
on its own; in auto mode the sub-second reactive layer
(`docs/inflight-channel-hop.md`) still handles an interfered channel after
the freeze. `channel = <member>` pins the link there: no measurement **and
no reactive hop** — the pin is static. A drone found off the pin is still
relocated to it.

There is no home channel: no member is privileged, and nothing falls back
to a fixed channel on link loss.

Design spec: `docs/superpowers/specs/2026-10-03-auto-channel-set-design.md`
(gitignored, local only — this page is the durable record, and where the
two disagree this page describes the code). The reactive layer it builds
on is `docs/inflight-channel-hop.md`.

## Config

Both ends carry the set. The drone's list may be a superset of the GS's,
never a subset: the GS only ever proposes or orders members, and the drone
parks only on members. If the drone's set does not contain the GS's, the
link may never form (a drone parked on a channel the GS never searches is
never found) or never reach the pick (the drone ignores a hop order to a
channel outside its set, so every relocation fails). No loader can check
this across the two devices; it is an operator rule.

**The default set `[40, 64, 112, 144]` is provisional** until bench
row 8 runs (`maburcal` on 64 and 112; swap 64 for 128 if it fails — see
"Open items"). Both bundles carry the same note.

GS, `gs/bundle/maburgs.default.toml`:

```toml
[radio]
channels = [40, 64, 112, 144]   # the set; the drone parks on a member, the GS picks among them
channel  = "auto"               # or a member: pin the link there, no measurement
width    = 40
tx_card  = -1
relays   = []

[radio.scan]
dwell_ms        = 250   # measurement observe per 20 MHz half
settle_ms       = 30    # after every retune
min_rounds      = 3     # full passes before the ranking is mature
search_ms       = 100   # DISC burst at the start of every unlinked dwell on a primary (5 beacon periods)
op_window_ms    = 300   # one card: how long the sole card beacons on op between dwells
search_after_ms = 5000  # on link loss, keep every card on op this long before sweeping
pick_margin     = 20    # auto: the pick must beat the channel the link is on by this many busy units
one_card_ms     = 5000  # one card, auto: measure silently this long before the first DISC
max_ms          = 30000 # auto: the pick is frozen at the latest this long after GS start
busy_dbm        = -83   # both modes: energy level that counts the air as busy (an NHM bucket edge)
blocked_pct     = 50    # both modes: % of busy foreign airtime that makes a half/window "blocked"
```

Drone, `bundle/mabur.default.toml`:

```toml
[radio]
channels = [40, 64, 112, 144]   # parks on the remembered member (else the first); follows the GS to any member
width    = 40
```

### Which knob drives which piece

| piece | runs when | knobs |
|---|---|---|
| **search** — find the drone over the set (`ChannelScout`, search job) | link down, both modes | `radio.scan.search_ms`, `op_window_ms`, `search_after_ms` |
| **measure** — rank the set, pick, freeze (`ChannelScout` measure job, `BootPick`) | pick open, `channel = "auto"` only | `radio.scan.dwell_ms`, `settle_ms`, `min_rounds`, `pick_margin`, `one_card_ms`, `max_ms` |
| **busy air** — what every measurer calls blocked | always | `radio.scan.busy_dbm`, `blocked_pct` (boot ranker, in-flight ranker, verdict alike) |
| **relocate** — the link forms where the drone is, one order moves it to `want` (pin or pick) | each link-up with `op ≠ want`, both modes | `[hop] confirm_ms`, `confirm_extend_ms`, `verify_ms`, `max_hops_per_min`, `one_card_repeats` (shared with the reactive hop) |
| **verdict** — is `op` bad right now | always (OSD / sideport `hop.verdict`) | `[hop] window_ms`, `persist`, `[hop.verdict] *` |
| **reactive hop** — in-flight scout, freshness burst, order/verify/withdraw | `channel = "auto"` only, after the freeze | everything else in `[hop]` |
| **calibration pick** — one measure round, then a relocate to the quietest member (`CalPicker`) | each `maburcal start`, auto mode with two or more cards | `radio.scan.dwell_ms`, `settle_ms`, `pick_margin`, `busy_dbm`, `blocked_pct`; the relocate's `[hop] confirm_ms`, `max_hops_per_min` |
| **one-card mode** | exactly one USB card in the roster (one `[[radio.cards]]` entry, or auto-scan finding one card) | no in-flight scout; `one_card_ms`, `op_window_ms`, `one_card_repeats` apply |

There is no other mode switch: auto and pinned are the only two modes.
(`[hop] enable` existed until 2026-10-04 and made "pin, then roam" the
default — a pinned 40 that read dirty hopped away and `want` followed, so
`scan.pick` said 40 while the link lived elsewhere. It is deleted, and a
removed key fails boot.)

The drone's `[link]` timers that touch channels:

```toml
[link]
failsafe_ms     = 3000     # no word from the ground station this long = drop to the most robust link setting
rc_drain_ms     = 5        # how often commands from the ground station are applied
rendezvous_ms   = 30000    # still silent this long after failsafe = back to RENDEZVOUS (the drone stays on its channel)
move_confirm_ms = 2000     # after a channel move, hear the ground station within this or return to the channel we came from
```

`rendezvous_ms` only drives the FAILSAFE → RENDEZVOUS state change, never a
retune.

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
| `radio.scan.busy_dbm` | −104 – −70, on an NHM bucket edge (`-104 -101 -98 -95 -92 -89 -86 -83 -80 -75 -70`; `gs/src/nhm_busy.h` `busy_dbm_is_edge`) |
| `radio.scan.blocked_pct` | 1.0–100.0 |

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
constants).

Deleting a state file makes that end forget its remembered channel on the
next boot: the drone falls back to `radio.channels[0]`, the GS searches
from `channels[0]` (or the pin) rather than re-finding a drone sitting on a
channel the GS already "knows". Useful when a device was last parked
somewhere you no longer want it defaulting to. The 2026-10-03 deploy of the
channel set is recorded in `docs/findings/2026-08-29-deploy-flag-days.md`.

## Drone behaviour (`RcAgent`, `drone/src/`)

- **Boot channel.** `channel_` = the remembered member (if the state file
  names one that is still in `radio.channels`), else `radio.channels[0]`.
  InitWrite tunes there at `radio.width`. No scan, no search; the drone
  listens and sends `T_TELEM` at ~1 Hz in every state — that is its
  "shout".
- **Follows members only.** A DISC proposing a member the drone is not on:
  the drone acks agreement from its current channel; once the first
  verifying RCF promotes that pair, the retune runs on the next `tick()`
  (after main has sent one Telem saying LINKED on the current channel;
  `act_.retune(ch, "disc")`), and the move is marked unconfirmed. The GS
  never sends such a DISC — every copy proposes the channel it is sent on,
  and the GS moves a linked drone only with hop orders ("Finding the
  drone" below); the path stays for robustness. A DISC proposing a
  **non-member**: the drone acks its own current channel instead (`maburgs`
  logs `ack_override` on `ChannelPlan::on_ack`). An RCF `hop_ch` that is
  not a member is ignored for the retune, but the `(hop_epoch, hop_ch)`
  pair is still recorded as seen, so a repeated non-member order is not
  re-evaluated on every RCF.
- **Unconfirmed move goes back.** After `move_confirm_ms` with nothing
  heard from the GS on the new channel: retune to `move_from_ch_` — the
  channel the drone came from, DISC move or hop alike — and enter
  RENDEZVOUS there ("There is no home to fall through to",
  `drone/src/rc_agent.cpp`).
- **Long loss: stay put.** FAILSAFE entry and the `rendezvous_ms` timeout
  change state only (LINKED → FAILSAFE → RENDEZVOUS); neither retunes.
  The drone is always on exactly one member, which is what makes the GS
  sweep below complete — a search burst will eventually land on whatever
  channel the drone is parked on.
- **Retune mechanics**: TX-gate exclusive, 5 ms drain, `FastRetune`, TX
  power re-applied, calibration deferral (latched and replayed on the
  sweep's falling edge), stderr `maburd: retune A -> B (reason)`. Reasons:
  `disc`, `hop`, `move_unconfirmed`.
- **Wire.** `Disc.op_channel` / `DiscAck.agreed_channel` and
  `Rcf.hop_ch` / `hop_epoch` carry members (`common/include/mabur/rc_proto.h`).

Invariant: the drone is always on a member of its set, and only moves on
a GS proposal, a GS order, or an unconfirmed move back to where it came
from.

## GS: search and measure (`ChannelScout`, `gs/src/channel_scout.{h,cpp}`)

### Where the code lives

Everything this page describes on the GS side is wired inside
`gs/src/channel_core.{h,cpp}` (`ChannelCore`): `main.cpp` builds one with
the card roster and calls it at six seams (lifecycle, the rc sink, the
drain loop, `tick()`, the send path, the sideport). The decision units are
`ChannelPlan`, `BootPick`, `ChannelScout`, `HopVerdict`, `HopRanker`,
`HopController` and `InflightScout`; a calibration run's pick reaches
the core through `CalPicker` (`gs/src/cal_picker.h`, scored by
`gs/src/cal_round_pick.h`). The browser GS drives the same
`ChannelCore` (`web/src/web_gs.cpp`; `docs/web-gs.md`). Design:
`docs/superpowers/specs/2026-10-04-web-gs-channel-core-design.md`
(gitignored, local only; this page and `docs/inflight-channel-hop.md` are
the durable record).

### The scout

`ChannelScout` is the long-lived scheduler that owns the spare USB card
(two-card) or the sole card between op windows (one card) whenever it has
work: `has_work = search || (measure && !frozen)` — it searches while the
link is down and/or measures while the pick is still open (auto mode
only); with no work it parks the card on `op` at `radio.width` and sits
idle.

**Dwell schedule.** Measuring at width 40: round-robin over the 20 MHz
**half set** (`pair_pick.h`'s `scan_half_set` — each member's pair, low
half then high half, config order). Search only (pinned), or measuring at
width 20: the members themselves. Per dwell:

| phase | when | what |
|---|---|---|
| retune + settle | always | `FastRetune(half, 20)`, `settle_ms` |
| **search burst** | link down AND the dwell channel is a member | send DISC proposing that channel every beacon period for `search_ms`; listen for the ack |
| gap | after a burst | one beacon period, so the card's own TX is out of its receiver |
| discard read | measuring | zero the delta counters before the observe |
| **observe** | pick open (auto) | silent `dwell_ms`, read FA/CCA/own/foreign/NHM |

Pinned mode: no observe, ever (`measure = false`) — a sweep of four
primaries ≈ 0.6 s. Auto, link down: burst + observe (≈ 3.2 s per round
over eight halves at the shipped defaults). Auto, linked, pick open:
observe only.

**Op's own pair is never measured while linked.** With the search off
(linked, or inside `search_after_ms` after a loss) and the pick open, the
scout skips both halves of `op`'s pair (`op` alone at `width = 20`): no
retune, no observe. They carry the drone's own video — at width 40
undecodable to a 20 MHz observe and so counted busy, and in NHM's airtime
either way — and scoring them would make the boot hop leave a clean `op`
almost every time. A skipped half is completed in the scheduler (stamped
one dwell ahead) so the round-robin and `rounds()` keep moving. `op` keeps
only its pre-link visits: `ChannelScout::mature()` waits for every member
**except** `op` (and for `op` too while searching, when it is visited like
any other), and `op_ranked()` says whether `op`'s pre-link visits reached
the effective `min_rounds`. When they did not, the pick freezes in place
(`"op unmeasured"`, below).

**The leak.** Unlinked, the TX card holds its DISC during every observe
(`quiet()`), so the observe is silent. Linked, RCFs must flow, so the TX
card is never held; its TX leaks into the adjacent scout as undecodable
energy. The ranker subtracts `k × tx_frames_during_observe` from the
dwell's busy score (`ChannelRanker::busy()`: `max(cca − own − leak, 0) +
fa + foreign`), where `tx_frames_during_observe` is the control-frame
counter delta over the observe span (`ChannelScout::set_tx_frames()`, fed
`ctrl_sent_total_`, incremented once per successful control send across
every card) and `k` (`leak_per_frame`) is a **compiled constant**, `1.0`
(`gs/src/main.cpp`, and `web/src/web_gs.cpp` for the browser GS), not a
config key. The leak is uniform across candidates, so it biases only the
stay-or-move comparison, and `pick_margin` absorbs the residual. `k` is
provisional: bench row 7 could not fit it ("Open items").

Why every dwell is silent and why `cca − own` is subtracted: the scout
card sits centimetres from the beaconing card, whose TX otherwise reads as
busy on every candidate; and one decoded single-MPDU frame costs one CCA
event (bench record: `docs/findings/2026-09-13-channel-select-bench.md`).

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
linked"`), and a link found off `op`/`want` is then relocated by the
one-card hop path.

**Width.** The scout card is opened at 20 MHz and tunes 20 MHz for its
dwells; it rejoins the link at `radio.width` on `op` when it has no work
(`ChannelScout::park_()`). The core's one-shot width resync
(`gs/src/width_resync.h`) covers the scout-card-died and
freeze-before-scout-started edge cases (`docs/bw40.md` "Boot scan").

**Send gating** (`gs/src/channel_core.cpp`): the scout owns a card
(`scout_owns_()`: `working() || search requested || (!pinned &&
pick_open())`) whenever the core must not also touch it — mechanical
retune, TX selection, verdict input, width resync, the in-flight scout and
freshness burst, the relocation gate all key on it. The send gate and DISC
routing use the narrower `working()`/`beaconing()`: one-card mode drops
the sole card's non-DISC frames while it is off `op`; two-card mode holds
the TX card's DISC while `quiet()`. The scout's own DISC bursts are sent
by the scout card from the scout thread's own schedule, not the core
loop's beacon.

**Relays** keep beaconing DISC on `op` whenever `ready()`, but never as
the only path (`scan_disc_targets()` in `gs/src/scout_pick.h`): a CPE
still booting, unplugged or owned by another client must not be the only
way a DISC leaves the GS. A relay-only GS (no USB card) searches the set
on its last relay but never measures (`scan.state` stays `off`).

**Who scouts** (`gs/src/scout_pick.h`): the last scout-capable card — the
spare USB card on a two-USB GS, the only card on a one-USB GS. A CPE510
relay (`docs/cpe510-relay.md`) has no FA/CCA/NHM reads and never scouts.
With no `[[radio.cards]]` block, auto-scan uses every supported USB card
found (two found = two-card mode); an explicit `[[radio.cards]]` list with
one entry is how you fly one card with two plugged in.

## Finding the drone: link where found, then relocate

**A DISC is discovery only; every move of a linked link is a hop order.**
The drone retunes only after the first RCF that verifies under the pending
pair (`docs/link-pairing.md` "Rendezvous, as built"), so a drone found on a
member X ≠ `op` must be linked on X first — RCFs sent on `op` would never
reach it.

- **The DISC proposes the channel it is sent on.** One DISC is built per
  tick proposing `op`; every copy fanned to a card on another member is
  re-proposed and re-tagged for that card's channel
  (`disc_for_channel()`, `gs/src/vrx_controller.h`). A search burst's DISC
  on X therefore proposes X: the drone agrees to stay, and never retunes
  itself on promotion.
- **The link forms where the drone is.** When the ack that **opened** the
  session (BEACONING → SESSION inside that `on_rc_frame` call; our nonce,
  not key-mismatch flagged — a stranger's drone cannot drag the cards) was
  heard on a member X ≠ `op` (that body's `rx_channel`),
  `ChannelPlan::link_found()` moves `op` to X for **every** card (the TX
  card too, via the mechanical retune), the scout parks there, and the
  proposal follows. `M all <op> <X> link_found`; stderr `maburgs channel:
  drone found on X (op Y): the link forms there`.
- **Then it is relocated.** `ChannelPlan::want()` is where the link
  *should* live: the pin in pinned mode, else the start channel, later the
  committed pick. While linked with `op ≠ want` (`relocate_due()`), the
  boot-pick state machine (`gs/src/boot_pick.h`) hands out one **relocate**
  order per link-up — once the scout owns no card, no hop is in flight and
  the controller is `Idle`/`Hold` — through the ordinary hop machinery
  (two cards: the non-TX card leads; one card: the one-card path).
  `HopTick::relocate`: H kind `relocate`; the channel left is not backed
  off as fled; exempt from `cooldown_ms`, counted against
  `max_hops_per_min`; and it is the one hop a pinned GS ever places.
  **Onto the pin the confirm lands it** (`HopTick::no_verify`): there is
  no verify window, because its question — is the channel we landed on
  better than the one we left — has no answer when the target is the only
  place the link may live. `verify_pass` follows `lead_confirm` on the
  next tick; the pin is never backed off and a dirty pin never reads as a
  failed hop. Relocations onto an auto pick keep their verify, and
  `verify_pass` lands them. Anything else — withdraw, verify fail (no
  retry: the only target offered is `want`), a session lost before or
  after the confirm, the hop cap — accepts the channel the link is on
  (`set_want(op)`) until the next link-up. While a relocation is due or in
  flight no reactive order or freshness burst may take the controller.
- **Pinned mode:** a drone found off the pin links there and is relocated
  to the pin on that link-up; if that fails the GS stays where the link is
  until the next loss (then beacons on that `op`) — the sideport shows
  `link.channel ≠ scan.pick`. The reactive hop never runs pinned: an
  interfered pin is reported (`hop.verdict`, the OSD) and stays. The
  controller is never handed a reactive trigger in any arm of the core's
  dispatch (cleared once, before the arms; the relocation arm re-arms it).
- A confirmed hop of either kind moves `want` with `op`: a reactive hop's
  destination is where the link should live from then on, so a later
  link-up does not relocate back onto the channel it fled.

## The pick: maturity, commit, relocation, freeze

**Ranking.** `pair_proposal(all, current, set, min_rounds, pick_margin,
blocked_pct)` (`gs/src/pair_pick.h`; at width 20 `ChannelRanker::proposal`
over single channels): a pair is ranked once both halves have `min_rounds`
visits (the one-card deadline uses the *effective* `min_rounds` — 1, once
the prelude is done and fewer than `min_rounds` full passes exist, else
the configured value — so a one-card prelude pick counts as measured).
Score = the worse half's worst-visit busy; the blocked tier (either half's
NHM busy ≥ `radio.scan.blocked_pct`) ranks after every unblocked pair. The
channel the link currently sits on (`op`) is the reference: a candidate
must beat it by `pick_margin`, ties stay on `op`. Config order breaks
remaining ties. Returns `op` when nothing else is ranked.

The whole pick is `BootPick` (`gs/src/boot_pick.{h,cpp}`, pure, every
exit covered by `tests/test_boot_pick.cpp`); `ChannelCore` only feeds it a
`BootPickIn` per tick and applies its one `BootPickOut`.

**At maturity** (every member but `op` ranked — `op` too while searching —
`ChannelScout::mature()`):

- **Op unmeasured** (`!op_ranked()`): a linked scout never visits `op`'s
  own pair, so a drone found at once leaves `op` with only its pre-link
  visits. A working link is not moved on a one-sided comparison: freeze in
  place (`"op unmeasured"`); the reactive hop covers a dirty `op` later.
- **No link** (GS powered first): `ChannelPlan::commit()` moves `op` and
  `want` (the TX card retunes; stderr `maburgs channel: commit <from> ->
  <to> (no link)`, `M all <from> <to> commit`, the state file is written).
  Freeze (`"commit"`). The drone, when it appears on its remembered
  member, is found there by a search burst, the link forms there, and it
  is relocated to the pick (above).
- **Linked, proposal ≠ op**: `set_want(pick)` and the scout is frozen
  (stops measuring, parks, frees the lead card); the relocation moves the
  link — this **is** the boot hop (`maburgs channel: boot pick wants
  <pick>, link on <op>: relocating`, `H relocate`). `restore_rung` = the
  verdict's `ref_rung` (the current rung — "keep the current rung"). The
  pick freezes when the controller is back in `Idle`/`Hold`: `"relocated"`
  on `verify_pass`, else `"relocate failed, staying"` (the link stays
  where it is, `want` follows it).
- **Linked, one card**: the scout owns the only card while the pick is
  open, so the pick freezes at link-up (`"one-card linked"`) and the
  relocation, if due, follows once the scout has parked, by the one-card
  hop path.
- **Proposal == op**: freeze in place (`"in place"`).

**Freeze** closes the pick for the GS process's lifetime: the scout stops
measuring and rejoins the link at full width. The one later round is a
calibration run's own ("Calibration pick", below), which reuses the scout
without reopening the pick. Logged
`maburgs channel: pick frozen on <pick> (<why>) after <rounds> rounds`
plus a `K` line (`K <t> <picked|none> <rounds>
<ch>:<worst_busy>:<floor|nan>:<busy|-> ... pair=<lo>+<hi>|-`; `picked` is
`want` after the decision — every freeze-in-place exit has accepted `op`
first). The reasons, first match wins: `"scout card died"`, `"one-card
linked"`, `"relocated"` / `"relocate failed, staying"`, `"max_ms"`
(unmeasured: stay where the link is; a relocation in flight is never
yanked — it freezes when it resolves), `"link lost before relocate"` (the
link dropped between wanting the pick and placing the order — the pick
stands, the next link-up relocates there), `"op unmeasured"`, `"commit"`,
`"in place"`, `"calibration running"`.

No eligibility/arm gate: the relocation is placed armed or not. It is a
measured, verified, withdrawn-on-failure move; mid-flight it meets an
armed drone only after a GS restart or a loss, and then it costs what a
reactive hop costs.

Timing at the defaults, two cards, both powered together: link on the
remembered member within ~1 s; three rounds of eight halves ≈ 10 s;
relocation lands ≈ 0.5 s later.

## Calibration pick

Before its first sweep a `maburcal` run (`docs/calibration.md`) moves the
pair onto the quietest member of the set. Only in auto mode with a scout
card (two or more cards, not relay-only); pinned and one-card GSs run on
`op` with no measurement.

- **One measure round.** `ChannelCore::begin_cal_measure()` arms it; it
  starts once the link is live, the boot pick is closed, nothing else
  (a relocate, a hop, a relay sweep, an in-flight dwell, the scout) holds
  the card, and the scout thread is running with its card up
  (`maburgs channel: calibration measure round on card N (op X)`).
  `ChannelScout::begin_round()` gives the scout a fresh ranker for one
  pass: every 20 MHz half of the set once, `op`'s pair skipped as while
  linked, the leak subtracted as usual; then it parks. The round is
  bounded at twice a full round's dwell time, never under 1 s (4.5 s at
  the defaults; `... round overran <n> ms; staying on <op>`). A round
  still waiting when the run starts calibrating is dropped (`... round
  dropped: a run is calibrating`), and the run's abort, failure or pick
  timeout cancels it (`... round cancelled`), so it never holds the card
  outside a pick.
- **Op's reading** comes from the verdict window, not a dwell: the worst
  valid card on `op`, decoded foreign frames scaled to `dwell_ms`, NHM
  busy minus our own airtime. Without a reading newer than the round's
  start, `op` is unread and the run stays.
- **The score** (`cal_round_pick`) uses only terms both instruments
  measure alike: decoded foreign frames plus NHM foreign busy %, the busy
  term only when both sides have a reading (the `cca − own` term is
  dropped: under A-MPDU video it clamps to 0 on `op`). A pair scores as
  its worse half and needs a reading on both. A candidate with no busy
  reading is not ranked while any candidate has one (scored on foreign
  frames alone it would beat the candidates that pay the busy term).
  Leaving `op` takes beating
  it by `pick_margin` in every tier; a blocked candidate never replaces an
  unblocked `op`. Logged as a `K` record with the round's scores and
  `maburgs channel: calibration measure picks <best> (op <op>[ unread:
  staying])`.
- **The relocate** is the boot pick's own: `set_want(best)` and the boot
  pick re-armed, so it hands out one relocate order next tick
  (`maburgs channel: calibration wants <c>, link on <op>: relocating`).
  `H relocate` carries reason `cal`, the order line ends `(cal)`, and the
  confirm lands it with no verify window. Refused, with the run staying on
  `op`: not a member or already there, the link not live, a hop or
  relocate in flight, the boot pick open, the scout or a relay sweep
  holding a card, or the hop cap (`max_hops_per_min`; `... calibration
  relocate to <c> refused: hop cap`). It settles as `landed`, `withdrawn`,
  `held`, `never placed` (not handed out within `confirm_ms`, or the link
  went) or `refused (hop cap)` (`maburgs channel: calibration relocate to
  <c> <outcome>`); every failure leaves `want` on `op`. The remembered
  channel follows a landed relocate like any hop.

From the run's first SWEEP to its end `cal_running` freezes every channel
change (`hop_active()` false, the scout idle); the pick happens before
that, which is why `CalSession`'s `picking` state is outside `running()`.

## In-flight hop after the freeze

After the freeze two things can move the link: the per-link-up relocation
onto `want` (above), and — in auto mode only — the reactive hop
(`docs/inflight-channel-hop.md`), off an `interfered` verdict. The reactive
hop's candidate list is `radio.channels` and its exhaustion path holds
rather than falling back anywhere — see that page's §1, §3, §5 and §7.

## Observability

- **`scan.log`** (`gs/src/scan_log.h`): marker `scanlog 7`. Header line:
  `scanlog 7 channels=<c1,c2,...> mode=<auto|pinned> dwell_ms=<n>
  min_rounds=<n> cards=<n>`. Record shapes are documented in
  `scan_log.h` (`docs/bw40.md`,
  `docs/findings/2026-09-25-nhm-airtime-spike.md` for the width and NHM
  fields); D ends with the dwell's NHM busy % and a relay-sweep rx % (`-`
  for USB dwells). `M` reasons: `commit`, `ack_override`, `link_found`,
  plus the in-flight hop's `hop_lead`/`hop_follow`/`hop_withdraw`/
  `hop_one_card`. `H` kinds include `relocate` alongside the in-flight
  hop's `order`/`verify_fail`/`escape`; every `H` ends with a reason,
  `cal` on a calibration run's relocate and `-` otherwise. A `K` line also
  closes a calibration measure round. Files from before `scanlog 5` have
  `split_home`/`reunite` in `M` and no `relocate`; pre-merge 2026-10-03
  bench builds named `relocate` `boot_order` — read them as what they were
  (`docs/data-provenance.md`).
- **Sideport**: `scan.state` ∈ `off | scouting | moving | frozen` (`off` —
  pinned, no scout-capable card, or a relay-only GS; `scouting` — the pick
  is open; `moving` — the pick's relocation (the boot hop) is in flight;
  `frozen` — the pick is closed). `scan.rounds` (the scout's round count).
  `scan.pick` is **latched**: the channel the pick froze on
  (`ChannelPlan::want()` at the freeze, after any accept-in-place), or the
  pin from the start in pinned mode — never the live `op`, which a
  reactive hop or a found-elsewhere link moves; null while the pick is
  open. `scan.pinned`: the live pin (`ChannelCore::pinned()`), which a
  live `set radio.channel` moves. `link.channel`: the live channel of the
  TX card. There is no `link.home`.
- **stderr**: `maburd: channel set [<c1,c2,...>], parking on <n>[
  (remembered)]` (drone boot); `maburgs channel: set [<c1,c2,...>] mode
  <auto|pinned> start <n>[ (remembered)]` (GS boot); `maburgs channel:
  commit <from> -> <to> (no link)`; `maburgs channel: drone found on <x>
  (op <n>): the link forms there`; `maburgs channel: boot pick wants <pick>,
  link on <op>: relocating`; `maburgs channel: relocate <from> -> <to>
  placed|refused (hop cap)`; `maburgs channel: relocation to <want> did not
  land; staying on <op>`;
  `maburgs channel: pick frozen on <pick> (<why>) after <n> rounds`;
  `maburgs channel: one-card prelude ranking picks <n> (op <n>)[, linked:
  not committed]`; `maburgs channel: drone acked <n>, not in our set;
  ignored`; `maburgs channel: <reason> card <n> <from> -> <to>` (every `M`
  line, echoed to stderr); the calibration pick's `maburgs channel:
  calibration …` lines (above) and a relocate line ending `(cal)`;
  `maburgs channel: could not write
  /etc/maburgs.channel`; `maburd: retune <a> -> <b> (<reason>)`.
- **`tools/maburtop.py`**: shows `scan.state` (including `moving`) and
  `scan.rounds` next to the channel, and `pin` while `scan.pinned`.
- **`tools/flightreport.py`**: reads `scanlog` up to 7 (an `H` line's
  reason as `reason`, `None` before 7); treats `relocate`
  (and the bench builds' `boot_order`) as a HOP-section order kind
  alongside `order`/`verify_fail`/`escape`; still parses `split_home` out
  of an older-marker file, since recordings made before 2026-10-03 carry it
  (`docs/data-provenance.md`).

## Open items

Bench record (2026-10-04, rows 1-10 and the channel-core extraction
regression check): `docs/findings/2026-10-04-channel-set-bench.md`. Still
open from it:

- **Row 6 — the event-score ranker is fooled by a strong co-located
  jammer.** The jam's frames decode once each as `foreign` on the jammed
  pair but leak into every other half as several FA/CCA events, so the
  jammed pair won. NHM busy was the honest signal but sat below
  `blocked_pct = 50`. Candidate fixes: an airtime-weighted foreign term, a
  lower `blocked_pct`, or NHM busy as a primary score term.
- **Row 7 — `leak_per_frame = 1.0` is unfitted.** With the drone centimetres
  from the GS its own TX cannot be separated from the control-frame leak;
  measure with the drone ≥ 20 m away or on a dummy load.
- **Row 8 — the default set.** `maburcal` on 64 and 112 and a fixed-rung
  linkbench 40/2 on 64 vs 136 have not run; swap 64 for 128 if 64 fails
  (`docs/findings/2026-09-26-bw40-channel-candidates.md`).
- Row 5's pinned one-card case has not run.

## Out of scope

A runtime auto/pin switch. Width negotiation over DISC (both ends keep
`radio.width = 40`). Relay channel capability — the operator rule that
every member must be CPE-tunable stays a documented rule,
`docs/cpe510-relay.md` "ath9k quirks" (the current CPE image tunes the
whole 36–177 grid, 144 included; an older image cannot tune 144, and the
scan has no way to ask the relay). 2.4 GHz, per-card channels.

## History

- The pre-2026-10-03 design (a privileged home channel plus candidates,
  boot-only pick, split/reunite on link loss) and its spike:
  `docs/findings/2026-09-13-channel-scan.md`; its bench, which established
  the silent dwell and the `cca − own` subtraction:
  `docs/findings/2026-09-13-channel-select-bench.md`.
- The no-home channel set's bench (2026-10-04):
  `docs/findings/2026-10-04-channel-set-bench.md`.
