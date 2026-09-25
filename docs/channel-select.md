# Auto channel selection

At boot the GS measures a configured candidate list — with a spare card, or
with its only card interleaved with beaconing — ranks channels by that
card's own busy counters, proposes the least busy one in DISC, and the
drone follows. Both ends always fall back to a shared **home** channel
whenever they lose each other, so a reboot or a lost pair can always find
each other without a laptop-side step. The pick is boot-only: it freezes at
the first accepted DISC_ACK and never re-scans on its own — there is no
proactive re-ranking while the link is healthy. Every measurement, the
decision and every retune are logged to a new per-session file, `scan.log`.

**In-flight migration is built** (`docs/inflight-channel-hop.md`,
2026-09-14) as a separate, reactive-only layer on top of this one: it
shares this page's candidate list and home channel, reuses `scan.log` (now
`scanlog 2`) and the same debug-log session directory, but runs its own
verdict engine, ranker and state machine, and does not touch anything
this page describes. The two are cleanly separated: this page's scan
freezes once, at the first DISC_ACK, and everything below still describes
exactly that boot-time behaviour; a healthy link never moves regardless of
which layer is asking. The 1 Hz per-card energy sample this page used to
mention (`scan.log`'s `A` record, `radio.scan.energy_period_ms`) is
**gone** — the in-flight hop's verdict-window reads replaced it as the
in-session energy source; see `docs/inflight-channel-hop.md` and
`docs/observability.md`.

Design spec: `docs/superpowers/specs/2026-09-13-auto-channel-select-design.md`
(gitignored — this page is the durable record). Continues the spike in
`docs/channel-scan-findings-2026-09-13.md` (lands via the separate branch
`docs-channel-scan-2026-09-13`).

## Config

Both ends keep `radio.channel` as a plain number. It means **home**: the
channel each end boots on and the channel both return to whenever they
lose each other. There is no `"auto"` value — both ends need a concrete
channel to find each other on.

GS, `gs/bundle/maburgs.default.toml` (relevant keys):

```toml
[radio]
channel = 136
width   = 40               # HT40 on the standard pair (132+136 for home 136)
tx_card = -1               # -1 = auto-select the best-SNR card

# Boot-time channel scan: while waiting for the drone the spare card measures
# these plus `channel` (home) and the DISC proposes the least busy one. The
# pick freezes at the first DISC_ACK; a GS restart is the only re-scan.
# One card: the same card alternates home windows and dwells.
# Candidates are 40 MHz pair PRIMARIES on home's side of the grid (home 136 =
# 132+136, primary upper half), all spur-free: 144 (140+144, next door),
# 64 (60+64) and 112 (108+112), the DFS block clear of analog/DJI/Walksnail
# (docs/bw40.md "Channels").
[radio.scan]
enable           = true
candidates       = [144, 64, 112]
dwell_ms         = 250
settle_ms        = 30
min_rounds       = 3
home_window_ms   = 300
split_after_ms   = 5000     # after link loss, beacon on the op channel this long, then also on home
home_margin      = 20       # leave home only if a candidate's worst visit is >= 20 busy units lower
```

Drone, `bundle/mabur.default.toml` (verbatim, the relevant keys):

```toml
[radio]
usb_vid    = 3034
usb_pid    = 0
channel    = 136
width      = 40        # HT40 on the standard pair (132+136 for home 136); per-rung width comes from the GS ladder (docs/bw40.md)
follow_gs  = true        # honour the GS's DISC op_channel (auto channel select)
power_mode = "none"      # set to offset to use rate_walls_rel
tx_threads = 4
```

```toml
[link]
vtx_id        = 1
failsafe_ms   = 3000
rc_drain_ms   = 5
rendezvous_ms = 30000
move_confirm_ms = 2000  # after a GS-commanded retune, hear the GS within this or return home
tick_ms       = 100
```

Validation (`gs/src/config.cpp`, `drone/src/config.cpp`): every candidate in
`[1,177]`; `dwell_ms` in `[50,10000]`; `settle_ms` in `[0,1000]`;
`min_rounds` in `[1,100]`; `home_margin` in `[0,100000]`; `home_window_ms` in `[40,10000]`;
`split_after_ms` in `[0,600000]`; `move_confirm_ms` in `[200,30000]`;
unknown keys fail boot as everywhere. `radio.scan.energy_period_ms` and the
`scan.log` `A` record it drove are removed (2026-09-14) — see
`docs/inflight-channel-hop.md`.
`radio.scan.enable = false` makes every DISC propose home and nothing
moves. `follow_gs = false` makes the drone ack home and never retune, so
the GS's `ChannelPlan` never sees a disagreeing ack and stays on home too.

**These two configs are coupled.** On a one-card GS the single radio is
also the scout, so after the first ack it can be away from the link
channel — finishing a dwell, settling, or serving the split's home
window — for up to one scout cycle, `2·settle_ms + home_window_ms +
beacon period + 2·dwell_ms` (880 ms at the shipped defaults) before the drone hears anything from it. That sum must
stay well under the drone's `link.move_confirm_ms` (2000 ms), or the drone
declares the move unconfirmed and goes home while the GS is merely
mid-hop, and the pair retries the move forever.

No `[[radio.cards]]` block pins nothing: `maburgs` auto-probes the USB bus
and uses every supported card it finds, which is two-card mode. Adding an
explicit `[[radio.cards]]` block (commented out by default) pins exactly
that set — one entry is how you fly one card without unplugging an
antenna, and switches the GS into one-card interleave mode below.

**Calibration:** `maburcal` stores walls relative to the chip's
per-channel anchor (`docs/calibration.md`), so one run on home is valid
on every candidate; no per-channel tables. `maburd` re-applies TX power
right after every retune, so the walls land on the new channel's own
anchor rather than the boot channel's.

## Rules

- **Home keeps a margin.** A candidate replaces home only if its worst visit
  is at least `home_margin` busy units below home's (default 20: a clean
  channel reads 0-10, a weak AP 20-140, a router or the FPV band 145-445).
  Among the candidates themselves the lowest worst visit still wins.
- At `radio.width = 40` candidates are pair primaries sharing home's
  `ht40_offset`; the boot scan scores both halves and the pick is per pair
  (`docs/bw40.md`).
- **Blocked tier (2026-09-25).** A channel whose worst-visit NHM busy %
  (`hop.verdict.busy_dbm`/`blocked_pct`, spec
  `docs/superpowers/specs/2026-09-25-nhm-airtime-design.md`) reaches
  `blocked_pct` ranks after every unblocked channel regardless of its
  event score (`worst_busy`/floor), tiebroken among the blocked by lower
  busy %; `home_margin` still applies, but only within a tier — a blocked
  home loses to any unblocked candidate outright. At `radio.width = 40` a
  pair is blocked if **either** half is (`docs/bw40.md`). Rationale and
  measured defaults in
  `docs/nhm-airtime-spike-findings-2026-09-25.md`.

- **Home is a number on both ends**, configured independently; both must
  agree on it out of band (it is never negotiated). A cold boot, a
  reboot, or either end losing the other for long enough always lands
  back on home.
- **Boot-only pick, frozen at first ack.** The GS scout proposes the
  ranker's best candidate once `min_rounds` full passes are complete
  (home before that). The first DISC_ACK a peer accepts freezes the pick
  for the GS process's lifetime — later link losses re-propose the same
  frozen pick and the scout never runs again. A drone that appears before
  the scan matures gets home; that is the price of boot-only.
- **Ack-on-home, then move.** A DISC is a proposal, never a command. The
  drone acks agreement to a NEW channel from the channel it is CURRENTLY
  on — the ack itself must still reach the GS on the channel the GS is
  listening on — and only then requests its own retune. A DISC proposing
  the drone's current channel is a no-op ack. This makes the exchange
  idempotent: a repeated DISC for a move already agreed to, or already
  made, touches nothing further.
- **Invariant: the drone is on the op channel only while LINKED or
  FAILSAFE, home otherwise.** Entering RENDEZVOUS by any path — the
  ordinary failsafe timeout, or an unconfirmed move — retunes home first.
  `channel_` is constructed from `cfg.radio.channel`, so a reboot lands on
  home with no special-case code.
- **Move confirm.** After a GS-commanded retune the drone marks the move
  unconfirmed and waits `move_confirm_ms` (2 s default) for anything from
  the GS on the new channel. If nothing arrives it retunes home and
  re-enters RENDEZVOUS rather than waiting out the full
  `failsafe_ms + rendezvous_ms` (33 s at the shipped defaults) on a
  channel where the GS never hears it. On the GS's usual 50-70%
  per-frame uplink odds this costs about 2 s per retry cycle.
- **Retune mechanics.** The drone's `Actuator::retune()` takes the TX gate
  exclusive against every USB sender, sleeps 5 ms so the DISC_ACK that
  precedes the retune (sent, per the rule above, on the OLD channel) has
  time to actually leave the antenna before the chip is reprogrammed out
  from under it, then calls `FastRetune`. Skipping that drain would race
  the ack out onto the NEW channel, where the GS — still listening on the
  channel it proposed the move from — never hears it, and every single
  move would fall into the lost-ack retry cycle. TX power survives a
  retune: `FastRetune` never rewrites TXAGC. Every move prints
  `maburd: retune <from> -> <to> (<reason>)` on stderr, where `reason` is
  `disc` (a DISC proposed a channel we agreed to), `move_unconfirmed` (the
  confirm window expired) or `rendezvous` (the rendezvous timer sent us
  home) — all three can end on home, so the number pair alone does not say
  which fired. A retune requested while a calibration sweep is running is
  latched instead of performed and prints
  `maburd: retune <from> -> <to> (<reason>) deferred (calibration active)`;
  the agent replays it on the sweep's falling edge (devourer forbids a
  channel set concurrent with the sweep's TX-power calls, and a sweep
  outlasts `rendezvous_ms` by 6x).
- **GS split after `split_after_ms`.** On link loss the GS stays on the
  op channel with every card and keeps beaconing there for
  `split_after_ms` (5 s default), so a short fade resumes in place with
  no retune on either end and two-card diversity holds through it. Past
  that window the rendezvous set becomes `{op, home}`: with two cards,
  card 0 goes home and beacons there while the other card keeps beaconing
  on the op channel; with one card, the card interleaves — a
  `home_window_ms` window on home (beacon every 20 ms and listen, leaving
  only after one quiet beacon period so a landed ack has time to arrive),
  then a candidate dwell, then home again. The first ack or video heard on
  either channel re-unites every card there; the GS never needs to know
  the drone's timers.
- **The scan does not stop at `min_rounds`**; that is only the floor
  below which the DISC proposal is home. The scan does stop if the scout
  card dies mid-scan — the scan is abandoned and frozen on whatever the
  ranker has measured so far, GS-only: the drone has no idea a scan is
  running at all. Logged as a GS stderr line (`maburgs channel: scout
  card N died, scan abandoned at R rounds, card rejoins at W MHz once
  reopened`, `gs/src/main.cpp`; W = `radio.width` — the revive, or the core
  loop's one-shot width resync, brings the card back at the link width);
  see `scan.log` below for the frozen pick that results.
- **One-card mode gates every send while the scout is off-home.** With a
  single pinned card the same radio is doing scouting and TX, so the core
  thread sends DISC (and everything else) only while the scout reports it
  is on home; the stderr status line's `scoutgate=<n>` counter is the
  cumulative count of sends withheld this way.

## Boot / battery-swap timeline

A drone power-cycle with the GS already up and its pick already frozen
from an earlier boot:

1. Pilot swaps the battery. The air link drops; both radios are still on
   the frozen op channel.
2. The GS's `ChannelPlan` keeps every card beaconing on the op channel for
   `split_after_ms`, in case this is just a fade.
3. Past `split_after_ms` with still no ack, the plan enters the `{op,
   home}` split: two cards diverge (card 0 → home) or a single card starts
   interleaving home windows and op-channel dwells.
4. The drone reboots. `RcAgent`'s `channel_` is constructed from
   `cfg.radio.channel` (home) and it starts in BOOT → RENDEZVOUS — on home
   by construction, no scan of its own.
5. The drone, listening on home, hears the GS's home-window beacon
   carrying the frozen pick as `Disc.op_channel`.
6. Because `follow_gs` is true and the proposed channel differs from
   home, the drone acks agreement **from home**, then requests its own
   retune to the op channel and marks the move unconfirmed.
7. The GS reads `agreed_channel` off that ack; it matches the frozen op
   channel, so the plan reunites every card there (`M ... reunite`) — no
   new commit, since the pick did not change.
8. The drone retunes (5 ms TX drain, then `FastRetune`) and waits up to
   `move_confirm_ms` for a GS frame on the new channel. The first
   accepted DISC or RCF from the GS confirms the move (the GS→drone wire
   carries only DISC and RCF, never video); LINKED follows on the next
   accepted DISC.
9. If the ack never lands, the drone waits out `move_confirm_ms`, goes
   back home, and retries the ack on the next beacon — roughly a 2 s
   cycle at the uplink's usual per-frame odds until one lands.

## `scan.log`

New per-session file in the GS debug-log session directory (see
`docs/observability.md`), opened whenever `debug_log.enable` is set, like
`ctl.log`. Marker `scanlog 4` (bumped from `scanlog 3` by the NHM airtime
work, 2026-09-25, `docs/nhm-airtime-spike-findings-2026-09-25.md`;
`scanlog 3` was bumped from `scanlog 2` by the 40 MHz rungs work,
2026-09-24, `docs/bw40.md`; `scanlog 2` itself was bumped from `scanlog 1`
by `docs/inflight-channel-hop.md`, which added the `V`/`H` records below
and removed `A`). Space-separated; formats locked by
`tests/test_scan_log.cpp`; `nan` for an invalid float, `-` for an invalid
int. Copied verbatim from `gs/src/scan_log.h`, the boot-time-scan records
only (the in-flight hop's `V`/`H` formats, and `D`'s trailing in-session
columns, are in `docs/inflight-channel-hop.md`):

```
scanlog 4 <header_info>
C <t> <card> <chip> <gen> <tx>x<rx> <bw_mask_hex> <tune5g_lo>-<tune5g_hi>
  <fast_retune> <fa_ok> <igi_ok> <nhm_ok> <floor_ok>        # card caps
D <t> <card> <ch> <round> <observe_ms> <cca> <fa> <own> <foreign> <igi|->
  <floor_dbm|nan> <flags_hex> <sess> <to_us> <read_us> <back_us> <bw>
  <busy|->                                                    # one scout dwell
K <t> <picked|none> <rounds> <ch>:<worst_busy>:<floor|nan>:<busy|-> ...
  pair=<lo>+<hi>|-                                            # the pick
M <t> <card|all> <from> <to> <reason>                        # a link move
```

`scanlog 3` added one trailing column to each of `D` and `K`: `D`'s `<bw>`
is the dwell's tuned width (20 during a boot scan — every half is scanned
at 20 MHz — `radio.width` for an in-flight dwell); `K`'s trailing
`pair=<lo>+<hi>|-` is the picked channel's standard 40 MHz pair, `-` when
`radio.width` is 20 or no pick was made. `scanlog 4` adds NHM busy-airtime
evidence to both: `D` gains a further trailing `<busy|->` (the NHM busy %
over the dwell's observe span, `-` when the card has no NHM or the read was
invalid); `K`'s per-channel entries change shape from `ch:worst_busy[:floor]`
(floor optional) to `ch:worst_busy:<floor|nan>:<busy|->` (floor now always
present, as a number or `nan`, so the entry's field count is fixed) — see
`docs/data-provenance.md` for the break. A `scanlog 3` file has the old K
entry shape and no `D` busy column; a `scanlog 2` file has neither the `bw`
nor the `busy` column. Detail on the 40 MHz boot scan (dwelling every half,
picking a pair) is in `docs/bw40.md`; detail on the NHM busy evidence
(verdict `blocked` bit, both rankers' blocked tier) is in
`docs/inflight-channel-hop.md` §2/§3 and
`docs/nhm-airtime-spike-findings-2026-09-25.md`.

- **C** — once per card at bring-up: `GetAdapterCaps` identity (chip,
  generation, chains, `bw_mask`, tunable 5 GHz span, fast-retune flag)
  plus the four validity flags of one `GetRxEnergy(true)` read.
- **D** — one per scout dwell, the `SurveyDwell` fields the ranker
  consumes; `flags` is the chanmig `SurveyFlag` mask in hex. Boot-time
  dwells and the in-flight hop's dwells share this record; the trailing
  `sess`/`to_us`/`read_us`/`back_us` columns are `0 0 0 0` for a boot-time
  dwell and populated for an in-session one (`docs/inflight-channel-hop.md`
  §3), the next trailing `bw` is the dwell's tuned width (`scanlog 3`), and
  the final trailing `busy` is the dwell's NHM reading (`scanlog 4`).
- **K** — the pick at freeze: rounds completed and the full ranking as
  `ch:worst_busy:floor:busy` entries (unranked channels omitted, `scanlog
  4` shape), so the decision is reproducible from the log alone, plus the
  trailing `pair=<lo>+<hi>|-` (`scanlog 3`). `K <t> none 0 pair=-` when a
  peer appeared before `min_rounds`.
- **M** — every GS retune that changes where the link lives: `commit`,
  `ack_override` (the ack disagreed with the proposal and won anyway),
  `split_home` (entering the `{op, home}` set), `reunite`, plus the
  in-flight hop's `hop_lead`/`hop_follow`/`hop_withdraw`/`hop_one_card`
  (`docs/inflight-channel-hop.md` §1). Scout dwells and one-card interleave
  hops are NOT `M` lines — `D` carries the dwells, and the interleave is
  implied by `split_home`.
- **A** (removed 2026-09-14) used to carry one card's in-flight
  frame-free energy sample every `radio.scan.energy_period_ms` while
  linked. Both the record and the config key are gone: the in-flight
  hop's verdict-engine window reads replaced it as the in-session energy
  source, at the verdict engine's ~150 ms cadence instead of 1 Hz — see
  `docs/inflight-channel-hop.md` and `docs/observability.md`.

## Sideport keys (as built)

The player's compact GS bar marks the channel `ch:149(a)` whenever the GS
reports `scan.state` other than `off`, so a pilot can tell an auto-selected
channel from a configured one; an older maburgs with no `scan` block shows
the plain `ch:149`.

The spec sketched a `radio.*` block; the shipped schema instead extends the
existing `link` object and adds one new top-level object, to match what
already existed (`link.channel` predates this feature and already drives
the player OSD's `ch` field):

- `link.channel` — the live channel of the GS's TX card.
- `link.home` — the configured home channel.
- `scan` (top level, not under `link` — it outlives any one session and
  describes the receiver's own scan/freeze state, not the link it
  eventually picks): `scan.state` (`off|scouting|frozen`), `scan.rounds`,
  `scan.pick` (the frozen channel, or `null` before freeze).
- `cards[i].energy` — `{cca, fa, own, foreign, igi}`, `igi` itself `null`
  when that card's IGI read is invalid; `null` if none has been taken yet.
  Originally the last 1 Hz `A`-record sample; since 2026-09-14 it is
  refilled every ~150 ms from the in-flight hop's verdict-engine window
  instead (`docs/inflight-channel-hop.md`), so it no longer goes stale for
  up to a second between samples.

`flightreport.py` had no SCAN section as of this design; the log-file
parser it deferred landed with the in-flight hop instead
(`tools/flightreport.py`'s HOP section reads `V`/`H`/`D` off `scanlog 2` —
see `docs/inflight-channel-hop.md` — it does not parse the boot-time
`C`/`K`/`M` records this page describes). `tools/maburtop.py` gained the
new header fields (`scan.state` and `scan.rounds`, and `link.home` next to
`link.channel` — it does not display `scan.pick`, which the sideport still
emits) and a per-card `busy` column computing
`(cca − min(cca, own)) + fa + foreign` from `cards[i].energy` — the same
score the ranker uses, clamped so a card whose own-frame count exceeds its
CCA count reads 0 rather than going negative.

## The `cca − own` assumption

The ranker's busy score (`gs/src/channel_ranker.{h,cpp}`) is:

```
busy = (cca_ofdm − canonical_frames) + fa_ofdm + foreign_frames
```

`cca_ofdm` counts every OFDM CCA event the chip saw, including the ones
caused by our own beacon and RCF traffic — subtracting `own` (canonical-SA
frames decoded) is meant to leave foreign busy only, on the assumption that
one decoded own-frame costs exactly one CCA event. If a decoded frame
actually costs more than one CCA event (aggregation, retries at the PHY),
the subtraction under-corrects and channels get scored busier than they
are. This is exactly the check Task 14's bench validation runs: compare a
beacon card's `A`-line `cca` against `own` during a linked session at MCS
0, and if the gap exceeds the expected foreign/false-alarm floor, replace
the `− own` term with a measured `k × own` and re-run
`test_channel_ranker` pinned to that constant.

## Bench validation 2026-09-13

Bench, drone on the desk, GS session `/media/dvr/log/0069` (`scan.log`
carries every K/M line below). Deployed binary-before-config on both ends;
rollbacks `maburgs.pre-chansel` / `maburd.pre-chansel` + `*.toml.pre-chansel`
(the drone's `maburd.pre-cal` was pruned to make room).

| check | result |
|---|---|
| C record, both GS cards | `RTL8822E jaguar3 2x2 1f 5080-6165 1 1 1 1 1` — all four sensors valid incl. the absolute floor; the drone (no floor opt-in) reports `floor=0` |
| scout cadence, two cards | 11 visits per channel in 18 s (round ≈ 1.1 s at the defaults) |
| pick, two cards, home 136, cands 149/153/161 | `K 136 44 136:5:-95 149:52:-94 153:53:-92 161:53:-95`, `M all 136 136 commit`, no drone retune |
| pick, two cards, home 153, cands 136/149/161 | `K 153 15 153:9 136:152 149:45 161:39` — home won again; 136 took one 152-busy visit |
| pick + move, ONE card, home 153 | `K 161 17 153:32 136:42 149:12 161:8`, `M all 153 161 commit`, drone `retune 153 -> 161 (disc)`, video in SESSION on 161; `scoutgate=3` sends held during the join window |
| ausniff on the moved link | 60.2 fps / 1 gap first pass (post-restart phantom), then 60.3 fps / 0 gaps / 0 incomplete |
| link loss (drone stopped), one card | `M 0 161 153 split_home` at loss + 5 s |
| drone restart, one card | drone boots on 153, caught in a home window: GS `M 0 153 161 reunite`, drone `retune 153 -> 161 (disc)`, 60.4 fps |
| `cca − own` (beacons) | on home the scouting card saw own=12 beacons and cca=12: one decoded single-MPDU frame is one CCA event |
| `cca − own` (video, A records) | cca 226-948 against own ≈ 2200 frames/s: CCA counts PPDUs, so under A-MPDU `cca − own` clamps to 0 and the A record's busy is `fa + foreign` |
| A records at 1 Hz | no change in ausniff cadence (60.3 fps with them on) |
| restored flight config (home 136, two cards) | `K 136 10 136:1 149:50 153:53 161:58`, commit home, 60.5 fps |
| interference test after the fix: home 136, candidates 44 (router, 80 MHz on 36-48), 120 (clean), 149/153/157/161 (neighbourhood), drone off then on | 13 rounds: 44 worst 274 (305 foreign frames), 157 worst 145 (undecodable), 149 worst 20 (43 frames), 120/136/153/161 at 1-2; at freeze (26 rounds) `K 136 26 136:6 44:336 120:62 149:140 153:67 157:445 161:16` — the worst-visit rule caught later bursts on 120/153 and home won; commit home, 60.4 fps |

Not done: the 2 s RF fade (needs an antenna pull), the two-card split/reunite
(needs a home-losing pick, see the bias below), the `tx_gate` exclusive
latency measurement, and `lat.log` e2e with A records on vs off.

### Findings

**The readings were dominated by our own beacons (FIXED the same day).**
With two cards the beaconing card's DISC TX (every 20 ms, a few cm from the
scout) leaks into the scout on EVERY channel: on home the leak decodes as our
own frames and `cca − own` subtracts it (home read ~2), on a candidate it is
undecodable energy and counts as busy (~25-40 per 250 ms with zero frames).
The same channel read 5 as home and 152 as a candidate minutes apart. With
one card the card's own TX leaked into its receiver during the home window
(home read ~25 against candidates at 0-12). Silencing the beacon only for
the home dwell, and parking the home card 40 MHz away, changed nothing —
the leak is on the candidate side — so parking was dropped again.

Fix: every dwell is silent. Two cards: `ChannelScout::quiet()` is true
during every dwell and the core sends no DISC while it is; once per
scheduler round the scout opens a beacon window of `home_window_ms` (quiet
false) so the drone can still be found mid-scan — a round is now ~1.4-1.9 s
instead of ~1.1 s, and worst-case discovery latency during the scan is one
round. One card: the home cycle is a beacon phase (`home_window_ms`,
`at_home()`), a quiet gap, then a silent `dwell_ms` observe with its own
discard read (~0.9 s per candidate instead of ~0.6 s). Home's D line now
carries `own = 0` and `observe_ms = dwell_ms`.

After the fix (drone off, home 153, candidates 136/149/161): two cards,
scout = card 1: 136 0.7 / 149 6.6 / 153 2.9 / 161 1.6 mean busy (worst
5-10), floors −92…−96 — home is no longer privileged. One card, card 0:
0.0 / 5.8 / 4.1 / 1.1; card 1: 0.4 / 8.6 / 5.0 / 3.8. Clean channels now
sit within ~10 units of each other, so with no improvement margin the pick
among clean channels is effectively arbitrary — keep `candidates` to
channels you are happy to fly (calibrated ones under `power_mode =
"offset"`), or leave it empty to stay on home unless home is clearly busy.

**`[[radio.cards]]` needs decimal VIDs.** The TOML subset rejects `0x0bda`
(`'0x0bda' is not a valid value`) and maburgs crash-loops on the respawn;
the bundle's commented example now says `3034`.

**The GS's own Wi-Fi AP drops stations around a restart.** The `aicwf_sdio`
AP logged STA churn at the moment of a maburgs restart (its USB resets), and
a laptop on that AP saw "no route to host" for ~20 s. Not a reboot: uptime
and the session directory were continuous.

## Deploy

**Binary before config, unusually** (the same exception `docs/deploy.md`
records for the GS card auto-scan). Every key this feature adds has a
default, so the NEW binaries boot unchanged on the OLD config. That is not
the same as scanning being off: `radio.scan.enable` defaults **true**, so
the scout DOES run — but `candidates` defaults empty, so the only channel
it ever visits is home, the pick is home, and the DISC proposes home. The
link therefore stays pinned to home because there is nothing else to
propose, not because the feature is disabled; on a one-card GS the DISC
send is additionally gated by the home window. Config-first would be the
unsafe order here: the old binary rejects the new keys, exits, and its
wrapper respawns it
forever at 2 s. So swap `maburgs` and `maburd`, confirm both are up, then
push `gs/bundle/maburgs.default.toml` → GS `/etc/maburgs.toml` and
`bundle/mabur.default.toml` → drone `/etc/mabur.toml`.

Both ends, but no `RC_VERSION` bump:
`Disc.op_channel`/`DiscAck.agreed_channel` are existing wire fields that
both ends now mean literally, so an old binary paired with a new one just
never moves off home rather than desyncing — there is no flag day here,
unlike most wire changes in this repo. Rolling a binary back means
restoring its old config alongside it, as always.

## Out of scope

In-flight channel migration was out of scope for this design — it is now
built as a separate reactive layer, `docs/inflight-channel-hop.md`
(2026-09-14); this page's own pick still never moves again once frozen,
and the boot-time scan described above is unchanged by that feature's
arrival. Still out of scope, for both pages: proactive re-ranking of a
healthy link, 40 MHz and 2.4 GHz candidates, and per-card channels.
