# Handover — in-flight channel hop: bench, flight and validation (2026-09-15)

Everything in `docs/superpowers/plans/2026-09-14-inflight-channel-hop.md` that
can be done on a host is done, and the bench half was done on 2026-09-15
(deployed both ends, matrix rows 1/4/5/6, four fixes — see "What the bench
found"). What is left is the flights and the PR. This page is what a fresh
session (or a human at the bench) needs to finish the job.

Read `docs/inflight-channel-hop.md` first — it describes what was built. This
page only covers **what is left**, and the things a bench run could invalidate.

## State

| | |
|---|---|
| Branch | `inflight-hop`, off `master` at `f51f8e0`; bench session added 5 commits on top of `4679ff8` (`dfe09ad` verdict units, `fd1b8d5` session gate, `17b7764` TX freeze, `9ef9b00` flightreport loader, plus this doc) |
| Host suite | 144/144 (`ctest -R 'test_\|host_e2e\|gs_e2e\|gs_au_e2e\|player_e2e'`) |
| Cross-builds | `tools/build-arm64.sh` and `tools/build-arm.sh` both clean |
| **Deployed** | **Yes, both ends, 2026-09-15.** Drone `maburd` (rollback `/usr/bin/maburd.pre-hop`, drone config untouched). GS `maburgs` at `17b7764` (rollback `maburgs.pre-hop` + `/etc/maburgs.toml.pre-hop`), `maburplay` (`maburplay.pre-hop`), `/usr/bin/maburtop`. GS config carries `[hop]`/`[hop.verdict]`, `energy_period_ms` removed. |
| **Benched** | **Yes** — rows 1, 4, 5, 6 of the matrix plus both dwell-period baselines; results in `docs/inflight-channel-hop.md` Measurements. Rows 2 (O4) and 3 (fade) not run. |
| Flown | **No.** |
| PR | Not opened — the owner's call. |
| `hop.enable` | **`false`** on the GS again (shipped default) after the bench; the GS is back on two cards, `dwell_period_ms` 333. |

The devourer half is on `../devourer` branch **`feat/scout-energy-ctrl-batch`**
(head `3ad4c55` at bench time), still **not on devourer `master`**; the GS
build consumed it. Keep it checked out (or merge it) before any GS rebuild.

## What the bench found — read before the next run

Four bugs, all fixed on the branch and re-verified on air, plus four
behaviours that are design calls left for the owner.

**Fixed (each has a unit test and a commit message with the trace):**

1. **Verdict fed raw units** (`dfe09ad`). `VerdictCardIn::rssi_dbm/snr_db`
   received the aggregator's raw EMAs (RSSI 64, SNR 66) against
   `weak_rssi_dbm -78` / `weak_snr_db 12`: `weak` could never trip, so
   `Fade` was unreachable on hardware. V lines now read `-58.0 31.4`.
2. **Hop block ran during the boot rendezvous** (`fd1b8d5`). With the
   link still coming up the s1 loss window reads 80–90 % and the boot
   scout's own dwells read as `raised`, i.e. `interfered` by construction.
   The shadow controller `would_order`ed 149, 165, 120 and exhausted before
   the boot pick's K line; the freshness burst retuned the boot scout's
   card mid-dwell. Everything now sits behind `hop_active(in_session,
   scout_joined)`.
3. **TX selector switched onto the lead card mid-hop** (`17b7764`). The
   decisive one: within 200 ms of three of the first run's four orders the
   RCF uplink — the frames carrying the order — moved to the target
   channel. Two orders the drone never saw at all (Telem `hop_epoch` echo
   never advanced), one it saw late and got stranded on. The order whose
   TX card happened to stay put confirmed in 139 ms. Same hold the scout
   dwell already had, now for `ChannelPlan::hopping()` too.
4. **`flightreport` skipped the HOP section on a rejoined session**
   (`9ef9b00`). The first `scan.log` after a deploy starts with the old
   binary's `scanlog 1`; the loader read line 1 only.

**Design calls, not changed (numbers in `docs/inflight-channel-hop.md`):**

- `impaired` trips on 65 % of rest windows (the `recovered_x × trailing
  mean` term vs one lost aggregate) — `ref_rung` is frozen nearly always.
- A withdrawn order strands the drone for `move_confirm_ms` and every
  order placed meanwhile burns the rate cap (36 s `hold_cap` in run 1).
- The origin channel is never backed off, and a 5 ms observe scores a
  250 f/s jammer at ~6, so a `verify_fail` can hop straight back into it.
- One-card hops cost a 4-rung ladder cascade (drone ~200 ms ahead of the
  sole GS radio during the five-RCF order window).

**Bench procedure that worked** (for rows 2/3 and any rerun): jam with
`DEVOURER_TX_SA=02:4a:41:4d:00:01 tools/bench/benchjam.sh --channel <op>
--secs 70` — without the SA override txdemo's default is mabur's canonical
address and the GS counts the jam as its OWN traffic. Read the op from the
sideport (`link.channel`) after every GS restart: the boot scan re-picks
(165 three times, 120 once today). `ausniff --seconds 100` in parallel.
Evidence: `H`/`V`/`D` in the session `scan.log`, `E hop_restore` in
`ctl.log`, `drone.channel`/`drone.hop_epoch`/`link.tx_card` at 200 ms in
`flight.jsonl` (that record is what exposed bug 3), `maburd: retune ...
(hop)` lines in the drone's `/tmp/mabur.log` (no timestamps).

## Deploy — a flag day, and the plan's own advice is wrong

`RC_VERSION` goes **8 → 9**. Both binaries must swap. Between the two swaps
the pair is mismatched, and a mismatched pair has no control link and — since
`DISC_ACK` carries `CAP_FRAME_WIRE` — **no video at all**, which looks exactly
like the stale-caps deadlock. Restarting a daemon does not fix it; finishing
the deploy does.

The GS config also changes in both directions: `[hop]` and `[hop.verdict]` are
added, and `radio.scan.energy_period_ms` is **removed**. That means neither
ordering works on the GS — an old binary rejects the new `[hop]` table, and a
new binary rejects the leftover `energy_period_ms`. Either way the daemon
exits and its wrapper crash-loops it forever at 2 s with a repeating
`unknown key` line in `/tmp/maburgs.log`.

> Task 11 step 6 of the plan says "binary-before-config on the GS". **That is
> wrong for this branch** and contradicts `docs/deploy.md`. Do not follow it.

The correct sequence (`docs/deploy.md:32`):

1. **Stop both daemons.**
2. `df` the drone and prune — the rootfs fits at most 2 `maburd` binaries.
3. Swap the **GS config and GS binary together**. The config needs `[hop]` +
   `[hop.verdict]` added (copy from `gs/bundle/maburgs.default.toml`) and
   `radio.scan.energy_period_ms` deleted.
4. Swap the **drone binary**. The drone config is **unchanged** by this branch
   (`bundle/mabur.default.toml` untouched) — do not edit it.
5. **Start both.**

Rollback is paired: an old binary needs its old config restored alongside it.
Rolling forward is usually shorter.

## What has not been done

Task 15 step 5 (flights) and step 6 (PR); matrix rows 2 and 3. Sections 1
and 2 below are DONE as of 2026-09-15 — kept as the procedure for a rerun.

### 1. Baseline — dwells on, hops off

Deploy per above with `hop.enable = false` (the shipped default), then:

```
tools/bench/ausniff.py --seconds 60
```

Expect: 59.7 fps class, 0 gaps. Then check `scan.log` in the session
directory for `V` lines (verdict windows, in-session only since `fd1b8d5`) and `D`
lines with `sess 1` carrying `to_us`/`read_us`/`back_us`.

**This is also the first real measurement of the dwell cost.** The design
budgeted ~10 ms with the devourer work in place, against 26 ms without. Record
the three step timings — they decide whether `dwell_period_ms = 333` stays the
default, and they fill the first table in `docs/inflight-channel-hop.md`.

Repeat with `dwell_period_ms` at 100 to see the cost at 3× the rate.

### 2. Condition matrix — hops on

Set `hop.enable = true`. Every row records into
`docs/inflight-channel-hop.md`'s tables, which currently read `UNMEASURED`.

| # | Condition | Expect |
|---|---|---|
| 1 | Co-channel jam: `tools/bench/benchjam.sh --channel <op> --secs 70`, `DEVOURER_TX_SA` exported | `H order` within 450 ms of the first `V interfered`; `lead_confirm` within 200 ms more; `E hop_restore` to the pre-onset rung; `verify_pass`. `ausniff` clean during. |
| 2 | DJI O4 on the op channel | Same, with `raised` evidence (FA-driven) rather than `contended` |
| 3 | Fade — drone carried out of range | `V fade` lines, **zero** `H order`, ladder demotes as before |
| 4 | Jammer on a *candidate* channel only | **No hop.** The ranker excludes it — `hop.target` never equals it, its `D` scores rise |
| 5 | Drone RCF `rx_pps` with dwells on vs at rest | Unchanged (the non-TX card is the one scouting) |
| 6 | One-card GS — pin `[[radio.cards]]` to one entry, repeat row 1 | `H order`, `OneCardRetune` after 5 RCFs (~250 ms), `verify_pass` |

Row 6 matters more than its position suggests: see *One-card* below.

### 3. Observe-only flight, then enabled

Fly first with `hop.enable = false`. Afterwards read the HOP section of
`tools/flightreport.py`. With hops disabled the controller still runs and logs
every decision it *would* have made, so that section prints:

- a `SHADOW` hop table (`would_order` events, tagged `[SHADOW]`),
- the verdict histogram,
- **a per-bit evidence tally** (`impaired`, `weak`, `fading`, `contended`,
  `raised`) and per-card medians of `foreign`/`fa`/`rssi`/`snr` over
  non-healthy windows.

That evidence tally is the calibration instrument. The spec's own open item
says the thresholds are bench numbers and *the observe-only flights are the
calibration*. If the tally shows windows tripping `raised` that you know were
fades, or `unknown` windows dominating (impaired but no term explains why),
that is the signal to retune `[hop.verdict]` before flipping `enable`.

Only then fly with `hop.enable = true`.

## Timings to check against

Derived from config defaults and code paths before the bench. Measured
2026-09-15: two-card onset→order 150 ms, →video 268 ms, →`verify_pass`
1.27 s; one-card onset→order 153 ms, →retune 411 ms, →video 427 ms,
→`verify_pass` 1.16 s. The table stands as the derivation.

| Milestone | One card | Note |
|---|---|---|
| Detect (trigger latches) | ~300 ms | `persist` 2 × `window_ms` 150 |
| → `Order` | ~633 ms | +333 ms: the ranker needs 2 fresh visits, so a second burst |
| → physical retune | ~883 ms | +5 × `feedback_ms` **50** |
| → video confirmed | ~0.9–1.1 s | This is the spec's sub-second goal |
| → `verify_pass` | ~1.9–2.1 s | +`verify_ms` 1000. A different milestone — do not conflate |

Two cards are faster: the scout thread keeps the ranker warm, so the first
burst already finds ranked candidates and the ~333 ms second-burst wait
disappears. The final review's two-card budget, traced end to end, is
**~450-550 ms onset-to-video** — detection ~300 ms, freshness burst ~30-40 ms,
order onto the air within one `feedback_ms` plus slotter hold and RCF loss
(~50-150 ms), drone TX-gate + drain + retune ~20 ms, first AU ~17 ms. The rung
is already restored when video returns, because `restore_rung` fires
synchronously at the order.

`feedback_ms` is **50** in `gs/bundle/maburgs.default.toml`, overriding the
struct default of 100 in `gs/src/config.h`. Computing from the header alone
gives +500 ms for the one-card repeats and the wrong conclusion that the path
misses its target.

## One-card: the least-validated path

**Bench row 6 passed 2026-09-15** (`order` → `one_card_retune` +258 ms →
`lead_confirm` +274 ms → `verify_pass`), with the ladder cascade noted above
as the cost.

Two real bugs were found in it *after* the per-task reviews had passed, both
by reading source while writing documentation:

1. The order retuned the sole radio immediately, before it could have been
   transmitted — so every one-card hop would have failed, with the link dark
   for the window.
2. The pre-order survey was gated so it could never run, so the ranker never
   received a visit and a one-card GS could only ever hop home or hold.

Both are fixed and covered by a `gs_e2e` scenario. But they are the two that
got furthest, and a third gate — the survey's rate limiter — was added late to
stop a held station sweeping its only radio off-air continuously. **Run bench
row 6 before trusting one-card operation.**

## Watch for these specifically on the first runs

Each came out of the final whole-branch review. None blocks a flight; each has
a named symptom, so look for it rather than discovering it later.

- **False `verify_fail` on a channel that looks clean.** The `impaired` term is
  read from a **500 ms** sliding loss window while every other verdict term is a
  true 150 ms delta, and that window is not blanked at a hop. So the first two
  or so verify windows after landing still carry pre-hop, old-channel loss.
  `Interfered` also needs `contended ∨ raised`, which are genuinely fresh — but
  on a target where FA or foreign frames stay above threshold, a good-enough
  channel can still take a `verify_fail` and a 30 s backoff. **Symptom:**
  `H verify_fail` within ~2 windows of a `lead_confirm`, on a channel whose own
  `V` lines look clean.
- **`fading` measured against the old channel.** `HopVerdict::reset()` keeps the
  trailing RSSI history, which right after a hop holds only old-channel samples.
  For up to ~5 s the new channel's RSSI is compared against the old channel's
  median. Fail-safe (a spurious `fading` suppresses `Interfered`), but it can
  mask a genuinely bad target for those seconds.
- **Hold flap.** Hold events are now logged on edges, not per tick, which cut a
  ~100 Hz flood by 15×. But `trigger` is 2-of-3 over 150 ms windows, so an
  alternating impaired/clean pattern still toggles it about every 300 ms —
  roughly 6.7 `H` lines/s and `hop.holds` climbing ~3.3/s. Plausible under
  marginal interference. If you see it, a minimum hold dwell closes it.
- **One-card timing is thin by construction.** Five RCF repeats at
  `feedback_ms` 50 consume 250 ms of a 500 ms `confirm_ms`, leaving 250 ms for
  the sole radio to retune, the drone to have already retuned, and one video AU
  to arrive. If one-card hops withdraw more often than they confirm, that
  budget is the first thing to look at.

## What is verified by inspection rather than by a test

Three one-line call sites in `main.cpp` are correct by reading, not by
execution — the host has no boot scout and no radio, and `gs_e2e` evaluates its
own copy of the confirm gate rather than the shipped line:

- the hop-confirm read of a body's receive channel, and the stamp that sets it
  in `radio_frontend.cpp` (this is the one that matters — it is what stops the
  ground station following to a channel the drone was never told about);
- the boot-pick publication into the ranker;
- the store-blank call at the first impaired window.

All three fail safe if wrong, but bench row 1 and row 6 are the first real
exercise any of them get.

## Verify on hardware — things the host cannot prove

- **Epoch discipline.** The drone treats a hop order as idempotent on the
  `(epoch, channel)` pair; its only escapes are a new epoch or a
  `move_confirm_ms` timeout home. Confirm on the wire that the GS bumps the
  epoch on **every** order *and* **every** withdrawal. A reused epoch on a
  withdrawal is silently ignored and the two ends disagree about the channel.
- **Timeout ordering.** GS `confirm_ms` (500) must stay below the drone's
  `move_confirm_ms`. Both are config; nothing enforces the relationship across
  the two files.
- **Dwell cost on a marginal card.** Card 0 is historically the weaker one.
  The cost of a dwell on a card already near its margin is unmeasured.
- **`scout_when_disabled` defaults true**, so the spare card is dwelled every
  ~333 ms *even with hops disabled*. The first flight therefore has RF
  behaviour changes with the kill switch on. That is deliberate (it is how the
  observe-only flight collects ranking data), but it is not a no-op flight.

## Known limitations, accepted deliberately

- **Non-atomic dwell-busy publish.** The main loop's "is this card mid-dwell"
  check is not atomic with the scout thread's publish, so a reader can observe
  "free" a few instructions before the scout takes the card. Window is
  nanoseconds against a 333 ms period; the consequence is one mistimed dwell,
  corrected within a tick by the retune loop and the channel resync. Closing it
  means locking in the main loop's hot path at three sites — a real cost on the
  video path for an improbable, self-healing event.
- **Burst dwells are invisible to the per-card sideport counters.** The
  pre-order survey does not feed `dwell_recs`, so `cards[i].dwell.visits`
  **undercounts** real scouting activity. Read it as "periodic scout visits",
  not "all visits".
- **The 5 s `hop_restore` match window** in `flightreport.py` is an
  uncalibrated guess with no flight data behind it. It fails safe (an unmatched
  restore prints as not-found rather than mis-pairing), but the number should
  be revisited once real logs exist.
- **`verify_fail` retries print as separate rows**, not folded into one
  episode. Faithful to the state machine (each retry is a fresh order at a
  bumped epoch); slightly harder to read as a narrative.
- Minor and cosmetic: `RcAgent::channel()`'s doc comment does not list hop as a
  writer of `channel_`; `HopRankEntry::visits` reports the fresh count rather
  than the raw deque size; `ChannelPlan::hop_target()`/`hop_lead()` return
  stale values after a withdraw until the next order (every caller gates on
  `hopping()` first).

## Decisions made during implementation that a bench run could overturn

The spec went stale in about ten places; each divergence was decided
deliberately, and the commit messages on this branch carry the reasoning. The
ones a bench result could legitimately reverse:

- **Rate cap includes verify-fail retries** (spec §5 says "including retries").
  If the bench shows a link that needed a 5th retry inside a minute to recover,
  revisit.
- **`dwell_period_ms` reused as the survey rate limit.** Chosen so the survey
  inherits the scout's designed duty cycle (~30 ms in 333 ms). If the measured
  dwell cost is far from ~10 ms, this number changes meaning.
- **The one-card survey takes the only radio off-air for ~30 ms** per sweep
  (3 default candidates). Measure the actual video impact in row 6.
- **`blank_store` suspends only the rung store's EWMA writes**, not s3 demote
  decisions — the spec requires the ladder to run unfrozen during detection. If
  flights show the ladder thrashing during a hop, this is the knob.

## Repo hygiene, unrelated to this work

- A stray git worktree exists at `.claude/worktrees/ladder-controller`
  (detached at `d105fd1`), plus an untracked `.claude/worktrees/devourer`.
  `CLAUDE.md` forbids worktrees for mabur work. Left untouched — removing one
  is destructive and it is not this branch's.
- `tests/vectors/profile.json` diverges from its own generator
  (`tools/genvectors/gen_vectors.py`) on `master`, independent of this branch:
  re-running the documented regen produces a large unrelated diff.
