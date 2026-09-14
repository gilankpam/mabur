# Handover — in-flight channel hop: bench, flight and validation (2026-09-15)

Everything in `docs/superpowers/plans/2026-09-14-inflight-channel-hop.md` that
can be done on a host is done. Nothing has touched hardware. This page is what
a fresh session (or a human at the bench) needs to finish the job.

Read `docs/inflight-channel-hop.md` first — it describes what was built. This
page only covers **what is left**, and the things a bench run could invalidate.

## State

| | |
|---|---|
| Branch | `inflight-hop`, 36 commits off `master` at `f51f8e0`, head `8076eff` |
| Host suite | 143/143 (`ctest -R 'test_\|host_e2e\|gs_e2e\|gs_au_e2e\|player_e2e'`) |
| Cross-builds | `tools/build-arm64.sh` and `tools/build-arm.sh` both clean |
| Flown | **No.** Never deployed, never benched, never flown. |
| PR | Not opened — the owner's call. |
| `hop.enable` | **`false`** in the shipped bundle. The feature observes and logs; it never acts until flipped. |

The devourer half (`GetRxEnergyScout`, batched control transfers) is on
`../devourer` branch **`feat/scout-energy-ctrl-batch` @ `b4c6e5a`**, built by
another agent in parallel. It is **not on devourer `master`**. The GS
cross-build consumes whatever `../devourer` has checked out, so that branch
must stay checked out — or be merged to `master` — before building a GS binary
to deploy.

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

Plan Task 11 step 6, and Task 15 steps 4–6.

### 1. Baseline — dwells on, hops off

Deploy per above with `hop.enable = false` (the shipped default), then:

```
tools/bench/ausniff.py --seconds 60
```

Expect: 59.7 fps class, 0 gaps. Then check `scan.log` in the session
directory for `V` lines (verdict windows, present in every link state) and `D`
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

All derived from config defaults and code paths. **None measured.**

| Milestone | One card | Note |
|---|---|---|
| Detect (trigger latches) | ~300 ms | `persist` 2 × `window_ms` 150 |
| → `Order` | ~633 ms | +333 ms: the ranker needs 2 fresh visits, so a second burst |
| → physical retune | ~883 ms | +5 × `feedback_ms` **50** |
| → video confirmed | ~0.9–1.1 s | This is the spec's sub-second goal |
| → `verify_pass` | ~1.9–2.1 s | +`verify_ms` 1000. A different milestone — do not conflate |

Two cards are faster: the scout thread keeps the ranker warm, so the first
burst already finds ranked candidates and the ~333 ms second-burst wait
disappears.

`feedback_ms` is **50** in `gs/bundle/maburgs.default.toml`, overriding the
struct default of 100 in `gs/src/config.h`. Computing from the header alone
gives +500 ms for the one-card repeats and the wrong conclusion that the path
misses its target.

## One-card: the least-validated path

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
deliberately and is recorded in the execution ledger (gitignored, this machine
only). The ones a bench result could legitimately reverse:

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
