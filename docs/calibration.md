# TX-power wall calibration — `maburcal`

An operator ssh'd into the ground station gets their own vtx's per-rate
PA compression walls, measured, applied and verified, in about 75 seconds:

```
$ ssh root@10.18.0.1
# maburcal start
```

No toolchain, no repo checkout, no laptop-side step. This page is the
durable home for the kit — the design spec lives under
`docs/superpowers/specs/`, which is gitignored and exists on one machine
only, so anything that must survive belongs here instead.

`maburcal` supersedes `bench/txagcbench/`, deleted 2026-09-10. The
measurement *history* that tool produced — the wall table, the transfer
curve, the comb finding — is still current hardware fact and lives on in
`docs/txagc-calibration.md`; only the tooling for producing a *new* unit's
numbers moved.

## What a run does

`maburcal start` drives the whole thing from the GS: it sends one command
to `maburgs`' loopback-only `CalControl` listener (`127.0.0.1:8400`,
unreachable off-box), streams progress every 500 ms, and prints a final
table when the drone returns to normal video. Under the hood:

1. **Coarse sweep** (~36 s of sweep time): every 4th TXAGC index, 0..124,
   across all 8 MCS rows, 20 frames per cell.
2. **Fine sweep** (~0-41 s of sweep time, skipped for rows with no dip):
   ±8 indices around each row's coarse dip, at full resolution, 100
   frames per cell. This also re-measures the exact cell the coarse pass
   flagged, about a minute later — two independent readings of the same
   operating point, which is the run's built-in thermal-drift check
   (the `drift` flag).
3. **Apply**: the GS computes final walls and sends them to the drone,
   which validates them, backs up and patches `/etc/mabur.toml`,
   reprograms the per-rate diffs live, and flips `power_mode` to
   `"offset"` — no restart.
4. **Verify** (~2 s of sweep time): the drone immediately sweeps its own
   eight newly parked indices; the GS tallies delivery at each and
   reports it.

Each phase also carries a further ~4 s tail after its last frame
(`phase_slack_ms` in `cal_session.h`) — a listen window the GS waits out
before declaring the phase over and issuing the next command. The
figures above are pure sweep time (cells × (settle + frames × gap), from
`gs/src/cal_plan.h`); the totals below fold in three of these ~4 s tails
(one per phase) on top of that sweep time, plus a small apply/report
overhead — they are not a straight sum of the three headline numbers
above.

On the reference unit a full run is **~72 s** (three rows — MCS 0-2 — never
dip, so they skip the fine phase). A unit whose PA walls every rate runs
closer to 87 s. The entire run is **radio-silent from the GS**: no RCF,
no keepalive DISC, nothing but the sweep frames themselves and the two
`T_CAL_CMD`/`T_CAL_RESULT` control frames — video and telemetry both
pause and resume with the session.

`maburcal status` polls a running session; `maburcal abort` cancels one.
None of the three take arguments.

## Prerequisite: the pair must already be linked and flying video

There is no other way to reach the drone. `T_CAL_CMD` rides the same
uplink as everything else, and the drone refuses to enter calibration
without `CAP_CALIBRATE` in the DISC handshake and a session already
`LINKED`. If the link is down, fix that first — calibration cannot be
used to bring it up.

Corollary: losing the link *during* a run is expected, not an error. The
drone's sweep is open-loop and wall-clock-bounded; it finishes or times
out and restores itself whether or not the GS is still talking. Killing
`maburgs` mid-sweep leaves the drone to time out on its own and resume
flying video with an **unchanged** config — nothing was written, because
nothing reached the apply step.

## Geometry

Put the drone and GS at a normal bench distance — close enough that the
low end of the sweep (weak, low-index transmissions on the fastest MCS
rows) still has a chance to be heard, far enough that the high end
doesn't pin the GS's RX front end into its own compression. The
`saturated` flag below is exactly this second failure watched for you;
if it fires, back off and rerun rather than trusting the table.

## Reading the result

The final table has one row per MCS (0-7), each with a wall, a park
index, verify-pass delivery, and health flags:

```
run nonce=... base_ref=53 margin=1.00dB
rate    wall  park verify   flags
mcs0      91    87    99%   no_dip
mcs1      91    87   100%   no_dip
mcs2      91    87    99%   no_dip
mcs3      95    91    98%
mcs4      73    69    97%
mcs5      54    50    96%   drift
mcs6      51    47    99%
mcs7      49    45    98%
legacy  91 (derived from mcs0)
base_ref_idx 53
written: /etc/mabur.toml (backup /etc/mabur.toml.pre-cal)
```

**The wall is the end of the first contiguous ≥90% delivery run**, scanning
upward from the sensitivity floor. Past that first dip, delivery is a
reproducible *comb*, not a cliff — islands of 90%+ delivery reappear at
higher indices (mcs7 on the reference unit reads 4% at idx 56 but 88% at
57). Those islands are **not usable headroom**. This is why the kit walks
the whole range rather than bisecting: a search that stops at the first
"good" reading past a bad one would land inside the comb and overdrive
the PA.

### Health flags

| Flag | Meaning | What to do |
|---|---|---|
| `saturated` | Peak median RSSI crossed the saturation threshold — the GS's own RX front end is compressing, not (only) the drone's PA. | Walls read **low** under this flag, which is the *conservative* direction: it costs some power headroom, never overdrives anything. Safe to ship, but move the drone farther out and rerun if you want a tighter number. |
| `no_dip` | The row never dropped below 90% delivery anywhere in the sweep — there is no compression wall to find. The reported number is the *RSSI saturation knee* instead (where the per-cell median RSSI curve stops rising), not a delivery-derived wall. | Normal for MCS 0-2 on healthy hardware (BPSK/QPSK never compresses within the sweep's range). If it fires on a higher MCS, that rate is unusually clean — nothing to fix. |
| `undetermined` | No cell in the row ever reached 90% delivery at any index — no first-dip exists to find. | **That rate's config entry is left untouched** — the kit never invents a wall from data that can't support one. Check geometry (likely too far for that rate) and rerun if you need a real number. |
| `narrow` | The floor edge (where delivery first reaches 90%, ascending) sits within ~4 indices of the wall. The usable window between "too weak to hear" and "compressing" is too thin to trust. | Move closer and rerun; a wall this close to its own floor is not a reliable measurement. |
| `card_disagree` | The two GS RX cards' independently-computed walls differ by more than a couple of indices. | Points at an antenna or card problem, not a PA — check cabling/orientation on the disagreeing card before trusting either number. |
| `drift` | The coarse and fine phases measured the same cell differently. | Informational; large drift suggests thermal movement in the PA during the run — a rerun after the hardware has settled is reasonable if it's large. |

### `legacy_wall_idx` is derived, not swept

There is no legacy OFDM mode in mabur's wire encoding
(`common/include/mabur/profile.h`'s `PhyMode` is `{HT, VHT}` only), so the
kit does not sweep a ninth row for it. Instead `legacy_wall_idx` is set to
whatever the MCS0 result comes out to. **This is a stated physical
assumption, not a measurement**: legacy OFDM 6 Mb/s and HT MCS0 are both
BPSK 1/2 over the same OFDM waveform, so their peak-to-average ratio — and
therefore their PA compression wall — should track each other. The
reference config's own numbers corroborate it (both read 91), and the
hardware acceptance checklist below re-checks it on every unit calibrated:
if `legacy_wall_idx` ever diverges meaningfully from the MCS0 result on a
real run, the assumption needs revisiting, not the code.

## When it doesn't work

Three failure shapes fall outside every health flag above, because a flag
is only computed from the sweep phases' own delivery data.

**`maburcal start` refuses immediately, with a one-line reason.** Before
any sweep frame goes out, the GS checks preconditions and returns one of:
`err a calibration session is already running`, `err refused: link is
down`, or `err refused: peer does not advertise CAP_CALIBRATE`. The first
two are exactly what they say — wait for the running session to finish
(or `maburcal abort` it), or get the link back to `LINKED` first. A
half-deployed `RC_VERSION` 6-vs-7 pair (see the flag-day note below)
never completes a `SESSION` handshake at the wire level at all, so it
surfaces here as **"link is down"**, not as the capability error — the
capability check is only reachable once `LINKED` is already true, which a
version-mismatched pair never reaches. If `CAP_CALIBRATE` itself is ever
the refusal on a pair that otherwise links fine, that means one side
predates this kit; rebuild and redeploy both binaries from the same
commit.

**The session starts, then ends in `state=failed` with no video loss and
no config change.** `maburcal start`'s streamed progress lines will show
`state=await_ack` repeating, then `state=failed`. This is the drone never
acknowledging the phase command within `ack_timeout_ms` (3 s, repeated
every 200 ms until then) — the *only* path to `Failed` in `CalSession`,
and it can only happen after `start()` already passed the link/capability
checks above. The reason string it records internally
(`"calibration ack timeout"`) is not currently surfaced through
`status`/`start`'s output, so `state=failed` with no other detail is all
you get. The most likely real cause is RF, not configuration: the uplink
is already lossy by design (30-50% per frame, `rcf-uplink-loss`), so a
genuinely poor link at that moment can lose all of the ~15 repeats inside
the 3 s window even though it looked `LINKED` a second earlier. Improve
geometry/orientation and retry before suspecting anything else. Nothing
was written in this case — the drone only writes config after reaching
`Result`, several states past `AwaitAck`.

**The `verify` column is blank on every rate.** Read this first, before
the low-delivery case below: a dash in `verify` for *all eight* rates is
not a delivery problem, it means the drone never ran its verify sweep at
all. The report says so explicitly — `not written: walls were measured but
the drone never ran its verify sweep`. Two things produce it:

- The `T_CAL_RESULT` frame never arrived. It rides the same 30-50%-lossy
  uplink as everything else, so the GS repeats it every 200 ms (bounded,
  ~15 tries) until the drone's first verify frame acks it — the drone
  sweeps verify only after a successful apply, so that frame *is* the ack.
  Fifteen consecutive losses is unlikely but possible on a bad link.
- The drone refused the apply — an out-of-range table, a backup or write
  failure, or a candidate config that would not reload. All of these
  return before verify is armed, and all of them leave `/etc/mabur.toml`
  exactly as it was. `/tmp/maburd.log` on the drone names which.

Either way **nothing was written**. Re-run; if it repeats, read the drone
log before touching geometry.

**Verify delivery reads low on a rate that has a number there and no flag
at all.** Flags are computed from the coarse/fine sweep data; the verify
pass has none of its own; a rate can measure a clean wall and still show
poor delivery when the drone parks there a minute or two later. Likely
causes are geometry having moved between the sweep and the verify pass,
or a wall estimate that a flag should have caught but the sweep data
didn't quite cross the threshold for. There is no automatic signal for
this beyond reading the `verify` column yourself — if a rate reads low
there, treat that number over the flag: rerun (a `drift` flag on the same
rate in the rerun would corroborate it), or back the parked power off by
widening the margin — edit `radio.wall_margin_db` in `/etc/mabur.toml` on
the drone and re-run — and check whether verify delivery recovers.

`maburcal start` takes no arguments, and in particular there is no
`--margin`. Calibrating `wall_margin_db` is an explicit non-goal: it is
the operator's safety choice, hand-set on the drone, and it is applied
exactly once, there. The flag that used to exist moved only the GS's own
park bookkeeping (this report's `park` column and the indices the verify
plan expected frames at) — it never reached the hardware, because
`maburcal` patches `rate_walls_idx`, `legacy_wall_idx`, `base_ref_idx` and
`power_mode`, and `wall_margin_db` is not one of them. All it could
achieve was making the `park` column disagree with what the drone flew.

## What gets written, and how to roll back

`maburcal` patches exactly four keys in `/etc/mabur.toml`:
`radio.rate_walls_idx`, `radio.legacy_wall_idx`, `radio.base_ref_idx`, and
`radio.power_mode` (set to `"offset"`). The patch is line-surgical — every
comment and every other key survives untouched, because this file is also
`bundle/mabur.default.toml` verbatim (see below).

Before writing, the candidate values are validated with the exact same
derivation and range check `maburd` uses at boot
(`drone/src/config.cpp`) — a fresh `mabur::load_config()` call against the
*candidate* file, not just a TOML-syntax check, since syntax passing is not
the same as boot succeeding. If validation fails, or the rewritten file
somehow fails to reload, nothing is touched: the original config is copied
to `/etc/mabur.toml.pre-cal` only once the new file has already proven
loadable, and the rename that publishes it is atomic. This is deliberately
the same shape as every other config-load safety net in this codebase
(`docs/deploy.md`): a config `maburd` cannot load makes its wrapper
respawn it forever at 2 s, which turns a calibration run into a trip for a
laptop and a serial cable.

**To roll back a calibration**, restore the pre-run file:

```sh
cp /etc/mabur.toml.pre-cal /etc/mabur.toml
/etc/init.d/S00mabur restart
```

`.pre-cal` is overwritten on every successful run, so it always holds the
config from immediately before the *most recent* calibration — not
necessarily the factory-default one.

## Walls are re-derived after a wipe, never restored from the bundle

Since PR #50 the three shipped bundle files
(`bundle/mabur.default.toml` included) are the live flight configs off
the drone and GS, verbatim, down to every knob — the standing rule is
"retune in the repo or it's lost at the next wipe." **That rule does not
extend to the walls section.** `rate_walls_idx`, `legacy_wall_idx` and
`base_ref_idx` are per-unit PA and efuse measurements; the numbers in
`bundle/mabur.default.toml` are one specific board's, kept there only as
a documented reference (and inert unless `power_mode` happens to read
`"offset"` on a board that never ran its own calibration). Restoring a
wiped drone from the bundle default and calling it done would silently
fly someone else's PA compression points.

After any drone wipe or replacement, the walls section is the one part of
the config that must be **re-measured with `maburcal start`**, not copied
from git. Everything else in the bundle is fair game to restore as-is.

## Offline analysis: `maburcal report`

```sh
maburcal report /media/dvr/log/0042/cal.log
```

re-renders a saved run with no daemon involved — the same table `start`
prints live, computed straight from the log file. This is the entire
replacement for `bench/txagcbench`'s deleted Python analyzer: raw per-cell
tallies, final walls, and verify results all survive in `cal.log`, so
deleting the old tool cost the tool, not the ability to examine a run
after the fact.

**One `cal.log` can hold several runs.** The normal retry path is: run,
see a `narrow` or `saturated` flag, reposition the drone, run again — and
nothing about a retry rotates the session directory, so the second run's
records land in the same file as the first's. Each run is delimited by
its own `R <nonce> <base_ref> <margin_db>` line; `maburcal report` renders
every run the file holds, in order, and each run's `C`/`W`/`V` rows are
scoped to the `R` line that started it (a rate's park index always uses
*that run's own* margin, never a different run's). `cal.log` is written
regardless of `debug_log.enable` — a calibration run is a bounded, rare,
deliberately-triggered trace, not the continuous per-second logging that
knob exists to gate — so a run's data is never silently lost to a debug
logging default. See `docs/observability.md` for the file's exact format
and where it lives when debug logging is off.

## Hardware acceptance checklist

This is written for a real bench session — work through it on deployed
hardware; nothing here was run as part of writing this page.

### 1. Two-device flag-day deploy

`RC_VERSION` bumped 6 → 7 for the two new frame types (`T_CAL_CMD`,
`T_CAL_RESULT`). Deploy config-before-binary on both devices as usual
(`docs/deploy.md`); no config keys move for this bump, so the flag day is
binary-only. **Between swapping `maburd` and swapping `maburgs` the pair
has no control link and no video** — indistinguishable from the
stale-caps restart deadlock. Do not restart either daemon trying to fix
it; finish the deploy. Confirm video resumes once both binaries match.

### 2. `ausniff`

```sh
python3 tools/bench/ausniff.py    # expect ~59.8 fps, 0 gaps
```

This is the standing regression gate for any `maburgs` change — it reads
the AU ring from outside the daemon, so it is not circular the way
gating on the sideport would be. Take a second pass if the first reports
a `frame_id_gap`; the first pass after a restart can show a phantom one.

### 3. `maburcal start` and the acceptance table

```sh
ssh root@10.18.0.1 maburcal start
```

> **The kit reports the *measured wall*, which is not always the same
> number as `bundle/mabur.default.toml`'s `rate_walls_idx`.** That
> array's mcs3 entry reads 91, while the measured mcs3 wall documented in
> `docs/txagc-calibration.md` is ~95. That same page's "suggested clamp
> values" line lists `{3: 91}` as *wall − ~1 dB margin* — a park index,
> not a wall — so the shipped array appears to carry a park value in the
> mcs3 slot that `power_plan.h` treats as a raw wall, subtracting the
> margin a second time and parking mcs3 about 1 dB lower than intended.
> That is a hypothesis about the existing shipped config, not a
> confirmed finding — check it at the bench rather than editing the
> bundle on the strength of this paragraph. **A `maburcal` run reporting
> 95 for mcs3 on this unit is behaving correctly**; a mismatch against
> the *bundle's* 91 is the thing this paragraph explains, not a bug in
> the kit.

Check every one of these against the run:

| Check | Expected |
|---|---|
| Wall table | `[91,91,91,95,73,56,51,49]` (measured walls — see note above), mcs5 may read 54 |
| MCS 0-2 | flagged `no_dip`, wall ~91 from the knee — **not 127** |
| `base_ref_idx` | 53 on this unit |
| `legacy_wall_idx` | equals the MCS0 result (91) |
| Run duration | ~72 s |
| Radio silence | no GS PPDUs on air during a phase (capture to confirm) |
| TX power restored after a session | after BOTH a successful and an aborted calibration, confirm the index override is cleared (`-1`) and the rate-diff table is live — not a flat index masking it |
| Ladder restored after a session | video resumes on the operating ladder, not the last swept cell's |
| venc ring over a full session | ~180 s with nobody draining it: confirm no fault, and that the resume burst of ring-resident stale frames does not wedge the pipeline |
| devourer TX-setter thread safety | the TX writer thread now calls power setters while the agent thread calls GetThermalStatus/GetTxStats — undocumented in devourer; watch for corruption or hangs |
| Video-silence fallback | a full coarse+fine+verify run sends zero video for ~72 s; confirm it does not trip the rendezvous/continuity fallback into a confusing intermediate state, and that video resumes cleanly |
| cal.log written | present at the path the GS logged at calibration start, even though `debug_log.enable = false` in the shipped config |
| Verify pass | high delivery at every parked index |
| Config | `/etc/mabur.toml` patched, comments intact, `.pre-cal` is the byte-identical original |
| Video | resumes with no restart |

### 4. Interruption test

Kill `maburgs` mid-sweep. Confirm the drone returns to flying video on
its own (open-loop timeout) with an **unchanged** `/etc/mabur.toml` — no
partial write, no `.pre-cal` created, no lingering `"offset"` mode from a
run that never reached apply.

### 5. Two open measurements

These are tuning knobs shipped with placeholder values, not defects —
both are safe to leave as shipped, but cheap to improve on real hardware:

- **`settle_ms`** (`gs/src/cal_plan.h`, currently 100 ms): inherited from
  `txagcbench`'s default and never actually measured against this
  hardware's real TXAGC settle time. Coarse alone spends ~28.8 s settling
  and only ~11.5 s sending at 100 ms. Sweep 20 / 50 / 100 ms and confirm
  the wall table is unchanged at each; if 20 ms reproduces it, set
  `kSettleMs = 20` and a run drops to roughly 50 s.
- **`sat_rssi_dbm`** (`gs/src/cal_analysis.h`, currently -45.0): inherited
  from `txagcbench`'s bench precondition ("attenuate until max RSSI
  ≤ -45 dBm"). The 2026-07-29 clean reference run peaked at -67 dBm, well
  clear of it, so confirm the flag does not fire on a normal-geometry run,
  then deliberately provoke it by moving the drone within a metre of the
  GS and confirm it does. If the threshold never fires at any workable
  bench geometry it is discriminating against nothing and should be
  raised until it actually separates a good run from a saturated one.
