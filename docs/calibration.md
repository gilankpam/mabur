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

1. **Coarse sweep** (~29 s of sweep time): every 4th relative TXAGC
   index, rel −40..63 (26 cells per row, 208 total), across all 8 MCS
   rows, 20 frames per cell.
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

### The drone will be in `RENDEZVOUS` by the end of every run

The GS is radio-silent for the whole session, so the drone's `RcAgent`
sees no RCF and no DISC and ages out of `LINKED` on its own schedule:
`LINKED` → `FAILSAFE` at `link.failsafe_ms` (3 s), `FAILSAFE` →
`RENDEZVOUS` at a further `link.rendezvous_ms` (30 s) — about 33 s into
the coarse sweep, on every run. This is expected and benign, not a
symptom: `RENDEZVOUS` is a passive waiting state, and `set_ladder` is
gated on `cal_active` while a session runs (`drone/src/main.cpp`), so the
agent cannot fight the sweep for the radio. The falling edge of
`cal_active` re-applies the operating ladder and TX power together.

The operator's job is only to **confirm the pair re-links after each
session**: video should resume within a couple of DISC beacons, with an
IDR at the join. If it doesn't, that is the ordinary stale-caps
restart-deadlock shape and not a calibration bug — see
`docs/deploy.md`.

Corollary: losing the link *during* a run is expected, not an error. The
drone's sweep is open-loop and wall-clock-bounded; it finishes or times
out and restores itself whether or not the GS is still talking. Killing
`maburgs` mid-sweep leaves the drone to time out on its own and resume
flying video with an **unchanged** config — nothing was written, because
nothing reached the apply step.

## One calibration covers every channel

Walls are stored as signed indices **relative to the chip's own TXAGC
anchor** (`radio.rate_walls_rel`, `radio.legacy_wall_rel`). The anchor is
the per-channel-group reference devourer programs from the module's
efuse on every channel set (39 / 53 / 57 on this unit for ch136 / 149 /
165), and the chip adds it itself, so `maburcal` on any channel produces
a table that is valid on home and on every auto-select candidate
(`docs/channel-select.md`). The anchor never leaves the drone: it is not
in config, not on the wire, not in `cal.log`.

Measured 2026-09-13 (six runs, two interleaved passes over ch136 / 149 /
165 at one geometry, GS `/media/dvr/log/0074/cal.log`):

| rate | abs 136 (anchor 39) | abs 149 (53) | abs 165 (57) | abs spread | rel 136 | rel 149 | rel 165 | rel spread |
|---|---|---|---|---|---|---|---|---|
| mcs3 | 84 | 95 | 92.5 | 11 | 45 | 42 | 35.5 | 9.5 |
| mcs4 | 61 | 73 | 73 | 12 | 22 | 20 | 16 | 6 |
| mcs5 | 45 | 61 | 62 | 17 | 6 | 8 | 5 | 3 |
| mcs6 | 48 | 57 | 61 | 13 | 9 | 4 | 4 | 5 |
| mcs7 | 44 | 57 | 57 | 13 | 5 | 4 | 0 | 5 |

Same-channel repeatability is 0-9 indices (mcs3/4 are the noisy rows).
On the rows with a real PA wall the relative wall is flat to 3-5 indices
while the absolute one moves 13-17 with the anchor. The residual is a
~1 dB slope toward ch165 on mcs6/7, inside the default 1 dB
`wall_margin_db`; calibrate on home and fly the candidates.

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
run nonce=... margin=1.00dB
rate   wall(rel)  park verify   flags
mcs0          63    59    99%   no_dip
mcs1          63    59   100%   no_dip
mcs2          62    58    99%   no_dip
mcs3          45    41    98%
mcs4          22    18    97%
mcs5           6     2    96%   drift
mcs6           9     5    99%
mcs7           5     1    98%
legacy  63 (derived from mcs0)
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
| `no_dip` | The row never dropped below 90% delivery anywhere in the sweep — there is no compression wall to find, so the rate is parked at the **rail**, `+63`, the top of the chip's 7-bit per-rate diff field. The number is not a measurement; the flag is what says so. | Normal for MCS 0-2 on healthy hardware (BPSK/QPSK never compresses within the sweep's range). If it fires on a higher MCS, that rate is unusually clean — nothing to fix. |
| `undetermined` | No cell in the row ever reached 90% delivery at any index — no first-dip exists to find. Reported as `-128` in `cal.log`. | **The config line is left untouched** — the kit never invents a wall from data that can't support one. Check geometry (likely too far for that rate) and rerun if you need a real number. |
| `narrow` | The floor edge (where delivery first reaches 90%, ascending) sits within ~4 indices of the wall. The usable window between "too weak to hear" and "compressing" is too thin to trust. | Move closer and rerun; a wall this close to its own floor is not a reliable measurement. |
| `card_disagree` | The two GS RX cards' independently-computed walls differ by more than a couple of indices. | Points at an antenna or card problem, not a PA — check cabling/orientation on the disagreeing card before trusting either number. |
| `drift` | The coarse and fine phases measured the same cell differently. | Informational; large drift suggests thermal movement in the PA during the run — a rerun after the hardware has settled is reasonable if it's large. |

### `legacy_wall_rel` is derived, not swept

There is no legacy OFDM mode in mabur's wire encoding
(`common/include/mabur/profile.h`'s `PhyMode` is `{HT, VHT}` only), so the
kit does not sweep a ninth row for it. Instead `legacy_wall_rel` is set to
whatever the MCS0 result comes out to. **This is a stated physical
assumption, not a measurement**: legacy OFDM 6 Mb/s and HT MCS0 are both
BPSK 1/2 over the same OFDM waveform, so their peak-to-average ratio — and
therefore their PA compression wall — should track each other. The
reference config's own numbers corroborate it (both read 63), and the
hardware acceptance checklist below re-checks it on every unit calibrated:
if `legacy_wall_rel` ever diverges meaningfully from the MCS0 result on a
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

**One rate reads `0%` in `verify` while its neighbors show real
percentages.** This looks similar to the two shapes above but means
something different from both, and the report is deliberately built to
tell them apart:

- A `-` in `verify` (wall column also shows a number less than 0, and
  `undetermined` in flags) means this rate's wall was never determined at
  all — nothing was ever parked for it, so there was nothing to verify.
  Benign, and unrelated to the drone's radio.
- `-` in `verify` on *every* rate, alongside `not written`, means the
  drone's verify sweep never ran at all (the previous case above) — no
  rate was confirmed, full stop.
- `0%` on one rate, with `written: /etc/mabur.toml` still printed and
  other rates showing real delivery, means this rate genuinely *was*
  parked, the drone genuinely *did* sweep verify (proven by every other
  rate's nonzero reading), and this one rate's parked power is dead —
  the GS heard nothing there at all. This is the single most important
  reading the verify pass exists to produce, and it must not be confused
  with either "-" case above: unlike them, it says the config on the
  drone right now is untransmittable at this MCS.

  The most likely causes are specific to that one rate: a wall measured
  too high for it (the coarse/fine sweep's own dip landed a bit
  optimistic, without quite tripping a flag), or an antenna/geometry
  problem that only affects that rate's bandwidth or the RX card that
  happens to win verify's single-card best-of for it. Re-run first — a
  repeat pins it as real rather than a one-off miss on the verify pass
  itself. If it repeats, treat that MCS row as unreliable: widen
  `radio.wall_margin_db` on the drone if several rates show the same
  shape, or avoid that rate in the ladder (`link.max_mcs`) until a
  rerun at different bench geometry gives it a real number.

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
`maburcal` patches `rate_walls_rel`, `legacy_wall_rel` and `power_mode`,
and `wall_margin_db` is not one of them. All it could achieve was making
the `park` column disagree with what the drone flew.

## What gets written, and how to roll back

`maburcal` patches exactly three keys in `/etc/mabur.toml`:
`radio.rate_walls_rel`, `radio.legacy_wall_rel`, and `radio.power_mode`
(set to `"offset"`). The patch is line-surgical — every comment and every
other key survives untouched, because this file is also
`bundle/mabur.default.toml` verbatim (see below).

Before writing, the candidate values are validated with the exact same
derivation and range check `maburd` uses at boot (`drone/src/config.cpp`)
— every relative wall in `[-64, 63]` and `rel − round(wall_margin_db · 4)
≥ −64` under `power_mode = "offset"` — a fresh `mabur::load_config()` call
against the *candidate* file, not just a TOML-syntax check, since syntax
passing is not the same as boot succeeding. If validation fails, or the
rewritten file somehow fails to reload, nothing is touched: the original config is copied
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
extend to the walls section.** `rate_walls_rel` and `legacy_wall_rel` are
per-unit PA measurements, relative to the chip's own efuse anchor; the
numbers in `bundle/mabur.default.toml` are one specific board's, kept there only as
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
its own `R <nonce> <margin_db>` line (`callog 3`; older logs carried
`base_ref` there too); `maburcal report` renders
every run the file holds, in order, and each run's `C`/`W`/`V` rows are
scoped to the `R` line that started it (a rate's park index always uses
*that run's own* margin, never a different run's). `cal.log` is written
regardless of `debug_log.enable` — a calibration run is a bounded, rare,
deliberately-triggered trace, not the continuous per-second logging that
knob exists to gate — so a run's data is never silently lost to a debug
logging default. See `docs/observability.md` for the file's exact format
and where it lives when debug logging is off.

## Bench validation, 2026-09-11 — what four real runs showed

The kit was deployed to the bench pair (`RC_VERSION` 7 both ends) and run
four times: three on channel 136, once on channel 149. Raw data is on the
GS at `/media/dvr/log/0057/cal.log` (+ `cal-run1-headerless.log`) and
`/media/dvr/log/0058/cal.log` (+ `cal.log.callog1` for the ch149 run).

**The mechanism works.** Every structural check passed: the run drives
itself end to end, `/etc/mabur.toml` is patched surgically (three lines,
every comment and unrelated key intact) with a byte-identical `.pre-cal`
backup, the verify pass reads 96-100% at every parked index, video resumes
with no restart (`ausniff` 60.3 fps / 0 gaps after every session), and
killing `maburgs` mid-sweep leaves the drone flying with a **bit-identical**
config — nothing written. Radio silence is corroborated statistically
rather than by capture: MCS 0-2 delivered 3840/3840 coarse frames with zero
loss across the first three runs, and GS uplink self-blanking would have
cost ~0.35% of them.

**Two measurement results are NOT yet trustworthy. Read the numbers for
MCS 0-2 as advisory, whatever flags they carry.**

### 2026-09-11: walls looked per-channel — resolved 2026-09-13

The 2026-09-11 runs found the anchor and the absolute walls moving with
the channel, and this page told you to calibrate on the channel you fly.
The controlled 2026-09-13 sweep (table at the top of this page) showed
the walls move *with the anchor*: relative to it they are flat within run
noise. Walls are now stored relative, which retires the rule. Note the
earlier claim that "a ch149 table flown on ch136 parks every rate ~1.5 dB
high" had the sign wrong — under the diff formula the chip adds the
ch136 anchor, so the old absolute table parked *low* there.

### Defect: one noisy coarse cell reroutes a no-dip row

MCS 0-2 never compress, so they are supposed to take the no-dip path.
In two runs of four, a *single* coarse cell in the mcs2 row read below
90% — 4 frames lost out of 20 — and that one cell ended the "first
contiguous ≥90% run", putting the row on the delivery path instead. It
reported 101 (ch136) and 111 (ch149) against a no-dip rail of 102, and
**both were written to the flight config**: mcs2 parked at 97 is roughly
6 dB above where run 1 put it, in the overdriving direction.

With 20 frames per coarse cell a 90% threshold has no noise margin, and a
run has 256 cells, so an outlier is likely *every* run. The `narrow` flag
fires on the resulting row and is reported — but flags never block, so the
number is applied anyway.

The rail change above shrank the damage **in the observed cases only**.
Both outliers happened to sit near the top of the sweep (idx 112), so the
rerouted row reported 101 against a rail of 102 — near-harmless. That is
luck, not a fix: an outlier at idx 40 would report a wall of 36 and park
that rate about 16 dB low. The failure mode is still live; it is the
`no_dip` flag going missing that tells you it happened.

### EVM was tried as a second instrument and the sweep frames carry none

EVM is the direct observable of PA compression — it degrades under drive
whether or not the frame still decodes, which is exactly what delivery
cannot see on the rows above. It arrives on the same `RxBody` as RSSI, so
it was recorded per cell (`callog 2`) and the bench re-run on
2026-09-11.

**It came back empty.** Every sweep frame, every rate, every index
reported rxevm `0x80` on both streams — the Jaguar3 type1 phy-status
page's "this stream was not measured" (`FrameParserJaguar3.h`), which
reaches `RxBody` as raw −128 and reads as an impossible −64 dB if taken
at face value. Ordinary video on the same link at the same moment
reported −14.5 dB, so the chip measures EVM fine. RSSI on those same
sweep frames is valid and tracks the index ramp cleanly, so the
phy-status page is present and parsed — it is the per-stream EVM field
specifically that is blank.

The code was **reverted**; `cal.log` is `callog 1` again with no EVM
columns. `maburcal` still accepts a `callog 2` file because bench runs in
that shape are on the DVR.

The plausible difference is the frames themselves: 64-byte single
(non-aggregated) probe-request frames, versus the large A-MPDU-aggregated
data frames video sends. **Untested.** If EVM is worth another attempt,
that is the experiment — a longer sweep payload, or sweep frames sent as
aggregated data frames — and it changes `cal_wire.h`, so it needs both
binaries redeployed together. Do not re-add the recording without
changing the frames first; it produced nothing but `-999` columns.

### Fixed: the RSSI knee is gone, no-dip rows park at the rail

The original rule for a row with no compression wall was the *RSSI
saturation knee* — the lowest index whose median RSSI was within 1 dB of
the row's peak. It was not reproducible: mcs0 read **72, 56, 84 and 56**
across four runs of one unit, and run 4 put mcs0 at 56 and mcs1 at 68 —
same PA, same modulation class, same run, 3 dB apart. The rule cannot do
better. The transfer curve creeps at ~0.2 dB/idx, so a 1 dB tolerance
band already spans ~5 indices before 1 dB of RSSI quantization moves it
further, and `peak` is a maximum over 32 quantized samples, which is both
upward-biased and jumpy. The design's claim that coarse resolution puts
the knee within ±2 indices "because the curve is flat there" does not
hold — near the tolerance boundary the curve is still climbing.

A no-dip row parks at the rail: `+63`, the top of the diff field.

Three things make this better than the knee, not merely more stable:

- **It is exact and identical every run.** No scatter to reason about.
- **It is provably inside measured-good territory.** The rail is below
  the top of the sweep, and a no-dip row just delivered ≥90% at every
  index through 124. The knee was never validated by delivery anywhere.
- **It does not cost range.** On the measured curve the last real gain
  lands by idx ~80; everything above is flat to within quantization. The
  knee, firing early by construction, was giving up ~1-1.5 dB on
  precisely the rates the link falls back to when it is struggling.

One defect remains -- the noisy-cell reroute above. MCS 3-7 are the
measured product; a no-dip MCS 0-2 row is now a deterministic constant.

## Hardware acceptance checklist

Work through this on deployed hardware. Most of it was exercised on
2026-09-11 (above); the rows that still say "capture to confirm" were not.

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

Check every one of these against the run:

| Check | Expected |
|---|---|
| Wall table | `rel [63,63,62,45,22,6,9,5] ± run noise (mcs5-7 within ±5)` |
| MCS 0-2 | flagged `no_dip`, wall = `+63` |
| `legacy_wall_rel` | equals the MCS0 result (63) |
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
| Re-link after the session | the drone is in `RENDEZVOUS` by ~33 s into the coarse phase, every run (see above) — confirm it rejoins and that an IDR lands at the join, after BOTH a successful and an aborted run |

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

### 2026-09-14 relative walls

Deployed commit `3d1bbe1` (branch `relative-walls`). `RC_VERSION` 7 → 8,
plus `/etc/mabur.toml` swapped from `rate_walls_idx`/`legacy_wall_idx`/
`base_ref_idx` to `rate_walls_rel`/`legacy_wall_rel` — a config-key flag
day on top of the binary flag day, drone stopped/config+binary
swapped/started together as usual. Both device binaries rotated
(`maburd.pre-relwalls`, `maburgs.pre-relwalls`, `maburcal.pre-relwalls`);
md5s of the deployed files matched the local build outputs exactly.
Drone daemon PID (926) stayed constant across the whole session — no
respawn observed at any point.

`maburcal start` on ch136 (nonce 1651516276, margin 1.00 dB):

| rate | wall(rel) | park | verify | flags |
|---|---|---|---|---|
| mcs0 | 63 | 59 | 100% | no_dip |
| mcs1 | 63 | 59 | 100% | no_dip |
| mcs2 | 63 | 59 | 100% | no_dip |
| mcs3 | 41 | 37 | 100% | narrow,drift |
| mcs4 | 24 | 20 | 100% | card_disagree |
| mcs5 | 9 | 5 | 100% | drift |
| mcs6 | 9 | 5 | 100% | |
| mcs7 | 6 | 2 | 100% | drift |

`legacy_wall_rel = 63` (derived from mcs0). `written: /etc/mabur.toml`.
mcs5/6/7 landed at 9/9/6 — within ±5 of the 2026-09-13 reference
(6/9/5) — and mcs0-2 parked `no_dip` at the 63 rail, matching
expectations. Drone confirmed post-write: `rate_walls_rel = [63, 63,
63, 41, 24, 9, 9, 6]`, `legacy_wall_rel = 63`, no `base_ref` key,
`power_mode = "offset"`.

`ausniff` (15 s passes, `/dev/shm/mabur-au`), with the `incomplete`
count that the first write of this section omitted — **most passes
below are not fully clean**; only the two rows marked `{}` + `fid_gaps
0` are:

| point | aus | incomplete | fid_gaps | resyncs | fps |
|---|---|---|---|---|---|
| post-deploy, pass 1 | 907 | `{'1': 1}` | 0 | 0 | 60.5 |
| post-deploy, pass 2 | 909 | `{'1': 3}` | 0 | 0 | 60.6 |
| post-calibration, pass 1 | 895 | `{}` | 3 | 0 | 59.7 |
| post-calibration, pass 2 | 907 | `{'1': 1}` | 1 | 0 | 60.5 |
| post-calibration, pass 3 | 907 | `{'1': 2}` | 0 | 0 | 60.5 |
| home-scan (candidates default) | 908 | `{'1': 2}` | 0 | 0 | 60.6 |
| forced `candidates=[149]`, `home_margin=0`, pass 1 | 892 | `{'1': 2}` | 1 | 1 | 59.5 |
| forced `candidates=[149]`, `home_margin=0`, pass 2 | 909 | `{'1': 3}` | 0 | 0 | 60.6 |
| config restored, final | 908 | `{}` | 0 | 0 | 60.6 |

`fid_gaps` resolved to 0 on a follow-up pass every time it went
nonzero, in line with the documented first-pass-after-restart phantom
— but `incomplete` on stream `'1'` did not: only the post-calibration
pass 1 row and the final config-restored row show `{}`, and those two
still disagree with each other on `fid_gaps` (3 vs 0). The other seven
rows all carry a nonzero `incomplete['1']`; see the steady-state and
TX-power A/B passes below for the clean-bench characterization of that
residual.

Channel auto-select: with the shipped defaults
(`candidates=[120,149,165]`, `home_margin=20`) the scan committed to
home 136 (`K none 0` / `M all 136 136 commit`) — expected on a quiet
bench. Forcing `candidates=[149]`, `home_margin=0` and restarting three
times still produced `K none 0` / `M all 136 136 commit` every time:
136 was never worse than 149 by any margin at this bench's noise
floor. `pgrep -a maburd` on the drone during this window returned
`926 /usr/bin/maburd` unchanged on both a check taken mid-wait for the
first forced-candidate restart and a second taken mid-wait for the
third — no respawn. The GS config was restored to
`candidates=[120,149,165]`, `home_margin=20` and confirmed live via
grep; **the pair ran the whole session, and finished it, on channel
136**. Step 8 (ch149 cross-channel re-check) was skipped by controller
ruling.

### 2026-09-14 follow-up: steady-state and TX-power A/B

The passes above were taken while the operator was also exercising
channel-select restarts on the GS, so they are not a steady-state
baseline. With the pair left untouched, three clean passes at the
calibrated table, three at an old-power control (the pre-deploy
absolute parks re-expressed relative to the ch136 anchor 39 —
`rate_walls_rel = [59, 59, 59, 25, 26, 2, 5, -1]`, applied via a drone
restart), and two after restoring the calibrated table (md5
`91aac883f215c2d975d00db420f37bb8` on the drone):

| point | aus | incomplete | fid_gaps | resyncs | fps |
|---|---|---|---|---|---|
| calibrated `[63,63,63,41,24,9,9,6]`, pass 1 | 1205 | `{'1': 1}` | 1 | 0 | 60.3 |
| calibrated, pass 2 | 1205 | `{'1': 1}` | 0 | 0 | 60.3 |
| calibrated, pass 3 | 1206 | `{'1': 1}` | 0 | 0 | 60.3 |
| old-power control `[59,59,59,25,26,2,5,-1]`, pass 1 | 1205 | `{'1': 1}` | 0 | 0 | 60.3 |
| old-power control, pass 2 | 1205 | `{'1': 1}` | 1 | 0 | 60.3 |
| old-power control, pass 3 | 1204 | `{'1': 1}` | 1 | 0 | 60.2 |
| calibrated table restored, pass 1 | 1205 | `{}` | 0 | 0 | 60.3 |
| calibrated table restored, pass 2 | 1204 | `{'1': 2}` | 1 | 0 | 60.2 |

The incomplete-enhancement-AU residual (~0.08% of enh AUs) is the same
at the old and new TX-power tables, so it is bench background loss and
not attributable to relative walls.
