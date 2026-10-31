# TX-power walls and air capacity — `maburcal`

An operator ssh'd into the ground station measures, per channel width the
link flies, two per-unit hardware facts of their own vtx and writes them
into the drone's config:

- the **walls**: the TXAGC index at which each MCS's PA starts to
  compress (`radio.rate_walls_rel_20` / `_40`), and
- the **efficiency**: the share of the nominal PHY rate the air really
  delivers at each MCS (`air_clock.efficiency_20` / `_40`), which the
  drone's bitrate formula and air clock price against
  (`docs/airtime-model.md` §7).

Both are measured on this drone and this GS, under the coding flags the
link flies (`radio.ldpc`, STBC on), at the drone's production frame size
and aggregation. A full run of four sweeps takes about 80 seconds:

```
$ ssh root@<gs>
# maburcal start                 # everything the GS's ladder needs
# maburcal start power20 cap40   # a subset, any order; the GS sorts it
# maburcal status
# maburcal abort
# maburcal report /media/dvr/log/cal/cal.log
```

No toolchain, no repo checkout, no laptop-side step. Why the run has the
shape it has (20 frames per cell, one pass, a 2 s capacity dwell):
`docs/findings/2026-10-31-calibration-shortening-spike.md`. Earlier
measurements that are still hardware fact — the comb past the first dip,
the transfer curve, channel-to-channel repeatability:
`docs/findings/2026-07-16-txagc-calibration.md`,
`docs/findings/2026-09-11-maburcal-bench.md`.

## Running it

`maburcal` (`gs/bundle/maburcal`, installed to `/usr/bin` by
`gs/bundle/install.sh`) speaks one UDP datagram at a time to `maburgs`'
`CalControl` listener on `127.0.0.1:8400`, loopback only: you have to be
logged into the GS to start a run.

**What a run measures.** A run is a list of sweeps, each a kind (`power`
or `cap`) at a width (20 or 40). `maburcal start` with no arguments runs
the GS's own composition (`default_sweeps`, `gs/src/cal_control.h`):
`power20, cap20`, plus `power40, cap40` when the GS's `radio.width` is 40
**and** some `[[link.ladder]]` rung has `bw = 40`. The width is the
narrower of the GS's `radio.width` and the linked drone's DISC_ACK
`agreed_width`, so a 40 MHz GS linked to a drone flying 20 composes the
20 MHz pair only. Arguments `power20|power40|cap20|cap40` pick a subset;
duplicates drop. Whatever was typed, the session runs widths ascending and,
within a width, **power before capacity**: capacity is measured at the
power the walls just parked. The reply names the run as it will go:
`ok started power20,cap20,power40,cap40`.

Refusals come back at once, before anything airs:

| reply | why |
|---|---|
| `err bad token: <tok>` | not one of the four tokens |
| `err refused: GS radio.width is 20` | a `*40` token while the effective width (above) is 20 |
| `err a calibration session is already running` | a run, or its channel pick, is in progress |
| `err refused: link is down` | `maburgs` is not in `SESSION` |
| `err refused: peer does not advertise CAP_CALIBRATE` | the drone's DISC_ACK lacks the bit (a build without the mabur-dialect calibration) |

The "GS radio.width is 20" wording is also what a 40 MHz GS says when it
is the linked drone that flies 20.

**Progress.** `start` then polls `status` every 500 ms and prints each
reply, until the state is `done`, `failed` or `idle` (aborted):

```
ok state=sweep sweep=2/4 kind=capacity bw=20 cell=3/8
```

`state` is one of `idle picking await_ack sweep await_done analyze apply
verify done failed`; `sweep=k/n` the running sweep, 1-based; `kind` is
`verify` during a verify pass; `cell=i/n` the cell of the latest frame the
GS tallied (840 power cells per sweep, 8 capacity points, up to 8 verify
cells). `mismatch=<n>` appears once frames have been dropped for a kind,
width or flags that do not match the running sweep (see "The sweep
frame"). A `failed` line ends with `fail_reason=<why>` (free text to the
end of the line). Then `start` finds the run's `cal.log` and prints the
final table (below). `maburcal abort` cancels a run in any state.

**Run it with `[radio] relays = []`.** The session tallies only cards 0
and 1, and relay cards come after the USB cards in the roster: on a GS
with one USB card the relay *is* card 1, and its sweep frames (another
antenna, ath9k RSSI) would enter the cells. `[mavlink] enable` is not
needed: the tunnel calibration rides is always built; the UDP endpoint is
a separate tenant (`docs/mavlink.md`).

### The pair must already be linked and flying video

There is no other way to reach the drone: every calibration message rides
the MAVLink tunnel inside the authenticated RC link. If the link is down,
fix that first — calibration cannot bring it up.

## What a run does

### 1. Channel pick

Before the first sweep the GS puts the pair on the quietest member of
`radio.channels` (`CalPicker`, `gs/src/cal_picker.h`, over `ChannelCore`):

- **Auto, two or more cards** (`channel = "auto"`): one fresh measure
  round by the boot scout — every 20 MHz half of every member once, op's
  own pair skipped (it carries the drone's video); op is read from the
  verdict's live window instead. Both sides are scored on the terms that
  mean the same thing on either instrument: decoded foreign frames per
  `radio.scan.dwell_ms` plus NHM foreign busy %. A pair scores as its
  worse half; leaving op takes beating it by `radio.scan.pick_margin` in
  every tier, and a blocked candidate never replaces an unblocked op; op
  unread means stay. When the best member is not op, the GS orders the
  boot pick's relocate (scan.log `H relocate … cal`, no verify window —
  the confirm lands it) and waits for it to settle. Detail:
  `docs/channel-select.md` "Calibration pick".
- **Pinned, or one card** (no spare card, or a relay-only roster): no
  measurement; the run uses op.

The whole pick is bounded by `pick_timeout_ms` (8 s); the round itself
by twice a full round's dwell time (4.5 s at the defaults). A relocate
that is refused (hop cap, a hop in flight) or does not confirm leaves the
pair on op and the run proceeds there. The outcome is the `pick` token on
`cal.log`'s `R` line (`auto|pinned|onecard|stayed`) and the note on the
start line (`auto: 112 is the best member`, `auto: moved to 112`, `stayed
on 64, relocate unconfirmed`, `stayed on 64, relocate refused`, `stayed
on 64, measure timed out`, `pinned`, `onecard`). The remembered-channel
store follows a calibration relocate like any hop.

### 2. Per sweep

1. **Command.** The GS sends `MABUR_CAL_SWEEP{nonce, seq, kind, bw, …}`
   every 200 ms until acked; 3 s without an ack fails the run
   (`calibration ack timeout`). `seq` is the sweep's position in the run.
   The drone is idempotent by (nonce, seq): a repeat of the running seq
   re-acks without touching the sweep, an older seq is ignored, a newer
   one starts the next sweep. One hold: a SWEEP is not sent while a scout
   dwell begun just before the start still has a card off-channel
   (`cal_cmd_clear`, `gs/src/hop_burst_gate.h`).
2. **Ack.** `MABUR_CAL_ACK{nonce, seq, result, start_in_ms, flags}`. The
   drone starts the sweep 200 ms after accepting it, so the ack can land
   first; `flags` is the stamp mask the sweep flies (`ldpc`, `stbc`), and
   the GS tallies only frames carrying it. A CRC-clean frame of the
   commanded kind and width stands in for a lost ack. A refusal
   (`anchor_unreadable`: the chip cannot report its TXAGC reference) fails
   the run.
3. **Silence.** From the ack the GS sends nothing — no RCF, no DISC
   keepalive, no live MAVLink — for the sweep's planned duration plus
   `phase_slack_ms` (1 s) (`CalSession::radio_silent()`).
4. **Done.** When its last frame is out the drone sends
   `MABUR_CAL_SWEEP_DONE{nonce, seq, offered[8], blocked_pct[8]}`, then
   repeats it 100 ms later and every 200 ms after that until the run's
   next command arrives or the run closes. The first copy ends the GS's
   silence early; duplicates are ignored. Without any copy the GS's window
   closes at planned end + 1 s.
5. **Analyse.** Power: one wall per rate (below). Capacity: one
   efficiency per rate (below).
6. **Apply.** `MABUR_CAL_APPLY{nonce, seq, bw, has, walls[8], legacy,
   eff[8]}` every 200 ms until `MABUR_CAL_APPLIED{nonce, seq, bw, ok,
   reason}`; 5 s without an answer fails the run (`calibration apply
   timeout`), a refusal fails it with the drone's reason. An adopted
   apply's `reason` is empty, or a note of what the drone kept that the
   apply did not carry (below, "What gets written"). `seq` names the
   sweep the apply belongs to: a run applies twice at one width (power,
   then capacity), and the drone answers a repeat of an apply it already
   made from its cache instead of applying again. A sweep with nothing to
   write (every wall undetermined, every capacity point `tx_bound`) sends
   no apply.
7. **Verify** (power applies only). The drone parks each newly written
   rate at `park = clamp(wall − round(wall_margin_db · 4), −64, 63)` and
   sends 20 frames there (5 ms settle, 1 ms gap), starting 200 ms after
   the apply; the GS computes the same parks (`mabur/cal_park.h`, one rule
   for both ends), stays silent for the pass plus 1 s and tallies delivery
   per rate into the `verify` column.

Between sweeps the GS is not silent: RCF and the DISC keepalive go out as
usual while it waits for an ack or an APPLIED.

### 3. The power sweep

One full-resolution pass per width: every relative index −41..+63 for
each of the eight MCS rows, 840 cells, **20 frames per cell, 5 ms settle,
1 ms gap** — 21 s (`gs/src/cal_plan.h`). The low end is −41, not −40, so
the grid lands on **+63 exactly**: a no-dip row parks at that rail, and
the rail has to be a cell the run measured. Each frame is the 64-byte
stamp (below), sent alone (`no_agg`) at the cell's (MCS, width), LDPC per
`radio.ldpc`, STBC on. The drone zeroes the per-rate diff table and reads
the chip's anchor once per run, at its first accepted command; every
power sweep zeroes the diffs again, so each row is swept from the same
base.

**The wall is the end of the first contiguous ≥90 % delivery run**,
scanning upward from the sensitivity floor. Past the first dip delivery is
a reproducible *comb*, not a cliff — islands of ≥90 % reappear higher up,
and they are **not usable headroom**. That is why the sweep walks the
whole range rather than bisecting: a search that stopped at a "good"
reading past a bad one would land inside the comb and overdrive the PA.
Scoring uses the **best single card**, never the union of both:
diversity protects against fading, not against PA compression (both cards
receive the same degraded waveform), and a union would push the wall up
(`gs/src/cal_analysis.cpp`). Every shortcut errs low, never high.

### 4. The capacity sweep

Eight points (MCS 0–7) at the sweep's width, each a **300 ms settle then a
2 s dwell** of back-to-back frames (18.4 s), at the drone's production
power for that width (its plan diffs live, no index override — the walls
the preceding power sweep just wrote, or the existing table when the run
has no power sweep at that width) and in the production A-MPDU mode
(`ampdu_mode_for(ampdu, mcs, bw)`). Frames are the stamp padded to the
video body size at the configured geometry, `13 + blocks_per_body[1] ·
(16 + symbol_size[1])` (1405 B at the bundle's `[fec]`), sent as stream 1
through `RadioTx::send_bodies` in batches of three, so every
`radio.tx_threads` sender stays busy. The drone counts frames `offered`
and `blocked_pct`, the share of the dwell its submitter spent waiting for
the device (the `UsbTxPool`'s space-wait counter; with `tx_threads = 1`
the time inside the synchronous send).

The GS sums each CRC-clean frame's air size — the 26-byte QoS-Data
header, the body and the 4-byte FCS — on the best card, and computes

```
eff = delivered bits / (dwell_s × nominal(mcs, bw))
```

with `nominal` the HT long-GI PHY rate (`rc::phy_rate_mbps`), rounded to
0.01 and clamped to [0.05, 1.0] (`analyze_capacity`). A point whose
sender was blocked less than 50 % of the dwell measured the drone's CPU or
USB, not the air: it is flagged `tx_bound` and **not written** (the
config keeps its value, as an undetermined wall does). A point that
delivered under 90 % of what was offered is flagged `lossy` and is
written — delivered is delivered — but the parked power or the geometry
is suspect. A capacity sweep whose SweepDone never arrives writes nothing:
without `offered` and `blocked_pct` a `tx_bound` point cannot be told
apart.

Why delivered bits and not a fit: the air clock and the bitrate formula
price delivered capacity, and the bench numbers the bundle carried were
defined as exactly this ratio, so a measured table replaces them like for
like.

### 5. The end of a run

After the last sweep the session is `done`, and the GS sends
`MABUR_CAL_ABORT` three times, 200 ms apart: the drone closes the run at
once — restoring the current rung's width table, clearing the override and
resuming video — instead of waiting out its 15 s `await_next_ms`. The
abort keeps everything the applies wrote. `maburcal abort` and a failure
send the same three Aborts, if a SWEEP ever went out.

The drone needs none of it to get home: every state of its sweep engine
(`CalSweep`, `drone/src/cal_sweep.h`) has a deadline — `await_next_ms`
(15 s) for the next command after any sweep, apply or verify, and
`hard_cap_ms` (180 s) for the whole run from the first accepted SWEEP.
Killing `maburgs` mid-run leaves the drone to time out, restore its power
and resume video on its own.

### Timing

| step | time |
|---|---|
| channel pick | 0 pinned or one card; ~2 s for the round (six halves at 280 ms) plus up to `[hop] confirm_ms` for a relocate |
| power sweep | 21 s per width (840 × 25 ms) + 200 ms start + ack |
| apply + verify | ~1.5 s per power sweep (8 × 25 ms of verify air, 200 ms start, 1 s listen tail) |
| capacity sweep | 18.4 s per width (8 × 2.3 s) + 200 ms start + ack |
| apply (capacity) | one round trip |

The host loopback test (`test_cal_e2e`) reaches `done` for four sweeps at
82 s simulated and for `power20` alone at 22 s. The target is a full run
well under two minutes and a walls-only 20 MHz run under half a minute.

## The drone during a run

**Video stops for the whole run.** From the first accepted SWEEP until
the run closes (`cal_active`) the drone queues no video and drains what
was queued; Telem and the MAV component's own uplink pause while a sweep
or verify pass airs (`cal_sweeping`) and go out between sweeps. Its
calibration replies go out whenever they are due. Video resumes when the
run closes.

**It will be in `RENDEZVOUS` by the end of every run.** Every sweep
outlasts `link.failsafe_ms` (3 s), so the drone's `RcAgent` ages out of
`LINKED` inside every sweep, and `link.rendezvous_ms` (30 s) later is in
`RENDEZVOUS`. It may re-link in a gap between sweeps or stay out until the
run ends; either is benign. `set_ladder`, the A-MPDU switch, retunes and
the TX-power table are all gated on `cal_active` (a retune ordered
meanwhile is deferred and replayed after), so the agent cannot fight the
sweep for the radio. The operator's only job is to **confirm the pair
re-links after each run**: video back within a couple of DISC beacons,
with an IDR at the join. If it does not, that is the ordinary stale-caps
shape, not a calibration bug (`docs/deploy.md`).

**The run survives the lapsed session.** Leaving `LINKED` clears the
drone's pairing session, yet the run's later messages are still tagged
under the pair it started with. So the drone's `RcAgent::check_mav`
re-verifies a T_MAV frame that fails the published session under the
**run pair** — the pair of the last accepted cal-only frame — while a run
is open, with its own counter domain; it accepts that way only frames
made entirely of calibration messages, and a parameter set in such a
frame drops the whole frame. The GS freezes the tag context when the run
first commands the drone and packs calibration messages on the tunnel's
cal lane, which tags with that frozen context whatever its own session
state. The drone's replies go out untagged from the TX writer thread,
ungated by session state. Detail: `docs/mavlink.md` "Calibration over the
tunnel".

**No GS card changes channel.** From the first SWEEP to the end
(`CalSession::running()` → `ChannelCore`'s `cal_running`) the scout,
in-flight verdict, hop controller and periodic scout are all off, so no
card leaves the channel being measured; a link-pairing move edge armed by
a re-pair between sweeps is held until the run ends (`CalMoveEdgeHold`).
The pick (above) runs before this freeze, which is why `picking` is
outside `running()`.

## The sweep frame (stamp)

`common/include/mabur/cal_wire.h`, a 12-byte header:

```
magic "MCAL" | u8 kind (1 power, 2 verify, 3 capacity) | u8 rate | i8 idx |
u8 bw (20/40) | u8 flags (bit0 ldpc, bit1 stbc) | u8 seq_hi | u16 seq (LE)
```

then fill `(0x5A ^ idx)` to the frame length (64 B for power and verify,
the video body size for capacity). `seq` is a 24-bit running frame
counter. `idx` is signed and relative to the chip's anchor. The parser
refuses an unknown kind, a rate over 7, an index outside [−64, 63], a
width other than 20/40, unknown flag bits, or a fill that does not match
(with or without a trailing FCS): `maburgs` keeps CRC-bad frames, and the
fill check is what stops a corrupted frame being attributed to the wrong
cell. A frame whose kind, width or flags do not match the running sweep
is counted (`stamp_mismatch`, the status line's `mismatch=`) and never
tallied.

## One calibration covers every channel

Walls are signed indices **relative to the chip's own TXAGC anchor**. The
anchor is the per-channel-group reference devourer programs from the
module's efuse on every channel set, and the chip adds it itself, so a run
on any member of the channel set produces a table valid on every member.
The anchor never leaves the drone: it is not in config, not on the wire,
not in `cal.log`. `maburd` re-applies TX power (`ReApplyTxPower()`) after
every retune, so the diffs sit on the anchor of the channel the link is
on. Across three channel groups on one unit (anchors 39 / 53 / 57) the
rows with a real wall stayed within 3-5 indices relative to the anchor
while the absolute wall moved 13-17 with it
(`docs/findings/2026-09-11-maburcal-bench.md`). Same-channel
repeatability at 20 frames per cell is 0-1 index (the spike page).
An 8812CU has a flat anchor, so its walls are exact only on the channel
calibrated (`docs/deploy.md` "Supported drone cards").

## Per-width tables on the drone

```toml
[radio]
power_mode        = "none"      # "offset" flies the walls below
rate_walls_rel_20 = [63, 63, 63, 63, 45, 23, 22, 21]
rate_walls_rel_40 = [63, 63, 63, 63, 45, 28, 28, 23]
cal_flags_20      = "ldpc+stbc"
cal_flags_40      = "ldpc+stbc"
legacy_wall_rel   = 63
wall_margin_db    = 1.0

[air_clock]
efficiency_20 = [0.93, 0.88, 0.84, 0.80, 0.76, 0.76, 0.78, 0.76]
efficiency_40 = [0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75]
```

- **`rate_walls_rel_20` / `_40`** are both optional; `power_mode =
  "offset"` needs at least one. Every entry is in [−64, 63] and
  `wall − round(wall_margin_db · 4) ≥ −64`. The single-table
  `radio.rate_walls_rel` is gone: a config that still has it fails boot
  (unknown key) — rename it by hand (`docs/deploy.md`).
- **A width with no table flies the other width's** (`RadioCfg::walls_for`)
  and `maburd` says so at boot: `maburd radio: no 40 MHz walls, flying the
  20 MHz table at 40`. Walls at 40 MHz have measured ≥ the 20 MHz ones on
  every row so far, so the 20 MHz table at 40 is the conservative
  direction; the reverse (40's table at 20) is not, and the line is the
  hint to calibrate that width.
- **`cal_flags_20` / `_40`** record what the table was measured under:
  `ldpc+stbc`, `stbc`, `ldpc` or `none` (default `ldpc+stbc`). Walls
  measured with LDPC off sit below the LDPC-on ones, so flying an
  LDPC-on table with `radio.ldpc = false` is the overdriving direction.
  `maburd` warns at boot — `maburd warning: radio: 20 MHz walls were
  measured with ldpc on; radio.ldpc is off — re-run maburcal` — and a
  `RADIO_LDPC` parameter set that creates the mismatch answers `ACCEPTED`
  plus a `STATUSTEXT` WARNING with the same text (`docs/mavlink.md`). It
  warns, it does not refuse. Both checks consider only widths the radio
  can fly (40 only with `radio.width = 40`) and only under `"offset"`.
- **`legacy_wall_rel`** (control and telemetry: legacy/CCK) is one
  number for both widths, derived from the **20 MHz mcs0** wall — there
  is no legacy row to sweep, and legacy 6 Mb/s and HT MCS0 are both BPSK
  1/2 on the same OFDM waveform, so their PA wall should track. A
  physical assumption, re-checked on every unit by the acceptance
  checklist. A run with no 20 MHz power sweep derives it from 40 MHz mcs0
  only when the config has no `rate_walls_rel_20` either.
- **The live table follows the rung.** The chip has one per-rate diff
  table (no width axis), so `maburd` keeps one plan per width
  (`make_width_plans`, `drone/src/power_tables.h`) and programs the one
  for the applied rung's width on every rung change, under the TX gate
  (`LivePowerTable`, `drone/src/main.cpp`). Each switch logs `maburd
  radio: power table 40 MHz live (mcs5 diff N)`, with ` (20 MHz table)`
  appended when that width flies the other's table; with no walls at all
  (`"none"`) it is `power table: chip default (no walls)` after
  calibration has touched the chip. Bring-up programs `radio.width`'s
  plan; a `RADIO_WALL_MARGN` set rebuilds both plans and writes the live
  one (refused "calibration in progress" during a run). Control frames are
  20 MHz MCS0 whatever the rung and ride the legacy diff in both plans.
- **The efficiency tables** are read at `maburd` start only: a run's new
  `efficiency_<bw>` takes effect at the next `maburd` start (walls take
  effect when the run ends).

## What gets written, and how to roll back

Per apply, line-surgical in `/etc/mabur.toml` (`drone/src/cal_apply.h`):

| apply | keys |
|---|---|
| power at `<bw>` | `radio.rate_walls_rel_<bw>`, `radio.cal_flags_<bw>` (the sweep's stamp flags), `radio.legacy_wall_rel` (20 MHz, or 40 MHz with no 20 MHz table), `radio.power_mode = "offset"` |
| capacity at `<bw>` | `air_clock.efficiency_<bw>` |

An undetermined wall (`-128` on the wire) and a capacity point sent as 0
(`tx_bound`) leave that entry as it is. A key the file lacks — a first
40 MHz calibration on a file with only `rate_walls_rel_20` — is
**inserted** next to the other width's line (or after `power_mode`;
`efficiency_<bw>` after the other width's or after `shed_ms`), the one
place the patcher adds a line. Every comment and every other key survive
byte for byte.

An inserted line has no entry of its own for a left rate to keep:

- **Walls, never the rail.** An undetermined rate in a new
  `rate_walls_rel_40` takes that rate's `rate_walls_rel_20` wall — 40 MHz
  walls measure at or above the 20 MHz ones, so it errs low — and the
  APPLIED reason says so: `ok; mcs6 kept from the 20 MHz table`. With no
  20 MHz table to take it from (a first 20 MHz table, whatever the file
  holds at 40, or a first 40 MHz table with no 20 MHz one) the apply is
  refused and nothing is written: `undetermined mcs5,6; no table to keep`.
  A rate kept this way is not verified (nothing was parked for it).
- **Efficiency.** A left rate in a new `efficiency_<bw>` takes what the
  loader flies for that width with the key absent (`AirClockCfg`'s
  default, nominal 1.0 today), so the insert changes nothing the drone
  flew for that rate.

Before anything is written the candidate is validated with the loader's
own rules (every wall in [−64, 63], `wall − round(wall_margin_db · 4) ≥
−64`, every efficiency in [0.05, 1.0]), patched, written to
`<path>.new`, fsynced and loaded with `mabur::load_config` — the function
`maburd` boots with — and only then published by an atomic rename, under
the config-write lock shared with the MAVLink parameter component (a
parameter set during a run is refused "calibration in progress"). A
config `maburd` cannot load would crash-loop it at 2 s (`docs/deploy.md`);
every refusal leaves the file exactly as it was, and APPLIED carries the
reason. The drone's log: `maburd cal: applied 20 MHz walls` /
`… efficiency`, or `maburd cal: apply 20 MHz refused: <why>`.

**Backup.** The first successful apply of a run copies the file as it was
before the run to `/etc/mabur.toml.pre-cal`; later applies of the same run
do not touch it. So `.pre-cal` is always the file from immediately before
the most recent run that applied anything.

**To roll back**, restore it on the drone and restart `maburd` (from a
non-interactive ssh command, start the wrapper under `setsid` —
`docs/deploy.md`):

```sh
cp /etc/mabur.toml.pre-cal /etc/mabur.toml
/etc/init.d/S00mabur restart
```

`maburcal start` takes no margin: `wall_margin_db` is the operator's
safety choice, set on the drone (`RADIO_WALL_MARGN` or by hand) and never
written by the kit. The report's `park` column uses the GS's own 1.0 dB
(the `margin=` on the header line); the drone parks verify at its own
margin and the GS tallies verify by rate, so a different drone margin
changes only what `park` predicts.

## Walls and efficiency are re-measured after a wipe, never restored

The shipped bundle files are the live flight configs, verbatim, and the
standing rule is "retune in the repo or it's lost at the next wipe".
**That rule does not cover the walls and efficiency tables.** They are
per-unit measurements; the numbers in `bundle/mabur.default.toml` are one
reference unit's, kept as documentation and inert under the bundle's
`power_mode = "none"` (efficiency is not inert — it prices every rung —
which is one more reason to calibrate a new unit before flying it). After
a drone wipe or replacement, run `maburcal start`; restore everything
else from the bundle as-is.

## Geometry

Put the drone and GS at a normal bench distance — close enough that the
low end of the sweep (weak transmissions on the fastest rows) has a
chance to be heard, far enough that the high end does not compress the
GS's own RX front end. The `saturated` flag watches the second failure;
if it fires, move the drone out and re-run rather than trusting the
table. Run where the air is quiet (the pick helps): random loss on both
cards reads walls low (below).

## Reading the result

```
run nonce=1873455120 margin=1.00dB ch=112 pick=auto ldpc=1
                20 MHz                    40 MHz
rate    wall  park verify    eff     wall  park verify    eff   flags
mcs0      63    59   100%    .93       63    59   100%    .77   20:no_dip 40:no_dip
mcs1      63    59   100%    .88       63    59   100%    .75   20:no_dip 40:no_dip
mcs2      63    59   100%    .84       63    59   100%    .75   20:no_dip 40:no_dip
mcs3      63    59   100%    .80       63    59   100%    .75   20:no_dip 40:no_dip
mcs4      45    41   100%    .76       45    41   100%    .74
mcs5      23    19   100%    .76       28    24   100%    .77
mcs6      22    18    95%    .78       28    24    99%    .76
mcs7      21    17   100%      -       23    19    98%    .75   20:tx_bound
legacy 63 (from 20 MHz mcs0)
written: /etc/mabur.toml (backup /etc/mabur.toml.pre-cal)
```

One row per rate; per width the wall, its park (`wall − round(margin ·
4)`), the verify delivery at the park, and the efficiency. `-` means this
run has no number there (that kind or width was not in the run, or the
wall was undetermined). Flags carry a `20:`/`40:` prefix. A run that never
swept 40 prints the 20 MHz block alone; a width with capacity points but
no walls in this run adds `<bw> MHz eff measured at the existing walls`.
A `tx_bound` point's `eff` prints `-`: it was measured (`cal.log` keeps
it) but never written, and the flag says why. An adopted apply that kept
something adds a line such as `40 MHz apply: ok; mcs6 kept from the 20 MHz
table`. The last line comes from the apply records: `written: …` when any
apply succeeded, else `not written: <bw> MHz: <reason>; …` or `not
written: this run produced no apply attempt`.

### Health flags

| Flag | Meaning | What to do |
|---|---|---|
| `no_dip` | The row never dropped below 90 % anywhere: no wall to find, so it parks at the **rail**, `+63`, the top of the 7-bit diff field — a constant, not a measurement. | Normal for MCS 0-3 under LDPC+STBC. If it is *missing* on those rows, see "Random loss reads walls low". |
| `undetermined` | No cell reached 90 % anywhere: no first run exists. `-128` in `cal.log`. | The wall is left as it was; in a first 40 MHz table it takes the 20 MHz wall, and a first table with nothing to take from is refused (above). Never the rail. Check geometry and re-run. |
| `narrow` | The floor edge (first ≥90 % cell) is within 4 indices of the wall. | Move closer and re-run; a wall that close to its own floor is not trustworthy. |
| `saturated` | The best card's peak median RSSI is above −45 dBm: the GS front end may be compressing. | Walls read **low** under it (the safe direction). Move the drone out and re-run for a tighter number. |
| `card_disagree` | The two cards' walls differ by more than 2 indices. | An antenna or card problem, not a PA one: check the disagreeing card. |
| `tx_bound` | Capacity: the sender was blocked < 50 % of the dwell — the drone's CPU/USB, not the air, was the limit. | Not written; the config keeps its value. Expected at the top MCS if the drone cannot saturate the air there. |
| `lossy` | Capacity: the best card got < 90 % of the frames offered. | Written. Interference, or the parked power is too low for this geometry; re-run on a quieter channel if it fires on low MCS. |

Thresholds are `CalThresholds` in `gs/src/cal_analysis.h`. The bit values
are in "`cal.log`" below; `drift` (the old coarse/fine disagreement) is
retired.

### Random loss reads walls low

With 20 frames per cell a 90 % threshold has no noise margin: one cell
that loses 3 frames ends the first contiguous run, and the row's wall
becomes the cell below it. Because the error always ends the run early,
it is always **low** — power given up, never overdrive — but a lossy
channel can take a wall down by tens of indices: a few percent of random
loss on **both** cards collapses most rows (loss on one card alone is
absorbed by the best-card rule). The kit does not detect this. The tell is
MCS 0-3 coming back **without** `no_dip`, or walls far below the
reference unit's; re-run on a quieter channel or closer geometry.

## When it doesn't work

**`start` refuses at once** — see the refusal table under "Running it".
A half-deployed pair (mismatched `RC_VERSION`) never reaches `SESSION`,
so it shows as `link is down`, not as the capability error. A
`CAP_CALIBRATE` refusal on a pair that otherwise links means one side
predates the dialect calibration: deploy both binaries from the same
commit.

**The run ends `failed`.** The `fail_reason=` on the last status line
says which:

- `calibration ack timeout` — no ack, and no frame standing in for one,
  within 3 s of SWEEPs every 200 ms. The uplink is lossy by design
  (`docs/findings/2026-08-14-rcf-uplink-loss.md`); improve geometry and
  retry before suspecting anything else.
- `sweep refused by the drone: anchor_unreadable` — the chip did not
  report its TXAGC reference. The drone restores its power table at once
  (`maburd cal: sweep refused (TXAGC anchor unreadable); restoring
  operating power`).
- `calibration apply timeout` — no APPLIED within 5 s of APPLYs.
- `apply refused by the drone: <reason>` — the drone validated and
  refused: an out-of-range table, a key it could not find or insert, a
  first table with undetermined rates and no table to keep them from
  (`undetermined mcs5,6; no table to keep`), a backup or write failure, or
  a candidate that would not load. The file is unchanged; `/tmp/mabur.log`
  on the drone has the full reason.

A failure after an earlier apply succeeded keeps that apply: the walls of
a finished `power20` stay written when a later `cap40` fails. The report
lists every apply's outcome.

**The `verify` column is blank for a width that has walls.** The verify
pass follows only an APPLIED ok, so that width's apply did not land: the
run's `fail_reason`, and the report's last line, say why (it reads
`written` when another width's apply landed). A `-` in `verify` on a rate
whose wall is also `-` is just an undetermined rate: nothing was parked
there.

**One rate reads `0%` in `verify` while its neighbours show real
percentages.** This rate was parked, the drone did sweep verify (every
other rate proves it), and the GS heard nothing at this rate's park: the
config on the drone right now is untransmittable at this MCS and width.
Likely a wall a little optimistic without quite tripping a flag, or a
geometry problem specific to that rate's card. Re-run first; if it
repeats, widen `radio.wall_margin_db` (several rates) or keep that rate
out of the ladder until a re-run gives it a real number.

**Verify reads low on a rate with no flag.** Flags come from the sweep;
verify has none of its own, and geometry can move between the sweep and
the verify a few seconds later. Treat the verify number over the flag:
re-run, or widen the margin on the drone and re-run.

**`eff` is `-` for a width whose capacity sweep ran.** Every copy of its
SweepDone was lost inside the GS's 1 s listen tail, so nothing was written
for that width. Re-run `cap<bw>`.

**Every high-MCS point is `tx_bound`.** The drone cannot saturate the air
at those rates; the table keeps its existing values there, which is the
truthful outcome.

**A latched SWEEP was dropped.** A SWEEP accepted only under the run pair
that names a nonce other than the open run's is dropped and counted:
`cal_latch_drop=N` on the drone's `maburd tx_send:` line (cumulative,
printed only while video flows). A frame that fails both the published
session and the run pair is a `reject=` on the `stats:` line's `mav` part
and raises Telem's `auth_reject`. Both stay at zero on a normal run.

## Observability

**`maburgs` log** (`/tmp/maburgs.log`, `gs/src/cal_wiring.h`):

```
maburgs: CAL start -> ok started power20,cap20,power40,cap40
maburgs: calibration started -> /media/dvr/log/cal/cal.log sweeps=[power20,cap20,power40,cap40] ch=112 (auto: moved to 112)
maburgs: calibration apply 1/4 power20: ok
maburgs: calibration sweep 1/4 power20 done
…
maburgs: calibration done
```

An adopted apply with a note prints it in place of `ok` (`calibration
apply 3/4 power40: ok; mcs6 kept from the 20 MHz table`). Then
`calibration failed: <why>` / `calibration aborted: <why>`, and the
channel pick's own `maburgs channel: calibration …` lines
(`docs/channel-select.md`). The start line is printed when the run first
commands the drone, after the pick.

**`maburd` log** (`/tmp/mabur.log`): `maburd cal: applied …` (with
` -- <note>` when it kept something) / `apply … refused: …`, `maburd radio: power table <bw> MHz live (mcs5 diff
N)[ (… table)][ for calibration]` on every table switch (a capacity sweep
programs its width's table "for calibration"), the boot wall and ldpc
lines above, and `cal_latch_drop=` on `maburd tx_send:`.

**Sideport** `link.cal{state, sweep, kind, bw, cell}` while a run owns the
air (`await_ack` through `verify`; absent while picking and after the
end): `sweep` and `cell` are `"k/n"` strings as on the status line, `kind`
`power|capacity|verify`. maburtop shows `CAL <kind><bw> <i>/<n>` (e.g.
`CAL power40 312/840`) on its top bar and in the compact header
(`docs/observability.md`).

### `cal.log`

Written whatever `debug_log.enable` says — a calibration is a bounded
trace on a deliberate action, and its only after-the-fact record. With
debug logging on it goes in the session directory
(`<debug_log.dir>/NNNN/cal.log`); off (the shipped GS bundle) in
`<debug_log.dir>/cal/cal.log`, i.e. `/media/dvr/log/cal/cal.log`. The
start line names the resolved path, and `maburcal start` finds it there.

Format `callog 4` (`gs/src/cal_log.h`; `gs/bundle/maburcal` is the
in-repo reader and changes with it):

```
callog 4                                              # once per file
R <nonce> <margin_db> <ch> <auto|pinned|onecard|stayed> <ldpc 0|1>   # once per run
S <seq> <power|capacity> <bw> <frames_per_cell|dwell_ms>             # once per sweep
C <rate> <idx> <expected> <recv0> <recv1> <corrupt> <rssi0> <rssi1>  # power cells
W <rate> <wall> <floor> <best_card> <flags>                          # per rate, power
P <rate> <offered> <recv_best> <air_bits_best> <eff_x1000> <blocked_pct> <flags>  # per rate, capacity
A <bw> <0|1> <reason to end of line>                                 # per apply
V <rate> <idx> <pct>                                                 # per verify cell
```

`S` scopes the rows after it to its width until the next `S`; a verify
pass writes no `S` of its own (its `V` rows follow that sweep's `A`). The
`R` line is written at the run's first ack, so a run that failed before
any ack has none; `ldpc` is the drone's announced stamp flag (the GS has
no LDPC setting of its own). Every index is signed and relative to the
drone's anchor; a card with no RSSI in a cell writes `-999`; an
undetermined rate writes `-128` for wall and floor. `P`'s `eff_x1000` is
the measured value even for a `tx_bound` point. An ok `A` row's reason is
empty or the drone's note. Flags bitmask: 1
`no_dip`, 2 `undetermined`, 4 `narrow`, 8 `saturated`, 16
`card_disagree`, 64 `tx_bound`, 128 `lossy` (32 was `drift`, retired and
never reused). A `V` row means "this rate was parked", never "verified
clean": `pct` 0 is a real measurement.

One file holds several runs (a retry after moving the drone lands in the
same file), each delimited by its own `R`. A file in an older format is
renamed `cal.log.callog<N>` when `maburgs` starts, so one file never
mixes formats. `maburcal report` reads `callog 1`-`4`: 1 and 2
carry absolute indices and a `base_ref` on `R` (`-1` = undetermined), 3 is
the single-width relative format (`docs/data-provenance.md`).

## Offline analysis: `maburcal report`

```sh
maburcal report /media/dvr/log/cal/cal.log
```

re-renders every run in a saved file — the same table `start` prints,
straight from the log, no daemon involved. Nothing else parses `cal.log`.

## Hardware acceptance checklist

Work through this on the bench pair after any change to the kit. A change
to the dialect, the stamp or the RC wire is a two-device flag day: deploy
both binaries (and the drone config first, if a key changed) before
judging anything (`docs/deploy.md`). Record each result in a dated
findings page.

| gate | command | expected |
|---|---|---|
| ausniff before/after | `python3 tools/bench/ausniff.py` on the GS (scp the repo copy to `/tmp` first) | ~fps of the mode, 0 gaps, both runs |
| full run | `maburcal start` on a 40 MHz GS | four sweeps, `done`, wall-clock < 120 s; both width blocks in the table |
| walls | the table vs the spike's reference at the same geometry | within ±2 on mcs4–7, mcs0–3 `no_dip` |
| capacity | `eff` per width vs the spike's low-MCS points | within ±0.03 at mcs0–4; high points plausible, or flagged `tx_bound` and left at the existing value |
| subset | `maburcal start power20` | one sweep, `done` < 30 s, `_40` lines untouched (`diff` against `.pre-cal`) |
| 20 MHz GS | temporarily `radio.width = 20` and an all-20 ladder on the GS | `maburcal start` runs two sweeps; `maburcal start power40` → `err refused: GS radio.width is 20` |
| table swap | `link.static_mcs` pinned at 20 then at 40 (or watch rung changes) | `maburd` log `power table 40 MHz live`; `GetTxPowerState` read-back differs between widths |
| config | `/etc/mabur.toml` diff vs `.pre-cal` | only the per-width keys above plus `power_mode`; comments intact |
| legacy | `legacy_wall_rel` after a `power20` | equals the 20 MHz mcs0 wall |
| interruption | kill `maburgs` mid-sweep | drone back to video within `await_next_ms`; config byte-identical |
| abort | `maburcal abort` mid-sweep | video back within 2 s; nothing written |
| re-link | after every run | `SESSION` again within a couple of beacons, IDR at the join |
| dirty op | start with a jammer or busy air on op | `H relocate … cal` in `scan.log`; the run on the quiet member |
| tunnel lane | the drone's `stats:`/`tx_send:` lines and the sideport during a run | `mav … reject=` flat, `drone.auth_reject` false, `link.mav.reject` flat, `cal_latch_drop=0` |
| parameter set during a run | `mavparam.py set ENC_BITRATE_MAX …` mid-run | `FAILED`, "calibration in progress" |

`sat_rssi_dbm` (−45) is inherited from the old bench tool's precondition;
the spike saw card 1 sit at the −45 readout rail at close geometry. Worth
checking on a normal-geometry run that `saturated` stays off, and that it
fires when the drone is moved within a metre of the GS.
