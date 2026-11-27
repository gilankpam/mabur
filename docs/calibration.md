# TX-power walls and air capacity — `maburcal`

Two per-unit hardware facts of the vtx are measured, by different means:

- the **walls**: the TXAGC index at which each MCS's PA starts to
  compress. The **drone measures these itself**, on one chain listening to
  the other, per channel and width, and keeps the curves in
  `/etc/mabur.walls`. No receiver is involved; the GS only triggers it.
- the **efficiency**: the share of the nominal PHY rate the air really
  delivers at each MCS (`air_clock.efficiency_20` / `_40`), which the
  drone's bitrate formula and air clock price against
  (`docs/airtime-model.md` §7). The GS measures it with a capacity sweep
  and the drone writes it into `/etc/mabur.toml`.

The walls are flown only under `radio.power_mode = "offset"`, which the
operator sets; nothing switches it. A power sweep's time per (channel,
width) is set by its capture count and the drone's per-capture time,
measured per width on the bench and logged by each sweep (`time_ms`,
`captures`, below); it never exceeds the 20 s deadline
(`cal::kWallDeadlineMs`, shared by both ends). A capacity sweep takes about
18 s per width, at the power the drone flies for the link's channel.

```
$ ssh root@<gs>
# maburcal start                 # everything the GS's ladder needs
# maburcal start power20 cap40   # a subset, any order; the GS sorts it
# maburcal start power20@112     # measure channel 112 at 20 MHz; the pair stays put
# maburcal status
# maburcal abort
# maburcal report /media/dvr/log/cal/cal.log
```

No toolchain, no repo checkout, no laptop-side step. Why the capacity
sweep has the shape it has (a 2 s dwell):
`docs/findings/2026-10-31-calibration-shortening-spike.md`. How the walls
measurement was found to work:
`docs/findings/2026-11-26-drone-self-calibration-cross-path.md`.
Earlier measurements that are still hardware fact — the comb past the
first dip, the transfer curve:
`docs/findings/2026-07-16-txagc-calibration.md`,
`docs/findings/2026-09-11-maburcal-bench.md`.

## Running it

`maburcal` ([`gs/bundle/maburcal`](https://github.com/gilankpam/mabur-ground/blob/master/gs/bundle/maburcal), installed to `/usr/bin` by
[`gs/bundle/install.sh`](https://github.com/gilankpam/mabur-ground/blob/master/gs/bundle/install.sh)) speaks one UDP datagram at a time to `maburgs`'
`CalControl` listener on `127.0.0.1:8400`, loopback only: you have to be
logged into the GS to start a run.

**What a run measures.** A run is a list of sweeps, each a kind (`power`
or `cap`) at a width (20 or 40). `power` is the drone's wall
self-measurement; on the wire it is the Wall kind. `maburcal start` with no
arguments runs the GS's own composition (`default_sweeps`, [`gs/src/cal_control.h`](https://github.com/gilankpam/mabur-ground/blob/master/gs/src/cal_control.h)):
`power20, cap20`, plus `power40, cap40` when the GS receiver width
(derived from the ladder, `docs/bw40.md`) is 40
**and** some `[[link.ladder]]` rung has `bw = 40`. The width is the
narrower of the GS receiver width and the linked drone's Telem
`width`, so a 40 MHz GS linked to a drone flying 20 composes the
20 MHz pair only. Tokens:

| token | sweep |
|---|---|
| `power20`, `power40` | the drone measures the walls of the link's channel at that width |
| `power20@<ch>`, `power40@<ch>` | the same for channel `<ch>` (decimal, 1..177); the pair does not move |
| `cap20`, `cap40` | the capacity sweep at that width |

Duplicates drop; two `power` tokens at one width naming different channels
both run. Whatever was typed, the session runs widths ascending and,
within a width, **power before capacity** (channels ascending): capacity
is measured at the walls a preceding power sweep just stored for the link's channel. The reply
names the run as it will go: `ok started power20,cap20,power40,cap40`.

Refusals come back at once, before anything airs:

| reply | why |
|---|---|
| `err bad token: <tok>` | not one of the tokens above (`cap20@112`, `power20@0`, `power20@` are bad) |
| `err refused: GS receiver width is 20 (load a 40 MHz ladder to calibrate 40 MHz)` | a `*40` token while the effective width (above) is 20 |
| `err a calibration session is already running` | a run, or its channel pick, is in progress |
| `err refused: link is down` | `maburgs` is not in `SESSION` |
| `err refused: waiting for drone telemetry` | `maburgs` is in `SESSION` but no Telem has arrived since the ack, so the drone's caps are not known yet (about the first second of a session); retry |
| `err refused: peer does not advertise CAP_CALIBRATE` | the drone's Telem `chip_caps` lacks the bit |

The "GS receiver width is 20" wording is also what a 40 MHz GS says when it
is the linked drone that flies 20.

**Progress.** `start` then polls `status` every 500 ms and prints each
reply, until the state is `done`, `failed` or `idle` (aborted):

```
ok state=sweep sweep=2/4 kind=capacity bw=20 cell=3/8
```

`state` is one of `idle picking await_ack sweep await_done analyze apply
done failed`; `sweep=k/n` the running sweep, 1-based; `kind` is `wall` or
`capacity`; `cell=i/n` the point of the latest capacity frame the GS
tallied (8 capacity points); for a wall sweep it is `<s>/20`, the seconds
since the ack against the drone's 20 s deadline.
`mismatch=<n>` appears once frames have been dropped for a kind, width or
flags that do not match the running sweep (see "The sweep frame"). A
`failed` line ends with `fail_reason=<why>` (free text to the end of the
line). Then `start` finds the run's `cal.log` and prints the final report
(below). `maburcal abort` cancels a run in any state.

**Run it with `[radio] relays = []`.** The session tallies only cards 0
and 1, and relay cards come after the USB cards in the roster: on a GS
with one USB card the relay *is* card 1, and its capacity frames (another
antenna, ath9k RSSI) would enter the tally. `[mavlink] enable` is not
needed: the tunnel calibration rides is always built; the UDP endpoint is
a separate tenant (`docs/mavlink.md`).

### The pair must already be linked and flying video

There is no other way to reach the drone: every calibration message rides
the MAVLink tunnel inside the authenticated RC link. If the link is down,
fix that first — calibration cannot bring it up.

## What a run does

### 1. Channel pick

Before the first sweep the GS puts the pair on the quietest member of
`radio.channels` (`CalPicker`, [`gs/src/cal_picker.h`](https://github.com/gilankpam/mabur-ground/blob/master/gs/src/cal_picker.h), over `ChannelCore`):

- **Auto, two or more cards** (`channel = "auto"`): one fresh measure
  round by the boot scout — every member once (one dwell per pair at
  40 MHz), op's own pair skipped (it carries the drone's video); op is
  read from the verdict's live window instead. Both sides are scored on
  the terms that mean the same thing on either instrument: decoded
  foreign frames per `radio.scan.dwell_ms` plus NHM foreign busy %. A
  candidate is one reading — its own dwell, at 40 MHz one dwell on the
  pair — and a candidate without a busy reading is not ranked while any
  has one; leaving op takes a better tier (an unblocked candidate against
  a blocked op moves at once; a blocked candidate never replaces an
  unblocked op), or — once both are in the same tier — beating op's busy
  % by `radio.scan.pick_margin_pct` points; op unread means stay. When the
  best member is not op, the GS orders the
  boot pick's relocate (scan.log `H relocate … cal`, no verify window —
  the confirm lands it) and waits for it to settle. Detail:
  [`docs/channel-select.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/channel-select.md) "Calibration pick".
- **Pinned, or one card** (no spare card, or a relay-only roster): no
  measurement; the run uses op.

The whole pick is bounded by `pick_timeout_ms` (8 s); the round itself
by twice a full round's dwell time (2.2 s at the defaults). A relocate
that is refused (hop cap, a hop in flight) or does not confirm leaves the
pair on op and the run proceeds there. The outcome is the `pick` token on
`cal.log`'s `R` line (`auto|pinned|onecard|stayed`) and the note on the
start line (`auto: 112 is the best member`, `auto: moved to 112`, `stayed
on 64, relocate unconfirmed`, `stayed on 64, relocate refused`, `stayed
on 64, measure timed out`, `pinned`, `onecard`). The remembered-channel
store follows a calibration relocate like any hop.

### 2. Per sweep

1. **Command.** The GS sends `MABUR_CAL_SWEEP{nonce, seq, kind, bw, channel,
   …}` every 200 ms until acked; 3 s without an ack fails the run
   (`calibration ack timeout`). `seq` is the sweep's position in the run.
   `channel` names the channel a Wall sweep measures; 0 means the link's
   channel, which the drone resolves to the channel it flies (its
   SWEEP_DONE names that channel). `maburgs` never sends 0: it fills the
   channel in itself (the picker's op, else the session's channel; none
   known fails the run, `no link channel known`). The drone
   is idempotent by (nonce, seq): a repeat of the running seq re-acks
   without touching the sweep, an older seq is ignored, a newer one starts
   the next sweep. One hold: a SWEEP is not sent while a scout dwell begun
   just before the start still has a card off-channel (`cal_cmd_clear`,
   [`gs/src/hop_burst_gate.h`](https://github.com/gilankpam/mabur-ground/blob/master/gs/src/hop_burst_gate.h)).
2. **Ack.** `MABUR_CAL_ACK{nonce, seq, result, start_in_ms, flags}`. A
   Capacity sweep starts 200 ms after it is accepted, so the ack can land
   first; `flags` is the stamp mask it flies (`ldpc`, `stbc`), and the GS
   tallies only frames carrying it. A CRC-clean frame of the commanded
   kind and width stands in for a lost ack of a capacity sweep; a wall
   sweep's ack may be lost the same way, and then its **report stands in
   for it** (nothing airs during the measurement, so the report is the
   first thing the GS can hear; it carries no stamp flags, so the run's
   `ldpc` in `cal.log` is `-` if no ack was ever heard). A wall sweep whose
   ack timeout (3 s) passes without any ack does not fail: the GS goes
   silent for the window the last SWEEP copy opens and waits for the
   report (`no ack and no wall report from the drone` if none comes). A
   capacity sweep still fails the run at the ack timeout. Refusals:

   | result | meaning | the GS |
   |---|---|---|
   | `anchor_unreadable` | the chip would not zero its rate diffs or report its TXAGC reference (checked when a run's first sweep is accepted; a wall sweep's own anchor read failing later ends it `device_error`) | fails the run |
   | `busy` | reserved in the enum, never sent by the drone | — |
   | `unsupported` | a Wall sweep on a chip that cannot self-measure (anything but the 8812EU), or a sweep of a kind the drone does not know (neither wall nor capacity) | skips the sweep with a note (`X` line in `cal.log`) and goes on |
   | `bad_channel` | a Wall channel not in the drone's `radio.channels`, or on 2.4 GHz (the capture is validated at 5 GHz only) | fails the run, `channel not in the drone's set` |
   | `bad_width` | a 40 MHz Wall sweep on a drone with `radio.width = 20` | fails the run, `width not flown by the drone` |

3. **Silence.** From the ack the GS sends nothing — no RCF, no DISC
   keepalive, no live MAVLink — for the sweep's planned duration plus
   `phase_slack_ms` (1 s) (`CalSession::radio_silent()`). A Wall sweep's
   planned duration is its 20 s deadline. The drone is silent too during a
   Wall measurement (below).
4. **Done.** `MABUR_CAL_SWEEP_DONE{nonce, seq, …}`. For a Capacity sweep it
   goes out when the last frame is out and carries `offered[8]`,
   `blocked_pct[8]`. For a Wall sweep it carries `wall_result`, `channel`,
   `bw`, the eight `walls`, `floor_evm_x10` and `anchor`. The drone repeats
   it 100 ms later and every 200 ms after that until the run's next
   command arrives or the run closes. The first copy ends the GS's silence
   early, and a Wall sweep's `Ok` report goes straight to the next sweep;
   duplicates are ignored, and so is a late Wall report arriving during
   the following sweep. A SWEEP repeat of a sweep the drone has already
   finished (the ack, or the whole sweep, outran the GS) is answered with
   the stored report again on its repeat schedule, not a fresh ack. Without
   any copy the GS's window closes at planned end + 1 s, and a Wall sweep
   then fails the run (`no wall report from the drone`).
5. **Result.** A Wall sweep with a result other than `ok` fails the run,
   `wall measurement failed: <code>` (codes below). Capacity: one
   efficiency per rate (below), then:
6. **Apply** (capacity only). `MABUR_CAL_APPLY{nonce, seq, bw, eff[8]}`
   every 200 ms until `MABUR_CAL_APPLIED{nonce, seq, bw, ok, reason}`; 5 s
   without an answer fails the run (`calibration apply timeout`), a
   refusal fails it with the drone's reason. `seq` names the sweep the
   apply belongs to: the drone answers a repeat of an apply it already made
   from its cache instead of applying again. A sweep with nothing to write
   (every capacity point `tx_bound` or `lost`) sends no apply. A Wall
   sweep has no apply: the drone stores the measurement itself.

Between sweeps the GS is not silent: RCF and the DISC keepalive go out as
usual while it waits for an ack or an APPLIED.

### 3. The wall measurement

The drone measures one PA's transmit quality directly. PA A transmits a
QPSK (MCS1) probe frame; chain B, its antenna transfer switch parked so it
hears the antenna side of the front end, captures the signal through the
chip's logic-analyzer mode, and `cross_evm`
([`drone/src/cross_evm.h`](../drone/src/cross_evm.h)) demodulates the frame and returns its error
vector magnitude (EVM, dB; lower is cleaner) at the TXAGC index under test.
Evidence and the register-level recipe: the cross-path finding above. PA B
is never measured: PA A's curve sets the walls for both.

The work is done by `WallSweep` ([`drone/src/wall_sweep.h`](../drone/src/wall_sweep.h)) on the
TX writer thread, against a capture session on the radio
(`DeviceCaptureSession`, [`drone/src/maburd/wall_ctl.h`](../drone/src/maburd/wall_ctl.h)):

1. **Setup.** Under the retune gate held exclusive the drone retunes to the
   commanded channel if it is not the link's, switches to 20 MHz when the
   request is 20 and the radio is at 40, clears any flat power override,
   zeroes the rate diffs, re-applies TX power for the channel and reads the
   **anchor**: the chip's MCS7 TXAGC index with the diffs zeroed. A chip
   that will not zero its diffs or report the anchor ends the sweep
   `device_error`. 40 MHz is measured on the link's standard pair (every
   member of `radio.channels` shares one at 40, so the measured channel
   lands on it).
2. **Park check, first.** One capture at index 28 and the weakest listener
   gain with the switch unparked, one after parking it. The parked level
   (envelope) must be at least 10 dB above the unparked one, or the sweep
   ends `unparked` (as it does when the park itself does not take or the
   rise is not finite, an all-zero capture): an unparked listener passes every other rule while
   reading PA A about 3 dB too good, the overdriving direction. The check
   is relative on purpose, since the absolute parked level differs between
   channels and airframes. A park-check capture holding no TX at all (no
   frame at a level of 30 dB or more, envelope under 30 dB: the
   demodulator can read a "frame" out of an empty capture's noise) says
   nothing about the park and is retried, up to twice.
3. **Points.** Absolute TXAGC index 28 to 124 in steps of 4, ascending: 25
   points. A point is the median of up to three captures that held a
   clean, unclipped frame at a level of at least 30 dB; a capture without
   one is retried, at most twice per point. A point may end with no value.
4. **Gain.** Four listener gain words, strongest first. The sweep starts at
   the strongest and only steps down: a clipped capture, or a listener
   level above 57 dB (where the listener's own distortion adds), restarts
   that index one word weaker. A clipped capture is never used for a point,
   and clipping at the weakest word leaves the point empty.
5. **Restore, on every exit path** (done, invalid, abort, deadline, device
   error): the capture session ends (switch route, listener chain and TX
   buffer layout back to flight's), then width, channel and TX power. If
   the session's own restore fails the result is `device_error` and nothing
   is stored; the radio's state is then unknown, so `maburd` keeps
   repeating the report for 1 s, logs `maburd cal: ERROR: the wall
   measurement could not restore the radio; exiting for wrapper respawn …`
   and exits, and its wrapper starts it again with a fresh bring-up (the
   link drops for that restart). The same happens when the measurement is
   aborted or cut off and its restore fails, with no report.

**The drone sends nothing during the measurement** (`radio_quiet`): from
the setup to the restore every send is dropped, so no DISC_ACK, no
re-ack of a repeated SWEEP, no Telem, for up to the 20 s deadline. The GS is
silent for the same window. The deadline, not an abort, bounds a run: an
abort is only checked between captures, and the drone is off the link's
channel or has a chain detached while it measures.

**Validity.** A result is stored only when the measurement runs to its
end and the curve passes the checks; every other result stores nothing.
The results other than `ok` (the first four are checked in the order
listed):

| result code | condition |
|---|---|
| `unparked` | the park check failed (above) |
| `too_few_points` | fewer than 20 of the 25 points have a value |
| `floor_too_high` | the best point is above −26 dB |
| `not_saturated` | the last point is below −11 dB (the PA was never driven into compression) |
| `deadline` | the 20 s measurement deadline ran out |
| `device_error` | the radio refused a step, or its restore failed |
| `aborted` | the run was cancelled mid-measurement |

`ok` is the only result that stores anything: a failed or doubtful
measurement never replaces a stored one and is never flown.

**20 MHz tuning.** At `radio.width = 40` the drone airs 20 MHz rungs as
20 MHz frames on the 40 MHz tuning, but the 20 MHz curve is measured with
the radio switched to a 20 MHz tuning (`FastSetBandwidth`), the case the
measurement was validated on. The two widths' curves agree within about one
index; the difference between those tunings is not measured.

**The probe is one PA at low duty;** flight is two PAs at high duty, and a
hot PA reads about 1 dB worse than the probe. `radio.wall_margin_db`
covers it.

### 4. Walls from a curve

`walls_from_curve` ([`drone/src/wall_curve.h`](../drone/src/wall_curve.h)): for each MCS, the crossing is
the linearly interpolated TXAGC index at which the curve first rises
through that MCS's threshold, searching upward from the floor (the first
lowest point). `wall = floor(crossing − anchor)`, clamped to [−64, 63];
no crossing means 63. Walls are relative to the anchor, as before: the
chip adds its own per-channel-group reference on every channel.

| MCS | 0–2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|
| threshold, EVM dB (LDPC on, STBC on) | none (63) | −9.0 | −11.0 | −15.0 | −15.5 | −17.0 |

The thresholds are one reference unit's PA A EVM at that unit's walls
(MCS7 spread −13.9 to −18.1, MCS4 −10.0 to −11.2 across three channels and
both widths). **With `radio.ldpc = false` every threshold is lowered by
4 dB** (the BCC versus LDPC+STBC difference at MCS7), and the
measurement's log line says so. The walls are derived from the stored curve
at load and whenever `radio.ldpc` or the margin changes, so neither needs
a re-measure and a later correction of the thresholds does not either.
Against a receiver, the MCS7 LDPC+STBC wall lies within 3.5 steps of the
−17 dB crossing on the channels checked; on one channel the receiver's
walls sit 5–8 steps above the curve, the self-measurement reading low, the
safe side.

### 5. The capacity sweep

Eight points (MCS 0–7) at the sweep's width, each a **300 ms settle then a
2 s dwell** of back-to-back frames (18.4 s), at the drone's production
power for that width (the link channel's plan live, no index override —
the walls a preceding power sweep just stored, or the chip's own table
when that channel has none) and in the production A-MPDU mode
(`ampdu_mode_for(ampdu, mcs, bw)`). Frames are the stamp padded to the
video body size at the configured geometry, `13 + blocks_per_body[1] ·
(16 + symbol_size[1])` (1405 B at the default `[fec]`), sent as stream 1
through `RadioTx::send_bodies` in batches of three, so every
`radio.tx_threads` sender stays busy. The drone counts frames `offered`
and `blocked_pct`, the share of the dwell its submitter spent waiting for
the device (the `UsbTxPool`'s space-wait counter; with `tx_threads = 1`
the time inside the synchronous send).

The GS sums each CRC-clean frame's air size — the 26-byte QoS-Data
header, the payload and the 4-byte FCS, counted once whether or not the
received body still carries it (`route_cal_body` sizes from the payload
length `parse_cal_payload` matched) — per card, with the receive time of
each card's first and last clean frame of the point (the RX thread's
stamp, `m.mono_us`, not the core loop's tick). On the best card (most
bits) it computes

```
eff = (delivered bits − first frame's bits) / (span_s × nominal(mcs, bw))
span_s = last frame's receive time − first frame's receive time
```

with `nominal` the HT long-GI PHY rate (`rc::phy_rate_mbps`), rounded to
0.01 and clamped to [0.05, 1.0] (`analyze_capacity`). The point is timed
by the GS's receive clock, not the commanded dwell, because the drone's
blast only *enqueues* for the dwell: its TX path (the `UsbTxPool` and the
device's own queue) keeps airing after the blast returns, so the air is
busy for longer than 2 s and delivered bits over the dwell read high
(past 1.0 at mcs0). Over the receive span the air was
saturated throughout. n frames span n − 1 inter-arrival gaps, so the
first frame's bits — on the air before its stamp — are left out; A-MPDU
bursts and USB receive batching blur either end by a few ms, under 0.5 %
of a 2 s span (only the first MPDU of the first A-MPDU is left out, about
0.1-0.2 % high). A best card that heard fewer than 20 clean frames, or over
less than 500 ms, cannot be timed: its eff falls back to bits over the
commanded dwell and the point is flagged `short_span` (written as usual;
such a point is nearly always `lost` too).

The drone's SweepDone goes out on the control path the moment its last
blast returns, ahead of the frames still queued behind it, so the GS
does not close a capacity sweep at the SweepDone: it keeps tallying until
no frame of the sweep has reached it for 50 ms (`tail_quiet_ms`) of its
own processing time — measured from the core loop's tick after the pass
that handled the last one, not from that frame's receive stamp, so a slow
loop pass with the drain still queued behind it does not read as quiet —
within the usual listen window. Closing at the SweepDone would drop the last point's
tail — reading as loss on that point — and count whatever arrived after
the next sweep began as `mismatch=`. A point whose
sender was blocked less than 50 % of the dwell measured the drone's CPU or
USB, not the air: it is flagged `tx_bound` and **not written** (the
config keeps its value, as an undetermined wall does). A point that
delivered under 90 % of what was offered is flagged `lossy` and is
written — delivered is delivered — but the parked power or the geometry
is suspect. Under 50 % it is flagged `lost` as well and is **not
written**: a dwell that lost over half its frames measured the loss, not
the air. `lossy` and `lost` compare the best card's whole count with `offered`;
with the tail listened through, every frame the drone handed its TX path
had its chance to arrive. A capacity sweep whose SweepDone never arrives
writes nothing: without `offered` and `blocked_pct` a `tx_bound` point
cannot be told apart.

Why delivered bits and not a fit: the air clock and the bitrate formula
price delivered capacity, and the bench numbers the defaults carry were
defined as exactly this ratio, so a measured table replaces them like for
like.

### 6. The end of a run

After the last sweep the session is `done`, and the GS sends
`MABUR_CAL_ABORT` three times, 200 ms apart: the drone closes the run at
once — restoring the current rung's power plan, clearing the override and
resuming video — instead of waiting out its 15 s `await_next_ms`. The
abort keeps everything the applies wrote. `maburcal abort` and a failure
send the same three Aborts, if a SWEEP ever went out.

The drone needs none of it to get home: every state of its sweep engine
(`CalSweep`, `drone/src/cal_sweep.h`) has a deadline — `await_next_ms`
(15 s) for the next command after any sweep or apply, and
`hard_cap_ms` (180 s) for the whole run from the first accepted SWEEP.
Killing `maburgs` mid-run leaves the drone to time out, restore its power
and resume video on its own.

### Timing

| step | time |
|---|---|
| channel pick | 0 pinned or one card; ~1 s for the round (three members at 280 ms) plus up to `[hop] confirm_ms` for a relocate |
| wall measurement | captures × the per-capture time, plus the channel and width switches (each sweep logs `time_ms` and `captures`); never more than 20 s |
| capacity sweep | 18.4 s per width (8 × 2.3 s) + 200 ms start + ack |
| apply (capacity) | one round trip |

The whole run is bounded by the drone's 180 s `hard_cap_ms`.

## The drone during a run

**Video stops for the whole run.** From the first accepted SWEEP until
the run closes (`cal_active`) the drone queues no video and drains what
was queued; Telem and the MAV component's own uplink pause while a
capacity sweep airs (`cal_sweeping`) and go out between sweeps. Its
calibration replies go out whenever they are due, except during a wall
measurement, which sends nothing at all (above). Video resumes when the
run closes.

**It will be in `RENDEZVOUS` by the end of a long run.** Every sweep
outlasts `link.failsafe_ms` (3 s), so the drone's `RcAgent` ages out of
`LINKED` inside every sweep, and `link.rendezvous_ms` (30 s) later is in
`RENDEZVOUS`. It may re-link in a gap between sweeps or stay out until the
run ends; either is benign. `set_ladder`, the A-MPDU switch, retunes and
the TX-power table are all gated on `cal_active` (a retune ordered
meanwhile is deferred and replayed after), so the agent cannot fight the
sweep for the radio. The operator's only job is to **confirm the pair
re-links after each run**: video back within a couple of DISC beacons,
with an IDR at the join, and on the link's channel and width after a
`power<bw>@<ch>` run. If it does not, that is the ordinary stale-caps
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

Capacity frames only; a wall measurement airs no stamped frame.
`common/include/mabur/cal_wire.h`, a 12-byte header:

```
magic "MCAL" | u8 kind (3 capacity) | u8 rate | i8 idx |
u8 bw (20/40) | u8 flags (bit0 ldpc, bit1 stbc) | u8 seq_hi | u16 seq (LE)
```

then fill `(0x5A ^ idx)` to the frame length (the video body size).
`seq` is a 24-bit running frame counter. The parser refuses any kind but
capacity, a rate over 7, an index outside [−64, 63], a width other than
20/40, unknown flag bits, or a fill that does not match (with or without
a trailing FCS): `maburgs` keeps CRC-bad frames, and the fill check is
what stops a corrupted frame being attributed to the wrong point. A frame
whose kind, width or flags do not match the running sweep is counted
(`stamp_mismatch`, the status line's `mismatch=`) and never tallied.

## The cache: `/etc/mabur.walls`

One line per (channel, width), holding the measured curve and the anchor,
not the walls. A text file beside `/etc/mabur.channel`:

```
maburwalls 1
<ch> <bw> <anchor> <e0> <e1> … <e24>
```

`<ch>` is 1..177, `<bw>` 20 or 40, `<anchor>` 0..127, then 25 values, one per
TXAGC index 28, 32, … 124: the point's EVM in dB × 10 (an integer, −600..0),
or `x` for a point with no value. Entries are in channel, then width,
order.

- Written only after an `ok` measurement, replacing that channel and
  width's line: the whole file to `<path>.tmp`, fsync, atomic rename. A
  failed write leaves the entry live until `maburd` restarts and logs
  `maburd cal: wall cache … write … failed`; the result stays `ok`.
- Read at `maburd` start: `maburd radio: wall cache /etc/mabur.walls: N
  measured (channel, width)`. A missing, unreadable, malformed or
  wrong-version file is "nothing measured", with the reason in the same
  log line; it is never a boot failure.
- **Hand-editing:** LF line ends, single spaces between tokens, a final
  newline, a header line exactly `maburwalls 1`. Any malformed line (wrong
  token count, a value out of range, a duplicate (channel, width), no
  trailing newline) discards the **whole** file: "nothing measured". Over
  64 KB is rejected the same way (a full file of 354 entries is about
  50 KB).
- An entry whose curve fails the measurement's validity checks (fewer
  than 20 values, best point above −26 dB, last point below −11 dB; see
  "Validity") parses but is never flown: that (channel, width) flies as
  unmeasured, and `maburd` logs `maburd radio: wall cache ch <ch> <bw>
  MHz: curve fails its checks, not flown` at start.
- Entries for channels no longer in `radio.channels` are kept and unused.
- A wipe of the drone loses the cache: re-measure, never restore.

## The power table follows channel and width

`power_mode` (`docs/deploy.md`):

- `"none"` (the default): the chip's own table is flown; measurements are
  stored but never programmed.
- `"offset"`: the drone flies the measured walls. **The operator sets it;
  nothing switches it automatically**, not a measurement and not a
  `maburcal` run. `"offset"` with an empty cache is legal and flies the
  chip's own table until something is measured.

The chip has one per-rate diff table with no channel or width axis, so
`maburd` keeps one plan per (channel, width) in the cache
(`make_channel_plans`, [`drone/src/power_tables.h`](../drone/src/power_tables.h)) and programs the live
one (`LivePowerTable`, [`drone/src/live_power_table.h`](../drone/src/live_power_table.h)) at bring-up,
on every rung width change and on every retune, inside the retune's gate
hold after the TX power re-apply. A plan's walls come from the stored curve
under the current `radio.ldpc`; the plan is built from the eight walls
with `radio.wall_margin_db` and `radio.legacy_wall_rel` as the power plan
always was (`park = clamp(wall − round(wall_margin_db · 4), −64, 63)`).

**Lookup for (channel, width):**

1. its own entry;
2. else the same channel's other width, logged (the two widths' curves
   agree within about one index);
3. else the chip's own table, logged once per channel and width.

An unmeasured channel is never flown on another channel's walls. Each
switch logs `maburd radio: power table ch <ch> <bw> MHz: measured plan
(mcs5 diff N)`, with `the 20 MHz plan` / `the 40 MHz plan` in place of
`measured plan` when it flies the other width's, or `maburd radio: power
table ch <ch> <bw> MHz: chip's own table (no plan)`; ` for calibration` is
appended for a table a capacity sweep programs. Control frames are 20 MHz
MCS0 whatever the rung and ride the legacy diff in every plan.

A `RADIO_WALL_MARGN` or `RADIO_LDPC` set re-derives every plan and writes
the live one (refused "calibration in progress" during a run). A measurement
made during a run rebuilds the plans at once but does not program them
(calibration owns the table): the run's capacity sweeps and its restore
do. Only the 8812EU can self-measure; any other drone card answers
`unsupported` and flies `"none"` (`docs/deploy.md` "Supported drone cards").

`legacy_wall_rel` (control and telemetry: legacy/CCK) is one number for
every plan, in the config. Legacy 6 Mb/s and HT MCS0 are BPSK 1/2 on the
same OFDM waveform; it is a physical assumption, re-checked by the
acceptance checklist.


## The anchor

Walls are signed indices **relative to the chip's own TXAGC anchor**. The
anchor is the per-channel-group reference devourer programs from the
module's efuse on every channel set, and the chip adds it itself. A
measurement reads it on the channel it measures and stores it with the
curve; the walls a plan flies come from `crossing − anchor` of that
channel's own entry, so a channel's walls are its own, not another's
shifted. `maburd` re-applies TX power (`ReApplyTxPower()`) after every
retune, so the diffs sit on the anchor of the channel the link is on. The
anchor is in `/etc/mabur.walls` and `SWEEP_DONE`, nowhere else.

## Config keys

```toml
[radio]
power_mode      = "none"      # "offset" flies the measured walls
legacy_wall_rel = 63
wall_margin_db  = 1.0

[air_clock]
efficiency_20 = [0.93, 0.88, 0.84, 0.80, 0.76, 0.76, 0.78, 0.76]
efficiency_40 = [0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75]
```

- **No wall keys.** The per-width wall tables and their coding-flag keys
  that earlier builds kept in this file do not exist: a config that still
  has one fails boot as an unknown key (`docs/deploy.md`); delete the
  line. The curves live in `/etc/mabur.walls`.
- **`wall_margin_db`** is the operator's safety choice, set on the drone
  (`RADIO_WALL_MARGN` or by hand) and never written by the kit; a change
  re-derives every plan.
- **`ldpc`** shifts the thresholds by 4 dB (above). A `RADIO_LDPC` set
  re-derives every plan, so a drone that flew LDPC and now sets
  `radio.ldpc = false` flies lower walls from the same curves.
- **The efficiency tables** are read at `maburd` start only: a run's new
  `efficiency_<bw>` takes effect at the next `maburd` start.

## What gets written, and how to roll back

Walls are written by the drone alone, to `/etc/mabur.walls` (above); there
is no backup of it, and a wrong entry is replaced by measuring that channel
and width again, or removed by deleting its line.

A capacity apply is line-surgical in `/etc/mabur.toml`
(`drone/src/cal_apply.h`): `air_clock.efficiency_<bw>` only. A capacity
point sent as 0 (`tx_bound` or `lost`) leaves that entry as it is. A key
the file lacks — a first 40 MHz calibration on a file with only
`efficiency_20` — is **inserted** next to the other width's line (or after
`shed_ms`), the one place the patcher adds a line; a left rate in an
inserted line takes what the loader flies for that width with the key
absent (`AirClockCfg`'s default, nominal 1.0), so the insert changes
nothing the drone flew for that rate. Every comment and every other key
survive byte for byte.

Before anything is written the candidate is validated with the loader's
own rules (every efficiency in [0.05, 1.0]), patched, written to
`<path>.new`, fsynced and loaded with `mabur::load_config` — the function
`maburd` boots with — and only then published by an atomic rename, under
the config-write lock shared with the MAVLink parameter component (a
parameter set during a run is refused "calibration in progress"). A
config `maburd` cannot load would crash-loop it at 2 s (`docs/deploy.md`);
every refusal leaves the file exactly as it was, and APPLIED carries the
reason. The drone's log: `maburd cal: applied 20 MHz efficiency`, or
`maburd cal: apply 20 MHz refused: <why>`.

**Backup.** The first successful apply of a run copies the file as it was
before the run to `/etc/mabur.toml.pre-cal`; later applies of the same run
do not touch it. **To roll back**, restore it on the drone and restart
`maburd` (from a non-interactive ssh command, start the wrapper under
`setsid` — `docs/deploy.md`):

```sh
cp /etc/mabur.toml.pre-cal /etc/mabur.toml
/etc/init.d/S00mabur restart
```

## Walls and efficiency are re-measured after a wipe, never restored

The drone's flight tuning is `maburd`'s built-in defaults; the shipped
`bundle/mabur.default.toml` holds only per-drone keys. The built-in
efficiency tables are one reference unit's numbers, and they price every
rung on a drone that never calibrated, which is one more reason to
calibrate a new unit before flying it. After a drone wipe or replacement,
`/etc/mabur.walls` is gone: measure every channel the link flies at both
widths (`maburcal start power20@<ch> power40@<ch> …`, one token per
channel and width, then `maburcal start` for capacity), then set
`power_mode = "offset"`. One run is bounded by the drone's 180 s
`hard_cap_ms`, and a wall token can take its whole window, the 20 s
deadline plus the GS's 1 s slack and the 200 ms start: eight wall tokens
fit one run at worst, six beside both capacity sweeps. Measure more
channels over several runs.

## Geometry

The walls measurement needs no receiver, so it does not depend on the
GS's distance or antennas. It does need the drone's own antennas
connected (a detached one reads as a failed park check or a floor above
−26 dB). The capacity sweep still wants a normal bench distance:
close enough to hear the fastest rows, far enough not to compress the GS's
RX front end (the `saturated` flag fires on a peak RSSI above −45 dBm).
Run capacity where the air is quiet (the pick helps).

## Reading the result

`maburcal report` output for a log holding a normal run, a run whose
first record was a wall report (ack lost, `ldpc=-`) and a run on a drone
that cannot self-measure:

```
run nonce=1873455120 margin=1.00dB ch=112 pick=auto ldpc=1
walls ch=112 20 MHz anchor=39 floor=-30.6dB: 63 63 63 63 44 26 24 20
walls ch=112 40 MHz anchor=39 floor=-29.8dB: 63 63 63 63 45 28 28 23
rate     eff20   flags
mcs0       .93
mcs1       .88
mcs2       .84
mcs7         -   20:tx_bound
walls stored by the drone (/etc/mabur.walls)
written: /etc/mabur.toml (backup /etc/mabur.toml.pre-cal)

run nonce=1873455121 margin=1.00dB ch=112 pick=pinned ldpc=-
walls ch=112 20 MHz anchor=39 floor=-30.6dB: 63 63 63 63 44 26 24 20
walls stored by the drone (/etc/mabur.walls)

run nonce=1873455122 margin=1.00dB ch=112 pick=onecard ldpc=1
rate     eff20   flags
mcs0       .93
skipped power20 (sweep 0): unsupported
written: /etc/mabur.toml (backup /etc/mabur.toml.pre-cal)
```

Per wall sweep one line, `walls ch=<ch> <bw> MHz anchor=<a> floor=<dB>dB:`
then the eight walls, MCS0 first (63 = no crossing, the rail). The
capacity table has one column per width swept (`eff20`, `eff40`) and a
`flags` column with a `20:`/`40:` prefix; `-` is a point with no written
number (`tx_bound` or `lost`: measured, kept in `cal.log`, never written).
There are no wall, park or verify columns and no legacy line. Then one
`skipped <token> (sweep <seq>): <reason>` line per skipped sweep, and the
tail: `walls stored by the drone (/etc/mabur.walls)` when the run reported
walls, then the apply line when there was a capacity sweep or an apply
(`written: …`, or `not written: <bw> MHz: <reason>; …`, or `not written:
this run produced no apply attempt`). A run with no walls, no capacity and
no apply prints `not written: this run produced no measurements`; a
walls-only run prints no written line.

### Reading a wall

- **`63`** for MCS0–2 is by design: they have no threshold. For MCS3–7 it
  means the curve never rose through the threshold: the row parks at the
  rail, the top of the 7-bit diff field. A constant, not a measurement.
- **Walls lower than a receiver's** are the safe direction. Measured on a
  channel group where the receiver's walls were 5–8 steps above the curve,
  the cause is not understood.
- **The floor** (`floor_evm_x10` / `floor` on the report) is the best EVM
  of the curve; a healthy PA A reads −26 dB or better, and a floor near the
  limit is the first sign of a bad listener setup or antenna.

### Capacity flags

| Flag | Meaning | What to do |
|---|---|---|
| `tx_bound` | Capacity: the sender was blocked < 50 % of the dwell — the drone's CPU/USB, not the air, was the limit. | Not written; the config keeps its value. Expected at the top MCS if the drone cannot saturate the air there. |
| `lossy` | Capacity: the best card got < 90 % of the frames offered. | Written (unless also `lost`). Interference, or the power is too low for this geometry; re-run on a quieter channel if it fires on low MCS. |
| `lost` | Capacity: the best card got < 50 % of the frames offered (always with `lossy`). | Not written; the config keeps its value and `maburcal report` prints `-`. Re-run on a quieter channel or closer. |
| `short_span` | Capacity: the best card heard < 20 clean frames, or over < 500 ms, too few to time the point by its receive span; `eff` is bits over the commanded dwell, which reads high by the drone's TX-queue drain. | Written. Nearly always comes with `lost` (then not written); alone, re-run closer or on a quieter channel. |

Thresholds are `CalThresholds` in [`gs/src/cal_analysis.h`](https://github.com/gilankpam/mabur-ground/blob/master/gs/src/cal_analysis.h). The bit values
are in "`cal.log`" below. The old wall flags (`no_dip`, `undetermined`,
`narrow`, `card_disagree`) belong to the retired GS power sweep and appear
only when `maburcal report` reads an older log; `saturated` is still set
by a capacity point whose best card peaks above −45 dBm.

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
- `sweep refused by the drone: anchor_unreadable` (every refusal carries the `sweep refused by the drone: ` prefix) — the chip did not
  report its TXAGC reference, or refused to zero its rate diffs (a read
  off a live diff table is reference + diff, and every wall would shift).
  The drone restores its power table at once.
- `no link channel known` — a `power` token without `@<ch>` before the GS
  knows the link's channel; nothing was sent. Name the channel.
- `channel not in the drone's set` / `width not flown by the drone` — the
  `@<ch>` token names a channel outside `radio.channels` or on 2.4 GHz, or
  a 40 MHz measurement was asked of a drone at `radio.width = 20`.
- `wall measurement failed: <code>` — the drone's result. Nothing was
  stored and the cache is as it was. `unparked`: the antenna-switch park
  did not raise the listener level by 10 dB (check the antennas and the
  drone's own radio, re-run); `too_few_points`, `floor_too_high`: the
  listener or the PA was not clean enough to read (an antenna, a wedged
  chain; re-run); `not_saturated`: the PA never compressed within the
  index range, so no curve to cut (a very low-power module); `deadline`:
  20 s were not enough; `device_error`: the radio refused a step or did
  not restore. A failed restore is the drone's `maburd cal: ERROR:
  CrossPathEnd failed` line, and `maburd` then restarts itself about 1 s
  after the report (the link drops and comes back; nothing to do on the
  GS); `aborted`: cancelled.
- `no wall report from the drone` — no copy of the SWEEP_DONE arrived by
  the window's end (the 20 s deadline plus the GS's 1 s slack). The
  measurement may still have succeeded on the drone: its log line says.
  `no ack and no wall report from the drone` is the same with no ack heard
  either.
- `calibration apply timeout` — no APPLIED within 5 s of APPLYs.
- `apply refused by the drone: <reason>` — the drone validated and
  refused: an out-of-range efficiency, a key it could not find or insert, a
  backup or write failure, or a candidate that would not load. The file
  is unchanged; `/tmp/mabur.log` on the drone has the full reason.

A failure after an earlier sweep succeeded keeps its result: the walls of
a finished `power20` stay stored when a later `cap40` fails. The report
lists every apply's outcome.

**The drone was silent and the pair did not come back.** The measurement
detached a chain and switched channel and width, and every exit path
restores them. If video does not return within a couple of DISC beacons
after the run, look for `maburd cal: wall … result=` in `/tmp/mabur.log`.
After an `ERROR: CrossPathEnd failed` line `maburd` restarts itself
within about a second (its wrapper respawns it); if the pair stays down
without one, restart `maburd` or power-cycle the drone.

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
and raises Telem's `auth_reject`. Both stay at zero on a normal run. The
run's own trailing frames under its pair after it closed (the repeats of
the GS's closing Abort) are `cal_late=` there instead, about two per run,
and raise nothing.

## Observability

**`maburgs` log** (`/tmp/maburgs.log`, [`gs/src/cal_wiring.h`](https://github.com/gilankpam/mabur-ground/blob/master/gs/src/cal_wiring.h)):

```
maburgs: CAL start -> ok started power20,cap20,power40,cap40
maburgs: calibration started -> /media/dvr/log/cal/cal.log sweeps=[power20,cap20,power40,cap40] ch=112 (auto: moved to 112)
maburgs: calibration sweep 1/4 power20 done
maburgs: calibration sweep 2/4 cap20 done
maburgs: calibration apply 2/4 cap20: ok
…
maburgs: calibration done
```

A wall sweep the drone refused as unsupported prints `calibration sweep
1/4 power20 skipped: the drone cannot measure its walls` in place of
`done`. An adopted apply always prints `ok` (the drone sends no note). Then
`calibration failed: <why>` / `calibration aborted: <why>`, and the
channel pick's own `maburgs channel: calibration …` lines
([`docs/channel-select.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/channel-select.md)). The start line is printed when the run first
commands the drone, after the pick.

**`maburd` log** (`/tmp/mabur.log`). The measurement's one line:

```
maburd cal: wall ch=112 bw=20 result=Ok park_rise=17.4 floor=-30.6 anchor=39 walls=63,63,63,63,44,26,24,20 captures=88 time_ms=6100
```

`result` is one of `Ok Unparked TooFewPoints FloorTooHigh NotSaturated
Deadline DeviceError Aborted`; `park_rise` the parked-minus-unparked level
in dB, `floor` the best EVM in dB, `captures` the probe captures taken,
`time_ms` the measurement's wall-clock time from the setup; ` (LDPC off:
thresholds lowered 4 dB)` is appended when `radio.ldpc` is off. Also at
boot `maburd radio: wall cache …` (and `… curve fails its checks, not
flown` per invalid entry), on a failed restore `maburd cal: ERROR:
CrossPathEnd failed …` then `maburd cal: ERROR: the wall measurement could
not restore the radio; exiting …`, on every table switch `maburd radio:
power table ch <ch> <bw> MHz: …` (above), `maburd cal: applied <bw> MHz
efficiency` / `apply <bw> MHz refused: …`, `maburd cal:
wall cache write to … failed`, and `cal_latch_drop=` on `maburd tx_send:`.

**Sideport** `link.cal{state, sweep, kind, bw, cell}` while a run owns the
air (`await_ack` through the end of the sweep; absent while picking and
after the end): `sweep` is a `"k/n"` string as on the status line, `kind`
`wall|capacity`, and `cell` the capacity point (`i/8`), or for a wall sweep
the seconds since the ack (`s/20`). maburtop shows `CAL <kind><bw> …` on its
top bar and in the compact header
([`docs/observability.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/observability.md)).

### `cal.log`

Written whatever `debug_log.enable` says — a calibration is a bounded
trace on a deliberate action, and its only after-the-fact record. With
debug logging on it goes in the session directory
(`<debug_log.dir>/NNNN/cal.log`); off (the shipped GS bundle) in
`<debug_log.dir>/cal/cal.log`, i.e. `/media/dvr/log/cal/cal.log`. The
start line names the resolved path, and `maburcal start` finds it there.

Format `callog 6` ([`gs/src/cal_log.h`](https://github.com/gilankpam/mabur-ground/blob/master/gs/src/cal_log.h); [`gs/bundle/maburcal`](https://github.com/gilankpam/mabur-ground/blob/master/gs/bundle/maburcal) is the
in-repo reader and changes with it):

```
callog 6                                                             # once per file
R <nonce> <margin_db> <channel> <auto|pinned|onecard|stayed> <ldpc 0|1|->  # once per run
S <seq> <wall|capacity> <bw> <param>                                 # once per accepted sweep
W <bw> <channel> <anchor> <floor_evm_x10> <w0> <w1> … <w7>           # per finished wall sweep
X <seq> <token> <reason>                                             # a sweep the drone could not run
P <rate> <offered> <recv_best> <air_bits_best> <eff_x1000> <blocked_pct> <flags> <span_ms>  # per rate, capacity
A <bw> <0|1> <reason to end of line>                                 # per apply
```

`S` is written for a sweep the drone accepted (a wall sweep acknowledged
only by its report is written when the report arrives); a skipped sweep
gets an `X` and no `S`. `S`'s `param` is the measured channel for a wall sweep and the dwell in
ms for capacity; it scopes the rows after it to its width until the next
`S`. `W` records what the drone reported, as it reported it: the walls
relative to its anchor, the floor in 0.1 dB, 63 = no crossing. `X` is
written for a sweep the drone refused as unsupported (`X 0 power20
unsupported`), which the GS skipped. `C` (power cells) and `V` (verify)
lines are no longer written. The `R` line is written at the run's first accepted or skipped sweep, so
a run that failed before that has none; `ldpc` is the drone's announced
stamp flag (the GS has no LDPC setting of its own), or `-` when no ack was
heard before the first record (a first sweep acknowledged only by its
report). `P`'s
`eff_x1000` is the measured value even for a `tx_bound` or `lost` point;
`span_ms` is the best card's first-to-last receive span it was timed over
(written even when `short_span` sent eff to the dwell instead). An ok `A`
row's reason is empty. Capacity flags bitmask: 8 `saturated`, 64
`tx_bound`, 128 `lossy`, 256 `lost`, 512 `short_span` (1, 2, 4 and 16 were
power-sweep flags, 32 `drift`, never reused).

One file holds several runs (a retry lands in the same file), each
delimited by its own `R`. A file in an older format is renamed
`cal.log.callog<N>` when `maburgs` starts, so one file never mixes
formats. `maburcal report` reads `callog 1`–`6`: 1 and 2 carry absolute
indices and a `base_ref` on `R` (`-1` = undetermined), 3 is the
single-width relative format, 4 is 5 without `P`'s `span_ms`, its `eff`
divided by the commanded dwell and so reading high, and 5 has `S` kind `power` (param = frames per cell), the `C` cell
lines, per-rate `W <rate> <wall> <floor> <best_card> <flags>`, `V` lines
and no `X` (`docs/data-provenance.md`).

## Offline analysis: `maburcal report`

```sh
maburcal report /media/dvr/log/cal/cal.log
```

re-renders every run in a saved file — the same report `start` prints,
straight from the log, no daemon involved. Nothing else parses `cal.log`.

## Hardware acceptance checklist

Work through this on the bench pair after any change to the kit. A change
to the dialect, the stamp or the RC wire is a two-device flag day: deploy
both binaries (and the drone config first, if a key changed) before
judging anything (`docs/deploy.md`). Record each result in a dated
findings page.

| gate | command | expected |
|---|---|---|
| wall, link channel | `maburcal start power20` | one sweep, `done`, `maburd cal: wall … result=Ok`, the pair back on its channel with video |
| wall, named channels | `maburcal start power20@<ch> power40@<ch>` on each channel of the set | each `result=Ok`, the −17 dB crossing within 2 index steps of the reference, `/etc/mabur.walls` holds one line per (channel, width) |
| link after a measurement | after every wall sweep | `SESSION` again within a couple of beacons, video, uplink delivery as before the trigger |
| table follows | hop across measured and unmeasured channels under `power_mode = "offset"` | the `power table ch … ` log names the right plan, the other width's, or the chip's table |
| rungs | each ladder rung, forced, on three channels at the stored walls and default margin | flies clean |
| full run | `maburcal start` on a 40 MHz GS | four sweeps, `done`, both widths; capacity matches a run from before |
| 20 MHz GS | an all-20 ladder on the GS (derived receiver width 20) | `maburcal start` runs two sweeps; `maburcal start power40` → `err refused: GS receiver width is 20 (load a 40 MHz ladder to calibrate 40 MHz)` |
| unsupported drone | a drone on another chip, `maburcal start` | the wall sweeps are skipped (`X` lines), capacity completes |
| config | `/etc/mabur.toml` diff vs `.pre-cal` after a capacity run | only `efficiency_<bw>`; comments intact |
| abort | `maburcal abort` mid-capacity, and mid-wall | video back; the drone flies normally; the cache is as it was |
| interruption | kill `maburgs` mid-sweep | drone back to video within `await_next_ms` (or the measurement's 20 s deadline); config and cache byte-identical |
| re-link | after every run | `SESSION` again within a couple of beacons, IDR at the join |
| dirty op | start with a jammer or busy air on op | `H relocate … cal` in `scan.log`; the run on the quiet member |
| tunnel lane | the drone's `stats:`/`tx_send:` lines and the sideport during a run | `mav … reject=` flat (`cal_late=` may rise by ~2 at the end of a run), `drone.auth_reject` false, `link.mav.reject` flat, `cal_latch_drop=0` |
| parameter set during a run | `mavparam.py set ENC_BITRATE_MAX …` mid-run | `FAILED`, "calibration in progress" |
