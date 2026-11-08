# Phase lock

The GS holds every frame's capture-time slot a fixed margin before the
vblank release it lands on. Locked, glass latency is flat (no
sensor/panel beat sawtooth) and the regulator places frames by capture
time, so late-tail frames keep their slot. The evidence is
`docs/findings/2026-10-17-phase-lock-spike.md` and
`docs/findings/2026-10-17-phase-lock-spike-2.md`.

Two actuators hold it, one at a time: the drone's sensor frame rate,
steered through the RCF ("The loop" below), or, with
`display.refresh_sync` on and an RK356x display controller, the GS
display's own frame length ("Display actuator: refresh sync"), which moves
the vblank to the frames instead and leaves the drone on its trim.

## The loop

1. **maburplay measures.** For every frame, `FrameRegulator` computes
   `want = pts + delivery floor + allowance` (the lateness window, p1 and
   p99 − p1 + 1 ms capped at `display.sched_cap_ms`) and the release of
   the first vblank at or after it. The margin is release − want, in
   [0, P). It depends only on the capture clock, the window and the vblank
   grid, not on the frame's own arrival. It is measured in both placement
   modes.
2. **maburplay decides** (`gs/player/src/phase_lock.h`). Each 1 s window:
   - error = median of (margin − `display.phase_margin_ms`), wrapped into
     ±P/2;
   - a window with fewer than 10 margins is empty; one whose error spread
     (p90 − p10) exceeds 4 ms is incoherent: it never counts toward
     locking, counts toward unlocking and leaves the offset alone;
   - the phase slope (least squares, skipping the first 150 ms while the
     last offset reaches the drone) names the offset that would match the
     panel, n = o − slope / G with G = 1/60 ms/s per mfps (a rate error of
     D mfps at ~60 fps moves the phase D/60000 s per s); windows whose fit
     residual exceeds 1 ms RMS give no slope;
   - `centre` = EMA of n (α 0.3), the offset sent = centre − 30 mfps per
     ms of error, the correction clamped to ±60 mfps while acquiring and
     ±20 while locked, the whole clamped to ±500 mfps.
   The sensor only takes whole lines, so near lock the line count toggles
   between neighbours.
3. **maburgs forwards.** maburplay sends `phase <mfps>` to RecControl
   (UDP 127.0.0.1:8401) on change and every second. maburgs puts the value
   in every RCF; after 3 s without a `phase` line it sends 0.
4. **The drone applies.** `RcAgent` hands each new `snr_off` to venc. The
   encoder thread sets the sensor to the trim request plus the offset,
   clamped to ±500 mfps and never above the sensor mode's ceiling (maxFps
   × 1000: a mode whose maximum is its nominal rate, like 1728x972@100,
   has the driver refuse anything faster), once the boot trim has locked
   (or given up): one sensor writer. 0 restores the trim's own request. Link loss keeps the
   last value. A refused value is not retried until another request succeeds.

## States

| state | entered when | offset | placement |
|---|---|---|---|
| wait | start; 2 s without a non-empty window; a pts discontinuity (centre and offset reset to 0) | held | arrival |
| acq | the first non-empty window after wait; lock lost | centre − clamp(30·e, ±60) | arrival |
| lock | \|e\| < 1 ms for 3 coherent windows | centre − clamp(30·e, ±20) | capture |
| oor | acquiring for 60 s without locking | 0 (the drone runs on its trim) | arrival |

lock → acq after 2 windows with \|e\| > 3 ms or incoherent. oor → wait
after 60 s. An unreachable panel (50 Hz, 59 Hz) never produces a coherent
window and ends in oor.

## Display actuator: refresh sync

With `display.refresh_sync = "nudge"` or `"follow"` the player bends the
display controller's frame length so the vblank moves to the camera's
frames; the measurement (the regulator's margins), the target margin
(`phase_margin_ms`), the states and capture placement are the sensor
steer's. Code: `gs/player/src/display_lock_core.h` (the controller, pure
logic), `display_lock.h` (its thread), `line_step.h` (NUDGE's step),
`vop2_frame_length.h` (the register).

**The register.** The RK3568/RK3566 VOP2's `DSP_VTOTAL_VS_END` for the
presenter's video port (`0xfe040000 + 0x0c50 + 0x100 × crtc_index`,
mapped through `/dev/mem` once the device tree's `compatible` names
`rockchip,rk3568` or `rockchip,rk3566`): the frame's total lines in bits
28:16, the sync width in 12:0, and an undocumented immediate bit (15): a
write with it set applies to the frame being scanned out now and holds
until the next write. A write of `vtotal + lines` goes in only in the
first half of the frame that started at the vblank the thread woke for --
a total set once the scan-out has passed it runs the frame to the
counter's end, one very long frame. A late wake writes nothing. A release
writes the mode's own value with the immediate bit clear, so it latches at
a frame boundary. The controller asks for a release only once 0 lines
hold: a register off the mode's total gets an immediate write of the
mode's own total first (early in a frame, like every write), and the
release a vblank later, so clearing the immediate bit never lands on a
bent frame.

From its first vblank the lock reads the register: a value that is not
this mode's total and sync width is read again every vblank for up to a
second (right after a modeset -- a hotplug bringing the display up from
the console's mode -- the first vblanks can still carry the previous
mode's total), and one still there after that leaves the lock off for that
presenter's life
(`maburplay: refresh sync: the display controller's total (0x…) is not
the mode's (N lines) -- off`). A total within one line short of it up to
twice it, or an immediate write still held, counts as ours, left by a
run that did not put it back: the controller takes it as the register's
lines and brings it back the same way (0 lines, then the release), with
refresh sync off too.

**The target.** G is the vblank shift the written lines have applied so
far, counted modulo one camera frame Pt on a reference that turns once per
camera frame, so the frames' samples stand still on it. Each margin m
(taken at time t) becomes a sample u = G(t) + wrap(M − m), M =
`phase_margin_ms`: where G should have been for that frame to leave M
(positive: release later, longer frames). The target is the circular
median of the last 2 s of samples (at least 50), the error e =
wrap(target − G). The thread reports each write that landed (`applied`),
and G integrates only those: a skipped or capped write never winds it up.

**The camera's period** (Pc) comes from the decoded frames' capture stamps
(the decode pump's pts sink): every 10 s, a reading over the last 10 s
(at least 500 stamps) of span / round(span / median gap). A reading
within 1 % of the nominal rate is a candidate; two in a row within
100 ppm that differ from the period in use by more than 20 ppm replace
it. Until then the nominal 1e6 / fps names it, fps being the camera rate
the display follow judges by (the drone's reported mode, else the
measured cadence), rounded, passed once a second; a change of it
restarts the measurement and the lock.

**NUDGE** writes the mode's total ± one line, one way only: shorter
frames when the display is the slower (its period over the camera's),
longer otherwise; the drift between the two brings the vblank back the
other way, so the step never moves against it. It starts moving once the
vblank is more than 3 lines away (counted the way lines move it; more
than a quarter refresh the wrong way goes round) and stops within half a
line. It needs the refresh within 1 % of the camera's frame rate
(otherwise wait: "the refresh is not the camera's frame rate"), and it
waits for the camera's measured period before its first step ("measuring
the camera's frame rate", ~20 s: two 10 s readings) with the register at
the mode's own: the one direction is the sign of Pc − Pm, a few ppm
either way, which the nominal rate cannot give -- a wrong guess walks the
vblank away from the frames until the measurement lands. The drone sits
on its trim meanwhile, so the period measured is the one NUDGE goes on
to follow.

So NUDGE's time to lock is that wait plus the pull-in below: some 20 s
plus 10–60 s at 1080p60. Neither actuator holds the phase meanwhile (the
sensor steer was dropped when the display took over), so the picture
runs unlocked, a few ms later, until NUDGE is in step. It starts over
with each presenter (a display rebuild or hotplug builds a fresh lock)
and with a change of the camera's rate.

Its reach is one line a frame: the vblank moves a line a frame against a
drift of |Pc − Pm| a frame, so it pulls in at (line − drift) a frame and
holds only while the drift is under a line. NUDGE waits once the drift on
the period in use passes 0.8 of a line ("the camera drifts more than a
line a frame"). Under the display actuator the drone runs on its sensor
trim, which sits ~0.040·k Hz under the nominal rate (`docs/airtime-model.md`
§3; k = (fps/60)² above 60): 11.1 µs a frame at both rates, 0.75 of a line at
1080p60 (14.8 µs a line) and 1.25 lines at 1125-line 100 Hz (8.9 µs a
line). At 60 Hz NUDGE holds, but pulls in at a quarter line a frame: up
to three quarters of a refresh the long way round, ~56 s. At 100 Hz it
cannot hold, and the actuator choice hands over to the sensor steer
(below).

**FOLLOW** holds each frame as long as one camera frame: B = Pc / line −
vtotal + trim lines over the mode's total, clamped to −0.5 % … +1 % of
vtotal, written every frame, with the error brought back in whole lines on
top (c = round(e / line), 0 inside half a line). The total B + c is capped
at ±2 % of vtotal (22 lines at 1125; the regulator's vblank estimator
accepts a flip delta within 2 % of its period) and is never below the
smaller of 0 and floor(B) (a pull-in goes the long way round instead).
Once locked, frames that keep coming out longer or shorter than the grid
are learnt into it: trim += 0.002 × clamp(want − B, ±2), |trim| ≤ 3. It
needs a camera no faster than the mode by more than 0.5 % (otherwise
wait: "the camera is faster than the display's mode").

**States.** wait (off, a rate mismatch, unusable as above, 2 s without
margins, a pts discontinuity, a camera rate change, a mode set), acq (the
first margins after wait), lock (|e| < 1 ms for 3 consecutive 1 s
judgements; capture placement on), lock → acq after 2 judgements with
|e| > 3 ms. There is no oor state: FOLLOW reaches any camera from 0.5 %
faster than the mode up, and NUDGE past its reach hands over to the
sensor steer. Off, a rate mismatch and the unusable cases put the
register back at the mode's own value; waiting for margins, FOLLOW holds
its grid and NUDGE the mode's total.

**The thread.** One per presenter, built after its `init()` while
`display.phase_lock` is on, stopped before it is retired (a display
rebuild, a hotplug, a follow) and at shutdown, never carried to the next
presenter. It wakes on each of the presenter's vblanks (`drmWaitVBlank`
on its CRTC; no DRM master needed), steps the controller and writes the
register; it never touches DRM state. `SCHED_FIFO` at `rt_priority − 1`
(at least 1), below the release thread it must never delay; 0 runs it at
ordinary priority. Stopping it brings a register it left off the mode's
own back: 0 lines written for up to 5 vblanks, one more waited, then the
mode's own value.

**Choosing the actuator** (housekeeping, every iteration, with
`display.phase_lock` on): the display while the presenter's lock is
engaged (refresh sync not off, the register mapped and the mode's total
taken in its first second of vblanks); otherwise the sensor steer,
created then.
While the sensor steer runs with `refresh_sync` not off, the reason is
logged once each time it changes -- also when the steer was already
running before refresh sync was set:
`maburplay: refresh sync nudge unavailable (<why>) -- steering the
sensor` -- `not an RK356x`, `/dev/mem: …`, the display controller's total
not the mode's, `no display`, `no vblank from the display` (a lock
that has not seen its first vblank within 2 s; until then whichever
actuator ran keeps running, so a start or a rebuild does not swap back
and forth), or `the camera drifts more than a line a frame`. The last
said is forgotten when the display engages or `refresh_sync` goes off, so
a later fallback for the same reason is said again. When the display
engages with the sensor steer running, the player sends one `phase 0`
and drops it: `maburplay: refresh sync nudge -- the drone's sensor runs
on its trim`.

NUDGE waiting past its reach hands over to the sensor steer and latches
there for the presenter's life: the lock is set off, which brings the
register back to the mode's own value. Without the latch the pair would
swap back and forth -- the sensor steer pulls the camera onto the
display's rate, NUDGE sees it within reach, engages and drops the steer,
the drone returns to its trim, and NUDGE is past its reach again. A live
set of `display.refresh_sync` and a new presenter (a display rebuild, a
hotplug, a follow) clear the latch; NUDGE then hands over again if the
camera, back on its trim, still drifts past its reach.

A live set of `display.refresh_sync` (8403) goes to the running lock:
nudge ↔ follow restarts its window and trim, off releases the register
to the mode's own value and the next iteration brings the sensor steer
back, nudge/follow from off engages the display and drops it. The host
build has no lock and always steers the sensor.

The goggles settings menu's "Phase lock by" row (`video.refresh_sync`,
advanced, `docs/gs-menu.md`) sends the same set, labelled Camera (`off`),
Display (nudge) and Display (follow); it warns `n/a` when a Display choice
is selected but cannot act on this GS: `display.phase_lock` is off (no lock is built),
the display's register is not mapped (not an RK356x -- the Orange Pi has
none -- or `/dev/mem` refused), or the lock refused the controller's
total within its first second of vblanks. The value still saves either way. No warning
while the lock waits for its first vblank, and none for NUDGE handing
over past its reach (that is logged).

**What the drone sees.** Nothing new: under the display actuator the
player sends no `phase` lines, maburgs' RCF `snr_off` is 0 (after the
one `phase 0`, or its 3 s timeout), and the drone runs on its trim.

## Rate mismatch

The player measures the drone's frame rate (median pts step over 1 s,
`SourceRate`) and compares it with the panel's refresh (`RateMatch`,
`gs/player/src/rate_match.h`). More than 0.55 Hz apart (the steer reaches
±0.5 fps) for 3 consecutive seconds is a mismatch: a drone `venc.fps` that
does not match `screen_mode`, a camera rate change the display has not
followed yet, or a display that fell back to another mode because the sink
does not offer the requested one (or offers no mode at the camera's rate).
While it lasts the lock is held in wait and sends nothing (maburgs' RCF
`snr_off` falls to 0 after its 3 s timeout, so the drone runs on its
trim; under refresh sync the register goes back to the mode's own
value), capture-time placement is off, the OSD fps cell reads
`src/panel` in caution, and the player logs
`maburplay: rate mismatch: source X Hz, panel Y Hz -- phase lock paused`
(and `... cleared` once 3 matched seconds follow). A drone reporting
low-power is never a mismatch: its rate is low on purpose. The check runs
whether or not the GS OSD is up; the low-power flag comes from its sideport
reader and counts only while fresh. With the reader up but the sideport
quiet the check holds its last decision; with no reader at all the drone is
taken as armed. A pair close enough to call a match that the lock still
gives up on (out of range: the steer cannot reach the panel, e.g. a mode
whose ceiling sits just under it) tints the OSD fps cell caution too.

A sensor that refuses every `SetFps` (or a drone build with no venc
steering at all) never moves, so the centre walks to the range limit
without ever producing a coherent window: the lock unlocks, goes oor
after 60 s, and retries a minute later -- a steady acq/oor cycle visible
in `reg.log`, paired with a `[sensor] steer: ... refused` line on the
drone.

## Wire

`Rcf::snr_off`: int16 little-endian at RCF head offset 15, milli-fps on the
drone's trim request, 0 = no steering. `RC_VERSION` 16. The web GS steers
too, in GS mode; spotters send no RCFs at all and so always send 0.

### Web GS

`web/` (GS mode) runs the same loop in the browser instead of against a
display's vblank: the page measures its own refresh from rAF timestamps
(`VsyncGrid`, `web/ui/src/lib/phase.js`: the period is a ratio estimate,
the summed steps over the summed whole periods they span, taken over the
steps near the median), feeds the
same `PhaseLock`/`RateMatch` C++ sources through WASM exports
(`web/src/web_phase.cpp`), and holds a decoded frame to its release tick
instead of drawing it on arrival (`HoldQueue`). The offset reaches the
drone the same way as from `maburplay`: through the core's RCF `snr_off`.
Spotter mode sends no RCFs and runs no phase-lock calls at all. See
`docs/web-gs.md` ("Phase lock") for the toggle, the hold, and the Debug
tab group.

## Deploy

`RC_VERSION` 16 is a flag day for `maburd`, `maburgs` and the hosted web GS
page: finish the deploy on both devices (and republish the web page) before
judging the link, per `docs/deploy.md`. On the GS, the `maburplay` binary
goes in before its config -- an older binary exits on the new
`[display]` keys (`phase_lock`, `phase_margin_ms`, `refresh_sync`).
A live `set display.refresh_sync` (8403 or the menu) only rewrites a line
the file already has (`docs/gs-live-config.md`, "File rules"): a GS whose
TOML predates the key needs the bundle's `refresh_sync` line first.
A `maburd` from before the ±500 mfps range clamps `snr_off` at ±200 mfps,
so a panel that needs more (phone-class refresh rates) cannot lock until
the drone is redeployed.

## Configuration

`maburplay.toml` `[display]`: `phase_lock` (code false, bundle true),
`phase_margin_ms` (code and bundle 4, range 1-10), `sched_cap_ms` (bundle
16; with the lock on, capture placement runs only while locked).
`vsync_lead_ms` (code and bundle 5) is the release's lead before the
vblank, the only one -- recording runs under the same lead as live view.
It covers the release thread's wake lateness (`reg.log` `rlate=`), the
present() path and the kernel's latch deadline. `rt_priority` (code and
bundle 10, range 0-50) is the release thread's `SCHED_FIFO` priority; 0
runs it at ordinary priority. A refused request (the process lacks
`CAP_SYS_NICE`) is logged once and the thread runs at ordinary priority
(`rt=0`).
`phase_lock` needs `vsync_lock = true`, `sched_cap_ms > 0` and
`regulate_ms > 0`, or the player refuses to start.

`refresh_sync` (code `"off"`, bundle `"nudge"`) picks which actuator holds
the phase: `"nudge"` moves the display's refresh one line a frame, one way
only, for a panel that blanks on a larger change -- it reaches a camera
drifting under a line a frame against the refresh, and past that (the
drone's trim at 100 fps) the player steers the sensor instead;
`"follow"` bends every frame's length to the camera's, for a panel that
takes any frame length; `"off"` steers the drone's sensor rate instead.
On a GS that is not an
RK356x (no VOP2 frame-length register) the set still applies and writes
the value, but the player keeps steering the sensor and logs it.

`screen_mode` takes three forms (`gs/player/src/mode_pick.h`); the
bundle ships `"auto"`:

- `"auto"`: the display follows the camera. Among the display's
  non-interlaced modes within 0.5 Hz of the drone's measured frame rate,
  the largest (`w*h`; a tie goes to the closest rate).
- `WIDTHxHEIGHT`: the display follows the camera at this size: that size's
  mode closest to the camera's rate, within 0.5 Hz.
- `WIDTHxHEIGHT@RATE`, RATE an integer or a decimal: pinned, the camera's
  rate aside. Among non-interlaced modes of that size within 0.5 Hz of
  RATE, the one with the closest exact rate (from the pixel clock and
  totals): a sink offering 60.000 and 59.94 under "@60" gets 60.000,
  `@59.94` forces 59.94, a 59.94-only sink still matches "@60" and the lock
  steers the camera there (~3 lines, centre near −50 mfps). No match falls
  back to the sink's preferred mode.

`"auto"` and `WxH` are resolved to one `WxH@R` mode each time a
presenter is built (startup, a hotplug, a live set) at the camera's
current rate -- or, with the drone in low power or its report quiet, at
the last rate seen out of low power, so a replug or a set while disarmed
does not move the display to the low-power rate. Before the first frame
the rate is not known, so `"auto"` takes the sink's preferred mode and
`WxH` that size at the preferred mode's rate.

Once a second (`DisplayFollower`, `gs/player/src/display_follow.h`), with
the camera's rate measured and the drone reporting it is out of low power,
the player re-picks the mode for the camera's rate and, when 3 consecutive
windows pick the same mode other than the running one, rebuilds the
presenter on it and logs `maburplay: display follows camera:
1920x1080@100.000`. This runs whether or not there is a rate mismatch, so
`"auto"` also moves up to a larger mode at a rate the panel already runs;
one window's rate (a fade losing over half its frames, a camera change
mid-window) never rebuilds alone. Like any presenter rebuild it restarts a
burned recording in progress. The follow is an in-process rebuild, not a
`set`: the file keeps `"auto"`.

A rebuild keeps the DRM device open across the swap
(`DrmPresenter::retire`), so the kernel's fbdev console never takes the
output back at its own 1280x720@60 in between. When the new presenter's
mode differs from the running one it turns the output off and holds it
off 0.5 s before its modeset (about 0.75 s dark in all): an Avatar Goggles
X stays blank when a new mode follows the old one within a few tens of
milliseconds, and the follow's 60 → 100 Hz switch at startup did exactly
that. A rebuild on the same mode (a `colortrans.enable` or overlay switch)
keeps the output up.

A camera rate the display has no mode for (an Avatar Goggles X has no
90 Hz) takes the modes at the display's rate nearest the camera's (an
exact tie: the higher rate), by the same size rule, and the rate mismatch
stays (the OSD fps cell reads `90/100`). The running mode is kept over the
pick when it fits, is at least as large, and is as near the camera's rate
within 0.5 Hz (within 0.5 Hz of the camera when the display has its rate),
so a 90 fps drone on a 60/120 Hz panel, whose measured rate crosses the
midpoint window to window, does not rebuild back and forth. A connector
with no modes (a failed EDID read) never rebuilds, and a follow rebuild
whose presenter came up on another mode (the sink's list changed) is not
issued again for that mode until the pick settles or moves elsewhere. The
presenter logs `DrmPresenter: mode 1920x1080 @60.000 Hz (requested
1920x1080@60.000)`.

The drone has no phase-lock config.

## Threads

maburplay runs four loops, plus refresh sync's display-lock thread, so
the release deadline never waits on anything slower than itself:

- **Feed thread** (`FeedLoop`, `feed_loop.h`): blocks on the AU doorbell,
  reads the ring, and makes every decoder *input* call (`AuRouter`,
  `StreamFeeder`). It posts submit notes and delivery notes.
- **Decode-output thread** (`DecodePump`, `decode_pump.h`): drives
  `backend->poll()` (MPP blocks in it up to 4 ms waiting for a picture)
  and returns released frames to the decoder -- the decoder's whole output
  side. Per decoded frame, in order: the frame's submit notes (`lat`
  submit stamps, incomplete-AU flags for the lateness window),
  `lat.on_decoded`, the source-rate sample, the hand-over to the release
  thread, then the framebuffer prime, so the release thread's present()
  finds the frame's framebuffer cached, and last the frame's capture stamp
  to the display-lock thread's camera period. It also takes the submit
  notes after every poll round, frame or not, so they never queue up in
  the feed while the decoder emits nothing.
- **Release thread** (`ReleaseLoop`, `release_loop.h`, `SCHED_FIFO` at
  `rt_priority`): owns the `FrameRegulator`'s decisions and every call
  into the display. It sleeps on an absolute timer armed to the next
  release (or a 10 ms idle tick), the hand-over queue and the display's
  flip events. Each wake: reap flips, release due frames, offer newly
  handed-over frames (a late one is presented at once), set the
  presenter's paced mode, run the chain-break heal, and commit a pending
  OSD on its own when no frame is held. A completed flip also feeds the
  burned recorder: the flip sink runs `RecorderTap::on_flip`
  (`recorder_tap.h`), which calls `BurnRecorder::submit()` on each
  recorded slot of the vblank grid (one buffer reference and a mailbox
  post). It never calls the decoder: displaced frames and frames the
  display is done with go back through a release queue the decode-output
  thread drains. Each offer's phase margin is queued for housekeeping.
- **Display-lock thread** (`DisplayLock`, `display_lock.h`, `SCHED_FIFO`
  at `rt_priority − 1`; RK356x with `display.phase_lock` and
  `refresh_sync` available): one per presenter, waking on its vblanks to
  write the frame-length register ("Display actuator: refresh sync"). It
  takes margins and stamps from the other threads through the
  controller's mutex and never calls into the presenter's DRM state.
- **Housekeeping** (the main loop, 2 ms): delivery notes (raw DVR, OSD
  figures), the record button, the phase lock itself (it picks the
  actuator, takes the release thread's margins while the source and panel
  rates match, and ticks and sends the sensor offset or feeds the display
  lock), the rate match, OSD composition, the 1 Hz stats,
  `lat.log`/`frame.log`/`reg.log`, the decode watchdog and the display
  hotplug retry. It composes the screen's OSD only while no publish is
  pending, so it never draws into a buffer that is on screen or about to
  be; the burned recording's overlay, a canvas of its own at the record
  size, composes from the same input whether a publish is pending or not.
  It also installs and removes the recorder tap's hook (`RecorderTap::set`
  after `BurnRecorder::start()`, `clear()` before `stop()`), so no record
  start or stop runs on the release thread. A flush, a watchdog reset or
  re-creation, a hotplug reacquire and shutdown park the decode-output
  thread and then the release thread (which drops everything the display
  and the regulator hold), return every frame to the decoder, and resume
  them after; a thread that does not park within 2 s exits the player for
  respawn.

`--decode-only` (the hardware decode gate) and the host build have no
decode-output or release thread: one loop polls the decoder and releases
each frame on the spot.

## Observability

- `reg.log` (`# reglog 5`) regulator line: `lock=<off|wait|acq|lock|oor>
  lerr=<ms>ms loff=<n> lctr=<f> lact=<sensor|nudge|follow>` -- under
  `sensor` the offset and centre in mfps, under `nudge`/`follow` the
  lines in the frame-length register and the grid's lines -- and the
  release thread's
  `rlate= hand= rt= flushes= recstarts=` plus the burned recorder's
  `burn_in= burn_enc= burn_drop=` (docs/observability.md
  "Regulator line").
- maburplay stderr: `phase-lock: <state> err= off= centre=` on each state
  change of the sensor steer; `refresh-sync: <state> err= lines=` on each
  of the display lock's. The display-lock thread's own lines:
  `refresh-sync: on (nudge: one line at a time)` /
  `on (follow: each frame as long as the camera's)`, every wait reason as
  it changes (`refresh-sync: rate mismatch`, `waiting for frames`, …),
  NUDGE's direction (`refresh-sync: frames made only longer (the display
  no slower than the camera)` / `shorter (the display slower)`),
  `refresh-sync: in step` at the first lock, and every 5 s
  `refresh-sync: err=<ms>ms lines=<+n> (grid <f>, trim <+f>)
  moved=<+us>/5s cam=<us>us` (trim: FOLLOW's learnt lines in the grid, 0
  under NUDGE). The actuator choice logs the two
  `maburplay: refresh sync …` lines above.
- Drone stats line: `snr_off=` (the offset on the sensor); `/tmp/mabur.log`
  `[sensor] steer:` lines for the first steer, clamps and refusals.
- `tools/flightreport.py` (session mode): PHASE LOCK — share of rows
  locked, acquire time, the actuator (the last row's `lact=`), offset
  changes per minute, the offset range in that actuator's unit (mfps or
  lines, over its rows), |lerr| while locked.
