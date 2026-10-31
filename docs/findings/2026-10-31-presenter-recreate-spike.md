# Presenter recreate spike: can maburplay tear its live display down and rebuild it mid-run?

**Status:** spike for the GS live-config work (live `screen_mode` and OSD-plane
changes). The mechanism is the hotplug acquisition sequence in
`gs/player/src/main.cpp`, run on a live presenter. Nothing shipped from this
spike; the throwaway patch lived on a local branch and is not merged.

## Question

The GS settings menu will change the display mode and the OSD switches
while video plays. `DrmPresenter` has no re-modeset; the only path that
builds one at run time is the hotplug retry, which constructs a presenter
only while none exists, and nothing in the player destroys a live one. Is
destroy-then-rebuild clean on both GS boards, with a recording running,
and across a real mode change?

## Method

A throwaway maburplay (master `f901ee2a` plus the spike patch) handles
SIGUSR2 on the housekeeping thread: stop a running burned recorder
(`stop_burn`), `park_output`, `release.set_sink(nullptr)`,
`presenter.reset()`, build a fresh `DrmPresenter` on the configured mode
(or on `MABUR_SPIKE_MODE` for odd presses), `install_colortrans`,
`regulator.set_panel_period`, `phase_lock->on_discontinuity`, clear the
margin samples, `set_flip_sink`, `release.set_sink`, `resume_output`,
`start_burn_if_needed`. The stock player was stopped and the spike binary
run by hand against the board's own `/etc/maburplay.toml`, with the bench
drone linked at 60 fps. The stock player was restored afterwards on both
boards.

## Results

| Board | Round | Recreates | Teardown / total per recreate | Outcome |
|---|---|---|---|---|
| Radxa ZERO 3 (mpp, 2560x1440@99.95) | same mode, no recording | 5 | ~190 / ~500 ms | video continues, vsync lock back, no errors |
| Radxa | burned recording running (target both, 1080p every=1) | 3 | ~320 / ~630 ms | recorder stopped and restarted each time, one new DVR file per recreate, `recstarts=4` |
| Radxa | 1440p100 ↔ 1080p60 | 4 | 90–180 / 390–440 ms | new mode each time; phase lock acquires at 1080p60 (source 59.99 Hz), reads "rate mismatch, paused" at 1440p100 as it should |
| Orange Pi Zero 2W (v4l2req, 1920x1080@60) | same mode | 5 | ~75 / ~215 ms | `v4l2req errors=0` throughout, vsync locked |
| OPi | raw recording running | 3 | ~75 / ~215 ms | raw DVR untouched (it does not depend on the presenter) |
| OPi | 1080p60 ↔ 1440p60 | 4 | 45–70 / 185–230 ms | new mode each time, plane CTM reinstalled, phase lock re-acquires |

Across 12 recreates in one process on each board: the regulator's held
count kept climbing, `present=60` every second, no decode watchdog, no
"did not park" exit, no DRM errors in the kernel log, no leaked or
double-released frames visible (the decoder's pool would have starved).

## Costs seen

- **Two modesets per recreate.** Closing the DRM fd drops master and the
  kernel restores the console's mode (Radxa: `1920x1080p60` lands, then
  the requested mode ~0.4 s later). The screen shows the console mode for
  roughly the teardown time. An in-place re-modeset inside `DrmPresenter`
  would avoid it; this spike did not need it.
- **Radxa: one `mpp_rkvdec2 … resetting` per recreate**, ~0.4 s after the
  modeset. The park aborts the open streamed picture (`stream: …
  aborted=` grew by one per recreate) and the kernel resets the decoder;
  it recovers every time. The same reset appears occasionally under the
  stock player, so it is the known price of a long park, not a new fault.
- OPi: the OSD is on the primary plane and the video on the overlay
  plane; both came back on every rebuild with no sign of the DE33 scaler
  table trap in the logs (the picture itself was not inspected during
  the run).

## Conclusion

Destroy-then-rebuild is a usable mechanism for a live display-mode or
OSD-plane change on both boards. It costs a few hundred milliseconds of
parked video, a console flash, and on the Radxa one decoder reset. The
live-config design builds on it; the in-place re-modeset stays an
optimisation if the flash matters later.
