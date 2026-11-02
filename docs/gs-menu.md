# The goggles settings menu

A five-way-pad settings menu drawn into the OSD plane on top of the video
-- or, before the first video frame ever arrives, over the startup splash
("Before the first video frame" below) -- for adjusting the link, the
display and recording without a laptop. It is
read-and-write live: every row reads the running config and maburgs' live
keys, and every value the operator picks goes out over the same two
loopback setters `docs/gs-live-config.md` describes — nothing here reads
or writes a TOML file directly. The menu is never burned into a
recording: `OsdComposer::compose_burn` does not see it, only
`compose_screen` does, and it draws last there so it wins every pixel
collision with the GS bar or the MSP OSD.

## Buttons and gestures

Five GPIO lines feed one shared button timer (`button_bank.h`,
`key_timer.h`): `up`, `down`, `left`, `right`, `sel`. A press only reaches
the timer once it has held its debounced level for 50 ms
(`ButtonDebounce::kDebounceMs`); the timer itself then turns that into a
tap, a hold (600 ms, `KeyTimer::kHoldMs`) or, on `up`/`down` while a
number picker is open, an auto-repeat (first repeat at 400 ms, then every
100 ms, `KeyTimer::kRepeatDelayMs`/`kRepeatMs`). A button already held
when the player starts, or idling at the pressed polarity because the
wiring is inverted, seeds the timer silently rather than firing on sight.

- **Closed:** holding ▶ past 600 ms, or tapping SEL, opens the menu. A
  bare tap on ▶ while closed does nothing but toast "Hold ▶ to open the
  menu" — a 20 ms brush on the pin never opens it.
- **Open:** ▲▼ move the row cursor (tap), or step a number picker,
  auto-repeating while held. ◀ tapped goes back a level, or closes the
  menu from the top level. ▶ tapped, or held past 600 ms, opens a group,
  opens a row's picker, runs an action, or commits a picker to its value
  — a held ▶ fires that once, at the 600 ms mark, and nothing again when
  the finger lifts, the same one-shot rule that keeps a long press from
  double-firing. SEL tapped is a second ▶. ◀ held past 600 ms closes the
  menu from anywhere, cancelling whatever picker or confirm prompt was
  open without sending it.

`menu_buttons_wired()` needs `up`/`down`/`left`/`right` all configured;
`sel` is optional. With any of the four missing the bank never opens and
the player runs with no menu — logged once, non-fatally, same as a
missing record-button pin.

## Screen anatomy

Four boxes (`menu_layer.h`), each a filled panel in `tok::kMenuPanel`:

- **Panel** — top-left, inset 24 px at 1080p, 45 % of the screen's width,
  up to 10 rows scrolled around the cursor, the crumb and link pill, two
  help lines and a keys line.
- **Picker** — opens to the right of the panel while a value is being
  changed: a choice list, a number spinner, or a go-ahead confirm prompt.
  Neither carries a title line; the row beside it already names it.
- **Toast** — one line, centred above the GS bar, shown for 1800 ms
  (`MenuModel::kToastMs`).
- **Banner** — centred near the top, shown with the menu closed too; while
  the menu is open it moves below the panel and picker so it never covers
  either. It exists only when the menu itself is on (buttons wired, or
  `MABUR_MENU_DEBUG`); a unit with no menu never draws it, and shows the
  camera/panel mismatch only through the OSD's fps cell.

Every design length — text size, padding, inset, the panel's 45 % — is
multiplied by `screen_h / 1080` and by the operator's menu size
(`[menu] scale`: Normal ×1, Large ×1.25, Larger ×1.5), so a given size
takes the same share of the screen at 720p, 1080p, 1440p and 2160p. Text
snaps to the nearest size `gs_osd.gfont` bakes (`GsFont::nearest`); the
boxes follow the snapped size. Row text is 22 px and help/keys/tag text
15 px at 1080p Normal; at 720p Normal the help text clamps up to the
smallest baked size, 13 px. The panel gives up width before a confirm
picker (the widest fixed-width picker) would be squeezed against the
screen's right edge — on a 4:3 screen at Larger it is narrower than
45 % × 1.5. Nothing else on the OSD plane follows the menu size: the GS
bar and the MSP OSD keep their own.

The selected row or choice is `tok::kMenuSelect`; a drone-owned row's VTX
tag (reserved for when one exists) is `tok::kMenuTag` — both defined next
to the GS bar's own tokens in `gs_layer.h`. Every pixel the menu writes is
premultiplied and passed through `colour_inverse()`, same as the bar
(`gs_draw.h`). The menu has two `MenuLayer` instances, one per DRM buffer
(there is no burn twin, since it is never burned); a surface with no
pixels — the presenter has given the OSD plane up — makes `paint()` draw
nothing and return 0 rather than fail.

## Rows

Every row is built fresh from live state on each call (`menu_tree.cpp`),
so nothing shown goes stale while the menu sits open. A GS-owned row's
value reads `—` until maburgs' live keys have been read at least once
(below) — the defaults a fresh `GsLiveView` starts with are not maburgs'
real values.

| Row | Owner | Live key(s) | Notes |
|---|---|---|---|
| **Video** | | | |
| `video.camera` "Camera mode" | drone | `VENC_MODE` | Choices are binned sizes then native sizes, for every rate the display shows (every rate with no connector modes); each 120 fps choice notes "letterboxed 2.1:1". The drone's current value is pulled to the front when it is off that list, noted "display can't show" and unpickable if its rate is one the display cannot show at all (`keep_current_if_unpickable`), else picked straight through. Confirm/apply drag `REC_FPS` down first when the onboard recorder is on and set above the new rate. |
| `video.fix` "Fix: set camera to …" | drone | `VENC_MODE` (+ `REC_FPS`) | Visible only while the camera's own rate is one the display cannot show; targets the display-shown rate in `60/90/100/120` nearest the camera's (a tie keeps the higher rate), at the camera's own size when that rate's sensor mode offers it, else that mode's first size. Same drag/confirm as Camera mode. |
| `video.display` "Display" | player | `screen_mode` | Choices: `auto` (largest size at the camera's rate) or a `WxH` the display offers within the camera's rate. Confirm: splits a running burned recording. Warns `≠ camera` only when the display has modes but none at the camera's rate — not when the mode list itself is empty (a failed EDID read warns nothing). |
| `video.colour` "Colour correction" | player | `colortrans.enable` | On/off. Confirm: splits a running burned recording. |
| `video.mirror` "Mirror" | drone | `VENC_MIRROR` | On/off. Flips the picture left-right, for a camera mounted mirrored. |
| `video.flip` "Flip" | drone | `VENC_FLIP` | On/off. Flips the picture upside down, for a camera mounted upside down. |
| `video.slices` "Row slices" (advanced) | drone | `VENC_SLICES` | On (4) / Off (1); any stored value above 1 reads On. |
| **Link** | | | |
| `link.width` "Width" | gs | `link.static_bw` (fixed) or `link.ladder` (adaptive) | 40/20 MHz. Under a fixed rate this only re-pins the width. Under adaptive it loads that width's default ladder (MCS 0,1,2,3,4 at 0.50/0.25 bw 40; MCS 0,1,2,3,4,5 at 0.50/0.25 bw 20, the bundle default) unless the live ladder is already one of those two tables, with a confirm naming the rung count. |
| `link.rate` "Rate" | gs | `link.static_mcs` (+ `static_bw`/`static_overhead_base`/`static_overhead_enh` into fixed) | Adaptive/Fixed. Switching to fixed sends the current rung's width and FEC overhead, then MCS last (capped at 5), so the link moves once; switching to adaptive is one set, `static_mcs = -1`, re-entering the ladder on that rung. |
| `link.mcs` "MCS" (fixed only) | gs | `link.static_mcs` | 0–5. |
| `link.fec_base` "FEC base" (fixed only) | gs | `link.static_overhead_base`(+`_enh`) | 0.10–2.00 step 0.05; a base taken below the current enh drags enh down to it in the same sets, since base must stay ≥ enh. |
| `link.fec_enh` "FEC enh" (fixed only) | gs | `link.static_overhead_enh` | 0.10–base step 0.05. |
| `link.ladder` "Ladder" (advanced) | gs | `link.ladder` | One `R<i>` group per rung: MCS (0–5), Width (40/20), FEC base/enh, Move up, Move down, Delete rung (confirm; refused at one rung left), plus "+ Add rung" below 8 rungs (copies the last rung, MCS+1 capped at 5). Every rung edit re-sends the whole ladder — the live key takes it as one unit. |
| `link.nack` "Retransmit (NACK)" (advanced) | gs | `link.nack.enable` | On/off. |
| `link.bitrate` "Max bitrate" | drone | `ENC_BITRATE_MAX` | 1–100 Mb/s step 1; the wire key is kbps. |
| `link.airtime` "Airtime share" (advanced) | drone | `ENC_AIRTIME` | 5–100 % step 5; the wire key is a 0–1 fraction. |
| `link.ldpc` "LDPC coding" (advanced) | drone | `RADIO_LDPC` | On/off. Keeps its applied row's `status` text amber in the help line (`warn_after_ok`) rather than a plain "✓ saved" once the set lands `accepted` with a non-empty status — the drone's calibration ties a wall-margin note to this flag. |
| **Recording** | | | |
| `rec.target` "REC records to" | player | `dvr.target` | GS / VTX SD card / Both. Saved text is "✓ at next recording" (2200 ms) while a recording runs, else "✓ saved" (1300 ms) — same rule on every recording row below. Warns "SD recorder off" while routed at least partly to the VTX SD card (`vtx`/`both`) and `REC_ENABLE == "0"`. |
| `rec.mode` "GS recording" | player | `dvr.mode` | Burned (with OSD) / Raw (as received); hidden when `dvr.target` does not record to the GS. |
| `rec.burn.bitrate` "Bitrate" (advanced) | player | `dvr.burned.bitrate_kbps` | 2–20 Mb/s step 1; visible only in burned mode. |
| `rec.burn.every` "Frame rate" (advanced) | player | `dvr.burned.every` | Every 1st–4th frame, labelled in fps once the running mode is known; visible only in burned mode. |
| `rec.burn.size` "Size" (advanced) | player | `dvr.burned.size` | Same as video / 1920×1080 / 1280×720 / 2560×1440; visible only in burned mode. |
| `rec.vtx` "VTX SD recorder" | drone | `REC_ENABLE` | On/off; confirms the rebuild pause. |
| `rec.vtx.bitrate` "SD bitrate" | drone | `REC_BITRATE` | 2–80 Mb/s step 1; the wire key is kbps. Visible only while `rec.vtx` is on. |
| `rec.vtx.fps` "SD frame rate" | drone | `REC_FPS` | The camera's own fps and its whole divisors ≥ 30, descending, plus the drone's current value first when it is off that list. Visible only while `rec.vtx` is on. |
| `rec.vtx.size` "SD size" | drone | `REC_SIZE` | The camera's own sensor mode's sizes (`WxH`), the camera's own size appended when it is off that list, plus the drone's current value first when it is off the resulting list. Visible only while `rec.vtx` is on. |
| **OSD** | | | |
| `osd.gs` "Link stats bar" | player | `osd.gs.enable` | On/off. Confirm: splits a running burned recording, only when turning ON (turning an overlay off never splits — the composer's erase reaches the recorder's canvas directly). |
| `osd.fc` "Flight controller OSD" | player + gs | `osd.enable` + `msp.enable` | On/off, shown as `mixed` (with a help line naming which half is off) when the render and maburgs' MSP feed disagree; picking On or Off sends both. Warns "FC link off" while the drone's own `MSP_ENABLE == "0"` (nothing feeds the OSD even with the render on). |
| **Power & FC** | | | |
| `power.lowpwr` "Low power when disarmed" | drone | `LOWPWR_ENABLE` (+ `MSP_ENABLE`) | On/off. Turning it on while `MSP_ENABLE == "0"` sends `MSP_ENABLE=1` first (confirm "Also turns the FC link on.") — low power needs the FC link for arm state. |
| `power.msp` "FC link (MSP)" | drone | `MSP_ENABLE` (+ `LOWPWR_ENABLE`) | On/off. Turning it off while `LOWPWR_ENABLE == "1"` sends `LOWPWR_ENABLE=0` first (confirm "Also turns Low power off.") — low power depends on this link. |
| **System** | | | |
| `sys.adv` "Show advanced settings" | player | `menu.show_advanced` | On/off; shows the rows marked (advanced) above in every group. |
| `sys.size` "Menu size" | player | `menu.scale` | Normal / Large / Larger (×1, ×1.25, ×1.5). Previews live: while its picker is open the whole menu is drawn at the size under the cursor (`MenuModel::preview`), nothing written; ◀ or a held ◀ drops back to the saved size, ▶ saves it — and the menu stays at the picked size while that set runs, so it never flicks back to the old size before the save lands. |
| `sys.debug` "Debug log" | gs | `debug_log.enable` | On/off. |

**Not covered yet:** a `Channel` group and a `Calibration` group.

## Applying a row

Picking a value that differs from the row's current one turns into one
or more `SetStep`s (`menu_model.h`), each tagged with the owner it goes
to. `MenuApplier` runs one row's steps in order, one row at a time (no
per-owner runners): a later step is never sent once an earlier one has
answered `err` or timed out.

- A player-owned step goes through `PlayerLiveSetter`, which calls
  `PlayerConfigControl::submit_local` — the same in-process `submit()`
  path a `set` on 8403 takes, never through the socket, so the menu never
  fights an operator's own 8403 client for the daemon's one pending slot
  (`docs/gs-live-config.md` "Protocol").
- A GS-owned step goes through `GsLiveSetter`, an ordinary
  `LiveUdpClient` exchange to maburgs' 8402.
- A drone-owned step goes through `DroneLiveSetter`, over its own
  `LiveUdpClient` to the same 8402 (so its reply never collides with a
  GS step's or the view refresh's own exchange). Its `vtx set <PARAM_ID>
  <value>` only confirms the line was sent (`ok`) or refused outright
  (`err <why>`); the actual outcome is read back later from
  `link.vtx.rows[]` (`docs/observability.md` "`link.vtx`"). `DroneSetWatch`
  (`menu_apply.h`) is the settle rule: a set is taken as settled by the
  row's next terminal state once any of — the watch has seen the row
  leave idle (`sending`/`in_progress`) since the set began; the row's
  result time on maburgs' own clock (the sideport's `t_ms` minus the
  row's `age_ms`) reads later than the one in the snapshot taken at
  submit; or the row carried no result at all before the submit. A bare
  "the age got smaller" comparison is kept only as the fallback for a
  peer that never exported `t_ms` on one side or the other — on its own
  it is not trustworthy, since a result that was already very recent at
  submit (say 50 ms old) can be followed by a genuinely new result that
  still reads a *bigger* age (150 ms, off a datagram that arrived later)
  while still being the later event; only the absolute result time tells
  the two apart. A stale sideport or `link.vtx.linked == false` mid-wait
  ends the step at "link lost"; a `failed`/`unsupported` row ends it at
  the row's own `status` text (or "refused" when that's empty);
  `timeout` ends it at "no answer" — same wording, same `kErrorMs`
  display, as any other owner's failure.
- A step that gets no reply inside `LiveUdpClient::kReplyTimeoutMs`
  (8 s) ends the row at "No answer — it may not have applied", shown for
  `MenuModel::kErrorMs` (6000 ms), and releases every owner the row's
  steps had locked — the next set on that owner can start at once. A
  drone step's own wait on the sideport is capped separately, at
  `DroneLiveSetter::kDroneSettleMs` (30 s) — a backstop only, since the
  client's own reply deadline above always ends the "was it sent" half
  first.
- A row's lock covers every owner its own steps touch, not only its own
  (`osd.fc`'s On/Off touches both player and GS). Pressing a row that
  needs an owner still locked toasts "One GS change at a time" / "One
  drone change at a time" instead of starting a second one.
- A settled row shows "✓ saved" (or its own `saved_text`, such as the
  recording rows' "✓ at next recording") for `MenuModel::kSavedMs`
  (1300 ms) or `kSavedLongMs` (2200 ms) respectively.
- Any drone-owned row refuses with "Needs the drone linked" while the
  session is unlinked, checked first, before the row's own `refuse`
  (`MenuModel::press_right`). Linked, each drone row still carries
  `drone_refuse` (`menu_tree.cpp`): a drone/GS table mismatch
  (`link.vtx.table_mismatch`) refuses every drone row with "Drone
  settings differ from this GS — update", and a row whose own parameter
  has never answered refuses with "Reading the drone's settings…" until
  it has. A refused row is dimmed and reads "—"; its ▶ toasts the refuse
  text instead of opening a picker, and the same text is its help line
  while selected.
- The drone group's header line under the crumb (`drone_header`,
  `MenuModel::set_header`) carries the same table-mismatch and
  still-reading texts for the whole group, plus "Some settings did not
  answer." once a fetch gave up with `link.vtx.list == "partial"`; it
  reads empty with no snapshot, a stale session, no `link.vtx` block, or
  once every row has landed. An amber mark beside the link pill
  (`drone_pill_mark`, `MenuModel::set_pill_mark`) reads "no heartbeat"
  once the drone is linked but its MAVLink heartbeat has lapsed
  (`link.vtx.alive == false`) — not before the first heartbeat has ever
  arrived on this link, which is not itself a fault.

## The GS view refresh

maburgs' nine live keys reach the menu as one `GsLiveView`
(`gs_live_view.h`), kept current by a bare `get` on the **same**
`LiveUdpClient` `GsLiveSetter` sends sets on — that client has one
exchange in flight at a time, so a refresh sent between two steps of a
row would make the next step's own send fail "busy". `MenuSession`
therefore only ever marks a refresh pending, and sends it once no row is
applying and the client is idle.

- A GS-owned row's ▶ is refused with a "Reading GS settings…" toast until
  the first `get` has landed; while the menu is open and still unread it
  retries `MenuSession::kRetryMs` (2 s) after the previous attempt was
  sent, never while one is still outstanding -- so with maburgs down,
  each attempt first waits out the full `LiveUdpClient::kReplyTimeoutMs`
  (8 s) reply timeout, and the next one goes out the moment that settles.
- A GS-owned row pressed while a refresh is already in flight is **held**
  (one slot): the model shows it "applying" right away, and it actually
  starts once the refresh's reply lands — or, if that reply never came
  ("no answer"), it ends there at "No answer" instead of waiting out the
  same 8 s timeout a second time. A second row pressed while one is
  already held gets "One GS change at a time" rather than queueing.
- Every GS step that lands marks another refresh pending, so the view
  catches up with what the menu itself just set with no separate manual
  read.
- `parse_get_all` never throws on a dump it only partly understands: an
  unknown key, a line it cannot split, or a ladder it cannot parse is
  skipped, and whatever it did read still lands.

## Display follows the camera

The camera rate every piece below judges by -- the follow rule, the
Video `Display` row's own `camera_hz` dependency, and the menu's
`camera_hz_now()` -- is `follow_camera_hz` (`display_follow.h`): the fps
of the drone's reported `VENC_MODE` once the sideport holds one,
overriding the decode pump's own measured cadence outright, with no
cross-check between the two. The reported mode wins because the measured
rate lags a real camera change by a whole averaging window and reads low
under loss; with no sideport, or no `VENC_MODE` value yet, the measured
rate is all there is. Only the mismatch banner's own printed text (below)
still names the measured rate rather than this reported-first one, so the
two numbers can disagree in the rare case a stale measurement and a fresh
`VENC_MODE` report point at different fps.

`DisplayFollower` (`display_follow.h`) judges, once a second, whether the
presenter should move to a different connector mode — never while
`screen_mode` is pinned to `WxH@R`, never while the drone reports low
power (it drops its own fps on purpose; the screen should sit still),
never before the camera's rate has been measured, and never with an empty
mode list (a failed EDID read keeps the running mode rather than looping
the presenter rebuild forever). Otherwise it follows whenever the running
mode does not fit the want, is smaller than the pick, or sits further
from the camera's rate than the pick does — so `"auto"` also climbs to a
larger mode at a rate the panel already runs, not only on an outright
mismatch. A rebuild only fires once three consecutive one-second windows
agree on the same target mode, so one bad window (a fade, a camera change
mid-window) never blanks the screen by itself; a rebuild whose presenter
then lands on some other mode is not reissued for that target until the
pick settles or moves elsewhere. The rate a presenter build resolves
against stays at the last one seen out of low power, so a live
`screen_mode` set (or a replug) during low power never moves the display
to the low-power rate.

Video's `Display` row's "≠ camera" warning uses the same distinction: it
says nothing when the mode list is simply empty, and warns only when the
display has modes but none of them sit at the camera's rate (the same
reported-first `follow_camera_hz` the follow rule itself uses).

**The same mismatch drives the menu's banner,** not only the `Display`
row's warning -- but only when the menu itself is on (buttons wired, or
`MABUR_MENU_DEBUG`); a unit with no menu never shows the banner, only the
OSD fps cell's own mismatch mark. Each one-second window that finds a
mismatch (and does not itself trigger a rebuild) sets the banner to:

```
⚠ Camera <src> fps — this display has no <src> Hz mode. Showing at <shown> Hz (judder).
```

(`<src>` the decode pump's own measured rate, not necessarily the
reported-first rate the mismatch itself was judged on above; `<shown>`
the rate the running mode actually shows at; `main.cpp`, the
display-follow window). The banner is
cleared at the start of every window and only re-set when that window
still finds a mismatch, so it tracks the condition window to window; it
is unrelated to the three-window agreement a rebuild itself needs. The
banner box (`menu_layer.h`/`.cpp`) is drawn centred, 12 % down the screen,
with the menu closed; while the menu is open it moves below the panel
and picker instead, so it never covers either.

## Before the first video frame

`maburplay` puts a startup splash on screen until the first decoded frame
(`drm_presenter.h`'s splash); with no drone linked that can be forever,
and the menu has to work the whole time, not just once video is flowing.
Opening the menu while the splash is still up takes its plane: on the
common hardware, where the OSD and the splash share the primary plane,
the menu replaces the splash outright and shows over black; where the
OSD has a plane of its own above the splash, the menu simply draws on
top of it and the splash stays lit underneath. On the shared-plane
hardware that repoint is the display's only visible change — the
splash is still considered "up" until a real video frame arrives, so
the display's one-time hand-off onto the video plane (which also
restates the video plane's stacking order and the colour-correction
matrix, and only then frees the splash's buffer) still runs exactly
once, whether the menu happens to be open or closed at that moment.
Closing the menu again before any frame has arrived brings the splash
straight back, repainted fresh (the `show_splash` lambda, `main.cpp`,
reusing the same still-allocated buffer); closing it after a frame has
arrived does nothing special, since that hand-off already ran.

The decision is a small, host-tested rule, `SplashMenuGate`
(`gs/player/src/splash_menu.h`), ticked every housekeeping iteration from
the menu's open/closed state and whether a video frame has ever been
decoded; `DrmPresenter::set_osd_over_splash()` is the one knob it drives,
and `commit_osd_if_pending()` reads it to let a standalone OSD commit
through despite the splash. Each switch logs once: `maburplay: menu:
over splash` on open, `maburplay: menu: splash restored` on a close that
still precedes the first frame.

## Config

`[input.<button>]` — one table per GPIO line: `input.rec` (the DVR
toggle) plus the five menu buttons `input.up`/`down`/`left`/`right`/`sel`.
Each takes `pin` (the header pin number, required), `active_low` (default
`true`) and `bias` (`"pull-up"` | `"pull-down"` | `"none"`, default
`"pull-up"`). No table for a given button means that button is not
wired; the five menu buttons are per-unit wiring and have no sensible
shared default, so the shipped bundle leaves all five commented out.

`[menu]` has two keys, both persisted across a boot and both reaching the
running menu on its next tick when set live on 8403 — no presenter
rebuild, no recording touched:

- `show_advanced` (bool, default `false`): whether the menu shows its
  advanced rows.
- `scale` (`"normal"` | `"large"` | `"larger"`, default `"normal"`): the
  menu's size, ×1, ×1.25 or ×1.5 ("Screen anatomy" above). Any other
  value fails the load. A live set re-lays the menu out; the GS's
  `maburplay.toml` must already carry the key for the set to be written
  (`docs/gs-live-config.md`).

## Debug verbs (`MABUR_MENU_DEBUG=1`)

With `MABUR_MENU_DEBUG` set in maburplay's environment (checked once at
startup, never in a `--decode-only` run), the menu is built — and claims
the OSD plane — even with no `[input.*]` button table at all; the button
bank simply never opens, so a bench script can drive the whole menu over
the 8403 listener with no hands on real buttons and no eyes on the
goggles. Three verbs are offered to 8403's `extra_verb` hook alongside
the ordinary `set`/`get` (`menu_debug.h`):

`key` and `view` both answer `err menu off` outright when
`MABUR_MENU_DEBUG` is set but the menu itself did not come up (e.g. the
GS font it borrows failed to load) — checked before anything else, so
neither verb ever lies with an `ok` the menu cannot act on
(`handle_menu_debug_verb`, `menu_debug.cpp`).

- `key <up|down|left|right|sel> <tap|hold|repeat>` — with the menu on,
  queues one key event for the main loop's next iteration, the same
  queue a real button press would land in, and answers `ok` at once (the
  event itself runs on the next tick, not synchronously with the reply).
  An unparseable verb answers the usage line with the `err ` prefix:
  `err key <up|down|left|right|sel> <tap|hold|repeat>`.
- `view` — with the menu on, answers `ok` followed by a stable,
  line-oriented dump of the menu's current view (`menu_view_text()`):
  `open`, `crumb`, `linked`, one `row` line per visible row (marked `> `
  when selected, with its id/label/value/warn/status and a comma-joined
  flag list — `dim`/`busy`/`vtx`/`adv`/`ok`/`err`), `help` (plus
  `help_error 1` for an error help line), `keys`, `picker`
  (`none`/`choices`/`number`/`confirm`) and, for a choices picker, one
  `choice` line per entry (`>` marks the picker's cursor, `*` the current
  value), `number`, `confirm`, `toast` and `banner`. Every field always
  prints, even empty, so a line's absence always means the view itself
  lacks it.
- `snap` — independent of whether the menu is on at all (it reads the
  OSD plane, not the menu): writes the OSD plane's last published buffer
  to `/tmp/maburplay-osd.pam` (PAM P7, `TUPLTYPE RGB_ALPHA`, straight
  alpha un-premultiplied from the plane's own premultiplied pixels),
  written to a `.tmp` path and renamed on, so a reader never opens it
  mid-write; answers `ok /tmp/maburplay-osd.pam`, or `err no osd
  surface` before the first real frame.

### Linked-drone walk-through

A manual check of the drone rows end to end, run from a shell on the GS
(`ssh root@$GS`) with the drone linked and `MABUR_MENU_DEBUG=1` set in
maburplay's environment:

a. `printf 'view' | nc -u -w1 127.0.0.1 8403` right after link-up: the
   Video group's drone rows carry real values and the header line reads
   empty (the table has already settled); caught mid-fetch instead it
   reads "Reading the drone's settings…".
b. Queue Mirror's row, ▶, confirm: the row settles "✓ saved", the
   picture flips, and `tools/bench/ausniff.py` shows the rebuild gap
   followed by zero resyncs.
c. Camera mode → a native 1920×1080 choice at the display's own rate:
   "✓ saved", the display stays at that rate, and the picked zoom shows
   in the choice's own label.
d. Low power when disarmed On with the FC link off: the confirm reads
   "Also turns the FC link on.", the row goes "applying 1/2" then
   "2/2", and both rows read On once it settles.
e. VTX SD recorder On: the three sub-rows (SD bitrate, SD frame rate, SD
   size) appear once the re-list after the set lands.
f. `printf 'vtx set ENC_BITRATE_MAX abc' | nc -u -w1 127.0.0.1 8402`
   answers `err bad value for int32` straight from maburgs' own
   refusal (`ParamClient::set`), before any `DroneSetWatch` is even
   armed, and leaves the menu's own rows untouched.
g. Power the drone off mid-set: the row ends either at "No answer — it
   may not have applied" (`LiveUdpClient::kReplyTimeoutMs`, 8 s) or
   `err link lost` (the sideport going stale, caught by
   `DroneSetWatch::check`); either way the next drone row is usable
   again once the drone relinks.

## Where the code lives

`menu_model.h`/`.cpp` — the pure state machine: button events in, a
`MenuView` and the `SetStep`s to run out. `menu_tree.h`/`.cpp` — the rows
themselves, built from live state through `MenuTreeDeps`, including the
drone rows' `drone_choice`/`drone_bool`/`drone_number`, their shared
`drone_refuse`, and the `drone_header`/`drone_pill_mark` pair the group
header and link-pill mark read. `menu_layer.h`/`.cpp` — paints a
`MenuView` into the OSD surface. `menu_apply.h`/`.cpp` — runs a row's
steps against the player's own live-config engine, maburgs' 8402, or the
drone: `DroneLiveSetter` is the drone side of that, and `DroneSetWatch`
inside it is the sideport settle rule ("Applying a row" above).
`menu_session.h`/`.cpp` — ties button events, the applier and the GS view
refresh together, and owns the `maburplay: menu: open` / `close` log
lines. `button_bank.h`/`.cpp` and `key_timer.h`/`.cpp` — the five GPIO
lines and their gesture timing. `gs_live_view.h`/`.cpp` — parses maburgs'
`get` dump. `display_follow.h` — the display-follows-camera rule,
including `follow_camera_hz`'s reported-mode-first pick. `menu_view_feed.h`
— paces `MenuModel::view()` for the presentation loop. `menu_debug.h`/
`.cpp` — the `MABUR_MENU_DEBUG` verbs. `splash_menu.h` — the
menu-before-any-video decision rule (`SplashMenuGate`, "Before the first
video frame" above); `DrmPresenter::set_osd_over_splash()`/
`commit_osd_if_pending()` (`drm_presenter.h`/`.cpp`) act on it. Wiring
(opening the bank, loading the GS font the menu borrows, claiming the OSD
plane, the startup/debug log lines) is in `gs/player/src/main.cpp`.

`common/include/mabur/camera_modes.h` — the drone's sensor-mode table
(rate, binned/native, sizes, zoom label) the Camera mode row's choice
list and labels are built from; shared with the web VTX tab's own copy of
the same table, kept equal by a test. `gs/src/param_host.h`/`.cpp`
(maburgs) — `ParamHost`, the hosted MAVLink parameter client against the
drone's component: the `link.vtx` sideport block this whole menu reads
(`docs/observability.md` "`link.vtx`"), and the `vtx set` verb it answers
on 8402 (`docs/gs-live-config.md` "Protocol"). Each submitted set and its
terminal outcome reach maburgs' stderr as `maburgs: vtx set <ID> <text>`
and `maburgs: vtx <ID> accepted` / `maburgs: vtx <ID> failed <status>`
(`ParamHost::take_log_lines`) — a link drop mid-set that never reaches a
terminal state logs nothing there.
