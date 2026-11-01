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

- **Panel** — top-left, inset 24 px at 1080p (scaled by `screen_h / 1080`),
  45 % of the screen's width, up to 10 rows scrolled around the cursor,
  the crumb and link pill, two help lines and a keys line.
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
| `video.display` "Display" | player | `screen_mode` | Choices: `auto` (largest size at the camera's rate) or a `WxH` the display offers within the camera's rate. Confirm: splits a running burned recording. Warns `≠ camera` only when the display has modes but none at the camera's rate — not when the mode list itself is empty (a failed EDID read warns nothing). |
| `video.colour` "Colour correction" | player | `colortrans.enable` | On/off. Confirm: splits a running burned recording. |
| **Link** | | | |
| `link.width` "Width" | gs | `link.static_bw` (fixed) or `link.ladder` (adaptive) | 40/20 MHz. Under a fixed rate this only re-pins the width. Under adaptive it loads that width's default ladder (MCS 0,1,2,3,4 at 0.50/0.25 bw 40; MCS 0,2,4,5 at 0.50/0.25 bw 20) unless the live ladder is already one of those two tables, with a confirm naming the rung count. |
| `link.rate` "Rate" | gs | `link.static_mcs` (+ `static_bw`/`static_overhead_base`/`static_overhead_enh` into fixed) | Adaptive/Fixed. Switching to fixed sends the current rung's width and FEC overhead, then MCS last (capped at 5), so the link moves once; switching to adaptive is one set, `static_mcs = -1`, re-entering the ladder on that rung. |
| `link.mcs` "MCS" (fixed only) | gs | `link.static_mcs` | 0–5. |
| `link.fec_base` "FEC base" (fixed only) | gs | `link.static_overhead_base`(+`_enh`) | 0.10–2.00 step 0.05; a base taken below the current enh drags enh down to it in the same sets, since base must stay ≥ enh. |
| `link.fec_enh` "FEC enh" (fixed only) | gs | `link.static_overhead_enh` | 0.10–base step 0.05. |
| `link.ladder` "Ladder" (advanced) | gs | `link.ladder` | One `R<i>` group per rung: MCS (0–5), Width (40/20), FEC base/enh, Move up, Move down, Delete rung (confirm; refused at one rung left), plus "+ Add rung" below 8 rungs (copies the last rung, MCS+1 capped at 5). Every rung edit re-sends the whole ladder — the live key takes it as one unit. |
| `link.nack` "Retransmit (NACK)" (advanced) | gs | `link.nack.enable` | On/off. |
| **Recording** | | | |
| `rec.target` "REC records to" | player | `dvr.target` | GS / VTX SD card / Both. Saved text is "✓ at next recording" (2200 ms) while a recording runs, else "✓ saved" (1300 ms) — same rule on every recording row below. |
| `rec.mode` "GS recording" | player | `dvr.mode` | Burned (with OSD) / Raw (as received); hidden when `dvr.target` does not record to the GS. |
| `rec.burn.bitrate` "Bitrate" (advanced) | player | `dvr.burned.bitrate_kbps` | 2–20 Mb/s step 1; visible only in burned mode. |
| `rec.burn.every` "Frame rate" (advanced) | player | `dvr.burned.every` | Every 1st–4th frame, labelled in fps once the running mode is known; visible only in burned mode. |
| `rec.burn.size` "Size" (advanced) | player | `dvr.burned.size` | Same as video / 1920×1080 / 1280×720 / 2560×1440; visible only in burned mode. |
| **OSD** | | | |
| `osd.gs` "Link stats bar" | player | `osd.gs.enable` | On/off. Confirm: splits a running burned recording, only when turning ON (turning an overlay off never splits — the composer's erase reaches the recorder's canvas directly). |
| `osd.fc` "Flight controller OSD" | player + gs | `osd.enable` + `msp.enable` | On/off, shown as `mixed` (with a help line naming which half is off) when the render and maburgs' MSP feed disagree; picking On or Off sends both. |
| **System** | | | |
| `sys.adv` "Show advanced settings" | player | `menu.show_advanced` | On/off; shows the rows marked (advanced) above in every group. |
| `sys.debug` "Debug log" | gs | `debug_log.enable` | On/off. |

**Not covered yet:** a `Channel` group, a `Calibration` group, and any
row owned by the drone. `MenuNode::owner = Owner::kDrone` is a real value
the model understands, but `MenuApplier::start` refuses outright any row
carrying a drone-owned step — there is no drone setter wired up yet.

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
- A step that gets no reply inside `LiveUdpClient::kReplyTimeoutMs`
  (8 s) ends the row at "No answer — it may not have applied", shown for
  `MenuModel::kErrorMs` (6000 ms), and releases every owner the row's
  steps had locked — the next set on that owner can start at once.
- A row's lock covers every owner its own steps touch, not only its own
  (`osd.fc`'s On/Off touches both player and GS). Pressing a row that
  needs an owner still locked toasts "One GS change at a time" / "One
  drone change at a time" instead of starting a second one.
- A settled row shows "✓ saved" (or its own `saved_text`, such as the
  recording rows' "✓ at next recording") for `MenuModel::kSavedMs`
  (1300 ms) or `kSavedLongMs` (2200 ms) respectively.

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
display has modes but none of them sit at the camera's measured rate.

**The same mismatch drives the menu's banner,** not only the `Display`
row's warning -- but only when the menu itself is on (buttons wired, or
`MABUR_MENU_DEBUG`); a unit with no menu never shows the banner, only the
OSD fps cell's own mismatch mark. Each one-second window that finds a
mismatch (and does not itself trigger a rebuild) sets the banner to:

```
⚠ Camera <src> fps — this display has no <src> Hz mode. Showing at <shown> Hz (judder).
```

(`<src>` the camera's measured rate, `<shown>` the rate the running mode
actually shows at; `main.cpp`, the display-follow window). The banner is
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
top of it and the splash stays lit underneath. Closing the menu again
before any frame has arrived brings the splash straight back, repainted
fresh (the `show_splash` lambda, `main.cpp`); closing it after a frame
has arrived does nothing special, since the ordinary first-frame
hand-off onto the video plane already ran.

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

`[menu]` has one key, `show_advanced` (bool, default `false`): whether
the menu shows its advanced rows, persisted across a boot. A live `set
menu.show_advanced = true|false` on 8403 reaches the running menu on its
next tick — no presenter rebuild, no recording touched.

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

## Where the code lives

`menu_model.h`/`.cpp` — the pure state machine: button events in, a
`MenuView` and the `SetStep`s to run out. `menu_tree.h`/`.cpp` — the rows
themselves, built from live state through `MenuTreeDeps`. `menu_layer.h`/
`.cpp` — paints a `MenuView` into the OSD surface. `menu_apply.h`/`.cpp`
— runs a row's steps against the player's own live-config engine or
maburgs' 8402. `menu_session.h`/`.cpp` — ties button events, the applier
and the GS view refresh together, and owns the `maburplay: menu: open` /
`close` log lines. `button_bank.h`/`.cpp` and `key_timer.h`/`.cpp` — the
five GPIO lines and their gesture timing. `gs_live_view.h`/`.cpp` —
parses maburgs' `get` dump. `display_follow.h` — the display-follows-
camera rule. `menu_view_feed.h` — paces `MenuModel::view()` for the
presentation loop. `menu_debug.h`/`.cpp` — the `MABUR_MENU_DEBUG` verbs.
`splash_menu.h` — the menu-before-any-video decision rule
(`SplashMenuGate`, "Before the first video frame" above);
`DrmPresenter::set_osd_over_splash()`/`commit_osd_if_pending()`
(`drm_presenter.h`/`.cpp`) act on it. Wiring (opening the bank, loading
the GS font the menu borrows, claiming the OSD plane, the startup/debug
log lines) is in `gs/player/src/main.cpp`.
