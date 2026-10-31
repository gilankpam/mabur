# GS live config — `set`/`get` on 8402/8403

Both ground-station daemons take a small, closed set of config keys live:
an operator sends one `set <key> = <value>` to a loopback UDP listener,
the change reaches the running process, and — only once the process has
taken it — it is written into the daemon's own TOML file. `maburgs`
listens on `127.0.0.1:8402`, `maburplay` on `127.0.0.1:8403`, next to
`CalControl` (`8400`) and `RecControl` (`8401`).

The pipeline both daemons run — re-read the file, patch the key's line(s),
load the result with the boot loader as a candidate, apply the candidate
to the process, publish — is one shared engine,
`mabur::LiveApplyEngine<Config, Key>` (`common/include/mabur/live_apply.h`),
also used as `mabur::LiveConfigControl<Applier>` for the listener itself.
Each daemon wraps it with its own hook struct and key table:
`gs/src/gs_config_apply.{h,cpp}` (`GsConfigApplier`, `GsConfigControl`) and
`gs/player/src/player_config_apply.{h,cpp}` (`PlayerConfigApplier`,
`PlayerConfigControl`). The bench driver for both is
`tools/bench/gsparam.py`.

## Protocol

One UDP datagram in, one reply out:

```
set <key> = <value>
get [<key>]
```

- `set` answers `ok` once the file holds the new value (or settles with
  nothing to write — see "Same-value sets" below), or `err <reason>`.
- `get <key>` answers `ok <value>`; a bare `get` answers `ok` followed by
  one `<key> = <value>` line per live key, in the key table's own order.
  `get` always answers from the **live, in-process copy**, never the
  file.
- Errors: `err busy` (a set is already in flight on this daemon), `err not
  live: <key>`, the usage line (`err usage: set <key> = <value> | get
  [<key>]`), or `err line too long (max 2048 bytes)` for a datagram larger
  than the listener's buffer (detected with `MSG_TRUNC`, so an oversized
  datagram is refused outright rather than acted on as its truncated
  prefix).
- Both listeners bind `127.0.0.1` only: a `set` requires being logged into
  the GS itself. There is no other authentication: any process on the GS
  can set any live key.
- One set in flight per daemon at a time: the applier's own `busy` refusal
  keeps a second `set` from queueing, and `LiveConfigControl` keeps only
  one pending sender address.

**Same-value sets.** A `set` whose value text already equals the live
value's own rendering answers `ok` and writes nothing — caught either at
submit (the raw text matches) or, for a value spelled differently
(`0.50` against a live `0.5`, a respaced ladder), once the candidate has
loaded and its rendering is compared against the live one. Neither path
touches the file or the `.pre-live` backup. A set whose text is the live
value's rendering is not refused either: it skips the precondition checks
(a pin double-sent while its relocate is in flight answers `ok`).

So a set to the current value cannot retry something the process did not
reach. When `debug_log.enable = true` failed to open its session at boot
(the live value still reads `true`), or a pin's relocate did not land (the
live value reads the pin, the link sits elsewhere), set the key away and
back: `false` then `true`, or `"auto"` then the channel.

## maburgs — `kGsLiveKeys` (`gs/src/gs_config_apply.cpp`)

| Key | Type | Applies to | Notes |
|---|---|---|---|
| `radio.channel` | int or `"auto"` | `ChannelCore::set_pin` / `set_auto` | An order placed only *after* the file holds the new value (`order_after_publish`), not an apply before it. A pin to a member is checked at submit, before any file work, by `ChannelCore::pin_refusal` — everything `set_pin` refuses, in its words: `relocate in flight`, `hop in flight` (a one-card hop still riding its RCFs included), `calibration running` (a run, its measure round or its relocate), and, for a linked move off the current channel, `sweep in flight` or `hop cap`. A channel that isn't a member, or a value that isn't a channel, is the loader's refusal, in its message. `"auto"` is never refused (`set_auto` withdraws a pin relocate not yet placed). A set to the live value (the pinned channel, or `"auto"` while auto) is a same-value `ok` with no check and no file write. `ok` on a linked pin means the order was accepted and the file holds it, not that the link has moved: the relocate runs afterwards, and one that does not land (the drone does not follow, the link drops) leaves the GS pinned with the link on the channel it was on, while `get` and the file read the requested one. The sideport shows it as `scan.pick` (the pin) ≠ `link.channel` (the link), and stderr logs `relocation to <want> did not land; staying on <op>`. An order still refused once the file is published (a hop started between the check and the order) re-publishes the previous file text under the still-held write lock; if that restore fails too, the reply is `err <reason>; restoring the file failed: <error>`. |
| `link.static_mcs` | int | `VrxController::set_pin` | `mcs < 0` is adaptive (the ladder drives the operating point); `mcs >= 0` bypasses the ladder with a fixed rung. Applied together with `static_bw`/`static_overhead_base`/`static_overhead_enh` as one call. Back to adaptive from a pin re-enters the ladder on the rung with the pinned `{mcs, bw}`, else rung 0 — an `E … unpin` ctl line — with the feedback timeout counted from that moment: the ladder gets no feedback while pinned, and that is not a feedback loss. |
| `link.static_bw` | int | `VrxController::set_pin` | |
| `link.static_overhead_base` | real | `VrxController::set_pin` | |
| `link.static_overhead_enh` | real | `VrxController::set_pin` | |
| `link.ladder` | array of inline tables | `ChannelCore::replace_ladder` | Rebuilds the ladder controller inside the session (`VrxController::replace_ladder`). The old rung's `{mcs, bw}` is looked up in the new ladder: present → restored to that index; absent → rung 0. The new controller keeps the old one's last feedback time, so the feedback timeout does not fire on the swap. A hop's restore rung, frozen as a ladder index at the impairment onset, is translated the same way (by `{mcs, bw}`, else rung 0), so a hop after the swap restores the pre-onset operating point. A rung the loader would silently drop for being above `link.max_mcs` instead refuses the whole set (`rung <i> exceeds link.max_mcs`), rather than flying a ladder shorter than what was asked for. The rewrite replaces the first run of `[[link.ladder]]` blocks in the file; a second run further down (which the loader would append) refuses the set with `the config file holds more [[link.ladder]] rungs than the set`. Also re-headers `ctl.log` — see below. |
| `link.nack.enable` | bool | `NackTracker::set_enable` | Turning it off also clears the tracker's pending retransmit requests; its request counter is kept (not reset) so a later re-enable stays monotonic. |
| `debug_log.enable` | bool | `DebugSession::enable`/`disable` + opening/closing the six per-session log files | A failing `enable` (the log root is unwritable, e.g. `/media/dvr` missing) reports `debug_log.dir <dir> unusable` and closes nothing, opens nothing — the running session, if any, is untouched. |
| `msp.enable` | bool | build or drop the MSP OSD sink (the MSP stream's UDP feed to `maburplay`, `[msp.out]`) | |

The ladder value is one line: an inline array of inline tables with
exactly the four keys `mcs`, `bw`, `overhead_base`, `overhead_enh` in any
order, e.g.:

```
set link.ladder = [{mcs = 0, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, {mcs = 2, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}]
```

mabur's TOML parser has no inline-table support, so this shape is checked
by hand (exactly those four keys, no duplicates, no nested table, a
non-empty array) rather than by the shared parser; the submitted rungs are
then written out in the bundle's `[[link.ladder]]` block layout and
re-parsed that way to validate value syntax. **Comments inside the
replaced `[[link.ladder]]` region — between or inside the old rung blocks
— are not preserved.** Only the run of blank/comment lines that leads into
the *next* table (by position, not ownership) survives the replace, since
it belongs to whatever follows, not to the ladder.

## maburplay — `kPlayerLiveKeys` (`gs/player/src/player_config_apply.cpp`)

| Key | Type | Apply rule | Notes |
|---|---|---|---|
| `screen_mode` | string | Presenter rebuild | Refused as `mode not offered: <mode>` before anything is torn down, when the connector's current mode list does not carry it. |
| `colortrans.enable` | bool | Presenter rebuild | The LUT/CTM setters are call-once per presenter, so a toggle has to recreate it (`docs/colortrans.md`). |
| `osd.enable` | bool | Presenter rebuild if it changes whether *any* OSD plane is wanted, else an in-place layer open/close | Switching an overlay on loads its font first; a missing font refuses the set before anything else changes. |
| `osd.gs.enable` | bool | same rule as `osd.enable` | |
| `dvr.target` | string (`"gs"`/`"vtx"`/`"both"`) | read at the next recording's start | A target set while a recording is running is latched for the *next* recording; the GS bar's REC field, sized for the recording it shows, is re-laid out once the running one stops. |
| `dvr.mode` | string (`"raw"`/`"burned"`) | read at the next recording's start | `"burned"` is refused on a build with no MPP hardware encoder (`unsupported_on_build`) — the Orange Pi image always refuses it, live exactly as at boot. |
| `dvr.burned.bitrate_kbps` | int | read at the next recording's start | |
| `dvr.burned.every` | int | read at the next recording's start | |
| `dvr.burned.size` | string (`"WxH"` or `""` for the decoded size) | read at the next recording's start | |

A running recording keeps the plan (`RecPlan`: `gs`/`vtx`/`burned` and
the `dvr.burned.*` size, cadence and bitrate) it was started with; a
`dvr.*` set never changes a recording already in progress, including a
burned recorder restarted inside it (below), which starts with the
latched settings. A presenter rebuild (`screen_mode`, `colortrans.enable`, or an
OSD switch that changes the plane) restarts a *burned* recording in
progress — new file, no video park — since the recorder's palette is
fixed at its start from the overlays live then. Switching an overlay *on*
in place, with no presenter rebuild, restarts a running burned recording
for the same reason (its palette would otherwise be keyed on the overlays
present when it started); switching one *off* in place needs no restart,
since the composer's erase reaches the recorder's map directly.

A presenter rebuild itself also counts as an **output park**
(`park_output("display rebuild")`, `gs/player/src/main.cpp`): it bumps the
regulator line's `flushes=` counter the same way a display hotplug does,
so a latch gate (`tools/bench/latchgate.py`) excludes it rather than
reading it as a missed latch.

## File rules

- **`.pre-live` backup, written once per daemon run**, before the first
  successful publish — the same `.pre-*` operator-rollback convention as
  `maburcal`'s `.pre-cal` (`docs/calibration.md`, `docs/deploy.md`).
- The candidate scratch file is `<realpath>.new`, written beside the
  config's **resolved** target — the same scratch name the drone's own
  config writer uses (`common/include/mabur/config_write.h`).
- **The publish resolves symlinks.** `maburplay`'s `/etc/maburplay.toml`
  is typically a symlink onto a vfat `/config` mount; every step —
  reading the original, writing the scratch, the backup, the rename —
  happens beside the symlink's *target*, and both the file and its parent
  directory are fsynced. The symlink itself is never unlinked or
  replaced.
- **`ok` means the file holds it.** A `set` only answers `ok` once the
  publish has actually landed (or settled with nothing to write, per
  "Same-value sets"). A refused or failed set leaves the file
  byte-identical and writes no backup. A publish that fails after a
  successful process-side apply re-applies the previous config to the
  process, putting it back in line with the unchanged file.
- **One set in flight per daemon** — see Protocol, above (`err busy`).

## Threading

Each daemon does the file work on a dedicated worker thread (`mbr-cfgset`
on `maburgs`, `mbr-plcfgset` on `maburplay`): it re-reads the file,
patches and loads the candidate, and — once told to — publishes it,
holding `lock_config_writes()` the whole time, from the re-read through
the publish (and through any restore of the previous text). The apply
itself runs on the owning thread, below.

The **owning thread** — `maburgs`' core loop, `maburplay`'s housekeeping
loop — is the one that actually changes the running process: it polls the
worker's phase each tick, applies an accepted candidate (or reverts to the
previous config after a failed publish), and never blocks on file I/O or
on the worker directly.

**The owning thread must never take `lock_config_writes()` itself, nor
call `publish_config()` (which takes it).** The worker holds that lock
while it waits for the owning thread's next step, so an owning thread
blocked on the same lock would be waiting for a worker that is waiting for
it — a guaranteed deadlock. Any publish the owning thread needs for a
reason other than this engine belongs on a thread of its own.

## The log line

One line per `set`, logged by the applier (not the listener, so nothing
is logged twice), timed from submit to result:

```
<daemon>: cfg set <key> = <value> -> ok (<ms> ms)
<daemon>: cfg set <key> = <value> -> err <reason> (<ms> ms)
```

A publish failure that forces a revert also logs
`<daemon>: cfg set <key>: publish failed, reverting` and, if reapplying
the previous config to the process itself fails, `... revert failed:
<reason>`; a restore of the previous *file* that fails after an order was
refused post-publish logs `... restoring the previous file failed:
<reason>`, and the reply carries it too (see `radio.channel`).

## The ladder header rule (`ctllog 13`)

A `set link.ladder` rebuilds the ladder controller inside the session
(`ChannelCore::replace_ladder`) and writes a fresh `ctllog 13
<header_info>` line into `ctl.log` as the very next line, right after the
apply — so every row logged after the swap is read against the new
ladder's header, not the boot one. `CtlLog::header()`
(`gs/src/ctl_log.{h,cpp}`) is the same marker form the file's first line
uses (not a `#`-prefixed comment), so a reader reads each row against the
most recent `ctllog` line above it — which also covers a rejoined file.
`flightreport.py` stamps every row with that header's ladder (its DWELL
table and BW40 time read a pre-swap row on the old ladder);
`tools/bench/switchloss.py` replaces its ladder on every `ctllog` line.
The swap's rung re-entry follows the header as an `E <t_ms> 0 <idx> ladder_swap …`
line (from the new controller's rung 0 to the restored index), so hop
analysis, which pairs `hop_restore` lines with hop orders, never takes a
swap for a hop's restore. The swap also re-arms the probe/penalty
change-detect from the new controller's zeroed counters, so it writes no
phantom row. A log rotation (a drone restart mid-flight) after a swap
restates the swapped ladder's header in the new session file, not the
boot one.

## `tools/bench/gsparam.py`

```
GS=<gs-ip> python3 tools/bench/gsparam.py gs     get [KEY]
GS=<gs-ip> python3 tools/bench/gsparam.py gs     set KEY VALUE
GS=<gs-ip> python3 tools/bench/gsparam.py player get [KEY]
GS=<gs-ip> python3 tools/bench/gsparam.py player set KEY VALUE
GS=<gs-ip> python3 tools/bench/gsparam.py --all gs
GS=<gs-ip> python3 tools/bench/gsparam.py --all player --alt-mode MODE [--opi]
```

`GS` (the ground station's ssh address) comes from the environment, with
no default. Every exchange runs over `ssh root@$GS`, which runs a short
`python3 -c` UDP client against the loopback listener on the GS itself
(busybox + python3 there): one datagram out, up to 5 s for the one
datagram back.

`--all` drives every key in the named daemon's live table away from its
bundle default and back, one key at a time (never two keys away at once),
checking at each step that the `set` answered `ok`, a follow-up `get`
reads the value back, and the daemon's TOML on the GS holds it — scoped
to the key's own `[section]`, since several leaf names (`enable` most of
all) are shared across sections. `link.ladder`'s file shape is a run of
`[[link.ladder]]` blocks rather than one line, so it is checked by block
count instead. `screen_mode` has no single generic alternate (which modes
a connector offers depends on the display), so `--all player` needs
`--alt-mode MODE`. `--opi` marks the Orange Pi build: `dvr.mode`'s
restore to `"burned"` is expected to come back `err` rather than `ok` (no
MPP encoder) and is skipped from the file check.

## What the Orange Pi refuses

The Orange Pi Zero 2W build (`docs/opi-zero2w.md`) has no MPP hardware
encoder, so `dvr.mode = "burned"` fails `unsupported_on_build` on a live
`set` exactly as it would at boot — `err dvr.mode "burned" needs the MPP
encoder; this build has none -- set dvr.mode = "raw"` — and nothing is
applied or published.
