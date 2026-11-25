# Deploying maburd / maburgs

There is no backward-compatibility obligation in mabur (see CLAUDE.md),
but a deploy still touches two devices and is never atomic. That window,
and the config-load behaviour below, are what actually bite. The dated
runbooks of past flag days are in
`docs/findings/2026-08-29-deploy-flag-days.md`.

**A version mismatch has no video at all.** Any `RC_VERSION` bump
(`common/include/mabur/rc_proto.h`) is a flag day: a drone and GS at
different versions exchange only the frozen handshake
(`docs/link-pairing.md` "Frozen handshake") and reject every other frame
in BOTH directions, so the symptom is no control link and no video.
Restarting either daemon does not fix a version mismatch; finish the
deploy. The RC wire goldens live in
`tests/test_rc.cpp`. `RC_VERSION` 18 (calibration moves onto the MAVLink
tunnel; `T_CAL_CMD`/`T_CAL_RESULT` deleted) is a flag day like every other
bump, and it also removes a drone config key: `radio.rate_walls_rel` fails
boot, and the old binary refuses its replacements. On the drone: stop
`maburd`, edit the config — rename the key to `rate_walls_rel_20` (it
holds 20 MHz walls; with no `_40` the 40 MHz rungs fly the 20 MHz table,
which `maburd` logs) and add `cal_flags_20 = "none"` (the old kit swept
with LDPC and STBC off) — swap the binary, start. Then `maburgs` with
[`gs/bundle/maburcal`](https://github.com/gilankpam/mabur-ground/blob/master/gs/bundle/maburcal) and [`tools/maburtop.py`](https://github.com/gilankpam/mabur-ground/blob/master/tools/maburtop.py) ([`gs/bundle/install.sh`](https://github.com/gilankpam/mabur-ground/blob/master/gs/bundle/install.sh)
installs all three), and re-run `maburcal start` to measure both widths
under the flags the link flies (`docs/calibration.md`).

**A config error crash-loops the daemon.** A stale, removed or unknown key
makes the daemon fail to start. A config-load error exits **1** in `maburd`
and **2** in `maburgs` (the `load_config` try/catch in each `main()`; the
two do not agree, so do not key a script off either number). Neither
daemon's wrapper (`S00mabur`, `S96maburgs`, `maburgs.service` under
systemd) checks the exit code: each respawns the daemon forever at an
unconditional 2 s delay. Both wrappers truncate the daemon's log on every
respawn, so `/tmp/mabur.log` / `/tmp/maburgs.log` holds only the current
run: one `error: <file>:<line>: <key>: unknown key` line that keeps coming
back, and a daemon that never stays down. `maburplay`'s `S97maburplay` is
the opposite: it stops respawning on exit 2 (config error) or 143
(SIGTERM), so a player config error leaves the GS screen black with no
loop to notice.

**The clean sequence:** stop both daemons, edit both configs, swap both
binaries (`df -h /` and prune first: the drone rootfs fits at most two
`maburd` binaries, and an over-the-air install briefly needs a third:
"Over-the-air update"), start both. Rollback, when you actually need one, is
PAIRED: an old binary needs its old config restored alongside it. There is
no forward or backward config compatibility, so rolling forward is usually
the shorter path. The generic checklist is "Flag-day checklist" below.

**The install scripts.** `bundle/install.sh root@<drone>` and mabur-ground's
`gs/bundle/install.sh root@<gs>` build (unless a prebuilt binary is passed
as the second argument), stop the service, scp the binary and the init
script, then check the target's config before touching it:

- a `.toml` already present is left alone ("never clobber a tuned one");
- neither a `.toml` nor a legacy `.json`: the shipped default is copied in;
- only a legacy `.json`: the script prints an error and exits 1 rather
  than seed the repo default over a tuned config (see "JSON to TOML
  cutover").

Then they start the service. Neither migrates an existing config: a key
change still has to be made by hand. What mabur-ground's `gs/bundle/install.sh` does beyond that (the
board pick, the extra tools, the stop wait, `maburplay`) is in
mabur-ground's [`docs/deploy.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/deploy.md).

**Restarting the drone daemon over ssh: use `setsid`.** `S00mabur`'s
respawn loop is a background subshell of the shell that started it, so a
plain `ssh root@drone '/etc/init.d/S00mabur start'` gives you a `maburd`
that dies with the ssh session: the daemon comes up, ssh returns, and the
link drops seconds later for no visible reason. Detach it:
`setsid /etc/init.d/S00mabur start </dev/null >/dev/null 2>&1 &`. This
applies to any wrapper started from a non-interactive ssh command.
**Never chain work after a `stop` in the same ssh command:** `stop` kills
PIDs read from `/var/run/mabur-loop.pid` and `/var/run/mabur-logwatch.pid`,
and after a long uptime a stale PID can name something else, ending the
command early with the rest silently not run. Stop in one invocation,
verify with `ps`, then swap in the next.

## Drone init script

The OpenIPC image installs the wrapper as **`/etc/init.d/S00mabur`**
(`firmware/package/mabur/mabur.mk`), so BusyBox rcS starts `maburd`
first (`docs/findings/2026-09-07-boot-time.md`). The source file in this
repo is still called `bundle/S96mabur`. `bundle/install.sh` stops whichever
of `S00mabur` / `S96mabur` exists, installs the wrapper as `S00mabur`,
removes any old `/etc/init.d/S96mabur`, and starts `S00mabur`, so one name
owns the service. (Both names share one pidfile, so a leftover second copy
does not start `maburd` twice at boot; it only leaves two names to
stop/start by.)

The wrapper respawns `maburd` 2 s after any exit, truncates `/tmp/mabur.log`
at every (re)start, and runs a logwatch loop that truncates the log again
once it passes 4 MB (`/tmp` is RAM-backed). On the GS, `S96maburgs` runs
the same respawn loop (and `rmmod 8812eu` on start, so devourer can claim
the cards), logging to `/tmp/maburgs.log`; `S97maburplay` logs to
`/tmp/maburplay.log`.

## Over-the-air update

A GS can replace the drone's `maburd` over the radio, including a GS at
another `RC_VERSION`: this is how a flag day is finished without a cable.
The wire side (messages, FTP subset, update-only session) is
`docs/mavlink.md` "Over-the-air update". The drone side is `maburd`
staging the file and `S00mabur` doing the rest.

**The update dir `/tmp/mabur-up`** (RAM, gone at reboot):

| File | Written by | Meaning |
|---|---|---|
| `maburd` | the FTP server | the staged new binary; the only file the radio can write |
| `go` | `maburd` on START | the SHA-256 START verified; the loop reads and deletes it, then installs only a staged binary that still hashes to it |
| `mabur.toml` | the loop, via the new binary's `--migrate-config` | the current config trimmed to what the new binary loads |
| `linked` | the trial `maburd` | the trial published a session |
| `proven` | the trial `maburd` | linked, and encoder frames moved for 5 s |
| `result` | `S00mabur` | `committed`, `rolled_back <reason>` or `refused <reason>`; the reason words are in `docs/mavlink.md` |
| `trial.log` | `S00mabur` on rollback | the last 256 KiB of `/tmp/mabur.log`, which the next run truncates |

**One install**, as `S00mabur` runs it after `maburd` exits for a START:

1. Read and delete `go`, and remove the stale `proven`, `linked` and
   `result`. The staged `maburd` must still hash to `go`'s SHA-256, else
   `refused bad_hash` and nothing runs from it. The new binary must
   report the same board as the running one (`maburd --identify`; a hand
   build reports `local` and takes no release update), else
   `refused wrong_board`.
2. The new binary migrates the current config into `mabur.toml`
   (`maburd --migrate-config`); a failure is `refused bad_config`. The
   config is migrated only here: a drone's own `maburd` never migrates.
3. Copy the live `/usr/bin/maburd` and `/etc/mabur.toml` to `.prev`
   (always a copy to `.prev.new`, `sync`, then `mv`, so the live files
   never go missing), then install the new pair the same way, and `sync`
   once more so both renames are on flash before the trial starts.
4. **Trial**: run `MABUR_TRIAL=1 maburd -c /etc/mabur.toml` for up to
   40 s. `proven` before then: `result` is `committed` and the new pair
   stays. The trial exiting: `rolled_back exited`. 40 s with no `proven`:
   `rolled_back no_link` (never linked) or `rolled_back no_video`.
5. **Rollback** keeps the failed trial's log in `/tmp/mabur-up/trial.log`,
   moves `/usr/bin/maburd.prev` and `/etc/mabur.toml.prev` back over the
   live pair, then the loop respawns as usual.

After a commit the trial `maburd` keeps running as the drone's daemon and
takes the next update the same way; a second update needs no reboot.

Overlay space: during an install the overlay briefly holds **three**
copies of `maburd` (live, `.prev`, and `.new`), and after a commit two
(live and `.prev`). Size the overlay for that, and prune other `maburd.pre-*`
copies first. A failed install (no space) reports `rolled_back exited` and
leaves the old pair running; nothing live was replaced.

**Manual recovery** over ssh, when a drone is left on a bad pair and the
loop did not roll back:

```sh
mv /usr/bin/maburd.prev /usr/bin/maburd
mv /etc/mabur.toml.prev /etc/mabur.toml
/etc/init.d/S00mabur restart    # detach it as under "Restarting the drone daemon"
```

**Bootstrapping and `sysupgrade`.** A drone can take an OTA only if its
`S00mabur` and `maburd` already have it: the first OTA-capable install is
`bundle/install.sh` from this release, or `sysupgrade -n` of this
release's image. A `sysupgrade` after an OTA keeps the overlay binary
(`/overlay/root/usr/bin/maburd`) over the image's: use `sysupgrade -n`, or
delete `/overlay/root/usr/bin/maburd` and `maburd.prev` first, or the
drone keeps running the OTA'd build. The release's per-board `maburd`
asset (`docs/releases.md`) is what the GS pushes.

## Supported drone cards

`tools/build-arm.sh` builds maburd for the **RTL8812EU** only (devourer
rtl8822e) — the bench drone's card. The **RTL8812CU** (rtl8822c) is
supported through a separate firmware image, the board
`ssc338q_fpv_emax-wyvern-link-alpha` (`firmware/devices/`), whose `mabur.mk` builds devourer with
the 8822C alone (`BR2_PACKAGE_MABUR_RADIO_8812CU`). One image, one chip: a
binary on the wrong card fails at `CreateRtlDevice` ("unsupported chip").
The shipped `usb_pid = 0` scan tries `a81a`, then `c812`/`c82c`, then the
8812AU ids `881a`/`8812` (`drone/src/maburd/radio_glue.cpp`), so the same `mabur.toml`
boots either board; the `opened device 0bda:XXXX` line in `/tmp/mabur.log`
says which card was found. GS cards stay 8812EU.

The first behavioural difference is **TX power**. Mechanism is identical
(same TXAGC block, same per-rate diffs, same offset), so maburcal and
`power_mode = "offset"` work unchanged (`docs/calibration.md`) — but the
anchor the relative walls ride on is not:

- **8812EU**: per-channel efuse reference (39 on ch136, 53 on ch149 on the
  bench unit), which absorbs the channel-to-channel gain difference — one
  wall table covers every channel.
- **8812CU**: devourer's flat reference, index 40 on every channel, no
  efuse per-channel correction and no efuse rate shape. With the shipped
  `power_mode = "none"` the card flies that flat, uncalibrated power. After
  maburcal the walls are exact on the calibrated channel and approximate
  (a few dB, unmeasured) elsewhere, e.g. after an auto-select move or an
  in-flight hop. Calibrate on the channel flown most. Reading the CU's
  efuse per channel is a devourer change, worth it only if a mismatch is
  actually seen.

The second is devourer's **wide-TX IGI floor**
(`DeviceConfig::Tuning::wide_tx_igi_floor`), a DIG lower bound that is
switched on by the session's first 40 MHz frame and stays on while the card
is tuned to 40 MHz. It exists because on a silent channel the receiver can
settle where the secondary 20 MHz never reads idle, and the MAC then holds
the first 40 MHz frame forever. Every IGI step above the DIG window costs
the RCF uplink idle-channel sensitivity, so `maburd` starts low and raises
the floor only while 40 MHz sends stall. The starting floor is per image,
from the one Jaguar3 chip that image compiles in:

- **8812EU**: `0x22`.
- **8812CU**: `0x26`. The stall was found on the 8822C.

The cap on both is `0x35`.

A watchdog thread (`drone/src/igi_floor_watchdog.h`, fed by
`drone/src/send_timer.h`) raises the floor in flight. It times every video
send, on the sender pool and on the direct path, and judges one-second
windows. A window is slow when its longest radio send is 30 ms or more, or
when the chip's TX queue is 60 % full or more at its deepest in the window
(the queue is read every 100 ms; frames can back up inside the chip while
USB sends still return quickly). A window that is not slow is clean when
the chip queue stayed under 30 % (a healthy link peaks near 22 %), and
murky between 30 % and 60 %. Two slow windows among the last three judged
ones within 30 s start an episode; murky and clean windows do not count
toward it.

- The floor steps up by `+2`, then `+4`, then `+8`, then goes straight to
  the cap, never past it. After each step it waits 300 ms to settle, then
  judges the next window.
- During the climb a slow or a murky window raises again: a level that only
  partly helps is not proven. Three clean windows in a row keep the level;
  the episode ends and the failure hold resets.
- If the floor is at the cap and a window is still slow or murky, or no
  window is judged for 3 s (the link is gone, the rung dropped to 20 MHz,
  or the channel is busy), the climb is undone: the floor returns to the
  level the episode started from, and a new episode waits out a hold of
  30 s that doubles at each failure up to 300 s.

A busy channel gates the judging. Foreign airtime stalls sends and fills the
chip queue exactly as an IGI stall does, and a higher floor does not cure
it, so a window in which the channel's busy airtime reads 20 % or more is
not judged at all: it neither counts slow, nor clean, nor starts or keeps
an episode. The thread arms one 240 ms busy-airtime window a second on the
radio and reads it at the next second; a reading is used for at most 2 s,
and a retune or a tick that is not eligible drops it. The accepted risk: an
IGI stall on a channel with 20 % or more foreign airtime is not recovered.

Only windows in which video airs at 40 MHz, the radio is up, no
calibration is running and no retune happened are judged; on any other
tick the thread reads neither the chip queue nor the busy airtime. A kept
level lasts for the process and survives hops, and is never lowered, in
failsafe or otherwise. An unproven climb is undone by the rules above in
any link state, failsafe included. The 20 MHz ladder never arms the floor
(devourer applies it only while tuned 40 MHz wide), so the shipped
all-20 MHz ladder sees no floor, and the watchdog judges nothing and makes
no chip reads. With the cap at or below the base the thread is not
started. The watchdog takes no `maburd` lock: a sender stalled in the chip
holds `tx_gate` and the sink lock, and the watchdog exists to notice that.

The log lines (stderr):

```
maburd radio: IGI floor base 0x22 cap 0x35            (once at start; " (env)" appended when an override is set)
maburd radio: IGI floor 0x22 -> 0x24 (longest send 87 ms, chip queue 91 %, busy 0 %)
maburd radio: IGI floor 0x24 kept
maburd radio: IGI floor 0x35 -> 0x22 reverted, no help; holding 30 s
maburd radio: IGI floor watchdog off (radio has no runtime floor)
maburd radio: IGI floor 0x24: USB error on the IGI write, DIG applies it
maburd radio: IGI floor watchdog: no chip queue reading, sends only
maburd radio: IGI floor watchdog: no busy reading, busy gate off
```

`watchdog off` means the radio refuses a runtime floor and the thread
stops; `USB error` means the write failed on USB but the floor is recorded and
the periodic DIG tick enforces it; `no chip queue reading` means the radio
cannot report its queue, so only slow sends count (logged once). A failed
queue read counts as an empty queue for that tick, with no log line. `no busy
reading` means the radio cannot arm a busy window, so the gate is off (logged
once); a failed busy read leaves that second unknown, with no log line. The
`stats:` line carries `igi_floor=0x%02x`, the level currently set, after
`txq_drop=`: it starts at the base and changes only on a raise or a revert.

For a bench run, `MABUR_IGI_FLOOR_BASE` and `MABUR_IGI_FLOOR_CAP` override the
start and the cap (decimal, or hex with `0x`; an invalid value is ignored,
one above `0x7f` too). A cap equal to or below the base turns the watchdog
off, leaving a fixed floor at the base.

The 8812CU is 2T2R, so STBC airs on two real chains; none of the 8822E
constraints in devourer's 8822E quirks page
(`third_party/devourer/docs/8822e-quirks.md`: spur channels, single-path
1SS TX, DPDT front end) apply to it. **The 8812CU path, its `0x26` start and the
watchdog on it, are not hardware-verified with `maburd`**; the only
on-hardware check of the CU is an external tester's rig.

## Config format

The three configs are TOML. The parser (`common/src/toml.cpp`) accepts a
deliberate subset and rejects everything else with a `file:line` message
rather than reinterpreting it:

| Accepted | Rejected |
|---|---|
| `# comment`, own line or trailing | inline tables `{ a = 1 }` |
| `[table]`, `[table.sub]` | dotted keys in assignments (`a.b = 1`) |
| `[[array.of.tables]]` | literal `'strings'`, multi-line `"""` |
| `key = "basic string"` (`\\`, `\"`) | dates, hex, underscored ints |
| `key = 149`, `-24`, `1.0` | arrays of arrays, mixed-type arrays |
| `key = [1, 2, 3]`, multi-line, trailing comma | duplicate keys, table redefinition |
| `key = true` / `false` | everything else |

An int loads into a float field (`airtime_budget = 1` is fine); a float
never loads into an int field (`symbol_size = 332.0` fails).

Two ordering rules TOML imposes: top-level scalars must precede the first
`[table]` header, and once `[[link.ladder]]` opens you cannot go back to
adding `[link]` scalars.

At startup each daemon prints the known keys the file did not set, with
the value they fell back to (`config: N key(s) defaulted:`). After
hand-editing a config, read that list: anything unexpected in it is a knob
you dropped.

**A defaulted key needs no line for MAVLink or `maburcal` to set it.** A
parameter set (`PARAM_EXT_SET`) or a calibration patches the key's line
when there is one and inserts it at the end of its table otherwise,
creating the `[table]` when that is missing too
(`set_toml_key`, `common/include/mabur/config_write.h`).

**`maburd --migrate-config <in> <out>`** trims a drone config to what that
binary needs, judging every line with its own loader: a key it rejects as
unknown is removed (its whole table when the name is a table), and a line
whose removal leaves the loaded config equal is removed as a default. The
per-drone keys — `radio.channels`, the calibration results
(`radio.rate_walls_rel_20`/`_40`, `[]` included, `radio.cal_flags_20`/`_40`,
`radio.legacy_wall_rel`), `link.key_file`, `venc.sensor_bin` — are never
removed, even when they equal today's default. Every
other line, comments included, is kept as written. Each removal is one
`migrate: removed unknown <key>` or `migrate: removed default <key> =
<value>` line on stderr. `<out>` is written (`<out>.new` + rename) only when
the result loads; any other loader refusal (a bad value, a line the parser
rejects) exits 1 with `migrate: <reason>` and writes nothing. `<in>` is
never written. It opens no radio, MI or USB, so it is safe beside a running
`maburd`. It is a command you run; `maburd` itself never migrates, so an
unknown key still fails boot.

**The drone's tuning is built into `maburd`; its file holds only what
belongs to that drone.** `bundle/mabur.default.toml` sets five keys —
`radio.channels`, `radio.rate_walls_rel_20 = []` and `_40 = []` (not
calibrated), `link.key_file`, `venc.sensor_bin` — and every other key flies
`maburd`'s built-in default, which is the flight tuning; retuning means
changing that default in code. The drone's startup defaulted-key list is
therefore long and expected: it is where the tuning shows, one
`<key>=<value>` per key the file leaves out (a missing `[table]` is read as
an empty one, so its keys are listed too; only a missing `[venc]` reports as
`(section absent)`, since it has a required key). A value set on purpose is a line in the file;
a test pins the bundle to those five keys
(`bundle_is_the_per_drone_keys_only` in `test_config`).

The GS bundles — [`gs/bundle/maburgs.default.toml`](https://github.com/gilankpam/mabur-ground/blob/master/gs/bundle/maburgs.default.toml),
[`gs/player/bundle/maburplay.default.toml`](https://github.com/gilankpam/mabur-ground/blob/master/gs/player/bundle/maburplay.default.toml) — still write every key
the loader knows, so their defaulted-key lists are **empty** and anything
in them is a real gap (`bundle_default_sets_every_known_key` in
`test_player_config`, `bundle_default_sets_every_known_key_but_radio_cards`
in `test_gs_config`): a new GS key goes into its bundle in the same commit.

`rate_walls_rel_20`/`_40` and `air_clock.efficiency_20`/`_40` are per-UNIT
calibrations (`docs/calibration.md`). A drone that never calibrated flies
`power_mode = "none"` (the card's own power) and the built-in efficiency
tables, the reference 8812EU unit's. Measure your own vtx: ssh to the GS
and run `maburcal start`, which measures both per width, writes them to
the drone's `/etc/mabur.toml` (backing the old file up to `.pre-cal`) and
sets `power_mode = "offset"` itself — the walls fly when the run ends, the
efficiency tables from the next `maburd` start; no laptop-side step.
⚠ `"none"` also skips the `SetTxPowerOffsetQdb(0)` beside the plan, so a
global offset left in the chip by a bench tool survives a `maburd` restart;
power-cycle if you need a known baseline.

`/etc/maburgs.toml` and `/etc/maburplay.toml` are, likewise, no longer
only edited by hand: both daemons write their own config file back when
an operator changes a live key over their loopback `set`/`get` listener
(`127.0.0.1:8402`, `127.0.0.1:8403` — [`docs/gs-live-config.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/gs-live-config.md)), backing
the previous file up to `.pre-live` the same way, once per daemon run.

The single permitted omission is `radio.cards` on the GS: its *absence* is
the auto-scan setting (a `[[radio.cards]]` list pins exactly that set and
skips the bus probe), so no value can express it; the block is commented
out in place in the bundle. `stats.host`/`stats.port` are not in the bundle
either: the loader still accepts them as a single-destination fallback, but
fails boot if either appears alongside `[[stats.out]]`
([`gs/src/config.cpp`](https://github.com/gilankpam/mabur-ground/blob/master/gs/src/config.cpp)).

**What the defaulted-keys report does not catch:**

- It checks for whole keys that are absent from the file; it says nothing
  about a key that IS present but short. Drop one `[[stats.out]]` entry or
  one `[[link.ladder]]` rung while hand-editing and the report stays
  silent — the array key was supplied, just with fewer elements.
- An absent `[[stats.out]]` array does not follow the `(section absent)`
  convention the rest of the report uses — it reports as its scalar
  fallback fields, `stats.host`/`stats.port`, defaulted individually,
  which reads like two ordinary scalar defaults rather than "the whole
  export list is gone."

**Scripted edits.** Keys sit under `[table]` headers rather than dotted
paths, so a bare `sed -i 's/^symbol_size.*/symbol_size = 656/'` hits
`[fec]` **and** `[msp]` — the same trap exists for `enable`, `window`,
`port` and `host`. Anchor the range:

    sed -i '/^\[fec\]/,/^\[/ s/^symbol_size[[:space:]]*=.*/symbol_size = 656/' /etc/mabur.toml

The bundles column-align their `=` (`symbol_size     = 332`) — a naive
`^symbol_size = ` anchor matches nothing against that padding and sed
still exits 0, so a hand-written replacement must tolerate the padding the
way the anchor above does. Do not "fix" the bundle files' alignment to make
a plain anchor work; fix the sed instead, and verify with a `grep` of the
target key afterward. Prefer `vi` for one-off changes.

## JSON to TOML cutover

All three configs are TOML; there is no JSON parser and no converter.
Both install scripts refuse to run on a device that has a legacy `.json`
but no `.toml`, rather than seed the repo default over a tuned config:
convert that file by hand (read the `.json`, write the `.toml`, then check
the defaulted-keys report). The original per-device runbook is in
`docs/findings/2026-08-29-deploy-flag-days.md`.

## Pairing

The link key (`docs/link-pairing.md`) is the file `[link] key_file`
names: `/etc/mabur.key` on the drone, `/config/mabur.key` on a GS running
the mabur-ground image (its CONFIG partition, beside the configs), and
`/etc/mabur.key` on a GS installed by `install.sh`. Generate one key and
put the same file on both ends:

```sh
(echo "# mabur link key, generated $(date -I)"; openssl rand -hex 16) > mabur.key
chmod 600 mabur.key
scp -O mabur.key root@<drone>:/etc/mabur.key
scp -O mabur.key root@<gs>:/config/mabur.key && ssh root@<gs> sync
```

A GS whose `maburgs.toml` was seeded by an older image still names
`/etc/mabur.key`; check its `key_file` line before copying.

Then **Load** the same `mabur.key` file in the web page (the Link key
row's Load button) — the page keeps it in its own browser storage
(`webgs.key`), separate from the rest of its config, and passes it to the
core as a `link.key` overlay. Spotter mode has no key row and needs
nothing here. The key file is read at startup: restart `maburd` and
`maburgs` after copying it.

**Verify by comparing three fingerprints**, never the key itself: the
drone's boot log (`link: key <fp> (<source>)`), the GS's boot log (same
line, `maburgs:` prefixed), and the page's Load/Clear row. All three must
read the same 4 hex characters (or all three `default` on a stock install
with no key file, which also prints a `link: DEFAULT key in use` line). A
daemon or page showing `default` while the others show a real fingerprint
did not get the file — re-check the `scp`/Load step on that one end, it is
not a drone/GS mismatch.

Keep `mabur.key` with the flight configs on the host; it is the backup,
and there is no way to recover a lost key from either device (neither
prints it, only the fingerprint).

## Flag-day checklist

1. **Know what the change crosses.**
   - `RC_VERSION` bump: both daemons, plus the hosted web page (the web GS
     speaks the same RC wire). Between the two swaps there is no control
     link and no video.
   - AU ring version (`kAuRingVersion`, [`gs/src/au_ring.h`](https://github.com/gilankpam/mabur-ground/blob/master/gs/src/au_ring.h)): `maburgs` and
     `maburplay` together. A mismatched pair shows no video and no crash —
     the player keeps waiting on a ring of the other version.
   - Sideport keys: [`tools/maburtop.py`](https://github.com/gilankpam/mabur-ground/blob/master/tools/maburtop.py), [`tools/flightreport.py`](https://github.com/gilankpam/mabur-ground/blob/master/tools/flightreport.py) and
     `maburplay`'s OSD from the same commit.
   - Config keys: per device, independent of the other end — e.g. the
     build that derives the GS receiver width from `[link]` drops
     `radio.width` from `/etc/maburgs.toml` in the same deploy as the
     `maburgs` binary; the drone's own `radio.width` is untouched.
2. **Order binary and config by the direction of the key change.** A new
   key with an in-code default: binary first, then the key (the old binary
   refuses the unknown key). A removed or renamed key: the new binary
   refuses the old one, so edit the config while the daemon is stopped,
   before the new binary starts. Stopping the daemon and swapping both
   together is always safe. For `maburplay` an ordering mistake is silent:
   exit 2, no respawn.
3. **Make room and keep one rollback.** `df -h /` on the drone first; keep
   the old binary and its config side by side (`maburd.pre-<change>`,
   `mabur.toml.pre-<change>`), and prune older copies. `maburd` writes two
   `.pre-*` copies of its own next to `/etc/mabur.toml`: `.pre-cal`
   (`maburcal`) and `.pre-param` (a MAVLink parameter set), each the file
   as it was before that tool's most recent write; neither is a rollback
   for a deploy.
4. **Stop, verify, swap, start** — each in its own ssh call on the drone,
   with `setsid` for the start.
5. **Verify.** The `config: N key(s) defaulted:` list after each start;
   matching `link: key <fp>` lines on both ends; [`tools/bench/ausniff.py`](https://github.com/gilankpam/mabur-ground/blob/master/tools/bench/ausniff.py)
   (~60 fps, 0 gaps — the first pass right after a restart can show a
   phantom gap, take a second). A change touching the AirBalancer, venc,
   UEP overhead or the bitrate policy also gates on
   [`tools/bench/aucadence.py`](https://github.com/gilankpam/mabur-ground/blob/master/tools/bench/aucadence.py) ([`docs/observability.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/observability.md)).
6. **Roll back paired**: the old binary with its old config, on both ends
   if the wire changed.

## Device facts

**Drone.**

- Use `scp -O`: the drone's dropbear has no sftp-server, so modern scp's
  default sftp mode fails. (`-O` works against the GS's openssh too.)
- `maburd` is a glibc-DYNAMIC armv7 binary (`tools/build-arm.sh`; libusb is
  linked statically). Every NEEDED library is on the OpenIPC rootfs;
  `libstdc++.so.6` lives in `/usr/lib`, not `/lib`.
- `maburd` runs the encoder in-process. There is no `venc.bitrate` key: the
  encoder rate comes only from `RcAgent::run_bitrate_policy()`
  (`docs/link-adaptation.md`). `curl 127.0.0.1:8301/venc` on the drone
  (`venc.debug_port`) shows the encoder's status.
- The remembered channel is `/etc/mabur.channel` ([`docs/channel-select.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/channel-select.md)).
- The image loads waybeam_venc's IMX415 driver, `sensor_imx415_star6e.ko`,
  not the stock `sensor_imx415_mipi.ko` (the image's
  `load_sigmastar`; `docs/camera-modes.md`). It is a prebuilt module
  committed into `firmware/devices/_mabur-common`'s overlay, not built with the
  image: rebuild it in `../waybeam_venc` (`make drivers-star6e`, strip
  with `--strip-debug`, vermagic `4.9.84 SMP preempt mod_unload ARMv7
  thumb2 p2v8`) and replace that file. It has to be the six-mode build
  with the retimed native modes: `venc.binning = false` asks for
  2944x1656@60 and 2272x1278@100, and maburd fails boot with `sensor mode
  WxH@F not offered by the driver` when the loaded module lacks the mode
  it asked for (an older six-mode build offers 2304x1296@100 instead, and
  times both native modes under their nominal rate); the binned tiers
  work on an older build too. A drone on an image with an
  older module takes the new one by hand: copy it into
  `/lib/modules/4.9.84/sigmastar/` under the name `load_sigmastar` loads
  (keep the old file) and reboot; `lsmod` then lists
  `sensor_imx415_star6e`.

**GS, Orange Pi Zero 2W GS, CPE510 relay:** mabur-ground's
[`docs/deploy.md`](https://github.com/gilankpam/mabur-ground/blob/master/docs/deploy.md) "Device facts".
