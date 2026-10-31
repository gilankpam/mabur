# Deploying maburd / maburgs

There is no backward-compatibility obligation in mabur (see CLAUDE.md),
but a deploy still touches two devices and is never atomic. That window,
and the config-load behaviour below, are what actually bite. The dated
runbooks of past flag days are in
`docs/findings/2026-08-29-deploy-flag-days.md`.

**A version mismatch has no video at all.** Any `RC_VERSION` bump
(`common/include/mabur/rc_proto.h`) is a flag day: a drone and GS at
different versions reject each other's frames in BOTH directions, and
because DISC_ACK is what carries `CAP_FRAME_WIRE`, the symptom is no
control link and no video. Restarting either daemon does not fix a
version mismatch; finish the deploy. The RC wire goldens live in
`tests/test_rc.cpp`. `RC_VERSION` 18 (calibration moves onto the MAVLink
tunnel; `T_CAL_CMD`/`T_CAL_RESULT` deleted) is a flag day like every other
bump, and it also removes a drone config key: `radio.rate_walls_rel` fails
boot, and the old binary refuses its replacements. On the drone: stop
`maburd`, edit the config — rename the key to `rate_walls_rel_20` (it
holds 20 MHz walls; with no `_40` the 40 MHz rungs fly the 20 MHz table,
which `maburd` logs) and add `cal_flags_20 = "none"` (the old kit swept
with LDPC and STBC off) — swap the binary, start. Then `maburgs` with
`gs/bundle/maburcal` and `tools/maburtop.py` (`gs/bundle/install.sh`
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
`maburd` binaries), start both. Rollback, when you actually need one, is
PAIRED: an old binary needs its old config restored alongside it. There is
no forward or backward config compatibility, so rolling forward is usually
the shorter path. The generic checklist is "Flag-day checklist" below.

**The install scripts.** `bundle/install.sh root@<drone>` and
`gs/bundle/install.sh root@<gs>` build (unless a prebuilt binary is passed
as the second argument), stop the service, scp the binary and the init
script, then check the target's config before touching it:

- a `.toml` already present is left alone ("never clobber a tuned one");
- neither a `.toml` nor a legacy `.json`: the shipped default is copied in;
- only a legacy `.json`: the script prints an error and exits 1 rather
  than seed the repo default over a tuned config (see "JSON to TOML
  cutover").

Then they start the service. Neither migrates an existing config: a key
change still has to be made by hand. `gs/bundle/install.sh` also keeps
`/usr/local/bin/maburgs.prev` and installs `tools/maburtop.py` and
`gs/bundle/maburcal` to `/usr/bin`. Without a binary argument it reads the
board's `/proc/device-tree/model` and ships that board's glibc build
(`out/arm64/maburgs` on a Radxa, `out/arm64/maburgs-opi` on an Orange Pi),
and it waits for maburgs to exit after `S96maburgs stop`: stop returns
before the daemon is gone, and copying over a running binary fails with
"Text file busy". A hand swap needs the same wait
(`while pidof maburgs; do sleep 1; done`). There is no installer for `maburplay`
(see "Device facts").

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
(openipc-builder `package/mabur/mabur.mk`), so BusyBox rcS starts `maburd`
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

## Supported drone cards

`tools/build-arm.sh` builds maburd for the **RTL8812EU** only (devourer
rtl8822e) — the bench drone's card. The **RTL8812CU** (rtl8822c) is
supported through a separate firmware image, the openipc-builder board
`ssc338q_fpv_emax-wyvern-link-alpha`, whose `mabur.mk` builds devourer with
the 8822C alone (`BR2_PACKAGE_MABUR_RADIO_8812CU`). One image, one chip: a
binary on the wrong card fails at `CreateRtlDevice` ("unsupported chip").
The shipped `usb_pid = 0` scan tries `a81a`, then `c812`/`c82c`, then the
8812AU ids `881a`/`8812` (`drone/src/main.cpp`), so the same `mabur.toml`
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
the first 40 MHz frame forever. The shipped ladder sends every rung at
40 MHz, so on the drone the floor is on for the whole flight, and it costs
the RCF uplink idle-channel sensitivity. `maburd` picks the value per image,
from the one Jaguar3 chip that image compiles in:

- **8812EU**: `0` (no floor). The EU has flown the 40 MHz rungs without a
  floor and never stalled. An EU unit that does stall loses the downlink
  at its first 40 MHz frame, with the TX FIFO full; give that unit a floor.
- **8812CU**: devourer's default, `0x35`. The stall was found on the 8822C,
  and the default is the lowest value that has cleared every CU unit tested.

The 8812CU is 2T2R, so STBC airs on two real chains; none of the 8822E
constraints in devourer's 8822E quirks page
(`third_party/devourer/docs/8822e-quirks.md`: spur channels, single-path
1SS TX, DPDT front end) apply to it. **The 8812CU path is not
hardware-verified on this bench** — there is no CU unit here; its only
on-hardware check is an external tester's rig.

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

**The bundles set every knob.** All three shipped defaults —
`bundle/mabur.default.toml`, `gs/bundle/maburgs.default.toml`,
`gs/player/bundle/maburplay.default.toml` — are the flight configs, with
every key the loader knows written out explicitly. On a stock bundle the
startup defaulted-key list is therefore **empty**, and anything in it is a
real gap. Three tests hold that line (`bundle_default_sets_every_known_key`
in `test_config` and `test_player_config`,
`bundle_default_sets_every_known_key_but_radio_cards` in `test_gs_config`),
so **a new config key must be written into its bundle in the same commit**
or the host suite fails.

The one deliberate divergence from a calibrated drone is
`radio.power_mode`, which ships `"none"`. `rate_walls_rel_20`/`_40` and
`air_clock.efficiency_20`/`_40` are per-UNIT calibrations
(`docs/calibration.md`) and the shipped file cannot know the board it
lands on, so it carries the reference 8812EU unit's numbers. The walls are
inert under `"none"` — parsed and range-checked, never programmed; the
efficiency tables are not (they price every rung). Measure your own vtx:
ssh to the GS and run `maburcal start`, which measures both per width,
writes them to the drone's `/etc/mabur.toml` (backing the old file up to
`.pre-cal`) and flips `power_mode` to `"offset"` itself — the walls fly
when the run ends, the efficiency tables from the next `maburd` start; no
laptop-side step.
⚠ `"none"` also skips the `SetTxPowerOffsetQdb(0)` beside the plan, so a
global offset left in the chip by a bench tool survives a `maburd` restart;
power-cycle if you need a known baseline.

The single permitted omission is `radio.cards` on the GS: its *absence* is
the auto-scan setting (a `[[radio.cards]]` list pins exactly that set and
skips the bus probe), so no value can express it; the block is commented
out in place in the bundle. `stats.host`/`stats.port` are not in the bundle
either: the loader still accepts them as a single-destination fallback, but
fails boot if either appears alongside `[[stats.out]]`
(`gs/src/config.cpp`).

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

The link key (`docs/link-pairing.md`) lives in `/etc/mabur.key` on both
ends (`[link] key_file`). Generate one key and put the same file on both
ends:

```sh
(echo "# mabur link key, generated $(date -I)"; openssl rand -hex 16) > mabur.key
chmod 600 mabur.key
scp -O mabur.key root@<drone>:/etc/mabur.key
scp -O mabur.key root@<gs>:/etc/mabur.key
```

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
   - AU ring version (`kAuRingVersion`, `gs/src/au_ring.h`): `maburgs` and
     `maburplay` together. A mismatched pair shows no video and no crash —
     the player keeps waiting on a ring of the other version.
   - Sideport keys: `tools/maburtop.py`, `tools/flightreport.py` and
     `maburplay`'s OSD from the same commit.
   - Config keys: per device, independent of the other end.
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
   matching `link: key <fp>` lines on both ends; `tools/bench/ausniff.py`
   (~60 fps, 0 gaps — the first pass right after a restart can show a
   phantom gap, take a second). A change touching the AirBalancer, venc,
   UEP overhead or the bitrate policy also gates on
   `tools/bench/aucadence.py` (`docs/observability.md`).
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
- The remembered channel is `/etc/mabur.channel` (`docs/channel-select.md`).
- The image loads waybeam_venc's IMX415 driver, `sensor_imx415_star6e.ko`,
  not the stock `sensor_imx415_mipi.ko` (`openipc-builder`'s
  `load_sigmastar`; `docs/camera-modes.md`). maburd fails boot with
  `sensor mode WxH@F not offered by the driver` under the stock one. A
  drone on an older image takes it by hand: copy the module into
  `/lib/modules/4.9.84/sigmastar/` under the name `load_sigmastar` loads
  (keep the old file) and reboot; `lsmod` then lists
  `sensor_imx415_star6e`.

**GS.**

- `maburgs` and `maburplay` live in `/usr/local/bin`; `maburtop` and
  `maburcal` in `/usr/bin` (the GS shell's default PATH lacks
  `/usr/local/bin`); the OSD fonts and splash in `/usr/local/share/mabur/`
  (staged in `out/arm64/` by `tools/build-arm64.sh`). The remembered
  channel is `/etc/maburgs.channel`.
- Never load the 8812eu kernel module on the GS cards; `S96maburgs`
  unloads it on start.
- `maburgs` is glibc-DYNAMIC, built per board against that image's
  Buildroot sysroot (`tools/build-arm64.sh` stage 6 → `out/arm64/maburgs`,
  stage 7 → `out/arm64/maburgs-opi`). NEEDED: `libusb-1.0.so.0` plus
  libstdc++/libm/libgcc_s/libc and the loader, all on both images. The
  image repo's own package builds it the same way. `out/arm64/maburgs-static`
  is the musl-static build of the same daemon, a rollback that runs on any
  board but costs ~2 ms of GS latency per frame
  (`docs/findings/2026-10-15-gs-musl-latency.md`).
- **`/etc/maburplay.toml` is a symlink to `/config/maburplay.toml`** (the
  vfat partition). Back up and edit the **target**, not the link: BusyBox
  `sed -i` on the symlink replaces it with a regular overlay file, after
  which the `/config` copy goes silently dead, and `cp -a` on the link
  backs up only the link.
- **Bench: running `tools/bench/mavparam.py` from the host** needs
  `maburgs`'s `[mavlink] bind` set to the GS's LAN address, not its
  loopback default (`docs/mavlink.md`). `maburgs`'s config is
  `/etc/maburgs.toml` on the overlay (only `maburplay.toml` is the vfat
  symlink): back up `/etc/maburgs.toml`, edit it, restart `S96maburgs`.
- There is no `maburplay` installer: copy `out/arm64/maburplay.default.toml`
  (staged by `tools/build-arm64.sh`) to the config by hand on a fresh GS.
  With no config, `maburplay -c /etc/maburplay.toml` exits 2 and
  `S97maburplay` does not respawn.
- `maburplay` is glibc-DYNAMIC (Mesa EGL/GLESv2/GBM + librga for the
  burned-DVR colortrans stage, `docs/colortrans.md`). NEEDED:
  `librockchip_mpp.so.1`, `libdrm.so.2`, `libEGL.so.1`, `libGLESv2.so.2`,
  `libgbm.so.1`, `librga.so.2`, plus libstdc++/libm/libgcc_s/libc and the
  loader. Check the GS image carries them before swapping, e.g.
  `ssh root@<gs> 'ls /usr/lib/libEGL.so.1 /usr/lib/libGLESv2.so.2 /usr/lib/libgbm.so.1 /usr/lib/librga.so.2'`
  — miss it and the binary will not exec at all.
  `out/arm64/maburplay-static` is the musl-static build of the same player
  without the GPU stage, for an image that lacks Mesa.

**Orange Pi Zero 2W GS** (`docs/opi-zero2w.md`). Same layout and init
scripts as above; the daemon is `out/arm64/maburgs-opi` (installed as
`/usr/local/bin/maburgs`, `gs/bundle/install.sh` picks it by model) and
the player binary is `out/arm64/maburplay-opi`
(installed as `/usr/local/bin/maburplay`), glibc-dynamic against the OPi
image's libav: NEEDED `libavcodec.so.61`, `libavutil`, `libdrm.so.2`,
libstdc++. Its config must say `backend = "v4l2req"`, `dvr.mode = "raw"`
— a Radxa `maburplay.toml` exits 2. Check the target first:
`ssh $GS cat /proc/device-tree/model` must read `OrangePi Zero 2W`.

    ssh $GS '/etc/init.d/S97maburplay stop'
    scp -O out/arm64/maburplay-opi $GS:/usr/local/bin/maburplay
    ssh $GS '/etc/init.d/S97maburplay start'

maburplay needs mabur's FFmpeg patch in the board's libavcodec, so put the
patched libavcodec on before (or together with) a new maburplay: a new
maburplay against an unpatched libavcodec fails `init()` and the wrapper
respawns it forever at 2 s. An image built from the image repo carries the
patch already; to put a rebuilt libavcodec on a board by hand, copy it
onto the overlay, where it shadows the image's:

    scp -O ../sbc-groundstations-gilankpam/output/orangepi_zero2w_defconfig/target/usr/lib/libavcodec.so.61.19.100 $GS:/usr/lib/

and check `find /overlay/root/usr/lib -name 'libavcodec*'` after a later
image flash (a stale copy there shadows the new image's).

**CPE510 relay** (`radio.relays`, `docs/cpe510-relay.md`). The Radxa ZERO 3
has no Ethernet: the CPE needs a USB-Ethernet adapter on the GS. The CPE
serves DHCP on its LAN, 10.83.11.100-199, with **no router and no DNS**
(mabur-openwrt `90-mabur-lan`), so `relays` entries are numeric
`"ipv4:port"`. After a CPE `sysupgrade -n` its SSH host key changes:
`ssh-keygen -R 10.83.11.1`.

## GS image: p1 squashfs swap

The GS boots U-Boot → `/boot/Image` + `/boot/extlinux/extlinux.conf`, read
out of the squashfs on p1 (`/dev/mmcblk1p1`, 264160 KiB = 270,499,840
bytes; U-Boot `CONFIG_CMD_SQUASHFS`). The kernel command line is the
`append` line of that extlinux.conf (image repo:
`board/common/overlay/boot/extlinux/extlinux.conf`), so a kernel parameter
(`rk_vcodec.rkvdec2_stream=0`: the rkvdec2 driver is built in) means a new
squashfs. Everything else the GS keeps — `/etc` configs,
`/usr/local/bin/maburgs`/`maburplay` — lives on the overlayfs partition
(upper dir `/overlay/root`) and survives a p1 swap. A library copied into
the upper dir's `usr/` shadows the image's: check
`find /overlay/root/usr -name 'librockchip_mpp*'` is empty after a swap.

Below, `$GS` is the ground station's ssh target (`root@<gs-ip>`).

Back up p1 first:
`ssh $GS 'dd if=/dev/mmcblk1p1 bs=1M' > p1.orig.img`
(270,499,840 bytes); rollback is the same sequence with that file. The
SD-card image stays the last resort if p1 does not boot.

Flash any image `IMG` (from the image repo's `output/`):

    IMG=<squashfs>; SZ=$(stat -c %s $IMG); MD5=$(md5sum < $IMG | cut -d' ' -f1)
    scp -O $IMG $GS:/tmp/p1.new.img
    ssh $GS 'md5sum /tmp/p1.new.img'                 # == $MD5
    ssh $GS '/etc/init.d/S97maburplay stop'
    ssh $GS '/etc/init.d/S96maburgs stop'
    ssh $GS 'pidof maburplay maburgs || echo none'   # none
    timeout 60 ssh $GS 'dd if=/tmp/p1.new.img of=/dev/mmcblk1p1 bs=1M conv=fsync && sync && echo written && echo 1 > /proc/sys/kernel/sysrq && echo s > /proc/sysrq-trigger && echo b > /proc/sysrq-trigger' || true
    # once it is back up (~25 s): the partition holds the image
    ssh $GS "head -c $SZ /dev/mmcblk1p1 | md5sum"   # == $MD5

Write and reboot in **one** ssh command, and reboot through sysrq only:
once p1 is rewritten, nothing on the old lower filesystem can be exec'd
except what is still in the page cache — BusyBox `reboot` cannot, and after
`echo 3 > /proc/sys/vm/drop_caches` neither can `head`/`md5sum` nor sshd
itself (every new ssh connection is reset). If that happens anyway,
power-cycle the GS: the `sync` has already landed the image. Verify the
write after the boot, as above; to check it before rebooting, the
read-back needs a static binary in `/tmp` and must run in the same ssh
command as the `dd`.

Verify after boot: `uname -v`, `cat /proc/cmdline`,
`cat /sys/module/rk_vcodec/parameters/rkvdec2_stream` (`Y` on the
stream-mode image), `dmesg | grep "link mode off"`.
