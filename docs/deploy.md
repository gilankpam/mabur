# Deploying maburd / maburgs

There is no backward-compatibility obligation in mabur (see CLAUDE.md),
but a deploy still touches two devices and is never atomic. That window,
and the config-load behaviour below, are what actually bite.

The 2026-08-12 constant-TX-power change (`docs/data-provenance.md`)
bumped `RC_VERSION` 1 -> 2, the first bump the protocol has ever had; the
2026-08-15 RCF shrink (dropping the write-only `ack_seq`, `score` and
`layer_delivery` fields, none of which `maburd` ever read) bumped it
2 -> 3. mabur owns the RC wire outright now: the goldens live in
`tests/test_rc.cpp` rather than mirrored from devourer's frozen
`tools/precoder/rc_proto.py` (pinned at version 1, no longer an oracle),
so further bumps are cheap and need no deprecation path — the deploy
window is their entire cost, and that cost is real.
**A drone and GS at different versions reject each other's
frames in BOTH directions**, and because DISC_ACK is what carries
`CAP_FRAME_WIRE`, the symptom is NO VIDEO AT ALL — visually identical to
the stale-caps restart deadlock (below). Restarting either daemon will
not fix a version mismatch; recovery is to finish the deploy. Deploy
order is config-before-binary on both devices: a stale or removed key
makes `maburd`/`maburgs` fail to start (a config-load error exits **1** in
`maburd` and **2** in `maburgs` — both in the `load_config` try/catch in
their respective `main()`; the two daemons do NOT agree, so do not key a
script off either number), and unlike `maburplay`'s
`S97maburplay` — which stops respawning on exit
2 or 143 — neither daemon's wrapper (`S96mabur`, `S96maburgs`,
`maburgs.service` under systemd) checks the exit code at all, so it
crash-loops the daemon forever at the unconditional 2 s respawn delay.
The visible symptom is a repeating `unknown key` line in `/tmp/mabur.log`
/ `/tmp/maburgs.log` every 2 seconds, not a daemon that exits and stays
down. The clean sequence is still: stop both daemons, edit both configs,
swap both binaries (`df` and prune first — the drone rootfs fits max 2
maburd), start both. Rollback, when you actually need one, is PAIRED: an
old binary needs its old config restored alongside it — there is no
forward or backward config compatibility and no attempt at any, so
rolling forward is usually the shorter path. **The repo's install scripts will not do
the config edit for you, and will refuse to run rather than guess:**
`bundle/install.sh` and `gs/bundle/install.sh` scp the binary
(`bundle/install.sh:37`, `gs/bundle/install.sh:32`), then check the
target's config before touching it (`bundle/install.sh:39-54`,
`gs/bundle/install.sh:37-52`) — copy the shipped `.toml` default only if
the device has NEITHER a `.toml` NOR a legacy `.json`; if a `.toml`
already exists, leave it alone ("never clobber a tuned one"); if only a
legacy `.json` exists (a device that has never been converted), **print
an error and exit 1 instead of seeding the repo default over it**, since
a bare "no `.toml`" check can't tell "already converted, nothing to do"
from "never converted, about to lose the tuned config" — and the latter
boots cleanly on repo defaults, with no crash, no signal, and nothing
in the startup defaulted-keys report to catch it (that report compares
against compiled defaults, not the device's old config). Convert the
`.json` to `.toml` by hand first (see "JSON to TOML cutover" below), then
re-run the installer. Once past that check, both scripts start the
service immediately (`bundle/install.sh:57-61`,
`gs/bundle/install.sh:57-61`); neither migrates an existing config, so on
a device that has ever been tuned the four removed keys must also be
deleted from `/etc/mabur.toml` and `/etc/maburgs.toml` BY HAND before the
new binaries start.

**Restarting the drone daemon over ssh: use `setsid`.** `S96mabur`'s respawn
loop is a background subshell of the shell that started it, so a plain
`ssh root@drone '/etc/init.d/S96mabur start'` gives you a `maburd` that dies
with the ssh session — the daemon comes up, ssh returns, and the link drops
seconds later for no visible reason. Detach it:
`setsid /etc/init.d/S96mabur start </dev/null >/dev/null 2>&1 &`. (Measured
during the 2026-08-28 Part A gate; it applies to any wrapper started from a
non-interactive ssh command, not just this one.) The mirror image also happens: on
2026-08-29 an `ssh root@drone '...; /etc/init.d/S96mabur stop; mv ...'`
one-liner died at the `stop` and never ran the rest — the daemons stopped,
the ssh command silently returned, and the swap that was supposed to follow
it had not happened. `S96mabur stop` kills a PID read from
`/var/run/mabur-loop.pid`, and after hours of uptime that PID can name
something else. **Never chain work after a `stop` in the same ssh command:
stop in one invocation, verify with `ps`, then swap in the next.**

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
the value they fell back to. After hand-editing a config, read that list:
anything unexpected in it is a knob you dropped.

**The bundles set every knob (2026-09-10).** All three shipped defaults —
`bundle/mabur.default.toml`, `gs/bundle/maburgs.default.toml`,
`gs/player/bundle/maburplay.default.toml` — are the live flight configs off
the drone and the GS, extended so that every key its loader knows is written
out explicitly: live value where the device had one, struct default where it
did not.

The one deliberate divergence from the flown value is
`radio.power_mode`, which ships `"none"` while this drone flies `"offset"`.
`rate_walls_idx` is a per-UNIT calibration (`docs/calibration.md`) and the
shipped file cannot know the wall of the board it lands on, so it carries the
author's 8812EU numbers as a reference and leaves them inert — parsed and
range-checked, never programmed. Measure your own vtx before setting
`"offset"`: ssh to the GS and run `maburcal start`, which sweeps the walls,
writes them to the drone's `/etc/mabur.toml` (backing the old file up to
`.pre-cal`) and flips `power_mode` to `"offset"` itself — no restart, no
laptop-side step. ⚠ `"none"` also skips the `SetTxPowerOffsetQdb(0)` beside
the plan, so a global offset left in the chip by a bench tool survives a
`maburd` restart; power-cycle if you need a known baseline. On a stock bundle the startup defaulted-key list is therefore
**empty**, and anything in it is a real gap. Three tests hold that line
(`bundle_default_sets_every_known_key` in `test_config` and
`test_player_config`, `bundle_default_sets_every_known_key_but_radio_cards`
in `test_gs_config`), so **a new config key must be written into its bundle
in the same commit** or the host suite fails.

The single permitted omission is `radio.cards` on the GS: its *absence* is
the auto-scan setting (a `[[radio.cards]]` list pins exactly that set and
skips the bus probe), so no value can express it. `stats.host`/`stats.port`
are absent for a harder reason — the loader fails boot if either appears
alongside `[[stats.out]]`. Both are commented out in place in the bundle.

**Scripted edits.** `json_cli` no longer applies. Keys sit under `[table]`
headers rather than dotted paths, so a bare
`sed -i 's/^symbol_size.*/symbol_size = 656/'` hits `[fec]` **and**
`[msp]` — the same trap exists for `enable`, `window`, `port` and `host`.
Anchor the range:

    sed -i '/^\[fec\]/,/^\[/ s/^symbol_size[[:space:]]*=.*/symbol_size = 656/' /etc/mabur.toml

The bundles column-align their `=` (`symbol_size     = 332`, 43 of
`bundle/mabur.default.toml`'s 59 keys are padded this way) — a naive
`^symbol_size = ` anchor matches nothing against that padding and sed
still exits 0, so a hand-written replacement must tolerate the padding
the way the anchor above does. Do not "fix" the bundle files' alignment
to make a plain anchor work; it's deliberate and readable — fix the sed
instead, and verify with a `grep` of the target key afterward. Prefer
`vi` for one-off changes.

## JSON to TOML cutover

All three configs moved from JSON to TOML on 2026-09-07. There is
deliberately no converter script: the operator reads each device's old
`.json` and hand-writes the new `.toml` against it, so a tuned value
never survives a mechanical transform that might silently drop or
reshape it.

**Per device, independent, no wire impact.** Config never crosses the
air — it only ever configures the local daemon — so a TOML drone against
a JSON GS is a perfectly fine pair mid-cutover; this is nothing like the
`RC_VERSION` flag days above. What is NOT independent is binary and
config on the SAME device: they swap **together**. An old binary cannot
read `.toml` (it doesn't know the format exists) and a new binary will
not read the old `.json` (parsing is TOML-only, no fallback), so the two
must change in the same window or that one device fails to boot.

1. **Drone.** `df -h /` first — the 5.8 MB rootfs fits at most two
   `maburd` binaries, so prune old `.pre-*` copies before staging a new
   one. Read the live config, hand-write the TOML against it, then swap
   both together:
   ```sh
   ssh root@<drone> 'cat /etc/mabur.json'                 # read the tuned values
   # hand-write /tmp/mabur.toml against that output
   ssh root@<drone> '/etc/init.d/S96mabur stop'
   scp -O /tmp/mabur.toml       root@<drone>:/etc/mabur.toml
   ssh root@<drone> 'mv /usr/bin/maburd /usr/bin/maburd.pre-toml'
   scp -O out/arm/maburd        root@<drone>:/usr/bin/maburd
   ssh root@<drone> '/etc/init.d/S96mabur start'
   ```
2. **GS.** Same shape: read `/etc/maburgs.json`, hand-write
   `/etc/maburgs.toml`, stop `S96maburgs`, swap config and binary
   together (rollback copy `maburgs.pre-toml`), start.
3. **Player.** Same shape again for `maburplay`/`maburplay.json`/
   `maburplay.toml`. There is no `gs/player/bundle/install.sh` to do this
   step for you — copy `out/arm64/maburplay.default.toml` (staged by
   `tools/build-arm64.sh` next to the fonts and splash) to
   `/etc/maburplay.toml` by hand, or hand-write it against the live
   `maburplay.json` the same way as the other two. Do not skip this: with
   no `/etc/maburplay.toml`, `maburplay -c /etc/maburplay.toml` exits 2,
   and `S97maburplay` treats exit 2 as terminal — no respawn, permanently
   black GS screen, not a crash-loop you'd notice in a log tail.
4. **Verify with the defaulted-keys report**, not by eye. Each daemon
   prints `config: N key(s) defaulted:` on boot — that line, read
   immediately after each swap, is the only safety net against a knob
   the hand-conversion dropped. An unexpected name in it means a value
   that should have carried over from the old `.json` fell back to the
   compiled default instead.
5. **Rollback is paired and per device**, same rule as everywhere else in
   this page: restore the old `.json` **and** the `.pre-toml` binary
   together, never one alone — a `.pre-toml` binary started against the
   new `.toml`, or vice versa, just fails to boot again.
6. **Delete the old `.json` files only after a clean flight** on the new
   config, per device.

Two holes in that safety net, worth knowing before you trust it blind:

- The defaulted-keys report checks for whole keys that are absent from
  the file; it says nothing about a key that IS present but short. Drop
  one `[[stats.out]]` entry or one `[[link.ladder]]` rung while
  hand-converting and the report stays silent — the array key was
  supplied, just with fewer elements than before.
- An absent `[[stats.out]]` array does not follow the `(section absent)`
  convention the rest of the report uses — it reports as its scalar
  fallback fields, `stats.host`/`stats.port`, defaulted individually,
  which reads like two ordinary scalar defaults rather than "the whole
  export list is gone."

## Stale-caps restart deadlock — retired

The old failure mode: either daemon restarting (drone reboot, `maburgs`
restart, GS reboot — anything that resets one side's session state while
the other side keeps running) left a rebooted GS stuck at
`peer_caps_ == 0`, refusing video forever (`REFUSING video: peer session
did not advertise CAP_FRAME_WIRE`) because it had no fast path back to
re-learning the live drone's caps, and a rebooted drone likewise had no
way to re-teach a GS that was still nominally LINKED. The only known
recovery was the manual dance: `waybeam stop` on the drone, wait for the
GS log to print `video tail -> frame wire`, then `waybeam start`.

The caps-reteach pair retires this dance for same-version pairs:
- the drone acks a keep-alive DISC while already LINKED, instead of
  ignoring it, so a GS that forgot its caps gets them re-taught without
  needing to re-enter rendezvous;
- the GS sends its keep-alive DISC on a fast cadence
  (`unacked_keepalive_ms`, default-only, no config key) whenever its
  peer's caps are unknown, instead of the slow steady-state
  `beacon_keepalive_ms`, so the re-teach happens in seconds rather than
  however long the next slow beacon would take.

Gate-verified on hardware 2026-08-28: 5x drone `maburd` restart and 5x GS
`maburgs` restart, each under a live peer, all 10 recovered unaided
(`fps` back to ~60, `frame_id_gaps` flat, no `REFUSING`/`chip_caps=0x0000`
in the GS log) within the standard ~25 s post-restart wait — no
`waybeam stop`/`start`, no manual restart of either daemon.

**Both binaries must carry the fix for it to self-heal.** An old-GS +
new-drone pair (or new-GS + old-drone) is *safe* — same `RC_VERSION`, no
crash-loop, no frame rejection — but **un-healed**: the side still
running the old binary lacks its half of the re-teach, so a restart on a
half-deployed pair still needs the manual `waybeam stop` -> wait for
`video tail -> frame wire` -> `waybeam start` dance until the deploy is
finished on both ends. ⚠ Since the 2026-08-29 venc fold-in that
escape hatch no longer exists on the drone at all — there is no `waybeam` to
stop — so the equivalent is `S96mabur stop` / `start`, which restarts the
encoder along with the link. Keeping both ends on the same build is what
makes that never necessary.


## The venc fold-in — what a drone deploy is now (2026-08-29)

`maburd` runs the encoder itself. The SigmaStar MI pipeline (VIF/VPE/ISP/VENC
+ the JPEG channel) is brought up inside the daemon from the `venc` config
section, frames are handed to the FEC path in-process, and the encoder knobs
are direct function calls — no `waybeam` process, no HTTP control plane, no
frame-shm ring between two processes. Consequences for a deploy:

- **The drone binary is a DYNAMIC glibc executable**, not the old static musl
  one (`tools/build-arm.sh` builds it with the OpenIPC Buildroot toolchain;
  `tools/build-arm-glibc.sh` and `cmake/arm-musl.cmake` are gone). NEEDED:
  `libstdc++.so.6` (in `/usr/lib`, not `/lib`), `libm`, `libgcc_s`, `libc`,
  `ld-linux-armhf.so.3` — all present on the OpenIPC rootfs. It is ~950 KB
  stripped, roughly half the musl binary, which is why the rootfs fits the
  rollback rotation at all.
- **`waybeam` must be inert before `maburd` starts.** Two processes cannot
  own the MI pipeline; the loser gets no camera. The flag day therefore stops
  `S95waybeam`, clears its execute bits so it never runs at boot again, and
  moves `/usr/bin/waybeam` to `/usr/bin/waybeam.retired`. ⚠ It must be
  `chmod a-x`, not `chmod -x`: with no class specified, `chmod` applies the
  umask, so under the drone's 0022 umask `chmod -x` clears only the OWNER's
  x bit and leaves `-rw-r-xr-x` — which root can still execute, and which
  BusyBox `rcS`'s `[ -x ]` test still accepts. Hit and fixed during the
  2026-08-29 flag day; verify with
  `[ -x /etc/init.d/S95waybeam ] && echo STILL EXECUTABLE`.
- **The config gains `venc` (pipeline bring-up) and `encoder` (RcAgent's
  bitrate/ROI policy), and loses `waybeam` and `frame_ring_name`.** Strict
  keys mean the old config fails boot against the new binary and vice versa,
  so this is a paired swap like any other. `bundle/mabur.default.toml` carries
  the shipped shape; the bench's tuned values live in `/etc/mabur.toml`.
- **There is no `venc.bitrate` key and never will be.** The encoder rate comes
  only from `RcAgent::run_bitrate_policy()` — see `docs/link-adaptation.md`.
- **The GS deploys with the drone.** The `Telem` struct widened to 70 bytes for
  the venc ring stats, so a mismatched pair drops `T_TELEM` on CRC and the
  sideport's `drone.*` block reads `null` until both ends run the same build.
  Video itself is unaffected by that particular mismatch, but "no drone rows in
  maburtop" after a half-finished deploy is this, not a dead uplink.

### Flag-day sequence (drone)

```sh
ssh root@<drone> 'df -h /; ls -la /usr/bin/waybeam* /usr/bin/maburd*'   # prune first
tools/build-arm.sh                                # -> out/arm/maburd
scp -O out/arm/maburd  root@<drone>:/tmp/maburd.new
scp -O <new-config>    root@<drone>:/tmp/mabur.json.new
ssh root@<drone> '
  /etc/init.d/S95waybeam stop; /etc/init.d/S96mabur stop; sleep 1
  mv /usr/bin/waybeam /usr/bin/waybeam.retired && chmod a-x /etc/init.d/S95waybeam
  mv /usr/bin/maburd /usr/bin/maburd.pre-foldin
  mv /etc/mabur.json /etc/mabur.json.pre-foldin
  mv /tmp/maburd.new /usr/bin/maburd && chmod 755 /usr/bin/maburd
  mv /tmp/mabur.json.new /etc/mabur.json
  reboot'
```

Reboot rather than a restart, deliberately: it is the only thing that proves
the boot order (`S95waybeam` inert, `S96mabur` bringing the camera up from
cold) instead of leaving it to the next unplanned power cycle.

Post-boot gates: `ausniff` ~60 fps / 0 `frame_id_gaps`; `curl 127.0.0.1:8301/venc`
on the drone answers with advancing `frames`; the GS sideport shows the
`drone.*` telemetry rows again; the GS ctl log shows a normal cold climb and
park.

### Rollback — the trio, and the two steps that bite

⚠ **2026-08-29 cleanup: waybeam no longer exists on the drone at all** —
binary (incl. every rollback copy), `/etc/waybeam.json`, and `S95waybeam`
were deleted after the fold-in soaked. Rolling back the fold-in is now a
re-deploy, not a file swap: rebuild waybeam in `../openipc-builder`
(waybeam-venc pkg, pinned f956a52), scp it to `/usr/bin/waybeam`, restore
`/etc/waybeam.json` from the archived copy
(`out/drone-waybeam-config-final-2026-08-29.json` on the dev host), and
recreate `S95waybeam` from the openipc-builder package's `init.d/`. The
same cleanup also removed **every** `maburd.pre-*`/`mabur.json.pre-*`
rollback from the drone and the `*.pre-*` binaries from the GS (archived
on the dev host as `out/drone-rollback-archive-2026-08-29.tar.gz` and
`out/gs-rollback-archive-2026-08-29.tar.gz`) — rollback of anything now
means rebuild-from-git (or unpack the archive) + redeploy, per the
roll-forward policy. After the pieces are back in place, the sequence
below still applies:

```sh
ssh root@<drone> '
  /etc/init.d/S96mabur stop; killall maburd; sleep 1
  mv /usr/bin/maburd /usr/bin/maburd.foldin
  mv /usr/bin/maburd.pre-foldin /usr/bin/maburd
  mv /etc/mabur.json /etc/mabur.json.foldin
  mv /etc/mabur.json.pre-foldin /etc/mabur.json
  chmod 755 /etc/init.d/S95waybeam
  /etc/init.d/S95waybeam start
  # 1. WAIT for waybeam HTTP to answer -- do not sleep a fixed interval
  for i in $(seq 60); do
    wget -q -O - "http://127.0.0.1/api/v1/get?video0.bitrate" && break
    sleep 1
  done
  # 2. re-apply the bitrate AND restart maburd, always as a pair
  wget -q -O - "http://127.0.0.1/api/v1/set?video0.bitrate=3000"
  setsid /etc/init.d/S96mabur start </dev/null >/dev/null 2>&1 &'
```

Both numbered steps are load-bearing, and skipping either reproduces the
waybeam bitrate wedge (`docs/`-recorded 2026-08-28; hit again during the
2026-08-29 bench restore). The failure is loud and looks like a radio problem:
the encoder floods at its last rate into a rung-0 link, the ladder is trapped
at `mcs1`, `txq_drop` climbs into six figures, the GS sees ~48 fps with 1019
`frame_id_gaps`, and the SoC hits 70 °C. Why each step:

1. A `set?video0.bitrate=` issued while waybeam's HTTP server is still
   initialising returns `Connection refused` and is silently lost. Poll until
   it answers.
2. The **pre-fold-in** `maburd`'s RcAgent only pushes a bitrate when its
   computed value CHANGES, so on a parked link it will not re-assert over
   whatever waybeam came up with. The restart forces the stamp on entering
   LINKED. (Current `maburd` also re-asserts every 5 s —
   `RcAgent::kReassertMs`, `docs/link-adaptation.md` — but that is exactly
   the binary this runbook is rolling BACK from, so step 2 stands.)

Restore the GS's `maburgs.pre-foldin` at the same time, for the telemetry
reason above.

## `maburplay` — `display.chain_budget` (2026-09-02)

Additive with an in-code default of 3 (0 = unbounded, the prior behavior),
same rules as the vsync keys below: binary first, key optional. Rolling
back to a pre-2026-09-02 binary (`maburplay.pre-chain` or older) with the
key present in `/etc/maburplay.json` fails strict keys at boot — strip it
first. Range [0, 60]; the bench A/B that sizes it is in
`docs/observability.md` under the regulator line.

## `maburplay` — the vsync-locked regulator (2026-08-31)

The three new player config keys — `display.vsync_lock`,
`display.vsync_lead_ms`, `display.lat_log_dir` (removed 2026-09-06 — see
below) — are **additive, with in-code defaults**. Strict keys only rejects
a key that is UNPRESENT in the binary but PRESENT in the file; it says
nothing about a key that is merely absent from the file, which just takes
the compiled-in default.
So this swap, unlike the RC-version and venc flag days above, needs
**no config edit before the binary swap**: drop in the new `maburplay`
against the existing `/etc/maburplay.json` and it boots with
`vsync_lock: true`, `vsync_lead_ms: 6`, `lat_log_dir: "/media/dvr/log"`
without either key ever having been written to the file.

**Rollback gotcha, the other direction.** Strict keys still cuts the
other way once the file HAS been touched: if `/etc/maburplay.json` picks
up any `display.vsync_*` key or `lat_log_dir` — which the vsync A/B
protocol does, by design, since it toggles `vsync_lock` in the file
between arms (`docs/bench-protocols-latency-2026-08-31.md`) — a
pre-vsync `maburplay.pre-vsync` binary will reject the file at boot and
`S97maburplay` stops respawning on that exit code, same failure shape as
the drone/GS mismatch above but silent (no crash-loop to notice; the
player just doesn't come back). **Strip the `display.vsync_*` /
`lat_log_dir` keys from the file BEFORE dropping back to
`maburplay.pre-vsync`** — config-before-binary applies rolling back too,
not only rolling forward.

## 2026-09-04 RC_VERSION 6 (probe stream)

The discrete s3 probe is replaced by an always-on probe-stream canary
(`docs/link-adaptation.md` "Probe stream"); the RCF head gains a fixed
`probe_profile` byte and `RCF_F_PROBE_ENH`/`CAP_ENH_PROBE`/`CAP_S3_PROBE`
are deleted — RC_VERSION 5 → 6, so this is a version-mismatch flag day
like the 2026-08-12/2026-08-15 bumps above: a mismatched pair rejects
each other's frames in both directions and, since DISC_ACK carries
`CAP_FRAME_WIRE`, looks like no video at all until both ends match.

**GS config first.** `/etc/maburgs.toml` gains the optional `link.probe`
block (live defaults if omitted: `enable true, rung_offset 1, clean_ms
2000, max_util <0 ⇒ down_util, min_syms 40, silence_ms 500, pin_mcs -1`)
and loses five flat keys that now FAIL BOOT — delete them before the
binary swap:

```sh
grep -nE '^[[:space:]]*probe_(ms|settle_ms|max_util|s3_min_syms|s3_silence_ms)[[:space:]]*=' /etc/maburgs.toml
```

`probe_s3_min_syms` has a successor, `link.s3_min_syms` (default 50) —
carry over a non-default value before deleting the old key.

Drone config (`/etc/mabur.toml`) is untouched — there is no drone-side
probe config; the RCF byte is the only switch. Sequence:

1. Edit `/etc/maburgs.toml` on the GS (delete the five keys above; add
   `link.probe` or rely on defaults).
2. Swap `maburgs` AND `tools/maburtop.py` together (the sideport's
   `classes` keys change shape — `s2`/`s3` gone, `probe` added — and an
   old `maburtop` against a new `maburgs` mis-renders the signal panel).
3. Swap `maburd`.

Between steps 2 and 3 the pair is mismatched (RC_VERSION 5 drone vs 6
GS): no control link, no video, exactly the deploy-window symptom this
page opens with. Finish the deploy; do not restart either daemon hoping
to fix it.

Rollback is paired, as always: `maburgs.pre-probe` / `maburd.pre-probe`
with `maburgs.json.pre-probe` (the five flat keys restored, `link.probe`
removed) alongside the GS binary — an old GS binary against a config
carrying `link.probe` fails strict keys at boot just as surely as the
reverse.

## 2026-09-06 debug-log consolidation

The five separate debug files (`ctl-NNNN_<date>.log`, `probe-NNNN_<date>.log`,
`au-NNNN.log`, `flight-NNNN.jsonl`, `lat-NNNN.log`, the last three written by
the standalone `flightrec.py` recorder) collapse into one per-session
directory, `<debug_log.dir>/NNNN/` holding `ctl.log`, `probe.log`, `au.log`,
`flight.jsonl` and `lat.log`, gated by one knob: `debug_log.enable`. This is
**config-before-binary on BOTH ground-station daemons**, not just one — an
unknown key fails boot into the usual 2 s respawn loop (see the top of this
page).

- `/etc/maburgs.toml` loses four keys that now FAIL BOOT — delete them
  before swapping `maburgs`: `link.ctl_log`, `link.ctl_log_dir`,
  `link.ctl_log_period_ms`, `link.rung_stats.rung_log_period_s`. Their
  replacements live under the new `debug_log` block (`enable`, `dir`,
  `ctl_period_ms`, `rung_period_s`) — additive, its own key, not a rename
  in place.
- `/etc/maburplay.toml` loses `display.lat_log_dir` — also now FAIL BOOT.
  maburplay holds no logging config of its own any more: it follows the
  `/tmp/mabur-session` marker maburgs writes and needs no key at all.
- **After both binaries land, delete `/root/flightrec.py` and
  `/etc/init.d/S95flightrec` from the ground station.** A leftover S95
  keeps boot-starting the old recorder, which still binds UDP `:8300` —
  starving `maburtop`, which now binds that port directly — and still
  attaches to the AU ring as a second outside reader, racing maburgs'
  own in-process ring publish for no reason.
- **Half-deployed signature:** a session directory containing every file
  EXCEPT `lat.log` (`ctl.log`, `probe.log`, `au.log`, `flight.jsonl` all
  present) means `maburgs` was updated but `maburplay` was not — the old
  player is still writing (or failing to write) `lat-NNNN.log` the old
  way, or not writing anything if it never had `lat_log_dir` set. Swap
  `maburplay` to close the gap; there is no wire-format risk in doing so
  on its own schedule, since this is config/logging only, not an
  RC_VERSION bump.

## 2026-09-09 GS card auto-scan

`maburgs` discovers its radios itself. With no `[[radio.cards]]` in
`/etc/maburgs.toml` it probes the USB bus at startup and receives on every
supported card it finds, identifying each by chip-id — one device-recipient
vendor read of `SYS_CFG2`, no claim and no reset, so a non-radio device that
shares the Realtek VID is asked one question, STALLs it, and is left alone.
Cards are ordered by physical port (`bus-port.path`, logged at startup as
`cards: card 0 = 0bda:a81a at usb 2-1.1`), so `card 0` in the stats and the
OSD is the same antenna across reboots.

Deploy notes, none of which are wire-format:

- **This is a GS-only change.** No RC_VERSION bump, no drone deploy, no flag
  day. Swap `maburgs` alone.
- **Binary before config, unusually.** The new binary reads an old config
  (explicit `[[radio.cards]]` blocks) exactly as before — the list still
  pins, and skips the probe entirely. So swap the binary, confirm, then
  delete the card blocks.
- **The rollback trap is silent.** An OLD `maburgs` with the NEW card-less
  config does not fail boot: absent `radio.cards` used to mean *one default
  card*, so it comes up receiving on a single radio and looks exactly like a
  dead antenna. Rolling back the binary means restoring
  `/etc/maburgs.toml.pre-autoscan` alongside it — the standing paired-rollback
  rule, with no error message to remind you.
- **An empty bus exits 1** after a 15 s wait rather than running blind; the
  `S96maburgs` wrapper respawns at 2 s, which is also the retry a card that
  enumerates late needs. The 2 s per-card reopen retry is unchanged.
- **`radio.tx_card` is no longer range-checked at config load** under
  auto-scan — the count is hardware, not config. A pin past the cards found
  logs `warning: radio.tx_card N but only M card(s) found; falling back to
  auto-select` and runs on auto, rather than costing the uplink because one
  card did not enumerate.

## 2026-09-10 RC_VERSION 7 (TX-power calibration kit)

Two new frame types, `T_CAL_CMD` and `T_CAL_RESULT`, carry the
`maburcal` calibration protocol between `maburgs` and `maburd`
(`docs/calibration.md`) — `RC_VERSION` 6 → 7, a version-mismatch flag day
like the 2026-08-12/2026-08-15/2026-09-04 bumps above. A mismatched pair
rejects each other's frames in both directions and, since `DISC_ACK`
carries `CAP_FRAME_WIRE`, **looks like no video at all** between the two
swaps — exactly the stale-caps restart deadlock's symptom. Restarting
either daemon will not fix it; finish the deploy.

No config keys move. This is a binary-only flag day on both ends —
deploy `maburd` and `maburgs` together, in either order, and confirm
video before treating the deploy as done. Rollback is the usual paired
one: an old `maburd`/`maburgs` pair (`.pre-cal` binaries, if kept) talks
`RC_VERSION` 6 to itself and needs no config change to go with it, since
none of this bump touches config.

`ausniff` is the standing gate for this change (`tools/bench/ausniff.py`)
— run it once both binaries are up, expecting ~59.8 fps and 0 gaps; a
`frame_id_gap` on the very first post-deploy pass can be a phantom from
the restart itself, so take a second pass before treating it as real.
