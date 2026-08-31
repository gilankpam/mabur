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
this for you and will produce exactly the crash-loop above if you let
them:** `bundle/install.sh` and `gs/bundle/install.sh` scp the binary
(`:32` in both), then copy the shipped default config ONLY if the device
has none (`bundle/install.sh:33`, `gs/bundle/install.sh:38-39` — "never
clobber a tuned one"), then start the service immediately
(`bundle/install.sh:36-40`, `gs/bundle/install.sh:44-48`). Neither
migrates an existing config, so on a device that has ever been tuned the
four removed keys must be deleted from `/etc/mabur.json` and
`/etc/maburgs.json` BY HAND before the new binaries start.

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
  so this is a paired swap like any other. `bundle/mabur.default.json` carries
  the shipped shape; the bench's tuned values live in `/etc/mabur.json`.
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

## `maburplay` — the vsync-locked regulator (2026-08-31)

The three new player config keys — `display.vsync_lock`,
`display.vsync_lead_ms`, `display.lat_log_dir` — are **additive, with
in-code defaults**. Strict keys only rejects a key that is UNPRESENT in
the binary but PRESENT in the file; it says nothing about a key that is
merely absent from the file, which just takes the compiled-in default.
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
