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
the stale-caps restart deadlock, which will send you to `restart
maburd`. That will not help. Recovery is to finish the deploy. Deploy
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

