#!/usr/bin/env bash
# Build + deploy maburgs to an OpenIPC SBC GS (BusyBox init): binary + init
# script + config, then restart the service.
# Usage: gs/bundle/install.sh root@<gs-ip> [path/to/maburgs]
# Without a binary argument it builds, then picks the glibc build for the
# board it finds: out/arm64/maburgs on the Radxa ZERO 3, out/arm64/maburgs-opi
# on the Orange Pi Zero 2W (out/arm64/maburgs-static is the musl rollback).
#
# This targets the BusyBox/init.d GS image (service = /etc/init.d/S96maburgs).
# A systemd Radxa uses gs/bundle/maburgs.service instead; that unit file is
# kept in the repo for reference but is not deployed by this script.
set -euo pipefail
cd "$(dirname "$0")/../.."

# Build first so a deploy always ships the current source. Incremental, fast.
# Skipped when a prebuilt binary is passed as $2.
if [ $# -lt 2 ]; then
  bash tools/build-arm64.sh
fi

HOST=${1:?usage: install.sh root@<gs-ip> [binary]}
if [ $# -ge 2 ]; then
  BIN=$2
else
  # The glibc builds are per board (each links that image's own sysroot), so
  # the board decides which one ships.
  MODEL=$(ssh "$HOST" 'tr -d "\0" < /proc/device-tree/model' || true)
  case "$MODEL" in
    *OrangePi*) BIN=out/arm64/maburgs-opi ;;
    *Radxa*) BIN=out/arm64/maburgs ;;
    *) echo "error: unknown GS board '$MODEL' -- pass the binary as the second argument" >&2
       exit 1 ;;
  esac
fi
[ -f "$BIN" ] || { echo "error: $BIN not built (run tools/build-arm64.sh)"; exit 1; }

# Stop maburgs (keeping one rollback) before overwriting its binary: a running
# maburgs holds /usr/local/bin/maburgs busy, so scp'ing over it in place fails
# with "Text file busy". Downtime spans the copy, which is fine here.
ssh "$HOST" '
  [ -f /usr/local/bin/maburgs ] && cp -a /usr/local/bin/maburgs /usr/local/bin/maburgs.prev
  # The device may carry an older wrapper whose stop exits nonzero when
  # maburgs is not running, or no wrapper yet: never let that abort the
  # install before anything is copied.
  [ -x /etc/init.d/S96maburgs ] && /etc/init.d/S96maburgs stop
  # stop returns before the daemon has exited; copying over a binary that is
  # still running fails with "Text file busy".
  for i in 1 2 3 4 5 6 7 8 9 10; do pidof maburgs >/dev/null || break; sleep 1; done
  if pidof maburgs >/dev/null; then echo "error: maburgs did not exit" >&2; exit 1; fi
  true
'

# -O forces the legacy SCP transfer protocol so this works against targets with
# no /usr/libexec/sftp-server (BusyBox); also accepted by full OpenSSH.
scp -O "$BIN" "$HOST:/usr/local/bin/maburgs"
scp -O gs/bundle/S96maburgs "$HOST:/etc/init.d/S96maburgs"
# maburtop and maburcal go to /usr/bin: the GS shell's default PATH does not
# include /usr/local/bin (maburgs itself is only ever launched by absolute
# path).
scp -O tools/maburtop.py "$HOST:/usr/bin/maburtop"
scp -O gs/bundle/maburcal "$HOST:/usr/bin/maburcal"
# Config: seed the default only if NEITHER a tuned .toml NOR a legacy .json
# exists. A bare "no .toml" check would seed the repo default over an
# un-converted device's tuned /etc/maburgs.json -- the daemon then boots
# cleanly on repo defaults (no crash, no signal; the startup defaulted-keys
# report can't catch it either, since it compares against compiled defaults,
# not the old config) and silently runs on the wrong values. Convert first.
if ssh "$HOST" '[ -f /etc/maburgs.toml ]'; then
  : # tuned config already present -- never clobber it
elif ssh "$HOST" '[ -f /etc/maburgs.json ]'; then
  echo "error: $HOST has /etc/maburgs.json but no /etc/maburgs.toml -- convert" >&2
  echo "the tuned JSON config to TOML by hand before deploying (docs/deploy.md," >&2
  echo "'JSON to TOML cutover'). Refusing to seed the repo default over it." >&2
  exit 1
else
  scp -O gs/bundle/maburgs.default.toml "$HOST:/etc/maburgs.toml"
fi

# Start via the init script. S96maburgs' start does `rmmod 8812eu` so devourer
# can claim the cards over libusb; its daemon stdout is redirected to
# /tmp/maburgs.log, so start returns without holding this ssh channel open.
ssh "$HOST" '
  set -e
  chmod +x /usr/local/bin/maburgs /etc/init.d/S96maburgs /usr/bin/maburtop /usr/bin/maburcal
  /etc/init.d/S96maburgs start
'
echo "installed. logs: ssh $HOST 'tail -f /tmp/maburgs.log'"
