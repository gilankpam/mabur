#!/usr/bin/env bash
# Build + deploy maburgs to an OpenIPC SBC GS (BusyBox init): binary + init
# script + config, then restart the service.
# Usage: gs/bundle/install.sh root@<gs-ip> [path/to/maburgs]
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
BIN=${2:-out/arm64/maburgs}
[ -f "$BIN" ] || { echo "error: $BIN not built (run tools/build-arm64.sh)"; exit 1; }

# Stop maburgs (keeping one rollback) before overwriting its binary: a running
# maburgs holds /usr/local/bin/maburgs busy, so scp'ing over it in place fails
# with "Text file busy". Downtime spans the copy, which is fine here.
ssh "$HOST" '
  [ -f /usr/local/bin/maburgs ] && cp -a /usr/local/bin/maburgs /usr/local/bin/maburgs.prev
  /etc/init.d/S96maburgs stop
'

# -O forces the legacy SCP transfer protocol so this works against targets with
# no /usr/libexec/sftp-server (BusyBox); also accepted by full OpenSSH.
scp -O "$BIN" "$HOST:/usr/local/bin/maburgs"
scp -O gs/bundle/S96maburgs "$HOST:/etc/init.d/S96maburgs"
# maburtop goes to /usr/bin: the GS shell's default PATH does not include
# /usr/local/bin (maburgs itself is only ever launched by absolute path).
scp -O tools/maburtop.py "$HOST:/usr/bin/maburtop"
# Config: install the default only if none exists (never clobber a tuned one).
ssh "$HOST" "[ -f /etc/maburgs.json ]" || \
  scp -O gs/bundle/maburgs.default.json "$HOST:/etc/maburgs.json"

# Start via the init script. S96maburgs' start does `rmmod 8812eu` so devourer
# can claim the cards over libusb; its daemon stdout is redirected to
# /tmp/maburgs.log, so start returns without holding this ssh channel open.
ssh "$HOST" '
  set -e
  chmod +x /usr/local/bin/maburgs /etc/init.d/S96maburgs /usr/bin/maburtop
  /etc/init.d/S96maburgs start
'
echo "installed. logs: ssh $HOST 'tail -f /tmp/maburgs.log'"
