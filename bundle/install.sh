#!/usr/bin/env bash
# Build + deploy maburd + its init script + default config to a camera, then
# restart the service. maburd is a DYNAMIC glibc ARM binary (venc runs
# in-process; see tools/build-arm.sh); every library it NEEDs is
# already on the OpenIPC rootfs, and libusb is still linked statically, so
# nothing extra is copied.
# Usage: bundle/install.sh root@<camera-ip> [path/to/maburd]
#
# maburd runs the encoder itself (no waybeam). A camera still running
# waybeam must have it retired first (stop it, chmod a-x
# /etc/init.d/S95waybeam, move the binary aside) or two processes fight over
# the MI pipeline and neither encodes.
set -euo pipefail
cd "$(dirname "$0")/.."

# Build first so a deploy always ships the current source. Incremental, fast.
# Skipped when a prebuilt binary is passed as $2.
if [ $# -lt 2 ]; then
  bash tools/build-arm.sh
fi

HOST=${1:?usage: install.sh root@<camera-ip> [binary]}
BIN=${2:-out/arm/maburd}
[ -f "$BIN" ] || { echo "error: $BIN not built (run tools/build-arm.sh)" >&2; exit 1; }

# The wrapper is installed as /etc/init.d/S00mabur -- the name the OpenIPC
# image uses (firmware/package/mabur/mabur.mk), so busybox rcS starts
# maburd first. The repo file keeps its historical name, bundle/S96mabur.
# A camera with an old /etc/init.d/S96mabur gets it stopped and removed
# below, so one name owns the service. (Both copies share one pidfile, so
# the second start at boot is a no-op, but stop/start by the wrong name
# and the docs disagreeing is how a deploy goes wrong.)
#
# Stop maburd before overwriting its binary: a running maburd holds
# /usr/bin/maburd busy, so scp'ing over it in place fails with "Text file
# busy". Downtime spans the copy, which is fine here.
ssh "$HOST" 'for s in S00mabur S96mabur; do
  [ -x /etc/init.d/$s ] && /etc/init.d/$s stop
done; true'

# -O forces the legacy SCP transfer protocol: OpenIPC/BusyBox targets ship no
# /usr/libexec/sftp-server, so modern scp's default sftp mode fails ("scp:
# Connection closed"). -O is also accepted by full OpenSSH, so it's safe here.
scp -O "$BIN" "$HOST:/usr/bin/maburd"

# Config: seed the default only if NEITHER a tuned .toml NOR a legacy .json
# exists. A bare "no .toml" check would seed the repo default over an
# un-converted device's tuned /etc/mabur.json -- the daemon then boots
# cleanly on repo defaults (no crash, no signal; the startup defaulted-keys
# report can't catch it either, since it compares against compiled defaults,
# not the old config) and silently flies on the wrong values. Convert first.
if ssh "$HOST" '[ -f /etc/mabur.toml ]'; then
  : # tuned config already present -- never clobber it
elif ssh "$HOST" '[ -f /etc/mabur.json ]'; then
  echo "error: $HOST has /etc/mabur.json but no /etc/mabur.toml -- convert the" >&2
  echo "tuned JSON config to TOML by hand before deploying (docs/deploy.md," >&2
  echo "'JSON to TOML cutover'). Refusing to seed the repo default over it." >&2
  exit 1
else
  scp -O bundle/mabur.default.toml "$HOST:/etc/mabur.toml"
fi
scp -O bundle/S96mabur "$HOST:/etc/init.d/S00mabur"

ssh "$HOST" '
  set -e
  rm -f /etc/init.d/S96mabur
  chmod +x /usr/bin/maburd /etc/init.d/S00mabur
  /etc/init.d/S00mabur start
'

echo "installed. logs: ssh $HOST 'cat /tmp/mabur.log'  (or stderr via serial)"
