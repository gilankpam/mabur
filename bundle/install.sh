#!/usr/bin/env bash
# Build + deploy maburd + its init script + default config to a camera running
# waybeam, then restart the service. maburd is a fully static ARM binary (see
# tools/build-arm.sh) so no shared libs (e.g. libusb) need to be copied.
# Usage: bundle/install.sh root@<camera-ip> [path/to/maburd]
#
# waybeam's shm RTP output (waybeam.json .outgoing = shm://mabur / rtp) is a
# one-time, out-of-band setup on the camera — this script does not touch it.
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

# Stop maburd before overwriting its binary: a running maburd holds
# /usr/bin/maburd busy, so scp'ing over it in place fails with "Text file
# busy". Downtime spans the copy, which is fine here.
ssh "$HOST" '/etc/init.d/S96mabur stop'

# -O forces the legacy SCP transfer protocol: OpenIPC/BusyBox targets ship no
# /usr/libexec/sftp-server, so modern scp's default sftp mode fails ("scp:
# Connection closed"). -O is also accepted by full OpenSSH, so it's safe here.
scp -O "$BIN" "$HOST:/usr/bin/maburd"
ssh "$HOST" '[ -f /etc/mabur.json ]' || scp -O bundle/mabur.default.json "$HOST:/etc/mabur.json"
scp -O bundle/S96mabur "$HOST:/etc/init.d/S96mabur"

ssh "$HOST" '
  set -e
  chmod +x /usr/bin/maburd /etc/init.d/S96mabur
  /etc/init.d/S96mabur start
'

echo "installed. logs: ssh $HOST 'cat /tmp/mabur.log'  (or stderr via serial)"
