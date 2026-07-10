#!/usr/bin/env bash
# Deploy maburd + its init script + default config to a camera running
# waybeam. maburd is a fully static ARM binary (see tools/build-arm.sh) so
# no shared libs (e.g. libusb) need to be copied to the target.
set -euo pipefail
HOST=${1:?usage: install.sh root@<camera-ip>}
cd "$(dirname "$0")/.."

[ -f out/arm/maburd ] || { echo "run tools/build-arm.sh first" >&2; exit 1; }

scp out/arm/maburd "$HOST:/usr/bin/maburd"
ssh "$HOST" '[ -f /etc/mabur.json ]' || scp bundle/mabur.default.json "$HOST:/etc/mabur.json"
scp bundle/S96mabur "$HOST:/etc/init.d/S96mabur"

# NOTE: verify the json_cli flag spelling below against
# ../waybeam_venc/tools on the first real deploy — this repo cannot run
# on-device commands, so these are unverified against the live waybeam
# build. The spec records json_cli as the supported config mechanism.
ssh "$HOST" '
  set -e
  chmod +x /usr/bin/maburd /etc/init.d/S96mabur
  json_cli -s .outgoing.enabled true -i /etc/waybeam.json
  json_cli -s .outgoing.server "shm://mabur" -i /etc/waybeam.json
  json_cli -s .outgoing.streamMode "rtp" -i /etc/waybeam.json
  /etc/init.d/S95waybeam restart || true
  /etc/init.d/S96mabur restart
'

echo "installed. logs: ssh $HOST 'cat /tmp/mabur.log'  (or stderr via serial)"
