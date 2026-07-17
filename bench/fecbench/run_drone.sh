#!/bin/sh
# Runs fecbench-arm ON the drone (SSC338Q) with the platform QUIESCED:
# stops BOTH maburd and waybeam so the bench owns the SoC. Does NOT restart
# them (bench rig; bring services back with restore_drone.sh or a reboot).
set -e
cd "$(dirname "$0")"
DRONE=${DRONE:-root@192.168.10.152}
MODE=${1:-all}
SIM=${2:-6}

scp -O fecbench-arm "$DRONE:/tmp/fecbench"
ssh "$DRONE" "/etc/init.d/S96mabur stop; /etc/init.d/S95waybeam stop; sleep 1; \
  ps | grep -E 'maburd|waybeam' | grep -v grep && echo 'WARN: still running'; \
  /tmp/fecbench $MODE $SIM"
