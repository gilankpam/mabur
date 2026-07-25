#!/bin/sh
# Brings the drone services back after a fecbench session.
set -e
DRONE=${DRONE:-root@192.168.10.152}
ssh "$DRONE" "/etc/init.d/S95waybeam start; sleep 1; /etc/init.d/S96mabur start; \
  sleep 1; ps | grep -E 'maburd|waybeam' | grep -v grep"
