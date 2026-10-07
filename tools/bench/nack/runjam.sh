#!/usr/bin/env bash
# Jammer arms (docs/fec-nack.md "Bench"): one fresh maburgs per arm config,
# the host 8822EU jammer (tools/bench/benchjam.sh) on the op channel for DUR s.
# Arms are the cfg/<name>.toml files, staged on the GS under /tmp/cfg/.
#
# The jammer disturbs the GS management Wi-Fi, so every ssh retries up to
# 18 x 5 s (90 s); an arm whose maburgs restart did not write a NEW
# /tmp/mabur-session aborts the whole run (exit 1) rather than measuring a
# dead or stale daemon.
#
# Before a run: stop the production daemon in its own ssh
# (`/etc/init.d/S96maburgs stop`, verify with ps; its wrapper respawns
# anything else it owns) and stage the as-built out/arm64/maburgs as GSBIN
# (default /usr/local/bin/maburgs.fecnack -- deliberately not the spike's
# maburgs.nack, which speaks the RC 14 spike wire and cannot link an RC 15
# drone).
#
# Env: DUR (s, 300), PPS (jammer frames/s, 180 = the 2026-10-05 calibration;
# recalibrate per docs/fec-nack.md), CH (op channel, 144), ARMS, GSBIN (the
# side GS binary each arm runs; killed by name between arms), GSARGS (extra
# args, e.g. "--loss-sim 8303" for a loss-sim build).
cd /home/gilankpam/Projects/drone/mabur
G=root@10.18.0.1
DUR=${DUR:-300}; PPS=${PPS:-180}; CH=${CH:-144}
GSBIN=${GSBIN:-/usr/local/bin/maburgs.fecnack}; GSARGS=${GSARGS:-}
# Each arm killalls GSBIN by basename: never let that name the production
# daemon (its wrapper would respawn it beside the arm's instance).
[ "$(basename "$GSBIN")" = maburgs ] && { echo "refusing: GSBIN is the production daemon name"; exit 1; }
S=$(dirname "$0")

# ssh with retry: 18 tries, 5 s apart. Returns the last ssh's status.
rssh() {
  local i rc=1
  for i in $(seq 1 18); do
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$@" && return 0
    rc=$?
    echo "  ssh try $i failed (rc=$rc), retrying in 5 s" >&2
    sleep 5
  done
  return $rc
}

prev_session=""
run_arm() {
  local name=$1 session
  echo "=== $name start $(date +%T) pps=$PPS ch=$CH"
  rssh $G "killall $(basename "$GSBIN") 2>/dev/null; sleep 1; rm -f /tmp/mabur-session; true" || exit 1
  # Idempotent launch: a retry after a dropped ssh kills the instance the
  # failed try may have started instead of running a second one beside it.
  rssh $G "killall $(basename "$GSBIN") 2>/dev/null; sleep 1; rm -f /tmp/mabur-session; setsid $GSBIN -c /tmp/cfg/$name.toml $GSARGS </dev/null >/tmp/maburgs.log 2>&1 & true" || exit 1
  sleep 20
  session=$(rssh $G 'cat /tmp/mabur-session 2>/dev/null; true')
  if [ -z "$session" ] || [ "$session" = "$prev_session" ]; then
    echo "!!! $name: maburgs restart produced no new /tmp/mabur-session (got '${session}', previous '${prev_session}'); aborting" >&2
    exit 1
  fi
  prev_session=$session
  echo "  session $session"
  nohup bash tools/bench/benchjam.sh --channel "$CH" --pps "$PPS" --secs $((DUR+5)) > "$S/jam_$name.log" 2>&1 &
  sleep 5
  "$S/gsdelta.sh" "$DUR"
  sleep 8
  rssh $G 'SD=$(cat /tmp/mabur-session); cp /tmp/maburgs.log $SD/maburgs.log; echo "session $SD"; grep "^stats:" /tmp/maburgs.log | tail -1 | grep -o "frames\[clean/trunc/drop\]=[0-9/]*"; true'
  echo "=== $name end $(date +%T) -- copy $session here and run $S/arm_report.py on it"
}
for a in ${ARMS:-A0_control A1_nack}; do run_arm "$a"; done
echo JAM_ARMS_DONE
