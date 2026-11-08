#!/usr/bin/env bash
# tools/bench/fecab/run.sh -- bench A/B rig for FEC placement. Each arm
# swaps the drone's maburd binary (a bind mount, not a reflash) -- unmounted
# lazily (umount -l, looped) since a plain umount fails while the running
# maburd still holds the bind open, which would otherwise leave it stacking
# a fresh bind on top every arm -- and pushes a fixed host config with one
# knob (bitrate_max_kbps) overridden, then
# drives the GS loss-sim rig through three steps via sched.py (clean,
# l2b1, l5b3) while snapshotting the drone's per-thread CPU and, once
# sched.py finishes, capturing the drone's own stderr log for the arm
# (/tmp/mabur.log -- the init wrapper truncates it on every maburd
# respawn, so it has to be read before the NEXT arm's swap). Analysis is
# tools/bench/fecab/analyze.py OUTDIR SESSROOT, against the GS session(s)
# this run produced. The run saves the drone's live /etc/mabur.toml once,
# before the first arm overwrites it, and restore_drone puts it back. On
# exit -- normal completion OR an abort anywhere in the run -- that same
# restore_drone trap also unmounts any stacked binds and kills maburd one
# last time so the drone comes back up on its installed /usr/bin/maburd
# and its own config rather than being left on whichever arm's
# binary/config was applied last.
#
# An aborted run can leave the GS's maburgs.losssim and sched.py running
# under setsid (killed only by the NEXT run's first arm, or by hand with
# `killall maburgs.losssim` on the GS).
#
# Env (no defaults -- every bench run names its own targets):
#   DRONE  drone ssh target (e.g. root@<drone-ip>)
#   GS     GS ssh target, reached over its Wi-Fi hotspot (drops; see R())
#   OUT    local output directory for .run files and CPU snapshots
#   KBPS   drone bitrate_max_kbps to set for every arm
#   ARMS   space-separated "name:binary:config" triples -- binary is a
#          path already present ON THE DRONE, config a HOST TOML file
#   STEP   seconds per loss-sim step (default 60)
set -u
D=${DRONE:?}; G=${GS:?}; OUT=${OUT:?}; KBPS=${KBPS:?}; ARMS=${ARMS:?}
STEP=${STEP:-60}
SSH="ssh -o BatchMode=yes -o ConnectTimeout=8 -o LogLevel=ERROR"
SELF_DIR=$(dirname "$(readlink -f "$0")")
SNAP="$SELF_DIR/snap.sh"
mkdir -p "$OUT"

# ssh retry wrapper: the GS is reached over its Wi-Fi hotspot, which drops
# mid-session -- retry on exit 255 (ssh's own connection-failure code,
# never a remote command's exit status) rather than fail the whole arm.
R() { local i rc; for i in $(seq 1 30); do $SSH "$@"; rc=$?; [ $rc -ne 255 ] && return $rc; echo "(ssh retry $i)" >&2; sleep 5; done; return 255; }

# Save the drone's live config once, before the first arm's scp overwrites
# it -- only if a previous aborted run hasn't already left one there (a
# re-run after an abort must not save over the ORIGINAL config with some
# arm's modified one). restore_drone puts this back on every exit path.
R "$D" "[ -f /tmp/mabur.toml.fecab-orig ] || cp /etc/mabur.toml /tmp/mabur.toml.fecab-orig"

scp_retry() {
  local i rc
  for i in $(seq 1 30); do
    scp -O -o BatchMode=yes -o ConnectTimeout=8 -o LogLevel=ERROR "$1" "$2"
    rc=$?; [ $rc -ne 255 ] && return $rc
    echo "(scp retry $i)" >&2; sleep 5
  done
  return 255
}

# Restores the drone to its installed maburd: lazily unmount every stacked
# bind (umount -l, looped -- a plain umount fails ("target busy") while
# maburd still has the bind open, and looped because each arm that ran may
# have stacked another bind on top of the last) and killall so the init
# loop respawns the now-unbound /usr/bin/maburd. Runs on every exit path
# via the trap below -- normal completion, an aborted arm, or Ctrl-C -- so
# the drone is never left running a bench binary after this script has
# stopped driving it.
restore_drone() {
  echo "restoring drone: config from /tmp/mabur.toml.fecab-orig, unmounting /usr/bin/maburd bind(s), killing maburd to respawn the installed binary" >&2
  local remaining
  remaining=$(R "$D" "[ -f /tmp/mabur.toml.fecab-orig ] && cp /tmp/mabur.toml.fecab-orig /etc/mabur.toml; n=0; while mount | grep -q ' /usr/bin/maburd '; do umount -l /usr/bin/maburd || break; n=\$((n+1)); [ \$n -ge 20 ] && break; done; killall maburd 2>/dev/null; mount | grep -c ' /usr/bin/maburd '; true")
  if [ "$remaining" != "0" ]; then
    echo "WARNING: $remaining bind(s) still mounted at /usr/bin/maburd on $D after restore -- drone may still be running a bench binary" >&2
  fi
}
trap restore_drone EXIT

arm() {
  local name=$1 binary=$2 config=$3 log=$OUT/$name.run
  echo "=== $name binary=$binary config=$config $(date +%T)" | tee "$log"

  R "$G" 'killall maburgs.losssim 2>/dev/null; sleep 2; rm -f /tmp/mabur-session /tmp/sched.log /tmp/sched.pid; pidof maburgs.losssim; true'

  if ! R "$D" "[ -x '$binary' ]"; then
    echo "FAILED bind: $binary not found or not executable on $D" | tee -a "$log" >&2
    return 1
  fi
  # A plain umount fails ("target busy") while the previous arm's maburd
  # still holds the bind open, so it has to be lazy (umount -l) and looped
  # -- one unmount per existing bind -- or a failed umount leaves the old
  # bind in place and the new mount --bind just stacks another one on top.
  if ! R "$D" "n=0; while mount | grep -q ' /usr/bin/maburd '; do umount -l /usr/bin/maburd || break; n=\$((n+1)); [ \$n -ge 20 ] && break; done; mount --bind '$binary' /usr/bin/maburd"; then
    echo "FAILED bind: mount --bind $binary -> /usr/bin/maburd failed on $D" | tee -a "$log" >&2
    return 1
  fi
  local nbinds
  nbinds=$(R "$D" "mount | grep -c ' /usr/bin/maburd '")
  if [ "$nbinds" != "1" ]; then
    echo "FAILED bind: expected exactly one bind at /usr/bin/maburd on $D after mount --bind, found $nbinds -- a stale bind from a previous arm likely failed to unmount" | tee -a "$log" >&2
    return 1
  fi

  if ! grep -q '^bitrate_max_kbps' "$config"; then
    echo "FAILED: $config has no bitrate_max_kbps key -- appending one blind could land it in whatever TOML table is last and make maburd respawn-loop; add the key to the config instead" | tee -a "$log" >&2
    return 1
  fi
  local tmpcfg; tmpcfg=$(mktemp)
  sed -E "s/^bitrate_max_kbps[[:space:]]*=.*/bitrate_max_kbps = $KBPS/" "$config" > "$tmpcfg"
  if ! scp_retry "$tmpcfg" "$D:/etc/mabur.toml"; then
    echo "FAILED scp: $config -> $D:/etc/mabur.toml" | tee -a "$log" >&2
    rm -f "$tmpcfg"
    return 1
  fi
  rm -f "$tmpcfg"

  R "$D" "grep -E '^bitrate_max_kbps' /etc/mabur.toml; killall maburd" | tee -a "$log"
  sleep 15

  # Confirm the respawned maburd really is running the bound file, not a
  # stale process that outlived the mount swap or the previous arm's
  # binary -- hash the RUNNING process (/proc/<pid>/exe) rather than the
  # path /usr/bin/maburd, which only proves what's mounted there, not what
  # any already-running process has open.
  local pid md5out h1 h2
  pid=$(R "$D" "pidof maburd" | awk '{print $1}')
  if [ -z "$pid" ]; then
    echo "FAILED bind: no running maburd pid on $D after respawn" | tee -a "$log" >&2
    return 1
  fi
  md5out=$(R "$D" "md5sum /proc/$pid/exe '$binary'")
  if [ $? -ne 0 ] || [ -z "$md5out" ]; then
    echo "FAILED bind: md5sum check on $D failed after respawn" | tee -a "$log" >&2
    return 1
  fi
  echo "$md5out" | tee -a "$log"
  h1=$(awk 'NR==1{print $1}' <<< "$md5out")
  h2=$(awk 'NR==2{print $1}' <<< "$md5out")
  if [ -z "$h1" ] || [ "$h1" != "$h2" ]; then
    echo "FAILED bind: running maburd (pid $pid) md5 ($h1) != $binary md5 ($h2)" | tee -a "$log" >&2
    return 1
  fi

  R "$G" "pidof maburgs.losssim >/dev/null || setsid /tmp/maburgs.losssim -c /tmp/cfg/bench.toml --loss-sim 8303 </dev/null >/tmp/maburgs.log 2>&1 &"
  sleep 25
  # Idempotent start: an ssh retry here (R retries only on ssh's own exit
  # 255, which fires on a dropped connection whether or not the remote
  # command already ran) must not launch a second sched.py writing the
  # same /tmp/sched.log. BusyBox has no pgrep, and a `ps | grep sched.py`
  # run via the same `sh -c` would match its own command line -- so a pid
  # file written by the launching shell is the robust check instead.
  # The pid is written from INSIDE the backgrounded shell (echo $$), then
  # that same shell execs python3 in place: the pidfile always names the
  # actual sched.py process, never an intermediate setsid fork whose own
  # pid could already be gone by the time something checks it.
  local inner="echo \$\$ > /tmp/sched.pid; exec python3 /tmp/sched.py $STEP </dev/null >/tmp/sched.log 2>&1"
  R "$G" "if [ -f /tmp/sched.pid ] && kill -0 \$(cat /tmp/sched.pid) 2>/dev/null; then echo \"sched.py already running pid=\$(cat /tmp/sched.pid)\"; else rm -f /tmp/sched.pid /tmp/sched.log; setsid sh -c '$inner' </dev/null >/dev/null 2>&1 & sleep 1; echo \"sched.py started pid=\$(cat /tmp/sched.pid)\"; fi" | tee -a "$log"

  for sname in clean l2b1 l5b3; do
    sleep 5;              R "$D" "sh -s" < "$SNAP" > "$OUT/$name.$sname.snap0"
    sleep $((STEP - 8));  R "$D" "sh -s" < "$SNAP" > "$OUT/$name.$sname.snap1"
    sleep 3
  done

  R "$G" "until grep -q SCHED_DONE /tmp/sched.log; do sleep 1; done; cat /tmp/sched.log; SD=\$(cat /tmp/mabur-session); cp /tmp/maburgs.log \$SD/maburgs.log; cp /tmp/sched.log \$SD/sched.log; echo session \$SD" | tee -a "$log"

  # The drone's own stderr log for this arm, read now -- the init wrapper
  # truncates /tmp/mabur.log on every maburd respawn, so it must be read
  # before the NEXT arm's bind swap and killall, not after this script
  # exits. analyze.py parses this for the fec_worker backlog_drops/
  # frame_end_us gauges (tools/bench/fecab/analyze.py).
  R "$D" "cat /tmp/mabur.log" > "$OUT/$name.dlog"
}

for spec in $ARMS; do
  IFS=: read -r name binary config <<< "$spec"
  if ! arm "$name" "$binary" "$config"; then
    echo "ABORT: arm $name failed -- stopping the run" >&2
    exit 1
  fi
done
echo ALL_DONE
