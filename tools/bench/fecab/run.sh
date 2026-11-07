#!/usr/bin/env bash
# tools/bench/fecab/run.sh -- bench A/B rig for FEC placement. Each arm
# swaps the drone's maburd binary (a bind mount, not a reflash) and pushes
# a fixed host config with one knob (bitrate_max_kbps) overridden, then
# drives the GS loss-sim rig through three steps via sched.py (clean,
# l2b1, l5b3) while snapshotting the drone's per-thread CPU. Analysis is
# tools/bench/fecab/analyze.py OUTDIR SESSROOT, against the GS session(s)
# this run produced.
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

scp_retry() {
  local i rc
  for i in $(seq 1 30); do
    scp -O -o BatchMode=yes -o ConnectTimeout=8 -o LogLevel=ERROR "$1" "$2"
    rc=$?; [ $rc -ne 255 ] && return $rc
    echo "(scp retry $i)" >&2; sleep 5
  done
  return 255
}

arm() {
  local name=$1 binary=$2 config=$3 log=$OUT/$name.run
  echo "=== $name binary=$binary config=$config $(date +%T)" | tee "$log"

  R "$G" 'killall maburgs.losssim 2>/dev/null; sleep 2; rm -f /tmp/mabur-session /tmp/sched.log; pidof maburgs.losssim; true'

  R "$D" "umount /usr/bin/maburd 2>/dev/null; mount --bind $binary /usr/bin/maburd && mount | grep maburd"

  local tmpcfg; tmpcfg=$(mktemp)
  if grep -q '^bitrate_max_kbps' "$config"; then
    sed -E "s/^bitrate_max_kbps[[:space:]]*=.*/bitrate_max_kbps = $KBPS/" "$config" > "$tmpcfg"
  else
    cat "$config" > "$tmpcfg"
    printf '\nbitrate_max_kbps = %s\n' "$KBPS" >> "$tmpcfg"
  fi
  if ! scp_retry "$tmpcfg" "$D:/etc/mabur.toml"; then
    echo "scp of $config to $D failed" >&2
    rm -f "$tmpcfg"
    return 1
  fi
  rm -f "$tmpcfg"

  R "$D" "grep -E '^bitrate_max_kbps' /etc/mabur.toml; killall maburd" | tee -a "$log"
  sleep 15

  R "$G" "pidof maburgs.losssim >/dev/null || setsid /tmp/maburgs.losssim -c /tmp/cfg/bench.toml --loss-sim 8303 </dev/null >/tmp/maburgs.log 2>&1 &"
  sleep 25
  R "$G" "setsid python3 /tmp/sched.py $STEP </dev/null >/tmp/sched.log 2>&1 &"

  for sname in clean l2b1 l5b3; do
    sleep 5;              R "$D" "sh -s" < "$SNAP" > "$OUT/$name.$sname.snap0"
    sleep $((STEP - 8));  R "$D" "sh -s" < "$SNAP" > "$OUT/$name.$sname.snap1"
    sleep 3
  done

  R "$G" "until grep -q SCHED_DONE /tmp/sched.log; do sleep 1; done; cat /tmp/sched.log; SD=\$(cat /tmp/mabur-session); cp /tmp/maburgs.log \$SD/maburgs.log; cp /tmp/sched.log \$SD/sched.log; echo session \$SD" | tee -a "$log"
}

for spec in $ARMS; do
  IFS=: read -r name binary config <<< "$spec"
  arm "$name" "$binary" "$config"
done
echo ALL_DONE
