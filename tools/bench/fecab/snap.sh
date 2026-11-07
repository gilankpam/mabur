# tools/bench/fecab/snap.sh -- drone-side CPU snapshot for the FEC
# placement A/B (tools/bench/fecab/run.sh). Run via `ssh $DRONE "sh -s" <
# snap.sh`: no shebang, no execute bit needed, POSIX sh only (busybox ash
# on the drone). Prints maburd's per-thread utime+stime (clock ticks) and
# the system-wide /proc/stat `cpu` line; tools/bench/fecab/analyze.py
# diffs a snap0/snap1 pair to get per-thread CPU%.
P=$(pidof maburd)
echo "SNAP $(cut -d' ' -f1 /proc/uptime) pid=$P"
head -1 /proc/stat
for t in /proc/$P/task/*; do
  s=$(cat $t/stat); s=${s##*) }; set -- $s
  echo "T ${t##*/} $(tr ' ' _ <$t/comm) ${12} ${13}"
done
