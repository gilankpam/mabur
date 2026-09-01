#!/usr/bin/env bash
# 656-symbol hole sweep driver — sym 649..661 x mcs 0..7, LDPC+STBC,
# pwr-mode none, ch149, 8s cells. RX runs continuously on the GS
# (seq re-anchor patch); cells are separated by >=4s idle so the RX json
# segments by activity bursts in launch order.
set -u
DRONE=root@192.168.10.152
OUT="$(dirname "$0")/hole_sweep"
mkdir -p "$OUT"
MANIFEST="$OUT/manifest.csv"
echo "idx,mcs,sym,bitrate,start_epoch,end_epoch,tx_frames,exit" > "$MANIFEST"
idx=0
for mcs in 0 1 2 3 4 5 6 7; do
  case $mcs in
    0) rate=3M ;;
    1) rate=6M ;;
    *) rate=8M ;;
  esac
  for sym in 649 650 651 652 653 654 655 656 657 658 659 660 661; do
    idx=$((idx+1))
    for attempt in 1 2; do
      t0=$(date +%s)
      out=$(ssh -o BatchMode=yes -o ConnectTimeout=8 $DRONE \
        "timeout 40 /tmp/linkbench-tx --channel 149 --mcs $mcs --bitrate $rate \
         --time 8 --overhead 0.15 --symbol-size $sym --bpb 4 --window 32 \
         --ldpc --stbc --pwr-mode none --tx-threads 4 2>&1" 2>/dev/null | tail -2)
      rc=$?
      t1=$(date +%s)
      frames=$(printf '%s' "$out" | grep -o '[0-9]* frames' | head -1 | cut -d' ' -f1)
      if printf '%s' "$out" | grep -q '^done:'; then
        echo "$idx,$mcs,$sym,$rate,$t0,$t1,${frames:-0},$rc" >> "$MANIFEST"
        break
      fi
      echo "cell $idx mcs$mcs sym$sym attempt $attempt FAILED: $out" >&2
      if [ "$attempt" = 2 ]; then
        echo "$idx,$mcs,$sym,$rate,$t0,$t1,0,FAIL" >> "$MANIFEST"
      fi
      sleep 5
    done
    sleep 4
  done
  echo "=== mcs $mcs done ($(date +%T)) ===" >&2
done
echo "SWEEP-COMPLETE cells=$idx" >&2
