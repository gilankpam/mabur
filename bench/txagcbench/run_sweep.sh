#!/usr/bin/env bash
# One-command txagcbench run on the drone <-> GS rig: refuse if the daemons
# own the radios, push binaries, record on GS while the drone sweeps, pull
# the JSONL back, analyze. Extra args pass through to analyze_sweep.py
# (e.g. ./run_sweep.sh --emit-table).
# Spec: docs/superpowers/specs/2026-07-16-txagcbench-design.md.
set -euo pipefail
cd "$(dirname "$0")/../.."

DRONE=${DRONE:-root@192.168.10.152}
GS=${GS:-root@10.18.0.1}
TX_BIN=${TX_BIN:-out/arm/txagcbench-tx}
RX_BIN=${RX_BIN:-out/arm64/txagcbench-rx}
CHANNEL=${CHANNEL:-149}
OUT=${OUT:-/tmp/txagc_sweep.jsonl}
TX_ARGS=${TX_ARGS:-}
RX_ARGS=${RX_ARGS:-}

[ -f "$TX_BIN" ] || { echo "error: $TX_BIN missing — run tools/build-arm.sh" >&2; exit 1; }
[ -f "$RX_BIN" ] || { echo "error: $RX_BIN missing — run tools/build-arm64.sh" >&2; exit 1; }

# The daemons own the radios; a sweep under a live maburd/maburgs measures
# nothing but contention.
if ssh "$DRONE" pidof maburd >/dev/null 2>&1; then
  echo "error: maburd running on $DRONE — stop it first (/etc/init.d/S96mabur stop)" >&2
  exit 1
fi
if ssh "$GS" pidof maburgs >/dev/null 2>&1; then
  echo "error: maburgs running on $GS — stop it first" >&2
  exit 1
fi

scp "$TX_BIN" "$DRONE:/tmp/txagcbench-tx"
scp "$RX_BIN" "$GS:/tmp/txagcbench-rx"

echo "starting recorder on $GS ..."
RX_PID=$(ssh "$GS" "rm -f /tmp/sweep.jsonl; nohup /tmp/txagcbench-rx \
  --channel $CHANNEL --out /tmp/sweep.jsonl $RX_ARGS \
  >/tmp/txagcbench-rx.log 2>&1 & echo \$!")

cleanup() {
  ssh "$GS" "kill -INT $RX_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 8   # RX bring-up (RadioFrontend open_and_start is ~2-3 s; margin)

if ! ssh "$GS" "kill -0 $RX_PID" 2>/dev/null; then
  echo "error: recorder died during bring-up — log tail:" >&2
  ssh "$GS" "tail -20 /tmp/txagcbench-rx.log" >&2 || true
  exit 1
fi

echo "sweeping on $DRONE ..."
ssh "$DRONE" "/tmp/txagcbench-tx --channel $CHANNEL $TX_ARGS"

cleanup   # stop the recorder now so it flushes and releases the radio; the EXIT trap stays armed as a failure-path safety net (idempotent)
sleep 2
scp "$GS:/tmp/sweep.jsonl" "$OUT"
ssh "$GS" "tail -3 /tmp/txagcbench-rx.log" || true

python3 bench/txagcbench/analyze_sweep.py "$OUT" "$@"
