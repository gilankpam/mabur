#!/usr/bin/env bash
# 8822E A-MPDU TX validation spike — drone (RTL8812EU) txdemo -> GS rxdemo,
# driven over ssh from the host, no SDR. Mirrors the cell matrix of
# ../devourer/tests/ampdu_spike.sh; verdict comes from ampdu_e_analyze.py
# (paggr fraction + inter-frame tsfl-delta histogram + unique-counter fps —
# NOT the equal-tsfl burst marker, which does not fire on an 8812EU RX).
#
# First run + findings: docs/dq-spike-findings-2026-08-31.md §11
# (2026-09-01: the E die aggregates; 93% of arrivals at pure MPDU airtime).
#
# Prereqs:
#   - txdemo cross-built for the drone:  cmake --build build-arm-glibc
#     --target txdemo   (env per tools/build-arm.sh), scp -O to drone /tmp
#   - rxdemo cross-built for the GS:     cmake --build build-arm64
#     --target rxdemo   (env per tools/build-arm64.sh), scp to GS /tmp
#   - BOTH daemons stopped (S96mabur on the drone, S96maburgs on the GS);
#     restart + ausniff gate afterwards.
# GS busybox has no pkill — killall only.
set -u
DRONE=${DRONE:-root@192.168.10.152}
GS=${GS:-root@10.18.0.1}
CH=${CH:-161}; RATE=${RATE:-MCS5}; PAYLOAD=${PAYLOAD:-1396}
SECS=${SECS:-8}; AGG=${AGG:-16}; DENSITY=${DENSITY:-7}
OUT=/tmp/ampdu_e_spike   # on the GS
LOGS=${LOGS:-./ampdu_e_logs}
mkdir -p "$LOGS"

ssh $GS "mkdir -p $OUT"

kill_all() {
  ssh $GS 'killall -9 rxdemo 2>/dev/null; for i in 1 2 3 4 5; do ps | grep -v grep | grep -q rxdemo || break; sleep 1; done; true'
  ssh $DRONE 'killall -9 txdemo 2>/dev/null; true'
}
trap kill_all EXIT

run_cell() { # $1 = name, rest = extra TX env
  local name="$1"; shift
  echo "=== cell $name ($*)"
  kill_all; sleep 1
  ssh $GS "env DEVOURER_VID=0x0bda DEVOURER_PID=0xa81a DEVOURER_CHANNEL=$CH \
    DEVOURER_STREAM_OUT=1 DEVOURER_EVENT_FLUSH=0 DEVOURER_LOG_LEVEL=warn \
    /tmp/rxdemo > $OUT/rx_$name.jsonl 2> $OUT/rx_$name.err" &
  sleep 7 # rxdemo bring-up
  ssh $DRONE "env DEVOURER_VID=0x0bda DEVOURER_PID=0xa81a DEVOURER_CHANNEL=$CH \
    DEVOURER_TX_RATE=$RATE DEVOURER_TX_QOS_DATA=1 \
    DEVOURER_TX_PAYLOAD_BYTES=$PAYLOAD DEVOURER_TX_GAP_US=0 \
    DEVOURER_LOG_LEVEL=warn $* \
    /tmp/txdemo > /tmp/tx_$name.jsonl 2> /tmp/tx_$name.err & \
    TXPID=\$!; sleep $SECS; kill -INT \$TXPID 2>/dev/null; sleep 1; \
    kill -9 \$TXPID 2>/dev/null; true"
  sleep 1
  ssh $GS 'killall -INT rxdemo 2>/dev/null; true'
  sleep 1
  kill_all
}

# singles baseline: QoS-Data no-ack, mgmt queue (the default QSEL)
run_cell control       DEVOURER_TX_QOS_NOACK=1
# data-queue routing alone, no AGG_EN (measured SLOWER than mgmt — BE CW)
run_cell qsel0         DEVOURER_TX_QOS_NOACK=1 DEVOURER_TX_QSEL=0
# AGG_EN + data queue + retry0, single URBs
run_cell ampdu_rty0    DEVOURER_TX_QOS_NOACK=1 DEVOURER_TX_QSEL=0 \
                       DEVOURER_TX_AMPDU=$AGG/$DENSITY/0
# + one-URB co-queue delivery
run_cell ampdu_rty0_urb DEVOURER_TX_QOS_NOACK=1 DEVOURER_TX_QSEL=0 \
                       DEVOURER_TX_AMPDU=$AGG/$DENSITY/0 \
                       DEVOURER_TX_BATCH=$AGG DEVOURER_TX_USB_AGG=$AGG
# the PRODUCT path: SetAmpduMode (0x455=0x20 pacing + noack + density 7)
run_cell mode          DEVOURER_TX_AMPDU_MODE=0/$AGG \
                       DEVOURER_TX_BATCH=$AGG DEVOURER_TX_USB_AGG=$AGG
# + deep multi-URB feed (doc: threads ~4 saturates on HalMAC)
run_cell mode_thr4     DEVOURER_TX_AMPDU_MODE=0/$AGG \
                       DEVOURER_TX_BATCH=$AGG DEVOURER_TX_USB_AGG=$AGG \
                       DEVOURER_TX_THREADS=4
# mgmt-queue AGG_EN check — LAST (wedged the 8822BU; on the E it is inert)
run_cell ampdu_mgmt    DEVOURER_TX_QOS_NOACK=1 DEVOURER_TX_AMPDU=$AGG/$DENSITY \
                       DEVOURER_TX_BATCH=$AGG DEVOURER_TX_USB_AGG=$AGG

scp -q $GS:$OUT/'*' "$LOGS/" 2>/dev/null
scp -q -O $DRONE:/tmp/'tx_*' "$LOGS/" 2>/dev/null
echo "logs in $LOGS — analyze with tools/bench/ampdu_e_analyze.py $LOGS"
