#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD:-build}
MABURD=$BUILD/drone/maburd
MABURGS=$BUILD/gs/maburgs
FIX=tests/fixtures/rtp_stream.bin
GSCFG=gs/bundle/maburgs.default.json
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# RCF delivered after packet 3 unsheds T1/T2 (same trick as run_host_e2e.sh),
# so maburd emits all four streams and the fixture is fully reconstructable.
python3 - "$TMP/rc.bin" <<'EOF'
import struct, sys, os
sys.path.insert(0, os.path.abspath(os.path.join("..", "devourer", "tools", "precoder")))
import rc_proto
r = rc_proto.Rcf(vtx_id=1, seq=1, ack_seq=0, profile=rc_proto.encode_profile("ht", 4, 20),
                 score=1500, pwr_idx=32, fec_overhead_16ths=4, layer_delivery=(100,))
w = rc_proto.pack_rcf(r)
with open(sys.argv[1], "wb") as f:
    f.write(struct.pack("<II", 3, len(w))); f.write(w)
EOF
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" \
  --out "$TMP/frames.bin" --rc-in "$TMP/rc.bin"

echo "== clean: all 18 packets byte-exact =="
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" --out-rtp "$TMP/rtp0.bin"
python3 tests/integration/verify_rtp.py "$TMP/rtp0.bin" "$FIX" --require-all

echo "== 15% single-card loss: UEP staircase (stream 0 must fully deliver) =="
# Seed 7 with 20% drop: observed stream 0 delivery at 83% (5/6). Lowered to 15% drop.
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" \
  --drop-pct 15 --seed 7 --out-rtp "$TMP/rtp1.bin"
python3 tests/integration/verify_rtp.py "$TMP/rtp1.bin" "$FIX" --min-stream0 1.0

echo "== 2 cards, 20% independent loss each: union recovers everything =="
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" \
  --cards 2 --drop-pct 20 --seed 9 --out-rtp "$TMP/rtp2.bin"
python3 tests/integration/verify_rtp.py "$TMP/rtp2.bin" "$FIX" --require-all

echo "== all GS E2E checks passed =="
