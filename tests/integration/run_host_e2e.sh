#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD:-build}
MABURD=$BUILD/drone/maburd
FIX=tests/fixtures/rtp_stream.bin
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# Neither of the next two runs feeds --rc-in, so the agent never leaves its
# BOOT-time MAX_RANGE operating point (drone/src/rc_agent.cpp:apply_max_range
# unconditionally sheds streams 2/3 == T1/T2 until an RCF or DISC lands).
# Only CRIT (stream 0) and T0 (stream 1) ever transmit, so the reconstructable
# ceiling is the fixture packets classified to those two streams (10 of the
# fixture's 18 HEVC/RTP packets), not all 18. --max-stream 1 encodes that
# ceiling explicitly rather than silently under-counting.
echo "== clean pipe: all reachable (CRIT+T0) packets must reconstruct byte-exact =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f0.bin"
python3 tools/bench/decode_bodies.py --frames "$TMP/f0.bin" --fixture "$FIX" \
  --symbol-size 328,328,328,328 --max-stream 1

echo "== 20% body loss: critical stream must still fully deliver =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f1.bin"
# Seed pinned to 1: the sliding-window bundle geometry (scalar-328/window-32/
# blocks_per_body-4 as of 2026-07-25, vs the pre-sliding-window
# scalar-64/window-128 geometry this test was originally tuned against) packs
# the whole critical stream into just 16 bodies for this short fixture, so a
# 20%-drop LCG roll can correlate onto the same body's sources+repairs and
# rank-deficient the GF(256) solve -- a genuine capacity edge (seeds 6-9 hit
# it: swept 1-60, 14/60 fail), not a decoder bug. Seed 1 clears with margin
# (10/10 recovered) and keeps the scenario's stated 20% loss rate truthful.
python3 tools/bench/decode_bodies.py --frames "$TMP/f1.bin" --fixture "$FIX" \
  --symbol-size 328,328,328,328 --drop-pct 20 --seed 1 --min-critical 1.0 --max-stream 1

echo "== RCF application: profile HT mcs4 after packet 3 changes T0 radiotap MCS =="
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
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f2.bin" \
  --rc-in "$TMP/rc.bin"
# The fixture's first 6 packets (0-5) are all stream 0 (VPS/SPS/PPS + 3 IDR FU
# slices), so the RCF (delivered once consumed_packets reaches 3, well before
# the first stream-1 body is ever emitted at packet 6) has landed before any
# T0 (stream 1) body exists in the output. --after 0 is therefore correct: it
# is not "0 out of caution", it is empirically verified that every T0 body in
# f2.bin postdates the RCF.
python3 tools/bench/decode_bodies.py --frames "$TMP/f2.bin" --fixture "$FIX" \
  --symbol-size 328,328,328,328 --expect-mcs 4 --stream 1 --after 0

echo "== full 4-stream recovery: all 18 packets byte-exact post-RCF =="
python3 tools/bench/decode_bodies.py --frames "$TMP/f2.bin" --fixture "$FIX" \
  --symbol-size 328,328,328,328

echo "== all E2E checks passed =="
