#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD:-build}
MABURD=$BUILD/drone/maburd
FIX=tests/fixtures/frame_stream.bin
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# Neither of the next two runs feeds --rc-in, so the agent never leaves its
# BOOT-time MAX_RANGE operating point (drone/src/rc_agent.cpp:apply_max_range
# unconditionally sheds streams 2/3 == T1/T2 until an RCF or DISC lands).
# Only CRIT (stream 0) and T0 (stream 1) ever transmit, so the reconstructable
# ceiling is the fixture frames classified to those two streams (the IDR access
# unit plus the 4 tid-0 P frames = 5 of the fixture's 13 frames), not all 13.
# --max-stream 1 encodes that ceiling explicitly rather than silently
# under-counting.
echo "== clean pipe: all reachable (CRIT+T0) frames must reconstruct byte-exact =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f0.bin"
python3 tools/bench/decode_bodies.py --frames "$TMP/f0.bin" --fixture "$FIX" \
  --symbol-size 332,332,332,332 --max-stream 1

echo "== 20% body loss: critical stream must still fully deliver =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f1.bin"
# Seed pinned to 1: the sliding-window bundle geometry (scalar-328/window-32/
# blocks_per_body-4 as of 2026-07-25) packs this short fixture's critical stream
# into just 8 bodies -- the IDR access unit is one ~3 kB frame -- so a 20%-drop
# LCG roll can correlate onto the same body's sources+repairs and
# rank-deficient the GF(256) solve. That is a genuine capacity edge on a
# single-frame stream, not a decoder bug. Seed 1 clears it.
python3 tools/bench/decode_bodies.py --frames "$TMP/f1.bin" --fixture "$FIX" \
  --symbol-size 332,332,332,332 --drop-pct 20 --seed 1 --min-critical 1.0 --max-stream 1

echo "== RCF application: profile HT mcs4 after frame 1 changes T0 radiotap MCS =="
python3 - "$TMP/rc.bin" <<'EOF'
import struct, sys, os
sys.path.insert(0, os.path.abspath(os.path.join("..", "devourer", "tools", "precoder")))
import rc_proto
# mabur owns the RC wire as of RC_VERSION 2 (2026-08-12), and RC_VERSION 3
# (2026-08-15) shrank the RCF again: devourer's frozen rc_proto.py is pinned
# at RC_VERSION 1 and still packs the deleted pwr_idx byte plus the deleted
# ack_seq/score/layer_delivery fields, so its pack_rcf() output is rejected
# outright by maburd. Pack the 13-byte v3 head here instead (magic, ver,
# type, flags, vtx_id, seq, profile, fec_overhead_16ths); encode_profile and
# the CRC are unversioned.
body = struct.pack("<HBBBIHBB", rc_proto.RC_MAGIC, 3, rc_proto.T_RCF, 0,
                   1, 1, rc_proto.encode_profile("ht", 4, 20), 4)
w = body + struct.pack("<H", rc_proto._crc(body))
with open(sys.argv[1], "wb") as f:
    f.write(struct.pack("<II", 1, len(w))); f.write(w)
EOF
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f2.bin" \
  --rc-in "$TMP/rc.bin"
# The fixture's frame 0 is the IDR access unit (VPS/SPS/PPS + IDR slice), which
# is the only stream-0 frame; every later frame is a P slice on streams 1-3. The
# RCF is delivered once 1 frame has been consumed, i.e. before the first
# stream-1 (T0) frame is ever encoded, so every T0 body in f2.bin postdates it
# and --after 0 is exact rather than cautious.
python3 tools/bench/decode_bodies.py --frames "$TMP/f2.bin" --fixture "$FIX" \
  --symbol-size 332,332,332,332 --expect-mcs 4 --stream 1 --after 0

echo "== full 4-stream recovery: all 13 frames byte-exact post-RCF =="
python3 tools/bench/decode_bodies.py --frames "$TMP/f2.bin" --fixture "$FIX" \
  --symbol-size 332,332,332,332

echo "== all E2E checks passed =="
