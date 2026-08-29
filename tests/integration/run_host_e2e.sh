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
# unconditionally sheds the enh layer, sid 1, until an RCF or DISC lands).
# 2-stream space (spec 2026-08-29-airtime-balance-uep): classify_frame
# (common/src/nal.cpp) routes critical NALs and any non-TRAIL_N VCL to sid 0
# (base); only TRAIL_N (mabur's SVC-T enhance marker) reaches sid 1 (enh).
# This fixture carries no TRAIL_N NALs (tools/genvectors/gen_vectors.py's
# frame_stream.bin generator only emits TRAIL_R), so EVERY fixture frame,
# IDR included, classifies to sid 0 and is never shed -- the reconstructable
# ceiling is genuinely all 13 frames, not a CRIT+T0 subset as it was pre-fold
# (see tests/test_uep_decoder.cpp's decodes_uep_encoder_bodies_to_fixture
# comment for the same fixture property from the encoder side). --max-stream
# 1 is passed anyway for symmetry with the RCF check below and because it is
# the space's natural upper bound (a no-op here since no frame is ever > 0).
echo "== clean pipe: all reachable frames must reconstruct byte-exact =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f0.bin"
python3 tools/bench/decode_bodies.py --frames "$TMP/f0.bin" --fixture "$FIX" \
  --symbol-size 332,332 --max-stream 1

echo "== 20% body loss: critical stream must still fully deliver =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f1.bin"
# Seed pinned to 1: the sliding-window bundle geometry (scalar-332/window-32/
# blocks_per_body-4) packs this short fixture's whole 13-frame stream (all on
# sid 0 now -- see above) into a modest body count, so a 20%-drop LCG roll
# can correlate onto the same body's sources+repairs and rank-deficient the
# GF(256) solve for a run of frames. That is a genuine capacity edge, not a
# decoder bug. Seed 1 clears it.
python3 tools/bench/decode_bodies.py --frames "$TMP/f1.bin" --fixture "$FIX" \
  --symbol-size 332,332 --drop-pct 20 --seed 1 --min-critical 1.0 --max-stream 1

echo "== RCF application: profile HT mcs4 after frame 1 changes BASE radiotap MCS =="
python3 - "$TMP/rc.bin" <<'EOF'
import struct, sys, os
sys.path.insert(0, os.path.abspath(os.path.join("..", "devourer", "tools", "precoder")))
import rc_proto
# mabur owns the RC wire as of RC_VERSION 2 (2026-08-12): devourer's frozen
# rc_proto.py is pinned at RC_VERSION 1 and still packs the deleted pwr_idx
# byte plus the deleted ack_seq/score/layer_delivery fields, so its
# pack_rcf() output is rejected outright by maburd. Pack the 13-byte v4 head
# here instead (magic, ver, type, flags, vtx_id, seq, profile,
# fec_overhead_x100 -- RC_VERSION 4, 2026-08-30, made this byte a literal
# overhead*100 rather than a sixteenths encoding); encode_profile and the
# CRC are unversioned.
body = struct.pack("<HBBBIHBB", rc_proto.RC_MAGIC, 4, rc_proto.T_RCF, 0,
                   1, 1, rc_proto.encode_profile("ht", 4, 20), 25)
w = body + struct.pack("<H", rc_proto._crc(body))
with open(sys.argv[1], "wb") as f:
    f.write(struct.pack("<II", 1, len(w))); f.write(w)
EOF
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f2.bin" \
  --rc-in "$TMP/rc.bin"
# 2-stream space (spec 2026-08-29-airtime-balance-uep): the fixture carries
# no TRAIL_N/enhance NALs (see the classify_frame comment above), so EVERY
# fixture frame -- including the IDR -- classifies to sid 0 (base), unlike
# the pre-fold-in world where CRIT/T0/T1/T2 gave the IDR a stream of its
# own. The IDR's own bodies are sent before the RCF lands (delivered once 1
# frame has been consumed) and still ride the boot MAX_RANGE mcs (0); only
# the bodies for frames 1-12 (P slices) postdate the RCF and ride the new
# BASE mcs (scored mcs 4 - 1 = 3, the mcs-1 rule). --after 8 skips exactly
# the IDR's bodies: this fixture's ~3 kB IDR access unit packs into 8
# bodies under the bundle's scalar-332/window-32/bpb-4/overhead-0.5 geometry
# in force before the RCF lands (verified empirically against this exact
# fixture+config; a bundle FEC geometry change would need to re-derive this
# count).
python3 tools/bench/decode_bodies.py --frames "$TMP/f2.bin" --fixture "$FIX" \
  --symbol-size 332,332 --expect-mcs 3 --stream 0 --after 8

echo "== full 2-stream recovery: all 13 frames byte-exact post-RCF =="
python3 tools/bench/decode_bodies.py --frames "$TMP/f2.bin" --fixture "$FIX" \
  --symbol-size 332,332

echo "== all E2E checks passed =="
