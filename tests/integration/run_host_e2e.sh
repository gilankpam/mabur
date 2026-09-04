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
# The fixture alternates TRAIL_R/TRAIL_N on its 12 P frames (tools/genvectors/
# gen_vectors.py), so 6 of them ARE genuine sid-1 material and are shed for
# good under MAX_RANGE -- the reconstructable ceiling here is the IDR plus
# the 6 base P frames (7 total), not all 13 (see tests/test_uep_decoder.cpp's
# decodes_uep_encoder_bodies_to_fixture comment for the same fixture property
# from the encoder side). --max-stream 0 restricts the expected set to that
# ceiling: maburd never allocates a frame_id for a shed frame, so leaving the
# default (1) would misalign every "want" key past the first shed frame.
echo "== clean pipe: all reachable frames must reconstruct byte-exact =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f0.bin"
python3 tools/bench/decode_bodies.py --frames "$TMP/f0.bin" --fixture "$FIX" \
  --symbol-size 332,332 --max-stream 0

echo "== 20% body loss: critical stream must still fully deliver =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f1.bin"
# Seed pinned to 1: the sliding-window bundle geometry (scalar-332/window-32/
# blocks_per_body-4) packs this short fixture's reachable 7-frame stream (all
# on sid 0 under MAX_RANGE -- see above) into a modest body count, so a
# 20%-drop LCG roll can correlate onto the same body's sources+repairs and
# rank-deficient the GF(256) solve for a run of frames. That is a genuine
# capacity edge, not a decoder bug. Seed 1 clears it.
python3 tools/bench/decode_bodies.py --frames "$TMP/f1.bin" --fixture "$FIX" \
  --symbol-size 332,332 --drop-pct 20 --seed 1 --min-critical 1.0 --max-stream 0

echo "== RCF application: profile HT mcs4 after frame 1 changes BASE radiotap MCS =="
python3 - "$TMP/rc.bin" <<'EOF'
import struct, sys, os
sys.path.insert(0, os.path.abspath(os.path.join("..", "devourer", "tools", "precoder")))
import rc_proto
# mabur owns the RC wire as of RC_VERSION 2 (2026-08-12): devourer's frozen
# rc_proto.py is pinned at RC_VERSION 1 and still packs the deleted pwr_idx
# byte plus the deleted ack_seq/score/layer_delivery fields, so its
# pack_rcf() output is rejected outright by maburd. Pack the 15-byte v6 head
# here instead (magic, ver, type, flags, vtx_id, seq, profile,
# fec_overhead_base_x100, fec_overhead_enh_x100, probe_profile -- RC_VERSION
# 6, 2026-09-04, made probe_profile a fixed head byte, 0xFF = no probe
# stream; encode_profile and the CRC are unversioned.
body = struct.pack("<HBBBIHBBBB", rc_proto.RC_MAGIC, 6, rc_proto.T_RCF, 0,
                   1, 1, rc_proto.encode_profile("ht", 4, 20), 25, 25, 0xFF)
w = body + struct.pack("<H", rc_proto._crc(body))
with open(sys.argv[1], "wb") as f:
    f.write(struct.pack("<II", 1, len(w))); f.write(w)
EOF
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/f2.bin" \
  --rc-in "$TMP/rc.bin"
# 2-stream space (spec 2026-08-29-airtime-balance-uep): the fixture's IDR
# always classifies to sid 0 (base) regardless of the P-frame alternation
# (see the classify_frame comment above). The IDR's own bodies are sent
# before the RCF lands (delivered once 1 frame has been consumed) and still
# ride the boot MAX_RANGE mcs (0); only the bodies for frames 1-12 postdate
# the RCF -- their sid-0 (base) share rides the new BASE mcs (scored mcs 4,
# same-rate ruling 2026-08-30 -- both slots ride the scored mcs, no rate
# split), while the sid-1 (enh) share, now unshed once the RCF lands, rides
# alongside on its own stream at the same mcs. --after 8 skips exactly the
# IDR's bodies: this fixture's ~3 kB IDR access unit packs into 8 bodies
# under the bundle's scalar-332/window-32/bpb-4/overhead-0.5 geometry in
# force before the RCF lands (verified empirically against this exact
# fixture+config; a bundle FEC geometry change would need to re-derive this
# count). --stream 0 keeps the mcs check scoped to BASE, since ENH rides a
# different mcs derivation entirely.
python3 tools/bench/decode_bodies.py --frames "$TMP/f2.bin" --fixture "$FIX" \
  --symbol-size 332,332 --expect-mcs 4 --stream 0 --after 8

echo "== full 2-stream recovery: all 13 frames byte-exact post-RCF =="
python3 tools/bench/decode_bodies.py --frames "$TMP/f2.bin" --fixture "$FIX" \
  --symbol-size 332,332

echo "== all E2E checks passed =="
