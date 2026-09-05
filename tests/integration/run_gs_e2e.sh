#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD:-build}
MABURD=$BUILD/drone/maburd
MABURGS=$BUILD/gs/maburgs
FIX=tests/fixtures/frame_stream.bin
GSCFG=gs/bundle/maburgs.default.json
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# RCF delivered after frame 1 (same trick as run_host_e2e.sh). 2-stream
# space (spec 2026-08-29-airtime-balance-uep): the fixture alternates
# TRAIL_R/TRAIL_N on its 12 P frames, so classify_frame routes 6 of them to
# sid 1 (enh) -- and the RCF IS needed for reachability here (MAX_RANGE
# sheds sid 1 until it lands), on top of exercising the RCF path and
# dropping the FEC overhead from the boot default (0.5) to 0.25 partway
# through, same as run_host_e2e.sh's RCF check.
python3 - "$TMP/rc.bin" <<'EOF'
import struct, sys, os
sys.path.insert(0, os.path.abspath(os.path.join("..", "devourer", "tools", "precoder")))
import rc_proto
# mabur owns the RC wire as of RC_VERSION 2 (2026-08-12): devourer's frozen
# rc_proto.py is pinned at RC_VERSION 1 and still packs the deleted pwr_idx
# byte plus the deleted ack_seq/score/layer_delivery fields, so its
# pack_rcf() output is rejected outright by maburd. Pack the 15-byte v6 head
# here instead (magic, ver, type, flags, vtx_id, seq, profile,
# fec_overhead_base_x100, fec_overhead_enh_x100, probe_profile --
# RC_VERSION 6, 2026-09-04, made probe_profile a fixed head byte, 0xFF = no
# probe stream; encode_profile and the CRC are unversioned.
body = struct.pack("<HBBBIHBBBB", rc_proto.RC_MAGIC, 6, rc_proto.T_RCF, 0,
                   1, 1, rc_proto.encode_profile("ht", 4, 20), 25, 25, 0xFF)
w = body + struct.pack("<H", rc_proto._crc(body))
with open(sys.argv[1], "wb") as f:
    f.write(struct.pack("<II", 1, len(w))); f.write(w)
EOF
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" \
  --out "$TMP/frames.bin" --rc-in "$TMP/rc.bin"

# maburgs captures each reassembled AU via --out-aus (PR C: the RTP output
# is gone), so verify_aus.py compares NAL lists against the fixture frames.
# --require-all iterates every sid present in the fixture (both 0 and 1
# here), so this is already a both-streams-present assertion; --min-stream1
# is added anyway for symmetry with --min-stream0 and to be explicit about
# what's being gated.
echo "== clean: all 13 frames recovered NAL-exact, both streams =="
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" --out-aus "$TMP/aus0.bin"
python3 tests/integration/verify_aus.py "$TMP/aus0.bin" "$FIX" --require-all \
  --min-stream0 1.0 --min-stream1 1.0

echo "== 15% single-card loss: both streams must fully deliver =="
# Seed pinned to 3: 15% loss lands across the WHOLE 30-body population (IDR
# at 0.5x overhead + P frames at 0.25x post-RCF, split across sid 0 base and
# sid 1 enh), so a bad LCG roll can rank-deficient the GF(256) solve for a
# frame on either stream -- a genuine FEC-capacity edge, not a decoder bug.
# Verified empirically against this exact fixture+seed: both streams clear
# (stream 0 7/7, stream 1 6/6); the loss floor is a property of this
# fixture's geometry, not a promise every seed holds.
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" \
  --drop-pct 15 --seed 3 --out-aus "$TMP/aus1.bin"
python3 tests/integration/verify_aus.py "$TMP/aus1.bin" "$FIX" \
  --min-stream0 1.0 --min-stream1 1.0

echo "== 2 cards, 20% independent loss each: union recovers everything, both streams =="
# Seed pinned to 2: --cards N round-robins *frames* across cards rather than
# duplicating them, so each body lands on exactly one card's independent
# per-card LCG drop roll, and a short fixture can correlate that roll onto the
# same body's sources+repairs and rank-deficient the GF(256) solve. That is a
# genuine FEC-capacity edge, not a decoder bug. Whole-frame units make it
# sharper than the pre-frame-shm packet stream did: ONE unrecoverable body
# can cost a whole frame. Swept seeds 1-60 at the unchanged 20% drop-pct
# under the current bundle geometry (scalar-332/window-32/bpb-4); seed 2
# clears both streams.
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" \
  --cards 2 --drop-pct 20 --seed 2 --out-aus "$TMP/aus2.bin"
python3 tests/integration/verify_aus.py "$TMP/aus2.bin" "$FIX" --require-all \
  --min-stream0 1.0 --min-stream1 1.0

echo "== all GS E2E checks passed =="
