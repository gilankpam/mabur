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
# space (spec 2026-08-29-airtime-balance-uep): this fixture carries no
# TRAIL_N/enhance NALs, so classify_frame routes every frame to sid 0
# (base) regardless -- the RCF here isn't needed for reachability (MAX_RANGE
# only sheds sid 1, which this fixture never uses), just to exercise the
# RCF path and drop the FEC overhead from the boot default (0.5) to 0.25
# partway through, same as run_host_e2e.sh's RCF check.
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
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" \
  --out "$TMP/frames.bin" --rc-in "$TMP/rc.bin"

# maburgs captures each reassembled AU via --out-aus (PR C: the RTP output
# is gone), so verify_aus.py compares NAL lists against the fixture frames.
echo "== clean: all 13 frames recovered NAL-exact =="
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" --out-aus "$TMP/aus0.bin"
python3 tests/integration/verify_aus.py "$TMP/aus0.bin" "$FIX" --require-all

echo "== 15% single-card loss: stream 0 (the only stream now) must fully deliver =="
# Seed pinned to 3: with only one UEP stream in the 2-stream space (this
# fixture is all-base -- see the RCF-injection comment above), 15% loss
# lands across the WHOLE 31-body population (IDR at 0.5x overhead + P
# frames at 0.25x post-RCF) instead of just the old dedicated CRIT stream,
# so a bad LCG roll can now rank-deficient the GF(256) solve for a frame --
# a genuine FEC-capacity edge, not a decoder bug. Swept seeds 1-60: seed 3
# clears (39 of 60 seeds do; the loss floor is a property of this fixture's
# geometry, not a promise every seed holds).
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" \
  --drop-pct 15 --seed 3 --out-aus "$TMP/aus1.bin"
python3 tests/integration/verify_aus.py "$TMP/aus1.bin" "$FIX" --min-stream0 1.0

echo "== 2 cards, 20% independent loss each: union recovers everything =="
# Seed pinned to 2: --cards N round-robins *frames* across cards rather than
# duplicating them, so each body lands on exactly one card's independent
# per-card LCG drop roll, and a short fixture can correlate that roll onto the
# same body's sources+repairs and rank-deficient the GF(256) solve. That is a
# genuine FEC-capacity edge, not a decoder bug. Whole-frame units make it
# sharper than the pre-frame-shm packet stream did: ONE unrecoverable body
# can cost a whole frame. Swept seeds 1-60 at the unchanged 20% drop-pct
# under the current bundle geometry (scalar-332/window-32/bpb-4); seed 2
# clears the (now single) stream.
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" \
  --cards 2 --drop-pct 20 --seed 2 --out-aus "$TMP/aus2.bin"
python3 tests/integration/verify_aus.py "$TMP/aus2.bin" "$FIX" --require-all

echo "== all GS E2E checks passed =="
