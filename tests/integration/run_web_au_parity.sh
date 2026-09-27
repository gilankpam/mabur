#!/usr/bin/env bash
# WebGs AU output == maburgs --dry-run --out-aus on the same frame file,
# clean and lossy (spec 2026-09-27-web-gs section 6.3). webgs runs Spotter
# mode (always decodes, like the dry-run, which negotiates no session) with
# --fixed-gap: the dry-run has no GapTimeoutPolicy.
set -euo pipefail
cd "$(dirname "$0")/../.."
: "${BUILD:?}"
MABURD=$BUILD/drone/maburd
MABURGS=$BUILD/gs/maburgs
WEBGS=$BUILD/web/webgs
FIX=tests/fixtures/frame_stream.bin
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# frames.bin generation: the same fixture -> maburd --dry-run path as
# run_gs_au_e2e.sh, including its RCF (read the rationale there).
# RCF delivered after frame 1 (same trick as run_host_e2e.sh / run_gs_e2e.sh):
# the fixture alternates TRAIL_R/TRAIL_N on its 12 P frames (2-stream space,
# spec 2026-08-29-airtime-balance-uep), so without an RCF the boot MAX_RANGE
# op point (drone/src/rc_agent.cpp:apply_max_range) would shed the 6 genuine
# sid-1 (enh) frames for good -- feeding one here is what makes the ring
# carry both streams, not just an incidental side effect of the RCF check.
python3 - "$TMP/rc.bin" <<'EOF'
import re, struct, sys, os
sys.path.insert(0, os.path.abspath(os.path.join("..", "devourer", "tools", "precoder")))
import rc_proto
# mabur owns the RC wire as of RC_VERSION 2 (2026-08-12): devourer's frozen
# rc_proto.py is pinned at RC_VERSION 1 and still packs the deleted pwr_idx
# byte plus the deleted ack_seq/score/layer_delivery fields, so its
# pack_rcf() output is rejected outright by maburd. Pack the 18-byte head
# here instead (magic, ver, type, flags, vtx_id, seq, profile,
# fec_overhead_base_x100, fec_overhead_enh_x100, probe_profile, hop_ch,
# hop_epoch, rec -- RC_VERSION 6, 2026-09-04, made probe_profile a fixed head
# byte, 0xFF = no probe stream; every bump since (7, T_CAL_CMD/T_CAL_RESULT
# plus a wider Telem; 8, relative calibration indices; 9, 2026-09-14, RCF
# gains hop_ch/hop_epoch -- 0/0 = no hop order issued; 11, 2026-09-26, RCF
# gains rec -- 0 = unknown) moved only the version byte plus trailing bytes.
# encode_profile and the CRC are unversioned.
# So read that byte from the header rather than pinning it: as a literal it
# half-landed the RC_VERSION 8 bump (2026-09-13) -- this script kept packing
# 7, maburd dropped the RCF as a foreign peer's, the shed of sid 1 never
# lifted, and the lost enhance stream read as a decoder bug.
RC_VERSION = int(re.search(r"RC_VERSION\s*=\s*(\d+)",
                           open("common/include/mabur/rc_proto.h").read()).group(1))
body = struct.pack("<HBBBIHBBBBBBB", rc_proto.RC_MAGIC, RC_VERSION, rc_proto.T_RCF, 0,
                   1, 1, rc_proto.encode_profile("ht", 4, 20), 25, 25, 0xFF, 0, 0, 0)
w = body + struct.pack("<H", rc_proto._crc(body))
with open(sys.argv[1], "wb") as f:
    f.write(struct.pack("<II", 1, len(w))); f.write(w)
EOF

"$MABURD" -c bundle/mabur.default.toml --dry-run --in "$FIX" --out "$TMP/frames.bin" \
  --rc-in "$TMP/rc.bin" >/dev/null 2>&1

# Both sides load the SAME config. au_ring off: the dry-run would otherwise
# open the bundle's /dev/shm ring on the host (webgs has no ring at all).
sed -e "/^\[au_ring\]/,/^\[/ s|^enable *= .*|enable = false|" \
    gs/bundle/maburgs.default.toml > "$TMP/gs.toml"
sed -n '/^\[au_ring\]/,/^\[/p' "$TMP/gs.toml" | grep -q '^enable = false$' || {
  echo "FAIL: au_ring.enable sed did not match -- bundle layout changed" >&2; exit 1; }

for spec in "0 1" "5 7" "15 3"; do
  set -- $spec; drop=$1; seed=$2
  rm -f "$TMP/ref.aus" "$TMP/web.aus"
  "$MABURGS" -c "$TMP/gs.toml" --dry-run --in "$TMP/frames.bin" --drop-pct "$drop" \
      --seed "$seed" --out-aus "$TMP/ref.aus" 2>/dev/null
  "$WEBGS" replay "$TMP/frames.bin" "$TMP/web.aus" -c "$TMP/gs.toml" --mode spotter \
      --drop-pct "$drop" --seed "$seed" --fixed-gap
  [ -s "$TMP/ref.aus" ] || { echo "FAIL: maburgs wrote no AUs drop=$drop seed=$seed"; exit 1; }
  cmp "$TMP/ref.aus" "$TMP/web.aus" || { echo "FAIL: AU mismatch drop=$drop seed=$seed"; exit 1; }
  echo "ok drop=$drop seed=$seed ($(stat -c %s "$TMP/ref.aus") bytes)"
done
echo "== web_au_parity passed =="
