#!/usr/bin/env bash
# Shared by run_web_au_parity.sh and web/tests/test_wasm_parity.sh:
# writes $1/frames.bin (fixture -> maburd --dry-run, with an RCF) and
# $1/gs.toml (the bundle's maburgs config with au_ring off). $2 optionally
# names another frame-shm fixture (default tests/fixtures/frame_stream.bin).
# Needs BUILD (host build dir, for drone/maburd); run from the repo root.
set -euo pipefail
: "${BUILD:?}"
TMP=$1
MABURD=$BUILD/drone/maburd
FIX=${2:-tests/fixtures/frame_stream.bin}

# frames.bin generation: the same fixture -> maburd --dry-run path as
# run_gs_au_e2e.sh, including its RCF (read the rationale there).
# RCF delivered after frame 1 (same trick as run_host_e2e.sh / run_gs_e2e.sh):
# the fixture alternates TRAIL_R/TRAIL_N on its 12 P frames (2-stream space,
# spec 2026-08-29-airtime-balance-uep), so without an RCF the boot MAX_RANGE
# op point (drone/src/rc_agent.cpp:apply_max_range) would shed the 6 genuine
# sid-1 (enh) frames for good -- feeding one here is what makes the ring
# carry both streams, not just an incidental side effect of the RCF check.
python3 - "$TMP/rc.bin" <<'EOF'
import sys, os, struct
# Packed by tests/integration/mabur_rc.py: RC_VERSION read from rc_proto.h, tag under the dry-run replay session (1,1).
sys.path.insert(0, os.path.join("tests", "integration"))
from mabur_rc import pack_rcf, encode_profile
w = pack_rcf(1, encode_profile(4, 20), 25, 25)
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

