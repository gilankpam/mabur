#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD:-build}
MABURD=$BUILD/drone/maburd
MABURGS=$BUILD/gs/maburgs
FIX=tests/fixtures/frame_stream.bin
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# RCF delivered after frame 1 (same trick as run_host_e2e.sh / run_gs_e2e.sh):
# the fixture alternates TRAIL_R/TRAIL_N on its 12 P frames (2-stream space,
# spec 2026-08-29-airtime-balance-uep), so without an RCF the boot MAX_RANGE
# op point (drone/src/rc_agent.cpp:apply_max_range) would shed the 6 genuine
# sid-1 (enh) frames for good -- feeding one here is what makes the ring
# carry both streams, not just an incidental side effect of the RCF check.
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

echo "== fixture -> maburd bodies -> maburgs -> AU ring must be byte-exact =="
"$MABURD" -c bundle/mabur.default.toml --dry-run --in "$FIX" --out "$TMP/bodies.bin" \
  --rc-in "$TMP/rc.bin"

# The gs bundle's fec.symbol_size must match the drone bundle's encode-time
# symbol_size (332, adopted 2026-07-29 to dodge the mcs6+STBC 1392-1400 B PHY
# hole); without it the sliding-window decode geometry mismatches and yields 0
# AUs. The au_ring paths point at this run's temp dir.
#
# Range-anchored sed, not a bare one: `symbol_size` also appears under [msp].
sed -e "/^\[fec\]/,/^\[/ s|^symbol_size = .*|symbol_size = 332|" \
    -e "/^\[au_ring\]/a path = \"$TMP/au-ring\"\nsocket = \"$TMP/au-ring.sock\"" \
    gs/bundle/maburgs.default.toml > "$TMP/gs.toml"

# The sed above is layout-sensitive (bare `^symbol_size = ` anchor): if the
# bundle's [fec] section is ever reworded or re-indented, it silently stops
# matching and the test would still pass, just against whatever symbol_size
# the bundle shipped with. Assert the pin actually landed.
sed -n '/^\[fec\]/,/^\[/p' "$TMP/gs.toml" | grep -q '^symbol_size = 332$' || {
  echo "FAIL: fec.symbol_size sed did not match in $TMP/gs.toml -- bundle layout changed" >&2
  exit 1
}

"$MABURGS" -c "$TMP/gs.toml" --dry-run --in "$TMP/bodies.bin"

python3 tools/bench/ausniff.py --ring "$TMP/au-ring" --oneshot \
  --dump-annexb "$TMP/ring-aus.bin" --json > "$TMP/sniff.json"

# Reachable ceiling: the RCF above lands before frame index 1 (delivered
# once 1 frame has been consumed), so only the IDR is ever processed under
# the BOOT MAX_RANGE op point (drone/src/rc_agent.cpp:apply_max_range) --
# and the IDR always classifies to sid 0 regardless. Every frame from index
# 1 on (both the base and the genuine enh/TRAIL_N frames the fixture now
# alternates in, 2-stream space, spec 2026-08-29-airtime-balance-uep) is
# processed post-RCF, i.e. unshed. The ceiling is therefore all 13 fixture
# frames, on BOTH streams -- this is the standing gate that actually
# exercises sid 1 end-to-end through the AU ring.
python3 - "$FIX" "$TMP/ring-aus.bin" "$TMP/sniff.json" <<'EOF'
import json, sys, os
sys.path.insert(0, "tools/bench")
import decode_bodies as db
fixture = db.read_fixture(sys.argv[1])
want = b"".join(f["annexb"] for f in fixture)
got = open(sys.argv[2], "rb").read()
sniff = json.load(open(sys.argv[3]))
assert sniff["resyncs"] == 0, sniff
assert not sniff["incomplete"], f"incomplete AUs: {sniff}"
assert sniff["dropped_oversize"] == 0, sniff
assert got == want, (f"ring AU bytes != reachable fixture frames "
                     f"({len(got)} vs {len(want)} bytes)")
print(f"OK byte-exact: {sniff['aus']} AUs, {len(got)} bytes")
EOF

echo "== gs_au_e2e passed =="
