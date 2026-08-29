#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD:-build}
MABURD=$BUILD/drone/maburd
MABURGS=$BUILD/gs/maburgs
FIX=tests/fixtures/frame_stream.bin
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "== fixture -> maburd bodies -> maburgs -> AU ring must be byte-exact =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/bodies.bin"

# The gs bundle's fec.symbol_size default (328) is stale vs the drone bundle's
# encode-time symbol_size (332, adopted 2026-07-29 to dodge the mcs6+STBC
# 1392-1400 B PHY hole — see ladder-controller memory); without overriding it
# here the sliding-window decode geometry mismatches the encoder and yields 0
# AUs. Force it to 332 to match the drone bundle actually used above.
python3 - "$TMP/gs.json" <<'EOF'
import json, sys
c = json.load(open("gs/bundle/maburgs.default.json"))
tmp = sys.argv[1].rsplit("/", 1)[0]
c["fec"]["symbol_size"] = 332
c["au_ring"] = {"enable": True, "path": tmp + "/au-ring",
                "socket": tmp + "/au-ring.sock"}
json.dump(c, open(sys.argv[1], "w"))
EOF

"$MABURGS" -c "$TMP/gs.json" --dry-run --in "$TMP/bodies.bin"

python3 tools/bench/ausniff.py --ring "$TMP/au-ring" --oneshot \
  --dump-annexb "$TMP/ring-aus.bin" --json > "$TMP/sniff.json"

# Reachable ceiling: no --rc-in, so the drone agent stays at its BOOT
# MAX_RANGE op point and only sid 0 (base) transmits, sid 1 (enh) shed —
# same ceiling tests/integration/run_host_e2e.sh encodes with
# --max-stream 1. This fixture carries no TRAIL_N/enhance NALs (2-stream
# space, spec 2026-08-29-airtime-balance-uep), so the ceiling is all 13
# fixture frames regardless of the shed.
python3 - "$FIX" "$TMP/ring-aus.bin" "$TMP/sniff.json" <<'EOF'
import json, sys, os
sys.path.insert(0, "tools/bench")
import decode_bodies as db
fixture = db.read_fixture(sys.argv[1])
want = b"".join(f["annexb"] for f in fixture
                if db.classify_frame(f["annexb"])[0] <= 1)
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
