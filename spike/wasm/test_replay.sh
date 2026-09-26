#!/usr/bin/env bash
# THROWAWAY SPIKE: gsweb replay must emit byte-identical AU records to
# maburgs --dry-run --out-aus on the same bodies file (lossless + lossy).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
GSWEB=${GSWEB:-$HERE/build-native/gsweb}
MABURD=${MABURD:-$ROOT/build/drone/maburd}
MABURGS=${MABURGS:-$ROOT/build/gs/maburgs}
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
cd "$ROOT"

# Fixture -> maburd bodies, with the synthetic RCF run_gs_au_e2e.sh feeds so
# the enh (sid 1) frames are not shed.
python3 - "$TMP/rc.bin" <<'EOF'
import re, struct, sys, os
sys.path.insert(0, os.path.abspath(os.path.join("..", "devourer", "tools", "precoder")))
import rc_proto
RC_VERSION = int(re.search(r"RC_VERSION\s*=\s*(\d+)",
                           open("common/include/mabur/rc_proto.h").read()).group(1))
body = struct.pack("<HBBBIHBBBBBBB", rc_proto.RC_MAGIC, RC_VERSION, rc_proto.T_RCF, 0,
                   1, 1, rc_proto.encode_profile("ht", 4, 20), 25, 25, 0xFF, 0, 0, 0)
w = body + struct.pack("<H", rc_proto._crc(body))
with open(sys.argv[1], "wb") as f:
    f.write(struct.pack("<II", 1, len(w))); f.write(w)
EOF
"$MABURD" -c bundle/mabur.default.toml --dry-run --in tests/fixtures/frame_stream.bin \
  --out "$TMP/fix.bin" --rc-in "$TMP/rc.bin" 2>/dev/null
sed -e "/^\[fec\]/,/^\[/ s|^symbol_size = .*|symbol_size = 332|" \
    -e "/^\[au_ring\]/,/^\[/ s|^path *= .*|path = \"$TMP/au-ring\"|" \
    -e "/^\[au_ring\]/,/^\[/ s|^socket *= .*|socket = \"$TMP/au-ring.sock\"|" \
    gs/bundle/maburgs.default.toml > "$TMP/gs.toml"

parity() {  # $1 bodies file, $2 drop pct, $3 seed, $4 label
  "$MABURGS" -c "$TMP/gs.toml" --dry-run --in "$1" --cards 1 --drop-pct "$2" --seed "$3" \
    --out-aus "$TMP/ref.bin" 2>/dev/null
  "$GSWEB" replay "$1" "$TMP/out.bin" --drop-pct "$2" --seed "$3" 2>"$TMP/stat.txt"
  test -s "$TMP/ref.bin" || { echo "FAIL $4: maburgs wrote no AUs"; exit 1; }
  cmp "$TMP/ref.bin" "$TMP/out.bin" || { echo "FAIL $4: AU records differ"; exit 1; }
  echo "OK $4: $(stat -c %s "$TMP/out.bin") bytes identical; $(grep STAT "$TMP/stat.txt")"
}
parity "$TMP/fix.bin" 0 1 "fixture lossless"
parity "$TMP/fix.bin" 10 7 "fixture lossy 10%"

# Review focus 4: a wrong symbol size yields 0 AUs and says why.
"$GSWEB" replay "$TMP/fix.bin" "$TMP/bad.bin" --symbol-size 164 2>"$TMP/bad.txt"
grep -q 'aus=0 ' "$TMP/bad.txt" || { echo "FAIL bad-cfg: expected aus=0"; cat "$TMP/bad.txt"; exit 1; }
grep -qE 'bad_cfg=[1-9]' "$TMP/bad.txt" || { echo "FAIL bad-cfg: expected bad_cfg>0"; cat "$TMP/bad.txt"; exit 1; }
echo "OK bad symbol size: $(grep STAT "$TMP/bad.txt")"

if [ -n "${BIG:-}" ]; then . "$HERE/test_replay_big.sh"; fi
echo "== test_replay passed =="
