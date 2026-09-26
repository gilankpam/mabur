# THROWAWAY SPIKE: sourced by test_replay.sh with BIG=1. 1800 frames
# (~30 s at 60 fps, ~40 KB/frame) through the real UepEncoder, so the
# lossy runs exercise GF(256) repairs at volume.
MKBODIES=${MKBODIES:-$HERE/build-native/mkbodies}
"$MKBODIES" "$TMP/big.bin" 1800 40 1
parity "$TMP/big.bin" 0 1 "synthetic lossless"
parity "$TMP/big.bin" 10 7 "synthetic lossy 10%"
parity "$TMP/big.bin" 25 9 "synthetic lossy 25%"

# Review focus 1: joining mid-stream. Every complete AU emitted after a join
# at body 5000 must be byte-identical to the lossless run's AU with that pts.
"$GSWEB" replay "$TMP/big.bin" "$TMP/full.bin" 2>/dev/null
"$GSWEB" replay "$TMP/big.bin" "$TMP/join.bin" --skip 5000 2>"$TMP/join.txt"
python3 - "$TMP/full.bin" "$TMP/join.bin" <<'EOF'
import struct, sys
def recs(p):
    b = open(p, "rb").read(); i = 0; out = []
    while i < len(b):
        n, sid, fl, pts = struct.unpack_from("<IBBI", b, i); i += 10
        out.append((sid, fl, pts, b[i:i + n])); i += n
    return out
full = {(r[2], r[0]): r[3] for r in recs(sys.argv[1]) if r[1] & 0x04}
join = [r for r in recs(sys.argv[2]) if r[1] & 0x04]
assert len(join) > 100, f"too few complete AUs after join: {len(join)}"
bad = [r for r in join if full.get((r[2], r[0])) != r[3]]
assert not bad, f"{len(bad)} joined AUs differ from the full run"
print(f"OK mid-stream join: {len(join)} complete AUs, all identical to the full run")
EOF
