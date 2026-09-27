#!/usr/bin/env bash
# Same WebGs code, native vs WASM: identical AU records and identical
# control trace (gs mode, synthetic DISC_ACK) over clean and lossy replays.
# Needs the host build (BUILD: drone/maburd, web/webgs) and the WASM build
# (web/build-wasm/webgs_node.js, see web/CMakeLists.txt).
# Run: nix-shell -p nodejs python3 --run "BUILD=$PWD/build web/tests/test_wasm_parity.sh"
set -euo pipefail
: "${BUILD:?}"
W=$(cd "$(dirname "$0")/.." && pwd)
cd "$W/.."
NODE_BIN=$W/build-wasm/webgs_node.js
[ -f "$NODE_BIN" ] || { echo "FAIL: $NODE_BIN missing -- build the WASM tree first" >&2; exit 1; }
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
parity() {  # <dir> <drop> <seed> <label>
  local D=$1
  rm -f "$D"/{n,w}.{aus,trace}
  "$BUILD/web/webgs" replay "$D/frames.bin" "$D/n.aus" -c "$D/gs.toml" --mode gs \
     --fake-ack --drop-pct "$2" --seed "$3" --control-trace "$D/n.trace" 2>/dev/null
  node "$NODE_BIN" replay "$D/frames.bin" "$D/w.aus" -c "$D/gs.toml" --mode gs \
     --fake-ack --drop-pct "$2" --seed "$3" --control-trace "$D/w.trace" 2>/dev/null
  [ -s "$D/n.aus" ] || { echo "FAIL: native wrote no AUs ($4 drop=$2 seed=$3)"; exit 1; }
  [ -s "$D/n.trace" ] || { echo "FAIL: native wrote no control trace ($4 drop=$2 seed=$3)"; exit 1; }
  cmp "$D/n.aus" "$D/w.aus" || { echo "FAIL: AU mismatch ($4 drop=$2 seed=$3)"; exit 1; }
  diff "$D/n.trace" "$D/w.trace" > /dev/null || {
    echo "FAIL: control trace mismatch ($4 drop=$2 seed=$3)"
    diff "$D/n.trace" "$D/w.trace" | head; exit 1; }
  echo "ok $4 drop=$2 seed=$3 ($(stat -c %s "$D/n.aus") AU bytes," \
       "$(wc -l < "$D/n.trace") trace lines)"
}

# 1. The standing fixture, same generation as tests/integration/
#    run_web_au_parity.sh. Both sides load the same derived config via -c
#    (NODERAWFS: host paths work).
mkdir "$TMP/fix"
bash tests/integration/web_frames_gen.sh "$TMP/fix"
parity "$TMP/fix" 0 1 fixture
parity "$TMP/fix" 10 7 fixture

# 2. A long synthetic stream (10 s at 60 fps, IDR every 2 s, 2-6 kB P frames
#    alternating base/enh -- gen_vectors.py's framing), so the lossy runs
#    repair at volume and the control loop ticks ~200 times with non-zero
#    loss inputs: the float path the short fixture never reaches.
mkdir "$TMP/long"
python3 - "$TMP/long/fixture.bin" <<'PY'
import struct, sys
def pat(n, seed): return bytes(((i * 31 + seed * 17 + 7) & 0xFF) for i in range(n))
def hdr(t, tid): return bytes([(t << 1) & 0xFF, (tid + 1) & 0x07])
def annexb(*nals): return b"".join(b"\x00\x00\x00\x01" + n for n in nals)
with open(sys.argv[1], "wb") as f:
    for i in range(600):
        if i % 120 == 0:
            flags, data = 0x01, annexb(hdr(32, 0) + pat(20, 1), hdr(33, 0) + pat(40, 2),
                                       hdr(34, 0) + pat(12, 3), hdr(19, 0) + pat(12000, i))
        else:
            enh = i % 2 == 1
            flags = 0x04 if enh else 0
            data = annexb(hdr(0 if enh else 1, i % 3) + pat(2000 + (i * 397) % 4000, i))
        meta = struct.pack("<IBBH", (i * 16667) & 0xFFFFFFFF, 0x01, flags, 0)
        f.write(struct.pack("<I", len(meta) + len(data))); f.write(meta + data)
PY
bash tests/integration/web_frames_gen.sh "$TMP/long" "$TMP/long/fixture.bin"
parity "$TMP/long" 0 1 long
parity "$TMP/long" 10 7 long
parity "$TMP/long" 25 9 long
echo "== wasm_parity passed =="
