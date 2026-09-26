#!/usr/bin/env bash
# THROWAWAY SPIKE: the WASM build must produce byte-identical AU records to
# the native build (proves GF(256) scalar + FEC + reassembly identical in
# WASM), and decode fast enough: >= 6300 bodies/s at 10 % loss = 3x the
# live bench's ~2100 video bodies/s.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
NATIVE=$HERE/build-native/gsweb
NODEJS="node $HERE/build-wasm/gsweb_node.js"
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
"$HERE/build-native/mkbodies" "$TMP/big.bin" 1800 40 1 2>/dev/null
for spec in "0 1" "10 7" "25 9"; do
  set -- $spec
  "$NATIVE" replay "$TMP/big.bin" "$TMP/n.bin" --drop-pct "$1" --seed "$2" 2>/dev/null
  $NODEJS replay "$TMP/big.bin" "$TMP/w.bin" --drop-pct "$1" --seed "$2" 2>/dev/null
  cmp "$TMP/n.bin" "$TMP/w.bin" || { echo "FAIL wasm != native at drop $1%"; exit 1; }
  echo "OK wasm == native at drop $1% ($(stat -c %s "$TMP/w.bin") bytes)"
done
t0=$(date +%s.%N)
$NODEJS replay "$TMP/big.bin" "$TMP/w.bin" --drop-pct 10 --seed 7 2>"$TMP/stat.txt"
t1=$(date +%s.%N)
bodies=$(grep -oE 'bodies=[0-9]+' "$TMP/stat.txt" | cut -d= -f2)
rate=$(python3 -c "print(int($bodies / ($t1 - $t0)))")
echo "wasm replay: $bodies bodies in $(python3 -c "print(round($t1-$t0,2))") s = $rate bodies/s"
[ "$rate" -ge 6300 ] || { echo "FAIL throughput $rate < 6300 bodies/s"; exit 1; }
echo "== test_wasm_replay passed =="
