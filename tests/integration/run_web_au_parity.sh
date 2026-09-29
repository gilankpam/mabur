#!/usr/bin/env bash
# WebGs AU output == maburgs --dry-run --out-aus on the same frame file,
# clean and lossy (spec 2026-09-27-web-gs section 6.3). webgs runs Spotter
# mode (always decodes, like the dry-run, which negotiates no session) with
# --fixed-gap: the dry-run has no GapTimeoutPolicy.
set -euo pipefail
cd "$(dirname "$0")/../.."
: "${BUILD:?}"
MABURGS=$BUILD/gs/maburgs
WEBGS=$BUILD/web/webgs
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# frames.bin + gs.toml (same config on both sides, au_ring off): shared with
# web/tests/test_wasm_parity.sh.
bash tests/integration/web_frames_gen.sh "$TMP"

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
