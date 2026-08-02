#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD:-build}
MABURD=$BUILD/drone/maburd
MABURGS=$BUILD/gs/maburgs
MABURPLAY=$BUILD/gs/player/maburplay
FIX=tests/fixtures/frame_stream.bin
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "== fixture -> maburd bodies -> maburgs -> AU ring (same chain as gs_au_e2e) =="
"$MABURD" -c bundle/mabur.default.json --dry-run --in "$FIX" --out "$TMP/bodies.bin"

# Same symbol_size 332 pin as run_gs_au_e2e.sh: now redundant with PR #11's
# encode-time default, but it documents the encode geometry contract this
# test relies on (the gs bundle default is stale vs the drone bundle).
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

echo "== maburplay --oneshot: drain ring, null backend, DVR on =="
python3 - "$TMP/play.json" <<'EOF'
import json, sys
cfg = {
    "ring_path": sys.argv[1].rsplit("/", 1)[0] + "/au-ring",
    "socket": sys.argv[1].rsplit("/", 1)[0] + "/none.sock",
    "backend": "null",
    "dvr": {"enabled": True, "dir": sys.argv[1].rsplit("/", 1)[0]},
}
json.dump(cfg, open(sys.argv[1], "w"))
EOF

"$MABURPLAY" -c "$TMP/play.json" --oneshot > "$TMP/stats.json"
cat "$TMP/stats.json"

python3 - "$TMP/stats.json" <<'EOF'
import json, sys
s = json.load(open(sys.argv[1]))
assert s["delivered"] == 5, s
assert s["dropped_enhance_incomplete"] == 0, s
assert s["resyncs"] == 0, s
assert s["dvr_samples"] == 5, s
assert s["dvr_fragments"] >= 1, s
print(f"OK stats: {s}")
EOF

# --- PART A: container-level gate on the fixture-chain DVR file -------
# tests/fixtures/frame_stream.bin's NAL payloads (tools/genvectors/
# gen_vectors.py's pat() filler) are correctly NAL-type-tagged (VPS/SPS/
# PPS/IDR/TRAIL) so framing/FEC/AU-grouping all round-trip byte-exact --
# but the payload bits themselves are deterministic filler, not semantically
# valid HEVC syntax (e.g. the IDR access unit's own slice references a PPS
# id that its own in-band PPS never declares). That's a bitstream-semantics
# problem, not a container one: ffprobe's avformat_find_stream_info parses
# NAL/VPS/SPS/PPS on *any* open of this file (exhaustively verified --
# -count_packets vs -count_frames, minimal probesize, even a bare open with
# no -show_entries at all: same errors every time), so a blanket
# empty-stderr assertion is unattainable here regardless of which ffprobe
# invocation is used. This gate therefore tolerates exactly that known,
# bitstream-level noise (grep -iv 'vps|sps|pps|nal|hevc') and fails on
# anything else -- container/box-level complaints (moov/moof/trun), I/O
# errors, demuxer failures -- so it stays a real container-integrity check
# without pretending the fixture is decodable. Also tolerates ffmpeg's own
# "Last message repeated N times" log-dedup notice, which refers back to
# whichever bitstream-noise line it's collapsing and carries none of the
# VPS/SPS/PPS/NAL/HEVC keywords itself.
echo "== ffprobe gate A (container-level, fixture-chain DVR file) =="
set +e
OUT_A=$(nix-shell -p ffmpeg --run \
  "ffprobe -v error -count_packets -select_streams v -show_entries stream=codec_name,nb_read_packets -of csv=p=0 $TMP/record_*.mp4" \
  2> "$TMP/ffprobeA.stderr")
RC_A=$?
set -e
echo "ffprobe A: $OUT_A (rc=$RC_A)"
if [ "$RC_A" -ne 0 ]; then
  echo "ffprobe A exited $RC_A" >&2
  cat "$TMP/ffprobeA.stderr" >&2
  exit 1
fi
if [ "$OUT_A" != "hevc,5" ]; then
  echo "unexpected ffprobe A output: $OUT_A (want hevc,5)" >&2
  exit 1
fi
FILTERED_A=$(grep -iv 'vps\|sps\|pps\|nal\|hevc\|last message repeated' "$TMP/ffprobeA.stderr" || true)
if [ -n "$FILTERED_A" ]; then
  echo "ffprobe A stderr has non-bitstream-noise lines:" >&2
  echo "$FILTERED_A" >&2
  exit 1
fi

# --- PART B: real-HEVC decode gate, via maburplay --mux-annexb --------
# Formalizes the positive control used to root-cause Part A's limits: a
# real (ffmpeg/libx265) HEVC stream fed through the exact same
# HevcParams/DvrMux calls the live sink uses (via maburplay's --mux-annexb
# test-support mode), then actually decoded frame-by-frame by ffprobe.
echo "== generate 9-frame real HEVC elementary stream =="
nix-shell -p ffmpeg --run \
  "ffmpeg -v error -f lavfi -i testsrc2=size=320x240:rate=60 -frames:v 9 -c:v libx265 -x265-params aud=1:keyint=5 -f hevc $TMP/real.265"

echo "== maburplay --mux-annexb: real stream -> DVR fMP4 =="
"$MABURPLAY" --mux-annexb "$TMP/real.265" "$TMP/real.mp4"

echo "== ffprobe gate B (real decode, real HEVC file) =="
OUT_B=$(nix-shell -p ffmpeg --run \
  "ffprobe -v error -count_frames -select_streams v -show_entries stream=codec_name,nb_read_frames -of csv=p=0 $TMP/real.mp4" \
  2> "$TMP/ffprobeB.stderr")
ERR_B=$(cat "$TMP/ffprobeB.stderr")
echo "ffprobe B: $OUT_B"
if [ -n "$ERR_B" ]; then
  echo "ffprobe B stderr not empty:" >&2
  echo "$ERR_B" >&2
  exit 1
fi
if [ "$OUT_B" != "hevc,9" ]; then
  echo "unexpected ffprobe B output: $OUT_B (want hevc,9)" >&2
  exit 1
fi

echo "== player_e2e passed =="
