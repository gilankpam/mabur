#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD:-build}
MABURD=$BUILD/drone/maburd
MABURGS=$BUILD/gs/maburgs
MABURPLAY=$BUILD/gs/player/maburplay
FIX=tests/fixtures/frame_stream.bin
# Golden for PART C below: sha256 of the ARGB dump produced by the pinned
# synthetic-font + canned-snapshot render. Regenerate ONLY when a deliberate
# change to the OSD sizing/blitting rules makes the old pixels wrong -- run
# the --osd-render command below by hand and paste the printed hash.
OSD_SHA_EXPECTED=e202e5d127467752984bda2c135d166661f8c0eb2c8481a77d54b39ed8f76a83
# Golden for PART D below. Same rule as OSD_SHA_EXPECTED: to re-bless after
# an intentional visual change, run the --gs-render command below by hand
# and paste the hash. Pinned against the SYNTHETIC font on purpose -- hashing
# the 13 MB shipped .gfont would make a font bump look like a rendering
# regression. A re-bless needs a PIXEL DIFF against the old dump, not just a
# hash swap: the geometry floor below cannot see a single-field blanking, a
# pre/post value transposition, or a pure recolour.
#
# Re-blessed 2026-08-04 (was c311690a...): the FPS baseline was derived from
# the WRONG atlas's cell height, which left the hero FPS box overlapping the
# JIT/MBPS box -- by exactly 0 px with the synthetic font, and by 1..3 px
# with the real one (tests/test_gs_asset.cpp). Diff verified: the FPS value
# and label translate up exactly 8 px and NOTHING else on the 1920x1080
# surface changes (0 differing px outside the FPS cell band, 0 lit px left in
# the vacated rows, lit-pixel total identical at 74690).
GS_SHA_EXPECTED=6479dbb19f72279babbf4ea54afc1efc6bcfb30063e1b08fa2c13928bcddf4b3
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

# --- PART C: OSD render gate (font -> layout -> raster, no DRM) --------
# Deterministic end-to-end check of the OSD pixel path: a synthetic atlas
# (so no multi-MB font fixture is committed) plus a canned MSP snapshot
# render to a fixed ARGB dump, pinned by hash. Guards the sizing rules and
# the blitter against silent regressions. The DRM half of the OSD cannot be
# exercised off-hardware, so everything below it is gated here instead.
echo "== OSD render gate =="
python3 tools/msp/gen_font.py --synthetic "$TMP/syn.mfont" --glyph-size 4x6
python3 - "$TMP/snap.bin" <<'EOF'
import struct, sys

def msp(cmd, payload):
    out = bytearray(b"$M<")
    out.append(len(payload)); out.append(cmd)
    ck = len(payload) ^ cmd
    for b in payload:
        out.append(b); ck ^= b
    out.append(ck & 0xFF)
    return bytes(out)

msg = b""
msg += msp(182, bytes([2]))                                   # CLEAR
msg += msp(182, bytes([5, 0, 1]))                             # SET_OPTIONS, HD 50x18
msg += msp(182, bytes([3, 1, 2, 0]) + b"MABUR OSD")           # DRAW_STRING
msg += msp(182, bytes([3, 9, 20, 0]) + b"0123456789")         # DRAW_STRING
msg += msp(182, bytes([4]))                                   # DRAW_SCREEN
open(sys.argv[1], "wb").write(msg)
EOF

"$MABURPLAY" --osd-render "$TMP/snap.bin" --out-osd "$TMP/osd.bin" \
  --font "$TMP/syn.mfont" --screen 320x180 --scale sharp

# Sanity floor before the hash: an all-transparent dump would match a stale
# golden just as happily as a correct render, so assert the geometry and
# that pixels actually landed. 18 non-blank cells (the space in "MABUR OSD"
# is blank) x 8 lit pixels per 4x6 synthetic glyph = 144.
python3 - "$TMP/osd.bin" <<'EOF'
import struct, sys
d = open(sys.argv[1], "rb").read()
magic, w, h, stride = struct.unpack("<4I", d[:16])
assert magic == 0x5244534F, hex(magic)
assert (w, h, stride) == (320, 180, 320), (w, h, stride)
assert len(d) == 16 + h * stride * 4, len(d)
px = struct.unpack("<%dI" % (h * stride), d[16:])
nz = sum(1 for p in px if p)
assert nz == 144, nz
print(f"OK osd dump: {w}x{h} stride={stride} lit={nz}")
EOF

OSD_SHA=$(sha256sum "$TMP/osd.bin" | cut -d' ' -f1)
echo "osd render sha256: $OSD_SHA"
if [ "$OSD_SHA" != "$OSD_SHA_EXPECTED" ]; then
  echo "OSD render hash changed (expected $OSD_SHA_EXPECTED)" >&2
  exit 1
fi

# --- PART D: GS link-status overlay render gate -----------------------
# The DRM half of the GS overlay cannot run off-hardware, so this is the
# host-side gate for everything below it: .gfont load, layout arithmetic,
# thresholds, formatting and the mask blitter. Same dump format as the MSP
# --osd-render gate above; the eight sizes are the 1080p design sizes, which
# resolve exactly at this screen height.
echo "== GS render gate =="
python3 tools/msp/gen_gsfont.py --synthetic "$TMP/syn.gfont" \
  --sizes 19,21,22,24,26,34,38,56

"$MABURPLAY" --gs-render tests/fixtures/gs_snapshot_nominal.json \
  --out-gs "$TMP/gs.bin" --gsfont "$TMP/syn.gfont" --screen 1920x1080 \
  --rec recording --rec-elapsed 767 --fps 60 --jit 3 --mbps 24.6

# Sanity floor before the hash: an all-transparent dump would match a stale
# golden just as happily as a correct render. Beyond "pixels landed", each
# block is checked to HUG the inset edge it is anchored to -- a mere ">0
# pixels somewhere in this corner" test passes just as happily for a block
# that has drifted a hundred px off its anchor, which is precisely the class
# of bug the layout has already had once.
python3 - "$TMP/gs.bin" <<'EOF'
import struct, sys
d = open(sys.argv[1], "rb").read()
magic, w, h, stride = struct.unpack("<4I", d[:16])
assert magic == 0x5244534F, hex(magic)
assert (w, h, stride) == (1920, 1080, 1920), (w, h, stride)
assert len(d) == 16 + h * stride * 4, len(d)
px = struct.unpack("<%dI" % (h * stride), d[16:])

def lit(x0, y0, x1, y1):
    return sum(1 for y in range(y0, y1) for x in range(x0, x1)
               if px[y * stride + x])

def bbox(x0, y0, x1, y1):
    on = [(x, y) for y in range(y0, y1) for x in range(x0, x1)
          if px[y * stride + x]]
    assert on, "region %s is empty" % ((x0, y0, x1, y1),)
    return (min(p[0] for p in on), min(p[1] for p in on),
            max(p[0] for p in on), max(p[1] for p in on))

assert lit(0, 0, w, h) > 0, "nothing drawn at all"
# Row 2: the central ~60% x 55% of frame is empty by design.
assert lit(400, 300, 1520, 780) == 0, "OSD drew in the centre of frame"
# Nothing outside the 5% title-safe inset (96 x 54 at 1080p).
assert lit(0, 0, 96, h) == 0, "drew left of the safe inset"
assert lit(w - 96, 0, w, h) == 0, "drew right of the safe inset"
assert lit(0, 0, w, 54) == 0, "drew above the safe inset"
assert lit(0, h - 54, w, h) == 0, "drew below the safe inset"
# The design puts no block in the top-left corner; anything there is a
# block that has escaped one of the other three.
assert lit(0, 0, 960, 540) == 0, "drew in the unused top-left quadrant"

# Per-block anchoring. The search windows are separated by the real gaps
# between blocks, so a bbox reaching a window edge means two blocks have run
# together (or one has moved into another's corner) -- checked before the
# anchor itself, since a merged bbox would make the anchor test meaningless.
TOL = 8
RIGHT, BOTTOM = w - 96 - 1, h - 54 - 1
for name, win, anchors in [
    ("top-right",     (960, 0, w, 540),    ("right", "top")),
    ("bottom-left",   (0, 540, 720, h),    ("left", "bottom")),
    ("bottom-centre", (720, 540, 1300, h), ("hcentre", "bottom")),
    ("bottom-right",  (1300, 540, w, h),   ("right", "bottom")),
]:
    x0, y0, x1, y1 = bbox(*win)
    print("  %-14s bbox x %d..%d y %d..%d" % (name, x0, x1, y0, y1))
    assert win[0] + 32 < x0 and x1 < win[2] - 32, "%s crowds its window" % name
    assert win[1] + 32 < y0 and y1 < win[3] - 32, "%s crowds its window" % name
    for a in anchors:
        if a == "left":
            assert abs(x0 - 96) <= TOL, "%s not flush left (x0=%d)" % (name, x0)
        elif a == "right":
            assert abs(x1 - RIGHT) <= TOL, "%s not flush right (x1=%d)" % (name, x1)
        elif a == "top":
            assert abs(y0 - 54) <= TOL, "%s not flush top (y0=%d)" % (name, y0)
        elif a == "bottom":
            assert abs(y1 - BOTTOM) <= TOL, "%s not flush bottom (y1=%d)" % (name, y1)
        elif a == "hcentre":
            # Loose deliberately: the loss row is centred on its WORST-CASE
            # field widths, so short values ("2.1%" in a "100.0%" box) sit
            # tens of px off centre legitimately. Still ~600 px from any
            # edge anchor, which is what this has to discriminate against.
            assert abs((x0 + x1) // 2 - w // 2) <= 48, "%s not centred" % name
print("OK gs render: %dx%d lit=%d" % (w, h, lit(0, 0, w, h)))
EOF

GS_SHA=$(sha256sum "$TMP/gs.bin" | cut -d' ' -f1)
echo "gs render sha256: $GS_SHA"
if [ "$GS_SHA" != "$GS_SHA_EXPECTED" ]; then
  echo "GS render hash changed (expected $GS_SHA_EXPECTED)" >&2
  exit 1
fi

echo "== player_e2e passed =="
