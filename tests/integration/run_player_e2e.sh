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
# Golden for PART D, the GS link-status OSD (the compact bar). Same rule as
# OSD_SHA_EXPECTED, and stricter: re-bless only from a pixel diff, never
# from a hash swap alone -- the geometry floor below sees a bar that moved or lost a row, but
# not a transposed value pair or a blanked field.
#
# Re-blessed (was 27566469...): the recording indicator moved out
# of row 0 and up to the TOP-RIGHT corner, on its own. Row 0 goes back to
# exactly the pixels it had before the indicator existed (d4cc28b3's row 0),
# row 1 is untouched, and the only new ink is the indicator's own band at
# the top inset -- which is what the third band the gate now asserts is.
#
# Re-blessed (was d4cc28b3...): the bar gained the recording
# indicator, rendered identically to the essential overlay's (dot, REC,
# mm:ss clock), and this invocation now passes --rec recording so the pixel
# path covers it.
#
# Re-blessed (was f5c5b69c...): the bar went from one row to two.
# A single line capped the type at 22 px on a 1080p panel -- too small to
# read on the GS screen -- and splitting the eleven items across two rows
# (radio above, picture below) lets the same "largest baked size that fits"
# rule pick 38 px instead. Every pixel moves, so a diff against the old dump
# says nothing; what was checked instead is the geometry floor below (two
# bands, both centred, block hugging the bottom) plus the per-row strings
# pinned in tests/test_gs_compact.cpp.
#
# Re-blessed (was 658c9593...): compact bar mcs cell shows width
# (40 MHz rungs). GsSnapshot::bw and
# the "mcs:<mcs>/<bw>" cell format were added in commit 66e52ba; the fixture
# (tests/fixtures/gs_snapshot_nominal.json) carries no ctl.rung.bw or op.bw,
# so the cell now reads "mcs:5/--" instead of "mcs:5", widening row 0 and
# recentring every item on it. Verified this is the ONLY change: rebuilt
# maburplay against the pre-commit (5df752c) sources with the fixture and
# CLI flags unchanged reproduces the old 658c9593... hash exactly, and a
# pixel diff old->new shows every differing pixel confined to the row-0
# scanline band (y 944..996) -- 31,963 px differ (lit 135,694 -> 139,190,
# delta 3,496), ZERO of them in the REC corner band (y 42..94) or row 1 (y
# 1003..1055), which stay byte-identical.
# Re-blessed (was c7194602...): the REC corner item draws at half
# the bar's size (nearest baked; 19 px here). Pixel diff old vs new: 11768 px
# changed, all inside x 1639..1886 y 42..94 (the REC field), 0 outside; REC
# ink still ends flush at x 1886 and its top stays on the 40 px inset.
# Re-blessed (was 6e8c67a6...): item labels became icons ("ch:" -> the
# wifi icon and a space, ...) and "res:1280x720" became "720p", at an
# unchanged type size. Pixel diff old vs new (the old dump rebuilt from the
# previous sources reproduces 6e8c67a6... exactly): the same three ink
# bands at the same rows (y 42..70, 944..996, 1003..1055); ZERO differing px
# in the REC band; both bar rows narrower and still centred (row 0 x
# 264..1633 -> 407..1490, row 1 143..1688 -> 374..1457); lit 129,731 ->
# 90,091 as the label text left.
# Re-blessed (was a8df1c23...): the two rows share a centred column grid
# instead of each centring on its own width. Pixel diff old vs new (old
# dump rebuilt from the previous sources reproduces a8df1c23... exactly):
# same three ink bands at the same rows, ZERO differing px in the REC band,
# lit total identical at 90,091 (a pure horizontal translation of items),
# and both rows now start at x 308 (row 0 308..1413, row 1 308..1523; were
# 407..1490 and 374..1457).
# Re-blessed (was 46d4286e...): the fps cell reserves "999/999" for the
# drone-fps/panel-Hz mismatch reading (source/panel), widening the column
# it shares with airtime. Same three ink bands at the same rows (type size
# unchanged), lit total identical at 90,091, rows now 275..1446 and
# 275..1556 (were 308..1413 and 308..1523).
# Re-blessed (was 404627bd...): row 0 gained the uplink item ("--" here:
# the fixture carries no drone.rcf), trailing temp at item spacing outside
# the column grid. Type size unchanged (the same three ink bands at the same
# rows, y 42..70 / 944..996 / 1003..1055); row 0 now x 275..1578 (was
# 275..1446), row 1 unchanged at 275..1556; lit 90,091 -> 95,057.
BAR_SHA_EXPECTED=3e4de517167dc40ba449e2225cb8d77cfe53b0f4d871d0a356b8b6428d5d2638
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# RCF delivered after frame 1 (same trick as run_gs_au_e2e.sh): the fixture
# alternates TRAIL_R/TRAIL_N on its 12 P frames (2-stream space), so
# without an RCF the boot MAX_RANGE op point (drone/src/rc_agent.cpp:apply_max_range) would shed the 6 genuine
# sid-1 (enh) frames for good and this chain would never exercise ENH
# through the player/DVR path.
python3 - "$TMP/rc.bin" <<'EOF'
import sys, os, struct
# Packed by tests/integration/mabur_rc.py: RC_VERSION read from rc_proto.h, tag under the dry-run replay session (1,1).
sys.path.insert(0, os.path.join("tests", "integration"))
from mabur_rc import pack_rcf, encode_profile
w = pack_rcf(1, encode_profile(4, 20), 25, 25)
with open(sys.argv[1], "wb") as f:
    f.write(struct.pack("<II", 1, len(w))); f.write(w)
EOF

echo "== fixture -> maburd bodies -> maburgs -> AU ring (same chain as gs_au_e2e) =="
"$MABURD" -c bundle/mabur.default.toml --dry-run --in "$FIX" --out "$TMP/bodies.bin" \
  --rc-in "$TMP/rc.bin"

# Same symbol_size 332 pin as run_gs_au_e2e.sh: now redundant with PR #11's
# encode-time default, but it documents the encode geometry contract this
# test relies on (the gs bundle default is stale vs the drone bundle).
#
# Range-anchored sed, not a bare one: `symbol_size` also appears under [msp].
sed -e "/^\[fec\]/,/^\[/ s|^symbol_size = .*|symbol_size = 332|" \
    -e "/^\[au_ring\]/,/^\[/ s|^path *= .*|path = \"$TMP/au-ring\"|" \
    -e "/^\[au_ring\]/,/^\[/ s|^socket *= .*|socket = \"$TMP/au-ring.sock\"|" \
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

echo "== maburplay --oneshot: drain ring, null backend, DVR on =="
cat > "$TMP/play.toml" <<EOF
ring_path = "$TMP/au-ring"
socket = "$TMP/none.sock"
backend = "null"

[dvr]
autostart = true
dir = "$TMP"
EOF

"$MABURPLAY" -c "$TMP/play.toml" --oneshot > "$TMP/stats.json"
cat "$TMP/stats.json"

python3 - "$TMP/stats.json" <<'EOF'
import json, sys
s = json.load(open(sys.argv[1]))
assert s["delivered"] == 13, s
assert s["dropped_enhance_incomplete"] == 0, s
assert s["resyncs"] == 0, s
assert s["dvr_samples"] == 13, s
assert s["dvr_fragments"] >= 1, s
assert s["stream"] == "off:no_decoder", s   # null backend: no stream mode, whole AUs
assert s["streamed"] == 0, s
assert s["stream_aborted"] == 0, s
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
# The redirect belongs to ffprobe, INSIDE --run: hung off nix-shell it also
# captures nix-shell's own stderr, and on a cold store that is a few hundred
# lines of "these N paths will be fetched" / "copying path ..." which the
# noise filter below then reports as ffprobe diagnostics. Warm stores hide
# this; CI does not have one.
OUT_A=$(nix-shell -p ffmpeg --run \
  "ffprobe -v error -count_packets -select_streams v -show_entries stream=codec_name,nb_read_packets -of csv=p=0 $TMP/record-*.mp4 2> '$TMP/ffprobeA.stderr'")
RC_A=$?
set -e
echo "ffprobe A: $OUT_A (rc=$RC_A)"
if [ "$RC_A" -ne 0 ]; then
  echo "ffprobe A exited $RC_A" >&2
  cat "$TMP/ffprobeA.stderr" >&2
  exit 1
fi
if [ "$OUT_A" != "hevc,13" ]; then
  echo "unexpected ffprobe A output: $OUT_A (want hevc,13)" >&2
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
# Redirect inside --run, same reason as gate A -- and this gate is stricter
# still, failing on ANY stderr line at all.
OUT_B=$(nix-shell -p ffmpeg --run \
  "ffprobe -v error -count_frames -select_streams v -show_entries stream=codec_name,nb_read_frames -of csv=p=0 $TMP/real.mp4 2> '$TMP/ffprobeB.stderr'")
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

# --- PART D: GS link-status OSD render gate ---------------------------
# The DRM half of the GS OSD cannot run off-hardware, so this is the
# host-side gate for everything below it: .gfont load, layout arithmetic,
# formatting and the mask blitter. Same dump format as the MSP --osd-render
# gate above. The bar is two rows of icon-tagged items along the bottom
# edge, plus the recording indicator alone in the TOP-RIGHT corner, so the
# invariants are three bands of ink and nothing else: the indicator up top,
# then the two rows hugging the bottom inset. The synthetic font carries
# eight sizes (and a box for every icon); the bar takes its size from its
# sizing reference row (gs_compact.h).
echo "== GS render gate =="
python3 tools/msp/gen_gsfont.py --synthetic "$TMP/syn.gfont" \
  --sizes 19,21,22,24,26,34,38,56
"$MABURPLAY" --gs-render tests/fixtures/gs_snapshot_nominal.json \
  --out-gs "$TMP/gsbar.bin" --gsfont "$TMP/syn.gfont" --screen 1920x1080 \
  --rec recording --rec-elapsed 767 \
  --fps 60 --jit 5.2 --mbps 8.1 --res 1280x720 --lat 45/78

python3 - "$TMP/gsbar.bin" <<'EOF'
import struct, sys
d = open(sys.argv[1], "rb").read()
magic, w, h, stride = struct.unpack("<4I", d[:16])
assert magic == 0x5244534F, hex(magic)
assert (w, h, stride) == (1920, 1080, 1920), (w, h, stride)
px = struct.unpack("<%dI" % (h * stride), d[16:])

lit_rows = [y for y in range(h) if any(px[y * stride + x] for x in range(w))]
assert lit_rows, "nothing drawn at all"

# Group the lit scanlines into contiguous bands. THREE: the recording
# indicator alone at the top inset, then the two stacked rows at the bottom.
# The two rows are separated by a gap wide enough that no glyph bridges
# them -- a single bottom band means the row pitch collapsed and they are
# overlapping.
bands = []
for y in lit_rows:
    if bands and y == bands[-1][1] + 1:
        bands[-1][1] = y
    else:
        bands.append([y, y])
print("  ink bands: %s" % (bands,))
assert len(bands) == 3, "expected REC + two rows, got %d bands" % len(bands)

def extent(y0, y1):
    on = [x for y in range(y0, y1 + 1) for x in range(w) if px[y * stride + x]]
    return min(on), max(on)

# --- the recording indicator, alone in the top-right corner ---
ry0, ry1 = bands[0]
rx0, rx1 = extent(ry0, ry1)
print("  rec: x %d..%d y %d..%d" % (rx0, rx1, ry0, ry1))
# Its box's top edge sits on the 40 px inset; the ink starts a few px in
# (the cell's ascender gap and the shadow pad).
assert 40 <= ry0 < 40 + 24, "rec not at the top inset (y0=%d)" % ry0
# Right-flushed against the same 32 px inset the rows use.
assert 1920 - 32 - 24 <= rx1 <= 1920 - 32, "rec not flush right (x1=%d)" % rx1
# In the RIGHT half, unambiguously -- this is what says "corner", not "top".
assert rx0 > w // 2, "rec is not in the right half (x0=%d)" % rx0

# --- the two bottom rows ---
row_x = []
for i, (y0, y1) in enumerate(bands[1:]):
    x0, x1 = extent(y0, y1)
    row_x.append((x0, x1))
    print("  row %d: x %d..%d y %d..%d" % (i, x0, x1, y0, y1))
    # Inside the bar's own 32 px horizontal inset.
    assert x0 >= 32 and x1 <= w - 32, "row %d crosses its inset" % i
# The rows share a column grid: both start at column 0's left edge (each
# first item opens with its icon, so the ink lines up too).
assert row_x[0][0] == row_x[1][0], "rows do not share a left edge: %r" % (row_x,)
# The GRID is centred, loosely: every box is sized from its worst case, so
# short values sit legitimately off centre -- 160 px still discriminates
# against an edge-anchored grid.
gx0, gx1 = min(x for x, _ in row_x), max(x for _, x in row_x)
assert abs((gx0 + gx1) // 2 - w // 2) <= 160, "grid not centred (x %d..%d)" % (gx0, gx1)

# The block hugs the bottom: within the 24 px inset plus the cell's own
# descender.
assert h - 1 - bands[-1][1] < 40, "bar does not hug the bottom"
# Two rows, not three: the bottom block stays inside a band a bit over two
# cells tall (the chosen synthetic atlas is 38 px, cell 76).
assert bands[-1][1] - bands[1][0] < 3 * 76, "bottom block is taller than two rows"
# Nothing between the corner item and the rows.
assert bands[1][0] - bands[0][1] > 100, "rec and the rows have run together"
print("  OK gs compact bar: %dx%d lit=%d" %
      (w, h, sum(1 for y in range(h) for x in range(w) if px[y * stride + x])))
EOF

BAR_SHA=$(sha256sum "$TMP/gsbar.bin" | cut -d' ' -f1)
echo "gs compact bar sha256: $BAR_SHA"
if [ "$BAR_SHA" != "$BAR_SHA_EXPECTED" ]; then
  echo "GS compact bar render hash changed (expected $BAR_SHA_EXPECTED)" >&2
  exit 1
fi

# --- PART E: live run, SIGTERM -> clean shutdown ----------------------
# The oneshot drain never reaches the run loop. Run it for real on the ring
# maburgs left behind (null backend, no doorbell, no DVR, no overlays), then
# stop it the way S97maburplay does: exit 0, promptly, through the shutdown
# drain (null backend: nothing streams -> "stream idle").
echo "== PART E: live run, SIGTERM =="
cat > "$TMP/live.toml" <<EOF
ring_path = "$TMP/au-ring"
socket = "$TMP/none.sock"
backend = "null"
screen_mode = "1920x1080@60"

[dvr]
autostart = false
dir = "$TMP"
target = "gs"

[osd]
enable = false

[osd.gs]
enable = false
font = "$TMP/no-such.gfont"
EOF
"$MABURPLAY" -c "$TMP/live.toml" > "$TMP/live.out" 2> "$TMP/live.err" &
LIVE=$!
# Any exit from here on, a set -e one included, takes the player with it.
trap 'kill -9 "$LIVE" 2>/dev/null || true; rm -rf "$TMP"' EXIT
# SIGTERM only once the handler is in: main installs it (std::signal) well
# before it starts the feed thread and prints "feed thread on", so that line
# proves both. A fixed sleep could land before it under load, and SIGTERM's
# default action would kill the player (exit 143) instead of draining.
READY=0
for _ in $(seq 1 100); do   # up to 5 s
  if grep -q '^maburplay: feed thread on' "$TMP/live.err" 2>/dev/null; then READY=1; break; fi
  kill -0 "$LIVE" 2>/dev/null || break
  sleep 0.05
done
if [ "$READY" != 1 ]; then
  echo "FAIL: no 'maburplay: feed thread on' line within 5 s" >&2
  kill -9 "$LIVE" 2>/dev/null; cat "$TMP/live.err" >&2; exit 1
fi
# The delivered=13 check below needs the feed's first pump (the 13 retained
# AUs, read in well under a millisecond) to run before the drain request:
# there is no line for that, so give it a moment. The handler race above does
# not depend on this.
sleep 0.2

# Live config on 127.0.0.1:8403 (player_config_apply.h), against the real
# run loop: a set reaches its hook and the file, a refused one leaves the
# file byte-identical, get answers from the live copy. Skipped (not failed)
# only when another process holds the port on this host.
cfg_cmd() {
  python3 - "$1" <<'PY'
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(5)
s.sendto(sys.argv[1].encode(), ("127.0.0.1", 8403))
print(s.recv(4096).decode())
PY
}
# A failure here must not leave the player running (and holding 8403).
fail_live() {
  echo "FAIL: $1" >&2
  kill -9 "$LIVE" 2>/dev/null
  cat "$TMP/live.err" >&2
  exit 1
}
expect_reply() {  # <command> <wanted reply prefix>
  local got
  got=$(cfg_cmd "$1") || fail_live "no reply to '$1'"
  case "$got" in
    "$2"*) ;;
    *) fail_live "'$1' -> '$got' (want '$2...')" ;;
  esac
}
if grep -q '^maburplay: cfg: cannot bind' "$TMP/live.err"; then
  echo "SKIP live config: 127.0.0.1:8403 is held by another process"
else
  echo "== PART E: live config set/get =="
  expect_reply 'get dvr.target' 'ok "gs"'
  expect_reply 'set dvr.target = "both"' 'ok'
  grep -q '^target = "both"' "$TMP/live.toml" || fail_live "dvr.target not in the file"
  expect_reply 'get dvr.target' 'ok "both"'
  expect_reply 'set screen_mode = "1280x720@60"' 'ok'
  grep -q '^screen_mode = "1280x720@60"' "$TMP/live.toml" || fail_live "screen_mode not in the file"
  BEFORE_SHA=$(sha256sum "$TMP/live.toml" | cut -d' ' -f1)
  expect_reply 'set osd.gs.enable = true' 'err gs font: cannot open'
  expect_reply 'set backend = "mpp"' 'err not live: backend'
  [ "$(sha256sum "$TMP/live.toml" | cut -d' ' -f1)" = "$BEFORE_SHA" ] ||
    fail_live "a refused set changed the file"
  expect_reply 'get osd.gs.enable' 'ok false'
  grep -q '^maburplay: cfg set dvr.target = "both" -> ok (' "$TMP/live.err" ||
    fail_live "no cfg set log line"
fi
kill -TERM "$LIVE"
for _ in $(seq 1 50); do kill -0 "$LIVE" 2>/dev/null || break; sleep 0.1; done
if kill -0 "$LIVE" 2>/dev/null; then
  echo "FAIL: maburplay still running 5 s after SIGTERM" >&2
  kill -9 "$LIVE"; cat "$TMP/live.err" >&2; exit 1
fi
set +e; wait "$LIVE"; RC_LIVE=$?; set -e
cat "$TMP/live.err"
[ "$RC_LIVE" = 0 ] || { echo "FAIL: exit $RC_LIVE after SIGTERM (want 0)" >&2; exit 1; }
grep -q '^maburplay: shutdown: stream idle' "$TMP/live.err" || {
  echo "FAIL: no 'maburplay: shutdown: stream idle' line" >&2; exit 1; }
grep -q '^maburplay: feed thread on' "$TMP/live.err" || { echo "FAIL: no feed thread" >&2; exit 1; }
grep -q '^maburplay: feed: delivered=13 ' "$TMP/live.err" || {
  echo "FAIL: the feed thread did not read the 13 retained AUs" >&2; exit 1; }

echo "== player_e2e passed =="
