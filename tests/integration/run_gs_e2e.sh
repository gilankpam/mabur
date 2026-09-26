#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD:-build}
MABURD=$BUILD/drone/maburd
MABURGS=$BUILD/gs/maburgs
FIX=tests/fixtures/frame_stream.bin
GSCFG=gs/bundle/maburgs.default.toml
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# RCF delivered after frame 1 (same trick as run_host_e2e.sh). 2-stream
# space (spec 2026-08-29-airtime-balance-uep): the fixture alternates
# TRAIL_R/TRAIL_N on its 12 P frames, so classify_frame routes 6 of them to
# sid 1 (enh) -- and the RCF IS needed for reachability here (MAX_RANGE
# sheds sid 1 until it lands), on top of exercising the RCF path and
# dropping the FEC overhead from the boot default (0.5) to 0.25 partway
# through, same as run_host_e2e.sh's RCF check.
python3 - "$TMP/rc.bin" <<'EOF'
import re, struct, sys, os
sys.path.insert(0, os.path.abspath(os.path.join("..", "devourer", "tools", "precoder")))
import rc_proto
# mabur owns the RC wire as of RC_VERSION 2 (2026-08-12): devourer's frozen
# rc_proto.py is pinned at RC_VERSION 1 and still packs the deleted pwr_idx
# byte plus the deleted ack_seq/score/layer_delivery fields, so its
# pack_rcf() output is rejected outright by maburd. Pack the 18-byte head
# here instead (magic, ver, type, flags, vtx_id, seq, profile,
# fec_overhead_base_x100, fec_overhead_enh_x100, probe_profile, hop_ch,
# hop_epoch, rec -- RC_VERSION 6, 2026-09-04, made probe_profile a fixed head
# byte, 0xFF = no probe stream; every bump since (7, T_CAL_CMD/T_CAL_RESULT
# plus a wider Telem; 8, relative calibration indices; 9, 2026-09-14, RCF
# gains hop_ch/hop_epoch -- 0/0 = no hop order issued; 11, 2026-09-26, RCF
# gains rec -- 0 = unknown) moved only the version byte plus trailing bytes.
# encode_profile and the CRC are unversioned.
# So read that byte from the header rather than pinning it: as a literal it
# half-landed the RC_VERSION 8 bump (2026-09-13) -- this script kept packing
# 7, maburd dropped the RCF as a foreign peer's, the shed of sid 1 never
# lifted, and the lost enhance stream read as a decoder bug.
RC_VERSION = int(re.search(r"RC_VERSION\s*=\s*(\d+)",
                           open("common/include/mabur/rc_proto.h").read()).group(1))
body = struct.pack("<HBBBIHBBBBBBB", rc_proto.RC_MAGIC, RC_VERSION, rc_proto.T_RCF, 0,
                   1, 1, rc_proto.encode_profile("ht", 4, 20), 25, 25, 0xFF, 0, 0, 0)
w = body + struct.pack("<H", rc_proto._crc(body))
with open(sys.argv[1], "wb") as f:
    f.write(struct.pack("<II", 1, len(w))); f.write(w)
EOF
"$MABURD" -c bundle/mabur.default.toml --dry-run --in "$FIX" \
  --out "$TMP/frames.bin" --rc-in "$TMP/rc.bin"

# maburgs captures each reassembled AU via --out-aus (PR C: the RTP output
# is gone), so verify_aus.py compares NAL lists against the fixture frames.
# --require-all iterates every sid present in the fixture (both 0 and 1
# here), so this is already a both-streams-present assertion; --min-stream1
# is added anyway for symmetry with --min-stream0 and to be explicit about
# what's being gated.
echo "== clean: all 13 frames recovered NAL-exact, both streams =="
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" --out-aus "$TMP/aus0.bin"
python3 tests/integration/verify_aus.py "$TMP/aus0.bin" "$FIX" --require-all \
  --min-stream0 1.0 --min-stream1 1.0

echo "== 15% single-card loss: both streams must fully deliver =="
# Seed pinned to 3: 15% loss lands across the WHOLE 30-body population (IDR
# at 0.5x overhead + P frames at 0.25x post-RCF, split across sid 0 base and
# sid 1 enh), so a bad LCG roll can rank-deficient the GF(256) solve for a
# frame on either stream -- a genuine FEC-capacity edge, not a decoder bug.
# Verified empirically against this exact fixture+seed: both streams clear
# (stream 0 7/7, stream 1 6/6); the loss floor is a property of this
# fixture's geometry, not a promise every seed holds.
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" \
  --drop-pct 15 --seed 3 --out-aus "$TMP/aus1.bin"
python3 tests/integration/verify_aus.py "$TMP/aus1.bin" "$FIX" \
  --min-stream0 1.0 --min-stream1 1.0

echo "== 2 cards, 20% independent loss each: union recovers everything, both streams =="
# Seed pinned to 2: --cards N round-robins *frames* across cards rather than
# duplicating them, so each body lands on exactly one card's independent
# per-card LCG drop roll, and a short fixture can correlate that roll onto the
# same body's sources+repairs and rank-deficient the GF(256) solve. That is a
# genuine FEC-capacity edge, not a decoder bug. Whole-frame units make it
# sharper than the pre-frame-shm packet stream did: ONE unrecoverable body
# can cost a whole frame. Swept seeds 1-60 at the unchanged 20% drop-pct
# under the current bundle geometry (scalar-332/window-32/bpb-4); seed 2
# clears both streams.
"$MABURGS" -c "$GSCFG" --dry-run --in "$TMP/frames.bin" \
  --cards 2 --drop-pct 20 --seed 2 --out-aus "$TMP/aus2.bin"
python3 tests/integration/verify_aus.py "$TMP/aus2.bin" "$FIX" --require-all \
  --min-stream0 1.0 --min-stream1 1.0

echo "== in-flight hop, two-card: injected verdict -> RCF -> plan -> ladder -> verify_pass =="
# Spec 2026-09-14-inflight-channel-hop, task-15-brief.md step 1. The
# injection seam is MABUR_HOP_INJECT=<ch>:<score>, read only when
# gs/CMakeLists.txt compiled MABUR_TEST in (auto-ON for MABUR_BUILD_TESTS,
# never a device build -- tools/build-arm64.sh passes
# -DMABUR_BUILD_TESTS=OFF), and only consulted by maburgs's OWN
# --dry-run path (gs/src/main.cpp's run_hop_inject_test()), so every other
# scenario in this file is unaffected. Reuses this file's own
# $TMP/frames.bin (maburd's real dry-run encode of the fixture, from the
# top of this script) -- the confirming AU below is a genuine video body
# out of that stream, not a synthetic one.
#
# hop.enable defaults false in the bundle (first flights are observe-only,
# spec section 10) -- both configs below flip it on, same as debug_log
# (off by default; ctl.log/scan.log only exist when it's on).
HOPCFG2="$TMP/gs-hop2.toml"
sed -e "/^\[debug_log\]/,/^\[/ s|^enable *= .*|enable = true|" \
    -e "/^\[debug_log\]/,/^\[/ s|^dir *= .*|dir = \"$TMP/hop2\"|" \
    -e "/^\[hop\]/,/^\[/ s|^enable *= .*|enable = true|" \
    "$GSCFG" > "$HOPCFG2"
# Layout-sensitive sed, like run_gs_au_e2e.sh's symbol_size pin: assert it
# actually landed rather than silently running against the bundle default.
grep -qx "dir = \"$TMP/hop2\"" "$HOPCFG2" || {
  echo "FAIL: debug_log.dir sed did not match in $HOPCFG2 -- bundle layout changed" >&2
  exit 1
}
mkdir -p "$TMP/hop2"
# DebugSession's session marker is a FIXED path (/tmp/mabur-session, not
# test-isolated): without clearing it a second maburgs run in this same
# script (the one-card scenario below) would ADOPT this run's session
# directory instead of allocating its own 0000 under a fresh root.
rm -f /tmp/mabur-session
MABUR_HOP_INJECT="149:500" MABUR_HOP_RCF_OUT="$TMP/hop2-rcf.bin" \
  "$MABURGS" -c "$HOPCFG2" --dry-run --in "$TMP/frames.bin" --cards 2

SCAN2="$TMP/hop2/0000/scan.log"
CTL2="$TMP/hop2/0000/ctl.log"
[ -f "$SCAN2" ] || { echo "FAIL: $SCAN2 missing"; exit 1; }
[ -f "$CTL2" ] || { echo "FAIL: $CTL2 missing"; exit 1; }

# RCF bytes, decoded with the real parse_rcf (not inferred offsets).
"$BUILD/tests/rcf_dump" "$TMP/hop2-rcf.bin" > "$TMP/hop2-rcf.txt"
grep -qx "hop_ch=149" "$TMP/hop2-rcf.txt" || {
  echo "FAIL: RCF does not carry hop_ch=149:"; cat "$TMP/hop2-rcf.txt"; exit 1; }
grep -qx "hop_epoch=1" "$TMP/hop2-rcf.txt" || {
  echo "FAIL: RCF does not carry a new hop_epoch (want 1):"; cat "$TMP/hop2-rcf.txt"; exit 1; }

# ChannelPlan moved the lead card (M ... hop_lead), then followed once the
# injected AU confirmed on the target (M ... hop_follow).
grep -qE '^M [0-9.]+ 1 [0-9]+ 149 hop_lead$' "$SCAN2" || {
  echo "FAIL: no scan.log M line with reason hop_lead:"; cat "$SCAN2"; exit 1; }
grep -qE '^M [0-9.]+ all [0-9]+ 149 hop_follow$' "$SCAN2" || {
  echo "FAIL: no scan.log M line with reason hop_follow:"; cat "$SCAN2"; exit 1; }
# The ladder's restore -> ctl.log hop_restore, fired at the order (same
# tick vrx.restore_rung() runs in run_hop_inject_test's apply_action()).
grep -qE '^E [0-9.]+ -?[0-9]+ -?[0-9]+ hop_restore ' "$CTL2" || {
  echo "FAIL: no ctl.log E line with reason hop_restore:"; cat "$CTL2"; exit 1; }
# verify_pass after >= hop.verify_ms (1000 ms bundle default) of healthy
# windows following the confirm.
grep -qE '^H [0-9.]+ verify_pass ' "$SCAN2" || {
  echo "FAIL: no scan.log H verify_pass line:"; cat "$SCAN2"; exit 1; }
echo "OK: two-card hop end to end (order -> RCF -> hop_lead -> hop_follow -> hop_restore -> verify_pass)"

echo "== in-flight hop, one card: old channel for one_card_repeats RCFs, then OneCardRetune =="
# Same injection seam, --cards 1: gs/src/main.cpp's HopController drives
# HopAction::OneCardRetune only after hop.one_card_repeats (5) real RCF
# sends carrying the order on the OLD channel -- the two real bugs the
# brief calls out (moving the radio immediately on Order, and never
# collecting ranking data on one card) both live on this path, and until
# now nothing but reading the code pinned either fix.
HOPCFG1="$TMP/gs-hop1.toml"
sed -e "/^\[debug_log\]/,/^\[/ s|^enable *= .*|enable = true|" \
    -e "/^\[debug_log\]/,/^\[/ s|^dir *= .*|dir = \"$TMP/hop1\"|" \
    -e "/^\[hop\]/,/^\[/ s|^enable *= .*|enable = true|" \
    "$GSCFG" > "$HOPCFG1"
grep -qx "dir = \"$TMP/hop1\"" "$HOPCFG1" || {
  echo "FAIL: debug_log.dir sed did not match in $HOPCFG1 -- bundle layout changed" >&2
  exit 1
}
mkdir -p "$TMP/hop1"
rm -f /tmp/mabur-session
MABUR_HOP_INJECT="149:500" MABUR_HOP_RCF_OUT="$TMP/hop1-rcf.bin" \
  "$MABURGS" -c "$HOPCFG1" --dry-run --in "$TMP/frames.bin" --cards 1

SCAN1="$TMP/hop1/0000/scan.log"
[ -f "$SCAN1" ] || { echo "FAIL: $SCAN1 missing"; exit 1; }
"$BUILD/tests/rcf_dump" "$TMP/hop1-rcf.bin" > "$TMP/hop1-rcf.txt"
grep -qx "hop_ch=149" "$TMP/hop1-rcf.txt" || {
  echo "FAIL: one-card RCF does not carry hop_ch=149:"; cat "$TMP/hop1-rcf.txt"; exit 1; }

grep -qE '^H [0-9.]+ order ' "$SCAN1" || {
  echo "FAIL: no scan.log H order line (one-card):"; cat "$SCAN1"; exit 1; }
grep -qE '^H [0-9.]+ one_card_retune ' "$SCAN1" || {
  echo "FAIL: no scan.log H one_card_retune line:"; cat "$SCAN1"; exit 1; }
# feedback_ms is 50 in the bundle: 5 repeats land the retune ~250 ms after
# the order, never before -- this is the "radio moved immediately" bug's
# regression guard. elapsed_ms is one_card_retune's own field (H's 6th
# column), not a timestamp difference.
RETUNE_ELAPSED=$(awk '/ one_card_retune /{print $NF; exit}' "$SCAN1")
awk -v e="$RETUNE_ELAPSED" 'BEGIN{exit !(e >= 200 && e <= 400)}' || {
  echo "FAIL: one_card_retune elapsed_ms=$RETUNE_ELAPSED, want ~250 (5 * feedback_ms 50)"; exit 1; }
grep -qE '^M [0-9.]+ all [0-9]+ 149 hop_one_card$' "$SCAN1" || {
  echo "FAIL: no scan.log M line with reason hop_one_card:"; cat "$SCAN1"; exit 1; }
grep -qE '^M [0-9.]+ all [0-9]+ 149 hop_follow$' "$SCAN1" || {
  echo "FAIL: no scan.log M line with reason hop_follow (one-card):"; cat "$SCAN1"; exit 1; }
grep -qE '^H [0-9.]+ verify_pass ' "$SCAN1" || {
  echo "FAIL: no scan.log H verify_pass line (one-card):"; cat "$SCAN1"; exit 1; }
echo "OK: one-card hop end to end (order on old channel x5 -> OneCardRetune -> confirm -> verify_pass)"

echo "== all GS E2E checks passed =="
