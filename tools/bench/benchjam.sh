#!/usr/bin/env bash
# benchjam — co-channel 802.11 interferer for BENCH link-stress ONLY.
#
# A thin wrapper around devourer's txdemo (which is already a continuous
# frame injector with all the knobs). It parks a spare RTL8822EU on the
# mabur op channel and floods QoS-Data frames, so the real air path sees
# contention + collision loss: CCA-defer on the drone TX, whole-aggregate
# misses on the GS RX, FEC repair, and a ladder that demotes under LOSS.
#
# What it is / is not (see docs/link-adaptation.md, memory ladder-stress):
#   - It is a packet interferer — like another 5 GHz network on the channel.
#   - It is NOT a graded path-loss / fading source and NOT a CW noise jammer.
#     Close in it is an on/off cliff, not a smooth SNR ramp; for rung-by-rung
#     promote/demote you still want attenuation or a walk-out.
#
# Bench only. Do not ship, do not wire into any config. One radio, TX-only
# while it runs (it cannot also listen). Co-locate + attenuate or it just
# desenses the GS RX chains instead of grading them.
#
# The source address matters. txdemo's built-in TX SA is the canonical mabur
# SA 57:42:75:05:d6:00 -- the very address the GS filters on to decide a frame
# is its own (gs/src/radio_frontend.cpp sa_canonical), so jamming with the
# default would have the GS book the interference as mabur traffic instead of
# counting it foreign. We therefore default DEVOURER_TX_SA to a locally
# administered unicast address that is emphatically not mabur's; --sa overrides.
#
# Severity levers, gentle -> harsh:  raise --pps, raise --bytes, raise
# --power, lower --rate.  For bursty interference use --burst-on/--burst-off.
#
# Stop with Ctrl-C (clean de-init), or pass --secs N to auto-stop.
set -euo pipefail

# --- defaults (a deliberately moderate starting point) ----------------------
DEVOURER_DIR="${DEVOURER_DIR:-$(cd "$(dirname "$0")/../../../devourer" && pwd)}"
TXDEMO="${TXDEMO:-$DEVOURER_DIR/build/txdemo}"
CHANNEL=136          # mabur home channel (docs/channel-select.md); set to the op channel in use
RATE="6M"            # low legacy rate => each frame holds the air longer => more airtime steal
BYTES=1000           # on-air PSDU size
PPS=250              # frames/sec (translated to txdemo's inter-frame delay)
POWER=""             # TX-power LUT step (DEVOURER_TX_PKT_OFSET); empty = library calibrated default
BURST_ON=""          # ms on  (bursty duty cycle; empty = steady)
BURST_OFF=""         # ms off
SECS=""              # auto-stop after N s; empty = run until Ctrl-C
BUS=""; PORT=""      # pin the exact dongle; empty = auto-detect the 0bda:a81a
VID="0x0bda"; PID="0xa81a"
SA="02:4a:41:4d:00:01"   # frame source address: locally administered, unicast, "JAM"
CANONICAL_SA="57:42:75:05:d6:00"

usage() {
  cat <<EOF
benchjam — co-channel 802.11 interferer for BENCH link-stress ONLY.
Parks a spare RTL8822EU on the mabur op channel and floods QoS-Data frames
so the real air path sees contention + collision loss. It is a packet
interferer (another network on the channel), NOT a graded fading/noise
source. Bench only; TX-only while running; co-locate + attenuate.

usage: $(basename "$0") [options]
  --channel N     op channel to jam           (default $CHANNEL)
  --rate STR      DEVOURER_TX_RATE, e.g. 6M, 24M, HT_MCS4   (default $RATE)
  --bytes N       on-air PSDU bytes           (default $BYTES)
  --pps N         frames per second           (default $PPS)
  --power N       TX-power LUT step, higher=louder (default: calibrated)
  --burst-on MS   bursty: air for MS ms ...    (default: steady)
  --burst-off MS  ... then idle for MS ms
  --secs N        auto-stop after N seconds    (default: until Ctrl-C)
  --sa MAC        frame source address         (default $SA)
                  must NOT be mabur's canonical $CANONICAL_SA,
                  or the GS counts the jam as its own traffic
  --bus N --port a.b.c   pin the dongle        (default: auto-detect $VID:$PID)
  -h|--help
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --channel) CHANNEL="$2"; shift 2;;
    --rate)    RATE="$2"; shift 2;;
    --bytes)   BYTES="$2"; shift 2;;
    --pps)     PPS="$2"; shift 2;;
    --power)   POWER="$2"; shift 2;;
    --burst-on)  BURST_ON="$2"; shift 2;;
    --burst-off) BURST_OFF="$2"; shift 2;;
    --secs)    SECS="$2"; shift 2;;
    --sa)      SA="$2"; shift 2;;
    --bus)     BUS="$2"; shift 2;;
    --port)    PORT="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "unknown arg: $1" >&2; usage; exit 2;;
  esac
done

# --- the source address is load-bearing, so check it ------------------------
printf '%s' "$SA" | grep -qiE '^([0-9a-f]{2}:){5}[0-9a-f]{2}$' || {
  echo "--sa: not a MAC address: $SA" >&2; exit 2
}
if [ "$(printf '%s' "$SA" | tr 'A-Z' 'a-z')" = "$CANONICAL_SA" ]; then
  echo "--sa: refusing the canonical mabur SA $CANONICAL_SA -- the GS would" >&2
  echo "      filter these frames in as its own traffic and the jam would not" >&2
  echo "      show up as foreign. Pick any other locally administered address." >&2
  exit 2
fi

[ -x "$TXDEMO" ] || {
  echo "txdemo not found/executable at $TXDEMO" >&2
  echo "build it:  nix-shell -p pkg-config libusb1 --run \\" >&2
  echo "           \"cmake --build $DEVOURER_DIR/build -j --target txdemo\"" >&2
  exit 3
}

# --- auto-detect the dongle's bus/port from sysfs if not pinned -------------
if [ -z "$BUS" ]; then
  vid="${VID#0x}"; pid="${PID#0x}"
  for d in /sys/bus/usb/devices/*-*; do
    b="$(basename "$d")"; case "$b" in *:*) continue;; esac   # skip interface dirs
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] || continue
    BUS="${b%%-*}"; PORT="${b#*-}"          # "5-1" -> bus 5, port "1"
    break
  done
  [ -n "$BUS" ] || { echo "no $VID:$PID found on the bus — plug the card in" >&2; exit 3; }
fi

# --- translate pps -> txdemo pacing -----------------------------------------
# txdemo's inter-frame delay is integer ms (DEVOURER_TX_INTERVAL_MS, default 2).
# For >=1000 pps we cannot express sub-ms there, so flatten the interval and
# let send_packet pace naturally (DEVOURER_TX_GAP_US=0). Below that, round.
if [ "$PPS" -ge 1000 ]; then
  export DEVOURER_TX_INTERVAL_MS=0
  export DEVOURER_TX_GAP_US=0
  pace="flat-out (~as fast as the chip will take)"
else
  ivl=$(( 1000 / PPS )); [ "$ivl" -lt 1 ] && ivl=1
  export DEVOURER_TX_INTERVAL_MS="$ivl"
  pace="~$(( 1000 / ivl )) fps (interval ${ivl} ms)"
fi

# --- assemble the txdemo environment ----------------------------------------
export DEVOURER_USB_BUS="$BUS" DEVOURER_USB_PORT="$PORT"
export DEVOURER_CHANNEL="$CHANNEL"
export DEVOURER_TX_RATE="$RATE"
export DEVOURER_TX_PAYLOAD_BYTES="$BYTES"
export DEVOURER_TX_QOS_DATA=1          # realistic broadcast QoS-Data => marks the medium busy
export DEVOURER_TX_SA="$SA"            # never the canonical SA; see the note up top
[ -n "$POWER" ]     && export DEVOURER_TX_PKT_OFSET="$POWER"
[ -n "$BURST_ON" ]  && export DEVOURER_TX_BURST_ON_MS="$BURST_ON"
[ -n "$BURST_OFF" ] && export DEVOURER_TX_BURST_OFF_MS="$BURST_OFF"

cat >&2 <<EOF
== benchjam (BENCH interferer — co-channel, TX-only) ==
  dongle    : $VID:$PID  bus $BUS port $PORT
  channel   : $CHANNEL   rate $RATE   psdu ${BYTES}B
  source    : $SA (foreign to mabur -- GS counts it as interference)
  pacing    : $pace
  power     : ${POWER:-calibrated default}${BURST_ON:+   burst ${BURST_ON}on/${BURST_OFF:-0}off ms}
  stop      : Ctrl-C${SECS:+   (auto-stop in ${SECS}s)}
  reminder  : this steals airtime on ch $CHANNEL; make sure the link is on it,
              and keep TX power/distance sane or you just desense the GS.
EOF

# --- run txdemo directly (its RPATH resolves libusb from the nix store, so no
#     nix-shell wrapper is needed — and a wrapper would swallow our SIGINT and
#     leave txdemo reparented and running). --secs uses timeout's clean SIGINT
#     so the chip de-inits; otherwise exec so Ctrl-C reaches txdemo directly.
if [ -n "$SECS" ]; then
  # timeout returns 124 when it has to stop the command at the deadline — for
  # --secs that IS the intended stop, so report success instead.
  rc=0; timeout -s INT "${SECS}s" "$TXDEMO" || rc=$?
  [ "$rc" = 124 ] && rc=0
  exit "$rc"
else
  exec "$TXDEMO"
fi
