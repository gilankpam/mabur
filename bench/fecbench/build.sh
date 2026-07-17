#!/bin/sh
# Builds fecbench for host (x86 sanity) and for the SSC338Q (armv7, static,
# -mfpu=neon-vfpv4 to match common/CMakeLists.txt so the bench exercises the
# deployed neon-vtbl2-q16 GF path). Links mabur common/ sources UNMODIFIED —
# the baseline rows are the shipped code.
set -e
cd "$(dirname "$0")"
MABUR=../..
SHA=$(git -C "$MABUR" rev-parse --short HEAD 2>/dev/null || echo unknown)
DIRTY=$(git -C "$MABUR" diff --quiet -- common 2>/dev/null || echo "-dirty")

SRCS="candidates.cpp fecbench.cpp
  $MABUR/common/src/gf256.cpp
  $MABUR/common/src/sw_wire.cpp
  $MABUR/common/src/sw_encoder.cpp
  $MABUR/common/src/uep_encoder.cpp
  $MABUR/common/src/sbi.cpp
  $MABUR/common/src/frag.cpp
  $MABUR/common/src/nal.cpp
  $MABUR/common/src/crc16.cpp"
FLAGS="-O2 -std=c++17 -pthread -I $MABUR/common/include -DFECBENCH_SHA=\"\\\"$SHA$DIRTY\\\"\""

if command -v g++ >/dev/null 2>&1; then
  eval g++ $FLAGS $SRCS -o fecbench-host
else
  nix-shell -p gcc --run "g++ $FLAGS $SRCS -o fecbench-host"
fi
echo "built fecbench-host"

ARMXX=$MABUR/toolchain/cc/bin/armv7l-unknown-linux-musleabihf-g++
if [ -x "$ARMXX" ]; then
  eval "$ARMXX" $FLAGS -static -mfpu=neon-vfpv4 $SRCS -o fecbench-arm
  echo "built fecbench-arm"
else
  echo "arm toolchain not found at $ARMXX — skipped fecbench-arm" >&2
fi
