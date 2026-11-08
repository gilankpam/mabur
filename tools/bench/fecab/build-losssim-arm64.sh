#!/usr/bin/env bash
# tools/bench/fecab/build-losssim-arm64.sh -- cross-builds the loss-sim
# maburgs the FEC placement A/B rig (tools/bench/fecab/run.sh) needs on
# the GS: musl-static, MABUR_LOSS_SIM=ON (the bench's --loss-sim control
# socket, tools/bench/losssim.py's protocol), MABUR_PLAYER_HW=OFF (no
# player code in this binary). Its own build dir (build-arm64-losssim) so
# it never collides with tools/build-arm64.sh's build-arm64; reuses the
# SAME toolchain/ staging tools/build-arm64.sh stages 1-2 build (cross
# gcc + pkg-config + static libusb), so running that script first (or
# ever) makes this one a fast no-op on those steps.
#
# RUN THIS SCRIPT BARE, same reason as tools/build-arm64.sh: wrapping it
# in `nix-shell -p pkg-config libusb1` leaks the host's libusb into the
# cross build, because Nix's pkg-config wrapper overrides
# PKG_CONFIG_LIBDIR whenever libusb1 is present in the shell.
set -euo pipefail
cd "$(dirname "$0")/../../.."

mkdir -p toolchain out/arm64

TARGET_TRIPLE=aarch64-unknown-linux-musl

# --- 1. Cross compiler (Nix's musl-static aarch64 stdenv) -----------------
if [ ! -e toolchain/cc-arm64 ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.aarch64-multiplatform.pkgsStatic.stdenv.cc' \
    -o toolchain/cc-arm64
fi
CC_BINDIR="$(readlink -f toolchain/cc-arm64)/bin"
export MABUR_CROSS_CC="$CC_BINDIR/${TARGET_TRIPLE}-gcc"
export MABUR_CROSS_CXX="$CC_BINDIR/${TARGET_TRIPLE}-g++"
export PATH="$CC_BINDIR:$PATH"

if [ ! -x "$MABUR_CROSS_CC" ]; then
  echo "error: expected cross gcc at $MABUR_CROSS_CC -- tuple prefix may have" >&2
  echo "changed upstream; check $CC_BINDIR for the actual *-gcc name and fix" >&2
  echo "TARGET_TRIPLE above." >&2
  exit 1
fi

# --- 1b. pkg-config itself -------------------------------------------------
if [ ! -e toolchain/pkg-config ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkg-config' \
    -o toolchain/pkg-config
fi
PKG_CONFIG_BINDIR="$(readlink -f toolchain/pkg-config)/bin"
export PATH="$PKG_CONFIG_BINDIR:$PATH"

# --- 2. Static libusb for the same target ----------------------------------
# NB: `nix-build -o toolchain/libusb-arm64-out` on a multi-output
# derivation's `.dev` output always *appends* "-dev" to whatever symlink
# name is given (so "-o toolchain/libusb-arm64-out" for the .dev output
# actually creates toolchain/libusb-arm64-out-dev, not
# toolchain/libusb-arm64-out) -- hence reading back
# "toolchain/libusb-arm64-out-dev" below rather than the name passed to -o
# (same quirk tools/build-arm64.sh works around).
if [ ! -e toolchain/libusb-arm64-out ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.aarch64-multiplatform.pkgsStatic.libusb1' \
    -o toolchain/libusb-arm64-out
fi
if [ ! -e toolchain/libusb-arm64-out-dev ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.aarch64-multiplatform.pkgsStatic.libusb1.dev' \
    -o toolchain/libusb-arm64-out
fi

export MABUR_STAGING="$PWD/toolchain/staging"
mkdir -p "$MABUR_STAGING"
ln -sfn "$(readlink -f toolchain/libusb-arm64-out)/lib" "$MABUR_STAGING/lib"
ln -sfn "$(readlink -f toolchain/libusb-arm64-out-dev)/include" "$MABUR_STAGING/include"

unset PKG_CONFIG_PATH || true
PKG_CONFIG_LIBDIR="$(readlink -f toolchain/libusb-arm64-out-dev)/lib/pkgconfig"
export PKG_CONFIG_LIBDIR
export PKG_CONFIG_SYSROOT_DIR=""

CPATH="$(readlink -f toolchain/libusb-arm64-out-dev)/include${CPATH:+:$CPATH}"
export CPATH

# --- 3. Configure + build maburgs only, loss-sim on, no player ------------
cmake -S . -B build-arm64-losssim -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-musl.cmake \
  -DCMAKE_BUILD_TYPE=Release -DMABUR_BUILD_TESTS=OFF -DMABUR_BUILD_DRONE=OFF \
  -DMABUR_BUILD_GS=ON -DMABUR_LOSS_SIM=ON -DDEVOURER_JAGUAR1=OFF -DDEVOURER_8814=OFF \
  -DDEVOURER_JAGUAR2_8822B=OFF -DDEVOURER_JAGUAR2_8821C=OFF \
  -DDEVOURER_JAGUAR3_8822C=OFF -DDEVOURER_JAGUAR3_8822E=ON -DDEVOURER_8733B=OFF \
  -DDEVOURER_KESTREL_8852B=OFF -DDEVOURER_KESTREL_8852C=OFF \
  -DDEVOURER_LOG_MAX_LEVEL=WARN -DMABUR_PLAYER_HW=OFF

cmake --build build-arm64-losssim -j"$(nproc)" --target maburgs

"${TARGET_TRIPLE}-strip" build-arm64-losssim/gs/maburgs -o out/arm64/maburgs-losssim
