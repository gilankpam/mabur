#!/usr/bin/env bash
# Cross-build maburd as a DYNAMIC glibc binary with the OpenIPC Buildroot
# toolchain. Companion to (and eventual replacement for) build-arm.sh —
# rationale in docs/superpowers/plans/2026-08-28-venc-foldin.md Task B1.
#
# Deviation from the brief: `pkg-config` is not on this NixOS host's bare
# PATH (same gap build-arm.sh works around for its own musl build — see
# that script's step 1b). devourer's CMakeLists calls
# find_package(PkgConfig REQUIRED) unconditionally, so this script stages
# pkg-config the same way: `nix-build -o toolchain/pkg-config`, a GC root
# under toolchain/ (gitignored) so re-running this script is a fast no-op.
set -euo pipefail
cd "$(dirname "$0")/.."

OPENIPC_HOST_BIN="${OPENIPC_HOST_BIN:-$PWD/../openipc-builder/openipc/output/host/bin}"
export OPENIPC_HOST_BIN
[ -x "$OPENIPC_HOST_BIN/arm-openipc-linux-gnueabihf-gcc" ] || {
  echo "error: OpenIPC toolchain not found at $OPENIPC_HOST_BIN" >&2; exit 1; }

mkdir -p toolchain/glibc-staging out/arm
export MABUR_GLIBC_STAGING="$PWD/toolchain/glibc-staging"

# --- pkg-config itself (see deviation note above). Staged identically to
#     build-arm.sh's step 1b.
if [ ! -e toolchain/pkg-config ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkg-config' \
    -o toolchain/pkg-config
fi
PKG_CONFIG_BINDIR="$(readlink -f toolchain/pkg-config)/bin"
export PATH="$PKG_CONFIG_BINDIR:$PATH"

# --- static libusb for the glibc target (once) -----------------------------
LIBUSB_VER=1.0.27
if [ ! -e "$MABUR_GLIBC_STAGING/lib/libusb-1.0.a" ]; then
  mkdir -p toolchain/libusb-src && cd toolchain/libusb-src
  [ -d "libusb-$LIBUSB_VER" ] || {
    curl -LO "https://github.com/libusb/libusb/releases/download/v$LIBUSB_VER/libusb-$LIBUSB_VER.tar.bz2"
    tar xf "libusb-$LIBUSB_VER.tar.bz2"; }
  cd "libusb-$LIBUSB_VER"
  ./configure --host=arm-openipc-linux-gnueabihf --prefix="$MABUR_GLIBC_STAGING" \
    --enable-static --disable-shared --disable-udev \
    CC="$OPENIPC_HOST_BIN/arm-openipc-linux-gnueabihf-gcc"
  make -j"$(nproc)" && make install
  cd ../../..
fi
# Unset any host PKG_CONFIG_PATH so devourer's pkg_check_modules resolves
# against the staged ARM static libusb, not a host one (mirrors
# build-arm.sh's same guard).
unset PKG_CONFIG_PATH || true
export PKG_CONFIG_LIBDIR="$MABUR_GLIBC_STAGING/lib/pkgconfig"
export PKG_CONFIG_SYSROOT_DIR=""

# devourer's UsbOpen.cpp/UsbDeviceLock.cpp do
# `#include <libusb-1.0/libusb.h>`, which needs the *parent* include/ dir on
# the search path, not include/libusb-1.0 (what libusb-1.0.pc's Cflags:
# alone gives). Mirrors build-arm.sh's CPATH step.
CPATH="$MABUR_GLIBC_STAGING/include${CPATH:+:$CPATH}"
export CPATH

cmake -S . -B build-arm-glibc -DCMAKE_TOOLCHAIN_FILE=cmake/arm-openipc.cmake \
  -DCMAKE_BUILD_TYPE=Release -DMABUR_BUILD_TESTS=OFF \
  -DDEVOURER_JAGUAR1=OFF -DDEVOURER_8814=OFF -DDEVOURER_JAGUAR2_8822B=OFF \
  -DDEVOURER_JAGUAR2_8821C=OFF -DDEVOURER_JAGUAR3_8822C=OFF \
  -DDEVOURER_JAGUAR3_8822E=ON -DDEVOURER_KESTREL_8852B=OFF \
  -DDEVOURER_KESTREL_8852C=OFF -DDEVOURER_LOG_MAX_LEVEL=WARN
cmake --build build-arm-glibc -j"$(nproc)" --target maburd
"$OPENIPC_HOST_BIN/arm-openipc-linux-gnueabihf-strip" \
  build-arm-glibc/drone/maburd -o out/arm/maburd-glibc
"$OPENIPC_HOST_BIN/arm-openipc-linux-gnueabihf-readelf" -d out/arm/maburd-glibc | head -12
