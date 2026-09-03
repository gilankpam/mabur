#!/usr/bin/env bash
# Cross-build the drone-side ARM binaries with the OpenIPC Buildroot
# toolchain: maburd (DYNAMIC glibc, armv7-a hard-float) plus the two bench
# TX harnesses. This replaced the musl/static build on 2026-08-29 with the
# venc fold-in flag day (was tools/build-arm-glibc.sh; the musl script and
# cmake/arm-musl.cmake are deleted).
#
# Why glibc-dynamic, not musl-static any more: maburd now runs the encoder
# in-process, and the SigmaStar MI libraries it drives (libmi_venc.so and
# friends, shipped in the OpenIPC rootfs) are dlopen'd at RUNTIME by
# drone/venc/star6e_*.c — they are never linked here. dlopen against
# prebuilt glibc shared objects requires a glibc DYNAMIC executable; a
# static binary has no loader to do it with, so musl-static is simply not
# an option any more and there is no second ABI to keep alive. Note
# drone/vendor is NOT those libraries: it is the frame-shm ring
# (venc_frame_ring.[ch]), ordinary C compiled into maburd like any other
# source. The OpenIPC rootfs the drone runs supplies every
# NEEDED library (libstdc++.so.6 lives in /usr/lib, not /lib — the loader
# finds it, but remember that when auditing a stripped rootfs).
#
# Deviation from the brief: `pkg-config` is not on this NixOS host's bare
# PATH. devourer's CMakeLists calls find_package(PkgConfig REQUIRED)
# unconditionally, so this script stages pkg-config via
# `nix-build -o toolchain/pkg-config`, a GC root under toolchain/
# (gitignored) so re-running this script is a fast no-op.
set -euo pipefail
cd "$(dirname "$0")/.."

OPENIPC_HOST_BIN="${OPENIPC_HOST_BIN:-$PWD/../openipc-builder/openipc/output/host/bin}"
export OPENIPC_HOST_BIN
[ -x "$OPENIPC_HOST_BIN/arm-openipc-linux-gnueabihf-gcc" ] || {
  echo "error: OpenIPC toolchain not found at $OPENIPC_HOST_BIN" >&2; exit 1; }

mkdir -p toolchain/glibc-staging out/arm
export MABUR_GLIBC_STAGING="$PWD/toolchain/glibc-staging"

# --- pkg-config itself (see deviation note above). Staged identically to
#     the musl build used to stage it (deleted with that script).
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
# against the staged ARM static libusb, not a host one (libusb is the one
# dependency still linked statically — the drone rootfs has no libusb).
unset PKG_CONFIG_PATH || true
export PKG_CONFIG_LIBDIR="$MABUR_GLIBC_STAGING/lib/pkgconfig"
export PKG_CONFIG_SYSROOT_DIR=""

# devourer's UsbOpen.cpp/UsbDeviceLock.cpp do
# `#include <libusb-1.0/libusb.h>`, which needs the *parent* include/ dir on
# the search path, not include/libusb-1.0 (what libusb-1.0.pc's Cflags:
# alone gives).
CPATH="$MABUR_GLIBC_STAGING/include${CPATH:+:$CPATH}"
export CPATH

# Every chip we do not fly is switched OFF by name. New devourer releases add
# new DEVOURER_<chip> options that default ON, so they opt themselves in on the
# next sync: DEVOURER_8733B did exactly that on 2026-09-03 and cost +385 KB
# (982 KB -> 1.37 MB) on a rootfs with ~3 MB free that holds two maburd
# generations. After syncing ../devourer, diff its CMakeLists option() lines
# against this list.
cmake -S . -B build-arm-glibc -DCMAKE_TOOLCHAIN_FILE=cmake/arm-openipc.cmake \
  -DCMAKE_BUILD_TYPE=Release -DMABUR_BUILD_TESTS=OFF \
  -DDEVOURER_JAGUAR1=OFF -DDEVOURER_8814=OFF -DDEVOURER_JAGUAR2_8822B=OFF \
  -DDEVOURER_JAGUAR2_8821C=OFF -DDEVOURER_JAGUAR3_8822C=OFF \
  -DDEVOURER_JAGUAR3_8822E=ON -DDEVOURER_8733B=OFF \
  -DDEVOURER_KESTREL_8852B=OFF \
  -DDEVOURER_KESTREL_8852C=OFF -DDEVOURER_LOG_MAX_LEVEL=WARN
# linkbench-tx / txagcbench-tx are the drone-side halves of the two bench
# harnesses (bench/txagcbench/run_sweep.sh expects out/arm/txagcbench-tx).
# They carry over from the musl script unchanged; they build none of
# drone/venc, but they ship to the same rootfs, so one toolchain is enough.
cmake --build build-arm-glibc -j"$(nproc)" --target maburd linkbench-tx txagcbench-tx
STRIP="$OPENIPC_HOST_BIN/arm-openipc-linux-gnueabihf-strip"
"$STRIP" build-arm-glibc/drone/maburd                     -o out/arm/maburd
"$STRIP" build-arm-glibc/bench/linkbench/linkbench-tx     -o out/arm/linkbench-tx
"$STRIP" build-arm-glibc/bench/txagcbench/txagcbench-tx   -o out/arm/txagcbench-tx
"$OPENIPC_HOST_BIN/arm-openipc-linux-gnueabihf-readelf" -d out/arm/maburd | head -12
