#!/usr/bin/env bash
# Cross-build maburd for ARM (armv7-a hard-float) / musl, fully static.
#
# Deviations from the original Bootlin-based plan (see
# .superpowers/sdd/task-16-brief.md and task-16-report.md for the full
# rationale): this host is NixOS, where prebuilt toolchains like Bootlin's
# are dynamically linked against /lib64/ld-linux-x86-64.so.2 — a path that
# does not exist on NixOS, so the downloaded compiler cannot even execute.
# Instead this script uses Nix's own cross stdenv
# (pkgsCross.armv7l-hf-multiplatform.pkgsStatic), which targets
# armv7l-unknown-linux-musleabihf and works natively on NixOS. It also
# provides a prebuilt *static* libusb1 for that target, so — unlike the
# draft — this script does NOT configure/build libusb from source; it just
# resolves the Nix package and points pkg-config at it.
#
# Both the compiler and libusb are realized via `nix-build` with an output
# symlink under toolchain/ (gitignored). That symlink is a GC root, so the
# store paths survive `nix-collect-garbage` between runs and re-running this
# script is a fast no-op (nix-build short-circuits, cmake/make do incremental
# rebuilds).
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p toolchain out/arm

TARGET_TRIPLE=armv7l-unknown-linux-musleabihf

# --- 1. Cross compiler (Nix's musl-static ARMv7 hard-float stdenv) ---------
if [ ! -e toolchain/cc ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.armv7l-hf-multiplatform.pkgsStatic.stdenv.cc' \
    -o toolchain/cc
fi
CC_BINDIR="$(readlink -f toolchain/cc)/bin"
export MABUR_CROSS_CC="$CC_BINDIR/${TARGET_TRIPLE}-gcc"
export MABUR_CROSS_CXX="$CC_BINDIR/${TARGET_TRIPLE}-g++"
export PATH="$CC_BINDIR:$PATH"

if [ ! -x "$MABUR_CROSS_CC" ]; then
  echo "error: expected cross gcc at $MABUR_CROSS_CC — tuple prefix may have" >&2
  echo "changed upstream; check $CC_BINDIR for the actual *-gcc name and fix" >&2
  echo "TARGET_TRIPLE above." >&2
  exit 1
fi

# --- 1b. pkg-config itself. Not present on a bare NixOS PATH outside a
#         nix-shell (mirrors why the host build needs
#         `nix-shell -p pkg-config libusb1`); staged the same way as the
#         cross compiler above so this script is self-contained.
if [ ! -e toolchain/pkg-config ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkg-config' \
    -o toolchain/pkg-config
fi
PKG_CONFIG_BINDIR="$(readlink -f toolchain/pkg-config)/bin"
export PATH="$PKG_CONFIG_BINDIR:$PATH"

# --- 2. Static libusb for the same target (prebuilt by Nix, not built from
#        source: pkgsCross...pkgsStatic.libusb1 already ships a static
#        libusb-1.0.a + libusb-1.0.pc for armv7l-unknown-linux-musleabihf).
# NB: `nix-build -o toolchain/libusb-out` on a multi-output derivation's
# `.dev` output always *appends* "-dev" to whatever symlink name is given
# (so "-o toolchain/libusb-out" for the .dev output actually creates
# toolchain/libusb-out-dev, not toolchain/libusb-out) — hence reading back
# "toolchain/libusb-out-dev" below rather than the name passed to -o.
if [ ! -e toolchain/libusb-out ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.armv7l-hf-multiplatform.pkgsStatic.libusb1' \
    -o toolchain/libusb-out
fi
if [ ! -e toolchain/libusb-out-dev ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.armv7l-hf-multiplatform.pkgsStatic.libusb1.dev' \
    -o toolchain/libusb-out
fi

# CMAKE_FIND_ROOT_PATH staging dir: a minimal fake sysroot with lib/ and
# include/ symlinked straight at the two Nix libusb outputs. Also see
# cmake/arm-musl.cmake for why CMAKE_FIND_ROOT_PATH_MODE_{LIBRARY,INCLUDE}
# is BOTH, not the original plan's ONLY: pkg-config resolves libusb's
# location to its real (out-of-root) /nix/store path, and ONLY mode drops
# any find_library()/find_path() candidate outside CMAKE_FIND_ROOT_PATH
# instead of falling back to it verbatim — silently losing the -L flag.
export MABUR_STAGING="$PWD/toolchain/staging"
mkdir -p "$MABUR_STAGING"
ln -sfn "$(readlink -f toolchain/libusb-out)/lib" "$MABUR_STAGING/lib"
ln -sfn "$(readlink -f toolchain/libusb-out-dev)/include" "$MABUR_STAGING/include"

# Unset any host PKG_CONFIG_LIBDIR/PATH from the environment so devourer's
# pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0) cannot
# resolve against the host's libusb (e.g. from `nix-shell -p libusb1` used
# for the host build) instead of the staged ARM static one.
unset PKG_CONFIG_PATH || true
PKG_CONFIG_LIBDIR="$(readlink -f toolchain/libusb-out-dev)/lib/pkgconfig"
export PKG_CONFIG_LIBDIR
export PKG_CONFIG_SYSROOT_DIR=""

# devourer's UsbOpen.cpp/UsbDeviceLock.cpp do
# `#include <libusb-1.0/libusb.h>` on Linux, which needs the *parent*
# include/ dir on the search path — not include/libusb-1.0, which is what
# libusb-1.0.pc's `Cflags:` alone provides (-I.../include/libusb-1.0, so a
# plain `#include <libusb.h>` resolves but the versioned-subdir spelling
# doesn't). On the host build this second path comes for free: Nix's
# cc-wrapper auto-injects `-isystem <pkg>/include` for every package pulled
# in via `nix-shell -p libusb1` (see its NIX_CFLAGS_COMPILE). We're driving
# the cross gcc directly via `nix-build` instead of `nix-shell -p`, so that
# auto-injection doesn't happen — CPATH below reproduces it explicitly.
CPATH="$(readlink -f toolchain/libusb-out-dev)/include${CPATH:+:$CPATH}"
export CPATH

# --- 3. Configure + build. Only the maburd target: devourer's example
#        binaries (rxdemo, txdemo, ...) are not needed on the drone and some
#        pull in extra libusb example plumbing not worth static-linking here.
#        MABUR_BUILD_TESTS=OFF: the host test suite needs a host-runnable
#        libusb + GoogleTest et al; irrelevant for a cross artifact.
cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=cmake/arm-musl.cmake \
  -DCMAKE_BUILD_TYPE=Release -DMABUR_BUILD_TESTS=OFF \
  -DDEVOURER_JAGUAR1=OFF -DDEVOURER_8814=OFF -DDEVOURER_JAGUAR2_8822B=OFF \
  -DDEVOURER_JAGUAR2_8821C=OFF -DDEVOURER_JAGUAR3_8822C=OFF \
  -DDEVOURER_JAGUAR3_8822E=ON -DDEVOURER_LOG_MAX_LEVEL=WARN

cmake --build build-arm -j"$(nproc)" --target maburd

"${TARGET_TRIPLE}-strip" build-arm/drone/maburd -o out/arm/maburd

# `file` itself isn't on a bare NixOS PATH either; stage it like pkg-config.
if [ ! -e toolchain/file ]; then
  nix-build -E \
    'with import <nixpkgs> {}; file' \
    -o toolchain/file
fi
"$(readlink -f toolchain/file)/bin/file" out/arm/maburd
